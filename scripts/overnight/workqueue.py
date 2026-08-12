#!/usr/bin/env python3
"""Atomic claim/complete helpers for the overnight queues."""

from __future__ import annotations

import argparse
import fcntl
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from paths import CATALOG, GAP_QUEUE, HEARTBEAT, MOTION_QUEUE, STOP, ensure_dirs


def now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def morning_reached(cutoff_iso: str) -> bool:
    cutoff = datetime.fromisoformat(cutoff_iso)
    return datetime.now(timezone.utc).astimezone() >= cutoff


def load_json(path: Path, default):
    if not path.exists():
        return default
    return json.loads(path.read_text())


def dump_json(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
    tmp.replace(path)


def _lock_path(path: Path) -> Path:
    return path.with_suffix(path.suffix + ".lock")


def with_lock(path: Path, fn):
    ensure_dirs()
    lock_path = _lock_path(path)
    with lock_path.open("a+") as handle:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
        data = load_json(path, {"updated": now_iso(), "items": []})
        result = fn(data)
        data["updated"] = now_iso()
        dump_json(path, data)
        return result


def heartbeat(loop: str, note: str, extra: Optional[Dict[str, Any]] = None) -> None:
    ensure_dirs()
    payload = load_json(HEARTBEAT, {})
    payload[loop] = {
        "at": now_iso(),
        "note": note,
        **(extra or {}),
    }
    dump_json(HEARTBEAT, payload)


def stopped() -> bool:
    return STOP.exists()


def claim_motion(n: int = 1, want: str = "review") -> List[Dict[str, Any]]:
    """Claim the next motion items.

    want=render  -> not yet rendered, not claimed for render
    want=review  -> rendered, not yet reviewed
    """

    def _claim(data):
        claimed = []
        for item in data["items"]:
            if len(claimed) >= n:
                break
            if want == "render":
                if item.get("rendered") or item.get("render_claim"):
                    continue
                item["render_claim"] = now_iso()
                claimed.append(item)
            elif want == "review":
                if (not item.get("rendered") or item.get("reviewed")
                        or item.get("review_claim")):
                    continue
                item["review_claim"] = now_iso()
                claimed.append(item)
        return claimed

    return with_lock(MOTION_QUEUE, _claim)


def complete_motion(item_id: str, **fields) -> None:
    def _done(data):
        for item in data["items"]:
            if item["id"] == item_id:
                item.update(fields)
                return item
        raise KeyError(item_id)

    with_lock(MOTION_QUEUE, _done)


def claim_gap(n: int = 1) -> List[Dict[str, Any]]:
    def _claim(data):
        claimed = []
        for item in data["items"]:
            if len(claimed) >= n:
                break
            if item.get("status") in ("done", "in_progress"):
                continue
            item["status"] = "in_progress"
            item["claimed_at"] = now_iso()
            claimed.append(item)
        return claimed

    return with_lock(GAP_QUEUE, _claim)


def complete_gap(item_id: str, **fields) -> None:
    def _done(data):
        for item in data["items"]:
            if item["id"] == item_id:
                item.update(fields)
                item.setdefault("status", "done")
                item["completed_at"] = now_iso()
                return item
        raise KeyError(item_id)

    with_lock(GAP_QUEUE, _done)


def summarize() -> Dict[str, Any]:
    motion = load_json(MOTION_QUEUE, {"items": []})
    gap = load_json(GAP_QUEUE, {"items": []})
    items = motion.get("items") or []
    gitems = gap.get("items") or []
    return {
        "stop": stopped(),
        "motion": {
            "total": len(items),
            "rendered": sum(1 for i in items if i.get("rendered")),
            "reviewed": sum(1 for i in items if i.get("reviewed")),
            "failed_render": sum(1 for i in items if i.get("render_error")),
            "failed_review": sum(1 for i in items if i.get("verdict") == "fail"),
            "unclear": sum(1 for i in items if i.get("verdict") == "unclear"),
        },
        "gap": {
            "total": len(gitems),
            "done": sum(1 for i in gitems if i.get("status") == "done"),
            "in_progress": sum(1 for i in gitems if i.get("status") == "in_progress"),
            "pending": sum(
                1 for i in gitems if i.get("status") not in ("done", "in_progress")
            ),
        },
        "catalog": CATALOG.exists(),
    }


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action",
        choices=("status", "claim-review", "claim-render", "claim-gap", "stop"),
    )
    parser.add_argument("-n", type=int, default=1)
    args = parser.parse_args(argv)
    ensure_dirs()
    if args.action == "status":
        print(json.dumps(summarize(), indent=2))
        return 0
    if args.action == "stop":
        STOP.write_text("stopped %s\n" % now_iso())
        print("wrote", STOP)
        return 0
    if args.action == "claim-review":
        print(json.dumps(claim_motion(args.n, "review"), indent=2))
        return 0
    if args.action == "claim-render":
        print(json.dumps(claim_motion(args.n, "render"), indent=2))
        return 0
    if args.action == "claim-gap":
        print(json.dumps(claim_gap(args.n), indent=2))
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
