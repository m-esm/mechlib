#!/usr/bin/env python3
"""Count shipped backlog rows a gallery visitor cannot date.

The GOAL promises "one new implemented every hour, sortable by newest". The
live proof of that is the gallery's Latest-added chip, which sorts cards by
``added`` in ``docs/models/index.json``. ``added`` is keyed by *GLB filename*
and pinned to the file's first appearance (gallery/build_gallery.py, the
``previous_added`` lookup). So when an hour's work adds a new *mode* to an
existing API -- ``kerf_bend_cutter(mode="meander")``, ``auxetic_panel(mode=
"star")`` -- the card it lands on keeps its original date and the new work is
invisible to the sort.

A shipped row is **datable** when the card carrying its API reports an ``added``
date equal to the commit date of the SHA in the row's note. A row is
undatable when its card is shared with other rows (the card can only carry one
date) or when the dates disagree.

Prints ``undatable/shipped`` and one line per undatable row. Exit code 1 while
the count is above the target so this can become a gate later.

Usage (repo root):
    python3 design/roadmap/evidence/backlog_row_datable.py [--target N]
    python3 design/roadmap/evidence/backlog_row_datable.py --local
"""
from __future__ import annotations

import argparse
import collections
import json
import re
import subprocess
import sys
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
BACKLOG = ROOT / "scripts" / "overnight" / "pattern-lattice-kerf-backlog.md"
LOCAL_INDEX = ROOT / "docs" / "models" / "index.json"
LIVE_INDEX = "https://m-esm.github.io/mechlib/models/index.json"
ROW = re.compile(r"\|\s*\d\d\s*\|")
SHA = re.compile(r"\b[0-9a-f]{7,40}\b")


def shipped_rows() -> list[tuple[str, str, str]]:
    """(id, api, sha) for every row marked shipped."""
    out = []
    for line in BACKLOG.read_text().splitlines():
        if not ROW.match(line):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        row_id, api, status, note = cells[0], cells[1], cells[3], cells[4]
        if status != "shipped":
            continue
        match = SHA.search(note)
        out.append((row_id, api, match.group() if match else ""))
    return out


def card_dates(local: bool) -> dict[str, str]:
    """api -> ``added`` date (YYYY-MM-DD) of the gallery card carrying it."""
    from mechlib.usecases import GALLERY_FILE_TO_API as file_to_api

    if local:
        index = json.loads(LOCAL_INDEX.read_text())
    else:
        with urllib.request.urlopen(LIVE_INDEX, timeout=30) as fh:
            index = json.load(fh)
    dates: dict[str, str] = {}
    for model in index["models"]:
        api = file_to_api.get(model["file"], "")
        if api and api not in dates:
            dates[api] = model["added"][:10]
    return dates


def commit_date(sha: str) -> str:
    if not sha:
        return ""
    done = subprocess.run(
        ["git", "show", "-s", "--format=%cs", sha],
        cwd=ROOT, capture_output=True, text=True,
    )
    return done.stdout.strip() if done.returncode == 0 else ""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", type=int, default=0)
    ap.add_argument("--local", action="store_true",
                    help="read docs/models/index.json instead of the live site")
    args = ap.parse_args()

    rows = shipped_rows()
    dates = card_dates(args.local)
    per_card = collections.Counter(api.split("(")[0] for _, api, _ in rows)

    bad: list[str] = []
    for row_id, api, sha in rows:
        base = api.split("(")[0]
        card = dates.get(base)
        want = commit_date(sha)
        if card is None:
            bad.append(f"{row_id} {api}: no gallery card")
        elif per_card[base] > 1:
            bad.append(
                f"{row_id} {api}: card shared by {per_card[base]} rows, "
                f"stuck at added={card} (shipped {want or '?'})")
        elif card != want:
            bad.append(f"{row_id} {api}: card added={card} != shipped {want or '?'}")

    for line in bad:
        print(line)
    print(f"undatable {len(bad)}/{len(rows)} shipped rows (target {args.target})")
    return 1 if len(bad) > args.target else 0


if __name__ == "__main__":
    sys.exit(main())
