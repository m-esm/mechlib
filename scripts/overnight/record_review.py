#!/usr/bin/env python3
"""Record one visual-review JSON and append it to MOTION_REPORT.md."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from paths import MOTION_REPORT, REVIEWS, ensure_dirs
from workqueue import complete_motion, now_iso


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("review_json")
    args = parser.parse_args()
    ensure_dirs()
    data = json.loads(Path(args.review_json).read_text())
    demo = data["demo"]
    dest = REVIEWS / ("%s.json" % demo)
    dest.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
    complete_motion(
        demo,
        reviewed=True,
        verdict=data.get("verdict"),
        review_path=str(dest),
        review_at=now_iso(),
        issues=data.get("issues") or [],
    )
    issues = data.get("issues") or []
    issue_lines = "\n".join(
        "- **%s**: %s" % (i.get("severity", "?"), i.get("what", ""))
        for i in issues
    ) or "- none"
    block = (
        "\n## %s — %s\n\n"
        "- reviewed: %s\n"
        "- kind: %s\n"
        "- motion: %s\n"
        "- cycle_closes: %s\n"
        "- looks_like_intended: %s\n"
        "- frozen_that_should_move: %s\n"
        "\n%s\n\n"
        "**Issues**\n%s\n"
        % (
            demo,
            data.get("verdict", "unclear"),
            now_iso(),
            data.get("kind", "?"),
            (data.get("motion_reads_as") or "").strip(),
            data.get("cycle_closes"),
            data.get("looks_like_intended_mechanism"),
            ", ".join(data.get("bodies_that_look_frozen") or []) or "none",
            (data.get("notes") or "").strip(),
            issue_lines,
        )
    )
    if not MOTION_REPORT.exists():
        MOTION_REPORT.write_text(
            "# Overnight motion review\n\n"
            "Visual confirmation of gallery mechanisms. Generated overnight; "
            "not a hand-maintained registry.\n"
        )
    with MOTION_REPORT.open("a") as handle:
        handle.write(block)
    print(dest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
