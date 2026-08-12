#!/usr/bin/env python3
"""Record one gap-research JSON and append it to GAP_REPORT.md."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from paths import GAP_REPORT, GAPS, ensure_dirs
from workqueue import complete_gap, now_iso


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("gap_json")
    args = parser.parse_args()
    ensure_dirs()
    data = json.loads(Path(args.gap_json).read_text())
    slice_id = data["id"]
    dest = GAPS / ("%s.json" % slice_id)
    dest.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")
    complete_gap(
        slice_id,
        status="done",
        report=str(dest),
        candidate_count=len(data.get("candidates") or []),
        new_categories=data.get("new_categories") or [],
    )
    candidates = data.get("candidates") or []
    lines = []
    for item in candidates:
        lines.append(
            "- **%s** [%s] fit=%s — %s"
            % (
                item.get("name", "?"),
                item.get("priority", "?"),
                item.get("mechlib_fit", "?"),
                item.get("why", ""),
            )
        )
    cats = data.get("new_categories") or []
    cat_lines = [
        "- **%s** — %s" % (c.get("name", "?"), c.get("why", ""))
        for c in cats
    ] or ["- none"]
    block = (
        "\n## %s — %s\n\n"
        "%s\n\n"
        "**New categories**\n%s\n\n"
        "**Candidates**\n%s\n"
        % (
            slice_id,
            data.get("title", ""),
            (data.get("summary") or "").strip(),
            "\n".join(cat_lines),
            "\n".join(lines) or "- none that survive the semi-primitive filter",
        )
    )
    if not GAP_REPORT.exists():
        GAP_REPORT.write_text(
            "# Overnight missing-mechanism survey\n\n"
            "First-principles gap research against the live mechlib catalog. "
            "Generated overnight; not a hand-maintained registry.\n"
        )
    with GAP_REPORT.open("a") as handle:
        handle.write(block)
    print(dest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
