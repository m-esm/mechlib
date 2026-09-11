#!/usr/bin/env python3
"""Count shipped pattern/lattice/kerf APIs a gallery visitor cannot trace.

GOAL.md claims *researched* FDM-printable cells for makers and AI agents who
reach them through the live gallery or ``use_case()``. Slice 1 of
``researched-rows-cite-nothing.md`` locked URLs/DOIs onto 10 backlog notes and
two docstrings. Cards and use-case strings still carry surnames ("Grima",
"NASA", "Howell", "Schoen") the way the backlog did yesterday.

A shipped API (unique ``api.split("(")[0]`` from the backlog) is
**uncited-in-gallery** when neither its gallery card (``origin``,
``applications``, ``description``, and the rest of the JSON object) nor
``mechlib.usecases.USE_CASES[api]`` contains a retrievable identifier: a URL,
a DOI, or an arXiv id. Docstrings are out of this probe's path.

Run from the repo root. Prints ``uncited-in-gallery N/M``; exit 1 when N is
non-zero. Default index is local ``docs/models/index.json``; ``--live`` fetches
the published gallery.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
BACKLOG = ROOT / "scripts" / "overnight" / "pattern-lattice-kerf-backlog.md"
LOCAL_INDEX = ROOT / "docs" / "models" / "index.json"
LIVE_INDEX = "https://m-esm.github.io/mechlib/models/index.json"
ROW = re.compile(r"\|\s*\d\d\s*\|")
CITATION = re.compile(
    r"https?://\S+"          # any link
    r"|\b10\.\d{4,9}/\S+"    # DOI
    r"|\barXiv:\s*\d{4}\.\d{4,5}",  # arXiv id
    re.IGNORECASE,
)


def unique_shipped_apis() -> list[str]:
    """First-seen unique ``api.split('(')[0]`` among shipped backlog rows."""
    out: list[str] = []
    seen: set[str] = set()
    for line in BACKLOG.read_text().splitlines():
        if not ROW.match(line):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        api, status = cells[1], cells[3]
        if status != "shipped":
            continue
        name = api.split("(")[0]
        if name in seen:
            continue
        seen.add(name)
        out.append(name)
    return out


def load_index(live: bool) -> dict:
    if live:
        with urllib.request.urlopen(LIVE_INDEX, timeout=30) as fh:
            return json.load(fh)
    return json.loads(LOCAL_INDEX.read_text())


def uncited_in_gallery(live: bool = False) -> tuple[list[str], int]:
    from mechlib.usecases import GALLERY_FILE_TO_API, USE_CASES

    apis = unique_shipped_apis()
    files_by_api: dict[str, list[str]] = {}
    for fname, mapped in GALLERY_FILE_TO_API.items():
        files_by_api.setdefault(mapped, []).append(fname)
    cards_by_file = {m["file"]: m for m in load_index(live)["models"]}

    bad: list[str] = []
    for api in apis:
        blobs: list[str] = [USE_CASES.get(api, "")]
        for fname in files_by_api.get(api, ()):
            card = cards_by_file.get(fname)
            if card is not None:
                blobs.append(json.dumps(card, ensure_ascii=False))
        if any(CITATION.search(blob or "") for blob in blobs):
            continue
        bad.append(api)
    return bad, len(apis)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-v", "--verbose", action="store_true",
        help="print one miss line per uncited API on stderr",
    )
    parser.add_argument(
        "--local", action="store_true",
        help="read docs/models/index.json (default; ignored if --live)",
    )
    parser.add_argument(
        "--live", action="store_true",
        help="fetch https://m-esm.github.io/mechlib/models/index.json",
    )
    args = parser.parse_args()
    bad, total = uncited_in_gallery(live=bool(args.live))
    if args.verbose:
        for api in bad:
            print(
                f"{api}\tno-retrievable-source-on-card-or-usecase",
                file=sys.stderr,
            )
    print(f"uncited-in-gallery {len(bad)}/{total}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
