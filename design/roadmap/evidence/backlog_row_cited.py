#!/usr/bin/env python3
"""Count shipped pattern/lattice/kerf rows a reader cannot trace to a source.

GOAL.md claims the list holds *researched* cells. A row earns "researched" only
when someone else can go read the thing: a URL, a DOI, or an arXiv id, reachable
either from the backlog row's note or from the docstring of the function the row
names. Author surnames and journal names ("Grima star", "MDPI meander") are not
retrievable: they name a memory, not a source.

Run from the repo root. Prints ``uncited N/M``; exit 1 when N is non-zero.
"""
from __future__ import annotations

import inspect
import re
import sys
from pathlib import Path

BACKLOG = Path("scripts/overnight/pattern-lattice-kerf-backlog.md")
ROW = re.compile(r"\|\s*\d\d\s*\|")
CITATION = re.compile(
    r"https?://\S+"          # any link
    r"|\b10\.\d{4,9}/\S+"    # DOI
    r"|\barXiv:\s*\d{4}\.\d{4,5}",  # arXiv id
    re.IGNORECASE,
)


def uncited_rows() -> tuple[list[tuple[str, str]], int]:
    import mechlib

    bad: list[tuple[str, str]] = []
    shipped = 0
    for line in BACKLOG.read_text().splitlines():
        if not ROW.match(line):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        row_id, api, status, note = cells[0], cells[1], cells[3], cells[4]
        if status != "shipped":
            continue
        shipped += 1
        if CITATION.search(note):
            continue
        func = getattr(mechlib, api.split("(")[0], None)
        doc = inspect.getdoc(func) or ""
        if CITATION.search(doc):
            continue
        bad.append((row_id, api))
    return bad, shipped


def main() -> int:
    bad, shipped = uncited_rows()
    if "-v" in sys.argv or "--verbose" in sys.argv:
        for row_id, api in bad:
            print(f"{row_id}\t{api}\tno-retrievable-source", file=sys.stderr)
    print(f"uncited {len(bad)}/{shipped}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
