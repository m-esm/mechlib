#!/usr/bin/env python3
"""Count pattern/lattice/kerf backlog rows whose claimed state is unverifiable.

A row is unverifiable when either:
  * its ``api`` column names a function that is not on the installed ``mechlib``, or
  * it is marked ``shipped`` and the note carries no hex string that resolves to a
    commit in this repository.

Run from the repo root. Prints one integer; exit code 1 when that integer is
non-zero so the gate can fail CI once the backlog is clean.
"""
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

BACKLOG = Path(__file__).with_name("overnight") / "pattern-lattice-kerf-backlog.md"
ROW = re.compile(r"\|\s*\d\d\s*\|")
SHA = re.compile(r"\b[0-9a-f]{7,40}\b")


def unverifiable_rows() -> list[tuple[str, str, str]]:
    import mechlib

    bad: list[tuple[str, str, str]] = []
    for line in BACKLOG.read_text().splitlines():
        if not ROW.match(line):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        row_id, api, _kind, status, note = cells[0], cells[1], cells[2], cells[3], cells[4]
        if not hasattr(mechlib, api.split("(")[0]):
            bad.append((row_id, api, "api-missing"))
            continue
        if status != "shipped":
            continue
        match = SHA.search(note)
        if match is None:
            bad.append((row_id, api, "no-sha"))
            continue
        probe = subprocess.run(
            ["git", "cat-file", "-e", match.group() + "^{commit}"],
            capture_output=True,
        )
        if probe.returncode != 0:
            bad.append((row_id, api, "sha-unresolvable"))
    return bad


def main() -> int:
    bad = unverifiable_rows()
    if "-v" in sys.argv or "--verbose" in sys.argv:
        for row_id, api, why in bad:
            print(f"{row_id}\t{api}\t{why}", file=sys.stderr)
    print(len(bad))
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
