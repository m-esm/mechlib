---
state: shipped
lens: telemetry
created: 2026-09-08
metric: pattern/lattice/kerf backlog rows whose claimed state cannot be resolved to code plus a commit
before: 7 (2026-09-08, main @ b035f97)
target: 0
measure: python3 scripts/backlog_provenance.py
evidence:
  - design/roadmap/evidence/2026-09-08-backlog-provenance.txt
  - design/roadmap/evidence/2026-09-08-lattice-pattern-kerf-catalog.json
  - design/roadmap/evidence/2026-09-08-gallery-latest-uiwalk.json
  - design/roadmap/evidence/2026-09-08-gallery-latest-order.png
  - design/roadmap/evidence/2026-09-08-gallery-latest-lattice.png
  - design/roadmap/evidence/2026-09-08-backlog-provenance-after.txt
slices: 3/3
after: 0
---
# The pattern/lattice/kerf backlog cannot prove what it shipped

## Why, against GOAL.md

The asked-for surface already exists. `scripts/overnight/pattern-lattice-kerf-backlog.md`
holds **22** researched rows, 21 marked shipped, and the hourly cron drops one per tick.
Live https://m-esm.github.io/mechlib/ serves **195** cards at v0.11.0, of which **14** are
`mechlib.lattices` / `mechlib.patterns`, spread over 8 distinct `added` days from 2026-08-04
to 2026-09-08 (`…-lattice-pattern-kerf-catalog.json`). The Latest-added chip sorts newest
first and works: pressed, the first four cards are `honeycomb_core`, `lattice_flexure`,
`cubic_lattice`, `gyroid_lattice`, and searching `lattice` under it returns 13 parts,
`kerf` returns 3 (`…-gallery-latest-uiwalk.json`, `…-gallery-latest-order.png`,
`…-gallery-latest-lattice.png`). Adding a 23rd row buys nothing the gallery does not
already show.

What is not true is the shipped column. Run against the installed library and this
repo's history, **7 of 22 rows cannot be resolved**: rows 04, 10 and 12 say
`THIS HOUR` where a SHA belongs; rows 06, 08 and 21 carry prose only; row 14 names
`kagome_lattice`, an API that is not on `mechlib.__all__` (the code landed as
`kagome_panel`). That is a hand-maintained ledger drifting from the source of truth,
which AGENTS.md forbids. GOAL number 3 (printed-and-verified parts, today 0 of 20) is
blocked on exactly this: you cannot pick a part to slice when the row pointing at it
does not name the commit that built it.

Out of scope: new backlog entries, new geometry, CadQuery/OpenSCAD, and the
still-pending row 22 `lattice_flexure(kind="v")`.

## What better looks like

`python3 scripts/backlog_provenance.py` prints `0`. Every shipped row names an API on
`mechlib.__all__` and a hex string that `git cat-file -e <sha>^{commit}` resolves in this
repo. The hourly cron writes the SHA of the commit it just made, so the ledger cannot
drift a second time.

## Slices

- [x] Backfill the 6 `no-sha` rows (04, 06, 08, 10, 12, 21) with the real commits, found
      by `git log --oneline -S<mode-string> -- mechlib/lattices.py mechlib/flexures.py`.
      Measure drops from 7 to 1.
- [x] Fix row 14 to name the shipped API `kagome_panel` and its commit. Measure reads 0.
- [x] Wire `scripts/backlog_provenance.py` into the same CI job as the gallery drift gate,
      and make the hourly cron append the SHA it just committed instead of `THIS HOUR`,
      so a new row cannot land unverifiable.

## Shipped 2026-09-08

Gate is `tests/test_backlog_provenance.py` (runs in the existing CI job; the
workflow now checks out with `fetch-depth: 0` so historic SHAs resolve). The
rule it enforces is written into the backlog file's own header, so the hourly
cron reads it before marking a row. Negative-tested: reverting row 04 to
`THIS HOUR` makes the measure print 1 and the test fail; restored, it prints 0
and 142 backlog/lattice/flexure/usecase tests pass.

Rows fixed: 04 `SHA 3694df1`, 06 `SHA 53204ba`, 08 `SHA d1882e5`,
10 `SHA 16184d5`, 12 `SHA e9833c9`, 21 `SHA 5563753`, and row 14 renamed from
the never-shipped `kagome_lattice` to the real `kagome_panel` `SHA 434302b`.
