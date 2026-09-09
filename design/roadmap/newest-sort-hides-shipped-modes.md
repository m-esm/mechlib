---
state: proposed
lens: telemetry
created: 2026-09-09
metric: shipped backlog rows a gallery visitor cannot date (card `added` != the row's commit date, or a card shared by several rows)
before: 17 of 21 (2026-09-09, main @ 4195b24)
target: 0
measure: python3 design/roadmap/evidence/backlog_row_datable.py
evidence:
  - design/roadmap/evidence/backlog_row_datable.py
  - design/roadmap/evidence/2026-09-09-backlog-row-datable.txt
slices: 0/3
---
# "Sortable by newest" does not show what shipped this hour

## Why, against GOAL.md

The count is already met and over-met: the backlog holds **22** researched
pattern/lattice/kerf rows against a target of 20, 21 shipped, and
`scripts/backlog_provenance.py` prints 0 — every shipped row names a real API
and a resolvable commit. Adding row 23 buys nothing.

The third GOAL clause is the one that fails: *"one new implemented every hour,
sortable by newest"*. Its live proof is the gallery's Latest-added chip, which
sorts cards by `added` in `docs/models/index.json`. `added` is keyed by **GLB
filename** and pinned to that file's first appearance
(`gallery/build_gallery.py`, the `previous_added` lookup — a new date is only
minted when `previous_added.get(model["file"])` misses). So an hour that adds a
new *mode* to an existing API lands on a card that already exists, and the card
keeps its old date.

`python3 design/roadmap/evidence/backlog_row_datable.py` against the live index
reads **17 of 21** shipped rows undatable:

- **11 rows collapse onto 2 cards.** All 7 `kerf_bend_cutter(mode=…)` rows share
  one card frozen at `added=2026-08-06`; all 4 `auxetic_panel(mode=…)` rows share
  one frozen at the same date. Seven hours of September work sort as August.
- **6 rows disagree with their own commit.** Rows 17/18/19 shipped 2026-09-02
  but their cards read 2026-09-07; row 20 shipped 09-03, card says 09-08; row 21
  shipped 09-06, card says 09-08; row 02 is off by a day. The date recorded is
  when the GLB first got *built*, not when the geometry landed.

So the chip does sort, and the four rows that happened to arrive as brand-new
single-mode APIs (01, 14, 15, 16) are honest — but a visitor asking "what was
implemented this hour?" is told August for anything that extended an existing
part. That is the hourly cadence being invisible in the only place it is
claimed. It also blocks the print-and-verify goal downstream: you cannot pick
"the newest cell" to slice when the sort lies about seven of them.

Out of scope: new backlog rows, new geometry, the pending row 22
`lattice_flexure(kind="v")`, changing the Latest-added UI, and re-dating cards
outside the pattern/lattice/kerf set.

## What better looks like

`python3 design/roadmap/evidence/backlog_row_datable.py` prints
`undatable 0/21`. Every shipped row is a thing a visitor can date: a mode that
shipped in an hour has its own card whose `added` equals the commit date of the
SHA in the row's note, so pressing Latest-added replays the build order.

## Slices

- [ ] Give each shipped mode its own gallery demo/GLB so
      `kerf_bend_cutter(mode="meander")` is a card, not a hidden argument of one.
      Measure drops from 17 toward 6 (11 collapsed rows resolve). Keep the
      collision gate green: `python3 gallery/collision_gate.py -q`.
- [ ] Seed `added` from the geometry's commit date instead of first-build time:
      when `previous_added` misses, fall back to
      `git log --diff-filter=A --format=%cs -1 -- <demo source>` before `now()`.
      Backfills rows 02/17/18/19/20/21; measure reads 0.
- [ ] Wire the probe into the CI job that already runs
      `tests/test_backlog_provenance.py`, so a future hour cannot ship a mode
      that the newest-sort cannot see.
