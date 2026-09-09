---
state: in-progress
lens: telemetry
created: 2026-09-09
metric: shipped backlog rows a gallery visitor cannot date (card `added` != the row's commit date, or a card shared by several rows)
before: 17 of 21 (2026-09-09, main @ 4195b24); 18 of 22 after row 22 shipped
target: 0
measure: python3 design/roadmap/evidence/backlog_row_datable.py
evidence:
  - design/roadmap/evidence/backlog_row_datable.py
  - design/roadmap/evidence/2026-09-09-backlog-row-datable.txt
  - design/roadmap/evidence/2026-09-09-backlog-row-datable-after.txt
slices: 1/3
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
      Measure drops from 15 toward 2 (13 collapsed rows resolve). Keep the
      collision gate green: `python3 gallery/collision_gate.py -q`.
- [x] Seed `added` from the GLB's birth commit instead of first-build wall
      clock. Measure 18 -> 15.
- [ ] Wire the probe into the CI job that already runs
      `tests/test_backlog_provenance.py`, so a future hour cannot ship a mode
      that the newest-sort cannot see.

## Slice 2 shipped 2026-09-09

`gallery/build_gallery.py` gained `_glb_birth_dates()`: one
`git log --diff-filter=A --reverse --name-only -- docs/models/*.glb` call
resolves every GLB's first commit, and `added` now falls back to that birth
timestamp before `now()`. `previous_added` still wins, so the 172 cards that
already agreed did not move. Missing or unusable git returns `{}` and the build
keeps working (sdist, shallow checkout).

23 cards re-dated. Two kinds, both corrections: 16 cards built at
`2026-09-04T21:46:18Z` in one batch now carry the commit that actually
introduced them, and 5 lattice cards move *earlier* to the day their geometry
landed (`cubic_lattice` 09-07 -> 09-02, `gyroid_lattice` 09-07 -> 09-02,
`kelvin_cell` 09-07 -> 09-02, `honeycomb_core` / `lattice_flexure` 09-08 ->
09-07). Two cards move later (`fix_pin` 08-04 -> 08-30, `arc_ratchet_2d` 08-05
-> 08-30) because their stored dates predate the GLB itself.

Verified: 977 tests pass, `scripts/backlog_provenance.py` prints 0, collision
gate 193 ok / 0 failures.

**Residual, deliberately not chased.** The measure reads 15, not the 6 first
predicted. 13 of the 15 are collapsed cards (slice 1). The other 2 are a
one-day lag that this rule cannot remove: `isogrid_panel` geometry commit
2026-09-01 but its GLB was first committed 2026-09-02, and `honeycomb_core`
geometry 2026-09-03 vs GLB 2026-09-07. The gallery is rebuilt in a later commit
than the geometry, so GLB birth is a *lower bound* on ship date, not the ship
date. Closing those two needs the row's own SHA (the backlog already carries
it) rather than any file's history, which is slice 3's business.
