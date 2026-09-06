---
state: killed
lens: journey
created: 2026-09-06
metric: design/critiques/*.md that state a gallery demo was sliced or printed
before: 0 (2026-09-06 measure below)
target: 20
measure: python3 -c "from pathlib import Path; n=0; p=Path('design/critiques');
files=list(p.glob('*.md')) if p.exists() else []
for f in files:
    t=f.read_text(encoding='utf-8').lower()
    if 'sliced' in t or 'printed' in t: n+=1
print(n)"
evidence:
  - design/roadmap/evidence/2026-09-06-gallery-stl-no-print-proof.json
  - design/roadmap/evidence/2026-09-06-gallery-search-spur-gear.png
  - design/roadmap/evidence/2026-09-06-gallery-playground-download-stl.png
  - design/roadmap/evidence/2026-09-06-softsolder-bosl2-gear-installed.png
  - design/roadmap/evidence/2026-09-06-print-proof-measure.txt
slices: 0/3
after:
---
# Print-proof looks for gallery demos

## Why, against GOAL.md

GOAL number 3 is printed-and-verified parts: demos with a `design/critiques/` look that states it was sliced or printed — **0/20**.

GOAL's done path is: `search_use_cases("job")` → one function → watertight mesh that **prints** → live gallery. Numbers 1 and 2 already read 263/263 and 191/191. The last hop is missing: a maker who finds `spur_gear` cannot finish "will this print tonight?"

Walked that hop this tick (`design/roadmap/evidence/2026-09-06-gallery-stl-no-print-proof.json` against https://m-esm.github.io/mechlib/): search `spur_gear` shows GEARS-01..04 with live-params / bodies / mm badges and no print/slice/verified mark (`…-gallery-search-spur-gear.png`). Tune opens a playground whose actions are Play / Copy code / **Download STL** / Reset defaults; status is `installing mechlib wheels…`; no orientation, nozzle, layer height, empty-layer check, or "this was sliced" (`…-gallery-playground-download-stl.png`). `design/critiques/` has one HTML look and zero files that say sliced or printed (`…-print-proof-measure.txt` prints `0`).

Comparable product: Ed Nisley's BOSL2 change-gear post. Same library-generator journey, then a photo of the printed 28T gear **installed on the mini-lathe** plus a PrusaSlicer preview in the article (`…-softsolder-bosl2-gear-installed.png`). That is the path mechlib's gallery does not complete.

This proposal only adds critique looks (and an optional gallery badge when a look exists). It does not add CadQuery/OpenSCAD/NopSCADlib, finished product models, or wiper-kit docs.

## What better looks like

Twenty gallery demos each have a `design/critiques/` look that states the demo was sliced or printed (orientation, nozzle, layer, and a yes/no on empty layers or a bed photo). The GOAL measure prints `20`. A maker who downloads STL from Tune can read that look instead of guessing.

## Slices

- [ ] Critique template + first 7 print-class demos (`spur_gear_mesh`, `hex_nut_mesh`, `dovetail`, `snap`/`living_hinge`, `teardrop` bore coupon, `herringbone`/`worm`, `lattice` coupon) with a look that states sliced. No new geometry APIs.
- [ ] Next 7 demos (clamps, ratchets, fasteners, a multi-body animated demo) with the same look shape.
- [ ] Remaining 6 to hit 20. Optional: gallery card badge when a matching look exists. Measure reads 20.
