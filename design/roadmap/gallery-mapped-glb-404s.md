---
state: promoted
lens: telemetry
created: 2026-09-07
metric: GALLERY_FILE_TO_API keys absent from live Pages models/index.json
before: 5 (2026-09-07 measure below)
target: 0
measure: python3 -c "import json,urllib.request; from mechlib.usecases import GALLERY_FILE_TO_API as g; idx=json.load(urllib.request.urlopen('https://m-esm.github.io/mechlib/models/index.json')); listed={m['file'] for m in idx['models']}; print(sum(1 for f in g if f not in listed))"
evidence:
  - design/roadmap/evidence/2026-09-07-gallery-mapped-glb-404s.json
  - design/roadmap/evidence/2026-09-07-gallery-search-lattice-flexure.png
  - design/roadmap/evidence/2026-09-07-gallery-search-honeycomb-core.png
  - design/roadmap/evidence/2026-09-07-gallery-search-gyroid-lattice.png
  - design/roadmap/evidence/2026-09-07-gallery-uiwalk-index.json
slices: 0/3
after:
---
# Live gallery 404s for mapped demo GLBs

## Why, against GOAL.md

GOAL number 2 is use cases with a gallery demo GLB, and the done path is that an agent can `search_use_cases("job")`, call one function, and **see it in the live gallery**.

The Python map already lists those demos (`GALLERY_FILE_TO_API` has 195 files / 192 APIs). Live Pages does not. HEAD + `models/index.json` this tick (`design/roadmap/evidence/2026-09-07-gallery-mapped-glb-404s.json`): **5** mapped files are missing from https://m-esm.github.io/mechlib/models/index.json. Two of them HTTP **404** (`honeycomb_core_demo.glb`, `lattice_flexure_demo.glb`). Three more 200 on disk/Pages (`cubic_lattice`, `gyroid_lattice`, `kelvin_cell`) but are omitted from the catalog JSON, so search cannot find them.

uiwalk of the live gallery: search `lattice_flexure` → **0 OF 190 PARTS**, empty state “Nothing matches lattice_flexure” (`…-gallery-search-lattice-flexure.png`). Same empty for `honeycomb_core` and `gyroid_lattice`. Looked at those three shots: catalog loads (v0.11.0, 190 + 32 utils); the three shipped lattice APIs do not appear.

This proposal only rebuilds/reindexes the already-mapped demos onto `docs/` so Deploy Pages can serve them. It does not add CadQuery/OpenSCAD/NopSCADlib, finished product models, or wiper-kit docs.

## What better looks like

Every key in `GALLERY_FILE_TO_API` is a row in live `models/index.json` and returns HTTP 200 at `models/<file>`. The measure prints `0`. Search on https://m-esm.github.io/mechlib/ for `lattice_flexure` / `honeycomb_core` / `gyroid_lattice` shows a card, not “Nothing matches”.

## Slices

- [ ] Build and commit the two missing GLBs (`honeycomb_core_demo.glb`, `lattice_flexure_demo.glb`) under `docs/models/`. Direct URLs stop 404ing.
- [ ] Rebuild `docs/models/index.json` so the three already-200 GLBs (`cubic_lattice`, `gyroid_lattice`, `kelvin_cell`) are catalog rows. Live search stops returning 0/190.
- [ ] Gate: fail CI when a `GALLERY_FILE_TO_API` key is absent from `docs/models/` or from `index.json`, so Pages cannot drift again. Measure reads 0.
