---
state: proposed
lens: telemetry
created: 2026-09-11
metric: unique shipped pattern/lattice/kerf APIs whose gallery card AND use-case string contain no retrievable source (URL, DOI, or arXiv id)
before: 12 of 12 (2026-09-11, main @ 3ed40de)
target: 0
measure: python3 design/roadmap/evidence/gallery_row_cited.py
evidence:
  - design/roadmap/evidence/gallery_row_cited.py
  - design/roadmap/evidence/2026-09-11-gallery-row-cited.txt
slices: 0/3
---
# The gallery is the researched claim a maker actually reads

## Why, against GOAL.md

GOAL.md is written for "Python-fluent makers and AI coding agents who need
researched FDM-printable pattern, lattice, and kerf cells" and points them at
the live gallery. The count clause is already over-met: the backlog holds **22**
rows against a target of 20. `python3 scripts/backlog_provenance.py` prints
**0**. Slice 1 of `researched-rows-cite-nothing.md` put retrievable identifiers
on 10 backlog notes plus the `kerf_bend_cutter` / `auxetic_panel` docstrings.
That slice does not close this. A visitor who never opens `lattices.py` still
cannot open a source.

The collapse is 22 backlog rows, 12 unique APIs, 12 cards, 0 retrievable
identifiers on the visitor path. `python3 design/roadmap/evidence/gallery_row_cited.py`
reads **uncited-in-gallery 12/12**. Zero of 195 cards in `docs/models/index.json`
contain a URL, DOI, or arXiv id. All 12 pattern APIs' `USE_CASES` strings have
none. Cards get `applications` from `gallery/applications.py` ->
`mechlib.usecases.USE_CASES` / `GALLERY_FILE_TO_API`, so the live site and
`use_case("auxetic_panel")` show the same surnames the backlog used to:

- `origin` on `isogrid_panel_demo.glb` is `classic ref: NASA isogrid (triangular rib sheet)`
  with no NASA document number and no URL.
- `origin` on `auxetic_panel_demo.glb` is `classic ref: auxetic re-entrant honeycomb (Lakes, 1987)`
  with no DOI. `applications` / `use_case("auxetic_panel")` name "Grima
  arrowhead and star-shaped honeycomb NPR cells" the same way row 10 did
  yesterday, still with nothing to open.
- `lattice_flexure` carries "Howell". `gyroid_lattice` carries "Schoen".
  `bcc_lattice` carries "BCC FFF metamaterial truss". Same class of unresolvable
  memory as "MDPI meander" on the old notes.

Docstrings are not on this path. The two `References:` blocks slice 1 added are
invisible to a gallery card and to `use_case()`. The cost is the audience
GOAL.md named. A maker clicking Latest-added cannot check whether a kerf pitch
is the published one. An agent asked to extend `auxetic_panel` gets a surname
and invents a cell. Print-and-verify stays blocked: there is nothing on the
card to verify a print against.

Out of scope: new backlog rows, new geometry, docstring edits, citing the
remaining uncited backlog rows, gallery GLB rebuilds, and wiring CI (slice 3).

## What better looks like

`python3 design/roadmap/evidence/gallery_row_cited.py` prints
`uncited-in-gallery 0/12`. Every unique shipped pattern/lattice/kerf API
resolves from the card a visitor sees *and* from `use_case()`: a DOI, an arXiv
id, a NASA document number, or a permanent URL, not a surname. The rule is
retrievable identifier, not a classic-ref clause. Where genuinely no source
exists, the honest fix is to say so on the card and drop the word researched
for that API, not to fabricate a citation.

## Slices

- [ ] Mirror already-locked citations for `kerf_bend_cutter` and
      `auxetic_panel` into `USE_CASES` (and therefore the card `applications`
      line). Measure 12 -> 10.
- [ ] Cite the remaining 10 APIs on the card/use-case path once slice 2 of
      `researched-rows-cite-nothing.md` has identifiers to copy (or lock them
      here if that slice has not landed). Measure 10 -> 0.
- [ ] Stop cards from dropping identifiers: either flow a `source` field from
      the backlog/docstring into `docs/models/index.json`, or keep the
      identifier inside `USE_CASES` and add a CI check that
      `gallery_row_cited.py` stays at 0.

## Measuring it

```bash
python3 design/roadmap/evidence/gallery_row_cited.py -v
```

Prints `uncited-in-gallery N/M` on stdout (M = unique shipped APIs), one
`api<TAB>no-retrievable-source-on-card-or-usecase` line per miss on stderr,
exit 1 while N is non-zero. `--live` fetches the published
`https://m-esm.github.io/mechlib/models/index.json`; default is local
`docs/models/index.json`. Baseline capture:
`design/roadmap/evidence/2026-09-11-gallery-row-cited.txt` (12/12 at `3ed40de`).
