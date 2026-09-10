---
state: proposed
lens: telemetry
created: 2026-09-10
metric: shipped pattern/lattice/kerf rows with no retrievable source (no URL, DOI, or arXiv id in the backlog note or in the named function's docstring)
before: 22 of 22 (2026-09-10, main @ 2201698)
target: 0
measure: python3 design/roadmap/evidence/backlog_row_cited.py
evidence:
  - design/roadmap/evidence/backlog_row_cited.py
  - design/roadmap/evidence/2026-09-10-backlog-row-cited.txt
slices: 0/3
---
# "Researched" is the one word in GOAL.md nothing checks

## Why, against GOAL.md

The count clause is met and over-met. `scripts/overnight/pattern-lattice-kerf-backlog.md`
holds **22** rows against a target of 20, 21 of them shipped plus row 22
`lattice_flexure(kind="v")` landed at `fe201be`. `python3 scripts/backlog_provenance.py`
prints **0**, so every shipped row names a function on `mechlib.__all__` and carries a
SHA that resolves. The newest-sort clause has its own open proposal
(`newest-sort-hides-shipped-modes.md`, slice 2 shipped). Row 23 buys nothing.

The clause with no gate behind it is the adjective: *researched* pattern, lattice, and
kerf cells. `python3 design/roadmap/evidence/backlog_row_cited.py` reads
**uncited 22/22**. Not one shipped row lets a reader reach the thing it was researched
from. What the notes carry instead is a name that only works if you already know it:

- Row 10 says `Grima double-arrowhead NPR cells`, row 11 `Grima star-shaped honeycomb`.
  Grima has published dozens of auxetic papers. Which geometry, which cell angles,
  which paper? The note does not say.
- Row 08 says `MDPI meander square-wave labyrinth`. MDPI is a publisher with hundreds
  of thousands of papers, not a citation.
- Row 02 says `NASA triangular rib sheet` for isogrid, the one row with a real
  canonical source (the NASA isogrid design handbook), and still does not name it.
- Rows 15 through 20 (BCC, octet, Kelvin, cubic, gyroid, honeycomb core) carry pure
  geometric description and nothing else. `gyroid_lattice`'s note asserts
  `wall>=1.2 cell>=8` as an FDM rule with no origin at all, and the docstring repeats
  the number without one either.
- The `Sources:` footer at the bottom of the backlog is five bare phrases
  (`LivingHingeGenerator, MDPI meander, FFF review, NASA isogrid PA+CF, BCC FFF truss`),
  unattached to any row and unresolvable as written.

Checked all 12 public APIs behind these rows: **zero** docstrings contain a URL, DOI,
or arXiv id, so the fallback path the probe allows finds nothing either.

This is exactly the drift AGENTS.md forbids in a different guise. `backlog_provenance.py`
made the *shipped* column honest by demanding a commit; nothing makes the *researched*
column honest, so it degrades into whatever the implementing hour remembered. The cost
is concrete for the audience GOAL.md names. A maker asks whether a kerf pitch is safe at
0.4 mm nozzle and there is no paper to read. An AI coding agent asked to extend
`auxetic_panel` with a new mode has no prior art to match against, so it invents a cell
and the library's claim to be researched weakens by one more row. It also blocks the
print-and-verify goal: a printed part is only *verified* against a published expectation,
and today there is nothing to verify against.

Out of scope: new backlog rows, new geometry, re-dating gallery cards (that is the
newest-sort proposal), vetoed entries, and the vitamins tables.

## What better looks like

`python3 design/roadmap/evidence/backlog_row_cited.py` prints `uncited 0/22`. Every
shipped row resolves to something a reader can open: a DOI, an arXiv id, a NASA
document number, or a permanent URL, attached to the row it justifies and mirrored into
the docstring of the function so it survives outside this repo. The rule is retrievable
identifier, not a name. Where genuinely no source exists, the honest fix is to say so in
the note and drop the word researched for that row, not to fabricate a citation.

## Slices

- [ ] Cite the 9 `kerf_bend_cutter` and 4 `auxetic_panel` mode rows (13 rows, the two
      collapsed families). These have real literature: Grima's auxetic series and the
      living-hinge/kerf-bending body of work. Measure 22 -> 9. Docstring gets a
      `References:` block per mode so the citation ships with the wheel.
- [ ] Cite the 9 remaining rows (honeycomb panel/core, isogrid, kagome, BCC, octet,
      Kelvin, cubic, gyroid). Measure 9 -> 0. The `wall>=1.2 cell>=8` gyroid rule either
      gets the FFF paper it came from or gets demoted to a measured in-house default
      with the print that established it.
- [ ] Wire the probe into the CI job that already runs `tests/test_backlog_provenance.py`
      so a future hour cannot mark a row shipped without a retrievable source, and
      replace the loose `Sources:` footer with per-row citations.

## Measuring it

```bash
python3 design/roadmap/evidence/backlog_row_cited.py -v
```

Prints `uncited N/M` on stdout, one `id<TAB>api<TAB>no-retrievable-source` line per
failing row on stderr, exit 1 while N is non-zero. Baseline capture:
`design/roadmap/evidence/2026-09-10-backlog-row-cited.txt` (22/22 at `2201698`).
