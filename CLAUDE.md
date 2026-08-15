# mechlib

Semi-primitive parametric geometry components for FDM-printed mechanical design
(trimesh + shapely + manifold3d). Python >= 3.9, zero project coupling.

## Scope (locked)

This library holds SEMI-PRIMITIVE PARTS ONLY: reusable, project-agnostic
geometry building blocks. Primitives, sweeps, cutters, gear generators,
fasteners, closures, patterns, text, mesh utilities, plate packing, STEP export.

Never add:

- Finished designed parts or product models (brackets, housings, enclosures,
  shower heads, robot chassis). Those live in consumer projects.
- Project assemblies or anything bound to one project's dimensions.
- Gallery entries that showcase a "part". The gallery demos API usage only:
  one minimal entry per API surface.

If a helper in a consumer project is generic enough to be reused, promote it
here with explicit parameters. If it only makes sense with one project's
dimensions baked in, it stays in that project.

## Decoupling contract

Functions take explicit arguments only. No reading of project parameter
modules, no imports from consuming projects, no mutable module state.
Consumers keep thin facade modules that bind their own params.

## Layout

- `mechlib/` — the package (prim, sweep, cutters, closures, gears, mechanisms,
  fasteners, fixtures, meshutil, packing, patterns, text, stepio, usecases).
- `mechlib/usecases.py` — **machinery use cases for every public API** (where
  each part shows up in real machines). Source of truth for AI part selection
  and for the gallery "Used in" lines. See "Choosing a part" below.
- `gallery/build_gallery.py` — regenerates `docs/models/*.glb` plus
  `docs/models/index.json`. Run from repo root: `python3 gallery/build_gallery.py`.
- `docs/` — GitHub Pages site (interactive gallery) at
  https://m-esm.github.io/mechlib/, served from `main:/docs`. Committing
  rebuilt GLBs publishes them.
- `tests/` — pytest suite: `python3 -m pytest tests/ -q`.

## Choosing a part (for AI agents)

When an agent needs geometry for a mechanism (leg, clamp, gear train, seal,
linear stage, …), **do not invent it from boxes and cylinders** if a mechlib
API already covers it.

1. Read use cases: `mechlib/usecases.py`, or at runtime:
   ```python
   from mechlib.usecases import use_case, search_use_cases, all_use_cases
   search_use_cases("robot joint")   # [(api_name, text), ...]
   use_case("four_bar")              # one machinery situation string
   ```
2. Match the **job** to a concrete situation in those strings (shaper ram,
   CV halfshaft, O-ring face seal, printer bed level, …), then call that API.
3. Browse the same text on the live gallery cards ("Used in") at
   https://m-esm.github.io/mechlib/ — each card’s applications field comes
   from `usecases.py`.
4. New public function: add a `USE_CASES["fn_name"] = "..."` line in
   `mechlib/usecases.py` (and a gallery GLB mapping if the demo file name
   does not match), same PR as the function.

## Workflow

- Work continues on `main`. Commit there. Do not open a feature branch or
  PR unless asked (standing exception, 2026-08-15).
- Version lives in BOTH `pyproject.toml` and `mechlib/__init__.py`; bump
  together.
- New public function: add to module, re-export in `mechlib/__init__.py`
  (`__all__` too), add a README API-table row, add a minimal gallery demo,
  add a test.
- After changing any geometry function used by the gallery, rebuild the
  gallery and visually verify the affected GLBs (delegate the render check to
  a subagent; keep screenshots out of the main context).
- **Gallery collision gate (required before commit):** every multi-body
  gallery demo is built at rest and, if it is in `ANIMATE`, across a phase
  sweep. A body pair that was clear at rest and later gains solid overlap
  fails. Designed contacts live in `gallery/collision_gate.py` →
  `ALLOW_CONTACT`. Run manually with
  `python3 gallery/collision_gate.py` (or `-q`). Parametrized pytest:
  `tests/test_gallery_collisions.py` (covered by CI). Install the local
  pre-commit hook once per clone:
  `cp scripts/pre-commit-gallery-collisions.sh .git/hooks/pre-commit && chmod +x .git/hooks/pre-commit`
  (skips when the staged diff does not touch `mechlib/`, `gallery/`, or the
  gate itself).
- Consumers install editable: `pip3 install -e ~/Desktop/myprojects/mechlib`.
  Known consumers: finnish-doors (Klonk). Planned: finnish-windows, parviz,
  dual-axis-turntable, wall-shelf-clamp.
