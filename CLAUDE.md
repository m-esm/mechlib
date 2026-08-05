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
  fasteners, fixtures, meshutil, packing, patterns, text, stepio).
- `gallery/build_gallery.py` — regenerates `docs/models/*.glb` plus
  `docs/models/index.json`. Run from repo root: `python3 gallery/build_gallery.py`.
- `docs/` — GitHub Pages site (interactive gallery) at
  https://m-esm.github.io/mechlib/, served from `main:/docs`. Committing
  rebuilt GLBs publishes them.
- `tests/` — pytest suite: `python3 -m pytest tests/ -q`.

## Workflow

- Version lives in BOTH `pyproject.toml` and `mechlib/__init__.py`; bump
  together.
- New public function: add to module, re-export in `mechlib/__init__.py`
  (`__all__` too), add a README API-table row, add a minimal gallery demo,
  add a test.
- After changing any geometry function used by the gallery, rebuild the
  gallery and visually verify the affected GLBs (delegate the render check to
  a subagent; keep screenshots out of the main context).
- Consumers install editable: `pip3 install -e ~/Desktop/myprojects/mechlib`.
  Known consumers: finnish-doors (Klonk). Planned: finnish-windows, parviz,
  dual-axis-turntable, wall-shelf-clamp.
