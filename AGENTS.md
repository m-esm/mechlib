# Agent notes for mechlib

## What this library is

Semi-primitive **parametric mechanical geometry** for FDM (trimesh + shapely +
manifold3d). Functions return watertight meshes or dicts of named meshes.
Explicit arguments only — no project config imports.

## How to pick a part

**Use-case catalogue (required reading before inventing geometry):**

| Resource | What it is |
| --- | --- |
| `mechlib/usecases.py` | Source of truth: machinery situations per public API |
| `from mechlib.usecases import search_use_cases, use_case` | Runtime lookup for agents |
| https://m-esm.github.io/mechlib/ | Same text as "Used in" on every gallery card |

```python
from mechlib.usecases import search_use_cases, use_case

# From a job description:
search_use_cases("self-locking clamp")
# -> [('toggle_clamp', 'Welding and woodworking hold-downs, ...'), ...]

use_case("toggle_clamp")
# -> full situation string
```

Prefer the matching API over composing primitives. Only fall back to
`boxc` / `cyl` / cutters when no use case fits.

## Do not

- Add finished product models (brackets, enclosures, robot chassis) here —
  those stay in consumer projects.
- Hand-maintain parallel registries of parts; use-cases live only in
  `usecases.py` and flow into the gallery at build time.
- Skip the collision gate when changing multi-body demos
  (`python3 gallery/collision_gate.py` or pre-commit).

## Verify

```bash
python3 -m pytest tests/ -q
python3 gallery/collision_gate.py -q   # after geometry / gallery edits
```
