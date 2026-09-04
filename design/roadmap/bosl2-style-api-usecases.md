---
state: building
lens: outside-in
created: 2026-09-03
metric: public API names covered by USE_CASES or ALIASES
before: 190 (263 total; 2026-09-03 measure below)
target: 263
measure: python3 -c "import mechlib; from mechlib.usecases import USE_CASES, ALIASES; a=set(mechlib.__all__); print(len(a & (set(USE_CASES)|set(ALIASES))), len(a))"
evidence:
  - design/roadmap/evidence/2026-09-03-bosl2-gears-toc.png
  - design/roadmap/evidence/2026-09-03-bosl2-gears-helical-figures.png
  - design/roadmap/evidence/2026-09-03-mechlib-gallery-no-function-index.png
  - design/roadmap/evidence/2026-09-03-uncovered-public-api.txt
slices: 1/3
after:
---
# BOSL2-style use case for every public name

## Why, against GOAL.md

GOAL number 1 is public API covered by a use case: **190/263, target 263/263**.
BOSL2's `gears.scad` wiki is a comparable parts library: a TOC of every public
function (`circular_pitch`, `spur_gear`, `spur_gear2d`, `ring_gear`,
`ring_gear2d`, `worm`, `worm2d`, `gear_dist`, `planetary_gears`, …) each with a
one-line situation, then rendered figures (pressure-angle teeth, helical
left/right, skew-axis mesh). uiwalk of that page is
`design/roadmap/evidence/2026-09-03-bosl2-gears-toc.png` and
`…-helical-figures.png`.

mechlib's live gallery (`https://m-esm.github.io/mechlib/`) is a demo-card
browser, not that function index. uiwalk landed on an empty **Utility API**
band (`PARTS -`, `VERSION loading`) with no per-name row for the 73 uncovered
exports — `design/roadmap/evidence/2026-09-03-mechlib-gallery-no-function-index.png`.
`search_use_cases` cannot find `bolt_mesh`, `hex_nut_mesh`, `internal_gear_2d`,
`four_bar_pose`, `cam_profile_2d`, … so an agent falls back to `boxc`/`cyl`.

This proposal only adds use-case strings (and a gallery demo when the name
returns geometry). It does not add CadQuery/OpenSCAD/NopSCADlib.

## What better looks like

Every name in `mechlib.__all__` has a `USE_CASES` or `ALIASES` row so the GOAL
measure prints `263 263`. Names that return a mesh or 2D profile get a gallery
demo GLB (keeps GOAL number 2 at 100%). Pure helpers stay in the existing
Utility API list — one-line situation, no fake GLB.

## Slices

- [x] Geometry-producing uncovered names get a use case + gallery demo: `bolt_mesh`, `hex_nut_mesh`, `washer_mesh`, `cam_profile_2d`, `internal_gear_2d`, `trochoid_profile_2d`, `link_bar`, `helix_solid`, `gable_roof`, `chamfer_cutter`, `countersink`, `slot_neg`, `hex_corner_chamfer`, `extrude_down`, `extrude_poly_z`, `extrude_snapped`. `solid_cube` / `void_cube` deferred to slice 3 (boolean mesh probes already on the Utility API list; a fake GLB would drop GOAL measure 2 below 100%).
- [ ] Pose/kinematics uncovered names get a use case (demo only if a mesh is honest): `four_bar_pose`, `lazy_tongs_pose`, `tripod_pose`, `cycloidal_pose`, `escapement_pose`, `geneva_wheel_angle`, `iris_control_range`, `cam_lift`, `cv_velocity_fluctuation`, `cv_velocity_ratio`, `quick_return_ratio`.
- [ ] Remaining helpers/constants (`uni`, `sub`, `DEFAULT_SEGMENTS`, `ideg`, `rot2`, …) get one-line use cases and stay on the Utility API list; measure reads 263/263.
