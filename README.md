# mechlib

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

`mechlib` is a standalone collection of pure parametric geometry primitives for FDM-printed mechanical design using trimesh, Shapely, and manifold3d. The primitives were mined from finnish-doors (the Klonk door CAD project), finnish-windows, and parviz, which contained three or more independent reimplementations of the same geometry helpers.

## API

### `mechlib.prim`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `cyl` | Create and orient a cylinder mesh. | finnish-doors `src/shared/geom_util.py` |
| `boxc` | Create a box centered at a point. | finnish-doors `src/shared/geom_util.py` |
| `rbox` | Extrude a box with rounded vertical corners. | finnish-doors `src/shared/geom_util.py` |
| `frustum` | Create a truncated cone mesh. | finnish-doors `src/shared/geom_util.py` |
| `sector2d` | Create a circular-sector polygon. | finnish-doors `src/shared/geom_util.py` |
| `rot2` | Rotate two-dimensional points by degrees. | finnish-doors `src/shared/geom_util.py` |
| `ideg` | Compute the involute-angle function in degrees. | finnish-doors `src/shared/geom_util.py` |
| `mesh_from_tris` | Build and repair a mesh from triangle tuples. | finnish-doors `src/shared/geom_util.py` |
| `largest` | Select the largest connected mesh component. | finnish-doors `src/shared/geom_util.py` |
| `extrude_down` | Extrude Polygon or MultiPolygon downward. | finnish-doors `src/shared/geom_util.py` |
| `hex_poly` | Create a regular hexagon from across-flats width. | finnish-doors `src/projects/klonk/gears2d.py` |

### `mechlib.sweep` and `mechlib.fixtures`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `extrude_twist` | Extrude a profile through an explicit angular sweep. | finnish-doors `src/shared/geom_util.py` |
| `swept_keyed_bore` | Sweep a keyed bore polygon through free rotation. | finnish-doors `src/projects/klonk/shaft.py` |
| `board_cradle` | Build four corner standoffs and capture walls for a PCB. | finnish-doors `src/projects/klonk/system_layout.py` |

### `mechlib.meshutil`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `to_manifold` | Convert a trimesh mesh to manifold3d. | parviz `src/geo.py` |
| `from_manifold` | Convert manifold3d output to trimesh. | parviz `src/geo.py` |
| `sub` | Subtract one watertight mesh from another. | parviz `src/geo.py` |
| `uni` | Union a sequence of watertight meshes. | parviz `src/geo.py` |
| `inter` | Intersect two watertight meshes. | parviz `src/geo.py` |
| `export_stl` | Export STL through repair and geometry guards. | parviz `src/geo.py` |
| `inflate` | Offset vertices outward along vertex normals. | finnish-windows `src/build_combined.py` |
| `bbox_overlap` | Test axis-aligned bounding-box overlap. | parviz `src/assembly_check.py` |
| `overlap_volume` | Measure exact manifold intersection volume. | parviz `src/assembly_check.py` |
| `inside` | Check that all probes lie inside a solid. | parviz `src/checks.py` |
| `clear` | Check that no probes lie inside a solid. | parviz `src/checks.py` |
| `bore_pierces` | Probe bore clearance along its real axis. | parviz `src/checks.py` |
| `void_cube` | Confirm a local void by cube intersection. | parviz `src/checks.py` |
| `solid_cube` | Confirm local material by cube intersection. | parviz `src/checks.py` |
| `self_thickness` | Sample ray-based material thickness. | parviz `src/wallcheck.py` |
| `cube_rotations` | Return all 24 proper cube rotations. | parviz `src/refparts.py` |
| `fit_transform` | Rigidly fit a real mesh to a placeholder. | parviz `src/refparts.py` |
| `decimate` | Cluster mesh vertices on a regular grid. | parviz `src/refparts.py` |
| `orient` | Rotate positive Z onto a normal. | parviz `src/geo.py` |
| `extrude_poly_z` | Extrude Polygon or MultiPolygon between Z planes. | finnish-doors `src/projects/klonk/housings.py` |
| `largest_poly` | Select the largest polygon in a geometry. | finnish-doors `src/projects/klonk/housings.py` |

### `mechlib.cutters`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `teardrop` | Build a print-aware bore cutter with an arbitrary up vector. | parviz `src/head.py` |
| `ss_bore` | Build a support-light clamshell bore. | finnish-doors `src/shared/geom_util.py` |
| `dbore` | Build a unified explicit-dimension double-D bore. | parviz and finnish-windows |
| `dbore_hub` | Build a cylindrical hub with a double-D socket. | parviz `src/geo.py` |
| `chamfer_cutter` | Build a 45 degree end-chamfer cutter. | parviz `src/standins/foot_pin.py` |
| `hex_corner_chamfer` | Chamfer hex corners while preserving flats. | parviz `src/standins/m8_nut.py` |
| `countersink` | Cut a bore lead-in down to a thread crest. | parviz `src/standins/m8_nut.py` |
| `slot_neg` | Build an adjustable pedestal obround negative. | parviz `src/chassis.py` |
| `blind_socket` | Build a blind locating-pin socket. | parviz `src/geo.py` |
| `gable_roof` | Build a two-slope support-light opening roof. | finnish-doors `src/projects/klonk/housings.py` |
| `counterbore` | Build a through-hole plus cylindrical head pocket. | New, inspired by finnish-doors fastener seats |
| `bearing_seat` | Build 608, 695, or MR105 pocket cutters. | New, inspired by finnish-doors 608 seats |
| `crush_ribs` | Build tapered ribs on opposing pocket walls. | New, generalized from finnish-doors `motors.py` |

### `mechlib.closures`

| Function or type | Purpose | Origin project |
| --- | --- | --- |
| `press_lid` | Build a plate and hollow friction plug. | finnish-doors `src/projects/klonk/system_layout.py` |
| `clamshell_shiplap` | Build a perimeter lip and matching lid slot. | finnish-doors `src/projects/klonk/housings.py` |
| `ydovetail` | Build a self-supporting Y-axis dovetail prism. | finnish-doors `src/projects/klonk/system_layout.py` |
| `SnapSpec` | Hold unified snap-catch dimensions. | finnish-doors `system_layout.py` and `build_powerbank.py` |
| `snap_catch` | Build a ramped outer-wall catch. | finnish-doors `system_layout.py` and `build_powerbank.py` |
| `snap_finger` | Build a cantilever finger and hook. | finnish-doors `system_layout.py` and `build_powerbank.py` |
| `nut_ac` | Convert nut across-flats to across-corners. | parviz `src/geo.py` |
| `nut_slot` | Build a seated captive-nut trap negative. | parviz `src/geo.py` |
| `screw_post` | Build a cylindrical boss along a normal. | parviz `src/geo.py` |
| `fix_pin` | Build a locating pin with a buried root. | parviz `src/geo.py` |

### `mechlib.gears`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `spur_gear_2d` | Generate an involute spur or sector gear polygon. | finnish-doors `src/projects/klonk/gears2d.py` |
| `mesh_phase` | Compute driven-gear tooth phase. | finnish-doors `src/projects/klonk/gears2d.py` |
| `spur_gear_mesh` | Extrude a spur gear and cut a round bore. | New, unifying finnish-windows and parviz |
| `roller_sprocket_2d` | Generate a conjugate pin-envelope sprocket. | New, generalized from parviz `src/tracks.py` |

### `mechlib.mechanisms`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `coarse_pitch` | Look up a printable coarse metric pitch. | parviz `src/threads.py` |
| `helix_solid` | Sweep and cap a watertight helical profile. | parviz `src/threads.py` |
| `thread_solid` | Build a printable external thread or internal cutter. | parviz `src/threads.py` |
| `tap` | Cut an internal thread into a solid. | parviz `src/threads.py` |
| `knurl` | Cut vertical grip flutes around a cylinder. | parviz `src/standins/m4_bolt.py` |
| `torsion_spring_mesh` | Build a torsion-spring assembly preview. | finnish-doors `src/projects/klonk/shaft.py` |

### `mechlib.fasteners`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `zmin0` | Move a mesh onto the Z zero plane. | parviz `src/standins/_common.py` |
| `bolt_mesh` | Build a head-down bolt stand-in. | parviz `src/standins/_common.py` |
| `hex_nut_mesh` | Build a bored hex-nut stand-in. | parviz `src/standins/_common.py` |
| `washer_mesh` | Build an annular washer stand-in. | parviz `src/standins/_common.py` |
| `fastener_mesh` | Build and orient pan, SHCS, or CSK fasteners. | Unified from all three source projects |
| `pick_length` | Select the next usable standard screw length. | finnish-windows `tools/add_screws_glb.py` |

### `mechlib.patterns` and `mechlib.text`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `polar_ring` | Return evenly spaced XY ring points. | Unified from finnish-windows `tools/add_teeth.py` |
| `lighten_cell_poly` | Build one rectangular or hex lightening cell. | finnish-doors `src/projects/klonk/housings.py` |
| `lighten_grid_centres` | Yield rect or hex lattice centers over a box. | finnish-doors `src/projects/klonk/housings.py` |
| `text_polygon` | Convert text to counter-preserving polygons. | finnish-doors `src/projects/klonk/housings.py` |

## DECOUPLING CONTRACT

mechlib functions take explicit arguments only. They never read project parameter modules, never import from consuming projects, and never hold mutable module state. Consumers keep thin facade modules that bind their own params.

Consumers are finnish-doors (Klonk), finnish-windows, and parviz.

## Gallery

Explore every visual component in the [interactive 3D gallery](https://m-esm.github.io/mechlib/). Run `python3 gallery/build_gallery.py` from the repository root to regenerate `docs/models/`.

Licensed under the MIT License.
