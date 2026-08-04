# mechlib

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

`mechlib` is a standalone collection of pure parametric geometry primitives for FDM-printed mechanical design using trimesh, Shapely, and manifold3d. The primitives were mined from finnish-doors (the Klonk door CAD project), finnish-windows, and parviz, which contained three or more independent reimplementations of the same geometry helpers.

## API

| Function | Purpose | Origin module |
| --- | --- | --- |
| `cyl` | Create and orient a cylinder mesh. | `geom_util.py` |
| `boxc` | Create a box mesh centered at a point. | `geom_util.py` |
| `rbox` | Extrude a box with rounded vertical corners. | `geom_util.py` |
| `frustum` | Create a truncated cone mesh. | `geom_util.py` |
| `sector2d` | Create a two-dimensional circular sector polygon. | `geom_util.py` |
| `rot2` | Rotate two-dimensional points by degrees. | `geom_util.py` |
| `ideg` | Compute the involute-angle function in degrees. | `geom_util.py` |
| `mesh_from_tris` | Build and repair a mesh from triangle tuples. | `geom_util.py` |
| `largest` | Select the largest connected mesh component. | `geom_util.py` |
| `extrude_down` | Extrude Shapely polygons downward from a top plane. | `geom_util.py` |
| `hex_poly` | Create a regular hexagon from its across-flats width. | `gears2d.py` |
| `extrude_twist` | Extrude a profile through an explicit angular sweep. | `geom_util.py` |
| `swept_keyed_bore` | Sweep a keyed bore polygon through free rotation. | `shaft.py` |
| `spur_gear_2d` | Generate an involute spur or sector gear polygon. | `gears2d.py` |
| `mesh_phase` | Compute the driven-gear phase for tooth alignment. | `gears2d.py` |
| `board_cradle` | Build four corner standoffs and capture walls for a PCB. | `system_layout.py` |

## DECOUPLING CONTRACT

mechlib functions take explicit arguments only. They never read project parameter modules, never import from consuming projects, and never hold mutable module state. Consumers keep thin facade modules that bind their own params.

Consumers are finnish-doors (Klonk), finnish-windows, and parviz.

Licensed under the MIT License.
