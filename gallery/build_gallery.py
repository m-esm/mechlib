#!/usr/bin/env python3
"""Build the static mechlib model gallery."""

import inspect
import json
import math
import sys
from pathlib import Path

import numpy as np
import trimesh
from shapely import affinity
from shapely.geometry import Point, box


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import mechlib
from mechlib.fixtures import board_cradle
from mechlib.gears import mesh_phase, spur_gear_2d
from mechlib.prim import boxc, cyl, frustum, hex_poly, rbox, sector2d
from mechlib.sweep import extrude_twist, swept_keyed_bore


OUTPUT_DIR = ROOT / "docs" / "models"
PALETTE = (
    (64, 196, 255, 255),
    (255, 166, 77, 255),
    (129, 224, 153, 255),
    (205, 145, 255, 255),
    (255, 111, 145, 255),
    (255, 218, 92, 255),
    (72, 219, 202, 255),
    (116, 152, 255, 255),
    (249, 132, 219, 255),
    (154, 219, 86, 255),
    (255, 125, 89, 255),
    (99, 207, 255, 255),
    (239, 181, 255, 255),
)


def signature(function):
    """Return the live function signature for gallery metadata."""
    return "%s%s" % (function.__name__, inspect.signature(function))


def color_mesh(mesh, color, name):
    """Copy a mesh, assign opaque vertex colors, and give it a stable name."""
    colored = mesh.copy()
    colored.visual.vertex_colors = np.tile(
        np.asarray(color, dtype=np.uint8), (len(colored.vertices), 1)
    )
    colored.metadata["name"] = name
    return colored


def export_model(filename, meshes):
    """Export named meshes as one GLB scene."""
    scene = trimesh.Scene()
    for name, mesh, color in meshes:
        colored = color_mesh(mesh, color, name)
        scene.add_geometry(colored, geom_name=name, node_name=name)
    path = OUTPUT_DIR / filename
    path.write_bytes(scene.export(file_type="glb"))
    print("wrote %s (%d bytes)" % (path.relative_to(ROOT), path.stat().st_size))


def lobed_twist():
    """Create a three-lobed solid with one full turn over 30 mm."""
    count = 96
    angles = np.linspace(0.0, 2.0 * math.pi, count, endpoint=False)
    radii = 7.5 + 2.0 * np.cos(3.0 * angles)
    profile = [(r * math.cos(a), r * math.sin(a)) for r, a in zip(radii, angles)]
    heights = np.linspace(0.0, 30.0, 81)
    return extrude_twist(profile, None, heights, lambda z: 12.0 * z)


def keyed_bore_pair():
    """Create an input D-bore and its free-rotation swept envelope."""
    keyed = Point(0.0, 0.0).buffer(7.0, resolution=48).intersection(
        box(-7.1, -7.1, 5.2, 7.1)
    )
    swept = swept_keyed_bore(keyed, 50)
    input_mesh = trimesh.creation.extrude_polygon(keyed, 4.0)
    swept_mesh = trimesh.creation.extrude_polygon(swept, 4.0)
    swept_mesh.apply_translation((22.0, 0.0, 0.0))
    return input_mesh, swept_mesh


def gear_pair():
    """Create a correctly phased 18:28 involute gear pair and assert clearance."""
    n_driver, n_driven, module = 18, 28, 1.5
    driver_poly = spur_gear_2d(N=n_driver, m=module, pa=20.0, bl=0.35)
    driven_poly = spur_gear_2d(N=n_driven, m=module, pa=20.0, bl=0.35)
    phase = mesh_phase(n_driver, n_driven, 0.0)
    center_distance = module * (n_driver + n_driven) / 2.0
    driven_poly = affinity.rotate(driven_poly, phase, origin=(0.0, 0.0))
    driven_poly = affinity.translate(driven_poly, xoff=center_distance)

    driver = trimesh.creation.extrude_polygon(driver_poly, 5.0)
    driven = trimesh.creation.extrude_polygon(driven_poly, 5.0)
    overlap = trimesh.boolean.intersection([driver, driven], engine="manifold")
    overlap_volume = (
        0.0 if overlap is None or len(overlap.faces) == 0 else abs(float(overlap.volume))
    )
    assert overlap_volume < 0.01, "gear overlap is %.6f mm^3" % overlap_volume
    print("gear intersection volume: %.6f mm^3" % overlap_volume)
    return driver, driven


def cradle_demo():
    """Create four corner cradle assemblies and a board reference solid."""
    rect = (-20.0, -15.0, 40.0, 30.0, 8.0)
    cradle = board_cradle(rect, fl=0.0)
    board = boxc((40.0, 30.0, 1.6), center=(0.0, 0.0, 4.8))
    return cradle, board


def build():
    """Generate all gallery assets and their runtime manifest."""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    keyed_input, keyed_swept = keyed_bore_pair()
    gear_driver, gear_driven = gear_pair()
    cradle, board = cradle_demo()

    models = [
        {
            "file": "cyl_demo.glb",
            "name": "cyl",
            "module": "mechlib.prim",
            "signature": signature(cyl),
            "description": "A configurable cylinder oriented along any principal axis.",
            "origin": "Originally extracted from geom_util.py in finnish-doors.",
            "meshes": [("cylinder", cyl(r=8, h=20), PALETTE[0])],
        },
        {
            "file": "boxc_demo.glb",
            "name": "boxc",
            "module": "mechlib.prim",
            "signature": signature(boxc),
            "description": "A box positioned by its center and XYZ extents.",
            "origin": "Originally extracted from geom_util.py in finnish-doors.",
            "meshes": [("centered_box", boxc((24, 16, 10)), PALETTE[1])],
        },
        {
            "file": "rbox_demo.glb",
            "name": "rbox",
            "module": "mechlib.prim",
            "signature": signature(rbox),
            "description": "A clean enclosure block with rounded vertical corners.",
            "origin": "Originally extracted from geom_util.py in finnish-doors.",
            "meshes": [("rounded_box", rbox((24, 16, 10), r=4), PALETTE[2])],
        },
        {
            "file": "frustum_demo.glb",
            "name": "frustum",
            "module": "mechlib.prim",
            "signature": signature(frustum),
            "description": "A truncated cone between two radii and two Z planes.",
            "origin": "Originally extracted from geom_util.py in finnish-doors.",
            "meshes": [("frustum", frustum(6, 10, 16), PALETTE[3])],
        },
        {
            "file": "sector2d_demo.glb",
            "name": "sector2d",
            "module": "mechlib.prim",
            "signature": signature(sector2d),
            "description": "A circular sector polygon, extruded here to reveal its usable profile.",
            "origin": "Originally extracted from geom_util.py in finnish-doors.",
            "meshes": [
                (
                    "extruded_sector",
                    trimesh.creation.extrude_polygon(sector2d(-30, 210, 18), 4.0),
                    PALETTE[4],
                )
            ],
        },
        {
            "file": "hex_poly_demo.glb",
            "name": "hex_poly",
            "module": "mechlib.prim",
            "signature": signature(hex_poly),
            "description": "A regular hexagon defined by across-flats width, shown as a solid.",
            "origin": "Originally extracted from gears2d.py in finnish-doors.",
            "meshes": [
                (
                    "extruded_hexagon",
                    trimesh.creation.extrude_polygon(hex_poly(af=16), 4.0),
                    PALETTE[5],
                )
            ],
        },
        {
            "file": "extrude_twist_demo.glb",
            "name": "extrude_twist",
            "module": "mechlib.sweep",
            "signature": signature(extrude_twist),
            "description": "A three-lobed profile swept through one full turn over 30 mm.",
            "origin": "Originally extracted from geom_util.py in finnish-doors.",
            "meshes": [("three_lobed_twist", lobed_twist(), PALETTE[6])],
        },
        {
            "file": "swept_keyed_bore_demo.glb",
            "name": "swept_keyed_bore",
            "module": "mechlib.sweep",
            "signature": signature(swept_keyed_bore),
            "description": "A D-shaped keyed bore beside its 50 degree free-rotation envelope.",
            "origin": "Originally extracted from shaft.py in finnish-doors.",
            "meshes": [
                ("input_keyed_bore", keyed_input, PALETTE[7]),
                ("swept_envelope", keyed_swept, PALETTE[8]),
            ],
        },
        {
            "file": "spur_gear_pair_demo.glb",
            "name": "spur_gear_2d + mesh_phase",
            "module": "mechlib.gears",
            "signature": "%s; phased with %s" % (
                signature(spur_gear_2d),
                signature(mesh_phase),
            ),
            "description": "An 18-tooth driver and 28-tooth driven gear in collision-free mesh.",
            "origin": "Originally extracted from gears2d.py in finnish-doors.",
            "meshes": [
                ("18_tooth_driver", gear_driver, PALETTE[9]),
                ("28_tooth_driven", gear_driven, PALETTE[10]),
            ],
        },
        {
            "file": "board_cradle_demo.glb",
            "name": "board_cradle",
            "module": "mechlib.fixtures",
            "signature": signature(board_cradle),
            "description": "Four PCB corner standoffs and capture walls around a 40 x 30 mm board.",
            "origin": "Originally extracted from system_layout.py in finnish-doors.",
            "meshes": [
                ("corner_cradles", cradle, PALETTE[11]),
                ("board_reference", board, PALETTE[12]),
            ],
        },
    ]

    manifest_models = []
    for model in models:
        export_model(model["file"], model.pop("meshes"))
        manifest_models.append(model)

    manifest = {"version": mechlib.__version__, "models": manifest_models}
    index_path = OUTPUT_DIR / "index.json"
    index_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print("wrote %s" % index_path.relative_to(ROOT))


if __name__ == "__main__":
    build()
