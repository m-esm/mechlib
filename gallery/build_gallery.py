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
from mechlib.closures import (
    clamshell_shiplap,
    fix_pin,
    nut_slot,
    press_lid,
    screw_post,
    snap_catch,
    snap_finger,
    ydovetail,
)
from mechlib.cutters import (
    bearing_seat,
    blind_socket,
    counterbore,
    crush_ribs,
    dbore,
    dbore_hub,
    ss_bore,
    teardrop,
)
from mechlib.fasteners import fastener_mesh, hex_nut_mesh, washer_mesh
from mechlib.fixtures import board_cradle
from mechlib.gears import (
    mesh_phase,
    roller_sprocket_2d,
    spur_gear_2d,
    spur_gear_mesh,
)
from mechlib.mechanisms import knurl, tap, thread_solid, torsion_spring_mesh
from mechlib.meshutil import sub, uni
from mechlib.patterns import lighten_cell_poly, lighten_grid_centres, polar_ring
from mechlib.prim import boxc, cyl, frustum, hex_poly, rbox, sector2d
from mechlib.sweep import extrude_twist, swept_keyed_bore
from mechlib.text import text_polygon


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


def cut_block(cutter, extents=(24.0, 18.0, 12.0)):
    """Subtract a centered cutter from a display block."""
    return sub(boxc(extents), cutter)


def shell_box(extents, wall=2.0):
    """Create an open-top rectangular shell centered in XY with its floor at Z zero."""
    w, d, h = extents
    outer = rbox((w, d, h), center=(0, 0, h / 2.0), r=3.0)
    inner = boxc((w - 2 * wall, d - 2 * wall, h), center=(0, 0, wall + h / 2.0))
    return sub(outer, inner)


def closure_demos():
    """Build exploded press-lid, clamshell, dovetail, and snap demonstrations."""
    base = shell_box((34, 28, 14))
    lid = press_lid(34, 28, 30, 24, (0, 0))
    lid.apply_translation((0, 0, 23))

    outer = boxc((34, 28, 14))
    lip, slot = clamshell_shiplap(outer)
    lip.apply_translation((-22, 0, 0))
    slot.apply_translation((22, 0, 8))

    tongue = ydovetail(0, -8, 8)
    tongue.apply_translation((-9, 0, 0))
    groove = ydovetail(0, -8, 8, clear=0.25)
    receiver = sub(boxc((16, 20, 10), (9, 0, 14)), groove.copy())

    catch = snap_catch("x", 0, 0, 1, 10)
    finger = snap_finger("x", 0, 0, 1, 10)
    finger.apply_translation((8, 0, 2))
    return (base, lid), (lip, slot), (tongue, receiver), (catch, finger)


def cutter_demos():
    """Build print-aware cutter demonstrations and their cut solids."""
    tear = teardrop(4, 26, axis="x", up=(0, 0, 1))
    tear_cut = cut_block(tear, (24, 18, 16))

    support_bore = ss_bore(5, 4.5, 26, (0, 0, 0), axis="x", split_z=12)
    support_cut = cut_block(support_bore, (24, 20, 20))

    socket_blank = cyl(7, 10)
    socket = sub(socket_blank, dbore(5.5, 3.7, 12, clear=0.1))
    socket.apply_translation((-10, 0, 0))
    hub = dbore_hub(7, 10, shaft_d=5.5, flat=3.7, clear=0.1)
    hub.apply_translation((10, 0, 0))

    cb = counterbore(3.4, 7.0, 3.2, 16)
    cb_cut = cut_block(cb, (18, 18, 16))

    seat = bearing_seat("608", open_column=False)
    housing = cyl(15, 10)
    housing.apply_translation((0, 0, 5))
    housing = sub(housing, seat)
    half = boxc((40, 20, 20), (0, -10, 5))
    housing = sub(housing, half)
    bearing = washer_mesh(22, 8, 7)
    bearing.apply_translation((0, 0, 1.2))
    return (tear_cut, tear), (support_cut, support_bore), (socket, hub), cb_cut, (housing, bearing)


def rib_demo():
    """Build tapered ribs around a rectangular component reference."""
    ribs = crush_ribs((18, 12, 16), 0.6, 6, 10, count=3, interference=0.12)
    component = boxc((18, 12, 16))
    return ribs, component


def nut_and_pin_demos():
    """Build a captive-nut cut and a boss plus locating pin/socket pair."""
    trap = nut_slot((0, 0, 0), length=16, nib=True)
    trap_block = sub(boxc((14, 20, 8), (0, 5, 0)), trap)
    trap_block = sub(trap_block, boxc((20, 20, 10), (-7, 5, 0)))
    nut = hex_nut_mesh(5.5, 2.6, 3.0)
    nut.apply_translation((0, 0, -1.3))

    post = screw_post((-10, 0, 0), (0, 0, 1), 10)
    post = sub(post, cyl(1.7, 12, center=(-10, 0, 5)))
    pin = fix_pin(2, 7, (0, 0, 1), (8, 0, 0))
    socket_block = boxc((12, 12, 6), (8, 0, -3))
    socket = blind_socket(2.25, 5, (0, 0, 1), (8, 0, 0))
    socket_block = sub(socket_block, socket)
    return (trap_block, nut), (post, pin, socket_block)


def gear_and_sprocket_demos():
    """Build the solid spur gear and conjugate roller sprocket assemblies."""
    gear = spur_gear_mesh(20, 1.5, 6, bore_d=5)
    sprocket_poly = roller_sprocket_2d(14, 10, 2.0, clear=0.275, outer_d=47.3)
    sprocket = trimesh.creation.extrude_polygon(sprocket_poly, 6)
    rp = 10 / (2 * math.sin(math.pi / 14))
    pins = []
    for x, y in polar_ring(14, rp):
        p = cyl(1.0, 10, center=(x, y, 3))
        pins.append(p)
    return gear, sprocket, trimesh.util.concatenate(pins)


def mechanism_demos():
    """Build threaded, knurled, and torsion-spring demonstrations."""
    bolt = thread_solid(8, 16, seg=40)
    head = cyl(7, 4)
    head.apply_translation((0, 0, -2))
    bolt = uni([head, bolt])
    bolt.apply_translation((-11, 0, 0))

    nut_blank = cyl(13 / math.sqrt(3), 7, sections=6)
    nut_blank.apply_translation((0, 0, 3.5))
    nut = tap(nut_blank, 8, (0, 0, 0), 7)
    nut = sub(nut, boxc((18, 18, 12), (9, 0, 3.5)))
    nut.apply_translation((11, 0, 0))

    knob = cyl(8, 7)
    knob.apply_translation((0, 0, 3.5))
    knob = knurl(knob, 8, 0, 7, n=20)
    spring = torsion_spring_mesh()
    return (bolt, nut), knob, spring


def panel_and_text_demos():
    """Build a hex-lightened panel and raised mechlib plaque."""
    panel = boxc((44, 30, 3), (0, 0, 1.5))
    windows = []
    for cx, cy in lighten_grid_centres(-18, -11, 18, 11, 5, 2, "hex"):
        poly = lighten_cell_poly(cx, cy, 5, "hex")
        cut = trimesh.creation.extrude_polygon(poly, 5)
        cut.apply_translation((0, 0, -1))
        windows.append(cut)
    panel = sub(panel, uni(windows))

    letters = text_polygon("mechlib", 12)
    text_mesh = mechlib.extrude_poly_z(letters, 0.0, 1.5)
    text_mesh.apply_translation((-23, -4.5, 3))
    plaque = rbox((52, 18, 3), center=(0, 0, 1.5), r=3)
    return panel, plaque, text_mesh


def fastener_demo():
    """Build three screw styles with a nut and washer stand-in."""
    screws = []
    for x, style in zip((-14, 0, 14), ("pan", "shcs", "csk")):
        screws.append(fastener_mesh(3, 14, style=style, at=(x, 0, 0)))
    nut = hex_nut_mesh(5.5, 2.6, 3.0)
    nut.apply_translation((-5, 10, 0))
    washer = washer_mesh(8, 3.4, 1.0)
    washer.apply_translation((5, 10, 0))
    return screws + [nut, washer]


def build():
    """Generate all gallery assets and their runtime manifest."""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    keyed_input, keyed_swept = keyed_bore_pair()
    gear_driver, gear_driven = gear_pair()
    cradle, board = cradle_demo()
    tear_demo, ss_demo, dbore_demo, cb_demo, bearing_demo = cutter_demos()
    press_demo, shiplap_demo, dovetail_demo, snap_demo = closure_demos()
    ribs, rib_component = rib_demo()
    nut_demo, pin_demo = nut_and_pin_demos()
    solid_gear, sprocket, pins = gear_and_sprocket_demos()
    threads, knurled, spring = mechanism_demos()
    panel, plaque, lettering = panel_and_text_demos()
    fasteners = fastener_demo()

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

    models.extend([
        {
            "file": "teardrop_demo.glb",
            "name": "teardrop",
            "module": "mechlib.cutters",
            "signature": signature(teardrop),
            "description": "A support-free teardrop bore shown through a cut block.",
            "origin": "Extracted from head.py in parviz.",
            "meshes": [("cut_block", tear_demo[0], PALETTE[0])],
        },
        {
            "file": "ss_bore_demo.glb",
            "name": "ss_bore",
            "module": "mechlib.cutters",
            "signature": signature(ss_bore),
            "description": "A round lower cradle and support-light chamfered upper bore.",
            "origin": "Extracted from geom_util.py in finnish-doors.",
            "meshes": [("housing_cut", ss_demo[0], PALETTE[1])],
        },
        {
            "file": "dbore_demo.glb",
            "name": "dbore + dbore_hub",
            "module": "mechlib.cutters",
            "signature": "%s; %s" % (signature(dbore), signature(dbore_hub)),
            "description": "A double-D socket blank beside a ready-made keyed hub.",
            "origin": "Unified from parviz and finnish-windows.",
            "meshes": [
                ("double_d_socket", dbore_demo[0], PALETTE[2]),
                ("double_d_hub", dbore_demo[1], PALETTE[3]),
            ],
        },
        {
            "file": "counterbore_demo.glb",
            "name": "counterbore",
            "module": "mechlib.cutters",
            "signature": signature(counterbore),
            "description": "A through-hole with a cylindrical recessed-head pocket.",
            "origin": "New abstraction inspired by finnish-doors fastener seats.",
            "meshes": [("counterbored_block", cb_demo, PALETTE[7])],
        },
        {
            "file": "bearing_seat_demo.glb",
            "name": "bearing_seat 608",
            "module": "mechlib.cutters",
            "signature": signature(bearing_seat),
            "description": "A cut-away retained 608 seat with a bearing reference ring.",
            "origin": "New abstraction from the Klonk 22.25 mm 608 pocket practice.",
            "meshes": [
                ("seat_cutaway", bearing_demo[0], PALETTE[8]),
                ("608_reference", bearing_demo[1], PALETTE[12]),
            ],
        },
        {
            "file": "crush_ribs_demo.glb",
            "name": "crush_ribs",
            "module": "mechlib.cutters",
            "signature": signature(crush_ribs),
            "description": "Six tapered vertical ribs squeezing a rectangular component.",
            "origin": "Generalized from tcst_hold_ribs in finnish-doors.",
            "meshes": [
                ("tapered_ribs", ribs, PALETTE[5]),
                ("component_reference", rib_component, PALETTE[11]),
            ],
        },
        {
            "file": "press_lid_demo.glb",
            "name": "press_lid",
            "module": "mechlib.closures",
            "signature": signature(press_lid),
            "description": "A hollow box and its friction-plug lid in an exploded pose.",
            "origin": "Extracted from system_layout.py in finnish-doors.",
            "meshes": [
                ("open_box", press_demo[0], PALETTE[1]),
                ("exploded_lid", press_demo[1], PALETTE[0]),
            ],
        },
        {
            "file": "clamshell_shiplap_demo.glb",
            "name": "clamshell_shiplap",
            "module": "mechlib.closures",
            "signature": signature(clamshell_shiplap),
            "description": "The matching raised base lip and oversized lid receiver slot.",
            "origin": "Extracted from housings.py in finnish-doors.",
            "meshes": [
                ("base_lip", shiplap_demo[0], PALETTE[9]),
                ("lid_slot", shiplap_demo[1], PALETTE[10]),
            ],
        },
        {
            "file": "ydovetail_demo.glb",
            "name": "ydovetail pair",
            "module": "mechlib.closures",
            "signature": signature(ydovetail),
            "description": "A self-supporting slide tongue beside its clearanced receiver.",
            "origin": "Extracted from system_layout.py in finnish-doors.",
            "meshes": [
                ("dovetail_tongue", dovetail_demo[0], PALETTE[4]),
                ("dovetail_receiver", dovetail_demo[1], PALETTE[6]),
            ],
        },
        {
            "file": "snap_pair_demo.glb",
            "name": "snap_catch + snap_finger",
            "module": "mechlib.closures",
            "signature": "%s; %s" % (signature(snap_catch), signature(snap_finger)),
            "description": "A ramped retention catch and matching cantilever hook.",
            "origin": "Unified from system_layout.py and build_powerbank.py.",
            "meshes": [
                ("snap_catch", snap_demo[0], PALETTE[5]),
                ("snap_finger", snap_demo[1], PALETTE[8]),
            ],
        },
        {
            "file": "nut_slot_demo.glb",
            "name": "nut_slot",
            "module": "mechlib.closures",
            "signature": signature(nut_slot),
            "description": "A cut-away seated nut trap with its M3 nut stand-in.",
            "origin": "Extracted from geo.py in parviz.",
            "meshes": [
                ("trap_cutaway", nut_demo[0], PALETTE[2]),
                ("nut_standin", nut_demo[1], PALETTE[10]),
            ],
        },
        {
            "file": "pins_and_posts_demo.glb",
            "name": "screw_post + fix_pin + blind_socket",
            "module": "mechlib.closures / mechlib.cutters",
            "signature": "%s; %s; %s" % (
                signature(screw_post), signature(fix_pin), signature(blind_socket)),
            "description": "A bored screw boss beside a locating pin and blind socket block.",
            "origin": "Extracted from geo.py in parviz.",
            "meshes": [
                ("screw_post", pin_demo[0], PALETTE[3]),
                ("fix_pin", pin_demo[1], PALETTE[5]),
                ("socket_block", pin_demo[2], PALETTE[7]),
            ],
        },
        {
            "file": "spur_gear_mesh_demo.glb",
            "name": "spur_gear_mesh",
            "module": "mechlib.gears",
            "signature": signature(spur_gear_mesh),
            "description": "A printable involute spur gear with a measured round bore.",
            "origin": "Unified from finnish-windows and parviz gear builders.",
            "meshes": [("20_tooth_gear", solid_gear, PALETTE[9])],
        },
        {
            "file": "roller_sprocket_demo.glb",
            "name": "roller_sprocket_2d + polar_ring",
            "module": "mechlib.gears / mechlib.patterns",
            "signature": "%s; %s" % (signature(roller_sprocket_2d), signature(polar_ring)),
            "description": "A conjugate 14-tooth sprocket surrounded by track-pin references.",
            "origin": "Generalized from tracks.py in parviz.",
            "meshes": [
                ("roller_sprocket", sprocket, PALETTE[11]),
                ("pitch_circle_pins", pins, PALETTE[4]),
            ],
        },
        {
            "file": "thread_demo.glb",
            "name": "thread_solid + tap",
            "module": "mechlib.mechanisms",
            "signature": "%s; %s" % (signature(thread_solid), signature(tap)),
            "description": "An M8 threaded bolt beside a cut-away tapped hex nut.",
            "origin": "Extracted from threads.py in parviz.",
            "meshes": [
                ("m8_bolt", threads[0], PALETTE[12]),
                ("tapped_nut_cutaway", threads[1], PALETTE[10]),
            ],
        },
        {
            "file": "knurl_demo.glb",
            "name": "knurl",
            "module": "mechlib.mechanisms",
            "signature": signature(knurl),
            "description": "A cylindrical thumb knob with printable vertical flutes.",
            "origin": "Extracted from m4_bolt.py in parviz.",
            "meshes": [("knurled_knob", knurled, PALETTE[6])],
        },
        {
            "file": "torsion_spring_demo.glb",
            "name": "torsion_spring_mesh",
            "module": "mechlib.mechanisms",
            "signature": signature(torsion_spring_mesh),
            "description": "A five-turn torsion spring preview with moving and ground legs.",
            "origin": "Extracted from shaft.py in finnish-doors.",
            "meshes": [("torsion_spring", spring, PALETTE[12])],
        },
        {
            "file": "lighten_grid_demo.glb",
            "name": "lighten_grid panel",
            "module": "mechlib.patterns",
            "signature": "%s; %s" % (
                signature(lighten_grid_centres), signature(lighten_cell_poly)),
            "description": "A structural panel opened by a staggered hexagonal lattice.",
            "origin": "Extracted from housings.py in finnish-doors.",
            "meshes": [("lightened_panel", panel, PALETTE[2])],
        },
        {
            "file": "text_polygon_demo.glb",
            "name": "text_polygon",
            "module": "mechlib.text",
            "signature": signature(text_polygon),
            "description": "The word mechlib converted to counter-preserving raised geometry.",
            "origin": "Extracted from housings.py in finnish-doors.",
            "meshes": [
                ("plaque", plaque, PALETTE[7]),
                ("mechlib_text", lettering, PALETTE[0]),
            ],
        },
        {
            "file": "fastener_trio_demo.glb",
            "name": "fastener_mesh + nut + washer",
            "module": "mechlib.fasteners",
            "signature": "%s; %s; %s" % (
                signature(fastener_mesh), signature(hex_nut_mesh), signature(washer_mesh)),
            "description": "Pan, socket, and countersunk screws with nut and washer stand-ins.",
            "origin": "Unified from all three source projects.",
            "meshes": [
                ("pan_screw", fasteners[0], PALETTE[5]),
                ("shcs_screw", fasteners[1], PALETTE[9]),
                ("csk_screw", fasteners[2], PALETTE[10]),
                ("hex_nut", fasteners[3], PALETTE[3]),
                ("washer", fasteners[4], PALETTE[12]),
            ],
        },
    ])

    manifest_models = []
    for model in models:
        export_model(model["file"], model.pop("meshes"))
        manifest_models.append(model)

    utility_functions = [
        ("mechlib.meshutil", mechlib.to_manifold, "Convert trimesh geometry to manifold3d."),
        ("mechlib.meshutil", mechlib.from_manifold, "Convert manifold3d results back to trimesh."),
        ("mechlib.meshutil", mechlib.sub, "Subtract one watertight solid from another."),
        ("mechlib.meshutil", mechlib.uni, "Union a sequence of watertight solids."),
        ("mechlib.meshutil", mechlib.inter, "Intersect two watertight solids."),
        ("mechlib.meshutil", mechlib.export_stl, "Export STL through a watertightness gate."),
        ("mechlib.meshutil", mechlib.inflate, "Offset mesh vertices along their normals."),
        ("mechlib.meshutil", mechlib.bbox_overlap, "Check axis-aligned bounding-box overlap."),
        ("mechlib.meshutil", mechlib.overlap_volume, "Measure exact manifold overlap volume."),
        ("mechlib.meshutil", mechlib.inside, "Probe points expected inside a solid."),
        ("mechlib.meshutil", mechlib.clear, "Probe points expected outside a solid."),
        ("mechlib.meshutil", mechlib.bore_pierces, "Probe a bore along its real axis."),
        ("mechlib.meshutil", mechlib.void_cube, "Confirm local void with a cube intersection."),
        ("mechlib.meshutil", mechlib.solid_cube, "Confirm local material with a cube intersection."),
        ("mechlib.meshutil", mechlib.self_thickness, "Sample ray-based wall thickness."),
        ("mechlib.meshutil", mechlib.cube_rotations, "Return the 24 proper cube rotations."),
        ("mechlib.meshutil", mechlib.fit_transform, "Fit a real mesh to an oriented placeholder."),
        ("mechlib.meshutil", mechlib.decimate, "Cluster vertices on a regular grid."),
        ("mechlib.meshutil", mechlib.orient, "Rotate positive Z onto a normal vector."),
        ("mechlib.meshutil", mechlib.extrude_poly_z, "Extrude Polygon or MultiPolygon between Z planes."),
        ("mechlib.meshutil", mechlib.largest_poly, "Select the largest Shapely polygon."),
        ("mechlib.mechanisms", mechlib.coarse_pitch, "Look up a printable coarse metric pitch."),
        ("mechlib.fasteners", mechlib.pick_length, "Select the next usable standard screw length."),
        ("mechlib.closures", mechlib.nut_ac, "Convert nut across-flats to across-corners."),
    ]
    utilities = [
        {"module": module, "signature": signature(function), "description": description}
        for module, function, description in utility_functions
    ]
    manifest = {
        "version": mechlib.__version__,
        "models": manifest_models,
        "utilities": utilities,
    }
    index_path = OUTPUT_DIR / "index.json"
    index_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print("wrote %s" % index_path.relative_to(ROOT))


if __name__ == "__main__":
    build()
