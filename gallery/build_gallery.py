#!/usr/bin/env python3
"""Build the static mechlib model gallery."""

import importlib.util
import inspect
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import trimesh


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import mechlib
from mechlib.cams import (
    barrel_cam,
    cam_lift,
    heart_cam,
    plate_cam,
    snail_cam,
)
from mechlib.closures import (
    clamshell_shiplap,
    fix_pin,
    nut_slot,
    press_lid,
    push_pin,
    screw_post,
    snap_catch,
    snap_finger,
    setscrew,
    ydovetail,
)
from mechlib.clutches import freewheel_clutch, torque_limiter
from mechlib.couplings import jaw_coupling, oldham_coupling, universal_joint
from mechlib.cutters import (
    bearing_seat,
    blind_socket,
    counterbore,
    crush_ribs,
    dbore,
    dbore_hub,
    revolved_gable_cavity,
    slot_cutter,
    ss_bore,
    teardrop,
    tapered_cavity,
    u_channel_between,
)
from mechlib.drives import (
    flat_worm,
    planet_stage,
    printed_worm,
    worm_coupon,
    worm_wheel_band,
)
from mechlib.fasteners import fastener_mesh, hex_nut_mesh, washer_mesh
from mechlib.fixtures import board_cradle, saddle
from mechlib.flexures import bistable_beam, cross_flexure, wave_spring
from mechlib.gears import (
    bevel_gear_pair,
    cycloidal_drive,
    herringbone_gear,
    mesh_phase,
    rack_2d,
    roller_sprocket_2d,
    spur_gear_2d,
    spur_gear,
    spur_gear_mesh,
    worm,
)
from mechlib.indexing import escapement, geneva_pair, intermittent_gear_pair
from mechlib.linear import (
    archimedes_screw,
    differential_screw,
    scroll_drive,
)
from mechlib.linkages import (
    four_bar,
    quick_return,
    scotch_yoke,
    toggle_clamp,
)
from mechlib.mechanisms import (
    dog_slot_coupling,
    helix_tube,
    knurl,
    tap,
    thread_solid,
    threaded_rod,
    torsion_spring_mesh,
)
from mechlib.patterns import directed_holes, lighten_cell_poly, lighten_grid_centres, polar_ring
from mechlib.prim import boxc, chamfer_prism, cyl, frustum, hex_poly, rbox, sector2d, seg_cylinder
from mechlib.pulleys import grooved_drum, timing_pulley
from mechlib.ratchets import (
    arc_ratchet_2d,
    check_ratchet_sense_and_sweep,
    compliant_clutch,
    compliant_clutch_2d,
    pip_ratchet_hub,
    pip_ratchet_hub_2d,
    ratchet_ring,
    ratchet_ring_2d,
    spring_cartridge_ratchet,
    spring_cartridge_ratchet_2d,
)
from mechlib.sweep import extrude_twist, loft, ring_pts, swept_keyed_bore
from mechlib.text import text_block, text_polygon


def _load_demos():
    """Load gallery/demos.py as a module (works when run as a script)."""
    path = Path(__file__).resolve().parent / "demos.py"
    spec = importlib.util.spec_from_file_location("gallery_demos", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


demos = _load_demos()
PALETTE = demos.PALETTE
PLAY = demos.PLAY

OUTPUT_DIR = ROOT / "docs" / "models"
WHEELS_DIR = ROOT / "docs" / "wheels"
PLAYGROUND_DIR = ROOT / "docs" / "playground"
MANIFOLD_WHEEL = "manifold3d-3.5.2-cp312-cp312-emscripten_3_1_58_wasm32.whl"
PYODIDE_VERSION = "0.27.7"


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


def _humanize(name):
    """Simple deterministic label for a kwarg name."""
    return name.replace("_", " ").strip().title()


def _play_field(demo_name, demo_fn):
    """Build the index.json play field for a demo, or None if no PLAY entry."""
    if demo_name not in PLAY:
        return None
    sig = inspect.signature(demo_fn)
    params = []
    for pname, (pmin, pmax, pstep) in PLAY[demo_name].items():
        if pname not in sig.parameters:
            raise KeyError("PLAY[%r] has unknown param %r" % (demo_name, pname))
        default = sig.parameters[pname].default
        if default is inspect.Parameter.empty:
            raise ValueError("PLAY param %s.%s has no default" % (demo_name, pname))
        if isinstance(default, bool):
            ptype = "int"
            default_out = int(default)
        elif isinstance(default, int) and not isinstance(default, bool):
            ptype = "int"
            default_out = int(default)
        else:
            ptype = "float"
            default_out = float(default)
        params.append({
            "name": pname,
            "label": _humanize(pname),
            "type": ptype,
            "default": default_out,
            "min": pmin if ptype == "float" else int(pmin),
            "max": pmax if ptype == "float" else int(pmax),
            "step": pstep if ptype == "float" else int(pstep),
        })
    return {"demo": demo_name, "params": params}


def _build_mechlib_wheel():
    """Build mechlib wheel into docs/wheels/, preserving other wheels.

    Always builds from a temp copy so pip does not leave a ``build/`` tree
    (or dirty egg-info) in the repo root.
    """
    WHEELS_DIR.mkdir(parents=True, exist_ok=True)
    for stale in WHEELS_DIR.glob("mechlib-*.whl"):
        stale.unlink()

    with tempfile.TemporaryDirectory(prefix="mechlib-wheel-") as tmp:
        tmp_path = Path(tmp)
        shutil.copy2(ROOT / "pyproject.toml", tmp_path / "pyproject.toml")
        shutil.copytree(ROOT / "mechlib", tmp_path / "mechlib")
        result = subprocess.run(
            [sys.executable, "-m", "pip", "wheel", ".", "--no-deps", "-w", str(WHEELS_DIR)],
            cwd=str(tmp_path),
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            sys.stderr.write(result.stdout)
            sys.stderr.write(result.stderr)
            raise RuntimeError("pip wheel failed for mechlib")
    wheels = sorted(WHEELS_DIR.glob("mechlib-*.whl"))
    if not wheels:
        raise RuntimeError("no mechlib-*.whl produced in docs/wheels")
    print("wrote %s" % wheels[-1].relative_to(ROOT))
    return wheels[-1].name


def build():
    """Generate all gallery assets and their runtime manifest."""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    PLAYGROUND_DIR.mkdir(parents=True, exist_ok=True)

    models = [
        {
            "file": "cyl_demo.glb",
            "name": "cyl",
            "module": "mechlib.prim",
            "signature": signature(cyl),
            "description": "A configurable cylinder oriented along any principal axis.",
            "origin": "Originally extracted from geom_util.py in finnish-doors.",
            "demo": "demo_cyl",
        },
        {
            "file": "boxc_demo.glb",
            "name": "boxc",
            "module": "mechlib.prim",
            "signature": signature(boxc),
            "description": "A box positioned by its center and XYZ extents.",
            "origin": "Originally extracted from geom_util.py in finnish-doors.",
            "demo": "demo_boxc",
        },
        {
            "file": "rbox_demo.glb",
            "name": "rbox",
            "module": "mechlib.prim",
            "signature": signature(rbox),
            "description": "A clean enclosure block with rounded vertical corners.",
            "origin": "Originally extracted from geom_util.py in finnish-doors.",
            "demo": "demo_rbox",
        },
        {
            "file": "frustum_demo.glb",
            "name": "frustum",
            "module": "mechlib.prim",
            "signature": signature(frustum),
            "description": "A truncated cone between two radii and two Z planes.",
            "origin": "Originally extracted from geom_util.py in finnish-doors.",
            "demo": "demo_frustum",
        },
        {
            "file": "sector2d_demo.glb",
            "name": "sector2d",
            "module": "mechlib.prim",
            "signature": signature(sector2d),
            "description": "A circular sector polygon, extruded here to reveal its usable profile.",
            "origin": "Originally extracted from geom_util.py in finnish-doors.",
            "demo": "demo_sector2d",
        },
        {
            "file": "hex_poly_demo.glb",
            "name": "hex_poly",
            "module": "mechlib.prim",
            "signature": signature(hex_poly),
            "description": "A regular hexagon defined by across-flats width, shown as a solid.",
            "origin": "Originally extracted from gears2d.py in finnish-doors.",
            "demo": "demo_hex_poly",
        },
        {
            "file": "extrude_twist_demo.glb",
            "name": "extrude_twist",
            "module": "mechlib.sweep",
            "signature": signature(extrude_twist),
            "description": "A three-lobed profile swept through one full turn over 30 mm.",
            "origin": "Originally extracted from geom_util.py in finnish-doors.",
            "demo": "demo_extrude_twist",
        },
        {
            "file": "swept_keyed_bore_demo.glb",
            "name": "swept_keyed_bore",
            "module": "mechlib.sweep",
            "signature": signature(swept_keyed_bore),
            "description": "A D-shaped keyed bore beside its 50 degree free-rotation envelope.",
            "origin": "Originally extracted from shaft.py in finnish-doors.",
            "demo": "demo_swept_keyed_bore",
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
            "demo": "demo_spur_gear_pair",
        },
        {
            "file": "board_cradle_demo.glb",
            "name": "board_cradle",
            "module": "mechlib.fixtures",
            "signature": signature(board_cradle),
            "description": "Four PCB corner standoffs and capture walls around a 40 x 30 mm board.",
            "origin": "Originally extracted from system_layout.py in finnish-doors.",
            "demo": "demo_board_cradle",
        },
        {
            "file": "teardrop_demo.glb",
            "name": "teardrop",
            "module": "mechlib.cutters",
            "signature": signature(teardrop),
            "description": "A support-free teardrop bore shown through a cut block.",
            "origin": "Extracted from head.py in parviz.",
            "demo": "demo_teardrop",
        },
        {
            "file": "ss_bore_demo.glb",
            "name": "ss_bore",
            "module": "mechlib.cutters",
            "signature": signature(ss_bore),
            "description": "A round lower cradle and support-light chamfered upper bore.",
            "origin": "Extracted from geom_util.py in finnish-doors.",
            "demo": "demo_ss_bore",
        },
        {
            "file": "dbore_demo.glb",
            "name": "dbore + dbore_hub",
            "module": "mechlib.cutters",
            "signature": "%s; %s" % (signature(dbore), signature(dbore_hub)),
            "description": "A double-D socket blank beside a ready-made keyed hub.",
            "origin": "Unified from parviz and finnish-windows.",
            "demo": "demo_dbore",
        },
        {
            "file": "counterbore_demo.glb",
            "name": "counterbore",
            "module": "mechlib.cutters",
            "signature": signature(counterbore),
            "description": "A through-hole with a cylindrical recessed-head pocket.",
            "origin": "New abstraction inspired by finnish-doors fastener seats.",
            "demo": "demo_counterbore",
        },
        {
            "file": "bearing_seat_demo.glb",
            "name": "bearing_seat 608",
            "module": "mechlib.cutters",
            "signature": signature(bearing_seat),
            "description": "A cut-away retained 608 seat with a bearing reference ring.",
            "origin": "New abstraction from the Klonk 22.25 mm 608 pocket practice.",
            "demo": "demo_bearing_seat",
        },
        {
            "file": "crush_ribs_demo.glb",
            "name": "crush_ribs",
            "module": "mechlib.cutters",
            "signature": signature(crush_ribs),
            "description": "Six tapered vertical ribs squeezing a rectangular component.",
            "origin": "Generalized from tcst_hold_ribs in finnish-doors.",
            "demo": "demo_crush_ribs",
        },
        {
            "file": "press_lid_demo.glb",
            "name": "press_lid",
            "module": "mechlib.closures",
            "signature": signature(press_lid),
            "description": "A hollow box and its friction-plug lid in an exploded pose.",
            "origin": "Extracted from system_layout.py in finnish-doors.",
            "demo": "demo_press_lid",
        },
        {
            "file": "clamshell_shiplap_demo.glb",
            "name": "clamshell_shiplap",
            "module": "mechlib.closures",
            "signature": signature(clamshell_shiplap),
            "description": "The matching raised base lip and oversized lid receiver slot.",
            "origin": "Extracted from housings.py in finnish-doors.",
            "demo": "demo_clamshell_shiplap",
        },
        {
            "file": "ydovetail_demo.glb",
            "name": "ydovetail pair",
            "module": "mechlib.closures",
            "signature": signature(ydovetail),
            "description": "A self-supporting slide tongue beside its clearanced receiver.",
            "origin": "Extracted from system_layout.py in finnish-doors.",
            "demo": "demo_ydovetail",
        },
        {
            "file": "snap_pair_demo.glb",
            "name": "snap_catch + snap_finger",
            "module": "mechlib.closures",
            "signature": "%s; %s" % (signature(snap_catch), signature(snap_finger)),
            "description": "A ramped retention catch and matching cantilever hook.",
            "origin": "Unified from system_layout.py and build_powerbank.py.",
            "demo": "demo_snap_pair",
        },
        {
            "file": "nut_slot_demo.glb",
            "name": "nut_slot",
            "module": "mechlib.closures",
            "signature": signature(nut_slot),
            "description": "A cut-away seated nut trap with its M3 nut stand-in.",
            "origin": "Extracted from geo.py in parviz.",
            "demo": "demo_nut_slot",
        },
        {
            "file": "pins_and_posts_demo.glb",
            "name": "screw_post + fix_pin + blind_socket",
            "module": "mechlib.closures / mechlib.cutters",
            "signature": "%s; %s; %s" % (
                signature(screw_post), signature(fix_pin), signature(blind_socket)),
            "description": "A bored screw boss beside a locating pin and blind socket block.",
            "origin": "Extracted from geo.py in parviz.",
            "demo": "demo_pins_and_posts",
        },
        {
            "file": "spur_gear_mesh_demo.glb",
            "name": "spur_gear_mesh",
            "module": "mechlib.gears",
            "signature": signature(spur_gear_mesh),
            "description": "A printable involute spur gear with a measured round bore.",
            "origin": "Unified from finnish-windows and parviz gear builders.",
            "demo": "demo_spur_gear_mesh",
        },
        {
            "file": "roller_sprocket_demo.glb",
            "name": "roller_sprocket_2d + polar_ring",
            "module": "mechlib.gears / mechlib.patterns",
            "signature": "%s; %s" % (signature(roller_sprocket_2d), signature(polar_ring)),
            "description": "A conjugate 14-tooth sprocket surrounded by track-pin references.",
            "origin": "Generalized from tracks.py in parviz.",
            "demo": "demo_roller_sprocket",
        },
        {
            "file": "thread_demo.glb",
            "name": "thread_solid + tap",
            "module": "mechlib.mechanisms",
            "signature": "%s; %s" % (signature(thread_solid), signature(tap)),
            "description": "An M8 threaded bolt beside a cut-away tapped hex nut.",
            "origin": "Extracted from threads.py in parviz.",
            "demo": "demo_thread",
        },
        {
            "file": "knurl_demo.glb",
            "name": "knurl",
            "module": "mechlib.mechanisms",
            "signature": signature(knurl),
            "description": "A cylindrical thumb knob with printable vertical flutes.",
            "origin": "Extracted from m4_bolt.py in parviz.",
            "demo": "demo_knurl",
        },
        {
            "file": "torsion_spring_demo.glb",
            "name": "torsion_spring_mesh",
            "module": "mechlib.mechanisms",
            "signature": signature(torsion_spring_mesh),
            "description": "A five-turn torsion spring preview with moving and ground legs.",
            "origin": "Extracted from shaft.py in finnish-doors.",
            "demo": "demo_torsion_spring",
        },
        {
            "file": "lighten_grid_demo.glb",
            "name": "lighten_grid panel",
            "module": "mechlib.patterns",
            "signature": "%s; %s" % (
                signature(lighten_grid_centres), signature(lighten_cell_poly)),
            "description": "A structural panel opened by a staggered hexagonal lattice.",
            "origin": "Extracted from housings.py in finnish-doors.",
            "demo": "demo_lighten_grid",
        },
        {
            "file": "text_polygon_demo.glb",
            "name": "text_polygon",
            "module": "mechlib.text",
            "signature": signature(text_polygon),
            "description": "The word mechlib converted to counter-preserving raised geometry.",
            "origin": "Extracted from housings.py in finnish-doors.",
            "demo": "demo_text_polygon",
        },
        {
            "file": "fastener_trio_demo.glb",
            "name": "fastener_mesh + nut + washer",
            "module": "mechlib.fasteners",
            "signature": "%s; %s; %s" % (
                signature(fastener_mesh), signature(hex_nut_mesh), signature(washer_mesh)),
            "description": "Pan, socket, and countersunk screws with nut and washer stand-ins.",
            "origin": "Unified from all three source projects.",
            "demo": "demo_fastener_trio",
        },
        {
            "file": "worm_demo.glb",
            "name": "worm + helical wheel",
            "module": "mechlib.gears",
            "signature": "%s; %s" % (signature(worm), signature(spur_gear)),
            "description": "A true helical worm beside its lead-angle-matched wheel.",
            "origin": "Extracted from gears.py in dual-axis-turntable.",
            "demo": "demo_worm",
        },
        {
            "file": "spur_gear_sector_demo.glb",
            "name": "spur_gear sector",
            "module": "mechlib.gears",
            "signature": signature(spur_gear),
            "description": "A full-featured involute sector with a central hub and bore.",
            "origin": "Extracted from gears.py in dual-axis-turntable.",
            "demo": "demo_spur_gear_sector",
        },
        {
            "file": "loft_demo.glb",
            "name": "loft",
            "module": "mechlib.sweep",
            "signature": signature(loft),
            "description": "An organic solid joined through five equal-count point rings.",
            "origin": "Extracted from build.py in dual-axis-turntable.",
            "demo": "demo_loft",
        },
        {
            "file": "push_pin_demo.glb",
            "name": "push_pin",
            "module": "mechlib.closures",
            "signature": signature(push_pin),
            "description": "A barbed printed press pin with a conical lead-in tip.",
            "origin": "Extracted from build.py in dual-axis-turntable.",
            "demo": "demo_push_pin",
        },
        {
            "file": "chamfer_prism_demo.glb",
            "name": "chamfer_prism",
            "module": "mechlib.prim",
            "signature": signature(chamfer_prism),
            "description": "A rounded enclosure prism with a clean hull-chamfered top.",
            "origin": "Extracted from build.py in dual-axis-turntable.",
            "demo": "demo_chamfer_prism",
        },
        {
            "file": "threaded_rod_demo.glb",
            "name": "threaded_rod M8",
            "module": "mechlib.mechanisms",
            "signature": signature(threaded_rod),
            "description": "A fast radial-grid M8 display and light-duty external thread.",
            "origin": "Extracted from lib.py in wall-shelf-clamp.",
            "demo": "demo_threaded_rod",
        },
        {
            "file": "setscrew_demo.glb",
            "name": "setscrew boss",
            "module": "mechlib.closures",
            "signature": signature(setscrew),
            "description": "A cut-away sleeve showing its external boss and inward pilot.",
            "origin": "Extracted from lib.py in wall-shelf-clamp.",
            "demo": "demo_setscrew",
        },
        {
            "file": "slot_cutter_demo.glb",
            "name": "slot_cutter",
            "module": "mechlib.cutters",
            "signature": signature(slot_cutter),
            "description": "An FDM blade slot with visible square-corner dog-bone relief.",
            "origin": "Extracted from build.py in torque-lever.",
            "demo": "demo_slot_cutter",
        },
        {
            "file": "tapered_cavity_demo.glb",
            "name": "tapered_cavity",
            "module": "mechlib.cutters",
            "signature": signature(tapered_cavity),
            "description": "A cut-away hollow whose stepped roof closes without bridging.",
            "origin": "Extracted from lighten_legs.py in tripod.",
            "demo": "demo_tapered_cavity",
        },
        {
            "file": "u_channel_between_demo.glb",
            "name": "u_channel_between",
            "module": "mechlib.cutters",
            "signature": signature(u_channel_between),
            "description": "Joined arbitrary-angle open U segments forming an S-shaped run.",
            "origin": "Extracted from build.py in jumper-wire-sockets.",
            "demo": "demo_u_channel_between",
        },
        {
            "file": "revolved_gable_cavity_demo.glb",
            "name": "revolved_gable_cavity",
            "module": "mechlib.cutters",
            "signature": signature(revolved_gable_cavity),
            "description": "A cut-away annular chamber beneath a 45 degree gable roof.",
            "origin": "Generalized from build.py in massage-shower-head.",
            "demo": "demo_revolved_gable_cavity",
        },
        {
            "file": "directed_holes_demo.glb",
            "name": "directed_holes",
            "module": "mechlib.patterns",
            "signature": signature(directed_holes),
            "description": "Eight vector-directed bores cut through a hemispherical dome.",
            "origin": "Generalized from build.py in massage-shower-head.",
            "demo": "demo_directed_holes",
        },
        {
            "file": "saddle_demo.glb",
            "name": "saddle",
            "module": "mechlib.fixtures",
            "signature": signature(saddle),
            "description": "A shell-trimmed rib cradling a skew cylindrical reference part.",
            "origin": "Extracted from pickle_build.py in mini-powerbank.",
            "demo": "demo_saddle",
        },
        {
            "file": "text_block_demo.glb",
            "name": "text_block",
            "module": "mechlib.text",
            "signature": signature(text_block),
            "description": "Two centered lines of raised polygon text stacked on a plaque.",
            "origin": "Extracted from build.py in torque-lever.",
            "demo": "demo_text_block",
        },
        {
            "file": "seg_cylinder_demo.glb",
            "name": "seg_cylinder",
            "module": "mechlib.prim",
            "signature": signature(seg_cylinder),
            "description": "A cylinder spanning two skew points in three dimensions.",
            "origin": "Extracted from build.py in massage-shower-head.",
            "demo": "demo_seg_cylinder",
        },
        {
            "file": "printed_worm_demo.glb",
            "name": "printed_worm",
            "module": "mechlib.drives",
            "signature": signature(printed_worm),
            "description": "A journalled printed worm with runout threads, keyed bore, thrust collars, and radial set-screw hole.",
            "origin": "Extracted from the Klonk build_worm generator.",
            "demo": "demo_printed_worm",
        },
        {
            "file": "flat_worm_pair_demo.glb",
            "name": "flat_worm + worm_wheel_band",
            "module": "mechlib.drives",
            "signature": "%s; %s" % (
                signature(flat_worm), signature(worm_wheel_band)),
            "description": "The bench-proven three-start input worm and lead-angle-matched wheel band at crossed axes.",
            "origin": "Extracted from the Klonk flat-drive generators.",
            "demo": "demo_flat_worm_pair",
        },
        {
            "file": "worm_coupon_demo.glb",
            "name": "worm_coupon",
            "module": "mechlib.drives",
            "signature": signature(worm_coupon),
            "description": "The inexpensive print-frame worm and short wheel-band pieces used to test mesh quality before a full drive.",
            "origin": "Project-neutral form of the Klonk flat-drive mesh coupon.",
            "demo": "demo_worm_coupon",
        },
        {
            "file": "planet_stage_demo.glb",
            "name": "planet_stage",
            "module": "mechlib.drives",
            "signature": signature(planet_stage),
            "description": "An assembled 12:9:30 top-loading planetary stage with three planets and a downward hex-output carrier.",
            "origin": "Project-neutral form of the Klonk printed planetary stage.",
            "demo": "demo_planet_stage",
        },
        {
            "file": "pip_ratchet_demo.glb",
            "name": "print-in-place accordion ratchet",
            "module": "mechlib.ratchets",
            "signature": "%s; %s; %s; %s" % (
                signature(ratchet_ring_2d), signature(ratchet_ring),
                signature(pip_ratchet_hub_2d), signature(pip_ratchet_hub)),
            "description": "Three captive rigid pawls, each reseated by a printed accordion spring, inside a matching undercut ring.",
            "origin": "Extracted from the Klonk print-in-place follower ratchet.",
            "demo": "demo_pip_ratchet",
        },
        {
            "file": "spring_cartridge_ratchet_demo.glb",
            "name": "spring-cartridge rigid-pawl ratchet",
            "module": "mechlib.ratchets",
            "signature": "%s; %s; validator %s" % (
                signature(spring_cartridge_ratchet_2d),
                signature(spring_cartridge_ratchet),
                signature(check_ratchet_sense_and_sweep)),
            "description": "A slotted hub and three separately printable rigid pawls inside a self-energising ring.",
            "origin": "Adapted from experiments/spring_ratchet_fable/design.py.",
            "demo": "demo_spring_cartridge_ratchet",
        },
        {
            "file": "compliant_clutch_demo.glb",
            "name": "compliant clutch torque limiter",
            "module": "mechlib.ratchets",
            "signature": "%s; %s" % (
                signature(compliant_clutch_2d), signature(compliant_clutch)),
            "description": "Integral spiral flexures engaging an internal sawtooth race, shown with the torque-limit face fraction.",
            "origin": "Extracted from Klonk gen_compliant_2d with the torque-limit experiment delta.",
            "demo": "demo_compliant_clutch",
        },
        {
            "file": "arc_ratchet_demo.glb",
            "name": "compliant arc-arm ratchet",
            "module": "mechlib.ratchets",
            "signature": signature(arc_ratchet_2d),
            "description": "Three trailing tension-loaded arc flexures engaging a self-energising undercut ring.",
            "origin": "Recovered from the pre-bb26eec Klonk follower-ratchet revision.",
            "demo": "demo_arc_ratchet",
        },
        {
            "file": "helix_tube_demo.glb",
            "name": "helix_tube",
            "module": "mechlib.mechanisms",
            "signature": signature(helix_tube),
            "description": "A capped solid wire swept through five turns with a radial-inward moving frame.",
            "origin": "Extracted from the finnish-doors wrap-spring demonstration.",
            "demo": "demo_helix_tube",
        },
        {
            "file": "rack_2d_demo.glb",
            "name": "rack_2d",
            "module": "mechlib.gears",
            "signature": signature(rack_2d),
            "description": "Eight pressure-angle trapezoidal teeth at pi times module circular pitch, extruded thin for display.",
            "origin": "Generalized from the finnish-doors intercom plunger rack.",
            "demo": "demo_rack_2d",
        },
        {
            "file": "dog_slot_coupling_demo.glb",
            "name": "dog_slot_coupling",
            "module": "mechlib.mechanisms",
            "signature": signature(dog_slot_coupling),
            "description": "A downward collar dog riding in a boss arc slot to provide controlled angular lost motion.",
            "origin": "Generalized from finnish-doors coupling variant A.",
            "demo": "demo_dog_slot_coupling",
        },
        {
            "file": "four_bar_demo.glb",
            "name": "four_bar",
            "module": "mechlib.linkages",
            "signature": signature(four_bar),
            "description": (
                "A flat-printable four-bar linkage kit with printed pivot pins, "
                "assembled by circle-circle position kinematics; the defaults are "
                "Hoecken straight-line proportions with an extended coupler "
                "tracer point. The canonical leg drive for hobby walking robots."),
            "origin": (
                "Mechanical-movements wave v0.6.0; classic ref: Hoecken "
                "straight-line linkage (1926), cognate of the Chebyshev linkage"),
            "demo": "demo_four_bar",
        },
        {
            "file": "toggle_clamp_demo.glb",
            "name": "toggle_clamp",
            "module": "mechlib.linkages",
            "signature": signature(toggle_clamp),
            "description": (
                "An over-center knee toggle clamp — base, clamp arm, connecting "
                "link, and handle posed just past dead center where the mechanism "
                "self-locks. The ubiquitous hold-down clamp of workshop jigs and "
                "welding fixtures."),
            "origin": (
                "Mechanical-movements wave v0.6.0; classic ref: 507 Mechanical "
                "Movements No. 175 / toggle (knee) joint"),
            "demo": "demo_toggle_clamp",
        },
        {
            "file": "scotch_yoke_demo.glb",
            "name": "scotch_yoke",
            "module": "mechlib.linkages",
            "signature": signature(scotch_yoke),
            "description": (
                "A crank disc and pin driving a slotted yoke between guide rails, "
                "converting rotation into exact simple-harmonic reciprocation. "
                "The standard rotary-to-linear drive of pneumatic valve "
                "actuators."),
            "origin": (
                "Mechanical-movements wave v0.6.0; classic ref: 507 Mechanical "
                "Movements No. 94 / Scotch yoke"),
            "demo": "demo_scotch_yoke",
        },
        {
            "file": "quick_return_demo.glb",
            "name": "quick_return",
            "module": "mechlib.linkages",
            "signature": signature(quick_return),
            "description": (
                "A crank and slotted-lever quick-return whose pivot offset sweeps "
                "toward the Whitworth configuration; the working:return time "
                "ratio is exposed as mechanism metadata. The classic ram drive of "
                "metal shaping machines."),
            "origin": (
                "Mechanical-movements wave v0.6.0; classic ref: 507 Mechanical "
                "Movements Nos. 98-100 / crank and slotted lever, Whitworth "
                "quick return"),
            "demo": "demo_quick_return",
        },
        {
            "file": "plate_cam_demo.glb",
            "name": "plate_cam",
            "module": "mechlib.cams",
            "signature": "%s; follower lift from %s" % (
                signature(plate_cam), signature(cam_lift)),
            "description": "A radial plate cam synthesized from dwell, linear, SHM, and cycloidal motion-law segments, roller-compensated and extruded with a hub and D-flat bore. Programs the valve timing of a model engine or an automaton's motion.",
            "origin": "Mechanical-movements wave v0.6.0; classic ref: 507 Mechanical Movements No. 380 / plate cam",
            "demo": "demo_plate_cam",
        },
        {
            "file": "snail_cam_demo.glb",
            "name": "snail_cam",
            "module": "mechlib.cams",
            "signature": signature(snail_cam),
            "description": "A snail drop cam with an Archimedean rise over most of a revolution and a single radial drop face for slow-lift, sudden-release duty. Trips the strike train of a clock or a trip hammer.",
            "origin": "Mechanical-movements wave v0.6.0; classic ref: 507 Mechanical Movements No. 382 / snail drop cam",
            "demo": "demo_snail_cam",
        },
        {
            "file": "heart_cam_demo.glb",
            "name": "heart_cam",
            "module": "mechlib.cams",
            "signature": "%s; follower lift from %s" % (
                signature(heart_cam), signature(cam_lift)),
            "description": "A heart cam with symmetric linear rise and fall, giving constant-velocity follower motion and a unique angular reset position. Returns a chronograph seconds hand to zero.",
            "origin": "Mechanical-movements wave v0.6.0; classic ref: heart piece, chronograph reset (horology)",
            "demo": "demo_heart_cam",
        },
        {
            "file": "barrel_cam_demo.glb",
            "name": "barrel_cam",
            "module": "mechlib.cams",
            "signature": signature(barrel_cam),
            "description": "A barrel cam whose closed groove follows a motion-law z(theta) program around the drum, with a mating follower pin and guide. Winds the level line of a fishing reel or drives a toolchanger drum.",
            "origin": "Mechanical-movements wave v0.6.0; classic ref: 507 Mechanical Movements No. 389 / barrel cam",
            "demo": "demo_barrel_cam",
        },
        {
            "file": "geneva_pair_demo.glb",
            "name": "geneva_pair",
            "module": "mechlib.indexing",
            "signature": signature(geneva_pair),
            "description": (
                "An external Geneva (Maltese cross) pair posed mid-engagement: "
                "the driver's crank pin indexes the slotted wheel one slot per "
                "revolution while its crescent-cut locking disc holds the wheel "
                "during dwell. The film-advance indexer of movie projectors and "
                "automated assembly turrets."
            ),
            "origin": ("Mechanical-movements wave v0.6.0; classic ref: "
                       "507 Mechanical Movements No. 214 region / Geneva drive"),
            "demo": "demo_geneva_pair",
        },
        {
            "file": "escapement_demo.glb",
            "name": "escapement",
            "module": "mechlib.indexing",
            "signature": signature(escapement),
            "description": (
                "A clock escapement: a forward-raked escape wheel and a pivoted "
                "anchor, flat parts posed with one pallet resting on a tooth tip; "
                "the style parameter selects recoil anchor or Graham deadbeat "
                "pallet faces. The visible tick of longcase and printed clock "
                "builds."
            ),
            "origin": ("Mechanical-movements wave v0.6.0; classic ref: "
                       "anchor escapement (Hooke, c. 1657) / Graham deadbeat"),
            "demo": "demo_escapement",
        },
        {
            "file": "intermittent_gear_pair_demo.glb",
            "name": "intermittent_gear_pair",
            "module": "mechlib.indexing",
            "signature": signature(intermittent_gear_pair),
            "description": (
                "A mutilated-gear intermittent pair posed meshed: the driver "
                "keeps involute teeth on one arc with a plain locking segment, "
                "advancing the notch-locked driven gear one tooth group per "
                "revolution. The digit-advance scheme of mechanical counters and "
                "washing-machine timers."
            ),
            "origin": ("Mechanical-movements wave v0.6.0; classic ref: "
                       "507 Mechanical Movements intermittent-gearing plates"),
            "demo": "demo_intermittent_gear_pair",
        },
        {
            "file": "herringbone_gear_demo.glb",
            "name": "herringbone_gear",
            "module": "mechlib.gears",
            "signature": "%s; phased with %s" % (
                signature(herringbone_gear), signature(mesh_phase)),
            "description": "A double-helical involute gear whose mirrored tooth "
                           "halves cancel axial thrust and self-center in mesh. "
                           "The standard drive gear of printed extruders and "
                           "quiet printed gearboxes.",
            "origin": "Mechanical-movements wave v0.6.0; classic ref: Citroen "
                      "double-helical (herringbone) gear",
            "demo": "demo_herringbone_gear",
        },
        {
            "file": "cycloidal_drive_demo.glb",
            "name": "cycloidal_drive",
            "module": "mechlib.gears",
            "signature": signature(cycloidal_drive),
            "description": "A single-stage cycloidal reducer: an eccentric input "
                           "precesses a shortened-epitrochoid disc against a ring "
                           "of housing pins, with output taken through oversized "
                           "pin holes. The robot-joint reducer of Nabtesco RV "
                           "and DIY actuators.",
            "origin": "Mechanical-movements wave v0.6.0; classic ref: cycloidal "
                      "speed reducer",
            "demo": "demo_cycloidal_drive",
        },
        {
            "file": "bevel_gear_pair_demo.glb",
            "name": "bevel_gear_pair",
            "module": "mechlib.gears",
            "signature": signature(bevel_gear_pair),
            "description": "A straight bevel pair on 90-degree axes, the involute "
                           "profile lofted toward the pitch apex in the Tredgold "
                           "approximation. Right-angle drives of hand drills, "
                           "angle grinders, and differentials.",
            "origin": "Mechanical-movements wave v0.6.0; classic ref: straight "
                      "bevel / miter gear pair",
            "demo": "demo_bevel_gear_pair",
        },
        {
            "file": "scroll_drive_demo.glb",
            "name": "scroll_drive",
            "module": "mechlib.linear",
            "signature": signature(scroll_drive),
            "description": "A lathe-chuck scroll plate whose raised Archimedean "
                           "spiral rib drives three arc-toothed jaws in sync on a "
                           "common gripping circle. The self-centering heart of "
                           "every 3-jaw lathe chuck.",
            "origin": "Mechanical-movements wave v0.6.0; classic ref: 3-jaw "
                      "scroll chuck",
            "demo": "demo_scroll_drive",
        },
        {
            "file": "differential_screw_demo.glb",
            "name": "differential_screw",
            "module": "mechlib.linear",
            "signature": signature(differential_screw),
            "description": "One shaft with two same-hand thread sections of "
                           "slightly different pitch; travel per revolution is "
                           "the pitch difference, giving micrometer-fine motion "
                           "from coarse printed threads. Micrometer heads and "
                           "optical fine-focus stages.",
            "origin": "Mechanical-movements wave v0.6.0; classic ref: 507 "
                      "Mechanical Movements differential screw plates",
            "demo": "demo_differential_screw",
        },
        {
            "file": "archimedes_screw_demo.glb",
            "name": "archimedes_screw",
            "module": "mechlib.linear",
            "signature": signature(archimedes_screw),
            "description": "A helical flight on a shaft inside a half-pipe "
                           "trough, posed inclined; each turn carries one pocket "
                           "of material uphill. Grain augers, snowblowers, and "
                           "wastewater lift screws.",
            "origin": "Mechanical-movements wave v0.6.0; classic ref: Archimedes "
                      "water screw",
            "demo": "demo_archimedes_screw",
        },
        {
            "file": "oldham_coupling_demo.glb",
            "name": "oldham_coupling",
            "module": "mechlib.couplings",
            "signature": signature(oldham_coupling),
            "description": (
                "Two hubs with perpendicular diametral tongues drive a floating "
                "cross-slotted disc, transmitting constant-velocity rotation "
                "between parallel shafts with lateral offset. The anti-rotation "
                "coupling of scroll compressors and the stepper-to-leadscrew "
                "coupler of 3D printers."
            ),
            "origin": (
                "Mechanical-movements wave v0.6.0; classic ref: Oldham "
                "double-slider coupling (507 Mechanical Movements, shaft "
                "couplings)"
            ),
            "demo": "demo_oldham_coupling",
        },
        {
            "file": "universal_joint_demo.glb",
            "name": "universal_joint",
            "module": "mechlib.couplings",
            "signature": signature(universal_joint),
            "description": (
                "Two forked yokes joined by a four-trunnion cross spider transmit "
                "rotation between shafts intersecting at an angle, with the "
                "classic sinusoidal speed fluctuation twice per revolution. The "
                "joint of automotive driveshafts and socket-wrench extensions."
            ),
            "origin": (
                "Mechanical-movements wave v0.6.0; classic ref: Cardan/Hooke's "
                "universal joint (507 Mechanical Movements)"
            ),
            "demo": "demo_universal_joint",
        },
        {
            "file": "jaw_coupling_demo.glb",
            "name": "jaw_coupling",
            "module": "mechlib.couplings",
            "signature": signature(jaw_coupling),
            "description": (
                "Two jaw hubs interleave through a lobed elastomer spider "
                "(printed in TPU in practice) that carries torque in compression, "
                "damping vibration and failing safe if the spider dies. The "
                "Lovejoy-style motor-to-pump coupling of light industry."
            ),
            "origin": (
                "Mechanical-movements wave v0.6.0; classic ref: Lovejoy "
                "elastomeric jaw (spider) coupling"
            ),
            "demo": "demo_jaw_coupling",
        },
        {
            "file": "torque_limiter_demo.glb",
            "name": "torque_limiter",
            "module": "mechlib.clutches",
            "signature": "%s; preload spring %s" % (
                signature(torque_limiter), signature(helix_tube)),
            "description": (
                "Spring-seated radiused detent bumps engage matching pockets "
                "between two faces; above the trip torque set by the detent "
                "geometry they cam out and ratchet past, protecting the "
                "drivetrain. The slip clutch inside every cordless drill clutch "
                "collar."
            ),
            "origin": (
                "Mechanical-movements wave v0.6.0; classic ref: spring-detent "
                "safety (overload) clutch"
            ),
            "demo": "demo_torque_limiter",
        },
        {
            "file": "freewheel_clutch_demo.glb",
            "name": "freewheel_clutch",
            "module": "mechlib.clutches",
            "signature": signature(freewheel_clutch),
            "description": (
                "Cylindrical rollers between a smooth inner hub and ramped "
                "outer-ring pockets wedge instantly in one direction and release "
                "in the other, a silent one-way clutch with no tooth steps. Used "
                "in starter-motor bendix drives and conveyor backstops."
            ),
            "origin": (
                "Mechanical-movements wave v0.6.0; classic ref: roller-ramp "
                "overrunning (sprag-less one-way) clutch"
            ),
            "demo": "demo_freewheel_clutch",
        },
        {
            "file": "timing_pulley_demo.glb",
            "name": "timing_pulley",
            "module": "mechlib.pulleys",
            "signature": signature(timing_pulley),
            "description": (
                "A GT2-style toothed belt pulley with flanges, hub and setscrew "
                "boss, its tooth spaces cut as circular arcs per the GT2 "
                "approximation, shown meshing a toothed belt segment. A printed "
                "drop-in for 3D-printer and benchtop CNC belt axes."),
            "origin": ("Mechanical-movements wave v0.6.0; classic ref: GT2 "
                       "synchronous belt drive (507 Mechanical Movements "
                       "belt-drive series)"),
            "demo": "demo_timing_pulley",
        },
        {
            "file": "winch_drum_demo.glb",
            "name": "grooved_drum",
            "module": "mechlib.pulleys",
            "signature": "%s; wound cable %s" % (
                signature(grooved_drum), signature(helix_tube)),
            "description": (
                "A cylindrical winch drum with a helical groove pitched to the "
                "cable so the rope spools in a single controlled layer, shown "
                "with its first wound turns. Used on crane hoists and 4x4 "
                "recovery winches."),
            "origin": ("Mechanical-movements wave v0.6.0; classic ref: 507 "
                       "Mechanical Movements winch/windlass entries (LeBus "
                       "grooved drum)"),
            "demo": "demo_winch_drum",
        },
        {
            "file": "fusee_demo.glb",
            "name": "grooved_drum",
            "module": "mechlib.pulleys",
            "signature": signature(grooved_drum),
            "description": (
                "A conical spirally grooved fusee whose hyperbolic winding "
                "radius compensates a weakening mainspring to hold drive torque "
                "constant. The classic constant-torque heart of marine "
                "chronometers and English pocket watches."),
            "origin": ("Mechanical-movements wave v0.6.0; classic ref: 507 "
                       "Mechanical Movements No. 46 / fusee chain and "
                       "spring-box"),
            "demo": "demo_fusee",
        },
        {
            "file": "cross_flexure_demo.glb",
            "name": "cross_flexure",
            "module": "mechlib.flexures",
            "signature": signature(cross_flexure),
            "description": (
                "A monolithic cross-axis flexural pivot: two rigid blocks joined "
                "only by two crossing thin blades, giving limited frictionless, "
                "backlash-free rotation about the crossing point. Used in "
                "precision instruments, optical mounts and printed robot "
                "joints."),
            "origin": ("Mechanical-movements wave v0.6.0; classic ref: "
                       "cross-spring pivot (Bendix Free-Flex)"),
            "demo": "demo_cross_flexure",
        },
        {
            "file": "wave_spring_demo.glb",
            "name": "wave_spring",
            "module": "mechlib.flexures",
            "signature": signature(wave_spring),
            "description": (
                "A crest-to-crest wave spring: annular strips with sinusoidal "
                "axial waviness stacked half a wave apart so opposing crests "
                "bear on each other, giving spring rate in half the height of a "
                "coil spring. Used for bearing preload and clutch-pack "
                "take-up."),
            "origin": ("Mechanical-movements wave v0.6.0; classic ref: Smalley "
                       "crest-to-crest wave spring"),
            "demo": "demo_wave_spring",
        },
        {
            "file": "bistable_beam_demo.glb",
            "name": "bistable_beam",
            "module": "mechlib.flexures",
            "signature": signature(bistable_beam),
            "description": (
                "A flat-printed buckled-beam bistable switch: pre-curved cosine "
                "beams clamped in a frame carry a central shuttle that snaps "
                "between two stable positions. Used for clicky toggle latches, "
                "haptic buttons and one-piece switches."),
            "origin": ("Mechanical-movements wave v0.6.0; classic ref: "
                       "buckled-beam bistable compliant mechanism (BYU CMR)"),
            "demo": "demo_bistable_beam",
        },
    ]

    manifest_models = []
    for model in models:
        demo_name = model.pop("demo")
        demo_fn = getattr(demos, demo_name)
        meshes = demo_fn()
        export_model(model["file"], meshes)
        play = _play_field(demo_name, demo_fn)
        if play is not None:
            model["play"] = play
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
        ("mechlib.meshutil", mechlib.audit, "Audit pairwise overlap and clearance."),
        ("mechlib.meshutil", mechlib.min_distance, "Approximate sampled surface distance."),
        ("mechlib.meshutil", mechlib.approach_clear, "Measure free travel toward an opening."),
        ("mechlib.meshutil", mechlib.slicer_area, "Predict perimeter-plus-infill layer area."),
        ("mechlib.packing", mechlib.shelf_pack, "Pack brim-grown footprints across plates."),
        ("mechlib.stepio", mechlib.export_assembly, "Export positioned meshes as a STEP assembly."),
        ("mechlib.meshutil", mechlib.extrude_snapped, "Snap tangent vertices before extrusion."),
        ("mechlib.cutters", mechlib.lobe_cavity_polys, "Build hollow lobe polygons around ribs."),
    ]
    utilities = [
        {"module": module, "signature": signature(function), "description": description}
        for module, function, description in utility_functions
    ]

    demos_src = Path(__file__).resolve().parent / "demos.py"
    demos_dst = PLAYGROUND_DIR / "demos.py"
    shutil.copy2(demos_src, demos_dst)
    print("wrote %s" % demos_dst.relative_to(ROOT))

    wheel_name = _build_mechlib_wheel()

    manifest = {
        "version": mechlib.__version__,
        "playground": {
            "pyodide": PYODIDE_VERSION,
            "wheels": [
                "wheels/%s" % MANIFOLD_WHEEL,
                "wheels/%s" % wheel_name,
            ],
            "demos": "playground/demos.py",
        },
        "models": manifest_models,
        "utilities": utilities,
    }
    index_path = OUTPUT_DIR / "index.json"
    index_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print("wrote %s" % index_path.relative_to(ROOT))


if __name__ == "__main__":
    build()
