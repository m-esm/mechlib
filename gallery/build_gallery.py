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
from mechlib.gears import (
    mesh_phase,
    rack_2d,
    roller_sprocket_2d,
    spur_gear_2d,
    spur_gear,
    spur_gear_mesh,
    worm,
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
