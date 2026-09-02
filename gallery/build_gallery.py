#!/usr/bin/env python3
"""Build the static mechlib model gallery."""

import importlib.util
import inspect
import json
import math
import shutil
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import cKDTree


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import mechlib
from mechlib.cams import (
    barrel_cam,
    cam_lift,
    face_cam,
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
from mechlib.bearings import plain_bushing, printed_ball_bearing, thrust_washer
from mechlib.chains import (
    chain_dual_output,
    chain_reverse,
    chain_s_wrap,
    drag_chain,
    drag_chain_link,
    roller_chain,
    roller_chain_link,
)
from mechlib.clutches import dog_clutch, freewheel_clutch, torque_limiter
from mechlib.couplings import (
    double_cardan_joint,
    hirth_coupling,
    jaw_coupling,
    oldham_coupling,
    oldham_pose,
    tripod_cv_joint,
    hooke_pose,
    universal_joint,
)
from mechlib.cutters import (
    bearing_seat,
    blind_socket,
    counterbore,
    crush_ribs,
    dbore,
    dbore_hub,
    gasket_channel,
    labyrinth_seal,
    oring_groove,
    revolved_gable_cavity,
    slot_cutter,
    ss_bore,
    teardrop,
    tapered_cavity,
    u_channel_between,
)
from mechlib.drives import (
    flat_worm,
    harmonic_drive,
    planet_stage,
    printed_worm,
    worm_coupon,
    worm_wheel_band,
)
from mechlib.fasteners import fastener_mesh, hex_nut_mesh, washer_mesh
from mechlib.fixtures import (
    board_cradle,
    kinematic_coupling,
    repeatable_dock,
    saddle,
    three_point_leveller,
)
from mechlib.flexures import (
    belleville_washer,
    bistable_beam,
    coil_spring,
    cross_flexure,
    flexure_stage,
    leaf_spring,
    spiral_power_spring,
    wave_spring,
)
from mechlib.fluid import (
    external_gear_pump,
    gerotor_pump,
    hose_barb,
    peristaltic_pump_head,
    rotary_spool_valve,
)
from mechlib.gears import (
    bevel_gear_pair,
    cycloidal_drive,
    herringbone_gear,
    mesh_phase,
    rack_2d,
    ring_gear,
    ring_gear_mesh,
    roller_sprocket_2d,
    spur_gear_2d,
    spur_gear,
    spur_gear_mesh,
    worm,
)
from mechlib.grippers import collet_chuck, eccentric_cam_clamp, iris_diaphragm
from mechlib.guides import linear_way, telescoping_stage
from mechlib.indexing import escapement, geneva_pair, intermittent_gear_pair
from mechlib.cams import roller_follower
from mechlib.closures import annular_snap
from mechlib.couplings import beam_coupling
from mechlib.drives import flywheel
from mechlib.fasteners import shaft_key, thread_insert, tslot_nut
from mechlib.fluid import check_valve
from mechlib.grippers import bellows_suction_cup
from mechlib.indexing import detent_pair, star_wheel
from mechlib.linear import lead_screw
from mechlib.mechanisms import handwheel, shaft_collar, star_knob
from mechlib.pulleys import v_belt_pulley
from mechlib.ratchets import ratchet_wheel_pawl
from mechlib.joints import ball_socket_joint, clevis, gimbal_rings, knuckle_hinge
from mechlib.lattices import auxetic_panel, bcc_lattice, honeycomb_panel, isogrid_panel, kagome_panel, kerf_bend_cutter
from mechlib.linear import (
    archimedes_screw,
    differential_screw,
    rack_pinion,
    scroll_drive,
    screw_jack,
    swash_plate,
)
from mechlib.linkages import (
    bell_crank,
    chebyshev_linkage,
    four_bar,
    lazy_tongs,
    pantograph_linkage,
    peaucellier_linkage,
    quick_return,
    sarrus_linkage,
    scotch_yoke,
    scott_russell_linkage,
    slider_crank,
    toggle_clamp,
    watt_linkage,
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
from mechlib.pulleys import (
    belt_tensioner,
    eccentric_idler_mount,
    grooved_drum,
    idler_pulley,
    timing_pulley,
)
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
from mechlib import find_vitamin, vitamin, vitamin_addresses


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


def _load_applications():
    """Load gallery/applications.py (works when run as a script)."""
    path = Path(__file__).resolve().parent / "applications.py"
    spec = importlib.util.spec_from_file_location("gallery_applications", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


_applications = _load_applications()
applications_for = _applications.applications_for

OUTPUT_DIR = ROOT / "docs" / "models"
WHEELS_DIR = ROOT / "docs" / "wheels"
PLAYGROUND_DIR = ROOT / "docs" / "playground"
MANIFOLD_WHEEL = "manifold3d-3.5.2-cp312-cp312-emscripten_3_1_58_wasm32.whl"
PYODIDE_VERSION = "0.27.7"


# Ordered category table. The key is the mechlib module leaf name, so a model's
# category is DERIVED from its ``module`` field rather than hand-tagged per entry.
# ``group`` buckets categories into the three shelves the gallery renders, and the
# order of this dict is the render order: movements first, primitives last.
# A module leaf missing from this table fails the build (see ``_category``), so a
# new mechlib module can never silently land in an unsorted pile.
CATEGORY_GROUPS = (
    ("movements", "Mechanical movements",
     "Parts that do something: convert, index, limit, or transmit motion."),
    ("elements", "Machine elements",
     "Threads, fasteners, closures, and the hardware that joins parts together."),
    ("blocks", "Building blocks",
     "Primitives, sweeps, cutters, and patterns you compose everything else from."),
)

CATEGORIES = {
    # movements
    "linkages":   ("movements", "Linkages", "Bar mechanisms that trade rotation for stroke, straight lines, or scale."),
    "grippers":   ("movements", "Grippers & clamps", "Parts that hold, centre, and grip."),
    "cams":       ("movements", "Cams & followers", "Profiles that program a follower's displacement."),
    "indexing":   ("movements", "Indexing", "Continuous rotation in, stepped rotation out."),
    "gears":      ("movements", "Gears", "Involute, internal, herringbone, bevel, cycloidal, and racks."),
    "ratchets":   ("movements", "Ratchets", "One-way pawls, pips, and compliant detents."),
    "clutches":   ("movements", "Clutches", "Torque limiting and freewheeling."),
    "couplings":  ("movements", "Couplings", "Shaft-to-shaft joints that tolerate misalignment, Cardan through constant-velocity."),
    "joints":     ("movements", "Joints", "Kinematic pairs: spherical, hinge, gimbal."),
    "drives":     ("movements", "Worm & planetary", "High-ratio reductions in a printable envelope."),
    "fluid":      ("movements", "Pumps & valves", "Rotors and plugs that move and route fluid."),
    "linear":     ("movements", "Rotary to linear", "Screws, scrolls, and augers."),
    "guides":     ("movements", "Guides & ways", "Prismatic ways that pin a body to a single axis."),
    "pulleys":    ("movements", "Pulleys & drums", "Belt, cable, idler, and constant-torque profiles."),
    "chains":     ("movements", "Chains & carriers", "Link trains that transmit force or carry cable."),
    "flexures":   ("movements", "Flexures", "Springs, pivots, bistable beams, and straight-line stages."),
    # machine elements
    "mechanisms": ("elements", "Threads & shafts", "Printable threads, knurls, helices, and drive slots."),
    "fasteners":  ("elements", "Fasteners", "Screw, nut, and washer stand-ins for fit checks."),
    "bearings":   ("elements", "Bearings & bushings", "Printed journal and thrust support for turning shafts."),
    "closures":   ("elements", "Closures & joints", "Lids, snaps, dovetails, and captive hardware."),
    "fixtures":   ("elements", "Fixtures", "Cradles, saddles, and exactly constrained mounts."),
    "vitamins":   ("elements", "Bought parts", "Catalog envelopes."),
    # building blocks
    "prim":       ("blocks", "Primitives", "The solids and profiles everything else starts from."),
    "sweep":      ("blocks", "Sweeps & lofts", "Profiles carried along a path."),
    "cutters":    ("blocks", "Cutters", "Negative geometry: bores, sockets, cavities, seals, and reliefs."),
    "patterns":   ("blocks", "Patterns", "Repeat, ring, and lighten across a surface."),
    "lattices":   ("blocks", "Lattices & kerf patterns", "Metamaterial cells and bend patterns cut from flat stock."),
    "text":       ("blocks", "Text", "Embossed and engraved labels."),
    # utility-only modules (no gallery models, but utilities are grouped by them)
    "meshutil":   ("blocks", "Mesh utilities", "Booleans, probes, fitting, and export gates."),
    "packing":    ("blocks", "Plate packing", "Arrange footprints across build plates."),
    "stepio":     ("blocks", "STEP export", "Hand positioned meshes off to CAD."),
}

CATEGORY_ORDER = {key: index for index, key in enumerate(CATEGORIES)}
GROUP_ORDER = {key: index for index, (key, _title, _blurb) in enumerate(CATEGORY_GROUPS)}


def _category(module_field):
    """Derive a category key from a model's ``module`` string.

    Accepts the ``"mechlib.a / mechlib.b"`` form used by cross-module demos and
    keys off the first module. Raises if the module is not in ``CATEGORIES`` so
    that adding a mechlib module without categorising it breaks the build loudly.
    """
    leaf = module_field.split("/")[0].strip().split(".")[-1]
    if leaf not in CATEGORIES:
        raise KeyError(
            "module %r has no CATEGORIES entry in build_gallery.py; add one so the "
            "gallery can sort and filter it" % module_field
        )
    return leaf


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


# Name tokens that mark a value as a pure number rather than a length. Matched
# per underscore-separated token, so `pins` is a count while `pin_d` is a
# diameter in millimetres.
_UNITLESS_TOKENS = frozenset((
    "n", "res", "resolution", "sections", "segments", "samples", "count",
    "teeth", "slots", "turns", "waves", "pins", "planets", "rollers",
    "layers", "starts", "lobes", "branch", "frac", "fraction", "ratio",
))
# Tooth counts, matched on the whole name only: `z` is a count, `split_z` is a
# Z coordinate in millimetres.
_UNITLESS_NAMES = frozenset(("z", "z1", "z2"))


def _unit(pname, ptype):
    """Infer a slider's unit from its name, following the library's own naming.

    mechlib is millimetres and degrees throughout and marks angles with a
    ``_deg`` suffix, so the unit is derivable rather than something to hand-tag
    on 290-odd sliders. Anything that is not an angle or a count is a length.
    """
    low = pname.lower()
    if low.endswith("_deg") or "deg" in low or "angle" in low:
        return "deg"
    if low in _UNITLESS_NAMES or _UNITLESS_TOKENS.intersection(low.split("_")):
        return ""
    return "mm"


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
        out_min = pmin if ptype == "float" else int(pmin)
        out_max = pmax if ptype == "float" else int(pmax)
        out_step = pstep if ptype == "float" else int(pstep)
        # A default outside the slider range silently desyncs the UI: the input
        # clamps, the geometry regenerates at the clamped value, but the call
        # line the user copies still shows the unreachable default.
        if not (out_min <= default_out <= out_max):
            raise ValueError(
                "PLAY[%r][%r]: default %r is outside the slider range [%r, %r]; "
                "widen the range or move the default onto it"
                % (demo_name, pname, default_out, out_min, out_max)
            )
        # ... and a default off the step grid cannot be dragged back to.
        offset = (default_out - out_min) / out_step
        if abs(offset - round(offset)) > 1e-6:
            raise ValueError(
                "PLAY[%r][%r]: default %r is not on the step grid (min %r, step %r); "
                "the slider can never return to it"
                % (demo_name, pname, default_out, out_min, out_step)
            )
        params.append({
            "name": pname,
            "label": _humanize(pname),
            "unit": _unit(pname, ptype),
            "type": ptype,
            "default": default_out,
            "min": out_min,
            "max": out_max,
            "step": out_step,
        })
    return {"demo": demo_name, "params": params}


# ---------------------------------------------------------------------------
# Baked mechanism animation
# ---------------------------------------------------------------------------
#
# Mechanisms in the gallery move because this build regenerates each animatable
# demo across a full cycle of its motion-phase parameter and RECOVERS each
# body's rigid transform numerically. No motion law is written here: the
# kinematics live in mechlib (four_bar_pose, cam_lift, the Geneva layout's
# wheel_angle, tooth ratios) and the demo threads the phase into them. What
# lands in index.json is only what the geometry itself did.
#
# Manifest convention: a body that never moves across the cycle is OMITTED from
# ``animation.bodies``. Playback must therefore treat a missing body as
# stationary rather than as an error. Grounds, frames, rails, and housings are
# the usual absentees, and dropping them roughly halves the manifest cost.
#
# ``ANIMATE`` names the phase parameter, the full period, and the frame count
# for each animatable demo. It is config, not derived data: which kwarg is "the
# input" and how far it must travel is mechanism knowledge that exists nowhere
# in the code. Every entry is gate-checked below against the live signature and
# against the geometry, so a wrong cycle fails the build rather than shipping.
#
# The cycle must return every body's TRANSFORM to identity, not merely its
# picture. A geared body that ends the loop one tooth on looks right in the
# last frame and then unwinds hundreds of degrees in the single interpolation
# step back to frame 0, which is exactly the flicker the loop is supposed to
# avoid. So a reduction ratio sets the period: the driven member has to come
# back round a whole number of times too, and the frame count rises with it to
# keep the angular step under _MAX_STEP_DEG.
ANIMATE = {
    # Planar linkages: crank rotation is the input, one turn closes everything.
    "demo_four_bar":         ("crank_angle_deg",  360.0,  24, False),
    "demo_quick_return":     ("crank_angle_deg",  360.0,  24, False),
    "demo_scotch_yoke":      ("angle_deg",        360.0,  24, False),
    # Toggle clamp: base is pose-invariant (full-swing envelope); drive_deg
    # maps through a sine onto the open↔lock arc so one linear ramp is enough.
    "demo_toggle_clamp":     ("drive_deg",        360.0,  24, False),
    # Cams: the follower sweeps the profile through one revolution.
    "demo_plate_cam":        ("follower_deg",     360.0,  24, False),
    "demo_heart_cam":        ("follower_deg",     360.0,  24, False),
    "demo_barrel_cam":       ("pin_phase_deg",    360.0,  24, False),
    # Listed on purpose even though it is rejected every build: the snail cam's
    # drop face is a genuine discontinuity, and the skip line is the standing,
    # re-measured proof of that rather than a claim in a comment. If the cam
    # ever gains a return ramp this starts animating on its own.
    "demo_snail_cam":        ("follower_deg",     360.0,  24, False),
    # Geneva: the wheel takes 360/slots per crank turn, so six crank turns
    # bring the six-slot wheel back to where it started. The loop shows six
    # index-and-dwell events. Not closed: the driver's crescent-cut locking
    # disc has no rotational symmetry, so only a whole turn returns its
    # picture, and the transform closes at the same place anyway.
    "demo_geneva_pair":      ("crank_deg",       2160.0, 108, False),
    # Equal tooth counts, so one turn closes both. 72 frames, not 24: a
    # 24-tooth gear has a 15 degree tooth pitch, and stepping it 15 degrees a
    # frame renders as a gear standing perfectly still.
    "demo_herringbone_gear": ("drive_deg",        360.0,  72, False),
    # 16:24 bevel pair -- the gear turns twice for every three of the pinion,
    # so three pinion turns close both transforms.
    "demo_bevel_gear_pair":  ("drive_deg",       1080.0, 144, False),
    # Closed tracks. These are pairs whose transforms only realign after many
    # turns (18:28 needs fourteen), but whose PICTURE comes back after a single
    # tooth. Measured, not assumed: the driver is 18-fold symmetric to 0.064 mm
    # and the driven 28-fold to 0.067 mm, so 20 degrees of driver puts both
    # exactly one tooth on and nobody can tell. 13 frames is 1.7 degrees a step
    # against a 20 degree pitch, well clear of the wagon-wheel band.
    "demo_spur_gear_pair":   ("drive_deg",         20.0,  13, True),
    # One gear, 20-fold symmetric to 0.065 mm, so one tooth is 18 degrees.
    "demo_spur_gear_mesh":   ("drive_deg",         18.0,  13, True),
    # ----- gap-analysis wave v0.8.0 -----
    # Internal 40:20 pair. Both members turn the SAME way (that is what makes it
    # an internal mesh) and the ring runs at half the pinion's rate, so two
    # pinion turns are needed to bring the ring's transform back to identity.
    # 96 frames, not 24: the pinion's tooth pitch is 18 degrees, and the
    # aliasing gate measures 74% apparent motion at 7.5 degrees a frame. The
    # shorter one-tooth closed track was tried and rejected -- the ring blank is
    # sampled at 768 segments, which is not a multiple of 40, so simplify()
    # thins its arcs unevenly and the picture gap comes out at 0.51 mm.
    "demo_ring_gear_mesh":   ("drive_deg",        720.0,  96, False),
    # Straight-line and scaling linkages. Four of the five are oscillating
    # mechanisms, not crank-through ones: a Peaucellier crank driven a whole
    # turn passes through the fixed pivot and throws the tracer to infinity,
    # Watt's levers rock, the Sarrus folds, the tongs squeeze. Their demos take
    # a 0-360 deg drive PHASE and map it through a sine onto the working arc,
    # which is what a long connecting rod does to a crank turn: one linear phase
    # ramp, one out-and-back travel, and the cycle closes on its own pose. The
    # pantograph needs none of that -- its stylus genuinely goes round a circle.
    "demo_peaucellier":      ("drive_deg",        360.0,  24, False),
    "demo_watt":             ("drive_deg",        360.0,  24, False),
    "demo_sarrus":           ("drive_deg",        360.0,  24, False),
    "demo_pantograph":       ("crank_angle_deg",  360.0,  24, False),
    "demo_lazy_tongs":       ("drive_deg",        360.0,  24, False),
    # ----- gap-analysis wave v0.9.0 -----
    "demo_slider_crank":     ("crank_angle_deg",  360.0,  24, False),
    "demo_chebyshev":        ("drive_deg",        360.0,  24, False),
    "demo_scott_russell":    ("drive_deg",        360.0,  24, False),
    "demo_bell_crank":       ("pose_deg",         360.0,  24, False),
    "demo_face_cam":         ("pin_phase_deg",    360.0,  24, False),
    "demo_swash_plate":      ("phase_deg",        360.0,  48, False),
    # Screw jack / dog clutch: open [0,1] lifts cannot close a loop; demos
    # map drive_deg through a cosine onto lift/engage so one turn is out-and-back.
    "demo_screw_jack":       ("drive_deg",        360.0,  24, False),
    # Finite rack has no tooth-pitch picture identity; demo maps drive_deg
    # through a sine onto ±2 teeth so the cycle closes on the same pose.
    "demo_rack_pinion":      ("drive_deg",        360.0,  36, False),
    "demo_dog_clutch":       ("drive_deg",        360.0,  24, False),
    # Equal-tooth external mesh: 1:1 counter-rotation, one turn closes both.
    # 48 frames: a 12-tooth gear has a 30 deg pitch and 24 frames would alias.
    "demo_external_gear_pump": ("phase_deg",      360.0,  48, False),
    # Tripod CV joint: the one coupling whose motion law is in the module.
    # tripod_pose gives it in closed form -- the housing angle equals the input
    # angle exactly (that IS constant velocity), and the spider centre rides a
    # circle of radius pitch_r*(1-cos b)/2 at three times input speed. One input
    # turn closes everything. 360 rather than 120 degrees because the tulip
    # floor carries a double-D output bore and the shaft carries drive flats, so
    # neither member is 3-fold symmetric. 36 frames, not 24: the tulip is the
    # coarsest body and 10 degree steps put its apparent motion at 0.91.
    "demo_tripod_cv_joint":  ("phase_deg",        360.0,  36, False),
    # Gerotor: the pair is 6/7, so the two rotors only return to the same POSE
    # after seven inner turns. They return to the same PICTURE far sooner -- the
    # inner rotor is exactly 6-fold symmetric and the outer exactly 7-fold, and
    # one inner tooth pitch of 60 deg turns the outer by 60*6/7, which is one of
    # its own tooth pitches. The symmetry is exact rather than approximate: the
    # profile sample count and both circle tessellations are locked to multiples
    # of the lobe counts. The full 2520 deg pose-closing cycle was tried and the
    # gate rejects it on aliasing at any sane frame count.
    "demo_gerotor_pump":     ("rotor_deg",         60.0,  13, True),
    # Valve: body and cap are fixed, the plug is a rigid body on the Z axis, and
    # a whole turn returns it to identity. 24 frames walks through all three
    # detents plus the closed positions between them.
    "demo_rotary_spool_valve": ("plug_deg",       360.0,  24, False),
    # Gimbal: each ring is built once and rigidly transformed down the chain, and
    # nothing limits a gimbal ring's rotation, so a full turn returns every
    # body's transform to identity. Ring 2 turns about two axes at once, which
    # is what sets the 36-frame count.
    "demo_gimbal_rings":     ("tilt_deg",         360.0,  36, False),
    # Eccentric cam lock: the handle angle is a genuine motion phase, the cam is
    # built once at the clamped pose and then moved, and a whole turn brings all
    # four bodies back to identity.
    "demo_eccentric_cam_clamp": ("handle_deg",    360.0,  24, False),
    # Escapement: one phase turn is one pendulum period. The wheel steps two
    # teeth (one per beat) and is teeth-fold symmetric, so the picture closes
    # on a closed track; the anchor returns to the rest pose. 48 frames keep
    # the two impulse windows readable (each is ~14% of the cycle).
    "demo_escapement":       ("phase_deg",        360.0,  48, True),
    # Cycloidal: default 12 pins → 11:1. Disc lobes (11) are coprime with the
    # 6 output holes, so the picture only repeats after 11 input turns
    # (3960 deg). 144 frames keep the input step at 27.5 deg (under the
    # 30 deg cap). A drive tab on the input shaft stops the eccentric from
    # aliasing (bare shaft scored ~45% apparent motion).
    "demo_cycloidal_drive":  ("phase_deg",       3960.0, 144, False),
    # Oldham: both hubs and the disc turn 1:1; the disc centre orbits at
    # 2-omega. One input turn returns every transform. 36 frames keep the
    # disc's 20 deg orbit step well under the 30 deg cap.
    "demo_oldham_coupling":  ("phase_deg",        360.0,  36, False),
    # Hooke: input and output both close after one turn (Cardan tangent
    # relation). 36 frames keep the spider's two-axis step readable.
    "demo_universal_joint":  ("phase_deg",        360.0,  36, False),
}

# Mechanisms deliberately left out, and why. Each was measured, not assumed.
# The recurring reason is the period: a loop can only be seamless if every body
# comes back, and a body's own rotational symmetry is what decides how soon.
# Where the finest symmetry is 1-fold -- a D-flat bore, an eccentric cam, a
# disc whose lobe count is coprime with its hole count -- nothing short of a
# whole turn of that body will do, and the input has to turn as many times as
# the ratio demands.
#
#   demo_worm              The demo poses worm and wheel for display, already
#                          interpenetrating by 93 mm^3, and the pair has no
#                          measurable conjugate contact (overlap stays 0 across
#                          a +/-8 degree wheel band), so the drive sense cannot
#                          be recovered from the geometry -- only asserted.
#   demo_cycloidal_drive   Animated: cycloidal_pose + 11 input turns (3960 deg)
#                          for the default 12-pin / 6-hole layout. Listed in
#                          ANIMATE. Shorter closed tracks fail because lobes
#                          and holes are coprime.
#   demo_planet_stage      The sun's D-flat bore leaves it 1-fold symmetric
#                          (measured), so the sun needs a whole turn; the
#                          carrier runs at 2/7 sun and is 3-fold, so it needs
#                          seven of them. Shortest repeating picture is 2520
#                          deg of sun, and the sun's 30 deg tooth pitch then
#                          needs 252 frames. Feasible, just not worth it.
#   intermittent_gear_pair mechlib exposes no engagement relation (unlike
#                          geneva_wheel_angle / escapement_pose). Rotating the
#                          pair at a continuous ratio would drive the locking
#                          segment through the notch: an invented motion.
#   escapement             Animated: escapement_pose gives the stepped wheel
#                          + swinging anchor law; listed in ANIMATE.
#   oldham_coupling,       Animated: oldham_pose gives the 2-omega disc orbit
#   universal_joint        and hooke_pose gives the Cardan tangent relation.
#                          Listed in ANIMATE.
#   double_cardan_joint    Two Hooke joints in series; the intermediate shaft
#                          pose is still not in the module (cv_velocity_ratio
#                          is the SPEED relation only).
#   iris_diaphragm,        Bounded out-and-back travels, which a single linear
#   knuckle_hinge,         phase ramp cannot express, and with a hard stop in
#   ball_socket_joint      the way there is no 360 to ramp through instead.
#   collet_chuck,          These DEFORM rather than repose: fingers deflect,
#   the flexure springs,   coils compress, panels flex. The rigid-recovery gate
#   auxetic/kerf panels    rejects that by design.
#   harmonic_drive         The flex spline is anisotropically scaled to the
#                          wave-generator ellipse every frame: that is real
#                          strain-wave deformation, not a rigid body. The bake
#                          correctly rejects it (residual ~0.15 mm over a
#                          0.11 mm limit). Wave generator alone would spin, but
#                          leaving the cup frozen misrepresents the ratio.
#   linear_way,            Pure translation along an open interval: the cycle
#   telescoping_stage      genuinely does not close.
#   eccentric_idler_mount  Geometrically it would pass, but it is a static
#                          adjustment you set once and lock with a setscrew;
#                          animating it would claim the part spins in service.
#   jaw_coupling,          Every body turns together, so the whole model just
#   freewheel_clutch,      rotates -- indistinguishable from moving the camera.
#   timing_pulley

# Largest rotation any body may take between consecutive frames, before a
# slerped keyframe track stops reading as rotation at all.
_MAX_STEP_DEG = 30.0

# ... and the guard the angle cap misses. A rotationally symmetric body stepped
# by (or near) its own tooth pitch lands back on itself: the transform moved,
# the picture did not. Comparing the frame-to-frame point clouds catches that
# without having to detect each body's symmetry order -- when the apparent
# displacement collapses relative to the real one, the part is about to render
# frozen or running backwards. Measured: the gear pairs sampled every tooth
# pitch scored 0.01 here; every honestly-moving body scores above 0.8.
_MIN_APPARENT_FRACTION = 0.5

# A body is accepted as rigid when the recovered transform reproduces the
# regenerated frame to within RIGID_REL of its own bounding diagonal. Demos
# that pose finished meshes (``_spin``) hit this at machine precision.
RIGID_REL = 1e-6

# Demos that rebuild a profile at the new pose (the linkage bars) cannot reach
# RIGID_REL: shapely re-polygonises each bore in the world frame, so the vertex
# set is a slightly different discretisation of the same rigid part rather than
# a permutation of it. Those pass a looser gate -- one part in five hundred of
# the body, capped at a quarter millimetre, still well under one FDM extrusion
# width, so the transform reproduces the real frame to within something nobody
# can print or see.
#
# That threshold also separates re-facetting from a body that genuinely
# reshapes with the pose, and the separation is measured rather than assumed.
# Re-polygonisation noise is bounded by the facetting: across every animated
# linkage it tops out around 0.03 mm and does not climb with phase. A body that
# truly reshapes with the pose (old toggle_clamp base, sized from the current
# pose bbox) climbs monotonically past REPOSE_ABS and is rejected by the bake.
REPOSE_ABS = 0.25
REPOSE_REL = 2e-3


def _kabsch(source, target):
    """Rigid transform taking ``source`` points onto ``target`` points."""
    cs, ct = source.mean(axis=0), target.mean(axis=0)
    cov = (source - cs).T @ (target - ct)
    u, _s, vt = np.linalg.svd(cov)
    # Guard the reflection case: flip the least singular direction so the
    # recovered matrix is a rotation and never a mirror.
    flip = np.sign(np.linalg.det(vt.T @ u.T))
    rotation = vt.T @ np.diag([1.0, 1.0, flip]) @ u.T
    return rotation, ct - rotation @ cs


def _principal_frame(points):
    """Centroid plus a right-handed principal axis frame of a point set."""
    centre = points.mean(axis=0)
    centred = points - centre
    _w, vectors = np.linalg.eigh(centred.T @ centred)
    axes = vectors[:, ::-1].copy()
    axes[:, 2] = np.cross(axes[:, 0], axes[:, 1])
    return centre, axes


def _rotation_gap(a, b):
    """Angle in radians between two rotation matrices."""
    cosine = (float(np.trace(a.T @ b)) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, cosine)))


def _recover_rigid(source, target, limit, seed):
    """Recover the rigid transform from ``source`` to ``target`` vertices.

    Tries index correspondence first, which is exact whenever the demo poses a
    finished mesh. Falls back to a correspondence-free fit (principal-axis
    seeds plus the previous frame's transform, each refined by nearest-neighbour
    iterations) for demos that rebuild the profile at the new pose, where
    shapely may reorder the boolean result ring.

    A symmetric body admits several transforms that reproduce it exactly: a
    linkage bar is a capsule with two equal bores, so flipping it end-over-end
    and sliding it back fits just as well as the pose it actually reached.
    Those alternatives draw the same picture but interpolate through nonsense,
    so among the candidates that fit within ``limit`` this keeps the one
    closest to ``seed`` -- the previous frame. Continuity, not a preferred
    answer, breaks the tie. Returns ``(rotation, translation, residual)``, the
    residual being the largest distance any vertex ends up from the
    regenerated body.
    """
    rotation, translation = _kabsch(source, target)
    residual = float(np.abs(source @ rotation.T + translation - target).max())
    if residual <= RIGID_REL * float(np.linalg.norm(np.ptp(target, axis=0))):
        return rotation, translation, residual

    tree = cKDTree(target)
    src_centre, src_axes = _principal_frame(source)
    tgt_centre, tgt_axes = _principal_frame(target)
    starts = [seed]
    for sx in (1.0, -1.0):
        for sy in (1.0, -1.0):
            axes = src_axes @ np.diag([sx, sy, 1.0])
            axes[:, 2] = np.cross(axes[:, 0], axes[:, 1])
            guess = tgt_axes @ axes.T
            starts.append((guess, tgt_centre - guess @ src_centre))

    solutions = []
    for guess, offset in starts:
        for _step in range(40):
            _d, index = tree.query(source @ guess.T + offset)
            guess, offset = _kabsch(source, target[index])
        forward, _index = tree.query(source @ guess.T + offset)
        back, _index = cKDTree(source @ guess.T + offset).query(target)
        solutions.append((float(max(forward.max(), back.max())), guess, offset))

    within = [s for s in solutions if s[0] <= limit]
    if not within:
        return min(solutions, key=lambda s: s[0])[1:] + (
            min(s[0] for s in solutions),)
    cost, guess, offset = min(
        within, key=lambda s: (_rotation_gap(seed[0], s[1]),
                               float(np.linalg.norm(s[2] - seed[1]))))
    return guess, offset, cost


def _quaternion_xyzw(rotation):
    """Convert a 3x3 rotation to an xyzw quaternion."""
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    w, x, y, z = trimesh.transformations.quaternion_from_matrix(matrix)
    return [float(x), float(y), float(z), float(w)]


def _pose_gap(vertices, rotation, translation):
    """How far one full cycle leaves the body from frame 0's POSE.

    The check for a modulo-wrapped track, where the player interpolates from
    the last frame back to frame 0: a gear that ends the loop one tooth on is
    the same picture but a different pose, and slerping back to frame 0 from
    there spins it most of a turn in a single frame.
    """
    return float(np.abs(
        vertices @ rotation.T + translation - vertices).max())


def _picture_gap(vertices, rotation, translation):
    """How different one full cycle leaves the body from frame 0's PICTURE.

    The check for a ``closed`` track, where the player snaps from the last
    frame straight to frame 0 with nothing interpolated between them. Only the
    rendered result has to match, so the poses may legitimately differ by a
    symmetry of the part -- one tooth pitch on a spur gear. Compared as point
    clouds, which is what "looks the same" means.
    """
    posed = vertices @ rotation.T + translation
    forward, _index = cKDTree(vertices).query(posed)
    backward, _index = cKDTree(posed).query(vertices)
    return float(max(forward.max(), backward.max()))


def _bake_animation(demo_name, demo_fn):
    """Bake per-frame rigid motion for one demo, or return None if it cannot.

    Regenerates the demo across the cycle, recovers every body's transform
    against frame 0, and verifies each recovery before it is trusted. A body
    that changes vertex count, or that no recovered transform reproduces,
    means the demo is rebuilding geometry rather than reposing it, and the
    whole model is skipped rather than animated with an invented pose.
    """
    param, cycle_deg, frames, closed = ANIMATE[demo_name]
    # A closed track's frame list is INCLUSIVE of the cycle end: n frames span
    # P = 0..C in n-1 steps and the player snaps from the last straight back to
    # the first. A modulo track emits n frames covering [0, C) and the player
    # interpolates the wrap, so the cycle end is sampled only to check it.
    span = frames - 1 if closed else frames
    last = span
    signature_params = inspect.signature(demo_fn).parameters
    if param not in signature_params:
        raise KeyError("ANIMATE[%r] names unknown param %r" % (demo_name, param))
    base = signature_params[param].default
    if base is inspect.Parameter.empty:
        raise ValueError("ANIMATE param %s.%s has no default" % (demo_name, param))
    base = float(base)

    # Frame 0 is the plain default call -- the same one that wrote the GLB --
    # so the baked motion lines up with the geometry the page loads.
    reference = demo_fn()
    reference_v = {name: np.asarray(mesh.vertices, dtype=float)
                   for name, mesh, _color in reference}
    diagonals = {name: float(np.linalg.norm(mesh.extents))
                 for name, mesh, _color in reference}

    tracks = {name: {"t": [[0.0, 0.0, 0.0]], "q": [[0.0, 0.0, 0.0, 1.0]]}
              for name in reference_v}
    moved = set()
    residuals = []
    steps = {name: 0.0 for name in reference_v}
    seeds = {name: (np.eye(3), np.zeros(3)) for name in reference_v}
    poses = {name: value.copy() for name, value in reference_v.items()}
    visible = {name: (0.0, 0.0) for name in reference_v}
    wraps = {}
    for index in range(1, last + 1):
        frame = demo_fn(**{param: base + index * cycle_deg / span})
        posed = {name: np.asarray(mesh.vertices, dtype=float)
                 for name, mesh, _color in frame}
        if set(posed) != set(reference_v):
            print("  skip %s: body set changed at frame %d" % (demo_name, index))
            return None
        for name, target in posed.items():
            source = reference_v[name]
            if len(source) != len(target):
                print("  skip %s: body %r vertex count %d -> %d at frame %d"
                      % (demo_name, name, len(source), len(target), index))
                return None
            limit = max(RIGID_REL * diagonals[name],
                        min(REPOSE_ABS, REPOSE_REL * diagonals[name]))
            previous = seeds[name]
            rotation, translation, residual = _recover_rigid(
                source, target, limit, previous)
            seeds[name] = (rotation, translation)
            if residual > limit:
                print("  skip %s: body %r not rigid at frame %d "
                      "(residual %.4g mm, limit %.4g mm)"
                      % (demo_name, name, index, residual, limit))
                return None
            residuals.append(residual)
            steps[name] = max(steps[name],
                              math.degrees(_rotation_gap(previous[0], rotation)))
            current = source @ rotation.T + translation
            travel = float(np.abs(current - poses[name]).max())
            if travel > visible[name][0]:
                forward, _index = cKDTree(poses[name]).query(current)
                backward, _index = cKDTree(current).query(poses[name])
                visible[name] = (travel,
                                 float(max(forward.max(), backward.max())))
            poses[name] = current
            if index == last:
                wraps[name] = (_picture_gap if closed else _pose_gap)(
                    source, rotation, translation)
                if not closed:
                    continue
            if (np.abs(translation).max() > 1e-9 or
                    np.abs(rotation - np.eye(3)).max() > 1e-9):
                moved.add(name)
            tracks[name]["t"].append([float(v) for v in translation])
            tracks[name]["q"].append(_quaternion_xyzw(rotation))

    # Seamlessness: one full cycle must land back on frame 0 -- on its pose for
    # a modulo track, on its picture for a closed one. A cam with a drop face
    # fails this either way, which is the point: its cycle genuinely does not
    # close.
    for name, wrap in sorted(wraps.items()):
        if wrap > max(REPOSE_ABS, REPOSE_REL * diagonals[name]):
            print("  skip %s: body %r does not close the cycle "
                  "(pose is %.4g mm off frame 0 after one full cycle)"
                  % (demo_name, name, wrap))
            return None

    # Enough frames to read as motion rather than as a strobe.
    coarse = max(steps.items(), key=lambda item: item[1])
    if coarse[1] > _MAX_STEP_DEG:
        print("  skip %s: %d frames leaves body %r turning %.1f deg per frame "
              "(limit %.1f); raise the frame count in ANIMATE"
              % (demo_name, frames, coarse[0], coarse[1], _MAX_STEP_DEG))
        return None
    apparent = 1.0
    for name, (travel, seen) in sorted(visible.items()):
        if travel <= 1e-9:
            continue
        fraction = seen / travel
        apparent = min(apparent, fraction)
        if fraction < _MIN_APPARENT_FRACTION:
            print("  skip %s: %d frames aliases body %r -- it travels %.3g mm "
                  "per frame but only looks %.3g mm different (%.0f%%); raise "
                  "the frame count in ANIMATE"
                  % (demo_name, frames, name, travel, seen, 100.0 * fraction))
            return None

    if not moved:
        print("  skip %s: no body moves across the cycle" % demo_name)
        return None

    bodies = {}
    for name in sorted(moved):
        track = tracks[name]
        # Keep consecutive quaternions in one hemisphere so a shortest-path
        # interpolator never spins a body the long way round between frames.
        quats = []
        previous = None
        for quat in track["q"]:
            vector = np.asarray(quat, dtype=float)
            if previous is not None and float(previous @ vector) < 0.0:
                vector = -vector
            previous = vector
            quats.append([round(float(v), 6) for v in vector])
        bodies[name] = {
            "t": [[round(float(v), 6) for v in row] for row in track["t"]],
            "q": quats,
        }

    worst = max(residuals) if residuals else 0.0
    print("  animated %-24s %-16s cycle %6.1f deg, %3d frames%s, %d/%d bodies, "
          "residual %8.2e mm, wrap %8.2e mm, step %5.2f deg, visible %3.0f%%"
          % (demo_name, param, cycle_deg, frames, " closed" if closed else "      ",
             len(bodies), len(tracks), worst, max(wraps.values()), coarse[1],
             100.0 * apparent))
    animation = {
        "fps": 24,
        "cycle_deg": float(cycle_deg),
        "param": param,
        "bodies": bodies,
    }
    if closed:
        animation["closed"] = True
    return animation


def _check_node_names(filename, bodies):
    """Fail the build if baked body names miss the GLB's node names.

    Playback keys off node names; a silent mismatch animates nothing at all,
    so this reads the exported file back rather than trusting the writer.
    """
    scene = trimesh.load(str(OUTPUT_DIR / filename), file_type="glb")
    nodes = set(scene.graph.nodes_geometry)
    missing = set(bodies) - nodes
    if missing:
        raise ValueError(
            "%s: baked bodies %s are not node names in the GLB (%s)"
            % (filename, sorted(missing), sorted(nodes)))
    return len(nodes)


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
            "file": "fix_pin_demo.glb",
            "name": "fix_pin",
            "module": "mechlib.closures / mechlib.cutters",
            "signature": "%s; %s" % (signature(fix_pin), signature(blind_socket)),
            "description": "A locating pin with a matching blind socket block.",
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
            "file": "find_vitamin_demo.glb",
            "name": "find_vitamin",
            "module": "mechlib.vitamins",
            "signature": signature(find_vitamin),
            "description": (
                "Catalog envelopes from find_vitamin: 608 vs 695, an M3 SHCS, "
                "and a GA12-N20, one body per query so the sizes read side by side."
            ),
            "origin": "Bought-part catalog envelopes (display meshes, not printed STLs).",
            "demo": "demo_find_vitamin",
        },
        {
            "file": "vitamin_demo.glb",
            "name": "vitamin",
            "module": "mechlib.vitamins",
            "signature": signature(vitamin),
            "description": (
                "Catalog envelopes from vitamin address lookup: 608-2RS, "
                "an 18650 cell, a NEMA17, and an SG90, one body per address."
            ),
            "origin": "Bought-part catalog envelopes (display meshes, not printed STLs).",
            "demo": "demo_vitamin",
        },
        {
            "file": "vitamin_addresses_demo.glb",
            "name": "vitamin_addresses",
            "module": "mechlib.vitamins",
            "signature": signature(vitamin_addresses),
            "description": (
                "One catalog envelope per vitamin family, addresses taken from "
                "vitamin_addresses() so the row tracks the live catalog."
            ),
            "origin": "Bought-part catalog envelopes (display meshes, not printed STLs).",
            "demo": "demo_vitamin_addresses",
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
            "file": "arc_ratchet_2d_demo.glb",
            "name": "arc_ratchet_2d",
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
            "file": "oldham_pose_demo.glb",
            "name": "oldham_pose",
            "module": "mechlib.couplings",
            "signature": signature(oldham_pose),
            "description": (
                "Oldham 2-omega disc orbit at a known parallel-shaft offset: "
                "posed copies at 0/45/90/135 deg so the floating disc centre "
                "walks a circle of radius |offset|/2 at twice shaft speed. "
                "Printer stepper-to-screw couplers and scroll-compressor drives."
            ),
            "origin": (
                "Mechanical-movements wave v0.6.0; classic ref: Oldham "
                "double-slider coupling (507 Mechanical Movements, shaft "
                "couplings)"
            ),
            "demo": "demo_oldham_pose",
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
            "file": "hooke_pose_demo.glb",
            "name": "hooke_pose",
            "module": "mechlib.couplings",
            "signature": signature(hooke_pose),
            "description": (
                "Cardan lag and lead twice per revolution at a known bend: "
                "posed copies at 0/45/90/135 deg with input vs output index "
                "flags so the mismatch is readable at rest. Driveshafts, "
                "steering columns, and socket-wrench extensions."
            ),
            "origin": (
                "Mechanical-movements wave v0.6.0; classic ref: Cardan/Hooke's "
                "universal joint (507 Mechanical Movements)"
            ),
            "demo": "demo_hooke_pose",
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
        # ------------------------------------------------------------------
        # Gap-analysis wave v0.8.0, in gallery category order.
        # ------------------------------------------------------------------
        {
            "file": "peaucellier_linkage_demo.glb",
            "name": "peaucellier_linkage",
            "module": "mechlib.linkages",
            "signature": signature(peaucellier_linkage),
            "description": (
                "Seven bars that draw a mathematically exact straight line: two "
                "anchor links and a rhombus form an inversor about the fixed "
                "pivot, and a crank holds the input point on a circle through "
                "that pivot, so the tracer runs a perfect line. The first exact "
                "straight-line linkage, and still the reference case for "
                "guiding motion without a slideway."),
            "origin": (
                "Gap-analysis wave v0.8.0; classic ref: Peaucellier (1864) and "
                "Lipkin (1871) inversor cell"),
            "demo": "demo_peaucellier",
        },
        {
            "file": "watt_linkage_demo.glb",
            "name": "watt_linkage",
            "module": "mechlib.linkages",
            "signature": signature(watt_linkage),
            "description": (
                "Two rocking levers joined by a coupler, whose tracer point "
                "holds an approximate straight line over its working stroke; "
                "the departure from the line is measured across the stroke and "
                "returned as mechanism metadata. Watt guided a beam-engine "
                "piston rod with it, and it still locates the axle of a "
                "solid-axle rear suspension."),
            "origin": (
                "Gap-analysis wave v0.8.0; classic ref: James Watt parallel "
                "motion (1784)"),
            "demo": "demo_watt",
        },
        {
            "file": "sarrus_linkage_demo.glb",
            "name": "sarrus_linkage",
            "module": "mechlib.linkages",
            "signature": signature(sarrus_linkage),
            "description": (
                "Two hinge chains folding in orthogonal planes constrain the "
                "top plate to pure vertical translation: no rotation, no X or Y "
                "travel, and no sliding pair anywhere. Platform height is "
                "2*bar_len*sin(fold). The spatial answer to the straight-line "
                "problem, and the one that suits FDM best because nothing "
                "slides against a layer line."),
            "origin": (
                "Gap-analysis wave v0.8.0; classic ref: Sarrus (1853) "
                "parallel-motion linkage"),
            "demo": "demo_sarrus",
        },
        {
            "file": "pantograph_linkage_demo.glb",
            "name": "pantograph_linkage",
            "module": "mechlib.linkages",
            "signature": signature(pantograph_linkage),
            "description": (
                "A parallelogram chain anchored at one fixed pivot: trace a "
                "shape with the stylus pin and the output pin draws the same "
                "shape scaled about the pivot. The ratio comes from the "
                "parallelogram closure, so it is exact at every position, and "
                "the achieved value is measured back off the posed geometry."),
            "origin": (
                "Gap-analysis wave v0.8.0; classic ref: 507 Mechanical "
                "Movements No. 246 / pantograph"),
            "demo": "demo_pantograph",
        },
        {
            "file": "lazy_tongs_demo.glb",
            "name": "lazy_tongs",
            "module": "mechlib.linkages",
            "signature": signature(lazy_tongs),
            "description": (
                "Nuremberg scissors: bars pinned at their centres and ends form "
                "a row of rhombs, so squeezing the frame end shoots the output "
                "yoke out along a straight line at the rhomb count times the "
                "stroke. Guide slots at both ends keep the travel on axis. The "
                "extension of a lazy-tongs riveter and of a folding gate."),
            "origin": (
                "Gap-analysis wave v0.8.0; classic ref: 507 Mechanical "
                "Movements No. 122 / lazy tongs, Nuremberg scissors"),
            "demo": "demo_lazy_tongs",
        },
        {
            "file": "iris_diaphragm_demo.glb",
            "name": "iris_diaphragm",
            "module": "mechlib.grippers",
            "signature": signature(iris_diaphragm),
            "description": (
                "A print-in-place iris: leaves on their own pivot posts, each "
                "on its own Z layer, swung together by a drive ring whose "
                "radial slots pick up a pin on every leaf. Each leaf's inner "
                "edge is a circular arc struck about a point that walks off "
                "the axis as the leaf turns, so the opening stays round and "
                "shrinks from 24 mm to 6 mm over 6.6 degrees of ring travel."),
            "origin": (
                "Gap-analysis wave v0.8.0; classic ref: Joseph Petzval's lens "
                "diaphragm, 1840s, in the stacked-plane form FDM needs"),
            "demo": "demo_iris_diaphragm",
        },
        {
            "file": "collet_chuck_demo.glb",
            "name": "collet_chuck",
            "module": "mechlib.grippers",
            "signature": signature(collet_chuck),
            "description": (
                "An ER-style split collet with its taper nut and spindle "
                "nose. Slots cut alternately from each end turn the sleeve "
                "into a ring of fingers, and drawing its 8 degree cone into "
                "the matching cone in the nose squeezes them onto whatever is "
                "in the bore. Grips on axis, and closes as far as the slots "
                "can shut."),
            "origin": (
                "Gap-analysis wave v0.8.0; classic ref: ER collet system, "
                "Rego-Fix, 1973"),
            "demo": "demo_collet_chuck",
        },
        {
            "file": "eccentric_cam_clamp_demo.glb",
            "name": "eccentric_cam_clamp",
            "module": "mechlib.grippers",
            "signature": signature(eccentric_cam_clamp),
            "description": (
                "A rotary cam lock: a disc turning about a pivot offset from "
                "its centre, so the lift under the follower plate is the cam "
                "radius plus the eccentricity times the cosine of the handle "
                "angle. It is set to clamp a few degrees past top dead "
                "centre, where the clamped part's spring-back drives the "
                "handle into its stop instead of backing it out."),
            "origin": (
                "Gap-analysis wave v0.8.0; classic ref: eccentric quick-clamp, "
                "Brown 1868 US patent 79,000 lineage"),
            "demo": "demo_eccentric_cam_clamp",
        },
        {
            "file": "ring_gear_demo.glb",
            "name": "ring_gear",
            "module": "mechlib.gears",
            "signature": signature(ring_gear),
            "description": "An internal (annular) involute gear: the teeth are "
                           "cut inward from a rim, so the tip circle is inside "
                           "the pitch circle and the root circle outside it. "
                           "The fixed ring of a planetary gearbox, a strain-wave "
                           "drive's circular spline, and an involute spline "
                           "socket are all this part.",
            "origin": "Gap-analysis wave v0.8.0; classic ref: annular/internal "
                      "involute gear, AGMA 917 internal-gear proportions",
            "demo": "demo_ring_gear",
        },
        {
            "file": "ring_gear_mesh_demo.glb",
            "name": "ring_gear_mesh",
            "module": "mechlib.gears",
            "signature": signature(ring_gear_mesh),
            "description": "A pinion posed in mesh inside an internal ring gear. "
                           "The ring flanks are generated, not drawn: the mating "
                           "pinion is rolled through the conjugate internal "
                           "motion and the swept region is subtracted from the "
                           "blank. Internal pairs turn the same way and sit at a "
                           "centre distance equal to the DIFFERENCE of the pitch "
                           "radii, which is why they pack a high ratio into a "
                           "small envelope.",
            "origin": "Gap-analysis wave v0.8.0; classic ref: internal spur "
                      "gearing, shaper (Fellows) generation",
            "demo": "demo_ring_gear_mesh",
        },
        {
            "file": "tripod_cv_joint_demo.glb",
            "name": "tripod_cv_joint",
            "module": "mechlib.couplings",
            "signature": signature(tripod_cv_joint),
            "description": (
                "Three crowned barrels on a tripod spider ride in three "
                "straddling tracks of an outer tulip, transmitting genuinely "
                "constant angular velocity across a shaft angle and plunging "
                "freely along the housing axis. Unlike the Cardan joint there "
                "is no speed fluctuation at all; unlike a Rzeppa ball joint it "
                "needs no hardened balls or ground raceways, so it prints. The "
                "inboard joint of most front-wheel-drive halfshafts."),
            "origin": (
                "Gap-analysis wave v0.8.0; classic ref: Glaenzer Spicer "
                "tripode joint (1960s), the plunging half of the modern "
                "front-drive halfshaft"),
            "demo": "demo_tripod_cv_joint",
        },
        {
            "file": "double_cardan_joint_demo.glb",
            "name": "double_cardan_joint",
            "module": "mechlib.couplings",
            "signature": signature(double_cardan_joint),
            "description": (
                "Two Hooke joints in series with their intermediate yokes 90 "
                "degrees apart, so the second joint's speed error is the exact "
                "inverse of the first's and the output runs at constant "
                "velocity. The intermediate shaft between them still swings by "
                "the full single-joint fluctuation, which is the price this fix "
                "pays and the tripod does not. The centred double Cardan of "
                "truck driveshafts and steering columns."),
            "origin": (
                "Gap-analysis wave v0.8.0; classic ref: double Cardan (Spicer "
                "10-series) constant-velocity driveshaft joint"),
            "demo": "demo_double_cardan_joint",
        },
        {
            "file": "ball_socket_joint_demo.glb",
            "name": "ball_socket_joint",
            "module": "mechlib.joints",
            "signature": signature(ball_socket_joint),
            "description": (
                "A ball on a stem snaps into a split-finger socket whose mouth "
                "is pinched narrower than the ball, so the lip wraps past the "
                "equator and holds it: three rotational degrees of freedom "
                "inside a cone limited by the stem hitting the mouth rim. The "
                "tie-rod end and camera-mount joint, printed as two parts that "
                "click together."),
            "origin": "Gap-analysis wave v0.8.0",
            "demo": "demo_ball_socket_joint",
        },
        {
            "file": "knuckle_hinge_demo.glb",
            "name": "knuckle_hinge",
            "module": "mechlib.joints",
            "signature": signature(knuckle_hinge),
            "description": (
                "Interleaved barrels around one captive printed pin give a "
                "butt hinge that prints flat in a single piece and breaks free "
                "on the first swing. A full-width collar on each leaf ends in a "
                "radial flank through the pin axis, and the two flanks meet "
                "face to face to arrest the leaf at exactly stop_deg."),
            "origin": "Gap-analysis wave v0.8.0",
            "demo": "demo_knuckle_hinge",
        },
        {
            "file": "gimbal_rings_demo.glb",
            "name": "gimbal_rings",
            "module": "mechlib.joints",
            "signature": signature(gimbal_rings),
            "description": (
                "Nested rings hang from integral trunnion pins on alternating "
                "orthogonal axes and print flat as one piece: two rings are a "
                "Cardan suspension, three a full gimbal that keeps the centre "
                "level however the frame moves. Each ring radius is derived so "
                "its outer corners stay one clearance off the parent bore "
                "through the whole swing."),
            "origin": (
                "Gap-analysis wave v0.8.0; classic ref: Cardan suspension, "
                "gimbal-mounted compass"),
            "demo": "demo_gimbal_rings",
        },
        {
            "file": "gerotor_pump_demo.glb",
            "name": "gerotor_pump",
            "module": "mechlib.fluid",
            "signature": signature(gerotor_pump),
            "description": (
                "An internal-gear pump: a six-lobe trochoidal inner rotor "
                "running inside a seven-lobe outer rotor at an offset axis. "
                "The crescent chambers between the two open from zero on one "
                "side of the offset and close again on the other, carrying "
                "fluid from the inlet kidney port to the outlet. Displacement "
                "is measured off the real swept chamber polygons and lands in "
                "the metadata."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: Hill, gerotor patent US1682563 (1928)",
            "demo": "demo_gerotor_pump",
        },
        {
            "file": "hose_barb_demo.glb",
            "name": "hose_barb",
            "module": "mechlib.fluid",
            "signature": signature(hose_barb),
            "description": (
                "A hose tail: a stack of sawtooth rings on a stop boss and a "
                "mounting flange. Each ring steps straight out to a crest "
                "wider than the tube bore and then tapers back, so the tube "
                "walks on over the taper and jams against the square shoulder "
                "coming off. The foot can also be an external printed thread."),
            "origin": "Gap-analysis wave v0.8.0",
            "demo": "demo_hose_barb",
        },
        {
            "file": "rotary_spool_valve_demo.glb",
            "name": "rotary_spool_valve",
            "module": "mechlib.fluid",
            "signature": signature(rotary_spool_valve),
            "description": (
                "A rotary plug valve: a cross-drilled cylinder in a ported "
                "body. Each detent turns the plug so one L-shaped passage "
                "bridges a chosen pair of body ports and blocks the rest; the "
                "detent angles and the port pairs they join are derived from "
                "the port and passage angles rather than tabulated. Printed "
                "plug on printed bore leaks, so it routes rather than seals."),
            "origin": "Gap-analysis wave v0.8.0",
            "demo": "demo_rotary_spool_valve",
        },
        {
            "file": "peristaltic_pump_head_demo.glb",
            "name": "peristaltic_pump_head",
            "module": "mechlib.fluid",
            "signature": signature(peristaltic_pump_head),
            "description": (
                "A peristaltic pump head: three posts on a rotor pinch a "
                "silicone tube against a circular race, so each post drags a "
                "sealed slug of fluid round the wrap. The two tube slots leave "
                "the race tangentially, which is what stops a post shearing "
                "the tube where it peels off the wall. The tubing is a bought "
                "consumable, not a printed part."),
            "origin": "Gap-analysis wave v0.8.0",
            "demo": "demo_peristaltic_pump_head",
        },
        {
            "file": "linear_way_demo.glb",
            "name": "linear_way",
            "module": "mechlib.guides",
            "signature": signature(linear_way),
            "description": (
                "A prismatic guideway that constrains a body to one axis: a "
                "dovetail, vee, or T-slot section swept along the travel "
                "direction, a carriage opened out by the running clearance, "
                "and a tapered gib whose axial position sets the preload. "
                "45-degree ramps at both ends stop the travel and every "
                "inside corner carries a chip-relief groove."),
            "origin": (
                "Gap-analysis wave v0.8.0; classic ref: machine-tool slideways, "
                "Whitworth/Maudslay lineage, standard in Machinery's Handbook"),
            "demo": "demo_linear_way",
        },
        {
            "file": "telescoping_stage_demo.glb",
            "name": "telescoping_stage",
            "module": "mechlib.guides",
            "signature": signature(telescoping_stage),
            "description": (
                "Nested square tubes that extend to nearly the section count "
                "times one section length and cannot pull apart. Each parent "
                "carries a 45-degree inward lip at its far end and each child "
                "an external collar at its near end; the collar is wider than "
                "the lip opening, so the stage stops at full extension."),
            "origin": "Gap-analysis wave v0.8.0",
            "demo": "demo_telescoping_stage",
        },
        {
            "file": "idler_pulley_demo.glb",
            "name": "idler_pulley",
            "module": "mechlib.pulleys",
            "signature": signature(idler_pulley),
            "description": (
                "A crowned idler wheel shown facing a GT2 timing pulley: the "
                "running surface bulges a fraction of a millimetre from edge to "
                "mid-width so a flat belt walks itself back to centre under "
                "tension, no flange required. The same function also builds a "
                "toothed idler by reusing the timing_pulley tooth profile, and can "
                "swap the plain bore for a 608/695/MR105 bearing pocket."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: crowned flat-belt idler pulley",
            "demo": "demo_idler_pulley",
        },
        {
            "file": "eccentric_idler_mount_demo.glb",
            "name": "eccentric_idler_mount",
            "module": "mechlib.pulleys",
            "signature": signature(eccentric_idler_mount),
            "description": (
                "An eccentric take-up bushing carrying an idler pulley on its "
                "offset bore: turning the bushing with a hex key sweeps the idler "
                "through a circle equal to twice the eccentricity, setting belt "
                "tension, and a radial setscrew boss locks it in place. The same "
                "eccentric-bushing take-up used on printer belt idlers and chain "
                "tensioner sprockets."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: eccentric take-up bushing / belt idler tensioner",
            "demo": "demo_eccentric_idler_mount",
        },
        {
            "file": "belt_tensioner_demo.glb",
            "name": "belt_tensioner",
            "module": "mechlib.pulleys",
            "signature": signature(belt_tensioner),
            "description": (
                "A curved cantilever spring arm printed as one part: its tip is "
                "pushed back a set preload distance when mounted against the "
                "belt, so it keeps the idler loaded against the belt line with no "
                "metal spring and re-deflects as the belt stretches. Peak bending "
                "strain is checked against a PLA/PETG limit at build time."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: compliant cantilever belt/chain tensioner",
            "demo": "demo_belt_tensioner",
        },
        {
            "file": "drag_chain_link_demo.glb",
            "name": "drag_chain_link",
            "module": "mechlib.chains",
            "signature": signature(drag_chain_link),
            "description": (
                "One link of a print-in-place cable-carrier (energy chain): a "
                "male pivot boss nests into the next link's female socket on a "
                "vertical (self-bridging) bore, with a keyway tab/groove pair "
                "that lets the joint articulate freely up to bend_deg but jams "
                "solid the other way, so a run of these curls one direction and "
                "stays rigid the other. An optional press-fit lid caps the cable "
                "trough."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: e.g. igus E-chain, 1970s",
            "demo": "demo_drag_chain_link",
        },
        {
            "file": "drag_chain_demo.glb",
            "name": "drag_chain",
            "module": "mechlib.chains",
            "signature": signature(drag_chain),
            "description": (
                "An eight-link run of drag_chain_link posed with a straight "
                "section and a bend that curls the tail to its designed minimum "
                "bend radius, the classic U-turn a cable carrier makes guiding a "
                "wire or hose along a moving axis. Shows the link's stop actually "
                "working in an assembled run, not just in isolation."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: e.g. igus E-chain, 1970s",
            "demo": "demo_drag_chain",
        },
        {
            "file": "roller_chain_link_demo.glb",
            "name": "roller_chain_link",
            "module": "mechlib.chains",
            "signature": signature(roller_chain_link),
            "description": (
                "One pitch segment of a bush roller chain: inner and outer link "
                "plates, a bushing pressed between the inner plates, a roller "
                "that spins free around the bushing, and the pin that runs "
                "through it all. Sized to seat directly in a sprocket built by "
                "gears.roller_sprocket_2d at the same pitch and roller diameter."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: Hans Renold bush roller chain, 1880",
            "demo": "demo_roller_chain_link",
        },
        {
            "file": "roller_chain_demo.glb",
            "name": "roller_chain",
            "module": "mechlib.chains",
            "signature": signature(roller_chain),
            "description": (
                "A sprocket built by gears.roller_sprocket_2d with a run of bare "
                "rollers wrapped around 200 degrees of its pitch circle, each "
                "seated in its tooth gap with running clearance by construction: "
                "the geometric proof that roller_chain_link's roller diameter and "
                "pitch actually mate with the sprocket function."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: Hans Renold bush roller chain, 1880",
            "demo": "demo_roller_chain",
        },
        {
            "file": "chain_reverse_demo.glb",
            "name": "chain_reverse",
            "module": "mechlib.chains",
            "signature": signature(chain_reverse),
            "description": (
                "A backside idler on the slack span reverses rotation; bare "
                "rollers are seated with running clearance."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: Hans Renold bush roller chain, 1880",
            "demo": "demo_chain_reverse",
        },
        {
            "file": "chain_s_wrap_demo.glb",
            "name": "chain_s_wrap",
            "module": "mechlib.chains",
            "signature": signature(chain_s_wrap),
            "description": (
                "An S-wrap takeoff: the chain wraps the driver, then the "
                "backside of the output sprocket so the shaft reverses without "
                "gears; bare rollers are seated with running clearance."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: Hans Renold bush roller chain, 1880",
            "demo": "demo_chain_s_wrap",
        },
        {
            "file": "chain_dual_output_demo.glb",
            "name": "chain_dual_output",
            "module": "mechlib.chains",
            "signature": signature(chain_dual_output),
            "description": (
                "A driver sprocket, a backside idler, and a forward output on "
                "one serpentine run of bare rollers, each seated in its tooth "
                "gap with running clearance by construction: the geometric "
                "proof that one chain can take off simultaneous forward and "
                "reverse shafts, as on a jackshaft."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: Hans Renold bush roller chain, 1880",
            "demo": "demo_chain_dual_output",
        },
        {
            "file": "belleville_washer_demo.glb",
            "name": "belleville_washer",
            "module": "mechlib.flexures",
            "signature": signature(belleville_washer),
            "description": (
                "A coned annular disc spring, shown as a three-disc series "
                "stack with the cone direction alternating. Disc springs give "
                "the most force per unit volume of any printable spring, and "
                "above a free-cone-height to thickness ratio of sqrt(2) they "
                "snap through instead of resisting smoothly; the flag is "
                "derived from the geometry and stored in metadata."),
            "origin": ("Gap-analysis wave v0.8.0; classic ref: Julien "
                       "Belleville disc spring 1867, DIN 2093 proportions"),
            "demo": "demo_belleville_washer",
        },
        {
            "file": "coil_spring_demo.glb",
            "name": "coil_spring",
            "module": "mechlib.flexures",
            "signature": signature(coil_spring),
            "description": (
                "A helical compression spring swept along a cylinder, with one "
                "dead coil at each end closed up to a flat seat and a choice "
                "of round or rectangular wire. The pitch is bounded from below "
                "by the wire diameter plus a printable kerf, and the helix "
                "angle by a self-support floor, so unprintable geometry raises "
                "rather than slicing into a solid tube."),
            "origin": ("Gap-analysis wave v0.8.0; classic ref: closed-and-"
                       "ground helical compression spring, ASM spring design"),
            "demo": "demo_coil_spring",
        },
        {
            "file": "spiral_power_spring_demo.glb",
            "name": "spiral_power_spring",
            "module": "mechlib.flexures",
            "signature": signature(spiral_power_spring),
            "description": (
                "A flat Archimedean spiral strip anchored to an arbor at its "
                "inner end and to the barrel wall at its outer end: a clock "
                "mainspring. Winding the arbor stores the turns reported in "
                "metadata, and because its torque falls as it unwinds it is "
                "the part that pairs with grooved_drum(radius_law='fusee'), "
                "which exists to compensate exactly this."),
            "origin": ("Gap-analysis wave v0.8.0; classic ref: barrel and "
                       "fusee clock mainspring, 507 Mechanical Movements No. 46"),
            "demo": "demo_spiral_power_spring",
        },
        {
            "file": "leaf_spring_demo.glb",
            "name": "leaf_spring",
            "module": "mechlib.flexures",
            "signature": signature(leaf_spring),
            "description": (
                "A semi-elliptic multi-leaf spring: circular-arc leaves of "
                "decreasing length stacked on a common centre, the longest "
                "carrying a rolled eye at each end, held by a clamp band at "
                "the crown. Metadata reports the free camber, the travel to "
                "flat, and the rate from the classic n*E*b*t^3/(6*L^3) "
                "formula at the supplied modulus."),
            "origin": ("Gap-analysis wave v0.8.0; classic ref: semi-elliptic "
                       "carriage leaf spring, Obadiah Elliott 1804"),
            "demo": "demo_leaf_spring",
        },
        {
            "file": "flexure_stage_demo.glb",
            "name": "flexure_stage",
            "module": "mechlib.flexures",
            "signature": signature(flexure_stage),
            "description": (
                "A monolithic compound parallelogram flexure: an outer blade "
                "pair carries a secondary bar and an inner pair hangs back "
                "down from it to the motion stage, so the parasitic arc drop "
                "of one parallelogram cancels the other and the stage runs "
                "straight with no backlash or friction. Peak blade strain is "
                "computed and over-strained configurations are refused."),
            "origin": ("Gap-analysis wave v0.8.0; classic ref: compound "
                       "parallelogram flexure, Awtar and Slocum MIT PERG"),
            "demo": "demo_flexure_stage",
        },
        {
            "file": "plain_bushing_demo.glb",
            "name": "plain_bushing",
            "module": "mechlib.bearings",
            "signature": signature(plain_bushing),
            "description": (
                "A flanged plain bushing: a plastic sleeve that a steel shaft "
                "turns directly inside, with the running fit set explicitly by "
                "printing the bore at bore_d plus clear. Axial relief grooves "
                "hold grease and give swarf somewhere to go; the flange takes "
                "the light axial load a journal bearing can carry. The "
                "low-precision, high-reliability choice wherever a rolling "
                "bearing is overkill."),
            "origin": ("Gap-analysis wave v0.8.0; classic ref: sintered bronze "
                       "flange bushing (Oilite, 1930s)"),
            "demo": "demo_plain_bushing",
        },
        {
            "file": "thrust_washer_demo.glb",
            "name": "thrust_washer",
            "module": "mechlib.bearings",
            "signature": signature(thrust_washer),
            "description": (
                "An axial thrust bearing built as a pair of washers with "
                "toroidal raceways facing each other and a printed cage "
                "spacing bought steel balls between them. Set pair=False for a "
                "single flat thrust washer whose face is relieved with grease "
                "pockets or raised pads, which drops the contact area and with "
                "it the PV product."),
            "origin": ("Gap-analysis wave v0.8.0; classic ref: 51100-series "
                       "single-direction thrust ball bearing"),
            "demo": "demo_thrust_washer",
        },
        {
            "file": "printed_ball_bearing_demo.glb",
            "name": "printed_ball_bearing",
            "module": "mechlib.bearings",
            "signature": signature(printed_ball_bearing),
            "description": (
                "A radial ball bearing that prints in one piece, already "
                "assembled: inner race, spacer cage, balls, outer race. Each "
                "raceway is a ball-radius groove truncated at 45 degrees of "
                "latitude so no race wall ever overhangs past the FDM limit "
                "and nothing bridges over the balls, and each ball has its "
                "bottom cap cut off at 45 degrees so its first layer is a flat "
                "disc. The shoulders left by that truncation are narrower than "
                "the ball, which is what keeps it captured."),
            "origin": ("Gap-analysis wave v0.8.0; classic ref: Conrad deep-"
                       "groove ball bearing (1903), adapted to FDM limits"),
            "demo": "demo_printed_ball_bearing",
        },
        {
            "file": "kinematic_coupling_demo.glb",
            "name": "kinematic_coupling",
            "module": "mechlib.fixtures",
            "signature": signature(kinematic_coupling),
            "description": (
                "An exactly constrained two-plate mount: three balls on the top plate "
                "drop into three seats on the base and touch at exactly six points, one "
                "per degree of freedom, so the top plate has one resting pose and returns "
                "to it every time. Maxwell cuts three vee grooves; Kelvin cuts a trihedral "
                "pit, a vee, and a flat. Shown book-flipped so both mating faces are visible."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: Maxwell 1871 and Kelvin's "
                      "three-point clamp, still the standard optical-mount kinematic seat",
            "demo": "demo_kinematic_coupling",
        },
        {
            "file": "repeatable_dock_demo.glb",
            "name": "repeatable_dock",
            "module": "mechlib.fixtures",
            "signature": signature(repeatable_dock),
            "description": (
                "A kinematic coupling plus the preload that keeps it closed, since a bare "
                "coupling resists nothing in the lift-off direction and falls apart when "
                "tipped. A disc magnet on a central boss or a central cap screw pulls along "
                "the coupling axis only, so the preload never becomes a seventh contact, and "
                "a bolt circle phased between the seats mounts both halves."),
            "origin": "Gap-analysis wave v0.8.0; the standard tool-changer and probe-mount "
                      "arrangement (magnetic kinematic mounts, Renishaw-style probe docks)",
            "demo": "demo_repeatable_dock",
        },
        {
            "file": "three_point_leveller_demo.glb",
            "name": "three_point_leveller",
            "module": "mechlib.fixtures",
            "signature": signature(three_point_leveller),
            "description": (
                "Three screws thread through a table plate and rest their ball tips in "
                "kinematic seats on a base plate, holding the table on exactly six contacts "
                "while leaving height, tip, and tilt adjustable. One turn of one screw raises "
                "its corner by one thread pitch; mm_per_turn and tilt_per_turn_deg are in the "
                "metadata. This is how optical mounts and printer beds level without fighting "
                "an over-constrained bolt pattern."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: the kinematic three-screw "
                      "levelling mount used on optical tables and surface plates",
            "demo": "demo_three_point_leveller",
        },
        {
            "file": "oring_groove_demo.glb",
            "name": "oring_groove",
            "module": "mechlib.cutters",
            "signature": signature(oring_groove),
            "description": (
                "An O-ring gland cut into a flat face at a given pitch diameter. "
                "Groove width and depth are derived from the O-ring cross-section, "
                "target squeeze, and target gland fill rather than hardcoded, and "
                "the fill fraction is validated into the 70-85% band that keeps "
                "the ring from either leaking (too loose) or hydrolocking and "
                "extruding out of the joint (too tight)."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: Parker O-Ring Handbook gland design, AS568/ISO 3601.",
            "demo": "demo_oring_groove",
        },
        {
            "file": "labyrinth_seal_demo.glb",
            "name": "labyrinth_seal",
            "module": "mechlib.cutters",
            "signature": signature(labyrinth_seal),
            "description": (
                "A rotor comb and stator comb whose fins interleave around a shaft "
                "without ever touching, throttling flow across 2*teeth-1 stages "
                "through radial and axial clearance alone. Needs no elastomer and "
                "has no wear surface, so it is the seal to reach for on a printed "
                "rotating joint that only needs to keep out dust or splash."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: turbine labyrinth seal, early 20th c.",
            "demo": "demo_labyrinth_seal",
        },
        {
            "file": "gasket_channel_demo.glb",
            "name": "gasket_channel",
            "module": "mechlib.cutters",
            "signature": signature(gasket_channel),
            "description": (
                "A cord-stock gasket groove that follows an arbitrary closed 2D "
                "path, here a rounded-rectangle lid, rather than only a "
                "circular pitch diameter, for enclosures and kidney-shaped lids "
                "an O-ring can't seal."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: O-ring cord stock gasket practice.",
            "demo": "demo_gasket_channel",
        },
        {
            "file": "auxetic_panel_demo.glb",
            "name": "auxetic_panel",
            "module": "mechlib.lattices",
            "signature": signature(auxetic_panel),
            "description": (
                "A flat panel that gets WIDER, not narrower, when you stretch it: "
                "its internal cell topology supplies a negative Poisson's ratio "
                "instead of the base plastic. Seven interchangeable cell types "
                "(reentrant bowtie honeycomb, rigid rotating squares on corner "
                "hinges, Grima double-arrowhead cells, Grima star-shaped "
                "honeycomb cells, a chiral node-and-tangent-ligament grid, "
                "an anti-tetrachiral opposite-sense square grid, and "
                "houndstooth interlocking-L / broken-chevron cells) trade off "
                "stiffness, hinge fatigue life, and print complexity for the same "
                "auxetic effect."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: auxetic re-entrant honeycomb (Lakes, 1987)",
            "demo": "demo_auxetic_panel",
        },
        {
            "file": "honeycomb_panel_demo.glb",
            "name": "honeycomb_panel",
            "module": "mechlib.lattices",
            "signature": signature(honeycomb_panel),
            "description": (
                "A flat lightening panel of regular hexagonal cells with a solid "
                "border frame, so the edge is never a row of half-cells. Cells "
                "are flat-top (two sides horizontal): cell is the centre-to-centre "
                "pitch and the across-flats of the wall centreline, and strut_t "
                "is the printed wall. Stretching it in-plane makes it thinner "
                "transversely (positive Poisson), the opposite of auxetic_panel."),
            "origin": "Gap-analysis wave v0.11.x; classic ref: regular hexagonal honeycomb core",
            "demo": "demo_honeycomb_panel",
        },
        {
            "file": "isogrid_panel_demo.glb",
            "name": "isogrid_panel",
            "module": "mechlib.lattices",
            "signature": signature(isogrid_panel),
            "description": (
                "A flat NASA-style isogrid sheet: equilateral-triangle "
                "through-cells with ribs at 0/60/120 deg and a solid border "
                "frame, so the edge is never a row of half-cells. cell is the "
                "wall-centreline triangle side and vertex pitch; strut_t is "
                "the printed rib. Distinct from honeycomb_panel (flat-top hex "
                "holes). Print flat, no supports."),
            "origin": "Gap-analysis wave v0.11.x; classic ref: NASA isogrid (triangular rib sheet)",
            "demo": "demo_isogrid_panel",
        },
        {
            "file": "kagome_panel_demo.glb",
            "name": "kagome_panel",
            "module": "mechlib.lattices",
            "signature": signature(kagome_panel),
            "description": (
                "A flat Kagome (trihexagonal) lightening sheet whose through-holes "
                "are BOTH equilateral triangles and regular hexagons in the 3.6.3.6 "
                "tiling, with a solid border frame so the edge is never a row of "
                "half-cells. cell is the coarse vertex pitch (hex-hole centre "
                "spacing); strut_t is the printed wall between a triangle and a "
                "neighbouring hex. Distinct from honeycomb_panel (hex holes only) "
                "and isogrid_panel (triangle holes only). Print flat, no supports."),
            "origin": "Gap-analysis wave v0.11.x; classic ref: Kagome / trihexagonal lattice",
            "demo": "demo_kagome_panel",
        },
        {
            "file": "bcc_lattice_demo.glb",
            "name": "bcc_lattice",
            "module": "mechlib.lattices",
            "signature": signature(bcc_lattice),
            "description": (
                "A 3D body-centred-cubic strut lattice: each cubic cell has "
                "eight round struts from its corners to a shared body-centre "
                "node, corners shared with neighbours so the tiled block is one "
                "continuous space-frame. strut_d is the round strut diameter "
                "(nozzle-snapped); node_d blends the joints. Unlike the flat "
                "honeycomb/isogrid/kagome sheets this is a volumetric truss for "
                "lightweight cores and crush structures. metadata reports "
                "relative_density. Shallow body-diagonal overhangs want a slow "
                "profile or light support."),
            "origin": "Gap-analysis wave v0.11.x; classic ref: BCC FFF metamaterial truss",
            "demo": "demo_bcc_lattice",
        },
        {
            "file": "kerf_bend_cutter_demo.glb",
            "name": "kerf_bend_cutter",
            "module": "mechlib.lattices",
            "signature": signature(kerf_bend_cutter),
            "description": (
                "A cutter that turns a rigid flat slab into a bendable, twistable, "
                "or rollable one by slitting it with a staggered kerf pattern, the "
                "same trick as laser-kerfed plywood. Slit layouts include straight "
                "lattice for single-axis bending or a living hinge, diagonal for "
                "torsion, spiral for a helical cylindrical wrap, wave for sinusoidal "
                "channels, hex for hexagonal living-hinge edge slits, cross for "
                "an X-lattice of bars plus ~30° diagonal arms, chevron for "
                "nested 45° arrowhead slits, and diamond for elongated "
                "brick-wall outline rhombi, and fishbone for paired 45° "
                "herringbone ribs, plus biaxial orthogonal slits for a "
                "2-axis wrap; all "
                "validate kerf width and bridge width against real FDM printability "
                "floors before cutting."),
            "origin": "Gap-analysis wave v0.8.0; classic ref: kerf bending (laser-cut plywood furniture)",
            "demo": "demo_kerf_bend_cutter",
        },
        # ------------------------------------------------------------------
        # Gap-analysis wave v0.9.0: textbook classics still missing.
        # ------------------------------------------------------------------
        {
            "file": "slider_crank_demo.glb",
            "name": "slider_crank",
            "module": "mechlib.linkages",
            "signature": signature(slider_crank),
            "description": (
                "The fundamental rotary-to-reciprocating conversion: a crank disc "
                "drives a connecting rod whose far pin rides a guided slider. "
                "Stroke is exactly 2*crank_r for the inline case. Every piston "
                "engine, compressor, and many shapers start from this chain."),
            "origin": (
                "Gap-analysis wave v0.9.0; classic ref: 507 Mechanical Movements "
                "crank and slider / steam-engine piston drive"),
            "demo": "demo_slider_crank",
        },
        {
            "file": "chebyshev_linkage_demo.glb",
            "name": "chebyshev_linkage",
            "module": "mechlib.linkages",
            "signature": signature(chebyshev_linkage),
            "description": (
                "Chebyshev's lambda four-bar: rockers of 5u on a ground of 4u "
                "joined by a coupler of 2u. The coupler midpoint holds an "
                "approximate straight line (error a few hundredths of the unit) "
                "over its working stroke. The Russian school's answer to Watt."),
            "origin": (
                "Gap-analysis wave v0.9.0; classic ref: P. L. Chebyshev lambda "
                "linkage (1869)"),
            "demo": "demo_chebyshev",
        },
        {
            "file": "scott_russell_linkage_demo.glb",
            "name": "scott_russell_linkage",
            "module": "mechlib.linkages",
            "signature": signature(scott_russell_linkage),
            "description": (
                "Exact straight-line motion from a half-length inversor: a bar "
                "of length 2L has its midpoint driven on a circle of radius L "
                "about a pivot on the slider guide. One end rides the guide; the "
                "other draws a diameter line of stroke 4L. Far fewer bars than "
                "Peaucellier, at the cost of one prismatic joint."),
            "origin": (
                "Gap-analysis wave v0.9.0; classic ref: John Scott Russell "
                "straight-line linkage (1840s)"),
            "demo": "demo_scott_russell",
        },
        {
            "file": "bell_crank_demo.glb",
            "name": "bell_crank",
            "module": "mechlib.linkages",
            "signature": signature(bell_crank),
            "description": (
                "A two-arm lever on a ground pivot that redirects force through "
                "a fixed included angle (default 90 degrees). The simplest force "
                "redirector in mechanical design: brake pedals, throttle links, "
                "and print-bed levellers all use one."),
            "origin": (
                "Gap-analysis wave v0.9.0; classic ref: 507 Mechanical Movements "
                "bell-crank lever"),
            "demo": "demo_bell_crank",
        },
        {
            "file": "face_cam_demo.glb",
            "name": "face_cam",
            "module": "mechlib.cams",
            "signature": signature(face_cam),
            "description": (
                "An axial face cam: the end face of a disc carries a raised "
                "track whose height follows a motion-law program, and a roller "
                "follower rides it for pure axial stroke. Complements plate "
                "(radial) and barrel (groove-on-drum) cams."),
            "origin": (
                "Gap-analysis wave v0.9.0; classic ref: face / end cam, "
                "automotive and textile machinery"),
            "demo": "demo_face_cam",
        },
        {
            "file": "swash_plate_demo.glb",
            "name": "swash_plate",
            "module": "mechlib.linear",
            "signature": signature(swash_plate),
            "description": (
                "A disc fixed at a tilt to a spinning shaft drives a ring of "
                "piston shoes through an axial stroke of 2*pitch_r*tan(tilt). "
                "The rotary-to-multi-piston conversion of axial piston pumps "
                "and helicopter cyclic controls."),
            "origin": (
                "Gap-analysis wave v0.9.0; classic ref: swash-plate axial "
                "piston pump / helicopter rotor control"),
            "demo": "demo_swash_plate",
        },
        {
            "file": "screw_jack_demo.glb",
            "name": "screw_jack",
            "module": "mechlib.linear",
            "signature": signature(screw_jack),
            "description": (
                "A printed-thread screw standing in a base nut with a lifting "
                "pad on top. One turn raises the pad by the thread pitch. The "
                "simplest self-locking rotary-to-linear actuator."),
            "origin": (
                "Gap-analysis wave v0.9.0; classic ref: screw jack / "
                "machine-screw linear actuator"),
            "demo": "demo_screw_jack",
        },
        {
            "file": "rack_pinion_demo.glb",
            "name": "rack_pinion",
            "module": "mechlib.linear",
            "signature": signature(rack_pinion),
            "description": (
                "An involute spur pinion posed in mesh with a straight rack. "
                "One pinion turn advances the rack by pi*module*teeth along "
                "the pitch line. Steering racks, linear stages, and CNC axes."),
            "origin": (
                "Gap-analysis wave v0.9.0; classic ref: rack and pinion "
                "drive"),
            "demo": "demo_rack_pinion",
        },
        {
            "file": "dog_clutch_demo.glb",
            "name": "dog_clutch",
            "module": "mechlib.clutches",
            "signature": signature(dog_clutch),
            "description": (
                "Two hubs with interleaved axial teeth that lock in torsion "
                "when pressed together and freewheel when withdrawn. Positive "
                "engagement, no slip once seated: gearbox dog rings and "
                "quick-change spindles."),
            "origin": (
                "Gap-analysis wave v0.9.0; classic ref: dog / claw clutch"),
            "demo": "demo_dog_clutch",
        },
        {
            "file": "hirth_coupling_demo.glb",
            "name": "hirth_coupling",
            "module": "mechlib.couplings",
            "signature": signature(hirth_coupling),
            "description": (
                "Two faces carrying radial V-teeth that seat into each other's "
                "valleys. Self-centring, zero-backlash torsion once preloaded: "
                "the standard machine-tool spindle and indexing-table face "
                "coupling."),
            "origin": (
                "Gap-analysis wave v0.9.0; classic ref: Albert Hirth face "
                "coupling (patent 1894)"),
            "demo": "demo_hirth_coupling",
        },
        {
            "file": "clevis_demo.glb",
            "name": "clevis",
            "module": "mechlib.joints",
            "signature": signature(clevis),
            "description": (
                "A U-fork, a pin, and an eye: the basic pin joint for linking "
                "a rod end to a lever or turnbuckle. Prints as three separate "
                "parts that assemble with the designed pin clearance."),
            "origin": (
                "Gap-analysis wave v0.9.0; classic ref: clevis pin joint / "
                "shackle"),
            "demo": "demo_clevis",
        },
        {
            "file": "external_gear_pump_demo.glb",
            "name": "external_gear_pump",
            "module": "mechlib.fluid",
            "signature": signature(external_gear_pump),
            "description": (
                "Two meshing spur gears in a figure-eight bore carry fluid "
                "around the outside of each gear from inlet to outlet. The "
                "classic positive-displacement hydraulic pump; complements "
                "the internal gerotor already in the library."),
            "origin": (
                "Gap-analysis wave v0.9.0; classic ref: external gear pump"),
            "demo": "demo_external_gear_pump",
        },
        {
            "file": "harmonic_drive_demo.glb",
            "name": "harmonic_drive",
            "module": "mechlib.drives",
            "signature": signature(harmonic_drive),
            "description": (
                "A strain-wave gear: an elliptical wave generator deforms a "
                "thin flex spline so it meshes a rigid circular spline at two "
                "opposite zones. Ratio is -fs_teeth/(cs_teeth-fs_teeth); the "
                "default 50:48 pair is 24:1. The high-ratio compact reducer "
                "of robot joints."),
            "origin": (
                "Gap-analysis wave v0.9.0; classic ref: C. W. Musser "
                "harmonic drive (1957)"),
            "demo": "demo_harmonic_drive",
        },
        {
            "file": "v_belt_pulley_demo.glb",
            "name": "v_belt_pulley",
            "module": "mechlib.pulleys",
            "signature": signature(v_belt_pulley),
            "description": (
                "A trapezoidal-groove wedge-belt sheave with stock hub "
                "styles (flat plate, one-sided, two-sided hub), keyway and "
                "setscrew options. Washing machines, drill presses, lathes, "
                "and HVAC blowers."),
            "origin": (
                "Gap-analysis wave v0.10.0; classic ref: V-belt drive"),
            "demo": "demo_v_belt_pulley",
        },
        {
            "file": "flywheel_demo.glb",
            "name": "flywheel",
            "module": "mechlib.drives",
            "signature": signature(flywheel),
            "description": (
                "A rim-heavy energy-storage wheel, spoked or webbed, with "
                "mass, moment of inertia, and stored energy computed from "
                "the actual mesh. Punch presses, engines, spin-casters, "
                "and gyro demos."),
            "origin": (
                "Gap-analysis wave v0.10.0; classic ref: engine / press "
                "flywheel"),
            "demo": "demo_flywheel",
        },
        {
            "file": "shaft_collar_demo.glb",
            "name": "shaft_collar",
            "module": "mechlib.mechanisms",
            "signature": signature(shaft_collar),
            "description": (
                "A split clamping collar (or setscrew ring) that axially "
                "locates bearings, gears, and sprockets on a shaft without "
                "machining a shoulder into it."),
            "origin": (
                "Gap-analysis wave v0.10.0; classic ref: clamping shaft "
                "collar"),
            "demo": "demo_shaft_collar",
        },
        {
            "file": "star_knob_demo.glb",
            "name": "star_knob",
            "module": "mechlib.mechanisms",
            "signature": signature(star_knob),
            "description": (
                "A DIN 6336 style lobed clamping knob with a printed "
                "internal thread. The hand-operated fastener of jigs, "
                "fixtures, machine adjustment, and camera rigs."),
            "origin": (
                "Gap-analysis wave v0.10.0; classic ref: DIN 6336 star "
                "knob"),
            "demo": "demo_star_knob",
        },
        {
            "file": "handwheel_demo.glb",
            "name": "handwheel",
            "module": "mechlib.mechanisms",
            "signature": signature(handwheel),
            "description": (
                "A spoked handwheel for leadscrew and valve adjustment, "
                "here in the crank variant with a free-spinning captured "
                "grip. Manual drives for screw jacks, indexing tables, "
                "and winches."),
            "origin": (
                "Gap-analysis wave v0.10.0; classic ref: DIN 950 spoked "
                "handwheel / crank handle"),
            "demo": "demo_handwheel",
        },
        {
            "file": "beam_coupling_demo.glb",
            "name": "beam_coupling",
            "module": "mechlib.couplings",
            "signature": signature(beam_coupling),
            "description": (
                "A helical-slit flexible coupling: torsionally stiff yet "
                "compliant in angular, parallel, and axial misalignment. "
                "The standard printed coupling of 3D-printer Z axes and "
                "CNC builds."),
            "origin": (
                "Gap-analysis wave v0.10.0; classic ref: helical beam / "
                "slit coupling"),
            "demo": "demo_beam_coupling",
        },
        {
            "file": "lead_screw_demo.glb",
            "name": "lead_screw",
            "module": "mechlib.linear",
            "signature": signature(lead_screw),
            "description": (
                "A trapezoidal (Tr) power screw and matching nut: flat "
                "crests print cleanly at power-screw leads where a sharp V "
                "thread would not. Vices, C-clamps, Z-axis stages, and "
                "press screws."),
            "origin": (
                "Gap-analysis wave v0.10.0; classic ref: trapezoidal lead "
                "screw (ISO 2903 Tr / ACME)"),
            "demo": "demo_lead_screw",
        },
        {
            "file": "roller_follower_demo.glb",
            "name": "roller_follower",
            "module": "mechlib.cams",
            "signature": signature(roller_follower),
            "description": (
                "A pivoted lever carrying a roller (plain, or a bought 625 "
                "bearing in a press pocket) that rides a plate, heart, or "
                "snail cam. Valve lifters, pump drives, and automata."),
            "origin": (
                "Gap-analysis wave v0.10.0; classic ref: oscillating "
                "roller follower"),
            "demo": "demo_roller_follower",
        },
        {
            "file": "detent_pair_demo.glb",
            "name": "detent_pair",
            "module": "mechlib.indexing",
            "signature": signature(detent_pair),
            "description": (
                "A spring-loaded plunger clicking into the notches of a "
                "dial, guided by a housing with a coil-spring pocket. "
                "Pan-tilt heads, adjustable arms, rotary selectors, and "
                "folding-leg locks."),
            "origin": (
                "Gap-analysis wave v0.10.0; classic ref: spring detent / "
                "click positioning"),
            "demo": "demo_detent_pair",
        },
        {
            "file": "star_wheel_demo.glb",
            "name": "star_wheel",
            "module": "mechlib.indexing",
            "signature": signature(star_wheel),
            "description": (
                "A rotating disc with concave pockets that meters "
                "cylindrical objects at a fixed pitch. The packaging-"
                "industry star wheel of filling, capping, and sorting "
                "machines."),
            "origin": (
                "Gap-analysis wave v0.10.0; classic ref: packaging star "
                "wheel"),
            "demo": "demo_star_wheel",
        },
        {
            "file": "ratchet_wheel_pawl_demo.glb",
            "name": "ratchet_wheel_pawl",
            "module": "mechlib.ratchets",
            "signature": signature(ratchet_wheel_pawl),
            "description": (
                "The canonical bolt-on ratchet: an external sawtooth wheel "
                "and a separate pivoted pawl with a leaf or coil spring. "
                "Winch drums, come-alongs, windlasses, and webbing "
                "tensioners."),
            "origin": (
                "Gap-analysis wave v0.10.0; classic ref: ratchet wheel "
                "and pawl"),
            "demo": "demo_ratchet_wheel_pawl",
        },
        {
            "file": "thread_insert_demo.glb",
            "name": "thread_insert",
            "module": "mechlib.fasteners",
            "signature": signature(thread_insert),
            "description": (
                "A heat-set threaded insert, its receiving boss, and the "
                "tapered cavity cutter: durable machine-screw threads in "
                "printed parts. Enclosure lids, motor mounts, and "
                "adjustment points."),
            "origin": (
                "Gap-analysis wave v0.10.0; classic ref: heat-set insert "
                "(McMaster 94180A style)"),
            "demo": "demo_thread_insert",
        },
        {
            "file": "tslot_nut_demo.glb",
            "name": "tslot_nut",
            "module": "mechlib.fasteners",
            "signature": signature(tslot_nut),
            "description": (
                "A printed drop-in or slide-in nut that keys into "
                "2020/3030/4040 aluminium-extrusion slots, with a printed "
                "thread and an anti-rattle spring leaf. The standard way "
                "to bolt parts to printer frames and CNC gantries."),
            "origin": (
                "Gap-analysis wave v0.10.0; classic ref: T-slot / "
                "extrusion nut"),
            "demo": "demo_tslot_nut",
        },
        {
            "file": "shaft_key_demo.glb",
            "name": "shaft_key",
            "module": "mechlib.fasteners",
            "signature": signature(shaft_key),
            "description": (
                "A DIN 6885 sunk key with its shaft and hub keyway "
                "cutters, sized from the shaft diameter table. Torque "
                "transmission into gear hubs, hand cranks, and pulleys "
                "where a D-bore slips."),
            "origin": (
                "Gap-analysis wave v0.10.0; classic ref: parallel / "
                "Woodruff machine key (DIN 6885)"),
            "demo": "demo_shaft_key",
        },
        {
            "file": "annular_snap_demo.glb",
            "name": "annular_snap",
            "module": "mechlib.closures",
            "signature": signature(annular_snap),
            "description": (
                "A circumferential ridge snapping into a groove: the "
                "cylindrical snap fit of bottle caps, filter housings, "
                "pipe couplings, and round enclosure lids, as unionable "
                "ridge and groove features."),
            "origin": (
                "Gap-analysis wave v0.10.0; classic ref: annular snap-fit "
                "joint"),
            "demo": "demo_annular_snap",
        },
        {
            "file": "check_valve_demo.glb",
            "name": "check_valve",
            "module": "mechlib.fluid",
            "signature": signature(check_valve),
            "description": (
                "A ball-check non-return valve built around a bought "
                "bearing ball on a conical seat, with a snap-on cap and "
                "hose-barb ports. Pump outlet lines, siphon breaks, and "
                "air lines."),
            "origin": (
                "Gap-analysis wave v0.10.0; classic ref: ball check "
                "valve"),
            "demo": "demo_check_valve",
        },
        {
            "file": "bellows_suction_cup_demo.glb",
            "name": "bellows_suction_cup",
            "module": "mechlib.grippers",
            "signature": signature(bellows_suction_cup),
            "description": (
                "A bellows vacuum cup for TPU pick-and-place: a thin "
                "sealing lip, compliant folds, and a hose-barb stem for "
                "the vacuum line, with collapsed height and cup volume in "
                "the metadata."),
            "origin": (
                "Gap-analysis wave v0.10.0; classic ref: bellows vacuum "
                "suction cup"),
            "demo": "demo_bellows_suction_cup",
        },
    ]

    previous_added = {}
    index_path = OUTPUT_DIR / "index.json"
    if index_path.exists():
        try:
            old = json.loads(index_path.read_text(encoding="utf-8"))
            for old_m in old.get("models", []):
                if old_m.get("file") and old_m.get("added"):
                    previous_added[old_m["file"]] = old_m["added"]
        except (OSError, json.JSONDecodeError, TypeError):
            previous_added = {}

    manifest_models = []
    built_demos = []
    animated = []
    for model in models:
        demo_name = model.pop("demo")
        built_demos.append(demo_name)
        demo_fn = getattr(demos, demo_name)
        started = time.perf_counter()
        meshes = demo_fn()
        # Measured, not guessed: the playground uses this to decide which parts
        # are too heavy to regenerate on every slider tick. Pyodide runs these
        # roughly 10-50x slower than native, so a part costing hundreds of
        # milliseconds here costs seconds in a browser.
        model["build_ms"] = int(round((time.perf_counter() - started) * 1000))
        export_model(model["file"], meshes)
        category = _category(model["module"])
        group, _title, _blurb = CATEGORIES[category]
        model["category"] = category
        model["group"] = group
        model["rank"] = CATEGORY_ORDER[category]
        # Derived card facts: how many bodies the demo returns, how big it is in
        # mm, and whether it is tunable. Nothing here is hand-maintained.
        model["parts"] = len(meshes)
        model["size_mm"] = [
            round(float(v), 1)
            for v in trimesh.util.concatenate([m for _n, m, _c in meshes]).extents
        ]
        play = _play_field(demo_name, demo_fn)
        if play is not None:
            model["play"] = play
        if demo_name in ANIMATE:
            animation = _bake_animation(demo_name, demo_fn)
            if animation is not None:
                _check_node_names(model["file"], animation["bodies"])
                model["animation"] = animation
                animated.append(demo_name)
        # Real machinery situations (not code). Table lives in applications.py.
        model["applications"] = applications_for(
            model["file"], model.get("description", ""))
        # Drop any leftover code-usage field from older index builds.
        model.pop("usage", None)
        model["added"] = previous_added.get(model["file"]) or datetime.now(
            timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        manifest_models.append(model)

    unbuilt = set(ANIMATE) - set(built_demos)
    if unbuilt:
        raise KeyError("ANIMATE names demos with no gallery model: %s"
                       % sorted(unbuilt))
    print("baked animation for %d of %d listed demos" % (len(animated), len(ANIMATE)))

    manifest_models.sort(key=lambda m: m["rank"])

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
        {
            "module": module,
            "signature": signature(function),
            "description": description,
            "category": _category(module),
        }
        for module, function, description in utility_functions
    ]

    demos_src = Path(__file__).resolve().parent / "demos.py"
    demos_dst = PLAYGROUND_DIR / "demos.py"
    shutil.copy2(demos_src, demos_dst)
    print("wrote %s" % demos_dst.relative_to(ROOT))

    wheel_name = _build_mechlib_wheel()

    used_categories = {m["category"] for m in manifest_models}
    used_categories.update(u["category"] for u in utilities)

    manifest = {
        "version": mechlib.__version__,
        "groups": [
            {"key": key, "title": title, "blurb": blurb}
            for key, title, blurb in CATEGORY_GROUPS
        ],
        "categories": [
            {
                "key": key,
                "group": group,
                "title": title,
                "blurb": blurb,
                "rank": CATEGORY_ORDER[key],
            }
            for key, (group, title, blurb) in CATEGORIES.items()
            if key in used_categories
        ],
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
    index_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print("wrote %s" % index_path.relative_to(ROOT))


if __name__ == "__main__":
    build()
