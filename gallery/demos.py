"""Parametrized gallery demos for native build and Pyodide playground.

Each ``demo_*`` function returns a list of ``(name, mesh, color)`` tuples
matching the gallery GLB writer. Defaults reproduce today's GLBs byte-for-byte.
"""

from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np
import shapely.geometry as sg
import trimesh
from shapely import affinity
from shapely.geometry import Point, box
from shapely.ops import unary_union

import mechlib
from mechlib import find_vitamin, vitamin, vitamin_addresses
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
    _hooke_spider_matrix,
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
from mechlib.grippers import (
    collet_chuck,
    eccentric_cam_clamp,
    iris_control_range,
    iris_diaphragm,
)
from mechlib.guides import linear_way, telescoping_stage
from mechlib.indexing import (
    escapement,
    geneva_pair,
    geneva_wheel_angle,
    intermittent_gear_pair,
)
from mechlib.joints import ball_socket_joint, clevis, gimbal_rings, knuckle_hinge
from mechlib.lattices import auxetic_panel, honeycomb_panel, isogrid_panel, kerf_bend_cutter
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
from mechlib.meshutil import sub, uni
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
    compliant_clutch,
    pip_ratchet_hub,
    ratchet_ring,
    spring_cartridge_ratchet,
)
from mechlib.sweep import extrude_twist, loft, ring_pts, swept_keyed_bore
from mechlib.text import text_block, text_polygon

# RGBA colors matching the gallery GLB writer (alpha always 255).
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

Color = Tuple[int, int, int, int]
MeshEntry = Tuple[str, trimesh.Trimesh, Color]
MeshList = List[MeshEntry]


def _spin(mesh, deg, axis=(0.0, 0.0, 1.0), center=(0.0, 0.0, 0.0)):
    """Rotate a copy of ``mesh`` by ``deg`` about ``axis`` through ``center``.

    Motion-phase parameters pose already-built meshes with this rather than
    regenerating geometry at the new pose. That keeps the motion a bit-exact
    rigid transform (which is what ``build_gallery.py`` bakes into the gallery
    animation) and leaves the default GLB untouched at zero phase.
    """
    posed = mesh.copy()
    if deg:
        posed.apply_transform(trimesh.transformations.rotation_matrix(
            math.radians(deg), axis, center))
    return posed

# PLAY maps demo function name -> {kwarg: (min, max, step)} for slider UI.
# Invariant enforced for every entry: min <= default <= max, and
# (default - min) is an exact multiple of step (within 1e-9), so the slider
# can always be dragged back to the demo's own default. Segment/sections
# ranges are otherwise kept as cheap as the ~400ms native / ~2s Pyodide
# regen budget allows; a few (e.g. demo_printed_worm) lower the demo's own
# default tessellation to fit that budget -- see the per-demo comments.
PLAY: dict = {
    "demo_cyl": {
        "r": (2, 20, 1),
        "h": (4, 40, 1),
        "sections": (24, 96, 8),
    },
    "demo_boxc": {
        "w": (8, 40, 1),
        "d": (6, 30, 1),
        "h": (4, 24, 1),
    },
    "demo_rbox": {
        "w": (10, 40, 1),
        "d": (8, 30, 1),
        "h": (4, 24, 1),
        "r": (1, 8, 1),
    },
    "demo_frustum": {
        "r0": (2, 14, 1),
        "r1": (2, 16, 1),
        "h": (6, 30, 1),
        "sections": (24, 96, 8),
    },
    "demo_sector2d": {
        "a0_deg": (-90.0, 0.0, 5.0),
        "a1_deg": (90.0, 270.0, 10.0),
        "radius": (8.0, 28.0, 1.0),
        "extrude_h": (2.0, 10.0, 0.5),
        "n": (24, 64, 8),
    },
    "demo_hex_poly": {
        "af": (8.0, 28.0, 1.0),
        "extrude_h": (2.0, 10.0, 0.5),
    },
    "demo_extrude_twist": {
        "base_r": (4.0, 12.0, 0.5),
        "lobe_amp": (0.5, 4.0, 0.25),
        "height": (15.0, 40.0, 1.0),
        "turns_deg": (180.0, 720.0, 30.0),
        "count": (24, 96, 8),
        "z_samples": (21, 81, 10),
    },
    "demo_swept_keyed_bore": {
        "radius": (4.0, 12.0, 0.5),
        "flat_x": (1.7, 8.0, 0.5),
        "free_angle": (20.0, 90.0, 5.0),
        "extrude_h": (2.0, 8.0, 0.5),
        "spacing": (16.0, 30.0, 1.0),
    },
    "demo_spur_gear_pair": {
        "drive_deg": (0.0, 360.0, 5.0),
        "n_driver": (12, 24, 1),
        "n_driven": (18, 36, 1),
        "module": (1.0, 2.5, 0.25),
        "thickness": (3.0, 10.0, 0.5),
        "backlash": (0.1, 0.6, 0.05),
    },
    "demo_board_cradle": {
        "board_w": (24.0, 60.0, 2.0),
        "board_d": (18.0, 50.0, 2.0),
        "board_h": (4.0, 12.0, 1.0),
        "board_t": (1.0, 2.5, 0.2),
        "standoff": (2.0, 8.0, 0.5),
    },
    "demo_teardrop": {
        "r": (2.0, 8.0, 0.5),
        "length": (12.0, 36.0, 2.0),
        "block_w": (16.0, 36.0, 2.0),
        "block_d": (12.0, 28.0, 2.0),
        "block_h": (10.0, 24.0, 2.0),
    },
    "demo_ss_bore": {
        "R": (3.0, 8.0, 0.5),
        "Robj": (2.5, 7.0, 0.5),
        "length": (14.0, 36.0, 2.0),
        "split_z": (6.0, 16.0, 1.0),
    },
    "demo_dbore": {
        "shaft_d": (3.5, 8.0, 0.5),
        "flat": (2.45, 6.0, 0.25),
        "hub_r": (5.0, 12.0, 0.5),
        "hub_h": (6.0, 16.0, 1.0),
        "clear": (0.05, 0.3, 0.05),
    },
    "demo_counterbore": {
        "through_d": (2.0, 6.0, 0.2),
        "cb_d": (4.0, 12.0, 0.5),
        "cb_h": (1.2, 6.0, 0.5),
        "length": (10.0, 24.0, 1.0),
    },
    "demo_crush_ribs": {
        "comp_w": (10.0, 28.0, 1.0),
        "comp_d": (8.0, 20.0, 1.0),
        "comp_h": (8.0, 24.0, 1.0),
        "rib_t": (0.4, 1.2, 0.1),
        "count": (2, 5, 1),
        "interference": (0.02, 0.25, 0.05),
    },
    "demo_press_lid": {
        "box_w": (24.0, 48.0, 2.0),
        "box_d": (20.0, 40.0, 2.0),
        "box_h": (10.0, 22.0, 1.0),
        "wall": (1.2, 3.0, 0.2),
        "lid_lift": (14.0, 32.0, 1.0),
    },
    "demo_clamshell_shiplap": {
        "w": (24.0, 48.0, 2.0),
        "d": (20.0, 40.0, 2.0),
        "h": (10.0, 22.0, 1.0),
        "spacing": (16.0, 32.0, 2.0),
    },
    "demo_ydovetail": {
        "tongue_y0": (-12.0, -4.0, 1.0),
        "tongue_y1": (4.0, 12.0, 1.0),
        "clear": (0.1, 0.5, 0.05),
        "receiver_w": (12.0, 24.0, 1.0),
        "receiver_d": (14.0, 28.0, 1.0),
    },
    "demo_snap_pair": {
        "depth": (6.0, 16.0, 1.0),
        "finger_dx": (4.0, 14.0, 1.0),
        "finger_dz": (0.0, 6.0, 0.5),
    },
    "demo_nut_slot": {
        "length": (10.0, 24.0, 1.0),
        "nib": (0, 1, 1),
        "nut_af": (4.0, 8.0, 0.5),
        "nut_h": (1.35, 4.0, 0.25),
    },
    "demo_pins_and_posts": {
        "post_h": (6, 16, 1),
        "pin_r": (1, 4, 1),
        "pin_h": (4, 12, 1),
        "socket_r": (1.5, 4.0, 0.25),
        "socket_depth": (3, 8, 1),
    },
    "demo_spur_gear_mesh": {
        "drive_deg": (0.0, 360.0, 5.0),
        "teeth": (12, 36, 1),
        "module": (1.0, 2.5, 0.25),
        "thickness": (3.0, 12.0, 0.5),
        "bore_d": (2.0, 10.0, 0.5),
    },
    "demo_roller_sprocket": {
        "teeth": (10, 16, 1),
        "pitch": (8.0, 12.0, 0.5),
        "thickness": (3.0, 10.0, 0.5),
        "pin_r": (0.5, 1.5, 0.25),
    },
    "demo_thread": {
        # COARSE_PITCH only keys 3/4/5/6/8; min/max must both be keys.
        "d": (5.0, 8.0, 3.0),
        "length": (10.0, 28.0, 1.0),
        "head_r": (4.0, 12.0, 0.5),
        "head_h": (2.0, 8.0, 0.5),
        "spacing": (11.0, 30.0, 1.0),
    },
    "demo_knurl": {
        "r": (4.0, 14.0, 0.5),
        "h": (4.0, 14.0, 0.5),
        "n": (12, 32, 2),
    },
    "demo_lighten_grid": {
        "panel_w": (28.0, 60.0, 2.0),
        "panel_d": (18.0, 44.0, 2.0),
        "panel_t": (1.5, 5.0, 0.5),
        "cell": (3.0, 8.0, 0.5),
        "wall": (1.0, 4.0, 0.5),
    },
    "demo_text_polygon": {
        "size": (8.0, 18.0, 1.0),
        "extrude_h": (0.7, 3.0, 0.2),
        "plaque_w": (36.0, 70.0, 2.0),
        "plaque_d": (12.0, 28.0, 1.0),
        "plaque_r": (1.0, 6.0, 0.5),
    },
    "demo_worm": {
        "module": (1.0, 2.5, 0.25),
        "worm_length": (14.0, 36.0, 2.0),
        "pitch_d": (9.8, 20.0, 0.5),
        "wheel_teeth": (24, 48, 2),
        "wheel_thickness": (4.0, 14.0, 1.0),
        "backlash": (0.15, 0.6, 0.05),
    },
    "demo_spur_gear_sector": {
        "module": (1.0, 2.5, 0.25),
        "teeth": (24, 48, 2),
        "thickness": (4.0, 12.0, 1.0),
        "bore": (2.0, 8.0, 0.5),
        "sector_deg": (90.0, 150.0, 5.0),
        "hub_d": (10.0, 20.0, 1.0),
    },
    "demo_loft": {
        "r0": (5.0, 14.0, 0.5),
        "r_mid": (4.0, 10.0, 0.5),
        "r_top": (3.0, 9.0, 0.5),
        "height": (18.0, 40.0, 2.0),
        "ring_n": (24, 64, 8),
        "resolution": (16, 32, 4),
    },
    "demo_push_pin": {
        "d": (3.0, 8.0, 0.5),
        "length": (10.0, 28.0, 1.0),
    },
    "demo_chamfer_prism": {
        "w": (16.0, 44.0, 2.0),
        "d": (12.0, 32.0, 2.0),
        "h": (6.0, 20.0, 1.0),
        "r": (2.0, 8.0, 0.5),
        "chamfer": (1.0, 4.0, 0.5),
    },
    "demo_threaded_rod": {
        "d": (5.0, 12.0, 0.5),
        "pitch": (0.75, 2.0, 0.1),
        "length": (12.0, 36.0, 2.0),
    },
    "demo_setscrew": {
        "body_w": (16.0, 32.0, 2.0),
        "body_d": (12.0, 24.0, 1.0),
        "body_h": (10.0, 20.0, 1.0),
    },
    "demo_slot_cutter": {
        "slot_len": (8.0, 22.0, 1.0),
        "slot_w": (2.0, 8.0, 0.5),
        "block_w": (16.0, 36.0, 2.0),
        "block_d": (10.0, 22.0, 1.0),
        "block_h": (3.0, 12.0, 1.0),
    },
    "demo_tapered_cavity": {
        "cavity_r": (4.0, 12.0, 0.5),
        "depth": (12.0, 30.0, 2.0),
        "taper_h": (6.0, 16.0, 1.0),
        "taper_step": (0.3, 1.2, 0.1),
        "body": (16.0, 32.0, 2.0),
    },
    "demo_u_channel_between": {
        "channel_w": (2.5, 7.0, 0.5),
        "wall": (0.8, 2.0, 0.2),
        "depth": (5.0, 14.0, 1.0),
        "body_size": (24.0, 48.0, 2.0),
        "body_r": (2.0, 8.0, 0.5),
    },
    "demo_revolved_gable_cavity": {
        "outer_r": (12.0, 28.0, 1.0),
        "outer_h": (12.0, 28.0, 1.0),
        "bore_r": (3.0, 10.0, 0.5),
        "cavity_r0": (5.0, 14.0, 0.5),
        "cavity_r1": (12.0, 24.0, 1.0),
        "sections": (24, 96, 8),
    },
    "demo_directed_holes": {
        "radius": (10.0, 22.0, 1.0),
        "n_holes": (4, 12, 1),
        "hole_r": (0.95, 3.5, 0.25),
        "hole_len": (14.0, 32.0, 2.0),
        "subdivisions": (2, 4, 1),
    },
    "demo_saddle": {
        "rib_t": (2.0, 8.0, 0.5),
        "shell": (0.9, 4.0, 0.5),
        "cyl_r": (4.0, 14.0, 0.5),
        "span": (8.0, 18.0, 1.0),
    },
    "demo_text_block": {
        "size": (5.0, 12.0, 0.5),
        "gap": (0.4, 2.0, 0.2),
        "plaque_w": (30.0, 60.0, 2.0),
        "plaque_d": (18.0, 40.0, 2.0),
        "text_h": (0.8, 2.5, 0.2),
    },
    "demo_seg_cylinder": {
        "r": (2.0, 8.0, 0.5),
        "x0": (-16.0, -4.0, 1.0),
        "y0": (-12.0, -2.0, 1.0),
        "z1": (8.0, 28.0, 1.0),
    },
    "demo_printed_worm": {
        # Range capped at 40 (native ~283ms at the default/max) rather than the
        # mechlib default of 160: sections=64 already runs ~448ms native and
        # sections=72 (the old demo default) ~512ms, well past the ~400ms
        # slider budget. The demo's own default was lowered from 72 to 40 to
        # match -- see demo_printed_worm() below.
        "sections": (24, 40, 8),
    },
    "demo_flat_worm_pair": {
        "gap": (0.05, 2.0, 0.1),
    },
    "demo_compliant_clutch": {
        "lock_face_frac": (0.2, 0.48, 0.02),
    },
    "demo_helix_tube": {
        "radius": (4.0, 12.0, 0.5),
        "wire_r": (0.55, 2.0, 0.1),
        "turns": (2.0, 8.0, 0.5),
        "z0": (-10.0, -2.0, 0.5),
        "z1": (2.0, 12.0, 0.5),
    },
    "demo_rack_2d": {
        "teeth": (4, 16, 1),
        "module": (1.0, 2.5, 0.25),
        "thickness": (2.0, 8.0, 0.5),
    },
    "demo_dog_slot_coupling": {
        "spacing": (10.0, 28.0, 1.0),
    },
    "demo_bearing_seat": {
        "housing_r": (12.0, 22.0, 0.5),
        "housing_h": (6.0, 16.0, 1.0),
    },
    "demo_four_bar": {
        "crank_angle_deg": (0.0, 360.0, 5.0),
        "l_crank": (5.5, 18.0, 1.0),
    },
    "demo_toggle_clamp": {
        # drive_deg is the motion phase (out-and-back through open ↔ lock);
        # overcenter_deg is how far past dead-center the locked pose sits.
        "drive_deg": (0.0, 360.0, 15.0),
        "overcenter_deg": (0.0, 12.0, 1.0),
    },
    "demo_scotch_yoke": {
        "angle_deg": (0.0, 360.0, 5.0),
    },
    "demo_quick_return": {
        "crank_angle_deg": (0.0, 360.0, 5.0),
    },
    "demo_plate_cam": {
        "lift": (3.0, 10.0, 1.0),
        "roller_r": (2.0, 5.0, 0.5),
        "thickness": (3.0, 8.0, 0.5),
        "follower_deg": (0.0, 330.0, 30.0),
    },
    "demo_snail_cam": {
        "lift": (4.0, 12.0, 1.0),
        "rise_deg": (280.0, 350.0, 10.0),
        "thickness": (3.0, 8.0, 0.5),
        "follower_deg": (10.0, 330.0, 30.0),
    },
    "demo_heart_cam": {
        "lift": (3.0, 9.0, 1.0),
        "thickness": (3.0, 8.0, 0.5),
        "follower_deg": (0.0, 330.0, 30.0),
    },
    "demo_barrel_cam": {
        "pin_phase_deg": (10.0, 330.0, 30.0),
        "groove_d": (1.5, 4.0, 0.5),
    },
    "demo_geneva_pair": {
        "crank_deg": (0.0, 360.0, 5.0),
        "slots": (3, 12, 1),
        "clearance": (0.1, 0.5, 0.05),
    },
    "demo_escapement": {
        # phase_deg: one pendulum period = two beats, two teeth advanced.
        "phase_deg": (0.0, 360.0, 5.0),
        "teeth": (20, 40, 2),
        "clearance": (0.15, 0.5, 0.05),
    },
    "demo_intermittent_gear_pair": {
        "module": (1.0, 2.5, 0.25),
        "clearance": (0.1, 0.5, 0.05),
    },
    "demo_herringbone_gear": {
        "drive_deg": (0.0, 360.0, 5.0),
        # z capped at 32 (not 36): z=36 at helix_deg=35 runs ~420-430ms
        # native (herringbone_gear() has no exposed tessellation knob to
        # cheapen), over the ~400ms slider budget. z=32 at helix_deg=35
        # measures ~370-380ms.
        "z": (12, 32, 2),
        "helix_deg": (10, 35, 5),
    },
    "demo_cycloidal_drive": {
        # One input turn per 360 deg of phase; full pose-close needs
        # (pins-1) turns (see ANIMATE). Slider covers one turn for inspection.
        "phase_deg": (0.0, 360.0, 5.0),
        "pins": (8, 14, 1),
        "explode": (0, 10, 1),
    },
    "demo_bevel_gear_pair": {
        "drive_deg": (0.0, 360.0, 5.0),
        # Old range (z1 10-24, z2 16-40) hit 2405ms native at its own max
        # with the mechlib default layers=10 -- a 7-12s Pyodide freeze. The
        # demo now passes layers=2 explicitly (see demo_bevel_gear_pair()):
        # bevel_gear_pair()'s loft rings scale linearly in radius, so two
        # rings already reproduce the exact ruled surface (volume differs
        # from layers=10 by <1e-8 relative, i.e. no visual change) at a
        # fraction of the cost. z1/z2 are also narrowed so the worst
        # reachable corner (z1=20, z2=32) stays ~370-400ms native.
        "z1": (10, 20, 2),
        "z2": (16, 32, 4),
    },
    "demo_scroll_drive": {
        "spiral_pitch": (4.75, 5.25, 0.25),
        "face_r": (8, 14, 1),
    },
    "demo_differential_screw": {
        "p1": (2.0, 3.0, 0.25),
        "p2": (1.25, 1.75, 0.25),
    },
    "demo_archimedes_screw": {
        "lead": (10, 20, 1),
        "turns": (2, 6, 1),
    },
    "demo_oldham_coupling": {
        "phase_deg": (0.0, 360.0, 5.0),
        "misalign": (0.0, 5.0, 0.5),
        "explode": (0.0, 12.0, 1.0),
        "clearance": (0.15, 0.4, 0.05),
        "sections": (24, 64, 8),
    },
    "demo_oldham_pose": {
        "phase_deg": (0.0, 360.0, 5.0),
        "misalign": (0.0, 8.0, 0.5),
        "explode": (0.0, 12.0, 1.0),
        "clearance": (0.15, 0.4, 0.05),
        "sections": (24, 64, 8),
    },
    "demo_universal_joint": {
        "phase_deg": (0.0, 360.0, 5.0),
        "bend_deg": (0.0, 35.0, 5.0),
        "clearance": (0.2, 0.5, 0.05),
        "sections": (24, 64, 8),
    },
    "demo_hooke_pose": {
        "phase_deg": (0.0, 360.0, 5.0),
        "bend_deg": (0.0, 45.0, 5.0),
        "clearance": (0.2, 0.5, 0.05),
        "sections": (24, 64, 8),
    },
    "demo_jaw_coupling": {
        "jaws": (3, 4, 1),
        "explode": (0.0, 12.0, 1.0),
        "clearance": (0.15, 0.4, 0.05),
        "sections": (24, 64, 8),
    },
    "demo_torque_limiter": {
        "detents": (4, 8, 1),
        "explode": (0.0, 14.0, 1.0),
        "sections": (24, 64, 8),
    },
    "demo_freewheel_clutch": {
        "rollers": (4, 7, 1),
        "sections": (24, 64, 8),
    },
    "demo_timing_pulley": {
        "teeth": (12, 64, 2),
        "bore_d": (3, 6, 1),
    },
    "demo_winch_drum": {
        "turns": (4, 16, 2),
        "cable_d": (2.0, 4.0, 0.5),
    },
    "demo_fusee": {
        "turns": (3, 12, 2),
        "radius_rise": (4.0, 12.0, 1.0),
    },
    "demo_cross_flexure": {
        "gap": (6, 16, 1),
        "blade_angle_deg": (20, 55, 5),
    },
    "demo_wave_spring": {
        "waves": (2, 6, 1),
        "turns": (1, 4, 1),
        "sections": (24, 96, 8),
    },
    "demo_bistable_beam": {
        "apex": (1.5, 3.5, 0.5),
        "beam_t": (0.6, 1.6, 0.2),
    },
    "demo_arc_ratchet": {
        "extrude_h": (1.5, 6.0, 0.5),
    },
    "demo_fastener_trio": {
        "size": (2.0, 5.0, 0.5),
        "length": (8.0, 20.0, 2.0),
        "spacing": (10.0, 20.0, 2.0),
        "washer_od": (6.0, 12.0, 1.0),
    },
    "demo_pip_ratchet": {
        "teeth": (5, 13, 2),
        "clearance": (0.1, 0.3, 0.05),
        "undercut_deg": (3.0, 11.0, 2.0),
    },
    "demo_planet_stage": {
        "sun_teeth": (9, 21, 3),
        "planet_teeth": (6, 12, 3),
        "module": (0.8, 1.4, 0.2),
        "face_width": (3.0, 8.0, 1.0),
    },
    "demo_spring_cartridge_ratchet": {
        "teeth": (8, 16, 2),
        "hook_deg": (3.0, 11.0, 2.0),
        "pawl_clear": (0.1, 0.3, 0.05),
        "ring_height": (2.0, 4.8, 0.4),
    },
    "demo_torsion_spring": {
        "mean_r": (4.8, 9.8, 1.0),
        "wire": (0.8, 2.0, 0.2),
        "turns": (3.0, 8.0, 1.0),
    },
    # --- gap-analysis wave v0.8.0 ---------------------------------------
    "demo_peaucellier": {
        "drive_deg": (0.0, 360.0, 15.0),
        "rhomb_len": (13.0, 18.0, 1.0),
    },
    "demo_watt": {
        "drive_deg": (0.0, 360.0, 15.0),
        "coupler_len": (16.0, 32.0, 2.0),
    },
    "demo_sarrus": {
        "drive_deg": (0.0, 360.0, 15.0),
        "bar_len": (14.0, 26.0, 2.0),
    },
    "demo_pantograph": {
        "crank_angle_deg": (0.0, 360.0, 15.0),
        "ratio": (1.5, 3.0, 0.25),
    },
    "demo_lazy_tongs": {
        "drive_deg": (0.0, 360.0, 15.0),
        "rhombs": (2, 6, 1),
    },
    "demo_iris_diaphragm": {
        "blades": (4, 12, 1),
        "close_pct": (0.0, 100.0, 5.0),
        "blade_t": (0.8, 2.0, 0.2),
    },
    "demo_collet_chuck": {
        "bore_d": (3.0, 12.0, 1.0),
        "slots": (2, 8, 2),
        "taper_deg": (4.0, 12.0, 2.0),
    },
    "demo_eccentric_cam_clamp": {
        "handle_deg": (0.0, 360.0, 15.0),
        "ecc": (2.0, 6.0, 0.5),
        "cam_r": (12.0, 20.0, 1.0),
    },
    "demo_ring_gear_mesh": {
        "z": (28, 48, 2),
        "m": (1.0, 2.5, 0.25),
        "width": (4.0, 12.0, 1.0),
    },
    "demo_tripod_cv_joint": {
        "angle_deg": (0.0, 24.0, 3.0),
        "phase_deg": (0.0, 360.0, 15.0),
        "clear": (0.2, 0.5, 0.05),
        "sections": (24, 64, 8),
    },
    "demo_double_cardan_joint": {
        "bend_deg": (0.0, 30.0, 5.0),
        "inter_len": (36.0, 56.0, 5.0),
        "clearance": (0.2, 0.5, 0.05),
        "sections": (24, 64, 8),
    },
    "demo_ball_socket_joint": {
        "ball_d": (8, 16, 1),
        "capture_deg": (10, 35, 5),
        "fingers": (3, 6, 1),
        "pose_deg": (0, 30, 5),
    },
    "demo_knuckle_hinge": {
        "knuckles": (3, 9, 2),
        "stop_deg": (60, 150, 10),
        "open_deg": (90, 180, 10),
        "leaf_len": (10, 26, 2),
    },
    "demo_gimbal_rings": {
        "rings": (2, 3, 1),
        "outer_d": (40, 70, 2),
        "ring_t": (6.0, 10.0, 0.5),
        "tilt_deg": (0, 60, 5),
    },
    "demo_gerotor_pump": {
        "rotor_deg": (0.0, 360.0, 5.0),
        "lobes": (4, 9, 1),
        "ecc": (1.1, 1.9, 0.1),
        "rotor_h": (4.0, 14.0, 1.0),
    },
    "demo_hose_barb": {
        "tube_id": (4.0, 14.0, 0.5),
        "barbs": (1, 6, 1),
        "interference": (0.2, 1.2, 0.1),
        "ramp_deg": (10.0, 45.0, 5.0),
    },
    "demo_rotary_spool_valve": {
        "plug_deg": (0.0, 360.0, 5.0),
        "ports": (2, 6, 1),
        "body_d": (26.0, 48.0, 2.0),
        "plug_d": (10.0, 22.0, 1.0),
    },
    "demo_peristaltic_pump_head": {
        "rollers": (2, 6, 1),
        "tube_od": (4.0, 10.0, 0.5),
        "occlusion": (0.5, 1.0, 0.05),
        "wrap_deg": (200.0, 320.0, 10.0),
    },
    "demo_linear_way": {
        "profile_index": (0, 2, 1),
        "angle_deg": (45.0, 70.0, 5.0),
        "clear": (0.15, 0.5, 0.05),
        "section_w": (18.0, 36.0, 2.0),
    },
    "demo_telescoping_stage": {
        "sections": (2, 4, 1),
        "extend": (0.0, 1.0, 0.1),
        "clear": (0.2, 0.5, 0.05),
        "length": (30.0, 90.0, 10.0),
    },
    "demo_idler_pulley": {
        "od": (10, 24, 2),
        "crown": (0.0, 0.6, 0.05),
    },
    "demo_eccentric_idler_mount": {
        "eccentricity": (0.5, 2.5, 0.5),
        "rotation_deg": (0, 360, 30),
    },
    "demo_belt_tensioner": {
        "preload_mm": (0.5, 4.0, 0.5),
        "sweep_deg": (20, 120, 10),
    },
    "demo_chain_reverse": {
        "wrap_deg": (120, 240, 10),
        "idler_teeth": (8, 16, 2),
    },
    "demo_chain_s_wrap": {
        "out_wrap_deg": (60, 180, 30),
        "out_teeth": (8, 16, 2),
    },
    "demo_chain_dual_output": {
        "in_pitches": (3, 6, 1),
        "idler_teeth": (8, 16, 2),
    },
    "demo_drag_chain_link": {
        "bend_deg": (15, 45, 5),
        "width": (8, 12, 1),
    },
    "demo_drag_chain": {
        "links": (6, 10, 1),
        "bend_deg": (15, 45, 5),
        "s_bend_at": (0, 8, 1),
    },
    "demo_roller_chain_link": {
        "roller_d": (7.75, 11.75, 1.0),
        "pitch": (12.7, 25.4, 6.35),
    },
    "demo_roller_chain": {
        "n_teeth": (8, 20, 2),
        "wrap_deg": (100, 300, 50),
    },
    "demo_belleville_washer": {
        "free_h": (1.2, 4.0, 0.2),
        "stack": (1, 5, 1),
        "inner_d": (6.0, 14.0, 1.0),
    },
    "demo_coil_spring": {
        "turns": (3.0, 10.0, 1.0),
        "pitch": (3.0, 8.0, 0.5),
        "wire_d": (1.2, 2.4, 0.2),
    },
    "demo_spiral_power_spring": {
        "turns": (3.0, 8.0, 1.0),
        "gap": (0.4, 0.8, 0.1),
        "strip_t": (0.8, 1.2, 0.2),
    },
    "demo_leaf_spring": {
        "leaves": (3, 6, 1),
        "leaf_t": (2.0, 3.0, 0.2),
        "camber": (5.0, 14.0, 1.0),
    },
    "demo_flexure_stage": {
        "travel": (1.0, 3.0, 0.5),
        "blade_len": (25.0, 45.0, 5.0),
        "blade_t": (0.8, 1.2, 0.2),
    },
    "demo_plain_bushing": {
        "bore_d": (4, 20, 1),
        "length": (6, 30, 1),
        "relief_grooves": (0, 6, 1),
    },
    "demo_thrust_washer": {
        "outer_d": (20, 36, 2),
        "balls": (4, 8, 1),
        "thickness": (1.6, 4.0, 0.4),
    },
    "demo_printed_ball_bearing": {
        "outer_d": (30, 44, 2),
        "balls": (3, 6, 1),
        "width": (10, 18, 1),
    },
    "demo_kinematic_coupling": {
        "pcd": (24.0, 64.0, 2.0),
        "ball_d": (4.0, 10.0, 1.0),
        "plate_t": (4.0, 10.0, 0.5),
        "kind_index": (0, 1, 1),
    },
    "demo_repeatable_dock": {
        "pcd": (30.0, 56.0, 2.0),
        "magnet_d": (6.0, 16.0, 1.0),
        "plate_t": (5.0, 9.0, 0.5),
        "preload_index": (0, 1, 1),
    },
    "demo_three_point_leveller": {
        "screw_pcd": (30.0, 60.0, 2.0),
        "screw_index": (0, 3, 1),
        "lift": (4.0, 12.0, 1.0),
        "kind_index": (0, 1, 1),
    },
    "demo_oring_groove": {
        "face_pcd": (24.0, 50.0, 2.0),
        "cs": (1.78, 3.53, 0.28),
        "squeeze": (0.10, 0.30, 0.02),
        "block": (58.0, 78.0, 2.0),
    },
    "demo_labyrinth_seal": {
        "shaft_d": (4.0, 12.0, 1.0),
        "teeth": (2, 6, 1),
        "tooth_t": (0.8, 2.0, 0.2),
        "gap": (0.2, 0.4, 0.05),
    },
    "demo_gasket_channel": {
        "width": (1.5, 5.0, 0.5),
        "depth": (0.9, 2.7, 0.3),
        "lid_w": (40.0, 70.0, 5.0),
        "lid_d": (28.0, 50.0, 4.0),
    },
    "demo_auxetic_panel": {
        "mode_index": (0, 2, 1),
        "cell": (8.0, 16.0, 2.0),
        "strut_t": (0.4, 1.2, 0.4),
    },
    "demo_honeycomb_panel": {
        "cell": (8.0, 16.0, 2.0),
        "strut_t": (0.4, 1.2, 0.4),
    },
    "demo_isogrid_panel": {
        "cell": (8.0, 16.0, 2.0),
        "strut_t": (0.4, 1.2, 0.4),
    },
    "demo_kerf_bend_cutter": {
        "mode_index": (0, 2, 1),
        "kerf": (0.4, 0.8, 0.1),
        "pitch": (4.0, 10.0, 1.0),
    },
    # ----- gap-analysis wave v0.9.0 -----
    "demo_slider_crank": {
        "crank_angle_deg": (0.0, 360.0, 5.0),
        "crank_r": (8.0, 14.0, 1.0),
    },
    "demo_chebyshev": {
        "drive_deg": (0.0, 360.0, 15.0),
        "unit": (8.0, 14.0, 1.0),
    },
    "demo_scott_russell": {
        "drive_deg": (0.0, 360.0, 15.0),
        "half_len": (14.0, 28.0, 2.0),
    },
    "demo_bell_crank": {
        "pose_deg": (0.0, 360.0, 5.0),
        "angle_deg": (60.0, 120.0, 5.0),
    },
    "demo_face_cam": {
        "pin_phase_deg": (0.0, 360.0, 5.0),
        "lift": (3.0, 10.0, 1.0),
    },
    "demo_swash_plate": {
        "phase_deg": (0.0, 360.0, 5.0),
        "tilt_deg": (8.0, 22.0, 1.0),
    },
    "demo_screw_jack": {
        # drive_deg maps through a cosine onto lift 0..1..0 so one turn closes.
        "drive_deg": (0.0, 360.0, 15.0),
    },
    "demo_rack_pinion": {
        # drive_deg maps through a sine onto ±2 teeth of travel.
        "drive_deg": (0.0, 360.0, 15.0),
    },
    "demo_dog_clutch": {
        # drive_deg maps through a cosine onto engage 0..1..0 so one turn closes.
        "drive_deg": (0.0, 360.0, 15.0),
        "dogs": (3, 6, 1),
    },
    "demo_hirth_coupling": {
        "teeth": (8, 18, 1),
    },
    "demo_clevis": {
        "gap": (6.0, 10.0, 1.0),
    },
    "demo_external_gear_pump": {
        "phase_deg": (0.0, 360.0, 5.0),
        "teeth": (10, 16, 1),
    },
    "demo_harmonic_drive": {
        "phase_deg": (0.0, 360.0, 5.0),
    },
}


def _cut_block(cutter, extents=(24.0, 18.0, 12.0)):
    """Subtract a centered cutter from a display block."""
    return sub(boxc(extents), cutter)


def _shell_box(extents, wall=2.0):
    """Create an open-top rectangular shell centered in XY with its floor at Z zero."""
    w, d, h = extents
    outer = rbox((w, d, h), center=(0, 0, h / 2.0), r=3.0)
    inner = boxc((w - 2 * wall, d - 2 * wall, h), center=(0, 0, wall + h / 2.0))
    return sub(outer, inner)


# ---------------------------------------------------------------------------
# Primitives
# ---------------------------------------------------------------------------


def demo_cyl(r=8, h=20, sections=96) -> MeshList:
    # Keep default types identical to the original gallery call (int r/h) so
    # the GLB stays byte-identical; float kwargs still work for playground.
    kwargs = {}
    if sections != 96:
        kwargs["sections"] = sections
    return [("cylinder", cyl(r=r, h=h, **kwargs), PALETTE[0])]


def demo_boxc(w=24, d=16, h=10) -> MeshList:
    return [("centered_box", boxc((w, d, h)), PALETTE[1])]


def demo_rbox(w=24, d=16, h=10, r=4) -> MeshList:
    return [("rounded_box", rbox((w, d, h), r=r), PALETTE[2])]


def demo_frustum(r0=6, r1=10, h=16, sections=96) -> MeshList:
    kwargs = {}
    if sections != 96:
        kwargs["sections"] = sections
    return [("frustum", frustum(r0, r1, h, **kwargs), PALETTE[3])]


def demo_sector2d(
    a0_deg: float = -30.0,
    a1_deg: float = 210.0,
    radius: float = 18.0,
    extrude_h: float = 4.0,
    n: int = 48,
) -> MeshList:
    mesh = trimesh.creation.extrude_polygon(
        sector2d(a0_deg, a1_deg, radius, n=n), extrude_h
    )
    return [("extruded_sector", mesh, PALETTE[4])]


def demo_hex_poly(af: float = 16.0, extrude_h: float = 4.0) -> MeshList:
    mesh = trimesh.creation.extrude_polygon(hex_poly(af=af), extrude_h)
    return [("extruded_hexagon", mesh, PALETTE[5])]


def demo_extrude_twist(
    base_r: float = 7.5,
    lobe_amp: float = 2.0,
    lobe_count: int = 3,
    height: float = 30.0,
    turns_deg: float = 360.0,
    count: int = 96,
    z_samples: int = 81,
) -> MeshList:
    angles = np.linspace(0.0, 2.0 * math.pi, count, endpoint=False)
    radii = base_r + lobe_amp * np.cos(lobe_count * angles)
    profile = [(r * math.cos(a), r * math.sin(a)) for r, a in zip(radii, angles)]
    heights = np.linspace(0.0, height, z_samples)
    deg_per_mm = turns_deg / height if height else 0.0
    mesh = extrude_twist(profile, None, heights, lambda z, d=deg_per_mm: d * z)
    return [("three_lobed_twist", mesh, PALETTE[6])]


def demo_swept_keyed_bore(
    radius: float = 7.0,
    flat_x: float = 5.2,
    free_angle: float = 50.0,
    extrude_h: float = 4.0,
    spacing: float = 22.0,
    resolution: int = 48,
) -> MeshList:
    keyed = Point(0.0, 0.0).buffer(radius, resolution=resolution).intersection(
        box(-radius - 0.1, -radius - 0.1, flat_x, radius + 0.1)
    )
    swept = swept_keyed_bore(keyed, free_angle)
    input_mesh = trimesh.creation.extrude_polygon(keyed, extrude_h)
    swept_mesh = trimesh.creation.extrude_polygon(swept, extrude_h)
    swept_mesh.apply_translation((spacing, 0.0, 0.0))
    return [
        ("input_keyed_bore", input_mesh, PALETTE[7]),
        ("swept_envelope", swept_mesh, PALETTE[8]),
    ]


def demo_spur_gear_pair(
    n_driver: int = 18,
    n_driven: int = 28,
    module: float = 1.5,
    thickness: float = 5.0,
    backlash: float = 0.35,
    pa: float = 20.0,
    drive_deg: float = 0.0,
) -> MeshList:
    driver_poly = spur_gear_2d(N=n_driver, m=module, pa=pa, bl=backlash)
    driven_poly = spur_gear_2d(N=n_driven, m=module, pa=pa, bl=backlash)
    phase = mesh_phase(n_driver, n_driven, 0.0)
    center_distance = module * (n_driver + n_driven) / 2.0
    driven_poly = affinity.rotate(driven_poly, phase, origin=(0.0, 0.0))
    driven_poly = affinity.translate(driven_poly, xoff=center_distance)

    driver = trimesh.creation.extrude_polygon(driver_poly, thickness)
    driven = trimesh.creation.extrude_polygon(driven_poly, thickness)
    # Conjugate motion: the driven gear counter-rotates at the exact tooth
    # ratio about its own centre, so the mesh phase established above holds at
    # every drive angle. The assertion below is the standing proof of that --
    # flip the sign here and the gears interpenetrate by tens of mm^3.
    driver = _spin(driver, drive_deg)
    driven = _spin(driven, -drive_deg * n_driver / float(n_driven),
                   center=(center_distance, 0.0, 0.0))
    overlap = trimesh.boolean.intersection([driver, driven], engine="manifold")
    overlap_volume = (
        0.0 if overlap is None or len(overlap.faces) == 0 else abs(float(overlap.volume))
    )
    assert overlap_volume < 0.01, "gear overlap is %.6f mm^3" % overlap_volume
    print("gear intersection volume: %.6f mm^3" % overlap_volume)
    return [
        ("18_tooth_driver", driver, PALETTE[9]),
        ("28_tooth_driven", driven, PALETTE[10]),
    ]


def demo_board_cradle(
    board_w: float = 40.0,
    board_d: float = 30.0,
    board_h: float = 8.0,
    board_t: float = 1.6,
    fl: float = 0.0,
    standoff: float = 4.0,
) -> MeshList:
    # board_cradle(rect, fl, ...) with rect = (cx, cy, w, d, h)
    rect = (-board_w / 2.0, -board_d / 2.0, board_w, board_d, board_h)
    cradle = board_cradle(rect, fl=fl, standoff=standoff)
    # Original: rect = (-20, -15, 40, 30, 8); board z = 4.8 = standoff + board_t/2? 
    # standoff default 4.0, board center z = 4.8 for 1.6 thick -> 4.0 + 0.8
    board = boxc((board_w, board_d, board_t), center=(0.0, 0.0, standoff + board_t / 2.0))
    return [
        ("corner_cradles", cradle, PALETTE[11]),
        ("board_reference", board, PALETTE[12]),
    ]


# ---------------------------------------------------------------------------
# Cutters
# ---------------------------------------------------------------------------


def demo_teardrop(
    r: float = 4.0,
    length: float = 26.0,
    block_w: float = 24.0,
    block_d: float = 18.0,
    block_h: float = 16.0,
) -> MeshList:
    tear = teardrop(r, length, axis="x", up=(0, 0, 1))
    tear_cut = _cut_block(tear, (block_w, block_d, block_h))
    return [("cut_block", tear_cut, PALETTE[0])]


def demo_ss_bore(
    R: float = 5.0,
    Robj: float = 4.5,
    length: float = 26.0,
    split_z: float = 12.0,
    block_w: float = 24.0,
    block_d: float = 20.0,
    block_h: float = 20.0,
) -> MeshList:
    support_bore = ss_bore(R, Robj, length, (0, 0, 0), axis="x", split_z=split_z)
    support_cut = _cut_block(support_bore, (block_w, block_d, block_h))
    return [("housing_cut", support_cut, PALETTE[1])]


def demo_dbore(
    shaft_d: float = 5.5,
    flat: float = 3.7,
    hub_r: float = 7.0,
    hub_h: float = 10.0,
    clear: float = 0.1,
    spacing: float = 10.0,
) -> MeshList:
    socket_blank = cyl(hub_r, hub_h)
    socket = sub(socket_blank, dbore(shaft_d, flat, hub_h + 2, clear=clear))
    socket.apply_translation((-spacing, 0.0, 0.0))
    hub = dbore_hub(hub_r, hub_h, shaft_d=shaft_d, flat=flat, clear=clear)
    hub.apply_translation((spacing, 0.0, 0.0))
    return [
        ("double_d_socket", socket, PALETTE[2]),
        ("double_d_hub", hub, PALETTE[3]),
    ]


def demo_counterbore(
    through_d: float = 3.4,
    cb_d: float = 7.0,
    cb_h: float = 3.2,
    length: float = 16.0,
    block: float = 18.0,
) -> MeshList:
    cb = counterbore(through_d, cb_d, cb_h, length)
    cb_cut = _cut_block(cb, (block, block, length))
    return [("counterbored_block", cb_cut, PALETTE[7])]


def demo_bearing_seat(
    housing_r: float = 15.0,
    housing_h: float = 10.0,
    open_column: bool = False,
) -> MeshList:
    seat = bearing_seat("608", open_column=open_column)
    housing = cyl(housing_r, housing_h)
    housing.apply_translation((0, 0, housing_h / 2.0))
    housing = sub(housing, seat)
    half = boxc((40, 20, 20), (0, -10, housing_h / 2.0))
    housing = sub(housing, half)
    bearing = washer_mesh(22, 8, 7)
    bearing.apply_translation((0, 0, 1.2))
    return [
        ("seat_cutaway", housing, PALETTE[8]),
        ("608_reference", bearing, PALETTE[12]),
    ]


def demo_crush_ribs(
    comp_w: float = 18.0,
    comp_d: float = 12.0,
    comp_h: float = 16.0,
    rib_t: float = 0.6,
    count: int = 3,
    interference: float = 0.12,
    rib_h: float = 10.0,
    rib_depth: float = 6.0,
) -> MeshList:
    # crush_ribs((18, 12, 16), 0.6, 6, 10, count=3, interference=0.12)
    ribs = crush_ribs(
        (comp_w, comp_d, comp_h), rib_t, rib_depth, rib_h,
        count=count, interference=interference,
    )
    component = boxc((comp_w, comp_d, comp_h))
    return [
        ("tapered_ribs", ribs, PALETTE[5]),
        ("component_reference", component, PALETTE[11]),
    ]


# ---------------------------------------------------------------------------
# Closures
# ---------------------------------------------------------------------------


def demo_press_lid(
    box_w: float = 34.0,
    box_d: float = 28.0,
    box_h: float = 14.0,
    wall: float = 2.0,
    lid_lift: float = 23.0,
    lid_clear_x: float = 30.0,
    lid_clear_y: float = 24.0,
) -> MeshList:
    base = _shell_box((box_w, box_d, box_h), wall=wall)
    lid = press_lid(box_w, box_d, lid_clear_x, lid_clear_y, (0, 0))
    lid.apply_translation((0, 0, lid_lift))
    return [
        ("open_box", base, PALETTE[1]),
        ("exploded_lid", lid, PALETTE[0]),
    ]


def demo_clamshell_shiplap(
    w: float = 34.0,
    d: float = 28.0,
    h: float = 14.0,
    spacing: float = 22.0,
) -> MeshList:
    outer = boxc((w, d, h))
    lip, slot = clamshell_shiplap(outer)
    lip.apply_translation((-spacing, 0, 0))
    slot.apply_translation((spacing, 0, 8))
    return [
        ("base_lip", lip, PALETTE[9]),
        ("lid_slot", slot, PALETTE[10]),
    ]


def demo_ydovetail(
    tongue_y0: float = -8.0,
    tongue_y1: float = 8.0,
    groove_y0: float = -11.0,
    groove_y1: float = 11.0,
    clear: float = 0.25,
    receiver_w: float = 16.0,
    receiver_d: float = 20.0,
    receiver_h: float = 10.0,
    tongue_x: float = 0.0,
    groove_x: float = 9.0,
) -> MeshList:
    tongue = ydovetail(tongue_x, tongue_y0, tongue_y1)
    tongue.apply_translation((-9, 0, 0))
    groove = ydovetail(groove_x, groove_y0, groove_y1, clear=clear)
    receiver = sub(boxc((receiver_w, receiver_d, receiver_h), (9, 0, 14)), groove.copy())
    return [
        ("dovetail_tongue", tongue, PALETTE[4]),
        ("dovetail_receiver", receiver, PALETTE[6]),
    ]


def demo_snap_pair(
    depth: float = 10.0,
    finger_dx: float = 8.0,
    finger_dz: float = 2.0,
) -> MeshList:
    catch = snap_catch("x", 0, 0, 1, depth)
    finger = snap_finger("x", 0, 0, 1, depth)
    finger.apply_translation((finger_dx, 0, finger_dz))
    return [
        ("snap_catch", catch, PALETTE[5]),
        ("snap_finger", finger, PALETTE[8]),
    ]


def demo_nut_slot(
    length: float = 16.0,
    nib: int = 1,
    nut_af: float = 5.5,
    nut_h: float = 2.6,
    nut_id: float = 3.0,
) -> MeshList:
    trap = nut_slot((0, 0, 0), length=length, nib=bool(nib))
    trap_block = sub(boxc((14, 20, 8), (0, 5, 0)), trap)
    trap_block = sub(trap_block, boxc((20, 20, 10), (10, 5, 0)))
    nut = hex_nut_mesh(nut_af, nut_h, nut_id)
    nut.apply_translation((0, 0, -1.3))
    return [
        ("trap_cutaway", trap_block, PALETTE[2]),
        ("nut_standin", nut, PALETTE[10]),
    ]


def demo_pins_and_posts(
    post_h=10,
    pin_r=2,
    pin_h=7,
    socket_r=2.25,
    socket_depth=5,
    bore_r=1.7,
) -> MeshList:
    # Defaults keep the original int literals so GLB bytes match HEAD.
    post = screw_post((-10, 0, 0), (0, 0, 1), post_h)
    bore_h = post_h + 2
    bore_z = post_h // 2 if isinstance(post_h, int) else post_h / 2.0
    post = sub(post, cyl(bore_r, bore_h, center=(-10, 0, bore_z)))
    pin = fix_pin(pin_r, pin_h, (0, 0, 1), (8, 0, 0))
    socket_block = boxc((12, 12, 6), (8, 0, -3))
    socket = blind_socket(socket_r, socket_depth, (0, 0, 1), (8, 0, 0))
    socket_block = sub(socket_block, socket)
    return [
        ("screw_post", post, PALETTE[3]),
        ("fix_pin", pin, PALETTE[5]),
        ("socket_block", socket_block, PALETTE[7]),
    ]


# ---------------------------------------------------------------------------
# Gears / mechanisms / patterns / text / fasteners
# ---------------------------------------------------------------------------


def demo_spur_gear_mesh(
    teeth: int = 20,
    module: float = 1.5,
    thickness: float = 6.0,
    bore_d: float = 5.0,
    drive_deg: float = 0.0,
) -> MeshList:
    gear = spur_gear_mesh(teeth, module, thickness, bore_d=bore_d)
    return [("20_tooth_gear", _spin(gear, drive_deg), PALETTE[9])]


def demo_roller_sprocket(
    teeth: int = 14,
    pitch: float = 10.0,
    pin_d: float = 2.0,
    clear: float = 0.275,
    outer_d: float = 47.3,
    thickness: float = 6.0,
    pin_r: float = 1.0,
    pin_h: float = 10.0,
) -> MeshList:
    # Keep outer_d=47.3 at the stock 14-tooth demo; auto-size when the fixed
    # blank is too small for the pin envelope (PLAY extremes).
    rp = pitch / (2 * math.sin(math.pi / teeth))
    env_r = pin_d / 2.0 + clear
    od = outer_d if outer_d > 2.0 * (rp - env_r) else None
    sprocket_poly = roller_sprocket_2d(
        teeth, pitch, pin_d, clear=clear, outer_d=od
    )
    sprocket = trimesh.creation.extrude_polygon(sprocket_poly, thickness)
    pins = []
    for x, y in polar_ring(teeth, rp):
        p = cyl(pin_r, pin_h, center=(x, y, thickness / 2.0))
        pins.append(p)
    return [
        ("roller_sprocket", sprocket, PALETTE[11]),
        ("pitch_circle_pins", trimesh.util.concatenate(pins), PALETTE[4]),
    ]


def demo_thread(
    d: float = 8.0,
    length: float = 16.0,
    head_r: float = 7.0,
    head_h: float = 4.0,
    nut_af: float = 13.0,
    nut_h: float = 7.0,
    spacing: float = 11.0,
) -> MeshList:
    # seg default inside thread_solid is >= 96 for GLB fidelity
    bolt = thread_solid(d, length)
    head = cyl(head_r, head_h)
    head.apply_translation((0, 0, -head_h / 2.0))
    bolt = uni([head, bolt])
    bolt.apply_translation((-spacing, 0, 0))

    nut_blank = cyl(nut_af / math.sqrt(3), nut_h, sections=6)
    nut_blank.apply_translation((0, 0, nut_h / 2.0))
    nut = tap(nut_blank, d, (0, 0, 0), nut_h)
    nut = sub(nut, boxc((18, 18, 12), (9, 0, nut_h / 2.0)))
    nut.apply_translation((spacing, 0, 0))
    return [
        ("m8_bolt", bolt, PALETTE[12]),
        ("tapped_nut_cutaway", nut, PALETTE[10]),
    ]


def demo_knurl(r: float = 8.0, h: float = 7.0, n: int = 20) -> MeshList:
    knob = cyl(r, h)
    knob.apply_translation((0, 0, h / 2.0))
    knob = knurl(knob, r, 0, h, n=n)
    return [("knurled_knob", knob, PALETTE[6])]


def demo_torsion_spring(
    mean_r: float = 6.8, wire: float = 1.2, turns: float = 5.0
) -> MeshList:
    spring = torsion_spring_mesh(mean_r=mean_r, wire=wire, turns=turns)
    return [("torsion_spring", spring, PALETTE[12])]


def demo_lighten_grid(
    panel_w: float = 44.0,
    panel_d: float = 30.0,
    panel_t: float = 3.0,
    cell: float = 5.0,
    wall: float = 2.0,
    margin_x: float = 18.0,
    margin_y: float = 11.0,
) -> MeshList:
    panel = boxc((panel_w, panel_d, panel_t), (0, 0, panel_t / 2.0))
    windows = []
    for cx, cy in lighten_grid_centres(
        -margin_x, -margin_y, margin_x, margin_y, cell, wall, "hex"
    ):
        poly = lighten_cell_poly(cx, cy, cell, "hex")
        cut = trimesh.creation.extrude_polygon(poly, panel_t + 2)
        cut.apply_translation((0, 0, -1))
        windows.append(cut)
    panel = sub(panel, uni(windows))
    return [("lightened_panel", panel, PALETTE[2])]


def demo_text_polygon(
    text: str = "mechlib",
    size: float = 12.0,
    extrude_h: float = 1.5,
    plaque_w: float = 52.0,
    plaque_d: float = 18.0,
    plaque_t: float = 3.0,
    plaque_r: float = 3.0,
    text_x: float = -23.0,
    text_y: float = -4.5,
) -> MeshList:
    letters = text_polygon(text, size)
    text_mesh = mechlib.extrude_poly_z(letters, 0.0, extrude_h)
    text_mesh.apply_translation((text_x, text_y, plaque_t))
    plaque = rbox((plaque_w, plaque_d, plaque_t), center=(0, 0, plaque_t / 2.0), r=plaque_r)
    return [
        ("plaque", plaque, PALETTE[7]),
        ("mechlib_text", text_mesh, PALETTE[0]),
    ]


def demo_fastener_trio(
    size: float = 3.0,
    length: float = 14.0,
    spacing: float = 14.0,
    nut_af: float = 5.5,
    nut_h: float = 2.6,
    nut_id: float = 3.0,
    washer_od: float = 8.0,
    washer_id: float = 3.4,
    washer_t: float = 1.0,
) -> MeshList:
    screws = []
    for x, style in zip((-spacing, 0.0, spacing), ("pan", "shcs", "csk")):
        screws.append(fastener_mesh(size, length, style=style, at=(x, 0, 0)))
    nut = hex_nut_mesh(nut_af, nut_h, nut_id)
    nut.apply_translation((-5, 10, 0))
    washer = washer_mesh(washer_od, washer_id, washer_t)
    washer.apply_translation((5, 10, 0))
    names = ("pan_screw", "shcs_screw", "csk_screw", "hex_nut", "washer")
    colors = (PALETTE[5], PALETTE[9], PALETTE[10], PALETTE[3], PALETTE[12])
    parts = screws + [nut, washer]
    return list(zip(names, parts, colors))


def demo_find_vitamin(spacing: float = 10.0) -> MeshList:
    queries = ("608", "695", "M3 SHCS", "GA12-N20")
    colors = (PALETTE[7], PALETTE[0], PALETTE[9], PALETTE[5])
    out = []
    cursor = None
    for i, query in enumerate(queries):
        hits = find_vitamin(query)
        if not hits:
            raise ValueError("find_vitamin(%r) returned no hits" % query)
        hit = hits[0]
        mesh = hit.envelope()
        name = hit.address.replace("/", "_")
        if cursor is not None:
            mesh.apply_translation((cursor - float(mesh.bounds[0][0]), 0.0, 0.0))
        cursor = float(mesh.bounds[1][0]) + spacing
        out.append((name, mesh, colors[i]))
    return out


def demo_vitamin(spacing: float = 10.0) -> MeshList:
    addresses = (
        "bearing/608-2rs",
        "cell/18650",
        "motor/nema17",
        "servo/sg90",
    )
    colors = (PALETTE[1], PALETTE[4], PALETTE[6], PALETTE[10])
    out = []
    cursor = None
    for i, address in enumerate(addresses):
        mesh = vitamin(address).envelope()
        name = address.replace("/", "_")
        if cursor is not None:
            mesh.apply_translation((cursor - float(mesh.bounds[0][0]), 0.0, 0.0))
        cursor = float(mesh.bounds[1][0]) + spacing
        out.append((name, mesh, colors[i]))
    return out


def demo_vitamin_addresses(spacing: float = 10.0) -> MeshList:
    first_of_family = {}
    for address in vitamin_addresses():
        family = address.split("/", 1)[0]
        if family not in first_of_family:
            first_of_family[family] = address
    out = []
    cursor = None
    for i, address in enumerate(first_of_family.values()):
        mesh = vitamin(address).envelope()
        name = address.replace("/", "_")
        if cursor is not None:
            mesh.apply_translation((cursor - float(mesh.bounds[0][0]), 0.0, 0.0))
        cursor = float(mesh.bounds[1][0]) + spacing
        out.append((name, mesh, PALETTE[i % len(PALETTE)]))
    return out


def demo_worm(
    module: float = 1.5,
    worm_length: float = 24.0,
    pitch_d: float = 14.3,
    wheel_teeth: int = 40,
    wheel_thickness: float = 8.0,
    backlash: float = 0.35,
    phase_deg: float = 3.0,
) -> MeshList:
    worm_mesh, lead_angle = worm(module, worm_length, pitch_d)
    worm_mesh.apply_transform(
        trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0])
    )
    worm_mesh.apply_translation(
        [0, 0, -(module * wheel_teeth + pitch_d) / 2]
    )
    wheel = spur_gear(
        module, wheel_teeth, wheel_thickness, backlash=backlash, helix_deg=lead_angle
    )
    wheel.apply_transform(
        trimesh.transformations.rotation_matrix(math.radians(phase_deg), [0, 0, 1])
    )
    wheel.apply_transform(
        trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0])
    )
    return [
        ("worm", worm_mesh, PALETTE[10]),
        ("helical_wheel", wheel, PALETTE[5]),
    ]


def demo_spur_gear_sector(
    module: float = 1.5,
    teeth: int = 36,
    thickness: float = 7.0,
    bore: float = 5.0,
    sector_deg: float = 125.0,
    hub_d: float = 14.0,
    full_disc: bool = False,
) -> MeshList:
    sector_gear = spur_gear(
        module, teeth, thickness, bore=bore, sector_deg=sector_deg,
        hub_d=hub_d, full_disc=full_disc,
    )
    return [("sector_gear", sector_gear, PALETTE[9])]


def demo_loft(
    r0: float = 9.0,
    r1: float = 8.0,
    r_mid: float = 6.5,
    r3: float = 7.5,
    r_top: float = 5.0,
    height: float = 30.0,
    ring_n: int = 64,
    resolution: int = 24,
) -> MeshList:
    # Original z heights: 0, 8, 15, 22, 30; scale middle rings with height
    z1 = height * 8.0 / 30.0
    z2 = height * 15.0 / 30.0
    z3 = height * 22.0 / 30.0
    specs = [
        (Point(0, 0).buffer(r0, resolution=resolution), 0),
        (Point(1, 0.5).buffer(r1, resolution=resolution), z1),
        (Point(0.5, 1).buffer(r_mid, resolution=resolution), z2),
        (Point(-0.5, 0.5).buffer(r3, resolution=resolution), z3),
        (Point(0, 0).buffer(r_top, resolution=resolution), height),
    ]
    mesh = loft([ring_pts(poly, ring_n, z) for poly, z in specs])
    return [("organic_loft", mesh, PALETTE[6])]


def demo_push_pin(d: float = 5.0, length: float = 18.0) -> MeshList:
    pin = push_pin(d, length)
    return [("barbed_push_pin", pin, PALETTE[4])]


def demo_chamfer_prism(
    w: float = 30.0,
    d: float = 22.0,
    h: float = 12.0,
    r: float = 5.0,
    chamfer: float = 2.0,
) -> MeshList:
    return [("chamfered_prism", chamfer_prism(w, d, h, r, chamfer), PALETTE[1])]


def demo_threaded_rod(
    d: float = 8.0, pitch: float = 1.25, length: float = 20.0
) -> MeshList:
    return [("m8_threaded_rod", threaded_rod(d, pitch, length), PALETTE[12])]


def demo_setscrew(
    body_w: float = 24.0,
    body_d: float = 18.0,
    body_h: float = 14.0,
) -> MeshList:
    body = boxc((body_w, body_d, body_h), (0, 0, body_h / 2.0))
    boss, hole = setscrew((0, -body_d / 2.0, body_h / 2.0), (0, 1, 0))
    feature = sub(uni([body, boss]), hole)
    cutaway = sub(feature, boxc((30, 28, 24), (15, -4, body_h / 2.0)))
    return [("setscrew_cutaway", cutaway, PALETTE[3])]


def demo_slot_cutter(
    slot_len: float = 14.0,
    slot_w: float = 4.0,
    z0: float = -1.0,
    z1: float = 7.0,
    block_w: float = 24.0,
    block_d: float = 14.0,
    block_h: float = 6.0,
) -> MeshList:
    block = boxc((block_w, block_d, block_h), (0, 0, block_h / 2.0))
    cutters = slot_cutter(slot_len, slot_w, z0, z1)
    return [("dogbone_slot", sub(block, uni(cutters)), PALETTE[7])]


def demo_tapered_cavity(
    cavity_r: float = 7.0,
    depth: float = 22.0,
    floor: float = 2.0,
    taper_h: float = 11.0,
    taper_step: float = 0.6,
    body: float = 22.0,
    body_h: float = 28.0,
    resolution: int = 32,
    cut_w: float = 30.0,
    cut_d: float = 14.0,
    cut_h: float = 32.0,
    cut_cy: float = -11.0,
) -> MeshList:
    poly = Point(0, 0).buffer(cavity_r, resolution=resolution)
    cavity = uni(tapered_cavity(poly, floor, depth, taper_h=taper_h, taper_step=taper_step))
    body_mesh = boxc((body, body, body_h), (0, 0, body_h / 2.0))
    hollow = sub(body_mesh, cavity)
    cutaway = sub(hollow, boxc((cut_w, cut_d, cut_h), (0, cut_cy, body_h / 2.0)))
    return [("tapered_cutaway", cutaway, PALETTE[2])]


def demo_u_channel_between(
    channel_w: float = 4.0,
    wall: float = 1.2,
    depth: float = 9.0,
    body_size: float = 34.0,
    body_r: float = 4.0,
) -> MeshList:
    points = [(-13, -11), (-4, -11), (7, -7), (7, -2),
              (-7, 2), (-7, 7), (4, 11), (13, 11)]
    cutters = []
    for p0, p1 in zip(points[:-1], points[1:]):
        cutters.extend(u_channel_between(p0, p1, channel_w, wall, depth))
    body = rbox((body_size, body_size, depth), center=(0, 0, depth / 2.0), r=body_r)
    return [("open_u_run", sub(body, uni(cutters)), PALETTE[11])]


def demo_revolved_gable_cavity(
    outer_r: float = 20.0,
    outer_h: float = 18.0,
    bore_r: float = 6.0,
    cavity_r0: float = 8.0,
    cavity_r1: float = 18.0,
    gable_h: float = 2.0,
    gable_w: float = 14.0,
    sections: int = 96,
    cut_w: float = 44.0,
    cut_d: float = 22.0,
    cut_h: float = 24.0,
    cut_cy: float = -11.0,
) -> MeshList:
    outer = cyl(outer_r, outer_h)
    outer.apply_translation((0, 0, outer_h / 2.0))
    bore = cyl(bore_r, outer_h + 2)
    bore.apply_translation((0, 0, outer_h / 2.0))
    shell = sub(outer, bore)
    shell = sub(
        shell,
        revolved_gable_cavity(cavity_r0, cavity_r1, gable_h, gable_w, sections=sections),
    )
    cutaway = sub(
        shell, boxc((cut_w, cut_d, cut_h), (0, cut_cy, outer_h / 2.0))
    )
    return [("gable_cavity_cutaway", cutaway, PALETTE[8])]


def demo_directed_holes(
    radius: float = 16.0,
    n_holes: int = 8,
    hole_r: float = 2.2,
    hole_len: float = 24.0,
    subdivisions: int = 4,
    ray_scale: float = 18.0,
    clip_w: float = 40.0,
    clip_h: float = 20.0,
) -> MeshList:
    sphere = trimesh.creation.icosphere(subdivisions=subdivisions, radius=radius)
    upper = trimesh.boolean.intersection(
        [sphere, boxc((clip_w, clip_w, clip_h), (0, 0, clip_h / 2.0))],
        engine="manifold",
    )
    angles = np.linspace(0, 2 * math.pi, n_holes, endpoint=False)
    rays = [np.asarray((math.cos(a), math.sin(a), 1.0), float) for a in angles]
    rays = [ray / np.linalg.norm(ray) for ray in rays]
    points = [tuple(ray_scale * ray) for ray in rays]
    vectors = [tuple(-ray) for ray in rays]
    bores = directed_holes(points, vectors, hole_r, hole_len)
    return [("perforated_dome", sub(upper, bores), PALETTE[0])]


def demo_saddle(
    span: float = 12.0,
    y_span: float = 8.0,
    z0: float = 7.0,
    z1: float = 11.0,
    rib_t: float = 4.0,
    shell: float = 2.4,
    cyl_r: float = 8.0,
    interior_w: float = 34.0,
    interior_d: float = 16.0,
    interior_h: float = 20.0,
    interior_cz: float = 7.0,
) -> MeshList:
    interior = boxc((interior_w, interior_d, interior_h), (0, 0, interior_cz))
    rib = saddle((-span, -y_span, z0), (span, y_span, z1), rib_t, 0, shell, interior)
    reference = seg_cylinder((-span, -y_span, z0), (span, y_span, z1), cyl_r)
    return [
        ("saddle_rib", rib, PALETTE[1]),
        ("cylinder_reference", reference, PALETTE[12]),
    ]


def demo_text_block(
    size: float = 7.0,
    gap: float = 1.0,
    plaque_w: float = 44.0,
    plaque_d: float = 28.0,
    plaque_t: float = 3.0,
    plaque_r: float = 4.0,
    text_z0: float = 3.0,
    text_h: float = 1.2,
    line0: str = "MECH",
    line1: str = "LIB",
) -> MeshList:
    plaque = rbox((plaque_w, plaque_d, plaque_t), center=(0, 0, plaque_t / 2.0), r=plaque_r)
    text_meshes = []
    for poly in text_block([line0, line1], 0, 0, size, gap=gap):
        mesh = mechlib.extrude_poly_z(poly, text_z0, text_z0 + text_h)
        text_meshes.append(mesh)
    return [
        ("plaque", plaque, PALETTE[7]),
        ("stacked_text", trimesh.util.concatenate(text_meshes), PALETTE[0]),
    ]


def demo_seg_cylinder(
    r: float = 4.0,
    x0: float = -9.0,
    y0: float = -6.0,
    z0: float = 0.0,
    x1: float = 10.0,
    y1: float = 7.0,
    z1: float = 18.0,
) -> MeshList:
    return [
        ("skew_segment", seg_cylinder((x0, y0, z0), (x1, y1, z1), r), PALETTE[10])
    ]


# ---------------------------------------------------------------------------
# Drives / ratchets
# ---------------------------------------------------------------------------


def demo_printed_worm(sections: int = 40) -> MeshList:
    # Default lowered from 72 (native ~512ms) to 40 (~283ms) to fit the
    # slider budget; see the PLAY["demo_printed_worm"] comment. This does
    # change the committed gallery GLB's thread smoothness slightly.
    return [("journalled_worm", printed_worm(sections=sections), PALETTE[0])]


def demo_flat_worm_pair(gap: float = 0.25) -> MeshList:
    input_worm = flat_worm()
    wheel_band = worm_wheel_band()
    length = float(input_worm.bounds[1, 2] - input_worm.bounds[0, 2])
    input_worm.apply_transform(
        trimesh.transformations.rotation_matrix(math.pi / 2.0, [0, 1, 0])
    )
    input_worm.apply_translation((-length / 2.0, -24.0 - gap, 1.75))
    wheel_band.apply_transform(
        trimesh.transformations.rotation_matrix(math.radians(11.30), [0, 0, 1])
    )
    return [
        ("flat_input_worm", input_worm, PALETTE[1]),
        ("helical_wheel_band", wheel_band, PALETTE[5]),
    ]


def demo_worm_coupon() -> MeshList:
    coupon = worm_coupon()
    coupon["wheel_band"].apply_translation((30.0, 0.0, 0.0))
    return [
        ("coupon_worm", coupon["worm"], PALETTE[2]),
        ("coupon_wheel_band", coupon["wheel_band"], PALETTE[9]),
    ]


def demo_planet_stage(
    sun_teeth: int = 12,
    planet_teeth: int = 9,
    module: float = 1.0,
    face_width: float = 5.0,
) -> MeshList:
    # ring_teeth, ring_outer_d, ring_face_width, and carrier_radius are all
    # derived (not independent PLAY params) so every slider combination
    # satisfies planet_stage()'s validated relations: ring_teeth must equal
    # sun_teeth + 2*planet_teeth, and the fixed ring needs enough rim past
    # the ring gear's addendum circle. The formulas reproduce the function's
    # own defaults exactly at sun_teeth=12, planet_teeth=9, module=1.0,
    # face_width=5.0 (ring_outer_d=34.5, ring_face_width=5.725,
    # carrier_radius=13.5), so the default GLB is unchanged.
    ring_teeth = sun_teeth + 2 * planet_teeth
    ring_clearance = 0.25
    ring_outer_d = module * (ring_teeth + 2) + 2 * ring_clearance + 2.0
    ring_face_width = face_width + 0.725
    center_distance = (sun_teeth + planet_teeth) * module / 2.0
    carrier_radius = center_distance + 3.0
    planetary = planet_stage(
        module=module, sun_teeth=sun_teeth, planet_teeth=planet_teeth,
        ring_teeth=ring_teeth, face_width=face_width,
        ring_outer_d=ring_outer_d, ring_face_width=ring_face_width,
        carrier_radius=carrier_radius,
    )
    meshes: MeshList = [
        ("sun", planetary["sun"], PALETTE[5]),
        ("ring", planetary["ring"], PALETTE[0]),
        ("carrier", planetary["carrier"], PALETTE[7]),
    ]
    for index, mesh in enumerate(planetary["planets"]):
        meshes.append(("planet_%d" % index, mesh, PALETTE[2 + index]))
    return meshes


def demo_pip_ratchet(
    teeth: int = 9, clearance: float = 0.15, undercut_deg: float = 7.0
) -> MeshList:
    return [
        ("undercut_ring", ratchet_ring(
            teeth=teeth, clearance=clearance, undercut_deg=undercut_deg), PALETTE[2]),
        ("accordion_hub", pip_ratchet_hub(
            teeth=teeth, clearance=clearance, undercut_deg=undercut_deg), PALETTE[3]),
    ]


def demo_spring_cartridge_ratchet(
    teeth: int = 12,
    hook_deg: float = 7.0,
    pawl_clear: float = 0.15,
    ring_height: float = 3.2,
) -> MeshList:
    cartridge_ring, cartridge_hub, cartridge_pawls = spring_cartridge_ratchet(
        teeth=teeth, hook_deg=hook_deg, pawl_clear=pawl_clear,
        ring_height=ring_height,
    )
    meshes: MeshList = [
        ("cartridge_ring", cartridge_ring, PALETTE[2]),
        ("cartridge_hub", cartridge_hub, PALETTE[7]),
    ]
    for index, pawl in enumerate(cartridge_pawls):
        meshes.append(("pawl_%d" % index, pawl, PALETTE[10]))
    return meshes


def demo_compliant_clutch(lock_face_frac: float = 0.34) -> MeshList:
    clutch_race, clutch_hub = compliant_clutch(lock_face_frac=lock_face_frac)
    return [
        ("clutch_race", clutch_race, PALETTE[1]),
        ("flexure_hub", clutch_hub, PALETTE[6]),
    ]


def demo_arc_ratchet(extrude_h: float = 3.0) -> MeshList:
    arc_ring_2d, arc_hub_2d = arc_ratchet_2d()
    arc_ring = trimesh.creation.extrude_polygon(arc_ring_2d, extrude_h)
    arc_hub = trimesh.creation.extrude_polygon(arc_hub_2d, extrude_h)
    return [
        ("arc_ring", arc_ring, PALETTE[4]),
        ("arc_flexure_hub", arc_hub, PALETTE[11]),
    ]


def demo_helix_tube(
    radius: float = 7.0,
    wire_r: float = 1.15,
    turns: float = 5.0,
    z0: float = -5.0,
    z1: float = 5.0,
) -> MeshList:
    # N=150 (mechlib default 420): helix_tube()'s face-list construction is
    # a pure-Python double loop over N*M points, ~450ms native at N=420
    # regardless of radius/wire_r/turns. N=150 cuts that to ~150-170ms with
    # only a ~0.7% volume change (slightly more faceted coil, same shape).
    coil = helix_tube(radius, wire_r, turns, z0, z1, N=150)
    return [("helical_tube", coil, PALETTE[12])]


def demo_rack_2d(
    teeth: int = 8, module: float = 1.5, thickness: float = 4.0
) -> MeshList:
    rack = trimesh.creation.extrude_polygon(rack_2d(teeth, module), thickness)
    return [("eight_tooth_rack", rack, PALETTE[5])]


def demo_dog_slot_coupling(spacing: float = 17.0) -> MeshList:
    coupling_boss, coupling_collar = dog_slot_coupling()
    coupling_boss.apply_translation((-spacing, 0.0, 0.0))
    coupling_collar.apply_translation((spacing, 0.0, 0.0))
    return [
        ("slotted_boss", coupling_boss, PALETTE[1]),
        ("dog_collar", coupling_collar, PALETTE[9]),
    ]


# ---------------------------------------------------------------------------
# Linkages (mechanical-movements wave v0.6.0)
# ---------------------------------------------------------------------------


def demo_four_bar(crank_angle_deg: float = 60.0, l_crank: float = 12.5):
    parts = four_bar(l_ground=25.0, l_crank=l_crank, l_coupler=25.0,
                     l_rocker=25.0, crank_angle_deg=crank_angle_deg,
                     coupler_ext=25.0)
    entries = [
        ("ground_link", parts["ground"], PALETTE[7]),
        ("rocker", parts["rocker"], PALETTE[2]),
        ("coupler", parts["coupler"], PALETTE[5]),
        ("crank", parts["crank"], PALETTE[4]),
        ("trace_point", parts["trace"], PALETTE[10]),
    ]
    for index, pin in enumerate(parts["pins"]):
        entries.append(("pin_%d" % index, pin, PALETTE[11]))
    return entries


# Handle travel for the gallery cycle: open (~-20 deg) through dead center to
# the design overcenter (positive). A sine of drive_deg maps one full turn onto
# that out-and-back swing — same pattern as peaucellier/watt/lazy_tongs.
TOGGLE_OPEN_DEG = -20.0


def demo_toggle_clamp(drive_deg: float = 0.0, overcenter_deg: float = 4.0):
    # Cosine of drive_deg: 0 deg = fully open, 180 deg = locked overcenter,
    # 360 deg = open again. Default drive_deg=0 keeps the GLB at the open pose.
    mid = 0.5 * (TOGGLE_OPEN_DEG + overcenter_deg)
    swing = 0.5 * (overcenter_deg - TOGGLE_OPEN_DEG)
    oc = mid - swing * math.cos(math.radians(drive_deg))
    parts = toggle_clamp(overcenter_deg=oc)
    return [
        ("base", parts["base"], PALETTE[7]),
        ("clamp_arm", parts["arm"], PALETTE[1]),
        ("connecting_link", parts["link"], PALETTE[5]),
        ("handle", parts["handle"], PALETTE[4]),
        ("pin_arm", parts["pins"][0], PALETTE[11]),
        ("pin_handle", parts["pins"][1], PALETTE[11]),
        ("pin_knee", parts["pins"][2], PALETTE[11]),
        ("pin_joint", parts["pins"][3], PALETTE[11]),
    ]


def demo_scotch_yoke(angle_deg: float = 35.0):
    parts = scotch_yoke(angle_deg=angle_deg)
    return [
        ("crank_disc", parts["crank_disc"], PALETTE[1]),
        ("crank_pin", parts["crank_pin"], PALETTE[5]),
        ("slotted_yoke", parts["yoke"], PALETTE[0]),
        ("rail_a", parts["rail_a"], PALETTE[7]),
        ("rail_b", parts["rail_b"], PALETTE[7]),
    ]


def demo_quick_return(crank_angle_deg: float = 40.0):
    parts = quick_return(crank_angle_deg=crank_angle_deg)
    return [
        ("base", parts["base"], PALETTE[7]),
        ("crank_disc", parts["crank_disc"], PALETTE[1]),
        ("crank_pin", parts["crank_pin"], PALETTE[5]),
        ("slotted_lever", parts["lever"], PALETTE[2]),
        ("pin_crank", parts["pins"][0], PALETTE[11]),
        ("pin_lever", parts["pins"][1], PALETTE[11]),
    ]


# ---------------------------------------------------------------------------
# Cams (mechanical-movements wave v0.6.0)
# ---------------------------------------------------------------------------


def _roller_follower(pitch_r, angle_deg, roller_r, thickness, stem_len=14.0,
                     stem_w=6.0):
    """Roller plus stem posed radially, roller center at ``pitch_r`` (mm)."""
    angle = math.radians(angle_deg)
    ca, sa = math.cos(angle), math.sin(angle)
    roller = cyl(roller_r, thickness,
                 center=(pitch_r * ca, pitch_r * sa, thickness / 2.0),
                 sections=64)
    stem_center = pitch_r + stem_len / 2.0 + roller_r * 0.3
    stem = boxc((stem_len, stem_w, thickness * 0.6),
                center=(0.0, 0.0, thickness / 2.0))
    stem.apply_translation((stem_center, 0.0, 0.0))
    stem.apply_transform(trimesh.transformations.rotation_matrix(angle, [0, 0, 1]))
    return roller, stem


def demo_plate_cam(lift: float = 8.0, roller_r: float = 3.0,
                   thickness: float = 5.0, follower_deg: float = 60.0,
                   base_r: float = 10.0) -> MeshList:
    segments = (
        ("cycloidal", lift, 120.0),
        ("dwell", 0.0, 60.0),
        ("cycloidal", -lift, 120.0),
        ("dwell", 0.0, 60.0),
    )
    cam = plate_cam(base_r=base_r, segments=segments, thickness=thickness,
                    roller_r=roller_r, hub_d=14.0, hub_h=3.0, bore_d=6.0,
                    flat=2.3)
    pitch_r = base_r + cam_lift(segments, follower_deg)
    roller, stem = _roller_follower(pitch_r, follower_deg, roller_r, thickness)
    return [
        ("plate_cam", cam, PALETTE[0]),
        ("roller", roller, PALETTE[1]),
        ("follower_stem", stem, PALETTE[2]),
    ]


def demo_snail_cam(lift: float = 9.0, rise_deg: float = 320.0,
                   thickness: float = 5.0, follower_deg: float = 250.0,
                   base_r: float = 10.0) -> MeshList:
    cam = snail_cam(base_r=base_r, lift=lift, thickness=thickness,
                    rise_deg=rise_deg, bore_d=6.0, flat=2.3)
    roller_r = 2.5
    u = min(max(follower_deg / rise_deg, 0.0), 1.0)
    pitch_r = base_r + lift * u + roller_r
    roller, stem = _roller_follower(pitch_r, follower_deg, roller_r, thickness)
    return [
        ("snail_cam", cam, PALETTE[5]),
        ("roller", roller, PALETTE[1]),
        ("follower_stem", stem, PALETTE[2]),
    ]


def demo_heart_cam(lift: float = 6.0, thickness: float = 5.0,
                   follower_deg: float = 90.0, base_r: float = 10.0) -> MeshList:
    cam = heart_cam(base_r=base_r, lift=lift, thickness=thickness,
                    bore_d=6.0, flat=2.3)
    roller_r = 2.5
    segments = (("linear", lift, 180.0), ("linear", -lift, 180.0))
    pitch_r = base_r + cam_lift(segments, follower_deg) + roller_r
    roller, stem = _roller_follower(pitch_r, follower_deg, roller_r, thickness)
    return [
        ("heart_cam", cam, PALETTE[4]),
        ("roller", roller, PALETTE[1]),
        ("follower_stem", stem, PALETTE[2]),
    ]


def demo_barrel_cam(pin_phase_deg: float = 40.0, groove_d: float = 3.0,
                    radius: float = 11.0, length: float = 28.0) -> MeshList:
    parts = barrel_cam(radius=radius, length=length, groove_w=4.25,
                       groove_d=groove_d,
                       segments=(("cycloidal", 10.0, 180.0),
                                 ("cycloidal", -10.0, 180.0)),
                       pin_d=4.0, pin_len=12.0,
                       pin_phase_deg=pin_phase_deg,
                       bore_d=6.0, flat=2.3)
    pin = parts["pin"]
    z_pin = pin.centroid[2]
    angle = math.radians(pin_phase_deg)
    block = boxc((8.0, 12.0, 9.0), center=(radius + 6.5, 0.0, z_pin))
    hole = cyl(2.15, 12.0, center=(radius + 6.5, 0.0, z_pin), axis="x",
               sections=48)
    guide = sub(block, hole)
    guide.apply_transform(trimesh.transformations.rotation_matrix(angle, [0, 0, 1]))
    return [
        ("barrel_cam", parts["barrel"], PALETTE[5]),
        ("follower_pin", pin, PALETTE[1]),
        ("pin_guide", guide, PALETTE[7]),
    ]


# ---------------------------------------------------------------------------
# Indexing (mechanical-movements wave v0.6.0)
# ---------------------------------------------------------------------------


def demo_geneva_pair(slots: int = 6, clearance: float = 0.25,
                     crank_deg: float = 0.0) -> MeshList:
    # Deep-slot counts need a long crank so the slot bottoms clear the hub;
    # high counts keep crank 10 so the rim stays clear of the driver web even
    # at the loosest clearance.
    crank_r = {3: 28.0, 4: 19.0}.get(slots, 10.0)
    pair = geneva_pair(slots=slots, crank_r=crank_r, clearance=clearance)
    driver, wheel = pair["driver"], pair["wheel"]
    if crank_deg:
        # mechlib's own crank-to-wheel relation, the same one geneva_pair
        # builds the crescent cutout around. The travel across one engagement
        # gives the index step including its sign, so nothing about the
        # direction or the 360/slots pitch is restated here.
        raw = geneva_wheel_angle(slots, crank_r)
        zero = raw(0.0)

        def travel(theta):
            return ((raw(theta) - zero + 180.0) % 360.0) - 180.0

        theta_e = wheel.metadata["engagement_angle_deg"] / 2.0
        index_step = travel(theta_e) - travel(-theta_e)
        # Split the crank angle into completed indexes plus the position
        # inside the current engagement window; the wheel dwells whenever the
        # pin is out of a slot, which is what makes this part worth printing.
        local = ((crank_deg + 180.0) % 360.0) - 180.0
        turns = round((crank_deg - local) / 360.0)
        centre = (float(wheel.metadata["center_distance"]), 0.0, 0.0)
        wheel = _spin(wheel, turns * index_step + travel(local), center=centre)
        driver = _spin(driver, crank_deg)
    return [
        ("pin_driver", driver, PALETTE[1]),
        ("slotted_wheel", wheel, PALETTE[0]),
    ]


def demo_escapement(phase_deg: float = 0.0, teeth: int = 30,
                    clearance: float = 0.25) -> MeshList:
    # phase_deg drives escapement_pose: anchor swings once, wheel steps two
    # teeth (one per beat). Default 0 keeps the GLB at the locked rest pose.
    pair = escapement(teeth=int(teeth), style="anchor", clearance=clearance,
                      phase_deg=phase_deg)
    return [
        ("escape_wheel", pair["wheel"], PALETTE[5]),
        ("anchor", pair["anchor"], PALETTE[4]),
    ]


def demo_intermittent_gear_pair(module: float = 1.5,
                                clearance: float = 0.25) -> MeshList:
    pair = intermittent_gear_pair(module=module, clearance=clearance)
    return [
        ("mutilated_driver", pair["driver"], PALETTE[7]),
        ("notched_driven", pair["driven"], PALETTE[9]),
    ]


# ---------------------------------------------------------------------------
# Wave gears and linear drives (mechanical-movements wave v0.6.0)
# ---------------------------------------------------------------------------


def demo_herringbone_gear(m: float = 1.5, z: int = 24, helix_deg: float = 25.0,
                          h: float = 10.0, drive_deg: float = 0.0) -> MeshList:
    """Meshed herringbone pair: mirrored chevrons, phased with mesh_phase.

    The mid-plane is the untwisted meshing plane, so spur phasing applies;
    the driven gear must carry the opposite hand or the helices cross.
    """
    driver = herringbone_gear(m=m, z=z, h=h, helix_deg=helix_deg,
                              bore_d=5.0, hand=1)
    driven = herringbone_gear(m=m, z=z, h=h, helix_deg=helix_deg,
                              bore_d=5.0, hand=-1)
    driven.apply_transform(trimesh.transformations.rotation_matrix(
        math.radians(mesh_phase(z, z, 0.0)), (0.0, 0.0, 1.0)))
    driven.apply_translation((m * z, 0.0, 0.0))
    # Equal tooth counts, so the driven gear counter-rotates 1:1.
    return [
        ("herringbone_driver", _spin(driver, drive_deg), PALETTE[0]),
        ("herringbone_driven", _spin(driven, -drive_deg,
                                     center=(m * z, 0.0, 0.0)), PALETTE[1]),
    ]


def demo_cycloidal_drive(phase_deg: float = 0.0, pins: int = 12,
                         explode: float = 0.0) -> MeshList:
    """Cycloidal reducer; default assembled so ANIMATE can orbit the disc.

    ``phase_deg`` is the input angle (``cycloidal_pose``). ``explode`` lifts
    parts along Z for inspection only — leave at 0 for motion.
    """
    drive = cycloidal_drive(pins=int(pins), phase_deg=phase_deg)
    if explode:
        drive["input"].apply_translation((0.0, 0.0, -explode))
        drive["disc"].apply_translation((0.0, 0.0, explode))
        drive["output"].apply_translation((0.0, 0.0, 2.0 * explode))
    return [
        ("housing_ring", drive["housing"], PALETTE[7]),
        ("cycloidal_disc", drive["disc"], PALETTE[4]),
        ("eccentric_input", drive["input"], PALETTE[5]),
        ("output_plate", drive["output"], PALETTE[2]),
    ]


def demo_bevel_gear_pair(m: float = 1.5, z1: int = 16, z2: int = 24,
                         drive_deg: float = 0.0) -> MeshList:
    """Straight bevel pair posed meshed on perpendicular axes.

    layers=2 is passed explicitly (mechlib default is 10): the Tredgold
    loft's rings scale linearly in radius between the two cone-distance
    endpoints, so the ruled surface is already exact with only two rings --
    more layers add colinear vertices with no shape change (volume differs
    by <1e-8 relative) but cost roughly linearly more compute. This alone
    cuts native render time from ~930ms to ~250ms at the default z1/z2 with
    no visual difference.
    """
    pair = bevel_gear_pair(m=m, z1=z1, z2=z2, bore1_d=4.0, bore2_d=5.0, layers=2)
    # Pinion axis is +Z and gear axis +Y (bevel_gear_pair poses them so); the
    # gear turns at the inverse tooth ratio in the opposite sense. The sense
    # is not a guess: the opposite sign drives the teeth 14 mm^3 into each
    # other, this one keeps the worst-case overlap at 0.09 mm^3 (the residue
    # of the Tredgold approximation itself).
    return [
        ("bevel_pinion", _spin(pair["pinion"], drive_deg), PALETTE[0]),
        ("bevel_gear", _spin(pair["gear"], -drive_deg * z1 / float(z2),
                             axis=(0.0, 1.0, 0.0)), PALETTE[9]),
    ]


def demo_scroll_drive(spiral_pitch: float = 5.0,
                      face_r: float = 10.0) -> MeshList:
    """Chuck scroll plate with three jaws posed self-centering."""
    drive = scroll_drive(spiral_pitch=spiral_pitch, face_r=face_r)
    meshes: MeshList = [("scroll_plate", drive["scroll"], PALETTE[7])]
    for index, jaw in enumerate(drive["jaws"]):
        meshes.append(("scroll_jaw_%d" % index, jaw, PALETTE[1 + index]))
    return meshes


def demo_differential_screw(p1: float = 2.0, p2: float = 1.75) -> MeshList:
    """Differential screw with both nuts engaged on their thread sections."""
    screw = differential_screw(p1=p1, p2=p2)
    return [
        ("twin_pitch_shaft", screw["shaft"], PALETTE[5]),
        ("frame_nut", screw["nut_frame"], PALETTE[0]),
        ("moving_nut", screw["nut_moving"], PALETTE[3]),
    ]


def demo_archimedes_screw(lead: float = 14.0, turns: float = 4.0) -> MeshList:
    """Inclined water screw: helical flight inside a half-pipe trough."""
    assembly = archimedes_screw(lead=lead, turns=turns)
    return [
        ("screw_flight", assembly["screw"], PALETTE[0]),
        ("half_pipe_trough", assembly["trough"], PALETTE[6]),
    ]


# ---------------------------------------------------------------------------
# Couplings and clutches (mechanical-movements wave v0.6.0)
# ---------------------------------------------------------------------------


def demo_oldham_coupling(
    phase_deg: float = 0.0,
    misalign: float = 3.0,
    explode: float = 6.0,
    clearance: float = 0.25,
    sections: int = 64,
) -> MeshList:
    parts = oldham_coupling(clearance=clearance, sections=sections)
    pose = oldham_pose(phase_deg=phase_deg, offset=misalign)
    hub_a = _spin(parts["hub_a"], pose["hub_a_deg"])
    hub_a.apply_translation((0.0, 0.0, -explode))
    disc = _spin(parts["disc"], pose["disc_deg"])
    disc.apply_translation((pose["disc_xy"][0], pose["disc_xy"][1], 0.0))
    # Hub B's tongue runs along Y, so its free sliding direction is Y.
    hub_b = parts["hub_b"].copy()
    hub_b.apply_translation((0.0, misalign, explode))
    hub_b = _spin(hub_b, pose["hub_b_deg"], center=(0.0, misalign, 0.0))
    return [
        ("tongue_hub_a", hub_a, PALETTE[0]),
        ("cross_slotted_disc", disc, PALETTE[5]),
        ("tongue_hub_b", hub_b, PALETTE[1]),
    ]


def demo_oldham_pose(
    phase_deg: float = 0.0,
    misalign: float = 6.0,
    explode: float = 0.0,
    clearance: float = 0.25,
    sections: int = 64,
) -> MeshList:
    """Four Oldham poses so the 2-omega disc orbit is visible on the static GLB.

    Copies at input 0/45/90/135 deg on a large parallel-shaft offset: the
    floating disc centre walks a circle of radius ``|offset| / 2`` at twice
    shaft speed. Assembled (explode=0) so the orbit is readable without the
    playground.
    """
    parts = oldham_coupling(clearance=clearance, sections=sections)
    spacing = 56.0
    phases = (0.0, 45.0, 90.0, 135.0)
    out: MeshList = []
    for i, phi0 in enumerate(phases):
        phase = phi0 + float(phase_deg)
        pose = oldham_pose(phase_deg=phase, offset=misalign)
        hub_a = _spin(parts["hub_a"], pose["hub_a_deg"])
        hub_a.apply_translation((0.0, 0.0, -explode))
        disc = _spin(parts["disc"], pose["disc_deg"])
        disc.apply_translation((pose["disc_xy"][0], pose["disc_xy"][1], 0.0))
        hub_b = parts["hub_b"].copy()
        hub_b.apply_translation((0.0, misalign, explode))
        hub_b = _spin(hub_b, pose["hub_b_deg"], center=(0.0, misalign, 0.0))
        dx = (i - 1.5) * spacing
        shift = (dx, 0.0, 0.0)
        for mesh in (hub_a, disc, hub_b):
            mesh.apply_translation(shift)
        tag = "phi%d" % int(round(phi0))
        out.extend([
            ("tongue_hub_a_%s" % tag, hub_a, PALETTE[0]),
            ("cross_slotted_disc_%s" % tag, disc, PALETTE[5]),
            ("tongue_hub_b_%s" % tag, hub_b, PALETTE[1]),
        ])
    return out


def demo_universal_joint(
    phase_deg: float = 0.0,
    bend_deg: float = 20.0,
    clearance: float = 0.3,
    sections: int = 64,
) -> MeshList:
    parts = universal_joint(
        bend_deg=bend_deg, clearance=clearance, sections=sections)
    pose = hooke_pose(bend_deg=bend_deg, phase_deg=phase_deg)
    yoke_a = _spin(parts["yoke_a"], pose["input_deg"])
    spider = parts["spider"].copy()
    if phase_deg:
        spider.apply_transform(_hooke_spider_matrix(bend_deg, phase_deg))
    yoke_b = parts["yoke_b"].copy()
    if phase_deg:
        yoke_b.apply_transform(trimesh.transformations.rotation_matrix(
            math.radians(pose["output_deg"]), pose["output_axis"]))
    return [
        ("fork_yoke_input", yoke_a, PALETTE[7]),
        ("cross_spider", spider, PALETTE[5]),
        ("fork_yoke_output", yoke_b, PALETTE[3]),
    ]


def demo_hooke_pose(
    phase_deg: float = 0.0,
    bend_deg: float = 45.0,
    clearance: float = 0.3,
    sections: int = 64,
) -> MeshList:
    """Four Cardan poses so lag/lead is visible on the static GLB.

    Copies at input 0/45/90/135 deg on a large legal bend: equal at 0 and
    90, lag near 45, lead near 135. Index flags on each shaft (cyan input,
    orange output) track ``hooke_pose`` angles so the twice-per-rev
    mismatch is readable without the playground.
    """
    parts = universal_joint(
        bend_deg=bend_deg, clearance=clearance, sections=sections)
    # Default universal_joint shaft station from the pin (fork_len + web +
    # half shaft): 15 + 2 + 6 = 23 mm. Flags sit on that station, outside
    # the shaft radius, so they do not boolean against the yoke.
    z_shaft = 23.0
    beta = math.radians(bend_deg)
    flag_len = 18.0
    flag_ext = (flag_len, 2.0, 3.0)
    shaft_r = 5.0
    flag_x = shaft_r + 0.4 + flag_len / 2.0
    spacing = 78.0
    phases = (0.0, 45.0, 90.0, 135.0)
    out: MeshList = []
    for i, phi0 in enumerate(phases):
        phase = phi0 + float(phase_deg)
        pose = hooke_pose(bend_deg=bend_deg, phase_deg=phase)
        yoke_a = _spin(parts["yoke_a"], pose["input_deg"])
        spider = parts["spider"].copy()
        yoke_b = parts["yoke_b"].copy()
        if phase:
            spider.apply_transform(_hooke_spider_matrix(bend_deg, phase))
            yoke_b.apply_transform(trimesh.transformations.rotation_matrix(
                math.radians(pose["output_deg"]), pose["output_axis"]))
        flag_in = boxc(flag_ext, (flag_x, 0.0, -z_shaft))
        flag_in = _spin(flag_in, pose["input_deg"])
        # Output shaft rest is +Z rotated by β about +X; then spin with ψ.
        flag_out = boxc(flag_ext, (flag_x, 0.0, z_shaft))
        flag_out.apply_transform(trimesh.transformations.rotation_matrix(
            beta, (1.0, 0.0, 0.0)))
        if phase:
            flag_out.apply_transform(trimesh.transformations.rotation_matrix(
                math.radians(pose["output_deg"]), pose["output_axis"]))
        dx = (i - 1.5) * spacing
        shift = (dx, 0.0, 0.0)
        for mesh in (yoke_a, spider, yoke_b, flag_in, flag_out):
            mesh.apply_translation(shift)
        tag = "phi%d" % int(round(phi0))
        out.extend([
            ("fork_yoke_input_%s" % tag, yoke_a, PALETTE[7]),
            ("cross_spider_%s" % tag, spider, PALETTE[5]),
            ("fork_yoke_output_%s" % tag, yoke_b, PALETTE[3]),
            ("input_index_%s" % tag, flag_in, PALETTE[0]),
            ("output_index_%s" % tag, flag_out, PALETTE[1]),
        ])
    return out


def demo_jaw_coupling(
    jaws: int = 3,
    explode: float = 6.0,
    clearance: float = 0.25,
    sections: int = 64,
) -> MeshList:
    parts = jaw_coupling(jaws=jaws, clearance=clearance, sections=sections)
    hub_a = parts["hub_a"]
    hub_a.apply_translation((0.0, 0.0, -explode))
    hub_b = parts["hub_b"]
    hub_b.apply_translation((0.0, 0.0, explode))
    return [
        ("jaw_hub_a", hub_a, PALETTE[2]),
        ("elastomer_spider", parts["spider"], PALETTE[4]),
        ("jaw_hub_b", hub_b, PALETTE[6]),
    ]


def demo_torque_limiter(
    detents: int = 6,
    explode: float = 8.0,
    clearance: float = 0.25,
    sections: int = 64,
) -> MeshList:
    parts = torque_limiter(
        detents=detents, clearance=clearance, sections=sections)
    driven = parts["driven"]
    driven.apply_translation((0.0, 0.0, explode))
    cavity = parts["driven"].metadata
    # N=150: see the comment in demo_helix_tube(). Without it this demo's
    # preload spring alone cost ~450ms native regardless of detents/sections.
    spring = helix_tube(
        cavity["cavity_r"] - 1.4, 0.7, 4.0,
        cavity["cavity_z0"] + explode, cavity["cavity_z1"] + explode, N=150)
    return [
        ("detent_driver", parts["driver"], PALETTE[9]),
        ("pocket_driven", driven, PALETTE[11]),
        ("preload_spring", spring, PALETTE[5]),
    ]


def demo_freewheel_clutch(
    rollers: int = 6,
    clearance: float = 0.25,
    sections: int = 64,
) -> MeshList:
    parts = freewheel_clutch(
        rollers=rollers, clearance=clearance, sections=sections)
    meshes: MeshList = [
        ("ramp_pocket_ring", parts["ring"], PALETTE[0]),
        ("inner_hub", parts["hub"], PALETTE[8]),
    ]
    for index, roller in enumerate(parts["rollers"]):
        meshes.append(("roller_%d" % index, roller, PALETTE[10]))
    return meshes


# ---------------------------------------------------------------------------
# Pulleys and flexures (mechanical-movements wave v0.6.0)
# ---------------------------------------------------------------------------


def _polar(r, deg):
    a = math.radians(deg)
    return r * math.cos(a), r * math.sin(a)


def _belt_segment_2d(teeth, pitch, wrap_deg=200.0, clearance=0.15):
    """Return a toothed GT2 belt arc posed to mesh a ``timing_pulley`` blank."""
    scale = pitch / 2.0
    pitch_r = teeth * pitch / (2.0 * math.pi)
    tip_r = pitch_r - 0.254 * scale
    root_r = pitch_r - 0.75 * scale
    r_back_in = tip_r + clearance + 0.25
    r_back_out = r_back_in + 0.9 * scale
    r_tooth_tip = root_r + clearance + 0.1
    half = wrap_deg / 2.0
    arc_out = [_polar(r_back_out, a) for a in np.linspace(-half, half, 72)]
    arc_in = [_polar(r_back_in, a) for a in np.linspace(half, -half, 72)]
    band = sg.Polygon(arc_out + arc_in)
    teeth_polys = []
    step = 360.0 / teeth
    k = 0
    while k * step <= half:
        angles = (k * step,) if k == 0 else (k * step, -k * step)
        for angle in angles:
            w_root = math.degrees(0.28 * pitch / r_back_in)
            w_tip = math.degrees(0.15 * pitch / r_tooth_tip)
            teeth_polys.append(sg.Polygon([
                _polar(r_back_in + 0.2, angle - w_root),
                _polar(r_back_in + 0.2, angle + w_root),
                _polar(r_tooth_tip, angle + w_tip),
                _polar(r_tooth_tip, angle - w_tip),
            ]))
        k += 1
    return unary_union([band] + teeth_polys).buffer(0)


def demo_timing_pulley(teeth: int = 30, bore_d: float = 3.0) -> MeshList:
    pulley = timing_pulley(teeth=teeth, bore_d=bore_d, hub_d=9.0,
                           setscrew_boss=True, setscrew_d=2.0)
    belt = trimesh.creation.extrude_polygon(
        _belt_segment_2d(teeth, 2.0), 6.0)
    belt.apply_translation((0, 0, 1.2))
    return [
        ("gt2_pulley", pulley, PALETTE[0]),
        ("belt_segment", belt, PALETTE[4]),
    ]


def demo_winch_drum(turns: float = 8.0, cable_d: float = 3.0) -> MeshList:
    drum = grooved_drum(radius_law="cylinder", turns=turns, cable_d=cable_d,
                        core_r=10.0)
    pitch = drum.metadata["groove_pitch"]
    wound = min(2.0, turns - 1.0)
    # N=150: see the comment in demo_helix_tube(). At the mechlib default
    # (N=420) this demo's cable alone cost ~450ms native regardless of turns.
    cable = helix_tube(10.0, cable_d / 2.0, wound, pitch, (1.0 + wound) * pitch, N=150)
    return [
        ("grooved_drum", drum, PALETTE[1]),
        ("cable", cable, PALETTE[7]),
    ]


def demo_fusee(turns: float = 7.0, radius_rise: float = 9.0) -> MeshList:
    fusee = grooved_drum(radius_law="fusee", turns=turns, cable_d=2.5,
                         core_r=7.0, radius_rise=radius_rise)
    return [("fusee_cone", fusee, PALETTE[5])]


def demo_cross_flexure(gap: float = 10.0,
                       blade_angle_deg: float = 45.0) -> MeshList:
    flex = cross_flexure(gap=gap, blade_angle_deg=blade_angle_deg)
    return [("cross_flexure_pivot", flex, PALETTE[2])]


def demo_wave_spring(waves: int = 3, turns: int = 2,
                     sections: int = 96) -> MeshList:
    spring = wave_spring(waves=waves, turns=turns, sections=sections)
    return [("crest_to_crest_spring", spring, PALETTE[3])]


def demo_bistable_beam(apex: float = 3.0, beam_t: float = 1.0) -> MeshList:
    switch = bistable_beam(apex=apex, beam_t=beam_t)
    return [("bistable_switch", switch, PALETTE[9])]


# ---------------------------------------------------------------------------
# Straight-line and scaling linkages (gap-analysis wave v0.8.0)
# ---------------------------------------------------------------------------

# Four of these five mechanisms are driven by an oscillating input rather than
# a whole crank turn (a Peaucellier crank that made a full revolution would
# send its tracer to infinity), so their demos take a 0-360 deg DRIVE PHASE and
# map it onto the working arc with a sine -- the motion a long connecting rod
# or a Scotch yoke gives. That keeps one linear phase ramp per demo, which is
# what ANIMATE can express, while the mechanism itself still travels out and
# back. The pantograph needs no such trick: its stylus really does go round.
PEAUCELLIER_SWING_DEG = 50.0
WATT_SWING_DEG = 25.0
SARRUS_FOLD_MID_DEG = 45.0
SARRUS_FOLD_SWING_DEG = 30.0
TONGS_MID_DEG = 37.5
TONGS_SWING_DEG = 17.5


def demo_peaucellier(drive_deg: float = 0.0, rhomb_len: float = 15.0):
    crank = PEAUCELLIER_SWING_DEG * math.sin(math.radians(drive_deg))
    parts = peaucellier_linkage(rhomb_len=rhomb_len, crank_angle_deg=crank)
    entries = [
        ("anchor_link_a", parts["long_a"], PALETTE[2]),
        ("anchor_link_b", parts["long_b"], PALETTE[2]),
        ("rhombus_pa", parts["rhomb_pa"], PALETTE[5]),
        ("rhombus_bq", parts["rhomb_bq"], PALETTE[5]),
        ("rhombus_pb", parts["rhomb_pb"], PALETTE[3]),
        ("rhombus_aq", parts["rhomb_aq"], PALETTE[3]),
        ("ground_link", parts["ground"], PALETTE[7]),
        ("crank", parts["crank"], PALETTE[4]),
    ]
    names = ("pin_o", "pin_c", "pin_p", "pin_a", "pin_b", "tracer_q")
    for name, pin in zip(names, parts["pins"]):
        entries.append((name, pin,
                        PALETTE[10] if name == "tracer_q" else PALETTE[11]))
    return entries


def demo_watt(drive_deg: float = 0.0, coupler_len: float = 24.0):
    angle = WATT_SWING_DEG * math.sin(math.radians(drive_deg))
    parts = watt_linkage(coupler_len=coupler_len, lever_angle_deg=angle)
    entries = [
        ("ground_link", parts["ground"], PALETTE[7]),
        ("lever_a", parts["lever_a"], PALETTE[2]),
        ("lever_b", parts["lever_b"], PALETTE[4]),
        ("coupler", parts["coupler"], PALETTE[5]),
        ("tracer_point", parts["tracer"], PALETTE[10]),
    ]
    for index, pin in enumerate(parts["pins"]):
        entries.append(("pin_%d" % index, pin, PALETTE[11]))
    return entries


def demo_sarrus(drive_deg: float = 0.0, bar_len: float = 20.0):
    fold = SARRUS_FOLD_MID_DEG + SARRUS_FOLD_SWING_DEG * math.sin(
        math.radians(drive_deg))
    parts = sarrus_linkage(bar_len=bar_len, fold_deg=fold)
    entries = [
        ("base_plate", parts["base"], PALETTE[7]),
        ("platform", parts["platform"], PALETTE[0]),
    ]
    names = ("chain_a_lower", "chain_a_upper", "chain_b_lower", "chain_b_upper")
    colors = (PALETTE[2], PALETTE[5], PALETTE[3], PALETTE[4])
    for name, bar, color in zip(names, parts["bars"], colors):
        entries.append((name, bar, color))
    for index, pin in enumerate(parts["pins"]):
        entries.append(("hinge_pin_%d" % index, pin, PALETTE[11]))
    return entries


def demo_pantograph(crank_angle_deg: float = 0.0, ratio: float = 2.0):
    angle = math.radians(crank_angle_deg)
    parts = pantograph_linkage(ratio=ratio,
                               p_x=32.0 + 9.0 * math.cos(angle),
                               p_y=9.0 * math.sin(angle))
    entries = [
        ("base_pad", parts["base"], PALETTE[7]),
        ("bar_pivot", parts["bar1"], PALETTE[2]),
        ("bar_stylus", parts["bar2"], PALETTE[5]),
        ("bar_output", parts["bar3"], PALETTE[4]),
        ("bar_closer", parts["bar4"], PALETTE[3]),
        ("output_point", parts["tracer"], PALETTE[10]),
    ]
    names = ("pin_f", "pin_a", "pin_c", "pin_d", "stylus_pin")
    for name, pin in zip(names, parts["pins"]):
        entries.append((name, pin, PALETTE[11]))
    return entries


def demo_lazy_tongs(drive_deg: float = 0.0, rhombs: int = 3):
    angle = TONGS_MID_DEG + TONGS_SWING_DEG * math.sin(math.radians(drive_deg))
    parts = lazy_tongs(rhombs=int(rhombs), angle_deg=angle)
    entries = [
        ("frame", parts["frame"], PALETTE[7]),
        ("output_yoke", parts["output"], PALETTE[10]),
    ]
    for index, bar in enumerate(parts["bars"]):
        entries.append(("bar_%d" % index, bar,
                        PALETTE[2] if index % 2 == 0 else PALETTE[5]))
    for index, pin in enumerate(parts["pins"]):
        entries.append(("pin_%d" % index, pin, PALETTE[11]))
    return entries


# ---------------------------------------------------------------------------
# Grippers and clamps (gap-analysis wave v0.8.0)
# ---------------------------------------------------------------------------


def demo_iris_diaphragm(blades: int = 6, close_pct: float = 0.0,
                        blade_t: float = 1.0) -> MeshList:
    # close_pct rather than control_deg: the usable ring travel is only a few
    # degrees and depends on the aperture geometry, so the slider is a
    # percentage of the range iris_control_range() reports for these defaults.
    parts = iris_diaphragm(
        blades=int(blades), blade_t=blade_t,
        control_deg=iris_control_range() * close_pct / 100.0)
    entries = [
        ("base_ring", parts["base"], PALETTE[7]),
        ("drive_ring", parts["drive_ring"], PALETTE[4]),
        ("retainer_cap", parts["cap"], PALETTE[12]),
    ]
    for index, blade in enumerate(parts["blades"]):
        entries.append(("leaf_%d" % index, blade, PALETTE[index % 3]))
    return entries


def demo_collet_chuck(bore_d: float = 6.0, slots: int = 4,
                      taper_deg: float = 8.0) -> MeshList:
    parts = collet_chuck(bore_d=bore_d, slots=int(slots), taper_deg=taper_deg)
    return [
        ("spindle_nose", parts["spindle_nose"], PALETTE[7]),
        ("split_collet", parts["collet"], PALETTE[1]),
        ("taper_nut", parts["nut"], PALETTE[9]),
    ]


def demo_eccentric_cam_clamp(handle_deg: float = 0.0, ecc: float = 4.0,
                             cam_r: float = 14.0) -> MeshList:
    parts = eccentric_cam_clamp(handle_deg=handle_deg, ecc=ecc, cam_r=cam_r)
    return [
        ("clamp_base", parts["base"], PALETTE[7]),
        ("eccentric_cam", parts["cam"], PALETTE[4]),
        ("follower_plate", parts["follower"], PALETTE[2]),
        ("pivot_pin", parts["pin"], PALETTE[10]),
    ]


# ---------------------------------------------------------------------------
# Internal (ring) gears (gap-analysis wave v0.8.0)
# ---------------------------------------------------------------------------


def demo_ring_gear(z: int = 32, m: float = 1.5, width: float = 8.0) -> MeshList:
    """Internal involute ring gear: teeth cut inward from an annular rim."""
    return [("internal_ring_gear", ring_gear(z=z, m=m, width=width), PALETTE[7])]


def demo_ring_gear_mesh(z: int = 40, m: float = 1.5, width: float = 6.0,
                        drive_deg: float = 0.0) -> MeshList:
    """Pinion running inside an internal ring gear, posed in mesh.

    Internal mesh: the two members turn the SAME way (an external pair
    counter-rotates) and the ring turns at z_pinion/z of the pinion. The
    pinion spins about its own centre at (centre_distance, 0), which is the
    DIFFERENCE of the pitch radii, not the sum.
    """
    z_pinion = 20
    pair = ring_gear_mesh(z=z, z_pinion=z_pinion, m=m, width=width,
                          bore_d=5.0, bl=0.35)
    centre = pair["ring"].metadata["centre_distance"]
    return [
        ("internal_ring_gear", _spin(pair["ring"], drive_deg * z_pinion / z),
         PALETTE[7]),
        ("ring_pinion", _spin(pair["pinion"], drive_deg,
                              center=(centre, 0.0, 0.0)), PALETTE[1]),
    ]


# ---------------------------------------------------------------------------
# Constant-velocity couplings (gap-analysis wave v0.8.0)
# ---------------------------------------------------------------------------


def demo_tripod_cv_joint(
    angle_deg: float = 15.0,
    phase_deg: float = 0.0,
    clear: float = 0.3,
    sections: int = 48,
) -> MeshList:
    parts = tripod_cv_joint(
        angle_deg=angle_deg, phase_deg=phase_deg, clear=clear,
        sections=sections)
    return [
        ("tulip_housing", parts["housing"], PALETTE[7]),
        ("tripod_spider", parts["spider"], PALETTE[5]),
        ("crowned_barrels", parts["rollers"], PALETTE[4]),
        ("tripod_input_shaft", parts["shaft"], PALETTE[1]),
    ]


def demo_double_cardan_joint(
    bend_deg: float = 15.0,
    inter_len: float = 46.0,
    clearance: float = 0.3,
    sections: int = 48,
) -> MeshList:
    parts = double_cardan_joint(
        bend_deg=bend_deg, inter_len=inter_len, clearance=clearance,
        sections=sections)
    return [
        ("cardan_input_yoke", parts["yoke_in"], PALETTE[0]),
        ("cardan_spider_in", parts["spider_in"], PALETTE[5]),
        ("cardan_intermediate", parts["intermediate"], PALETTE[8]),
        ("cardan_spider_out", parts["spider_out"], PALETTE[9]),
        ("cardan_output_yoke", parts["yoke_out"], PALETTE[3]),
    ]


# ---------------------------------------------------------------------------
# Joints (gap-analysis wave v0.8.0)
# ---------------------------------------------------------------------------


def demo_ball_socket_joint(
    ball_d: float = 10.0,
    capture_deg: float = 20.0,
    fingers: int = 4,
    pose_deg: float = 15.0,
) -> MeshList:
    parts = ball_socket_joint(ball_d=ball_d, capture_deg=capture_deg,
                              fingers=fingers)
    # The swing cone shrinks as capture_deg grows, so the slider pose is
    # clamped to whatever this geometry actually allows.
    limit = parts["ball"].metadata["swing_half_deg"]
    stud = _spin(parts["ball"], max(-limit, min(limit, pose_deg)),
                 axis=(1.0, 0.0, 0.0))
    return [
        ("socket_cup", parts["socket"], PALETTE[7]),
        ("ball_stud", stud, PALETTE[4]),
    ]


def demo_knuckle_hinge(
    knuckles: int = 5,
    stop_deg: float = 90.0,
    open_deg: float = 110.0,
    leaf_len: float = 16.0,
) -> MeshList:
    parts = knuckle_hinge(knuckles=knuckles, stop_deg=stop_deg,
                          open_deg=max(open_deg, stop_deg), leaf_len=leaf_len)
    return [
        ("hinge_leaf_fixed", parts["leaf_a"], PALETTE[0]),
        ("hinge_leaf_swinging", parts["leaf_b"], PALETTE[2]),
    ]


def demo_gimbal_rings(
    rings: int = 3,
    outer_d: float = 44.0,
    ring_t: float = 6.5,
    tilt_deg: float = 20.0,
) -> MeshList:
    parts = gimbal_rings(rings=rings, outer_d=outer_d, ring_t=ring_t,
                         tilt_deg=tilt_deg)
    return [("gimbal_ring_%d" % index, parts["ring_%d" % index],
             PALETTE[(4 * index) % 13])
            for index in range(int(round(rings)))]


# ---------------------------------------------------------------------------
# Pumps and valves (gap-analysis wave v0.8.0)
# ---------------------------------------------------------------------------


def demo_gerotor_pump(
    lobes: int = 6,
    ecc: float = 1.6,
    rotor_h: float = 8.0,
    rotor_deg: float = 0.0,
) -> MeshList:
    # lobe_circle_r is derived, not an independent PLAY param: the outer tooth
    # radius falls as the lobe count rises, and gerotor_pump() rejects any
    # combination where it drops below ecc (the teeth would become islands in
    # the cavity instead of protrusions). Scaling the pitch circle with the
    # lobe count keeps every slider corner inside that limit, and at lobes=6 it
    # reproduces gerotor_pump()'s own default of 16.0 exactly, so the default
    # GLB is the plain-default part.
    parts = gerotor_pump(lobes=lobes, lobe_circle_r=16.0 * lobes / 6.0,
                         ecc=ecc, rotor_h=rotor_h, phase_deg=rotor_deg)
    return [
        ("gerotor_housing", parts["housing"], PALETTE[7]),
        ("gerotor_outer_rotor", parts["outer"], PALETTE[2]),
        ("gerotor_inner_rotor", parts["inner"], PALETTE[4]),
        ("gerotor_port_cap", parts["cap"], PALETTE[0]),
    ]


def demo_hose_barb(
    tube_id: float = 6.0,
    barbs: int = 3,
    interference: float = 0.6,
    ramp_deg: float = 25.0,
) -> MeshList:
    return [("hose_barb", hose_barb(tube_id=tube_id, barbs=barbs,
                                    interference=interference,
                                    ramp_deg=ramp_deg), PALETTE[5])]


def demo_rotary_spool_valve(
    ports: int = 3,
    body_d: float = 34.0,
    plug_d: float = 16.0,
    plug_deg: float = 0.0,
) -> MeshList:
    # Every diameter that could collide is derived from the port count and the
    # plug size, so no slider corner asks for bores that merge at the plug
    # surface or a passage that reaches no second port. At the defaults these
    # formulas reproduce rotary_spool_valve()'s own defaults (port_d=5.0,
    # passage_d=4.5 rounds to 4.5, passages=((0, 120),)).
    body_d = max(body_d, plug_d + 10.0)
    pitch = plug_d * math.sin(math.pi / ports)
    port_d = max(2.0, min(5.0, pitch - 1.2))
    passage_d = max(2.0, min(port_d, plug_d - 3.0))
    parts = rotary_spool_valve(ports=ports, body_d=body_d, plug_d=plug_d,
                               port_d=port_d, passage_d=passage_d,
                               passages=((0.0, 360.0 / ports),),
                               plug_deg=plug_deg)
    return [
        ("spool_valve_body", parts["body"], PALETTE[7]),
        ("spool_valve_plug", parts["plug"], PALETTE[10]),
        ("spool_valve_cap", parts["cap"], PALETTE[1]),
    ]


def demo_peristaltic_pump_head(
    rollers: int = 3,
    tube_od: float = 6.0,
    occlusion: float = 0.9,
    wrap_deg: float = 240.0,
) -> MeshList:
    # Wall thickness tracks the tube size (a quarter of the OD is typical for
    # peristaltic silicone) so the squeeze gap stays sane across the slider.
    parts = peristaltic_pump_head(rollers=rollers, tube_od=tube_od,
                                  tube_wall=tube_od / 4.0,
                                  occlusion=occlusion, wrap_deg=wrap_deg)
    return [
        ("peristaltic_body", parts["body"], PALETTE[7]),
        ("peristaltic_rotor", parts["rotor"], PALETTE[3]),
        ("peristaltic_cap", parts["cap"], PALETTE[6]),
    ]


# ---------------------------------------------------------------------------
# Guides and ways (gap-analysis wave v0.8.0)
# ---------------------------------------------------------------------------

_WAY_PROFILES = ("dovetail", "vee", "tslot")


def demo_linear_way(
    profile_index: int = 0,
    angle_deg: float = 55.0,
    clear: float = 0.25,
    section_w: float = 26.0,
) -> MeshList:
    parts = linear_way(
        profile=_WAY_PROFILES[int(profile_index) % len(_WAY_PROFILES)],
        section_w=section_w, angle_deg=angle_deg, clear=clear)
    out = [
        ("way_rail", parts["rail"], PALETTE[7]),
        ("way_carriage", parts["carriage"], PALETTE[1]),
    ]
    if "gib" in parts:
        out.append(("way_gib", parts["gib"], PALETTE[4]))
    return out


def demo_telescoping_stage(
    sections: int = 3,
    extend: float = 0.6,
    clear: float = 0.3,
    length: float = 60.0,
) -> MeshList:
    count = int(sections)
    parts = telescoping_stage(sections=count, extend=extend, clear=clear,
                              length=length, outer_w=36.0)
    order = (0, 2, 5, 9)
    return [("stage_section_%d" % i, parts["section_%d" % i],
             PALETTE[order[i % len(order)]]) for i in range(count)]


# ---------------------------------------------------------------------------
# Idlers and belt tensioners (gap-analysis wave v0.8.0)
# ---------------------------------------------------------------------------


def demo_idler_pulley(od: float = 16.0, crown: float = 0.15) -> MeshList:
    idler = idler_pulley(od=od, width=8.0, bore_d=5.0, crown=crown)
    driver = timing_pulley(teeth=20, pitch=2.0, belt_w=8.0, bore_d=5.0,
                           hub_len=0.0)
    driver.apply_translation(
        (od / 2.0 + driver.metadata["tip_d"] / 2.0 + 6.0, 0, 0))
    return [
        ("idler", idler, PALETTE[0]),
        ("driver_pulley", driver, PALETTE[1]),
    ]


def demo_eccentric_idler_mount(eccentricity: float = 1.5,
                               rotation_deg: float = 0.0) -> MeshList:
    parts = eccentric_idler_mount(eccentricity=eccentricity,
                                  rotation_deg=rotation_deg)
    return [
        ("bushing", parts["bushing"], PALETTE[2]),
        ("idler", parts["pulley"], PALETTE[0]),
    ]


def demo_belt_tensioner(preload_mm: float = 2.5,
                        sweep_deg: float = 50.0) -> MeshList:
    tensioner = belt_tensioner(preload_mm=preload_mm, sweep_deg=sweep_deg)
    tip_x, tip_y = tensioner.metadata["tip_xy"]
    idler = idler_pulley(od=12.0, width=6.0, bore_d=5.0, clearance=0.25)
    idler.apply_translation((tip_x, tip_y, 6.0))
    return [
        ("tensioner_arm", tensioner, PALETTE[3]),
        ("idler", idler, PALETTE[0]),
    ]


# ---------------------------------------------------------------------------
# Chains and cable carriers (gap-analysis wave v0.8.0)
# ---------------------------------------------------------------------------


def demo_drag_chain_link(bend_deg: float = 30.0, width: float = 9.0) -> MeshList:
    parts = drag_chain_link(bend_deg=bend_deg, width=width)
    out = [
        ("link", parts["link"], PALETTE[0]),
        ("pin", parts["pin"], PALETTE[4]),
    ]
    if "lid" in parts:
        out.append(("lid", parts["lid"], PALETTE[7]))
    return out


def demo_drag_chain(links: int = 8, bend_deg: float = 30.0,
                    s_bend_at: int = 0) -> MeshList:
    # Clamp so any playground slider combo stays valid (s_bend_at <= links).
    s = min(s_bend_at, links)
    parts = drag_chain(links=links, bend_deg=bend_deg, reverse_bend=s > 0,
                       s_bend_at=s if s > 0 else None)
    out = []
    for name, mesh in parts.items():
        if name.startswith("link_"):
            out.append((name, mesh, PALETTE[0]))
        elif name.startswith("pin_"):
            out.append((name, mesh, PALETTE[4]))
        else:
            out.append((name, mesh, PALETTE[7]))
    return out


def demo_roller_chain_link(roller_d: float = 7.75, pitch: float = 12.7) -> MeshList:
    parts = roller_chain_link(roller_d=roller_d, pitch=pitch)
    return [
        ("outer_plate", parts["outer_plate"], PALETTE[0]),
        ("inner_plate", parts["inner_plate"], PALETTE[1]),
        ("roller", parts["roller"], PALETTE[4]),
        ("bushing", parts["bushing"], PALETTE[7]),
        ("pin", parts["pin"], PALETTE[9]),
    ]


def demo_roller_chain(n_teeth: int = 14, wrap_deg: float = 200.0) -> MeshList:
    parts = roller_chain(n_teeth=n_teeth, wrap_deg=wrap_deg)
    out = [("sprocket", parts["sprocket"], PALETTE[0])]
    for name, mesh in parts.items():
        if name != "sprocket":
            out.append((name, mesh, PALETTE[4]))
    return out


def demo_chain_reverse(wrap_deg: float = 160.0, idler_teeth: int = 10) -> MeshList:
    parts = chain_reverse(wrap_deg=wrap_deg, idler_teeth=idler_teeth)
    out = [("sprocket", parts["sprocket"], PALETTE[0]),
           ("idler", parts["idler"], PALETTE[1])]
    for name, mesh in parts.items():
        if name.startswith("roller_"):
            out.append((name, mesh, PALETTE[4]))
    return out


def demo_chain_s_wrap(wrap_deg: float = 160.0, out_wrap_deg: float = 150.0,
                      out_teeth: int = 12) -> MeshList:
    parts = chain_s_wrap(wrap_deg=wrap_deg, out_wrap_deg=out_wrap_deg,
                         out_teeth=out_teeth)
    out = [("sprocket", parts["sprocket"], PALETTE[0]),
           ("out_sprocket", parts["out_sprocket"], PALETTE[1])]
    for name, mesh in parts.items():
        if name.startswith("roller_"):
            out.append((name, mesh, PALETTE[4]))
    return out


def demo_chain_dual_output(in_pitches: int = 4, idler_teeth: int = 10) -> MeshList:
    parts = chain_dual_output(in_pitches=in_pitches, idler_teeth=idler_teeth)
    out = [("driver", parts["driver"], PALETTE[0]),
           ("out_forward", parts["out_forward"], PALETTE[2]),
           ("idler_reverse", parts["idler_reverse"], PALETTE[1])]
    for name, mesh in parts.items():
        if name.startswith("roller_"):
            out.append((name, mesh, PALETTE[4]))
    return out


# ---------------------------------------------------------------------------
# Springs and linear flexures (gap-analysis wave v0.8.0)
# ---------------------------------------------------------------------------


def demo_belleville_washer(free_h: float = 2.4, stack: int = 3,
                           inner_d: float = 10.0) -> MeshList:
    washer = belleville_washer(free_h=free_h, inner_d=inner_d, stack=stack,
                               arrangement="series")
    return [("belleville_stack", washer, PALETTE[5])]


def demo_coil_spring(turns: float = 6.0, pitch: float = 4.0,
                     wire_d: float = 2.0) -> MeshList:
    spring = coil_spring(turns=turns, pitch=pitch, wire_d=wire_d,
                         ends="closed")
    return [("coil_spring", spring, PALETTE[0])]


def demo_spiral_power_spring(turns: float = 6.0, gap: float = 0.5,
                             strip_t: float = 1.0) -> MeshList:
    parts = spiral_power_spring(turns=turns, gap=gap, strip_t=strip_t)
    return [
        ("mainspring_barrel", parts["barrel"], PALETTE[7]),
        ("mainspring_strip", parts["spring"], PALETTE[3]),
        ("mainspring_arbor", parts["arbor"], PALETTE[1]),
    ]


def demo_leaf_spring(leaves: int = 3, leaf_t: float = 2.0,
                     camber: float = 9.0) -> MeshList:
    parts = leaf_spring(leaves=leaves, leaf_t=leaf_t, camber=camber)
    out = [("leaf_clamp", parts["clamp"], PALETTE[6])]
    for k in range(leaves):
        out.append(("leaf_%d" % (k + 1), parts["leaf_%d" % (k + 1)],
                    PALETTE[(k + 2) % 13]))
    return out


def demo_flexure_stage(travel: float = 2.0, blade_len: float = 25.0,
                       blade_t: float = 1.0) -> MeshList:
    stage = flexure_stage(travel=travel, blade_len=blade_len, blade_t=blade_t)
    return [("flexure_stage", stage, PALETTE[11])]


# ---------------------------------------------------------------------------
# Bearings and bushings (gap-analysis wave v0.8.0)
# ---------------------------------------------------------------------------


def demo_plain_bushing(bore_d: float = 8.0, length: float = 12.0,
                       relief_grooves: int = 4) -> MeshList:
    bushing = plain_bushing(bore_d=bore_d, length=length,
                            relief_grooves=relief_grooves)
    return [("flanged_plain_bushing", bushing, PALETTE[5])]


def demo_thrust_washer(outer_d: float = 24.0, balls: int = 6,
                       thickness: float = 2.4) -> MeshList:
    parts = thrust_washer(outer_d=outer_d, thickness=thickness, pair=True,
                          balls=balls)
    return [
        ("thrust_housing_washer", parts["housing_washer"], PALETTE[1]),
        ("thrust_ball_cage", parts["cage"], PALETTE[6]),
        ("thrust_steel_balls", parts["balls"], PALETTE[9]),
        ("thrust_rotor_washer", parts["rotor_washer"], PALETTE[3]),
    ]


def demo_printed_ball_bearing(outer_d: float = 32.0, balls: int = 6,
                              width: float = 10.0) -> MeshList:
    parts = printed_ball_bearing(outer_d=outer_d, balls=balls, width=width)
    return [
        ("bearing_inner_race", parts["inner_race"], PALETTE[2]),
        ("bearing_ball_cage", parts["cage"], PALETTE[6]),
        ("bearing_balls", parts["balls"], PALETTE[8]),
        ("bearing_outer_race", parts["outer_race"], PALETTE[0]),
    ]


# ---------------------------------------------------------------------------
# Exact-constraint mounts (gap-analysis wave v0.8.0)
# ---------------------------------------------------------------------------


def _flip_aside(mesh, pivot, dx, dz):
    """Book-flip a mating half beside its partner so both faces point up."""
    out = mesh.copy()
    out.apply_transform(trimesh.transformations.rotation_matrix(
        math.pi, (0, 1, 0), pivot))
    out.apply_translation((dx, 0.0, dz))
    return out


def demo_kinematic_coupling(pcd: float = 40.0, ball_d: float = 6.0,
                            plate_t: float = 6.0, kind_index: int = 0):
    ball_d = float(ball_d)
    # kind is a string, so the slider drives an index; pcd and plate_t are
    # clamped to what the ball size can physically support.
    pcd = max(float(pcd), 3.0 * ball_d)
    plate_t = max(float(plate_t), ball_d / 2.0 + 1.7)
    kind = ("maxwell", "kelvin")[int(kind_index) % 2]
    parts = kinematic_coupling(kind=kind, pcd=pcd, ball_d=ball_d,
                               plate_t=plate_t, ball="hardware")
    span = parts["base"].metadata["plate_d"]
    pivot = (0.0, 0.0, parts["top"].bounds.mean(axis=0)[2])
    dx, dz = 1.12 * span, 0.1 * span
    return [
        ("coupling_base", parts["base"], PALETTE[7]),
        ("coupling_top", _flip_aside(parts["top"], pivot, dx, dz), PALETTE[3]),
        ("coupling_balls", _flip_aside(parts["balls"], pivot, dx, dz), PALETTE[5]),
    ]


def demo_repeatable_dock(pcd: float = 40.0, magnet_d: float = 10.0,
                         plate_t: float = 6.0, preload_index: int = 0):
    pcd = float(pcd)
    preload = ("magnet", "screw")[int(preload_index) % 2]
    magnet_d = min(float(magnet_d), pcd - 2.0 * 6.0 - 3.4)
    parts = repeatable_dock(preload=preload, pcd=pcd, magnet_d=magnet_d,
                            plate_t=max(float(plate_t), 5.0), ball="hardware")
    span = parts["base"].metadata["plate_d"]
    pivot = (0.0, 0.0, parts["top"].bounds.mean(axis=0)[2])
    dx, dz = 1.12 * span, 0.1 * span
    return [
        ("dock_base", parts["base"], PALETTE[6]),
        ("dock_top", _flip_aside(parts["top"], pivot, dx, dz), PALETTE[1]),
        ("dock_balls", _flip_aside(parts["balls"], pivot, dx, dz), PALETTE[5]),
    ]


def demo_three_point_leveller(screw_pcd: float = 44.0, screw_index: int = 2,
                              lift: float = 8.0, kind_index: int = 1):
    # Only nominal diameters with a printable coarse pitch are legal.
    screw_d = (4.0, 5.0, 6.0, 8.0)[int(screw_index) % 4]
    kind = ("maxwell", "kelvin")[int(kind_index) % 2]
    screw_pcd = float(screw_pcd)
    # Base wider than the table, so the seats stay visible in the gallery.
    parts = three_point_leveller(kind=kind, screw_pcd=screw_pcd,
                                 screw_d=screw_d, lift=float(lift),
                                 screw_len=float(lift) + 14.0,
                                 plate_d=screw_pcd + 5.0 * screw_d,
                                 table_d=screw_pcd + 2.0 * screw_d + 4.0,
                                 base_t=max(6.0, 0.35 * screw_d + 1.7))
    return [
        ("leveller_base", parts["base"], PALETTE[7]),
        ("leveller_table", parts["table"], PALETTE[2]),
        ("leveller_screws", parts["screws"], PALETTE[10]),
    ]


# ---------------------------------------------------------------------------
# Sealing cutters (gap-analysis wave v0.8.0)
# ---------------------------------------------------------------------------


def demo_oring_groove(
    face_pcd: float = 40.0,
    cs: float = 2.62,
    squeeze: float = 0.20,
    block: float = 60.0,
) -> MeshList:
    block_h = 8.0
    groove = oring_groove(face_pcd=face_pcd, cs=cs, squeeze=squeeze, mode="face")
    body = boxc((block, block, block_h), (0, 0, -block_h / 2.0))
    hollow = sub(body, groove)
    cutaway = sub(hollow, boxc((block + 4, block, block_h + 4),
                               (0, -block / 2.0 - 0.1, -block_h / 2.0)))
    return [("groove_cutaway", cutaway, PALETTE[6])]


def demo_labyrinth_seal(
    shaft_d: float = 8.0,
    teeth: int = 4,
    tooth_t: float = 1.2,
    gap: float = 0.3,
) -> MeshList:
    parts = labyrinth_seal(shaft_d=shaft_d, teeth=teeth, tooth_t=tooth_t, gap=gap)
    rotor, stator = parts["rotor"], parts["stator"]
    total_len = rotor.metadata["total_len"]
    half = boxc((100.0, 100.0, total_len + 4.0), (0, -50.0, total_len / 2.0))
    rotor_cut = sub(rotor, half)
    stator_cut = sub(stator, half)
    return [("rotor_cutaway", rotor_cut, PALETTE[3]),
            ("stator_cutaway", stator_cut, PALETTE[9])]


def demo_gasket_channel(
    width: float = 3.0,
    depth: float = 1.5,
    lid_w: float = 50.0,
    lid_d: float = 36.0,
) -> MeshList:
    lid_h = 8.0
    corner_r = 6.0
    outline = sg.box(-lid_w / 2, -lid_d / 2, lid_w / 2, lid_d / 2).buffer(
        corner_r, join_style=1).buffer(-corner_r, join_style=1)
    inset = 8.0
    channel_r = max(1.5, corner_r - 2.0)
    path = sg.box(-(lid_w / 2 - inset), -(lid_d / 2 - inset),
                  lid_w / 2 - inset, lid_d / 2 - inset).buffer(
        channel_r, join_style=1).buffer(-channel_r, join_style=1).exterior
    groove = gasket_channel(path=path, width=width, depth=depth)
    lid = trimesh.creation.extrude_polygon(outline, lid_h)
    lid.apply_translation((0, 0, -lid_h))
    cut = sub(lid, groove)
    cutaway = sub(cut, boxc((lid_w + 4, lid_d, lid_h + 4),
                            (0, -lid_d / 2.0 - 0.1, -lid_h / 2.0)))
    return [("gasket_lid_cutaway", cutaway, PALETTE[10])]


# ---------------------------------------------------------------------------
# Lattices and kerf patterns (gap-analysis wave v0.8.0)
# ---------------------------------------------------------------------------

_LATTICE_AUXETIC_MODES = ("reentrant", "rotating_squares", "chiral")
_LATTICE_KERF_MODES = ("lattice", "diagonal", "spiral", "wave", "hex", "cross", "chevron", "diamond", "fishbone", "meander")


def demo_auxetic_panel(mode_index: int = 0, cell: float = 12.0,
                       strut_t: float = 1.2) -> MeshList:
    mode = _LATTICE_AUXETIC_MODES[mode_index % 3]
    panel = auxetic_panel(mode=mode, width=60.0, height=60.0, thickness=3.0,
                          cell=cell, strut_t=strut_t)
    return [("auxetic_panel", panel, PALETTE[5])]


def demo_honeycomb_panel(cell: float = 12.0, strut_t: float = 1.2) -> MeshList:
    panel = honeycomb_panel(width=60.0, height=60.0, thickness=3.0,
                            cell=cell, strut_t=strut_t)
    return [("honeycomb_panel", panel, PALETTE[6])]


def demo_isogrid_panel(cell: float = 12.0, strut_t: float = 1.2) -> MeshList:
    panel = isogrid_panel(width=60.0, height=60.0, thickness=3.0,
                          cell=cell, strut_t=strut_t)
    return [("isogrid_panel", panel, PALETTE[7])]


def demo_kerf_bend_cutter(mode_index: int = 0, kerf: float = 0.5,
                          pitch: float = 6.0) -> MeshList:
    mode = _LATTICE_KERF_MODES[mode_index % len(_LATTICE_KERF_MODES)]
    width, height, thickness = 60.0, 40.0, 3.0
    cutters = kerf_bend_cutter(mode=mode, width=width, height=height,
                               thickness=thickness, kerf=kerf, pitch=pitch,
                               bridge=1.0)
    slab = boxc((width, height, thickness), center=(0.0, 0.0, thickness / 2.0))
    cut = sub(slab, uni(cutters))
    return [("kerf_bend_panel", cut, PALETTE[8])]


# ---------------------------------------------------------------------------
# Gap-analysis wave v0.9.0: classic mechanisms still missing from the catalog
# ---------------------------------------------------------------------------

CHEBYSHEV_MID_DEG = 70.0
CHEBYSHEV_SWING_DEG = 28.0  # stays inside the ~37..101 deg reachable arc
# Scott-Russell is exact for a full turn, but at 0/180 the bar is flat on
# the guide and at 90 the slider pin lands on the ground pivot. Drive a
# sine onto the open upper-right quadrant so the tracer walks a clear +Y
# segment and the slider stays clear of the ground pivot.
SCOTT_RUSSELL_MID_DEG = 50.0
SCOTT_RUSSELL_SWING_DEG = 20.0


def demo_slider_crank(crank_angle_deg: float = 35.0,
                      crank_r: float = 14.0) -> MeshList:
    parts = slider_crank(crank_r=crank_r, crank_angle_deg=crank_angle_deg)
    entries = [
        ("base", parts["base"], PALETTE[7]),
        ("crank_disc", parts["crank_disc"], PALETTE[4]),
        ("conrod", parts["conrod"], PALETTE[2]),
        ("slider", parts["slider"], PALETTE[10]),
    ]
    for index, pin in enumerate(parts["pins"]):
        entries.append(("pin_%d" % index, pin, PALETTE[11]))
    return entries


def demo_chebyshev(drive_deg: float = 0.0, unit: float = 10.0) -> MeshList:
    angle = CHEBYSHEV_MID_DEG + CHEBYSHEV_SWING_DEG * math.sin(
        math.radians(drive_deg))
    parts = chebyshev_linkage(unit=unit, crank_angle_deg=angle)
    entries = [
        ("ground", parts["ground"], PALETTE[7]),
        ("rocker_a", parts["rocker_a"], PALETTE[2]),
        ("rocker_b", parts["rocker_b"], PALETTE[4]),
        ("coupler", parts["coupler"], PALETTE[5]),
        ("tracer", parts["tracer"], PALETTE[10]),
    ]
    for index, pin in enumerate(parts["pins"]):
        entries.append(("pin_%d" % index, pin, PALETTE[11]))
    return entries


def demo_scott_russell(drive_deg: float = 0.0,
                       half_len: float = 20.0) -> MeshList:
    angle = SCOTT_RUSSELL_MID_DEG + SCOTT_RUSSELL_SWING_DEG * math.sin(
        math.radians(drive_deg))
    parts = scott_russell_linkage(half_len=half_len, crank_angle_deg=angle)
    entries = [
        ("base", parts["base"], PALETTE[7]),
        ("crank", parts["crank"], PALETTE[4]),
        ("bar", parts["bar"], PALETTE[2]),
        ("slider", parts["slider"], PALETTE[5]),
        ("tracer", parts["tracer"], PALETTE[10]),
    ]
    for index, pin in enumerate(parts["pins"]):
        entries.append(("pin_%d" % index, pin, PALETTE[11]))
    return entries


def demo_bell_crank(pose_deg: float = 25.0, angle_deg: float = 90.0) -> MeshList:
    parts = bell_crank(pose_deg=pose_deg, angle_deg=angle_deg)
    entries = [
        ("base", parts["base"], PALETTE[7]),
        ("crank", parts["crank"], PALETTE[4]),
        ("input_link", parts["link_a"], PALETTE[2]),
        ("output_link", parts["link_b"], PALETTE[10]),
    ]
    for index, pin in enumerate(parts["pins"]):
        entries.append(("pin_%d" % index, pin, PALETTE[11]))
    return entries


def demo_face_cam(pin_phase_deg: float = 0.0, lift: float = 6.0) -> MeshList:
    parts = face_cam(lift=lift, pin_phase_deg=pin_phase_deg)
    return [
        ("cam", parts["cam"], PALETTE[4]),
        ("follower_pin", parts["pin"], PALETTE[10]),
    ]


def demo_swash_plate(phase_deg: float = 0.0, tilt_deg: float = 15.0) -> MeshList:
    parts = swash_plate(phase_deg=phase_deg, tilt_deg=tilt_deg)
    entries = [
        ("shaft", parts["shaft"], PALETTE[11]),
        ("plate", parts["plate"], PALETTE[4]),
    ]
    for index, shoe in enumerate(parts["shoes"]):
        entries.append(("shoe_%d" % index, shoe, PALETTE[2 + index % 4]))
    return entries


def demo_screw_jack(drive_deg: float = 0.0) -> MeshList:
    # Cosine of drive_deg: 0 deg = pad down, 180 deg = full travel, 360 deg =
    # down again. An open lift_frac ramp cannot close a loop; this out-and-back
    # is the same pattern as toggle_clamp / peaucellier.
    lift = 0.5 * (1.0 - math.cos(math.radians(drive_deg)))
    parts = screw_jack(lift_frac=lift)
    return [
        ("base", parts["base"], PALETTE[7]),
        ("screw", parts["screw"], PALETTE[4]),
        ("pad", parts["pad"], PALETTE[10]),
    ]


def demo_rack_pinion(drive_deg: float = 0.0) -> MeshList:
    # ±2 teeth of travel over one drive turn; sine keeps the cycle closed on
    # the same pose (a finite rack has no tooth-pitch picture identity).
    phase_teeth = 2.0 * math.sin(math.radians(drive_deg))
    parts = rack_pinion(phase_teeth=phase_teeth)
    return [
        ("pinion", parts["pinion"], PALETTE[4]),
        ("rack", parts["rack"], PALETTE[2]),
    ]


def demo_dog_clutch(drive_deg: float = 0.0, dogs: int = 4) -> MeshList:
    # Cosine of drive_deg: 0 deg = fully withdrawn, 180 deg = fully engaged,
    # 360 deg = withdrawn. Direct engage_frac 0→1 cannot close a loop.
    engage = 0.5 * (1.0 - math.cos(math.radians(drive_deg)))
    parts = dog_clutch(dogs=int(dogs), engage_frac=engage)
    return [
        ("hub_a", parts["hub_a"], PALETTE[4]),
        ("hub_b", parts["hub_b"], PALETTE[2]),
    ]


def demo_hirth_coupling(teeth: int = 12) -> MeshList:
    parts = hirth_coupling(teeth=int(teeth))
    return [
        ("hub_a", parts["hub_a"], PALETTE[4]),
        ("hub_b", parts["hub_b"], PALETTE[2]),
    ]


def demo_clevis(gap: float = 8.0) -> MeshList:
    parts = clevis(gap=gap)
    return [
        ("fork", parts["fork"], PALETTE[4]),
        ("eye", parts["eye"], PALETTE[2]),
        ("pin", parts["pin"], PALETTE[11]),
    ]


def demo_external_gear_pump(phase_deg: float = 0.0,
                            teeth: int = 12) -> MeshList:
    parts = external_gear_pump(teeth=int(teeth), phase_deg=phase_deg)
    return [
        ("body", parts["body"], PALETTE[7]),
        ("gear_a", parts["gear_a"], PALETTE[4]),
        ("gear_b", parts["gear_b"], PALETTE[2]),
        ("cap", parts["cap"], PALETTE[12]),
    ]


def demo_harmonic_drive(phase_deg: float = 0.0) -> MeshList:
    parts = harmonic_drive(phase_deg=phase_deg)
    return [
        ("circular_spline", parts["circular_spline"], PALETTE[7]),
        ("flex_spline", parts["flex_spline"], PALETTE[4]),
        ("wave_generator", parts["wave_generator"], PALETTE[10]),
    ]


# --- gap-analysis wave v0.10.0 ----------------------------------------------
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
from mechlib.cams import roller_follower
from mechlib.meshutil import sub as _sub5, uni as _uni5
from mechlib.prim import cyl as _cyl5


def demo_v_belt_pulley(grooves: int = 1) -> MeshList:
    mesh = v_belt_pulley(grooves=int(grooves))
    return [("v_belt_pulley", mesh, PALETTE[7])]


def demo_flywheel(spokes: int = 4) -> MeshList:
    mesh = flywheel(spokes=int(spokes))
    return [("flywheel", mesh, PALETTE[1])]


def demo_shaft_collar(style: str = "split") -> MeshList:
    mesh = shaft_collar(style=style)
    return [("shaft_collar", mesh, PALETTE[6])]


def demo_star_knob(lobes: int = 5) -> MeshList:
    mesh = star_knob(lobes=int(lobes))
    return [("star_knob", mesh, PALETTE[9])]


def demo_handwheel() -> MeshList:
    parts = handwheel(crank=True)
    return [
        ("wheel", parts["wheel"], PALETTE[4]),
        ("grip", parts["grip"], PALETTE[11]),
    ]


def demo_beam_coupling(helix_starts: int = 1) -> MeshList:
    mesh = beam_coupling(helix_starts=int(helix_starts))
    return [("beam_coupling", mesh, PALETTE[3])]


def demo_lead_screw() -> MeshList:
    parts = lead_screw()
    nut = parts["nut"].copy()
    # Threaded running fit: the nut meshes at a 120 degree clocking.
    nut.apply_transform(
        trimesh.transformations.rotation_matrix(math.radians(120.0),
                                                (0, 0, 1)))
    nut.apply_translation((0.0, 0.0, 60.0 / 2.0 - 16.0 / 2.0))
    return [
        ("screw", parts["screw"], PALETTE[4]),
        ("nut", nut, PALETTE[2]),
    ]


def demo_roller_follower(pose_deg: float = 20.0) -> MeshList:
    parts = roller_follower(pose_deg=pose_deg)
    return [
        ("arm", parts["arm"], PALETTE[4]),
        ("roller", parts["roller"], PALETTE[2]),
        ("pin", parts["pin"], PALETTE[11]),
    ]


def demo_detent_pair() -> MeshList:
    parts = detent_pair()
    return [
        ("wheel", parts["wheel"], PALETTE[4]),
        ("plunger", parts["plunger"], PALETTE[2]),
        ("housing", parts["housing"], PALETTE[7]),
    ]


def demo_star_wheel(pockets: int = 6) -> MeshList:
    mesh = star_wheel(pockets=int(pockets))
    return [("star_wheel", mesh, PALETTE[5])]


def demo_ratchet_wheel_pawl() -> MeshList:
    parts = ratchet_wheel_pawl()
    return [
        ("wheel", parts["wheel"], PALETTE[4]),
        ("pawl", parts["pawl"], PALETTE[2]),
        ("spring", parts["spring"], PALETTE[11]),
    ]


def demo_thread_insert() -> MeshList:
    parts = thread_insert()
    boss = _sub5(parts["boss"], parts["cavity"])
    insert = parts["insert"].copy()
    insert.apply_translation((0.0, 0.0, 2.0))
    return [
        ("boss", boss, PALETTE[7]),
        ("insert", insert, PALETTE[1]),
    ]


def demo_tslot_nut(profile: str = "2020") -> MeshList:
    mesh = tslot_nut(profile=profile)
    return [("tslot_nut", mesh, PALETTE[6])]


def demo_shaft_key() -> MeshList:
    parts = shaft_key()
    shaft = _sub5(_cyl5(6.0, 30.0, center=(0, 0, 0), sections=64),
                  parts["shaft_keyway"])
    hub_ring = trimesh.creation.annulus(6.0, 11.0, 14.0, sections=64)
    hub = _sub5(hub_ring, parts["hub_keyway"])
    return [
        ("shaft", shaft, PALETTE[7]),
        ("key", parts["key"], PALETTE[1]),
        ("hub", hub, PALETTE[4]),
    ]


def demo_annular_snap() -> MeshList:
    parts = annular_snap()
    ridge = parts["ridge"].copy()
    ridge.apply_translation((0.0, 0.0, 6.0))
    inner = _uni5([
        trimesh.creation.annulus(18.0, 20.0, 12.0, sections=96),
        ridge,
    ])
    groove = parts["groove"].copy()
    groove.apply_translation((0.0, 0.0, 6.0))
    outer = _sub5(trimesh.creation.annulus(18.15, 22.0, 12.0, sections=96),
                  groove)
    outer.apply_translation((0.0, 0.0, 20.0))
    return [
        ("inner", inner, PALETTE[4]),
        ("outer", outer, PALETTE[2]),
    ]


def demo_check_valve() -> MeshList:
    parts = check_valve()
    return [
        ("body", parts["body"], PALETTE[7]),
        ("cap", parts["cap"], PALETTE[2]),
    ]


def demo_bellows_suction_cup(folds: int = 2) -> MeshList:
    mesh = bellows_suction_cup(folds=int(folds))
    return [("bellows_suction_cup", mesh, PALETTE[8])]
