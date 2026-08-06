"""Project-agnostic shaft-coupling generators (Oldham, Cardan, jaw)."""

import math

import shapely.affinity as affinity
import shapely.geometry as sg
from shapely.ops import unary_union
import trimesh
import trimesh.transformations as tf

from .meshutil import sub, uni
from .prim import boxc, cyl, sector2d

# --- constant-velocity joint additions (v0.8.0) ----------------------------
import numpy as np

from .cutters import dbore, teardrop
from .meshutil import inter
from .patterns import polar_ring
from .prim import frustum, seg_cylinder


def _extrude(poly, height, z0=0.0):
    if height <= 0:
        raise ValueError("extrusion height must be positive")
    mesh = trimesh.creation.extrude_polygon(poly, height)
    if not mesh.is_watertight:
        from .meshutil import from_manifold, to_manifold
        mesh = from_manifold(to_manifold(mesh))
    if z0:
        mesh.apply_translation((0.0, 0.0, z0))
    return mesh


def oldham_coupling(d=30.0, bore_d=8.0, tongue_w=8.0, tongue_h=4.0,
                    hub_len=12.0, face_gap=0.4, web=1.6, clearance=0.25,
                    sections=96):
    """Build an Oldham (double-slider) coupling as three assembled parts.

    Returns ``{"hub_a", "disc", "hub_b"}`` in assembled coordinates along +Z.
    Each hub carries a diametral tongue (hub A along X, hub B along Y, 90 deg
    apart) and the floating disc carries a matching slotted channel across each
    face. The disc slides in both slots at once, transmitting constant-velocity
    rotation between parallel shafts with a lateral offset: hub A may shift
    along X and hub B along Y relative to the disc. Slot width is
    ``tongue_w + 2 * clearance`` per mating pair; this tongue/slot fit is the
    single critical print dimension (0.15-0.25 mm per side for FDM).

    Dimensions in mm. ``d`` hub/disc diameter, ``bore_d`` shaft bore,
    ``tongue_w``/``tongue_h`` tongue cross-section, ``hub_len`` hub barrel
    length, ``face_gap`` axial gap between each hub face and the disc, ``web``
    disc material left between the two slot floors (>= 1.2), ``clearance``
    per-side mating clearance, ``sections`` circle segment count.
    """
    if (d <= 0 or bore_d <= 0 or (d - bore_d) / 2.0 < 1.2 or
            tongue_w < 1.2 or tongue_h < 1.2 or hub_len < 1.2 or
            face_gap < 0 or web < 1.2 or clearance < 0 or
            face_gap >= tongue_h or sections < 24):
        raise ValueError("oldham_coupling(): invalid coupling dimensions")
    slot_w = tongue_w + 2.0 * clearance
    tongue_l = d - 3.0
    if slot_w > tongue_l - 2.4 or tongue_l <= bore_d + 2.4:
        raise ValueError("oldham_coupling(): tongue or slot leaves no rim")
    slot_depth = tongue_h - face_gap + clearance
    disc_t = 2.0 * slot_depth + web
    disc_z0 = face_gap
    disc_z1 = face_gap + disc_t
    bore_r = (bore_d + clearance) / 2.0

    hub_a = uni([
        cyl(d / 2.0, hub_len, (0.0, 0.0, -hub_len / 2.0), sections=sections),
        boxc((tongue_l, tongue_w, tongue_h), (0.0, 0.0, tongue_h / 2.0)),
    ])
    hub_a = sub(hub_a, cyl(bore_r, hub_len + 1.0,
                           (0.0, 0.0, -(hub_len + 1.0) / 2.0),
                           sections=sections))
    hub_a.metadata["tongue_w"] = tongue_w
    hub_a.metadata["tongue_h"] = tongue_h

    disc = cyl(d / 2.0, disc_t, (0.0, 0.0, (disc_z0 + disc_z1) / 2.0),
               sections=sections)
    slot_a = boxc((d + 2.0, slot_w, slot_depth + 0.1),
                  (0.0, 0.0, disc_z0 + slot_depth / 2.0))
    slot_b = boxc((slot_w, d + 2.0, slot_depth + 0.1),
                  (0.0, 0.0, disc_z1 - slot_depth / 2.0))
    disc = sub(disc, uni([slot_a, slot_b]))
    disc.metadata["slot_w"] = slot_w
    disc.metadata["slot_depth"] = slot_depth

    face_b = disc_z1 + face_gap
    hub_b = uni([
        cyl(d / 2.0, hub_len, (0.0, 0.0, face_b + hub_len / 2.0),
            sections=sections),
        boxc((tongue_w, tongue_l, tongue_h),
             (0.0, 0.0, face_b - tongue_h / 2.0)),
    ])
    hub_b = sub(hub_b, cyl(bore_r, hub_len + 1.0,
                           (0.0, 0.0, face_b + (hub_len + 1.0) / 2.0),
                           sections=sections))
    hub_b.metadata["tongue_w"] = tongue_w
    hub_b.metadata["tongue_h"] = tongue_h

    return {"hub_a": hub_a, "disc": disc, "hub_b": hub_b}


def universal_joint(shaft_d=10.0, pin_d=5.0, fork_gap=18.0, tine_t=4.0,
                    yoke_w=12.0, fork_len=15.0, web_t=2.0, shaft_len=12.0,
                    bend_deg=20.0, boss_r=5.0, clearance=0.3, sections=64):
    """Build a Cardan (Hooke's) universal joint as three assembled parts.

    Returns ``{"yoke_a", "spider", "yoke_b"}`` posed at ``bend_deg``: yoke A's
    shaft points along -Z, yoke B's shaft is bent by ``bend_deg`` about X, and
    the rigid cross spider carries two pin bosses along X (yoke A) and two
    along the bent axis (yoke B). Pin holes are sized ``pin_d + clearance`` so
    the joint assembles (or prints in place) with FDM-friendly running fit.
    Output speed fluctuates sinusoidally, twice per revolution, by the classic
    Cardan error; keep ``bend_deg`` modest (<= ~35 deg in practice).

    Dimensions in mm, angles in degrees. ``fork_gap`` inner separation of the
    fork tines, ``tine_t`` tine thickness, ``yoke_w`` tine width (eye diameter
    at the pin), ``fork_len`` tine reach from the pin axis to the web,
    ``web_t``/``shaft_len`` yoke base dimensions, ``boss_r`` spider center boss
    radius, ``clearance`` diametral pin/hole clearance.
    """
    if (shaft_d <= 0 or pin_d <= 0 or fork_gap <= 0 or tine_t < 1.2 or
            yoke_w < pin_d + clearance + 2.4 or fork_len < 5.0 or
            web_t < 1.2 or shaft_len <= 0 or not 0.0 <= bend_deg <= 45.0 or
            boss_r <= 0 or 2.0 * boss_r + 2.0 * clearance > fork_gap or
            clearance < 0 or sections < 24):
        raise ValueError("universal_joint(): invalid joint dimensions")
    beta = math.radians(bend_deg)
    eye_r = yoke_w / 2.0
    x0 = fork_gap / 2.0
    pin_half = x0 + tine_t - 0.5

    tines = []
    for side in (-1.0, 1.0):
        xc = side * (x0 + tine_t / 2.0)
        tines.append(boxc((tine_t, yoke_w, fork_len),
                          (xc, 0.0, -fork_len / 2.0)))
        tines.append(cyl(eye_r, tine_t, (xc, 0.0, 0.0), axis="x",
                         sections=sections))
    web = boxc((fork_gap + 2.0 * tine_t, yoke_w, web_t),
               (0.0, 0.0, -fork_len - web_t / 2.0))
    shaft = cyl(shaft_d / 2.0, shaft_len,
                (0.0, 0.0, -fork_len - web_t - shaft_len / 2.0),
                sections=sections)
    yoke_a = uni([web, shaft] + tines)
    hole = cyl(pin_d / 2.0 + clearance / 2.0, fork_gap + 2.0 * tine_t + 2.0,
               axis="x", sections=sections)
    yoke_a = sub(yoke_a, hole)

    bar_a = cyl(pin_d / 2.0, 2.0 * pin_half, axis="x", sections=sections)
    bar_b = cyl(pin_d / 2.0, 2.0 * pin_half, axis="y", sections=sections)
    bar_b.apply_transform(tf.rotation_matrix(beta, (1.0, 0.0, 0.0)))
    boss = cyl(boss_r, 2.0 * boss_r, sections=sections)
    boss.apply_transform(tf.rotation_matrix(beta, (1.0, 0.0, 0.0)))
    spider = uni([bar_a, bar_b, boss])

    yoke_b = yoke_a.copy()
    yoke_b.apply_transform(tf.rotation_matrix(math.pi, (1.0, 0.0, 0.0)))
    yoke_b.apply_transform(tf.rotation_matrix(math.pi / 2.0, (0.0, 0.0, 1.0)))
    yoke_b.apply_transform(tf.rotation_matrix(beta, (1.0, 0.0, 0.0)))

    metadata = {"bend_deg": bend_deg, "pin_d": pin_d}
    for mesh in (yoke_a, spider, yoke_b):
        mesh.metadata.update(metadata)
    return {"yoke_a": yoke_a, "spider": spider, "yoke_b": yoke_b}


def jaw_coupling(d=30.0, bore_d=8.0, jaws=3, jaw_deg=30.0, jaw_h=7.0,
                 hub_len=12.0, jaw_r0=5.5, jaw_r1=13.0, spider_t=5.0,
                 face_gap=0.5, clearance=0.25, sections=96):
    """Build a Lovejoy-style elastomeric jaw coupling as three parts.

    Returns ``{"hub_a", "spider", "hub_b"}`` in assembled coordinates along
    +Z. Each hub carries ``jaws`` axial jaws of angular width ``jaw_deg``; hub
    B is clocked half a pitch so the jaws interleave, and the spider floats
    between the hub faces with one lobe in every gap (``2 * jaws`` lobes).
    Torque passes through the spider lobes in compression, damping vibration
    and failing safe if the spider dies. The spider is an elastomer part in
    practice: print the hubs in PLA/PETG and the spider in TPU.

    Dimensions in mm, angles in degrees. ``jaw_r0``/``jaw_r1`` inner/outer jaw
    radius, ``jaw_h`` jaw axial height, ``spider_t`` spider thickness,
    ``face_gap`` axial gap between each hub face and the far jaw tips,
    ``clearance`` radial/circumferential mating clearance. The lobe angular
    width is ``180 / jaws - jaw_deg`` minus twice the clearance angle; the
    generator raises when that leaves a lobe thinner than 2 deg.
    """
    if (d <= 0 or bore_d <= 0 or jaws < 2 or jaw_deg <= 0 or
            jaw_h < 1.2 or hub_len < 1.2 or spider_t < 1.2 or
            not 0 < jaw_r0 < jaw_r1 <= d / 2.0 or
            jaw_r0 < (bore_d + clearance) / 2.0 + 1.2 or
            face_gap < 0 or clearance < 0 or sections < 24):
        raise ValueError("jaw_coupling(): invalid coupling dimensions")
    pitch = 360.0 / jaws
    if jaw_deg >= 0.5 * pitch:
        raise ValueError("jaw_coupling(): jaw_deg leaves no gap for the spider")
    r_mid = 0.5 * (jaw_r0 + jaw_r1)
    clearance_ang = math.degrees(clearance / r_mid)
    lobe_deg = 0.5 * pitch - jaw_deg - 2.0 * clearance_ang
    if lobe_deg < 2.0:
        raise ValueError("jaw_coupling(): spider lobe angle collapses")
    spider_z0 = 0.5 * (jaw_h + face_gap - spider_t)
    if spider_z0 <= 0:
        raise ValueError("jaw_coupling(): spider_t exceeds the jaw band")

    jaw_poly = sector2d(-jaw_deg / 2.0, jaw_deg / 2.0, jaw_r1, n=16)
    jaw_poly = jaw_poly.difference(
        sg.Point(0.0, 0.0).buffer(jaw_r0, resolution=64)).buffer(0)

    def _hub_mesh():
        parts = [cyl(d / 2.0, hub_len, (0.0, 0.0, -hub_len / 2.0),
                     sections=sections)]
        for k in range(jaws):
            poly = affinity.rotate(jaw_poly, k * pitch, origin=(0.0, 0.0))
            parts.append(_extrude(poly, jaw_h))
        hub = uni(parts)
        bore = cyl((bore_d + clearance) / 2.0, hub_len + 1.0,
                   (0.0, 0.0, -hub_len / 2.0), sections=sections)
        return sub(hub, bore)

    hub_a = _hub_mesh()
    hub_b = _hub_mesh()
    hub_b.apply_transform(tf.rotation_matrix(
        math.radians(0.5 * pitch), (0.0, 0.0, 1.0)))
    hub_b.apply_transform(tf.rotation_matrix(math.pi, (1.0, 0.0, 0.0)))
    hub_b.apply_translation((0.0, 0.0, jaw_h + face_gap))

    lobe_poly = sector2d(-lobe_deg / 2.0, lobe_deg / 2.0,
                         jaw_r1 - clearance, n=16)
    lobe_poly = lobe_poly.difference(
        sg.Point(0.0, 0.0).buffer(jaw_r0 - clearance - 0.5,
                                  resolution=64)).buffer(0)
    center = sg.Point(0.0, 0.0).buffer(jaw_r0 - clearance, resolution=64)
    spider_2d = unary_union([center] + [
        affinity.rotate(lobe_poly, (j + 0.5) * 0.5 * pitch, origin=(0.0, 0.0))
        for j in range(2 * jaws)
    ]).buffer(0)
    spider = _extrude(spider_2d, spider_t, spider_z0)

    metadata = {"jaws": jaws, "lobe_deg": lobe_deg}
    for mesh in (hub_a, spider, hub_b):
        mesh.metadata.update(metadata)
    return {"hub_a": hub_a, "spider": spider, "hub_b": hub_b}


__all__ = (
    "oldham_coupling",
    "universal_joint",
    "jaw_coupling",
)


# ---------------------------------------------------------------------------
# Constant-velocity joints (v0.8.0)
#
# A single Hooke (Cardan) joint is NOT constant velocity: at a shaft angle its
# output speed oscillates twice per turn. ``cv_velocity_ratio`` states that
# error in closed form and ``tripod_cv_joint`` is the printable fix.
# ---------------------------------------------------------------------------

_CV_JOINTS = ("hooke", "tripod", "double_cardan", "double_cardan_intermediate")


def cv_velocity_ratio(angle_deg=15.0, phase_deg=0.0, joint="hooke"):
    """Return the instantaneous output/input angular velocity ratio.

    A pure number, no geometry. For a single Hooke (Cardan) joint bent through
    ``angle_deg`` the closed form is ``cos(b) / (1 - sin(b)**2 * cos(t)**2)``
    with ``b`` the shaft angle and ``t`` the input phase measured from the
    position where the input yoke's pin axis lies in the plane of the two
    shafts. It runs fast at ``t = 0, 180`` (``1/cos(b)``) and slow at
    ``t = 90, 270`` (``cos(b)``), so the output oscillates twice per input
    turn -- the "Cardan error" that shakes a driveline.

    ``joint`` selects the mechanism: ``"hooke"`` for the single Cardan joint,
    ``"tripod"`` for a three-trunnion constant-velocity joint (identically
    1.0 at every phase and every angle), ``"double_cardan"`` for two Hooke
    joints phased 90 degrees apart at equal angles (also identically 1.0),
    and ``"double_cardan_intermediate"`` for the intermediate shaft of that
    pair, which still fluctuates exactly like a single Hooke joint. Angles
    are in degrees.
    """
    if joint not in _CV_JOINTS:
        raise ValueError("cv_velocity_ratio(): joint must be one of %s"
                         % (_CV_JOINTS,))
    if not -89.0 <= angle_deg <= 89.0:
        raise ValueError("cv_velocity_ratio(): angle_deg must be within "
                         "+/-89 degrees")
    if joint in ("tripod", "double_cardan"):
        return 1.0
    beta = math.radians(angle_deg)
    theta = math.radians(phase_deg)
    denominator = 1.0 - (math.sin(beta) * math.cos(theta)) ** 2
    return math.cos(beta) / denominator


def cv_velocity_fluctuation(angle_deg=15.0, joint="hooke"):
    """Return the peak-to-peak output speed swing as a fraction of input.

    The number an engineer actually sizes a driveline against. For a single
    Hooke joint the extremes of ``cv_velocity_ratio`` are ``1/cos(b)`` and
    ``cos(b)``, so the peak-to-peak swing is ``sin(b)**2 / cos(b)``: 0.0069
    (+/-0.35 percent) at 5 degrees, 0.0693 (+/-3.5 percent) at 15 degrees,
    and 0.2088 (+/-10 percent) at 25 degrees. It grows without bound as the
    shaft angle approaches 90 degrees. A tripod or a correctly phased double
    Cardan returns exactly 0.0. Angles are in degrees.
    """
    if joint not in _CV_JOINTS:
        raise ValueError("cv_velocity_fluctuation(): joint must be one of %s"
                         % (_CV_JOINTS,))
    if not -89.0 <= angle_deg <= 89.0:
        raise ValueError("cv_velocity_fluctuation(): angle_deg must be within "
                         "+/-89 degrees")
    if joint in ("tripod", "double_cardan"):
        return 0.0
    beta = math.radians(angle_deg)
    return abs(math.sin(beta) ** 2 / math.cos(beta))


def tripod_pose(angle_deg=15.0, phase_deg=0.0, pitch_r=12.0, plunge=0.0):
    """Return the exact rigid pose of a three-trunnion tripod joint.

    The motion law, as numbers only. Put the housing axis on +Z with the joint
    centre at the origin and tilt the inner shaft by ``angle_deg`` about +X.
    Each trunnion centre must stay in its own track plane -- the plane through
    the housing axis at 0, 120 and 240 degrees. Writing that constraint for all
    three trunnions and summing gives ``housing_deg == phase_deg`` exactly, at
    every angle: the tripod is a true constant-velocity joint, not an
    approximation. Solving the two remaining independent equations puts the
    spider centre on a circle of radius ``pitch_r * (1 - cos(b)) / 2`` traversed
    at THREE times the input speed, which is where a tripod joint's third-order
    shudder comes from.

    Three is not a styling choice. Repeat the algebra for four trunnions and
    the constraint set is inconsistent (no spider centre satisfies it); for six
    it forces ``sin(2 * phase) == 0``. Only three closes.

    Returns ``{"housing_deg", "spider_deg", "centre", "orbit_r", "ratio"}``.
    ``centre`` is the spider centre in housing coordinates, with ``plunge``
    passed straight through as its Z: the tripod's axial degree of freedom is
    free, which is why tripods plunge and Rzeppa joints do not. Units are mm
    and degrees.
    """
    if pitch_r <= 0.0:
        raise ValueError("tripod_pose(): pitch_r must be positive")
    if not -89.0 <= angle_deg <= 89.0:
        raise ValueError("tripod_pose(): angle_deg must be within +/-89 deg")
    beta = math.radians(angle_deg)
    theta = math.radians(phase_deg)
    orbit_r = pitch_r * (1.0 - math.cos(beta)) / 2.0
    return {
        "housing_deg": float(phase_deg),
        "spider_deg": float(phase_deg),
        "centre": (orbit_r * math.cos(3.0 * theta),
                   orbit_r * math.sin(3.0 * theta),
                   float(plunge)),
        "orbit_r": orbit_r,
        "ratio": 1.0,
    }


def _barrel(r_max, half_len, crown_r, sections, slices=9):
    """Convex crowned barrel about +Z: radius ``r_max`` at the mid-plane."""
    rings = []
    angles = np.linspace(0.0, 2.0 * np.pi, sections, endpoint=False)
    for z in np.linspace(-half_len, half_len, slices):
        drop = crown_r - math.sqrt(max(crown_r * crown_r - z * z, 0.0))
        r = r_max - drop
        rings.append(np.c_[r * np.cos(angles), r * np.sin(angles),
                           np.full(sections, z)])
    return trimesh.Trimesh(vertices=np.vstack(rings)).convex_hull


def tripod_cv_joint(shaft_d=8.0, trunnions=3, trunnion_d=5.0, housing_d=40.0,
                    angle_deg=15.0, phase_deg=0.0, plunge=0.0, swing_deg=25.0,
                    plunge_travel=4.0, roller_wall=1.6, roller_len=6.0,
                    crown=0.4, hub_t=6.0, wall=2.4, floor_t=4.0, flare_h=6.0,
                    pin_d=3.0, shaft_len=30.0, clear=0.3, sections=48):
    """Build a plunging tripod constant-velocity joint, posed and assembled.

    Returns ``{"housing", "spider", "rollers", "shaft"}`` in assembled
    coordinates. The housing (tulip) axis is +Z with the joint centre at the
    origin, its mouth opening toward +Z and its closed floor toward -Z. Three
    tracks -- parallel-walled slots running the full depth of the tulip at 0,
    120 and 240 degrees -- straddle three crowned barrel rollers carried on the
    spider's three radial trunnions. The inner shaft is tilted ``angle_deg``
    about +X and both members are turned ``phase_deg``; ``plunge`` slides the
    whole inner assembly along the housing axis.

    Unlike a Hooke joint this transmits genuinely constant angular velocity:
    the housing turns degree for degree with the input at every angle and
    every phase (see ``tripod_pose`` for the algebra and ``cv_velocity_ratio``
    for what a single Cardan joint costs you instead). The spider centre
    orbits a small circle at three times input speed, which is the tripod's
    known third-order excitation, and the joint plunges freely along the
    housing axis because nothing constrains that direction.

    The tripod type is chosen over the Rzeppa ball type deliberately. A Rzeppa
    needs six hardened steel balls running in ground meridional raceways held
    by a slotted cage; nothing about that is an honest FDM part. Trunnions in
    straddling tracks print natively: the tulip prints mouth-up so its tracks
    are vertical open-ended slots with no bridging, the barrels print on end,
    and the only real compromise is the spider, whose three trunnions are
    horizontal cylinders that want a sliver of support or a lay-flat
    orientation. Nothing here needs bought hardware; a boot or a circlip is
    still wanted in service to stop the joint pulling apart at the open mouth.

    Metadata carries ``angle_max_deg`` and ``plunge_mm``, both measured off the
    part that was actually built rather than asserted: ``angle_max_deg`` is
    bisected against the three real limits (the shaft fouling the flared mouth,
    the spider hub fouling the tulip bore, and a barrel running out of track),
    and ``plunge_mm`` is the axial travel left before the lowest barrel lands
    on the tulip floor at the current angle. The bisection treats the inner
    shaft as a full round of ``shaft_d`` and ignores its drive flats, so it is
    a lower bound: at ``angle_max_deg`` every pair still measures at least
    ``clear`` apart. Also carried: ``velocity_ratio``
    (always 1.0), ``fluctuation`` (always 0.0), and ``hooke_fluctuation``, the
    peak-to-peak speed error a single Cardan joint would show at the same
    angle.

    Dimensions in mm, angles in degrees. ``swing_deg`` is the articulation the
    tulip is CUT for and drives every housing dimension; ``angle_deg`` and
    ``phase_deg`` and ``plunge`` only pose the parts, so one housing can be
    probed across its whole angular range. ``trunnion_d`` post diameter,
    ``roller_wall`` barrel wall over the post, ``roller_len`` barrel length,
    ``crown`` the radial relief at each barrel end (the crown is what keeps the
    running clearance equal to ``clear`` at every angle -- a plain cylindrical
    roller tilts out of its track plane and binds), ``hub_t`` spider hub
    thickness, ``wall`` tulip wall over the track floor, ``floor_t`` tulip
    floor thickness, ``flare_h`` height of the 45 degree mouth flare, ``pin_d``
    the cross-pin hole near the shaft end, ``clear`` per-side running
    clearance. Units are mm and degrees.
    """
    if trunnions != 3:
        raise ValueError(
            "tripod_cv_joint(): trunnions must be 3; the constant-velocity "
            "constraint is inconsistent for any other count (four admits no "
            "spider centre at all, six forces sin(2*phase)==0)")
    if (shaft_d <= 0 or trunnion_d <= 0 or housing_d <= 0 or clear <= 0 or
            roller_wall < 0.8 or roller_len < 2.0 or crown <= 0 or
            hub_t < 2.0 or wall < 1.2 or floor_t < 1.2 or flare_h < 0 or
            pin_d <= 0 or shaft_len <= 0 or plunge_travel < 0 or
            sections < 16 or crown >= roller_len / 2.0):
        raise ValueError("tripod_cv_joint(): invalid joint dimensions")
    if not 0.0 < swing_deg <= 45.0:
        raise ValueError("tripod_cv_joint(): swing_deg must be in (0, 45]")
    if shaft_len < hub_t + 2.0 * pin_d + 6.0:
        raise ValueError("tripod_cv_joint(): shaft_len is too short for the "
                         "hub plus its cross-pin hole")
    if angle_deg < 0.0:
        raise ValueError("tripod_cv_joint(): angle_deg must be non-negative")

    shaft_r = shaft_d / 2.0
    flat = 0.75 * shaft_d
    roller_ir = trunnion_d / 2.0 + clear
    roller_r = roller_ir + roller_wall
    track_hw = roller_r + clear
    r_out = housing_d / 2.0
    track_r_out = r_out - wall
    hub_r = shaft_r + 2.5
    half_len = roller_len / 2.0
    crown_r = (half_len * half_len + crown * crown) / (2.0 * crown)
    swing = math.radians(swing_deg)

    # Pitch radius: at full swing the barrel's outer end must still sit inside
    # the track floor with clearance, and the orbit itself scales with pitch_r,
    # so solve the two together.
    pitch_r = ((track_r_out - half_len - clear - 1.0) /
               (1.0 + (1.0 - math.cos(swing)) / 2.0))
    if pitch_r < hub_r + half_len + 1.0:
        raise ValueError("tripod_cv_joint(): housing_d leaves no room between "
                         "the spider hub and the track floor")

    def _orbit(beta):
        return pitch_r * (1.0 - math.cos(beta)) / 2.0

    def _reach(beta):
        """Axial half-extent of the lowest barrel about the spider centre."""
        return ((pitch_r + half_len) * math.sin(beta) +
                roller_r * math.cos(beta))

    z_hi = _reach(swing) + plunge_travel
    z_lo = -z_hi
    z_flare = max(z_hi - flare_h, 0.5)
    bore_r = 0.4 + max(
        hub_r + (hub_t / 2.0) * math.sin(swing) + _orbit(swing) + clear,
        shaft_r / math.cos(swing) + z_flare * math.tan(swing) +
        _orbit(swing) + clear)
    mouth_r = bore_r + (z_hi - z_flare)
    if mouth_r > track_r_out - 1.2:
        raise ValueError("tripod_cv_joint(): the flared mouth breaks into the "
                         "track floor; reduce flare_h or swing_deg")
    hub_drop = (hub_t / 2.0) * math.cos(swing) + hub_r * math.sin(swing)
    if hub_drop >= _reach(swing):
        raise ValueError("tripod_cv_joint(): the spider hub bottoms out "
                         "before the barrels do; reduce hub_t")

    def _slack(beta):
        """Smallest of the three real articulation limits, in mm."""
        orbit = _orbit(beta)
        return min(
            bore_r - (hub_r + (hub_t / 2.0) * math.sin(beta) + orbit + clear),
            bore_r - (shaft_r / math.cos(beta) + z_flare * math.tan(beta) +
                      orbit + clear),
            z_hi - _reach(beta))

    lo, hi = 0.0, math.radians(60.0)
    if _slack(lo) <= 0.0:
        raise ValueError("tripod_cv_joint(): the joint does not clear itself "
                         "even at zero angle")
    for _step in range(60):
        mid = 0.5 * (lo + hi)
        if _slack(mid) > 0.0:
            lo = mid
        else:
            hi = mid
    angle_max_deg = math.degrees(lo)
    if angle_deg > angle_max_deg:
        raise ValueError(
            "tripod_cv_joint(): angle_deg %.2f exceeds the %.2f degree "
            "articulation this housing was cut for; raise swing_deg"
            % (angle_deg, angle_max_deg))
    if abs(plunge) > z_hi + roller_len:
        raise ValueError("tripod_cv_joint(): plunge is off the end of the "
                         "tulip entirely")

    beta = math.radians(angle_deg)
    plunge_mm = z_hi - _reach(beta)
    pose = tripod_pose(angle_deg=angle_deg, phase_deg=phase_deg,
                       pitch_r=pitch_r, plunge=plunge)
    ring = polar_ring(trunnions, 1.0)

    # --- housing (tulip), built about +Z then turned by the phase -----------
    depth = z_hi - (z_lo - floor_t)
    body = cyl(r_out, depth, (0.0, 0.0, z_hi - depth / 2.0), sections=sections)
    cavity = [
        cyl(bore_r, z_flare - z_lo, (0.0, 0.0, (z_flare + z_lo) / 2.0),
            sections=sections),
        frustum(bore_r, mouth_r, z_hi - z_flare, z0=z_flare, sections=sections),
    ]
    slot_l = track_r_out + track_hw
    slot_h = z_hi + 2.0 - z_lo
    for ux, uy in ring:
        slot = boxc((slot_l, 2.0 * track_hw, slot_h),
                    (track_r_out - slot_l / 2.0, 0.0,
                     (z_lo + z_hi + 2.0) / 2.0))
        slot.apply_transform(tf.rotation_matrix(math.atan2(uy, ux),
                                                (0.0, 0.0, 1.0)))
        cavity.append(slot)
    out_bore = dbore(shaft_d, flat, floor_t + 2.0, axis="z", clear=clear)
    out_bore.apply_translation((0.0, 0.0, z_lo - floor_t / 2.0))
    cavity.append(out_bore)
    housing = sub(body, uni(cavity))

    # --- spider, barrels and shaft, built about the inner shaft axis --------
    posts = []
    barrels = []
    for ux, uy in ring:
        azimuth = math.atan2(uy, ux)
        tip = pitch_r + half_len + 0.4
        posts.append(seg_cylinder((0.35 * hub_r * ux, 0.35 * hub_r * uy, 0.0),
                                  (tip * ux, tip * uy, 0.0), trunnion_d))
        barrel = _barrel(roller_r, half_len, crown_r, sections)
        barrel = sub(barrel, cyl(roller_ir, roller_len + 2.0,
                                 sections=sections))
        barrel.apply_transform(tf.rotation_matrix(math.pi / 2.0,
                                                  (0.0, 1.0, 0.0)))
        barrel.apply_translation((pitch_r, 0.0, 0.0))
        barrel.apply_transform(tf.rotation_matrix(azimuth, (0.0, 0.0, 1.0)))
        barrels.append(barrel)
    spider = uni([cyl(hub_r, hub_t, sections=sections)] + posts)
    spider = sub(spider, dbore(shaft_d, flat, hub_t + 4.0, axis="z",
                              clear=clear))
    rollers = uni(barrels)

    shaft_z0 = -hub_t / 2.0
    shaft = cyl(shaft_r, shaft_len, (0.0, 0.0, shaft_z0 + shaft_len / 2.0),
                sections=sections)
    shaft = inter(shaft, boxc((flat, 4.0 * shaft_d, shaft_len + 2.0),
                              (0.0, 0.0, shaft_z0 + shaft_len / 2.0)))
    pin_cut = teardrop(pin_d / 2.0 + clear / 2.0, 3.0 * shaft_d, axis="x",
                       up=(0.0, 0.0, 1.0))
    pin_cut.apply_translation((0.0, 0.0, shaft_z0 + shaft_len - 6.0))
    shaft = sub(shaft, pin_cut)

    inner = (tf.translation_matrix(pose["centre"]) @
             tf.rotation_matrix(beta, (1.0, 0.0, 0.0)) @
             tf.rotation_matrix(math.radians(pose["spider_deg"]),
                                (0.0, 0.0, 1.0)))
    for mesh in (spider, rollers, shaft):
        mesh.apply_transform(inner)
    housing.apply_transform(tf.rotation_matrix(
        math.radians(pose["housing_deg"]), (0.0, 0.0, 1.0)))

    metadata = {
        "trunnions": trunnions,
        "angle_deg": float(angle_deg),
        "angle_max_deg": angle_max_deg,
        "plunge_mm": plunge_mm,
        "pitch_r": pitch_r,
        "orbit_r": pose["orbit_r"],
        "track_w": 2.0 * track_hw,
        "roller_d": 2.0 * roller_r,
        "bore_r": bore_r,
        "clear": clear,
        "velocity_ratio": 1.0,
        "fluctuation": 0.0,
        "hooke_fluctuation": cv_velocity_fluctuation(angle_deg, "hooke"),
    }
    for mesh in (housing, spider, rollers, shaft):
        mesh.metadata.update(metadata)
    return {"housing": housing, "spider": spider, "rollers": rollers,
            "shaft": shaft}


def double_cardan_joint(shaft_d=10.0, bend_deg=15.0, inter_len=46.0,
                        pin_d=5.0, fork_gap=18.0, tine_t=4.0, yoke_w=12.0,
                        fork_len=15.0, web_t=2.0, shaft_len=12.0, boss_r=5.0,
                        clearance=0.3, sections=48):
    """Build a double Cardan (two Hooke joints in series) as five parts.

    Returns ``{"yoke_in", "spider_in", "intermediate", "spider_out",
    "yoke_out"}``. The input shaft runs along -Z from a first Hooke joint whose
    centre is the origin; the intermediate shaft leaves at ``bend_deg`` about
    +X; a second identical joint sits ``inter_len`` along that shaft and bends
    the output by ``bend_deg`` again, so input and output are separated by
    ``2 * bend_deg`` in one plane. The two intermediate yokes come out exactly
    90 degrees apart on their own -- that falls out of the construction, it is
    not clocked in by hand -- and 90 degrees is what makes the second joint's
    Cardan error the exact inverse of the first's.

    This is the OTHER classical fix for the Hooke joint's speed error, and it
    is only a partial one: the OUTPUT runs at constant velocity, but the
    intermediate shaft between the two joints still swings by the full single
    joint fluctuation (``cv_velocity_fluctuation(bend_deg)``) and still carries
    that inertia torque. A tripod or Rzeppa joint has no fluctuating member at
    all. Cancellation also depends on the two shaft angles staying EQUAL: this
    generator poses them equal by construction, and real drivelines hold that
    with parallel flange faces or with a centring ball between the yokes, which
    is not modelled here. Say so before trusting one at a varying angle.

    Print each yoke with its fork mouth up so the tine bores are vertical, and
    the cross spiders on end. Nothing needs bought hardware. Dimensions in mm,
    angles in degrees; the yoke arguments are passed straight to
    ``universal_joint``. ``inter_len`` sets the joint-centre spacing and must
    leave the two intermediate yokes' shaft stubs overlapping, which is what
    fuses them into one rigid intermediate body. Units are mm and degrees.
    """
    reach = fork_len + web_t
    if not reach * 2.0 <= inter_len <= 2.0 * (reach + shaft_len):
        raise ValueError(
            "double_cardan_joint(): inter_len must lie in [%.1f, %.1f] so the "
            "intermediate yokes clear each other yet still fuse into one body"
            % (2.0 * reach, 2.0 * (reach + shaft_len)))
    if not 0.0 <= bend_deg <= 45.0:
        raise ValueError("double_cardan_joint(): bend_deg must be in [0, 45]")

    common = dict(shaft_d=shaft_d, pin_d=pin_d, fork_gap=fork_gap,
                  tine_t=tine_t, yoke_w=yoke_w, fork_len=fork_len,
                  web_t=web_t, shaft_len=shaft_len, bend_deg=bend_deg,
                  boss_r=boss_r, clearance=clearance, sections=sections)
    first = universal_joint(**common)
    second = universal_joint(**common)

    beta = math.radians(bend_deg)
    axis = (0.0, -math.sin(beta), math.cos(beta))
    place = (tf.translation_matrix([inter_len * v for v in axis]) @
             tf.rotation_matrix(beta, (1.0, 0.0, 0.0)))
    for mesh in second.values():
        mesh.apply_transform(place)
    intermediate = uni([first["yoke_b"], second["yoke_a"]])

    metadata = {
        "bend_deg": bend_deg,
        "total_angle_deg": 2.0 * bend_deg,
        "inter_len": inter_len,
        "phasing_deg": 90.0,
        "output_fluctuation": cv_velocity_fluctuation(2.0 * bend_deg,
                                                      "double_cardan"),
        "intermediate_fluctuation": cv_velocity_fluctuation(bend_deg, "hooke"),
    }
    parts = {"yoke_in": first["yoke_a"], "spider_in": first["spider"],
             "intermediate": intermediate, "spider_out": second["spider"],
             "yoke_out": second["yoke_b"]}
    for mesh in parts.values():
        mesh.metadata.update(metadata)
    return parts


__all__ += (
    "cv_velocity_ratio",
    "cv_velocity_fluctuation",
    "tripod_pose",
    "tripod_cv_joint",
    "double_cardan_joint",
)
