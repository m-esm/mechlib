"""Project-agnostic printed worm-drive and planetary-stage generators."""

import functools
import math

import shapely.affinity as affinity
import shapely.geometry as sg
import trimesh
import trimesh.transformations as tf

from .gears import mesh_phase, spur_gear_2d
from .meshutil import sub, uni
from .prim import cyl, frustum, hex_poly
from .sweep import extrude_twist


def worm_profile(theta, starts, outer_r, pitch_r, root_r,
                 tip_half, pitch_half, root_half):
    """Return the radial worm-thread profile at angular position ``theta``."""
    span = 360.0 / starts
    aa = abs(((theta + span / 2.0) % span) - span / 2.0)
    if aa <= tip_half:
        return outer_r
    if aa <= pitch_half:
        return outer_r + (pitch_r - outer_r) * (
            (aa - tip_half) / (pitch_half - tip_half))
    if aa <= root_half:
        return pitch_r + (root_r - pitch_r) * (
            (aa - pitch_half) / (root_half - pitch_half))
    return root_r


def printed_worm(
        handsign=1, motor_tail=None, module=1.2, pressure_angle=20.0,
        starts=1, length=50.0, bore_d=5.5, shaft_flat=3.7,
        clearance=0.25, lead_angle=5.0, journal_d=8.0,
        journal_len=9.0, engage_teeth=6, wheel_width=6.55,
        backlash=0.35, collar_d=10.5, collar_len=2.0,
        bearing_width=7.0, set_screw_d=2.6, set_screw_z=6.0,
        sections=160):
    """Build the journalled printed worm formerly named ``build_worm``.

    ``handsign`` selects a right-hand (``+1``) or left-hand (``-1``)
    thread. ``motor_tail`` controls only the plain negative-Z shaft tail;
    it defaults to ``journal_len`` and has no motor- or housing-specific
    geometry. The bore uses ``bore_d`` with symmetric ``shaft_flat`` flats.

    This function is named ``printed_worm`` at package level because mechlib
    already exposes the distinct general-purpose ``gears.worm`` API.
    """
    if (handsign not in (-1, 1) or module <= 0 or starts < 1 or length <= 0 or
            bore_d <= 0 or shaft_flat <= 0 or clearance < 0 or
            not 0 < lead_angle < 45 or journal_d <= 0 or journal_len < 0 or
            engage_teeth <= 0 or wheel_width <= 0 or backlash < 0 or
            collar_d < journal_d or collar_len <= 0 or bearing_width <= 0 or
            set_screw_d < 0 or sections < 24):
        raise ValueError("printed_worm(): invalid worm, bore, or journal dimensions")
    if motor_tail is None:
        motor_tail = journal_len
    if motor_tail < 0:
        raise ValueError("printed_worm(): motor_tail must be non-negative")

    axial_pitch = math.pi * module
    lead = starts * axial_pitch
    pitch_r = lead / (2.0 * math.pi * math.tan(math.radians(lead_angle)))
    outer_r = pitch_r + module
    root_r = pitch_r - 1.25 * module
    journal_r = journal_d / 2.0
    if root_r <= 0 or journal_r > root_r:
        raise ValueError("printed_worm(): journal or module leaves no thread root")
    thread_len = max(engage_teeth * axial_pitch,
                     wheel_width + 2.0 * axial_pitch)
    pitch_half = 90.0 / starts
    backlash_deg = math.degrees((backlash / 2.0) / pitch_r)
    tip_half = pitch_half - (
        module * math.tan(math.radians(pressure_angle)) / lead) * 360.0
    tip_half -= backlash_deg
    root_half = min(
        178.0,
        pitch_half + (
            1.25 * module * math.tan(math.radians(pressure_angle)) / lead
        ) * 360.0,
    )
    if not 0 < tip_half < pitch_half < root_half:
        raise ValueError("printed_worm(): thread widths collapse for these parameters")

    angles = [j * 360.0 / sections for j in range(sections)]
    base_angles = [math.radians(angle) for angle in angles]
    bore_r = (bore_d + clearance) / 2.0
    flat_half = (shaft_flat + clearance) / 2.0

    def inner_radius(angle):
        sine = abs(math.sin(angle))
        return bore_r if sine < 1e-6 else min(bore_r, flat_half / sine)

    inner = [
        (inner_radius(angle) * math.cos(angle),
         inner_radius(angle) * math.sin(angle))
        for angle in base_angles
    ]
    thread_z0 = length / 2.0 - thread_len / 2.0
    thread_z1 = length / 2.0 + thread_len / 2.0
    runout = outer_r - journal_r
    twist_rate = handsign * 360.0 / lead

    def phase(z):
        clamped = min(max(z, thread_z0 - runout), thread_z1 + runout)
        return twist_rate * (clamped - thread_z0)

    def profile(z, index):
        full = worm_profile(
            angles[index], starts, outer_r, pitch_r, root_r,
            tip_half, pitch_half, root_half)
        if thread_z0 <= z <= thread_z1:
            return full
        fraction = None
        if thread_z0 - runout <= z < thread_z0:
            fraction = (z - (thread_z0 - runout)) / runout
        elif thread_z1 < z <= thread_z1 + runout:
            fraction = 1.0 - (z - thread_z1) / runout
        if fraction is None:
            return journal_r
        base = journal_r + (root_r - journal_r) * fraction
        return base + (full - root_r) * fraction

    span = thread_len + 2.0 * runout
    layers = max(60, int(math.ceil(span / lead * 360.0 / 6.0)))
    heights = [-motor_tail, -motor_tail / 2.0, 0.0]
    heights += [thread_z0 - runout + span * k / layers
                for k in range(layers + 1)]
    heights += [length, length + journal_len]
    body = extrude_twist(
        [(0.0, 0.0)] * sections, inner, heights, phase,
        prof=profile, base_ang=base_angles)

    bearing_station = length / 2.0 + bearing_width / 2.0 + 0.5
    collars = []
    for z in (
            length / 2.0 - bearing_station + bearing_width / 2.0 + 1.0,
            length / 2.0 + bearing_station - bearing_width / 2.0 - 1.0):
        collars.append(cyl(collar_d / 2.0, collar_len, (0.0, 0.0, z)))
    result = uni([body] + collars)
    if set_screw_d > 0:
        cutter = cyl(set_screw_d / 2.0, outer_r * 2.0 + 2.0,
                     center=(0.0, 0.0, set_screw_z), axis="y")
        result = sub(result, cutter)
    result.metadata.update({
        "axial_pitch": axial_pitch,
        "lead": lead,
        "starts": starts,
    })
    return result


def _fw_thread_halfwidths(
        starts=3, module=1.5, pitch_r=4.5, pressure_angle=20.0,
        lead=14.137166941154069, worm_backlash=0.80):
    """Return tip, pitch, and root angular half-widths for the flat worm."""
    span = 360.0 / starts
    backlash_deg = math.degrees((worm_backlash / 2.0) / pitch_r)
    pitch_half = span / 4.0 - backlash_deg
    tip_half = pitch_half - (
        module * math.tan(math.radians(pressure_angle)) / lead) * 360.0
    root_half = min(
        span / 2.0 - 2.0,
        pitch_half + (
            1.25 * module * math.tan(math.radians(pressure_angle)) / lead
        ) * 360.0,
    )
    return tip_half, pitch_half, root_half


def _gen_flat_worm_uncached(
        starts, module, pitch_d, pressure_angle, worm_backlash,
        tip_trim, hand, thread_len, runout, use_695, journal_d,
        seat_len, spacer_d, bearing_spacer_len, collar_d, collar_len,
        length, shaft_d, shaft_flat, clearance, bore_depth,
        assembly_spin, flute_count, flute_d, flute_z0, flute_z1,
        sections):
    lead = starts * math.pi * module
    pitch_r = pitch_d / 2.0
    outer_r = pitch_r + module
    root_r = pitch_r - 1.25 * module
    journal_r = journal_d / 2.0
    collar_r = collar_d / 2.0
    spacer_r = spacer_d / 2.0 if use_695 else journal_r
    tip_half, pitch_half, root_half = _fw_thread_halfwidths(
        starts, module, pitch_r, pressure_angle, lead, worm_backlash)
    if not 0 < tip_half < pitch_half < root_half:
        raise ValueError("flat_worm(): thread widths collapse for these parameters")
    trimmed_outer_r = outer_r - tip_trim
    if not journal_r > 0 or not root_r > 0 or trimmed_outer_r <= pitch_r:
        raise ValueError("flat_worm(): invalid journal, root, or thread-tip radius")

    span_angle = 360.0 / starts

    def thread_profile(theta):
        aa = abs(((theta + span_angle / 2.0) % span_angle) - span_angle / 2.0)
        if aa <= tip_half:
            return trimmed_outer_r
        if aa <= pitch_half:
            return min(
                trimmed_outer_r,
                outer_r + (pitch_r - outer_r) *
                (aa - tip_half) / (pitch_half - tip_half),
            )
        if aa <= root_half:
            return pitch_r + (root_r - pitch_r) * (
                (aa - pitch_half) / (root_half - pitch_half))
        return root_r

    angles = [j * 360.0 / sections for j in range(sections)]
    base_angles = [math.radians(angle) for angle in angles]
    outer_len = seat_len if use_695 else max(0.0, (length - thread_len) / 2.0 - collar_len)
    spacer_len = bearing_spacer_len if use_695 else 0.0
    spacer_a = outer_len + max(0.0, spacer_r - journal_r)
    spacer_b = length - outer_len - max(0.0, spacer_r - journal_r)
    collar_z0 = outer_len + spacer_len
    thread_z0 = collar_z0 + collar_len
    thread_z1 = length - outer_len - spacer_len - collar_len
    collar_z3 = thread_z1 + collar_len
    full_z0 = thread_z0 + runout
    full_z1 = thread_z1 - runout
    if not 0 <= thread_z0 < full_z0 < full_z1 < thread_z1 <= length:
        raise ValueError("flat_worm(): length does not contain the thread and runouts")
    twist_rate = hand * 360.0 / lead

    def phase(z):
        return twist_rate * (min(max(z, thread_z0), thread_z1) - thread_z0)

    def profile(z, index):
        if full_z0 <= z <= full_z1:
            return thread_profile(angles[index])
        if collar_z0 - 1e-9 <= z < thread_z0 or thread_z1 < z <= collar_z3 + 1e-9:
            return collar_r
        fraction = None
        if thread_z0 <= z < full_z0:
            fraction = (z - thread_z0) / runout
        elif full_z1 < z <= thread_z1:
            fraction = 1.0 - (z - full_z1) / runout
        if fraction is None:
            if spacer_len > 0 and spacer_a - 1e-9 <= z <= spacer_b + 1e-9:
                return spacer_r
            return journal_r
        base = collar_r + (root_r - collar_r) * fraction
        return base + (thread_profile(angles[index]) - root_r) * fraction

    span_z = thread_z1 - thread_z0
    layers = max(60, int(math.ceil(span_z / lead * 360.0 / 5.0)))
    before = ([0.0, outer_len, spacer_a, collar_z0 - 1e-6, collar_z0]
              if spacer_len > 0 else [0.0, collar_z0 - 1e-6, collar_z0])
    after = ([collar_z3, collar_z3 + 1e-6, spacer_b,
              length - outer_len, length]
             if spacer_len > 0 else [collar_z3, collar_z3 + 1e-6, length])
    heights = before + [thread_z0 + span_z * k / layers
                        for k in range(layers + 1)] + after
    result = extrude_twist(
        [(0.0, 0.0)] * sections, None, heights, phase,
        prof=profile, base_ang=base_angles)

    bore_r = (shaft_d + clearance) / 2.0
    flat_height = shaft_flat - shaft_d / 2.0 + clearance / 2.0
    bore_poly = sg.Point(0.0, 0.0).buffer(bore_r, resolution=64).intersection(
        sg.box(-bore_r - 1.0, -bore_r - 1.0,
               bore_r + 1.0, flat_height))
    bore = trimesh.creation.extrude_polygon(bore_poly, bore_depth + 1.0)
    bore.apply_translation((0.0, 0.0, -1.0))
    bore.apply_transform(tf.rotation_matrix(
        math.radians(-assembly_spin), (0.0, 0.0, 1.0)))
    cuts = [bore, frustum(bore_r + 0.8, bore_r, 1.2, z0=-0.01)]
    if use_695 and flute_count:
        for index in range(flute_count):
            angle = math.radians(
                240.0 + 60.0 * index if flute_count == 2
                else 180.0 + 360.0 * (index + 0.5) / flute_count)
            flute = cyl(
                flute_d / 2.0, flute_z1 - flute_z0,
                (bore_r * math.cos(angle), bore_r * math.sin(angle),
                 (flute_z0 + flute_z1) / 2.0),
            )
            flute.apply_transform(tf.rotation_matrix(
                math.radians(-assembly_spin), (0.0, 0.0, 1.0)))
            cuts.append(flute)
    result = sub(result, uni(cuts))
    result.metadata.update({
        "axial_pitch": math.pi * module,
        "lead": lead,
        "lead_angle": math.degrees(math.atan2(lead, math.pi * pitch_d)),
        "starts": starts,
    })
    return result


@functools.lru_cache(maxsize=16)
def _gen_flat_worm_cached(*args):
    return _gen_flat_worm_uncached(*args)


def flat_worm(
        starts=3, module=1.5, pitch_d=9.0, pressure_angle=20.0,
        worm_backlash=0.80, tip_trim=0.25, hand=1,
        thread_len=16.0, runout=2.2, use_695=True, journal_d=4.95,
        seat_len=4.4, spacer_d=6.8,
        bearing_spacer_len=5.442135623730952, collar_d=6.8,
        collar_len=0.5, length=36.6842712474619, shaft_d=3.0,
        shaft_flat=2.33, clearance=0.25, bore_depth=8.7,
        assembly_spin=95.0, flute_count=2, flute_d=0.8,
        flute_z0=5.5, flute_z1=8.55, sections=144):
    """Build the bench-proven three-start flat-drive input worm.

    The default 695 layout includes bearing seats, thick torque-tube spacers,
    thrust collars, a blind N20-style D-bore, and two generic glue flutes. The
    expensive mesh is cached only in process; callers receive an independent
    copy so posing or coloring one result cannot mutate later calls.
    """
    if (starts < 1 or module <= 0 or pitch_d <= 0 or worm_backlash < 0 or
            tip_trim < 0 or hand not in (-1, 1) or thread_len <= 0 or
            runout <= 0 or journal_d <= 0 or seat_len < 0 or spacer_d <= 0 or
            bearing_spacer_len < 0 or collar_d <= 0 or collar_len <= 0 or
            length <= 0 or shaft_d <= 0 or shaft_flat <= 0 or clearance < 0 or
            bore_depth <= 0 or flute_count < 0 or flute_d <= 0 or
            flute_z1 <= flute_z0 or sections < 24):
        raise ValueError("flat_worm(): invalid thread, journal, or bore dimensions")
    args = (
        starts, module, pitch_d, pressure_angle, worm_backlash,
        tip_trim, hand, thread_len, runout, use_695, journal_d,
        seat_len, spacer_d, bearing_spacer_len, collar_d, collar_len,
        length, shaft_d, shaft_flat, clearance, bore_depth,
        assembly_spin, flute_count, flute_d, flute_z0, flute_z1,
        sections,
    )
    return _gen_flat_worm_cached(*args).copy()


def worm_wheel_band(
        width=3.5, z0=0.0, wheel_teeth=26, module=1.5,
        pressure_angle=20.0, backlash=0.35, tip_relief=0.03,
        tip_trim=0.55, hand=1, starts=3, pitch_d=9.0,
        read_face=2.4, read_r0=17.0, layers=12):
    """Build the plain helical worm-wheel band paired with ``flat_worm``.

    The transverse involute uses the worm's axial module. Its helical twist is
    the same hand and lead angle as the worm. ``read_face`` thins only the
    outer tooth tips so the source design's narrow optical slot remains usable;
    set it equal to ``width`` to omit that relief.
    """
    if (width <= 0 or wheel_teeth < 3 or module <= 0 or backlash < 0 or
            tip_relief < 0 or tip_trim < 0 or hand not in (-1, 1) or
            starts < 1 or pitch_d <= 0 or read_face <= 0 or read_r0 <= 0 or
            layers < 1):
        raise ValueError("worm_wheel_band(): invalid tooth, face, or relief dimensions")
    lead = starts * math.pi * module
    lead_angle = math.degrees(math.atan2(lead, math.pi * pitch_d))
    wheel_pitch_r = module * wheel_teeth / 2.0
    profile = spur_gear_2d(
        wheel_teeth, module, pa=pressure_angle,
        bl=backlash, t_relief=tip_relief)
    if tip_trim > 0:
        profile = profile.intersection(sg.Point(0.0, 0.0).buffer(
            wheel_pitch_r + module - tip_trim, resolution=256))
    points = list(profile.exterior.coords)[:-1]
    if not sg.LinearRing(points).is_ccw:
        points.reverse()
    rate = hand * math.degrees(
        math.tan(math.radians(lead_angle)) / wheel_pitch_r)
    heights = [z0 + width * k / layers for k in range(layers + 1)]
    band = extrude_twist(points, None, heights, lambda z: rate * (z - z0))

    relief = (width - read_face) / 2.0
    if relief > 0.01:
        outer_r = wheel_pitch_r + module + 1.0
        cuts = []
        for cut_z0 in (z0 - 0.05, z0 + width - relief):
            height = relief + 0.05
            outer = cyl(outer_r, height, (0.0, 0.0, cut_z0 + height / 2.0))
            inner = cyl(read_r0, height + 0.1,
                        (0.0, 0.0, cut_z0 + height / 2.0))
            cuts.append(sub(outer, inner))
        band = sub(band, uni(cuts))
    band.metadata.update({
        "axial_pitch": math.pi * module,
        "lead": lead,
        "lead_angle": lead_angle,
        "pitch_circumference": 2.0 * math.pi * wheel_pitch_r,
        "starts": starts,
        "wheel_teeth": wheel_teeth,
    })
    return band


def worm_coupon(
        starts=3, module=1.5, pitch_d=9.0, pressure_angle=20.0,
        worm_backlash=0.80, worm_tip_trim=0.25, hand=1,
        thread_len=16.0, runout=2.2, use_695=True, journal_d=4.95,
        seat_len=4.4, spacer_d=6.8,
        bearing_spacer_len=5.442135623730952, collar_d=6.8,
        collar_len=0.5, length=36.6842712474619, shaft_d=3.0,
        shaft_flat=2.33, clearance=0.25, bore_depth=8.7,
        assembly_spin=95.0, flute_count=2, flute_d=0.8,
        flute_z0=5.5, flute_z1=8.55, worm_sections=144,
        wheel_teeth=26,
        band_width=3.5, band_backlash=0.35, band_tip_relief=0.03,
        band_tip_trim=0.55, band_read_face=2.4,
        band_read_r0=17.0, band_layers=12):
    """Return a printable worm and short wheel band for bench mesh testing.

    The result is ``{"worm": Trimesh, "wheel_band": Trimesh}`` in each
    part's print frame. It intentionally omits the source project's housing,
    motor pocket, and jig so consumers can add their own fixture or simply
    hold the two inexpensive coupon pieces at the pitch center distance.
    """
    worm_mesh = flat_worm(
        starts=starts, module=module, pitch_d=pitch_d,
        pressure_angle=pressure_angle, worm_backlash=worm_backlash,
        tip_trim=worm_tip_trim, hand=hand, thread_len=thread_len,
        runout=runout, use_695=use_695, journal_d=journal_d,
        seat_len=seat_len, spacer_d=spacer_d,
        bearing_spacer_len=bearing_spacer_len, collar_d=collar_d,
        collar_len=collar_len, length=length, shaft_d=shaft_d,
        shaft_flat=shaft_flat, clearance=clearance,
        bore_depth=bore_depth, assembly_spin=assembly_spin,
        flute_count=flute_count, flute_d=flute_d,
        flute_z0=flute_z0, flute_z1=flute_z1, sections=worm_sections)
    band_mesh = worm_wheel_band(
        width=band_width, wheel_teeth=wheel_teeth, module=module,
        pressure_angle=pressure_angle, backlash=band_backlash,
        tip_relief=band_tip_relief, tip_trim=band_tip_trim,
        hand=hand, starts=starts, pitch_d=pitch_d,
        read_face=band_read_face, read_r0=band_read_r0,
        layers=band_layers)
    return {"worm": worm_mesh, "wheel_band": band_mesh}


def planet_stage(
        module=1.0, sun_teeth=12, planet_teeth=9, ring_teeth=30,
        n_planets=3, pressure_angle=20.0, backlash=0.10,
        ring_clearance=0.25, tip_relief=0.03, face_width=5.0,
        ring_outer_d=34.5, ring_face_width=5.725,
        carrier_radius=13.5, carrier_height=3.0, gear_gap=0.3,
        planet_pin_d=3.4, planet_bore_clearance=0.4,
        pin_extra=0.6, hex_af=7.0, hex_length=6.4,
        shaft_relief_d=6.4, shaft_relief_depth=2.8,
        sun_hub_d=9.0, sun_hub_height=0.8,
        shaft_d=3.0, shaft_flat=2.33, clearance=0.25,
        bore_leadin=1.2):
    """Build an assembled top-loading fixed-ring planetary stage.

    Returns ``{"sun": mesh, "planets": [mesh, ...], "ring": mesh,
    "carrier": mesh}``. The defaults reproduce the source 12T sun, three 9T
    planets, and 30T ring for a 3.5:1 reduction. Parts are returned in their
    assembled coordinates: the carrier's hex output points down, its printed
    pins point up, and the sun, planets, and fixed ring share the gear plane.
    Klonk-specific turret tabs, lid interfaces, and partial mounting flange are
    intentionally omitted.
    """
    if n_planets < 1:
        raise ValueError("planet_stage(): n_planets must be positive")
    if ring_teeth != sun_teeth + 2 * planet_teeth:
        raise ValueError(
            "planet_stage(): ring_teeth must equal sun_teeth + 2 * planet_teeth")
    if (sun_teeth + ring_teeth) % n_planets != 0:
        raise ValueError(
            "planet_stage(): (sun_teeth + ring_teeth) must be divisible by n_planets")
    if (module <= 0 or sun_teeth < 3 or planet_teeth < 3 or
            not 0 < pressure_angle < 45 or
            backlash < 0 or ring_clearance < 0 or tip_relief < 0 or
            face_width <= 0 or ring_outer_d <= 0 or
            ring_face_width <= 0 or carrier_radius <= 0 or carrier_height <= 0 or
            gear_gap < 0 or planet_pin_d <= 0 or planet_bore_clearance < 0 or
            pin_extra < 0 or hex_af <= 0 or hex_length <= 0 or
            shaft_relief_d < 0 or shaft_relief_depth < 0 or sun_hub_d <= 0 or
            sun_hub_height < 0 or shaft_d <= 0 or shaft_flat <= 0 or
            clearance < 0 or bore_leadin < 0):
        raise ValueError("planet_stage(): invalid gear, carrier, or shaft dimensions")

    center_distance = (sun_teeth + planet_teeth) * module / 2.0
    gear_z0 = carrier_height + gear_gap
    sun_profile = spur_gear_2d(
        sun_teeth, module, pa=pressure_angle,
        bl=backlash, t_relief=tip_relief)
    sun = trimesh.creation.extrude_polygon(sun_profile, face_width)
    if sun_hub_height > 0:
        hub = cyl(sun_hub_d / 2.0, sun_hub_height,
                  (0.0, 0.0, face_width + sun_hub_height / 2.0))
        sun = uni([sun, hub])
    bore_r = (shaft_d + clearance) / 2.0
    flat_height = shaft_flat - shaft_d / 2.0 + clearance / 2.0
    bore_poly = sg.Point(0.0, 0.0).buffer(bore_r, resolution=64).intersection(
        sg.box(-bore_r - 1.0, -bore_r - 1.0,
               bore_r + 1.0, flat_height))
    bore = trimesh.creation.extrude_polygon(
        bore_poly, face_width + sun_hub_height + 2.0)
    bore.apply_translation((0.0, 0.0, -1.0))
    cuts = [bore]
    if bore_leadin > 0:
        cuts.append(frustum(
            bore_r, bore_r + 0.8, bore_leadin,
            z0=face_width + sun_hub_height - bore_leadin + 0.01))
    sun = sub(sun, uni(cuts))
    sun.apply_translation((0.0, 0.0, gear_z0))

    planet_profile = spur_gear_2d(
        planet_teeth, module, pa=pressure_angle,
        bl=backlash, t_relief=tip_relief)
    planet_master = trimesh.creation.extrude_polygon(planet_profile, face_width)
    planet_bore = cyl(
        (planet_pin_d + planet_bore_clearance) / 2.0,
        face_width + 2.0, (0.0, 0.0, face_width / 2.0))
    planet_master = sub(planet_master, planet_bore)
    planets = []
    for index in range(n_planets):
        azimuth_deg = 90.0 + 360.0 * index / n_planets
        planet = planet_master.copy()
        planet.apply_transform(tf.rotation_matrix(
            math.radians(
                mesh_phase(sun_teeth, planet_teeth, azimuth_deg)
                + 180.0 / planet_teeth),
            (0.0, 0.0, 1.0),
        ))
        azimuth = math.radians(azimuth_deg)
        planet.apply_translation((
            center_distance * math.cos(azimuth),
            center_distance * math.sin(azimuth),
            gear_z0,
        ))
        planets.append(planet)

    plate = cyl(carrier_radius, carrier_height,
                (0.0, 0.0, carrier_height / 2.0))
    pins = []
    for index in range(n_planets):
        azimuth = math.radians(90.0 + 360.0 * index / n_planets)
        pins.append(cyl(
            planet_pin_d / 2.0, face_width + pin_extra,
            (center_distance * math.cos(azimuth),
             center_distance * math.sin(azimuth),
             carrier_height + (face_width + pin_extra) / 2.0),
        ))
    boss = trimesh.creation.extrude_polygon(hex_poly(hex_af), hex_length)
    boss.apply_translation((0.0, 0.0, -hex_length))
    carrier = uni([plate, boss] + pins)
    if shaft_relief_d > 0 and shaft_relief_depth > 0:
        relief = cyl(
            shaft_relief_d / 2.0, shaft_relief_depth,
            (0.0, 0.0, carrier_height - shaft_relief_depth / 2.0))
        carrier = sub(carrier, relief)

    ring_cutter = affinity.rotate(
        spur_gear_2d(
            ring_teeth, module, pa=pressure_angle, bl=0.0, t_relief=0.0),
        180.0 / ring_teeth, origin=(0.0, 0.0))
    ring_cutter = ring_cutter.buffer(ring_clearance)
    ring_profile = sg.Point(0.0, 0.0).buffer(
        ring_outer_d / 2.0, resolution=128).difference(ring_cutter).buffer(0)
    if ring_profile.is_empty:
        raise ValueError("planet_stage(): ring_outer_d leaves no fixed-ring rim")
    ring = trimesh.creation.extrude_polygon(ring_profile, ring_face_width)
    ring.apply_translation((0.0, 0.0, gear_z0 - 0.1))

    metadata = {
        "ratio": 1.0 + ring_teeth / sun_teeth,
        "sun_teeth": sun_teeth,
        "planet_teeth": planet_teeth,
        "ring_teeth": ring_teeth,
        "n_planets": n_planets,
    }
    for mesh in [sun, ring, carrier] + planets:
        mesh.metadata.update(metadata)
    return {"sun": sun, "planets": planets, "ring": ring, "carrier": carrier}


__all__ = (
    "printed_worm",
    "flat_worm",
    "worm_wheel_band",
    "worm_coupon",
    "planet_stage",
)
