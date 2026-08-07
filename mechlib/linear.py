"""Screw-and-spiral linear motion generators.

``scroll_drive`` is the lathe-chuck scroll plate with three self-centering
jaws, ``differential_screw`` turns coarse printed threads into micrometer-fine
travel, and ``archimedes_screw`` is the classic inclined water/grain screw.
"""

import math

import numpy as np
import shapely.geometry as sg
import trimesh
import trimesh.transformations as tf


def _annulus_sector(r_in, r_out, half_span_deg, n=24):
    """Annular-sector polygon centred on +X, from ``r_in`` to ``r_out``."""
    a = math.radians(half_span_deg)
    angles = np.linspace(-a, a, n)
    pts = [(r_out * math.cos(t), r_out * math.sin(t)) for t in angles]
    pts += [(r_in * math.cos(t), r_in * math.sin(t)) for t in angles[::-1]]
    return sg.Polygon(pts).buffer(0)


def scroll_drive(plate_r=30.0, plate_t=4.0, spiral_r0=6.0, spiral_pitch=5.0,
                 turns=4.0, rib_w=3.0, rib_h=4.0, jaw_span_deg=20.0,
                 jaw_h=9.0, face_r=10.0, bore_d=6.0, clearance=0.25,
                 spiral_pts=240):
    """Lathe-chuck scroll plate with three self-centering jaws.

    A raised Archimedean spiral rib (radius ``spiral_r0`` growing by
    ``spiral_pitch`` per turn for ``turns`` turns) stands on a plate of
    ``plate_r``/``plate_t`` (all mm). Each jaw carries arc-segment underside
    teeth that sit in the gaps between consecutive spiral turns; the segments
    are circular arcs (the classic chuck approximation, valid over a modest
    ``jaw_span_deg``). Returns ``{"scroll": mesh, "jaws": [m0, m1, m2]}``
    posed in assembly: jaws at 120-degree spacing with all gripping faces on a
    common circle of radius ``face_r``, so one scroll turn moves every jaw
    radially by ``spiral_pitch``. Radial jaw travel per scroll revolution is
    returned in metadata as ``travel_per_rev`` (mm). ``clearance`` (mm) is the
    radial/axial play between jaw teeth and the rib.
    """
    from .meshutil import extrude_poly_z, sub, uni
    from .prim import cyl

    if (plate_r <= 0 or plate_t <= 0 or spiral_r0 <= 0 or spiral_pitch <= 0 or
            turns < 1.0 or rib_w <= 0 or rib_h <= 0 or
            not 4.0 < jaw_span_deg < 60.0 or jaw_h <= 0 or face_r <= 0 or
            bore_d < 0 or clearance < 0 or spiral_pts < 48):
        raise ValueError("scroll_drive(): invalid scroll or jaw dimensions")
    tooth_w = spiral_pitch - rib_w - 2.0 * clearance
    if tooth_w < 1.2:
        raise ValueError("scroll_drive(): jaw tooth width falls below 1.2 mm")
    rib_end_r = spiral_r0 + spiral_pitch * turns
    if rib_end_r + rib_w / 2.0 > plate_r - 1.0:
        raise ValueError("scroll_drive(): spiral overruns the plate rim")
    if face_r + 0.6 + tooth_w / 2.0 > rib_end_r - spiral_pitch / 2.0:
        raise ValueError("scroll_drive(): face_r leaves no room for jaw teeth")

    theta_max = 2.0 * math.pi * turns
    thetas = np.linspace(0.0, theta_max, spiral_pts)
    radii = spiral_r0 + spiral_pitch * thetas / (2.0 * math.pi)
    spiral = sg.LineString(zip(radii * np.cos(thetas), radii * np.sin(thetas)))
    rib2d = spiral.buffer(rib_w / 2.0, cap_style=2, join_style=1)

    scroll = cyl(plate_r, plate_t, (0.0, 0.0, plate_t / 2.0))
    rib = extrude_poly_z(rib2d, plate_t, plate_t + rib_h)
    scroll = uni([scroll, rib])
    if bore_d > 0:
        scroll = sub(scroll, cyl(bore_d / 2.0, plate_t + 2.0,
                                 (0.0, 0.0, plate_t / 2.0)))

    z_tooth0 = plate_t + clearance
    z_body0 = plate_t + rib_h + clearance
    body_half = jaw_span_deg / 2.0 + 5.0
    jaws = []
    for index in range(3):
        alpha = math.radians(120.0 * index)
        bands = []
        k = 0
        while alpha + 2.0 * math.pi * (k + 1) <= theta_max + 1e-9:
            r_cross = spiral_r0 + spiral_pitch * (alpha / (2.0 * math.pi) + k)
            gap_c = r_cross + spiral_pitch / 2.0
            inner = gap_c - tooth_w / 2.0
            outer = gap_c + tooth_w / 2.0
            if inner >= face_r + 0.6 and outer <= plate_r - 1.0:
                bands.append((inner, outer))
            k += 1
        if not bands:
            raise ValueError("scroll_drive(): no jaw teeth fit this azimuth")
        parts = []
        for inner, outer in bands:
            tooth = _annulus_sector(inner, outer, jaw_span_deg / 2.0)
            parts.append(extrude_poly_z(tooth, z_tooth0, z_body0 + 0.5))
        body = _annulus_sector(face_r, bands[-1][1] + 1.5, body_half)
        parts.append(extrude_poly_z(body, z_body0, z_body0 + jaw_h))
        jaw = uni(parts)
        jaw.apply_transform(tf.rotation_matrix(alpha, (0.0, 0.0, 1.0)))
        jaws.append(jaw)

    metadata = {"spiral_pitch": spiral_pitch, "turns": turns,
                "travel_per_rev": spiral_pitch, "jaws": 3}
    scroll.metadata.update(metadata)
    for jaw in jaws:
        jaw.metadata.update(metadata)
    return {"scroll": scroll, "jaws": jaws}


def differential_screw(d=12.0, p1=2.0, p2=1.75, len1=14.0, len2=14.0,
                       nut_w=20.0, nut_h=10.0, knob_d=16.0, knob_h=4.0,
                       clearance=0.3):
    """Differential screw: one shaft, two same-hand thread sections, two nuts.

    Section 1 (``p1`` mm pitch, ``len1`` mm) runs in the frame nut and
    section 2 (``p2``, ``len2``) in the moving nut, both on a shaft of
    diameter ``d`` (mm). Because the threads are same-hand, one shaft turn
    advances the moving nut by the pitch difference:
    ``travel_per_rev = p1 - p2`` (mm, returned in metadata) — micrometer-fine
    motion from coarse printable threads. The shaft sections reuse
    ``thread_solid`` and the nuts are tapped with matching ``thread_solid``
    cutters at ``clearance`` (mm), so each screw/nut pair shares one thread
    form. Returns ``{"shaft", "nut_frame", "nut_moving"}``
    posed in assembly along +Z with a hand knob below the thread.
    """
    from .mechanisms import tap, thread_solid
    from .meshutil import uni
    from .prim import boxc, cyl

    if (d <= 0 or p1 <= 0 or p2 <= 0 or p1 == p2 or len1 <= 0 or len2 <= 0 or
            nut_w <= d or nut_h <= 0 or knob_d <= 0 or knob_h < 0 or
            clearance < 0):
        raise ValueError("differential_screw(): invalid screw or nut dimensions")
    if nut_h > min(len1, len2):
        raise ValueError("differential_screw(): nut_h exceeds a thread section")

    rod1 = thread_solid(d, len1, pitch=p1)
    rod2 = thread_solid(d, len2, pitch=p2)
    rod2.apply_translation((0.0, 0.0, len1))
    parts = [rod1, rod2]
    if knob_h > 0:
        parts.append(cyl(knob_d / 2.0, knob_h, (0.0, 0.0, -knob_h / 2.0)))
    shaft = uni(parts)

    def nut(z_center, pitch):
        # Snap the nut window to an integer pitch count from its section start
        # so the tapped thread registers with the shaft helix phase.
        s = 0.0 if z_center < len1 else len1
        z0 = s + round((z_center - nut_h / 2.0 - s) / pitch) * pitch
        z0 = min(max(z0, s), s + (len1 if s == 0.0 else len2) - nut_h)
        block = boxc((nut_w, nut_w, nut_h),
                     center=(0.0, 0.0, z0 + nut_h / 2.0))
        return tap(block, d, (0.0, 0.0, z0), nut_h, pitch=pitch,
                   clear=clearance)

    nut_frame = nut(len1 / 2.0, p1)
    nut_moving = nut(len1 + len2 / 2.0, p2)

    metadata = {"pitch1": p1, "pitch2": p2, "travel_per_rev": p1 - p2}
    for mesh in (shaft, nut_frame, nut_moving):
        mesh.metadata.update(metadata)
    return {"shaft": shaft, "nut_frame": nut_frame, "nut_moving": nut_moving}


def archimedes_screw(shaft_r=4.0, flight_r=12.0, lead=14.0, turns=4.0,
                     flight_t=1.6, trough_wall=2.0, clearance=0.3,
                     incline_deg=30.0, seg=64):
    """Archimedes screw: helical flight on a shaft in a half-pipe trough.

    A single flight of thickness ``flight_t`` (mm) sweeps from the shaft to
    ``flight_r`` (mm) with the given ``lead`` (mm per turn) for ``turns``
    turns; the open trough is a halved tube with ``trough_wall`` (mm) wall and
    ``clearance`` (mm) radial play over the flight tips. Returns
    ``{"screw", "trough"}`` posed in assembly, both inclined about the X axis
    by ``incline_deg`` (degrees) like a working water screw. One revolution
    carries each pocket one ``lead`` uphill (metadata ``travel_per_rev``).
    """
    from .mechanisms import helix_solid
    from .meshutil import sub, uni
    from .prim import boxc, cyl

    if (shaft_r <= 0 or flight_r <= shaft_r or lead <= 0 or turns < 1.0 or
            flight_t < 1.2 or trough_wall < 1.2 or clearance < 0 or
            not 0.0 <= incline_deg < 90.0 or seg < 16):
        raise ValueError("archimedes_screw(): invalid screw or trough dimensions")

    length = turns * lead
    prof = np.array([
        (shaft_r - 0.5, -flight_t / 2.0),
        (flight_r, -flight_t / 2.0),
        (flight_r, flight_t / 2.0),
        (shaft_r - 0.5, flight_t / 2.0),
    ])
    flight = helix_solid(prof, lead, turns, seg)
    shaft = cyl(shaft_r, length + 8.0, (0.0, 0.0, length / 2.0))
    screw = uni([flight, shaft])

    r_in = flight_r + clearance
    r_out = r_in + trough_wall
    tube = sub(cyl(r_out, length + 4.0, (0.0, 0.0, length / 2.0)),
               cyl(r_in, length + 8.0, (0.0, 0.0, length / 2.0)))
    upper = boxc((2.0 * r_out + 2.0, r_out + 2.0, length + 10.0),
                 center=(0.0, (r_out + 2.0) / 2.0, length / 2.0))
    trough = sub(tube, upper)

    tilt = tf.rotation_matrix(math.radians(incline_deg), (1.0, 0.0, 0.0))
    screw.apply_transform(tilt)
    trough.apply_transform(tilt)

    metadata = {"lead": lead, "turns": turns, "travel_per_rev": lead,
                "incline_deg": incline_deg}
    screw.metadata.update(metadata)
    trough.metadata.update(metadata)
    return {"screw": screw, "trough": trough}


def swash_plate(plate_r=22.0, plate_t=4.0, tilt_deg=15.0, phase_deg=0.0,
                pistons=4, piston_r=4.0, piston_len=14.0, pitch_r=14.0,
                shaft_d=8.0, shaft_len=30.0, shoe_t=3.0, clearance=0.25,
                sections=48):
    """Build a swash-plate rotary-to-axial converter with piston shoes.

    A disc of ``plate_r`` / ``plate_t`` (mm) is fixed at ``tilt_deg`` to the
    shaft axis and spins with the shaft. ``pistons`` shoes of radius
    ``piston_r`` ride the plate face on a pitch circle of ``pitch_r``; each
    shoe's axial height is ``z = pitch_r * tan(tilt) * cos(azimuth - phase)``
    relative to the plate centre, so one shaft turn drives every shoe through
    a stroke of ``2 * pitch_r * tan(tilt_deg)``. Returns
    ``{"shaft", "plate", "shoes", "stroke"}`` posed at ``phase_deg``.
    Units mm / degrees.
    """
    from .meshutil import uni
    from .prim import cyl

    if (plate_r <= 0 or plate_t < 1.2 or not 1.0 < tilt_deg < 40.0 or
            pistons < 2 or piston_r <= 0 or piston_len < 2.0 or
            pitch_r <= 0 or pitch_r + piston_r > plate_r - 1.0 or
            shaft_d <= 0 or shaft_len < plate_t + 4.0 or shoe_t < 1.2 or
            clearance < 0 or sections < 16):
        raise ValueError("swash_plate(): invalid plate or piston dimensions")
    tilt = math.radians(tilt_deg)
    stroke = 2.0 * pitch_r * math.tan(tilt)
    shaft = cyl(shaft_d / 2.0, shaft_len, center=(0.0, 0.0, 0.0),
                sections=sections)
    plate = cyl(plate_r, plate_t, center=(0.0, 0.0, 0.0), sections=96)
    # A plain disc is continuous-symmetric about its normal, so a pure spin
    # about Z is almost invisible to the gallery's apparent-motion gate.
    # A radial phase mark on the rim makes the rotation readable without
    # changing the plate's kinematics.
    mark = cyl(max(plate_r * 0.08, 1.2), plate_t + 0.4,
               center=(plate_r * 0.72, 0.0, 0.0), sections=24)
    plate = uni([plate, mark])
    # Tilt about X, then spin by phase about Z.
    plate.apply_transform(tf.rotation_matrix(tilt, (1.0, 0.0, 0.0)))
    plate.apply_transform(tf.rotation_matrix(math.radians(phase_deg),
                                             (0.0, 0.0, 1.0)))

    shoes = []
    for k in range(pistons):
        az = 2.0 * math.pi * k / pistons
        # Shoe azimuth in world frame; plate phase already applied so shoe
        # axial position uses the relative angle to the high point.
        rel = az - math.radians(phase_deg)
        z = pitch_r * math.tan(tilt) * math.cos(rel)
        x = pitch_r * math.cos(az)
        y = pitch_r * math.sin(az)
        shoe = cyl(piston_r, shoe_t,
                   center=(x, y, z + plate_t / 2.0 + clearance + shoe_t / 2.0),
                   sections=sections)
        rod = cyl(piston_r * 0.55, piston_len,
                  center=(x, y, z + plate_t / 2.0 + clearance + shoe_t
                          + piston_len / 2.0),
                  sections=sections)
        shoes.append(uni([shoe, rod]))
    metadata = {"tilt_deg": tilt_deg, "pistons": pistons, "stroke": stroke}
    shaft.metadata.update(metadata)
    plate.metadata.update(metadata)
    for shoe in shoes:
        shoe.metadata.update(metadata)
    return {"shaft": shaft, "plate": plate, "shoes": shoes, "stroke": stroke}


def screw_jack(d=12.0, pitch=2.0, travel=20.0, nut_w=22.0, nut_h=12.0,
               base_w=36.0, base_h=6.0, pad_w=28.0, pad_h=5.0,
               lift_frac=0.5, clearance=0.3, knob_d=18.0, knob_h=4.0):
    """Build a screw jack: threaded rod, nut/frame, and lifting pad.

    A printed external thread of diameter ``d`` and ``pitch`` (mm) stands in
    a base nut of across-flat ``nut_w``; a top pad of ``pad_w`` rides the
    screw. ``lift_frac`` in [0, 1] poses the pad between the nut top and the
    full ``travel`` (mm). One turn raises the pad by ``pitch`` (metadata
    ``travel_per_rev``). Returns ``{"base", "screw", "pad"}``. Units mm.
    """
    from .mechanisms import tap, thread_solid
    from .meshutil import uni
    from .prim import boxc, cyl

    if (d <= 0 or pitch <= 0 or travel <= 0 or nut_w <= d or nut_h <= 0 or
            base_w < nut_w or base_h < 1.2 or pad_w <= d or pad_h < 1.2 or
            not 0.0 <= lift_frac <= 1.0 or clearance < 0 or
            knob_d <= 0 or knob_h < 0):
        raise ValueError("screw_jack(): invalid jack dimensions")
    screw_len = nut_h + travel + pad_h + 2.0
    rod = thread_solid(d, screw_len, pitch=pitch)
    parts = [rod]
    if knob_h > 0:
        parts.append(cyl(knob_d / 2.0, knob_h, (0.0, 0.0, -knob_h / 2.0)))
    screw = uni(parts)

    base_block = boxc((base_w, base_w, base_h),
                      center=(0.0, 0.0, base_h / 2.0))
    nut_block = boxc((nut_w, nut_w, nut_h),
                     center=(0.0, 0.0, base_h + nut_h / 2.0))
    base = uni([base_block, nut_block])
    base = tap(base, d, (0.0, 0.0, base_h), nut_h, pitch=pitch,
               clear=clearance)

    pad_z0 = base_h + nut_h + lift_frac * travel
    pad = boxc((pad_w, pad_w, pad_h),
               center=(0.0, 0.0, pad_z0 + pad_h / 2.0))
    pad = tap(pad, d, (0.0, 0.0, pad_z0), pad_h, pitch=pitch,
              clear=clearance)

    metadata = {"pitch": pitch, "travel": travel, "travel_per_rev": pitch,
                "lift_frac": lift_frac}
    for mesh in (base, screw, pad):
        mesh.metadata.update(metadata)
    return {"base": base, "screw": screw, "pad": pad}


def rack_pinion(module=1.5, pinion_teeth=16, rack_teeth=18, width=8.0,
                pressure_angle=20.0, backlash=0.35, bore_d=6.0,
                base_height=None, phase_teeth=0.0, clearance=0.25):
    """Build a spur pinion posed in mesh with a straight rack.

    The pinion is an involute gear of ``pinion_teeth`` at ``module`` (mm);
    the rack has ``rack_teeth`` teeth of the same module. ``phase_teeth``
    (fractional tooth count) rotates the pinion about a fixed centre at
    ``(0, pitch_r)`` and shifts the rack by ``phase_teeth * pi * module``
    along the pitch line (no-slip rolling). Returns
    ``{"pinion", "rack", "travel_per_rev"}`` with the rack pitch line on
    ``y = 0``. Units mm.
    """
    from .gears import rack_2d, spur_gear_mesh
    from .meshutil import extrude_poly_z

    if (module <= 0 or pinion_teeth < 8 or rack_teeth < 4 or width < 1.2 or
            not 14.0 <= pressure_angle <= 30.0 or backlash < 0 or
            bore_d < 0 or clearance < 0):
        raise ValueError("rack_pinion(): invalid rack or pinion dimensions")
    pitch_r = module * pinion_teeth / 2.0
    travel_per_rev = math.pi * module * pinion_teeth
    travel = phase_teeth * math.pi * module
    # CW pinion (negative about +Z) drives the rack toward -X so the pitch
    # lines roll without slip; see bottom-point velocity of a pinion above
    # the rack.
    angle_deg = -phase_teeth * (360.0 / pinion_teeth)

    pinion = spur_gear_mesh(pinion_teeth, module, width, bore_d=bore_d,
                            pa=pressure_angle, bl=backlash)
    # Fixed-axis drive: rotate about the pinion centre, park it at x = 0.
    pinion.apply_transform(tf.rotation_matrix(math.radians(angle_deg),
                                              (0.0, 0.0, 1.0)))
    pinion.apply_translation((0.0, pitch_r, 0.0))

    rack_kwargs = dict(n_teeth=rack_teeth, module=module,
                       pa_deg=pressure_angle, backlash=backlash)
    if base_height is not None:
        rack_kwargs["base_height"] = base_height
    rack_poly = rack_2d(**rack_kwargs)
    rack = extrude_poly_z(rack_poly, 0.0, width)
    # Rigid shift (not a re-boolean) so phase sweeps recover as pure
    # translation. Opposite the pinion's CW rotation: rack travels -X.
    if travel:
        rack.apply_translation((-travel, 0.0, 0.0))

    metadata = {"module": module, "pinion_teeth": pinion_teeth,
                "travel_per_rev": travel_per_rev, "phase_teeth": phase_teeth}
    pinion.metadata.update(metadata)
    rack.metadata.update(metadata)
    return {"pinion": pinion, "rack": rack,
            "travel_per_rev": travel_per_rev}


__all__ = (
    "scroll_drive",
    "differential_screw",
    "archimedes_screw",
    "swash_plate",
    "screw_jack",
    "rack_pinion",
)
