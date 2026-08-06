"""Project-agnostic shaft-coupling generators (Oldham, Cardan, jaw)."""

import math

import shapely.affinity as affinity
import shapely.geometry as sg
from shapely.ops import unary_union
import trimesh
import trimesh.transformations as tf

from .meshutil import sub, uni
from .prim import boxc, cyl, sector2d


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
