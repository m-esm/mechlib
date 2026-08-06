"""Project-agnostic kinematic-pair generators (spherical, hinge, gimbal)."""

import math

import numpy as np
import shapely.geometry as sg
from shapely.ops import unary_union
import trimesh
import trimesh.transformations as tf

from .cutters import teardrop
from .meshutil import from_manifold, sub, to_manifold, uni
from .prim import boxc, cyl, frustum, sector2d, seg_cylinder


def _sphere(radius, subdiv):
    """Return an icosphere of the given radius (same topology for a given subdiv)."""
    return trimesh.creation.icosphere(subdivisions=subdiv, radius=radius)


def _subdiv(sections):
    """Map a circle segment count onto an icosphere subdivision level."""
    return max(2, min(4, int(round(math.log(max(sections, 16) / 8.0, 2.0)))))


def _yz_extrude(poly, width, z_axis):
    """Extrude a (y, z-z_axis) profile polygon along X, centred on X=0."""
    mesh = trimesh.creation.extrude_polygon(poly, width)
    if not mesh.is_watertight:
        mesh = from_manifold(to_manifold(mesh))
    swap = np.eye(4)
    swap[:3, :3] = [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
    mesh.apply_transform(swap)
    mesh.apply_translation((-width / 2.0, 0.0, z_axis))
    return mesh


def ball_socket_joint(ball_d=10.0, stem_d=5.0, capture_deg=20.0, clear=0.3,
                      neck_deg=40.0, wall=2.5, fingers=4, slot_w=1.2,
                      slot_over=3.0, lip_h=1.6, lead_deg=45.0, stem_len=10.0,
                      base_d=12.0, base_h=2.5, shank_d=8.0, shank_len=6.0,
                      pose_deg=0.0, sections=64):
    """Build a snap-together spherical joint (ball stud plus split socket).

    Returns ``{"ball", "socket"}`` posed in assembly with the ball centre at
    the origin, the stud pointing down -Z and the socket shank up +Z. The
    socket cavity is a sphere of ``ball_d/2 + clear`` whose mouth is pinched to
    ``ball_d/2 * cos(capture_deg)``, so the lip wraps ``capture_deg`` past the
    ball equator and retains the ball with an undercut of
    ``ball_d/2 * (1 - cos(capture_deg))``: that undercut alone sets the pull-out
    force. ``fingers`` axial slots split the lip so it can spread over the ball
    on assembly, and a ``lead_deg`` flare under the mouth is the lead-in that
    turns insertion into a wedge action instead of a square-edged impact. The
    ball is carried on a stem through a 45 degree neck cone that meets the
    sphere ``neck_deg`` below the equator, so nothing prints on a single-point
    pole; the stem swinging into the mouth rim is what limits the cone of
    motion, reported as ``swing_half_deg`` in the metadata (3 rotational DOF,
    free spin about the stem, ``2 * swing_half_deg`` of included swing).

    Print the socket MOUTH UP and the stud STEM DOWN, both without support:
    the cavity roof never exceeds ``lip_overhang_deg`` from vertical, the neck
    cone and the shank flare are both 45 degrees, and the base disc gives the
    stud a footprint. In that orientation the fingers are vertical cantilevers,
    so their roots see interlayer tension on the snap; the slots end in a round
    relief to keep the crack from running, and ``snap_strain_pct`` estimates
    the peak root strain (keep it under about 3 percent for PLA, more for
    PETG). Printing the socket on its side puts the finger bending in-plane
    instead, at the cost of supporting the cavity. Not print-in-place: the two
    parts snap together after printing. Units are mm and degrees.
    """
    if (ball_d <= 0 or stem_d <= 0 or clear <= 0 or wall < 1.2 or
            fingers < 2 or slot_w < 0.4 or slot_over <= 0 or lip_h < 0.8 or
            stem_len <= 0 or base_h < 0 or shank_len < 0 or sections < 24):
        raise ValueError("ball_socket_joint(): invalid joint dimensions")
    if not 3.0 <= capture_deg <= 45.0:
        raise ValueError("ball_socket_joint(): capture_deg must be 3-45 deg")
    if not capture_deg + 3.0 <= neck_deg <= 45.0:
        raise ValueError(
            "ball_socket_joint(): neck_deg must be capture_deg+3 .. 45 deg so "
            "the lip bears on the sphere and the neck cone stays inside it")
    if not 20.0 <= lead_deg <= 70.0:
        raise ValueError("ball_socket_joint(): lead_deg must be 20-70 deg")

    ball_r = ball_d / 2.0
    stem_r = stem_d / 2.0
    cav_r = ball_r + clear
    mouth_r = ball_r * math.cos(math.radians(capture_deg))
    undercut = ball_r - mouth_r
    neck_r = ball_r * math.cos(math.radians(neck_deg))
    if stem_r + 0.4 > mouth_r:
        raise ValueError("ball_socket_joint(): stem too fat for the mouth")
    if neck_r < stem_r + 0.4:
        raise ValueError("ball_socket_joint(): stem too fat for the neck")
    if undercut < 0.05:
        raise ValueError(
            "ball_socket_joint(): capture undercut below 0.05 mm; the socket "
            "would not retain the ball")
    lip_overhang = math.degrees(math.acos(min(1.0, mouth_r / cav_r)))
    if lip_overhang > 50.0:
        raise ValueError(
            "ball_socket_joint(): lip overhang %.0f deg needs support; lower "
            "capture_deg" % lip_overhang)
    swing_half = math.degrees(math.asin(min(1.0, mouth_r / cav_r))
                              - math.asin(min(1.0, stem_r / cav_r)))
    if swing_half <= 0.0:
        raise ValueError("ball_socket_joint(): geometry leaves no swing")
    if abs(pose_deg) > swing_half + 1e-9:
        raise ValueError(
            "ball_socket_joint(): pose_deg %.1f exceeds the %.1f deg swing"
            % (pose_deg, swing_half))

    out_r = cav_r + wall
    shank_r = shank_d / 2.0
    if shank_len > 0 and shank_r > out_r - 0.8:
        raise ValueError("ball_socket_joint(): shank as wide as the cup")
    z_rim = -math.sqrt(max(cav_r * cav_r - mouth_r * mouth_r, 0.0))
    z_bot = z_rim - lip_h
    flare_r = mouth_r + lip_h * math.tan(math.radians(lead_deg))
    if flare_r > out_r - 0.8:
        raise ValueError(
            "ball_socket_joint(): lead-in flare eats the lip wall; reduce "
            "lip_h or lead_deg, or raise wall")
    if slot_over > 0.8 * cav_r:
        raise ValueError("ball_socket_joint(): slots reach into the cup roof")
    if fingers * slot_w > math.pi * (cav_r + wall):
        raise ValueError("ball_socket_joint(): slots consume the lip")
    fingers = int(round(fingers))
    sections = int(round(sections))
    subdiv = _subdiv(sections)

    # --- ball stud -------------------------------------------------------
    neck_z = -ball_r * math.sin(math.radians(neck_deg))
    neck_h = neck_r - stem_r
    stem_top = neck_z - neck_h
    stud = [_sphere(ball_r, subdiv)]
    if neck_h > 1e-6:
        stud.append(frustum(stem_r, neck_r, neck_h, z0=stem_top,
                            sections=sections))
    stud.append(cyl(stem_r, stem_len,
                    center=(0.0, 0.0, stem_top - stem_len / 2.0),
                    sections=sections))
    base_z = stem_top - stem_len
    if base_h > 0:
        if base_d < stem_d:
            raise ValueError("ball_socket_joint(): base smaller than the stem")
        stud.append(cyl(base_d / 2.0, base_h,
                        center=(0.0, 0.0, base_z - base_h / 2.0),
                        sections=sections))
        base_z -= base_h
    ball = uni(stud)
    if pose_deg:
        ball.apply_transform(tf.rotation_matrix(
            math.radians(pose_deg), (1.0, 0.0, 0.0)))

    # --- socket ----------------------------------------------------------
    z_shoulder = cav_r + wall
    body = [cyl(out_r, z_shoulder - z_bot,
                center=(0.0, 0.0, (z_shoulder + z_bot) / 2.0),
                sections=sections)]
    z_top = z_shoulder
    if shank_len > 0:
        flare_h = out_r - shank_r
        body.append(frustum(out_r, shank_r, flare_h, z0=z_shoulder,
                            sections=sections))
        body.append(cyl(shank_r, shank_len,
                        center=(0.0, 0.0, z_shoulder + flare_h
                                + shank_len / 2.0),
                        sections=sections))
        z_top = z_shoulder + flare_h + shank_len
    socket = uni(body)
    cavity = uni([
        _sphere(cav_r, subdiv),
        frustum(mouth_r + (lip_h + 1.0) * math.tan(math.radians(lead_deg)),
                mouth_r, lip_h + 1.0, z0=z_bot - 1.0, sections=sections),
    ])
    socket = sub(socket, cavity)

    slot_z0 = z_bot - 1.0
    slot_r0 = cav_r - 1.0
    slot_r1 = out_r + 1.0
    knives = []
    for index in range(fingers):
        angle = 2.0 * math.pi * index / fingers
        knife = boxc((slot_r1 - slot_r0, slot_w, slot_over - slot_z0),
                     center=((slot_r1 + slot_r0) / 2.0, 0.0,
                             (slot_over + slot_z0) / 2.0))
        knife.apply_transform(tf.rotation_matrix(angle, (0.0, 0.0, 1.0)))
        knives.append(knife)
        knives.append(seg_cylinder(
            (slot_r0 * math.cos(angle), slot_r0 * math.sin(angle), slot_over),
            (slot_r1 * math.cos(angle), slot_r1 * math.sin(angle), slot_over),
            slot_w * 1.6))
    socket = sub(socket, uni(knives))

    finger_len = slot_over - z_bot
    strain = 300.0 * wall * undercut / (2.0 * finger_len * finger_len)
    meta = {
        "ball_d": ball_d,
        "mouth_d": 2.0 * mouth_r,
        "capture_deg": capture_deg,
        "undercut": undercut,
        "clear": clear,
        "swing_half_deg": swing_half,
        "swing_cone_deg": 2.0 * swing_half,
        "lip_overhang_deg": lip_overhang,
        "snap_strain_pct": strain,
        "pose_deg": pose_deg,
        "stud_height": z_rim - base_z,
        "socket_height": z_top - z_bot,
    }
    ball.metadata.update(meta)
    socket.metadata.update(meta)
    return {"ball": ball, "socket": socket}


def knuckle_hinge(leaf_w=36.0, leaf_len=16.0, leaf_t=3.0, knuckles=5,
                  pin_d=3.0, barrel_d=7.0, gap=0.3, stop_deg=90.0,
                  open_deg=110.0, stop_h=2.0, friction=0.0, sections=64):
    """Build a print-in-place knuckle hinge with an integral hard stop.

    Returns ``{"leaf_a", "leaf_b"}``. ``knuckles`` barrels alternate along the
    pin axis (X) around one continuous printed pin that is integral with leaf A
    and runs free inside leaf B's teardrop bores, so the pair prints as one
    piece and breaks loose on the first swing. Leaf A lies along -Y, leaf B
    along +Y, both flat in Z at ``open_deg=180``; ``open_deg`` is the included
    angle between the leaves, reduced by rotating leaf B about the pin. Each
    leaf carries a full-width collar sector around the barrel whose radial
    flank is a plane through the pin axis, half of ``stop_deg`` each side, so
    the two flanks meet face to face and arrest the leaf at exactly
    ``stop_deg`` of included angle. ``friction`` is a diametral interference
    band raised on the pin inside every leaf-B knuckle: the first rotation
    shears it and what is left holds the leaf where you put it (0.05-0.15 mm is
    a useful band; more welds the joint solid).

    Print flat with ``open_deg=180`` and no support: the pin bores are
    teardrops with the apex up, the collar flanks rise at most 90 degrees from
    the pin axis, and the barrels sit tangent to the bed, which leaves the
    usual small first-layer flat on the underside of each knuckle. The pin
    bore is ``pin_d + 2 * gap`` and the knuckles are spaced ``gap`` apart
    axially, so ``gap`` is the only fit dimension that matters. Units are mm
    and degrees.
    """
    if (leaf_w <= 0 or leaf_len <= 0 or leaf_t < 1.2 or pin_d <= 0 or
            barrel_d <= 0 or gap <= 0 or stop_h < 0.8 or friction < 0 or
            sections < 24):
        raise ValueError("knuckle_hinge(): invalid hinge dimensions")
    knuckles = int(round(knuckles))
    if knuckles < 3:
        raise ValueError("knuckle_hinge(): need at least 3 knuckles")
    if not 10.0 <= stop_deg <= 175.0:
        raise ValueError("knuckle_hinge(): stop_deg must be 10-175 deg")
    if not stop_deg - 1e-9 <= open_deg <= 180.0 + 1e-9:
        raise ValueError(
            "knuckle_hinge(): open_deg must lie between stop_deg and 180")
    pin_r = pin_d / 2.0
    bore_r = pin_r + gap
    barrel_r = barrel_d / 2.0
    if barrel_r < 1.42 * bore_r + 0.8:
        raise ValueError(
            "knuckle_hinge(): barrel too small for a teardrop bore and wall")
    if leaf_t >= barrel_d:
        raise ValueError("knuckle_hinge(): leaf thicker than the barrel")
    if leaf_t / 2.0 >= barrel_r:
        raise ValueError("knuckle_hinge(): leaf does not sit under the pin")
    collar_r = barrel_r + stop_h
    if leaf_len < collar_r + 2.0:
        raise ValueError("knuckle_hinge(): leaf shorter than its own collar")
    if friction > 0 and bore_r + friction / 2.0 > barrel_r - 0.8:
        raise ValueError("knuckle_hinge(): friction band bursts the knuckle")
    knuckle_w = (leaf_w - (knuckles - 1) * gap) / knuckles
    if knuckle_w < 2.0:
        raise ValueError(
            "knuckle_hinge(): knuckles below 2 mm wide; widen the leaf or "
            "use fewer knuckles")
    sections = int(round(sections))

    z_axis = barrel_r
    half = stop_deg / 2.0
    merge = math.degrees(math.asin((z_axis - leaf_t / 2.0) / collar_r))
    barrel = sg.Point(0.0, 0.0).buffer(barrel_r, resolution=sections // 4)
    profile_a = unary_union([
        sg.box(-leaf_len, -z_axis, 0.0, -z_axis + leaf_t),
        sector2d(180.0 - half, 180.0 + merge, collar_r, n=sections // 2),
        barrel,
    ]).buffer(0)
    profile_b = unary_union([
        sg.box(0.0, -z_axis, leaf_len, -z_axis + leaf_t),
        sector2d(-merge, half, collar_r, n=sections // 2),
        barrel,
    ]).buffer(0)

    starts = [-leaf_w / 2.0 + index * (knuckle_w + gap)
              for index in range(knuckles)]
    cuts = [[], []]
    for index, x0 in enumerate(starts):
        owner = index % 2
        cuts[1 - owner].append(cyl(
            barrel_r + gap, knuckle_w + 2.0 * gap,
            center=(x0 + knuckle_w / 2.0, 0.0, z_axis), axis="x",
            sections=sections))

    leaf_a = sub(_yz_extrude(profile_a, leaf_w, z_axis), uni(cuts[0]))
    leaf_b = sub(_yz_extrude(profile_b, leaf_w, z_axis), uni(cuts[1]))

    pin = [cyl(pin_r, leaf_w, center=(0.0, 0.0, z_axis), axis="x",
               sections=sections)]
    band_w = min(1.2, 0.5 * knuckle_w)
    if friction > 0:
        for index, x0 in enumerate(starts):
            if index % 2 == 0:
                continue
            pin.append(cyl(bore_r + friction / 2.0, band_w,
                           center=(x0 + knuckle_w / 2.0, 0.0, z_axis),
                           axis="x", sections=sections))
    leaf_a = uni([leaf_a] + pin)
    bore = teardrop(bore_r, leaf_w + 2.0, axis="x", up=(0.0, 0.0, 1.0))
    bore.apply_translation((0.0, 0.0, z_axis))
    leaf_b = sub(leaf_b, bore)

    swing = 180.0 - open_deg
    if swing:
        leaf_b.apply_transform(tf.rotation_matrix(
            math.radians(swing), (1.0, 0.0, 0.0), (0.0, 0.0, z_axis)))

    meta = {
        "pin_d": pin_d,
        "bore_d": 2.0 * bore_r,
        "gap": gap,
        "knuckles": knuckles,
        "knuckle_w": knuckle_w,
        "stop_deg": stop_deg,
        "open_deg": open_deg,
        "travel_deg": 180.0 - stop_deg,
        "collar_d": 2.0 * collar_r,
        "interference": friction,
        "pin_axis_z": z_axis,
    }
    leaf_a.metadata.update(meta)
    leaf_b.metadata.update(meta)
    return {"leaf_a": leaf_a, "leaf_b": leaf_b}


def gimbal_rings(rings=3, outer_d=44.0, ring_w=5.0, ring_t=6.5, gap=0.3,
                 pin_d=2.5, pin_len=3.0, tilt_deg=0.0, sections=64):
    """Build nested print-in-place gimbal rings on alternating axes.

    Returns ``{"ring_0", ... }`` from the outside in, all coplanar in the
    printed pose with the common pivot centre at the origin. Ring 0 is the
    frame; every inner ring hangs from two integral trunnion pins that run in
    teardrop sockets in the ring outside it, with the pin axes alternating X,
    Y, X, so two rings are a Cardan suspension and three a full gimbal.
    ``tilt_deg`` rotates each ring about its own trunnion axis relative to its
    parent, cumulatively, purely to pose the assembly: the rings are rigid
    bodies and nothing limits the rotation, so any angle is legal.

    Each ring radius is derived, not stacked: the inner ring is sized so that
    its top and bottom outer corners stay ``gap`` clear of the parent's bore
    through the whole swing, which is why the mid-plane radial gap reads wider
    than ``gap`` at zero tilt and reaches ``gap`` at ``crit_tilt_deg``. Pin to
    socket clearance is ``gap`` everywhere.

    Print flat, one piece, no support: every socket is a teardrop with the
    apex up, and ``ring_t`` is checked against the teardrop peak so the socket
    never breaks through the ring's top face. The trunnions themselves are
    short horizontal stubs whose undersides bridge across the socket, so
    expect a little droop on the pin's lower quadrant; the teardrop roof is
    what keeps that droop from binding. Units are mm and degrees.
    """
    rings = int(round(rings))
    if rings < 2:
        raise ValueError("gimbal_rings(): need at least 2 rings")
    if (outer_d <= 0 or ring_w < 1.2 or ring_t < 1.2 or gap <= 0 or
            pin_d <= 0 or pin_len <= 0 or sections < 24):
        raise ValueError("gimbal_rings(): invalid ring dimensions")
    pin_r = pin_d / 2.0
    socket_r = pin_r + gap
    if ring_t / 2.0 < 1.42 * socket_r + 0.6:
        raise ValueError(
            "gimbal_rings(): ring_t too thin for a teardrop socket and its "
            "roof; raise ring_t or drop pin_d")
    sections = int(round(sections))

    radii = [outer_d / 2.0]
    for _index in range(rings - 1):
        bore = radii[-1] - ring_w
        reach = bore - gap
        inner = reach * reach - (ring_t / 2.0) ** 2
        if inner <= 0.0:
            raise ValueError("gimbal_rings(): rings run out of room")
        radii.append(math.sqrt(inner))
    if radii[-1] - ring_w < 1.0:
        raise ValueError(
            "gimbal_rings(): the innermost ring closes up; use fewer rings, "
            "a narrower ring_w, or a bigger outer_d")
    for index in range(rings - 1):
        depth = pin_len + gap - (radii[index] - ring_w - radii[index + 1])
        if depth <= 0.4:
            raise ValueError("gimbal_rings(): pins too short to reach")
        if depth > ring_w - 1.2:
            raise ValueError(
                "gimbal_rings(): trunnion socket leaves no wall; shorten "
                "pin_len or widen ring_w")

    axes = ["fixed"] + ["x" if index % 2 else "y"
                        for index in range(1, rings)]
    crit = max(math.degrees(math.atan2(ring_t / 2.0, radii[index]))
               for index in range(1, rings))

    parts = {}
    transform = np.eye(4)
    for index in range(rings):
        outer = radii[index]
        body = [trimesh.creation.annulus(r_min=outer - ring_w, r_max=outer,
                                         height=ring_t, sections=sections)]
        if index:
            direction = (1.0, 0.0, 0.0) if axes[index] == "x" else (0.0, 1.0, 0.0)
            root = pin_len + 0.8
            for sign in (1.0, -1.0):
                pin = uni([
                    cyl(pin_r, root - 0.6,
                        center=(0.0, 0.0, (root - 0.6) / 2.0),
                        sections=sections),
                    frustum(pin_r, pin_r - 0.6, 0.6, z0=root - 0.6,
                            sections=sections),
                ])
                pin.apply_transform(tf.rotation_matrix(
                    math.pi / 2.0, (0.0, 1.0, 0.0) if axes[index] == "x"
                    else (-1.0, 0.0, 0.0)))
                if sign < 0:
                    pin.apply_transform(tf.rotation_matrix(
                        math.pi, (0.0, 0.0, 1.0)))
                pin.apply_translation(
                    np.asarray(direction) * sign * (outer - 0.8))
                body.append(pin)
        ring = uni(body)
        if index + 1 < rings:
            child = radii[index + 1]
            child_axis = axes[index + 1]
            direction = np.asarray((1.0, 0.0, 0.0) if child_axis == "x"
                                   else (0.0, 1.0, 0.0))
            length = pin_len + gap + 1.0
            holes = []
            for sign in (1.0, -1.0):
                hole = teardrop(socket_r, length, axis=child_axis,
                                up=(0.0, 0.0, 1.0))
                hole.apply_translation(
                    direction * sign * (child - 1.0 + length / 2.0))
                holes.append(hole)
            ring = sub(ring, uni(holes))
        meta = {
            "ring_index": index,
            "pivot_axis": axes[index],
            "outer_d": 2.0 * outer,
            "inner_d": 2.0 * (outer - ring_w),
            "gap": gap,
            "rings": rings,
            "crit_tilt_deg": crit,
            "tilt_deg": tilt_deg,
            "ring_radii": tuple(radii),
            "axes": tuple(axes),
        }
        ring.metadata.update(meta)
        if index:
            step = tf.rotation_matrix(
                math.radians(tilt_deg),
                (1.0, 0.0, 0.0) if axes[index] == "x" else (0.0, 1.0, 0.0))
            transform = transform @ step
        if index:
            ring.apply_transform(transform)
        parts["ring_%d" % index] = ring
    return parts


__all__ = (
    "ball_socket_joint",
    "knuckle_hinge",
    "gimbal_rings",
)
