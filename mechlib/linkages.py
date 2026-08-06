"""Planar linkage generators: four-bar kits, toggle clamp, Scotch yoke, quick-return.

Every generator returns a dict of named watertight meshes posed in assembly
(``planet_stage`` precedent) plus the solved joint coordinates. Links are
flat-printable capsules with pin bores; joints are printed pins standing
through the bores. Kinematic poses are solved by circle-circle intersection
and raise ``ValueError`` when the requested pose is unreachable.
"""

import math

import numpy as np
import shapely.geometry as sg
import trimesh
import trimesh.transformations as tf

from .cutters import teardrop
from .meshutil import largest_poly, sub, uni
from .prim import cyl


def _extrude(poly, z0, z1):
    """Extrude ``poly`` between world Z planes, closing open edges via manifold3d."""
    h = float(z1 - z0)
    if h <= 0:
        raise ValueError("extrusion height must be positive")
    mesh = trimesh.creation.extrude_polygon(poly, h)
    if not mesh.is_watertight:
        from .meshutil import from_manifold, to_manifold
        mesh = from_manifold(to_manifold(mesh))
    if z0:
        mesh.apply_translation((0.0, 0.0, z0))
    return mesh


def _capsule_2d(p0, p1, width):
    line = sg.LineString([(float(p0[0]), float(p0[1])),
                          (float(p1[0]), float(p1[1]))])
    return line.buffer(width / 2.0, resolution=48, cap_style=1, join_style=1)


def _bar_mesh(p0, p1, width, z0, z1, hole_d=0.0, holes=()):
    """Extrude a capsule bar from ``p0`` to ``p1`` and bore it at ``holes``."""
    bar = _capsule_2d(p0, p1, width)
    for hole in holes:
        bar = bar.difference(
            sg.Point(float(hole[0]), float(hole[1])).buffer(
                hole_d / 2.0, resolution=48))
    return _extrude(bar.buffer(0), z0, z1)


def _pin_xy(point, z0, z1, d, sections=48):
    """Vertical printed pivot pin centred on the XY ``point`` (mm)."""
    return cyl(d / 2.0, z1 - z0,
               center=(float(point[0]), float(point[1]), (z0 + z1) / 2.0),
               sections=sections)


def _circle_circle(c0, r0, c1, r1, branch, caller):
    """Intersect circle(c0, r0) with circle(c1, r1); ``branch`` picks the side."""
    dvec = np.asarray(c1, float) - np.asarray(c0, float)
    d = float(np.hypot(dvec[0], dvec[1]))
    if d < 1e-9:
        raise ValueError("%s: joint centres coincide" % caller)
    a = (r0 * r0 - r1 * r1 + d * d) / (2.0 * d)
    h2 = r0 * r0 - a * a
    if h2 < -1e-9:
        raise ValueError(
            "%s: pose unreachable (link circles do not intersect)" % caller)
    u = dvec / d
    mid = np.asarray(c0, float) + a * u
    perp = np.array([-u[1], u[0]])
    return mid + math.sqrt(max(h2, 0.0)) * (float(branch) * perp)


def link_bar(length, width, thickness, bore_d, clearance=0.25):
    """Flat-printable linkage bar: capsule profile with a pin bore at each end.

    ``length`` (mm) is the bore-centre distance — the kinematic link length;
    the overall bar length is ``length + width``. Bores are sized
    ``bore_d + clearance`` (mm) so a printed pin of ``bore_d`` runs free.
    """
    if (length <= 0 or width < 2.4 or thickness < 1.2 or bore_d <= 0 or
            clearance < 0 or bore_d + clearance >= width):
        raise ValueError("link_bar(): invalid bar dimensions")
    return _bar_mesh((0.0, 0.0), (length, 0.0), width, 0.0, thickness,
                     hole_d=bore_d + clearance,
                     holes=((0.0, 0.0), (length, 0.0)))


def four_bar_pose(l_ground, l_crank, l_coupler, l_rocker, crank_angle_deg,
                  branch=1):
    """Solve the four-bar assembly pose for one crank angle.

    Ground pivots sit at ``O1 = (0, 0)`` and ``O2 = (l_ground, 0)`` (mm).
    The crank pin ``A`` rides a circle of ``l_crank`` about ``O1`` at
    ``crank_angle_deg`` (degrees, CCW from +X); the moving pivot ``B`` is the
    circle-circle intersection of the coupler (``l_coupler`` about ``A``) and
    the rocker (``l_rocker`` about ``O2``). ``branch`` selects the open (+1)
    or crossed (-1) assembly. Raises ``ValueError`` when the pose is
    unreachable. Returns ``{"O1", "O2", "A", "B"}`` as (x, y) tuples in mm.
    """
    if min(l_ground, l_crank, l_coupler, l_rocker) <= 0:
        raise ValueError("four_bar_pose(): link lengths must be positive")
    if branch not in (1, -1):
        raise ValueError("four_bar_pose(): branch must be +1 or -1")
    th = math.radians(crank_angle_deg)
    A = np.array([l_crank * math.cos(th), l_crank * math.sin(th)])
    O2 = np.array([l_ground, 0.0])
    B = _circle_circle(A, l_coupler, O2, l_rocker, branch, "four_bar_pose()")
    return {"O1": (0.0, 0.0), "O2": (float(l_ground), 0.0),
            "A": (float(A[0]), float(A[1])), "B": (float(B[0]), float(B[1]))}


def four_bar(l_ground=25.0, l_crank=12.5, l_coupler=25.0, l_rocker=25.0,
             crank_angle_deg=60.0, branch=1, coupler_ext=0.0,
             width=6.0, thickness=4.0, bore_d=3.0, clearance=0.25,
             pin_extra=2.0):
    """Build a flat-printable four-bar linkage kit posed in assembly.

    Returns ``{"ground", "crank", "coupler", "rocker", "pins", "joints"}``;
    ``pins`` are the four printed pivot pins at the solved joints and
    ``joints`` is the ``four_bar_pose`` dict. The links are stacked one
    thickness apart (ground lowest, then rocker, coupler, crank) so the
    assembled parts never intersect. ``coupler_ext`` (mm) extends the coupler
    bar past the rocker joint and adds a ``"trace"`` boss at its tip — the
    coupler-curve tracer point. The defaults (ground 2 : crank 1 : coupler
    2.5 : rocker 2.5 with a 2.5 extension) are the Hoecken straight-line
    proportions. Raises ``ValueError`` for unreachable poses.
    """
    if (width < 2.4 or thickness < 1.2 or bore_d <= 0 or clearance < 0 or
            bore_d + clearance >= width or coupler_ext < 0 or pin_extra < 0):
        raise ValueError("four_bar(): invalid link or pin dimensions")
    joints = four_bar_pose(l_ground, l_crank, l_coupler, l_rocker,
                           crank_angle_deg, branch)
    O1, O2, A, B = (joints[key] for key in ("O1", "O2", "A", "B"))
    hole_d = bore_d + clearance
    t = thickness
    direction = np.asarray(B, float) - np.asarray(A, float)
    direction /= np.linalg.norm(direction)
    tip = np.asarray(A, float) + (l_coupler + coupler_ext) * direction
    parts = {
        "ground": _bar_mesh(O1, O2, width, 0.0, t, hole_d, (O1, O2)),
        "rocker": _bar_mesh(O2, B, width, t, 2.0 * t, hole_d, (O2, B)),
        "coupler": _bar_mesh(A, tip, width, 2.0 * t, 3.0 * t, hole_d, (A, B)),
        "crank": _bar_mesh(O1, A, width, 3.0 * t, 4.0 * t, hole_d, (O1, A)),
        "pins": tuple(_pin_xy(p, 0.0, 4.0 * t + pin_extra, bore_d)
                      for p in (O1, O2, A, B)),
        "joints": joints,
    }
    if coupler_ext > 0:
        parts["trace"] = _pin_xy(tip, 2.0 * t, 3.0 * t + pin_extra, width - 2.0)
    return parts


def _toggle_clamp_joints(arm_len, link_c, handle_len, link_k,
                         pivot_angle_deg, overcenter_deg):
    """Solve the five planar joints of a toggle clamp at one handle angle.

    Returns ``(P0, P1, K, C, T)`` as 2-vectors. Raises ``ValueError`` when the
    over-center travel is unreachable for the given link lengths.
    """
    P0 = np.array([0.0, 0.0])
    C0 = np.array([link_c, 0.0])
    dist_pc = handle_len + link_k
    pa = math.radians(pivot_angle_deg)
    P1 = C0 + dist_pc * np.array([math.cos(pa), math.sin(pa)])
    u0 = (C0 - P1) / dist_pc
    ca, sa = (math.cos(math.radians(overcenter_deg)),
              math.sin(math.radians(overcenter_deg)))
    u = np.array([u0[0] * ca - u0[1] * sa, u0[0] * sa + u0[1] * ca])
    K = P1 + handle_len * u
    candidates = [_circle_circle(P0, link_c, K, link_k, branch,
                                 "toggle_clamp()")
                  for branch in (1, -1)]
    C = min(candidates, key=lambda c: float(np.hypot(*(c - C0))))
    th_arm = math.atan2(C[1], C[0])
    T = arm_len * np.array([math.cos(th_arm), math.sin(th_arm)])
    return P0, P1, K, C, T


def _toggle_clamp_base_bounds(arm_len, link_c, handle_len, link_k,
                              pivot_angle_deg, base_pad,
                              oc_lo=-24.0, oc_hi=30.0, samples=25):
    """Axis-aligned plate that covers every reachable joint across a full swing.

    The base is sized from the pose envelope, not the current pose, so a handle
    animation reposes the moving bars without reshaping the ground plate.
    """
    points = []
    for i in range(samples):
        oc = oc_lo + (oc_hi - oc_lo) * i / float(samples - 1)
        try:
            P0, P1, K, C, T = _toggle_clamp_joints(
                arm_len, link_c, handle_len, link_k, pivot_angle_deg, oc)
        except ValueError:
            continue
        points.extend((P0, P1, K, C, T))
    if not points:
        raise ValueError("toggle_clamp(): no reachable pose in the base envelope")
    corners = np.asarray(points, dtype=float)
    lo = corners.min(axis=0) - base_pad
    hi = corners.max(axis=0) + base_pad
    return lo, hi


def toggle_clamp(arm_len=34.0, link_c=14.0, handle_len=22.0, link_k=10.0,
                 pivot_angle_deg=-105.0, overcenter_deg=4.0, width=6.0,
                 thickness=4.0, base_h=4.0, bore_d=3.0, clearance=0.25,
                 base_pad=6.0, pin_extra=2.0):
    """Build an over-center knee (toggle) clamp posed in its clamping plane.

    The clamp arm pivots at ``P0 = (0, 0)`` with the knee joint at ``link_c``
    and the toe at ``arm_len`` (mm) along it. The handle pivot ``P1`` sits at
    ``handle_len + link_k`` from the dead-center knee position, at
    ``pivot_angle_deg`` (degrees). At ``overcenter_deg = 0`` handle, knee, and
    arm joint are exactly collinear (dead center); a positive
    ``overcenter_deg`` rotates the handle past dead center onto the press side,
    where clamping reaction drives the knee further into its stop — the
    self-locking state. The arm joint ``C`` is re-solved by circle-circle
    intersection; unreachable over-center travel raises ``ValueError``.

    The base plate is sized for the full reachable handle swing so it does not
    change shape with ``overcenter_deg``; only the moving bars repose.

    Returns ``{"base", "arm", "link", "handle", "pins", "joints"}`` with the
    flat base bored at the two fixed pivots and the links stacked one
    thickness apart (arm, connecting link, handle) above it.
    """
    if (arm_len <= 0 or not 0 < link_c < arm_len or handle_len <= 0 or
            link_k <= 0 or width < 2.4 or thickness < 1.2 or base_h < 1.2 or
            bore_d <= 0 or clearance < 0 or bore_d + clearance >= width or
            base_pad < 0 or pin_extra < 0):
        raise ValueError("toggle_clamp(): invalid clamp dimensions")
    P0, P1, K, C, T = _toggle_clamp_joints(
        arm_len, link_c, handle_len, link_k, pivot_angle_deg, overcenter_deg)

    hole_d = bore_d + clearance
    t = thickness
    z_arm = base_h
    z_link = base_h + t
    z_handle = base_h + 2.0 * t
    lo, hi = _toggle_clamp_base_bounds(
        arm_len, link_c, handle_len, link_k, pivot_angle_deg, base_pad)
    plate = sg.box(float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1]))
    for pivot in (P0, P1):
        plate = plate.difference(
            sg.Point(float(pivot[0]), float(pivot[1])).buffer(
                hole_d / 2.0, resolution=48))
    joints = {"P0": (0.0, 0.0), "P1": (float(P1[0]), float(P1[1])),
              "K": (float(K[0]), float(K[1])),
              "C": (float(C[0]), float(C[1])),
              "T": (float(T[0]), float(T[1]))}
    return {
        "base": _extrude(plate.buffer(0), 0.0, base_h),
        "arm": _bar_mesh(P0, T, width, z_arm, z_arm + t, hole_d, (P0, C)),
        "link": _bar_mesh(K, C, width, z_link, z_link + t, hole_d, (K, C)),
        "handle": _bar_mesh(P1, K, width, z_handle, z_handle + t,
                            hole_d, (P1, K)),
        "pins": (
            _pin_xy(P0, 0.0, z_arm + t + pin_extra, bore_d),
            _pin_xy(P1, 0.0, z_handle + t + pin_extra, bore_d),
            _pin_xy(K, z_link, z_handle + t + pin_extra, bore_d),
            _pin_xy(C, z_arm, z_link + t + pin_extra, bore_d),
        ),
        "joints": joints,
    }


def scotch_yoke(crank_r=12.0, disc_r=17.0, pin_d=6.0, angle_deg=35.0,
                thickness=5.0, pin_h=6.0, yoke_wall=2.5, stem_w=8.0,
                stem_len=22.0, rail_w=4.0, clearance=0.25):
    """Build a Scotch yoke: crank disc + pin, slotted yoke, and guide rails.

    The crank pin at ``crank_r`` (mm) and ``angle_deg`` (degrees) engages the
    transverse slot of the yoke, which reciprocates along X in exact simple
    harmonic motion; the yoke is posed at ``x = crank_r * cos(angle_deg)``.
    The slot is ``pin_d + 2 * clearance`` wide. Two fixed rails flank the
    yoke block with ``clearance`` (mm) and constrain it to the X axis.

    Returns ``{"crank_disc", "crank_pin", "yoke", "rail_a", "rail_b",
    "joints"}``; parts are stacked (disc low, yoke and rails one thickness
    up, pin through both) so nothing intersects.
    """
    if (crank_r <= 0 or disc_r < crank_r + pin_d / 2.0 + 1.0 or pin_d <= 0 or
            thickness < 1.2 or pin_h < thickness or yoke_wall < 1.2 or
            stem_w <= 0 or stem_len <= 0 or rail_w < 1.2 or clearance < 0):
        raise ValueError("scotch_yoke(): invalid yoke dimensions")
    th = math.radians(angle_deg)
    px, py = crank_r * math.cos(th), crank_r * math.sin(th)
    dx = px  # yoke pose: pin x-coordinate is the yoke displacement
    slot_half_w = pin_d / 2.0 + clearance
    slot_half_l = crank_r + pin_d / 2.0 + clearance
    bx = slot_half_w + yoke_wall
    by = slot_half_l + yoke_wall
    block = sg.box(-bx, -by, bx, by)
    slot = sg.box(-slot_half_w, -slot_half_l, slot_half_w, slot_half_l)
    stem = sg.box(bx, -stem_w / 2.0, bx + stem_len, stem_w / 2.0)
    yoke2d = block.union(stem).difference(slot).buffer(0)
    yoke = _extrude(yoke2d, thickness, 2.0 * thickness)
    yoke.apply_translation((dx, 0.0, 0.0))

    rail_y = by + clearance
    rail_x0 = -crank_r - bx - 1.5
    rail_x1 = crank_r + bx + 1.5
    rail_a = _extrude(sg.box(rail_x0, rail_y, rail_x1, rail_y + rail_w),
                      0.0, 2.0 * thickness)
    rail_b = _extrude(sg.box(rail_x0, -rail_y - rail_w, rail_x1, -rail_y),
                      0.0, 2.0 * thickness)
    joints = {"O": (0.0, 0.0), "P": (px, py), "yoke_x": dx}
    return {
        "crank_disc": cyl(disc_r, thickness,
                          center=(0.0, 0.0, thickness / 2.0), sections=64),
        "crank_pin": _pin_xy((px, py), thickness, thickness + pin_h, pin_d),
        "yoke": yoke,
        "rail_a": rail_a,
        "rail_b": rail_b,
        "joints": joints,
    }


def quick_return_ratio(crank_r, pivot_dist):
    """Working:return time ratio of a crank and slotted-lever quick-return.

    ``crank_r`` (mm) is the crank-pin radius and ``pivot_dist`` (mm) the
    distance from the crank centre to the slotted-lever pivot. With
    ``pivot_dist > crank_r`` the lever oscillates (crank-shaper form) between
    tangency dead centers; with ``pivot_dist < crank_r`` the geometry sweeps
    to the Whitworth form where the slotted lever makes full non-uniform
    revolutions. Both give ratio ``(180 + 2a) / (180 - 2a)`` degrees with
    ``a = asin(min(crank_r, pivot_dist) / max(crank_r, pivot_dist))``.
    ``pivot_dist == crank_r`` is degenerate (infinite ratio) and raises
    ``ValueError``.
    """
    if crank_r <= 0 or pivot_dist <= 0:
        raise ValueError("quick_return_ratio(): dimensions must be positive")
    if math.isclose(crank_r, pivot_dist, rel_tol=1e-9, abs_tol=1e-12):
        raise ValueError(
            "quick_return_ratio(): pivot_dist == crank_r is degenerate")
    a = math.degrees(math.asin(min(crank_r, pivot_dist) /
                               max(crank_r, pivot_dist)))
    return (180.0 + 2.0 * a) / (180.0 - 2.0 * a)


def quick_return(crank_r=14.0, pivot_dist=34.0, lever_len=60.0,
                 crank_angle_deg=40.0, disc_r=18.0, pin_d=6.0,
                 lever_w=10.0, thickness=5.0, base_h=4.0,
                 pivot_bore_d=6.0, crank_bore_d=5.0, clearance=0.25,
                 base_pad=4.0, pin_extra=2.0):
    """Build a crank and slotted-lever quick-return mechanism posed in assembly.

    The crank pin at ``crank_r`` (mm) and ``crank_angle_deg`` (degrees)
    slides in the slot of a lever pivoted at ``pivot_dist`` (mm) from the
    crank centre; the lever sweeps slowly through its working stroke and
    snaps back through the return. ``pivot_dist`` sweeps the layout toward
    the Whitworth configuration (``pivot_dist < crank_r``). The slot runs
    from 12 mm to ``lever_len - 8`` mm off the lever pivot, so the crank
    sweep must keep the pin inside it. The returned dict carries
    ``"time_ratio"`` — see ``quick_return_ratio``.

    Returns ``{"base", "crank_disc", "crank_pin", "lever", "pins",
    "joints", "time_ratio"}`` with the flat base bored at both fixed pivots
    and the lever stacked one thickness above the crank disc.
    """
    s0, s1 = 12.0, lever_len - 8.0
    if (crank_r <= 0 or pivot_dist <= 0 or lever_len <= 0 or
            disc_r < crank_r + pin_d / 2.0 + 1.0 or pin_d <= 0 or
            lever_w < 2.4 or thickness < 1.2 or base_h < 1.2 or
            pivot_bore_d <= 0 or crank_bore_d <= 0 or clearance < 0 or
            pivot_bore_d + clearance >= lever_w or base_pad < 0 or
            pin_extra < 0):
        raise ValueError("quick_return(): invalid mechanism dimensions")
    pin_r = pin_d / 2.0
    if (abs(pivot_dist - crank_r) < s0 + pin_r + clearance or
            pivot_dist + crank_r > s1 - pin_r - clearance):
        raise ValueError(
            "quick_return(): crank pin travels outside the lever slot")
    th = math.radians(crank_angle_deg)
    O = np.array([0.0, 0.0])
    G = np.array([pivot_dist, 0.0])
    P = crank_r * np.array([math.cos(th), math.sin(th)])
    u = P - G
    u /= np.linalg.norm(u)
    T = G + lever_len * u
    slot = sg.LineString([tuple(G + s0 * u), tuple(G + s1 * u)]).buffer(
        (pin_d + 2.0 * clearance) / 2.0, resolution=24, cap_style=2)
    lever2d = _capsule_2d(G, T, lever_w)
    lever2d = lever2d.difference(
        sg.Point(float(G[0]), float(G[1])).buffer(
            (pivot_bore_d + clearance) / 2.0, resolution=48))
    lever2d = lever2d.difference(slot).buffer(0)

    plate = sg.box(-(disc_r + base_pad), -(disc_r + base_pad),
                   pivot_dist + 2.0 * base_pad, disc_r + base_pad)
    plate = plate.difference(sg.Point(0.0, 0.0).buffer(
        (crank_bore_d + clearance) / 2.0, resolution=48))
    plate = plate.difference(sg.Point(float(G[0]), float(G[1])).buffer(
        (pivot_bore_d + clearance) / 2.0, resolution=48))

    t = thickness
    joints = {"O": (0.0, 0.0), "G": (float(G[0]), 0.0),
              "P": (float(P[0]), float(P[1])),
              "T": (float(T[0]), float(T[1]))}
    return {
        "base": _extrude(plate.buffer(0), 0.0, base_h),
        "crank_disc": cyl(disc_r, t, center=(0.0, 0.0, base_h + t / 2.0),
                          sections=64),
        "crank_pin": _pin_xy(P, base_h + t, base_h + 2.0 * t + pin_extra,
                             pin_d),
        "lever": _extrude(lever2d, base_h + t, base_h + 2.0 * t),
        "pins": (
            _pin_xy(O, 0.0, base_h + t + pin_extra, crank_bore_d),
            _pin_xy(G, 0.0, base_h + 2.0 * t + pin_extra, pivot_bore_d),
        ),
        "joints": joints,
        "time_ratio": quick_return_ratio(crank_r, pivot_dist),
    }


# ---------------------------------------------------------------------------
# Straight-line and scaling linkages (gap-analysis wave v0.8.0)
# ---------------------------------------------------------------------------


def _xz_extrude(poly, y0, y1):
    """Extrude an ``(x, z)``-plane polygon along world Y from ``y0`` to ``y1``."""
    mesh = _extrude(poly, 0.0, float(y1) - float(y0))
    mesh.apply_transform(tf.rotation_matrix(math.pi / 2.0, (1.0, 0.0, 0.0)))
    mesh.apply_translation((0.0, float(y1), 0.0))
    return mesh


def _xz_bar(p0, p1, width, y0, y1, hole_d=0.0):
    """Bar between two ``(x, z)`` points, extruded along Y, bored at both ends."""
    poly = _capsule_2d(p0, p1, width)
    if hole_d > 0:
        for hole in (p0, p1):
            poly = poly.difference(
                sg.Point(float(hole[0]), float(hole[1])).buffer(
                    hole_d / 2.0, resolution=48))
    return _xz_extrude(poly.buffer(0), y0, y1)


def peaucellier_pose(long_len=30.0, rhomb_len=15.0, crank_len=10.0,
                     crank_angle_deg=0.0, branch=1):
    """Solve the Peaucellier-Lipkin cell pose for one crank angle.

    The fixed pivot ``O`` sits at the origin and the crank pivot ``C`` at
    ``(crank_len, 0)`` (mm). The crank pin ``P`` rides a circle of
    ``crank_len`` about ``C`` at ``crank_angle_deg`` (degrees, CCW from +X),
    so ``P`` runs on a circle that passes through ``O``. The cell inverts
    ``P`` about ``O`` with power ``k = long_len**2 - rhomb_len**2``, giving
    the tracer ``Q = k * P / |OP|**2``: the inverse of a circle through the
    centre of inversion is a straight line, so ``Q`` lies exactly on
    ``x = k / (2 * crank_len)`` for every reachable crank angle. ``A`` and
    ``B`` are the rhombus side vertices, the circle-circle intersections of
    ``rhomb_len`` about ``P`` and about ``Q``; ``branch`` (+1/-1) swaps them.
    ``|OA| = |OB| = long_len`` follows from the inversion identity.

    The crank cannot turn a full revolution: the rhombus closes only while
    ``|OP|`` stays inside ``[long_len - rhomb_len, long_len + rhomb_len]``,
    and anything outside raises ``ValueError``. Returns
    ``{"O", "C", "P", "Q", "A", "B"}`` as (x, y) tuples plus ``"power"`` and
    ``"tracer_x"``. Units are mm and degrees.
    """
    if rhomb_len <= 0 or long_len <= rhomb_len or crank_len <= 0:
        raise ValueError(
            "peaucellier_pose(): need long_len > rhomb_len > 0 and "
            "crank_len > 0 so the inversion power is positive")
    if branch not in (1, -1):
        raise ValueError("peaucellier_pose(): branch must be +1 or -1")
    power = long_len * long_len - rhomb_len * rhomb_len
    th = math.radians(crank_angle_deg)
    C = np.array([float(crank_len), 0.0])
    P = C + crank_len * np.array([math.cos(th), math.sin(th)])
    r = float(np.hypot(P[0], P[1]))
    lo, hi = long_len - rhomb_len, long_len + rhomb_len
    if r < 1e-9:
        raise ValueError(
            "peaucellier_pose(): the crank pin reaches the fixed pivot; the "
            "tracer runs to infinity")
    if not lo - 1e-9 <= r <= hi + 1e-9:
        raise ValueError(
            "peaucellier_pose(): crank angle %.3f deg puts |OP| = %.3f mm "
            "outside the rhombus range [%.3f, %.3f] mm"
            % (crank_angle_deg, r, lo, hi))
    Q = (power / (r * r)) * P
    A = _circle_circle(P, rhomb_len, Q, rhomb_len, branch,
                       "peaucellier_pose()")
    B = _circle_circle(P, rhomb_len, Q, rhomb_len, -branch,
                       "peaucellier_pose()")
    return {"O": (0.0, 0.0), "C": (float(C[0]), 0.0),
            "P": (float(P[0]), float(P[1])),
            "Q": (float(Q[0]), float(Q[1])),
            "A": (float(A[0]), float(A[1])),
            "B": (float(B[0]), float(B[1])),
            "power": float(power),
            "tracer_x": float(power / (2.0 * crank_len))}


def peaucellier_linkage(long_len=30.0, rhomb_len=15.0, crank_len=10.0,
                        crank_angle_deg=0.0, branch=1, width=6.0,
                        thickness=3.0, bore_d=3.0, clearance=0.25,
                        pin_extra=2.0):
    """Build a flat-printable Peaucellier-Lipkin exact straight-line cell.

    Peaucellier (1864) and Lipkin (1871) solved the problem Watt only
    approximated: seven bars that draw a mathematically exact straight line
    with revolute joints alone. Two anchor links ``O-A`` and ``O-B`` of
    ``long_len`` and a rhombus ``P-A-Q-B`` of side ``rhomb_len`` form an
    inversor about ``O``; the crank ``C-P`` holds ``P`` on a circle through
    ``O``, so the tracer ``Q`` travels the exact line
    ``x = (long_len**2 - rhomb_len**2) / (2 * crank_len)``.

    All parts print flat on the bed. The eight bodies are stacked one
    ``thickness`` apart in +Z (``O-A``, ``O-B``, the two rhombus layers,
    then ground and crank on top) so no two bars share a plane at a shared
    joint, and the frame sits above the cell so its short ``O-C`` pin never
    has to cross the swept band of the anchor links. The six printed pins
    stand through the stack in bores opened by ``clearance``; the tracer is
    the ``Q`` pin, the last entry of ``"pins"``, which stands proud of the
    stack.

    The crank only swings: at both ends of its arc the rhombus degenerates
    (flat at ``|OP| = long_len - rhomb_len``, collapsed where ``P`` meets
    ``Q``), and ``crank_angle_deg`` outside the reachable arc raises
    ``ValueError``. Keep the swing inside about +/-60 degrees at the default
    proportions to leave the two rhombus bars sharing a layer a working gap.
    Units are mm and degrees.
    """
    if (width < 2.4 or thickness < 1.2 or bore_d <= 0 or clearance < 0 or
            bore_d + clearance >= width or pin_extra < 0):
        raise ValueError("peaucellier_linkage(): invalid link or pin sizes")
    joints = peaucellier_pose(long_len, rhomb_len, crank_len,
                              crank_angle_deg, branch)
    O, C, P, Q = (joints[key] for key in ("O", "C", "P", "Q"))
    A, B = joints["A"], joints["B"]
    hole_d = bore_d + clearance
    t = float(thickness)
    parts = {
        "long_a": _bar_mesh(O, A, width, 0.0, t, hole_d, (O, A)),
        "long_b": _bar_mesh(O, B, width, t, 2.0 * t, hole_d, (O, B)),
        "rhomb_pa": _bar_mesh(P, A, width, 2.0 * t, 3.0 * t, hole_d, (P, A)),
        "rhomb_bq": _bar_mesh(B, Q, width, 2.0 * t, 3.0 * t, hole_d, (B, Q)),
        "rhomb_pb": _bar_mesh(P, B, width, 3.0 * t, 4.0 * t, hole_d, (P, B)),
        "rhomb_aq": _bar_mesh(A, Q, width, 3.0 * t, 4.0 * t, hole_d, (A, Q)),
        "ground": _bar_mesh(O, C, width, 4.0 * t, 5.0 * t, hole_d, (O, C)),
        "crank": _bar_mesh(C, P, width, 5.0 * t, 6.0 * t, hole_d, (C, P)),
    }
    parts["pins"] = (
        _pin_xy(O, 0.0, 5.0 * t + pin_extra, bore_d),
        _pin_xy(C, 4.0 * t, 6.0 * t + pin_extra, bore_d),
        _pin_xy(P, 2.0 * t, 6.0 * t + pin_extra, bore_d),
        _pin_xy(A, 0.0, 4.0 * t + pin_extra, bore_d),
        _pin_xy(B, t, 4.0 * t + pin_extra, bore_d),
        _pin_xy(Q, 2.0 * t, 6.0 * t + 2.0 * pin_extra, bore_d),
    )
    parts["joints"] = joints
    parts["tracer_x"] = joints["tracer_x"]
    parts["power"] = joints["power"]
    return parts


def watt_pose(lever_a_len=30.0, lever_b_len=30.0, coupler_len=24.0,
              lever_angle_deg=0.0, branch=1):
    """Solve Watt's parallel-motion pose for one lever angle.

    The two ground pivots sit at ``O1 = (-lever_a_len, +coupler_len/2)`` and
    ``O2 = (+lever_b_len, -coupler_len/2)`` (mm), so at
    ``lever_angle_deg = 0`` both levers lie horizontal, the coupler ``A-B``
    stands vertical on the Y axis, and the tracer sits at the origin. ``A``
    rides a circle of ``lever_a_len`` about ``O1`` at ``lever_angle_deg``
    (degrees, CCW from +X); ``B`` is the circle-circle intersection of the
    coupler about ``A`` and ``lever_b_len`` about ``O2``, with ``branch``
    (+1/-1) selecting the assembly. The tracer ``T`` divides the coupler in
    the INVERSE ratio of the lever lengths, ``AT : TB = lever_b_len :
    lever_a_len``, which is the midpoint for equal levers.

    ``T`` traces a figure-eight whose central crossing is an inflection: over
    the working stroke it stays within a few hundredths of a millimetre of
    the Y axis, but never exactly on it — this is an APPROXIMATE straight
    line, unlike ``peaucellier_pose``. Returns ``{"O1", "O2", "A", "B",
    "T"}`` as (x, y) tuples. Raises ``ValueError`` past the rocking limit.
    Units are mm and degrees.
    """
    if min(lever_a_len, lever_b_len, coupler_len) <= 0:
        raise ValueError("watt_pose(): link lengths must be positive")
    if branch not in (1, -1):
        raise ValueError("watt_pose(): branch must be +1 or -1")
    half = coupler_len / 2.0
    O1 = np.array([-float(lever_a_len), half])
    O2 = np.array([float(lever_b_len), -half])
    th = math.radians(lever_angle_deg)
    A = O1 + lever_a_len * np.array([math.cos(th), math.sin(th)])
    B = _circle_circle(O2, lever_b_len, A, coupler_len, branch, "watt_pose()")
    frac = lever_b_len / (lever_a_len + lever_b_len)
    T = A + frac * (B - A)
    return {"O1": (float(O1[0]), float(O1[1])),
            "O2": (float(O2[0]), float(O2[1])),
            "A": (float(A[0]), float(A[1])),
            "B": (float(B[0]), float(B[1])),
            "T": (float(T[0]), float(T[1]))}


def watt_linkage(lever_a_len=30.0, lever_b_len=30.0, coupler_len=24.0,
                 lever_angle_deg=0.0, stroke_deg=25.0, branch=1,
                 width=6.0, thickness=3.0, bore_d=3.0, clearance=0.25,
                 pin_extra=2.0, samples=41):
    """Build Watt's parallel motion posed in assembly, with its error measured.

    James Watt's 1784 three-bar linkage: two rocking levers joined by a
    coupler, whose tracer point holds an approximate straight line. Watt
    used it to guide the piston rod of a double-acting beam engine, and it
    is one of the very few 18th-century mechanisms still in current
    production use — the lateral locator of a solid-axle rear suspension is
    a Watt's linkage lying on its side.

    Bodies are ``{"ground", "lever_a", "lever_b", "coupler", "tracer"}``
    plus the four printed pins, stacked one ``thickness`` apart in +Z; the
    ``"tracer"`` boss marks the guided point on the coupler. All parts print
    flat. The returned ``"straight_dev"`` (mm) is the largest departure of
    the tracer from the exact Y axis over ``+/-stroke_deg`` of lever
    rotation, sampled at ``samples`` points, and ``"stroke"`` (mm) is the
    tracer travel over that arc; both are also written to the coupler's
    ``metadata``. ``straight_dev`` is never zero — the line is approximate.
    Raises ``ValueError`` when the stroke exceeds the rocking limit. Units
    are mm and degrees.
    """
    if (width < 2.4 or thickness < 1.2 or bore_d <= 0 or clearance < 0 or
            bore_d + clearance >= width or pin_extra < 0 or
            stroke_deg <= 0 or int(samples) < 5):
        raise ValueError("watt_linkage(): invalid link, pin, or stroke sizes")
    joints = watt_pose(lever_a_len, lever_b_len, coupler_len,
                       lever_angle_deg, branch)
    xs, ys = [], []
    for index in range(int(samples)):
        angle = -stroke_deg + 2.0 * stroke_deg * index / (int(samples) - 1)
        trace = watt_pose(lever_a_len, lever_b_len, coupler_len, angle,
                          branch)["T"]
        xs.append(abs(trace[0]))
        ys.append(trace[1])
    straight_dev = float(max(xs))
    stroke = float(max(ys) - min(ys))

    O1, O2, A, B, T = (joints[key] for key in ("O1", "O2", "A", "B", "T"))
    hole_d = bore_d + clearance
    t = float(thickness)
    coupler = _bar_mesh(A, B, width, 3.0 * t, 4.0 * t, hole_d, (A, B))
    coupler.metadata["straight_dev"] = straight_dev
    coupler.metadata["stroke"] = stroke
    return {
        "ground": _bar_mesh(O1, O2, width, 0.0, t, hole_d, (O1, O2)),
        "lever_a": _bar_mesh(O1, A, width, t, 2.0 * t, hole_d, (O1, A)),
        "lever_b": _bar_mesh(O2, B, width, 2.0 * t, 3.0 * t, hole_d, (O2, B)),
        "coupler": coupler,
        "tracer": _pin_xy(T, 4.0 * t, 4.0 * t + pin_extra + 1.0, width - 2.0),
        "pins": (
            _pin_xy(O1, 0.0, 2.0 * t + pin_extra, bore_d),
            _pin_xy(O2, 0.0, 3.0 * t + pin_extra, bore_d),
            _pin_xy(A, t, 4.0 * t + pin_extra, bore_d),
            _pin_xy(B, 2.0 * t, 4.0 * t + pin_extra, bore_d),
        ),
        "joints": joints,
        "straight_dev": straight_dev,
        "stroke": stroke,
    }


def sarrus_pose(bar_len=20.0, fold_deg=40.0, ear_h=8.0, hinge_off=11.0):
    """Solve the Sarrus linkage platform height for one fold angle.

    Sarrus (1853) beat Peaucellier by eleven years with a SPATIAL answer:
    two hinge chains folding in orthogonal planes constrain the platform to
    pure translation along Z, with revolute joints only and no sliding pair
    anywhere. Each chain runs base ear -> lower bar -> knee -> upper bar ->
    platform ear, both bars ``bar_len`` long, so the closed form is simply
    ``lift = 2 * bar_len * sin(fold_deg)``: the platform rises by that much
    above the base hinge axis, which itself sits ``ear_h`` above the base
    plate face at ``z = 0``.

    ``hinge_off`` (mm) is the distance from the axis of the plates to the
    chain-A hinge line (chain B is the same layout rotated 90 degrees about
    Z). Returns ``{"lift", "hinge_z", "top_hinge_z", "platform_z", "knee",
    "reach"}``; ``"knee"`` is the chain-A knee as an ``(x, z)`` pair and
    ``"platform_z"`` the underside of the platform. Units are mm and
    degrees.
    """
    if bar_len <= 0 or ear_h <= 0 or hinge_off <= 0:
        raise ValueError("sarrus_pose(): bar_len, ear_h, hinge_off must be "
                         "positive")
    if not 1.0 <= fold_deg <= 89.0:
        raise ValueError(
            "sarrus_pose(): fold_deg %.3f is outside the 1..89 degree working "
            "range (the chain is singular flat and singular upright)"
            % fold_deg)
    th = math.radians(fold_deg)
    lift = 2.0 * bar_len * math.sin(th)
    return {"lift": float(lift),
            "hinge_z": float(ear_h),
            "top_hinge_z": float(ear_h + lift),
            "platform_z": float(2.0 * ear_h + lift),
            "knee": (float(hinge_off + bar_len * math.cos(th)),
                     float(ear_h + bar_len * math.sin(th))),
            "reach": float(hinge_off + bar_len * math.cos(th))}


def sarrus_linkage(bar_len=20.0, fold_deg=40.0, plate=28.0, plate_t=3.0,
                   ear_h=8.0, bar_w=6.0, bar_t=3.0, bore_d=3.0,
                   clearance=0.25, pin_extra=2.0):
    """Build a Sarrus spatial straight-line linkage posed at one fold angle.

    The base plate lies with its top face at ``z = 0`` (body from
    ``-plate_t`` to ``0``) and carries two integral ears, one on the +X edge
    and one on the +Y edge. Chain A hinges about axes parallel to Y and
    folds in the XZ plane; chain B is the same chain rotated 90 degrees
    about Z and folds in the YZ plane. Each chain alone would let the
    platform rotate about its hinge axis; together they leave exactly one
    freedom, vertical translation, so the platform stays parallel to the
    base with zero X and Y travel and zero rotation for every fold angle.
    That is the whole claim of the mechanism, and it is why it suits FDM:
    with no sliding pair there is nothing to jam on layer lines.

    Returns ``{"base", "platform", "bars", "pins", "joints", "lift",
    "platform_z"}``; ``"bars"`` holds the four bars in chain-A-lower,
    chain-A-upper, chain-B-lower, chain-B-upper order. Print the base plate
    flat, ears up; print the platform flat too, which is upside down
    relative to the assembly (its ear bores are teardropped for that
    orientation); print the bars flat. The pins are horizontal in assembly
    but print standing. Units are mm and degrees.
    """
    if (bar_len <= 0 or plate_t < 1.2 or ear_h <= 0 or bar_w < 2.4 or
            bar_t < 1.2 or bore_d <= 0 or clearance < 0 or
            bore_d + clearance >= bar_w or pin_extra < 0):
        raise ValueError("sarrus_linkage(): invalid plate, bar, or pin sizes")
    if plate <= 3.0 * bar_w or plate <= 2.0 * (2.0 * bar_t + bar_w):
        raise ValueError(
            "sarrus_linkage(): plate %.2f mm is too small for the two chains; "
            "need plate > max(3*bar_w, 2*(2*bar_t + bar_w))" % plate)
    if ear_h < bar_w / 2.0 + plate_t:
        raise ValueError(
            "sarrus_linkage(): ear_h %.2f mm buries the hinge axis in the "
            "plate; need ear_h >= bar_w/2 + plate_t" % ear_h)
    hinge_off = plate / 2.0 - bar_w / 2.0
    pose = sarrus_pose(bar_len, fold_deg, ear_h, hinge_off)
    hole_d = bore_d + clearance
    knee = pose["knee"]
    z_top = pose["top_hinge_z"]
    pz = pose["platform_z"]
    t = float(bar_t)

    lower = _xz_bar((hinge_off, ear_h), knee, bar_w, -t, 0.0, hole_d)
    upper = _xz_bar(knee, (hinge_off, z_top), bar_w, 0.0, t, hole_d)

    def _ear(z_root, z_axis, y0, y1, peak_up):
        # Keep the round cap at the hinge axis, cut the root flush with the
        # plate face so nothing protrudes past the printed plate.
        clip = sg.box(hinge_off - bar_w, min(z_root, z_axis - bar_w),
                      hinge_off + bar_w, max(z_root, z_axis + bar_w))
        poly = _capsule_2d((hinge_off, z_root), (hinge_off, z_axis),
                           bar_w).intersection(clip)
        mesh = _xz_extrude(largest_poly(poly), y0, y1)
        cutter = teardrop(hole_d / 2.0, 4.0 * (y1 - y0), axis="y",
                          up=(0.0, 0.0, 1.0 if peak_up else -1.0))
        cutter.apply_translation((hinge_off, (y0 + y1) / 2.0, z_axis))
        return sub(mesh, cutter)

    base_plate = _extrude(sg.box(-plate / 2.0, -plate / 2.0,
                                 plate / 2.0, plate / 2.0), -plate_t, 0.0)
    plat_plate = _extrude(sg.box(-plate / 2.0, -plate / 2.0,
                                 plate / 2.0, plate / 2.0), pz, pz + plate_t)
    base_ear = _ear(-plate_t, ear_h, -2.0 * t, -t, True)
    plat_ear = _ear(pz, z_top, t, 2.0 * t, False)

    turn = tf.rotation_matrix(math.pi / 2.0, (0.0, 0.0, 1.0))

    def _turned(mesh):
        other = mesh.copy()
        other.apply_transform(turn)
        return other

    base = uni([base_plate, base_ear, _turned(base_ear)])
    platform = uni([plat_plate, plat_ear, _turned(plat_ear)])
    platform.metadata["lift"] = pose["lift"]
    platform.metadata["platform_z"] = pz

    pin_len = 4.0 * t + 2.0 * pin_extra
    pins = []
    for x, z in ((hinge_off, ear_h), (knee[0], knee[1]),
                 (hinge_off, z_top)):
        pin = cyl(bore_d / 2.0, pin_len, center=(x, 0.0, z), axis="y",
                  sections=48)
        pins.append(pin)
        pins.append(_turned(pin))
    return {
        "base": base,
        "platform": platform,
        "bars": (lower, upper, _turned(lower), _turned(upper)),
        "pins": tuple(pins),
        "joints": pose,
        "lift": pose["lift"],
        "platform_z": pz,
    }


def pantograph_pose(arm_a=18.0, arm_b=24.0, ratio=2.0, p_x=32.0, p_y=0.0,
                    branch=1):
    """Solve the pantograph pose for one position of the tracing point.

    The fixed pivot ``F`` sits at the origin. Bar 1 carries ``F``, the joint
    ``A`` at ``arm_a`` and the joint ``C`` at ``ratio * arm_a``, all
    collinear. Bar 2 runs ``A`` to the tracing point ``P`` at ``(p_x,
    p_y)``. Bar 3 carries ``C``, the joint ``D`` at ``arm_b`` and the
    output point ``Q`` at ``ratio * arm_b``. Bar 4 closes the parallelogram
    ``A-C-D-P`` with length ``(ratio - 1) * arm_a``. That parallelogram is
    what forces ``Q = ratio * P`` exactly, for every reachable position of
    ``P``: ``F``, ``P`` and ``Q`` stay collinear and ``|FQ| / |FP| =
    ratio``.

    ``branch`` (+1/-1) picks which side of ``FP`` the bar-1 joint ``A``
    falls on. Raises ``ValueError`` when ``P`` is out of reach, that is when
    ``|FP|`` leaves ``[|arm_a - arm_b|, arm_a + arm_b]``. Returns
    ``{"F", "A", "C", "D", "P", "Q"}`` as (x, y) tuples plus ``"ratio"``.
    Units are mm and degrees.
    """
    if arm_a <= 0 or arm_b <= 0:
        raise ValueError("pantograph_pose(): arm lengths must be positive")
    if ratio <= 1.0:
        raise ValueError(
            "pantograph_pose(): ratio must exceed 1 (trace at Q for the "
            "1/ratio reduction instead of rebuilding the linkage)")
    if branch not in (1, -1):
        raise ValueError("pantograph_pose(): branch must be +1 or -1")
    F = np.array([0.0, 0.0])
    P = np.array([float(p_x), float(p_y)])
    A = _circle_circle(F, arm_a, P, arm_b, branch, "pantograph_pose()")
    C = ratio * A
    D = C + (P - A)
    Q = ratio * P
    return {"F": (0.0, 0.0),
            "A": (float(A[0]), float(A[1])),
            "C": (float(C[0]), float(C[1])),
            "D": (float(D[0]), float(D[1])),
            "P": (float(P[0]), float(P[1])),
            "Q": (float(Q[0]), float(Q[1])),
            "ratio": float(ratio)}


def pantograph_linkage(arm_a=18.0, arm_b=24.0, ratio=2.0, p_x=32.0, p_y=0.0,
                       branch=1, width=6.0, thickness=3.0, bore_d=3.0,
                       clearance=0.25, pad_r=9.0, pin_extra=2.0):
    """Build a flat-printable pantograph posed at one tracing position.

    A parallelogram chain anchored at one fixed pivot: run the stylus pin
    ``P`` around a shape and the output pin ``Q`` draws the same shape
    scaled by ``ratio`` about the pivot. Reverse the roles for a
    ``1/ratio`` reduction. The scaling is exact and holds at every
    position, not just near a design point, because it comes from the
    parallelogram closure rather than from an approximation.

    Returns ``{"base", "bar1", "bar2", "bar3", "bar4", "tracer", "pins",
    "joints", "ratio", "achieved_ratio"}``. ``"achieved_ratio"`` is measured
    from the solved pose (``|FQ| / |FP|``), not copied from the argument.
    The four bars each get their own Z layer above the ground pad so no two
    bars share a plane; all parts print flat, and the five printed pins
    stand through bores opened by ``clearance``. ``"tracer"`` is the boss on
    the far end of bar 3 marking ``Q``. Units are mm and degrees.
    """
    if (width < 2.4 or thickness < 1.2 or bore_d <= 0 or clearance < 0 or
            bore_d + clearance >= width or pin_extra < 0 or
            pad_r < bore_d + 2.0):
        raise ValueError("pantograph_linkage(): invalid link or pad sizes")
    joints = pantograph_pose(arm_a, arm_b, ratio, p_x, p_y, branch)
    F, A, C, D = (joints[key] for key in ("F", "A", "C", "D"))
    P, Q = joints["P"], joints["Q"]
    hole_d = bore_d + clearance
    t = float(thickness)
    pad = sg.Point(0.0, 0.0).buffer(pad_r, resolution=64).difference(
        sg.Point(0.0, 0.0).buffer(hole_d / 2.0, resolution=48))
    fp = math.hypot(P[0], P[1])
    achieved = float(math.hypot(Q[0], Q[1]) / fp) if fp > 1e-9 else float(ratio)
    return {
        "base": _extrude(pad, 0.0, t),
        "bar1": _bar_mesh(F, C, width, t, 2.0 * t, hole_d, (F, A, C)),
        "bar2": _bar_mesh(A, P, width, 2.0 * t, 3.0 * t, hole_d, (A, P)),
        "bar3": _bar_mesh(C, Q, width, 3.0 * t, 4.0 * t, hole_d, (C, D)),
        "bar4": _bar_mesh(P, D, width, 4.0 * t, 5.0 * t, hole_d, (P, D)),
        "tracer": _pin_xy(Q, 4.0 * t, 5.0 * t + pin_extra, width - 2.0),
        "pins": (
            _pin_xy(F, 0.0, 2.0 * t + pin_extra, bore_d),
            _pin_xy(A, t, 3.0 * t + pin_extra, bore_d),
            _pin_xy(C, t, 4.0 * t + pin_extra, bore_d),
            _pin_xy(D, 3.0 * t, 5.0 * t + pin_extra, bore_d),
            _pin_xy(P, 2.0 * t, 5.0 * t + pin_extra + 1.0, bore_d),
        ),
        "joints": joints,
        "ratio": float(ratio),
        "achieved_ratio": achieved,
    }


def lazy_tongs_pose(rhombs=3, bar_len=30.0, angle_deg=35.0):
    """Solve the lazy-tongs (Nuremberg scissors) pose for one bar angle.

    ``rhombs`` scissor units are chained along +X. Every bar is ``bar_len``
    long and pinned at its centre to its partner and at both ends to the
    neighbouring unit, so each unit advances the chain by
    ``bar_len * cos(angle_deg)`` and every station spans
    ``bar_len * sin(angle_deg)`` across. The fixed pin sits at the origin
    and the sliding pin directly above it, so the tip advances to
    ``span = rhombs * bar_len * cos(angle_deg)``.

    Two numbers fall out. ``"stroke_mult"`` is ``rhombs``: the tip advances
    exactly ``rhombs`` times as far as the first unit's joint, which is the
    whole point of the mechanism. ``"gain"`` is
    ``rhombs * tan(angle_deg)``, the tip advance per unit of transverse
    squeeze at the driving end (negative sense: squeezing shortens the
    height and lengthens the span). Returns those plus ``"height"``,
    ``"pitch"``, ``"span"``, ``"tip"``, ``"stations"`` (the x of each pinned
    station) and ``"centres"``. Units are mm and degrees.
    """
    n = int(rhombs)
    if n < 1:
        raise ValueError("lazy_tongs_pose(): rhombs must be at least 1")
    if bar_len <= 0:
        raise ValueError("lazy_tongs_pose(): bar_len must be positive")
    if not 1.0 <= angle_deg <= 89.0:
        raise ValueError(
            "lazy_tongs_pose(): angle_deg %.3f is outside the 1..89 degree "
            "working range (the scissors are singular at both ends)"
            % angle_deg)
    th = math.radians(angle_deg)
    pitch = bar_len * math.cos(th)
    height = bar_len * math.sin(th)
    stations = tuple(float(i * pitch) for i in range(n + 1))
    centres = tuple(float((i + 0.5) * pitch) for i in range(n))
    return {"pitch": float(pitch), "height": float(height),
            "span": float(n * pitch), "tip": (float(n * pitch), 0.0),
            "stroke_mult": float(n),
            "gain": float(n * math.tan(th)),
            "stations": stations, "centres": centres}


def lazy_tongs(rhombs=3, bar_len=30.0, angle_deg=35.0, slot_lo_deg=20.0,
               slot_hi_deg=55.0, width=6.0, thickness=3.0, yoke_w=9.0,
               foot_len=10.0, bore_d=3.0, clearance=0.25, pin_extra=2.0):
    """Build a lazy-tongs (Nuremberg scissors) extension chain posed in assembly.

    Bars pinned at their centres and ends form a row of rhombs: squeeze the
    frame end and the output yoke shoots out along +X at ``rhombs`` times
    the stroke. The frame at ``x = 0`` holds one pin fixed at the origin and
    guides the other in a vertical slot; the output yoke repeats that pair
    at the tip, so the tip travels a straight line along the X axis. This
    is the extension of a lazy-tongs riveter, a folding gate, and the arm of
    a scissor lift.

    ``slot_lo_deg`` and ``slot_hi_deg`` fix the guide-slot span, so the
    frame and yoke are the same parts at every ``angle_deg`` inside that
    range — the mechanism reposes rather than reshapes. Returns
    ``{"frame", "output", "bars", "pins", "joints", "span", "height",
    "stroke_mult"}``; ``"bars"`` runs unit by unit, rising bar then falling
    bar. All parts print flat. Units are mm and degrees.
    """
    n = int(rhombs)
    if (width < 2.4 or thickness < 1.2 or bore_d <= 0 or clearance < 0 or
            bore_d + clearance >= width or pin_extra < 0 or foot_len <= 0 or
            yoke_w < bore_d + 2.4):
        raise ValueError("lazy_tongs(): invalid bar, yoke, or pin sizes")
    if not 1.0 <= slot_lo_deg < slot_hi_deg <= 89.0:
        raise ValueError(
            "lazy_tongs(): need 1 <= slot_lo_deg < slot_hi_deg <= 89")
    if not slot_lo_deg <= angle_deg <= slot_hi_deg:
        raise ValueError(
            "lazy_tongs(): angle_deg %.3f is outside the guide-slot span "
            "[%.3f, %.3f] deg" % (angle_deg, slot_lo_deg, slot_hi_deg))
    pose = lazy_tongs_pose(n, bar_len, angle_deg)
    h_lo = bar_len * math.sin(math.radians(slot_lo_deg))
    h_hi = bar_len * math.sin(math.radians(slot_hi_deg))
    if h_lo < width + clearance:
        raise ValueError(
            "lazy_tongs(): at %.3f deg the station height %.2f mm closes the "
            "gap between same-layer bars (needs > width + clearance)"
            % (slot_lo_deg, h_lo))
    if h_lo < bore_d * 2.0:
        raise ValueError(
            "lazy_tongs(): guide slot would break into the fixed bore; raise "
            "slot_lo_deg or bar_len")

    height = pose["height"]
    pitch = pose["pitch"]
    hole_d = bore_d + clearance
    t = float(thickness)
    bars = []
    for unit in range(n):
        x0, x1 = unit * pitch, (unit + 1) * pitch
        mid = ((x0 + x1) / 2.0, height / 2.0)
        rise = ((x0, 0.0), (x1, height))
        fall = ((x0, height), (x1, 0.0))
        bars.append(_bar_mesh(rise[0], rise[1], width, t, 2.0 * t, hole_d,
                              (rise[0], mid, rise[1])))
        bars.append(_bar_mesh(fall[0], fall[1], width, 2.0 * t, 3.0 * t,
                              hole_d, (fall[0], mid, fall[1])))

    def _yoke(x, sign):
        half = yoke_w / 2.0
        body = sg.box(min(x - half, x + sign * foot_len), -half,
                      max(x + half, x + sign * foot_len), h_hi + half)
        body = body.difference(sg.Point(x, 0.0).buffer(hole_d / 2.0,
                                                       resolution=48))
        slot = sg.LineString([(x, h_lo), (x, h_hi)]).buffer(
            hole_d / 2.0, resolution=24, cap_style=1)
        return body.difference(slot).buffer(0)

    span = pose["span"]
    frame = _extrude(_yoke(0.0, -1.0), 0.0, t)
    output = _extrude(_yoke(span, 1.0), 3.0 * t, 4.0 * t)
    pin_top = 4.0 * t + pin_extra
    pins = []
    for i in range(n + 1):
        x = i * pitch
        pins.append(_pin_xy((x, 0.0), 0.0, pin_top, bore_d))
        pins.append(_pin_xy((x, height), 0.0, pin_top, bore_d))
    for x in pose["centres"]:
        pins.append(_pin_xy((x, height / 2.0), t, 3.0 * t + pin_extra, bore_d))
    output.metadata["stroke_mult"] = pose["stroke_mult"]
    output.metadata["span"] = span
    return {
        "frame": frame,
        "output": output,
        "bars": tuple(bars),
        "pins": tuple(pins),
        "joints": pose,
        "span": span,
        "height": height,
        "stroke_mult": pose["stroke_mult"],
        "gain": pose["gain"],
    }


__all__ = (
    "link_bar",
    "four_bar_pose",
    "four_bar",
    "toggle_clamp",
    "scotch_yoke",
    "quick_return_ratio",
    "quick_return",
    "peaucellier_pose",
    "peaucellier_linkage",
    "watt_pose",
    "watt_linkage",
    "sarrus_pose",
    "sarrus_linkage",
    "pantograph_pose",
    "pantograph_linkage",
    "lazy_tongs_pose",
    "lazy_tongs",
)
