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

    Returns ``{"base", "arm", "link", "handle", "pins", "joints"}`` with the
    flat base bored at the two fixed pivots and the links stacked one
    thickness apart (arm, connecting link, handle) above it.
    """
    if (arm_len <= 0 or not 0 < link_c < arm_len or handle_len <= 0 or
            link_k <= 0 or width < 2.4 or thickness < 1.2 or base_h < 1.2 or
            bore_d <= 0 or clearance < 0 or bore_d + clearance >= width or
            base_pad < 0 or pin_extra < 0):
        raise ValueError("toggle_clamp(): invalid clamp dimensions")
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

    hole_d = bore_d + clearance
    t = thickness
    z_arm = base_h
    z_link = base_h + t
    z_handle = base_h + 2.0 * t
    corners = np.array([P0, P1, K, C, T])
    lo = corners.min(axis=0) - base_pad
    hi = corners.max(axis=0) + base_pad
    plate = sg.box(lo[0], lo[1], hi[0], hi[1])
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


__all__ = (
    "link_bar",
    "four_bar_pose",
    "four_bar",
    "toggle_clamp",
    "scotch_yoke",
    "quick_return_ratio",
    "quick_return",
)
