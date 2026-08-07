"""Project-agnostic holding, centring, and gripping generators."""

import math

import numpy as np
import shapely.geometry as sg
import trimesh
import trimesh.transformations as tf
from shapely import affinity
from shapely.geometry.polygon import orient
from shapely.ops import nearest_points, unary_union

from .cutters import slot_cutter
from .mechanisms import knurl, thread_solid
from .meshutil import from_manifold, sub, to_manifold, uni
from .patterns import polar_ring
from .prim import cyl, frustum, sector2d


def _extrude(poly, z0, z1):
    """Extrude a Shapely polygon between two world Z planes."""
    height = float(z1 - z0)
    if height <= 0:
        raise ValueError("extrusion height must be positive")
    mesh = trimesh.creation.extrude_polygon(poly, height)
    if not mesh.is_watertight:
        mesh = from_manifold(to_manifold(mesh))
    if z0:
        mesh.apply_translation((0.0, 0.0, z0))
    return mesh


def _capsule(p0, p1, width, sections=32):
    """Return a stadium polygon of ``width`` spanning two XY points."""
    return sg.LineString([(float(p0[0]), float(p0[1])),
                          (float(p1[0]), float(p1[1]))]).buffer(
        width / 2.0, resolution=max(4, sections // 8), cap_style=1,
        join_style=1)


def _disc(center, r, sections=64):
    return sg.Point(float(center[0]), float(center[1])).buffer(
        r, resolution=max(4, sections // 4))


def _annulus(r_in, r_out, sections=64):
    return _disc((0.0, 0.0), r_out, sections).difference(
        _disc((0.0, 0.0), r_in, sections))


# ---------------------------------------------------------------------------
# Iris diaphragm
# ---------------------------------------------------------------------------

def _iris_pin(pivot_r, drive_arm, control_rad):
    """Solve one blade's pose from the ring angle.

    The drive ring carries a radial slot per blade. At ring angle ``control``
    the slot for the reference blade (pivot on the +X axis) is the ray from the
    axis at that angle, so the drive pin sits where that ray meets the pin
    circle of radius ``drive_arm`` about the pivot. Returns
    ``(blade_rad, (pin_x, pin_y))``.
    """
    sin_c = math.sin(control_rad)
    disc = drive_arm ** 2 - (pivot_r * sin_c) ** 2
    if disc <= 1e-9:
        raise ValueError(
            "iris_diaphragm(): drive slot at %.2f deg misses the pin circle; "
            "reduce control_deg or lengthen drive_arm"
            % math.degrees(control_rad))
    s = pivot_r * math.cos(control_rad) + math.sqrt(disc)
    pin = (s * math.cos(control_rad), s * sin_c)
    return math.atan2(pin[1], pin[0] - pivot_r), pin


def _iris_control_rad(pivot_r, drive_arm, blade_rad):
    """Ring angle that puts the reference blade at ``blade_rad``."""
    return math.atan2(drive_arm * math.sin(blade_rad),
                      pivot_r + drive_arm * math.cos(blade_rad))


def _iris_leaf_r(blades, r_arc, offset_max):
    """Return how far a leaf must reach from its own arc centre.

    A point at radius ``rho`` is shut out only by a leaf whose arc centre is at
    least ``r_arc`` away from it. Walking the arc centres round the axis samples
    that distance every ``360 / blades``, so the leaf that first clears
    ``r_arc`` may already be a whole step past it; the leaf has to be long
    enough to still reach. Fewer leaves means a coarser step and a longer leaf.
    """
    worst = r_arc
    for offset in np.linspace(0.02 * offset_max, offset_max, 40):
        rho = np.linspace(max(0.02, r_arc - offset), r_arc, 40)
        cos_d = np.clip(
            (rho ** 2 + offset ** 2 - r_arc ** 2) / (2.0 * rho * offset),
            -1.0, 1.0)
        step = np.minimum(np.arccos(cos_d) + 2.0 * np.pi / blades, np.pi)
        reach = np.sqrt(rho ** 2 + offset ** 2
                        - 2.0 * rho * offset * np.cos(step))
        worst = max(worst, float(reach.max()))
    return worst * 1.02 + 0.8


def _iris_wedge_deg(blades, r_arc, offset_max, leaf_r):
    """Return the half angle a leaf must span about its own arc centre.

    Same walk as ``_iris_leaf_r``, but asking where the covering leaf sees the
    point rather than how far away it is: the leaf that shuts a point out may
    see it well off the line from its arc centre through the axis, and the leaf
    has to be wide enough to be there. Reported for the widest such point.
    """
    worst = 180.0 / blades
    phase = np.linspace(0.0, 2.0 * np.pi / blades, 24)
    turns = (2.0 * np.pi * np.arange(blades))[:, None] / blades
    for offset in np.linspace(0.02 * offset_max, offset_max, 30):
        for rho in np.linspace(max(0.02, r_arc - offset), r_arc, 30):
            angle = phase[None, :] + turns
            reach = np.sqrt(rho ** 2 + offset ** 2
                            - 2.0 * rho * offset * np.cos(angle))
            seen = np.abs(np.arctan2(rho * np.sin(angle),
                                     offset - rho * np.cos(angle)))
            # A point no leaf can shut out is inside the opening (the corners
            # of the curved polygon the arcs cut) and needs nobody covering it.
            shut = reach >= r_arc
            ok = shut & (reach <= leaf_r)
            seen = np.where(ok, seen, np.pi)
            need = seen.min(axis=0)[shut.any(axis=0)]
            if need.size:
                worst = max(worst, float(np.degrees(need.max())))
    return min(worst, 178.0)


def iris_control_range(aperture_max=24.0, aperture_min=6.0, pivot_r=26.0,
                       drive_arm=13.0):
    """Return an iris drive ring's travel from wide open to ``aperture_min``.

    The same closed form ``iris_diaphragm`` uses, split out because the usable
    range of ``control_deg`` depends on the aperture and pivot geometry and a
    caller driving the ring (a servo horn, a detent plate, a slider) needs the
    number before it builds anything. Ring travel is small by construction: the
    drive pin sits ``drive_arm`` out from a pivot ``pivot_r`` from the axis, so
    the ring only has to sweep the pin's own angular travel about the axis,
    roughly ``drive_arm / (pivot_r + drive_arm)`` of the leaf swing. Units are
    mm and degrees.
    """
    r_arc = aperture_max / 2.0
    offset_max = r_arc - aperture_min / 2.0
    if not 0.0 <= aperture_min < aperture_max:
        raise ValueError(
            "iris_control_range(): need 0 <= aperture_min < aperture_max")
    if pivot_r <= r_arc or drive_arm <= 0:
        raise ValueError(
            "iris_control_range(): pivot_r must clear the maximum aperture")
    if offset_max > 2.0 * pivot_r:
        raise ValueError("iris_control_range(): pivot_r too small to close")
    return math.degrees(_iris_control_rad(
        pivot_r, drive_arm, 2.0 * math.asin(offset_max / (2.0 * pivot_r))))


def _iris_blade_poly(blade_rad, pin, r_arc, leaf_r, pivot_r, sigma_deg,
                     wedge_local_deg, neck_w, pad_r, hole_r, sections):
    """Return the reference blade outline at one pose, pivot on the +X axis."""
    pivot = (pivot_r, 0.0)
    centre = (pivot_r - pivot_r * math.cos(blade_rad),
              -pivot_r * math.sin(blade_rad))
    heading = wedge_local_deg + math.degrees(blade_rad)
    band = _disc(centre, leaf_r, sections).difference(
        _disc(centre, r_arc, sections))
    wedge = affinity.translate(
        sector2d(heading - sigma_deg, heading + sigma_deg, leaf_r * 1.5,
                 n=max(16, sections // 2)),
        centre[0], centre[1])
    leaf = band.intersection(wedge)
    if leaf.is_empty:
        raise ValueError("iris_diaphragm(): empty blade leaf; check radii")
    near = nearest_points(leaf, sg.Point(pivot))[0]
    blade = unary_union([
        leaf,
        _capsule((near.x, near.y), pivot, neck_w, sections),
        _disc(pivot, pad_r, sections),
        _capsule(pivot, pin, neck_w, sections),
        _disc(pin, pad_r * 0.75, sections),
    ]).buffer(0)
    return blade.difference(_disc(pivot, hole_r, sections)), centre


def iris_diaphragm(blades=6, aperture_max=24.0, aperture_min=6.0, blade_t=1.0,
                   gap=0.25, control_deg=0.0, pivot_r=26.0, drive_arm=13.0,
                   post_d=5.0, pin_d=4.0, clearance=0.3, base_h=2.4,
                   ring_t=2.4, cap_t=2.0, rim=5.0, overlap_deg=12.0,
                   sections=64):
    """Build a stacked-plane iris diaphragm posed at one drive-ring angle.

    ``blades`` leaves each pivot on their own post standing on the base ring at
    radius ``pivot_r``, and each carries a drive pin that rides a radial slot in
    the drive ring above the stack. Turning the ring swings every leaf through
    the same angle, and because each leaf's inner edge is a circular arc of
    radius ``aperture_max / 2`` struck about a point that starts on the axis and
    walks outward as the leaf swings, the free opening stays circular and
    shrinks smoothly from ``aperture_max`` to ``aperture_min``. The arc centre
    sits ``pivot_r`` from the pivot, so its offset from the axis is
    ``2 * pivot_r * sin(blade_angle / 2)`` and the current opening radius is
    ``aperture_max / 2`` minus that offset -- both are reported in
    ``metadata`` along with ``control_range_deg``, the ring travel from wide
    open to ``aperture_min``.

    This is the stacked-plane variant, not the shingled single-plane camera
    iris: leaf ``k`` gets its own Z layer ``blade_t`` thick with ``gap`` of air
    above and below, so no two leaves ever touch and the whole assembly prints
    in place. Printability costs envelope. The pivot posts run the full height
    of the stack, so every leaf must stay clear of every other leaf's post at
    every ring angle, which forces ``pivot_r`` out past the leaf sweep and makes
    the housing roughly three to four times the maximum aperture. The
    constructor measures the leaf sweep, the leaf-to-post clearance and the
    closed-aperture coverage and raises ``ValueError`` rather than returning an
    iris that jams or leaks light.

    Returns ``{"base", "blades": [...], "drive_ring", "cap"}`` posed in
    assembly. The base is an annulus whose bore equals ``aperture_max`` with the
    pivot posts on it; the cap presses onto the post tops and traps the drive
    ring. Print it at ``control_deg=0``: wide open, every leaf lies entirely
    over the base annulus, so nothing bridges the bore and no support is needed.
    Z zero is the underside of the base and +Z is up through the bore.
    Units are mm and degrees.
    """
    blades = int(round(blades))
    if blades < 3 or blades > 16:
        raise ValueError("iris_diaphragm(): blades must be between 3 and 16")
    if not 0.0 <= aperture_min < aperture_max:
        raise ValueError(
            "iris_diaphragm(): need 0 <= aperture_min < aperture_max")
    if (blade_t < 0.8 or gap < 0.15 or pin_d < 2.0 or post_d < 2.0 or
            clearance < 0.1 or base_h < 1.2 or ring_t < 1.2 or cap_t < 1.2 or
            rim < 2.0 or drive_arm <= 0 or overlap_deg < 0 or sections < 24):
        raise ValueError("iris_diaphragm(): invalid feature sizes")
    r_arc = aperture_max / 2.0
    offset_max = r_arc - aperture_min / 2.0
    if pivot_r <= r_arc:
        raise ValueError(
            "iris_diaphragm(): pivot_r must clear the maximum aperture")
    if offset_max > 2.0 * pivot_r:
        raise ValueError("iris_diaphragm(): pivot_r too small to close")
    blade_max = 2.0 * math.asin(offset_max / (2.0 * pivot_r))
    control_max = math.radians(iris_control_range(
        aperture_max=aperture_max, aperture_min=aperture_min,
        pivot_r=pivot_r, drive_arm=drive_arm))
    if control_max > 0.7 * math.asin(min(1.0, drive_arm / pivot_r)):
        raise ValueError(
            "iris_diaphragm(): drive slot runs into its tangent point; "
            "lengthen drive_arm or reduce the aperture range")
    control_rad = math.radians(control_deg)
    if control_rad < -1e-9 or control_rad > control_max + 1e-9:
        raise ValueError(
            "iris_diaphragm(): control_deg must lie in [0, %.3f]"
            % math.degrees(control_max))
    control_rad = min(max(control_rad, 0.0), control_max)

    leaf_r = _iris_leaf_r(blades, r_arc, offset_max)
    sigma_deg = min(88.0, _iris_wedge_deg(blades, r_arc, offset_max, leaf_r)
                    + overlap_deg)
    post_r = post_d / 2.0
    sweep_r = max(leaf_r, math.sqrt(
        leaf_r ** 2 + offset_max ** 2
        - 2.0 * leaf_r * offset_max * math.cos(math.radians(sigma_deg))))
    if sweep_r + post_r + clearance > pivot_r:
        raise ValueError(
            "iris_diaphragm(): leaves sweep into the pivot posts; pivot_r "
            "must be at least %.2f mm" % (sweep_r + post_r + clearance))
    wedge_local_deg = 90.0 - math.degrees(blade_max) / 4.0
    neck_w = max(2.4, blade_t * 2.4)
    pad_r = post_r + max(1.6, 2.0 * clearance + 1.2)
    hole_r = post_r + clearance
    housing_r = pivot_r + drive_arm + rim
    ring_ri = pivot_r + post_r + clearance + 0.8
    if ring_ri >= housing_r - 2.0:
        raise ValueError("iris_diaphragm(): rim too small for the drive ring")

    blade_rad, pin = _iris_pin(pivot_r, drive_arm, control_rad)
    poly, centre = _iris_blade_poly(
        blade_rad, pin, r_arc, leaf_r, pivot_r, sigma_deg, wedge_local_deg,
        neck_w, pad_r, hole_r, sections)
    if poly.geom_type != "Polygon":
        raise ValueError("iris_diaphragm(): blade outline split into pieces")
    aperture_r = r_arc - 2.0 * pivot_r * math.sin(blade_rad / 2.0)

    # Sweep gates. Sampling the whole ring travel is what makes the "no leaf
    # ever touches a post" and "the aperture really closes" claims measurable
    # rather than hopeful.
    probes = np.linspace(0.0, control_max, 6)
    posts = [_disc(p, post_r + clearance, sections)
             for p in polar_ring(blades, pivot_r)]
    for probe in probes:
        p_rad, p_pin = _iris_pin(pivot_r, drive_arm, float(probe))
        p_poly, _c = _iris_blade_poly(
            p_rad, p_pin, r_arc, leaf_r, pivot_r, sigma_deg, wedge_local_deg,
            neck_w, pad_r, hole_r, sections)
        leaves = [affinity.rotate(p_poly, 360.0 * k / blades, origin=(0, 0))
                  for k in range(blades)]
        for k, leaf in enumerate(leaves):
            for j, post in enumerate(posts):
                if j == k:
                    continue
                if leaf.intersection(post).area > 1e-6:
                    raise ValueError(
                        "iris_diaphragm(): leaf %d fouls post %d at %.2f deg "
                        "of ring travel; increase pivot_r"
                        % (k, j, math.degrees(probe)))
        open_r = r_arc - 2.0 * pivot_r * math.sin(p_rad / 2.0)
        if open_r > r_arc - 0.5:
            continue
        opening = _disc((0.0, 0.0), r_arc - 0.02, sections).difference(
            unary_union(leaves))
        holes = [g for g in getattr(opening, "geoms", [opening])
                 if g.area > 0.05]
        if len(holes) != 1 or not holes[0].contains(sg.Point(0.0, 0.0)):
            raise ValueError(
                "iris_diaphragm(): %d leaves leave %d separate openings at "
                "%.2f deg of ring travel; raise overlap_deg"
                % (blades, len(holes), math.degrees(probe)))
        cut = sg.Point(0.0, 0.0).distance(holes[0].exterior)
        if cut < open_r - 0.35:
            raise ValueError(
                "iris_diaphragm(): the opening at %.2f deg measures %.2f mm "
                "against the %.2f mm the arcs predict; the leaves are too "
                "short" % (math.degrees(probe), cut, open_r))

    z_blades = [base_h + gap + k * (blade_t + gap) for k in range(blades)]
    stack_top = z_blades[-1] + blade_t
    ring_z0 = stack_top + gap
    ring_z1 = ring_z0 + ring_t
    cap_z0 = ring_z1 + gap
    cap_z1 = cap_z0 + cap_t

    base_poly = _annulus(r_arc, housing_r, sections)
    base = _extrude(base_poly, 0.0, base_h)
    base = uni([base] + [
        cyl(post_r, cap_z1, center=(p[0], p[1], cap_z1 / 2.0),
            sections=max(16, sections // 2))
        for p in polar_ring(blades, pivot_r)])

    blade_meshes = []
    for k in range(blades):
        turn = 360.0 * k / blades
        leaf = affinity.rotate(poly, turn, origin=(0, 0))
        mesh = _extrude(leaf, z_blades[k], z_blades[k] + blade_t)
        spin = math.radians(turn)
        pin_xy = (pin[0] * math.cos(spin) - pin[1] * math.sin(spin),
                  pin[0] * math.sin(spin) + pin[1] * math.cos(spin))
        peg_z0 = z_blades[k] + blade_t
        mesh = uni([mesh, cyl(pin_d / 2.0, ring_z1 - peg_z0,
                              center=(pin_xy[0], pin_xy[1],
                                      (peg_z0 + ring_z1) / 2.0),
                              sections=max(16, sections // 2))])
        blade_meshes.append(mesh)

    pin_r0 = math.hypot(*_iris_pin(pivot_r, drive_arm, control_max)[1])
    pin_r1 = pivot_r + drive_arm
    slot_w = pin_d + 2.0 * clearance
    ring_poly = _annulus(ring_ri, housing_r, sections)
    for k in range(blades):
        angle = 2.0 * math.pi * k / blades
        a = (math.cos(angle) * (pin_r0 - slot_w),
             math.sin(angle) * (pin_r0 - slot_w))
        b = (math.cos(angle) * (pin_r1 + slot_w),
             math.sin(angle) * (pin_r1 + slot_w))
        ring_poly = ring_poly.difference(
            sg.LineString([a, b]).buffer(slot_w / 2.0, resolution=8,
                                         cap_style=1))
    ring_poly = affinity.rotate(ring_poly.buffer(0), control_deg,
                                origin=(0, 0))
    ring = _extrude(ring_poly, ring_z0, ring_z1)

    cap_poly = _annulus(max(r_arc + 1.0, pivot_r - post_r - 2.0),
                        ring_ri + 3.0, sections)
    for p in polar_ring(blades, pivot_r):
        cap_poly = cap_poly.difference(_disc(p, post_r + 0.05, sections))
    cap = _extrude(cap_poly, cap_z0, cap_z1)

    meta = {
        "blades": blades,
        "aperture_max": float(aperture_max),
        "aperture_min": float(aperture_min),
        "aperture_r": float(aperture_r),
        "aperture_d": float(2.0 * aperture_r),
        "blade_angle_deg": float(math.degrees(blade_rad)),
        "control_deg": float(control_deg),
        "control_range_deg": float(math.degrees(control_max)),
        "pivot_r": float(pivot_r),
        "housing_r": float(housing_r),
        "arc_centre_offset": float(math.hypot(*centre)),
    }
    parts = {"base": base, "blades": blade_meshes, "drive_ring": ring,
             "cap": cap}
    for mesh in [base, ring, cap] + blade_meshes:
        mesh.metadata.update(meta)
    return parts


# ---------------------------------------------------------------------------
# Collet chuck
# ---------------------------------------------------------------------------

def collet_chuck(bore_d=6.0, slots=4, taper_deg=8.0, collet_len=24.0,
                 nut=True, wall=3.5, slot_w=0.8, thread_pitch=2.0,
                 nut_len=14.0, nose_len=16.0, clearance=0.3, flutes=20,
                 sections=64, seg=64):
    """Build an ER-style split collet with its taper nut and spindle nose.

    The collet is a sleeve with a straight ``bore_d`` bore and an outer cone of
    half angle ``taper_deg`` on its rear third. ``slots`` radial slots are cut
    alternately from the front and rear faces so the sleeve reads as a ring of
    fingers joined by a zigzag web: pushing it back into the matching female
    cone in the spindle nose squeezes every finger inward at once and grips
    whatever is in the bore on axis. The nut threads onto the spindle nose and
    carries a second, shorter female cone that bears on the collet's front, so
    tightening it drives the collet rearward into the nose cone. Closing is
    limited by the slots shutting, so the geometric grip range is
    ``slots / 2`` front slots times ``slot_w`` of circumference, that is
    ``bore_d - slots * slot_w / (2 * pi)`` at the small end; both ends of that
    range are reported as ``metadata["grip_range"]``.

    Printed collets grip printed shafts, tubing and light tooling. They are not
    a substitute for a steel ER collet on a milling spindle: PLA creeps under
    sustained clamp load and will not hold a set diameter, so print these in
    PETG or nylon and expect to re-tighten. Treat the geometric grip range as
    an upper bound and use about half of it. Keep ``slot_w`` at or above
    0.6 mm or the slicer fuses the fingers back together. The nut and nose
    threads are printable ISO 60 degree profiles, not fits for metal hardware.

    Returns ``{"collet", "nut", "spindle_nose"}`` posed on a common axis, with
    ``"nut"`` omitted when ``nut=False``. Z zero is the collet's rear face and
    +Z points out the front of the chuck. Print all three axis-vertical: the
    slots then run along the build direction and the cones stay inside the
    45 degree overhang limit for ``taper_deg`` under 45.
    Units are mm and degrees.
    """
    slots = int(round(slots))
    if slots < 2 or slots % 2:
        raise ValueError("collet_chuck(): slots must be an even count >= 2")
    if bore_d <= 0 or collet_len <= 0 or wall <= 0:
        raise ValueError("collet_chuck(): invalid bore, length or wall")
    if not 2.0 <= taper_deg <= 30.0:
        raise ValueError("collet_chuck(): taper_deg must be 2..30")
    if slot_w < 0.4:
        raise ValueError("collet_chuck(): slot_w below one nozzle width")
    if clearance < 0.1 or thread_pitch <= 0 or nut_len <= 0 or nose_len <= 0:
        raise ValueError("collet_chuck(): invalid clearance, thread or lengths")
    bore_r = bore_d / 2.0
    body_r = bore_r + wall
    taper_len = collet_len * 0.45
    drop = taper_len * math.tan(math.radians(taper_deg))
    rear_r = body_r - drop
    if rear_r < bore_r + 0.8:
        raise ValueError(
            "collet_chuck(): taper eats the rear wall; shorten collet_len, "
            "reduce taper_deg or thicken wall")
    close_d = slots * slot_w / (2.0 * math.pi)
    if close_d >= bore_d:
        raise ValueError("collet_chuck(): slots wider than the bore can close")

    collet = uni([
        frustum(rear_r, body_r, taper_len, z0=0.0, sections=sections),
        cyl(body_r, collet_len - taper_len,
            center=(0, 0, (collet_len + taper_len) / 2.0), sections=sections),
    ])
    collet = sub(collet, cyl(bore_r, collet_len + 4.0,
                             center=(0, 0, collet_len / 2.0),
                             sections=sections))
    slot_len = collet_len * 0.74
    reach_lo = bore_r - 0.6
    reach_hi = body_r + 1.0
    cutters = []
    mid_r = (reach_lo + reach_hi) / 2.0
    for index, (px, py) in enumerate(polar_ring(slots, mid_r)):
        front = index % 2 == 0
        z0 = collet_len - slot_len if front else -1.0
        z1 = collet_len + 1.0 if front else slot_len
        foot = collet_len if front else 0.0
        parts = slot_cutter(reach_hi - reach_lo, slot_w, z0, z1,
                            cx=mid_r, cy=0.0,
                            foot_z=foot, dogbone_r=slot_w / 2.0,
                            foot_relief=0.2, eps=0.2)
        turn = tf.rotation_matrix(math.atan2(py, px), (0, 0, 1))
        for part in parts:
            part.apply_transform(turn)
        cutters.extend(parts)
    collet = sub(collet, uni(cutters))

    seat_r = body_r + clearance
    shank_r = seat_r + 2.0
    thread_d = 2.0 * (shank_r + 0.75 * thread_pitch)
    flange_r = shank_r + 3.0
    thread_len = max(taper_len + 4.0, collet_len * 0.7)
    if thread_len >= collet_len - 2.0:
        raise ValueError(
            "collet_chuck(): collet too short for the nose thread")
    nose = uni([
        cyl(flange_r, nose_len, center=(0, 0, -nose_len / 2.0),
            sections=sections),
        cyl(shank_r, thread_len, center=(0, 0, thread_len / 2.0),
            sections=sections),
        thread_solid(thread_d, thread_len, pitch=thread_pitch, seg=seg),
    ])
    nose = sub(nose, uni([
        frustum(rear_r + clearance, seat_r, taper_len, z0=-0.2,
                sections=sections),
        cyl(seat_r, thread_len + 4.0,
            center=(0, 0, taper_len + (thread_len + 4.0) / 2.0 - 0.2),
            sections=sections),
        cyl(bore_r + 1.0, nose_len + 2.0,
            center=(0, 0, -nose_len / 2.0), sections=sections),
    ]))

    parts = {"collet": collet, "spindle_nose": nose}
    if nut:
        nut_r = thread_d / 2.0 + 3.5
        cone_len = nut_len * 0.4
        nut_z1 = collet_len + cone_len / 2.0 + 1.0
        nut_z0 = nut_z1 - nut_len
        if nut_z0 < taper_len:
            raise ValueError("collet_chuck(): nut_len too long for the collet")
        body = cyl(nut_r, nut_len, center=(0, 0, (nut_z0 + nut_z1) / 2.0),
                   sections=sections)
        cone_z0 = collet_len - cone_len / 2.0
        body = sub(body, uni([
            cyl(bore_r + 1.5, nut_z1 - cone_z0 - cone_len + 2.0,
                center=(0, 0, (cone_z0 + cone_len + nut_z1 + 2.0) / 2.0),
                sections=sections),
            frustum(body_r + 1.6, body_r - 1.4, cone_len, z0=cone_z0,
                    sections=sections),
            cyl(seat_r + 0.9, cone_z0 - nut_z0 + 1.0,
                center=(0, 0, (nut_z0 - 1.0 + cone_z0) / 2.0),
                sections=sections),
        ]))
        # The nut's internal thread has to start a whole number of pitches up
        # from the nose's, or the two helices are out of phase and the posed
        # assembly interferes even though each part is fine on its own.
        cut_z0 = math.floor((nut_z0 - 1.0) / thread_pitch) * thread_pitch
        thread_cut = thread_solid(thread_d, cone_z0 - cut_z0,
                                  pitch=thread_pitch, internal=True,
                                  clear=2.0 * clearance, seg=seg)
        thread_cut.apply_translation((0, 0, cut_z0))
        body = sub(body, thread_cut)
        body = knurl(body, nut_r, nut_z0 + 1.0, nut_z1 - 1.0,
                     n=int(flutes), depth=0.45)
        parts["nut"] = body

    meta = {
        "bore_d": float(bore_d),
        "slots": slots,
        "taper_deg": float(taper_deg),
        "collet_len": float(collet_len),
        "grip_range": (float(bore_d - close_d), float(bore_d)),
        "close_d": float(close_d),
        "thread_d": float(thread_d),
        "thread_pitch": float(thread_pitch),
        "body_d": float(2.0 * body_r),
        "rear_d": float(2.0 * rear_r),
    }
    for mesh in parts.values():
        mesh.metadata.update(meta)
    return parts


# ---------------------------------------------------------------------------
# Eccentric cam clamp
# ---------------------------------------------------------------------------

def eccentric_cam_clamp(cam_r=14.0, ecc=4.0, handle_len=42.0, bore_d=5.0,
                        handle_deg=0.0, overcentre_deg=8.0, width=8.0,
                        handle_w=7.0, handle_h=6.0, follower_t=6.0,
                        margin=5.0, base_h=3.0, gap=0.3, clearance=0.3,
                        plate_w=34.0, sections=64):
    """Build an over-centre eccentric cam clamp posed at one handle angle.

    A disc of radius ``cam_r`` turns on a pivot offset ``ecc`` from its centre,
    so the lift it presents to the follower plate above it is
    ``cam_r + ecc * cos(theta)`` and the full clamping throw is ``2 * ecc``.
    ``handle_deg`` measures the handle away from the clamped position: at zero
    the eccentric sits ``overcentre_deg`` past top dead centre, so the follower
    has already come back down off the peak by
    ``ecc * (1 - cos(overcentre_deg))`` and the clamped part's spring-back
    pushes the handle further into its stop instead of backing it out. That is
    what makes it hold under vibration. Unlike ``linkages.toggle_clamp``, which
    is a knee linkage, the mechanical advantage here comes from the eccentric
    radius alone: the pressure angle is ``atan(ecc * sin(theta) / cam_r)``, so a
    small ``ecc`` relative to ``cam_r`` buys force at the cost of throw.

    Everything is planar and stacked in Z. The cam disc and the follower share
    one layer ``width`` thick so the disc can push the plate; the lever sits in
    its own layer ``handle_h`` thick above them, which is what lets it swing
    right round without striking the plate it is clamping. The follower is
    guided by a tongue running in a pocket in the base's top face rather than by
    rails beside it, for the same reason: rails would stand in the layer the
    handle sweeps. Print it flat as drawn, base down, and the motion stays in
    the print plane. Nothing here is captive in Z, so the printed pin is a slip
    fit through the base bore and the stack wants a washer or a screw head on
    top. ``metadata`` reports ``throw``, ``lift``, ``lift_max``, ``lift_min``,
    ``pressure_angle_deg``, ``release_deg`` and ``overcentre_deg``.

    Returns ``{"base", "cam", "follower", "pin"}``. Z zero is the underside of
    the base plate; the follower rides in +Y and the pivot is at the XY origin.
    ``handle_deg`` runs a full 360 with no interference, so the clamp animates,
    but the working stroke is 0 to ``release_deg``.
    Units are mm and degrees.
    """
    if cam_r <= 0 or ecc <= 0 or handle_len <= 0 or bore_d <= 0:
        raise ValueError("eccentric_cam_clamp(): invalid cam or handle sizes")
    if ecc >= cam_r - bore_d / 2.0 - 1.2:
        raise ValueError(
            "eccentric_cam_clamp(): eccentricity breaks out of the cam disc; "
            "reduce ecc or grow cam_r")
    if not 0.0 < overcentre_deg < 45.0:
        raise ValueError("eccentric_cam_clamp(): overcentre_deg must be 0..45")
    if (width < 1.2 or handle_w < 2.4 or handle_h < gap + 1.5 or
            follower_t < 2.0 or
            margin < 2.0 or base_h < 1.2 or gap < 0.15 or clearance < 0.1 or
            plate_w < 4.0 * margin or sections < 24):
        raise ValueError("eccentric_cam_clamp(): invalid feature sizes")

    theta = math.radians(overcentre_deg + handle_deg)
    lift = cam_r + ecc * math.cos(theta)
    offset_dir = math.radians(90.0 + overcentre_deg)
    centre = (ecc * math.cos(offset_dir), ecc * math.sin(offset_dir))
    handle_dir = offset_dir + math.radians(102.0)
    tip = (handle_len * math.cos(handle_dir), handle_len * math.sin(handle_dir))

    z0 = base_h + gap
    z1 = z0 + width
    # The lever sits in its own layer above the follower, which is what lets it
    # swing a full turn without striking the plate it is clamping. Built once at
    # the clamped pose and then rotated as a rigid body, so every handle angle
    # reuses the same vertices and the pose is a true repose.
    bore = _disc((0, 0), (bore_d + clearance) / 2.0, sections)
    cam = uni([
        _extrude(_disc(centre, cam_r, sections).difference(bore), z0, z1),
        _extrude(_disc((0, 0), handle_w * 0.9, sections).difference(bore),
                 z1 - 0.01, z1 + handle_h),
        _extrude(unary_union([
            _capsule((0.0, 0.0), tip, handle_w, sections),
            _disc(tip, handle_w * 0.75, sections),
        ]).buffer(0).difference(bore), z1 + gap, z1 + handle_h),
    ])
    cam.apply_transform(tf.rotation_matrix(math.radians(handle_deg),
                                           (0, 0, 1)))

    half = plate_w / 2.0
    seat = lift + clearance
    pocket_d = base_h * 0.55
    follower = _extrude(sg.box(-half, 0.0, half, follower_t),
                        base_h - pocket_d + gap, z1)
    follower.apply_translation((0.0, seat, 0.0))

    reach = max(cam_r + ecc, handle_len * 0.55) + margin
    slide_top = cam_r + ecc + clearance + follower_t + margin
    base_poly = sg.box(-reach, -reach, reach, slide_top)
    base_poly = base_poly.difference(
        _disc((0, 0), (bore_d + clearance) / 2.0, sections))
    base = _extrude(base_poly, 0.0, base_h)
    # The follower is guided by a tongue running in a pocket in the base's top
    # face, not by rails beside it: rails would stand in the layer the handle
    # sweeps through and the handle would strike them on the way to release.
    pocket = _extrude(
        sg.box(-half - clearance, cam_r - ecc - 1.0,
               half + clearance, slide_top + 1.0),
        base_h - pocket_d, base_h + 1.0)
    base = sub(base, pocket)
    pin_top = z1 + handle_h + base_h
    pin = cyl(bore_d / 2.0, pin_top, center=(0, 0, pin_top / 2.0),
              sections=max(16, sections // 2))

    meta = {
        "throw": float(2.0 * ecc),
        "lift": float(lift),
        "lift_max": float(cam_r + ecc),
        "lift_min": float(cam_r - ecc),
        "overcentre_deg": float(overcentre_deg),
        "handle_deg": float(handle_deg),
        "cam_r": float(cam_r),
        "ecc": float(ecc),
        "release_deg": float(180.0 - overcentre_deg),
        "pressure_angle_deg": float(math.degrees(
            math.atan2(ecc * math.sin(theta), cam_r))),
    }
    parts = {"base": base, "cam": cam, "follower": follower, "pin": pin}
    for mesh in parts.values():
        mesh.metadata.update(meta)
    return parts


__all__ = (
    "iris_diaphragm",
    "iris_control_range",
    "collet_chuck",
    "eccentric_cam_clamp",
    "bellows_suction_cup",
)


def bellows_suction_cup(d=20.0, folds=2, lip_t=0.8, stem_d=6.0, barb=True,
                        sections=96):
    """Build a bellows vacuum suction cup for TPU pick-and-place tooling.

    The misumi-style bellows cup of printed pick-and-place heads: a sealing
    lip of ``lip_t`` at the ``d`` rim, ``folds`` convolutions that let the
    cup comply to uneven surfaces, and a stem of ``stem_d`` for the vacuum
    line -- a ``fluid.hose_barb`` tail when ``barb=True``, a plain tube
    otherwise. Every bellows flank runs at exactly 45 degrees from
    vertical, so the cup prints lip-down on z=0 with no support (print in
    TPU; PLA is too stiff to seal). The metadata carries ``compressed_h``
    (folds fully collapsed, for Z travel budgeting) and
    ``cup_volume_mm3`` (interior volume, for vacuum sizing). Units are mm.
    """
    if d <= 0 or lip_t < 0.4 or stem_d <= 0:
        raise ValueError("bellows_suction_cup(): d and stem_d must be "
                         "positive, lip_t at least 0.4 mm")
    folds = int(round(folds))
    if folds < 1:
        raise ValueError("bellows_suction_cup(): need at least 1 fold")
    r_hub = stem_d / 2.0 + 2.5
    if d / 2.0 - r_hub < 4.0:
        raise ValueError("bellows_suction_cup(): d too small for the stem")

    # Wall centerline: a 45-degree zigzag from the rim up to the hub.
    step_r = (d / 2.0 - r_hub) / folds
    h_fold = step_r + 2.4
    pts = [(d / 2.0 - lip_t / 2.0 - 1.5, lip_t / 2.0),
           (d / 2.0 - lip_t / 2.0, lip_t / 2.0)]
    radii = [d / 2.0]
    z = lip_t / 2.0
    for f in range(folds):
        r_a = d / 2.0 - step_r * f
        r_b = r_a - step_r
        pts.append((r_a - step_r / 2.0 - 1.2, z + step_r / 2.0 + 1.2))
        pts.append((r_b, z + h_fold))
        radii.append(r_b)
        z += h_fold
    z_body = z
    wall = sg.LineString(pts).buffer(lip_t / 2.0, cap_style=2,
                                     join_style=2)
    cup = _revolve_cup(wall, sections)

    # Hub flange and stem with the vacuum bore into the cup interior.
    flange = cyl(r_hub + 1.5, 2.0, center=(0, 0, z_body + 1.0),
                 sections=int(sections))
    parts = [cup, flange]
    if barb:
        from .fluid import hose_barb
        tail = hose_barb(tube_id=stem_d, barbs=2, foot="none",
                         sections=int(sections))
        tail.apply_translation((0, 0, z_body + 1.8))
        parts.append(tail)
        z_top = z_body + 1.8 + (tail.bounds[1][2] - tail.bounds[0][2])
    else:
        stem = cyl(stem_d / 2.0 + 1.6, 8.0,
                   center=(0, 0, z_body + 2.0 + 4.0),
                   sections=int(sections))
        parts.append(stem)
        z_top = z_body + 10.0
    cup = uni(parts)
    cup = sub(cup, cyl(stem_d / 2.0, z_top - z_body + 2.0,
                       center=(0, 0, (z_body + z_top) / 2.0),
                       sections=int(sections)))

    volume = 0.0
    for f in range(folds):
        r_a, r_b = radii[f], radii[f + 1]
        volume += math.pi / 3.0 * h_fold * (r_a ** 2 + r_a * r_b + r_b ** 2)
    cup.metadata.update({
        "d": float(d),
        "folds": int(folds),
        "lip_t": float(lip_t),
        "stem_d": float(stem_d),
        "barb": bool(barb),
        "compressed_h": float(folds * 2.0 * lip_t + 2.0),
        "cup_volume_mm3": float(volume),
    })
    return cup


def _revolve_cup(poly, sections):
    ring = orient(poly, 1.0)
    return trimesh.creation.revolve(np.asarray(ring.exterior.coords),
                                    sections=int(sections))
