"""Project-agnostic intermittent-motion and indexing generators.

External Geneva driver/wheel pair (closed-form layout plus a numeric sweep
check), anchor/deadbeat clock escapement, and a mutilated-gear intermittent
pair with a locking segment. All parts extrude flat from 2D profiles.
"""

import math

import numpy as np
import shapely.affinity as affinity
import shapely.geometry as sg
from shapely import set_precision, union_all
from shapely.ops import unary_union
import trimesh

from .gears import mesh_phase, spur_gear_2d


# Fixed-precision overlay grid, in mm. GEOS' floating-point overlay throws
# "found non-noded intersection" on near-tangent, high-vertex-count boolean
# chains; older GEOS builds (the one bundled with Pyodide) are far less
# tolerant, and there it surfaces as a C++ abort rather than a Python
# exception, so it cannot be caught. Passing ``grid_size`` routes every
# overlay through GEOS' precision model, where noding is exact by
# construction. 1e-6 mm is four orders of magnitude below FDM resolution.
_GRID = 1e-6


def _polar(r, angle_deg):
    a = math.radians(angle_deg)
    return r * math.cos(a), r * math.sin(a)


def _largest_polygon(geometry):
    if geometry.geom_type == "Polygon":
        return geometry
    return max(geometry.geoms, key=lambda item: item.area)


def _extrude(poly, height, z0=0.0):
    if height <= 0:
        raise ValueError("extrusion height must be positive")
    mesh = trimesh.creation.extrude_polygon(poly, height)
    if not mesh.is_watertight:
        # manifold3d closes solids that trip trimesh's dependency-free
        # triangulator without requiring the optional ``triangle`` package.
        from .meshutil import from_manifold, to_manifold
        mesh = from_manifold(to_manifold(mesh))
    if z0:
        mesh.apply_translation((0.0, 0.0, z0))
    return mesh


# ---------------------------------------------------------------------------
# External Geneva drive
# ---------------------------------------------------------------------------


def geneva_wheel_angle(slots, crank_r):
    """Return the crank-to-wheel angle relation of an external Geneva drive.

    The result is a callable ``wheel_angle(theta)`` giving the wheel's angular
    position in degrees for crank angle ``theta`` (degrees, 0 = drive pin on
    the line of centres, measured about the driver). It is the exact closed
    form, not a sampled table: with crank radius ``a = crank_r`` and centre
    distance ``c = a / sin(pi / slots)``, the pin at ``theta`` subtends
    ``atan2(a sin(theta), a cos(theta) - c)`` at the wheel centre. Outside the
    engagement window ``|theta| <= 90 - 180 / slots`` the pin is clear of every
    slot and the angle is held at the window edge, which is the drive's dwell.
    Over one crank turn the wheel therefore advances exactly one slot,
    ``360 / slots``.

    This is the same relation ``geneva_pair`` builds its parts around, exposed
    so callers can pose or animate a pair through its cycle without rebuilding
    the (much more expensive) full layout. Cheap: no geometry is constructed.
    """
    if slots < 3:
        raise ValueError("geneva_wheel_angle(): slots must be >= 3")
    if crank_r <= 0:
        raise ValueError("geneva_wheel_angle(): crank_r must be positive")
    a = crank_r
    c = a / math.sin(math.radians(180.0 / slots))
    theta_e = 90.0 - 180.0 / slots

    def wheel_angle(theta):
        """Wheel rotation (degrees) for crank angle ``theta`` about the driver."""
        t = (theta + 180.0) % 360.0 - 180.0
        clamped = min(max(t, -theta_e), theta_e)
        return math.degrees(math.atan2(a * math.sin(math.radians(clamped)),
                                       a * math.cos(math.radians(clamped)) - c))

    return wheel_angle


def _geneva_layout(slots, crank_r, pin_d, clearance, bore_d, pocket_margin):
    """Compute the closed-form 2D layout of an external Geneva pair.

    Returns a dict with the wheel and driver (wheel-plane) polygons in their
    posed frames plus the kinematic data needed to sweep the cycle. The
    driver centre is the origin; the wheel centre sits at ``(center_distance,
    0)``. Slot 0 of the wheel polygon points along +x in the wheel-local
    frame; ``wheel_angle(theta)`` gives the wheel rotation for crank angle
    ``theta`` (degrees, 0 = pin on the line of centres).
    """
    if slots < 3:
        raise ValueError("geneva_pair(): slots must be >= 3")
    if crank_r <= 0 or pin_d <= 0 or clearance < 0 or bore_d < 0:
        raise ValueError("geneva_pair(): invalid crank, pin, clearance, or bore value")
    n = slots
    a = crank_r
    beta = 180.0 / n
    c = a / math.sin(math.radians(beta))          # centre distance
    b = c * math.cos(math.radians(beta))          # pin radius from wheel centre at entry/exit
    pin_r = pin_d / 2.0
    slot_half = pin_r + clearance                 # slot half width
    # Rim radius: slot flanks must still guide the pin at entry/exit, where
    # the pin centre is at radius b on the slot axis.
    rim_r = math.hypot(b, slot_half) + 0.05
    slot_bottom = c - a - pin_r - 0.5
    if slot_bottom < max(0.8, bore_d / 2.0 + 1.2):
        raise ValueError(
            "geneva_pair(): slot bottoms reach the hub; increase crank_r or slots")
    hub_reach = c - rim_r - clearance - 0.10      # driver material limit near the wheel
    if hub_reach < 0.5:
        raise ValueError(
            "geneva_pair(): wheel rim crowds the driver centre; increase crank_r")

    # Locking-disc radius: limited by the pocket arcs, which must not cut the
    # slot-flank contact points (pin flank on the slot wall at entry/exit).
    pocket_limit = 1e9
    for sign in (1.0, -1.0):
        centre = _polar(c, sign * beta)
        for flank in (slot_half, -slot_half):
            point = (b, flank)
            dist = math.hypot(point[0] - centre[0], point[1] - centre[1])
            pocket_limit = min(pocket_limit, dist)
    pocket_limit -= pocket_margin
    depth_target = max(2.0, 0.25 * c)
    pocket_r = min(pocket_limit, c - rim_r + depth_target)
    if pocket_r < c - rim_r + 1.0:
        raise ValueError("geneva_pair(): no room for locking pockets")
    disc_r = pocket_r - clearance

    # Wheel polygon: rim disc minus radial slots minus locking-pocket scallops.
    # The pocket radius is chosen so each scallop is very nearly tangent to the
    # rim, so the subtrahends are unioned once and subtracted once (n sequential
    # differences compound rounding at every near-tangency) and every overlay
    # runs on the ``_GRID`` precision model.
    wheel = sg.Point(0.0, 0.0).buffer(rim_r, resolution=64)
    cuts = [affinity.rotate(sg.box(slot_bottom, -slot_half, rim_r + 1.5, slot_half),
                            k * 360.0 / n, origin=(0, 0))
            for k in range(n)]
    cuts += [sg.Point(*_polar(c, (2 * k + 1) * beta)).buffer(pocket_r, resolution=48)
             for k in range(n)]
    if bore_d > 0:
        cuts.append(sg.Point(0.0, 0.0).buffer(bore_d / 2.0, resolution=32))
    wheel = _largest_polygon(
        wheel.difference(union_all(cuts, grid_size=_GRID), grid_size=_GRID))

    theta_e = 90.0 - beta                      # crank half-angle of engagement
    # One definition of the kinematics, shared with the public accessor: the
    # crescent sweep below and any caller posing the pair must agree exactly.
    wheel_angle = geneva_wheel_angle(slots, crank_r)

    # Driver wheel-plane profile: locking disc, drive pin, and the hub web
    # that joins them below the wheel plane.
    pin = sg.Point(a, 0.0).buffer(pin_r, resolution=48)
    web_w = min(2.4, 2.0 * hub_reach)
    web = sg.LineString([(-0.55 * disc_r, 0.0), (0.0, 0.0)]).buffer(
        web_w / 2.0, cap_style=1, join_style=1)
    disc = sg.Point(0.0, 0.0).buffer(disc_r, resolution=64)

    # Crescent cutout: union over the engagement sweep of the wheel solid
    # that reaches toward the disc, computed in the driver frame. Disc
    # material within ``clearance + margin`` of the sweep is removed. During
    # dwell the surviving disc arc nests in the wheel's pocket (radius
    # ``pocket_r`` about the driver centre), so any nudge of the wheel jams a
    # pocket corner into the disc edge after at most a few degrees of lash.
    # The 121 clipped poses all carry fragments of the same probe arc, so their
    # union is a pile of near-coincident edges: the single worst noding case in
    # this module. Clip and union on the precision grid, and keep the probe
    # coarse (it only has to enclose ``disc_r + clearance + 0.15``, so its own
    # facet depth never reaches the disc edge).
    swept = []
    probe = sg.Point(0.0, 0.0).buffer(disc_r + 0.6, resolution=32)
    for theta in np.linspace(-theta_e, theta_e, 121):
        posed = affinity.rotate(wheel, wheel_angle(theta), origin=(0, 0))
        posed = affinity.translate(posed, c, 0.0)
        posed = affinity.rotate(posed, -theta, origin=(0, 0))
        hit = posed.intersection(probe, grid_size=_GRID)
        if not hit.is_empty:
            swept.append(hit)
    if not swept:
        raise ValueError("geneva_pair(): engagement sweep never reaches the disc")
    sweep = union_all(swept, grid_size=_GRID)
    # ``buffer`` is not a precision-model op, so snap its output before it is
    # used as a subtrahend: rounding the offset corners of a 1000-vertex sweep
    # leaves sub-micron edges that are exactly the degenerate input to avoid.
    cutout = set_precision(sweep.buffer(clearance + 0.15, resolution=16), _GRID)
    disc = disc.difference(cutout, grid_size=_GRID)
    if disc.area < 0.2 * math.pi * disc_r ** 2:
        raise ValueError("geneva_pair(): crescent cutout removes the locking arc")
    driver_plane = union_all([disc, pin, web], grid_size=_GRID)

    return {
        "wheel": wheel,
        "driver_plane": driver_plane,
        "center_distance": c,
        "rim_r": rim_r,
        "disc_r": disc_r,
        "pin_r": pin_r,
        "crank_r": a,
        "theta_e": theta_e,
        "wheel_angle": wheel_angle,
        "cutout": cutout,
    }


def _geneva_sweep_clear(layout, step_deg=3.0, eps=1e-6):
    """Rotate the driver through a full turn; the parts must never overlap."""
    c = layout["center_distance"]
    wheel_local = layout["wheel"]
    driver = layout["driver_plane"]
    wheel_angle = layout["wheel_angle"]
    for theta in np.arange(0.0, 360.0, step_deg):
        posed_wheel = affinity.rotate(
            wheel_local, wheel_angle(theta), origin=(0, 0))
        posed_wheel = affinity.translate(posed_wheel, c, 0.0)
        posed_driver = affinity.rotate(driver, theta, origin=(0, 0))
        if posed_wheel.intersection(posed_driver, grid_size=_GRID).area > eps:
            return False, theta
    return True, None


def geneva_pair(slots=6, crank_r=10.0, pin_d=3.0, thickness=4.0,
                clearance=0.25, bore_d=2.0, arm_thickness=2.4,
                pocket_margin=0.30):
    """Build an external Geneva (Maltese cross) pair posed mid-engagement.

    Returns ``{"driver": mesh, "wheel": mesh}``. The driver carries the crank
    pin on a lower arm and a locking disc coplanar with the wheel; its
    crescent cutout lets the wheel corners sweep past during engagement while
    the surviving disc arc locks the wheel's concave pockets during dwell.
    Centre distance is the closed form ``crank_r / sin(pi / slots)``; the pin
    enters the slots tangentially ``90 - 180 / slots`` degrees before the
    line of centres. Both parts print flat (the driver with its arm down);
    all dimensions in mm, angles in degrees. ``clearance`` is the radial gap
    between the pin and the slot flanks and between the locking surfaces.

    classic ref: 507 Mechanical Movements No. 214 region / Geneva drive.
    """
    if thickness <= 0 or arm_thickness <= 0:
        raise ValueError("geneva_pair(): thicknesses must be positive")
    layout = _geneva_layout(
        slots, crank_r, pin_d, clearance, bore_d, pocket_margin)
    ok, theta = _geneva_sweep_clear(layout)
    if not ok:
        raise ValueError(
            "geneva_pair(): parts collide at crank angle %.1f deg" % theta)
    c = layout["center_distance"]

    # Driver: bottom arm carrying the pin, hub web riser, coplanar lock disc.
    a = crank_r
    pin_r = layout["pin_r"]
    gap = 0.4
    wheel_z0 = arm_thickness + gap
    arm_w = pin_d + 2.0
    arm = sg.LineString([(0.0, 0.0), (a + pin_r + 1.0, 0.0)]).buffer(
        arm_w / 2.0, cap_style=1, join_style=1)
    arm_mesh = _extrude(_largest_polygon(arm), arm_thickness)
    web = layout["driver_plane"].intersection(
        sg.box(-layout["disc_r"], -2.0, 0.5, 2.0), grid_size=_GRID)
    web_mesh = _extrude(_largest_polygon(web), wheel_z0)
    disc_plane = layout["driver_plane"].difference(
        sg.Point(a, 0.0).buffer(pin_r + 0.05, resolution=48), grid_size=_GRID)
    disc_plane = disc_plane.difference(web, grid_size=_GRID)
    disc_plane = disc_plane.intersection(
        sg.Point(0.0, 0.0).buffer(layout["disc_r"] + 0.05, resolution=64),
        grid_size=_GRID)
    disc_mesh = _extrude(_largest_polygon(disc_plane), thickness, wheel_z0)
    pin_mesh = trimesh.creation.cylinder(
        radius=pin_r, height=wheel_z0 + thickness + 0.6, sections=48)
    pin_mesh.apply_translation((a, 0.0, (wheel_z0 + thickness + 0.6) / 2.0))
    from .meshutil import uni
    driver = uni([arm_mesh, web_mesh, disc_mesh, pin_mesh])

    wheel_poly = affinity.rotate(layout["wheel"], 180.0, origin=(0, 0))
    wheel_mesh = _extrude(wheel_poly, thickness)
    wheel_mesh.apply_translation((c, 0.0, wheel_z0))

    metadata = {
        "slots": slots,
        "center_distance": c,
        "index_angle_deg": 360.0 / slots,
        "engagement_angle_deg": 2.0 * layout["theta_e"],
        "motion_fraction": 2.0 * layout["theta_e"] / 360.0,
    }
    driver.metadata.update(metadata)
    wheel_mesh.metadata.update(metadata)
    return {"driver": driver, "wheel": wheel_mesh}


# ---------------------------------------------------------------------------
# Anchor / deadbeat escapement
# ---------------------------------------------------------------------------


def _escapement_profiles(teeth, style, wheel_r, tooth_depth, pallet_span,
                         clearance, recoil_deg):
    """Return ``(wheel_poly, anchor_poly, pivot)`` posed with one pallet on a tooth."""
    if style not in ("anchor", "deadbeat"):
        raise ValueError("escapement(): style must be 'anchor' or 'deadbeat'")
    if teeth < 12:
        raise ValueError("escapement(): teeth must be >= 12")
    if 2 * pallet_span != int(2 * pallet_span) or int(2 * pallet_span) % 2 == 0:
        raise ValueError("escapement(): pallet_span must be a half-integer (e.g. 7.5)")
    if tooth_depth <= 0 or tooth_depth > 0.35 * wheel_r:
        raise ValueError("escapement(): invalid tooth_depth")
    pitch = 360.0 / teeth
    sigma = pallet_span * pitch / 2.0
    if sigma > 80.0:
        raise ValueError("escapement(): pallet_span too wide for the tooth count")
    tip_r = wheel_r
    root_r = wheel_r - tooth_depth
    pivot_d = tip_r / math.sin(math.radians(sigma))
    pivot = (0.0, pivot_d)

    # Escape wheel: forward-raked sawtooth (club-tooth style), teeth lean CCW.
    lean, tip_land = 0.28, 0.10
    points = []
    for k in range(teeth):
        a = k * pitch
        points.append(_polar(root_r, a))
        points.append(_polar(tip_r, a + lean * pitch))
        points.append(_polar(tip_r, a + (lean + tip_land) * pitch))
    wheel = sg.Polygon(points).buffer(0)
    # Pose: a tooth-tip leading corner sits exactly at the entry pallet angle.
    alpha_e = 90.0 + sigma
    alpha_x = 90.0 - sigma
    phase = (alpha_e - lean * pitch) % pitch
    wheel = affinity.rotate(wheel, phase, origin=(0, 0))

    # Anchor blank: pivot boss, two arms, pallet wedges at the contact zones.
    half_w = 0.25 * pitch
    outer_r = tip_r + 1.5
    inner_r = root_r + 0.4

    def pallet(alpha):
        outer = [_polar(outer_r, alpha - half_w + t * 2 * half_w / 6.0)
                 for t in range(7)]
        # Trailing (impulse) side: flat tilted off the radial.
        trail_inner = _polar(inner_r, alpha + half_w - 0.10 * pitch)
        # Leading (locking) side.
        contact = _polar(tip_r, alpha)
        if style == "deadbeat":
            # Locking face: circular arc centred on the anchor pivot so a
            # locked tooth tip cannot rock the anchor (dead, no recoil).
            face_r = math.dist(pivot, contact) + clearance
            face = []
            for t in np.linspace(0.0, 1.0, 7):
                r = inner_r + t * (tip_r + 0.6 - inner_r)
                # intersect circle(pivot, face_r) with circle(O, r)
                d0, d1 = r, face_r
                dist = pivot_d
                along = (dist ** 2 - d1 ** 2 + d0 ** 2) / (2.0 * dist)
                h2 = d0 ** 2 - along ** 2
                if h2 < 0:
                    continue
                h = math.sqrt(h2)
                base = (pivot[0] * along / dist, pivot[1] * along / dist)
                x = base[0] - h * pivot[1] / dist
                y = base[1] + h * pivot[0] / dist
                # choose the intersection nearer the pallet angle
                cand = [(x, y), (base[0] + h * pivot[1] / dist,
                                 base[1] - h * pivot[0] / dist)]
                face.append(min(cand, key=lambda p: abs(
                    (math.degrees(math.atan2(p[1], p[0])) - alpha + 180.0) % 360.0 - 180.0)))
            lead = face
        else:
            # Recoil anchor: locking face is a flat tilted off the pivot arc,
            # so a driven tooth pushes the anchor backward (visible recoil).
            tilt = math.tan(math.radians(recoil_deg))
            lead = []
            for t in np.linspace(0.0, 1.0, 7):
                r = inner_r + t * (tip_r + 0.6 - inner_r)
                ang = alpha - half_w + 0.02 * pitch + tilt * (r - inner_r) / tip_r * 30.0
                lead.append(_polar(r, ang))
        return sg.Polygon(outer + [trail_inner] + lead).buffer(0)

    wedges = [pallet(alpha_e), pallet(alpha_x)]
    arms = []
    for alpha in (alpha_e, alpha_x):
        arms.append(sg.LineString(
            [pivot, _polar(outer_r - 0.5, alpha)]).buffer(1.0, cap_style=1))
    boss = sg.Point(*pivot).buffer(4.0, resolution=64)
    stem = sg.box(-1.2, pivot_d, 1.2, pivot_d + 12.0)
    blank = unary_union([boss, stem] + arms + wedges).buffer(0)
    anchor = _largest_polygon(blank.difference(wheel.buffer(clearance)))
    return wheel, anchor, pivot


def escapement(teeth=30, style="anchor", wheel_r=22.0, tooth_depth=3.2,
               pallet_span=7.5, thickness=4.0, clearance=0.25, bore_d=3.0,
               recoil_deg=12.0):
    """Build a clock escapement: escape wheel plus anchor, posed engaged.

    Returns ``{"wheel": mesh, "anchor": mesh}`` as flat coplanar parts with
    the entry pallet resting on one tooth tip (``clearance`` gap). ``style``
    selects the classic recoil anchor (flat tilted pallet faces) or the
    Graham deadbeat (locking faces are arcs centred on the anchor pivot, so
    the wheel stands dead while locked). ``pallet_span`` counts tooth pitches
    embraced by the anchor and must be a half-integer. Units mm and degrees.

    classic ref: anchor escapement (Hooke, c. 1657) / Graham deadbeat (1715).
    """
    if thickness <= 0 or bore_d < 0 or clearance < 0:
        raise ValueError("escapement(): invalid thickness, bore, or clearance")
    wheel_poly, anchor_poly, pivot = _escapement_profiles(
        teeth, style, wheel_r, tooth_depth, pallet_span, clearance, recoil_deg)
    if bore_d > 0:
        wheel_poly = _largest_polygon(wheel_poly.difference(
            sg.Point(0.0, 0.0).buffer(bore_d / 2.0, resolution=64)).buffer(0))
        anchor_poly = _largest_polygon(anchor_poly.difference(
            sg.Point(*pivot).buffer(bore_d / 2.0, resolution=64)).buffer(0))
    wheel = _extrude(wheel_poly, thickness)
    anchor = _extrude(anchor_poly, thickness)
    metadata = {
        "teeth": teeth,
        "style": style,
        "pallet_span": pallet_span,
        "pivot_distance": pivot[1],
    }
    wheel.metadata.update(metadata)
    anchor.metadata.update(metadata)
    return {"wheel": wheel, "anchor": anchor}


# ---------------------------------------------------------------------------
# Intermittent (mutilated) gear pair
# ---------------------------------------------------------------------------


def _intermittent_profiles(module, z1, z2, groups, clearance, lock_drop,
                           margin_deg, pa, backlash):
    """Return posed ``(driver_poly, driven_poly, meta)`` for the pair."""
    if module <= 0 or z1 < 8 or z2 < 8:
        raise ValueError("intermittent_gear_pair(): invalid module or tooth counts")
    if groups < 1 or z2 % groups != 0:
        raise ValueError("intermittent_gear_pair(): groups must divide z2")
    k = z2 // groups
    if k < 3:
        raise ValueError("intermittent_gear_pair(): z2 / groups must be >= 3")
    if k >= z1 - 3:
        raise ValueError("intermittent_gear_pair(): driver arc leaves no lock segment")
    rp1 = module * z1 / 2.0
    rp2 = module * z2 / 2.0
    cd = rp1 + rp2
    lock_r = rp1 - lock_drop * module
    if lock_r <= 0:
        raise ValueError("intermittent_gear_pair(): lock_drop eats the driver")
    p1 = 360.0 / z1
    center_at = 0.0 if k % 2 == 1 else p1 / 2.0
    sector_deg = (k - 0.5) * p1
    teeth_poly = spur_gear_2d(z1, module, sector=sector_deg,
                              center_at=center_at, pa=pa, bl=backlash)
    # Locking segment: plain arc on the toothless side, held clear of the mesh.
    arc_lo = center_at - sector_deg / 2.0 - p1 / 2.0 - margin_deg
    arc_hi = center_at + sector_deg / 2.0 + p1 / 2.0 + margin_deg
    span = arc_hi - arc_lo
    big = lock_r + 2.0
    wedge = sg.Polygon(
        [(0.0, 0.0)] +
        [_polar(big, arc_lo + span * t / 48.0) for t in range(49)])
    segment = sg.Point(0.0, 0.0).buffer(lock_r, resolution=128).difference(wedge)
    driver = _largest_polygon(unary_union([teeth_poly, segment]).buffer(0))
    driver = affinity.rotate(driver, -center_at, origin=(0, 0))

    driven = spur_gear_2d(z2, module, pa=pa, bl=backlash)
    rot = mesh_phase(z1, z2, 0.0) + (180.0 / z2 if k % 2 == 0 else 0.0)
    notch_r = lock_r + clearance
    # First notch faces the driver just after the tooth arc leaves the mesh.
    phi0 = 180.0 + (k / 2.0) * (360.0 / z2)
    tip_r2 = rp2 + module
    cos_gamma = (cd ** 2 + tip_r2 ** 2 - notch_r ** 2) / (2.0 * cd * tip_r2)
    if not -1.0 < cos_gamma < 1.0:
        raise ValueError("intermittent_gear_pair(): lock notch misses the driven gear")
    gamma = math.degrees(math.acos(cos_gamma))
    if 2.0 * gamma > 360.0 / groups - 0.5 * (360.0 / z2):
        raise ValueError("intermittent_gear_pair(): lock notches eat the tooth groups")
    for i in range(groups):
        phi = phi0 + i * 360.0 / groups - rot
        centre = _polar(cd, phi)
        driven = driven.difference(
            sg.Point(*centre).buffer(notch_r, resolution=96))
    driven = _largest_polygon(driven.buffer(0))
    driven = affinity.rotate(driven, rot, origin=(0, 0))
    driven = affinity.translate(driven, cd, 0.0)
    meta = {
        "pitches_advanced": k,
        "advance_per_rev_deg": 360.0 / groups,
        "center_distance": cd,
        "lock_r": lock_r,
    }
    return driver, driven, meta


def intermittent_gear_pair(module=1.5, z1=18, z2=18, groups=3, thickness=4.0,
                           clearance=0.25, lock_drop=0.5, margin_deg=12.0,
                           bore_d=0.0):
    """Build a mutilated-gear intermittent pair, posed meshed mid-engagement.

    Returns ``{"driver": mesh, "driven": mesh}``. The driver keeps ``z2 //
    groups`` involute teeth on one arc (profiles from ``spur_gear_2d``) and a
    plain locking segment elsewhere; the driven gear carries matching concave
    locking notches between its tooth groups. Each driver revolution advances
    the driven gear by ``360 / groups`` degrees, then the segment rides the
    notch and holds it. Units mm and degrees; both parts print flat.

    classic ref: 507 Mechanical Movements, intermittent rotary gearing plates.
    """
    if thickness <= 0 or clearance < 0 or bore_d < 0:
        raise ValueError("intermittent_gear_pair(): invalid thickness or clearance")
    driver_poly, driven_poly, meta = _intermittent_profiles(
        module, z1, z2, groups, clearance, lock_drop, margin_deg, 20.0, 0.35)
    if bore_d > 0:
        driver_poly = _largest_polygon(driver_poly.difference(
            sg.Point(0.0, 0.0).buffer(bore_d / 2.0, resolution=64)).buffer(0))
        driven_poly = _largest_polygon(driven_poly.difference(
            sg.Point(meta["center_distance"], 0.0).buffer(
                bore_d / 2.0, resolution=64)).buffer(0))
    driver = _extrude(driver_poly, thickness)
    driven = _extrude(driven_poly, thickness)
    driver.metadata.update(meta)
    driven.metadata.update(meta)
    return {"driver": driver, "driven": driven}


__all__ = (
    "geneva_pair",
    "escapement",
    "intermittent_gear_pair",
)
