"""Project-agnostic clutch generators (detent torque limiter, roller freewheel)."""

import math

import shapely.geometry as sg
from shapely.ops import unary_union
import trimesh

from .meshutil import largest_poly, sub, uni
from .prim import cyl


def _extrude(poly, height, z0=0.0):
    if height <= 0:
        raise ValueError("extrusion height must be positive")
    if poly.geom_type != "Polygon":
        # Coarse circle sampling can leave hairline slivers around pocket
        # edges after the ring difference; keep the real body.
        poly = largest_poly(poly)
    mesh = trimesh.creation.extrude_polygon(poly, height)
    if not mesh.is_watertight:
        from .meshutil import from_manifold, to_manifold
        mesh = from_manifold(to_manifold(mesh))
    if z0:
        mesh.apply_translation((0.0, 0.0, z0))
    return mesh


def _polar(r, angle_deg):
    a = math.radians(angle_deg)
    return r * math.cos(a), r * math.sin(a)


def torque_limiter(d=30.0, bore_d=8.0, detents=6, detent_r=10.0,
                   bump_r=2.5, driver_t=4.0, driven_t=4.0, hub_r=9.0,
                   hub_len=6.0, cavity_r=6.0, cavity_depth=4.5,
                   face_gap=0.2, clearance=0.25, sections=96):
    """Build a spring-detent slip clutch as ``{"driver", "driven"}`` parts.

    The driver face carries ``detents`` radiused (hemispherical) bumps on the
    ``detent_r`` pitch circle; the driven face carries matching pockets grown
    by ``clearance`` and a hub with a spring cavity above. A compression
    spring (not included; see the gallery demo) seats the bumps in the
    pockets. Above the trip torque set by the bump radius and detent count,
    the bumps cam out of the pockets and the faces ratchet past each other,
    protecting the drivetrain, then re-engage. Parts are returned in assembled
    coordinates along +Z with the driving plate below.

    Dimensions in mm. ``driver_t``/``driven_t`` plate thicknesses, ``hub_r``/
    ``hub_len`` driven-side hub, ``cavity_r``/``cavity_depth`` spring recess in
    the hub top, ``face_gap`` axial gap between the two faces (must be <=
    ``clearance`` so the bumps stay captured by their pockets), ``clearance``
    radial pocket clearance.
    """
    if (d <= 0 or bore_d <= 0 or detents < 3 or detent_r <= 0 or
            bump_r <= 0 or driver_t < 1.2 or
            driven_t < bump_r + clearance + 1.2 or
            hub_r <= 0 or hub_len < 1.2 or
            cavity_r <= (bore_d + clearance) / 2.0 + 1.2 or
            hub_r - cavity_r < 1.2 or cavity_depth > hub_len - 1.5 or
            not 0.0 <= face_gap <= clearance or clearance < 0 or
            detent_r + bump_r + clearance > d / 2.0 - 1.2 or
            detent_r - bump_r - clearance < (bore_d + clearance) / 2.0 + 1.2 or
            sections < 24):
        raise ValueError("torque_limiter(): invalid clutch dimensions")
    detent_pitch = 2.0 * math.pi * detent_r / detents
    if detent_pitch < 2.0 * (bump_r + clearance) + 1.0:
        raise ValueError("torque_limiter(): detents crowd the pitch circle")

    bore_r = (bore_d + clearance) / 2.0
    azimuths = [2.0 * math.pi * k / detents for k in range(detents)]

    parts = [cyl(d / 2.0, driver_t, (0.0, 0.0, driver_t / 2.0),
                 sections=sections)]
    for az in azimuths:
        bump = trimesh.creation.icosphere(subdivisions=2, radius=bump_r)
        bump.apply_translation(
            (detent_r * math.cos(az), detent_r * math.sin(az), driver_t))
        parts.append(bump)
    driver = uni(parts)
    driver = sub(driver, cyl(bore_r, driver_t + 2.0,
                             (0.0, 0.0, driver_t / 2.0), sections=sections))
    driver.metadata.update({"detents": detents, "detent_r": detent_r})

    z1 = driver_t + face_gap
    top = z1 + driven_t + hub_len
    driven = uni([
        cyl(d / 2.0, driven_t, (0.0, 0.0, z1 + driven_t / 2.0),
            sections=sections),
        cyl(hub_r, hub_len, (0.0, 0.0, z1 + driven_t + hub_len / 2.0),
            sections=sections),
    ])
    pockets = []
    for az in azimuths:
        pocket = trimesh.creation.icosphere(
            subdivisions=2, radius=bump_r + clearance)
        pocket.apply_translation(
            (detent_r * math.cos(az), detent_r * math.sin(az), z1))
        pockets.append(pocket)
    driven = sub(driven, uni(pockets))
    driven = sub(driven, uni([
        cyl(bore_r, top + 2.0, (0.0, 0.0, top / 2.0), sections=sections),
        cyl(cavity_r, cavity_depth + 1.0,
            (0.0, 0.0, top - cavity_depth / 2.0 + 0.5), sections=sections),
    ]))
    driven.metadata.update({
        "detents": detents,
        "detent_r": detent_r,
        "cavity_r": cavity_r,
        "cavity_z0": top - cavity_depth,
        "cavity_z1": top,
    })
    return {"driver": driver, "driven": driven}


def freewheel_clutch(rollers=6, hub_r=8.0, roller_r=3.0, pocket_deg=45.0,
                     wedge=0.15, ramp=1.5, wall=3.0, bore_d=5.0,
                     height=8.0, clearance=0.25, sections=96):
    """Build a roller-ramp one-way (overrunning) clutch as assembled parts.

    Returns ``{"ring", "hub", "rollers"}``: a smooth inner hub, an outer ring
    whose bore carries one ramped pocket per roller, and cylindrical rollers
    posed at the free end of their pockets. Rotation that rolls the rollers
    toward the shallow pocket end wedges them between hub and ramp (drive);
    the opposite sense releases them (overrun) with no tooth steps, unlike a
    pawl ratchet. The ramp rises linearly with angle from
    ``hub_r + 2 * roller_r - wedge`` to ``hub_r + 2 * roller_r + ramp``; for
    low-friction printed surfaces keep the wedge end shallow (a few degrees of
    effective strut angle).

    Dimensions in mm, angles in degrees. ``pocket_deg`` angular pocket width,
    ``wedge`` radial interference at the tight end, ``ramp`` extra radial
    depth at the loose end, ``wall`` ring material behind the deepest pocket
    floor point, ``height`` axial height, ``clearance`` mating clearance used
    for the roller rest pose and the hub bore.
    """
    if (rollers < 3 or hub_r <= 0 or roller_r <= 0 or
            not 10.0 <= pocket_deg <= 90.0 or wedge < 0 or ramp <= 0 or
            wall < 1.2 or bore_d < 0 or height < 1.2 or
            clearance < 0 or sections < 24):
        raise ValueError("freewheel_clutch(): invalid clutch dimensions")
    if bore_d > 0 and hub_r - (bore_d + clearance) / 2.0 < 1.2:
        raise ValueError("freewheel_clutch(): bore leaves no hub wall")
    pitch = 360.0 / rollers
    if pitch < pocket_deg + 5.0:
        raise ValueError("freewheel_clutch(): pockets overlap at this count")
    h_lo = hub_r + 2.0 * roller_r - wedge
    h_hi = hub_r + 2.0 * roller_r + ramp
    ring_r = h_hi + wall
    rc = hub_r + roller_r + clearance / 2.0
    roll_half = math.degrees(math.asin(roller_r / rc))
    side = math.degrees(clearance / rc)
    offset = pocket_deg / 2.0 - roll_half - side
    if offset <= 0:
        raise ValueError("freewheel_clutch(): pocket too narrow for roller")
    floor_at = h_lo + ramp * (pocket_deg / 2.0 + offset) / pocket_deg
    if floor_at < rc + roller_r + clearance / 2.0:
        raise ValueError("freewheel_clutch(): roller does not clear the ramp")

    void = sg.Point(0.0, 0.0).buffer(h_lo, resolution=sections)
    pockets = []
    for i in range(rollers):
        a0 = i * pitch - pocket_deg / 2.0
        points = []
        for s in range(9):
            fraction = s / 8.0
            points.append(_polar(h_lo + ramp * fraction,
                                 a0 + pocket_deg * fraction))
        for s in range(1, 7):
            fraction = s / 6.0
            points.append(_polar(h_lo, a0 + pocket_deg * (1.0 - fraction)))
        pockets.append(sg.Polygon(points))
    void = unary_union([void] + pockets).buffer(0)
    ring_poly = sg.Point(0.0, 0.0).buffer(
        ring_r, resolution=sections).difference(void).buffer(0)
    ring = _extrude(ring_poly, height)
    ring.metadata.update({
        "rollers": rollers,
        "roller_r": roller_r,
        "wedge": wedge,
        "ramp": ramp,
    })

    hub = cyl(hub_r, height, (0.0, 0.0, height / 2.0), sections=sections)
    if bore_d > 0:
        hub = sub(hub, cyl((bore_d + clearance) / 2.0, height + 2.0,
                           (0.0, 0.0, height / 2.0), sections=sections))

    roller_meshes = []
    for i in range(rollers):
        az = math.radians(i * pitch + offset)
        roller_meshes.append(cyl(
            roller_r, height,
            (rc * math.cos(az), rc * math.sin(az), height / 2.0),
            sections=sections))
    return {"ring": ring, "hub": hub, "rollers": roller_meshes}


__all__ = (
    "torque_limiter",
    "freewheel_clutch",
)
