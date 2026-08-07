"""Project-agnostic clutch generators (detent torque limiter, roller freewheel)."""

import math

import shapely.geometry as sg
from shapely import union_all
import trimesh

from .meshutil import largest_poly, sub, uni
from .prim import cyl


# Fixed-precision overlay grid, in mm. See the matching note in
# ``mechlib/indexing.py``: GEOS' floating-point overlay throws "found
# non-noded intersection" on near-coincident boundaries, and in the WASM
# build (Pyodide) that is a C++ abort, not a catchable Python exception.
# ``grid_size`` runs the overlay under GEOS' precision model instead, where
# noding is exact. 1e-6 mm is far below FDM resolution.
_GRID = 1e-6


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

    # ``resolution`` counts quarter-circle segments, so it is capped: past 64
    # (256 segments) the extra vertices buy nothing visually and only make
    # near-coincident boundaries more likely.
    res = min(int(sections), 64)
    void = sg.Point(0.0, 0.0).buffer(h_lo, resolution=res)
    # Each pocket's return path used to run at exactly ``h_lo``, i.e. along the
    # nominal void circle. Against the polygonal void that produced a chain of
    # crossings ~1e-3 mm apart (the vertices sit just outside the inscribed
    # facets) and, in older GEOS, a non-noded intersection. Closing the pocket
    # at the inscribed radius instead puts the whole return path strictly
    # inside the void, where the union absorbs it: same result, no grazing.
    inner = h_lo * math.cos(math.pi / (2.0 * res))
    pockets = []
    for i in range(rollers):
        a0 = i * pitch - pocket_deg / 2.0
        points = [_polar(inner, a0)]
        for s in range(1, 9):
            fraction = s / 8.0
            points.append(_polar(h_lo + ramp * fraction,
                                 a0 + pocket_deg * fraction))
        for s in range(1, 6):
            fraction = s / 6.0
            points.append(_polar(inner, a0 + pocket_deg * (1.0 - fraction)))
        pockets.append(sg.Polygon(points))
    void = union_all([void] + pockets, grid_size=_GRID)
    ring_poly = sg.Point(0.0, 0.0).buffer(ring_r, resolution=res).difference(
        void, grid_size=_GRID)
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


def dog_clutch(d=30.0, bore_d=8.0, dogs=4, dog_h=5.0, dog_frac=0.45,
               hub_len=12.0, face_gap=0.4, engage_frac=1.0,
               clearance=0.25, sections=96):
    """Build a positive-engagement dog clutch as two mating hubs.

    Each hub face carries ``dogs`` rectangular teeth of height ``dog_h``
    (mm) spanning ``dog_frac`` of each pitch sector; the mating hub's dogs
    fill the complementary sectors so the pair locks in torsion when
    pressed together. ``engage_frac`` in [0, 1] poses the axial engagement
    (1 = fully meshed with ``face_gap`` remaining, 0 = fully withdrawn by
    ``dog_h + face_gap``). Unlike a friction or detent clutch this transmits
    full torque once the dogs seat, with no slip. Returns
    ``{"hub_a", "hub_b"}`` along +Z. Units mm.
    """
    if (d <= 0 or bore_d <= 0 or dogs < 2 or dog_h < 1.2 or
            not 0.2 <= dog_frac <= 0.55 or hub_len < 1.2 or
            face_gap < 0 or not 0.0 <= engage_frac <= 1.0 or
            clearance < 0 or sections < 24):
        raise ValueError("dog_clutch(): invalid clutch dimensions")
    if (bore_d + clearance) / 2.0 + 1.2 >= d / 2.0:
        raise ValueError("dog_clutch(): bore leaves no hub wall")
    pitch = 360.0 / dogs
    dog_deg = pitch * dog_frac
    # Leave diametral clearance on the dog flanks via a reduced angle.
    flank_shrink = math.degrees(clearance / (d / 2.0)) if d > 0 else 0.0
    dog_deg = max(dog_deg - 2.0 * flank_shrink, pitch * 0.15)

    def hub(z0, phase_deg, dogs_up):
        body = cyl(d / 2.0, hub_len, (0.0, 0.0, z0 + hub_len / 2.0),
                   sections=sections)
        if bore_d > 0:
            body = sub(body, cyl((bore_d + clearance) / 2.0, hub_len + 2.0,
                                 (0.0, 0.0, z0 + hub_len / 2.0),
                                 sections=sections))
        teeth = []
        for i in range(dogs):
            a0 = phase_deg + i * pitch - dog_deg / 2.0
            sector = sg.Polygon([
                (0.0, 0.0),
                *[_polar(d / 2.0, a0 + dog_deg * s / 8.0) for s in range(9)],
            ])
            # Keep an annular dog (not to the bore).
            ring = sg.Point(0.0, 0.0).buffer(d / 2.0, resolution=48).difference(
                sg.Point(0.0, 0.0).buffer((bore_d + clearance) / 2.0 + 1.2,
                                         resolution=32))
            tooth2d = sector.intersection(ring)
            if tooth2d.is_empty:
                continue
            if tooth2d.geom_type == "MultiPolygon":
                tooth2d = max(tooth2d.geoms, key=lambda g: g.area)
            z_tooth0 = z0 + hub_len if dogs_up else z0 - dog_h
            teeth.append(_extrude(tooth2d, dog_h, z_tooth0))
        return uni([body] + teeth) if teeth else body

    # Hub A sits below, dogs pointing +Z; hub B above, dogs pointing -Z.
    withdrawn = (1.0 - engage_frac) * (dog_h + face_gap)
    z_a = 0.0
    z_b = hub_len + face_gap + withdrawn
    # Phase hub B by half a pitch so its dogs drop into hub A's gaps.
    hub_a = hub(z_a, 0.0, dogs_up=True)
    hub_b = hub(z_b, pitch / 2.0, dogs_up=False)
    metadata = {"dogs": dogs, "dog_h": dog_h, "engage_frac": engage_frac}
    hub_a.metadata.update(metadata)
    hub_b.metadata.update(metadata)
    return {"hub_a": hub_a, "hub_b": hub_b}


__all__ = (
    "torque_limiter",
    "freewheel_clutch",
    "dog_clutch",
)
