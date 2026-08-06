"""Project-agnostic cam generators: motion-law synthesis, plate, snail, heart,
and barrel cams."""

import math

import numpy as np
import shapely.geometry as sg
import trimesh
import trimesh.transformations as tf

from .meshutil import from_manifold, sub, to_manifold, uni

MOTION_LAWS = ("dwell", "linear", "shm", "cycloidal")

DEFAULT_SEGMENTS = (
    ("shm", 6.0, 90.0),
    ("dwell", 0.0, 90.0),
    ("cycloidal", -6.0, 120.0),
    ("dwell", 0.0, 60.0),
)


def _largest_polygon(geometry):
    if geometry.geom_type == "Polygon":
        return geometry
    return max(geometry.geoms, key=lambda item: item.area)


def _extrude(poly, height, z0=0.0):
    if height <= 0:
        raise ValueError("extrusion height must be positive")
    mesh = trimesh.creation.extrude_polygon(poly, height)
    if not mesh.is_watertight:
        mesh = from_manifold(to_manifold(mesh))
    if z0:
        mesh.apply_translation((0.0, 0.0, z0))
    return mesh


def _law_fraction(law, u):
    """Displacement fraction (0..1) of motion ``law`` at segment fraction ``u``."""
    if law == "linear":
        return u
    if law == "shm":
        return 0.5 * (1.0 - math.cos(math.pi * u))
    if law == "cycloidal":
        return u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
    return 0.0  # dwell


def _validate_segments(segments, require_closed=False):
    if not segments:
        raise ValueError("segments must be a non-empty list")
    sweep_sum = 0.0
    rise_sum = 0.0
    for law, rise, sweep in segments:
        if law not in MOTION_LAWS:
            raise ValueError("unknown cam motion law %r" % (law,))
        if sweep <= 0:
            raise ValueError("segment sweeps must be positive (deg)")
        if law == "dwell" and abs(rise) > 1e-9:
            raise ValueError("dwell segments must have zero rise")
        sweep_sum += sweep
        rise_sum += rise
    if abs(sweep_sum - 360.0) > 1e-6:
        raise ValueError(
            "segment sweeps must sum to 360 deg (got %r)" % (sweep_sum,))
    if require_closed and abs(rise_sum) > 1e-6:
        raise ValueError("segment rises must sum to zero for a closed groove")


def cam_lift(segments, angle_deg):
    """Return the follower lift (mm) at ``angle_deg`` for a segment list.

    ``segments`` is a list of ``(law, rise_mm, sweep_deg)`` tuples whose sweeps
    must sum to 360 deg; see ``cam_profile_2d`` for the law definitions.
    """
    _validate_segments(segments)
    angle = angle_deg % 360.0
    theta = 0.0
    lift = 0.0
    for index, (law, rise, sweep) in enumerate(segments):
        if angle <= theta + sweep or index == len(segments) - 1:
            u = min(max((angle - theta) / sweep, 0.0), 1.0)
            return lift + rise * _law_fraction(law, u)
        theta += sweep
        lift += rise
    return lift


def cam_profile_2d(base_r=10.0, segments=DEFAULT_SEGMENTS, roller_r=0.0, n=96):
    """Synthesize a radial plate-cam profile as a shapely ``Polygon``.

    ``base_r`` is the base-circle radius (mm). ``segments`` is a list of
    ``(law, rise_mm, sweep_deg)`` tuples; laws are ``dwell`` (constant radius,
    zero rise required), ``linear`` (constant velocity), ``shm`` (simple
    harmonic), and ``cycloidal`` (zero end acceleration). Sweeps must sum to
    360 deg. Rises that do not sum to zero leave a radial drop face where the
    profile closes, as on a snail cam. ``roller_r`` > 0 erodes the pitch curve
    by the roller radius, giving the cut profile for a roller follower.
    ``n`` is the total number of profile samples per revolution.
    """
    if base_r <= 0 or roller_r < 0 or n < 24:
        raise ValueError("invalid cam base radius, roller radius, or sampling")
    _validate_segments(segments)
    points = []
    theta = 0.0
    lift = 0.0
    min_r = base_r
    for law, rise, sweep in segments:
        k = max(2, int(round(n * sweep / 360.0)))
        for u in np.linspace(0.0, 1.0, k, endpoint=False):
            radius = base_r + lift + rise * _law_fraction(law, u)
            angle = math.radians(theta + u * sweep)
            points.append((radius * math.cos(angle), radius * math.sin(angle)))
            min_r = min(min_r, radius)
        theta += sweep
        lift += rise
    if min_r <= 0:
        raise ValueError("cam radius must stay positive over the whole profile")
    profile = sg.Polygon(points)
    if not profile.is_valid:
        profile = profile.buffer(0)
    if roller_r > 0:
        profile = profile.buffer(-roller_r, resolution=64)
        if profile.is_empty:
            raise ValueError("roller_r is too large for this cam profile")
    return _largest_polygon(profile)


def _bore_2d(bore_d, flat, keyway_w, keyway_d, clearance):
    """Centered bore polygon: round, D-flat, or keywayed (diameters in mm)."""
    radius = (bore_d + clearance) / 2.0
    bore = sg.Point(0, 0).buffer(radius, resolution=96)
    if flat is not None:
        flat_c = flat + clearance / 2.0
        bore = bore.intersection(sg.box(-3 * radius, -3 * radius,
                                        flat_c, 3 * radius))
    if keyway_w > 0 and keyway_d > 0:
        half_w = (keyway_w + clearance) / 2.0
        bore = bore.union(sg.box(-half_w, 0.0, half_w,
                                 radius + keyway_d)).buffer(0)
    return bore


def plate_cam(base_r=10.0, segments=DEFAULT_SEGMENTS, thickness=5.0,
              roller_r=0.0, hub_d=0.0, hub_h=0.0, bore_d=0.0, flat=None,
              keyway_w=0.0, keyway_d=0.0, clearance=0.25, n=96):
    """Extrude ``cam_profile_2d`` into a plate cam; prints flat on its face.

    ``thickness`` is the cam plate height (mm). ``hub_d``/``hub_h`` add a
    raised hub on the +Z face (mm). ``bore_d`` is the shaft bore diameter
    (mm); ``flat`` (mm from axis to the chord) cuts a D-flat into it, or
    ``keyway_w``/``keyway_d`` (mm) add a rectangular keyway at +Y.
    ``clearance`` (mm) opens the bore for a running fit on the shaft.
    """
    if (thickness <= 0 or hub_d < 0 or hub_h < 0 or bore_d < 0 or
            clearance < 0 or keyway_w < 0 or keyway_d < 0):
        raise ValueError("invalid plate cam dimensions")
    if flat is not None and not 0.0 <= flat < bore_d / 2.0:
        raise ValueError("flat must lie inside the bore radius")
    if hub_d > 0 and (hub_h <= 0 or hub_d < bore_d + 2.4):
        raise ValueError("hub needs hub_h > 0 and a >= 1.2 mm wall over the bore")
    profile = cam_profile_2d(base_r, segments, roller_r=roller_r, n=n)
    if bore_d > 0:
        profile = _largest_polygon(profile.difference(
            _bore_2d(bore_d, flat, keyway_w, keyway_d, clearance)).buffer(0))
    mesh = _extrude(profile, thickness)
    if hub_d > 0:
        hub = sg.Point(0, 0).buffer(hub_d / 2.0, resolution=96)
        if bore_d > 0:
            hub = hub.difference(
                _bore_2d(bore_d, flat, keyway_w, keyway_d, clearance)).buffer(0)
        mesh = uni([mesh, _extrude(_largest_polygon(hub), hub_h, thickness)])
    return mesh


def _snail_profile_2d(base_r, lift, rise_deg, n):
    """Archimedean rise over ``rise_deg`` with a radial drop face."""
    k_rise = max(8, int(round(n * rise_deg / 360.0)))
    k_base = max(2, int(round(n * (360.0 - rise_deg) / 360.0)))
    points = []
    for u in np.linspace(0.0, 1.0, k_rise, endpoint=False):
        radius = base_r + lift * u
        angle = math.radians(u * rise_deg)
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    for u in np.linspace(0.0, 1.0, k_base, endpoint=False):
        angle = math.radians(rise_deg + u * (360.0 - rise_deg))
        points.append((base_r * math.cos(angle), base_r * math.sin(angle)))
    return sg.Polygon(points)


def snail_cam(base_r=12.0, lift=8.0, thickness=5.0, rise_deg=330.0,
              bore_d=6.0, flat=None, clearance=0.25, n=96):
    """Build a snail (drop) cam; prints flat on its face.

    The profile rises along an Archimedean spiral from ``base_r`` by ``lift``
    (mm) over ``rise_deg``, then drops radially back to the base circle.
    Rotating one way it slowly lifts a follower and suddenly releases it, as
    in clock striking works and trip hammers. ``bore_d``/``flat``/``clearance``
    match ``plate_cam``.
    """
    if (base_r <= 0 or lift <= 0 or thickness <= 0 or
            not 0.0 < rise_deg < 360.0 or bore_d < 0 or clearance < 0):
        raise ValueError("invalid snail cam dimensions")
    if flat is not None and not 0.0 <= flat < bore_d / 2.0:
        raise ValueError("flat must lie inside the bore radius")
    profile = _snail_profile_2d(base_r, lift, rise_deg, n)
    if bore_d > 0:
        profile = _largest_polygon(profile.difference(
            _bore_2d(bore_d, flat, 0.0, 0.0, clearance)).buffer(0))
    return _extrude(profile, thickness)


def heart_cam(base_r=10.0, lift=6.0, thickness=5.0, roller_r=0.0,
              bore_d=6.0, flat=None, clearance=0.25, n=96):
    """Build a heart-shaped constant-velocity cam; prints flat on its face.

    Symmetric linear rise over 180 deg and linear return over 180 deg convert
    uniform rotation into uniform-velocity reciprocation; a spring-loaded
    lever pressing on the profile also finds a unique reset angle, the
    chronograph reset-to-zero heart piece. Arguments match ``plate_cam``.
    """
    segments = (("linear", lift, 180.0), ("linear", -lift, 180.0))
    return plate_cam(base_r, segments, thickness=thickness, roller_r=roller_r,
                     bore_d=bore_d, flat=flat, clearance=clearance, n=n)


def barrel_cam(radius=11.0, length=28.0, groove_w=4.25, groove_d=3.0,
               segments=(("cycloidal", 10.0, 180.0),
                         ("cycloidal", -10.0, 180.0)),
               pin_d=4.0, pin_len=10.0, pin_phase_deg=0.0,
               bore_d=0.0, flat=None, clearance=0.25, n=96):
    """Build a barrel (cylindrical) cam with a closed groove and follower pin.

    The groove centerline follows ``z(theta)`` on the drum surface, where the
    ``segments`` motion laws (as in ``cam_profile_2d``) give the axial travel
    in mm per sweep; the rises must sum to zero so the groove closes. The
    groove is centered on ``length`` and widened perpendicular to its slope.
    ``groove_w``/``groove_d`` are the groove width/depth (mm); ``pin_d`` must
    leave ``clearance`` (mm) inside ``groove_w``. ``bore_d``/``flat`` cut the
    shaft bore. Returns ``{"barrel": mesh, "pin": mesh}`` posed in assembly
    with the pin seated in the groove at ``pin_phase_deg``. Prints standing on
    end (axis +Z).
    """
    if (radius <= 0 or length <= 0 or groove_w <= 0 or groove_d <= 0 or
            groove_d >= radius or pin_d <= 0 or pin_len <= 0 or
            bore_d < 0 or clearance < 0 or n < 24):
        raise ValueError("invalid barrel cam dimensions")
    if pin_d + clearance > groove_w:
        raise ValueError("pin_d + clearance must not exceed groove_w")
    if flat is not None and not 0.0 <= flat < bore_d / 2.0:
        raise ValueError("flat must lie inside the bore radius")
    if bore_d > 0 and radius - groove_d < (bore_d + clearance) / 2.0 + 1.2:
        raise ValueError("groove breaks into the bore; reduce groove_d or bore_d")
    _validate_segments(segments, require_closed=True)

    angles = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    lifts = np.array([cam_lift(segments, math.degrees(a)) for a in angles])
    z_off = 0.5 * (length - (float(lifts.max()) + float(lifts.min())))
    zs = z_off + lifts
    dtheta = angles[1] - angles[0]
    slopes = (np.roll(lifts, -1) - np.roll(lifts, 1)) / (2.0 * dtheta * radius)
    half_w = 0.5 * groove_w * np.sqrt(1.0 + slopes ** 2)
    if (zs - half_w).min() < 1.2 or (zs + half_w).max() > length - 1.2:
        raise ValueError(
            "groove breaks the barrel ends; increase length or reduce travel")

    # Closed ribbon cutter: a 4-point radial/axial ring per station, swept
    # around the drum and closing on itself.
    r_in = radius - groove_d
    r_out = radius + 0.6
    verts = []
    for angle, z, hw in zip(angles, zs, half_w):
        ca, sa = math.cos(angle), math.sin(angle)
        verts.extend((
            (r_in * ca, r_in * sa, z - hw),
            (r_out * ca, r_out * sa, z - hw),
            (r_out * ca, r_out * sa, z + hw),
            (r_in * ca, r_in * sa, z + hw),
        ))
    faces = []
    for i in range(n):
        j = (i + 1) % n
        for side in range(4):
            side2 = (side + 1) % 4
            faces.append((i * 4 + side, j * 4 + side, j * 4 + side2))
            faces.append((i * 4 + side, j * 4 + side2, i * 4 + side2))
    cutter = trimesh.Trimesh(vertices=np.array(verts), faces=np.array(faces),
                             process=False)
    if cutter.volume < 0:
        cutter.invert()

    drum = trimesh.creation.cylinder(radius=radius, height=length, sections=96)
    drum.apply_translation((0.0, 0.0, length / 2.0))
    barrel = sub(drum, cutter)
    if bore_d > 0:
        bore = _extrude(_bore_2d(bore_d, flat, 0.0, 0.0, clearance),
                        length + 1.0, z0=-0.5)
        barrel = sub(barrel, bore)

    z_phase = z_off + cam_lift(segments, pin_phase_deg)
    pin_center = radius - groove_d + clearance + pin_len / 2.0
    pin = trimesh.creation.cylinder(radius=pin_d / 2.0, height=pin_len,
                                    sections=48)
    pin.apply_transform(tf.rotation_matrix(math.pi / 2.0, [0, 1, 0]))
    pin.apply_translation((pin_center, 0.0, z_phase))
    pin.apply_transform(tf.rotation_matrix(
        math.radians(pin_phase_deg), [0, 0, 1]))
    return {"barrel": barrel, "pin": pin}


__all__ = (
    "MOTION_LAWS",
    "DEFAULT_SEGMENTS",
    "cam_lift",
    "cam_profile_2d",
    "plate_cam",
    "snail_cam",
    "heart_cam",
    "barrel_cam",
)
