import math

import pytest
import shapely.geometry as sg
import trimesh

from mechlib.cams import (
    barrel_cam,
    cam_lift,
    cam_profile_2d,
    heart_cam,
    plate_cam,
    snail_cam,
)


def assert_polygon(poly):
    assert isinstance(poly, sg.Polygon)
    assert poly.is_valid
    assert not poly.is_empty
    assert poly.area > 0


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def _ring_points(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type == "Point":
        return [(geometry.x, geometry.y)]
    if hasattr(geometry, "geoms"):
        points = []
        for part in geometry.geoms:
            points.extend(_ring_points(part))
        return points
    return list(geometry.coords)


def radius_at(poly, angle_deg):
    """Boundary radius of a star-shaped profile along a ray from the origin."""
    angle = math.radians(angle_deg)
    ray = sg.LineString([(0.0, 0.0),
                         (1000.0 * math.cos(angle), 1000.0 * math.sin(angle))])
    hits = _ring_points(poly.boundary.intersection(ray))
    assert hits, "no profile boundary on ray at %r deg" % angle_deg
    return max(math.hypot(x, y) for x, y in hits)


CLOSED_SEGMENTS = (
    ("linear", 4.0, 90.0),
    ("shm", 2.0, 120.0),
    ("cycloidal", -6.0, 90.0),
    ("dwell", 0.0, 60.0),
)


def test_segment_validation_rejects_bad_sweeps_and_laws():
    with pytest.raises(ValueError):
        cam_profile_2d(10.0, (("linear", 4.0, 180.0), ("linear", -4.0, 170.0)))
    with pytest.raises(ValueError):
        cam_profile_2d(10.0, (("harmonic", 4.0, 180.0), ("linear", -4.0, 180.0)))
    with pytest.raises(ValueError):
        cam_profile_2d(10.0, (("dwell", 2.0, 180.0), ("linear", -2.0, 180.0)))
    with pytest.raises(ValueError):
        cam_profile_2d(10.0, (("linear", 4.0, 0.0), ("linear", -4.0, 360.0)))
    with pytest.raises(ValueError):
        cam_profile_2d(10.0, ())


def test_cam_lift_matches_analytic_laws():
    assert cam_lift(CLOSED_SEGMENTS, 0.0) == pytest.approx(0.0, abs=1e-9)
    assert cam_lift(CLOSED_SEGMENTS, 45.0) == pytest.approx(2.0, abs=1e-9)
    assert cam_lift(CLOSED_SEGMENTS, 90.0) == pytest.approx(4.0, abs=1e-9)
    # shm midpoint: half the rise.
    assert cam_lift(CLOSED_SEGMENTS, 150.0) == pytest.approx(5.0, abs=1e-9)
    # cycloidal midpoint: half the (negative) rise.
    assert cam_lift(CLOSED_SEGMENTS, 255.0) == pytest.approx(3.0, abs=1e-9)
    assert cam_lift(CLOSED_SEGMENTS, 330.0) == pytest.approx(0.0, abs=1e-9)
    assert cam_lift(CLOSED_SEGMENTS, 360.0) == pytest.approx(0.0, abs=1e-9)


def test_profile_radius_at_segment_boundaries_matches_law():
    profile = cam_profile_2d(10.0, CLOSED_SEGMENTS, n=360)
    assert_polygon(profile)
    for angle, expected in ((0.0, 10.0), (90.0, 14.0), (210.0, 16.0),
                            (300.0, 10.0)):
        assert radius_at(profile, angle) == pytest.approx(expected, abs=0.02)
    # Analytic mid-segment points.
    assert radius_at(profile, 45.0) == pytest.approx(12.0, abs=0.02)
    assert radius_at(profile, 150.0) == pytest.approx(15.0, abs=0.02)
    assert radius_at(profile, 255.0) == pytest.approx(13.0, abs=0.02)


def test_profile_is_monotonic_within_rise_and_fall_segments():
    profile = cam_profile_2d(10.0, CLOSED_SEGMENTS, n=360)
    rise = [radius_at(profile, angle) for angle in range(5, 90, 5)]
    assert all(b > a + 1e-6 for a, b in zip(rise, rise[1:]))
    shm_rise = [radius_at(profile, angle) for angle in range(95, 210, 5)]
    assert all(b > a + 1e-6 for a, b in zip(shm_rise, shm_rise[1:]))
    fall = [radius_at(profile, angle) for angle in range(215, 300, 5)]
    assert all(b < a - 1e-6 for a, b in zip(fall, fall[1:]))
    dwell = [radius_at(profile, angle) for angle in range(305, 360, 5)]
    assert dwell == pytest.approx([10.0] * len(dwell), abs=0.02)


def test_roller_compensation_offsets_pitch_curve():
    segments = (("linear", 6.0, 120.0), ("dwell", 0.0, 120.0),
                ("linear", -6.0, 120.0))
    plain = cam_profile_2d(10.0, segments, n=360)
    compensated = cam_profile_2d(10.0, segments, roller_r=3.0, n=360)
    assert_polygon(compensated)
    assert compensated.area < plain.area
    # On the high dwell the pitch curve is a true circle, so the roller
    # offset is exact: radius = base + lift - roller_r.
    assert radius_at(compensated, 180.0) == pytest.approx(13.0, abs=0.1)
    with pytest.raises(ValueError):
        cam_profile_2d(10.0, segments, roller_r=20.0, n=96)


def test_plate_cam_hub_bore_flat_and_keyway():
    plain = plate_cam(10.0, CLOSED_SEGMENTS, thickness=5.0, n=96)
    bored = plate_cam(10.0, CLOSED_SEGMENTS, thickness=5.0, bore_d=6.0, n=96)
    flatted = plate_cam(10.0, CLOSED_SEGMENTS, thickness=5.0, bore_d=6.0,
                        flat=2.3, n=96)
    keywayed = plate_cam(10.0, CLOSED_SEGMENTS, thickness=5.0, bore_d=6.0,
                         keyway_w=2.0, keyway_d=1.5, n=96)
    hubbed = plate_cam(10.0, CLOSED_SEGMENTS, thickness=5.0, bore_d=6.0,
                       flat=2.3, hub_d=14.0, hub_h=3.0, n=96)
    for mesh in (plain, bored, flatted, keywayed, hubbed):
        assert_mesh(mesh)
        assert mesh.bounds[0, 2] == pytest.approx(0.0, abs=1e-9)  # prints flat
    assert bored.volume < plain.volume
    # A D-flat removes less material than the full round bore.
    assert flatted.volume > bored.volume
    # A keyway removes more material than the round bore.
    assert keywayed.volume < bored.volume
    # The hub adds material on top of the flatted plate.
    assert hubbed.volume > flatted.volume
    assert hubbed.bounds[1, 2] == pytest.approx(8.0, abs=1e-9)
    with pytest.raises(ValueError):
        plate_cam(10.0, CLOSED_SEGMENTS, flat=2.3, bore_d=6.0, hub_d=7.0,
                  hub_h=3.0)


def test_snail_cam_spiral_rise_and_drop_face():
    cam = snail_cam(base_r=10.0, lift=8.0, thickness=5.0, rise_deg=320.0,
                    bore_d=6.0, flat=2.3)
    assert_mesh(cam)
    max_r = max(math.hypot(v[0], v[1]) for v in cam.vertices)
    assert max_r == pytest.approx(18.0, abs=0.1)
    mid_z = 2.5
    # Just before the drop face the profile is near full lift...
    inside = [17.5 * math.cos(math.radians(310.0)),
              17.5 * math.sin(math.radians(310.0)), mid_z]
    # ...and just past it the radius falls back to the base circle.
    outside = [17.5 * math.cos(math.radians(330.0)),
               17.5 * math.sin(math.radians(330.0)), mid_z]
    assert list(cam.contains([inside, outside])) == [True, False]


def test_heart_cam_is_symmetric_constant_velocity():
    cam = heart_cam(base_r=10.0, lift=6.0, thickness=5.0, bore_d=6.0)
    assert_mesh(cam)
    profile = cam_profile_2d(10.0, (("linear", 6.0, 180.0),
                                    ("linear", -6.0, 180.0)), n=360)
    assert radius_at(profile, 90.0) == pytest.approx(13.0, abs=0.02)
    assert radius_at(profile, 270.0) == pytest.approx(13.0, abs=0.02)
    assert radius_at(profile, 180.0) == pytest.approx(16.0, abs=0.02)
    assert radius_at(profile, 0.0) == pytest.approx(10.0, abs=0.02)


def test_barrel_cam_groove_closed_watertight_and_pin_clear():
    parts = barrel_cam(radius=11.0, length=28.0, groove_w=4.25, groove_d=3.0,
                       pin_d=4.0, pin_len=10.0, pin_phase_deg=40.0,
                       bore_d=6.0, flat=2.3)
    barrel, pin = parts["barrel"], parts["pin"]
    assert_mesh(barrel)
    assert_mesh(pin)
    assert barrel.volume < math.pi * 11.0 ** 2 * 28.0
    assert barrel.bounds[0, 2] >= -1e-6
    assert barrel.bounds[1, 2] <= 28.0 + 1e-6
    overlap = trimesh.boolean.intersection([barrel, pin], engine="manifold")
    overlap_volume = 0.0 if overlap is None or overlap.is_empty else abs(overlap.volume)
    assert overlap_volume < 1e-6
    with pytest.raises(ValueError):  # groove does not close
        barrel_cam(segments=(("cycloidal", 10.0, 180.0),
                             ("cycloidal", -8.0, 180.0)))
    with pytest.raises(ValueError):  # travel too long for the barrel
        barrel_cam(length=12.0)
    with pytest.raises(ValueError):  # pin does not fit the groove
        barrel_cam(pin_d=4.5)
