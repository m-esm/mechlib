import math

import shapely.affinity as sa
from shapely.geometry import Point

from mechlib.sweep import loft, ring_pts


def assert_section_area(mesh, z, expected_area, rel_tol=0.10):
    section = mesh.section(
        plane_origin=[0, 0, z],
        plane_normal=[0, 0, 1],
    )
    assert section is not None
    section_2d, _ = section.to_2D()
    assert section_2d.is_closed
    polygons = section_2d.polygons_full
    assert len(polygons) == 1
    assert math.isclose(polygons[0].area, expected_area, rel_tol=rel_tol)


def test_loft_aligns_start_rotated_identical_circles():
    radius, height = 7.0, 10.0
    circle = Point(0, 0).buffer(radius, resolution=24)
    rotated = sa.rotate(circle, 90, origin=(0, 0))
    mesh = loft([
        ring_pts(circle, 64, 0),
        ring_pts(rotated, 64, height),
    ])

    assert mesh.is_volume
    assert mesh.is_watertight
    expected_area = math.pi * radius**2
    for z in (2.5, 5.0, 7.5):
        assert_section_area(mesh, z, expected_area)
    expected_volume = expected_area * height
    assert math.isclose(mesh.volume, expected_volume, rel_tol=0.10)


def test_organic_loft_demo_remains_unchanged():
    specs = [
        (Point(0, 0).buffer(7, resolution=24), 0, 7),
        (Point(1.5, 0).buffer(9, resolution=24), 7, 9),
        (Point(-1, 1).buffer(6, resolution=24), 15, 6),
        (Point(1, -1).buffer(8, resolution=24), 23, 8),
        (Point(0, 0).buffer(5, resolution=24), 30, 5),
    ]
    mesh = loft([ring_pts(poly, 64, z) for poly, z, _ in specs])

    assert mesh.is_volume
    assert mesh.is_watertight
    for (z0, radius0), (z1, radius1) in zip(
        [(z, radius) for _, z, radius in specs],
        [(z, radius) for _, z, radius in specs][1:],
    ):
        z = (z0 + z1) / 2
        radius = (radius0 + radius1) / 2
        assert_section_area(mesh, z, math.pi * radius**2)
    assert math.isclose(mesh.volume, 5022.06, rel_tol=0.01)
