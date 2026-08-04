import math

import numpy as np
import shapely.geometry as sg
import trimesh

from mechlib.fixtures import board_cradle
from mechlib.gears import mesh_phase, spur_gear_2d
from mechlib.prim import (
    boxc,
    cyl,
    extrude_down,
    frustum,
    hex_poly,
    ideg,
    largest,
    mesh_from_tris,
    rbox,
    rot2,
    sector2d,
)
from mechlib.sweep import extrude_twist, swept_keyed_bore


def assert_positive_watertight_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def test_mesh_primitives_return_positive_watertight_meshes():
    v0, v1, v2, v3 = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
    tetrahedron = mesh_from_tris([
        (v0, v2, v1),
        (v0, v1, v3),
        (v1, v2, v3),
        (v2, v0, v3),
    ])
    disconnected = trimesh.util.concatenate([
        boxc([2, 2, 2]),
        boxc([1, 1, 1], (5, 0, 0)),
    ])
    meshes = [
        cyl(3.0, 8.0),
        boxc([2.0, 3.0, 4.0]),
        rbox([12.0, 10.0, 4.0], r=2.0),
        frustum(3.0, 1.5, 5.0),
        tetrahedron,
        largest(disconnected),
        extrude_down(sg.box(-2.0, -1.0, 2.0, 1.0), 3.0),
    ]
    for mesh in meshes:
        assert_positive_watertight_mesh(mesh)


def test_cylinder_volume_matches_analytic_volume():
    radius, height = 4.0, 11.0
    mesh = cyl(radius, height, sections=96)
    expected = math.pi * radius**2 * height
    assert math.isclose(mesh.volume, expected, rel_tol=0.02)


def test_extrude_twist_produces_watertight_helix():
    base = [(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0)]
    zlist = np.linspace(0.0, 20.0, 20)
    mesh = extrude_twist(base, None, zlist, lambda z: 360.0 * z / 20.0)
    assert_positive_watertight_mesh(mesh)


def test_spur_gear_is_valid_deterministic_polygon():
    first = spur_gear_2d(18, 1.5)
    second = spur_gear_2d(18, 1.5)
    assert isinstance(first, sg.Polygon)
    assert first.is_valid
    assert first.area > 0
    assert first.wkb == second.wkb


def test_swept_keyed_bore_increases_area():
    bore = hex_poly(6.0)
    swept = swept_keyed_bore(bore, free_angle=50)
    assert swept.area > bore.area


def test_mesh_phase_returns_float():
    assert isinstance(mesh_phase(18, 36, 27.0), float)


def test_board_cradle_returns_positive_watertight_mesh():
    mesh = board_cradle((0.0, 0.0, 30.0, 20.0, 8.0), fl=0.0)
    assert_positive_watertight_mesh(mesh)


def test_polygon_and_scalar_helpers():
    sector = sector2d(-30.0, 30.0, 5.0)
    assert isinstance(sector, sg.Polygon)
    assert sector.is_valid and sector.area > 0
    assert hex_poly(6.0).is_valid
    assert np.allclose(rot2([(1.0, 0.0)], 90.0)[0], (0.0, 1.0))
    assert isinstance(ideg(20.0), float)
