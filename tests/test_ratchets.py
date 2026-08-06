import math

import pytest
import shapely.geometry as sg
import trimesh

import mechlib
from mechlib.gears import rack_2d
from mechlib.mechanisms import dog_slot_coupling, helix_tube
from mechlib.ratchets import (
    arc_ratchet_2d,
    check_ratchet_sense_and_sweep,
    compliant_clutch,
    compliant_clutch_2d,
    pip_ratchet_hub,
    pip_ratchet_hub_2d,
    ratchet_ring,
    ratchet_ring_2d,
    spring_cartridge_ratchet,
    spring_cartridge_ratchet_2d,
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


def test_public_ratchet_api():
    names = (
        "ratchet_ring_2d", "ratchet_ring", "pip_ratchet_hub_2d",
        "pip_ratchet_hub", "spring_cartridge_ratchet_2d",
        "spring_cartridge_ratchet", "check_ratchet_sense_and_sweep",
        "compliant_clutch_2d", "compliant_clutch", "arc_ratchet_2d",
        "rack_2d", "helix_tube", "dog_slot_coupling",
    )
    assert all(hasattr(mechlib, name) for name in names)
    assert all(name in mechlib.__all__ for name in names)


def test_print_in_place_ratchet_pair_is_valid_clear_and_watertight():
    ring_2d = ratchet_ring_2d()
    hub_2d = pip_ratchet_hub_2d()
    assert_polygon(ring_2d)
    assert_polygon(hub_2d)
    assert ring_2d.intersection(hub_2d).area < 1e-6
    assert_mesh(ratchet_ring())
    assert_mesh(pip_ratchet_hub())


def test_shared_print_in_place_kwargs_preserve_pair_clearance():
    common = dict(teeth=12, tip_r=15.2, root_r=17.0,
                  outer_r=18.5, undercut_deg=6.0, clearance=0.2)
    ring = ratchet_ring_2d(**common)
    hub = pip_ratchet_hub_2d(**common)
    assert ring.intersection(hub).area < 1e-6


def test_spring_cartridge_builds_and_passes_sense_sweep_gate():
    ring, hub, pawls = spring_cartridge_ratchet_2d()
    for poly in (ring, hub) + pawls:
        assert_polygon(poly)
    assert ring.intersection(hub).area < 1e-6
    assert all(ring.intersection(pawl).area < 1e-6 for pawl in pawls)
    metrics = check_ratchet_sense_and_sweep(ring, pawls)
    assert metrics["drive_overlap_area"] > 0.05
    assert metrics["retracted_free_deg"] >= 30.0

    ring_mesh, hub_mesh, pawl_meshes = spring_cartridge_ratchet()
    for mesh in (ring_mesh, hub_mesh) + pawl_meshes:
        assert_mesh(mesh)


def test_compliant_clutch_modes_change_geometry_and_remain_clear():
    self_locking = compliant_clutch_2d(lock_face_frac=0.12)
    torque_limit = compliant_clutch_2d(lock_face_frac=0.34)
    for pair in (self_locking, torque_limit):
        for poly in pair:
            assert_polygon(poly)
        assert pair[0].intersection(pair[1]).area < 1e-6
    assert self_locking[0].wkb != torque_limit[0].wkb
    assert self_locking[0].area != pytest.approx(torque_limit[0].area)
    for mesh in compliant_clutch():
        assert_mesh(mesh)


def test_arc_ratchet_profiles_are_valid_and_clear():
    ring, hub = arc_ratchet_2d()
    assert_polygon(ring)
    assert_polygon(hub)
    assert ring.intersection(hub).area < 1e-6


def test_helix_tube_is_watertight_and_reaches_requested_z_span():
    spring = helix_tube(7.0, 1.15, 4.0, -4.0, 4.0)
    assert_mesh(spring)
    assert spring.bounds[0, 2] < -4.0
    assert spring.bounds[1, 2] > 4.0


def test_rack_has_requested_tooth_count_and_circular_pitch():
    n_teeth, module = 6, 1.5
    rack = rack_2d(n_teeth, module)
    assert_polygon(rack)
    tip_line = sg.LineString([(-100.0, module), (100.0, module)])
    tips = rack.boundary.intersection(tip_line)
    assert tips.geom_type == "MultiLineString"
    assert len(tips.geoms) == n_teeth
    centers = sorted((segment.bounds[0] + segment.bounds[2]) / 2.0
                     for segment in tips.geoms)
    assert all(b - a == pytest.approx(math.pi * module)
               for a, b in zip(centers, centers[1:]))


def test_dog_slot_coupling_pieces_are_watertight_and_clear_at_rest():
    boss, collar = dog_slot_coupling()
    assert_mesh(boss)
    assert_mesh(collar)
    overlap = trimesh.boolean.intersection([boss, collar], engine="manifold")
    overlap_volume = 0.0 if overlap is None or overlap.is_empty else abs(overlap.volume)
    assert overlap_volume < 1e-6
