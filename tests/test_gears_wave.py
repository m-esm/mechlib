import math

import numpy as np
import pytest
import shapely.geometry as sg
import trimesh
import trimesh.transformations as tf

from mechlib.gears import (
    _cycloidal_disc_2d,
    bevel_gear_pair,
    cycloidal_drive,
    herringbone_gear,
    mesh_phase,
)
from mechlib.linear import (
    archimedes_screw,
    differential_screw,
    scroll_drive,
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


def overlap_volume(a, b):
    overlap = trimesh.boolean.intersection([a, b], engine="manifold")
    if overlap is None or overlap.is_empty:
        return 0.0
    return abs(overlap.volume)


def test_herringbone_is_watertight_chevron_with_bore():
    gear = herringbone_gear(m=1.5, z=24, h=10.0, helix_deg=25.0, bore_d=5.0)
    assert_mesh(gear)
    assert gear.bounds[1, 2] - gear.bounds[0, 2] == pytest.approx(10.0)
    assert gear.metadata["teeth"] == 24
    assert gear.metadata["hand"] == 1
    # the two hands are mirror chevrons: same volume, different vertices
    left = herringbone_gear(m=1.5, z=18, h=8.0, helix_deg=30.0, hand=-1)
    right = herringbone_gear(m=1.5, z=18, h=8.0, helix_deg=30.0, hand=1)
    assert_mesh(left)
    assert right.volume == pytest.approx(left.volume, rel=1e-3)
    assert not np.allclose(right.vertices, left.vertices)
    # bore leaves a through hole: a probe rod along the axis stays clear
    probe = trimesh.creation.cylinder(radius=2.4, height=30.0, sections=32)
    assert overlap_volume(gear, probe) < 1e-6


def test_herringbone_pair_meshes_only_with_mirrored_hands():
    m, z = 1.5, 24
    phase = mesh_phase(z, z, 0.0)

    def posed(hand):
        gear = herringbone_gear(m=m, z=z, hand=hand)
        gear.apply_transform(tf.rotation_matrix(math.radians(phase), (0, 0, 1)))
        gear.apply_translation((m * z, 0.0, 0.0))
        return gear

    driver = herringbone_gear(m=m, z=z, hand=1)
    assert overlap_volume(driver, posed(-1)) < 1e-6
    assert overlap_volume(driver, posed(1)) > 1.0  # same hand crosses teeth


def test_herringbone_rejects_bad_params():
    with pytest.raises(ValueError):
        herringbone_gear(helix_deg=0.0)
    with pytest.raises(ValueError):
        herringbone_gear(hand=0)
    with pytest.raises(ValueError):
        herringbone_gear(z=5)


def test_cycloidal_disc_profile_is_tangent_to_all_rollers():
    pins, pin_circle_r, pin_d, ecc = 12, 22.0, 4.0, 1.5
    disc2d = _cycloidal_disc_2d(pins, pin_circle_r, pin_d, ecc,
                                clearance=0.25, samples=2112)
    assert_polygon(disc2d)
    expected = pin_d / 2.0 + 0.125
    for k in range(pins):
        angle = 2.0 * math.pi * k / pins
        pin = sg.Point(pin_circle_r * math.cos(angle) - ecc,
                       pin_circle_r * math.sin(angle))
        assert pin.distance(disc2d.exterior) == pytest.approx(expected, abs=0.03)


def test_cycloidal_drive_parts_watertight_clear_and_ratio():
    drive = cycloidal_drive()
    for mesh in drive.values():
        assert_mesh(mesh)
        assert mesh.metadata["ratio"] == 11.0
        assert mesh.metadata["hole_d"] == pytest.approx(4.0 + 2 * 1.5 + 0.25)
    names = list(drive)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            assert overlap_volume(drive[names[i]], drive[names[j]]) < 1e-6


def test_cycloidal_drive_validation():
    with pytest.raises(ValueError):
        cycloidal_drive(pins=3)
    with pytest.raises(ValueError):
        cycloidal_drive(ecc=4.0)  # ecc*pins >= pin_circle_r
    with pytest.raises(ValueError):
        cycloidal_drive(out_circle_r=16.0)  # holes break the disc edge


def test_bevel_pair_geometry_relations_and_pose():
    pair = bevel_gear_pair(m=1.5, z1=16, z2=24)
    pinion, gear = pair["pinion"], pair["gear"]
    assert_mesh(pinion)
    assert_mesh(gear)
    assert pinion.metadata["delta1_deg"] == pytest.approx(
        math.degrees(math.atan2(16, 24)))
    assert pinion.metadata["delta2_deg"] == pytest.approx(
        90.0 - math.degrees(math.atan2(16, 24)))
    assert pinion.metadata["ratio"] == pytest.approx(1.5)
    # pinion axis +Z, gear axis +Y after posing
    assert abs(pinion.bounds[1, 2]) > abs(pinion.bounds[0, 2])
    assert gear.bounds[1, 1] > 0 and gear.bounds[0, 1] >= -0.5
    # meshed with backlash: at most line-contact slivers
    assert overlap_volume(pinion, gear) < 0.5


def test_bevel_pair_bores_and_validation():
    pair = bevel_gear_pair(m=2.0, z1=12, z2=12, bore1_d=5.0, bore2_d=5.0)
    for mesh in pair.values():
        assert_mesh(mesh)
    with pytest.raises(ValueError):
        bevel_gear_pair(face_w=100.0)
    with pytest.raises(ValueError):
        bevel_gear_pair(z1=5)


def test_scroll_drive_assembly_is_clear_and_self_centering():
    drive = scroll_drive()
    assert_mesh(drive["scroll"])
    assert len(drive["jaws"]) == 3
    for jaw in drive["jaws"]:
        assert_mesh(jaw)
        assert overlap_volume(drive["scroll"], jaw) < 1e-6
        assert jaw.metadata["travel_per_rev"] == pytest.approx(5.0)
    # jaws 120 degrees apart, gripping faces on a common circle of face_r:
    # de-rotating jaw 1 by -120 deg lands its inner arc at the same min-x
    jaw0 = drive["jaws"][0]
    rotated = drive["jaws"][1].copy()
    rotated.apply_transform(tf.rotation_matrix(math.radians(-120.0), (0, 0, 1)))
    face_min_x = 10.0 * math.cos(math.radians(15.0))  # arc corner of the sector
    assert jaw0.bounds[0, 0] == pytest.approx(face_min_x, abs=0.05)
    assert rotated.bounds[0, 0] == pytest.approx(face_min_x, abs=0.05)


def test_scroll_drive_validation():
    with pytest.raises(ValueError):
        scroll_drive(spiral_pitch=3.4)  # tooth width below 1.2 mm
    with pytest.raises(ValueError):
        scroll_drive(turns=6.0)  # spiral overruns the plate
    with pytest.raises(ValueError):
        scroll_drive(face_r=24.0)  # no room for jaw teeth


def test_differential_screw_travel_metadata_and_clearance():
    screw = differential_screw(p1=2.0, p2=1.75)
    for mesh in screw.values():
        assert_mesh(mesh)
        assert mesh.metadata["travel_per_rev"] == pytest.approx(0.25)
    assert overlap_volume(screw["shaft"], screw["nut_frame"]) < 1e-6
    assert overlap_volume(screw["shaft"], screw["nut_moving"]) < 1e-6
    assert overlap_volume(screw["nut_frame"], screw["nut_moving"]) < 1e-6
    # shaft spans both sections plus the knob below
    assert screw["shaft"].bounds[0, 2] < 0.0
    assert screw["shaft"].bounds[1, 2] == pytest.approx(28.0, abs=0.01)


def test_differential_screw_pitch_grid_alignment_holds_for_other_pitches():
    screw = differential_screw(p1=2.2, p2=1.8)
    assert overlap_volume(screw["shaft"], screw["nut_frame"]) < 1e-6
    assert overlap_volume(screw["shaft"], screw["nut_moving"]) < 1e-6
    assert screw["shaft"].metadata["travel_per_rev"] == pytest.approx(0.4)
    with pytest.raises(ValueError):
        differential_screw(p1=2.0, p2=2.0)  # no differential effect


def test_archimedes_screw_inclined_and_clear():
    assembly = archimedes_screw()
    screw, trough = assembly["screw"], assembly["trough"]
    assert_mesh(screw)
    assert_mesh(trough)
    assert overlap_volume(screw, trough) < 1e-6
    # inclined: the screw leans 30 deg out of vertical, so its Y span grows
    # well past the flight diameter (2 * flight_r = 24 mm)
    y_span = screw.bounds[1, 1] - screw.bounds[0, 1]
    assert y_span > 34.0
    assert screw.metadata["travel_per_rev"] == pytest.approx(14.0)
    with pytest.raises(ValueError):
        archimedes_screw(flight_t=1.0)  # below the 1.2 mm wall rule
