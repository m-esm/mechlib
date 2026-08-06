import itertools
import math

import numpy as np
import pytest
import trimesh

from mechlib import meshutil
from mechlib.grippers import collet_chuck, eccentric_cam_clamp, iris_diaphragm


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def iris_meshes(parts):
    return [parts["base"], parts["drive_ring"], parts["cap"]] + parts["blades"]


def measured_aperture(parts, samples=120, lo=0.0, hi=14.0):
    """Binary-search the largest clear circle through the whole leaf stack."""
    angles = np.linspace(0.0, 2.0 * np.pi, samples, endpoint=False)

    def clear(radius):
        for blade in parts["blades"]:
            z = blade.bounds[0][2] + 0.5
            points = np.c_[radius * np.cos(angles), radius * np.sin(angles),
                           np.full(samples, z)]
            if blade.contains(points).any():
                return False
        return True

    for _ in range(26):
        mid = 0.5 * (lo + hi)
        if clear(mid):
            lo = mid
        else:
            hi = mid
    return lo


def test_iris_returns_named_parts_and_watertight_meshes():
    parts = iris_diaphragm()
    assert sorted(parts) == ["base", "blades", "cap", "drive_ring"]
    assert len(parts["blades"]) == 6
    for mesh in iris_meshes(parts):
        assert_mesh(mesh)
        # Each part is one body: the post holes never sever the cap, and the
        # drive slots never sever the ring.
        assert len(mesh.split(only_watertight=False)) == 1
    assert iris_diaphragm(blades=9)["base"].metadata["blades"] == 9
    assert len(iris_diaphragm(blades=9)["blades"]) == 9


def test_iris_open_aperture_matches_the_requested_maximum():
    parts = iris_diaphragm(control_deg=0.0)
    meta = parts["base"].metadata
    assert meta["aperture_r"] == pytest.approx(12.0)
    # Probing the stack finds the same circle the closed form claims; the
    # faceted leaf edge accounts for the remaining hundredths.
    assert measured_aperture(parts) == pytest.approx(12.0, abs=0.05)


def test_iris_aperture_shrinks_monotonically_to_the_requested_minimum():
    span = iris_diaphragm()["base"].metadata["control_range_deg"]
    assert 0.0 < span < 90.0
    previous = None
    for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
        parts = iris_diaphragm(control_deg=fraction * span)
        meta = parts["base"].metadata
        measured = measured_aperture(parts)
        assert measured == pytest.approx(meta["aperture_r"], abs=0.05)
        assert meta["aperture_r"] == pytest.approx(
            12.0 - 2.0 * 26.0 * math.sin(
                math.radians(meta["blade_angle_deg"]) / 2.0))
        if previous is not None:
            assert measured < previous - 0.5
        previous = measured
    assert parts["base"].metadata["aperture_r"] == pytest.approx(3.0, abs=1e-6)


def test_iris_leaves_never_touch_each_other_or_the_housing():
    span = iris_diaphragm()["base"].metadata["control_range_deg"]
    for fraction in (0.0, 0.2, 0.45, 0.7, 1.0):
        parts = iris_diaphragm(control_deg=fraction * span)
        blades = parts["blades"]
        for a, b in itertools.combinations(blades, 2):
            assert meshutil.overlap_volume(a, b) < 1e-6
        for blade in blades:
            assert meshutil.overlap_volume(blade, parts["base"]) < 1e-6
            assert meshutil.overlap_volume(blade, parts["drive_ring"]) < 1e-6
        assert meshutil.overlap_volume(parts["base"],
                                       parts["drive_ring"]) < 1e-6
    # Stacked planes: the designed running gap is what actually separates them.
    parts = iris_diaphragm()
    assert meshutil.min_distance(parts["blades"][0],
                                 parts["blades"][1]) == pytest.approx(0.25,
                                                                      abs=0.02)
    assert meshutil.min_distance(parts["blades"][0],
                                 parts["base"]) == pytest.approx(0.25,
                                                                 abs=0.02)


def test_iris_prints_over_the_base_annulus_when_wide_open():
    parts = iris_diaphragm(control_deg=0.0)
    for blade in parts["blades"]:
        radius = np.hypot(blade.vertices[:, 0], blade.vertices[:, 1])
        # Nothing overhangs the bore at the print pose, and nothing runs off
        # the outside of the base ring either.
        assert radius.min() > 12.0 - 0.05
        assert radius.max() < parts["base"].metadata["housing_r"]


def test_iris_rejects_geometry_it_cannot_satisfy():
    with pytest.raises(ValueError):
        iris_diaphragm(blades=2)
    with pytest.raises(ValueError):
        iris_diaphragm(aperture_min=30.0)
    with pytest.raises(ValueError):
        iris_diaphragm(pivot_r=10.0)
    with pytest.raises(ValueError):
        # Past the closed position the drive slot has nowhere left to push.
        iris_diaphragm(control_deg=40.0)
    with pytest.raises(ValueError):
        iris_diaphragm(blade_t=0.4)
    # A leaf sweep that runs into the neighbouring pivot posts is caught with
    # the pivot radius that would fix it.
    with pytest.raises(ValueError, match="pivot_r must be at least"):
        iris_diaphragm(pivot_r=18.0)


def test_collet_chuck_parts_are_single_watertight_bodies():
    parts = collet_chuck()
    assert sorted(parts) == ["collet", "nut", "spindle_nose"]
    for mesh in parts.values():
        assert_mesh(mesh)
        assert len(mesh.split(only_watertight=False)) == 1
    assert sorted(collet_chuck(nut=False)) == ["collet", "spindle_nose"]


def test_collet_slots_alternate_from_each_end():
    parts = collet_chuck(slots=4, collet_len=24.0)
    collet = parts["collet"]

    def pieces(z, radius):
        ring = trimesh.creation.annulus(radius - 0.15, radius + 0.15, 0.6,
                                        sections=192)
        ring.apply_translation((0, 0, z))
        band = trimesh.boolean.intersection([collet, ring], engine="manifold")
        return [p for p in band.split(only_watertight=False) if p.volume > 1e-4]

    # Two front slots reach down from the nose, two rear slots up from the
    # tail, and the middle of the sleeve sees all four.
    assert len(pieces(22.0, 5.9)) == 2
    assert len(pieces(1.0, 4.9)) == 2
    assert len(pieces(12.0, 5.9)) == 4
    assert len(collet_chuck(slots=6)["collet"].split(
        only_watertight=False)) == 1


def test_collet_taper_matches_the_requested_angle():
    for taper in (6.0, 8.0, 12.0):
        collet = collet_chuck(taper_deg=taper)["collet"]

        def radius_at(z):
            section = collet.section(plane_origin=[0, 0, z],
                                     plane_normal=[0, 0, 1])
            xy = np.asarray(section.vertices)
            return float(np.hypot(xy[:, 0], xy[:, 1]).max())

        z0, z1 = 1.0, 9.0
        rise = radius_at(z1) - radius_at(z0)
        assert math.degrees(math.atan2(rise, z1 - z0)) == pytest.approx(
            taper, abs=0.05)


def test_collet_bore_is_open_and_grip_range_matches_the_slot_width():
    parts = collet_chuck(bore_d=6.0, slots=4, slot_w=0.8)
    collet = parts["collet"]
    assert meshutil.bore_pierces(collet, [0, 0, -0.5], [0, 0, 1], 25.0, n=25)
    low, high = collet.metadata["grip_range"]
    assert high == pytest.approx(6.0)
    assert low == pytest.approx(6.0 - 4 * 0.8 / (2 * math.pi))
    # More slots close further.
    wider = collet_chuck(bore_d=6.0, slots=8, slot_w=0.8)
    assert wider["collet"].metadata["grip_range"][0] < low


def test_collet_assembly_parts_clear_each_other():
    parts = collet_chuck()
    for a, b in itertools.combinations(sorted(parts), 2):
        assert meshutil.overlap_volume(parts[a], parts[b]) < 1e-3
    assert meshutil.min_distance(parts["nut"],
                                 parts["spindle_nose"]) > 0.1


def test_collet_rejects_unprintable_or_impossible_geometry():
    with pytest.raises(ValueError):
        collet_chuck(slots=3)
    with pytest.raises(ValueError):
        collet_chuck(slot_w=0.2)
    with pytest.raises(ValueError):
        collet_chuck(taper_deg=45.0)
    with pytest.raises(ValueError, match="rear wall"):
        collet_chuck(collet_len=40.0, wall=1.5)
    with pytest.raises(ValueError):
        collet_chuck(collet_len=8.0)


def test_cam_clamp_parts_and_metadata():
    parts = eccentric_cam_clamp()
    assert sorted(parts) == ["base", "cam", "follower", "pin"]
    for mesh in parts.values():
        assert_mesh(mesh)
    meta = parts["cam"].metadata
    assert meta["throw"] == pytest.approx(8.0)
    assert meta["lift_max"] == pytest.approx(18.0)
    assert meta["lift_min"] == pytest.approx(10.0)
    assert meta["release_deg"] == pytest.approx(172.0)
    # Clamped short of top dead centre by the over-centre angle.
    assert meta["lift"] == pytest.approx(
        14.0 + 4.0 * math.cos(math.radians(8.0)))
    assert meta["lift"] < meta["lift_max"]


def test_cam_clamp_throw_measured_from_the_follower_travel():
    top = eccentric_cam_clamp(handle_deg=0.0)
    peak = eccentric_cam_clamp(handle_deg=-8.0)
    drop = eccentric_cam_clamp(handle_deg=172.0)
    # The follower is a rigid plate: its face position is the cam lift.
    face = lambda parts: float(parts["follower"].bounds[0][1])
    assert face(peak) - face(drop) == pytest.approx(8.0, abs=1e-6)
    assert face(peak) > face(top) > face(drop)
    assert face(peak) - face(top) == pytest.approx(
        4.0 * (1.0 - math.cos(math.radians(8.0))), abs=1e-6)


def test_cam_clamp_cam_radius_swings_by_the_eccentricity():
    cam = eccentric_cam_clamp(handle_deg=0.0)["cam"]
    disc = cam.slice_plane([0, 0, 8.0], [0, 0, -1])
    xy = np.asarray(disc.vertices)
    inside = np.hypot(xy[:, 0], xy[:, 1])
    inside = inside[inside > 6.0]  # drop the pivot bore
    assert inside.max() == pytest.approx(18.0, abs=0.05)
    assert inside.min() == pytest.approx(10.0, abs=0.05)


def test_cam_clamp_reposes_rigidly_without_interference():
    reference = eccentric_cam_clamp()
    for handle in (0.0, 45.0, 90.0, 135.0, 180.0, 270.0, 359.0, 360.0):
        parts = eccentric_cam_clamp(handle_deg=handle)
        for a, b in itertools.combinations(sorted(parts), 2):
            assert meshutil.overlap_volume(parts[a], parts[b]) < 1e-6
        for name, mesh in parts.items():
            assert len(mesh.vertices) == len(reference[name].vertices)
    # A full turn of the handle brings every body back where it started, which
    # is what makes the cycle animatable.
    closed = eccentric_cam_clamp(handle_deg=360.0)
    for name in reference:
        assert np.abs(np.asarray(closed[name].vertices)
                      - np.asarray(reference[name].vertices)).max() < 1e-6


def test_cam_clamp_rejects_impossible_geometry():
    with pytest.raises(ValueError):
        eccentric_cam_clamp(ecc=12.0)
    with pytest.raises(ValueError):
        eccentric_cam_clamp(overcentre_deg=0.0)
    with pytest.raises(ValueError):
        eccentric_cam_clamp(overcentre_deg=60.0)
    with pytest.raises(ValueError):
        eccentric_cam_clamp(handle_h=0.5)
    with pytest.raises(ValueError):
        eccentric_cam_clamp(cam_r=-1.0)
