import math

import numpy as np
import pytest
import trimesh
import trimesh.transformations as tf

from mechlib.joints import ball_socket_joint, gimbal_rings, knuckle_hinge
from mechlib.meshutil import bore_pierces, inside, min_distance, overlap_volume


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


# --------------------------------------------------------------------------
# ball_socket_joint
# --------------------------------------------------------------------------


def test_ball_socket_joint_parts_are_two_solid_bodies():
    parts = ball_socket_joint()
    assert set(parts) == {"ball", "socket"}
    for mesh in parts.values():
        assert_mesh(mesh)
        assert len(mesh.split(only_watertight=False)) == 1
    # The stud hangs below the ball centre, the socket shank rises above it.
    assert parts["ball"].bounds[1][2] == pytest.approx(5.0, abs=0.05)
    assert parts["ball"].bounds[0][2] < -10.0
    assert parts["socket"].bounds[1][2] > 10.0


def test_ball_socket_joint_lip_captures_the_ball():
    parts = ball_socket_joint(ball_d=10.0, capture_deg=20.0)
    meta = parts["ball"].metadata
    # Retention is geometric: the mouth is narrower than the ball.
    assert meta["mouth_d"] < 10.0
    assert meta["undercut"] == pytest.approx(5.0 * (1.0 - math.cos(
        math.radians(20.0))))
    # Pulling the stud straight out of the mouth drives it into the lip.
    pulled = parts["ball"].copy()
    pulled.apply_translation((0.0, 0.0, -1.0))
    assert overlap_volume(pulled, parts["socket"]) > 0.25
    # Seated, the ball floats on the designed running clearance.
    assert overlap_volume(parts["ball"], parts["socket"]) == pytest.approx(
        0.0, abs=1e-6)
    assert min_distance(parts["ball"], parts["socket"], n=3000) == (
        pytest.approx(0.3, abs=0.03))


def test_ball_socket_joint_swing_cone_matches_metadata():
    parts = ball_socket_joint()
    swing = parts["ball"].metadata["swing_half_deg"]
    ball_r, stem_r, cav_r = 5.0, 2.5, 5.3
    mouth_r = ball_r * math.cos(math.radians(20.0))
    assert swing == pytest.approx(math.degrees(
        math.asin(mouth_r / cav_r) - math.asin(stem_r / cav_r)))
    assert parts["ball"].metadata["swing_cone_deg"] == pytest.approx(2 * swing)
    # Inside the cone the stud is free; past it the stem fouls the mouth rim.
    for extra, expect_free in ((-2.0, True), (3.0, False)):
        posed = parts["ball"].copy()
        posed.apply_transform(tf.rotation_matrix(
            math.radians(swing + extra), (1.0, 0.0, 0.0)))
        volume = overlap_volume(posed, parts["socket"])
        assert (volume < 1e-6) is expect_free
    with pytest.raises(ValueError):
        ball_socket_joint(pose_deg=swing + 1.0)


def test_ball_socket_joint_slots_split_the_lip_but_not_the_cup():
    parts = ball_socket_joint(fingers=4, slot_w=1.4)
    socket = parts["socket"]
    assert len(socket.split(only_watertight=False)) == 1
    # A ring probe through the mouth wall meets four fingers.
    band = trimesh.creation.annulus(5.4, 7.7, 0.6, sections=128)
    band.apply_translation((0.0, 0.0, -3.0))
    pieces = trimesh.boolean.intersection([socket, band], engine="manifold")
    solids = [p for p in pieces.split(only_watertight=False) if p.volume > 1e-3]
    assert len(solids) == 4
    # Above the slot tops the cup is continuous again.
    band.apply_translation((0.0, 0.0, 3.0 + 4.5))
    pieces = trimesh.boolean.intersection([socket, band], engine="manifold")
    solids = [p for p in pieces.split(only_watertight=False) if p.volume > 1e-3]
    assert len(solids) == 1


def test_ball_socket_joint_rejects_impossible_geometry():
    with pytest.raises(ValueError):
        ball_socket_joint(capture_deg=60.0)
    with pytest.raises(ValueError):
        ball_socket_joint(capture_deg=30.0, neck_deg=31.0)
    with pytest.raises(ValueError):
        ball_socket_joint(stem_d=9.5)
    with pytest.raises(ValueError):
        ball_socket_joint(wall=0.5)
    with pytest.raises(ValueError):
        ball_socket_joint(ball_d=4.0, capture_deg=4.0)


# --------------------------------------------------------------------------
# knuckle_hinge
# --------------------------------------------------------------------------


def test_knuckle_hinge_prints_flat_with_a_running_gap():
    parts = knuckle_hinge(open_deg=180.0)
    assert set(parts) == {"leaf_a", "leaf_b"}
    for mesh in parts.values():
        assert_mesh(mesh)
        assert len(mesh.split(only_watertight=False)) == 1
        # Flat pose: both leaves sit on the bed.
        assert mesh.bounds[0][2] == pytest.approx(0.0, abs=1e-6)
    assert overlap_volume(parts["leaf_a"], parts["leaf_b"]) == pytest.approx(
        0.0, abs=1e-6)
    assert min_distance(parts["leaf_a"], parts["leaf_b"], n=4000) == (
        pytest.approx(0.3, abs=0.02))
    # The pin bore runs clear through leaf B.
    assert bore_pierces(parts["leaf_b"], (-19.0, 0.0, 3.5), (1.0, 0.0, 0.0),
                        38.0, n=40)


def test_knuckle_hinge_knuckles_alternate_along_the_pin():
    knuckles, leaf_w, gap = 5, 36.0, 0.3
    parts = knuckle_hinge(knuckles=knuckles, leaf_w=leaf_w, gap=gap,
                          open_deg=180.0)
    width = (leaf_w - (knuckles - 1) * gap) / knuckles
    assert parts["leaf_a"].metadata["knuckle_w"] == pytest.approx(width)
    owners = []
    for index in range(knuckles):
        x = -leaf_w / 2.0 + index * (width + gap) + width / 2.0
        # A point in the barrel wall, above the pin and inside the knuckle.
        probe = [[x, 0.0, 3.5 + 2.6]]
        in_a = inside(parts["leaf_a"], probe)
        in_b = inside(parts["leaf_b"], probe)
        assert in_a != in_b
        owners.append("a" if in_a else "b")
    assert owners == ["a", "b", "a", "b", "a"]


def test_knuckle_hinge_stop_face_arrests_the_leaf():
    stop = 90.0
    for open_deg, touching in ((180.0, False), (135.0, False), (stop, True)):
        parts = knuckle_hinge(stop_deg=stop, open_deg=open_deg)
        assert overlap_volume(parts["leaf_a"], parts["leaf_b"]) == (
            pytest.approx(0.0, abs=1e-6))
        distance = min_distance(parts["leaf_a"], parts["leaf_b"], n=3000)
        if touching:
            assert distance < 0.02
        else:
            assert distance == pytest.approx(0.3, abs=0.02)
    assert knuckle_hinge()["leaf_a"].metadata["travel_deg"] == 180.0 - stop
    with pytest.raises(ValueError):
        knuckle_hinge(stop_deg=90.0, open_deg=80.0)


def test_knuckle_hinge_friction_band_is_an_interference_fit():
    loose = knuckle_hinge(friction=0.0)
    tight = knuckle_hinge(friction=0.12)
    assert overlap_volume(loose["leaf_a"], loose["leaf_b"]) == pytest.approx(
        0.0, abs=1e-6)
    band = overlap_volume(tight["leaf_a"], tight["leaf_b"])
    # Two leaf-B knuckles, each gripped over a 1.2 mm band of pin.
    annulus = math.pi * ((1.5 + 0.3 + 0.06) ** 2 - (1.5 + 0.3) ** 2)
    assert 0.3 * annulus * 1.2 * 2 < band < annulus * 1.2 * 2
    assert tight["leaf_a"].metadata["interference"] == 0.12


def test_knuckle_hinge_rejects_impossible_geometry():
    with pytest.raises(ValueError):
        knuckle_hinge(pin_d=5.0)
    with pytest.raises(ValueError):
        knuckle_hinge(knuckles=2)
    with pytest.raises(ValueError):
        knuckle_hinge(leaf_w=10.0, knuckles=9)
    with pytest.raises(ValueError):
        knuckle_hinge(leaf_t=8.0)
    with pytest.raises(ValueError):
        knuckle_hinge(stop_deg=200.0)


# --------------------------------------------------------------------------
# gimbal_rings
# --------------------------------------------------------------------------


def test_gimbal_rings_are_separate_nested_bodies():
    parts = gimbal_rings()
    assert set(parts) == {"ring_0", "ring_1", "ring_2"}
    for mesh in parts.values():
        assert_mesh(mesh)
        assert len(mesh.split(only_watertight=False)) == 1
    stack = trimesh.util.concatenate([parts["ring_%d" % i] for i in range(3)])
    assert len(stack.split(only_watertight=True)) == 3
    radii = parts["ring_0"].metadata["ring_radii"]
    assert radii[0] > radii[1] > radii[2]
    assert parts["ring_0"].metadata["axes"] == ("fixed", "x", "y")
    for index in range(3):
        assert parts["ring_%d" % index].metadata["ring_index"] == index


def test_gimbal_rings_hold_the_running_gap_through_the_swing():
    crit = gimbal_rings()["ring_0"].metadata["crit_tilt_deg"]
    for tilt in (0.0, crit, 60.0):
        parts = gimbal_rings(tilt_deg=tilt)
        for i in range(3):
            for j in range(i + 1, 3):
                assert overlap_volume(parts["ring_%d" % i],
                                      parts["ring_%d" % j]) == pytest.approx(
                    0.0, abs=1e-6)
        for i in range(2):
            assert min_distance(parts["ring_%d" % i], parts["ring_%d" % (i + 1)],
                                n=2500) == pytest.approx(0.3, abs=0.02)


def test_gimbal_ring_radii_follow_the_corner_clearance_law():
    ring_w, ring_t, gap = 5.0, 6.5, 0.3
    radii = gimbal_rings()["ring_0"].metadata["ring_radii"]
    for outer, inner in zip(radii, radii[1:]):
        reach = outer - ring_w - gap
        assert inner == pytest.approx(math.sqrt(reach ** 2 - (ring_t / 2) ** 2))
        # The corner of the inner ring is exactly ``gap`` off the parent bore.
        assert math.hypot(inner, ring_t / 2) == pytest.approx(
            outer - ring_w - gap)


def test_gimbal_rings_repose_rigidly_from_tilt():
    flat = gimbal_rings(tilt_deg=0.0)
    tilted = gimbal_rings(tilt_deg=33.0)
    steps = {"ring_0": np.eye(4),
             "ring_1": tf.rotation_matrix(math.radians(33.0), (1, 0, 0)),
             "ring_2": tf.rotation_matrix(math.radians(33.0), (1, 0, 0))
             @ tf.rotation_matrix(math.radians(33.0), (0, 1, 0))}
    for name, transform in steps.items():
        posed = flat[name].copy()
        posed.apply_transform(transform)
        assert np.abs(posed.vertices - tilted[name].vertices).max() < 1e-9


def test_gimbal_rings_socket_keeps_its_teardrop_roof():
    parts = gimbal_rings()
    radii = parts["ring_0"].metadata["ring_radii"]
    # Directly over the ring-0 socket there is still solid ring above it.
    probe = [[radii[1] + 1.0, 0.0, 6.5 / 2.0 - 0.3]]
    assert inside(parts["ring_0"], probe)


def test_gimbal_rings_two_ring_cardan_and_bad_arguments():
    pair = gimbal_rings(rings=2)
    assert set(pair) == {"ring_0", "ring_1"}
    assert pair["ring_1"].metadata["pivot_axis"] == "x"
    with pytest.raises(ValueError):
        gimbal_rings(rings=1)
    with pytest.raises(ValueError):
        gimbal_rings(ring_t=3.0)
    with pytest.raises(ValueError):
        gimbal_rings(rings=4, outer_d=30.0)
    with pytest.raises(ValueError):
        gimbal_rings(pin_len=0.2)
