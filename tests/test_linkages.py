import math

import pytest
import trimesh

from mechlib.linkages import (
    four_bar,
    four_bar_pose,
    link_bar,
    quick_return,
    quick_return_ratio,
    scotch_yoke,
    toggle_clamp,
)


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def overlap_volume(a, b):
    overlap = trimesh.boolean.intersection([a, b], engine="manifold")
    if overlap is None or overlap.is_empty:
        return 0.0
    return abs(overlap.volume)


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def test_link_bar_dimensions_and_bores():
    bar = link_bar(20.0, 6.0, 4.0, 3.0)
    assert_mesh(bar)
    # overall length = length + width, bores keep the volume under the envelope
    assert bar.bounds[1, 0] - bar.bounds[0, 0] == pytest.approx(26.0, abs=1e-6)
    envelope = (20.0 * 6.0 + math.pi * 9.0) * 4.0
    assert bar.volume < envelope
    with pytest.raises(ValueError):
        link_bar(20.0, 4.0, 4.0, 4.0)  # bore + clearance swallows the bar


def test_four_bar_pose_preserves_link_lengths():
    joints = four_bar_pose(25.0, 12.5, 25.0, 25.0, 60.0)
    assert dist(joints["O1"], joints["A"]) == pytest.approx(12.5, abs=1e-9)
    assert dist(joints["A"], joints["B"]) == pytest.approx(25.0, abs=1e-9)
    assert dist(joints["B"], joints["O2"]) == pytest.approx(25.0, abs=1e-9)
    assert dist(joints["O1"], joints["O2"]) == pytest.approx(25.0, abs=1e-9)
    crossed = four_bar_pose(25.0, 12.5, 25.0, 25.0, 60.0, branch=-1)
    assert dist(crossed["A"], crossed["B"]) == pytest.approx(25.0, abs=1e-9)
    assert dist(crossed["B"], crossed["O2"]) == pytest.approx(25.0, abs=1e-9)
    assert dist(crossed["B"], joints["B"]) > 1.0  # the other assembly branch


def test_four_bar_unreachable_pose_raises():
    with pytest.raises(ValueError):
        four_bar_pose(25.0, 12.5, 5.0, 5.0, 0.0)
    with pytest.raises(ValueError):
        four_bar(25.0, 12.5, 5.0, 5.0, 0.0)


def test_four_bar_pins_sit_on_solved_joints():
    parts = four_bar(crank_angle_deg=60.0, coupler_ext=25.0)
    for name in ("ground", "crank", "coupler", "rocker", "trace"):
        assert_mesh(parts[name])
    assert len(parts["pins"]) == 4
    joints = parts["joints"]
    for pin, key in zip(parts["pins"], ("O1", "O2", "A", "B")):
        centroid = pin.centroid
        assert centroid[0] == pytest.approx(joints[key][0], abs=1e-6)
        assert centroid[1] == pytest.approx(joints[key][1], abs=1e-6)
    # stacked links and pinned joints never intersect in assembly
    links = [parts[n] for n in ("ground", "rocker", "coupler", "crank")]
    for i in range(len(links)):
        for j in range(i + 1, len(links)):
            assert overlap_volume(links[i], links[j]) < 1e-6
    for pin in parts["pins"]:
        for link in links:
            assert overlap_volume(pin, link) < 1e-6


def test_four_bar_hoecken_defaults_fully_rotate():
    for angle in range(0, 360, 15):
        parts = four_bar(crank_angle_deg=float(angle))
        assert dist(parts["joints"]["A"], parts["joints"]["B"]) == \
            pytest.approx(25.0, abs=1e-9)


def test_toggle_clamp_dead_center_and_overcenter():
    dead = toggle_clamp(overcenter_deg=0.0)
    joints = dead["joints"]
    assert joints["C"][0] == pytest.approx(14.0, abs=1e-9)
    assert joints["C"][1] == pytest.approx(0.0, abs=1e-9)
    # handle pivot, knee, and arm joint exactly collinear at dead center
    P1, K, C = joints["P1"], joints["K"], joints["C"]
    cross = ((C[0] - P1[0]) * (K[1] - P1[1]) -
             (C[1] - P1[1]) * (K[0] - P1[0]))
    assert cross == pytest.approx(0.0, abs=1e-9)

    locked = toggle_clamp(overcenter_deg=4.0)
    lj = locked["joints"]
    # over-center pushes the clamp joint onto the press side
    assert lj["C"][1] < -1e-3
    assert dist(lj["P1"], lj["K"]) == pytest.approx(22.0, abs=1e-9)
    assert dist(lj["K"], lj["C"]) == pytest.approx(10.0, abs=1e-9)
    assert dist((0.0, 0.0), lj["C"]) == pytest.approx(14.0, abs=1e-9)
    for name in ("base", "arm", "link", "handle"):
        assert_mesh(locked[name])
    for pin in locked["pins"]:
        assert_mesh(pin)


def test_toggle_clamp_excessive_overcenter_raises():
    with pytest.raises(ValueError):
        toggle_clamp(overcenter_deg=80.0)


def test_scotch_yoke_kinematics_and_clearance():
    angle = 35.0
    parts = scotch_yoke(angle_deg=angle)
    for name in ("crank_disc", "crank_pin", "yoke", "rail_a", "rail_b"):
        assert_mesh(parts[name])
    joints = parts["joints"]
    # pure SHM: yoke displacement is the pin's x coordinate
    assert joints["yoke_x"] == pytest.approx(12.0 * math.cos(
        math.radians(angle)), abs=1e-9)
    pin_centroid = parts["crank_pin"].centroid
    assert pin_centroid[0] == pytest.approx(joints["P"][0], abs=1e-6)
    assert pin_centroid[1] == pytest.approx(joints["P"][1], abs=1e-6)
    # pin runs free in the slot; yoke runs free between the rails
    assert overlap_volume(parts["crank_pin"], parts["yoke"]) < 1e-6
    assert overlap_volume(parts["yoke"], parts["rail_a"]) < 1e-6
    assert overlap_volume(parts["yoke"], parts["rail_b"]) < 1e-6
    assert overlap_volume(parts["crank_disc"], parts["rail_a"]) < 1e-6


def test_quick_return_ratio_formula_and_degeneracy():
    alpha = math.degrees(math.asin(14.0 / 34.0))
    expected = (180.0 + 2.0 * alpha) / (180.0 - 2.0 * alpha)
    assert quick_return_ratio(14.0, 34.0) == pytest.approx(expected)
    # Whitworth regime mirrors the slotted-lever regime
    assert quick_return_ratio(34.0, 14.0) == pytest.approx(expected)
    assert quick_return_ratio(14.0, 34.0) > 1.0
    with pytest.raises(ValueError):
        quick_return_ratio(20.0, 20.0)
    with pytest.raises(ValueError):
        quick_return_ratio(0.0, 34.0)


def test_quick_return_assembly_and_metadata():
    parts = quick_return(crank_angle_deg=40.0)
    for name in ("base", "crank_disc", "crank_pin", "lever"):
        assert_mesh(parts[name])
    assert parts["time_ratio"] == pytest.approx(quick_return_ratio(14.0, 34.0))
    joints = parts["joints"]
    # pin stays inside the slot span (12 .. lever_len - 8 off the pivot)
    assert 12.0 + 3.0 < dist(joints["G"], joints["P"]) < 52.0 - 3.0
    pin_centroid = parts["crank_pin"].centroid
    assert pin_centroid[0] == pytest.approx(joints["P"][0], abs=1e-6)
    assert pin_centroid[1] == pytest.approx(joints["P"][1], abs=1e-6)
    # pin runs free in the slot; lever clears the crank disc
    assert overlap_volume(parts["crank_pin"], parts["lever"]) < 1e-6
    assert overlap_volume(parts["crank_disc"], parts["lever"]) < 1e-6
    with pytest.raises(ValueError):
        quick_return(pivot_dist=60.0)  # pin travels past the slot end
