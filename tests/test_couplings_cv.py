"""Constant-velocity joint tests: tripod geometry and the Cardan error."""

import itertools
import math

import numpy as np
import pytest
import trimesh

from mechlib import meshutil
from mechlib.couplings import (
    cv_velocity_fluctuation,
    cv_velocity_ratio,
    double_cardan_joint,
    tripod_cv_joint,
    tripod_pose,
)
from mechlib.prim import cyl

TRIPOD_KEYS = {"housing", "spider", "rollers", "shaft"}


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


# ---------------------------------------------------------------------------
# The analytic core: what a Hooke joint costs and a CV joint does not.
# ---------------------------------------------------------------------------


def test_hooke_ratio_matches_the_closed_form():
    for angle in (0.0, 5.0, 15.0, 25.0, 40.0):
        beta = math.radians(angle)
        for phase in (0.0, 30.0, 45.0, 90.0, 137.0, 270.0):
            theta = math.radians(phase)
            expected = math.cos(beta) / (
                1.0 - (math.sin(beta) * math.cos(theta)) ** 2)
            assert cv_velocity_ratio(angle, phase) == pytest.approx(expected)
    # Fast at 0 and 180, slow at 90 and 270 -- twice per input turn.
    assert cv_velocity_ratio(20.0, 0.0) == pytest.approx(
        1.0 / math.cos(math.radians(20.0)))
    assert cv_velocity_ratio(20.0, 180.0) == pytest.approx(
        1.0 / math.cos(math.radians(20.0)))
    assert cv_velocity_ratio(20.0, 90.0) == pytest.approx(
        math.cos(math.radians(20.0)))


def test_zero_angle_and_cv_joints_have_ratio_exactly_one():
    for phase in (0.0, 37.0, 90.0, 214.0):
        assert cv_velocity_ratio(0.0, phase) == 1.0
        for joint in ("tripod", "double_cardan"):
            for angle in (0.0, 15.0, 35.0):
                assert cv_velocity_ratio(angle, phase, joint) == 1.0
        # The intermediate shaft of a double Cardan is a plain Hooke joint.
        assert (cv_velocity_ratio(15.0, phase, "double_cardan_intermediate")
                == cv_velocity_ratio(15.0, phase, "hooke"))
    assert cv_velocity_fluctuation(0.0) == 0.0


def test_fluctuation_is_the_ratio_swing_and_grows_with_angle():
    previous = -1.0
    for angle in (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0):
        swing = cv_velocity_fluctuation(angle)
        sampled = [cv_velocity_ratio(angle, p)
                   for p in np.linspace(0.0, 360.0, 721)]
        assert swing == pytest.approx(max(sampled) - min(sampled), abs=1e-9)
        beta = math.radians(angle)
        assert swing == pytest.approx(math.sin(beta) ** 2 / math.cos(beta))
        assert swing > previous
        previous = swing
    # Published figure: a single Hooke joint at 15 degrees swings about
    # +/-3.5 percent about unity. Checked against the closed form, not quoted.
    assert cv_velocity_fluctuation(15.0) == pytest.approx(0.06935, abs=5e-5)
    assert cv_velocity_fluctuation(15.0) / 2.0 == pytest.approx(0.0347,
                                                                abs=5e-4)
    for joint in ("tripod", "double_cardan"):
        for angle in (0.0, 15.0, 35.0):
            assert cv_velocity_fluctuation(angle, joint) == 0.0


def test_velocity_helpers_reject_bad_arguments():
    with pytest.raises(ValueError):
        cv_velocity_ratio(15.0, 0.0, "rzeppa")
    with pytest.raises(ValueError):
        cv_velocity_ratio(95.0, 0.0)
    with pytest.raises(ValueError):
        cv_velocity_fluctuation(15.0, "rzeppa")
    with pytest.raises(ValueError):
        cv_velocity_fluctuation(-95.0)


# ---------------------------------------------------------------------------
# The tripod motion law
# ---------------------------------------------------------------------------


def test_tripod_pose_keeps_every_trunnion_in_its_track_plane():
    """The constant-velocity proof: three plane constraints, all satisfied."""
    pitch_r = 12.7
    worst = 0.0
    for angle in (0.0, 7.0, 15.0, 22.0, 26.0):
        beta = math.radians(angle)
        for phase in np.linspace(0.0, 360.0, 37):
            pose = tripod_pose(angle_deg=angle, phase_deg=float(phase),
                               pitch_r=pitch_r)
            # Output angle equals input angle exactly, at every angle.
            assert pose["housing_deg"] == pytest.approx(phase)
            assert pose["spider_deg"] == pytest.approx(phase)
            centre = np.asarray(pose["centre"])
            psi = math.radians(pose["housing_deg"])
            theta = math.radians(phase)
            for k in range(3):
                phi = theta + k * 2.0 * math.pi / 3.0
                track = psi + k * 2.0 * math.pi / 3.0
                trunnion = centre + pitch_r * np.array([
                    math.cos(phi),
                    math.sin(phi) * math.cos(beta),
                    math.sin(phi) * math.sin(beta)])
                normal = np.array([-math.sin(track), math.cos(track), 0.0])
                worst = max(worst, abs(float(trunnion @ normal)))
            assert pose["orbit_r"] == pytest.approx(
                pitch_r * (1.0 - math.cos(beta)) / 2.0)
    assert worst < 1e-9


def test_tripod_pose_rejects_bad_arguments():
    with pytest.raises(ValueError):
        tripod_pose(pitch_r=0.0)
    with pytest.raises(ValueError):
        tripod_pose(angle_deg=91.0)


# ---------------------------------------------------------------------------
# The printed tripod joint
# ---------------------------------------------------------------------------


def test_tripod_returns_four_named_bodies_and_three_barrels():
    parts = tripod_cv_joint(sections=32)
    assert set(parts) == TRIPOD_KEYS
    for mesh in parts.values():
        assert_mesh(mesh)
    # Three separate barrels, one spider, one tulip.
    assert len(parts["rollers"].split(only_watertight=False)) == 3
    assert len(parts["spider"].split(only_watertight=False)) == 1
    assert len(parts["housing"].split(only_watertight=False)) == 1
    assert parts["housing"].metadata["trunnions"] == 3


def test_tripod_housing_carries_exactly_three_tracks():
    """A ring through the track band is severed into three arcs."""
    parts = tripod_cv_joint(sections=32)
    pitch_r = parts["housing"].metadata["pitch_r"]
    ring = meshutil.sub(cyl(pitch_r + 1.0, 3.0, sections=96),
                        cyl(pitch_r - 1.0, 5.0, sections=96))
    band = meshutil.inter(ring, parts["housing"])
    assert len(band.split(only_watertight=False)) == 3


def test_tripod_runs_at_the_designed_clearance_at_every_angle():
    """A joint that only clears at zero angle is not a joint."""
    clear = 0.3
    reference = tripod_cv_joint(sections=32)
    angle_max = reference["housing"].metadata["angle_max_deg"]
    assert angle_max > 25.0
    for angle, phase in ((0.0, 0.0), (7.0, 41.0), (15.0, 90.0),
                         (21.0, 137.0), (25.0, 250.0), (angle_max, 310.0)):
        parts = tripod_cv_joint(angle_deg=angle, phase_deg=phase,
                                clear=clear, sections=32)
        # The crowned barrels ride the track walls at the running fit.
        gap = meshutil.min_distance(parts["rollers"], parts["housing"], n=4000)
        assert gap == pytest.approx(clear, abs=0.04), (angle, phase, gap)
        # The barrels turn on their trunnion posts at the same fit.
        assert meshutil.min_distance(
            parts["spider"], parts["rollers"], n=3000) == pytest.approx(
                clear, abs=0.04)
        # Nothing else may touch, and the spider body itself stays clear.
        assert meshutil.min_distance(
            parts["spider"], parts["housing"], n=3000) > clear
        assert meshutil.min_distance(
            parts["shaft"], parts["housing"], n=3000) > clear
        for a, b in itertools.combinations(
                ("housing", "spider", "rollers"), 2):
            assert meshutil.overlap_volume(parts[a], parts[b]) < 1e-6
        assert meshutil.overlap_volume(
            parts["shaft"], parts["housing"]) < 1e-6


def test_tripod_plunges_by_the_reported_travel_and_then_bottoms_out():
    angle, phase = 15.0, 30.0
    parts = tripod_cv_joint(angle_deg=angle, phase_deg=phase, sections=32)
    travel = parts["housing"].metadata["plunge_mm"]
    assert travel > 1.0
    at_stop = tripod_cv_joint(angle_deg=angle, phase_deg=phase,
                              plunge=-travel, sections=32)
    assert meshutil.overlap_volume(
        at_stop["rollers"], at_stop["housing"]) < 1e-6
    assert meshutil.overlap_volume(
        at_stop["spider"], at_stop["housing"]) < 1e-6
    past = tripod_cv_joint(angle_deg=angle, phase_deg=phase,
                           plunge=-(travel + 1.5), sections=32)
    assert meshutil.overlap_volume(past["rollers"], past["housing"]) > 1.0


def test_tripod_metadata_is_derived_and_consistent():
    angle = 15.0
    parts = tripod_cv_joint(angle_deg=angle, sections=32)
    meta = parts["spider"].metadata
    assert meta["velocity_ratio"] == 1.0
    assert meta["fluctuation"] == 0.0
    assert meta["hooke_fluctuation"] == pytest.approx(
        cv_velocity_fluctuation(angle))
    assert meta["hooke_fluctuation"] > 0.06
    assert meta["angle_max_deg"] > angle
    assert meta["orbit_r"] == pytest.approx(
        meta["pitch_r"] * (1.0 - math.cos(math.radians(angle))) / 2.0)
    assert meta["track_w"] == pytest.approx(meta["roller_d"] + 2.0 * 0.3)
    # Every body carries the same derived numbers.
    for mesh in parts.values():
        assert mesh.metadata["angle_max_deg"] == meta["angle_max_deg"]


def test_tripod_rejects_bad_arguments():
    parts = tripod_cv_joint(sections=32)
    angle_max = parts["housing"].metadata["angle_max_deg"]
    with pytest.raises(ValueError):
        tripod_cv_joint(angle_deg=angle_max + 0.5, sections=32)
    with pytest.raises(ValueError):
        tripod_cv_joint(angle_deg=60.0, sections=32)
    with pytest.raises(ValueError):
        tripod_cv_joint(trunnions=4, sections=32)
    with pytest.raises(ValueError):
        tripod_cv_joint(trunnions=6, sections=32)
    with pytest.raises(ValueError):
        tripod_cv_joint(angle_deg=-1.0, sections=32)
    with pytest.raises(ValueError):
        tripod_cv_joint(clear=0.0, sections=32)
    with pytest.raises(ValueError):
        tripod_cv_joint(swing_deg=0.0, sections=32)
    with pytest.raises(ValueError):
        tripod_cv_joint(sections=8)
    with pytest.raises(ValueError):
        tripod_cv_joint(housing_d=18.0, sections=32)
    with pytest.raises(ValueError):
        tripod_cv_joint(flare_h=14.0, sections=32)
    # A hub thick enough to land on the tulip floor before the barrels do
    # would make plunge_mm a lie, so it is refused.
    with pytest.raises(ValueError):
        tripod_cv_joint(hub_t=11.0, swing_deg=5.0, sections=32)


# ---------------------------------------------------------------------------
# The other classical fix
# ---------------------------------------------------------------------------


def test_double_cardan_cancels_at_the_output_but_not_in_the_middle():
    bend = 15.0
    parts = double_cardan_joint(bend_deg=bend, sections=32)
    assert set(parts) == {"yoke_in", "spider_in", "intermediate",
                          "spider_out", "yoke_out"}
    for mesh in parts.values():
        assert_mesh(mesh)
    meta = parts["intermediate"].metadata
    assert meta["output_fluctuation"] == 0.0
    assert meta["intermediate_fluctuation"] == pytest.approx(
        cv_velocity_fluctuation(bend))
    assert meta["intermediate_fluctuation"] > 0.06
    assert meta["total_angle_deg"] == pytest.approx(2.0 * bend)
    # The two intermediate yokes fuse into ONE rigid body, which is what makes
    # the second joint's error the inverse of the first's rather than free.
    assert len(parts["intermediate"].split(only_watertight=False)) == 1
    for a, b in itertools.combinations(sorted(parts), 2):
        assert meshutil.overlap_volume(parts[a], parts[b]) < 1e-6


def test_double_cardan_phases_its_intermediate_yokes_ninety_degrees_apart():
    """Measured off the mesh: tine metal on one pin axis, air on the other."""
    bend, length, fork_gap, tine_t = 15.0, 46.0, 18.0, 4.0
    parts = double_cardan_joint(bend_deg=bend, inter_len=length,
                                fork_gap=fork_gap, tine_t=tine_t, sections=32)
    body = parts["intermediate"]
    beta = math.radians(bend)
    axis = np.array([0.0, -math.sin(beta), math.cos(beta)])
    pins_first = np.array([0.0, math.cos(beta), math.sin(beta)])
    pins_second = np.array([1.0, 0.0, 0.0])
    assert float(pins_first @ pins_second) == pytest.approx(0.0, abs=1e-12)
    reach = fork_gap / 2.0 + tine_t / 2.0
    for centre, pins, other in ((np.zeros(3), pins_first, pins_second),
                                (length * axis, pins_second, pins_first)):
        for side in (1.0, -1.0):
            assert meshutil.inside(
                body, [centre + side * reach * pins + 3.0 * other])
            assert meshutil.clear(
                body, [centre + side * reach * other + 3.0 * pins])


def test_double_cardan_rejects_bad_arguments():
    with pytest.raises(ValueError):
        double_cardan_joint(inter_len=20.0, sections=32)
    with pytest.raises(ValueError):
        double_cardan_joint(inter_len=120.0, sections=32)
    with pytest.raises(ValueError):
        double_cardan_joint(bend_deg=60.0, sections=32)
