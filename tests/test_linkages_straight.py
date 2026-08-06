"""Straight-line and scaling linkages: exactness, clearance, and validation."""
import math

import numpy as np
import pytest
import trimesh

from mechlib import meshutil
from mechlib.linkages import (
    lazy_tongs,
    lazy_tongs_pose,
    pantograph_linkage,
    pantograph_pose,
    peaucellier_linkage,
    peaucellier_pose,
    sarrus_linkage,
    sarrus_pose,
    watt_linkage,
    watt_pose,
)

CLEARANCE = 0.25


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def solids(parts):
    """Every Trimesh in a generator's return, flattened, named."""
    out = {}
    for key, value in parts.items():
        if isinstance(value, trimesh.Trimesh):
            out[key] = value
        elif isinstance(value, tuple) and value and isinstance(
                value[0], trimesh.Trimesh):
            for index, mesh in enumerate(value):
                out["%s[%d]" % (key, index)] = mesh
    return out


def max_overlap(meshes):
    names = sorted(meshes)
    worst = 0.0
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            worst = max(worst, meshutil.overlap_volume(meshes[a], meshes[b]))
    return worst


# --------------------------------------------------------------- Peaucellier


def test_peaucellier_tracer_line_is_exact():
    long_len, rhomb_len, crank_len = 30.0, 15.0, 10.0
    power = long_len ** 2 - rhomb_len ** 2
    xs, ys = [], []
    for step in range(-60, 61, 2):
        joints = peaucellier_pose(long_len, rhomb_len, crank_len, float(step))
        # the inversion identity holds link by link
        assert dist(joints["O"], joints["A"]) == pytest.approx(long_len,
                                                               abs=1e-9)
        assert dist(joints["O"], joints["B"]) == pytest.approx(long_len,
                                                               abs=1e-9)
        for a, b in (("P", "A"), ("A", "Q"), ("Q", "B"), ("B", "P")):
            assert dist(joints[a], joints[b]) == pytest.approx(rhomb_len,
                                                               abs=1e-9)
        r = math.hypot(*joints["P"])
        assert r * math.hypot(*joints["Q"]) == pytest.approx(power, rel=1e-12)
        xs.append(joints["Q"][0])
        ys.append(joints["Q"][1])
    xs, ys = np.array(xs), np.array(ys)
    # Fit a line to the traced path and measure how far the tracer leaves it.
    fit = np.polyfit(ys, xs, 1)
    deviation = float(np.abs(np.polyval(fit, ys) - xs).max())
    assert deviation < 1e-9 * 30.0
    # The line is the closed form, and it is vertical: no x drift at all.
    assert float(np.abs(xs - power / (2.0 * crank_len)).max()) < 1e-9 * 30.0
    assert ys.max() - ys.min() > 20.0


def test_peaucellier_assembly_parts_clear_each_other():
    parts = peaucellier_linkage()
    assert set(parts) == {"long_a", "long_b", "rhomb_pa", "rhomb_bq",
                          "rhomb_pb", "rhomb_aq", "ground", "crank", "pins",
                          "joints", "tracer_x", "power"}
    assert len(parts["pins"]) == 6
    assert parts["tracer_x"] == pytest.approx(33.75)
    assert parts["power"] == pytest.approx(675.0)
    for name, mesh in solids(parts).items():
        assert_mesh(mesh)
    for angle in (-50.0, 0.0, 50.0):
        posed = peaucellier_linkage(crank_angle_deg=angle)
        assert max_overlap(solids(posed)) < 1e-6
    # every pin sits on its solved joint
    joints = parts["joints"]
    for pin, key in zip(parts["pins"], ("O", "C", "P", "A", "B", "Q")):
        assert pin.centroid[0] == pytest.approx(joints[key][0], abs=1e-6)
        assert pin.centroid[1] == pytest.approx(joints[key][1], abs=1e-6)
    # bores clear the pins by half the designed diametral clearance
    assert meshutil.min_distance(parts["pins"][3], parts["long_a"]) == \
        pytest.approx(CLEARANCE / 2.0, abs=0.03)


def test_peaucellier_rejects_impossible_geometry():
    with pytest.raises(ValueError):  # rhombus longer than the anchor links
        peaucellier_pose(15.0, 30.0, 10.0, 0.0)
    with pytest.raises(ValueError):  # crank swung past the rhombus limit
        peaucellier_pose(30.0, 15.0, 10.0, 100.0)
    with pytest.raises(ValueError):
        peaucellier_linkage(crank_angle_deg=100.0)
    with pytest.raises(ValueError):  # bore swallows the bar
        peaucellier_linkage(width=3.0, bore_d=3.0)


# ----------------------------------------------------------------------- Watt


def test_watt_line_is_approximate_and_matches_metadata():
    parts = watt_linkage()
    dev = parts["straight_dev"]
    # An approximate straight line: small, but provably not zero.
    assert 0.0 < dev < 0.5
    assert parts["stroke"] > 20.0
    assert parts["coupler"].metadata["straight_dev"] == pytest.approx(dev)
    assert parts["coupler"].metadata["stroke"] == pytest.approx(parts["stroke"])
    # Recompute the number independently from the pose solver.
    worst = 0.0
    for step in range(-25, 26):
        trace = watt_pose(30.0, 30.0, 24.0, float(step))["T"]
        worst = max(worst, abs(trace[0]))
    assert worst == pytest.approx(dev, rel=1e-9)
    # ... and it really is worse further out: the error grows with the stroke.
    assert watt_linkage(stroke_deg=32.0)["straight_dev"] > dev
    # past the rocking limit the assembly simply cannot be built
    with pytest.raises(ValueError):
        watt_linkage(stroke_deg=40.0)


def test_watt_tracer_divides_the_coupler_inversely():
    joints = watt_pose(30.0, 30.0, 24.0, 18.0)
    assert dist(joints["O1"], joints["A"]) == pytest.approx(30.0, abs=1e-9)
    assert dist(joints["O2"], joints["B"]) == pytest.approx(30.0, abs=1e-9)
    assert dist(joints["A"], joints["B"]) == pytest.approx(24.0, abs=1e-9)
    # equal levers put the tracer on the coupler midpoint
    assert dist(joints["A"], joints["T"]) == pytest.approx(12.0, abs=1e-9)
    uneven = watt_pose(30.0, 20.0, 24.0, 10.0)
    at = dist(uneven["A"], uneven["T"])
    tb = dist(uneven["T"], uneven["B"])
    assert at / tb == pytest.approx(20.0 / 30.0, rel=1e-9)
    # the inverse rule is the one that flattens the path
    def sweep(frac):
        worst = 0.0
        for step in range(-20, 21):
            pose = watt_pose(30.0, 20.0, 24.0, float(step))
            a = np.array(pose["A"])
            b = np.array(pose["B"])
            worst = max(worst, abs(float((a + frac * (b - a))[0])))
        return worst
    assert sweep(20.0 / 50.0) < sweep(30.0 / 50.0)


def test_watt_assembly_parts_clear_each_other():
    parts = watt_linkage()
    assert set(parts) == {"ground", "lever_a", "lever_b", "coupler", "tracer",
                          "pins", "joints", "straight_dev", "stroke"}
    assert len(parts["pins"]) == 4
    for mesh in solids(parts).values():
        assert_mesh(mesh)
    for angle in (-20.0, 0.0, 20.0):
        posed = watt_linkage(lever_angle_deg=angle)
        assert max_overlap(solids(posed)) < 1e-6
    assert meshutil.min_distance(parts["pins"][2], parts["coupler"]) == \
        pytest.approx(CLEARANCE / 2.0, abs=0.03)
    # the tracer boss stands on the coupler at the guided point
    trace = parts["joints"]["T"]
    assert parts["tracer"].centroid[0] == pytest.approx(trace[0], abs=1e-6)
    assert parts["tracer"].centroid[1] == pytest.approx(trace[1], abs=1e-6)


def test_watt_rejects_impossible_geometry():
    with pytest.raises(ValueError):
        watt_pose(30.0, 30.0, 0.0, 0.0)
    with pytest.raises(ValueError):  # rocked past the assembly limit
        watt_pose(30.0, 30.0, 24.0, 120.0)
    with pytest.raises(ValueError):
        watt_linkage(stroke_deg=0.0)
    with pytest.raises(ValueError):
        watt_linkage(width=3.0, bore_d=3.0)


# --------------------------------------------------------------------- Sarrus


def test_sarrus_platform_is_pure_vertical_translation():
    reference = sarrus_linkage(fold_deg=30.0)
    base = np.asarray(reference["platform"].vertices, dtype=float)
    for fold in (20.0, 45.0, 60.0, 75.0):
        posed = sarrus_linkage(fold_deg=fold)
        moved = np.asarray(posed["platform"].vertices, dtype=float)
        assert len(moved) == len(base)
        delta = moved - base
        # X and Y travel is identically zero: that is the whole mechanism.
        assert float(np.abs(delta[:, 0]).max()) < 1e-9
        assert float(np.abs(delta[:, 1]).max()) < 1e-9
        # ... and Z is one rigid step, so the platform never rotates.
        rise = float(delta[:, 2].mean())
        assert float(np.abs(delta[:, 2] - rise).max()) < 1e-4
        expected = 2.0 * 20.0 * (math.sin(math.radians(fold)) -
                                 math.sin(math.radians(30.0)))
        assert rise == pytest.approx(expected, abs=1e-6)
        assert posed["lift"] == pytest.approx(
            2.0 * 20.0 * math.sin(math.radians(fold)), abs=1e-9)
    # closed form, straight from the pose solver
    for fold in (15.0, 50.0):
        pose = sarrus_pose(bar_len=17.0, fold_deg=fold)
        assert pose["lift"] == pytest.approx(
            2.0 * 17.0 * math.sin(math.radians(fold)), abs=1e-12)
        assert pose["platform_z"] == pytest.approx(pose["lift"] + 16.0)


def test_sarrus_assembly_parts_clear_each_other():
    parts = sarrus_linkage()
    assert set(parts) == {"base", "platform", "bars", "pins", "joints",
                          "lift", "platform_z"}
    assert len(parts["bars"]) == 4
    assert len(parts["pins"]) == 6
    for mesh in solids(parts).values():
        assert_mesh(mesh)
    # base plate and platform are one part each, ears fused on
    assert len(parts["base"].split(only_watertight=False)) == 1
    assert len(parts["platform"].split(only_watertight=False)) == 1
    for fold in (25.0, 40.0, 65.0):
        posed = sarrus_linkage(fold_deg=fold)
        assert max_overlap(solids(posed)) < 1e-6
    assert meshutil.min_distance(parts["pins"][0], parts["bars"][0]) == \
        pytest.approx(CLEARANCE / 2.0, abs=0.03)
    # the two chains work in orthogonal planes: chain A spans X, chain B Y
    span_a = parts["bars"][0].bounds[1] - parts["bars"][0].bounds[0]
    span_b = parts["bars"][2].bounds[1] - parts["bars"][2].bounds[0]
    assert span_a[0] > span_a[1]
    assert span_b[1] > span_b[0]
    assert span_a[0] == pytest.approx(span_b[1], abs=1e-6)


def test_sarrus_rejects_impossible_geometry():
    with pytest.raises(ValueError):  # folded flat is singular
        sarrus_pose(fold_deg=0.0)
    with pytest.raises(ValueError):
        sarrus_linkage(fold_deg=95.0)
    with pytest.raises(ValueError):  # plate too small for two chains
        sarrus_linkage(plate=15.0)
    with pytest.raises(ValueError):  # hinge axis buried in the plate
        sarrus_linkage(ear_h=2.0)


# ----------------------------------------------------------------- Pantograph


def test_pantograph_scales_exactly_at_every_position():
    for ratio in (1.5, 2.0, 3.0):
        for angle in range(0, 360, 30):
            p_x = 32.0 + 9.0 * math.cos(math.radians(angle))
            p_y = 9.0 * math.sin(math.radians(angle))
            pose = pantograph_pose(18.0, 24.0, ratio, p_x, p_y)
            fp = math.hypot(*pose["P"])
            fq = math.hypot(*pose["Q"])
            assert fq / fp == pytest.approx(ratio, rel=1e-12)
            # F, P and Q stay collinear
            cross = pose["P"][0] * pose["Q"][1] - pose["P"][1] * pose["Q"][0]
            assert abs(cross) < 1e-9
            # the parallelogram that enforces it keeps its side lengths
            assert dist(pose["F"], pose["A"]) == pytest.approx(18.0, abs=1e-9)
            assert dist(pose["A"], pose["P"]) == pytest.approx(24.0, abs=1e-9)
            assert dist(pose["C"], pose["D"]) == pytest.approx(24.0, abs=1e-9)
            assert dist(pose["P"], pose["D"]) == pytest.approx(
                (ratio - 1.0) * 18.0, abs=1e-9)


def test_pantograph_assembly_parts_clear_each_other():
    parts = pantograph_linkage()
    assert set(parts) == {"base", "bar1", "bar2", "bar3", "bar4", "tracer",
                          "pins", "joints", "ratio", "achieved_ratio"}
    assert len(parts["pins"]) == 5
    # measured from the posed geometry, not copied from the argument
    assert parts["achieved_ratio"] == pytest.approx(2.0, rel=1e-12)
    for mesh in solids(parts).values():
        assert_mesh(mesh)
    for angle in (0.0, 90.0, 210.0):
        posed = pantograph_linkage(p_x=32.0 + 9.0 * math.cos(
            math.radians(angle)), p_y=9.0 * math.sin(math.radians(angle)))
        assert max_overlap(solids(posed)) < 1e-6
    assert meshutil.min_distance(parts["pins"][1], parts["bar2"]) == \
        pytest.approx(CLEARANCE / 2.0, abs=0.03)
    tracer = parts["tracer"].centroid
    assert tracer[0] == pytest.approx(parts["joints"]["Q"][0], abs=1e-6)
    assert tracer[1] == pytest.approx(parts["joints"]["Q"][1], abs=1e-6)


def test_pantograph_rejects_impossible_geometry():
    with pytest.raises(ValueError):  # ratio 1 is a rigid parallelogram
        pantograph_pose(ratio=1.0)
    with pytest.raises(ValueError):  # stylus out of reach
        pantograph_pose(18.0, 24.0, 2.0, 90.0, 0.0)
    with pytest.raises(ValueError):
        pantograph_linkage(p_x=90.0)
    with pytest.raises(ValueError):
        pantograph_linkage(width=3.0, bore_d=3.0)


# ----------------------------------------------------------------- Lazy tongs


def test_lazy_tongs_multiplies_the_stroke_by_the_rhomb_count():
    for rhombs in (2, 3, 5):
        near = lazy_tongs_pose(rhombs, 30.0, 30.0)
        far = lazy_tongs_pose(rhombs, 30.0, 42.0)
        # the tip advances exactly `rhombs` times the first unit's advance
        measured = (far["span"] - near["span"]) / (far["pitch"] - near["pitch"])
        assert measured == pytest.approx(float(rhombs), rel=1e-12)
        assert near["stroke_mult"] == pytest.approx(float(rhombs))
        # and the reported gain is d(span)/d(height) at the pose
        step = 1e-6
        nudged = lazy_tongs_pose(rhombs, 30.0, 30.0 + step)
        gain = ((nudged["span"] - near["span"]) /
                (nudged["height"] - near["height"]))
        assert gain == pytest.approx(-near["gain"], rel=1e-5)
        assert near["gain"] == pytest.approx(
            rhombs * math.tan(math.radians(30.0)), rel=1e-12)
        assert near["span"] == pytest.approx(
            rhombs * 30.0 * math.cos(math.radians(30.0)), rel=1e-12)


def test_lazy_tongs_assembly_parts_clear_each_other():
    rhombs = 3
    parts = lazy_tongs(rhombs=rhombs)
    assert set(parts) == {"frame", "output", "bars", "pins", "joints", "span",
                          "height", "stroke_mult", "gain"}
    assert len(parts["bars"]) == 2 * rhombs
    assert len(parts["pins"]) == 3 * rhombs + 2
    for mesh in solids(parts).values():
        assert_mesh(mesh)
    assert parts["output"].metadata["stroke_mult"] == pytest.approx(3.0)
    for angle in (22.0, 35.0, 52.0):
        posed = lazy_tongs(rhombs=2, angle_deg=angle)
        assert max_overlap(solids(posed)) < 1e-6
    # the sliding pin runs free in the frame's guide slot
    assert meshutil.min_distance(parts["pins"][1], parts["frame"]) == \
        pytest.approx(CLEARANCE / 2.0, abs=0.03)
    # the output yoke translates along X only: its bore stays on the axis
    for angle in (25.0, 50.0):
        posed = lazy_tongs(rhombs=rhombs, angle_deg=angle)
        low = posed["output"].bounds[0]
        assert low[1] == pytest.approx(parts["output"].bounds[0][1], abs=1e-9)
        assert low[2] == pytest.approx(parts["output"].bounds[0][2], abs=1e-9)
        assert posed["span"] == pytest.approx(
            rhombs * 30.0 * math.cos(math.radians(angle)), rel=1e-12)


def test_lazy_tongs_rejects_impossible_geometry():
    with pytest.raises(ValueError):
        lazy_tongs_pose(0, 30.0, 35.0)
    with pytest.raises(ValueError):
        lazy_tongs_pose(3, 30.0, 0.0)
    with pytest.raises(ValueError):  # outside the guide-slot span
        lazy_tongs(angle_deg=60.0)
    with pytest.raises(ValueError):
        lazy_tongs(slot_lo_deg=60.0, slot_hi_deg=30.0)
    with pytest.raises(ValueError):  # bars would collide at the flat end
        lazy_tongs(bar_len=12.0, slot_lo_deg=20.0, angle_deg=35.0)
