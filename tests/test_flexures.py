import math

import numpy as np
import pytest
import trimesh

from mechlib.flexures import (
    belleville_washer,
    coil_spring,
    flexure_stage,
    leaf_spring,
    spiral_power_spring,
)
from mechlib.meshutil import min_distance, overlap_volume


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def solid_runs(mesh, points, coords):
    """Return the coordinate intervals along a probe line that lie inside."""
    hit = mesh.contains(points)
    runs, start = [], None
    for i, inside in enumerate(hit):
        if inside and start is None:
            start = coords[i]
        elif not inside and start is not None:
            runs.append((start, coords[i - 1]))
            start = None
    if start is not None:
        runs.append((start, coords[-1]))
    return runs


# --- belleville washer ------------------------------------------------------

def test_belleville_cone_height_matches_free_height():
    washer = belleville_washer()
    assert_mesh(washer)
    span = washer.bounds[1][2] - washer.bounds[0][2]
    assert span == pytest.approx(1.6, abs=1e-6)
    assert washer.metadata["free_h"] == pytest.approx(span)
    assert washer.metadata["h0"] == pytest.approx(0.6)
    assert washer.metadata["h0_over_t"] == pytest.approx(0.6)
    # Bore and rim come out at the requested diameters.
    assert washer.bounds[1][0] == pytest.approx(10.0, abs=0.05)
    inner = washer.slice_plane([0, 0, 1.2], [0, 0, 1])
    assert inner is not None


def test_belleville_snap_through_flag_flips_at_root_two():
    thickness = 1.0
    below = belleville_washer(thickness=thickness,
                              free_h=thickness * (1.0 + 1.40))
    above = belleville_washer(thickness=thickness,
                              free_h=thickness * (1.0 + 1.43))
    assert below.metadata["h0_over_t"] == pytest.approx(1.40)
    assert above.metadata["h0_over_t"] == pytest.approx(1.43)
    assert below.metadata["snap_through"] is False
    assert above.metadata["snap_through"] is True
    # The flag tracks the mesh, not the argument: measured cone height agrees.
    measured = (above.bounds[1][2] - above.bounds[0][2]) - thickness
    assert measured / thickness == pytest.approx(1.43, abs=1e-6)


def test_belleville_stacks_scale_volume_and_height():
    single = belleville_washer()
    free_h, thickness, gap = 1.6, 1.0, 0.4
    cases = {
        "series": (3, 3 * free_h + 2 * gap),
        "parallel": (3, (free_h - thickness) + 3 * thickness + 2 * gap),
        "alternating": (4, 2 * (thickness + gap) + 2 * free_h + gap),
    }
    for arrangement, (count, height) in cases.items():
        stack = belleville_washer(stack=count, arrangement=arrangement)
        assert_mesh(stack)
        assert stack.volume == pytest.approx(count * single.volume, rel=1e-6)
        assert stack.metadata["stack_height"] == pytest.approx(height)
        span = stack.bounds[1][2] - stack.bounds[0][2]
        assert span == pytest.approx(height, abs=1e-6)
        assert len(stack.split(only_watertight=False)) == count


def test_belleville_rejects_impossible_geometry():
    with pytest.raises(ValueError):
        belleville_washer(thickness=0.5)
    with pytest.raises(ValueError):
        belleville_washer(thickness=1.0, free_h=1.0)
    with pytest.raises(ValueError):
        belleville_washer(inner_d=19.0)
    with pytest.raises(ValueError):
        belleville_washer(arrangement="helical")
    with pytest.raises(ValueError):
        belleville_washer(stack=3, arrangement="alternating")
    with pytest.raises(ValueError):
        belleville_washer(stack=3, stack_gap=0.2)


# --- coil spring ------------------------------------------------------------

def test_coil_spring_turn_count_from_a_vertical_probe():
    turns, pitch = 5.5, 4.0
    spring = coil_spring(turns=turns, pitch=pitch, ends="open")
    assert_mesh(spring)
    z0, z1 = spring.bounds[0][2], spring.bounds[1][2]
    z = np.linspace(z0 - 1.0, z1 + 1.0, 900)
    probe = 6.0 / math.sqrt(2.0)                 # probe the coil at 45 degrees
    pts = np.c_[np.full_like(z, probe), np.full_like(z, probe), z]
    runs = solid_runs(spring, pts, z)
    # The wire crosses the probe once per completed turn past 45 degrees.
    assert len(runs) == int(math.floor(turns - 0.125)) + 1
    for lo, hi in runs:
        assert hi - lo == pytest.approx(2.0, abs=0.25)


def test_coil_spring_free_length_and_dead_coils():
    open_spring = coil_spring(turns=6, pitch=4.0, ends="open")
    closed = coil_spring(turns=6, pitch=4.0, ends="closed")
    ground = coil_spring(turns=6, pitch=4.0, ends="ground")
    for spring in (open_spring, closed, ground):
        assert_mesh(spring)
        span = spring.bounds[1][2] - spring.bounds[0][2]
        assert span == pytest.approx(spring.metadata["free_length"], rel=0.02)
    assert open_spring.metadata["free_length"] == pytest.approx(6 * 4.0 + 2.0)
    assert closed.metadata["free_length"] == pytest.approx(
        2 * 2.0 + 4 * 4.0 + 2.0)
    assert ground.metadata["free_length"] == pytest.approx(
        closed.metadata["free_length"] - 2 * 0.25 * 2.0)
    assert ground.bounds[0][2] == pytest.approx(0.0, abs=1e-6)
    assert closed.metadata["active_turns"] == 4.0
    assert open_spring.metadata["active_turns"] == 6.0


def test_coil_spring_section_and_index_metadata():
    round_wire = coil_spring(turns=4, pitch=4.0, ends="open")
    rect_wire = coil_spring(turns=4, pitch=4.0, ends="open", section="rect")
    assert_mesh(rect_wire)
    assert rect_wire.volume > round_wire.volume
    assert rect_wire.volume / round_wire.volume == pytest.approx(
        4.0 / math.pi, rel=0.05)
    assert round_wire.metadata["spring_index"] == pytest.approx(6.0)
    assert round_wire.metadata["helix_angle_deg"] == pytest.approx(
        math.degrees(math.atan2(4.0, math.pi * 12.0)))
    assert round_wire.metadata["coil_gap"] == pytest.approx(2.0)


def test_coil_spring_sweep_is_wound_consistently():
    # The sweep emits its own winding instead of running trimesh's graph-based
    # fix_normals, so the orientation has to hold on its own.
    for kwargs in (dict(), dict(ends="open"), dict(section="rect"),
                   dict(turns=10.0, pitch=3.0, wire_d=1.2)):
        spring = coil_spring(**kwargs)
        assert spring.is_winding_consistent
        assert spring.volume > 0


def test_coil_spring_rejects_unprintable_pitch_and_index():
    with pytest.raises(ValueError):
        coil_spring(wire_d=2.0, pitch=2.2)          # coils fuse
    with pytest.raises(ValueError):
        coil_spring(coil_d=60.0, wire_d=2.0, pitch=4.0)   # helix too flat
    with pytest.raises(ValueError):
        coil_spring(coil_d=5.0, wire_d=2.0)         # spring index below 3
    with pytest.raises(ValueError):
        coil_spring(turns=2.0, ends="closed")       # no active coils left
    with pytest.raises(ValueError):
        coil_spring(section="oval")


# --- spiral power spring ----------------------------------------------------

def test_spiral_power_spring_parts_and_metadata():
    parts = spiral_power_spring()
    assert sorted(parts) == ["arbor", "barrel", "spring"]
    for mesh in parts.values():
        assert_mesh(mesh)
    meta = parts["spring"].metadata
    assert meta["fill_fraction"] < 0.5
    assert meta["wound_turns"] > meta["unwound_turns"]
    assert meta["stored_turns"] == pytest.approx(
        meta["wound_turns"] - meta["unwound_turns"])
    assert meta["stored_turns"] > 1.0
    # Sampled arc length agrees with the closed form pi*n*(2*r0 + n*pitch).
    r0 = 8.0 + 0.3 + 0.5
    closed_form = math.pi * 6.0 * (2 * r0 + 6.0 * 1.5)
    assert meta["arc_length"] == pytest.approx(closed_form, rel=0.02)


def test_spiral_turn_gap_measured_along_a_radial_probe():
    gap, strip_t, turns = 0.5, 1.0, 6.0
    parts = spiral_power_spring(gap=gap, strip_t=strip_t, turns=turns)
    spring = parts["spring"]
    r = np.linspace(8.0, 30.0, 3000)
    pts = np.c_[np.zeros_like(r), r, np.full_like(r, 3.0)]   # ray at 90 deg
    runs = solid_runs(spring, pts, r)
    assert len(runs) == int(turns)
    for lo, hi in runs:
        assert hi - lo == pytest.approx(strip_t, abs=0.15)
    clear = [runs[i + 1][0] - runs[i][1] for i in range(len(runs) - 1)]
    assert min(clear) >= gap - 0.05
    assert max(clear) <= gap + 0.15


def test_spiral_parts_do_not_interfere():
    parts = spiral_power_spring()
    names = ("barrel", "spring", "arbor")
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            assert overlap_volume(parts[names[i]], parts[names[j]]) < 1e-3
    assert min_distance(parts["spring"], parts["arbor"]) == pytest.approx(
        0.3, abs=0.12)
    assert min_distance(parts["spring"], parts["barrel"]) == pytest.approx(
        0.3, abs=0.12)


def test_spiral_rejects_overlong_and_oversize_strips():
    with pytest.raises(ValueError):
        spiral_power_spring(turns=25.0)             # outer turn misses the barrel
    with pytest.raises(ValueError):
        spiral_power_spring(turns=13.0)             # over the 50% fill limit
    with pytest.raises(ValueError):
        spiral_power_spring(strip_t=0.6)
    with pytest.raises(ValueError):
        spiral_power_spring(gap=0.2)
    with pytest.raises(ValueError):
        spiral_power_spring(wall=1.5)               # no room for the T slot


# --- leaf spring ------------------------------------------------------------

def test_leaf_spring_parts_camber_and_rate():
    parts = leaf_spring()
    assert sorted(parts) == ["clamp", "leaf_1", "leaf_2", "leaf_3"]
    for mesh in parts.values():
        assert_mesh(mesh)
    main = parts["leaf_1"]
    assert main.bounds[1][1] == pytest.approx(9.0 + 2.0 / 2.0, abs=0.02)
    assert main.bounds[1][0] - main.bounds[0][0] == pytest.approx(
        90.0 + 2 * (4.0 / 2.0 + 1.5), abs=0.05)
    meta = main.metadata
    assert meta["camber"] == pytest.approx(9.0)
    assert meta["deflection_to_flat"] == pytest.approx(9.0)
    rate = 3 * 2000.0 * 10.0 * 2.0 ** 3 / (6.0 * 90.0 ** 3)
    assert meta["rate_n_per_mm"] == pytest.approx(rate)
    assert meta["load_to_flat_n"] == pytest.approx(rate * 9.0)
    # Leaves get shorter down the pack.
    assert (parts["leaf_2"].bounds[1][0]
            > parts["leaf_3"].bounds[1][0])


def test_leaf_spring_leaves_stay_clear_of_each_other_and_the_clamp():
    parts = leaf_spring(leaf_gap=0.5)
    assert overlap_volume(parts["leaf_1"], parts["leaf_2"]) < 1e-3
    assert overlap_volume(parts["leaf_2"], parts["leaf_3"]) < 1e-3
    assert min_distance(parts["leaf_1"], parts["leaf_2"]) == pytest.approx(
        0.5, abs=0.06)
    for name in ("leaf_1", "leaf_2", "leaf_3"):
        assert overlap_volume(parts["clamp"], parts[name]) < 1e-3
    assert min_distance(parts["clamp"], parts["leaf_1"]) == pytest.approx(
        0.3, abs=0.12)


def test_leaf_spring_rejects_bad_geometry():
    with pytest.raises(ValueError):
        leaf_spring(camber=30.0)                    # camber past span/4
    with pytest.raises(ValueError):
        leaf_spring(leaf_t=0.5)
    with pytest.raises(ValueError):
        leaf_spring(leaf_gap=0.2)
    with pytest.raises(ValueError):
        leaf_spring(eye_d=10.0)                     # clamp cannot pass the eye
    with pytest.raises(ValueError):
        leaf_spring(taper=0.05)                     # shortest leaf inside clamp


# --- flexure stage ----------------------------------------------------------

def test_flexure_stage_is_one_connected_body():
    for compound in (True, False):
        stage = flexure_stage(compound=compound)
        assert_mesh(stage)
        assert len(stage.split(only_watertight=False)) == 1
        assert stage.metadata["blades"] == (4 if compound else 2)


def test_flexure_stage_blade_thickness_measured_from_the_mesh():
    blade_t, blade_len = 1.2, 30.0
    stage = flexure_stage(blade_t=blade_t, blade_len=blade_len)
    wall, stage_h, clearance, width = 4.0, 12.0, 1.0, 60.0
    col_h = wall + stage_h + clearance
    y_mid = col_h + blade_len / 2.0
    x = np.linspace(0.0, width / 2.0, 3000)
    pts = np.c_[x, np.full_like(x, y_mid), np.full_like(x, 2.0)]
    runs = solid_runs(stage, pts, x)
    assert len(runs) == 2                       # inner and outer blade
    for lo, hi in runs:
        assert hi - lo == pytest.approx(blade_t, abs=0.06)
    # Inner blades sit on the stage edge, outer blades on the ground columns.
    assert runs[0][0] == pytest.approx(16.0 / 2.0 - blade_t, abs=0.06)
    assert sum(runs[1]) / 2.0 == pytest.approx(width / 2.0 - wall / 2.0,
                                               abs=0.06)


def test_flexure_stage_leaves_room_for_its_travel():
    travel, clearance = 2.0, 1.0
    stage = flexure_stage(travel=travel, clearance=clearance)
    y_probe = 4.0 + 12.0 / 2.0          # halfway up the motion stage
    x = np.linspace(0.0, 30.0, 3000)
    pts = np.c_[x, np.full_like(x, y_probe), np.full_like(x, 2.0)]
    runs = solid_runs(stage, pts, x)
    assert len(runs) == 2               # motion stage, then the ground column
    free = runs[1][0] - runs[0][1]
    assert free >= travel + clearance - 0.05
    # The root fillet is a close operation on the profile, so prove it did not
    # weld the motion stage to the base rail underneath it.
    y = np.linspace(0.0, 46.0, 4000)
    column = np.c_[np.zeros_like(y), y, np.full_like(y, 2.0)]
    vertical = solid_runs(stage, column, y)
    assert len(vertical) == 3           # base rail, stage, secondary bar
    assert vertical[1][0] - vertical[0][1] == pytest.approx(clearance, abs=0.05)


def test_flexure_stage_strain_metadata_and_refusal():
    stage = flexure_stage(travel=2.0, blade_t=1.0, blade_len=25.0)
    assert stage.metadata["peak_strain"] == pytest.approx(
        3.0 * 1.0 * 1.0 / 25.0 ** 2)
    assert stage.metadata["parasitic_mm"] == 0.0
    simple = flexure_stage(travel=1.0, blade_t=1.0, blade_len=25.0,
                           compound=False)
    assert simple.metadata["peak_strain"] == pytest.approx(
        3.0 * 1.0 * 1.0 / 25.0 ** 2)
    assert simple.metadata["parasitic_mm"] == pytest.approx(
        3.0 * 1.0 ** 2 / (5.0 * 25.0))
    with pytest.raises(ValueError):
        flexure_stage(travel=8.0, blade_t=1.0, blade_len=25.0)
    with pytest.raises(ValueError):
        flexure_stage(blade_t=0.5)
    with pytest.raises(ValueError):
        flexure_stage(stage_w=48.0)                 # stage hits the columns
    with pytest.raises(ValueError):
        flexure_stage(clearance=0.2)
