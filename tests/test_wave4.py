"""Gap-analysis wave v0.9.0: classic mechanisms still missing from the catalog."""
import math

import numpy as np
import pytest
import trimesh

from mechlib import meshutil
from mechlib.cams import face_cam
from mechlib.clutches import dog_clutch
from mechlib.couplings import hirth_coupling
from mechlib.drives import harmonic_drive
from mechlib.fluid import external_gear_pump
from mechlib.joints import clevis
from mechlib.linear import rack_pinion, screw_jack, swash_plate
from mechlib.linkages import (
    bell_crank,
    chebyshev_linkage,
    chebyshev_pose,
    four_bar,
    scott_russell_linkage,
    scott_russell_pose,
    slider_crank,
    slider_crank_pose,
)


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def solids(parts):
    out = {}
    for key, value in parts.items():
        if isinstance(value, trimesh.Trimesh):
            out[key] = value
        elif isinstance(value, (list, tuple)) and value and isinstance(
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


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def test_four_bar_trace_does_not_hit_coupler():
    parts = four_bar(coupler_ext=25.0)
    assert "trace" in parts
    assert_mesh(parts["trace"])
    assert meshutil.overlap_volume(parts["trace"], parts["coupler"]) < 1e-6
    # Layer-aware pins clear every link body.
    for pin in parts["pins"]:
        for name in ("ground", "rocker", "coupler", "crank"):
            assert meshutil.overlap_volume(pin, parts[name]) < 1e-6


# ---------------------------------------------------------------- slider-crank


def test_slider_crank_stroke_and_conrod_length():
    crank_r, conrod = 14.0, 40.0
    xs = []
    for ang in range(0, 360, 5):
        j = slider_crank_pose(crank_r, conrod, float(ang))
        assert dist(j["O"], j["A"]) == pytest.approx(crank_r, abs=1e-9)
        assert dist(j["A"], j["B"]) == pytest.approx(conrod, abs=1e-9)
        assert j["B"][1] == pytest.approx(0.0, abs=1e-12)
        xs.append(j["slider_x"])
    assert max(xs) - min(xs) == pytest.approx(2.0 * crank_r, abs=1e-6)
    assert j["stroke"] == pytest.approx(2.0 * crank_r, abs=1e-9)


def test_slider_crank_assembly_watertight():
    parts = slider_crank()
    assert set(parts) >= {"base", "crank_disc", "conrod", "slider", "pins",
                          "joints", "stroke"}
    for mesh in solids(parts).values():
        assert_mesh(mesh)
    with pytest.raises(ValueError):
        slider_crank_pose(10.0, 5.0, 90.0)  # conrod too short at TDC offset


def test_slider_crank_assembly_clears_and_disc_rotates():
    # Rails used to run under the disc and the conrod shared the slider's Z
    # slab; both produced hundreds of mm^3 of solid overlap. A plain disc also
    # has no rotational cue, so the gallery bake dropped it and the crank
    # looked frozen. Guard both.
    for ang in (0.0, 45.0, 90.0, 135.0, 180.0, 270.0):
        parts = slider_crank(crank_angle_deg=ang)
        meshes = {k: parts[k] for k in ("base", "crank_disc", "conrod", "slider")}
        assert max_overlap(meshes) < 1e-3, ang
    # Disc is rebuilt by rigid rotation of a fixed local blank: vertex count
    # is phase-invariant, and a 90 deg rebuild matches spinning the 0 deg mesh.
    d0 = slider_crank(crank_angle_deg=0.0)["crank_disc"]
    d90 = slider_crank(crank_angle_deg=90.0)["crank_disc"]
    assert len(d0.vertices) == len(d90.vertices)
    spun = d0.copy()
    spun.apply_transform(trimesh.transformations.rotation_matrix(
        math.radians(90.0), (0.0, 0.0, 1.0)))
    assert float(np.max(np.abs(spun.vertices - d90.vertices))) < 1e-9


# ---------------------------------------------------------------- Chebyshev


def test_chebyshev_tracer_is_nearly_straight():
    unit = 10.0
    ys, xs = [], []
    # Working arc for the 5:2:4 proportions with branch +1 is roughly 37..101.
    for ang in np.linspace(40.0, 98.0, 51):
        j = chebyshev_pose(unit, float(ang))
        assert dist(j["O1"], j["A"]) == pytest.approx(5.0 * unit, abs=1e-9)
        assert dist(j["O2"], j["B"]) == pytest.approx(5.0 * unit, abs=1e-9)
        assert dist(j["A"], j["B"]) == pytest.approx(2.0 * unit, abs=1e-9)
        xs.append(j["T"][0])
        ys.append(j["T"][1])
    mean_y = float(np.mean(ys))
    max_err = float(np.max(np.abs(np.array(ys) - mean_y)))
    # Classic Chebyshev: lateral error stays under a tenth of the unit over
    # the working stroke.
    assert max_err < 0.1 * unit
    assert max(xs) - min(xs) > 2.0 * unit


def test_chebyshev_assembly():
    parts = chebyshev_linkage()
    assert parts["max_error"] < 0.7
    for mesh in solids(parts).values():
        assert_mesh(mesh)
    # Rockers share no Z layer and never interpenetrate; tracer seats in a
    # coupler bore rather than punching solid stock.
    assert max_overlap({k: parts[k] for k in
                        ("ground", "rocker_a", "rocker_b", "coupler")}) < 1e-6
    assert meshutil.overlap_volume(parts["tracer"], parts["coupler"]) < 1e-6


# ---------------------------------------------------------------- Scott-Russell


def test_scott_russell_tracer_is_exact_y_axis():
    half = 20.0
    for ang in range(0, 360, 10):
        j = scott_russell_pose(half, float(ang))
        assert dist(j["O"], j["M"]) == pytest.approx(half, abs=1e-9)
        assert dist(j["A"], j["M"]) == pytest.approx(half, abs=1e-9)
        assert dist(j["B"], j["M"]) == pytest.approx(half, abs=1e-9)
        assert j["A"][1] == pytest.approx(0.0, abs=1e-12)
        assert j["B"][0] == pytest.approx(0.0, abs=1e-12)
    assert j["stroke"] == pytest.approx(4.0 * half)


def test_scott_russell_assembly():
    # Stacked layers keep the assembly clear across the open upper-right
    # quadrant the gallery drives through. Near 90 deg the slider pin lands
    # on the ground pivot (kinematic singularity of this layout); stay clear.
    for ang in (30.0, 45.0, 55.0, 70.0):
        parts = scott_russell_linkage(crank_angle_deg=ang)
        for mesh in solids(parts).values():
            assert_mesh(mesh)
        assert max_overlap(solids(parts)) < 1e-3, ang
    assert parts["stroke"] == pytest.approx(80.0)


# ---------------------------------------------------------------- bell crank


def test_bell_crank_included_angle():
    parts = bell_crank(arm_a=28.0, arm_b=22.0, angle_deg=90.0, pose_deg=20.0)
    j = parts["joints"]
    va = np.array(j["A"]) - np.array(j["O"])
    vb = np.array(j["B"]) - np.array(j["O"])
    cosang = float(np.dot(va, vb) / (np.linalg.norm(va) * np.linalg.norm(vb)))
    assert math.degrees(math.acos(np.clip(cosang, -1, 1))) == pytest.approx(
        90.0, abs=1e-6)
    assert set(parts) >= {"base", "crank", "link_a", "link_b", "pins", "joints"}
    assert len(parts["pins"]) == 5
    for mesh in solids(parts).values():
        assert_mesh(mesh)
    # Crank and the two links live on different Z layers.
    assert meshutil.overlap_volume(parts["crank"], parts["link_a"]) < 1e-6
    assert meshutil.overlap_volume(parts["crank"], parts["link_b"]) < 1e-6


# ---------------------------------------------------------------- face cam


def test_face_cam_closed_lift():
    parts = face_cam(lift=6.0)
    assert_mesh(parts["cam"])
    assert_mesh(parts["pin"])
    # Closed cycloidal rise-return: peak at 180 deg, trough at 0 deg.
    hi = face_cam(lift=6.0, pin_phase_deg=180.0)["lift_at_phase"]
    lo = face_cam(lift=6.0, pin_phase_deg=0.0)["lift_at_phase"]
    assert hi > lo
    assert hi - lo == pytest.approx(6.0, abs=0.05)


# ---------------------------------------------------------------- swash plate


def test_swash_plate_stroke_formula():
    tilt = 15.0
    pitch_r = 14.0
    parts = swash_plate(tilt_deg=tilt, pitch_r=pitch_r, pistons=4)
    expected = 2.0 * pitch_r * math.tan(math.radians(tilt))
    assert parts["stroke"] == pytest.approx(expected, rel=1e-9)
    assert_mesh(parts["shaft"])
    assert_mesh(parts["plate"])
    assert len(parts["shoes"]) == 4
    for shoe in parts["shoes"]:
        assert_mesh(shoe)


# ---------------------------------------------------------------- screw jack / rack


def test_screw_jack_and_rack_pinion():
    jack = screw_jack(lift_frac=0.4)
    for mesh in solids(jack).values():
        assert_mesh(mesh)
    assert jack["screw"].metadata["travel_per_rev"] == pytest.approx(2.0)

    rp = rack_pinion(module=1.5, pinion_teeth=16, phase_teeth=0.5)
    assert_mesh(rp["pinion"])
    assert_mesh(rp["rack"])
    assert rp["travel_per_rev"] == pytest.approx(math.pi * 1.5 * 16)
    # Fixed-axis pinion: centre stays at x = 0, y = pitch_r; rack shifts -X.
    pitch_r = 1.5 * 16 / 2.0
    travel = 0.5 * math.pi * 1.5
    cx = float(rp["pinion"].centroid[0])
    cy = float(rp["pinion"].centroid[1])
    assert cx == pytest.approx(0.0, abs=0.5)
    assert cy == pytest.approx(pitch_r, abs=0.5)
    rp0 = rack_pinion(module=1.5, pinion_teeth=16, phase_teeth=0.0)
    # Rack centroid moves by -travel along X for +phase_teeth.
    assert (rp["rack"].centroid[0] - rp0["rack"].centroid[0]
            ) == pytest.approx(-travel, abs=0.5)


# ---------------------------------------------------------------- dog / hirth


def test_dog_clutch_and_hirth():
    dog = dog_clutch(dogs=4, engage_frac=1.0)
    assert_mesh(dog["hub_a"])
    assert_mesh(dog["hub_b"])
    # Fully engaged hubs should interpenetrate less than a full dog height of
    # solid overlap is allowed (teeth seat into gaps); withdrawn should clear.
    dog_out = dog_clutch(dogs=4, engage_frac=0.0)
    ov_out = meshutil.overlap_volume(dog_out["hub_a"], dog_out["hub_b"])
    assert ov_out < 1e-3

    hirth = hirth_coupling(teeth=12)
    assert_mesh(hirth["hub_a"])
    assert_mesh(hirth["hub_b"])


# ---------------------------------------------------------------- clevis


def test_clevis_parts():
    parts = clevis()
    for key in ("fork", "eye", "pin"):
        assert_mesh(parts[key])
    # Pin should pass through both: its Y extent covers the fork width.
    pin_y = parts["pin"].bounds[:, 1]
    fork_y = parts["fork"].bounds[:, 1]
    assert pin_y[0] <= fork_y[0] + 0.5
    assert pin_y[1] >= fork_y[1] - 0.5


# ---------------------------------------------------------------- gear pump


def test_external_gear_pump():
    parts = external_gear_pump(teeth=12, module=1.5)
    for key in ("body", "gear_a", "gear_b", "cap"):
        assert_mesh(parts[key])
    assert parts["displacement_per_rev"] > 0
    # Conjugate across a tooth pitch: equal teeth, counter-rotation, no clash.
    for phase in (0.0, 15.0, 30.0, 45.0, 90.0):
        posed = external_gear_pump(teeth=12, module=1.5, phase_deg=phase)
        ov = meshutil.overlap_volume(posed["gear_a"], posed["gear_b"])
        assert ov < 1.0, "phase %.1f overlap %.4g mm^3" % (phase, ov)
    # Phase is a rigid re-pose: vertex count must not change with phase.
    n0 = len(parts["gear_a"].vertices)
    n1 = len(external_gear_pump(teeth=12, phase_deg=37.0)["gear_a"].vertices)
    assert n0 == n1


# ---------------------------------------------------------------- harmonic


def test_harmonic_drive_ratio():
    parts = harmonic_drive(cs_teeth=50, fs_teeth=48, module=0.8)
    assert parts["ratio"] == pytest.approx(-24.0)
    for key in ("circular_spline", "flex_spline", "wave_generator"):
        assert_mesh(parts[key])
    with pytest.raises(ValueError):
        harmonic_drive(cs_teeth=50, fs_teeth=49)  # odd tooth difference
    with pytest.raises(ValueError):
        harmonic_drive(cs_teeth=48, fs_teeth=50)  # reversed
