import math

import numpy as np
import pytest
import trimesh

from mechlib.cutters import _BEARINGS
from mechlib.meshutil import bore_pierces, inside
from mechlib.pulleys import belt_tensioner, eccentric_idler_mount, idler_pulley


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def test_idler_pulley_crown_profile_is_actually_crowned():
    crown = 0.6
    idler = idler_pulley(od=16.0, width=8.0, crown=crown, flanges=False)
    assert_mesh(idler)
    verts = idler.vertices
    edge_r = np.linalg.norm(verts[np.isclose(verts[:, 2], 0.0, atol=1e-6)][:, :2],
                            axis=1)
    mid_r = np.linalg.norm(verts[np.isclose(verts[:, 2], 4.0, atol=1e-6)][:, :2],
                           axis=1)
    assert edge_r.size > 0 and mid_r.size > 0
    assert mid_r.max() == pytest.approx(8.0 + crown, abs=1e-6)
    assert edge_r.max() == pytest.approx(8.0, abs=1e-6)
    assert mid_r.max() - edge_r.max() == pytest.approx(crown, abs=1e-6)
    assert idler.metadata["belt_contact_d"] == pytest.approx(16.0 + 2 * crown)


def test_idler_pulley_flat_needs_flange_or_crown():
    with pytest.raises(ValueError):
        idler_pulley(crown=0.0, flanges=False)
    # Either alone is fine.
    assert_mesh(idler_pulley(crown=0.0, flanges=True))
    assert_mesh(idler_pulley(crown=0.3, flanges=False))


def _flange_span(mesh, r_test, z_lo, z_hi, step=0.05):
    zs = np.arange(z_lo, z_hi, step)
    solid = np.array([inside(mesh, (r_test, 0.0, z)) for z in zs])
    assert solid.any() and (~solid).any(), "no flange/gap transition found"
    first_gap = zs[np.argmax(~solid)]
    last_gap = zs[len(zs) - 1 - np.argmax(~solid[::-1])]
    return first_gap, last_gap


def test_idler_pulley_flanges_retain_belt_width_plus_clearance():
    width, belt_clearance, flange_extra = 8.0, 0.4, 1.5
    idler = idler_pulley(od=16.0, width=width, crown=0.0, flanges=True,
                         flange_t=1.2, flange_extra=flange_extra,
                         belt_clearance=belt_clearance)
    assert_mesh(idler)
    r_test = 8.0 + flange_extra - 0.3  # inside flange radius, outside crown radius
    total_h = idler.bounds[1][2]
    # Keep 0.1 mm off the mesh's own top/bottom faces: a probe point exactly
    # on the boundary is an ``inside()`` edge case, not a real gap/flange.
    first_gap, last_gap = _flange_span(idler, r_test, 0.1, total_h - 0.1)
    separation = last_gap - first_gap
    assert separation == pytest.approx(width + belt_clearance, abs=0.15)


def test_idler_pulley_toothed_reuses_timing_pulley_tooth_count():
    teeth = 24
    idler = idler_pulley(toothed=True, teeth=teeth, pitch=2.0, width=8.0,
                         bore_d=5.0, flanges=False)
    assert_mesh(idler)
    pitch_r = teeth * 2.0 / (2.0 * math.pi)
    probe_r = pitch_r - 0.45
    ring = trimesh.creation.annulus(probe_r - 0.02, probe_r + 0.02, 8.0,
                                    sections=128)
    ring.apply_translation((0, 0, 2.0))
    band = trimesh.boolean.intersection([idler, ring], engine="manifold")
    pieces = band.split(only_watertight=False)
    assert len(pieces) == teeth
    assert idler.metadata["belt_contact_d"] == pytest.approx(
        teeth * 2.0 / math.pi)


def test_idler_pulley_bore_is_open_by_default():
    idler = idler_pulley(bore_d=5.0, clearance=0.25)
    assert_mesh(idler)
    total_h = idler.bounds[1][2]
    assert bore_pierces(idler, (0, 0, -1.0), (0, 0, 1.0), total_h + 2.0)


def test_idler_pulley_bearing_seat_pocket_matches_bearing_od():
    for kind, fit in (("608", "press"), ("695", "slip"), ("MR105", "press")):
        idler = idler_pulley(bearing=kind, bearing_fit=fit, od=30.0, width=10.0)
        assert_mesh(idler)
        bore_d, outer_d, _width = _BEARINGS[kind]
        expected_pocket = outer_d + (0.25 if fit == "press" else 0.35)
        assert idler.metadata["bearing_pocket_d"] == pytest.approx(
            expected_pocket, abs=1e-3)


def test_idler_pulley_rejects_bad_args():
    with pytest.raises(ValueError):
        idler_pulley(od=-1.0)
    with pytest.raises(ValueError):
        idler_pulley(crown=100.0)  # crown too large for od
    with pytest.raises(ValueError):
        idler_pulley(bore_d=15.0, od=16.0, clearance=0.25)  # wall too thin
    with pytest.raises(ValueError):
        idler_pulley(bearing="not-a-bearing")


def test_eccentric_idler_mount_adjustment_range_matches_twice_eccentricity():
    eccentricity = 1.8
    a = eccentric_idler_mount(eccentricity=eccentricity, rotation_deg=0.0)
    b = eccentric_idler_mount(eccentricity=eccentricity, rotation_deg=180.0)
    for parts in (a, b):
        assert set(parts) == {"bushing", "pulley"}
        assert_mesh(parts["bushing"])
        assert_mesh(parts["pulley"])
    ca = a["pulley"].bounds.mean(axis=0)[:2]
    cb = b["pulley"].bounds.mean(axis=0)[:2]
    separation = float(np.linalg.norm(ca - cb))
    assert separation == pytest.approx(2.0 * eccentricity, abs=0.05)
    assert a["bushing"].metadata["adjustment_range"] == pytest.approx(
        2.0 * eccentricity)


def test_eccentric_idler_mount_bore_stays_open_around_the_offset_axis():
    eccentricity = 1.5
    parts = eccentric_idler_mount(eccentricity=eccentricity, post_d=5.0,
                                  height=10.0, clearance=0.25)
    bushing = parts["bushing"]
    off = bushing.metadata["axis_offset"]
    assert bore_pierces(bushing, (off[0], off[1], -1.0), (0, 0, 1.0), 12.0)


def test_eccentric_idler_mount_rejects_bad_args():
    with pytest.raises(ValueError):
        eccentric_idler_mount(eccentricity=10.0)  # blows through the wall
    with pytest.raises(ValueError):
        eccentric_idler_mount(drive_af=20.0, bushing_od=14.0)  # hex too big


def test_belt_tensioner_geometry_and_metadata():
    tensioner = belt_tensioner(arm_len=30.0, sweep_deg=50.0, beam_t=1.4,
                               preload_mm=2.5)
    assert_mesh(tensioner)
    assert len(tensioner.split(only_watertight=False)) == 1
    expected_strain = 3.0 * 1.4 * 2.5 / (2.0 * 30.0 ** 2)
    assert tensioner.metadata["peak_strain"] == pytest.approx(expected_strain)
    assert tensioner.metadata["preload_deflection_mm"] == pytest.approx(2.5)
    tip_x, tip_y = tensioner.metadata["tip_xy"]
    total_h = tensioner.bounds[1][2] - tensioner.bounds[0][2]
    assert bore_pierces(tensioner, (tip_x, tip_y, -1.0), (0, 0, 1.0),
                        total_h + 2.0)


def test_belt_tensioner_rejects_overstrained_preload():
    with pytest.raises(ValueError):
        belt_tensioner(arm_len=30.0, beam_t=1.4, preload_mm=20.0)
    # A longer arm relieves the same preload back under the limit.
    assert_mesh(belt_tensioner(arm_len=80.0, beam_t=1.4, preload_mm=20.0))


def test_belt_tensioner_rejects_bad_args():
    with pytest.raises(ValueError):
        belt_tensioner(sweep_deg=200.0)
    with pytest.raises(ValueError):
        belt_tensioner(arm_len=5.0, boss_d=11.0)  # boss doesn't fit the arm
    with pytest.raises(ValueError):
        belt_tensioner(boss_d=4.0, idler_bore_d=5.0)  # bore wall too thin
