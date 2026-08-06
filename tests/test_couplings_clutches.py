import math

import pytest
import trimesh

from mechlib import clutches
from mechlib.clutches import freewheel_clutch, torque_limiter
from mechlib.couplings import jaw_coupling, oldham_coupling, universal_joint


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def overlap_volume(a, b):
    overlap = trimesh.boolean.intersection([a, b], engine="manifold")
    if overlap is None or overlap.is_empty:
        return 0.0
    return abs(overlap.volume)


def test_oldham_parts_watertight_and_clear_when_assembled():
    parts = oldham_coupling()
    for mesh in parts.values():
        assert_mesh(mesh)
    assert overlap_volume(parts["hub_a"], parts["disc"]) < 1e-6
    assert overlap_volume(parts["hub_b"], parts["disc"]) < 1e-6
    assert overlap_volume(parts["hub_a"], parts["hub_b"]) < 1e-6


def test_oldham_tongue_slot_clearance_tracks_parameter():
    tongue_w = 8.0
    loose = oldham_coupling(tongue_w=tongue_w, clearance=0.4)
    tight = oldham_coupling(tongue_w=tongue_w, clearance=0.15)
    assert loose["disc"].metadata["slot_w"] == pytest.approx(tongue_w + 0.8)
    assert tight["disc"].metadata["slot_w"] == pytest.approx(tongue_w + 0.3)
    assert loose["disc"].metadata["slot_depth"] == pytest.approx(
        tight["disc"].metadata["slot_depth"] + 0.25)
    assert loose["hub_a"].metadata["tongue_w"] == pytest.approx(tongue_w)
    for parts in (loose, tight):
        assert overlap_volume(parts["hub_a"], parts["disc"]) < 1e-6
        assert overlap_volume(parts["hub_b"], parts["disc"]) < 1e-6


def test_oldham_invalid_dimensions_raise():
    with pytest.raises(ValueError):
        oldham_coupling(d=10.0, bore_d=9.0)  # no hub wall left
    with pytest.raises(ValueError):
        oldham_coupling(web=0.5)  # disc web below minimum wall


def test_universal_joint_watertight_and_clear_across_bend_range():
    for bend_deg in (0.0, 20.0, 35.0):
        parts = universal_joint(bend_deg=bend_deg)
        for mesh in parts.values():
            assert_mesh(mesh)
        assert overlap_volume(parts["yoke_a"], parts["spider"]) < 1e-6
        assert overlap_volume(parts["yoke_b"], parts["spider"]) < 1e-6
        assert overlap_volume(parts["yoke_a"], parts["yoke_b"]) < 1e-6


def test_universal_joint_bend_rotates_output_shaft():
    straight = universal_joint(bend_deg=0.0)
    bent = universal_joint(bend_deg=30.0)
    assert straight["yoke_b"].bounds[1, 2] == pytest.approx(
        bent["yoke_b"].bounds[1, 2], rel=0.2)
    # A bent yoke reaches sideways beyond the straight one's footprint.
    assert abs(bent["yoke_b"].bounds[0, 1]) > abs(
        straight["yoke_b"].bounds[0, 1]) + 1.0


def test_universal_joint_invalid_dimensions_raise():
    with pytest.raises(ValueError):
        universal_joint(bend_deg=60.0)
    with pytest.raises(ValueError):
        universal_joint(tine_t=0.8)


def test_jaw_coupling_watertight_and_clear_when_assembled():
    for jaws in (3, 4):
        parts = jaw_coupling(jaws=jaws)
        for mesh in parts.values():
            assert_mesh(mesh)
        assert overlap_volume(parts["hub_a"], parts["spider"]) < 1e-6
        assert overlap_volume(parts["hub_b"], parts["spider"]) < 1e-6
        assert overlap_volume(parts["hub_a"], parts["hub_b"]) < 1e-6


def test_jaw_coupling_lobe_angle_and_validation():
    parts = jaw_coupling(jaws=3, jaw_deg=30.0, clearance=0.25)
    r_mid = 0.5 * (5.5 + 13.0)
    expect = 60.0 - 30.0 - 2.0 * math.degrees(0.25 / r_mid)
    assert parts["spider"].metadata["lobe_deg"] == pytest.approx(expect)
    with pytest.raises(ValueError):
        jaw_coupling(jaw_deg=58.0)  # no spider lobe left
    with pytest.raises(ValueError):
        jaw_coupling(jaw_r0=4.0)  # jaw root swallows the bore wall


def test_torque_limiter_watertight_and_clear_when_assembled():
    parts = torque_limiter()
    for mesh in parts.values():
        assert_mesh(mesh)
    assert overlap_volume(parts["driver"], parts["driven"]) < 1e-6
    cavity = parts["driven"].metadata
    assert cavity["cavity_z1"] - cavity["cavity_z0"] == pytest.approx(4.5)


def test_torque_limiter_detent_count_changes_geometry():
    six = torque_limiter(detents=6)
    eight = torque_limiter(detents=8)
    assert six["driver"].volume != pytest.approx(eight["driver"].volume)
    for parts in (six, eight):
        assert overlap_volume(parts["driver"], parts["driven"]) < 1e-6
    with pytest.raises(ValueError):
        torque_limiter(detents=12)  # detents crowd the pitch circle
    with pytest.raises(ValueError):
        torque_limiter(face_gap=0.4, clearance=0.25)  # bumps escape pockets


def test_freewheel_clutch_watertight_and_clear_when_assembled():
    parts = freewheel_clutch()
    assert_mesh(parts["ring"])
    assert_mesh(parts["hub"])
    assert len(parts["rollers"]) == 6
    for roller in parts["rollers"]:
        assert_mesh(roller)
        assert overlap_volume(parts["ring"], roller) < 1e-6
        assert overlap_volume(parts["hub"], roller) < 1e-6
    assert overlap_volume(parts["ring"], parts["hub"]) < 1e-6


def test_freewheel_clutch_roller_count_and_validation():
    four = freewheel_clutch(rollers=4)
    assert len(four["rollers"]) == 4
    for roller in four["rollers"]:
        assert overlap_volume(four["ring"], roller) < 1e-6
    with pytest.raises(ValueError):
        freewheel_clutch(rollers=8)  # pockets overlap at this count
    with pytest.raises(ValueError):
        freewheel_clutch(pocket_deg=20.0)  # roller cannot fit the pocket


# ---------------------------------------------------------------------------
# GEOS-degeneracy guard
#
# The freewheel ring unions ramp pockets into a circular void. The pocket
# return path used to run at exactly the void radius, so its vertices landed
# ~1e-3 mm outside the polygonal void and the two boundaries crossed back and
# forth along the same arc: the input that makes GEOS' floating-point overlay
# emit "found non-noded intersection". In the browser (Pyodide's older GEOS)
# that is a C++ abort, not a catchable Python exception. The union now runs on
# a fixed precision grid with the return path strictly inside the void.
# ---------------------------------------------------------------------------


def assert_no_degenerate_edges(geometry, label, min_edge=1e-9):
    """No consecutive duplicate vertices and no zero-length edges."""
    rings = []
    for part in getattr(geometry, "geoms", [geometry]):
        rings.append(part.exterior)
        rings.extend(part.interiors)
    for ring in rings:
        coords = list(ring.coords)
        assert len(coords) >= 4, "%s: degenerate ring" % label
        for i in range(len(coords) - 1):
            (x0, y0), (x1, y1) = coords[i], coords[i + 1]
            assert (x0, y0) != (x1, y1), (
                "%s: duplicate vertex at index %d" % (label, i))
            assert math.hypot(x1 - x0, y1 - y0) > min_edge, (
                "%s: zero-length edge at index %d" % (label, i))


@pytest.mark.parametrize("rollers", [4, 5, 6, 7])
@pytest.mark.parametrize("sections", [24, 32, 48, 64, 96])
def test_freewheel_sweep_is_sound_and_non_degenerate(
        rollers, sections, monkeypatch):
    captured = []
    original = clutches._extrude

    def spy(poly, height, z0=0.0):
        captured.append(poly)
        return original(poly, height, z0)

    monkeypatch.setattr(clutches, "_extrude", spy)
    parts = freewheel_clutch(rollers=rollers, sections=sections)
    monkeypatch.undo()

    assert captured, "ring polygon was never extruded"
    for poly in captured:
        assert poly.is_valid and not poly.is_empty and poly.area > 0
        assert_no_degenerate_edges(poly, "freewheel ring polygon")
    assert_mesh(parts["ring"])
    assert_mesh(parts["hub"])
    assert len(parts["rollers"]) == rollers
    for roller in parts["rollers"]:
        assert_mesh(roller)
