import math

import numpy as np
import pytest
import trimesh

from mechlib import meshutil
from mechlib.bearings import plain_bushing, printed_ball_bearing, thrust_washer


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def probe_ring(r_in, r_out, height, z_center, sections=128):
    """Return an annular probe solid centred on the Z axis."""
    ring = trimesh.creation.annulus(r_in, r_out, height, sections=sections)
    ring.apply_translation((0.0, 0.0, z_center))
    return ring


def downward_overhangs(mesh, ignore_z=(), limit=0.7071, tol=0.02):
    """Return downward-facing faces steeper than the 45 degree FDM limit."""
    normals = mesh.face_normals
    centers = mesh.triangles_center
    bad = []
    for i in range(len(normals)):
        if normals[i][2] >= -limit - tol:
            continue
        if any(abs(centers[i][2] - z) < 1e-4 for z in ignore_z):
            continue
        bad.append((float(normals[i][2]), centers[i].tolist()))
    return bad


# --------------------------------------------------------------- plain bushing


def test_plain_bushing_metadata_and_envelope():
    bushing = plain_bushing()
    assert_mesh(bushing)
    assert len(bushing.split(only_watertight=False)) == 1
    meta = bushing.metadata
    assert meta["nominal_bore_d"] == 8.0
    assert meta["bore_d"] == pytest.approx(8.25)
    assert meta["outer_d"] == pytest.approx(8.0 + 0.25 + 2 * 2.0)
    assert meta["bearing_area"] == pytest.approx(8.0 * 12.0)
    assert bushing.bounds[1][2] - bushing.bounds[0][2] == pytest.approx(12.0)
    # The flange sets the outside envelope, the sleeve the bore.
    assert bushing.bounds[1][0] == pytest.approx(meta["flange_d"] / 2.0,
                                                 abs=0.01)


def test_plain_bushing_bore_measures_bore_d_plus_clear():
    clear = 0.25
    bushing = plain_bushing(bore_d=8.0, clear=clear, relief_grooves=0)
    bore_r = (8.0 + clear) / 2.0
    inside = trimesh.creation.cylinder(radius=bore_r - 0.06, height=10.0,
                                       sections=128)
    inside.apply_translation((0.0, 0.0, 6.0))
    assert meshutil.overlap_volume(bushing, inside) == pytest.approx(0.0,
                                                                     abs=1e-6)
    outside = trimesh.creation.cylinder(radius=bore_r + 0.06, height=10.0,
                                        sections=128)
    outside.apply_translation((0.0, 0.0, 6.0))
    assert meshutil.overlap_volume(bushing, outside) > 1.0
    assert meshutil.bore_pierces(bushing, (0, 0, -1.0), (0, 0, 1.0), 14.0,
                                 n=20)


def test_plain_bushing_axial_grooves_leave_that_many_lands():
    grooves = 5
    bushing = plain_bushing(relief_grooves=grooves, groove_style="axial",
                            groove_w=1.2, groove_depth=0.6)
    assert_mesh(bushing)
    bore_r = (8.0 + 0.25) / 2.0
    # A thin shell just outside the bore is interrupted by every groove.
    shell = probe_ring(bore_r + 0.05, bore_r + 0.15, 4.0, 6.0)
    band = meshutil.inter(bushing, shell)
    pieces = [p for p in band.split(only_watertight=False) if p.volume > 1e-4]
    assert len(pieces) == grooves
    assert bushing.volume < plain_bushing(relief_grooves=0).volume


def test_plain_bushing_circumferential_grooves_split_the_bore_axially():
    grooves = 3
    bushing = plain_bushing(relief_grooves=grooves,
                            groove_style="circumferential")
    assert_mesh(bushing)
    bore_r = (8.0 + 0.25) / 2.0
    shell = probe_ring(bore_r + 0.05, bore_r + 0.15, 20.0, 6.0)
    band = meshutil.inter(bushing, shell)
    pieces = [p for p in band.split(only_watertight=False) if p.volume > 1e-4]
    assert len(pieces) == grooves + 1
    # V flanks at 45 degrees keep the bore self-supporting when printed
    # upright: no downward face steeper than the limit above the first layer.
    assert downward_overhangs(bushing, ignore_z=(0.0,)) == []


def test_plain_bushing_rejects_impossible_geometry():
    with pytest.raises(ValueError):
        plain_bushing(wall=0.5)
    with pytest.raises(ValueError):
        plain_bushing(groove_depth=1.6)          # eats the 0.8 mm wall
    with pytest.raises(ValueError):
        plain_bushing(bore_d=4.0, relief_grooves=12)   # grooves collide
    with pytest.raises(ValueError):
        plain_bushing(groove_style="helical")
    with pytest.raises(ValueError):
        plain_bushing(length=4.0, flange_t=5.0)
    with pytest.raises(ValueError):
        plain_bushing(length=1.0, lead_in=0.6)


# --------------------------------------------------------------- thrust washer


def test_thrust_washer_faces_reduce_contact_area():
    flat = thrust_washer(face="flat")
    pockets = thrust_washer(face="pockets", pockets=6)
    pads = thrust_washer(face="pads", pockets=6)
    for mesh in (flat, pockets, pads):
        assert_mesh(mesh)
        assert len(mesh.split(only_watertight=False)) == 1
        assert mesh.bounds[1][2] - mesh.bounds[0][2] == pytest.approx(2.4)
    assert flat.metadata["contact_ratio"] == 1.0
    assert pockets.volume < flat.volume
    assert pads.volume < flat.volume
    assert pads.metadata["contact_ratio"] == pytest.approx(
        (60.0 - 14.0) * 6 / 360.0)


def test_thrust_washer_pocket_count_is_carried_in_the_face():
    count = 7
    washer = thrust_washer(face="pockets", pockets=count, pocket_d=3.0)
    assert_mesh(washer)
    blank = thrust_washer(face="flat")
    voids = [p for p in meshutil.sub(blank, washer).split(only_watertight=False)
             if p.volume > 0.1]
    assert len(voids) == count
    for void in voids:
        assert void.bounds[1][2] == pytest.approx(2.4, abs=1e-6)


def test_thrust_washer_pair_is_a_ball_race_at_the_designed_clearance():
    clear = 0.3
    parts = thrust_washer(pair=True, balls=6, ball_d=3.0, clear=clear)
    assert sorted(parts) == ["balls", "cage", "housing_washer",
                             "rotor_washer"]
    for mesh in parts.values():
        assert_mesh(mesh)
    assert len(parts["balls"].split(only_watertight=False)) == 6
    for name in ("housing_washer", "rotor_washer", "cage"):
        assert meshutil.min_distance(parts["balls"], parts[name]) == \
            pytest.approx(clear, abs=0.05)
        assert meshutil.overlap_volume(parts["balls"], parts[name]) == \
            pytest.approx(0.0, abs=1e-6)
    assert meshutil.overlap_volume(parts["housing_washer"],
                                   parts["rotor_washer"]) == \
        pytest.approx(0.0, abs=1e-6)
    meta = parts["cage"].metadata
    assert meta["face_gap"] == pytest.approx(2.0 * (3.0 / 2.0 + clear
                                                    - meta["groove_depth"]))
    assert parts["rotor_washer"].bounds[1][2] == pytest.approx(
        meta["stack_height"])
    # The balls stand proud of both raceways, so the washers never touch.
    assert parts["balls"].bounds[0][2] < parts["housing_washer"].bounds[1][2]
    assert parts["balls"].bounds[1][2] > parts["rotor_washer"].bounds[0][2]


def test_thrust_washer_rejects_impossible_geometry():
    with pytest.raises(ValueError):
        thrust_washer(face="grooves")
    with pytest.raises(ValueError):
        thrust_washer(outer_d=9.0)               # annulus thinner than 1.6 mm
    with pytest.raises(ValueError):
        thrust_washer(face="pockets", pockets=24)
    with pytest.raises(ValueError):
        thrust_washer(pair=True, balls=24)       # cage webs vanish
    with pytest.raises(ValueError):
        thrust_washer(pair=True, ball_d=9.0)     # raceway eats the rims
    with pytest.raises(ValueError):
        thrust_washer(pair=True, thickness=1.4, ball_d=6.0)


# ----------------------------------------------------------- printed bearing


def test_printed_ball_bearing_splits_into_races_cage_and_balls():
    balls = 6
    parts = printed_ball_bearing(balls=balls)
    assert sorted(parts) == ["balls", "cage", "inner_race", "outer_race"]
    for mesh in parts.values():
        assert_mesh(mesh)
    assert len(parts["balls"].split(only_watertight=False)) == balls
    whole = trimesh.util.concatenate(list(parts.values()))
    assert len(whole.split(only_watertight=False)) == balls + 3
    assert whole.bounds[0][2] == pytest.approx(0.0)
    assert whole.bounds[1][2] == pytest.approx(10.0)
    assert parts["outer_race"].bounds[1][0] == pytest.approx(16.0, abs=0.02)


def test_printed_ball_bearing_gaps_equal_the_designed_clearance():
    clear = 0.3
    parts = printed_ball_bearing(clear=clear)
    pairs = [("balls", "inner_race"), ("balls", "outer_race"),
             ("balls", "cage"), ("inner_race", "outer_race"),
             ("cage", "inner_race"), ("cage", "outer_race")]
    for a, b in pairs:
        assert meshutil.min_distance(parts[a], parts[b]) == pytest.approx(
            clear, abs=0.05), "%s vs %s" % (a, b)
        assert meshutil.overlap_volume(parts[a], parts[b]) == pytest.approx(
            0.0, abs=1e-6), "%s vs %s" % (a, b)


def test_printed_ball_bearing_balls_cannot_escape():
    parts = printed_ball_bearing()
    races = meshutil.uni([parts["inner_race"], parts["outer_race"]])
    meta = parts["balls"].metadata
    # The truncated groove still leaves a shoulder gap narrower than the ball.
    assert meta["shoulder_gap"] < meta["ball_d"]
    for shift in ((0, 0, 1.0), (0, 0, -1.0), (1.0, 0, 0), (-1.0, 0, 0)):
        moved = parts["balls"].copy()
        moved.apply_translation(shift)
        assert meshutil.overlap_volume(moved, races) > 1.0, shift
    # And the balls lock the two races together axially.
    for dz in (1.0, -1.0):
        moved = parts["outer_race"].copy()
        moved.apply_translation((0.0, 0.0, dz))
        assert meshutil.overlap_volume(moved, parts["balls"]) > 1.0


def test_printed_ball_bearing_prints_without_support():
    parts = printed_ball_bearing()
    ball_bottom = parts["balls"].bounds[0][2]
    cage_bottom = parts["cage"].bounds[0][2]
    # Only the deliberate print-in-place first layers face down flat.
    assert downward_overhangs(parts["inner_race"], ignore_z=(0.0,)) == []
    assert downward_overhangs(parts["outer_race"], ignore_z=(0.0,)) == []
    assert downward_overhangs(parts["cage"], ignore_z=(cage_bottom,)) == []
    assert downward_overhangs(parts["balls"], ignore_z=(ball_bottom,)) == []
    # The ball flat is the 45 degree latitude cut, not a hemisphere.
    ball = parts["balls"].split(only_watertight=False)[0]
    a = parts["balls"].metadata["ball_d"] / 2.0
    height = ball.bounds[1][2] - ball.bounds[0][2]
    assert height == pytest.approx(a * (1.0 + math.sqrt(0.5)), abs=0.05)


def test_printed_ball_bearing_uncaged_variant():
    parts = printed_ball_bearing(cage=False, balls=5)
    assert sorted(parts) == ["balls", "inner_race", "outer_race"]
    for mesh in parts.values():
        assert_mesh(mesh)
    whole = trimesh.util.concatenate(list(parts.values()))
    assert len(whole.split(only_watertight=False)) == 5 + 2
    assert meshutil.min_distance(parts["balls"], parts["inner_race"]) == \
        pytest.approx(0.3, abs=0.05)


def test_printed_ball_bearing_auto_ball_fills_the_radial_budget():
    parts = printed_ball_bearing(outer_d=34.0, width=12.0, ball_d=None)
    meta = parts["balls"].metadata
    assert meta["ball_d"] == pytest.approx(
        (34.0 - 8.0) / 2.0 - 2 * 2.0 - 2 * 0.3)
    assert meta["pitch_r"] == pytest.approx(
        4.0 + 2.0 + 0.3 + meta["ball_d"] / 2.0)
    for mesh in parts.values():
        assert_mesh(mesh)


def test_printed_ball_bearing_refuses_unprintable_geometry():
    # A 608 footprint only leaves a 2.4 mm ball: not printable, so it fails.
    with pytest.raises(ValueError) as excinfo:
        printed_ball_bearing(bore_d=8.0, outer_d=22.0, width=7.0, ball_d=None)
    assert "printable floor" in str(excinfo.value)
    with pytest.raises(ValueError):
        printed_ball_bearing(ball_d=4.0)
    with pytest.raises(ValueError):
        printed_ball_bearing(bore_d=8.0, outer_d=22.0, width=7.0)
    with pytest.raises(ValueError):
        printed_ball_bearing(balls=9)             # balls would touch
    with pytest.raises(ValueError):
        printed_ball_bearing(width=6.0)           # ball stack will not fit
    with pytest.raises(ValueError):
        printed_ball_bearing(clear=0.05)          # would fuse in the slicer
    with pytest.raises(ValueError):
        printed_ball_bearing(race_wall=0.4)
