import numpy as np
import pytest
import trimesh

from mechlib.guides import linear_way, telescoping_stage
from mechlib.meshutil import bore_pierces, min_distance, overlap_volume

PROFILES = ("dovetail", "vee", "tslot")


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0
    assert len(mesh.split(only_watertight=False)) == 1


def test_linear_way_returns_named_parts_for_every_profile():
    for profile in PROFILES:
        parts = linear_way(profile=profile)
        assert set(parts) == {"rail", "carriage", "gib"}
        for mesh in parts.values():
            assert_mesh(mesh)
        assert parts["rail"].metadata["profile"] == profile
    plain = linear_way(gib=False)
    assert set(plain) == {"rail", "carriage"}
    for mesh in plain.values():
        assert_mesh(mesh)


def test_linear_way_running_clearance_is_exactly_as_designed():
    # The single most important property of a guideway: the sliding faces sit
    # one running clearance apart and nowhere closer.
    for profile in PROFILES:
        for clear in (0.2, 0.35):
            parts = linear_way(profile=profile, clear=clear)
            rail, carriage, gib = (parts["rail"], parts["carriage"],
                                   parts["gib"])
            assert min_distance(rail, carriage, 3000) == pytest.approx(
                clear, abs=0.01)
            assert min_distance(rail, gib, 3000) == pytest.approx(
                clear, abs=0.01)
            assert min_distance(gib, carriage, 3000) == pytest.approx(
                clear, abs=0.01)
            assert overlap_volume(rail, carriage) < 1e-3
            assert overlap_volume(rail, gib) < 1e-3
            assert overlap_volume(gib, carriage) < 1e-3
            assert rail.metadata["clear"] == clear


def test_dovetail_and_tslot_capture_the_carriage_but_the_vee_does_not():
    for profile in ("dovetail", "tslot"):
        meta = linear_way(profile=profile)["carriage"].metadata
        # The groove mouth is narrower than the rail's widest point, so the
        # carriage cannot be lifted straight off.
        assert meta["throat_w"] < meta["rail_max_w"]
        assert meta["throat_w"] < meta["widest_w"] - 0.5
        assert meta["captures"] is True
    vee = linear_way(profile="vee")["carriage"].metadata
    assert vee["throat_w"] == pytest.approx(vee["widest_w"], abs=1e-6)
    assert vee["captures"] is False


def test_dovetail_carriage_hooks_under_the_rail_in_real_geometry():
    parts = linear_way(profile="dovetail", section_w=26.0, section_h=6.0,
                       angle_deg=55.0, clear=0.25)
    carriage = parts["carriage"]
    # Probe just above the rail root: carriage material sits on both sides of
    # the rail there, i.e. two lips reach under the widest part of the rail.
    y = 0.25 + 0.4
    hits = carriage.contains(np.array([[x, y, 35.0] for x in
                                       np.linspace(-16.0, 16.0, 321)]))
    runs = np.diff(np.r_[0, hits.astype(int), 0])
    assert int((runs == 1).sum()) == 2
    # Those lips lie inboard of the rail's widest half-width (13 mm).
    xs = np.linspace(-16.0, 16.0, 321)[hits]
    assert xs.min() < -13.0 and xs.max() > 13.0
    inner = xs[xs > 0].min()
    assert inner < 13.0


def test_linear_way_end_stops_and_travel_metadata():
    length, carriage_len, stop_h, clear = 70.0, 28.0, 3.0, 0.25
    parts = linear_way(length=length, carriage_len=carriage_len,
                       stop_h=stop_h, clear=clear)
    rail = parts["rail"]
    assert rail.metadata["travel"] == pytest.approx(
        length - carriage_len - 2.0 * (stop_h - clear))
    # The ramps stand outboard of the rail section on the base plate and rise
    # to stop_h at each end of the rail.
    assert rail.bounds[1][1] == pytest.approx(6.0)
    for z in (0.5, length - 0.5):
        assert rail.contains(np.array([[15.0, 1.0, z]]))[0]
        assert rail.contains(np.array([[-15.0, 1.0, z]]))[0]
    # Mid-rail the same spot is open air, so the carriage runs freely there.
    assert not rail.contains(np.array([[15.0, 1.0, length / 2.0]]))[0]
    # Nothing at all when the stops are switched off.
    flat = linear_way(stop_h=0.0)["rail"]
    assert not flat.contains(np.array([[15.0, 1.0, 0.5]]))[0]


def test_linear_way_relief_grooves_open_the_inside_corners():
    relieved = linear_way(relief_r=0.7)
    sharp = linear_way(relief_r=0.0)
    for key in ("rail", "carriage"):
        assert_mesh(relieved[key])
        assert_mesh(sharp[key])
        # Relief only ever removes material from an inside corner.
        assert relieved[key].volume < sharp[key].volume
    # A probe sitting in the dovetail root corner (the flank foot lands at
    # x = 13 - 6/tan(55) = 8.8) is solid without relief and void once the
    # groove is cut.
    corner = np.array([[9.2, -0.2, 35.0]])
    assert sharp["rail"].contains(corner)[0]
    assert not relieved["rail"].contains(corner)[0]


def test_linear_way_mount_holes_pierce_both_flanges():
    rail = linear_way(mount_d=3.4, mount_pitch=25.0, length=70.0)["rail"]
    for sign in (1.0, -1.0):
        for z in (6.0, 35.0, 64.0):
            assert bore_pierces(rail, (sign * 16.0, -3.5, z), (0, 1, 0),
                                4.0, 9)
        # Between holes the flange is solid.
        assert not bore_pierces(rail, (sign * 16.0, -3.5, 20.0), (0, 1, 0),
                                4.0, 9)
    blank = linear_way(mount_d=0.0)["rail"]
    assert not bore_pierces(blank, (16.0, -3.5, 35.0), (0, 1, 0), 4.0, 9)


def test_linear_way_gib_taper_sets_the_preload_rate():
    parts = linear_way(gib_taper=0.8, carriage_len=28.0)
    gib = parts["gib"]
    assert_mesh(gib)
    assert gib.metadata["gib_preload_per_mm"] == pytest.approx(0.8 / 28.0)
    # The wedge really is a wedge: a unit-thick slab through its +Z end holds
    # more gib than the same slab through its -Z end.
    def slab_area(z):
        slab = trimesh.creation.box(extents=(120.0, 60.0, 1.0))
        slab.apply_translation((0.0, 0.0, z))
        return overlap_volume(gib, slab)

    assert slab_area(46.0) > slab_area(22.0) + 2.0
    steep = linear_way(gib_taper=1.4)["gib"]
    assert steep.metadata["gib_preload_per_mm"] > gib.metadata[
        "gib_preload_per_mm"]


def test_linear_way_screw_gib_taps_an_accessible_end_tab():
    # A coaxial push screw needs a pocket wider than the screw, and the
    # library says so instead of shipping a hole that breaks into the rail.
    with pytest.raises(ValueError):
        linear_way(gib_screw=True, gib_screw_d=3.0, gib_t=2.0)
    kw = dict(gib_screw_d=3.0, gib_t=4.0, wall=6.0, gib_tab_t=5.0)
    plain = linear_way(gib_screw=False, **kw)
    screwed = linear_way(gib_screw=True, **kw)
    for mesh in screwed.values():
        assert_mesh(mesh)
    assert screwed["carriage"].volume > plain["carriage"].volume
    assert screwed["carriage"].bounds[1][2] == pytest.approx(
        plain["carriage"].bounds[1][2] + 5.0)
    # The tab eats its own length out of the usable travel.
    assert screwed["carriage"].metadata["travel"] == pytest.approx(
        plain["carriage"].metadata["travel"] - 5.0)
    # The tapped hole runs down the gib's own axis, opens on the tab's outer
    # end face, and is surrounded by tab material: a screwdriver reaches it.
    ax, ay = 13.0 - 6.0 / np.tan(np.radians(55.0)), 0.0
    vx, vy = 13.0 - ax, 6.0
    flank = np.hypot(vx, vy)
    vx, vy = vx / flank, vy / flank
    px = ax + vx * flank / 2.0 + vy * (0.25 + 2.0)
    py = ay + vy * flank / 2.0 - vx * (0.25 + 2.0)
    carriage = screwed["carriage"]
    assert bore_pierces(carriage, (px, py, 49.2), (0, 0, 1), 4.6, 12)
    for dx, dy in ((2.0, 0.0), (-1.8, 0.0), (0.0, 2.0)):
        assert carriage.contains(np.array([[px + dx, py + dy, 51.5]]))[0]
    # The gib still runs on the design clearance against the carriage.
    assert min_distance(screwed["gib"], carriage, 3000) == pytest.approx(
        0.25, abs=0.01)
    assert overlap_volume(screwed["gib"], carriage) < 1e-3


def test_linear_way_rejects_bad_profiles_and_impossible_clearances():
    with pytest.raises(ValueError):
        linear_way(profile="linear")
    with pytest.raises(ValueError):
        linear_way(profile="dovetail_slide")
    with pytest.raises(ValueError):
        linear_way(clear=0.0)
    with pytest.raises(ValueError):
        linear_way(clear=-0.2)
    with pytest.raises(ValueError):
        linear_way(clear=4.0)
    with pytest.raises(ValueError):
        linear_way(angle_deg=10.0)
    with pytest.raises(ValueError):
        linear_way(section_w=12.0, angle_deg=45.0)
    with pytest.raises(ValueError):
        linear_way(carriage_len=70.0)
    with pytest.raises(ValueError):
        linear_way(wall=2.0)
    with pytest.raises(ValueError):
        linear_way(relief_r=3.0)
    with pytest.raises(ValueError):
        linear_way(gib_t=1.0)


def test_telescoping_stage_parts_and_length_metadata():
    stage = telescoping_stage(sections=3, length=60.0, extend=0.0)
    assert set(stage) == {"section_0", "section_1", "section_2"}
    for mesh in stage.values():
        assert_mesh(mesh)
    meta = stage["section_0"].metadata
    assert meta["sections"] == 3
    assert meta["retracted_length"] == pytest.approx(60.0)
    assert meta["travel_per_joint"] == pytest.approx(60.0 - 1.2 - 4.0)
    assert meta["extended_length"] == pytest.approx(
        60.0 + 2.0 * meta["travel_per_joint"])
    # Retracted, the whole stage is one section long.
    assert max(m.bounds[1][2] for m in stage.values()) == pytest.approx(60.0)
    out = telescoping_stage(sections=3, length=60.0, extend=1.0)
    assert max(m.bounds[1][2] for m in out.values()) == pytest.approx(
        meta["extended_length"])
    assert meta["extended_length"] > 2.5 * meta["retracted_length"]


def test_telescoping_stage_joints_run_on_the_designed_clearance():
    for clear in (0.25, 0.4):
        stage = telescoping_stage(sections=3, clear=clear, extend=0.5)
        for i in range(2):
            a, b = stage["section_%d" % i], stage["section_%d" % (i + 1)]
            assert min_distance(a, b, 3000) == pytest.approx(clear, abs=0.01)
            assert overlap_volume(a, b) < 1e-3


def test_telescoping_stops_actually_block_pullout():
    stage = telescoping_stage(sections=3, extend=1.0)
    outer, inner = stage["section_0"], stage["section_1"]
    # At the stop the parts touch without interfering.
    assert overlap_volume(outer, inner) < 1e-3
    # One more millimetre of extension drives the collar into the lip, which
    # is only possible if the stop is really there.
    for extra in (0.6, 1.0):
        past = inner.copy()
        past.apply_translation((0.0, 0.0, extra))
        assert overlap_volume(outer, past) > 0.5
    # The collar is wider than the opening the lip leaves.
    meta = outer.metadata
    assert meta["collar_w"] > meta["lip_opening"]
    assert meta["collar_w"] - meta["lip_opening"] == pytest.approx(
        2.0 * (1.2 - 0.3))


def test_telescoping_stage_rejects_impossible_stacks():
    with pytest.raises(ValueError):
        telescoping_stage(sections=1)
    with pytest.raises(ValueError):
        telescoping_stage(clear=0.0)
    with pytest.raises(ValueError):
        telescoping_stage(lip_t=0.2, clear=0.3)
    with pytest.raises(ValueError):
        telescoping_stage(sections=8)
    with pytest.raises(ValueError):
        telescoping_stage(length=4.0)
    with pytest.raises(ValueError):
        telescoping_stage(extend=1.5)
    with pytest.raises(ValueError):
        telescoping_stage(wall=0.5)
