import math

import pytest
import shapely.affinity as affinity
import shapely.geometry as sg
import trimesh

from mechlib.indexing import (
    _escapement_profiles,
    _geneva_layout,
    _geneva_sweep_clear,
    _intermittent_profiles,
    escapement,
    geneva_pair,
    intermittent_gear_pair,
)


def assert_polygon(poly):
    assert isinstance(poly, sg.Polygon)
    assert poly.is_valid
    assert not poly.is_empty
    assert poly.area > 0


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def overlap_volume(a, b):
    inter = trimesh.boolean.intersection([a, b], engine="manifold")
    return 0.0 if inter is None or inter.is_empty else abs(inter.volume)


def test_geneva_closed_form_relations_hold():
    slots, crank_r = 6, 10.0
    layout = _geneva_layout(slots, crank_r, 3.0, 0.25, 2.0, 0.30)
    c = layout["center_distance"]
    assert c == pytest.approx(crank_r / math.sin(math.pi / slots))
    assert layout["theta_e"] == pytest.approx(90.0 - 180.0 / slots)
    assert_polygon(layout["wheel"])
    # The wheel-plane profile is a MultiPolygon: the drive pin stands apart
    # from the crescent-cut locking disc (they join below the wheel plane).
    driver_plane = layout["driver_plane"]
    assert driver_plane.geom_type in ("Polygon", "MultiPolygon")
    assert driver_plane.is_valid and driver_plane.area > 0
    parts = geneva_pair(slots=slots, crank_r=crank_r)
    assert parts["wheel"].metadata["center_distance"] == pytest.approx(c)
    assert parts["wheel"].metadata["index_angle_deg"] == pytest.approx(60.0)
    assert parts["wheel"].metadata["engagement_angle_deg"] == pytest.approx(120.0)
    with pytest.raises(ValueError):
        geneva_pair(slots=2)


def test_geneva_cycle_sweep_never_collides():
    for slots, crank_r in ((3, 28.0), (6, 10.0), (12, 8.0)):
        layout = _geneva_layout(slots, crank_r, 3.0, 0.25, 2.0, 0.30)
        ok, theta = _geneva_sweep_clear(layout, step_deg=2.0)
        assert ok, "collision at crank angle %s for %d slots" % (theta, slots)


def test_geneva_posed_pair_is_watertight_clear_and_engaged():
    parts = geneva_pair()
    assert_mesh(parts["driver"])
    assert_mesh(parts["wheel"])
    assert overlap_volume(parts["driver"], parts["wheel"]) < 1e-6
    # Mid-engagement: the pin sits inside the wheel rim circle but inside a
    # slot void, so it is near the wheel solid yet not covered by it.
    layout = _geneva_layout(6, 10.0, 3.0, 0.25, 2.0, 0.30)
    wheel = affinity.translate(
        affinity.rotate(layout["wheel"], 180.0, origin=(0, 0)),
        layout["center_distance"], 0.0)
    pin_point = sg.Point(10.0, 0.0)
    assert not wheel.covers(pin_point)
    assert wheel.distance(pin_point) < 2.0


def test_geneva_dwell_locks_against_nudge():
    layout = _geneva_layout(6, 10.0, 3.0, 0.25, 2.0, 0.30)
    c = layout["center_distance"]
    theta = 180.0  # mid-dwell
    wheel = affinity.translate(
        affinity.rotate(layout["wheel"], layout["wheel_angle"](theta),
                        origin=(0, 0)), c, 0.0)
    driver = affinity.rotate(layout["driver_plane"], theta, origin=(0, 0))
    assert wheel.intersection(driver).area < 1e-6
    for nudge in (-6.0, 6.0):
        moved = affinity.rotate(wheel, nudge, origin=(c, 0.0))
        assert moved.intersection(driver).area > 0.5


def test_geneva_clearance_widens_slots():
    tight = _geneva_layout(6, 10.0, 3.0, 0.10, 2.0, 0.30)["wheel"]
    loose = _geneva_layout(6, 10.0, 3.0, 0.50, 2.0, 0.30)["wheel"]
    assert tight.wkb != loose.wkb
    assert tight.area > loose.area  # wider slots remove more material


def test_escapement_parts_are_flat_watertight_and_clear():
    for style in ("anchor", "deadbeat"):
        parts = escapement(style=style)
        assert_mesh(parts["wheel"])
        assert_mesh(parts["anchor"])
        assert overlap_volume(parts["wheel"], parts["anchor"]) < 1e-6


def test_escapement_tooth_count_and_engagement_gap():
    teeth = 30
    wheel, anchor, pivot = _escapement_profiles(
        teeth, "anchor", 22.0, 3.2, 7.5, 0.25, 12.0)
    assert_polygon(wheel)
    assert_polygon(anchor)
    circle = sg.LineString([
        (21.9 * math.cos(math.radians(0.5 * i)),
         21.9 * math.sin(math.radians(0.5 * i))) for i in range(721)])
    crossings = wheel.boundary.intersection(circle)
    assert crossings.geom_type == "MultiPoint"
    assert len(crossings.geoms) == 2 * teeth
    # One pallet engages one tooth: tip at the entry pallet angle, gap = clearance.
    tip = sg.Point(22.0 * math.cos(math.radians(135.0)),
                   22.0 * math.sin(math.radians(135.0)))
    assert anchor.distance(tip) == pytest.approx(0.25, abs=0.05)


def test_escapement_styles_differ_and_recoil_signature():
    anchor_wheel, anchor_anchor, _ = _escapement_profiles(
        30, "anchor", 22.0, 3.2, 7.5, 0.25, 12.0)
    dead_wheel, dead_anchor, _ = _escapement_profiles(
        30, "deadbeat", 22.0, 3.2, 7.5, 0.25, 12.0)
    assert anchor_anchor.wkb != dead_anchor.wkb
    # Nudging the wheel backward against the locked pallet: the recoil
    # anchor's tilted flat face blocks hard, while the deadbeat's
    # pivot-centred arc lets the tooth tip slide with almost no interference.
    recoil = affinity.rotate(anchor_wheel, -1.0, origin=(0, 0))
    dead = affinity.rotate(dead_wheel, -1.0, origin=(0, 0))
    overlap_recoil = recoil.intersection(anchor_anchor).area
    overlap_dead = dead.intersection(dead_anchor).area
    assert overlap_dead < 0.02
    assert overlap_recoil > 5.0 * overlap_dead


def test_escapement_parameter_validation():
    with pytest.raises(ValueError):
        escapement(style="swiss")
    with pytest.raises(ValueError):
        escapement(pallet_span=7.0)  # must be a half-integer
    with pytest.raises(ValueError):
        escapement(teeth=16, pallet_span=7.5)  # span too wide for tooth count


def test_intermittent_pair_is_watertight_and_meshed_clear():
    parts = intermittent_gear_pair()
    assert_mesh(parts["driver"])
    assert_mesh(parts["driven"])
    assert overlap_volume(parts["driver"], parts["driven"]) < 1e-6
    meta = parts["driver"].metadata
    assert meta["center_distance"] == pytest.approx(1.5 * (18 + 18) / 2.0)
    assert meta["advance_per_rev_deg"] == pytest.approx(120.0)
    assert meta["pitches_advanced"] == 6


def test_intermittent_lock_segment_and_notch_geometry():
    driver, driven, meta = _intermittent_profiles(
        1.5, 18, 18, 3, 0.25, 0.5, 12.0, 20.0, 0.35)
    assert_polygon(driver)
    assert_polygon(driven)
    lock_r = meta["lock_r"]
    # Locking segment present opposite the tooth arc; no teeth there.
    assert driver.covers(sg.Point(-(lock_r - 0.3), 0.0))
    assert not driver.covers(sg.Point(-14.5, 0.0))
    # Teeth retained on the engagement arc.
    assert driver.covers(sg.Point(13.8 * math.cos(math.radians(10.0)),
                                  13.8 * math.sin(math.radians(10.0))))
    # Driven gear: concave notch cut at the first lock position (240 deg).
    cd = meta["center_distance"]
    notch = sg.Point(cd + 14.6 * math.cos(math.radians(240.0)),
                     14.6 * math.sin(math.radians(240.0)))
    assert not driven.covers(notch)
    tooth = sg.Point(cd + 14.6 * math.cos(math.radians(60.0)),
                     14.6 * math.sin(math.radians(60.0)))
    assert driven.covers(tooth)


def test_intermittent_parameter_validation():
    with pytest.raises(ValueError):
        intermittent_gear_pair(groups=4)   # 4 does not divide 18
    with pytest.raises(ValueError):
        intermittent_gear_pair(groups=9)   # leaves < 3 teeth per group
    with pytest.raises(ValueError):
        intermittent_gear_pair(groups=1)   # no room for the lock segment
