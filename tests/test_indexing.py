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
    geneva_wheel_angle,
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


def test_geneva_wheel_angle_indexes_once_per_turn_and_dwells():
    for slots, crank_r in ((3, 28.0), (6, 10.0), (12, 8.0)):
        wheel_angle = geneva_wheel_angle(slots, crank_r)
        theta_e = 90.0 - 180.0 / slots

        def travel(theta):
            """Wheel motion since the engagement centre, unwrapped to +/-180."""
            return ((wheel_angle(theta) - wheel_angle(0.0) + 180.0) % 360.0) - 180.0

        # One slot per crank revolution, and the sense is consistent: the wheel
        # runs opposite the driver.
        step = travel(theta_e) - travel(-theta_e)
        assert step == pytest.approx(-360.0 / slots)
        # Dwell: outside the engagement window the pin is clear of every slot,
        # so the wheel is locked at the window edge and does not creep.
        held = travel(theta_e)
        for theta in (theta_e + 1.0, 90.0, 135.0, 180.0 - 1e-9):
            if theta > 180.0:
                continue
            assert travel(theta) == pytest.approx(held)
        # It is the same relation the full layout builds the parts around.
        layout = _geneva_layout(slots, crank_r, 3.0, 0.25, 2.0, 0.30)
        for theta in (-120.0, -30.0, 0.0, 15.0, 75.0):
            assert layout["wheel_angle"](theta) == pytest.approx(wheel_angle(theta))
    with pytest.raises(ValueError):
        geneva_wheel_angle(2, 10.0)
    with pytest.raises(ValueError):
        geneva_wheel_angle(6, 0.0)


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


# ---------------------------------------------------------------------------
# GEOS-degeneracy guard
#
# The Geneva boolean chain builds a rim disc, subtracts radial slots, and
# subtracts locking pockets whose radius makes each scallop very nearly
# tangent to the rim. Near-tangency plus high vertex density is what makes
# GEOS' floating-point overlay emit "found non-noded intersection"; older
# builds (the WASM one in the browser playground) abort the process instead
# of raising, so nothing downstream can recover. The overlays now run on a
# fixed precision grid. Zero-length edges and consecutive duplicate vertices
# are the locally testable proxy for that degeneracy class.
# ---------------------------------------------------------------------------

GENEVA_CRANK_R = {3: 28.0, 4: 19.0}


def assert_no_degenerate_edges(geometry, label, min_edge=1e-9):
    """No consecutive duplicate vertices and no zero-length edges."""
    rings = []
    parts = getattr(geometry, "geoms", [geometry])
    for part in parts:
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


@pytest.mark.parametrize("slots", list(range(3, 13)))
@pytest.mark.parametrize("clearance", [0.1, 0.25, 0.5])
def test_geneva_pair_sweep_is_sound_and_non_degenerate(slots, clearance):
    crank_r = GENEVA_CRANK_R.get(slots, 10.0)
    layout = _geneva_layout(slots, crank_r, 3.0, clearance, 2.0, 0.30)
    for key in ("wheel", "driver_plane", "cutout"):
        geometry = layout[key]
        assert geometry.is_valid and not geometry.is_empty
        assert_no_degenerate_edges(geometry, "geneva %s" % key)
    parts = geneva_pair(slots=slots, crank_r=crank_r, clearance=clearance)
    for mesh in parts.values():
        assert_mesh(mesh)
