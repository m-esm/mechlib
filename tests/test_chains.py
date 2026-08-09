import math
import itertools

import pytest
import trimesh
import trimesh.transformations as tf

from mechlib.chains import (
    chain_dual_output,
    chain_reverse,
    chain_s_wrap,
    drag_chain_link,
    drag_chain,
    roller_chain_link,
    roller_chain,
)
from mechlib.gears import roller_sprocket_2d
from mechlib.meshutil import extrude_poly_z, min_distance, overlap_volume


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def _pose(mesh, heading_deg, pos=(0.0, 0.0)):
    m = mesh.copy()
    T = tf.rotation_matrix(math.radians(heading_deg), (0, 0, 1))
    T[0, 3] = pos[0]
    T[1, 3] = pos[1]
    m.apply_transform(T)
    return m


# ---------------------------------------------------------------- drag_chain_link


def test_drag_chain_link_returns_named_watertight_parts():
    out = drag_chain_link()
    assert set(out) == {"link", "pin", "lid"}
    for mesh in out.values():
        assert_mesh(mesh)
    no_lid = drag_chain_link(lid=False)
    assert set(no_lid) == {"link", "pin"}


def test_drag_chain_link_min_bend_radius_matches_closed_form():
    for bend_deg, pitch in ((20.0, 18.0), (45.0, 24.0)):
        out = drag_chain_link(bend_deg=bend_deg, pitch=pitch)
        expect = pitch / (2.0 * math.sin(math.radians(bend_deg / 2.0)))
        assert out["link"].metadata["min_bend_radius"] == pytest.approx(expect)
        assert out["link"].metadata["pitch"] == pitch
        assert out["link"].metadata["bend_deg"] == bend_deg


def test_drag_chain_link_articulation_stop_is_real_geometry():
    # The stop tab/groove is the ONLY thing that should decide contact: two
    # links posed at the designed bend_deg must clear each other (free
    # articulation), while bend_deg + 5 degrees must show real interference
    # -- proving the stop faces are functional geometry, not decoration.
    bend_deg = 30.0
    out = drag_chain_link(bend_deg=bend_deg)
    link = out["link"]
    pitch = link.metadata["pitch"]

    allowed = _pose(link, bend_deg, pos=(pitch, 0.0))
    over = _pose(link, bend_deg + 5.0, pos=(pitch, 0.0))

    assert overlap_volume(link, allowed) == pytest.approx(0.0, abs=1e-6)
    assert overlap_volume(link, over) > 0.05

    # Straight (phi=0) must also be clear -- that is the assembled pose
    # drag_chain() starts every run from.
    straight = _pose(link, 0.0, pos=(pitch, 0.0))
    assert overlap_volume(link, straight) == pytest.approx(0.0, abs=1e-6)


def test_drag_chain_link_pivot_running_clearance():
    clear = 0.3
    out = drag_chain_link(clear=clear)
    link, pin = out["link"], out["pin"]
    pitch = link.metadata["pitch"]

    # The returned "pin" sits at THIS link's own socket (local x=0); the bore
    # it actually runs a tight fit through belongs to a NEIGHBOUR's boss
    # (local x=pitch). Translate a second link so its boss lands where the
    # pin is, and probe the real running clearance there.
    neighbour = link.copy()
    neighbour.apply_translation((-pitch, 0.0, 0.0))
    d = min_distance(pin, neighbour)
    assert d == pytest.approx(clear / 2.0, abs=0.05)
    assert overlap_volume(pin, neighbour) == pytest.approx(0.0, abs=1e-6)


def test_drag_chain_link_rejects_bad_arguments():
    with pytest.raises(ValueError):
        drag_chain_link(width=3.0)
    with pytest.raises(ValueError):
        drag_chain_link(bend_deg=200.0)
    with pytest.raises(ValueError):
        drag_chain_link(pitch=2.0)
    with pytest.raises(ValueError):
        drag_chain_link(pin_d=20.0)
    with pytest.raises(ValueError):
        drag_chain_link(roof_overhang=10.0)


# --------------------------------------------------------------------- drag_chain


def test_drag_chain_part_count_and_names():
    parts = drag_chain(links=6)
    assert set(parts) == set(
        ["link_%02d" % i for i in range(6)]
        + ["pin_%02d" % i for i in range(6)]
        + ["lid_%02d" % i for i in range(6)]
    )
    for mesh in parts.values():
        assert_mesh(mesh)

    no_lid = drag_chain(links=6, lid=False)
    assert len(no_lid) == 12


def test_drag_chain_run_has_no_link_interference():
    parts = drag_chain(links=9, straight_links=3, bend_deg=25.0)
    links = [parts["link_%02d" % i] for i in range(9)]
    for i, j in itertools.combinations(range(len(links)), 2):
        assert overlap_volume(links[i], links[j]) == pytest.approx(0.0, abs=1e-6)


def test_drag_chain_rejects_bad_arguments():
    with pytest.raises(ValueError):
        drag_chain(links=2)
    with pytest.raises(ValueError):
        drag_chain(straight_links=9, links=9)
    with pytest.raises(ValueError):
        drag_chain(straight_links=-1)


# --------------------------------------------------------------- roller_chain_link


def test_roller_chain_link_returns_named_watertight_parts():
    out = roller_chain_link()
    assert set(out) == {"inner_plate", "outer_plate", "roller", "bushing", "pin"}
    for mesh in out.values():
        assert_mesh(mesh)


def test_roller_chain_link_pitch_matches_measured_hole_spacing():
    # The plate footprint is a capsule spanning local x=0..pitch buffered by
    # plate_w/2, so its measured overall length is a real, independent probe
    # of the hole-to-hole spacing actually built into the mesh.
    for pitch, plate_w in ((12.7, 7.0), (9.525, 8.0)):
        out = roller_chain_link(pitch=pitch, plate_w=plate_w)
        plate = out["outer_plate"]
        span = plate.bounds[1][0] - plate.bounds[0][0]
        assert span == pytest.approx(pitch + plate_w, abs=1e-6)
        assert out["roller"].metadata["pitch"] == pitch


def test_roller_chain_link_seats_in_its_mating_sprocket():
    # "Say in the docstring which sprocket call it mates with" -- prove it:
    # build that exact sprocket and check the roller actually seats in a
    # tooth gap with the designed running clearance, computed from
    # roller_sprocket_2d's OWN parameters, not a hardcoded number.
    clear = 0.3
    n_teeth = 15
    out = roller_chain_link(clear=clear)
    roller = out["roller"]
    pitch = roller.metadata["pitch"]
    roller_d = roller.metadata["roller_d"]

    profile = roller_sprocket_2d(n_teeth, pitch, roller_d, clear=clear)
    sprocket = extrude_poly_z(profile, 0.0, 8.0)
    assert_mesh(sprocket)

    rp = pitch / (2.0 * math.sin(math.pi / n_teeth))
    seated = roller.copy()
    seated.apply_translation((rp, 0.0, 0.0))
    assert overlap_volume(seated, sprocket) == pytest.approx(0.0, abs=1e-6)
    assert min_distance(seated, sprocket) == pytest.approx(clear, abs=0.05)


def test_roller_chain_link_rejects_bad_arguments():
    with pytest.raises(ValueError):
        roller_chain_link(roller_d=-1.0)
    with pytest.raises(ValueError):
        roller_chain_link(roller_d=5.0, pin_d=4.0)
    with pytest.raises(ValueError):
        roller_chain_link(plate_w=1.0)


# -------------------------------------------------------------------- roller_chain


def test_roller_chain_sprocket_and_rollers_seat_without_interference():
    clear = 0.25
    out = roller_chain(n_teeth=12, wrap_deg=150.0, clear=clear)
    sprocket = out["sprocket"]
    assert_mesh(sprocket)
    rollers = [v for k, v in out.items() if k.startswith("roller_")]
    assert len(rollers) >= 5
    for r in rollers:
        assert_mesh(r)
        assert overlap_volume(r, sprocket) == pytest.approx(0.0, abs=1e-6)
        assert min_distance(r, sprocket) < 2.0 * clear + 0.2


def test_roller_chain_rejects_bad_arguments():
    with pytest.raises(ValueError):
        roller_chain(n_teeth=3)
    with pytest.raises(ValueError):
        roller_chain(wrap_deg=500.0)
    with pytest.raises(ValueError):
        roller_chain(roller_d=-1.0)


# ------------------------------------------------------------------ chain_reverse


def test_chain_reverse_returns_named_watertight_parts():
    out = chain_reverse()
    assert out["idler"].metadata["reversed"] is True
    rollers = [v for k, v in out.items() if k.startswith("roller_")]
    assert len(rollers) >= 8
    for mesh in out.values():
        assert_mesh(mesh)


def test_chain_reverse_idler_sits_on_the_back_of_the_span():
    # The reversal IS the layout: driver and idler centres on opposite sides
    # of the straight span's pin line y = rp_d means the idler is driven by
    # the back of the chain, so it turns opposite to the driver.
    pitch, n_teeth = 9.525, 12
    out = chain_reverse(n_teeth=n_teeth, pitch=pitch)
    rp_d = pitch / (2.0 * math.sin(math.pi / n_teeth))
    sx, sy = out["sprocket"].metadata["center"]
    ix, iy = out["idler"].metadata["center"]
    assert (sx, sy) == (0.0, 0.0)
    assert sy - rp_d < 0.0 < iy - rp_d
    assert ix == pytest.approx(6 * pitch)  # default span_pitches=6


def test_chain_reverse_rollers_clear_both_sprockets():
    clear = 0.25
    out = chain_reverse(n_teeth=12, idler_teeth=10, clear=clear)
    sprocket, idler = out["sprocket"], out["idler"]
    rollers = [(k, v) for k, v in out.items() if k.startswith("roller_")]
    for _, r in rollers:
        assert overlap_volume(r, sprocket) == pytest.approx(0.0, abs=1e-6)
        assert overlap_volume(r, idler) == pytest.approx(0.0, abs=1e-6)

    # Seated wrap rollers run at the designed clearance: the first driver
    # wrap roller (gap angle 90 + step) and the first idler wrap roller
    # (gap angle -90 + step) both sit in conjugate tooth gaps.
    step_d = 360.0 / 12
    step_i = 360.0 / 10
    ang = math.radians(90.0 + step_d)
    rp_d = 9.525 / (2.0 * math.sin(math.pi / 12))
    assert rollers[0][1].bounds is not None  # roller_00 is the driver wrap
    c = rollers[0][1].centroid
    assert c[0] == pytest.approx(rp_d * math.cos(ang), abs=1e-6)
    assert c[1] == pytest.approx(rp_d * math.sin(ang), abs=1e-6)
    assert min_distance(rollers[0][1], sprocket) == pytest.approx(clear, abs=0.05)
    assert min_distance(rollers[-1][1], idler) == pytest.approx(clear, abs=0.05)
    # The tangent station shared by span and idler must also seat at clear.
    tangent = rollers[-1 - int(round(80.0 / step_i))][1]
    assert min_distance(tangent, idler) == pytest.approx(clear, abs=0.05)


def test_chain_reverse_rejects_bad_arguments():
    with pytest.raises(ValueError):
        chain_reverse(idler_teeth=3)
    with pytest.raises(ValueError):
        chain_reverse(span_pitches=1)
    with pytest.raises(ValueError):
        chain_reverse(wrap_deg=500.0)
    with pytest.raises(ValueError):
        chain_reverse(idler_wrap_deg=200.0)
    with pytest.raises(ValueError):
        chain_reverse(n_teeth=40, idler_teeth=40, span_pitches=2)


# ------------------------------------------------------------------ chain_s_wrap


def test_chain_s_wrap_returns_named_watertight_parts():
    out = chain_s_wrap()
    assert {"sprocket", "out_sprocket"} <= set(out)
    assert out["out_sprocket"].metadata["reversed"] is True
    for mesh in out.values():
        assert_mesh(mesh)


def test_chain_s_wrap_output_wraps_the_back_side():
    # The reversed output must sit on the opposite side of the pin line from
    # the driver, and its wrap must be a real arc (>= 60 deg of rollers).
    pitch, n_teeth = 9.525, 12
    out = chain_s_wrap(out_wrap_deg=150.0)
    rp_d = pitch / (2.0 * math.sin(math.pi / n_teeth))
    sy = out["sprocket"].metadata["center"][1]
    oy = out["out_sprocket"].metadata["center"][1]
    assert (sy - rp_d) < 0.0 < (oy - rp_d)
    rollers = [v for k, v in out.items() if k.startswith("roller_")]
    sprocket, out_sp = out["sprocket"], out["out_sprocket"]
    for r in rollers:
        assert overlap_volume(r, sprocket) == pytest.approx(0.0, abs=1e-6)
        assert overlap_volume(r, out_sp) == pytest.approx(0.0, abs=1e-6)
    clear = 0.3
    assert min_distance(rollers[0], sprocket) == pytest.approx(clear, abs=0.05)
    assert min_distance(rollers[-1], out_sp) == pytest.approx(clear, abs=0.05)


def test_chain_s_wrap_rejects_bad_arguments():
    with pytest.raises(ValueError):
        chain_s_wrap(out_wrap_deg=30.0)
    with pytest.raises(ValueError):
        chain_s_wrap(out_wrap_deg=200.0)
    with pytest.raises(ValueError):
        chain_s_wrap(out_teeth=3)


# ------------------------------------------------------------- chain_dual_output


def test_chain_dual_output_returns_named_watertight_parts():
    out = chain_dual_output()
    assert {"driver", "out_forward", "idler_reverse"} <= set(out)
    assert out["driver"].metadata["reversed"] is False
    assert out["out_forward"].metadata["reversed"] is False
    assert out["idler_reverse"].metadata["reversed"] is True
    rollers = [v for k, v in out.items() if k.startswith("roller_")]
    assert len(rollers) >= 15
    for mesh in out.values():
        assert_mesh(mesh)


def test_chain_dual_output_serpentine_run_clears_everything():
    clear = 0.25
    out = chain_dual_output(clear=clear)
    sprockets = [out["driver"], out["out_forward"], out["idler_reverse"]]
    rollers = [v for k, v in out.items() if k.startswith("roller_")]
    for r in rollers:
        for sp in sprockets:
            assert overlap_volume(r, sp) == pytest.approx(0.0, abs=1e-6)
    for a, b in itertools.combinations(rollers, 2):
        assert overlap_volume(a, b) == pytest.approx(0.0, abs=1e-6)
    for a, b in itertools.combinations(sprockets, 2):
        assert overlap_volume(a, b) == pytest.approx(0.0, abs=1e-6)
    # Seated rollers at both ends of the path run at the design clearance.
    assert min_distance(rollers[0], out["driver"]) == pytest.approx(clear, abs=0.05)
    # First idler wrap roller: after the driver wrap and the in-span stations.
    idx = int(round(160.0 / (360.0 / 12))) + 4 + 1
    assert min_distance(rollers[idx], out["idler_reverse"]) == pytest.approx(
        clear, abs=0.05)


def test_chain_dual_output_forward_and_reverse_sides():
    # Rotation sense = which side of the travelling chain each shaft centre
    # lies on: driver and forward output on the RIGHT of travel (same
    # sense), the backside idler on the LEFT (reversed).
    out = chain_dual_output()
    rp_d = 9.525 / (2.0 * math.sin(math.pi / 12))
    rp_i = 9.525 / (2.0 * math.sin(math.pi / 10))
    # In-span runs +X along y = rp_d: driver below it, idler above.
    dy = out["driver"].metadata["center"][1] - rp_d
    iy = out["idler_reverse"].metadata["center"][1] - rp_d
    assert dy < 0.0 < iy
    # Mid-span leaves the idler at heading th1; the forward centre must be
    # right of that travel direction, like the driver (cross < 0).
    th1 = math.radians(round(60.0 / (360.0 / 10)) * (360.0 / 10))
    ci = out["idler_reverse"].metadata["center"]
    p2 = (ci[0] + rp_i * math.sin(th1), ci[1] - rp_i * math.cos(th1))
    cf = out["out_forward"].metadata["center"]
    cross = math.cos(th1) * (cf[1] - p2[1]) - math.sin(th1) * (cf[0] - p2[0])
    assert cross < 0.0


def test_chain_dual_output_rejects_bad_arguments():
    with pytest.raises(ValueError):
        chain_dual_output(idler_teeth=3)
    with pytest.raises(ValueError):
        chain_dual_output(in_pitches=1)
    with pytest.raises(ValueError):
        chain_dual_output(tail_pitches=0)
    with pytest.raises(ValueError):
        chain_dual_output(out_wrap_deg=500.0)
    with pytest.raises(ValueError):
        chain_dual_output(out_teeth=40, idler_teeth=40,
                          in_pitches=2, mid_pitches=2)


# ---------------------------------------------------- reverse-bend drag chain


def test_drag_chain_link_reverse_bend_articulates_both_ways():
    bend_deg = 30.0
    out = drag_chain_link(bend_deg=bend_deg, reverse_bend=True)
    link = out["link"]
    assert link.metadata["reverse_bend"] is True
    assert drag_chain_link()["link"].metadata["reverse_bend"] is False
    pitch = link.metadata["pitch"]

    for sign in (+1.0, -1.0):
        allowed = _pose(link, sign * bend_deg, pos=(pitch, 0.0))
        over = _pose(link, sign * (bend_deg + 5.0), pos=(pitch, 0.0))
        assert overlap_volume(link, allowed) == pytest.approx(0.0, abs=1e-6)
        assert overlap_volume(link, over) > 0.05


def test_drag_chain_s_bend_run_has_no_link_interference():
    parts = drag_chain(links=9, straight_links=2, bend_deg=25.0,
                       reverse_bend=True, s_bend_at=5)
    links = [parts["link_%02d" % i] for i in range(9)]
    for i, j in itertools.combinations(range(len(links)), 2):
        assert overlap_volume(links[i], links[j]) == pytest.approx(0.0, abs=1e-6)


def test_drag_chain_s_bend_rejects_bad_arguments():
    with pytest.raises(ValueError):
        drag_chain(s_bend_at=5)  # needs reverse_bend=True
    with pytest.raises(ValueError):
        drag_chain(reverse_bend=True, s_bend_at=99)
