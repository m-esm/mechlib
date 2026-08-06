import math
import itertools

import pytest
import trimesh
import trimesh.transformations as tf

from mechlib.chains import drag_chain_link, drag_chain, roller_chain_link, roller_chain
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
