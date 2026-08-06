import math
import warnings

import numpy as np
import pytest
import shapely.geometry as sg
import trimesh

from mechlib import meshutil
from mechlib.fluid import (
    gerotor_pump,
    hose_barb,
    peristaltic_pump_head,
    rotary_spool_valve,
)


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def section_polygons(mesh, z):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        path = mesh.section(plane_origin=[0.0, 0.0, z], plane_normal=[0, 0, 1])
        planar, _to_3d = path.to_planar(to_2D=np.eye(4))
    return planar.polygons_full


# --------------------------------------------------------------------------
# gerotor_pump
# --------------------------------------------------------------------------

def test_gerotor_pump_parts_names_and_closed_form_metadata():
    parts = gerotor_pump()
    assert set(parts) == {"inner", "outer", "housing", "cap"}
    for mesh in parts.values():
        assert_mesh(mesh)
    meta = parts["inner"].metadata
    assert meta["outer_lobes"] == meta["lobes"] + 1
    assert meta["ratio"] == pytest.approx(7.0 / 6.0)
    # Displacement is lobes chamber cycles per shaft turn, by the swept area.
    assert meta["displacement_per_rev"] == pytest.approx(
        meta["lobes"] * (meta["chamber_a_max"] - meta["chamber_a_min"])
        * meta["rotor_h"])
    assert meta["displacement_per_rev"] > 0.0
    assert meta["chamber_a_min"] < 0.1 * meta["chamber_a_max"]
    # The default tooth diameter sits inside both hard limits.
    assert meta["tooth_d"] < meta["tooth_d_undercut_limit"]
    assert meta["tooth_d"] < meta["tooth_d_merge_limit"]


def test_gerotor_profile_radii_match_the_trochoid_relations():
    parts = gerotor_pump()
    meta = parts["inner"].metadata
    z = parts["inner"].bounds[0][2] + meta["rotor_h"] / 2.0
    outer = section_polygons(parts["outer"], z)[0]
    cavity = sg.Polygon(outer.interiors[0]).buffer(-meta["clear"] / 2.0)
    radii = np.hypot(*np.asarray(cavity.exterior.coords).T)
    # Tooth tips at lobe_circle_r - tooth_r, root circle at + 2*ecc.
    assert radii.min() == pytest.approx(meta["cavity_tip_r"], abs=0.02)
    assert radii.max() == pytest.approx(meta["cavity_root_r"], abs=0.02)
    assert meta["cavity_root_r"] - meta["cavity_tip_r"] == pytest.approx(
        2.0 * meta["ecc"], abs=1e-6)

    inner = sg.Polygon(section_polygons(parts["inner"], z)[0].exterior)
    inner = inner.buffer(meta["clear"] / 2.0)
    own = np.hypot(np.asarray(inner.exterior.coords)[:, 0] - meta["ecc"],
                   np.asarray(inner.exterior.coords)[:, 1])
    lo = meta["lobe_circle_r"] - meta["ecc"] - meta["tooth_d"] / 2.0
    hi = meta["lobe_circle_r"] + meta["ecc"] - meta["tooth_d"] / 2.0
    assert own.min() == pytest.approx(lo, abs=0.02)
    assert own.max() == pytest.approx(hi, abs=0.02)


def test_gerotor_rotors_never_interfere_and_stay_in_contact():
    clear = 0.3
    for phase in (0.0, 17.0, 31.0, 48.0):
        parts = gerotor_pump(phase_deg=phase, clear=clear)
        inner, outer = parts["inner"], parts["outer"]
        assert meshutil.overlap_volume(inner, outer) < 1e-3
        gap = meshutil.min_distance(inner, outer, n=2500)
        # Sealed crescents need the rotors touching, not merely nested: the
        # measured gap is the designed running clearance, no more.
        assert 0.5 * clear <= gap <= 1.3 * clear
        assert meshutil.overlap_volume(outer, parts["housing"]) < 1e-3


def test_gerotor_displacement_matches_an_independent_area_calculation():
    parts = gerotor_pump()
    meta = parts["inner"].metadata
    z = parts["inner"].bounds[0][2] + meta["rotor_h"] / 2.0
    outer = section_polygons(parts["outer"], z)[0]
    cavity = sg.Polygon(outer.interiors[0]).buffer(-meta["clear"] / 2.0)
    inner = sg.Polygon(section_polygons(parts["inner"], z)[0].exterior)
    inner = inner.buffer(meta["clear"] / 2.0)
    free = cavity.area - inner.area
    # The free area is shared out over outer_lobes chambers and is constant, so
    # the mean chamber equals free/outer_lobes. A chamber cycle that runs from
    # a_min to a_max about that mean swings 2*(mean - a_min). Measured off the
    # meshes, independent of the generator's own polygons.
    swing = 2.0 * (free / meta["outer_lobes"] - meta["chamber_a_min"])
    assert swing == pytest.approx(meta["chamber_a_max"] - meta["chamber_a_min"],
                                  rel=0.1)
    assert meta["lobes"] * swing * meta["rotor_h"] == pytest.approx(
        meta["displacement_per_rev"], rel=0.1)


def test_gerotor_kidney_ports_open_the_chambers_and_seal_the_lands():
    parts = gerotor_pump()
    cap, meta = parts["cap"], parts["cap"].metadata
    z0, z1 = cap.bounds[0][2], cap.bounds[1][2]
    mid_r = (meta["port_r0"] + meta["port_r1"]) / 2.0
    for angle in (90.0, 270.0):
        a = math.radians(angle)
        assert meshutil.bore_pierces(
            cap, (mid_r * math.cos(a), mid_r * math.sin(a), z0 - 0.5),
            (0, 0, 1), (z1 - z0) + 1.0, n=16)
    for angle in (0.0, 180.0):
        a = math.radians(angle)
        assert not meshutil.bore_pierces(
            cap, (mid_r * math.cos(a), mid_r * math.sin(a), z0 - 0.5),
            (0, 0, 1), (z1 - z0) + 1.0, n=16)
    assert gerotor_pump(ports=False)["cap"].volume > cap.volume


def test_gerotor_rotors_repose_rigidly_over_one_tooth_pitch():
    lobes = 6
    start = gerotor_pump(lobes=lobes, phase_deg=0.0)
    end = gerotor_pump(lobes=lobes, phase_deg=360.0 / lobes)
    for name in ("inner", "outer"):
        a = np.sort(np.asarray(start[name].vertices), axis=0)
        b = np.sort(np.asarray(end[name].vertices), axis=0)
        assert a.shape == b.shape
        assert np.abs(a - b).max() < 1e-5


def test_gerotor_rejects_impossible_geometry():
    with pytest.raises(ValueError):
        gerotor_pump(lobes=2)
    with pytest.raises(ValueError):
        gerotor_pump(rotor_h=0.0)
    with pytest.raises(ValueError):
        # Far past the tooth-merge and undercut limits.
        gerotor_pump(tooth_d=30.0)
    with pytest.raises(ValueError):
        gerotor_pump(shaft_d=24.0)
    with pytest.raises(ValueError):
        gerotor_pump(port_seal=6.0)


# --------------------------------------------------------------------------
# hose_barb
# --------------------------------------------------------------------------

def test_hose_barb_crest_exceeds_the_tube_id_by_the_interference():
    tube_id, interference = 6.0, 0.6
    barb = hose_barb(tube_id=tube_id, interference=interference)
    assert_mesh(barb)
    meta = barb.metadata
    assert meta["crest_d"] == pytest.approx(tube_id + interference)
    verts = np.asarray(barb.vertices)
    band = ((verts[:, 2] > meta["barb_z0"] + 0.01) &
            (verts[:, 2] < meta["barb_z1"] - 0.01))
    crest = 2.0 * np.hypot(verts[band, 0], verts[band, 1]).max()
    assert crest == pytest.approx(tube_id + interference, abs=1e-6)
    assert crest > tube_id
    assert meta["root_d"] < tube_id


def test_hose_barb_stack_pitch_bore_and_feet():
    barb = hose_barb(barbs=4, ramp_deg=30.0, barb_gap=1.5)
    assert_mesh(barb)
    meta = barb.metadata
    rise = (meta["crest_d"] - meta["root_d"]) / 2.0
    assert meta["barb_pitch"] == pytest.approx(
        rise / math.tan(math.radians(30.0)) + 1.5)
    assert meta["barb_z1"] - meta["barb_z0"] == pytest.approx(
        4 * meta["barb_pitch"])
    # The bore runs right through, axially.
    assert meshutil.bore_pierces(barb, (0.0, 0.0, -1.0), (0, 0, 1),
                                 meta["total_h"] + 2.0, n=24)
    plain = hose_barb(foot="none", boss_h=0.0)
    assert_mesh(plain)
    assert plain.volume < barb.volume
    threaded = hose_barb(foot="thread")
    assert_mesh(threaded)
    assert threaded.metadata["foot"] == "thread"


def test_hose_barb_rejects_bad_arguments():
    with pytest.raises(ValueError):
        hose_barb(barbs=0)
    with pytest.raises(ValueError):
        hose_barb(ramp_deg=60.0)
    with pytest.raises(ValueError):
        hose_barb(bore_d=5.5)
    with pytest.raises(ValueError):
        hose_barb(foot="press")


# --------------------------------------------------------------------------
# rotary_spool_valve
# --------------------------------------------------------------------------

def test_rotary_spool_valve_parts_and_derived_routing():
    parts = rotary_spool_valve()
    assert set(parts) == {"body", "plug", "cap"}
    for mesh in parts.values():
        assert_mesh(mesh)
    meta = parts["body"].metadata
    # Three ports at 0/120/240 and one L passage give three L positions.
    assert meta["detent_angles_deg"] == (0.0, 120.0, 240.0)
    assert dict(meta["routing"]) == {0.0: ((0, 1),), 120.0: ((1, 2),),
                                     240.0: ((0, 2),)}
    assert meta["closed_deg"] == pytest.approx(60.0)

    four = rotary_spool_valve(ports=4, passages=((0.0, 180.0), (90.0, 270.0)))
    routing = dict(four["body"].metadata["routing"])
    assert routing[0.0] == ((0, 2), (1, 3))
    assert four["body"].metadata["closed_deg"] == pytest.approx(45.0)


def test_rotary_spool_valve_routes_flow_and_blocks_it():
    def open_ports(plug_deg):
        parts = rotary_spool_valve(plug_deg=plug_deg)
        meta = parts["body"].metadata
        solid = meshutil.uni([parts["body"], parts["plug"]])
        reach = meta["body_d"] / 2.0 + 2.0
        z = meta["port_z"]
        found = []
        for index, angle in enumerate(meta["port_angles_deg"]):
            a = math.radians(angle)
            inward = meshutil.bore_pierces(
                solid, (reach * math.cos(a), reach * math.sin(a), z),
                (-math.cos(a), -math.sin(a), 0.0), reach, n=36)
            outward = meshutil.bore_pierces(
                solid, (0.0, 0.0, z), (math.cos(a), math.sin(a), 0.0),
                reach, n=36)
            if inward and outward:
                found.append(index)
        return meta, tuple(found)

    meta, joined = open_ports(0.0)
    # The routed detent opens exactly the pair the derived table names, and
    # each leg is open from the body port right through to the plug axis.
    assert joined == (0, 1)
    _meta, joined = open_ports(120.0)
    assert joined == (1, 2)
    _meta, joined = open_ports(meta["closed_deg"])
    assert joined == ()


def test_rotary_spool_valve_oring_glands_and_validation():
    plain = rotary_spool_valve()
    sealed = rotary_spool_valve(oring_groove=True)
    for mesh in sealed.values():
        assert_mesh(mesh)
    assert sealed["body"].metadata["oring_cs"] == pytest.approx(1.8)
    assert plain["body"].metadata["oring_cs"] == 0.0
    # Two glands cut material out of the plug relative to the same seat height.
    tall = rotary_spool_valve(seat_h=sealed["body"].metadata["seat_h"])
    assert sealed["plug"].volume < tall["plug"].volume

    no_detent = rotary_spool_valve(detents=False)
    assert no_detent["body"].volume > plain["body"].volume

    with pytest.raises(ValueError):
        rotary_spool_valve(ports=1)
    with pytest.raises(ValueError):
        rotary_spool_valve(plug_d=32.0)
    with pytest.raises(ValueError):
        rotary_spool_valve(passages=((0.0,),))
    with pytest.raises(ValueError):
        rotary_spool_valve(passage_d=14.0)
    with pytest.raises(ValueError):
        rotary_spool_valve(port_angles_deg=(0.0, 90.0))


# --------------------------------------------------------------------------
# peristaltic_pump_head
# --------------------------------------------------------------------------

def test_peristaltic_head_squeezes_the_tube_by_the_designed_occlusion():
    parts = peristaltic_pump_head()
    assert set(parts) == {"body", "rotor", "cap"}
    for mesh in parts.values():
        assert_mesh(mesh)
    meta = parts["rotor"].metadata
    # Full occlusion closes the tube to twice its wall; the default backs off.
    assert meta["squeeze_gap"] == pytest.approx(
        meta["tube_od"] - meta["occlusion"] * (meta["tube_od"]
                                               - 2.0 * meta["tube_wall"]))
    verts = np.asarray(parts["rotor"].vertices)
    swept_r = np.hypot(verts[:, 0], verts[:, 1]).max()
    assert meta["race_outer_r"] - swept_r == pytest.approx(meta["squeeze_gap"],
                                                           abs=0.02)
    assert meshutil.overlap_volume(parts["rotor"], parts["body"]) < 1e-3
    # Only the axial running clearance is tighter than the squeeze gap.
    assert meshutil.min_distance(parts["rotor"], parts["body"],
                                 n=2000) == pytest.approx(meta["clear"],
                                                          abs=0.05)
    assert meta["channel_h"] > meta["tube_od"]
    full = peristaltic_pump_head(occlusion=1.0)
    assert full["rotor"].metadata["squeeze_gap"] == pytest.approx(
        2.0 * meta["tube_wall"])


def test_peristaltic_tube_slots_leave_the_race_tangentially():
    parts = peristaltic_pump_head()
    meta, body = parts["body"].metadata, parts["body"]
    z = 3.0 + meta["channel_h"] / 2.0
    open_half = (360.0 - meta["wrap_deg"]) / 2.0
    for angle, sense in ((open_half, -1.0), (360.0 - open_half, 1.0)):
        a = math.radians(angle)
        start = (meta["race_r"] * math.cos(a), meta["race_r"] * math.sin(a), z)
        direction = (-sense * math.sin(a), sense * math.cos(a), 0.0)
        assert meshutil.bore_pierces(body, start, direction, meta["body_d"],
                                     n=30)
        # The slot's outer edge is tangent to the race wall, so a probe one
        # tube radius outboard of the exit path stays inside the solid wall.
        off = (meta["tube_od"] + meta["clear"]) / 2.0 + 0.4
        blocked = (start[0] + off * math.cos(a), start[1] + off * math.sin(a), z)
        assert not meshutil.bore_pierces(body, blocked, direction,
                                         meta["body_d"] / 3.0, n=12)


def test_peristaltic_head_rejects_a_wrap_that_lets_the_tube_open():
    with pytest.raises(ValueError):
        # Three rollers sit 120 deg apart, so a 90 deg wrap leaves the tube
        # unoccluded and the pump back-feeds.
        peristaltic_pump_head(rollers=3, wrap_deg=90.0)
    with pytest.raises(ValueError):
        peristaltic_pump_head(occlusion=1.4)
    with pytest.raises(ValueError):
        peristaltic_pump_head(tube_wall=3.0)
    with pytest.raises(ValueError):
        peristaltic_pump_head(rollers=8, roller_d=14.0)
    with pytest.raises(ValueError):
        peristaltic_pump_head(rotor_h=20.0)
