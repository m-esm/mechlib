"""Wave v0.10.0 components: catalog gaps across all modules."""

import math

import pytest
import trimesh

from mechlib.cams import roller_follower
from mechlib.closures import annular_snap
from mechlib.couplings import beam_coupling
from mechlib.drives import flywheel
from mechlib.fasteners import shaft_key, thread_insert, tslot_nut
from mechlib.fluid import check_valve
from mechlib.grippers import bellows_suction_cup
from mechlib.indexing import detent_pair, star_wheel
from mechlib.linear import lead_screw
from mechlib.mechanisms import handwheel, shaft_collar, star_knob
from mechlib.pulleys import v_belt_pulley
from mechlib.ratchets import ratchet_wheel_pawl


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def assert_parts(parts):
    for mesh in parts.values():
        assert_mesh(mesh)


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


# ------------------------------------------------------------- v-belt pulley


def test_v_belt_pulley_metadata_and_envelope():
    pulley = v_belt_pulley()
    assert_mesh(pulley)
    meta = pulley.metadata
    assert meta["section"] == "3L"
    assert meta["pitch_d"] == pytest.approx(60.0)
    assert meta["outer_d"] == pytest.approx(60.0 + 2 * 0.3 * 6.0)
    assert meta["bore_d"] == pytest.approx(8.2)
    assert meta["grooves"] == 1
    assert meta["belt_line_speed_per_rpm"] == pytest.approx(
        math.pi * 60.0)


def test_v_belt_pulley_hub_styles_and_grooves():
    assert_mesh(v_belt_pulley(hub="A"))
    assert_mesh(v_belt_pulley(hub="C"))
    multi = v_belt_pulley(section="A", grooves=2, keyway=True)
    assert_mesh(multi)
    assert multi.metadata["grooves"] == 2


def test_v_belt_pulley_validation():
    with pytest.raises(ValueError):
        v_belt_pulley(section="C")
    with pytest.raises(ValueError):
        v_belt_pulley(pitch_d=20.0)  # groove root meets the bore
    with pytest.raises(ValueError):
        v_belt_pulley(hub="D")


# ------------------------------------------------------------------ flywheel


def test_flywheel_spoked_metadata():
    wheel = flywheel()
    assert_mesh(wheel)
    meta = wheel.metadata
    assert meta["style"] == "spoked"
    assert meta["spokes"] == 4
    assert meta["mass_g"] == pytest.approx(wheel.volume * 1.24 / 1000.0)
    # Rim-heavy: inertia must beat a solid disc of the same mass and OD.
    solid_i = 0.5 * (meta["mass_g"] / 1000.0) * (meta["rim_od"] / 2.0) ** 2
    assert meta["inertia_kg_mm2"] > 0.7 * solid_i
    assert meta["stored_energy_J"] > 0


def test_flywheel_web_and_keyway():
    assert_mesh(flywheel(style="web"))
    assert_mesh(flywheel(keyway=True, spokes=5))


def test_flywheel_validation():
    with pytest.raises(ValueError):
        flywheel(style="belt")
    with pytest.raises(ValueError):
        flywheel(rim_od=30.0, bore_d=20.0)
    with pytest.raises(ValueError):
        flywheel(spokes=2)


# ------------------------------------------------------- collar, knob, wheel


def test_shaft_collar_split():
    collar = shaft_collar()
    assert_mesh(collar)
    assert collar.metadata["style"] == "split"
    assert collar.metadata["bore_d"] == pytest.approx(8.2)
    assert collar.metadata["od"] == pytest.approx(16.0)


def test_shaft_collar_setscrew_and_validation():
    assert_mesh(shaft_collar(style="setscrew"))
    with pytest.raises(ValueError):
        shaft_collar(style="glue")
    with pytest.raises(ValueError):
        shaft_collar(screw="M9")


def test_star_knob():
    knob = star_knob()
    assert_mesh(knob)
    assert knob.metadata["thread"] == "M6"
    assert not knob.metadata["through"]
    assert_mesh(star_knob(through=True, thread="M4", lobes=6))
    with pytest.raises(ValueError):
        star_knob(lobes=2)


def test_handwheel_plain_and_crank():
    wheel = handwheel()
    assert_mesh(wheel)
    assert wheel.metadata["spokes"] == 3
    parts = handwheel(crank=True)
    assert_parts(parts)
    # The grip must clear the pin: grip volume is a tube, not a solid.
    grip = parts["grip"]
    assert grip.volume < math.pi * (14.0 / 2.0) ** 2 * 40.0 * 0.9
    with pytest.raises(ValueError):
        handwheel(spokes=1)


# ------------------------------------------------------------- beam coupling


def test_beam_coupling_clamp():
    coupling = beam_coupling()
    assert_mesh(coupling)
    meta = coupling.metadata
    assert meta["bore_a"] == pytest.approx(5.15)
    assert meta["bore_b"] == pytest.approx(8.15)
    assert meta["slit_pitch"] > meta["slit_w"]
    assert meta["clamp"]


def test_beam_coupling_variants():
    assert_mesh(beam_coupling(clamp=False))
    assert_mesh(beam_coupling(helix_starts=2))
    with pytest.raises(ValueError):
        beam_coupling(length=12.0)
    with pytest.raises(ValueError):
        beam_coupling(screw="M6")


# ---------------------------------------------------------------- lead screw


def test_lead_screw_pair():
    parts = lead_screw()
    assert_parts(parts)
    meta = parts["screw"].metadata
    assert meta["lead"] == pytest.approx(3.0)
    assert meta["style"] == "Tr"
    # Nut thread is clearance-grown: nut material starts beyond d/2+clear.
    assert parts["nut"].metadata["d"] == pytest.approx(12.0)


def test_lead_screw_multistart_flange():
    parts = lead_screw(starts=2, flange_d=30.0)
    assert_parts(parts)
    assert parts["screw"].metadata["lead"] == pytest.approx(6.0)
    with pytest.raises(ValueError):
        lead_screw(pitch=8.0)
    with pytest.raises(ValueError):
        lead_screw(starts=0)


# ----------------------------------------------------------- roller follower


def test_roller_follower_plain():
    parts = roller_follower()
    assert_parts(parts)
    meta = parts["arm"].metadata
    assert meta["roller"] == "plain"
    assert meta["arm_len"] == pytest.approx(36.0)
    assert meta["pin_d"] == pytest.approx(5.0)


def test_roller_follower_bearing():
    parts = roller_follower(roller="bearing", bearing="625", roller_d=20.0,
                            roller_w=8.0, spring_tab=True, pose_deg=20.0)
    assert_parts(parts)
    meta = parts["roller"].metadata
    assert meta["roller"] == "bearing"
    assert meta["bearing_pocket_d"] == pytest.approx(16.25)
    with pytest.raises(ValueError):
        roller_follower(roller="bearing")
    with pytest.raises(ValueError):
        roller_follower(clear=0.05)


# --------------------------------------------------- detent pair, star wheel


def test_detent_pair_with_housing():
    parts = detent_pair()
    assert_parts(parts)
    assert set(parts) == {"wheel", "plunger", "housing"}
    meta = parts["wheel"].metadata
    assert meta["detent_angle_deg"] == pytest.approx(30.0)
    assert meta["notch"] == "vee"
    # Plunger nose reaches into the notch: tip x sits inside the rim.
    tip_x = parts["plunger"].bounds[0][0]
    assert tip_x < parts["wheel"].metadata["wheel_r"]


def test_detent_pair_radiused():
    parts = detent_pair(notch="radiused", housing=False)
    assert_parts(parts)
    assert set(parts) == {"wheel", "plunger"}
    with pytest.raises(ValueError):
        detent_pair(detents=2)
    with pytest.raises(ValueError):
        detent_pair(notch="square")


def test_star_wheel():
    wheel = star_wheel()
    assert_mesh(wheel)
    meta = wheel.metadata
    assert meta["pockets"] == 6
    assert meta["pitch_mm"] == pytest.approx(
        2.0 * (meta["wheel_r"] - 0.35 * 22.0) * math.sin(math.pi / 6))
    assert meta["through"]


def test_star_wheel_floor_and_validation():
    floored = star_wheel(through=False, pockets=8, pocket_d=16.0)
    assert_mesh(floored)
    assert not floored.metadata["through"]
    with pytest.raises(ValueError):
        star_wheel(wheel_r=20.0)
    with pytest.raises(ValueError):
        star_wheel(pockets=2)


# ----------------------------------------------------------- ratchet + pawl


def test_ratchet_wheel_pawl_leaf():
    parts = ratchet_wheel_pawl()
    assert_parts(parts)
    assert set(parts) == {"wheel", "pawl", "spring"}
    meta = parts["wheel"].metadata
    assert meta["teeth"] == 14
    assert meta["circular_pitch"] == pytest.approx(2.0 * math.pi * 20.0 / 14)
    # Pawl tip boss overlaps the wheel tip circle: posed engaged.
    pawl_min_x = parts["pawl"].bounds[0][0]
    assert pawl_min_x < meta["tip_r"]


def test_ratchet_wheel_pawl_coil():
    parts = ratchet_wheel_pawl(spring="coil", flat=True)
    assert_parts(parts)
    assert set(parts) == {"wheel", "pawl"}
    with pytest.raises(ValueError):
        ratchet_wheel_pawl(teeth=4)
    with pytest.raises(ValueError):
        ratchet_wheel_pawl(spring="torsion")


# ---------------------------------------------- insert, t-slot nut, key


def test_thread_insert_set():
    parts = thread_insert()
    assert_parts(parts)
    assert set(parts) == {"insert", "boss", "cavity"}
    meta = parts["insert"].metadata
    assert meta["d"] == "M3"
    assert meta["insert_od"] == pytest.approx(4.6)
    assert meta["length"] == pytest.approx(4.0)
    # Cavity clears the insert OD for a heat-set slip fit.
    assert parts["cavity"].bounds[1][0] >= meta["insert_od"] / 2.0


def test_thread_insert_variants():
    parts = thread_insert(d="M5", boss=False)
    assert set(parts) == {"insert", "cavity"}
    assert parts["insert"].metadata["insert_od"] == pytest.approx(7.1)
    with pytest.raises(ValueError):
        thread_insert(d="M8")


def test_tslot_nut():
    nut = tslot_nut()
    assert_mesh(nut)
    meta = nut.metadata
    assert meta["profile"] == "2020"
    assert meta["slot_w"] == pytest.approx(6.0)
    assert meta["wing_w"] == pytest.approx(11.5)
    assert meta["thread_d"] == "M4"
    assert_mesh(tslot_nut(profile="3030", thread_d="M6",
                          style="slide_in", spring_leaf=False))
    with pytest.raises(ValueError):
        tslot_nut(thread_d="M8")


def test_shaft_key_parallel():
    parts = shaft_key()
    assert_parts(parts)
    meta = parts["key"].metadata
    assert meta["key_w"] == pytest.approx(4.0)
    assert meta["key_h"] == pytest.approx(4.0)
    assert meta["length"] == pytest.approx(24.0)


def test_shaft_key_woodruff():
    parts = shaft_key(style="woodruff")
    assert_parts(parts)
    assert parts["key"].metadata["style"] == "woodruff"
    with pytest.raises(ValueError):
        shaft_key(shaft_d=40.0)
    with pytest.raises(ValueError):
        shaft_key(style="flat")


# -------------------------------------------------------------- annular snap


def test_annular_snap_pair():
    parts = annular_snap()
    assert_parts(parts)
    assert set(parts) == {"ridge", "groove"}
    meta = parts["ridge"].metadata
    assert meta["ridge_h"] == pytest.approx(0.6)
    assert meta["lead_angle"] == pytest.approx(30.0)
    # Groove must not cut through a 2 mm outer wall: outer extent under
    # r_surf + clear + wall.
    groove_r = parts["groove"].bounds[1][0]
    assert groove_r < 40.0 / 2.0 - 2.0 + 0.15 + 2.0


def test_annular_snap_split():
    parts = annular_snap(split=True, lead_angle=45.0)
    assert_parts(parts)
    assert parts["ridge"].volume < annular_snap()["ridge"].volume
    with pytest.raises(ValueError):
        annular_snap(lead_angle=60.0)
    with pytest.raises(ValueError):
        annular_snap(ridge_h=2.0, wall=2.0)


# --------------------------------------------------------------- check valve


def test_check_valve_barbed():
    parts = check_valve()
    assert_parts(parts)
    assert set(parts) == {"body", "cap"}
    meta = parts["body"].metadata
    assert meta["ball_d"] == pytest.approx(9.525)
    assert meta["flow_direction"] == "+Z"
    assert meta["barbs"]


def test_check_valve_plain_ports():
    parts = check_valve(barbs=False, ball_d=12.0)
    assert_parts(parts)
    assert not parts["body"].metadata["barbs"]
    with pytest.raises(ValueError):
        check_valve(ball_d=6.0, port_d=6.0, barbs=False)


# ------------------------------------------------------------ suction cup


def test_bellows_suction_cup():
    cup = bellows_suction_cup()
    assert_mesh(cup)
    meta = cup.metadata
    assert meta["folds"] == 2
    assert meta["compressed_h"] == pytest.approx(2 * 2 * 0.8 + 2.0)
    assert meta["cup_volume_mm3"] > 0
    assert meta["barb"]


def test_bellows_suction_cup_plain_stem():
    cup = bellows_suction_cup(barb=False, folds=3, d=30.0)
    assert_mesh(cup)
    assert not cup.metadata["barb"]
    with pytest.raises(ValueError):
        bellows_suction_cup(d=10.0)
    with pytest.raises(ValueError):
        bellows_suction_cup(folds=0)
