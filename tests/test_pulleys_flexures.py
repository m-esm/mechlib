import math

import numpy as np
import pytest
import trimesh

from mechlib.flexures import bistable_beam, cross_flexure, wave_spring
from mechlib.pulleys import grooved_drum, timing_pulley


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def test_timing_pulley_pitch_diameter_matches_closed_form():
    for teeth, pitch in ((20, 2.0), (30, 2.0), (16, 3.0)):
        pulley = timing_pulley(teeth=teeth, pitch=pitch)
        assert_mesh(pulley)
        assert pulley.metadata["pitch_d"] == pytest.approx(
            teeth * pitch / math.pi)
        assert pulley.metadata["tip_d"] < pulley.metadata["pitch_d"]


def test_timing_pulley_tooth_count_and_belt_clearance():
    teeth = 24
    pulley = timing_pulley(teeth=teeth, flanges=False, hub_len=0.0)
    assert_mesh(pulley)
    pitch_r = teeth * 2.0 / (2.0 * math.pi)
    # A probe ring midway between tip and root crosses every tooth and space.
    probe_r = pitch_r - 0.45
    ring = trimesh.creation.annulus(probe_r - 0.02, probe_r + 0.02, 8.0,
                                    sections=128)
    ring.apply_translation((0, 0, 2.0))
    band = trimesh.boolean.intersection([pulley, ring], engine="manifold")
    pieces = band.split(only_watertight=False)
    assert len(pieces) == teeth
    # The printed bore is opened by the clearance.
    bore_probe = trimesh.creation.cylinder(
        radius=(5.0 + 0.25) / 2.0 - 0.05, height=30.0, sections=48)
    overlap = trimesh.boolean.intersection([pulley, bore_probe],
                                           engine="manifold")
    assert overlap is None or overlap.is_empty or abs(overlap.volume) < 1e-6


def test_timing_pulley_options_change_geometry_and_stay_watertight():
    plain = timing_pulley(flanges=False, hub_len=0.0)
    full = timing_pulley(flanges=True, setscrew_boss=True)
    assert_mesh(plain)
    assert_mesh(full)
    assert full.volume > plain.volume
    # Setscrew boss pierces a radial hole into the hub.
    assert full.bounds[1][0] > timing_pulley().bounds[1][0] - 1e-9


def test_grooved_drum_height_and_radius_law_metadata():
    for law in ("cylinder", "cone", "fusee"):
        drum = grooved_drum(radius_law=law, turns=6, cable_d=3.0)
        assert_mesh(drum)
        pitch = drum.metadata["groove_pitch"]
        assert pitch == pytest.approx(3.0 * 1.25)
        assert drum.metadata["body_height"] == pytest.approx(6 * pitch)
        z_span = drum.bounds[1][2] - drum.bounds[0][2]
        assert z_span == pytest.approx(6 * pitch + 2 * 1.6)
    assert grooved_drum(radius_law="cylinder").metadata["r_end"] == 10.0


def test_grooved_drum_radius_laws_are_monotonic():
    from mechlib.pulleys import _winding_radius
    for law in ("cone", "fusee"):
        radii = [_winding_radius(law, f / 20.0, 10.0, 6.0)
                 for f in range(21)]
        assert all(b >= a for a, b in zip(radii, radii[1:]))
        assert radii[0] == pytest.approx(10.0)
        assert radii[-1] == pytest.approx(16.0)
    flat = [_winding_radius("cylinder", f / 10.0, 10.0, 6.0)
            for f in range(11)]
    assert flat == pytest.approx([10.0] * 11)


def test_grooved_drum_rejects_bad_radius_law_and_thin_wall():
    with pytest.raises(ValueError):
        grooved_drum(radius_law="spiral")
    with pytest.raises(ValueError):
        grooved_drum(core_r=5.0, bore_d=8.0)


def test_cross_flexure_is_a_single_watertight_part():
    flex = cross_flexure()
    assert_mesh(flex)
    assert len(flex.split(only_watertight=False)) == 1
    # Blocks sandwich the blade gap: full height is gap plus two block_h.
    z_span = flex.bounds[1][2] - flex.bounds[0][2]
    assert z_span == pytest.approx(6.0 + 10.0 + 6.0)


def test_cross_flexure_blades_clear_each_other_and_land_on_blocks():
    flex = cross_flexure(blade_angle_deg=30.0, blade_w=5.0, blade_gap=1.2)
    assert_mesh(flex)
    # Mid-gap slab contains only the two blades: two disjoint solid slices.
    slab = trimesh.creation.box(extents=(40.0, 30.0, 0.4))
    slab.apply_translation((0, 0, 5.0))
    slice_mesh = trimesh.boolean.intersection([flex, slab], engine="manifold")
    pieces = [p for p in slice_mesh.split(only_watertight=False)
              if p.volume > 1e-3]
    assert len(pieces) == 2
    with pytest.raises(ValueError):
        cross_flexure(blade_angle_deg=70.0, gap=14.0)


def test_wave_spring_single_and_multi_turn_geometry():
    washer = wave_spring(turns=1)
    assert_mesh(washer)
    assert len(washer.split(only_watertight=False)) == 1
    assert washer.bounds[1][2] - washer.bounds[0][2] == pytest.approx(
        2 * 1.0 + 0.8)
    spring = wave_spring(turns=3)
    assert_mesh(spring)
    # Crest-fused turns form one body spanning two turn pitches.
    assert len(spring.split(only_watertight=False)) == 1
    pitch = 2 * 1.0 + 0.8 - 0.15
    assert spring.bounds[1][2] - spring.bounds[0][2] == pytest.approx(
        2 * pitch + 2 * 1.0 + 0.8)
    # Unfused turns remain a stack of separate washers.
    stack = wave_spring(turns=3, crest_fuse=0.0)
    assert len(stack.split(only_watertight=False)) == 3


def test_wave_spring_wave_count_modulates_rim():
    waves = 4
    spring = wave_spring(waves=waves, turns=1, sections=96)
    assert_mesh(spring)
    # The top-ring vertices (every 4th, ordered by theta) oscillate with
    # exactly ``waves`` full periods per revolution: 2*waves slope reversals.
    top_z = spring.vertices[2::4, 2]
    slope = np.sign(np.diff(np.r_[top_z, top_z[0]]))
    slope = slope[slope != 0]
    reversals = int(np.sum(slope != np.roll(slope, 1)))
    assert reversals == 2 * waves


def test_bistable_beam_is_flat_single_part_with_clear_travel():
    beam = bistable_beam()
    assert_mesh(beam)
    assert len(beam.split(only_watertight=False)) == 1
    assert beam.bounds[1][2] - beam.bounds[0][2] == pytest.approx(3.0)
    # The frame side rails keep clear of the shuttle travel band: a probe
    # slab around the shuttle y-range at mid-x only meets beams + shuttle.
    assert beam.bounds[1][0] == pytest.approx(44.0)
    assert beam.bounds[1][1] == pytest.approx(12.0)


def test_bistable_beam_parameter_validation():
    with pytest.raises(ValueError):
        bistable_beam(apex=5.0, beam_gap=8.0)
    with pytest.raises(ValueError):
        bistable_beam(wall=1.0)
    quad = bistable_beam(beams=4)
    assert_mesh(quad)
    assert len(quad.split(only_watertight=False)) == 1
