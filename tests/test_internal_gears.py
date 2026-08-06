import math

import numpy as np
import pytest
import trimesh
import trimesh.transformations as tf
from shapely import affinity

from mechlib import meshutil
from mechlib.gears import (
    internal_gear_2d,
    internal_mesh_phase,
    mesh_phase,
    ring_gear,
    ring_gear_mesh,
    spur_gear_2d,
)


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def _spin(mesh, deg, center=(0.0, 0.0, 0.0)):
    posed = mesh.copy()
    posed.apply_transform(tf.rotation_matrix(math.radians(deg), (0, 0, 1),
                                             center))
    return posed


def test_internal_profile_inverts_addendum_and_dedendum():
    # The one assertion that catches a ring gear built as an external gear:
    # the teeth point inward, so the tip circle is INSIDE the pitch circle and
    # the root circle is OUTSIDE it.
    z, m = 24, 1.5
    gear = ring_gear(z=z, m=m, width=5.0, z_pinion=16)
    assert_mesh(gear)
    pitch_d = gear.metadata["pitch_d"]
    assert pitch_d == pytest.approx(m * z)
    assert gear.metadata["tip_d"] < pitch_d < gear.metadata["root_d"]
    assert gear.metadata["tip_d"] == pytest.approx(m * z - 2.0 * m)
    assert gear.metadata["root_d"] == pytest.approx(m * z + 2.5 * m)
    assert gear.metadata["outer_d"] > gear.metadata["root_d"]
    assert gear.metadata["z"] == z

    # The extruded solid agrees with the metadata: the bore reaches the tip
    # circle and the outside is the rim circle.
    profile = internal_gear_2d(z=z, m=m, z_pinion=16)
    bore = np.asarray(profile.interiors[0].coords)
    radii = np.hypot(bore[:, 0], bore[:, 1])
    assert radii.min() == pytest.approx(gear.metadata["tip_d"] / 2.0, abs=0.02)
    assert radii.max() == pytest.approx(gear.metadata["root_d"] / 2.0, abs=0.02)
    span = gear.bounds[1] - gear.bounds[0]
    assert span[0] == pytest.approx(gear.metadata["outer_d"], abs=0.05)


def test_ring_gear_cuts_exactly_z_teeth():
    # Same probe as test_pulleys_flexures uses on timing_pulley: an annulus at
    # mid tooth height crosses every tooth and every space.
    z, m = 24, 1.5
    gear = ring_gear(z=z, m=m, width=5.0, z_pinion=16)
    probe_r = (m * z / 2.0) - 0.5 * m          # halfway down the ring addendum
    ring = trimesh.creation.annulus(probe_r - 0.02, probe_r + 0.02, 4.0,
                                    sections=256)
    ring.apply_translation((0, 0, 2.5))
    band = trimesh.boolean.intersection([gear, ring], engine="manifold")
    pieces = [p for p in band.split(only_watertight=False) if p.volume > 1e-4]
    assert len(pieces) == z


def test_ring_and_pinion_run_clear_through_a_full_tooth_pitch():
    # Internal mesh: both members turn the SAME way, ring at z_pinion/z of the
    # pinion. Sweep a full pinion tooth pitch and demand zero interference and
    # a real running clearance at every phase.
    for z, z_pinion in ((24, 16), (30, 12)):
        pair = ring_gear_mesh(z=z, z_pinion=z_pinion, width=5.0, bore_d=4.0)
        assert set(pair) == {"ring", "pinion"}
        for mesh in pair.values():
            assert_mesh(mesh)
        assert pair["ring"].metadata["ratio"] == pytest.approx(
            z / float(z_pinion))
        centre = pair["ring"].metadata["centre_distance"]
        assert centre == pytest.approx(1.5 * (z - z_pinion) / 2.0)

        worst_overlap = 0.0
        worst_gap = 9.9
        for phi in np.linspace(0.0, 360.0 / z_pinion, 5):
            pinion = _spin(pair["pinion"], phi, (centre, 0.0, 0.0))
            ring = _spin(pair["ring"], phi * z_pinion / float(z))
            worst_overlap = max(worst_overlap,
                                meshutil.overlap_volume(ring, pinion))
            worst_gap = min(worst_gap,
                            meshutil.min_distance(ring, pinion, 300))
        assert worst_overlap < 1e-6
        # In mesh, not floating apart: the flank gap is the designed backlash.
        assert 0.15 < worst_gap < 0.45


def test_internal_mesh_phase_lands_a_tooth_in_a_space():
    # It is NOT mesh_phase: the internal contact is seen at the same azimuth
    # from both centres, so the 180 degree term drops out.
    assert internal_mesh_phase(40, 20, 0.0) == pytest.approx(-9.0)
    # Proof by geometry rather than by algebra: place a pinion in a ring at a
    # spread of azimuths with each rule and measure the interference. The
    # internal rule never touches; the external one drives teeth into teeth.
    z, z_pinion, m = 24, 16, 1.5
    profile = internal_gear_2d(z=z, m=m, z_pinion=z_pinion)
    pinion = spur_gear_2d(z_pinion, m, bl=0.35)
    centre = m * (z - z_pinion) / 2.0
    external_rule_fouled = False
    for phi in (0.0, 13.0, 37.0, 90.0, 145.0):
        offset = (centre * math.cos(math.radians(phi)),
                  centre * math.sin(math.radians(phi)))
        good = affinity.translate(
            affinity.rotate(pinion, internal_mesh_phase(z, z_pinion, phi),
                            origin=(0, 0)), *offset)
        assert good.intersection(profile).area < 1e-9
        bad = affinity.translate(
            affinity.rotate(pinion, mesh_phase(z, z_pinion, phi),
                            origin=(0, 0)), *offset)
        external_rule_fouled |= bad.intersection(profile).area > 1.0
    assert external_rule_fouled
    # Phasing at an arbitrary azimuth still meshes: the pair is built at
    # phi=37 degrees and must stay clear.
    pair = ring_gear_mesh(z=24, z_pinion=16, width=4.0, bore_d=4.0,
                          phi_deg=37.0)
    assert meshutil.overlap_volume(pair["ring"], pair["pinion"]) < 1e-6
    assert meshutil.min_distance(pair["ring"], pair["pinion"], 300) < 0.45
    with pytest.raises(ValueError):
        internal_mesh_phase(2, 20, 0.0)


def test_internal_gear_rejects_impossible_geometry():
    # Tip fouling: fewer than 8 teeth of difference cannot be cut as involutes.
    with pytest.raises(ValueError, match="at least 8 teeth of difference"):
        internal_gear_2d(z=24, z_pinion=17)
    with pytest.raises(ValueError, match="at least 8 teeth of difference"):
        ring_gear(z=24, z_pinion=20, width=5.0)
    # A rim thinner than one printable wall cannot hold the teeth.
    with pytest.raises(ValueError, match="printable wall"):
        internal_gear_2d(z=24, z_pinion=16, rim=0.4)
    with pytest.raises(ValueError):
        internal_gear_2d(z=12)
    with pytest.raises(ValueError):
        internal_gear_2d(z=24, z_pinion=16, m=0.0)
    with pytest.raises(ValueError):
        internal_gear_2d(z=24, z_pinion=16, pa_deg=60.0)
    with pytest.raises(ValueError):
        ring_gear(z=24, z_pinion=16, width=0.0)


def test_generated_flank_is_step_count_insensitive():
    # The swept envelope has converged well before the default step count:
    # 10 and 40 samples per ring pitch differ by well under the backlash.
    coarse = internal_gear_2d(z=24, m=1.5, z_pinion=16, steps=10)
    fine = internal_gear_2d(z=24, m=1.5, z_pinion=16, steps=40)
    assert abs(coarse.area - fine.area) / fine.area < 1e-3
    # 0.17 mm^2 spread over 48 flanks is a scallop two orders of magnitude
    # under the 0.33 mm backlash, so the default 18 steps is not the limit.
    assert abs(coarse.area - fine.area) < 0.25
    assert coarse.hausdorff_distance(fine) < 0.05
