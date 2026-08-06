import math

import numpy as np
import pytest
import trimesh

import mechlib
from mechlib.drives import (
    flat_worm,
    planet_stage,
    printed_worm,
    worm_coupon,
    worm_wheel_band,
)


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def test_public_drive_api_and_version():
    assert mechlib.__version__ == "0.6.0"
    names = (
        "printed_worm", "flat_worm", "worm_wheel_band",
        "worm_coupon", "planet_stage",
    )
    assert all(hasattr(mechlib, name) for name in names)
    assert all(name in mechlib.__all__ for name in names)


def test_printed_worm_builds_at_defaults():
    assert_mesh(printed_worm())


def test_flat_worm_builds_at_defaults_and_cache_returns_equal_copies():
    first = flat_worm()
    second = flat_worm()
    assert_mesh(first)
    assert_mesh(second)
    assert first is not second
    assert np.array_equal(first.vertices, second.vertices)
    assert np.array_equal(first.faces, second.faces)


def test_worm_wheel_band_builds_at_defaults():
    assert_mesh(worm_wheel_band())


def test_worm_coupon_builds_printable_default_pair():
    coupon = worm_coupon()
    assert set(coupon) == {"worm", "wheel_band"}
    for mesh in coupon.values():
        assert_mesh(mesh)


def test_planet_stage_builds_named_watertight_default_parts():
    stage = planet_stage()
    assert set(stage) == {"sun", "planets", "ring", "carrier"}
    assert len(stage["planets"]) == 3
    for mesh in [stage["sun"], stage["ring"], stage["carrier"]] + stage["planets"]:
        assert_mesh(mesh)


def test_planet_stage_rejects_inconsistent_ring_tooth_count():
    with pytest.raises(ValueError, match="ring_teeth must equal"):
        planet_stage(ring_teeth=29)


def test_planet_stage_rejects_unassemblable_planet_spacing():
    with pytest.raises(ValueError, match="must be divisible by n_planets"):
        planet_stage(planet_teeth=8, ring_teeth=28)


def test_flat_worm_and_band_defaults_share_source_pitch_and_lead_relation():
    worm_mesh = flat_worm()
    band = worm_wheel_band()
    axial_pitch = worm_mesh.metadata["axial_pitch"]
    lead = worm_mesh.metadata["lead"]
    pitch_circumference = band.metadata["pitch_circumference"]
    starts = worm_mesh.metadata["starts"]
    wheel_teeth = band.metadata["wheel_teeth"]

    assert lead == pytest.approx(starts * axial_pitch)
    assert pitch_circumference == pytest.approx(wheel_teeth * axial_pitch)
    assert pitch_circumference / lead == pytest.approx(wheel_teeth / starts)
    assert lead == pytest.approx(3 * math.pi * 1.5)
