"""Exact-constraint fixture tests: the contact set IS the deliverable."""

import math

import numpy as np
import pytest
import trimesh
import trimesh.transformations as tf

from mechlib.fixtures import (
    kinematic_coupling,
    repeatable_dock,
    three_point_leveller,
)
from mechlib.mechanisms import coarse_pitch
from mechlib.patterns import polar_ring
from mechlib.meshutil import (
    bore_pierces,
    clear,
    inside,
    min_distance,
    overlap_volume,
)


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def surface_distance(mesh, points):
    """Unsigned distance from each probe point to a mesh surface."""
    pts = np.atleast_2d(np.asarray(points, float))
    return trimesh.proximity.ProximityQuery(mesh).on_surface(pts)[1]


# --- kinematic_coupling ------------------------------------------------------

def test_coupling_parts_are_watertight_and_named():
    for kind in ("maxwell", "kelvin"):
        hw = kinematic_coupling(kind=kind, ball="hardware")
        assert set(hw) == {"base", "top", "balls"}
        for mesh in hw.values():
            assert_mesh(mesh)
        # Bought balls are three separate bodies, not a printed part.
        assert len(hw["balls"].split(only_watertight=False)) == 3
        pr = kinematic_coupling(kind=kind, ball="printed")
        assert set(pr) == {"base", "top"}
        for mesh in pr.values():
            assert_mesh(mesh)
        # The printed bosses fuse into one rigid top plate.
        assert len(pr["top"].split(only_watertight=False)) == 1
        assert pr["top"].volume > hw["top"].volume


def test_coupling_constrains_exactly_six_points():
    for kind in ("maxwell", "kelvin"):
        parts = kinematic_coupling(kind=kind, ball="hardware")
        meta = parts["base"].metadata
        assert meta["dof_constrained"] == 6
        assert len(meta["contacts"]) == 6
        # Every part of the assembly carries the same contact claim.
        assert parts["top"].metadata["contacts"] == meta["contacts"]


def test_contact_points_lie_on_both_the_balls_and_the_seats():
    for kind in ("maxwell", "kelvin"):
        parts = kinematic_coupling(kind=kind, ball="hardware")
        base, balls = parts["base"], parts["balls"]
        contacts = np.asarray(parts["base"].metadata["contacts"], float)
        centres = np.asarray(parts["base"].metadata["ball_centers"], float)
        ball_r = parts["base"].metadata["ball_d"] / 2.0
        # A contact point touches the seat and the ball at the same time.
        assert surface_distance(base, contacts).max() < 1e-3
        assert surface_distance(balls, contacts).max() < 0.02
        # Every contact is exactly one ball radius from its ball centre.
        for point in contacts:
            radius = np.linalg.norm(centres - point, axis=1).min()
            assert radius == pytest.approx(ball_r, abs=1e-6)
        # Roll 20 degrees off each contact along the ball surface and the
        # ball pulls away from the seat: the touching is pointwise, not a band.
        for point in contacts:
            centre = centres[np.linalg.norm(centres - point, axis=1).argmin()]
            normal = (point - centre) / ball_r
            axis = np.cross(normal, (0.0, 0.0, 1.0))
            if np.linalg.norm(axis) < 1e-6:
                axis = np.array([1.0, 0.0, 0.0])
            rot = tf.rotation_matrix(math.radians(20.0),
                                     axis / np.linalg.norm(axis), centre)
            off = tf.transform_points([point], rot)[0]
            assert surface_distance(base, [off])[0] > 0.1


def test_seated_coupling_touches_without_interference():
    for kind in ("maxwell", "kelvin"):
        parts = kinematic_coupling(kind=kind, ball="printed")
        base, top = parts["base"], parts["top"]
        # Seated: the balls sit in the seats with no material interpenetration.
        assert overlap_volume(top, base) < 1e-3
        # ... and they really are touching, not floating above the seats: every
        # contact point lies on both surfaces at once. (Probing the metadata
        # points is deterministic; min_distance samples randomly and would flake.)
        contacts = np.asarray(base.metadata["contacts"], float)
        assert surface_distance(base, contacts).max() < 1e-3
        assert surface_distance(top, contacts).max() < 0.02
        # Push the top plate 0.25 mm further down and it must bite.
        pushed = top.copy()
        pushed.apply_translation((0.0, 0.0, -0.25))
        assert overlap_volume(pushed, base) > 0.05


def test_coupling_constrains_all_six_degrees_of_freedom():
    for kind in ("maxwell", "kelvin"):
        parts = kinematic_coupling(kind=kind, ball="printed")
        base, top = parts["base"], parts["top"]
        moves = {
            "x": tf.translation_matrix((0.25, 0.0, 0.0)),
            "y": tf.translation_matrix((0.0, 0.25, 0.0)),
            "z": tf.translation_matrix((0.0, 0.0, -0.25)),
            "rx": tf.rotation_matrix(math.radians(1.0), (1, 0, 0)),
            "ry": tf.rotation_matrix(math.radians(1.0), (0, 1, 0)),
            "rz": tf.rotation_matrix(math.radians(1.0), (0, 0, 1)),
        }
        for dof, matrix in moves.items():
            moved = top.copy()
            moved.apply_transform(matrix)
            assert overlap_volume(moved, base) > 0.05, (
                "%s coupling leaves %s free: it is not exactly constrained"
                % (kind, dof))
        # Sanity on the probe itself: lifting off is the one free direction,
        # which is why a real coupling needs preload (see repeatable_dock).
        lifted = top.copy()
        lifted.apply_translation((0.0, 0.0, 0.4))
        assert overlap_volume(lifted, base) == 0.0


def test_maxwell_and_kelvin_are_different_geometry():
    maxwell = kinematic_coupling(kind="maxwell", ball="hardware")
    kelvin = kinematic_coupling(kind="kelvin", ball="hardware")
    assert maxwell["base"].volume != pytest.approx(kelvin["base"].volume)
    # Maxwell: three vee grooves, so all six contacts share one height.
    mz = [p[2] for p in maxwell["base"].metadata["contacts"]]
    assert mz == pytest.approx([mz[0]] * 6)
    # Kelvin: the single flat seat contacts a whole ball radius lower.
    kz = sorted(p[2] for p in kelvin["base"].metadata["contacts"])
    assert kz[0] < kz[1] - 0.5
    assert kz[1:] == pytest.approx([kz[-1]] * 5)
    # Both plates seat at the same height, so the couplings are interchangeable.
    assert (maxwell["base"].metadata["seat_gap"]
            == pytest.approx(kelvin["base"].metadata["seat_gap"]))


def test_coupling_geometry_scales_with_ball_and_circle():
    parts = kinematic_coupling(pcd=30.0, ball_d=8.0, plate_t=8.0)
    meta = parts["base"].metadata
    assert_mesh(parts["base"])
    radii = [math.hypot(x, y) for x, y, _z in meta["ball_centers"]]
    assert radii == pytest.approx([15.0] * 3)
    # Ball centre height is the closed form for a 90 degree vee: r*sqrt(2)
    # above the groove apex, which sits groove_depth below the seating face.
    assert meta["ball_center_z"] == pytest.approx(
        -meta["groove_depth"] + 4.0 * math.sqrt(2.0))


def test_coupling_rejects_bad_arguments():
    with pytest.raises(ValueError):
        kinematic_coupling(kind="hertz")
    with pytest.raises(ValueError):
        kinematic_coupling(ball="glued")
    with pytest.raises(ValueError):
        # Seats overlap each other on too small a circle.
        kinematic_coupling(pcd=12.0, ball_d=6.0)
    with pytest.raises(ValueError):
        # Groove too shallow: the contact band would be above the plate face.
        kinematic_coupling(groove_depth=1.0)
    with pytest.raises(ValueError):
        # Groove deeper than the plate.
        kinematic_coupling(groove_depth=5.0, plate_t=6.0)
    with pytest.raises(ValueError):
        # A socket that deep would swallow the ball whole.
        kinematic_coupling(ball="hardware", socket_depth=3.0, ball_d=6.0)
    with pytest.raises(ValueError):
        kinematic_coupling(plate_d=30.0)


# --- repeatable_dock ---------------------------------------------------------

def test_dock_magnet_preload_holds_without_touching():
    parts = repeatable_dock(preload="magnet", ball="printed")
    assert set(parts) == {"base", "top"}
    for mesh in parts.values():
        assert_mesh(mesh)
    meta = parts["base"].metadata
    assert meta["preload"] == "magnet"
    assert meta["dof_constrained"] == 6
    assert len(meta["contacts"]) == 6
    # The preload boss reaches down toward the base but stops short of it:
    # a touching pad would be a seventh contact and kill the exact constraint.
    gap = meta["magnet_air_gap"]
    wall = (meta["magnet_d"] / 2.0 + 0.8, 0.0)
    assert inside(parts["top"], [(wall[0], wall[1], gap + 0.2)])
    assert clear(parts["top"], [(wall[0], wall[1], gap - 0.15)])
    assert overlap_volume(parts["top"], parts["base"]) < 1e-3
    # The magnet pocket is a real void in the base seating face.
    assert bore_pierces(parts["base"], (0.0, 0.0, -0.2), (0, 0, -1),
                        meta["magnet_t"] - 0.2, n=6)
    # ... and it stays blind, so the magnet cannot fall through.
    assert not bore_pierces(parts["base"], (0.0, 0.0, -0.2), (0, 0, -1),
                            meta["plate_t"] - 0.3, n=12)


def test_dock_screw_preload_and_bolt_circle():
    parts = repeatable_dock(preload="screw", ball="hardware", n_bolt=3)
    assert set(parts) == {"base", "top", "balls"}
    for mesh in parts.values():
        assert_mesh(mesh)
    meta = parts["base"].metadata
    assert meta["preload"] == "screw"
    assert meta["n_bolt"] == 3
    # The tapped hole runs down into the base from the seating face.
    assert bore_pierces(parts["base"], (0.0, 0.0, -0.3), (0, 0, -1), 3.0, n=8)
    # Every mounting hole is a clean through hole in both plates.
    for x, y in polar_ring(3, meta["bolt_pcd"] / 2.0,
                           phase=math.radians(90.0 + 60.0)):
        assert bore_pierces(parts["base"], (x, y, 0.5), (0, 0, -1),
                            meta["plate_t"] + 1.0, n=12)
        assert bore_pierces(parts["top"], (x, y, meta["seat_gap"] - 0.5),
                            (0, 0, 1), meta["plate_t"] + 1.0, n=12)


def test_dock_keeps_the_coupling_exactly_constrained():
    parts = repeatable_dock(preload="magnet", ball="printed")
    base, top = parts["base"], parts["top"]
    for matrix in (tf.translation_matrix((0.25, 0.0, 0.0)),
                   tf.translation_matrix((0.0, 0.0, -0.25)),
                   tf.rotation_matrix(math.radians(1.0), (0, 0, 1))):
        moved = top.copy()
        moved.apply_transform(matrix)
        assert overlap_volume(moved, base) > 0.05


def test_dock_rejects_bad_arguments():
    with pytest.raises(ValueError):
        repeatable_dock(preload="velcro")
    with pytest.raises(ValueError):
        # A magnet that wide would eat the seats.
        repeatable_dock(preload="magnet", magnet_d=30.0)
    with pytest.raises(ValueError):
        # A touching preload pad is a seventh contact.
        repeatable_dock(preload="magnet", magnet_gap=0.0)
    with pytest.raises(ValueError):
        # Six holes on the default circle put three of them in the seats.
        repeatable_dock(n_bolt=6)
    with pytest.raises(ValueError):
        repeatable_dock(bolt_pcd=200.0)


# --- three_point_leveller ----------------------------------------------------

def test_leveller_parts_and_adjustment_metadata():
    parts = three_point_leveller()
    assert set(parts) == {"base", "table", "screws"}
    for mesh in parts.values():
        assert_mesh(mesh)
    assert len(parts["screws"].split(only_watertight=False)) == 3
    meta = parts["base"].metadata
    assert meta["dof_constrained"] == 6
    assert len(meta["contacts"]) == 6
    # One turn of one screw raises its corner by exactly one thread pitch.
    assert meta["mm_per_turn"] == pytest.approx(coarse_pitch(6.0))
    # ... and tilts the table about the line through the other two screws,
    # which for an equilateral triad sits 1.5 screw-circle radii away.
    assert meta["tilt_per_turn_deg"] == pytest.approx(
        math.degrees(math.atan2(coarse_pitch(6.0), 1.5 * 44.0 / 2.0)))


def test_leveller_screw_tips_seat_and_threads_clear():
    parts = three_point_leveller()
    base, table, screws = parts["base"], parts["table"], parts["screws"]
    # Tips rest in their seats: touching, no interference.
    assert overlap_volume(screws, base) < 1e-3
    # The printed thread turns freely in the tapped table: no interference,
    # yet the crest stays within a fraction of a millimetre of the tapped
    # flank rather than rattling in an oversized bore.
    assert overlap_volume(screws, table) < 1e-3
    assert min_distance(screws, table, n=6000) < 0.6
    # The table floats on the screws alone.
    assert overlap_volume(table, base) == 0.0
    contacts = np.asarray(base.metadata["contacts"], float)
    assert surface_distance(base, contacts).max() < 1e-3
    assert surface_distance(screws, contacts).max() < 0.02


def test_leveller_rejects_bad_arguments():
    with pytest.raises(ValueError):
        three_point_leveller(kind="tripod")
    with pytest.raises(ValueError):
        three_point_leveller(tip_ratio=1.4)
    with pytest.raises(ValueError):
        # Screw circle far too tight for the tip seats.
        three_point_leveller(screw_pcd=8.0)
    with pytest.raises(ValueError):
        # Screw cannot reach through the table at that lift.
        three_point_leveller(screw_len=6.0, lift=12.0)
    with pytest.raises(ValueError):
        three_point_leveller(base_t=3.5, screw_d=8.0)
    with pytest.raises(ValueError):
        # No printable coarse pitch for an M7.
        three_point_leveller(screw_d=7.0)
