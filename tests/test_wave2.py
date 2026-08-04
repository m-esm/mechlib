import math

import numpy as np
import pytest
import shapely.geometry as sg
import trimesh

from mechlib.closures import (
    SnapSpec,
    clamshell_shiplap,
    fix_pin,
    nut_ac,
    nut_slot,
    press_lid,
    screw_post,
    snap_catch,
    snap_finger,
    ydovetail,
)
from mechlib.cutters import (
    bearing_seat,
    blind_socket,
    chamfer_cutter,
    counterbore,
    countersink,
    crush_ribs,
    dbore,
    dbore_hub,
    gable_roof,
    hex_corner_chamfer,
    slot_neg,
    ss_bore,
    teardrop,
)
from mechlib.fasteners import (
    bolt_mesh,
    fastener_mesh,
    hex_nut_mesh,
    pick_length,
    washer_mesh,
    zmin0,
)
from mechlib.gears import roller_sprocket_2d, spur_gear_mesh
from mechlib.mechanisms import (
    coarse_pitch,
    helix_solid,
    knurl,
    tap,
    thread_solid,
    torsion_spring_mesh,
)
from mechlib.meshutil import (
    bbox_overlap,
    bore_pierces,
    clear,
    cube_rotations,
    decimate,
    export_stl,
    extrude_poly_z,
    fit_transform,
    from_manifold,
    inflate,
    inside,
    inter,
    largest_poly,
    orient,
    overlap_volume,
    self_thickness,
    solid_cube,
    sub,
    to_manifold,
    uni,
    void_cube,
)
from mechlib.patterns import lighten_cell_poly, lighten_grid_centres, polar_ring
from mechlib.prim import boxc, cyl
from mechlib.text import text_polygon


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def test_manifold_conversions_and_direct_csg():
    a = boxc((10, 10, 10))
    b = cyl(2, 12)
    man = to_manifold(a)
    assert_mesh(from_manifold(man))
    cut = sub(a, b)
    joined = uni([a, boxc((2, 2, 2), (6, 0, 0))])
    common = inter(a, b)
    assert_mesh(cut)
    assert_mesh(joined)
    assert_mesh(common)
    assert cut.volume < a.volume


def test_export_stl_and_inflate(tmp_path):
    mesh = boxc((4, 5, 6))
    path = tmp_path / "box.stl"
    export_stl(mesh, path)
    loaded = trimesh.load(path, force="mesh")
    assert_mesh(loaded)
    grown = inflate(mesh, 0.2)
    assert grown.volume > mesh.volume


def test_overlap_helpers_and_cube_probes():
    a = boxc((10, 10, 10))
    b = boxc((4, 4, 4), (4, 0, 0))
    far = boxc((2, 2, 2), (20, 0, 0))
    assert bbox_overlap(a, b)
    assert not bbox_overlap(a, far)
    assert overlap_volume(a, b) > 0
    assert overlap_volume(a, far) == 0
    assert solid_cube(a, (0, 0, 0))
    assert void_cube(a, (20, 0, 0))


def test_point_and_bore_probes():
    blank = boxc((10, 10, 10))
    hole = cyl(1.5, 12, center=(0, 0, 0), axis="x")
    mesh = sub(blank, hole)
    assert inside(blank, [(0, 0, 0), (1, 1, 1)])
    assert clear(mesh, [(0, 0, 0)])
    assert bore_pierces(mesh, (-5, 0, 0), (1, 0, 0), 10)


def test_thickness_decimation_rotation_fit_and_polygon_helpers():
    box = boxc((10, 10, 10))
    p1, minimum, location = self_thickness(box, 40, seed=0)
    assert p1 == pytest.approx(10.0)
    assert minimum == pytest.approx(10.0)
    assert len(location) == 3

    dense = cyl(5, 8, sections=96)
    reduced = decimate(dense, 0.5)
    assert len(reduced.vertices) < len(dense.vertices)
    assert len(cube_rotations()) == 24
    transform = fit_transform(boxc((2, 4, 6)), boxc((2, 4, 6), (8, 3, 1)))
    assert transform.shape == (4, 4)

    rod = cyl(1, 8)
    orient(rod, (1, 0, 0))
    assert np.argmax(rod.extents) == 0
    multi = sg.MultiPolygon([sg.box(0, 0, 2, 2), sg.box(4, 0, 5, 1)])
    extrusion = extrude_poly_z(multi, 2, 5)
    assert_mesh(extrusion)
    assert largest_poly(multi).area == pytest.approx(4.0)


def test_extracted_bore_cutters_are_watertight():
    meshes = [
        teardrop(3, 12, axis="x", up=(0, -1, 0)),
        ss_bore(4, 3.5, 12, (0, 0, 0), axis="x", split_z=10),
        dbore(5.5, 3.7, 8, clear=0.1),
        dbore_hub(6, 8),
        chamfer_cutter(4, 0.4),
        slot_neg(0, 0, 2, 4, 3),
        blind_socket(2, 4, (1, 0, 0), (0, 0, 0)),
        gable_roof(-5, 10, 0, 12, 5, 4),
    ]
    for mesh in meshes:
        assert_mesh(mesh)


def test_hex_chamfer_and_countersink_remove_material():
    base = cyl(13 / math.sqrt(3), 6, center=(0, 0, 3), sections=6)
    chamfered = hex_corner_chamfer(base, 6, 1, 13 / math.sqrt(3))
    sunk = countersink(chamfered, 6, 1, 4.1)
    assert_mesh(chamfered)
    assert_mesh(sunk)
    assert sunk.volume < chamfered.volume < base.volume


def test_new_cutters_have_expected_behavior():
    cb = counterbore(3.4, 6.4, 3.0, 12.0)
    assert_mesh(cb)
    assert cb.extents[2] == pytest.approx(12.0)

    open_seat = bearing_seat("608", fit="press", open_column=True)
    retained = bearing_seat("608", fit="slip", open_column=False, extra_depth=1)
    assert_mesh(open_seat)
    assert_mesh(retained)
    assert open_seat.extents[0] == pytest.approx(22.25)
    assert retained.extents[0] == pytest.approx(22.35)

    ribs = crush_ribs((14, 10, 12), 0.5, 5, 8, count=3, interference=0.1)
    assert_mesh(ribs)
    assert ribs.body_count == 6


def test_press_lid_shiplap_and_dovetail():
    lid = press_lid(32, 26, 28, 22, (0, 0))
    lip, slot = clamshell_shiplap(boxc((32, 26, 12)))
    dovetail = ydovetail(0, -7, 7)
    for mesh in (lid, lip, slot, dovetail):
        assert_mesh(mesh)
    assert slot.volume > lip.volume


def test_snap_pair_uses_snap_spec():
    spec = SnapSpec(aw=7.0)
    catch = snap_catch("x", 10, 0, 1, 12, spec)
    finger = snap_finger("x", 10, 0, 1, 12, spec)
    assert_mesh(catch)
    assert_mesh(finger)
    assert catch.extents[1] == pytest.approx(7.0)


def test_nut_slot_post_and_pin():
    assert nut_ac("M3") == pytest.approx(5.5 * 2 / math.sqrt(3))
    trap = nut_slot((0, 0, 0), nib=True)
    post = screw_post((1, 2, 3), (0, 0, 1), 6)
    pin = fix_pin(2, 5, (1, 0, 0), (0, 0, 0))
    for mesh in (trap, post, pin):
        assert_mesh(mesh)


def test_spur_gear_mesh_is_watertight_and_bore_is_measured():
    gear = spur_gear_mesh(18, 1.5, 5.0, bore_d=5.0)
    assert_mesh(gear)
    outer_r = 1.5 * 18 / 2.0 + 1.5
    assert math.pi * (outer_r * 0.55) ** 2 * 5 < gear.volume
    assert gear.volume < math.pi * outer_r ** 2 * 5
    vertex_r = np.linalg.norm(np.asarray(gear.vertices)[:, :2], axis=1)
    assert vertex_r.min() == pytest.approx(2.5, abs=0.03)


def test_roller_sprocket_profile_is_valid_and_radially_bounded():
    n, pitch, pin_d, clear = 14, 10.0, 2.0, 0.275
    outer_d = 47.30
    poly = roller_sprocket_2d(n, pitch, pin_d, clear, outer_d)
    assert isinstance(poly, sg.Polygon)
    assert poly.is_valid and poly.area > 0
    radii = [math.hypot(x, y) for x, y in poly.exterior.coords]
    pitch_r = pitch / (2 * math.sin(math.pi / n))
    assert max(radii) <= outer_d / 2 + 0.03
    assert min(radii) < pitch_r
    assert min(radii) > pitch_r - pin_d / 2 - clear - 0.1


def test_coarse_pitch_and_helix_solid():
    assert coarse_pitch(8) == 1.25
    profile = np.array([(2.0, -0.4), (3.0, -0.2), (3.0, 0.2), (2.0, 0.4)])
    helix = helix_solid(profile, lead=1.0, turns=2.0, seg=24)
    assert_mesh(helix)


def test_m8_external_and_internal_threads_are_watertight():
    external = thread_solid(8, 8, seg=40)
    assert_mesh(external)

    blank = cyl(7, 10)
    blank.apply_translation((0, 0, 5))
    tapped = tap(blank, 8, (0, 0, 0), 10)
    assert_mesh(tapped)
    assert tapped.volume < blank.volume


def test_knurl_and_torsion_spring_mesh():
    head = cyl(6, 4)
    head.apply_translation((0, 0, 2))
    fluted = knurl(head, 6, 0, 4, n=12)
    spring = torsion_spring_mesh(turns=2)
    assert_mesh(fluted)
    assert_mesh(spring)
    assert fluted.volume < head.volume


def test_basic_hardware_standins_and_zmin0():
    moved = cyl(1, 2, center=(0, 0, -4))
    assert zmin0(moved).bounds[0][2] == pytest.approx(0)
    for mesh in (
        bolt_mesh(1.5, 10, 3, 2),
        hex_nut_mesh(5.5, 2.6, 3.0),
        washer_mesh(7.0, 3.4, 1.0),
    ):
        assert_mesh(mesh)


@pytest.mark.parametrize("style", ["pan", "shcs", "csk"])
def test_oriented_fastener_styles(style):
    screw = fastener_mesh(3, 12, style=style, axis="x", at=(1, 2, 3))
    assert_mesh(screw)
    assert screw.extents[0] >= 12


def test_pick_length_uses_source_standard_series():
    assert pick_length(10.4) == 10
    assert pick_length(13.0) == 16


def test_patterns_return_deterministic_valid_geometry():
    ring = polar_ring(4, 2, center=(1, -1), phase=math.pi / 2)
    assert ring[0] == pytest.approx((1, 1))
    rect = lighten_cell_poly(0, 0, 3, "rect")
    hexa = lighten_cell_poly(0, 0, 3, "hex")
    assert rect.is_valid and hexa.is_valid
    centres = list(lighten_grid_centres(0, 0, 6, 6, 2, 1, "hex"))
    assert len(centres) >= 4


def test_text_polygon_preserves_valid_letter_geometry(tmp_path, monkeypatch):
    monkeypatch.setenv("MPLCONFIGDIR", str(tmp_path))
    poly = text_polygon("mechlib", 12)
    assert poly is not None
    assert poly.is_valid and poly.area > 0
