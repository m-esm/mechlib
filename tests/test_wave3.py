import math

import numpy as np
import pytest
import shapely.geometry as sg
import trimesh

from mechlib import (
    approach_clear,
    audit,
    chamfer_prism,
    directed_holes,
    export_assembly,
    extrude_snapped,
    lobe_cavity_polys,
    loft,
    min_distance,
    orient,
    pack_by_category,
    place,
    place_right,
    push_pin,
    revolved_gable_cavity,
    ring_pts,
    saddle,
    seg_cylinder,
    setscrew,
    shelf_pack,
    slicer_area,
    slot_cutter,
    spur_gear,
    tapered_cavity,
    text_block,
    threaded_rod,
    u_channel_between,
    worm,
)


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


def test_worm_and_matching_helical_wheel_have_positive_clearance():
    module, teeth, pitch_d = 1.5, 40, 14.3
    worm_mesh, lead_angle = worm(module, 20, pitch_d)
    worm_mesh.apply_transform(trimesh.transformations.rotation_matrix(
        math.pi / 2, [1, 0, 0]))
    center_distance = (module * teeth + pitch_d) / 2.0
    worm_mesh.apply_translation([0, 0, -center_distance])

    wheel = spur_gear(module, teeth, 8, backlash=0.35,
                      helix_deg=lead_angle)
    wheel.apply_transform(trimesh.transformations.rotation_matrix(
        math.radians(3.0), [0, 0, 1]))
    wheel.apply_transform(trimesh.transformations.rotation_matrix(
        math.pi / 2, [0, 1, 0]))

    assert_mesh(worm_mesh)
    assert_mesh(wheel)
    overlap = trimesh.boolean.intersection([worm_mesh, wheel], engine="manifold")
    volume = 0.0 if overlap is None or overlap.is_empty else abs(overlap.volume)
    assert volume < 1.0
    assert min_distance(worm_mesh, wheel, n=1500) > 0.01


def test_spur_gear_sector_is_a_watertight_partial_wheel():
    sector = spur_gear(1.25, 32, 6, bore=4, sector_deg=120,
                       hub_d=12, full_disc=False)
    assert_mesh(sector)
    assert sector.extents[1] < 2 * (1.25 * 32 / 2 + 1.25)


@pytest.mark.parametrize("sector_deg", [90.0, 95.0, 100.0, 110.0, 120.0,
                                        125.0, 130.0, 140.0, 145.0, 150.0])
def test_spur_gear_sector_is_a_volume_at_every_sector_angle(sector_deg):
    # Sector half-angles that are an exact multiple of the back-disc sampling
    # step (2 deg) used to leave a zero-length edge in the profile, so the
    # extrusion was not a volume and the hub union raised
    # "Not all meshes are volumes!".
    gear = spur_gear(1.5, 36, 7.0, bore=5.0, sector_deg=sector_deg,
                     hub_d=14.0, full_disc=False)
    assert_mesh(gear)
    assert gear.is_volume


def test_spur_gear_full_disc_geometry_is_unchanged():
    gear = spur_gear(1.5, 36, 7.0, bore=5.0, hub_d=14.0)
    assert_mesh(gear)
    assert gear.volume == pytest.approx(15676.392425, rel=5e-3)


def test_ring_resampling_and_loft_make_a_watertight_solid():
    profiles = [
        (sg.Point(0, 0).buffer(5, resolution=12), 0),
        (sg.Point(1, 0).buffer(7, resolution=12), 6),
        (sg.Point(0, 1).buffer(4, resolution=12), 12),
    ]
    rings = [ring_pts(poly, 48, z) for poly, z in profiles]
    assert all(ring.shape == (48, 3) for ring in rings)
    assert_mesh(loft(rings))


def test_step_export_writes_iso_header(tmp_path):
    pytest.importorskip("OCP")
    parts = {
        "left": trimesh.creation.box((2, 3, 4)),
        "right": trimesh.creation.box((3, 2, 1)),
    }
    parts["right"].apply_translation((5, 0, 0))
    path = tmp_path / "assembly.step"
    assert export_assembly(parts, path) == path
    assert path.exists()
    assert path.read_text(encoding="ascii", errors="ignore").startswith("ISO-10303")


def test_shelf_pack_and_category_pack_use_multiple_nonoverlapping_plates():
    sized = []
    for i in range(10):
        mesh = trimesh.creation.box((40, 40, 4))
        sized.append(dict(name="part_%d" % i, mesh=mesh, fw=40.0, fd=40.0,
                          obj={}))
    plates = shelf_pack(sized, bed=(100, 100), gap=5)
    assert len(plates) > 1
    for plate in plates:
        for i, left in enumerate(plate):
            lx, ly = left["pos"]
            for right in plate[i + 1:]:
                rx, ry = right["pos"]
                separated = (abs(lx - rx) >= (left["fw"] + right["fw"]) / 2
                             or abs(ly - ry) >= (left["fd"] + right["fd"]) / 2)
                assert separated

    items = [("part_%d.stl" % i, row["mesh"], None, {})
             for i, row in enumerate(sized)]
    grouped = pack_by_category(items, bed=(100, 100), gap=5,
                               categories={"part_0": "Special"},
                               category_order=("Special",))
    assert grouped[0][0] == "Special"
    assert sum(len(parts) for _, parts in grouped) == 10


def test_setscrew_and_push_pin_are_oriented_printable_features():
    boss, hole = setscrew((1, 2, 3), (0, 0, -1))
    for mesh in (boss, hole, push_pin(4, 12, axis="x", flip=True)):
        assert_mesh(mesh)
    assert boss.bounds[0, 2] == pytest.approx(3.0)


def test_orient_handles_negative_z_degeneracy():
    pin = trimesh.creation.cylinder(radius=1, height=8)
    pin.apply_translation((0, 0, 4))
    orient(pin, (0, 0, -1))
    assert pin.bounds[:, 2] == pytest.approx([-8, 0])


def test_slot_taper_lobe_and_u_channel_cutters():
    slot = slot_cutter(12, 3, 0, 6)
    assert len(slot) == 2
    assert all(part.is_watertight for part in slot)
    assert slot[0].extents[0] > 12

    cavity_poly = sg.Point(0, 0).buffer(5, resolution=24)
    taper = tapered_cavity(cavity_poly, 0, 15, taper_h=11, taper_step=0.6)
    assert taper and all(part.is_watertight for part in taper)
    combined = trimesh.util.concatenate(taper)
    assert combined.section(plane_origin=[0, 0, 14.9],
                            plane_normal=[0, 0, 1]) is None

    class Section:
        polygons_full = [sg.Point(0, 0).buffer(7, resolution=24)]

    lobes = lobe_cavity_polys(Section(), wall=1.2, rib_w=1.6, n_rib=2)
    assert len(lobes) == 1 and lobes[0].area > 0
    channel = u_channel_between((0, 0), (12, 7), 3, 1, 9)
    assert len(channel) == 2 and all(part.is_watertight for part in channel)


def test_revolved_gable_cavity_has_zero_area_at_roof_peak():
    cavity = revolved_gable_cavity(8, 18, 2, 12, roof_angle=45, sections=64)
    assert_mesh(cavity)
    assert cavity.bounds[1, 2] == pytest.approx(14)
    assert cavity.section(plane_origin=[0, 0, 14.01],
                          plane_normal=[0, 0, 1]) is None


def test_audit_distance_and_approach_checks():
    touching_a = trimesh.creation.box((10, 10, 10))
    touching_b = trimesh.creation.box((10, 10, 10))
    touching_b.apply_translation((10, 0, 0))
    far_a = touching_a.copy(); far_a.apply_translation((40, 0, 0))
    far_b = touching_a.copy(); far_b.apply_translation((60, 0, 0))
    failures, warnings = audit(
        {"touch_a": touching_a, "touch_b": touching_b,
         "far_a": far_a, "far_b": far_b},
        clearance=0.3, allow=set())
    assert failures == []
    assert any("TIGHT" in warning for warning in warnings)
    assert min_distance(touching_a, touching_b, n=500) < 0.1
    assert approach_clear(touching_a, (5, 0, 0), (1, 0, 0), 20) == 20
    assert approach_clear(touching_a, (0, 0, 0), (1, 0, 0), 20) < 1


def test_slicer_area_and_precision_snapped_extrusion():
    poly = sg.box(0, 0, 10, 10)
    area = slicer_area([poly], infill=0.15, line_w=0.42, perimeters=3)
    assert 0 < area < poly.area
    snapped = extrude_snapped(poly.union(sg.Point(10, 5).buffer(2)), 2, 5)
    assert snapped and all(part.is_watertight for part in snapped)


def test_threaded_rod_is_watertight_and_reaches_major_diameter():
    rod = threaded_rod(8, 1.25, 14)
    assert_mesh(rod)
    od = max(rod.extents[:2])
    assert od == pytest.approx(8, rel=0.02)


def test_chamfer_prism_and_skew_segment_geometry():
    prism = chamfer_prism(24, 16, 10, 4, 1.5)
    segment = seg_cylinder((-3, 1, 2), (6, 5, 12), 2)
    assert_mesh(prism)
    assert_mesh(segment)
    assert prism.extents == pytest.approx([24, 16, 10])


def test_directed_holes_union_follows_vectors():
    holes = directed_holes([(0, 0, 0), (4, 0, 0)],
                           [(0, 0, 1), (0, 1, 1)], 2, 8)
    assert_mesh(holes)
    assert holes.bounds[1, 2] > 5
    assert holes.bounds[1, 1] > 5


def test_saddle_cradles_a_cylinder_and_text_helpers_align(monkeypatch, tmp_path):
    monkeypatch.setenv("MPLCONFIGDIR", str(tmp_path / "mpl"))
    interior = trimesh.creation.box((32, 24, 22))
    cradle = saddle((-10, -10, 2), (10, 10, 8), 3, 0, 2, interior)
    assert_mesh(cradle)
    assert cradle.extents[1] == pytest.approx(2)

    lines = text_block(["MECH", "LIB"], 0, 0, 6, gap=1.2)
    assert len(lines) == 2
    centered = place(lines[0], 3, 4)
    right = place_right(lines[1], 20, 4)
    assert (centered.bounds[0] + centered.bounds[2]) / 2 == pytest.approx(3)
    assert right.bounds[2] == pytest.approx(20)
