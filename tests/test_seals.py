import math

import pytest
import shapely.geometry as sg
import trimesh

from mechlib.cutters import gasket_channel, labyrinth_seal, oring_groove
from mechlib.meshutil import inter, min_distance, overlap_volume, sub
from mechlib.prim import boxc, cyl


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


# ---------------------------------------------------------------------------
# oring_groove
# ---------------------------------------------------------------------------


def test_oring_groove_face_mode_matches_gland_relations():
    cs = 2.62
    squeeze = 0.20
    fill = 0.78
    groove = oring_groove(face_pcd=40.0, cs=cs, squeeze=squeeze, fill=fill,
                          mode="face")
    assert_mesh(groove)
    gw = groove.metadata["groove_width"]
    gd = groove.metadata["groove_depth"]

    # Closed-form depth from the standard gland relation.
    assert gd == pytest.approx(cs * (1.0 - squeeze))

    # Independent hand computation of ring area vs groove area, not a call
    # back into the function's own math.
    ring_area = math.pi * (cs / 2.0) ** 2
    groove_area = gw * gd
    hand_fill = ring_area / groove_area
    assert hand_fill == pytest.approx(fill, rel=1e-6)
    assert groove.metadata["gland_fill_pct"] == pytest.approx(hand_fill * 100.0)
    assert groove.metadata["squeeze_pct"] == pytest.approx(squeeze * 100.0)


def test_oring_groove_face_cut_cavity_matches_metadata():
    groove = oring_groove(face_pcd=40.0, cs=2.62, squeeze=0.20, mode="face", z0=0.0)
    gw = groove.metadata["groove_width"]
    gd = groove.metadata["groove_depth"]
    pcd = 40.0

    slab = boxc((70.0, 70.0, 10.0), center=(0.0, 0.0, -5.0))
    cut = sub(slab, groove)
    assert_mesh(cut)

    # Recover exactly the material the groove removed, then isolate one
    # radial slice of it so bounding-box extents read as width and depth.
    removed = sub(slab, cut)
    probe = boxc((gw + 6.0, 0.8, 10.0), center=(pcd / 2.0, 0.0, -5.0))
    sliver = inter(removed, probe)
    assert sliver.volume > 1e-6

    xs = sliver.vertices[:, 0]
    zs = sliver.vertices[:, 2]
    measured_width = xs.max() - xs.min()
    measured_depth = zs.max() - zs.min()
    assert measured_width == pytest.approx(gw, abs=0.05)
    assert measured_depth == pytest.approx(gd, abs=0.05)
    assert zs.max() == pytest.approx(0.0, abs=0.05)


def test_oring_groove_bore_cut_cavity_matches_metadata():
    bore_d = 20.0
    groove = oring_groove(bore_d=bore_d, cs=3.53, squeeze=0.20, mode="bore", z0=0.0)
    gwidth = groove.metadata["groove_width"]
    gdepth = groove.metadata["groove_depth"]

    # A solid rod spanning both sides of the straddle: subtracting the
    # cutter removes material symmetrically from bore_d/2 - depth to
    # bore_d/2 + depth, so measuring the removed volume proves the cutter's
    # actual placement, not just its stated dimensions.
    rod = cyl(bore_d / 2.0 + gdepth + 5.0, gwidth + 12.0)
    rod.apply_translation((0.0, 0.0, 0.0))
    cut = sub(rod, groove)
    assert_mesh(cut)
    removed = sub(rod, cut)

    probe = boxc((2.0 * gdepth + 4.0, 0.8, gwidth + 4.0),
                center=(bore_d / 2.0, 0.0, 0.0))
    sliver = inter(removed, probe)
    assert sliver.volume > 1e-6
    xs = sliver.vertices[:, 0]
    zs = sliver.vertices[:, 2]
    measured_radial_span = xs.max() - xs.min()
    measured_axial_width = zs.max() - zs.min()
    assert measured_radial_span == pytest.approx(2.0 * gdepth, abs=0.05)
    assert measured_axial_width == pytest.approx(gwidth, abs=0.05)


def test_oring_groove_rejects_out_of_band_squeeze_and_fill():
    with pytest.raises(ValueError):
        oring_groove(face_pcd=40.0, squeeze=0.01, mode="face")
    with pytest.raises(ValueError):
        oring_groove(face_pcd=40.0, squeeze=0.5, mode="face")
    with pytest.raises(ValueError):
        oring_groove(face_pcd=40.0, fill=0.5, mode="face")
    with pytest.raises(ValueError):
        oring_groove(face_pcd=40.0, fill=0.95, mode="face")
    with pytest.raises(ValueError):
        # An explicit override that lands outside the fill band must still
        # raise, since the override path recomputes and validates too.
        oring_groove(face_pcd=40.0, cs=2.62, mode="face", width=2.0, depth=2.0)


def test_oring_groove_rejects_bad_mode_and_missing_diameters():
    with pytest.raises(ValueError):
        oring_groove(face_pcd=40.0, mode="radial")
    with pytest.raises(ValueError):
        oring_groove(mode="face")
    with pytest.raises(ValueError):
        oring_groove(mode="bore")
    with pytest.raises(ValueError):
        oring_groove(cs=-1.0, face_pcd=40.0, mode="face")


# ---------------------------------------------------------------------------
# labyrinth_seal
# ---------------------------------------------------------------------------


def test_labyrinth_seal_parts_and_metadata():
    parts = labyrinth_seal(shaft_d=8.0, teeth=4, tooth_t=1.2, gap=0.3)
    assert set(parts) == {"rotor", "stator"}
    rotor, stator = parts["rotor"], parts["stator"]
    assert_mesh(rotor)
    assert_mesh(stator)
    assert rotor.metadata["stages"] == 2 * 4 - 1
    assert stator.metadata["stages"] == 2 * 4 - 1
    assert rotor.metadata["radial_gap"] == pytest.approx(0.3)


def test_labyrinth_seal_non_contact_with_designed_gap():
    gap = 0.3
    parts = labyrinth_seal(shaft_d=8.0, teeth=4, tooth_t=1.2, gap=gap)
    rotor, stator = parts["rotor"], parts["stator"]
    assert overlap_volume(rotor, stator) < 1e-6
    d = min_distance(rotor, stator, n=4000)
    assert d == pytest.approx(gap, abs=0.03)


def test_labyrinth_seal_stage_count_by_probing():
    teeth = 4
    parts = labyrinth_seal(shaft_d=8.0, teeth=teeth, tooth_t=1.2, gap=0.3)
    rotor, stator = parts["rotor"], parts["stator"]
    fin_tip_r = rotor.metadata["fin_tip_r"]
    total_len = rotor.metadata["total_len"]

    # A thin shell just inside the fin tip radius crosses every rotor fin
    # as a separate disjoint band.
    fin_shell = trimesh.creation.annulus(fin_tip_r - 0.15, fin_tip_r - 0.02,
                                         total_len, sections=96)
    fin_shell.apply_translation((0.0, 0.0, total_len / 2.0))
    fin_slices = inter(rotor, fin_shell)
    fin_pieces = [p for p in fin_slices.split(only_watertight=False)
                 if p.volume > 1e-4]
    assert len(fin_pieces) == teeth

    root_r = rotor.metadata["rotor_root_r"]
    gap = rotor.metadata["radial_gap"]
    tooth_shell = trimesh.creation.annulus(root_r + gap + 0.02,
                                           root_r + gap + 0.15,
                                           total_len, sections=96)
    tooth_shell.apply_translation((0.0, 0.0, total_len / 2.0))
    tooth_slices = inter(stator, tooth_shell)
    tooth_pieces = [p for p in tooth_slices.split(only_watertight=False)
                   if p.volume > 1e-4]
    assert len(tooth_pieces) == teeth - 1
    assert len(fin_pieces) + len(tooth_pieces) == rotor.metadata["stages"]


def test_labyrinth_seal_rejects_bad_arguments():
    with pytest.raises(ValueError):
        labyrinth_seal(teeth=1)
    with pytest.raises(ValueError):
        labyrinth_seal(shaft_d=-1.0)
    with pytest.raises(ValueError):
        labyrinth_seal(tooth_t=0.0)
    with pytest.raises(ValueError):
        labyrinth_seal(gap=0.0)
    with pytest.raises(ValueError):
        # A pitch far too small forces the interleaved teeth to touch.
        labyrinth_seal(tooth_t=2.0, gap=0.5, pitch=1.0)


# ---------------------------------------------------------------------------
# gasket_channel
# ---------------------------------------------------------------------------


def _rounded_rect_ring(w=40.0, d=30.0, r=6.0):
    poly = sg.box(-w / 2.0, -d / 2.0, w / 2.0, d / 2.0).buffer(
        r, join_style=1).buffer(-r, join_style=1)
    return poly.exterior


def test_gasket_channel_cut_cavity_matches_metadata():
    width, depth = 3.0, 1.5
    ring = _rounded_rect_ring()
    groove = gasket_channel(path=ring, width=width, depth=depth, z0=0.0)
    assert_mesh(groove)
    assert groove.metadata["groove_width"] == pytest.approx(width)
    assert groove.metadata["groove_depth"] == pytest.approx(depth)
    assert groove.metadata["path_length"] == pytest.approx(ring.length, rel=1e-3)

    slab = boxc((70.0, 60.0, 10.0), center=(0.0, 0.0, -5.0))
    cut = sub(slab, groove)
    assert_mesh(cut)
    removed = sub(slab, cut)

    # Probe the straight top edge (y = +15, tangent along x) away from the
    # rounded corners, thin along the tangent so it isolates a local slice.
    probe = boxc((0.8, width + 6.0, 10.0), center=(0.0, 15.0, -5.0))
    sliver = inter(removed, probe)
    assert sliver.volume > 1e-6
    ys = sliver.vertices[:, 1]
    zs = sliver.vertices[:, 2]
    measured_width = ys.max() - ys.min()
    measured_depth = zs.max() - zs.min()
    assert measured_width == pytest.approx(width, abs=0.1)
    assert measured_depth == pytest.approx(depth, abs=0.05)


def test_gasket_channel_accepts_point_list_and_closes_it():
    pts = [(-15, -10), (15, -10), (15, 10), (-15, 10)]
    groove = gasket_channel(path=pts, width=2.5, depth=1.2)
    assert_mesh(groove)
    # The perimeter of the closed rectangle, independent of the function.
    expected_perimeter = 2 * (30 + 20)
    assert groove.metadata["path_length"] == pytest.approx(
        expected_perimeter, rel=0.1)


def test_gasket_channel_rejects_bad_arguments():
    with pytest.raises(ValueError):
        gasket_channel(path=None)
    with pytest.raises(ValueError):
        gasket_channel(path=[(0, 0), (1, 0), (1, 1)], width=0.0, depth=1.0)
    with pytest.raises(ValueError):
        gasket_channel(path=[(0, 0), (1, 0), (1, 1)], width=1.0, depth=-1.0)
    with pytest.raises(ValueError):
        gasket_channel(path=[(0, 0), (1, 0)], width=1.0, depth=1.0)
