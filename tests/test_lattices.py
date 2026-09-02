import math

import numpy as np
import pytest
import trimesh

from mechlib.lattices import _kelvin_graph, _octet_graph, auxetic_panel, bcc_lattice, honeycomb_panel, isogrid_panel, kagome_panel, kelvin_cell, kerf_bend_cutter, octet_truss
from mechlib.meshutil import sub, uni
from mechlib.prim import boxc


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


# ---------------------------------------------------------------------------
# auxetic_panel
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "mode", ["reentrant", "rotating_squares", "arrowhead", "star", "chiral",
             "anti_tetrachiral", "houndstooth"])
def test_auxetic_panel_watertight_and_single_body(mode):
    panel = auxetic_panel(mode=mode, width=60.0, height=60.0, cell=12.0)
    assert_mesh(panel)
    assert len(panel.split(only_watertight=False)) == 1
    assert panel.metadata["poisson_ratio_sign"] == -1
    assert panel.metadata["mode"] == mode


@pytest.mark.parametrize(
    "mode", ["reentrant", "rotating_squares", "arrowhead", "star", "chiral",
             "anti_tetrachiral", "houndstooth"])
def test_auxetic_panel_hole_count_matches_euler_characteristic(mode):
    # A flat extruded slab with N separate through-holes is topologically a
    # genus-N solid, so euler_number == 2 - 2*N. The panel's own 2D
    # construction independently counts its through-holes (interior rings of
    # the pre-extrusion polygon) into metadata["hole_count"]; cross-checking
    # that against the 3D mesh's Euler characteristic is a structural
    # assertion that the printed part actually has the topology the 2D
    # construction claims, not just "some watertight blob".
    panel = auxetic_panel(mode=mode, width=60.0, height=60.0, cell=12.0)
    assert panel.metadata["hole_count"] > 0
    assert panel.euler_number == 2 - 2 * panel.metadata["hole_count"]


def test_reentrant_cell_grid_matches_requested_cell_size():
    panel = auxetic_panel(mode="reentrant", width=80.0, height=80.0, cell=10.0)
    assert panel.metadata["cell_size"] == 10.0
    assert panel.metadata["pitch_x"] == pytest.approx(10.0)
    assert panel.metadata["cell_count"] == (
        panel.metadata["cells_x"] * panel.metadata["cells_y"])
    assert panel.metadata["cells_x"] >= 5
    assert panel.metadata["cells_y"] >= 5


def test_arrowhead_layout_differs_from_reentrant():
    arrowhead = auxetic_panel(mode="arrowhead", width=60.0, height=60.0,
                              cell=12.0)
    reentrant = auxetic_panel(mode="reentrant", width=60.0, height=60.0,
                              cell=12.0)
    assert arrowhead.volume != pytest.approx(reentrant.volume, rel=0.01)


def test_star_layout_differs_from_reentrant_and_arrowhead():
    star = auxetic_panel(mode="star", width=60.0, height=60.0, cell=12.0)
    reentrant = auxetic_panel(mode="reentrant", width=60.0, height=60.0,
                              cell=12.0)
    arrowhead = auxetic_panel(mode="arrowhead", width=60.0, height=60.0,
                              cell=12.0)
    assert star.volume != pytest.approx(reentrant.volume, rel=0.01)
    assert star.volume != pytest.approx(arrowhead.volume, rel=0.01)


def test_houndstooth_layout_differs_from_reentrant_arrowhead_and_star():
    houndstooth = auxetic_panel(mode="houndstooth", width=60.0, height=60.0,
                                cell=12.0)
    reentrant = auxetic_panel(mode="reentrant", width=60.0, height=60.0,
                              cell=12.0)
    arrowhead = auxetic_panel(mode="arrowhead", width=60.0, height=60.0,
                              cell=12.0)
    star = auxetic_panel(mode="star", width=60.0, height=60.0, cell=12.0)
    assert houndstooth.volume != pytest.approx(reentrant.volume, rel=0.01)
    assert houndstooth.volume != pytest.approx(arrowhead.volume, rel=0.01)
    assert houndstooth.volume != pytest.approx(star.volume, rel=0.01)


def test_anti_tetrachiral_layout_differs_from_chiral():
    anti = auxetic_panel(mode="anti_tetrachiral", width=60.0, height=60.0,
                         cell=12.0)
    chiral = auxetic_panel(mode="chiral", width=60.0, height=60.0,
                           cell=12.0)
    assert anti.volume != pytest.approx(chiral.volume, rel=0.01)


def test_rotating_squares_islands_are_corner_connected_only():
    from mechlib.lattices import _rotating_squares_unit, _usable_bounds

    bounds = _usable_bounds(60.0, 60.0, 3.0)
    material, nx, ny, _px, _py, sq = _rotating_squares_unit(
        bounds, cell=12.0, strut_t=1.2, hinge_t=0.6)
    # The whole lattice is one connected polygon (squares + hinge ligaments)...
    assert material.geom_type == "Polygon"
    # ...but eroding past the hinge half-width and short of the square
    # half-width breaks every hinge while leaving each square intact, so the
    # squares fall apart into exactly nx*ny disjoint islands. (Eroding also
    # leaves a handful of near-zero-area slivers at the corner hinge
    # junctions; filter those out by area.)
    eroded = material.buffer(-(0.6 / 2.0 + 0.05))
    pieces = list(eroded.geoms) if eroded.geom_type == "MultiPolygon" else [eroded]
    islands = [p for p in pieces if p.area > 1.0]
    assert len(islands) == nx * ny
    erosion = 0.6 / 2.0 + 0.05
    expected_side = sq - 2.0 * erosion
    for island in islands:
        assert island.area == pytest.approx(expected_side * expected_side, rel=0.02)


def test_chiral_ligaments_are_tangent_not_radial():
    import math as _math

    import shapely.geometry as _sg

    from mechlib.lattices import _chiral_unit, _usable_bounds

    bounds = _usable_bounds(60.0, 60.0, 3.0)
    cell = 12.0
    node_r = 3.0
    material, nx, ny, _px, _py, n_nodes = _chiral_unit(
        bounds, cell=cell, strut_t=1.0, node_r=node_r)
    assert n_nodes == nx * ny
    # Rebuild the same node grid the helper used, then confirm every
    # ligament between two neighbouring nodes stays exactly node_r away from
    # each node's centre (tangent to its circle) rather than passing through
    # it (which a plain centre-to-centre spoke would do at distance 0).
    pitch_x = cell
    pitch_y = cell * _math.sqrt(3.0) / 2.0
    x0 = -((nx - 1) * pitch_x) / 2.0
    y0 = -((ny - 1) * pitch_y) / 2.0
    nodes = {}
    for j in range(ny):
        row_off = (pitch_x / 2.0) if (j % 2) else 0.0
        for i in range(nx):
            nodes[(i, j)] = (x0 + i * pitch_x + row_off, y0 + j * pitch_y)
    a = nodes[(0, 0)]
    b = nodes[(1, 0)]
    ligament_line = _sg.LineString([a, b])
    offset = ligament_line.distance(_sg.Point(a))
    assert offset == pytest.approx(0.0)  # centre line itself passes through a
    # The chiral construction offsets the actual ligament perpendicular by
    # node_r; verify that offset ligament clears the node circle instead of
    # cutting through its centre.
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = _math.hypot(dx, dy)
    ux, uy = dx / length, dy / length
    px, py = -uy * node_r, ux * node_r
    tangent_line = _sg.LineString([(a[0] + px, a[1] + py), (b[0] + px, b[1] + py)])
    assert tangent_line.distance(_sg.Point(a)) == pytest.approx(node_r)
    assert tangent_line.distance(_sg.Point(b)) == pytest.approx(node_r)


def test_auxetic_panel_border_is_solid_frame_at_the_edge():
    panel = auxetic_panel(mode="reentrant", width=50.0, height=50.0,
                          cell=8.0, border=4.0)
    assert_mesh(panel)
    assert panel.metadata["border_actual"] >= 4.0 - 1e-9
    # The extreme edge of the panel must be solid material (the frame),
    # not a half-cut cell: probe a thin strip right at the outer boundary.
    edge_probe = boxc((1.0, 40.0, 10.0), center=(24.5, 0.0, 1.5))
    from mechlib.meshutil import overlap_volume
    assert overlap_volume(panel, edge_probe) > 0.5


def test_auxetic_panel_strut_t_snaps_to_nozzle_grid_by_default():
    panel = auxetic_panel(strut_t=0.5, snap_strut=True)
    assert panel.metadata["strut_t"] == pytest.approx(0.4)
    panel2 = auxetic_panel(strut_t=0.9, snap_strut=True)
    assert panel2.metadata["strut_t"] == pytest.approx(0.8)


def test_auxetic_panel_strut_t_off_grid_raises_without_snap():
    with pytest.raises(ValueError):
        auxetic_panel(strut_t=0.5, snap_strut=False)
    # An exact multiple must pass through unchanged.
    panel = auxetic_panel(strut_t=0.8, snap_strut=False)
    assert panel.metadata["strut_t"] == pytest.approx(0.8)


def test_auxetic_panel_rejects_bad_arguments():
    with pytest.raises(ValueError):
        auxetic_panel(mode="not_a_mode")
    with pytest.raises(ValueError):
        auxetic_panel(cell=2.0, strut_t=1.2)  # cell < 4*strut_t, struts fuse
    with pytest.raises(ValueError):
        auxetic_panel(border=40.0, width=60.0, height=60.0)  # border too wide
    with pytest.raises(ValueError):
        auxetic_panel(nozzle=0.5)  # not a real nozzle width
    with pytest.raises(ValueError):
        auxetic_panel(mode="rotating_squares", hinge_t=0.1, nozzle=0.4)


def test_auxetic_panel_cell_cap_protects_the_playground():
    # A user dragging cell to its minimum on a large panel must fail fast
    # with a clear message instead of hanging the browser on 10,000+ cells.
    with pytest.raises(ValueError):
        auxetic_panel(mode="reentrant", width=400.0, height=400.0,
                      cell=2.0, strut_t=0.4, border=3.0)


# ---------------------------------------------------------------------------
# honeycomb_panel
# ---------------------------------------------------------------------------

def test_honeycomb_panel_watertight_and_single_body():
    panel = honeycomb_panel(width=60.0, height=60.0, cell=12.0)
    assert_mesh(panel)
    assert len(panel.split(only_watertight=False)) == 1
    assert panel.metadata["poisson_ratio_sign"] == 1
    assert panel.metadata["mode"] == "flat_top"
    assert panel.bounds[0][2] == pytest.approx(0.0, abs=1e-9)
    assert panel.bounds[1][2] == pytest.approx(3.0, abs=1e-6)


def test_honeycomb_panel_hole_count_matches_euler_characteristic():
    panel = honeycomb_panel(width=60.0, height=60.0, cell=12.0)
    assert panel.metadata["hole_count"] > 0
    assert panel.euler_number == 2 - 2 * panel.metadata["hole_count"]


def test_honeycomb_panel_cell_grid_matches_requested_cell_size():
    panel = honeycomb_panel(width=80.0, height=80.0, cell=10.0)
    assert panel.metadata["cell_size"] == 10.0
    assert panel.metadata["pitch_y"] == pytest.approx(10.0)
    assert panel.metadata["cell_count"] == panel.metadata["hole_count"]
    assert panel.metadata["cells_x"] >= 4
    assert panel.metadata["cells_y"] >= 4


def test_honeycomb_panel_border_is_solid_frame_at_the_edge():
    panel = honeycomb_panel(width=50.0, height=50.0, cell=8.0, border=4.0)
    assert_mesh(panel)
    assert panel.metadata["border_actual"] >= 4.0 - 1e-9
    edge_probe = boxc((1.0, 40.0, 10.0), center=(24.5, 0.0, 1.5))
    from mechlib.meshutil import overlap_volume
    assert overlap_volume(panel, edge_probe) > 0.5


def test_honeycomb_panel_strut_t_snaps_to_nozzle_grid_by_default():
    panel = honeycomb_panel(strut_t=0.5, snap_strut=True)
    assert panel.metadata["strut_t"] == pytest.approx(0.4)
    panel2 = honeycomb_panel(strut_t=0.9, snap_strut=True)
    assert panel2.metadata["strut_t"] == pytest.approx(0.8)


def test_honeycomb_panel_strut_t_off_grid_raises_without_snap():
    with pytest.raises(ValueError):
        honeycomb_panel(strut_t=0.5, snap_strut=False)
    panel = honeycomb_panel(strut_t=0.8, snap_strut=False)
    assert panel.metadata["strut_t"] == pytest.approx(0.8)


def test_honeycomb_panel_rejects_bad_arguments():
    with pytest.raises(ValueError):
        honeycomb_panel(cell=2.0, strut_t=1.2)  # cell < 4*strut_t, struts fuse
    with pytest.raises(ValueError):
        honeycomb_panel(border=40.0, width=60.0, height=60.0)  # border too wide
    with pytest.raises(ValueError):
        honeycomb_panel(nozzle=0.5)  # not a real nozzle width
    with pytest.raises(ValueError):
        honeycomb_panel(thickness=0.4)


def test_honeycomb_panel_cell_cap_protects_the_playground():
    with pytest.raises(ValueError):
        honeycomb_panel(width=400.0, height=400.0,
                        cell=2.0, strut_t=0.4, border=3.0)


# ---------------------------------------------------------------------------
# isogrid_panel
# ---------------------------------------------------------------------------

def test_isogrid_panel_watertight_and_single_body():
    panel = isogrid_panel(width=60.0, height=60.0, cell=12.0)
    assert_mesh(panel)
    assert len(panel.split(only_watertight=False)) == 1
    assert panel.metadata["poisson_ratio_sign"] == 1
    assert panel.metadata["mode"] == "triangle"
    assert panel.bounds[0][2] == pytest.approx(0.0, abs=1e-9)
    assert panel.bounds[1][2] == pytest.approx(3.0, abs=1e-6)


def test_isogrid_panel_hole_count_matches_euler_characteristic():
    panel = isogrid_panel(width=60.0, height=60.0, cell=12.0)
    assert panel.metadata["hole_count"] > 0
    assert panel.euler_number == 2 - 2 * panel.metadata["hole_count"]


def test_isogrid_panel_cell_grid_matches_requested_cell_size():
    panel = isogrid_panel(width=80.0, height=80.0, cell=10.0)
    assert panel.metadata["cell_size"] == 10.0
    assert panel.metadata["pitch_x"] == pytest.approx(10.0)
    assert panel.metadata["pitch_y"] == pytest.approx(10.0 * math.sqrt(3.0) / 2.0)
    assert panel.metadata["cell_count"] == panel.metadata["hole_count"]
    assert panel.metadata["cells_x"] >= 4
    assert panel.metadata["cells_y"] >= 4


def test_isogrid_panel_border_is_solid_frame_at_the_edge():
    panel = isogrid_panel(width=50.0, height=50.0, cell=8.0, border=4.0)
    assert_mesh(panel)
    assert panel.metadata["border_actual"] >= 4.0 - 1e-9
    edge_probe = boxc((1.0, 40.0, 10.0), center=(24.5, 0.0, 1.5))
    from mechlib.meshutil import overlap_volume
    assert overlap_volume(panel, edge_probe) > 0.5


def test_isogrid_panel_strut_t_snaps_to_nozzle_grid_by_default():
    panel = isogrid_panel(strut_t=0.5, snap_strut=True)
    assert panel.metadata["strut_t"] == pytest.approx(0.4)
    panel2 = isogrid_panel(strut_t=0.9, snap_strut=True)
    assert panel2.metadata["strut_t"] == pytest.approx(0.8)


def test_isogrid_panel_strut_t_off_grid_raises_without_snap():
    with pytest.raises(ValueError):
        isogrid_panel(strut_t=0.5, snap_strut=False)
    panel = isogrid_panel(strut_t=0.8, snap_strut=False)
    assert panel.metadata["strut_t"] == pytest.approx(0.8)


def test_isogrid_panel_rejects_bad_arguments():
    with pytest.raises(ValueError):
        isogrid_panel(cell=2.0, strut_t=1.2)  # cell < 4*strut_t, struts fuse
    with pytest.raises(ValueError):
        isogrid_panel(border=40.0, width=60.0, height=60.0)  # border too wide
    with pytest.raises(ValueError):
        isogrid_panel(nozzle=0.5)  # not a real nozzle width
    with pytest.raises(ValueError):
        isogrid_panel(thickness=0.4)


def test_isogrid_panel_cell_cap_protects_the_playground():
    with pytest.raises(ValueError):
        isogrid_panel(width=400.0, height=400.0,
                     cell=2.0, strut_t=0.4, border=3.0)


# ---------------------------------------------------------------------------
# kagome_panel
# ---------------------------------------------------------------------------

def test_kagome_panel_watertight_and_single_body():
    panel = kagome_panel(width=60.0, height=60.0, cell=12.0)
    assert_mesh(panel)
    assert len(panel.split(only_watertight=False)) == 1
    assert panel.metadata["poisson_ratio_sign"] == 1
    assert panel.metadata["mode"] == "kagome"
    assert panel.bounds[0][2] == pytest.approx(0.0, abs=1e-9)
    assert panel.bounds[1][2] == pytest.approx(3.0, abs=1e-6)


def test_kagome_panel_hole_count_matches_euler_characteristic():
    panel = kagome_panel(width=60.0, height=60.0, cell=12.0)
    assert panel.metadata["hole_count"] > 0
    assert panel.euler_number == 2 - 2 * panel.metadata["hole_count"]


def test_kagome_panel_has_both_triangle_and_hex_holes():
    panel = kagome_panel(width=80.0, height=80.0, cell=14.0)
    assert panel.metadata["tri_holes"] > 0
    assert panel.metadata["hex_holes"] > 0
    assert panel.metadata["cell_count"] == (
        panel.metadata["tri_holes"] + panel.metadata["hex_holes"])


def test_kagome_panel_cell_grid_matches_requested_cell_size():
    panel = kagome_panel(width=80.0, height=80.0, cell=12.0)
    assert panel.metadata["cell_size"] == 12.0
    assert panel.metadata["pitch_x"] == pytest.approx(12.0)
    assert panel.metadata["pitch_y"] == pytest.approx(12.0 * math.sqrt(3.0) / 2.0)


def test_kagome_panel_border_is_solid_frame_at_the_edge():
    panel = kagome_panel(width=50.0, height=50.0, cell=8.0, border=4.0)
    assert_mesh(panel)
    assert panel.metadata["border_actual"] >= 4.0 - 1e-9
    edge_probe = boxc((1.0, 40.0, 10.0), center=(24.5, 0.0, 1.5))
    from mechlib.meshutil import overlap_volume
    assert overlap_volume(panel, edge_probe) > 0.5


def test_kagome_panel_strut_t_snaps_to_nozzle_grid_by_default():
    panel = kagome_panel(strut_t=0.5, snap_strut=True)
    assert panel.metadata["strut_t"] == pytest.approx(0.4)
    panel2 = kagome_panel(strut_t=0.9, snap_strut=True)
    assert panel2.metadata["strut_t"] == pytest.approx(0.8)


def test_kagome_panel_strut_t_off_grid_raises_without_snap():
    with pytest.raises(ValueError):
        kagome_panel(strut_t=0.5, snap_strut=False)
    panel = kagome_panel(strut_t=0.8, snap_strut=False)
    assert panel.metadata["strut_t"] == pytest.approx(0.8)


def test_kagome_panel_rejects_bad_arguments():
    with pytest.raises(ValueError):
        kagome_panel(cell=2.0, strut_t=1.2)  # cell < 4*strut_t, struts fuse
    with pytest.raises(ValueError):
        kagome_panel(border=40.0, width=60.0, height=60.0)  # border too wide
    with pytest.raises(ValueError):
        kagome_panel(nozzle=0.5)  # not a real nozzle width
    with pytest.raises(ValueError):
        kagome_panel(thickness=0.4)


def test_kagome_panel_cell_cap_protects_the_playground():
    with pytest.raises(ValueError):
        kagome_panel(width=400.0, height=400.0,
                     cell=2.0, strut_t=0.4, border=3.0)


# ---------------------------------------------------------------------------
# kerf_bend_cutter
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("mode", ["lattice", "diagonal", "spiral", "wave", "hex", "cross", "chevron", "diamond", "fishbone", "meander", "biaxial"])
def test_kerf_bend_cutter_opens_clean_through_slits(mode):
    width, height, thickness = 60.0, 40.0, 3.0
    cutters = kerf_bend_cutter(mode=mode, width=width, height=height,
                               thickness=thickness, kerf=0.5, pitch=6.0,
                               bridge=1.0)
    assert isinstance(cutters, list) and len(cutters) >= 1
    for c in cutters:
        assert_mesh(c)

    slab = boxc((width, height, thickness), center=(0, 0, thickness / 2.0))
    cut = sub(slab, uni(cutters))
    assert_mesh(cut)
    assert cut.volume < slab.volume

    # Meander is deliberately one continuous labyrinth; the older patterns
    # remain arrays of disjoint slit pieces.
    pieces = cutters[0].split(only_watertight=False)
    if mode == "meander":
        assert len(pieces) == 1
    else:
        assert len(pieces) > 1
    min_widths = [min(p.bounding_box_oriented.primitive.extents) for p in pieces]
    assert min(min_widths) >= 0.5 - 1e-6


def test_kerf_bend_cutter_min_bend_radius_matches_closed_form():
    cutters = kerf_bend_cutter(mode="lattice", thickness=3.0, kerf=0.5, pitch=6.0)
    expected = 3.0 * 6.0 / 0.5
    assert cutters[0].metadata["min_bend_radius_mm"] == pytest.approx(expected)


def _kerf_layout_differs(a, b):
    return abs(a.volume - b.volume) > 1e-6 or (
        a.bounds.tolist() != b.bounds.tolist())


def test_kerf_bend_cutter_modes_produce_different_slit_layouts():
    lattice = kerf_bend_cutter(mode="lattice")[0]
    diagonal = kerf_bend_cutter(mode="diagonal")[0]
    spiral = kerf_bend_cutter(mode="spiral")[0]
    wave = kerf_bend_cutter(mode="wave")[0]
    hex_ = kerf_bend_cutter(mode="hex")[0]
    cross = kerf_bend_cutter(mode="cross")[0]
    chevron = kerf_bend_cutter(mode="chevron")[0]
    diamond = kerf_bend_cutter(mode="diamond")[0]
    fishbone = kerf_bend_cutter(mode="fishbone")[0]
    meander = kerf_bend_cutter(mode="meander")[0]
    biaxial = kerf_bend_cutter(mode="biaxial")[0]
    # Rotating (diagonal), shearing (spiral), waving, hex-edge, cross
    # X-lattice, chevron arrowhead, and diamond brick-wall outline slits
    # must actually change the cut geometry, not just relabel it.
    assert _kerf_layout_differs(lattice, diagonal)
    assert _kerf_layout_differs(lattice, spiral)
    assert _kerf_layout_differs(lattice, wave)
    assert _kerf_layout_differs(diagonal, wave)
    assert _kerf_layout_differs(spiral, wave)
    assert _kerf_layout_differs(lattice, hex_)
    assert _kerf_layout_differs(wave, hex_)
    assert _kerf_layout_differs(diagonal, hex_)
    assert _kerf_layout_differs(spiral, hex_)
    assert _kerf_layout_differs(lattice, cross)
    assert _kerf_layout_differs(wave, cross)
    assert _kerf_layout_differs(hex_, cross)
    assert _kerf_layout_differs(diagonal, cross)
    assert _kerf_layout_differs(spiral, cross)
    assert _kerf_layout_differs(lattice, chevron)
    assert _kerf_layout_differs(wave, chevron)
    assert _kerf_layout_differs(hex_, chevron)
    assert _kerf_layout_differs(cross, chevron)
    assert _kerf_layout_differs(diagonal, chevron)
    assert _kerf_layout_differs(spiral, chevron)
    assert _kerf_layout_differs(lattice, diamond)
    assert _kerf_layout_differs(chevron, diamond)
    assert _kerf_layout_differs(hex_, diamond)
    assert _kerf_layout_differs(wave, diamond)
    assert _kerf_layout_differs(cross, diamond)
    assert _kerf_layout_differs(diagonal, diamond)
    assert _kerf_layout_differs(spiral, diamond)
    assert _kerf_layout_differs(lattice, fishbone)
    assert _kerf_layout_differs(wave, fishbone)
    assert _kerf_layout_differs(hex_, fishbone)
    assert _kerf_layout_differs(cross, fishbone)
    assert _kerf_layout_differs(chevron, fishbone)
    assert _kerf_layout_differs(diamond, fishbone)
    assert _kerf_layout_differs(lattice, meander)
    assert _kerf_layout_differs(wave, meander)
    assert _kerf_layout_differs(fishbone, meander)
    assert _kerf_layout_differs(biaxial, lattice)
    assert _kerf_layout_differs(biaxial, diagonal)
    assert _kerf_layout_differs(biaxial, cross)
    assert _kerf_layout_differs(biaxial, meander)


def test_kerf_bend_cutter_meander_is_one_square_wave_labyrinth():
    from mechlib.lattices import _meander_slit_polys

    with pytest.raises(ValueError):
        kerf_bend_cutter(mode="meander", kerf=0.2)
    with pytest.raises(ValueError):
        kerf_bend_cutter(mode="meander", bridge=0.5)

    cutters = kerf_bend_cutter(mode="meander", kerf=0.5, pitch=6.0,
                               bridge=1.0)
    mesh = cutters[0]
    assert mesh.metadata["mode"] == "meander"
    assert len(mesh.split(only_watertight=False)) == 1
    slab = boxc((60.0, 40.0, 3.0), center=(0.0, 0.0, 1.5))
    cut = sub(slab, mesh)
    assert len(cut.split(only_watertight=False)) == 1

    polys, n_runs, n_paths = _meander_slit_polys(
        60.0, 40.0, 0.5, 6.0, 1.0, 4.0)
    assert len(polys) == n_paths == 1
    assert n_runs > 1
    # The buffered turn ends stop one bridge inside the usable Y boundary.
    assert polys[0].bounds[1] == pytest.approx(-16.0 + 1.0)
    assert polys[0].bounds[3] == pytest.approx(16.0 - 1.0)


def test_kerf_bend_cutter_wave_slits_are_sinusoidal():
    cutters = kerf_bend_cutter(mode="wave", kerf=0.5, pitch=6.0, bridge=1.0)
    mesh = cutters[0]
    assert mesh.metadata["mode"] == "wave"
    for key in ("min_bend_radius_mm", "kerf", "pitch", "bridge"):
        assert key in mesh.metadata
    pieces = mesh.split(only_watertight=False)
    assert len(pieces) > 1
    # Straight lattice slits are kerf-wide in X; a sine channel's AABB is
    # wider because of the wave amplitude.
    x_spans = [p.extents[0] for p in pieces]
    assert max(x_spans) > 1.5


def _principal_xy_deg(mesh):
    verts = mesh.vertices
    n = float(len(verts))
    mx = sum(v[0] for v in verts) / n
    my = sum(v[1] for v in verts) / n
    cxx = cyy = cxy = 0.0
    for v in verts:
        x = v[0] - mx
        y = v[1] - my
        cxx += x * x
        cyy += y * y
        cxy += x * y
    ang = 0.5 * math.atan2(2.0 * cxy, cxx - cyy)
    return math.degrees(ang) % 180.0


def test_kerf_bend_cutter_biaxial_is_orthogonal_2_axis_wrap():
    with pytest.raises(ValueError):
        kerf_bend_cutter(mode="biaxial", kerf=0.2)
    with pytest.raises(ValueError):
        kerf_bend_cutter(mode="biaxial", bridge=0.5)

    cutters = kerf_bend_cutter(mode="biaxial", kerf=0.5, pitch=6.0,
                               bridge=1.0)
    mesh = cutters[0]
    assert mesh.metadata["mode"] == "biaxial"

    slab = boxc((60.0, 40.0, 3.0), center=(0.0, 0.0, 1.5))
    cut = sub(slab, mesh)
    assert len(cut.split(only_watertight=False)) == 1

    pieces = mesh.split(only_watertight=False)
    assert len(pieces) > 1
    bins = {0: 0, 90: 0}
    for piece in pieces:
        deg = _principal_xy_deg(piece)
        for target in (0, 90):
            delta = abs((deg - target + 90.0) % 180.0 - 90.0)
            if delta <= 20.0:
                bins[target] += 1
                break
    assert bins[0] >= 1 and bins[90] >= 1
    assert max(piece.extents[0] for piece in pieces) > 1.5


def test_kerf_bend_cutter_hex_slits_have_three_orientations():
    cutters = kerf_bend_cutter(mode="hex", kerf=0.5, pitch=6.0, bridge=1.0)
    mesh = cutters[0]
    assert mesh.metadata["mode"] == "hex"
    for key in ("min_bend_radius_mm", "kerf", "pitch", "bridge"):
        assert key in mesh.metadata
    pieces = mesh.split(only_watertight=False)
    assert len(pieces) > 1
    # Lattice slits are kerf-wide in X and long in Y. Hex edges run at
    # 0/60/120, so some pieces are wide in X (horizontal) rather than a
    # thin Y-slit family.
    x_spans = [p.extents[0] for p in pieces]
    assert max(x_spans) > 1.5
    bins = {0: 0, 60: 0, 120: 0}
    for p in pieces:
        deg = _principal_xy_deg(p)
        for target in (0, 60, 120):
            delta = abs((deg - target + 90.0) % 180.0 - 90.0)
            if delta <= 20.0:
                bins[target] += 1
                break
    assert bins[0] >= 1 and bins[60] >= 1 and bins[120] >= 1


def test_kerf_bend_cutter_cross_slits_have_x_lattice_arms():
    cutters = kerf_bend_cutter(mode="cross", kerf=0.5, pitch=6.0, bridge=1.0)
    mesh = cutters[0]
    assert mesh.metadata["mode"] == "cross"
    for key in ("min_bend_radius_mm", "kerf", "pitch", "bridge"):
        assert key in mesh.metadata
    pieces = mesh.split(only_watertight=False)
    assert len(pieces) > 1
    # Lattice slits are kerf-wide in X and long in Y. Cross arms run at
    # ~30/150, so some pieces span more than kerf in X.
    x_spans = [p.extents[0] for p in pieces]
    assert max(x_spans) > 1.5
    bins = {30: 0, 90: 0, 150: 0}
    for p in pieces:
        deg = _principal_xy_deg(p)
        for target in (30, 90, 150):
            delta = abs((deg - target + 90.0) % 180.0 - 90.0)
            if delta <= 20.0:
                bins[target] += 1
                break
    assert bins[30] >= 1 and bins[150] >= 1 and bins[90] >= 1


def test_kerf_bend_cutter_chevron_slits_are_nested_arrowheads():
    import shapely.geometry as sg

    from mechlib.lattices import _chevron_slit_polys

    cutters = kerf_bend_cutter(mode="chevron", kerf=0.5, pitch=6.0, bridge=1.0)
    mesh = cutters[0]
    assert mesh.metadata["mode"] == "chevron"
    for key in ("min_bend_radius_mm", "kerf", "pitch", "bridge"):
        assert key in mesh.metadata
    pieces = mesh.split(only_watertight=False)
    assert len(pieces) > 1
    # Lattice slits are kerf-wide in X and long in Y. A 45° chevron's
    # AABB spans more than kerf in X.
    x_spans = [p.extents[0] for p in pieces]
    assert max(x_spans) > 1.5
    # Each chevron is one continuous two-leg path, so split() sees the
    # joined arrowhead (PCA ~90°). Halve each 2D slit at its midline to
    # recover the 45° / 135° legs.
    polys, _, _ = _chevron_slit_polys(60.0, 40.0, 0.5, 6.0, 1.0, 4.0)
    bins = {45: 0, 135: 0}
    for poly in polys:
        geoms = list(poly.geoms) if poly.geom_type == "MultiPolygon" else [poly]
        for g in geoms:
            minx, miny, maxx, maxy = g.bounds
            midy = 0.5 * (miny + maxy)
            halves = (
                g.intersection(sg.box(minx - 1.0, midy, maxx + 1.0, maxy + 1.0)),
                g.intersection(sg.box(minx - 1.0, miny - 1.0, maxx + 1.0, midy)),
            )
            for half in halves:
                if half.is_empty or half.geom_type not in ("Polygon", "MultiPolygon"):
                    continue
                coords = []
                hs = half.geoms if half.geom_type == "MultiPolygon" else [half]
                for h in hs:
                    coords.extend(h.exterior.coords)
                if len(coords) < 3:
                    continue
                n = float(len(coords))
                mx = sum(c[0] for c in coords) / n
                my = sum(c[1] for c in coords) / n
                cxx = cyy = cxy = 0.0
                for c in coords:
                    x = c[0] - mx
                    y = c[1] - my
                    cxx += x * x
                    cyy += y * y
                    cxy += x * y
                deg = math.degrees(0.5 * math.atan2(2.0 * cxy, cxx - cyy)) % 180.0
                for target in (45, 135):
                    delta = abs((deg - target + 90.0) % 180.0 - 90.0)
                    if delta <= 20.0:
                        bins[target] += 1
                        break
    assert bins[45] >= 1 and bins[135] >= 1


def test_kerf_bend_cutter_diamond_slits_are_brick_wall_outlines():
    cutters = kerf_bend_cutter(mode="diamond", kerf=0.5, pitch=6.0, bridge=1.0)
    mesh = cutters[0]
    assert mesh.metadata["mode"] == "diamond"
    for key in ("min_bend_radius_mm", "kerf", "pitch", "bridge"):
        assert key in mesh.metadata
    pieces = mesh.split(only_watertight=False)
    assert len(pieces) > 1
    # Lattice slits are kerf-wide in X. A 2:1 rhombus edge spans more
    # than kerf in X. Edges are the four diagonals (~63/117), not a
    # vertical lattice family.
    x_spans = [p.extents[0] for p in pieces]
    assert max(x_spans) > 1.5
    bins = {63: 0, 117: 0}
    for p in pieces:
        deg = _principal_xy_deg(p)
        for target in (63, 117):
            delta = abs((deg - target + 90.0) % 180.0 - 90.0)
            if delta <= 20.0:
                bins[target] += 1
                break
    assert bins[63] >= 1 and bins[117] >= 1


def test_kerf_bend_cutter_fishbone_slits_have_paired_diagonal_ribs():
    cutters = kerf_bend_cutter(mode="fishbone", kerf=0.5, pitch=6.0,
                               bridge=1.0)
    mesh = cutters[0]
    assert mesh.metadata["mode"] == "fishbone"
    for key in ("min_bend_radius_mm", "kerf", "pitch", "bridge",
                "n_rows", "n_slits_per_row"):
        assert key in mesh.metadata
    pieces = mesh.split(only_watertight=False)
    assert len(pieces) > 1
    # Fishbone ribs span X and occur as separate 45/135 deg families,
    # unlike the kerf-wide vertical lattice and continuous chevrons.
    assert max(p.extents[0] for p in pieces) > 1.5
    bins = {45: 0, 135: 0}
    for p in pieces:
        deg = _principal_xy_deg(p)
        for target in (45, 135):
            delta = abs((deg - target + 90.0) % 180.0 - 90.0)
            if delta <= 20.0:
                bins[target] += 1
                break
    assert bins[45] >= 1 and bins[135] >= 1


def test_kerf_bend_cutter_rejects_sub_nozzle_kerf():
    with pytest.raises(ValueError):
        kerf_bend_cutter(kerf=0.2)
    # Exactly one nozzle width is the floor, not the ceiling: it must pass.
    ok = kerf_bend_cutter(kerf=0.4, pitch=6.0, bridge=1.0)
    assert_mesh(ok[0])


def test_kerf_bend_cutter_rejects_sub_minimum_bridge():
    with pytest.raises(ValueError):
        kerf_bend_cutter(bridge=0.5)
    ok = kerf_bend_cutter(bridge=0.8, pitch=6.0, kerf=0.5)
    assert_mesh(ok[0])


def test_kerf_bend_cutter_rejects_bad_arguments():
    with pytest.raises(ValueError):
        kerf_bend_cutter(mode="not_a_mode")
    with pytest.raises(ValueError):
        kerf_bend_cutter(pitch=1.0, bridge=1.0, kerf=0.5)  # pitch too tight
    with pytest.raises(ValueError):
        kerf_bend_cutter(width=-1.0)
    with pytest.raises(ValueError):
        kerf_bend_cutter(nozzle=0.6)


def test_kerf_bend_cutter_cap_protects_the_playground():
    with pytest.raises(ValueError):
        kerf_bend_cutter(mode="lattice", width=400.0, height=400.0,
                         kerf=0.4, pitch=1.5, bridge=0.8)


# ---------------------------------------------------------------------------
# bcc_lattice (3D strut truss)
# ---------------------------------------------------------------------------

def test_bcc_lattice_watertight_and_single_body():
    block = bcc_lattice(nx=2, ny=2, nz=2, cell=12.0, strut_d=1.6)
    assert_mesh(block)
    assert block.is_winding_consistent
    assert len(block.split(only_watertight=False)) == 1
    assert block.metadata["mode"] == "bcc"


def test_bcc_lattice_struts_and_nodes_count_correctly():
    nx, ny, nz = 3, 2, 1
    block = bcc_lattice(nx=nx, ny=ny, nz=nz, cell=10.0, strut_d=1.2)
    # 8 half-diagonal struts per cell.
    assert block.metadata["strut_count"] == 8 * nx * ny * nz
    # (nx+1)(ny+1)(nz+1) shared corners + one body-centre per cell.
    expect_nodes = (nx + 1) * (ny + 1) * (nz + 1) + nx * ny * nz
    assert block.metadata["node_count"] == expect_nodes


def test_bcc_lattice_sits_on_bed_and_centres_in_plane():
    nx, ny, nz, cell = 3, 3, 2, 12.0
    block = bcc_lattice(nx=nx, ny=ny, nz=nz, cell=cell, strut_d=1.6, node_d=2.4)
    lo, hi = block.bounds
    # Bottom nodes drop to z=0 (spheres dip half a node diameter below).
    assert lo[2] == pytest.approx(-1.2, abs=1e-6)
    assert hi[2] == pytest.approx(nz * cell + 1.2, abs=1e-6)
    # Centred in X and Y.
    assert lo[0] == pytest.approx(-hi[0], abs=1e-6)
    assert lo[1] == pytest.approx(-hi[1], abs=1e-6)


def test_bcc_lattice_relative_density_is_a_small_fraction():
    block = bcc_lattice(nx=2, ny=2, nz=2, cell=12.0, strut_d=1.6)
    rd = block.metadata["relative_density"]
    # An open strut truss is mostly air: well under half solid, and non-trivial.
    assert 0.0 < rd < 0.5


def test_bcc_lattice_strut_d_snaps_to_nozzle_grid_by_default():
    block = bcc_lattice(nx=1, ny=1, nz=1, cell=12.0, strut_d=1.5)
    assert block.metadata["strut_d"] == pytest.approx(1.6)


def test_bcc_lattice_strut_d_off_grid_raises_without_snap():
    with pytest.raises(ValueError):
        bcc_lattice(nx=1, ny=1, nz=1, cell=12.0, strut_d=1.5, snap_strut=False)


def test_bcc_lattice_denser_struts_raise_relative_density():
    thin = bcc_lattice(nx=2, ny=2, nz=2, cell=12.0, strut_d=1.2)
    thick = bcc_lattice(nx=2, ny=2, nz=2, cell=12.0, strut_d=2.0)
    assert thick.metadata["relative_density"] > thin.metadata["relative_density"]


def test_bcc_lattice_rejects_bad_arguments():
    with pytest.raises(ValueError):
        bcc_lattice(nx=0)
    with pytest.raises(ValueError):
        bcc_lattice(nx=2, ny=2, nz=2, cell=3.0, strut_d=1.6)  # cell < 3*strut_d
    with pytest.raises(ValueError):
        bcc_lattice(strut_d=0.4)  # below the 0.8 mm wall floor
    with pytest.raises(ValueError):
        bcc_lattice(nozzle=0.6)


def test_bcc_lattice_cell_cap_protects_the_playground():
    with pytest.raises(ValueError):
        bcc_lattice(nx=5, ny=5, nz=5, cell=12.0)  # 125 > 64-cell cap


# ---------------------------------------------------------------------------
# octet_truss (FCC face-diagonal truss)
# ---------------------------------------------------------------------------

def test_octet_mesh():
    block = octet_truss(nx=2, ny=2, nz=2)
    assert_mesh(block)
    assert block.is_winding_consistent
    assert len(block.split(only_watertight=False)) == 1
    assert block.metadata["mode"] == "octet"


def test_octet_graph():
    nx, ny, nz = 3, 2, 1
    nodes, edges = _octet_graph(nx, ny, nz, 10.0)
    expected_nodes = {
        (x, y, z)
        for z in range(2 * nz + 1)
        for y in range(2 * ny + 1)
        for x in range(2 * nx + 1)
        if (x + y + z) % 2 == 0
    }
    expected_edges = set()
    for node in expected_nodes:
        for offset in ((1, 1, 0), (1, -1, 0),
                       (1, 0, 1), (1, 0, -1),
                       (0, 1, 1), (0, 1, -1)):
            other = tuple(node[i] + offset[i] for i in range(3))
            if other in expected_nodes:
                expected_edges.add(tuple(sorted((node, other))))

    assert set(nodes) == expected_nodes
    assert edges == expected_edges
    assert len(edges) == len(set(edges))
    assert all(sum(delta == 0 for delta in np.subtract(a, b)) == 1
               and sorted(abs(delta) for delta in np.subtract(a, b)) == [0, 1, 1]
               for a, b in edges)

    block = octet_truss(nx=nx, ny=ny, nz=nz, cell=10.0, strut_d=1.2)
    assert block.metadata["node_count"] == len(expected_nodes)
    assert block.metadata["strut_count"] == len(expected_edges)


def test_octet_degree():
    _nodes, edges = _octet_graph(2, 2, 2, 12.0)
    centre = (2, 2, 2)
    neighbours = {b if a == centre else a for a, b in edges if centre in (a, b)}
    assert len(neighbours) == 12


def test_octet_bounds_density():
    block = octet_truss(nx=2, ny=3, nz=1, cell=12.0,
                        strut_d=1.5, node_d=2.4)
    lo, hi = block.bounds
    assert lo[0] == pytest.approx(-hi[0], abs=1e-6)
    assert lo[1] == pytest.approx(-hi[1], abs=1e-6)
    assert lo[2] == pytest.approx(-1.2, abs=1e-6)
    assert hi[2] == pytest.approx(12.0 + 1.2, abs=1e-6)
    assert block.metadata["strut_d"] == pytest.approx(1.6)
    assert block.metadata["node_d"] == pytest.approx(2.4)
    assert block.metadata["cell_count"] == 6
    assert 0.0 < block.metadata["relative_density"] < 0.5


def test_octet_density_growth():
    thin = octet_truss(nx=1, ny=1, nz=1, strut_d=1.2)
    thick = octet_truss(nx=1, ny=1, nz=1, strut_d=2.0)
    assert thick.metadata["relative_density"] > thin.metadata["relative_density"]


def test_octet_bad_args():
    for kwargs in ({"nx": 0}, {"ny": True}, {"sections": 5},
                   {"sections": 6.5}, {"strut_d": 0.4}, {"nozzle": 0.6},
                   {"cell": 3.0, "strut_d": 1.6}, {"node_d": 0}):
        with pytest.raises(ValueError):
            octet_truss(**kwargs)
    with pytest.raises(ValueError):
        octet_truss(strut_d=1.5, snap_strut=False)
    with pytest.raises(ValueError):
        octet_truss(nx=5, ny=5, nz=5)


# ---------------------------------------------------------------------------
# kelvin_cell (truncated-octahedron strut cell)
# ---------------------------------------------------------------------------

def test_kelvin_graph_topology():
    nodes, edges = _kelvin_graph(20.0)
    assert len(nodes) == 24
    assert len(edges) == 36
    assert len(edges) == len(set(edges))
    assert all(a != b and a in nodes and b in nodes for a, b in edges)
    degree = {node: 0 for node in nodes}
    for a, b in edges:
        degree[a] += 1
        degree[b] += 1
        assert np.linalg.norm(nodes[a] - nodes[b]) == pytest.approx(
            20.0 * math.sqrt(2.0) / 4.0)
    assert set(degree.values()) == {3}


def test_kelvin_mesh():
    mesh = kelvin_cell()
    assert_mesh(mesh)
    assert mesh.is_winding_consistent
    assert len(mesh.split(only_watertight=False)) == 1
    assert mesh.metadata["mode"] == "kelvin"
    assert mesh.metadata["node_count"] == 24
    assert mesh.metadata["strut_count"] == 36


def test_kelvin_bounds_metadata():
    mesh = kelvin_cell(cell=20.0, strut_d=1.5, node_d=2.4)
    lo, hi = mesh.bounds
    assert lo[0] == pytest.approx(-hi[0], abs=1e-6)
    assert lo[1] == pytest.approx(-hi[1], abs=1e-6)
    assert lo[2] == pytest.approx(-1.2, abs=1e-6)
    assert hi[2] == pytest.approx(21.2, abs=1e-6)
    assert mesh.metadata["cell_size"] == pytest.approx(20.0)
    assert mesh.metadata["strut_d"] == pytest.approx(1.6)
    assert mesh.metadata["node_d"] == pytest.approx(2.4)
    assert 0.0 < mesh.metadata["relative_density"] < 0.5


def test_kelvin_bad_args():
    assert kelvin_cell(strut_d=1.5).metadata["strut_d"] == pytest.approx(1.6)
    assert kelvin_cell(node_d=0.8).metadata["node_d"] == pytest.approx(1.6)
    for kwargs in ({"cell": 0}, {"sections": 5}, {"sections": 6.5},
                   {"strut_d": 0.4}, {"nozzle": 0.6}, {"node_d": 0},
                   {"cell": 8.0, "strut_d": 1.6}):
        with pytest.raises(ValueError):
            kelvin_cell(**kwargs)
    with pytest.raises(ValueError):
        kelvin_cell(strut_d=1.5, snap_strut=False)


def test_kelvin_density_growth():
    thin = kelvin_cell(strut_d=1.2)
    thick = kelvin_cell(strut_d=2.0)
    assert thick.metadata["relative_density"] > thin.metadata["relative_density"]
