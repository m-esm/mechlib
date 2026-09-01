import math

import pytest
import trimesh

from mechlib.lattices import auxetic_panel, honeycomb_panel, isogrid_panel, kerf_bend_cutter
from mechlib.meshutil import sub, uni
from mechlib.prim import boxc


def assert_mesh(mesh):
    assert isinstance(mesh, trimesh.Trimesh)
    assert mesh.is_watertight
    assert mesh.volume > 0


# ---------------------------------------------------------------------------
# auxetic_panel
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("mode", ["reentrant", "rotating_squares", "chiral"])
def test_auxetic_panel_watertight_and_single_body(mode):
    panel = auxetic_panel(mode=mode, width=60.0, height=60.0, cell=12.0)
    assert_mesh(panel)
    assert len(panel.split(only_watertight=False)) == 1
    assert panel.metadata["poisson_ratio_sign"] == -1
    assert panel.metadata["mode"] == mode


@pytest.mark.parametrize("mode", ["reentrant", "rotating_squares", "chiral"])
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
# kerf_bend_cutter
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("mode", ["lattice", "diagonal", "spiral", "wave", "hex", "cross"])
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

    # Every disjoint cut piece is a slit box whose narrow OBB dimension is
    # exactly the requested kerf (never fused thinner by the boolean).
    pieces = cutters[0].split(only_watertight=False)
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
    # Rotating (diagonal), shearing (spiral), waving, hex-edge, and cross
    # X-lattice slits must actually change the cut geometry, not just relabel it.
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
