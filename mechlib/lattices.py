"""Project-agnostic 2D metamaterial cells: auxetic panels and kerf-bend cutters."""

import math

import shapely.affinity
import shapely.geometry as sg
from shapely.ops import unary_union

from .meshutil import extrude_poly_z, from_manifold, to_manifold

_NOZZLE_WIDTHS = (0.4, 0.8, 1.2)
_AUXETIC_MODES = ("reentrant", "rotating_squares", "chiral")
_KERF_MODES = ("lattice", "diagonal", "spiral")
_MAX_CELLS = 2500


def _extrude(poly, height):
    """Extrude a (Multi)Polygon and repair through manifold3d if needed."""
    if poly is None or poly.is_empty:
        raise ValueError("lattice geometry collapsed to nothing; widen cell/strut_t")
    mesh = extrude_poly_z(poly, 0.0, height)
    if mesh is None:
        raise ValueError("lattice geometry collapsed to nothing; widen cell/strut_t")
    if not mesh.is_watertight:
        mesh = from_manifold(to_manifold(mesh))
    return mesh


def _validate_nozzle(nozzle):
    if nozzle not in _NOZZLE_WIDTHS:
        raise ValueError(
            "nozzle must be one of %s mm (common FDM nozzle widths), got %r"
            % (_NOZZLE_WIDTHS, nozzle))


def _snap_strut(strut_t, nozzle, snap, label="strut_t"):
    """Enforce that a strut thickness is an integer multiple of the nozzle width.

    Sub-nozzle-multiple struts print as a single wobbly extrusion pass instead
    of a clean 1x/2x/3x perimeter stack, so this either snaps to the nearest
    valid multiple (``snap=True``, the default) or raises with the corrected
    value so the caller can choose deliberately.
    """
    if strut_t <= 0:
        raise ValueError("%s must be positive" % label)
    n = max(1, round(strut_t / nozzle))
    snapped = n * nozzle
    if snap:
        return snapped
    if abs(strut_t - snapped) > 1e-6:
        raise ValueError(
            "%s=%.3g mm is not an integer multiple of nozzle=%.3g mm; nearest "
            "printable value is %.3g mm (pass snap_strut=True to auto-snap)"
            % (label, strut_t, nozzle, snapped))
    return strut_t


def _grid_centres(nx, ny, pitch_x, pitch_y, x0=0.0, y0=0.0):
    """Yield (x, y) centres of an ``nx`` by ``ny`` rectangular grid.

    Matches the ``patterns.lighten_grid_centres`` generator idiom: a plain
    nested loop over integer grid indices rather than a vectorised array, so
    callers can short-circuit or transform individual cells cheaply.
    """
    for j in range(ny):
        for i in range(nx):
            yield x0 + i * pitch_x, y0 + j * pitch_y


def _usable_bounds(width, height, border):
    return (-width / 2.0 + border, -height / 2.0 + border,
            width / 2.0 - border, height / 2.0 - border)


def _fit_frame(material, width, height, border, overlap):
    """Build a border frame whose inner edge overlaps the material's bbox.

    The tiling generators fit whole cells inside the requested interior, so
    the tiled material's true bounding box is usually a bit smaller than the
    nominal interior (an integer number of cells rarely divides it exactly).
    Cutting the frame's inner edge from the nominal interior instead of the
    material's real bbox would leave a hairline gap between the last cell and
    the frame, so the union comes out as two disjoint polygons and the panel
    literally falls apart. Cutting from the material bbox, inset by
    ``overlap``, guarantees the frame ring always laps onto real material.
    """
    minx, miny, maxx, maxy = material.bounds
    outer = sg.box(-width / 2.0, -height / 2.0, width / 2.0, height / 2.0)
    cut = sg.box(minx + overlap, miny + overlap, maxx - overlap, maxy - overlap)
    frame = outer.difference(cut)
    border_actual = min(width / 2.0 + minx, width / 2.0 - maxx,
                        height / 2.0 + miny, height / 2.0 - maxy)
    if border_actual < border - 1e-6:
        raise ValueError(
            "auxetic_panel(): tiled cells overflow the requested border "
            "(actual %.3g mm < requested %.3g mm); increase border or "
            "shrink cell" % (border_actual, border))
    return frame, border_actual


def _count_holes(poly):
    """Return the total interior-ring count across a (Multi)Polygon."""
    geoms = poly.geoms if poly.geom_type == "MultiPolygon" else [poly]
    return sum(len(g.interiors) for g in geoms)


# ---------------------------------------------------------------------------
# Auxetic unit cells
# ---------------------------------------------------------------------------

def _reentrant_unit(bounds, cell, strut_t):
    """Build the re-entrant honeycomb (bowtie) skeleton, unioned across the grid.

    Each cell is a concave ("re-entrant") hexagon: a central vertical spine
    strut flanked top and bottom by a pair of struts splayed outward at 30
    degrees. Because the flanking struts point inward toward the spine rather
    than outward the way a normal hexagon's do, stretching the panel in X
    rotates the splayed struts open and pulls the panel wider in Y too
    (negative Poisson's ratio). Tiles edge-to-edge in a plain rectangular
    grid with no row offset: the splay length is chosen so each cell's
    horizontal tips land exactly on its neighbours' tips, and each interior
    tip point is shared by four cells, closing a diamond-shaped hole between
    every 2x2 block.
    """
    theta = math.radians(30.0)
    l = cell / (2.0 * math.cos(theta))
    h = cell * 0.4
    pitch_x = 2.0 * l * math.cos(theta)
    pitch_y = h + 2.0 * l * math.sin(theta)
    minx, miny, maxx, maxy = bounds
    nx = max(1, int((maxx - minx) // pitch_x))
    ny = max(1, int((maxy - miny) // pitch_y))
    if nx * ny > _MAX_CELLS:
        raise ValueError(
            "reentrant grid would build %d cells (cap %d); increase cell or "
            "shrink the panel" % (nx * ny, _MAX_CELLS))
    x0 = -((nx - 1) * pitch_x) / 2.0
    y0 = -((ny - 1) * pitch_y) / 2.0
    segs = []
    for cx, cy in _grid_centres(nx, ny, pitch_x, pitch_y, x0, y0):
        top_l = (cx - l * math.cos(theta), cy + h / 2.0 + l * math.sin(theta))
        top_m = (cx, cy + h / 2.0)
        top_r = (cx + l * math.cos(theta), cy + h / 2.0 + l * math.sin(theta))
        bot_l = (cx - l * math.cos(theta), cy - h / 2.0 - l * math.sin(theta))
        bot_m = (cx, cy - h / 2.0)
        bot_r = (cx + l * math.cos(theta), cy - h / 2.0 - l * math.sin(theta))
        segs.append(sg.LineString([top_l, top_m, top_r]))
        segs.append(sg.LineString([bot_l, bot_m, bot_r]))
        segs.append(sg.LineString([top_m, bot_m]))
    material = unary_union(
        [s.buffer(strut_t / 2.0, cap_style=2, join_style=2) for s in segs])
    return material, nx, ny, pitch_x, pitch_y


def _rotating_squares_unit(bounds, cell, strut_t, hinge_t):
    """Build the rotating-squares skeleton: rigid squares, corner-only hinges.

    Rigid squares of side ``sq`` (derived from ``cell`` minus the hinge gap)
    sit on a grid and touch nothing but their four diagonal neighbours, each
    via a short hinge ligament of width ``hinge_t`` running corner to corner.
    Because the squares are joined only at points, stretching the lattice
    makes every square rotate about its hinge points (the Grima-Evans
    rotating-squares mechanism) rather than stretch the squares themselves,
    so the panel widens in the transverse direction as it is pulled
    (negative Poisson's ratio). ``hinge_t`` is deliberately NOT snapped to
    the nozzle grid like ``strut_t``: it is a compliant living hinge, not a
    structural strut, and 0.6 mm (the default) is already thinner than the
    0.8 mm minimum-wall rule because that is where the mechanism concentrates
    fatigue. Expect it to be the first thing to crack under repeated
    cycling; oversize it if the panel needs to survive more than a few dozen
    full-range actuations.
    """
    gap = 2.2 * hinge_t
    pitch = cell
    sq = pitch - gap
    if sq <= hinge_t:
        raise ValueError(
            "rotating_squares(): cell=%.3g mm leaves no room for a square "
            "once the hinge gap is subtracted; increase cell or shrink hinge_t"
            % cell)
    minx, miny, maxx, maxy = bounds
    nx = max(1, int((maxx - minx) // pitch))
    ny = max(1, int((maxy - miny) // pitch))
    if nx * ny > _MAX_CELLS:
        raise ValueError(
            "rotating_squares grid would build %d cells (cap %d); increase "
            "cell or shrink the panel" % (nx * ny, _MAX_CELLS))
    x0 = -((nx - 1) * pitch) / 2.0
    y0 = -((ny - 1) * pitch) / 2.0
    centres = {}
    squares = []
    for i in range(nx):
        for j in range(ny):
            cx, cy = x0 + i * pitch, y0 + j * pitch
            centres[(i, j)] = (cx, cy)
            squares.append(sg.box(cx - sq / 2.0, cy - sq / 2.0,
                                  cx + sq / 2.0, cy + sq / 2.0))
    # Every square's own top-right corner anchors up to two hinges: one
    # running right to its column-neighbour's top-left corner, one running up
    # to its row-neighbour's bottom-right corner. Diagonal-only hinges (each
    # square linked solely to its NE/NW neighbours) would preserve the
    # (i - j) parity of every move and split the lattice into two disjoint
    # diagonal networks instead of one connected sheet; anchoring both a
    # rightward and an upward hinge on the same corner threads every square
    # into a single spanning network while keeping every joint a true
    # corner-to-corner point contact.
    hinges = []
    for (i, j), (cx, cy) in centres.items():
        corner = (cx + sq / 2.0, cy + sq / 2.0)
        if (i + 1, j) in centres:
            ox, oy = centres[(i + 1, j)]
            hinges.append(sg.LineString([corner, (ox - sq / 2.0, oy + sq / 2.0)]))
        if (i, j + 1) in centres:
            ox, oy = centres[(i, j + 1)]
            hinges.append(sg.LineString([corner, (ox + sq / 2.0, oy - sq / 2.0)]))
    hinge_polys = [h.buffer(hinge_t / 2.0, cap_style=2, join_style=2)
                  for h in hinges]
    material = unary_union(squares + hinge_polys)
    return material, nx, ny, pitch, pitch, sq


def _chiral_unit(bounds, cell, strut_t, node_r):
    """Build the hexachiral skeleton: circular nodes, tangent ligaments.

    Nodes sit on a triangular grid; each interior node connects to its
    neighbours with a ligament offset tangent to the node circle (not
    through its centre), always on the same rotational side. That consistent
    offset is what makes the cell chiral: pulling the lattice apart torques
    every node about its own centre in the same sense, and the ligaments
    unwind like tangent spokes, opening the lattice in the transverse
    direction too (negative Poisson's ratio).
    """
    pitch_x = cell
    pitch_y = cell * math.sqrt(3.0) / 2.0
    minx, miny, maxx, maxy = bounds
    # Odd rows are offset by half a pitch (triangular grid), so reserve that
    # half-pitch out of the usable width or the offset rows overhang the
    # even rows' extent; also reserve a node radius on every side since the
    # node circles bulge past their centres.
    nx = max(2, int((maxx - minx - pitch_x / 2.0 - 2.0 * node_r) // pitch_x))
    ny = max(2, int((maxy - miny - 2.0 * node_r) // pitch_y))
    if nx * ny > _MAX_CELLS:
        raise ValueError(
            "chiral grid would build %d nodes (cap %d); increase cell or "
            "shrink the panel" % (nx * ny, _MAX_CELLS))
    x0 = -((nx - 1) * pitch_x) / 2.0
    y0 = -((ny - 1) * pitch_y) / 2.0
    nodes = {}
    for j in range(ny):
        row_off = (pitch_x / 2.0) if (j % 2) else 0.0
        for i in range(nx):
            nodes[(i, j)] = (x0 + i * pitch_x + row_off, y0 + j * pitch_y)
    neighbour_steps = ((1, 0), (0, 1), (-1, 1))
    ligaments = []
    for (i, j), (x0n, y0n) in nodes.items():
        for di, dj in neighbour_steps:
            other = (i + di, j + dj)
            if other not in nodes:
                continue
            x1, y1 = nodes[other]
            dx, dy = x1 - x0n, y1 - y0n
            length = math.hypot(dx, dy)
            if length < 1e-9:
                continue
            ux, uy = dx / length, dy / length
            px, py = -uy * node_r, ux * node_r
            ligaments.append(sg.LineString([
                (x0n + px, y0n + py), (x1 + px, y1 + py)]))
    circles = [sg.Point(x, y).buffer(node_r, resolution=16)
              for x, y in nodes.values()]
    ligament_polys = [ln.buffer(strut_t / 2.0, cap_style=2, join_style=2)
                      for ln in ligaments]
    material = unary_union(circles + ligament_polys)
    return material, nx, ny, pitch_x, pitch_y, len(nodes)


def auxetic_panel(mode="reentrant", width=60.0, height=60.0, thickness=3.0,
                  cell=12.0, strut_t=1.2, hinge_t=0.6, node_r=None,
                  border=3.0, nozzle=0.4, snap_strut=True):
    """Build a flat auxetic panel: negative Poisson's ratio under in-plane pull.

    Ordinary sheet material gets thinner when you stretch it. These panels do
    the opposite: their internal cell topology, not the base material,
    supplies the negative Poisson's ratio, so a rectangle of ordinary PLA or
    PETG stretched along X measurably widens along Y too. Three topologies
    are supported. ``"reentrant"`` is a bowtie/inverted-honeycomb lattice
    (concave hexagon cells whose splayed struts hinge open under tension).
    ``"rotating_squares"`` is rigid square islands joined only at their
    corners by short living hinges, so the squares rotate rather than stretch
    (the classic Grima-Evans mechanism); its corner hinge is the fatigue
    weak point and defaults to a thinner ``hinge_t=0.6`` mm on purpose, well
    below the library's normal 0.8 mm minimum-wall rule, because that IS the
    compliant feature. ``"chiral"`` is a hexagonal grid of circular nodes
    joined by ligaments tangent (not radial) to each node, so pulling the
    panel spins every node in the same rotational sense.

    All modes fill a solid ``border``-wide frame around the tiled interior so
    the panel edge is a continuous rim, never a row of half-cut cells; the
    frame is fitted to the tiled cells' actual extent (reported in
    ``metadata["border_actual"]``, always >= the requested ``border``)
    because an integer cell count rarely divides the interior exactly.
    ``strut_t`` (and, for chiral, the ligament width) must print as clean
    single- or double-perimeter walls, so it is snapped to the nearest
    integer multiple of ``nozzle`` (one of 0.4 / 0.8 / 1.2 mm) by default;
    pass ``snap_strut=False`` to get a ``ValueError`` with the corrected
    value instead of silent snapping. The panel sits flat with its bottom
    face at z=0 and centred on the XY origin; print it flat, cell layer
    down, no supports needed. Cell counts are capped so a default-sized
    panel builds in well under a second; shrink ``cell`` deliberately if you
    want a denser lattice, and expect the build to slow down accordingly.
    Units are mm and degrees.
    """
    if mode not in _AUXETIC_MODES:
        raise ValueError("auxetic_panel(): mode must be one of %s" % (_AUXETIC_MODES,))
    if width <= 0 or height <= 0 or thickness < 0.8:
        raise ValueError("auxetic_panel(): width/height must be positive and "
                          "thickness must be at least 0.8 mm")
    if cell <= 0:
        raise ValueError("auxetic_panel(): cell must be positive")
    if border <= 0 or border >= min(width, height) / 2.0:
        raise ValueError("auxetic_panel(): border must be positive and less "
                          "than half the panel's shorter side")
    _validate_nozzle(nozzle)
    strut_t = _snap_strut(strut_t, nozzle, snap_strut, "strut_t")
    if cell < 4.0 * strut_t:
        raise ValueError(
            "auxetic_panel(): cell=%.3g mm is too small for strut_t=%.3g mm; "
            "the struts would fuse solid (need cell >= 4*strut_t)"
            % (cell, strut_t))

    bounds = _usable_bounds(width, height, border)

    if mode == "reentrant":
        material, nx, ny, px, py = _reentrant_unit(bounds, cell, strut_t)
        overlap = strut_t
        extra = {}
    elif mode == "rotating_squares":
        if hinge_t <= 0 or hinge_t < nozzle:
            raise ValueError(
                "auxetic_panel(): hinge_t must be at least one nozzle width "
                "(%.2g mm)" % nozzle)
        material, nx, ny, px, py, sq = _rotating_squares_unit(
            bounds, cell, strut_t, hinge_t)
        overlap = hinge_t
        extra = {"square_side": sq, "hinge_t": hinge_t}
    else:  # chiral
        node_r_eff = node_r if node_r is not None else 0.3 * cell
        if node_r_eff <= strut_t:
            raise ValueError(
                "auxetic_panel(): node_r must exceed strut_t so ligaments "
                "stay tangent instead of crossing the node")
        material, nx, ny, px, py, n_nodes = _chiral_unit(
            bounds, cell, strut_t, node_r_eff)
        overlap = node_r_eff
        extra = {"node_r": node_r_eff, "node_count": n_nodes}

    frame, border_actual = _fit_frame(material, width, height, border, overlap)
    combined = unary_union([frame, material])
    combined = combined.intersection(sg.box(-width / 2.0, -height / 2.0,
                                            width / 2.0, height / 2.0))
    if combined.geom_type not in ("Polygon", "MultiPolygon"):
        raise ValueError("auxetic_panel(): degenerate panel geometry")
    hole_count = _count_holes(combined)

    mesh = _extrude(combined, thickness)
    mesh.metadata.update({
        "mode": mode,
        "cell_size": cell,
        "cells_x": nx,
        "cells_y": ny,
        "cell_count": nx * ny,
        "pitch_x": px,
        "pitch_y": py,
        "strut_t": strut_t,
        "border_actual": border_actual,
        "hole_count": hole_count,
        "poisson_ratio_sign": -1,
    })
    mesh.metadata.update(extra)
    return mesh


# ---------------------------------------------------------------------------
# Kerf bend cutters
# ---------------------------------------------------------------------------

def _validate_kerf(kerf, bridge, pitch, nozzle):
    _validate_nozzle(nozzle)
    if kerf < nozzle:
        raise ValueError(
            "kerf_bend_cutter(): kerf=%.3g mm is below one nozzle width "
            "(%.2g mm); the slicer fuses the slit shut" % (kerf, nozzle))
    if bridge < 0.8:
        raise ValueError(
            "kerf_bend_cutter(): bridge=%.3g mm is below the 0.8 mm minimum; "
            "the bridge snaps on the first bend" % bridge)
    if pitch <= bridge + kerf:
        raise ValueError(
            "kerf_bend_cutter(): pitch=%.3g mm must exceed bridge + kerf "
            "(%.3g mm) so a slit segment has positive length"
            % (pitch, bridge + kerf))


def _lattice_slit_polys(width, height, kerf, pitch, bridge, margin, shear):
    """Return staggered slit rectangles perpendicular to the local X axis.

    Rows run along Y at ``pitch`` spacing in X; each row is a chain of short
    slits separated by ``bridge``-wide uncut bridges, and every other row is
    offset half a bridge-pitch along Y so the panel flexes smoothly instead
    of concentrating the bend at one bridge line. ``shear`` progressively
    shifts each row along X, turning the straight lattice into a helix (used
    by the spiral mode).
    """
    usable_w = width - 2.0 * margin
    usable_h = height - 2.0 * margin
    n_rows = max(1, int(usable_w // pitch))
    seg_len = pitch - kerf
    n_slits = max(1, int(usable_h // (seg_len + bridge)))
    if n_rows * n_slits > _MAX_CELLS:
        raise ValueError(
            "kerf_bend_cutter(): lattice would cut %d slits (cap %d); "
            "increase pitch/bridge or shrink the panel"
            % (n_rows * n_slits, _MAX_CELLS))
    x0 = -((n_rows - 1) * pitch) / 2.0
    y0 = -((n_slits - 1) * (seg_len + bridge)) / 2.0
    polys = []
    for r in range(n_rows):
        row_x = x0 + r * pitch + r * shear
        stagger = (seg_len + bridge) / 2.0 if (r % 2) else 0.0
        for s in range(n_slits):
            y_c = y0 + s * (seg_len + bridge) + stagger
            if y_c - seg_len / 2.0 < -usable_h / 2.0 - 1e-6:
                continue
            if y_c + seg_len / 2.0 > usable_h / 2.0 + 1e-6:
                continue
            polys.append(sg.box(row_x - kerf / 2.0, y_c - seg_len / 2.0,
                                row_x + kerf / 2.0, y_c + seg_len / 2.0))
    return polys, n_rows, n_slits


def kerf_bend_cutter(mode="lattice", width=60.0, height=40.0, thickness=3.0,
                     kerf=0.5, pitch=6.0, bridge=1.0, angle_deg=45.0,
                     helix_shear=1.5, margin=4.0, nozzle=0.4):
    """Build slit-array cutter solids that turn a flat slab into a bendable one.

    This is a CUTTER, matching the ``mechlib.cutters`` contract: it returns
    negative geometry meant for ``meshutil.sub(slab, uni(cutters))``, not a
    finished part. Cutting a staggered grid of slits through an otherwise
    rigid slab lets it flex along the uncut bridges between slits, the same
    trick used for laser-kerfed plywood; at hinge scale it also IS a
    single-axis living hinge, so there is no separate ``living_hinge_kerf``
    function -- use ``mode="lattice"`` with ``pitch``/``bridge`` sized down
    to a single fold line for that case. ``mode="lattice"`` cuts slits
    perpendicular to the panel's local X axis so it bends about an axis
    parallel to Y (rolls up along X). ``mode="diagonal"`` rotates the same
    slit lattice by ``angle_deg`` (default 45) so the panel twists (shears)
    about its long axis instead of bending cleanly on one line, the
    torsion-compliant case. ``mode="spiral"`` shears each successive row of
    the lattice sideways by ``helix_shear`` mm, producing a helical slit
    pattern so the panel can be curled into a cylindrical wrap while also
    advancing along its axis, the way a spiral-cut tube is scored.

    Both FDM floors are validated, not just documented: ``kerf`` must be at
    least one ``nozzle`` width (0.4 mm) or the slicer's minimum feature size
    fuses the slit shut before it ever prints as a gap, and ``bridge`` (the
    uncut web left between consecutive slits) must be at least 0.8 mm or it
    snaps on the first bend cycle. Both raise ``ValueError`` below their
    floor. The panel is expected to span from z=0 to z=``thickness``; the
    returned cutters overshoot 0.5 mm past both faces so a boolean subtract
    is guaranteed to open a clean through-slit even against a slightly
    misaligned slab. Print the slab flat with slits vertical (through-slab)
    so no slit wall is a horizontal overhang. Cell counts are capped so a
    default-sized cutter set builds in well under a second.

    ``mesh.metadata["min_bend_radius_mm"]`` reports the approximate tightest
    radius the pattern can be bent to before the kerf gap closes solid again,
    from the standard kerf-bend approximation ``radius = thickness * pitch /
    kerf`` (the per-cut arc-length the bridge material has to absorb, set
    equal to the kerf gap being consumed). Units are mm and degrees.
    """
    if mode not in _KERF_MODES:
        raise ValueError("kerf_bend_cutter(): mode must be one of %s" % (_KERF_MODES,))
    if width <= 0 or height <= 0 or thickness <= 0:
        raise ValueError("kerf_bend_cutter(): width/height/thickness must be positive")
    if margin < 0 or margin >= min(width, height) / 2.0:
        raise ValueError("kerf_bend_cutter(): margin must be non-negative and "
                          "less than half the panel's shorter side")
    _validate_kerf(kerf, bridge, pitch, nozzle)

    shear = helix_shear if mode == "spiral" else 0.0
    polys, n_rows, n_slits = _lattice_slit_polys(
        width, height, kerf, pitch, bridge, margin, shear)
    slit_polys = unary_union(polys)
    if mode == "diagonal":
        slit_polys = shapely.affinity.rotate(slit_polys, angle_deg, origin=(0, 0))
    panel_rect = sg.box(-width / 2.0, -height / 2.0, width / 2.0, height / 2.0)
    slit_polys = slit_polys.intersection(panel_rect)
    if slit_polys.is_empty:
        raise ValueError("kerf_bend_cutter(): no slits fit inside the panel "
                         "margins; shrink margin or pitch")

    z0, z1 = -0.5, thickness + 0.5
    mesh = _extrude(slit_polys, z1 - z0)
    mesh.apply_translation((0.0, 0.0, z0))
    min_bend_radius = thickness * pitch / kerf
    mesh.metadata.update({
        "mode": mode,
        "kerf": kerf,
        "pitch": pitch,
        "bridge": bridge,
        "n_rows": n_rows,
        "n_slits_per_row": n_slits,
        "min_bend_radius_mm": min_bend_radius,
    })
    return [mesh]


__all__ = (
    "auxetic_panel",
    "kerf_bend_cutter",
)
