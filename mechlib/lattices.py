"""Project-agnostic metamaterial cells: honeycomb, isogrid, auxetic panels,
kerf-bend cutters (all 2D-extruded), plus 3D strut lattices."""

import math
import numbers

import manifold3d
import numpy as np
import shapely.affinity
import shapely.geometry as sg
import trimesh
from shapely.ops import unary_union

from .meshutil import extrude_poly_z, from_manifold, to_manifold, uni

_NOZZLE_WIDTHS = (0.4, 0.8, 1.2)
_AUXETIC_MODES = (
    "reentrant", "rotating_squares", "arrowhead", "star", "chiral",
    "anti_tetrachiral", "houndstooth")
_KERF_MODES = ("lattice", "diagonal", "spiral", "wave", "hex", "cross", "chevron",
               "diamond", "fishbone", "meander", "biaxial")
_MAX_CELLS = 2500
_MAX_GYROID_SAMPLES = 350000
_GYROID_DISTANCE_COMPENSATION = 1.1


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


def _count_holes(poly, min_area=0.5):
    """Return the total interior-ring count across a (Multi)Polygon.

    Rings smaller than ``min_area`` mm² are ignored. GEOS boolean ops can
    leave sub-nozzle sliver holes that vanish during extrusion/repair, so
    counting them would disagree with the printed solid's Euler genus.
    """
    geoms = poly.geoms if poly.geom_type == "MultiPolygon" else [poly]
    n = 0
    for g in geoms:
        for ring in g.interiors:
            if abs(sg.Polygon(ring).area) >= min_area:
                n += 1
    return n


def _prune_holes(poly, min_area=0.5):
    """Drop interior rings smaller than ``min_area`` mm² (GEOS slivers)."""
    def _one(g):
        if g.geom_type != "Polygon" or not g.interiors:
            return g
        keep = [r for r in g.interiors
                if abs(sg.Polygon(r).area) >= min_area]
        if len(keep) == len(g.interiors):
            return g
        return sg.Polygon(g.exterior, keep)

    if poly.geom_type == "MultiPolygon":
        return unary_union([_one(g) for g in poly.geoms])
    return _one(poly)


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


def _arrowhead_unit(bounds, cell, strut_t):
    """Build a tiled double-arrowhead skeleton from inverted triangles.

    Each repeat has opposing upper and lower arrowheads sharing a narrow
    central waist.  Their outer shoulders are shared with the neighbouring
    columns and their tips with the neighbouring rows, producing a connected
    rectangular tessellation.  Pulling the shoulders apart along X opens the
    concave notches about the waist and drives the tips apart along Y: the
    classic Grima double-arrowhead negative-Poisson mechanism.  Unlike the
    re-entrant bowtie's 30-degree splayed hexagons, this cell is made from two
    steep inverted-triangle outlines and has two enclosed openings per repeat.
    """
    pitch_x = cell
    pitch_y = cell
    shoulder_y = 0.18 * cell
    minx, miny, maxx, maxy = bounds
    nx = max(1, int((maxx - minx) // pitch_x))
    ny = max(1, int((maxy - miny) // pitch_y))
    if nx * ny > _MAX_CELLS:
        raise ValueError(
            "arrowhead grid would build %d cells (cap %d); increase cell or "
            "shrink the panel" % (nx * ny, _MAX_CELLS))
    x0 = -((nx - 1) * pitch_x) / 2.0
    y0 = -((ny - 1) * pitch_y) / 2.0
    segs = []
    half_x = pitch_x / 2.0
    half_y = pitch_y / 2.0
    for cx, cy in _grid_centres(nx, ny, pitch_x, pitch_y, x0, y0):
        top = (cx, cy + half_y)
        waist = (cx, cy)
        bottom = (cx, cy - half_y)
        upper_l = (cx - half_x, cy + shoulder_y)
        upper_r = (cx + half_x, cy + shoulder_y)
        lower_l = (cx - half_x, cy - shoulder_y)
        lower_r = (cx + half_x, cy - shoulder_y)
        segs.append(sg.LineString([top, upper_l, waist, upper_r, top]))
        segs.append(sg.LineString([waist, lower_l, bottom, lower_r, waist]))
    material = unary_union(
        [s.buffer(strut_t / 2.0, cap_style=2, join_style=2) for s in segs])
    return material, nx, ny, pitch_x, pitch_y


def _star_unit(bounds, cell, strut_t):
    """Build Grima hexagram cells on a connected triangular lattice.

    Each cell is the outline of two overlapping equilateral triangles, giving
    a six-pointed star with re-entrant vertices around its central hexagon.
    Cell centres lie on a triangular (hexagonal-neighbour) lattice, with the
    star tip radius set to half the centre pitch. Opposing tips of all six
    neighbours therefore coincide exactly, joining the cells into one sheet.
    Under an in-plane pull those re-entrant star vertices open and the rows
    spread transversely, producing the negative-Poisson Grima mechanism.
    """
    pitch_x = cell
    pitch_y = cell * math.sqrt(3.0) / 2.0
    radius = cell / 2.0
    extent_y = radius * math.sqrt(3.0) / 2.0
    minx, miny, maxx, maxy = bounds
    # A mitred buffer around each 60-degree tip projects one full strut_t
    # beyond the centreline vertex, so reserve that material on every edge.
    usable_w = maxx - minx - 2.0 * strut_t
    usable_h = maxy - miny - 2.0 * strut_t
    # Odd rows shift by half a pitch. Centre the combined even/odd-row span
    # so both row parities retain the requested border at opposite edges.
    nx = max(1, int((usable_w - pitch_x / 2.0 - 2.0 * radius) // pitch_x) + 1)
    ny = max(1, int((usable_h - 2.0 * extent_y) // pitch_y) + 1)
    if nx * ny > _MAX_CELLS:
        raise ValueError(
            "star grid would build %d cells (cap %d); increase cell or "
            "shrink the panel" % (nx * ny, _MAX_CELLS))
    row_offset_span = pitch_x / 2.0 if ny > 1 else 0.0
    x0 = -((nx - 1) * pitch_x + row_offset_span) / 2.0
    y0 = -((ny - 1) * pitch_y) / 2.0
    segs = []
    for j in range(ny):
        row_off = pitch_x / 2.0 if (j % 2) else 0.0
        for i in range(nx):
            cx = x0 + i * pitch_x + row_off
            cy = y0 + j * pitch_y
            points = [
                (cx + radius * math.cos(math.radians(60.0 * k)),
                 cy + radius * math.sin(math.radians(60.0 * k)))
                for k in range(6)
            ]
            segs.append(sg.LineString([
                points[0], points[2], points[4], points[0]]))
            segs.append(sg.LineString([
                points[1], points[3], points[5], points[1]]))
    material = unary_union(
        [s.buffer(strut_t / 2.0, cap_style=2, join_style=2) for s in segs])
    return material, nx, ny, pitch_x, pitch_y


def _houndstooth_unit(bounds, cell, strut_t):
    """Build interlocking L / broken-chevron houndstooth NPR struts.

    Each repeat is a pair of L-hooks in opposite corners (bottom-left and
    top-right), each a broken chevron whose inner corner pokes into the void
    as a re-entrant notch. Neighbouring cells share the mid-edge endpoints,
    so the tiled sheet is one connected network. Stretching along X opens
    those notches and drives the complementary hooks apart along Y: the
    FFF-review houndstooth negative-Poisson cell. The diagonal L pairing is
    not a rotation of the re-entrant bowtie, the double-arrowhead, or the
    hexagram star.
    """
    pitch_x = cell
    pitch_y = cell
    minx, miny, maxx, maxy = bounds
    nx = max(1, int((maxx - minx) // pitch_x))
    ny = max(1, int((maxy - miny) // pitch_y))
    if nx * ny > _MAX_CELLS:
        raise ValueError(
            "houndstooth grid would build %d cells (cap %d); increase cell or "
            "shrink the panel" % (nx * ny, _MAX_CELLS))
    x0 = -((nx - 1) * pitch_x) / 2.0
    y0 = -((ny - 1) * pitch_y) / 2.0
    segs = []
    half = pitch_x / 2.0
    # Corner of each L sits this far in from the cell centre, so the two
    # hooks stay apart and the re-entrant poke has room to open.
    arm = 0.22 * cell
    poke = 0.14 * cell
    for cx, cy in _grid_centres(nx, ny, pitch_x, pitch_y, x0, y0):
        segs.append(sg.LineString([
            (cx - half, cy),
            (cx - arm, cy),
            (cx - arm - poke, cy - poke),
            (cx, cy - arm),
            (cx, cy - half),
        ]))
        segs.append(sg.LineString([
            (cx + half, cy),
            (cx + arm, cy),
            (cx + arm + poke, cy + poke),
            (cx, cy + arm),
            (cx, cy + half),
        ]))
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


def _anti_tetrachiral_unit(bounds, cell, strut_t, node_r):
    """Build anti-tetrachiral cells: square-grid nodes, alternating tangents.

    Every interior node has four ligaments to its orthogonal neighbours. A
    checkerboard sign chooses which parallel tangent joins each pair: all four
    contacts around one node torque it in one sense, while the contacts around
    each neighbouring node torque it in the opposite sense. This alternating
    rotation is the anti-tetrachiral mechanism and distinguishes it from the
    same-sense triangular-grid chiral topology.
    """
    pitch_x = cell
    pitch_y = cell
    minx, miny, maxx, maxy = bounds
    nx = max(2, int((maxx - minx - 2.0 * node_r) // pitch_x) + 1)
    ny = max(2, int((maxy - miny - 2.0 * node_r) // pitch_y) + 1)
    if nx * ny > _MAX_CELLS:
        raise ValueError(
            "anti_tetrachiral grid would build %d nodes (cap %d); increase "
            "cell or shrink the panel" % (nx * ny, _MAX_CELLS))
    x0 = -((nx - 1) * pitch_x) / 2.0
    y0 = -((ny - 1) * pitch_y) / 2.0
    nodes = {
        (i, j): (x0 + i * pitch_x, y0 + j * pitch_y)
        for j in range(ny)
        for i in range(nx)
    }
    ligaments = []
    for (i, j), (x0n, y0n) in nodes.items():
        tangent_side = 1.0 if (i + j) % 2 == 0 else -1.0
        for di, dj in ((1, 0), (0, 1)):
            other = (i + di, j + dj)
            if other not in nodes:
                continue
            x1, y1 = nodes[other]
            dx, dy = x1 - x0n, y1 - y0n
            length = math.hypot(dx, dy)
            ux, uy = dx / length, dy / length
            px = -uy * node_r * tangent_side
            py = ux * node_r * tangent_side
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
    PETG stretched along X measurably widens along Y too. Seven topologies
    are supported. ``"reentrant"`` is a bowtie/inverted-honeycomb lattice
    (concave hexagon cells whose splayed struts hinge open under tension).
    ``"rotating_squares"`` is rigid square islands joined only at their
    corners by short living hinges, so the squares rotate rather than stretch
    (the classic Grima-Evans mechanism); its corner hinge is the fatigue
    weak point and defaults to a thinner ``hinge_t=0.6`` mm on purpose, well
    below the library's normal 0.8 mm minimum-wall rule, because that IS the
    compliant feature. ``"arrowhead"`` uses opposing inverted-triangle
    arrowheads sharing a waist; pulling along X opens their concave notches
    and expands the panel along Y. ``"star"`` tiles Grima six-pointed
    hexagram outlines on a hexagonal lattice; pulling a row opens the
    re-entrant star vertices and spreads adjacent rows. ``"chiral"`` is a
    hexagonal grid of
    circular nodes joined by ligaments tangent (not radial) to each node, so
    pulling the panel spins every node in the same rotational sense.
    ``"anti_tetrachiral"`` instead places circular nodes on a square grid and
    alternates the tangent side in a checkerboard, so neighbouring nodes
    rotate in opposite senses. ``"houndstooth"`` tiles interlocking L-shaped
    broken-chevrons whose inner corners are re-entrant, so stretching along
    X opens the notches and expands the panel along Y.

    All modes fill a solid ``border``-wide frame around the tiled interior so
    the panel edge is a continuous rim, never a row of half-cut cells; the
    frame is fitted to the tiled cells' actual extent (reported in
    ``metadata["border_actual"]``, always >= the requested ``border``)
    because an integer cell count rarely divides the interior exactly.
    ``strut_t`` (and, for chiral or anti-tetrachiral, the ligament width)
    must print as clean
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
    elif mode == "arrowhead":
        material, nx, ny, px, py = _arrowhead_unit(bounds, cell, strut_t)
        overlap = strut_t
        extra = {}
    elif mode == "star":
        material, nx, ny, px, py = _star_unit(bounds, cell, strut_t)
        overlap = strut_t
        extra = {}
    elif mode == "houndstooth":
        material, nx, ny, px, py = _houndstooth_unit(bounds, cell, strut_t)
        overlap = strut_t
        extra = {}
    elif mode == "anti_tetrachiral":
        node_r_eff = node_r if node_r is not None else 0.3 * cell
        if node_r_eff <= strut_t:
            raise ValueError(
                "auxetic_panel(): node_r must exceed strut_t so ligaments "
                "stay tangent instead of crossing the node")
        material, nx, ny, px, py, n_nodes = _anti_tetrachiral_unit(
            bounds, cell, strut_t, node_r_eff)
        overlap = node_r_eff
        extra = {"node_r": node_r_eff, "node_count": n_nodes}
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
    # Drop GEOS sliver interiors so the 2D hole count matches the extruded
    # solid's genus (platform GEOS builds differ on which slivers survive).
    combined = _prune_holes(combined)
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
# Regular honeycomb (positive Poisson)
# ---------------------------------------------------------------------------

def _flat_top_hex(cx, cy, across_flats):
    """Regular hexagon, flats on top and bottom, across-flats width ``across_flats``.

    Vertices sit at 0/60/120 deg (flat-top), matching ``patterns.lighten_cell_poly``
    hex orientation. Across-flats along Y is ``across_flats``; vertex radius is
    ``across_flats / sqrt(3)``.
    """
    r = across_flats / math.sqrt(3.0)
    angs = [math.radians(60.0 * i) for i in range(6)]
    return sg.Polygon([(cx + r * math.cos(a), cy + r * math.sin(a)) for a in angs])


def _honeycomb_unit(bounds, cell, strut_t):
    """Place flat-top hex holes on a regular hexagonal lattice inside ``bounds``.

    ``cell`` is the centre-to-centre pitch and the across-flats of the wall
    centreline. Each hole is the same hex inset by ``strut_t / 2`` so a shared
    wall between two cells prints at ``strut_t``. Columns use odd-q offset
    coordinates: odd columns shift up by half a pitch so every interior wall
    is shared, not a gap. Returns the hole polygons (not the walls) so the
    caller can punch them from a solid rectangular panel and keep a continuous
    rim instead of a row of half-cells.
    """
    # Flat-top: vertical across-flats = cell, horizontal neighbour at 1.5*R.
    pitch_x = cell * math.sqrt(3.0) / 2.0
    pitch_y = cell
    inner_af = cell - strut_t
    if inner_af <= 0:
        raise ValueError(
            "honeycomb_panel(): cell=%.3g mm is not larger than strut_t=%.3g mm; "
            "the cells would fuse solid" % (cell, strut_t))
    inner_r = inner_af / math.sqrt(3.0)
    hole_ext_x = inner_r
    hole_ext_y = inner_af / 2.0
    minx, miny, maxx, maxy = bounds
    inner_w = (maxx - minx) - 2.0 * hole_ext_x
    inner_h = (maxy - miny) - 2.0 * hole_ext_y
    if inner_w <= 0 or inner_h <= 0:
        raise ValueError(
            "honeycomb_panel(): no hex cell fits inside the bordered interior; "
            "increase the panel or shrink cell/border")
    nx = max(1, int(inner_w // pitch_x) + 1)
    ny = max(1, int(inner_h // pitch_y) + 1)
    if nx * ny > _MAX_CELLS:
        raise ValueError(
            "honeycomb grid would build %d cells (cap %d); increase cell or "
            "shrink the panel" % (nx * ny, _MAX_CELLS))
    x0 = -((nx - 1) * pitch_x) / 2.0
    y0 = -((ny - 1) * pitch_y) / 2.0
    interior = sg.box(minx, miny, maxx, maxy)
    holes = []
    for i in range(nx):
        col_off = (pitch_y / 2.0) if (i % 2) else 0.0
        n_row = ny - 1 if (i % 2 and ny > 1) else ny
        for j in range(n_row):
            hole = _flat_top_hex(
                x0 + i * pitch_x, y0 + j * pitch_y + col_off, inner_af)
            if interior.contains(hole):
                holes.append(hole)
    if not holes:
        raise ValueError(
            "honeycomb_panel(): no hex cell fits inside the bordered interior; "
            "increase the panel or shrink cell/border")
    return holes, nx, ny, pitch_x, pitch_y


def honeycomb_panel(width=60.0, height=60.0, thickness=3.0, cell=12.0,
                    strut_t=1.2, border=3.0, nozzle=0.4, snap_strut=True):
    """Build a flat regular-hex honeycomb panel (positive Poisson, lightening).

    A rectangular slab with a grid of through-holes on a regular hexagonal
    lattice. The cells are **flat-top** (two sides horizontal): ``cell`` is
    both the centre-to-centre pitch and the across-flats of the wall
    centreline, so a shared wall between two cells is ``strut_t`` thick.
    Stretching the panel in-plane makes it thinner in the transverse
    direction (positive Poisson's ratio), the opposite of ``auxetic_panel``.
    Use this for lightening a printed sheet, not for auxetic expansion.

    A solid ``border``-wide rim is left around the tiled holes so the panel
    edge is never a row of half-cells; the actual rim (panel edge to the
    nearest hole) is reported in ``metadata["border_actual"]`` and is always
    >= the requested ``border``, because an integer cell count rarely fills
    the interior exactly. ``strut_t`` is snapped to the nearest integer
    multiple of ``nozzle`` (one of 0.4 / 0.8 / 1.2 mm) by default; pass
    ``snap_strut=False`` to get a ``ValueError`` with the corrected value
    instead of silent snapping. The panel sits flat with its bottom face at
    z=0 and centred on the XY origin; print it flat, cell layer down, no
    supports needed. Cell counts are capped so a default-sized panel builds
    in well under a second. Units are mm.
    """
    if width <= 0 or height <= 0 or thickness < 0.8:
        raise ValueError("honeycomb_panel(): width/height must be positive and "
                          "thickness must be at least 0.8 mm")
    if cell <= 0:
        raise ValueError("honeycomb_panel(): cell must be positive")
    if border <= 0 or border >= min(width, height) / 2.0:
        raise ValueError("honeycomb_panel(): border must be positive and less "
                          "than half the panel's shorter side")
    _validate_nozzle(nozzle)
    strut_t = _snap_strut(strut_t, nozzle, snap_strut, "strut_t")
    if cell < 4.0 * strut_t:
        raise ValueError(
            "honeycomb_panel(): cell=%.3g mm is too small for strut_t=%.3g mm; "
            "the struts would fuse solid (need cell >= 4*strut_t)"
            % (cell, strut_t))

    bounds = _usable_bounds(width, height, border)
    holes, nx, ny, px, py = _honeycomb_unit(bounds, cell, strut_t)
    hole_union = unary_union(holes)
    hminx, hminy, hmaxx, hmaxy = hole_union.bounds
    border_actual = min(width / 2.0 + hminx, width / 2.0 - hmaxx,
                        height / 2.0 + hminy, height / 2.0 - hmaxy)
    if border_actual < border - 1e-6:
        raise ValueError(
            "honeycomb_panel(): tiled cells overflow the requested border "
            "(actual %.3g mm < requested %.3g mm); increase border or "
            "shrink cell" % (border_actual, border))

    panel = sg.box(-width / 2.0, -height / 2.0, width / 2.0, height / 2.0)
    combined = panel.difference(hole_union)
    if combined.geom_type not in ("Polygon", "MultiPolygon"):
        raise ValueError("honeycomb_panel(): degenerate panel geometry")
    combined = _prune_holes(combined)
    hole_count = _count_holes(combined)

    mesh = _extrude(combined, thickness)
    mesh.metadata.update({
        "mode": "flat_top",
        "cell_size": cell,
        "cells_x": nx,
        "cells_y": ny,
        "cell_count": len(holes),
        "pitch_x": px,
        "pitch_y": py,
        "strut_t": strut_t,
        "border_actual": border_actual,
        "hole_count": hole_count,
        "poisson_ratio_sign": 1,
    })
    return mesh


# ---------------------------------------------------------------------------
# Kagome (trihexagonal) lightening sheet
# ---------------------------------------------------------------------------

def _kagome_hex(cx, cy, circum_r):
    """Regular hexagon, vertex on +X, circumradius ``circum_r``."""
    pts = [
        (cx + circum_r * math.cos(math.radians(60.0 * k)),
         cy + circum_r * math.sin(math.radians(60.0 * k)))
        for k in range(6)
    ]
    return sg.Polygon(pts)


def _kagome_unit(bounds, cell, strut_t):
    """Place hexagon + triangle through-holes of a Kagome (3.6.3.6) tiling.

    Coarse vertices sit on a triangular lattice of spacing ``cell``. The six
    edge midpoints around each vertex form a hexagon hole; the three midpoints
    of each coarse triangle form a triangle hole. Both families are inset by
    ``strut_t / 2`` so remaining walls print at ``strut_t``. Distinct from
    honeycomb_panel (hex holes only) and isogrid_panel (triangle holes only).
    """
    pitch_x = cell
    pitch_y = cell * math.sqrt(3.0) / 2.0
    minx, miny, maxx, maxy = bounds
    pad = cell / 2.0
    nx = max(2, int((maxx - minx - 2.0 * pad) // pitch_x) + 1)
    ny = max(2, int((maxy - miny - 2.0 * pad) // pitch_y) + 1)
    n_est = nx * ny * 3
    if n_est > _MAX_CELLS:
        raise ValueError(
            "kagome grid would build ~%d holes (cap %d); increase cell or "
            "shrink the panel" % (n_est, _MAX_CELLS))
    x0 = -((nx - 1) * pitch_x) / 2.0
    y0 = -((ny - 1) * pitch_y) / 2.0

    def _vert(i, j):
        return (x0 + i * pitch_x + (j % 2) * (pitch_x / 2.0),
                y0 + j * pitch_y)

    interior = sg.box(minx, miny, maxx, maxy)
    inset = strut_t / 2.0
    hex_holes = []
    tri_holes = []
    for j in range(ny):
        for i in range(nx):
            vx, vy = _vert(i, j)
            hex_hole = _kagome_hex(vx, vy, cell / 2.0).buffer(-inset)
            if (hex_hole is not None and not hex_hole.is_empty
                    and hex_hole.geom_type in ("Polygon", "MultiPolygon")
                    and interior.contains(hex_hole)):
                hex_holes.append(hex_hole)

    def _mid(p, q):
        return ((p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0)

    def _add_tri(p0, p1, p2):
        tri = sg.Polygon([_mid(p0, p1), _mid(p1, p2), _mid(p2, p0)])
        tri = tri.buffer(-inset)
        if (tri is not None and not tri.is_empty
                and tri.geom_type in ("Polygon", "MultiPolygon")
                and interior.contains(tri)):
            tri_holes.append(tri)

    for j in range(ny - 1):
        for i in range(nx - 1):
            a = _vert(i, j)
            b = _vert(i + 1, j)
            c = _vert(i, j + 1)
            d = _vert(i + 1, j + 1)
            if j % 2 == 0:
                _add_tri(a, b, c)
                _add_tri(b, c, d)
            else:
                _add_tri(a, b, d)
                _add_tri(a, c, d)

    holes = hex_holes + tri_holes
    if not holes:
        raise ValueError(
            "kagome_panel(): no Kagome cell fits inside the bordered interior; "
            "increase the panel or shrink cell/border")
    return holes, nx, ny, pitch_x, pitch_y, len(hex_holes), len(tri_holes)


def kagome_panel(width=60.0, height=60.0, thickness=3.0, cell=12.0,
                 strut_t=1.2, border=3.0, nozzle=0.4, snap_strut=True):
    """Build a flat Kagome (trihexagonal) lightening panel.

    A rectangular slab with through-holes of **both** equilateral triangles
    and regular hexagons (the 3.6.3.6 Kagome tiling). ``cell`` is the coarse
    triangular-lattice vertex pitch (hex-hole centre spacing); ``strut_t`` is
    the printed wall between a triangle and a neighbouring hex. Distinct from
    ``honeycomb_panel`` (hex holes only) and ``isogrid_panel`` (triangle holes
    only). Stretching the panel in-plane makes it thinner transversely
    (positive Poisson). Use this for lightening a printed sheet.

    A solid ``border``-wide rim is left around the tiled holes so the panel
    edge is never a row of half-cells; the actual rim is reported in
    ``metadata["border_actual"]``. ``strut_t`` snaps to the nearest integer
    multiple of ``nozzle`` (0.4 / 0.8 / 1.2 mm) by default. The panel sits
    flat with its bottom face at z=0. Print flat, no supports. Units are mm.
    """
    if width <= 0 or height <= 0 or thickness < 0.8:
        raise ValueError("kagome_panel(): width/height must be positive and "
                          "thickness must be at least 0.8 mm")
    if cell <= 0:
        raise ValueError("kagome_panel(): cell must be positive")
    if border <= 0 or border >= min(width, height) / 2.0:
        raise ValueError("kagome_panel(): border must be positive and less "
                          "than half the panel's shorter side")
    _validate_nozzle(nozzle)
    strut_t = _snap_strut(strut_t, nozzle, snap_strut, "strut_t")
    if cell < 4.0 * strut_t:
        raise ValueError(
            "kagome_panel(): cell=%.3g mm is too small for strut_t=%.3g mm; "
            "the struts would fuse solid (need cell >= 4*strut_t)"
            % (cell, strut_t))

    bounds = _usable_bounds(width, height, border)
    holes, nx, ny, px, py, n_hex, n_tri = _kagome_unit(bounds, cell, strut_t)
    hole_union = unary_union(holes)
    hminx, hminy, hmaxx, hmaxy = hole_union.bounds
    border_actual = min(width / 2.0 + hminx, width / 2.0 - hmaxx,
                        height / 2.0 + hminy, height / 2.0 - hmaxy)
    if border_actual < border - 1e-6:
        raise ValueError(
            "kagome_panel(): tiled cells overflow the requested border "
            "(actual %.3g mm < requested %.3g mm); increase border or "
            "shrink cell" % (border_actual, border))

    panel = sg.box(-width / 2.0, -height / 2.0, width / 2.0, height / 2.0)
    combined = panel.difference(hole_union)
    if combined.geom_type not in ("Polygon", "MultiPolygon"):
        raise ValueError("kagome_panel(): degenerate panel geometry")
    combined = _prune_holes(combined)
    hole_count = _count_holes(combined)

    mesh = _extrude(combined, thickness)
    mesh.metadata.update({
        "mode": "kagome",
        "cell_size": cell,
        "cells_x": nx,
        "cells_y": ny,
        "cell_count": len(holes),
        "hex_holes": n_hex,
        "tri_holes": n_tri,
        "pitch_x": px,
        "pitch_y": py,
        "strut_t": strut_t,
        "border_actual": border_actual,
        "hole_count": hole_count,
        "poisson_ratio_sign": 1,
    })
    return mesh


# ---------------------------------------------------------------------------
# NASA-style isogrid (triangular through-ribs)
# ---------------------------------------------------------------------------

def _eq_triangle(cx, cy, side, up=True):
    """Equilateral triangle centred at ``(cx, cy)``. ``up`` points the apex +Y."""
    r = side / math.sqrt(3.0)
    if up:
        verts = (
            (cx, cy + r),
            (cx - side / 2.0, cy - r / 2.0),
            (cx + side / 2.0, cy - r / 2.0),
        )
    else:
        verts = (
            (cx, cy - r),
            (cx - side / 2.0, cy + r / 2.0),
            (cx + side / 2.0, cy + r / 2.0),
        )
    return sg.Polygon(verts)


def _isogrid_unit(bounds, cell, strut_t):
    """Place equilateral-triangle through-holes on a triangular lattice.

    ``cell`` is the wall-centreline triangle side and the vertex pitch. Ribs
    run at 0/60/120 deg. Each hole is the same triangle inset by
    ``strut_t / 2`` so a shared wall prints at ``strut_t``. Both up- and
    down-pointing cells are punched so the remaining material is the
    NASA-style isogrid (not a flat-top hex honeycomb). Returns hole
    polygons that sit fully inside ``bounds``.
    """
    inner_s = cell - strut_t * math.sqrt(3.0)
    if inner_s <= 0:
        raise ValueError(
            "isogrid_panel(): cell=%.3g mm is not larger than strut_t=%.3g mm; "
            "the cells would fuse solid" % (cell, strut_t))
    pitch_x = cell
    pitch_y = cell * math.sqrt(3.0) / 2.0
    r_inner = inner_s / math.sqrt(3.0)
    hole_ext_x = inner_s / 2.0
    hole_ext_y = r_inner
    minx, miny, maxx, maxy = bounds
    inner_w = (maxx - minx) - 2.0 * hole_ext_x
    inner_h = (maxy - miny) - 2.0 * hole_ext_y
    if inner_w <= 0 or inner_h <= 0:
        raise ValueError(
            "isogrid_panel(): no triangle cell fits inside the bordered "
            "interior; increase the panel or shrink cell/border")
    n_cols = max(2, int(inner_w // pitch_x) + 2)
    n_rows = max(2, int(inner_h // pitch_y) + 2)
    n_est = (n_cols - 1) * (n_rows - 1) * 2
    if n_est > _MAX_CELLS:
        raise ValueError(
            "isogrid grid would build %d cells (cap %d); increase cell or "
            "shrink the panel" % (n_est, _MAX_CELLS))
    x0 = -((n_cols - 1) * pitch_x) / 2.0
    y0 = -((n_rows - 1) * pitch_y) / 2.0
    interior = sg.box(minx, miny, maxx, maxy)

    def _vert(i, j):
        return (x0 + i * pitch_x + (j % 2) * (pitch_x / 2.0),
                y0 + j * pitch_y)

    holes = []
    for j in range(n_rows - 1):
        for i in range(n_cols - 1):
            a = _vert(i, j)
            b = _vert(i + 1, j)
            c = _vert(i, j + 1)
            d = _vert(i + 1, j + 1)
            if j % 2 == 0:
                pairs = ((a, b, c, True), (b, c, d, False))
            else:
                pairs = ((a, b, d, True), (a, c, d, False))
            for p0, p1, p2, up in pairs:
                cx = (p0[0] + p1[0] + p2[0]) / 3.0
                cy = (p0[1] + p1[1] + p2[1]) / 3.0
                hole = _eq_triangle(cx, cy, inner_s, up=up)
                if interior.contains(hole):
                    holes.append(hole)
    if not holes:
        raise ValueError(
            "isogrid_panel(): no triangle cell fits inside the bordered "
            "interior; increase the panel or shrink cell/border")
    nx = n_cols - 1
    ny = n_rows - 1
    return holes, nx, ny, pitch_x, pitch_y


def isogrid_panel(width=60.0, height=60.0, thickness=3.0, cell=12.0,
                  strut_t=1.2, border=3.0, nozzle=0.4, snap_strut=True):
    """Build a NASA-style isogrid panel (triangular through-cells, rib sheet).

    A rectangular slab with a grid of equilateral-triangle through-holes.
    The remaining material is ribs at 0/60/120 deg: ``cell`` is the
    wall-centreline triangle side and the vertex pitch, and ``strut_t`` is
    the printed rib thickness. Distinct from ``honeycomb_panel``, whose
    cells are flat-top hexagons.

    A solid ``border``-wide rim is left around the tiled holes so the panel
    edge is never a row of half-cells; the actual rim (panel edge to the
    nearest hole) is reported in ``metadata["border_actual"]`` and is always
    >= the requested ``border``, because an integer cell count rarely fills
    the interior exactly. ``strut_t`` is snapped to the nearest integer
    multiple of ``nozzle`` (one of 0.4 / 0.8 / 1.2 mm) by default; pass
    ``snap_strut=False`` to get a ``ValueError`` with the corrected value
    instead of silent snapping. The panel sits flat with its bottom face at
    z=0 and centred on the XY origin; print it flat, cell layer down, no
    supports needed. Cell counts are capped so a default-sized panel builds
    in well under a second. Units are mm.
    """
    if width <= 0 or height <= 0 or thickness < 0.8:
        raise ValueError("isogrid_panel(): width/height must be positive and "
                          "thickness must be at least 0.8 mm")
    if cell <= 0:
        raise ValueError("isogrid_panel(): cell must be positive")
    if border <= 0 or border >= min(width, height) / 2.0:
        raise ValueError("isogrid_panel(): border must be positive and less "
                          "than half the panel's shorter side")
    _validate_nozzle(nozzle)
    strut_t = _snap_strut(strut_t, nozzle, snap_strut, "strut_t")
    if cell < 4.0 * strut_t:
        raise ValueError(
            "isogrid_panel(): cell=%.3g mm is too small for strut_t=%.3g mm; "
            "the struts would fuse solid (need cell >= 4*strut_t)"
            % (cell, strut_t))

    bounds = _usable_bounds(width, height, border)
    holes, nx, ny, px, py = _isogrid_unit(bounds, cell, strut_t)
    hole_union = unary_union(holes)
    hminx, hminy, hmaxx, hmaxy = hole_union.bounds
    border_actual = min(width / 2.0 + hminx, width / 2.0 - hmaxx,
                        height / 2.0 + hminy, height / 2.0 - hmaxy)
    if border_actual < border - 1e-6:
        raise ValueError(
            "isogrid_panel(): tiled cells overflow the requested border "
            "(actual %.3g mm < requested %.3g mm); increase border or "
            "shrink cell" % (border_actual, border))

    panel = sg.box(-width / 2.0, -height / 2.0, width / 2.0, height / 2.0)
    combined = panel.difference(hole_union)
    if combined.geom_type not in ("Polygon", "MultiPolygon"):
        raise ValueError("isogrid_panel(): degenerate panel geometry")
    combined = _prune_holes(combined)
    hole_count = _count_holes(combined)

    mesh = _extrude(combined, thickness)
    mesh.metadata.update({
        "mode": "triangle",
        "cell_size": cell,
        "cells_x": nx,
        "cells_y": ny,
        "cell_count": len(holes),
        "pitch_x": px,
        "pitch_y": py,
        "strut_t": strut_t,
        "border_actual": border_actual,
        "hole_count": hole_count,
        "poisson_ratio_sign": 1,
    })
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


def _wave_slit_polys(width, height, kerf, pitch, bridge, margin):
    """Return staggered sinusoidal kerf channels (LivingHingeGenerator Wave).

    Rows run along Y at ``pitch`` spacing in X, matching the lattice floors
    and cell cap. Each slit is a sine-wave centreline buffered to ``kerf``
    width, broken by ``bridge``-wide uncut webs. Odd rows are staggered half
    a cell along Y and phase-shifted 180 deg so adjacent waves nest.
    """
    usable_w = width - 2.0 * margin
    usable_h = height - 2.0 * margin
    n_rows = max(1, int(usable_w // pitch))
    seg_len = pitch - kerf
    n_slits = max(1, int(usable_h // (seg_len + bridge)))
    if n_rows * n_slits > _MAX_CELLS:
        raise ValueError(
            "kerf_bend_cutter(): wave would cut %d slits (cap %d); "
            "increase pitch/bridge or shrink the panel"
            % (n_rows * n_slits, _MAX_CELLS))
    x0 = -((n_rows - 1) * pitch) / 2.0
    y0 = -((n_slits - 1) * (seg_len + bridge)) / 2.0
    # Stay inside the pitch so neighbouring rows do not collide.
    amp = 0.22 * pitch
    n_pts = max(16, int(math.ceil(seg_len / 0.4)))
    polys = []
    for r in range(n_rows):
        row_x = x0 + r * pitch
        stagger = (seg_len + bridge) / 2.0 if (r % 2) else 0.0
        phase = math.pi if (r % 2) else 0.0
        for s in range(n_slits):
            y_c = y0 + s * (seg_len + bridge) + stagger
            y_lo = y_c - seg_len / 2.0
            y_hi = y_c + seg_len / 2.0
            if y_lo < -usable_h / 2.0 - 1e-6:
                continue
            if y_hi > usable_h / 2.0 + 1e-6:
                continue
            pts = []
            for i in range(n_pts + 1):
                t = i / float(n_pts)
                y = y_lo + t * seg_len
                x = row_x + amp * math.sin(2.0 * math.pi * t + phase)
                pts.append((x, y))
            poly = sg.LineString(pts).buffer(kerf / 2.0, cap_style=2)
            if not poly.is_empty:
                polys.append(poly)
    return polys, n_rows, n_slits


def _hex_slit_polys(width, height, kerf, pitch, bridge, margin):
    """Return hexagonal living-hinge edge slits (LivingHingeGenerator Hex / KM Hex).

    Flat-top hexagonal tiling, same orientation as ``honeycomb_panel``.
    ``pitch`` is hex centre-to-centre (across-flats of the wall centreline).
    Each hex **edge** is a kerf slit shortened so an uncut ``bridge`` remains
    at the vertices; edges run at 0/60/120 deg. This is a cutter of edge
    slits, not hex through-holes.
    """
    usable_w = width - 2.0 * margin
    usable_h = height - 2.0 * margin
    # Odd-q offset: same column/row pitches as ``_honeycomb_unit``.
    pitch_x = pitch * math.sqrt(3.0) / 2.0
    pitch_y = pitch
    n_cols = max(1, int(usable_w // pitch_x))
    n_rows = max(1, int(usable_h // pitch_y))
    n_slits_est = n_cols * n_rows * 3
    if n_slits_est > _MAX_CELLS:
        raise ValueError(
            "kerf_bend_cutter(): hex would cut %d slits (cap %d); "
            "increase pitch/bridge or shrink the panel"
            % (n_slits_est, _MAX_CELLS))
    x0 = -((n_cols - 1) * pitch_x) / 2.0
    y0 = -((n_rows - 1) * pitch_y) / 2.0
    r = pitch / math.sqrt(3.0)
    inset = bridge / 2.0
    usable = sg.box(-usable_w / 2.0, -usable_h / 2.0,
                    usable_w / 2.0, usable_h / 2.0)
    seen = set()
    polys = []
    for i in range(n_cols):
        col_off = (pitch_y / 2.0) if (i % 2) else 0.0
        n_row = n_rows - 1 if (i % 2 and n_rows > 1) else n_rows
        for j in range(n_row):
            cx = x0 + i * pitch_x
            cy = y0 + j * pitch_y + col_off
            verts = []
            for k in range(6):
                a = math.radians(60.0 * k)
                verts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
            for k in range(6):
                p1 = verts[k]
                p2 = verts[(k + 1) % 6]
                a = (round(p1[0], 6), round(p1[1], 6))
                b = (round(p2[0], 6), round(p2[1], 6))
                key = (a, b) if a < b else (b, a)
                if key in seen:
                    continue
                seen.add(key)
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                length = math.hypot(dx, dy)
                if length <= bridge + 1e-9:
                    continue
                ux, uy = dx / length, dy / length
                q1 = (p1[0] + ux * inset, p1[1] + uy * inset)
                q2 = (p2[0] - ux * inset, p2[1] - uy * inset)
                poly = sg.LineString([q1, q2]).buffer(kerf / 2.0, cap_style=2)
                if poly.is_empty:
                    continue
                if not usable.contains(poly):
                    continue
                polys.append(poly)
    return polys, n_cols, n_rows


def _cross_slit_polys(width, height, kerf, pitch, bridge, margin):
    """Return X-lattice living-hinge slits (LivingHingeGenerator Cross / KM Cross).

    Lattice-family bars along local Y (kerf-wide in X) plus diagonal arms
    from each bar endpoint at approximately 30 deg. Odd rows offset by
    half the Y-repeat so arms from adjacent rows cross into X
    intersections. Arm length is ~46% of bar length. Bars whose centres
    land on a usable-Y edge are truncated at their midpoint so the
    repeat fits inside the solid rim (``margin``).
    """
    usable_w = width - 2.0 * margin
    usable_h = height - 2.0 * margin
    n_rows = max(1, int(usable_w // pitch))
    bar_len = pitch - kerf
    y_repeat = bar_len + bridge
    n_repeats = max(1, int(usable_h // y_repeat))
    # Each unit is a bar plus up to four arms.
    n_est = n_rows * (n_repeats + 1) * 5
    if n_est > _MAX_CELLS:
        raise ValueError(
            "kerf_bend_cutter(): cross would cut %d slits (cap %d); "
            "increase pitch/bridge or shrink the panel"
            % (n_est, _MAX_CELLS))
    x0 = -((n_rows - 1) * pitch) / 2.0
    span = n_repeats * y_repeat
    y_min = -span / 2.0
    arm_len = 0.46 * bar_len
    # Keep arm buffers from fusing onto the bar so split() still sees
    # the 30/150 deg family as its own pieces.
    arm_inset = kerf
    usable = sg.box(-usable_w / 2.0, -usable_h / 2.0,
                    usable_w / 2.0, usable_h / 2.0)
    half_k = kerf / 2.0

    def _add_bar(polys, x, y0, y1):
        if y1 - y0 <= kerf:
            return
        poly = sg.box(x - half_k, y0, x + half_k, y1)
        if poly.is_empty:
            return
        if not usable.contains(poly):
            poly = poly.intersection(usable)
            if poly.is_empty or poly.geom_type not in ("Polygon", "MultiPolygon"):
                return
        polys.append(poly)

    def _add_arm(polys, x, y, angle_deg):
        if arm_len <= arm_inset + 1e-9:
            return
        a = math.radians(angle_deg)
        ca, sa = math.cos(a), math.sin(a)
        p1 = (x + ca * arm_inset, y + sa * arm_inset)
        p2 = (x + ca * arm_len, y + sa * arm_len)
        poly = sg.LineString([p1, p2]).buffer(half_k, cap_style=2)
        if poly.is_empty:
            return
        if not usable.contains(poly):
            return
        polys.append(poly)

    polys = []
    for r in range(n_rows):
        row_x = x0 + r * pitch
        stagger = (y_repeat / 2.0) if (r % 2) else 0.0
        # Even rows: bars centred on k * y_repeat, including the two
        # usable-Y edges (those two are truncated at their midpoint).
        # Odd rows: offset half a repeat; those centres sit inside.
        if r % 2:
            k0, k1 = 0, n_repeats
        else:
            k0, k1 = 0, n_repeats + 1
        for k in range(k0, k1):
            y_c = y_min + k * y_repeat + stagger
            y_lo = y_c - bar_len / 2.0
            y_hi = y_c + bar_len / 2.0
            at_lo_edge = (not (r % 2)) and k == 0
            at_hi_edge = (not (r % 2)) and k == n_repeats
            if at_lo_edge:
                y_lo = y_c  # midpoint truncation
            if at_hi_edge:
                y_hi = y_c
            if y_lo < -usable_h / 2.0 - 1e-6:
                y_lo = -usable_h / 2.0
            if y_hi > usable_h / 2.0 + 1e-6:
                y_hi = usable_h / 2.0
            if y_hi - y_lo <= kerf:
                continue
            _add_bar(polys, row_x, y_lo, y_hi)
            # Arms from remaining endpoints, ~30 deg from +X.
            if not at_lo_edge:
                _add_arm(polys, row_x, y_lo, -30.0)
                _add_arm(polys, row_x, y_lo, 210.0)
            if not at_hi_edge:
                _add_arm(polys, row_x, y_hi, 30.0)
                _add_arm(polys, row_x, y_hi, 150.0)
    return polys, n_rows, n_repeats


def _chevron_leg_length(pitch, bridge, kerf):
    """Size a 45° chevron leg so nested rows keep an uncut parallel strip.

    Neighbouring rows' legs run parallel. The uncut strip between those
    legs is at least ``kerf`` and preferably ``bridge``. If a longer leg
    would close that strip, shorten rather than overlap.
    """
    half = math.sqrt(2.0)

    def _from_web(web):
        perp = web + kerf
        delta = perp * half
        # Short-leg branch: half in-row repeat sits below ``pitch``, so
        # several chevrons still fit on a default panel.
        y_half = pitch - delta
        leg = (2.0 * y_half - bridge) / half
        if y_half > 0.0 and leg >= 2.0 * kerf:
            return leg
        y_half = pitch + delta
        leg = (2.0 * y_half - bridge) / half
        if leg >= 2.0 * kerf:
            return leg
        return None

    for web in (max(kerf, bridge), kerf):
        leg = _from_web(web)
        if leg is not None:
            return leg
    s = max(kerf, (pitch - kerf) / 2.0)
    return s * half


def _chevron_slit_polys(width, height, kerf, pitch, bridge, margin):
    """Return nested 45° arrowhead slits (LivingHingeGenerator Chevron).

    Each chevron is two 45° legs meeting at an apex, cut as one
    LineString of three points buffered to ``kerf``. Rows stack along
    local X at ``pitch`` (apex-to-apex). Chevrons in a row run along
    local Y. Even rows point +X, odd rows point -X, and odd rows shift
    by half the in-row repeat so neighbouring legs run parallel (nested
    interrupted zigzag). The in-row gap between neighbouring chevron
    ends is ``bridge``.
    """
    usable_w = width - 2.0 * margin
    usable_h = height - 2.0 * margin
    n_rows = max(1, int(usable_w // pitch))
    leg = _chevron_leg_length(pitch, bridge, kerf)
    s = leg / math.sqrt(2.0)
    y_span = 2.0 * s
    y_repeat = y_span + bridge
    n_repeats = max(1, int(usable_h // y_repeat))
    n_est = n_rows * (n_repeats + 1)
    if n_est > _MAX_CELLS:
        raise ValueError(
            "kerf_bend_cutter(): chevron would cut %d slits (cap %d); "
            "increase pitch/bridge or shrink the panel"
            % (n_est, _MAX_CELLS))
    x0 = -((n_rows - 1) * pitch) / 2.0
    y0 = -((n_repeats - 1) * y_repeat) / 2.0
    usable = sg.box(-usable_w / 2.0, -usable_h / 2.0,
                    usable_w / 2.0, usable_h / 2.0)
    half_k = kerf / 2.0
    polys = []
    for r in range(n_rows):
        row_x = x0 + r * pitch
        # Even: legs extend +X from the apex. Odd: legs extend -X, and
        # the row is shifted half a repeat so legs nest in parallel.
        sign = 1.0 if (r % 2 == 0) else -1.0
        stagger = (y_repeat / 2.0) if (r % 2) else 0.0
        for k in range(n_repeats):
            y_c = y0 + k * y_repeat + stagger
            apex = (row_x, y_c)
            upper = (row_x + sign * s, y_c + s)
            lower = (row_x + sign * s, y_c - s)
            poly = sg.LineString([upper, apex, lower]).buffer(
                half_k, cap_style=2)
            if poly.is_empty:
                continue
            if not usable.contains(poly):
                poly = poly.intersection(usable)
                if poly.is_empty:
                    continue
                if poly.geom_type == "MultiPolygon":
                    polys.extend(
                        g for g in poly.geoms
                        if g.geom_type == "Polygon" and not g.is_empty)
                    continue
                if poly.geom_type != "Polygon":
                    continue
            polys.append(poly)
    return polys, n_rows, n_repeats


def _diamond_slit_polys(width, height, kerf, pitch, bridge, margin):
    """Return elongated diamond-outline brick-wall slits (LivingHingeGenerator Diamond).

    Running-bond rhombi elongated along local Y (~2:1). ``pitch`` is the
    short-axis (X) centre-to-centre. Each diamond is four kerf edges
    shortened so an uncut ``bridge`` remains at the vertices (same
    edge-inset as ``_hex_slit_polys``). Odd rows offset by half the
    in-row pitch. Outlines, not filled diamonds.
    """
    usable_w = width - 2.0 * margin
    usable_h = height - 2.0 * margin
    # Diagonals: short = pitch (X), long = 2*pitch (Y). Row pitch is half
    # the long diagonal so neighbouring rhombi share an edge in a
    # running-bond tessellation.
    dx = pitch
    dy = 2.0 * pitch
    pitch_x = pitch
    pitch_y = dy / 2.0
    n_cols = max(1, int(usable_w // pitch_x))
    n_rows = max(1, int(usable_h // pitch_y))
    n_slits_est = n_cols * n_rows * 4
    if n_slits_est > _MAX_CELLS:
        raise ValueError(
            "kerf_bend_cutter(): diamond would cut %d slits (cap %d); "
            "increase pitch/bridge or shrink the panel"
            % (n_slits_est, _MAX_CELLS))
    x0 = -((n_cols - 1) * pitch_x) / 2.0
    y0 = -((n_rows - 1) * pitch_y) / 2.0
    inset = bridge / 2.0
    usable = sg.box(-usable_w / 2.0, -usable_h / 2.0,
                    usable_w / 2.0, usable_h / 2.0)
    seen = set()
    polys = []
    for j in range(n_rows):
        row_off = (pitch_x / 2.0) if (j % 2) else 0.0
        n_col = n_cols - 1 if (j % 2 and n_cols > 1) else n_cols
        for i in range(n_col):
            cx = x0 + i * pitch_x + row_off
            cy = y0 + j * pitch_y
            verts = (
                (cx, cy + dy / 2.0),
                (cx + dx / 2.0, cy),
                (cx, cy - dy / 2.0),
                (cx - dx / 2.0, cy),
            )
            for k in range(4):
                p1 = verts[k]
                p2 = verts[(k + 1) % 4]
                a = (round(p1[0], 6), round(p1[1], 6))
                b = (round(p2[0], 6), round(p2[1], 6))
                key = (a, b) if a < b else (b, a)
                if key in seen:
                    continue
                seen.add(key)
                dxe = p2[0] - p1[0]
                dye = p2[1] - p1[1]
                length = math.hypot(dxe, dye)
                if length <= bridge + 1e-9:
                    continue
                ux, uy = dxe / length, dye / length
                q1 = (p1[0] + ux * inset, p1[1] + uy * inset)
                q2 = (p2[0] - ux * inset, p2[1] - uy * inset)
                poly = sg.LineString([q1, q2]).buffer(kerf / 2.0, cap_style=2)
                if poly.is_empty:
                    continue
                if not usable.contains(poly):
                    continue
                polys.append(poly)
    return polys, n_rows, n_cols


def _fishbone_slit_polys(width, height, kerf, pitch, bridge, margin):
    """Return paired 45 deg fish-skeleton ribs (LivingHingeGenerator Fishbone).

    Columns lie along local Y and repeat across local X at ``pitch``. Each
    unit has separate 45/135 deg ribs that approach a virtual point on the
    Y spine without joining, leaving an uncut ``bridge`` there. Successive
    units reverse along Y to form a herringbone, while shortened rib tips
    preserve the web between neighbouring units and columns. This differs
    from chevron's continuous two-leg arrowheads and alternating columns.
    """
    usable_w = width - 2.0 * margin
    usable_h = height - 2.0 * margin
    n_rows = max(1, int(usable_w // pitch))
    n_units = max(1, int(usable_h // pitch))
    n_est = n_rows * n_units * 2
    if n_est > _MAX_CELLS:
        raise ValueError(
            "kerf_bend_cutter(): fishbone would cut %d slits (cap %d); "
            "increase pitch/bridge or shrink the panel"
            % (n_est, _MAX_CELLS))

    x0 = -((n_rows - 1) * pitch) / 2.0
    y0 = -((n_units - 1) * pitch) / 2.0
    # Buffer boundaries, rather than just centreline ends, retain roughly
    # ``bridge`` at the virtual spine and at neighbouring rib tips.
    inner = (bridge + kerf) / 2.0
    outer = (pitch - bridge - kerf) / 2.0
    if outer <= inner:
        raise ValueError(
            "kerf_bend_cutter(): fishbone pitch must exceed "
            "2 * (bridge + kerf) so each rib has positive length")
    usable = sg.box(-usable_w / 2.0, -usable_h / 2.0,
                    usable_w / 2.0, usable_h / 2.0)
    half_k = kerf / 2.0
    polys = []
    for r in range(n_rows):
        spine_x = x0 + r * pitch
        for k in range(n_units):
            spine_y = y0 + k * pitch
            y_sign = 1.0 if (k % 2 == 0) else -1.0
            for x_sign in (-1.0, 1.0):
                p1 = (spine_x + x_sign * inner,
                      spine_y + y_sign * inner)
                p2 = (spine_x + x_sign * outer,
                      spine_y + y_sign * outer)
                poly = sg.LineString([p1, p2]).buffer(
                    half_k, cap_style=2)
                if poly.is_empty:
                    continue
                if not usable.contains(poly):
                    poly = poly.intersection(usable)
                    if poly.is_empty:
                        continue
                    if poly.geom_type == "MultiPolygon":
                        polys.extend(
                            g for g in poly.geoms
                            if g.geom_type == "Polygon" and not g.is_empty)
                        continue
                    if poly.geom_type != "Polygon":
                        continue
                polys.append(poly)
    return polys, n_rows, n_units


def _meander_slit_polys(width, height, kerf, pitch, bridge, margin):
    """Return one continuous square-wave meander-labyrinth kerf.

    Parallel runs follow local Y at ``pitch`` spacing in X. Horizontal
    U-turns join successive runs at alternating ends into one serpentine
    LineString. The turn centreline is inset by ``bridge + kerf / 2`` from
    the usable panel boundary, preserving a full uncut bridge beyond the
    buffered slit at both margins.
    """
    usable_w = width - 2.0 * margin
    usable_h = height - 2.0 * margin
    n_runs = max(1, int(usable_w // pitch))
    if n_runs > _MAX_CELLS:
        raise ValueError(
            "kerf_bend_cutter(): meander would cut %d parallel runs "
            "(cap %d); increase pitch or shrink the panel"
            % (n_runs, _MAX_CELLS))

    half_k = kerf / 2.0
    y_extent = usable_h / 2.0 - bridge - half_k
    if y_extent <= 0:
        raise ValueError(
            "kerf_bend_cutter(): no meander slit fits inside the panel "
            "margins while retaining the requested bridge")

    x0 = -((n_runs - 1) * pitch) / 2.0
    points = []
    for r in range(n_runs):
        x = x0 + r * pitch
        y_start = -y_extent if (r % 2 == 0) else y_extent
        y_end = -y_start
        # Same-Y connection from the preceding run makes the square U-turn;
        # alternating run direction moves the next turn to the opposite end.
        points.append((x, y_start))
        points.append((x, y_end))

    poly = sg.LineString(points).buffer(
        half_k, cap_style=2, join_style=2)
    return [poly], n_runs, 1


def _biaxial_slit_polys(width, height, kerf, pitch, bridge, margin):
    """Return orthogonal interrupted-slit families for two-axis wrapping."""
    vertical, n_rows, n_slits = _lattice_slit_polys(
        width, height, kerf, pitch, bridge, margin, 0.0)
    horizontal, _, _ = _lattice_slit_polys(
        height, width, kerf, pitch, bridge, margin, 0.0)
    horizontal = [
        shapely.affinity.rotate(poly, 90.0, origin=(0, 0))
        for poly in horizontal
    ]
    polys = vertical + horizontal
    if len(polys) > _MAX_CELLS:
        raise ValueError(
            "kerf_bend_cutter(): biaxial would cut %d slits (cap %d); "
            "increase pitch/bridge or shrink the panel"
            % (len(polys), _MAX_CELLS))
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
    ``mode="wave"`` replaces the straight slits with sinusoidal living-hinge
    channels (LivingHingeGenerator Wave / KM Wave): same staggered
    row-and-bridge layout, but each kerf follows a sine so the hinge flexes
    along a wavy web instead of a straight lattice. ``mode="hex"`` cuts
    hexagonal living-hinge **edge slits** (LivingHingeGenerator Hex / KM Hex):
    each edge of a flat-top hex tiling is a kerf slit shortened so an uncut
    ``bridge`` remains at the vertices, in three orientations at 0/60/120 deg.
    ``pitch`` is hex centre-to-centre. This is not ``honeycomb_panel``'s
    positive hex through-holes. ``mode="cross"`` cuts an X-lattice of
    lattice-family bars plus ~30 deg diagonal arms (LivingHingeGenerator
    Cross / KM Cross): arms from each bar endpoint interlock across
    half-repeat-staggered rows into X intersections. ``mode="chevron"``
    cuts nested 45 deg arrowhead slits (LivingHingeGenerator Chevron):
    each chevron is one continuous cut of two 45 deg legs; rows
    alternate direction and half-pitch offset so they interlock into
    interrupted zigzag lines. ``mode="diamond"`` cuts elongated
    diamond-outline brick-wall slits (LivingHingeGenerator Diamond): each
    rhombus is four kerf edges with an uncut ``bridge`` at the vertices;
    odd rows offset by half the in-row pitch. Diamonds elongate along Y
    (~2:1); ``pitch`` is the short-axis centre-to-centre. ``mode="fishbone"``
    cuts herringbone living-hinge ribs (LivingHingeGenerator Fishbone):
    separate 45/135 deg rib pairs approach a local-Y spine but leave an
    uncut ``bridge`` at the spine and rib tips, so they do not become
    continuous chevrons or fuse into adjacent units. ``mode="meander"``
    cuts one continuous square-wave labyrinth: parallel local-Y runs are
    joined by square U-turns at alternating ends while an uncut ``bridge``
    remains at the panel margins. ``mode="biaxial"`` overlays two orthogonal
    interrupted lattice-slit families so the panel can roll about both X and
    Y while the bridges keep the slab connected.

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

    if mode == "wave":
        polys, n_rows, n_slits = _wave_slit_polys(
            width, height, kerf, pitch, bridge, margin)
    elif mode == "hex":
        polys, n_rows, n_slits = _hex_slit_polys(
            width, height, kerf, pitch, bridge, margin)
    elif mode == "cross":
        polys, n_rows, n_slits = _cross_slit_polys(
            width, height, kerf, pitch, bridge, margin)
    elif mode == "chevron":
        polys, n_rows, n_slits = _chevron_slit_polys(
            width, height, kerf, pitch, bridge, margin)
    elif mode == "diamond":
        polys, n_rows, n_slits = _diamond_slit_polys(
            width, height, kerf, pitch, bridge, margin)
    elif mode == "fishbone":
        polys, n_rows, n_slits = _fishbone_slit_polys(
            width, height, kerf, pitch, bridge, margin)
    elif mode == "meander":
        polys, n_rows, n_slits = _meander_slit_polys(
            width, height, kerf, pitch, bridge, margin)
    elif mode == "biaxial":
        polys, n_rows, n_slits = _biaxial_slit_polys(
            width, height, kerf, pitch, bridge, margin)
    else:
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


# ---------------------------------------------------------------------------
# 3D triply-periodic minimal-surface (TPMS) lattice
# ---------------------------------------------------------------------------

def _gyroid_sheet_sdf(x, y, z, width, depth, height, cell, wall):
    """Positive-inside approximate SDF for a box-clipped gyroid sheet."""
    zc = z - height / 2.0
    k = 2.0 * math.pi / cell
    sx, cx = math.sin(k * x), math.cos(k * x)
    sy, cy = math.sin(k * y), math.cos(k * y)
    sz, cz = math.sin(k * zc), math.cos(k * zc)

    # Schoen gyroid level set. Dividing by its physical-space gradient turns
    # the field into a first-order signed-distance estimate, so ``wall`` is a
    # millimetre thickness rather than an arbitrary isovalue.
    field = sx * cy + sy * cz + sz * cx
    gx = k * (cx * cy - sz * sx)
    gy = k * (cy * cz - sx * sy)
    gz = k * (cz * cx - sy * sz)
    gradient = max(math.sqrt(gx * gx + gy * gy + gz * gz), 1e-12)
    # The field/gradient quotient is first-order: across the finite 0.6+ mm
    # offset it underestimates physical thickness by about 5-8 percent. A
    # conservative factor keeps the realised interior sheet at or above the
    # requested printable wall instead of merely labelling a thinner mesh.
    sheet = (wall * _GYROID_DISTANCE_COMPENSATION) / 2.0 - abs(field) / gradient

    # Intersection with the requested rectangular block. Including this box
    # SDF closes every cut sheet edge at the six faces, unlike extracting the
    # gyroid field alone and then truncating its triangles.
    box = min(width / 2.0 - abs(x),
              depth / 2.0 - abs(y),
              height / 2.0 - abs(zc))
    return min(sheet, box)


def gyroid_lattice(width=24.0, depth=24.0, height=24.0, cell=12.0,
                   wall=1.2, resolution=16):
    """Build a rectangular open-cell TPMS gyroid sheet as one printable body.

    The zero set is Schoen's periodic gyroid equation, thickened equally on
    both sides into a sheet and clipped to ``width`` x ``depth`` x ``height``.
    This is a continuous curved wall separating two interpenetrating passage
    networks, not a strut graph or a solid block. X/Y are centred and the
    block sits on z=0. Units are mm.

    ``resolution`` is the number of level-set samples per ``cell``. It must be
    fine enough that the grid edge is no larger than ``wall``; the complexity
    cap rejects previews whose sampled volume would be too expensive for the
    browser playground. Metadata reports realised mesh dimensions, cell and
    wall, resolution, period counts, and relative density.
    """
    values = (("width", width), ("depth", depth), ("height", height),
              ("cell", cell), ("wall", wall))
    for name, value in values:
        if (isinstance(value, bool) or not isinstance(value, numbers.Real)
                or not math.isfinite(float(value))):
            raise ValueError(
                "gyroid_lattice(): %s must be a finite real number" % name)
        if value <= 0:
            raise ValueError("gyroid_lattice(): %s must be positive" % name)
    width, depth, height = float(width), float(depth), float(height)
    cell, wall = float(cell), float(wall)

    if cell < 8.0:
        raise ValueError(
            "gyroid_lattice(): cell=%.3g mm is below the 8.0 mm printable floor"
            % cell)
    if wall < 1.2:
        raise ValueError(
            "gyroid_lattice(): wall=%.3g mm is below the 1.2 mm printable floor"
            % wall)
    if wall > cell / 3.0:
        raise ValueError(
            "gyroid_lattice(): wall=%.3g mm is too thick for cell=%.3g mm; "
            "use wall <= cell/3 to preserve both open passage networks"
            % (wall, cell))
    for name, value in (("width", width), ("depth", depth),
                        ("height", height)):
        if value < cell:
            raise ValueError(
                "gyroid_lattice(): %s=%.3g mm must span at least one cell "
                "(%.3g mm)" % (name, value, cell))
    if (isinstance(resolution, bool) or not isinstance(resolution, int)
            or resolution < 8 or resolution > 32):
        raise ValueError(
            "gyroid_lattice(): resolution must be an integer from 8 to 32")
    edge = cell / resolution
    if edge > wall:
        raise ValueError(
            "gyroid_lattice(): resolution=%d is too low for cell=%.3g mm and "
            "wall=%.3g mm; need at least %d samples per cell"
            % (resolution, cell, wall, math.ceil(cell / wall)))

    grid_shape = tuple(math.ceil(size / edge) + 1
                       for size in (width, depth, height))
    sample_count = math.prod(grid_shape)
    if sample_count > _MAX_GYROID_SAMPLES:
        raise ValueError(
            "gyroid_lattice(): preview needs %d level-set samples (cap %d); "
            "shrink the block or lower resolution"
            % (sample_count, _MAX_GYROID_SAMPLES))

    def sdf(x, y, z):
        return _gyroid_sheet_sdf(
            x, y, z, width, depth, height, cell, wall)

    manifold = manifold3d.Manifold.level_set(
        sdf,
        [-width / 2.0, -depth / 2.0, 0.0,
         width / 2.0, depth / 2.0, height],
        edge,
    )
    mesh = from_manifold(manifold)
    if len(mesh.faces) == 0 or mesh.volume <= 0:
        raise ValueError(
            "gyroid_lattice(): parameters produced no printable sheet body")

    # A clipped periodic surface can leave tiny closed corner chips when the
    # box face crosses a lobe just beyond its neck. They are not part of the
    # printable connected sheet, so retain the dominant body deterministically.
    components = mesh.split(only_watertight=False)
    removed = max(0, len(components) - 1)
    if removed:
        mesh = max(components, key=lambda component: component.volume)
    if not mesh.is_watertight or not mesh.is_winding_consistent:
        mesh = from_manifold(to_manifold(mesh))

    realised = tuple(float(value) for value in mesh.extents)
    bbox_volume = width * depth * height
    mesh.metadata.update({
        "mode": "gyroid",
        "dimensions": realised,
        "width": realised[0],
        "depth": realised[1],
        "height": realised[2],
        "cell": cell,
        "cell_size": cell,
        "wall": wall,
        "wall_thickness": wall,
        "resolution": resolution,
        "cells_x": width / cell,
        "cells_y": depth / cell,
        "cells_z": height / cell,
        "sample_count": sample_count,
        "components_removed": removed,
        "relative_density": float(mesh.volume / bbox_volume),
    })
    return mesh


# ---------------------------------------------------------------------------
# 3D body-centred-cubic (BCC) strut lattice
# ---------------------------------------------------------------------------

_MAX_STRUT_CELLS = 64  # nx*ny*nz cap: a strut union past this is a heavy playground toy


def _strut(a, b, radius, sections):
    """A capped cylinder solid spanning two 3D points."""
    return trimesh.creation.cylinder(radius=radius, segment=[a, b], sections=sections)


def _cubic_graph(nx, ny, nz, cell):
    """Return shared grid nodes and unique axis-aligned cube-edge struts."""
    nodes = {
        (ix, iy, iz): np.array((ix, iy, iz), dtype=float) * cell
        for iz in range(nz + 1)
        for iy in range(ny + 1)
        for ix in range(nx + 1)
    }
    edges = set()
    for key in nodes:
        for axis in range(3):
            neighbour = list(key)
            neighbour[axis] += 1
            neighbour = tuple(neighbour)
            if neighbour in nodes:
                edges.add((key, neighbour))
    return nodes, edges


def cubic_lattice(nx=2, ny=2, nz=2, cell=12.0, strut_d=1.6, node_d=None,
                  sections=12, nozzle=0.4, snap_strut=True):
    """Build a simple-cubic 3D strut lattice as one watertight mesh.

    Shared nodes occupy the rectangular ``(nx+1) * (ny+1) * (nz+1)`` grid.
    Each node connects only to its in-bounds neighbour along X, Y, or Z, so
    every strut is one unique cube edge. There are no BCC body-centre nodes,
    body diagonals, or octet/FCC face diagonals.

    ``strut_d`` snaps to the ``nozzle`` grid by default. ``node_d`` defaults
    to 1.5 times the realised strut diameter and is never allowed below it.
    The block is centred in X/Y, its nominal lower nodes lie at z=0, and
    relative density is measured against the requested cell-block volume.
    Units are mm.
    """
    for name, val in (("nx", nx), ("ny", ny), ("nz", nz)):
        if isinstance(val, bool) or not isinstance(val, int) or val < 1:
            raise ValueError("cubic_lattice(): %s must be a positive integer" % name)
    if nx * ny * nz > _MAX_STRUT_CELLS:
        raise ValueError(
            "cubic_lattice(): nx*ny*nz=%d exceeds the %d-cell playground cap; "
            "a larger strut union is too heavy to preview interactively"
            % (nx * ny * nz, _MAX_STRUT_CELLS))
    if cell <= 0:
        raise ValueError("cubic_lattice(): cell must be positive")
    if isinstance(sections, bool) or not isinstance(sections, int) or sections < 6:
        raise ValueError("cubic_lattice(): sections must be an integer at least 6")
    _validate_nozzle(nozzle)
    strut_d = _snap_strut(strut_d, nozzle, snap_strut, "strut_d")
    if strut_d < 0.8:
        raise ValueError(
            "cubic_lattice(): strut_d=%.3g mm is below the 0.8 mm FDM wall floor; "
            "use a bigger strut or a wider nozzle" % strut_d)
    if node_d is None:
        node_d = 1.5 * strut_d
    elif node_d <= 0:
        raise ValueError("cubic_lattice(): node_d must be positive")
    node_d = max(node_d, strut_d)
    min_cell = max(3.0 * strut_d, 2.0 * node_d)
    if cell < min_cell:
        raise ValueError(
            "cubic_lattice(): cell=%.3g mm is too small for strut_d=%.3g mm "
            "and node_d=%.3g mm; the square openings collapse (need cell >= %.3g mm)"
            % (cell, strut_d, node_d, min_cell))

    nodes, edges = _cubic_graph(nx, ny, nz, cell)
    solids = [
        _strut(nodes[a], nodes[b], strut_d / 2.0, sections)
        for a, b in sorted(edges)
    ]
    for pos in nodes.values():
        joint = trimesh.creation.icosphere(subdivisions=1, radius=node_d / 2.0)
        joint.apply_translation(pos)
        solids.append(joint)

    mesh = uni(solids)
    if not mesh.is_watertight or not mesh.is_winding_consistent:
        mesh = from_manifold(to_manifold(mesh))
    mesh.apply_translation((-nx * cell / 2.0, -ny * cell / 2.0, 0.0))

    bbox_vol = (nx * cell) * (ny * cell) * (nz * cell)
    mesh.metadata.update({
        "mode": "cubic",
        "cell_size": cell,
        "cells_x": nx,
        "cells_y": ny,
        "cells_z": nz,
        "cell_count": nx * ny * nz,
        "strut_count": len(edges),
        "node_count": len(nodes),
        "strut_d": strut_d,
        "node_d": node_d,
        "relative_density": float(mesh.volume / bbox_vol) if bbox_vol else 0.0,
    })
    return mesh


def bcc_lattice(nx=2, ny=2, nz=2, cell=12.0, strut_d=1.6, node_d=None,
                sections=12, nozzle=0.4, snap_strut=True):
    """Build a 3D body-centred-cubic (BCC) strut lattice as one watertight mesh.

    Each cubic ``cell`` carries eight round struts running from its eight
    corner nodes to a single body-centre node (the eight half body-diagonals),
    and corner nodes are shared with the neighbouring cells so the tiled block
    is one continuous space-frame. This is the classic printed metamaterial
    truss used for lightweight cores, energy-absorbing crush structures, and
    stiffness-tuned infill blocks -- distinct from the flat 2D lightening
    sheets (``honeycomb_panel`` / ``isogrid_panel`` / ``kagome_panel``), which
    are single-layer extrusions, not a volumetric strut network.

    ``strut_d`` is the round strut diameter; it snaps to the nearest integer
    multiple of ``nozzle`` (0.4 / 0.8 / 1.2 mm) by default so a strut prints as
    a clean 1x/2x/3x extrusion stack rather than a wobbly single pass (pass
    ``snap_strut=False`` to get a ``ValueError`` with the corrected value).
    ``node_d`` is the joint-sphere diameter that blends the eight struts at a
    node; it defaults to ``1.5 * strut_d`` and is clamped to be at least
    ``strut_d`` so a joint is never thinner than the struts it fuses.

    The block is centred on the origin in X and Y and sits with its bottom
    nodes at ``z=0``. ``metadata`` reports the realised strut/node diameters,
    the cell counts, the strut count, and the ``relative_density`` (solid
    volume / bounding-box volume) -- the number you tune a lattice core by.

    Note on print orientation: the half body-diagonal struts rise at ~35 deg
    from the print bed, a shallow overhang, so a BCC block usually wants either
    a slow-overhang profile or light support; that is a slicer choice, not a
    geometry defect. Units are mm.
    """
    for name, val in (("nx", nx), ("ny", ny), ("nz", nz)):
        if not isinstance(val, int) or val < 1:
            raise ValueError("bcc_lattice(): %s must be a positive integer" % name)
    if nx * ny * nz > _MAX_STRUT_CELLS:
        raise ValueError(
            "bcc_lattice(): nx*ny*nz=%d exceeds the %d-cell playground cap; a "
            "larger strut union is too heavy to preview interactively"
            % (nx * ny * nz, _MAX_STRUT_CELLS))
    if cell <= 0:
        raise ValueError("bcc_lattice(): cell must be positive")
    if sections < 6:
        raise ValueError("bcc_lattice(): sections must be at least 6 for a round strut")
    _validate_nozzle(nozzle)
    strut_d = _snap_strut(strut_d, nozzle, snap_strut, "strut_d")
    if strut_d < 0.8:
        raise ValueError(
            "bcc_lattice(): strut_d=%.3g mm is below the 0.8 mm FDM wall floor; "
            "use a bigger strut or a wider nozzle" % strut_d)
    if node_d is None:
        node_d = 1.5 * strut_d
    node_d = max(node_d, strut_d)
    # Struts must not fuse into a solid: the closest approach between two
    # half-diagonals sharing a corner is what limits how fine the cell can go.
    if cell < 3.0 * strut_d:
        raise ValueError(
            "bcc_lattice(): cell=%.3g mm is too small for strut_d=%.3g mm; the "
            "struts fuse into a solid block (need cell >= 3*strut_d)"
            % (cell, strut_d))

    strut_r = strut_d / 2.0
    node_r = node_d / 2.0

    # Node coordinates. Corner nodes on the (nx+1)x(ny+1)x(nz+1) integer grid;
    # body-centre nodes at each cell's middle. A dict de-duplicates the shared
    # corners so the union is over unique solids only.
    nodes = {}

    def node_key(ix, iy, iz):
        # half-integer indexing (x2) keeps corner and centre keys exact ints
        return (ix, iy, iz)

    for k in range(nz + 1):
        for j in range(ny + 1):
            for i in range(nx + 1):
                key = node_key(2 * i, 2 * j, 2 * k)
                nodes[key] = (i * cell, j * cell, k * cell)

    struts = []
    for k in range(nz):
        for j in range(ny):
            for i in range(nx):
                cx, cy, cz = (i + 0.5) * cell, (j + 0.5) * cell, (k + 0.5) * cell
                ckey = node_key(2 * i + 1, 2 * j + 1, 2 * k + 1)
                nodes[ckey] = (cx, cy, cz)
                centre = np.array([cx, cy, cz])
                for dk in (0, 1):
                    for dj in (0, 1):
                        for di in (0, 1):
                            corner = np.array([(i + di) * cell,
                                               (j + dj) * cell,
                                               (k + dk) * cell])
                            struts.append((corner, centre))

    # Solids: one capped cylinder per strut + one joint sphere per unique node.
    solids = [_strut(a, b, strut_r, sections) for (a, b) in struts]
    for pos in nodes.values():
        solids.append(trimesh.creation.icosphere(subdivisions=1, radius=node_r))
        solids[-1].apply_translation(pos)

    mesh = uni(solids)
    if not mesh.is_watertight:
        mesh = from_manifold(to_manifold(mesh))

    # Centre in X/Y, drop bottom nodes to z=0.
    mesh.apply_translation((-nx * cell / 2.0, -ny * cell / 2.0, 0.0))

    bbox_vol = (nx * cell) * (ny * cell) * (nz * cell)
    mesh.metadata.update({
        "mode": "bcc",
        "cell_size": cell,
        "cells_x": nx,
        "cells_y": ny,
        "cells_z": nz,
        "cell_count": nx * ny * nz,
        "strut_count": len(struts),
        "node_count": len(nodes),
        "strut_d": strut_d,
        "node_d": node_d,
        "relative_density": float(mesh.volume / bbox_vol) if bbox_vol else 0.0,
    })
    return mesh


# ---------------------------------------------------------------------------
# 3D octet / face-centred-cubic (FCC) strut lattice
# ---------------------------------------------------------------------------

def _octet_graph(nx, ny, nz, cell):
    """Return the shared FCC nodes and undirected face-diagonal edges."""
    nodes = {
        (ix, iy, iz): np.array((ix, iy, iz), dtype=float) * (cell / 2.0)
        for iz in range(2 * nz + 1)
        for iy in range(2 * ny + 1)
        for ix in range(2 * nx + 1)
        if (ix + iy + iz) % 2 == 0
    }
    offsets = []
    for a in (-1, 1):
        for b in (-1, 1):
            offsets.extend(((a, b, 0), (a, 0, b), (0, a, b)))

    edges = set()
    for key in nodes:
        for offset in offsets:
            neighbour = tuple(key[axis] + offset[axis] for axis in range(3))
            if neighbour in nodes:
                edges.add(tuple(sorted((key, neighbour))))
    return nodes, edges


def octet_truss(nx=2, ny=2, nz=2, cell=12.0, strut_d=1.6, node_d=None,
                 sections=12, nozzle=0.4, snap_strut=True):
    """Build a true octet-truss (FCC face-diagonal) lattice.

    Nodes occupy half-cell integer coordinates whose index sum is even: cube
    corners plus face centres, the finite face-centred-cubic graph. Each node
    connects only to in-bounds nearest neighbours at permutations of
    ``(+/-1, +/-1, 0)`` in that half-cell grid. The resulting tetrahedral and
    octahedral cells distinguish this structure from BCC body diagonals and a
    cubic edge grid.

    ``strut_d`` snaps to the ``nozzle`` grid by default. ``node_d`` defaults
    to 1.5 times the realised strut diameter and is never allowed below it.
    The block follows ``bcc_lattice`` conventions: centred in X/Y, nominal
    bottom nodes at z=0, and relative density measured against the requested
    cell-block volume. Units are mm.
    """
    for name, val in (("nx", nx), ("ny", ny), ("nz", nz)):
        if isinstance(val, bool) or not isinstance(val, int) or val < 1:
            raise ValueError("octet_truss(): %s must be a positive integer" % name)
    if nx * ny * nz > _MAX_STRUT_CELLS:
        raise ValueError(
            "octet_truss(): nx*ny*nz=%d exceeds the %d-cell playground cap; a "
            "larger strut union is too heavy to preview interactively"
            % (nx * ny * nz, _MAX_STRUT_CELLS))
    if cell <= 0:
        raise ValueError("octet_truss(): cell must be positive")
    if isinstance(sections, bool) or not isinstance(sections, int) or sections < 6:
        raise ValueError("octet_truss(): sections must be an integer at least 6")
    _validate_nozzle(nozzle)
    strut_d = _snap_strut(strut_d, nozzle, snap_strut, "strut_d")
    if strut_d < 0.8:
        raise ValueError(
            "octet_truss(): strut_d=%.3g mm is below the 0.8 mm FDM wall floor; "
            "use a bigger strut or a wider nozzle" % strut_d)
    if node_d is None:
        node_d = 1.5 * strut_d
    elif node_d <= 0:
        raise ValueError("octet_truss(): node_d must be positive")
    node_d = max(node_d, strut_d)
    # Face diagonals are spaced on a half-cell grid. Keeping three realised
    # strut diameters across a cell leaves an open octahedral/tetrahedral void
    # after cylinders and joint spheres blend, instead of approaching a slab.
    if cell < 3.0 * strut_d:
        raise ValueError(
            "octet_truss(): cell=%.3g mm is too small for strut_d=%.3g mm; the "
            "face-diagonal network fuses toward a solid (need cell >= 3*strut_d)"
            % (cell, strut_d))

    nodes, edges = _octet_graph(nx, ny, nz, cell)
    solids = [
        _strut(nodes[a], nodes[b], strut_d / 2.0, sections)
        for a, b in sorted(edges)
    ]
    for pos in nodes.values():
        joint = trimesh.creation.icosphere(subdivisions=1, radius=node_d / 2.0)
        joint.apply_translation(pos)
        solids.append(joint)

    mesh = uni(solids)
    if not mesh.is_watertight or not mesh.is_winding_consistent:
        mesh = from_manifold(to_manifold(mesh))
    mesh.apply_translation((-nx * cell / 2.0, -ny * cell / 2.0, 0.0))

    bbox_vol = (nx * cell) * (ny * cell) * (nz * cell)
    mesh.metadata.update({
        "mode": "octet",
        "cell_size": cell,
        "cells_x": nx,
        "cells_y": ny,
        "cells_z": nz,
        "cell_count": nx * ny * nz,
        "strut_count": len(edges),
        "node_count": len(nodes),
        "strut_d": strut_d,
        "node_d": node_d,
        "relative_density": float(mesh.volume / bbox_vol) if bbox_vol else 0.0,
    })
    return mesh


# ---------------------------------------------------------------------------
# Kelvin tetrakaidecahedral (truncated-octahedron) strut cell
# ---------------------------------------------------------------------------

def _kelvin_graph(cell):
    """Return the 24 vertices and 36 edges of one truncated octahedron."""
    integer_nodes = set()
    for zero_axis in range(3):
        other_axes = [axis for axis in range(3) if axis != zero_axis]
        for one_axis, two_axis in (other_axes, reversed(other_axes)):
            for one_sign in (-1, 1):
                for two_sign in (-1, 1):
                    point = [0, 0, 0]
                    point[one_axis] = one_sign
                    point[two_axis] = 2 * two_sign
                    integer_nodes.add(tuple(point))

    scale = cell / 4.0
    nodes = {
        key: np.array((key[0] * scale,
                       key[1] * scale,
                       key[2] * scale + cell / 2.0))
        for key in integer_nodes
    }
    keys = sorted(nodes)
    edges = {
        (a, b)
        for index, a in enumerate(keys)
        for b in keys[index + 1:]
        if sum((a[axis] - b[axis]) ** 2 for axis in range(3)) == 2
    }
    return nodes, edges


def kelvin_cell(cell=20.0, strut_d=1.6, node_d=None,
                sections=12, nozzle=0.4, snap_strut=True):
    """Build one Kelvin (truncated-octahedron) 3D strut cell.

    ``cell`` is the vertex-to-vertex bounding span along each principal axis.
    The exact vertex set is the permutations of ``(0, +/-1, +/-2)``, scaled
    to that span. Its 36 equal edges bound six square and eight regular
    hexagonal faces. X/Y are centred and the nominal lowest graph vertices
    lie at z=0; round joint spheres may extend beyond those nominal bounds.

    ``strut_d`` snaps to the ``nozzle`` grid by default. ``node_d`` defaults
    to 1.5 times the realised strut diameter and is never allowed below it.
    Units are mm.
    """
    if cell <= 0:
        raise ValueError("kelvin_cell(): cell must be positive")
    if isinstance(sections, bool) or not isinstance(sections, int) or sections < 6:
        raise ValueError("kelvin_cell(): sections must be an integer at least 6")
    _validate_nozzle(nozzle)
    strut_d = _snap_strut(strut_d, nozzle, snap_strut, "strut_d")
    if strut_d < 0.8:
        raise ValueError(
            "kelvin_cell(): strut_d=%.3g mm is below the 0.8 mm FDM wall floor; "
            "use a bigger strut or a wider nozzle" % strut_d)
    if node_d is None:
        node_d = 1.5 * strut_d
    elif node_d <= 0:
        raise ValueError("kelvin_cell(): node_d must be positive")
    node_d = max(node_d, strut_d)
    if cell < 4.0 * node_d:
        raise ValueError(
            "kelvin_cell(): cell=%.3g mm is too small for node_d=%.3g mm; "
            "the joints collapse the open faces (need cell >= 4*node_d)"
            % (cell, node_d))

    nodes, edges = _kelvin_graph(cell)
    solids = [
        _strut(nodes[a], nodes[b], strut_d / 2.0, sections)
        for a, b in sorted(edges)
    ]
    for pos in nodes.values():
        joint = trimesh.creation.icosphere(subdivisions=1, radius=node_d / 2.0)
        joint.apply_translation(pos)
        solids.append(joint)

    mesh = uni(solids)
    if not mesh.is_watertight or not mesh.is_winding_consistent:
        mesh = from_manifold(to_manifold(mesh))

    bbox_vol = cell ** 3
    mesh.metadata.update({
        "mode": "kelvin",
        "cell_size": cell,
        "strut_count": len(edges),
        "node_count": len(nodes),
        "strut_d": strut_d,
        "node_d": node_d,
        "relative_density": float(mesh.volume / bbox_vol),
    })
    return mesh


__all__ = (
    "auxetic_panel",
    "bcc_lattice",
    "cubic_lattice",
    "gyroid_lattice",
    "honeycomb_panel",
    "isogrid_panel",
    "kagome_panel",
    "kerf_bend_cutter",
    "kelvin_cell",
    "octet_truss",
)
