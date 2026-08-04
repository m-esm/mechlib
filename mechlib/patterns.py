"""Reusable polar and panel-lightening layout patterns."""

import math

import shapely.geometry as sg


def polar_ring(n, r, center=(0.0, 0.0), phase=0.0):
    """Return ``n`` evenly spaced XY points around a ring.

    ``phase`` is in radians, matching the source placement arithmetic.
    origin: finnish-windows tools/add_teeth.py:46
    """
    if n < 1 or r < 0:
        raise ValueError("polar_ring(): n must be positive and r non-negative")
    cx, cy = center
    step = 2.0 * math.pi / n
    return [(cx + r * math.cos(phase + k * step),
             cy + r * math.sin(phase + k * step)) for k in range(n)]


def lighten_cell_poly(cx, cy, cell, pattern):
    """Return one rectangular or flat-top hexagonal lightening cell.

    origin: finnish-doors src/projects/klonk/housings.py:1733
    """
    if pattern == "rect":
        h = cell / 2.0
        return sg.box(cx - h, cy - h, cx + h, cy + h)
    if pattern != "hex":
        raise ValueError("lighten_cell_poly(): pattern must be 'rect' or 'hex'")
    r = cell / math.sqrt(3.0)
    angs = [math.radians(60.0 * i) for i in range(6)]
    return sg.Polygon([(cx + r * math.cos(a), cy + r * math.sin(a)) for a in angs])


def lighten_grid_centres(minx, miny, maxx, maxy, cell, web, pattern):
    """Yield lattice centres covering a bounding box for rect or hex cells.

    origin: finnish-doors src/projects/klonk/housings.py:1744
    """
    pitch = cell + web
    if pattern == "rect":
        x = minx
        while x <= maxx + 1e-9:
            y = miny
            while y <= maxy + 1e-9:
                yield x, y
                y += pitch
            x += pitch
        return
    if pattern != "hex":
        raise ValueError("lighten_grid_centres(): pattern must be 'rect' or 'hex'")
    dx = pitch
    dy = pitch * math.sqrt(3.0) / 2.0
    row = 0
    y = miny
    while y <= maxy + 1e-9:
        x0 = minx + (0.5 * dx if (row % 2) else 0.0)
        x = x0
        while x <= maxx + 1e-9:
            yield x, y
            x += dx
        y += dy
        row += 1


def directed_holes(points, vectors, d, length):
    """Union bores starting at points and following corresponding vectors.

    ``d`` may be one diameter or a diameter sequence matching ``points``.
    origin: massage-shower-head build.py:178
    """
    import numpy as np
    import trimesh

    from .prim import seg_cylinder

    diameters = ([d] * len(points) if np.isscalar(d) else d)
    cyls = []
    for point, vector, diameter in zip(points, vectors, diameters):
        p0 = np.asarray(point, float)
        direction = np.asarray(vector, float)
        direction /= np.linalg.norm(direction)
        cyls.append(seg_cylinder(p0, p0 + direction * length, diameter))
    return trimesh.boolean.union(cyls, engine="manifold")
