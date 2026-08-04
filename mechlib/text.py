"""Optional text-to-polygon conversion."""

import numpy as np
import shapely.geometry as sg
import shapely.ops as so


def text_polygon(text, size, font_path=None):
    """Convert text outlines to Shapely polygons while preserving counters.

    ``font_path`` selects an explicit font file. ``None`` uses matplotlib's
    default font. Matplotlib remains an optional dependency imported on demand.
    origin: finnish-doors src/projects/klonk/housings.py:24
    """
    from matplotlib.font_manager import FontProperties
    from matplotlib.textpath import TextPath

    fp = FontProperties(fname=font_path) if font_path is not None else None
    tp = TextPath((0, 0), text, size=size, prop=fp)
    rings = []
    for loop in tp.to_polygons():
        if len(loop) < 3:
            continue
        p = sg.Polygon(np.asarray(loop, dtype=float))
        if not p.is_valid:
            p = p.buffer(0)
        if p.is_empty or p.area < 0.04:
            continue
        rings.append(p)
    if not rings:
        return None
    rings.sort(key=lambda p: -p.area)
    g = rings[0]
    for p in rings[1:]:
        try:
            pt = p.representative_point()
            inside = g.contains(pt)
        except Exception:
            inside = False
        if inside and g.intersection(p).area > 0.45 * p.area:
            g = g.difference(p)
        else:
            g = g.union(p)
    if g is None or g.is_empty:
        return None
    if g.geom_type == "MultiPolygon":
        g = so.unary_union(list(g.geoms))
    return g if (g is not None and not g.is_empty) else None


def place(poly2d, cx, cy):
    """Center a 2D geometry at a point.

    origin: torque-lever build.py:145
    """
    from shapely import affinity
    minx, miny, maxx, maxy = poly2d.bounds
    return affinity.translate(poly2d, cx - (minx + maxx) / 2,
                              cy - (miny + maxy) / 2)


def place_right(poly2d, right_x, cy):
    """Right-align a 2D geometry and center it vertically.

    origin: torque-lever build.py:152
    """
    from shapely import affinity
    minx, miny, maxx, maxy = poly2d.bounds
    return affinity.translate(poly2d, right_x - maxx,
                              cy - (miny + maxy) / 2)


def text_block(lines, cx, cy, h, gap=1.2):
    """Stack centered text polygons with the top line first.

    origin: torque-lever build.py:159
    """
    pitch = h + gap
    total = len(lines) * h + (len(lines) - 1) * gap
    return [place(text_polygon(s, h), cx,
                  cy + total / 2 - h / 2 - i * pitch)
            for i, s in enumerate(lines)]
