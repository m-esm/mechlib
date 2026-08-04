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
