"""Brim-aware first-fit-decreasing plate packing."""

import numpy as np
import trimesh.transformations as tf


_ROTZ = tf.rotation_matrix(np.pi / 2, [0, 0, 1])


def _brim(obj, settings):
    """Return the effective outer-brim width for a part."""
    if obj.get("brim_type", settings.get("brim_type")) in ("no_brim", "none"):
        return 0.0
    return float(obj.get("brim_width", settings.get("brim_width", 0.0)))


def _size(items, settings=None):
    """Orient mesh items longest-edge-first and calculate brim footprints.

    Items are ``(name, mesh, rotation, object_settings)`` tuples.
    origin: wall-shelf-clamp tools/export_bambu.py:107
    """
    settings = {} if settings is None else settings
    sized, seen = [], {}
    for name, mesh, rot, obj in items:
        m = mesh.copy()
        if rot is not None:
            m.apply_transform(rot)
        e = m.bounds[1] - m.bounds[0]
        if e[1] > e[0]:
            m.apply_transform(_ROTZ); e = m.bounds[1] - m.bounds[0]
        nm = name.rsplit(".", 1)[0]; seen[nm] = seen.get(nm, 0) + 1
        if seen[nm] > 1:
            nm = "%s_%d" % (nm, seen[nm])
        obj_settings = dict(obj)
        b = _brim(obj_settings, settings)
        sized.append(dict(name=nm, mesh=m, w=e[0], d=e[1],
                          fw=e[0] + 2 * b, fd=e[1] + 2 * b,
                          obj=obj_settings))
    return sized


def shelf_pack(sized, bed=(256.0, 256.0), gap=6.0):
    """Pack brim-grown footprints onto one or more plates.

    Returns a list of plates containing placement dictionaries.
    origin: wall-shelf-clamp tools/export_bambu.py:125

    The variants in parviz ``tools/export_bambu.py:324`` and finnish-windows
    ``src/export_bambu.py:146`` are weaker; this implementation is canonical.
    """
    bed_w, bed_d = bed
    sized = sorted(sized, key=lambda s: -s["fd"])
    plates, x, y, rowh = [[]], gap, gap, 0.0
    for p in sized:
        fw, fd = p["fw"], p["fd"]
        if x + fw + gap > bed_w:
            x, y, rowh = gap, y + rowh + gap, 0.0
        if y + fd + gap > bed_d:
            plates.append([]); x, y, rowh = gap, gap, 0.0
        plates[-1].append(dict(name=p["name"], mesh=p["mesh"], fw=fw, fd=fd,
                               pos=(x + fw / 2 - bed_w / 2,
                                    y + fd / 2 - bed_d / 2),
                               obj_settings=p["obj"]))
        x += fw + gap; rowh = max(rowh, fd)
    return plates


def pack_by_category(items, bed=(256.0, 256.0), gap=6.0, categories=None,
                     settings=None, category_order=()):
    """Group mesh items by optional category map, then pack separate plates.

    Returns ``(plate_name, placed_parts)`` pairs.
    origin: wall-shelf-clamp tools/export_bambu.py:146
    """
    categories = {} if categories is None else categories
    buckets = {}
    for it in items:
        cat = categories.get(it[0].rsplit(".", 1)[0], "Other")
        buckets.setdefault(cat, []).append(it)
    order = list(category_order) + [c for c in buckets if c not in category_order]
    out = []
    for cat in order:
        if cat not in buckets:
            continue
        sub = shelf_pack(_size(buckets[cat], settings=settings), bed=bed, gap=gap)
        n = len(sub)
        for i, parts in enumerate(sub, 1):
            out.append((("%s %d of %d" % (cat, i, n) if n > 1 else cat), parts))
    return out
