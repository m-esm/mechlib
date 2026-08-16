"""Display envelopes for bought vitamins. Not printed parts (R6)."""
from __future__ import annotations

import trimesh

from ..meshutil import sub
from ..prim import boxc, cyl
from .spec import Vitamin


def build(item: Vitamin) -> trimesh.Trimesh:
    if item.family == "bearing":
        return _bearing(item)
    if item.family == "fastener":
        return _shcs(item, length=max(12.0, 4.0 * float(item.d)))
    if item.family == "nut":
        return _nut(item)
    if item.family == "washer":
        return _washer(item)
    if item.family == "cell":
        return cyl(float(item.d) / 2.0, float(item.length), sections=48)
    if item.family == "sensor" and item.slug == "tcst1103":
        return boxc((item.body_l, item.body_w, item.body_h))
    if item.family == "sensor" and item.slug == "hc-sr04":
        return boxc((item.board_l, item.board_w, item.board_t + 12.0))
    if item.family == "motor" and item.slug == "ga12-n20":
        return boxc((item.body_len, item.env_y, item.env_x))
    if item.family == "motor" and item.slug == "nema17":
        return boxc((item.face, item.face, item.body_len))
    if item.family == "servo":
        long = float(item.dims.get("long", item.dims.get("body_l", 23.0)))
        thin = float(item.dims.get("thin", item.dims.get("body_w", 12.4)))
        tall = float(item.dims.get("tall", item.dims.get("spline_tip", 22.5)))
        return boxc((long, thin, tall))
    # Generic AABB fallback from any d/od/length-like keys.
    od = float(item.dims.get("od", item.dims.get("d", 10.0)))
    h = float(item.dims.get("width", item.dims.get("height", item.dims.get("length", 4.0))))
    return cyl(od / 2.0, h, sections=32)


def _bearing(item: Vitamin) -> trimesh.Trimesh:
    od = float(item.od)
    id_ = float(item.id)
    w = float(item.width)
    outer = cyl(od / 2.0, w, sections=48)
    inner = cyl(id_ / 2.0, w + 1.0, sections=32)
    ring = sub(outer, inner)
    # Visual race split: a shallow mid-groove so it reads as a bearing, not a tube.
    groove_r = 0.25 * od + 0.25 * id_
    groove = cyl(groove_r, max(0.6, 0.2 * w), sections=32)
    try:
        ring = sub(ring, groove)
    except Exception:
        pass
    return ring


def _shcs(item: Vitamin, length: float) -> trimesh.Trimesh:
    from ..fasteners import fastener_mesh
    return fastener_mesh(float(item.d), length, style="shcs")


def _nut(item: Vitamin) -> trimesh.Trimesh:
    from ..prim import hex_poly
    body = trimesh.creation.extrude_polygon(hex_poly(float(item.af)), float(item.height))
    hole = cyl(float(item.hole_d) / 2.0, float(item.height) + 1.0, sections=24)
    return sub(body, hole)


def _washer(item: Vitamin) -> trimesh.Trimesh:
    disc = cyl(float(item.od) / 2.0, float(item.height), sections=32)
    hole = cyl(float(item.id) / 2.0, float(item.height) + 1.0, sections=24)
    return sub(disc, hole)
