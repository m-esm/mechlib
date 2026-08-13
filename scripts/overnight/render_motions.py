#!/usr/bin/env python3
"""Render motion contact sheets for gallery mechanisms.

Rebuilds each demo at a handful of phase samples (or the rest pose for
non-animated multi-body mechanisms) and writes an iso/side/front contact
sheet plus a meta.json the visual-review loop can read.

No OpenGL. Painter's-algorithm raster via Pillow so this runs headless.
"""

from __future__ import annotations

import argparse
import importlib.util
import inspect
import json
import math
import sys
import traceback
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from PIL import Image, ImageDraw, ImageFont

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from paths import CATALOG, MOTION_QUEUE, RENDERS, ROOT, ensure_dirs
from workqueue import complete_motion, dump_json, load_json, now_iso

TILE = 220
MAX_FACES_PER_BODY = 10000
PHASE_SAMPLES = 6
BG = (11, 36, 56, 255)
LIGHT = np.array([0.45, -0.38, 0.81], dtype=float)
LIGHT /= np.linalg.norm(LIGHT)
VIEWS = (
    ("iso", np.array([1.0, 1.15, 0.85])),
    ("side", np.array([1.0, 0.0, 0.12])),
    ("front", np.array([0.0, -1.0, 0.18])),
)


def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def load_gallery():
    demos = _load_module(ROOT / "gallery" / "demos.py", "gallery_demos_overnight")
    build = _load_module(ROOT / "gallery" / "build_gallery.py", "gallery_build_overnight")
    return demos, build


def demo_functions(demos) -> Dict[str, object]:
    return {
        name: fn
        for name, fn in inspect.getmembers(demos, inspect.isfunction)
        if name.startswith("demo_")
    }


def phases_for(demo_name: str, demo_fn, animate: dict) -> Tuple[Optional[str], List[float], dict]:
    if demo_name not in animate:
        return None, [None], {"animated": False}
    param, cycle_deg, frames, closed = animate[demo_name]
    signature_params = inspect.signature(demo_fn).parameters
    if param not in signature_params:
        raise KeyError("%s ANIMATE param %r missing" % (demo_name, param))
    base = signature_params[param].default
    if base is inspect.Parameter.empty:
        raise ValueError("%s ANIMATE param %s has no default" % (demo_name, param))
    base = float(base)
    # Equal cycle/N steps alias on gears: a 16-tooth pinion sampled every
    # 180 deg looks frozen. Always include one baked-frame step, then uneven
    # fractions that are not integer tooth pitches.
    step = (1.0 / float(frames)) if frames else 0.07
    fracs = [0.0, min(step, 0.08), 0.22, 0.41, 0.63, 0.84][:PHASE_SAMPLES]
    values = [base + f * float(cycle_deg) for f in fracs]
    spec = {
        "animated": True,
        "param": param,
        "cycle_deg": float(cycle_deg),
        "frames_baked": int(frames),
        "closed": bool(closed),
        "base": base,
        "phase_fracs": fracs,
    }
    return param, values, spec


def _as_rgba(color) -> Tuple[int, int, int, int]:
    rgb = [int(c) for c in color[:3]]
    return (rgb[0], rgb[1], rgb[2], 255)


def build_phase(demo_fn, param: Optional[str], value) -> List[Tuple[str, object, Tuple[int, int, int, int]]]:
    if param is None or value is None:
        raw = demo_fn()
    else:
        raw = demo_fn(**{param: value})
    if not isinstance(raw, list) or not raw:
        raise ValueError("demo returned empty / non-list")
    return [(name, mesh, _as_rgba(color)) for name, mesh, color in raw]


def body_centroids(entries) -> Dict[str, List[float]]:
    out = {}
    for name, mesh, _color in entries:
        out[name] = [float(v) for v in mesh.centroid]
    return out


def travel_mm(series: List[Dict[str, List[float]]]) -> Dict[str, float]:
    names = set()
    for frame in series:
        names.update(frame)
    out = {}
    for name in sorted(names):
        pts = [frame[name] for frame in series if name in frame]
        if len(pts) < 2:
            out[name] = 0.0
            continue
        arr = np.asarray(pts, dtype=float)
        out[name] = float(np.linalg.norm(arr.max(axis=0) - arr.min(axis=0)))
    return out


def vertex_travel_mm(frames) -> Dict[str, float]:
    """Peak per-vertex displacement from frame 0.

    Centroid AABB travel is zero for a body that spins in place (gears,
    Geneva wheels, escape wheels). Vertex travel still sees the rotation.
    """
    first = {name: np.asarray(mesh.vertices, dtype=float)
             for name, mesh, _color in frames[0]}
    out = {name: 0.0 for name in first}
    for entries in frames[1:]:
        for name, mesh, _color in entries:
            if name not in first or len(mesh.vertices) != len(first[name]):
                continue
            peak = float(np.linalg.norm(
                np.asarray(mesh.vertices, dtype=float) - first[name],
                axis=1).max())
            if peak > out[name]:
                out[name] = peak
    return out


def _basis(eye: np.ndarray):
    view = -eye / np.linalg.norm(eye)  # camera looks toward origin
    world_up = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(view, world_up))) > 0.92:
        world_up = np.array([0.0, 1.0, 0.0])
    right = np.cross(view, world_up)
    right /= np.linalg.norm(right)
    up = np.cross(right, view)
    up /= np.linalg.norm(up)
    return right, up, view


def _project_points(points: np.ndarray, center: np.ndarray, right, up, view):
    rel = points - center
    return np.column_stack((rel @ right, rel @ up, rel @ view))


def raster_tile(entries, center, half, eye, size=TILE) -> Image.Image:
    right, up, view = _basis(eye)
    img = Image.new("RGBA", (size, size), BG)
    draw = ImageDraw.Draw(img, "RGBA")
    faces: List[Tuple[float, np.ndarray, Tuple[int, int, int, int]]] = []
    for _name, mesh, color in entries:
        verts = np.asarray(mesh.vertices, dtype=float)
        tris = np.asarray(mesh.faces, dtype=int)
        if len(tris) == 0 or len(verts) == 0:
            continue
        if len(tris) > MAX_FACES_PER_BODY:
            tris = tris[:: max(1, int(math.ceil(len(tris) / MAX_FACES_PER_BODY)))]
        proj = _project_points(verts, center, right, up, view)
        a, b, c = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
        normals = np.cross(b - a, c - a)
        nlen = np.linalg.norm(normals, axis=1)
        good = nlen > 1e-12
        if not np.any(good):
            continue
        normals = normals[good] / nlen[good, None]
        tris = tris[good]
        shade = np.clip(0.28 + 0.72 * (normals @ LIGHT), 0.12, 1.0)
        depth = proj[tris].mean(axis=1)[:, 2]
        pts = proj[tris][:, :, :2]
        rgb = np.array(color[:3], dtype=float)
        for z, tri, s in zip(depth, pts, shade):
            col = tuple(int(max(0, min(255, round(c * s)))) for c in rgb) + (255,)
            faces.append((float(z), tri, col))
    faces.sort(key=lambda item: item[0])  # far (small z toward camera? view from +eye)
    # Camera sits along +eye; view = -eye/|eye|, so points toward the camera
    # have larger (p-center)·view? eye is in +octant, view points toward
    # origin from camera, so a point near the camera is toward +eye from
    # center, and (p-center)·view = (p-center)·(-eye_hat) is negative.
    # Far points (opposite camera) are positive. Draw high z first.
    faces.sort(key=lambda item: -item[0])
    scale = (size * 0.5 - 8.0) / max(half, 1e-6)
    cx = cy = size * 0.5
    for _z, tri, col in faces:
        pix = [(cx + float(p[0]) * scale, cy - float(p[1]) * scale) for p in tri]
        draw.polygon(pix, fill=col)
    return img


def _font(size: int):
    for path in (
        "/System/Library/Fonts/Supplemental/Arial Unicode.ttf",
        "/System/Library/Fonts/SFNSMono.ttf",
        "/Library/Fonts/Arial.ttf",
    ):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def compose_sheet(tiles: List[List[Image.Image]], labels: Sequence[str], title: str) -> Image.Image:
    rows = len(tiles)
    cols = len(tiles[0])
    header = 28
    footer = 18
    sheet = Image.new("RGBA", (cols * TILE, rows * TILE + header + footer), (7, 24, 38, 255))
    draw = ImageDraw.Draw(sheet)
    font = _font(14)
    small = _font(11)
    draw.text((8, 6), title, fill=(234, 243, 251, 255), font=font)
    for r, row in enumerate(tiles):
        for c, tile in enumerate(row):
            sheet.paste(tile, (c * TILE, header + r * TILE))
    for c, label in enumerate(labels):
        draw.text((c * TILE + 6, header + rows * TILE + 2), label,
                  fill=(143, 176, 204, 255), font=small)
    return sheet


def render_demo(demo_name: str, demo_fn, animate: dict, catalog: dict) -> dict:
    param, values, spec = phases_for(demo_name, demo_fn, animate)
    frames = []
    centroids = []
    for value in values:
        entries = build_phase(demo_fn, param, value)
        frames.append(entries)
        centroids.append(body_centroids(entries))

    all_pts = []
    for entries in frames:
        for _name, mesh, _color in entries:
            all_pts.append(np.asarray(mesh.vertices, dtype=float))
    cloud = np.concatenate(all_pts, axis=0)
    center = 0.5 * (cloud.min(axis=0) + cloud.max(axis=0))
    # Fit using the same camera bases as the tiles.
    half = 0.0
    for _name, eye in VIEWS:
        right, up, view = _basis(eye)
        proj = _project_points(cloud, center, right, up, view)
        half = max(half, float(np.max(np.abs(proj[:, :2]))))
    half = max(half * 1.08, 1.0)

    view_rows = []
    for _name, eye in VIEWS:
        view_rows.append([raster_tile(entries, center, half, eye) for entries in frames])

    if spec["animated"]:
        labels = ["%s=%.1f" % (param, float(v)) for v in values]
        title = "%s  %s cycle %.1f deg  %d bodies" % (
            demo_name, param, spec["cycle_deg"], len(frames[0]))
    else:
        labels = ["rest"]
        title = "%s  rest pose  %d bodies" % (demo_name, len(frames[0]))

    sheet = compose_sheet(view_rows, labels, title)
    out_dir = RENDERS / demo_name
    out_dir.mkdir(parents=True, exist_ok=True)
    sheet_path = out_dir / "sheet.png"
    small_path = out_dir / "sheet_s.png"
    sheet.save(sheet_path)
    # Reviewer copy stays under the 1600 long-edge cap.
    sheet_s = sheet.copy()
    sheet_s.thumbnail((1400, 1400))
    sheet_s.save(small_path)

    travels = travel_mm(centroids)
    vtravels = vertex_travel_mm(frames)
    moving = sorted(
        name for name in travels
        if travels[name] >= 0.6 or vtravels.get(name, 0.0) >= 0.6)
    still = sorted(name for name in travels if name not in set(moving))
    info = catalog.get("demos", {}).get(demo_name, {})
    meta = {
        "demo": demo_name,
        "rendered_at": now_iso(),
        "sheet": str(sheet_path),
        "sheet_small": str(small_path),
        "views": [name for name, _eye in VIEWS],
        "phases": [None if v is None else float(v) for v in values],
        "bodies": [name for name, _m, _c in frames[0]],
        "travel_mm": travels,
        "vertex_travel_mm": vtravels,
        "moving": moving,
        "stationary": still,
        "category": info.get("category"),
        "group": info.get("group"),
        "applications": info.get("applications"),
        "description": info.get("description"),
        "usecase": info.get("usecase"),
        **spec,
    }
    dump_json(out_dir / "meta.json", meta)
    return meta


def render_one(demo_name: str, demo_fn, animate: dict, catalog: dict) -> dict:
    try:
        meta = render_demo(demo_name, demo_fn, animate, catalog)
        complete_motion(
            demo_name,
            rendered=True,
            render_error=None,
            sheet=meta["sheet_small"],
            moving=meta["moving"],
            stationary=meta["stationary"],
            animated=meta.get("animated", False),
        )
        return {"id": demo_name, "ok": True, "sheet": meta["sheet_small"]}
    except Exception as exc:  # noqa: BLE001 - overnight must keep going
        err = "%s: %s" % (type(exc).__name__, exc)
        complete_motion(
            demo_name,
            rendered=False,
            render_error=err,
            traceback=traceback.format_exc()[-2000:],
        )
        return {"id": demo_name, "ok": False, "error": err}


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--next", type=int, default=0, dest="next_n")
    parser.add_argument("--only", action="append", default=[])
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args(argv)
    ensure_dirs()

    queue = load_json(MOTION_QUEUE, {"items": []})
    if not queue.get("items"):
        raise SystemExit("motion queue missing; run catalog_inventory.py first")
    catalog = load_json(CATALOG, {})
    demos_mod, build_mod = load_gallery()
    fns = demo_functions(demos_mod)
    animate = dict(build_mod.ANIMATE)

    wanted = []
    if args.only:
        wanted = list(args.only)
    elif args.all:
        wanted = [item["id"] for item in queue["items"]]
    elif args.next_n:
        from workqueue import claim_motion
        claimed = claim_motion(args.next_n, "render")
        wanted = [item["id"] for item in claimed]
    else:
        raise SystemExit("pass --all, --next N, or --only demo_name")

    results = []
    for demo_name in wanted:
        item = next((i for i in queue["items"] if i["id"] == demo_name), None)
        if item and item.get("rendered") and not args.force and not item.get("render_error"):
            results.append({"id": demo_name, "ok": True, "skipped": True})
            continue
        if demo_name not in fns:
            complete_motion(demo_name, rendered=False, render_error="unknown demo")
            results.append({"id": demo_name, "ok": False, "error": "unknown demo"})
            continue
        print("render", demo_name, flush=True)
        results.append(render_one(demo_name, fns[demo_name], animate, catalog))

    ok = sum(1 for r in results if r.get("ok"))
    print("rendered %d/%d" % (ok, len(results)))
    print(json.dumps(results, indent=2))
    return 0 if ok == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
