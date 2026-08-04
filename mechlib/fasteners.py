"""Hardware stand-ins and display-quality oriented fasteners."""

import math

import numpy as np
import trimesh

from .meshutil import sub, uni
from .prim import cyl, hex_poly


def zmin0(m):
    """Translate a mesh so its minimum Z coordinate is zero.

    origin: parviz src/standins/_common.py:13
    """
    m.apply_translation((0, 0, -m.bounds[0][2]))
    return m


def bolt_mesh(shank_r, shank_l, head_r, head_h):
    """Build a head-down vertical bolt with its shank above the head.

    origin: parviz src/standins/_common.py:18
    """
    hd = cyl(head_r, head_h)
    hd.apply_translation((0, 0, head_h / 2))
    sh = cyl(shank_r, shank_l)
    sh.apply_translation((0, 0, head_h + shank_l / 2))
    return uni([hd, sh])


def hex_nut_mesh(af, h, bore_d):
    """Build a hex-nut stand-in from across-flats, height, and bore diameter.

    origin: parviz src/standins/_common.py:27
    """
    nt = cyl(af / math.sqrt(3.0), h, sections=6)
    nt = sub(nt, cyl(bore_d / 2, h + 2))
    return zmin0(nt)


def washer_mesh(od, id_, h):
    """Build a flat annular washer stand-in.

    origin: parviz src/standins/_common.py:33
    """
    r = sub(cyl(od / 2, h), cyl(id_ / 2, h + 2))
    return zmin0(r)


def _aligned(mesh, direction, origin):
    direction = np.asarray(direction, float)
    direction /= np.linalg.norm(direction)
    T = trimesh.geometry.align_vectors([0, 0, 1], direction)
    T[:3, 3] = origin
    mesh.apply_transform(T)
    return mesh


def fastener_mesh(d, L, style="pan", axis="z", at=(0.0, 0.0, 0.0), sections=32):
    """Build and orient a pan, socket-head, or countersunk fastener stand-in.

    The origin is the head bearing plane and the shank extends along ``axis``,
    which may be x, y, z, or a direction vector. Pan and SHCS heads sit proud
    opposite the axis; the CSK cone sinks along it. Dimensions scale from ``d``.
    origin: finnish-windows tools/add_screws_glb.py:33
    origin: finnish-windows tools/add_screws_glb2.py:27
    origin: finnish-doors src/shared/fasteners.py:115
    """
    if style not in ("pan", "shcs", "csk"):
        raise ValueError("fastener_mesh(): style must be 'pan', 'shcs', or 'csk'")
    if d <= 0 or L <= 0:
        raise ValueError("fastener_mesh(): d and L must be positive")
    direction = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0),
                 "z": (0.0, 0.0, 1.0)}.get(axis, axis)
    shaft = cyl(d / 2.0, L, sections=sections)
    shaft.apply_translation((0, 0, L / 2.0))
    if style == "csk":
        hh = 0.57 * d
        head = trimesh.creation.cone(radius=d, height=hh, sections=sections)
        head.apply_translation((0, 0, hh / 2.0))
    else:
        if style == "pan":
            rh, hh = 0.92 * d, 0.70 * d
        else:
            rh, hh = 0.92 * d, 1.00 * d
        head = cyl(rh, hh, sections=sections)
        head.apply_translation((0, 0, -hh / 2.0))
        if style == "shcs":
            socket_d = 0.55 * d
            socket_h = 0.25 * d
            socket = trimesh.creation.extrude_polygon(hex_poly(socket_d), socket_h + 0.1)
            socket.apply_translation((0, 0, -hh - 0.05))
            head = sub(head, socket)
    screw = uni([shaft, head])
    return _aligned(screw, direction, np.asarray(at, float))


def pick_length(span, std_lengths=(6, 8, 10, 12, 16, 20, 25, 30)):
    """Pick the first standard screw length no more than 0.6 mm short.

    origin: finnish-windows tools/add_screws_glb.py:19
    """
    for L in std_lengths:
        if L >= span - 0.6:
            return L
    return std_lengths[-1]
