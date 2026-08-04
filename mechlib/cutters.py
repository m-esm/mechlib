"""Mechanical cutter solids and print-aware pocket features."""

import math

import numpy as np
import shapely.geometry as sg
import trimesh
import trimesh.transformations as tf

from .meshutil import inter, orient, sub, uni
from .prim import boxc, cyl, frustum


def teardrop(r, length, axis="x", up=(0, -1, 0)):
    """Build a self-supporting bore cutter with an arbitrary up vector.

    origin: parviz src/head.py:38
    """
    a = {"x": (1.0, 0, 0), "y": (0, 1.0, 0), "z": (0, 0, 1.0)}[axis]
    u = np.asarray(up, float); u /= np.linalg.norm(u)
    if abs(float(np.dot(np.asarray(a, float), u))) > 1e-6:
        raise ValueError("teardrop(): up must be perpendicular to axis")
    cap = boxc({"x": (length, r, r), "y": (r, length, r), "z": (r, r, length)}[axis])
    cap.apply_transform(tf.rotation_matrix(np.pi / 4.0, a))
    cap.apply_translation(u * (r * np.sqrt(0.5)))
    return uni([cyl(r, length, axis=axis), cap])


def ss_bore(R, Robj, length, center, axis="x", split_z=0.0):
    """Build a support-light horizontal clamshell bore cutter.

    ``split_z`` is the lid-top height above the bore axis. The Klonk source used
    ``housing_z_half`` for this value. The specified 0.0 default preserves the
    extracted API but must be overridden with a real half-height comfortably
    larger than ``R``.
    origin: finnish-doors src/shared/geom_util.py:64
    """
    from shapely.geometry.polygon import orient as orient_polygon
    zmax = split_z - 3.0
    zv = min(max(1.0, Robj*math.sqrt(2.0) - R + 0.6), zmax - 1.0)
    ctop = R - (zmax - zv)
    if ctop < 0.4: ctop, ztop = 0.4, min(zmax, zv + (R - 0.4))
    else:          ztop = zmax
    pts = [(R*math.cos(a), R*math.sin(a)) for a in np.linspace(0.0, -math.pi, 32)]
    pts += [(-R, zv), (-ctop, ztop), (ctop, ztop), (R, zv)]
    prof = orient_polygon(sg.Polygon(pts))
    m = trimesh.creation.extrude_polygon(prof, length)
    m.apply_translation([0, 0, -length/2.0])
    Rm = np.eye(4)
    Rm[:3, :3] = [[0,0,1],[1,0,0],[0,1,0]] if axis == "x" else [[-1,0,0],[0,0,1],[0,1,0]]
    m.apply_transform(Rm)
    m.apply_translation(center)
    return m


def dbore(shaft_d, flat, length, axis="z", clear=0.0,
          round_clear=0.0, flat_clear=0.0):
    """Build a unified double-D bore negative from explicit shaft dimensions.

    ``clear`` is added radially to both features, while ``round_clear`` and
    ``flat_clear`` add feature-specific radial clearance. The canonical parviz
    defaults were a 4.93 mm shaft, 3.0 mm flats, and 0.12 mm general clearance.
    origin: parviz src/geo.py:308
    origin: finnish-windows src/build_r2_coupon.py:41
    origin: finnish-windows tools/gearbox.py:110
    """
    d = shaft_d + 2 * (clear + round_clear)
    flat_d = flat + 2 * (clear + flat_clear)
    round_bore = cyl(d / 2, length, axis=axis)
    big = d + 4
    if axis == "z":
        slab = boxc((flat_d, big, length))
    elif axis == "x":
        slab = boxc((length, flat_d, big))
    elif axis == "y":
        slab = boxc((flat_d, length, big))
    else:
        raise ValueError("dbore(): axis must be 'x', 'y', or 'z'")
    return inter(round_bore, slab)


def dbore_hub(outer_r, length, axis="z", shaft_d=4.93, flat=3.0,
              clear=0.12, round_clear=0.0, flat_clear=0.0):
    """Build a cylindrical hub with a parviz-style double-D socket.

    The shaft defaults, 4.93 mm diameter and 3.0 mm flats, come from parviz
    ``params.py``; the source bore clearance was 0.12 mm.
    origin: parviz src/geo.py:329
    """
    hub = cyl(outer_r, length, axis=axis)
    cut = dbore(shaft_d, flat, length + 2, axis=axis, clear=clear,
                round_clear=round_clear, flat_clear=flat_clear)
    return sub(hub, cut)


def chamfer_cutter(r, ch):
    """Build the outside ring that cuts a 45 degree end chamfer.

    The parviz foot-pin source used ``ch=0.4``.
    origin: parviz src/standins/foot_pin.py:44
    """
    slab = cyl(r + 1.0, ch)
    slab.apply_translation((0, 0, ch / 2))
    return sub(slab, frustum(r, r - ch, ch))


def hex_corner_chamfer(nut, z_face, up, r_circ, ch=0.6):
    """Cut 45 degree corner chamfers while preserving hex flats.

    The parviz M8 nut source used ``CH=0.6``.
    origin: parviz src/standins/m8_nut.py:92
    """
    r_ch = r_circ - ch
    band = cyl(r_circ + 2.0, ch)
    band.apply_translation((0, 0, z_face - up * ch / 2))
    cone = frustum(r_ch + ch, r_ch, ch)
    if up > 0:
        cone.apply_translation((0, 0, z_face - ch))
    else:
        cone.apply_transform(tf.rotation_matrix(np.pi, (1, 0, 0)))
        cone.apply_translation((0, 0, z_face + ch))
    return sub(nut, sub(band, cone))


def countersink(nut, z_face, up, r_th, cs_r=4.5):
    """Cut a 45 degree bore lead-in down to a thread crest radius.

    The parviz M8 nut source used ``CS_R=4.5``.
    origin: parviz src/standins/m8_nut.py:110
    """
    d = cs_r - r_th
    cone = frustum(cs_r, r_th, d)
    if up > 0:
        cone.apply_transform(tf.rotation_matrix(np.pi, (1, 0, 0)))
        cone.apply_translation((0, 0, z_face))
    else:
        cone.apply_translation((0, 0, z_face))
    return sub(nut, cone)


def slot_neg(x, y, radius, height, zc, extra_travel=0.0):
    """Build a pedestal obround with 0.4 mm source travel plus extra travel.

    ``extra_travel`` extends each end beyond parviz's 0.4 mm
    ``pan_cd_adjust`` default.
    origin: parviz src/chassis.py:196
    """
    travel = 0.4 + extra_travel
    slot = trimesh.creation.extrude_polygon(
        sg.LineString([(x - travel, y), (x + travel, y)]).buffer(radius), height)
    slot.apply_translation((0, 0, zc - height / 2))
    return slot


def blind_socket(r, deep, out_dir, face_pt, overshoot=1.0):
    """Build a blind pin-socket negative opening through a wall face.

    origin: parviz src/geo.py:250
    """
    L = deep + overshoot
    m = cyl(r, L)
    m.apply_translation((0, 0, L / 2 - deep))
    orient(m, out_dir)
    m.apply_translation(face_pt)
    return m


def gable_roof(x0, xlen, yc, w, ztop, rise):
    """Build a two-slope self-supporting roof above a wall opening.

    origin: finnish-doors src/projects/klonk/housings.py:313
    """
    half = w / 2.0
    flat = max(1.0, half - rise)
    poly = sg.Polygon([(yc - half, ztop - 0.2), (yc + half, ztop - 0.2),
                       (yc + flat, ztop + rise), (yc - flat, ztop + rise)])
    m = trimesh.creation.extrude_polygon(poly, xlen)
    m.apply_transform(np.array([[0.0, 0.0, 1.0, x0],
                                [1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0]]))
    return m


def counterbore(d_hole, d_head, head_depth, length, axis="z", head_end="top"):
    """Build a through-hole plus cylindrical head-recess cutter.

    ``head_end`` selects the positive-axis ``top`` or negative-axis ``bottom``.
    origin: finnish-doors src/shared/fasteners.py:115
    """
    if d_hole <= 0 or d_head < d_hole or head_depth <= 0 or length <= 0:
        raise ValueError("counterbore(): dimensions must be positive and d_head >= d_hole")
    if head_depth > length:
        raise ValueError("counterbore(): head_depth must not exceed length")
    if head_end not in ("top", "bottom"):
        raise ValueError("counterbore(): head_end must be 'top' or 'bottom'")
    through = cyl(d_hole / 2.0, length, axis=axis)
    head = cyl(d_head / 2.0, head_depth, axis=axis)
    sign = 1.0 if head_end == "top" else -1.0
    vec = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
    head.apply_translation(np.asarray(vec, float) * sign * (length - head_depth) / 2.0)
    return uni([through, head])


_BEARINGS = {
    "608": (8.0, 22.0, 7.0),
    "695": (5.0, 13.0, 4.0),
    "MR105": (5.0, 10.0, 4.0),
}


def bearing_seat(kind, fit="press", open_column=True, extra_depth=0.0):
    """Build a Z-axis bearing-seat cutter for 608, 695, or MR105 bearings.

    Bearing tuples are bore, OD, and width in millimetres. ``press`` adds 0.25
    mm diametral pocket clearance, matching Klonk's 22.25 mm 608 pocket practice;
    ``slip`` adds 0.35 mm. An open column carries the OD pocket through the full
    cutter depth. A closed column cuts only the bearing width at OD and continues
    with the inner clearance bore, leaving a 1.2 mm retaining shoulder. Positive
    ``extra_depth`` extends the pocket below its nominal width.
    origin: finnish-doors src/projects/klonk/params.py:315
    """
    if kind not in _BEARINGS:
        raise ValueError("bearing_seat(): unknown bearing kind %r" % kind)
    if fit not in ("press", "slip"):
        raise ValueError("bearing_seat(): fit must be 'press' or 'slip'")
    if extra_depth < 0:
        raise ValueError("bearing_seat(): extra_depth must be non-negative")
    bore_d, outer_d, width = _BEARINGS[kind]
    pocket_d = outer_d + (0.25 if fit == "press" else 0.35)
    shoulder = 1.2
    total = width + extra_depth + shoulder
    if open_column:
        pocket = cyl(pocket_d / 2.0, total)
        pocket.apply_translation((0, 0, total / 2.0))
        return pocket
    pocket_h = width + extra_depth
    pocket = cyl(pocket_d / 2.0, pocket_h)
    pocket.apply_translation((0, 0, shoulder + pocket_h / 2.0))
    inner = cyl((bore_d + 0.35) / 2.0, total)
    inner.apply_translation((0, 0, total / 2.0))
    return uni([pocket, inner])


def crush_ribs(component_size, rib_thickness, rib_length, rib_height, count=2,
               interference=0.1, wall_axis="y", wall_offset=None, center=(0, 0, 0)):
    """Build tapered vertical crush ribs on opposing rectangular pocket walls.

    ``component_size`` is XYZ. ``wall_offset`` is the absolute coordinate of the
    two inner wall planes and defaults to the component half-span plus rib
    protrusion minus interference. ``count`` ribs are distributed on each wall.
    The buried base uses full ``rib_length`` while the tip land is 45 percent as
    long, preserving the Arachne-safe taper from Klonk. ``interference`` is the
    squeeze per side at the tip.
    origin: finnish-doors src/projects/klonk/motors.py:197
    """
    if wall_axis not in ("x", "y"):
        raise ValueError("crush_ribs(): wall_axis must be 'x' or 'y'")
    if min(rib_thickness, rib_length, rib_height) <= 0 or count < 1:
        raise ValueError("crush_ribs(): rib dimensions and count must be positive")
    size = np.asarray(component_size, float)
    ctr = np.asarray(center, float)
    ai = 0 if wall_axis == "x" else 1
    ti = 1 - ai
    half_component = size[ai] / 2.0
    if wall_offset is None:
        wall_offset = half_component + rib_thickness - interference
    wall_offset = float(wall_offset)
    if wall_offset <= half_component - interference:
        raise ValueError("crush_ribs(): wall planes must lie outside the squeezed component")
    span = max(0.0, size[ti] - rib_length)
    offsets = np.linspace(-span / 2.0, span / 2.0, count)
    embed = 0.45
    half_b, half_t = rib_length / 2.0, max(0.5, rib_length * 0.225)
    ribs = []
    for side in (-1.0, 1.0):
        base = ctr[ai] + side * (wall_offset + embed)
        tip = ctr[ai] + side * (half_component - interference)
        for offset in offsets:
            tangential = ctr[ti] + offset
            if wall_axis == "y":
                poly = sg.Polygon([(tangential - half_b, base),
                                   (tangential + half_b, base),
                                   (tangential + half_t, tip),
                                   (tangential - half_t, tip)])
            else:
                poly = sg.Polygon([(base, tangential - half_b),
                                   (base, tangential + half_b),
                                   (tip, tangential + half_t),
                                   (tip, tangential - half_t)])
            rib = trimesh.creation.extrude_polygon(poly, rib_height)
            rib.apply_translation((0.0, 0.0, ctr[2] - rib_height / 2.0))
            ribs.append(rib)
    return trimesh.util.concatenate(ribs)
