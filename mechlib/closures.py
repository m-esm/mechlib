"""Press-fit, snap-fit, captive-nut, and locating closure features."""

from dataclasses import dataclass

import numpy as np
import shapely.geometry as sg
import trimesh
import trimesh.transformations as tf

from .meshutil import orient, sub, uni
from .prim import boxc, cyl, extrude_down, rbox


def press_lid(ox, oy, cav_x, cav_y, c, plate_t=2.4, plug_h=5.0,
              plug_wall=1.8, edge_r=5.5, clear=0.10):
    """Build a plate and hollow friction plug in a local mating-plane frame.

    ``edge_r=5.5`` and per-side ``clear=0.10`` are the Klonk source defaults.
    origin: finnish-doors src/projects/klonk/system_layout.py:370
    """
    cx, cy = c
    plate  = rbox([ox, oy, plate_t], (cx, cy, plate_t/2.0), r=edge_r)
    pox, poy = cav_x - 2*clear, cav_y - 2*clear
    plug_o = rbox([pox, poy, plug_h], (cx, cy, -plug_h/2.0),
                  r=max(2.0, edge_r - 1.0))
    plug_i = boxc([pox - 2*plug_wall, poy - 2*plug_wall, plug_h + 2],
                  (cx, cy, -plug_h/2.0))
    return trimesh.boolean.union([plate, trimesh.boolean.difference([plug_o, plug_i])])


def clamshell_shiplap(outer_solid, clear=0.05):
    """Build the base lip and matching lid slot for a clamshell perimeter.

    ``clear=0.05`` is Klonk's ``PEG_CLEAR`` per-side source default.
    origin: finnish-doors src/projects/klonk/housings.py:283
    """
    sec = outer_solid.section(plane_origin=[0, 0, 0], plane_normal=[0, 0, 1])
    loops = [np.asarray(d)[:, :2] for d in sec.discrete]
    area = lambda L: 0.5*abs(np.dot(L[:, 0], np.roll(L[:, 1], 1)) - np.dot(L[:, 1], np.roll(L[:, 0], 1)))
    ext = sg.Polygon(max(loops, key=area))
    t_in, t_w, t_h = 0.6, 1.2, 4.0
    lip2d    = ext.buffer(-t_in).difference(ext.buffer(-(t_in + t_w)))
    slot2d   = ext.buffer(-(t_in - clear)).difference(ext.buffer(-(t_in + t_w + clear)))
    base_lip = extrude_down(lip2d, t_h, top=t_h)
    lid_slot = extrude_down(slot2d, t_h + 0.4, top=t_h + 0.2)
    return base_lip, lid_slot


def ydovetail(xc, y0, y1, clear=0.0, wide=10.0, w=6.0, h=5.0, z=14.0):
    """Build a Y-axis dovetail prism with a print-safe flared top.

    Defaults bind Klonk ``join_dt_wide=10``, ``join_dt_w=6``,
    ``join_dt_h=5``, and ``join_z=14``.
    origin: finnish-doors src/projects/klonk/system_layout.py:358
    """
    wd, nw = wide + 2*clear, w + 2*clear
    prof = sg.Polygon([(-nw/2, 0), (nw/2, 0), (wd/2, h), (-wd/2, h)])
    m = trimesh.creation.extrude_polygon(prof, y1 - y0)
    m.apply_transform(tf.rotation_matrix(np.pi/2, [1, 0, 0]))
    m.apply_translation([xc, y1, z - h/2])
    return m


@dataclass(frozen=True)
class SnapSpec:
    """Dimensions for the unified Klonk catch and cantilever finger.

    Byte-identical copies existed in ``system_layout.py`` and
    ``build_powerbank.py``. Defaults bind ``SNAP_AW=8.0``, ``SNAP_AT=1.6``,
    ``SNAP_GAP=0.4``, ``SNAP_DROP=6.0``, and ``SNAP_HOOK=1.5``.
    origin: finnish-doors src/projects/klonk/system_layout.py:325
    """

    aw: float = 8.0
    at: float = 1.6
    gap: float = 0.4
    drop: float = 6.0
    hook: float = 1.5


def snap_catch(axis, fc, t, sgn, box_top, spec=SnapSpec()):
    """Build the ramped outer-wall catch for a snap closure.

    origin: finnish-doors src/projects/klonk/system_layout.py:325
    origin: finnish-doors src/projects/klonk/build_powerbank.py:45
    """
    zc, PROT = box_top - 4.0, 2.4
    tri = sg.Polygon([(0, -0.9), (sgn*PROT, -0.9), (0, 0.9)])
    m = trimesh.creation.extrude_polygon(tri, spec.aw)
    m.apply_transform(tf.rotation_matrix(np.pi/2, [1, 0, 0]))
    m.apply_translation([0, spec.aw/2, 0])
    if axis == 'x':
        m.apply_translation([fc, t, zc])
    else:
        m.apply_transform(tf.rotation_matrix(-np.pi/2, [0, 0, 1]))
        m.apply_translation([t, fc, zc])
    return m


def snap_finger(axis, fc, t, sgn, mate_z, spec=SnapSpec()):
    """Build a bridging cantilever finger and inward retention hook.

    origin: finnish-doors src/projects/klonk/system_layout.py:342
    origin: finnish-doors src/projects/klonk/build_powerbank.py:56
    """
    ax = fc + sgn*(spec.gap + spec.at/2)
    bc, bw = fc + sgn*1.0, (spec.gap + spec.at)
    z0 = mate_z - spec.drop
    if axis == 'x':
        bridge = boxc([bw, spec.aw, 2.6], (bc, t, mate_z + 1.3))
        arm    = boxc([spec.at, spec.aw, spec.drop + 2],
                      (ax, t, mate_z - spec.drop/2 + 1))
        hook   = boxc([spec.at + spec.hook, spec.aw, 1.4],
                      (ax - sgn*spec.hook/2, t, z0 + 0.9))
    else:
        bridge = boxc([spec.aw, bw, 2.6], (t, bc, mate_z + 1.3))
        arm    = boxc([spec.aw, spec.at, spec.drop + 2],
                      (t, ax, mate_z - spec.drop/2 + 1))
        hook   = boxc([spec.aw, spec.at + spec.hook, 1.4],
                      (t, ax - sgn*spec.hook/2, z0 + 0.9))
    return trimesh.boolean.union([bridge, arm, hook])


NUT = {"M2": (4.0, 1.6), "M2.5": (5.0, 2.0), "M3": (5.5, 2.6), "M4": (7.0, 3.2)}
NUT_NIB_PROUD = 0.25
NUT_NIB_RUN = 1.2
NUT_NIB_SEAT_CLEAR = 0.1
NUT_NIB_MIN_EXTRA = 1.5


def nut_ac(size="M3"):
    """Return ISO-style hex nut across-corners span from across-flats data.

    The module ``NUT`` table carries common ISO across-flats and thickness sizes.
    origin: parviz src/geo.py:141
    """
    return NUT[size][0] * 2.0 / np.sqrt(3.0)


def nut_slot(center, screw_axis="z", open_dir=(0, 1, 0), size="M3",
             length=14.0, c_af=0.2, c_t=0.2, seat=True, nib=False):
    """Build a seated slide-in captive hex-nut negative with optional crush nibs.

    The ISO nut size table and 0.25 mm by 1.2 mm nib defaults are module data.
    origin: parviz src/geo.py:148
    """
    axv = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}.get(screw_axis, screw_axis)
    a = np.asarray(axv, float); a /= np.linalg.norm(a)
    o = np.asarray(open_dir, float); o /= np.linalg.norm(o)
    if abs(np.dot(a, o)) > 1e-6:
        raise ValueError("nut_slot(): open_dir must be perpendicular to screw_axis")
    af, nut_t = NUT[size]
    back = nut_ac(size) / 2.0 if seat else length / 2.0
    if length <= back:
        raise ValueError("nut_slot(): length %.2f must exceed the nut seat %.2f"
                         % (length, back))
    if nib and (not seat or length < nut_ac(size) + NUT_NIB_MIN_EXTRA):
        raise ValueError("nut_slot(): nib needs a seated length >= %.2f, got %.2f"
                         % (nut_ac(size) + NUT_NIB_MIN_EXTRA, length))
    b = boxc((af + c_af, length, nut_t + c_t))
    T = np.eye(4)
    T[:3, 0] = np.cross(o, a); T[:3, 1] = o; T[:3, 2] = a
    T[:3, 3] = np.asarray(center, float) + o * (length / 2.0 - back)
    b.apply_transform(T)
    if nib:
        y0 = -length / 2.0 + nut_ac(size) + NUT_NIB_SEAT_CLEAR
        y1 = y0 + NUT_NIB_RUN
        back_ramp = NUT_NIB_PROUD / np.tan(np.deg2rad(60.0))
        lead_ramp = NUT_NIB_PROUD
        half_w = (af + c_af) / 2.0
        ribs = []
        for side in (-1.0, 1.0):
            wall = side * half_w
            inner = side * (half_w - NUT_NIB_PROUD)
            poly = sg.Polygon([(wall, y0), (wall, y1),
                               (inner, y1 - lead_ramp),
                               (inner, y0 + back_ramp)])
            rib = trimesh.creation.extrude_polygon(poly, nut_t + c_t)
            rib.apply_translation((0, 0, -(nut_t + c_t) / 2.0))
            rib.apply_transform(T)
            ribs.append(rib)
        b = sub(b, uni(ribs))
    return b


def screw_post(pos, normal, depth, boss_r=4.3):
    """Build a cylindrical boss along a face normal.

    ``boss_r=4.3`` is the parviz ``P['boss_r']`` source default.
    origin: parviz src/geo.py:230
    """
    m = cyl(boss_r, depth); m.apply_translation((0, 0, depth / 2))
    orient(m, normal); m.apply_translation(pos)
    return m


def fix_pin(r, length, direction, face_pt, bury=1.0):
    """Build a locating pin with enough burial to fuse into its part.

    origin: parviz src/geo.py:237
    """
    L = length + bury
    m = cyl(r, L)
    m.apply_translation((0, 0, L / 2 - bury))
    orient(m, direction)
    m.apply_translation(face_pt)
    return m


def setscrew(point, direction, into=14.0, hole_d=4.2,
             boss_d=12.0, boss_h=4.0, sections=32):
    """Build a locking set-screw boss and its inward pilot hole.

    Returns ``(boss, hole)`` for union then subtraction.
    origin: wall-shelf-clamp lib.py:107
    """
    p = np.asarray(point, float)
    d = np.asarray(direction, float); d = d / np.linalg.norm(d)
    boss = cyl(boss_d / 2.0, boss_h, sections=sections)
    orient(boss, d)
    boss.apply_translation(p - d * (boss_h / 2.0))
    H = into + boss_h + 2.0
    hole = cyl(hole_d / 2.0, H, sections=sections)
    orient(hole, d)
    hole.apply_translation(p + d * ((into - boss_h) / 2.0))
    return boss, hole


def push_pin(d, length, barb=True, axis="z", flip=False):
    """Build a barbed press pin with a conical lead-in.

    The pin runs from Z zero toward positive Z before optional orientation.
    origin: dual-axis-turntable src/build.py:131
    """
    parts = []
    shaft = cyl(d / 2, length, sections=32)
    shaft.apply_translation([0, 0, length / 2])
    parts.append(shaft)
    if barb:
        barb_r = d / 2 + 0.7
        b = cyl(barb_r, 2.2, sections=32)
        b.apply_translation([0, 0, length - 4.0])
        tip = trimesh.creation.cone(radius=barb_r, height=4.0, sections=32)
        tip.apply_translation([0, 0, length - 2.0])
        parts += [b, tip]
    m = trimesh.util.concatenate(parts)
    if flip:
        m.apply_transform(tf.rotation_matrix(np.pi, [1, 0, 0]))
    if axis == "y":
        m.apply_transform(tf.rotation_matrix(np.pi / 2, [1, 0, 0]))
    elif axis == "x":
        m.apply_transform(tf.rotation_matrix(np.pi / 2, [0, 1, 0]))
    return m
