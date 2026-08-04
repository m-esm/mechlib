"""Printable threads, knurling, and spring preview mechanisms."""

import math

import numpy as np
import trimesh

from .meshutil import sub, uni
from .prim import cyl


COARSE_PITCH = {3.0: 0.8, 4.0: 1.0, 5.0: 0.8, 6.0: 1.0, 8.0: 1.25}
SQ3 = np.sqrt(3.0)


def coarse_pitch(nominal_d):
    """Return a printable coarse pitch for a supported nominal diameter.

    This keeps stock M8, M6, and M5 pitch where printable and coarsens M3/M4
    for a 0.4 mm nozzle.
    origin: parviz src/threads.py:43
    """
    return COARSE_PITCH[float(nominal_d)]


def _profile(d_nom, pitch, internal, clear):
    """Build one closed ISO 68-1 thread-turn profile in radius and Z."""
    H = SQ3 / 2.0 * pitch
    r_maj = d_nom / 2.0
    if internal:
        r_tip = r_maj + clear / 2.0
        r_val = r_maj - 5.0 * H / 8.0 + clear / 2.0
    else:
        r_tip = r_maj - clear / 2.0
        r_val = r_maj - 5.0 * H / 8.0 - clear / 2.0
    r_back = max(0.05, r_val - 0.6 * pitch)
    fc = pitch / 8.0
    zf = 5.0 * pitch / 16.0
    surf = [(r_val, -pitch / 2.0),
            (r_val, -fc / 2.0 - zf),
            (r_tip, -fc / 2.0),
            (r_tip, +fc / 2.0),
            (r_val, +fc / 2.0 + zf),
            (r_val, +pitch / 2.0)]
    poly = [(r_back, -pitch / 2.0)] + surf + [(r_back, +pitch / 2.0)]
    return np.array(poly), r_tip, r_val, r_back


def helix_solid(prof, lead, turns, seg):
    """Sweep a closed radius-Z polygon along a capped watertight helix.

    origin: parviz src/threads.py:90
    """
    k = len(prof)
    n = max(16, int(round(seg * turns)))
    th = np.linspace(0.0, turns * 2.0 * np.pi, n)
    verts = np.empty((n * k, 3))
    for i, a in enumerate(th):
        z0 = lead * a / (2.0 * np.pi)
        c, s = np.cos(a), np.sin(a)
        verts[i * k:(i + 1) * k, 0] = prof[:, 0] * c
        verts[i * k:(i + 1) * k, 1] = prof[:, 0] * s
        verts[i * k:(i + 1) * k, 2] = prof[:, 1] + z0
    faces = []
    for i in range(n - 1):
        for j in range(k):
            a0, b0 = i * k + j, i * k + (j + 1) % k
            a1, b1 = (i + 1) * k + j, (i + 1) * k + (j + 1) % k
            faces.append((a0, b0, b1))
            faces.append((a0, b1, a1))
    fan = [(0, j, j + 1) for j in range(1, k - 1)]
    faces += [(f[0], f[2], f[1]) for f in fan]
    off = (n - 1) * k
    faces += [(off + f[0], off + f[1], off + f[2]) for f in fan]
    m = trimesh.Trimesh(vertices=verts, faces=np.array(faces), process=False)
    m.fix_normals()
    return m


def thread_solid(d_nom, length, pitch=None, internal=False, clear=0.25,
                 starts=1, seg=64):
    """Build a printable ISO 60 degree external thread or internal cutter.

    External mode returns a threaded rod. Internal mode returns its clearance-
    grown negative for subtraction from a nut or boss.
    origin: parviz src/threads.py:124
    """
    pitch = pitch or coarse_pitch(d_nom)
    prof, r_tip, r_val, r_back = _profile(d_nom, pitch, internal, clear)
    lead = pitch * starts
    turns = length / lead + 2.0
    band = helix_solid(prof, lead, turns, seg)
    band.apply_translation((0, 0, -pitch))
    core = cyl(r_val + 0.02, length + 6.0 * pitch)
    core.apply_translation((0, 0, length / 2.0))
    solid = uni([band, core])
    lo = cyl(d_nom * 2.0 + 4.0, 6.0 * pitch)
    lo.apply_translation((0, 0, -3.0 * pitch))
    hi = cyl(d_nom * 2.0 + 4.0, 6.0 * pitch)
    hi.apply_translation((0, 0, length + 3.0 * pitch))
    return sub(sub(solid, lo), hi)


def tap(mesh, d_nom, at, length, pitch=None, clear=0.25, axis="z"):
    """Cut a printable internal thread into a mesh at a point and axis.

    origin: parviz src/threads.py:151
    """
    cut = thread_solid(d_nom, length, pitch=pitch, internal=True, clear=clear)
    if axis == "y":
        cut.apply_transform(trimesh.transformations.rotation_matrix(
            -np.pi / 2.0, (1, 0, 0)))
    elif axis == "x":
        cut.apply_transform(trimesh.transformations.rotation_matrix(
            np.pi / 2.0, (0, 1, 0)))
    cut.apply_translation(at)
    return sub(mesh, cut)


def knurl(mesh, r, z0, z1, n=18, depth=0.35):
    """Cut evenly spaced vertical grip flutes around a cylindrical mesh.

    origin: parviz src/standins/m4_bolt.py:107
    """
    for i in range(n):
        a = 2.0 * np.pi * i / n
        f = cyl(0.5, (z1 - z0) + 0.2, sections=12)
        f.apply_translation(((r + 0.5 - depth) * np.cos(a),
                             (r + 0.5 - depth) * np.sin(a),
                             (z0 + z1) / 2.0))
        mesh = sub(mesh, f)
    return mesh


def torsion_spring_mesh(mean_r=6.8, wire=1.2, turns=5.0, z=(0.5, 7.5),
                        leg_r=1.4, anchor_at=(9.5, -60.0), moving_leg_z=10.5):
    """Build a representative torsion spring coil and its two straight legs.

    Klonk defaults were ``ts_mean_r=6.8``, ``ts_wire=1.2``, ``ts_turns=5.0``,
    ``ts_z=(0.5, 7.5)``, ``ts_leg_r=1.4``, and
    ``ts_anchor_at=(9.5, -60.0)``. ``moving_leg_z=10.5`` binds the source's
    derived ``lm_rib_band[0] + 2.5`` endpoint. ``leg_r`` is retained as an
    explicit source parameter for coordinating the matching anchor holes.
    origin: finnish-doors src/projects/klonk/shaft.py:395
    """
    z0, z1 = z
    n = 160
    t = np.linspace(0, turns*2*math.pi, n); zs = np.linspace(z0, z1, n)
    p = np.c_[mean_r*np.cos(t), mean_r*np.sin(t), zs]
    segs = [trimesh.creation.cylinder(radius=wire/2, segment=[p[i], p[i+1]])
            for i in range(n-1)]
    segs.append(trimesh.creation.cylinder(
        radius=wire/2, segment=[[mean_r, 0, z1], [mean_r, 0, moving_leg_z]]))
    ar, aa = anchor_at
    segs.append(trimesh.creation.cylinder(
        radius=wire/2,
        segment=[[mean_r, 0, z0],
                 [ar*math.cos(math.radians(aa)), ar*math.sin(math.radians(aa)), z0-1.5]]))
    return trimesh.util.concatenate(segs)
