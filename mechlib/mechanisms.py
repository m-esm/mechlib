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
                 starts=1, seg=96):
    """Build a printable ISO 60 degree external thread or internal cutter.

    External mode returns a threaded rod. Internal mode returns its clearance-
    grown negative for subtraction from a nut or boss. ``seg`` below 96 leaves
    a visible sawtooth on the crests under smoothed-normal rendering.
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


def helix_tube(R, rw, turns, z0, z1, M=16, N=420):
    """Sweep a capped solid tube along a helix and return a watertight mesh.

    The moving frame uses a radial-inward normal, avoiding accumulated frame
    twist. ``R`` is centerline radius, ``rw`` wire radius, ``turns`` may be
    fractional, and the helix advances from ``z0`` to ``z1``.
    origin: finnish-doors wrap_demo.py at 75ca785
    """
    if R <= 0 or rw <= 0 or turns == 0 or M < 3 or N < 2:
        raise ValueError("helix_tube(): invalid radius, turns, or resolution")
    theta = np.linspace(0.0, 2.0 * np.pi * turns, N)
    z = np.linspace(z0, z1, N)
    centerline = np.stack([R * np.cos(theta), R * np.sin(theta), z], axis=1)
    tangent = np.stack([
        -R * np.sin(theta),
        R * np.cos(theta),
        np.full_like(theta, (z1 - z0) / (2.0 * np.pi * turns)),
    ], axis=1)
    tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
    normal = np.stack([
        -np.cos(theta), -np.sin(theta), np.zeros_like(theta)
    ], axis=1)
    binormal = np.cross(tangent, normal)
    binormal /= np.linalg.norm(binormal, axis=1, keepdims=True)
    normal = np.cross(binormal, tangent)

    phi = np.linspace(0.0, 2.0 * np.pi, M, endpoint=False)
    rings = (
        normal[:, None, :] * np.cos(phi)[None, :, None]
        + binormal[:, None, :] * np.sin(phi)[None, :, None]
    ) * rw + centerline[:, None, :]
    vertices = rings.reshape(-1, 3)
    faces = []
    for i in range(N - 1):
        for j in range(M):
            a = i * M + j
            b = i * M + (j + 1) % M
            c = (i + 1) * M + j
            d = (i + 1) * M + (j + 1) % M
            faces.extend(((a, b, d), (a, d, c)))
    for index, base in ((0, 0), (N - 1, (N - 1) * M)):
        center_index = len(vertices)
        vertices = np.vstack([vertices, centerline[index]])
        for j in range(M):
            a = base + j
            b = base + (j + 1) % M
            faces.append((center_index, a, b) if index == 0
                         else (center_index, b, a))
    mesh = trimesh.Trimesh(vertices=vertices, faces=np.asarray(faces), process=False)
    mesh.fix_normals()
    return mesh


def dog_slot_coupling(r_in=6.0, r_out=12.0, dog_deg=30.0,
                      free_deg=50.0, boss_h=7.0, collar_h=5.0,
                      slot_a_clear=1.0, slot_z_clear=0.6,
                      bore_r=4.5, boss_wall=1.5, collar_overhang=2.5,
                      sections=64):
    """Build angular lost motion as ``(slotted_boss, dog_collar)`` meshes.

    The boss is an annulus with a through arc slot. The separate collar carries
    a downward radial dog, with its rest face against the drive wall and
    ``free_deg`` of relative rotation toward release. Both pieces have a round
    bore for consumers to replace or subtract with their own keyed interface.
    Looking from +Z, free travel is toward positive angle.

    origin: finnish-doors coupling_variants/build_coupling.py at 9de406d^
    """
    from shapely.geometry import Point

    from .prim import sector2d

    if (not 0 < bore_r < r_in < r_out or dog_deg <= 0 or free_deg < 0 or
            boss_h <= 0 or collar_h <= 0 or slot_a_clear < 0 or
            not 0 <= slot_z_clear < boss_h or boss_wall <= 0 or
            collar_overhang <= 0 or sections < 12):
        raise ValueError("dog_slot_coupling(): invalid dimensions")
    dog_lo, dog_hi = -dog_deg / 2.0, dog_deg / 2.0
    slot_lo = dog_lo - slot_a_clear
    slot_hi = dog_hi + free_deg + slot_a_clear
    slot_inner = r_in - 0.6
    if slot_inner <= bore_r:
        raise ValueError("dog_slot_coupling(): slot breaks through the bore wall")

    bore = Point(0, 0).buffer(bore_r, resolution=sections)
    boss_poly = Point(0, 0).buffer(
        r_out + boss_wall, resolution=sections).difference(bore)
    slot = sector2d(slot_lo, slot_hi, r_out + 0.6, n=sections).difference(
        Point(0, 0).buffer(slot_inner, resolution=sections))
    boss_poly = boss_poly.difference(slot).buffer(0)
    boss = trimesh.creation.extrude_polygon(boss_poly, boss_h)

    collar_poly = Point(0, 0).buffer(
        r_out + collar_overhang, resolution=sections).difference(bore).buffer(0)
    collar = trimesh.creation.extrude_polygon(collar_poly, collar_h)
    collar.apply_translation((0, 0, boss_h))
    dog_poly = sector2d(dog_lo, dog_hi, r_out, n=sections).difference(
        Point(0, 0).buffer(r_in, resolution=sections))
    dog_h = boss_h - slot_z_clear + 0.05
    dog = trimesh.creation.extrude_polygon(dog_poly, dog_h)
    dog.apply_translation((0, 0, slot_z_clear))
    collar = uni([collar, dog])
    return boss, collar


def threaded_rod(major_d, pitch, length, minor_d=None, n_theta=64,
                 steps_per_pitch=6):
    """Build a fast radial-grid external ISO-style thread.

    This is a display and light-duty alternative. ``thread_solid`` remains the
    robust cutter source for tapping.
    origin: wall-shelf-clamp lib.py:165
    """
    major_r = major_d / 2.0
    minor_r = (minor_d / 2.0) if minor_d else (major_r - 0.6134 * pitch)

    def rad(z_eff):
        t = (z_eff % pitch) / pitch
        if t < 0.45:   return minor_r + (major_r - minor_r) * (t / 0.45)
        elif t < 0.50: return major_r
        elif t < 0.95: return major_r - (major_r - minor_r) * ((t - 0.50) / 0.45)
        else:          return minor_r

    nz = max(2, int(length / pitch * steps_per_pitch))
    thetas = np.linspace(0, 2 * np.pi, n_theta, endpoint=False)
    zs = np.linspace(0, length, nz)
    R = np.empty((nz, n_theta))
    for i, z in enumerate(zs):
        for j, th in enumerate(thetas):
            R[i, j] = rad(z - pitch * th / (2 * np.pi))
    X = R * np.cos(thetas)[None, :]
    Y = R * np.sin(thetas)[None, :]
    Z = np.repeat(zs[:, None], n_theta, axis=1)
    verts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)
    faces = []
    idx = lambda i, j: i * n_theta + (j % n_theta)
    for i in range(nz - 1):
        for j in range(n_theta):
            a, b = idx(i, j), idx(i, j + 1)
            c, d = idx(i + 1, j), idx(i + 1, j + 1)
            faces.append([a, b, d]); faces.append([a, d, c])
    cb = len(verts); verts = np.vstack([verts, [0, 0, zs[0]]])
    ct = len(verts); verts = np.vstack([verts, [0, 0, zs[-1]]])
    for j in range(n_theta):
        faces.append([cb, idx(0, j + 1), idx(0, j)])
        faces.append([ct, idx(nz - 1, j), idx(nz - 1, j + 1)])
    m = trimesh.Trimesh(verts, np.array(faces), process=True)
    m.fix_normals()
    return m
