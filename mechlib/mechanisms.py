"""Printable threads, knurling, and spring preview mechanisms."""

import math

import numpy as np
import trimesh

from .meshutil import sub, uni
from .prim import boxc, cyl, frustum


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


_SCREW_CLEAR = {"M3": 3.4, "M4": 4.5, "M5": 5.5, "M6": 6.6}


def _metric_d(name):
    """Parse 'M4' style thread designations to a nominal diameter."""
    try:
        d = float(str(name).upper().lstrip("M"))
    except ValueError:
        raise ValueError("thread designation must look like 'M4'")
    if d <= 0:
        raise ValueError("thread designation must look like 'M4'")
    return d


def shaft_collar(bore_d=8.0, od=None, width=10.0, style="split",
                 screw="M4", slit_w=1.2, clear=0.2, sections=96):
    """Build a shaft collar for axially locating bearings, gears and sprockets.

    Two stock styles: ``style='split'`` is a clamping collar -- one radial
    slit of ``slit_w`` opens the ring and a tangential ``screw`` clearance
    hole (``'M3'``..``'M6'``) through a lug astride the slit pinches the bore
    closed on the shaft, which clamps surprisingly well in PETG/PLA;
    ``style='setscrew'`` is a one-piece ring with a radial
    ``closures.setscrew`` boss at mid-width. ``od`` defaults to
    ``bore_d + 8``. The bore is printed at ``bore_d + clear`` for an easy
    slip fit that the clamp then closes. Prints upright with the slit
    horizontal, no support. Units are mm.
    """
    if bore_d <= 0 or width <= 0 or clear < 0:
        raise ValueError("shaft_collar(): bore_d and width must be positive")
    if od is None:
        od = bore_d + 8.0
    bore_r = (bore_d + clear) / 2.0
    if od / 2.0 - bore_r < 2.4:
        raise ValueError("shaft_collar(): wall below 2.4 mm around the bore")
    if style not in ("split", "setscrew"):
        raise ValueError("shaft_collar(): style must be 'split' or "
                         "'setscrew'")
    if slit_w < 0.6:
        raise ValueError("shaft_collar(): slit_w must be at least 0.6 mm")
    hole_d = _SCREW_CLEAR.get(str(screw).upper())
    if hole_d is None:
        raise ValueError("shaft_collar(): screw must be one of %s"
                         % (sorted(_SCREW_CLEAR),))

    ring = trimesh.creation.annulus(bore_r, od / 2.0, width,
                                    sections=int(sections))
    ring.apply_translation((0.0, 0.0, width / 2.0))

    if style == "split":
        slit = boxc((od / 2.0 - bore_r + 1.5, slit_w, width + 2.0),
                    center=((bore_r + od / 2.0) / 2.0 + 0.25, 0.0,
                            width / 2.0))
        lug_r = hole_d + 4.0
        lug_w = hole_d + 6.0
        lug = boxc((lug_r, lug_w, width),
                   center=(od / 2.0 + lug_r / 2.0 - 1.5, 0.0, width / 2.0))
        body = uni([ring, lug])
        body = sub(body, slit)
        screw_hole = cyl(hole_d / 2.0, lug_w + 4.0, axis="y",
                         center=(od / 2.0 + 0.5, 0.0, width / 2.0),
                         sections=32)
        body = sub(body, screw_hole)
    else:
        from .closures import setscrew
        point = np.array([od / 2.0, 0.0, width / 2.0])
        direction = np.array([-1.0, 0.0, 0.0])
        boss, hole = setscrew(point, direction, into=od / 2.0,
                              hole_d=hole_d, boss_d=hole_d + 5.0,
                              boss_h=3.0, sections=32)
        body = sub(uni([ring, boss]), hole)

    body.metadata.update({
        "bore_d": float(bore_d + clear),
        "od": float(od),
        "width": float(width),
        "style": style,
        "screw": str(screw).upper(),
    })
    return body


def star_knob(lobes=5, d=32.0, h=18.0, thread="M6", through=False,
              boss_d=14.0, sections=96):
    """Build a DIN 6336 style lobed clamping knob with a threaded core.

    The standard operating element of jigs, fixtures and machine adjustment:
    ``lobes`` rounded gripping lobes around a central boss, with an internal
    ISO thread (printed via ``tap``) of designation ``thread`` (``'M4'``..
    ``'M8'``). ``through=False`` cuts a blind thread from the bottom face
    leaving a 2 mm crown, ``through=True`` threads the full height for a
    stud. Print upright on the flat bottom face, no support. Units are mm.
    """
    import shapely.geometry as sg
    from shapely.ops import unary_union

    lobes = int(round(lobes))
    if lobes < 3:
        raise ValueError("star_knob(): need at least 3 lobes")
    if d <= 0 or h <= 0:
        raise ValueError("star_knob(): d and h must be positive")
    d_thread = _metric_d(thread)
    if not 3.0 <= d_thread <= 8.0:
        raise ValueError("star_knob(): thread must be M3..M8")
    if boss_d < d_thread + 4.0:
        raise ValueError("star_knob(): boss_d leaves under 2 mm around the "
                         "thread")
    if boss_d >= d - 2.0:
        raise ValueError("star_knob(): boss_d must be smaller than d")
    if h < 0.6 * d_thread + 2.0:
        raise ValueError("star_knob(): h too short for the thread")

    r_out = d / 2.0
    sinp = math.sin(math.pi / lobes)
    r_lobe = r_out * sinp / (1.0 + sinp)
    r_centre = r_out - r_lobe
    r_core = max(boss_d / 2.0, r_centre - 0.3 * r_lobe)
    polys = [sg.Point(0.0, 0.0).buffer(r_core, resolution=int(sections) // 4)]
    for k in range(lobes):
        a = 2.0 * math.pi * k / lobes
        polys.append(sg.Point(r_centre * math.cos(a),
                              r_centre * math.sin(a)).buffer(
                                  r_lobe, resolution=int(sections) // 8))
    body = trimesh.creation.extrude_polygon(unary_union(polys).buffer(0), h)

    tap_len = h if through else h - 2.0
    body = tap(body, d_thread, (0.0, 0.0, 0.0), tap_len, axis="z")

    body.metadata.update({
        "lobes": int(lobes),
        "d": float(d),
        "h": float(h),
        "thread": str(thread).upper(),
        "through": bool(through),
    })
    return body


def handwheel(d=100.0, spokes=3, rim_w=12.0, rim_t=None, bore_d=8.0,
              clear=0.2, crank=False, grip_d=14.0, grip_len=40.0,
              pin_d=6.0, sections=96):
    """Build a spoked handwheel (DIN 950 style) for leadscrews and valves.

    The manual input wheel of screw jacks, valve stems and indexing tables;
    mates naturally with ``linear.screw_jack`` and ``fluid.rotary_spool_valve``.
    A rim of axial width ``rim_w`` and radial thickness ``rim_t`` (default
    8% of ``d``) is joined to the hub by ``spokes``; the bore is printed at
    ``bore_d + clear``. With ``crank=True`` a pin of ``pin_d`` rises from the
    rim and a separate free-spinning grip (``grip_d`` x ``grip_len``, bored
    at a 0.3 mm running clearance) is returned posed on it: the pin ends in
    a 45 degree snap head and the grip mouth is chamfered to flex over it, so
    the grip is captured but spins freely. Returns the wheel mesh alone, or
    ``{'wheel', 'grip'}`` when ``crank=True``. Print the wheel face-down and
    the grip open-end down, no support. Units are mm.
    """
    import shapely.geometry as sg

    if d <= 0 or rim_w <= 0 or bore_d <= 0 or clear < 0:
        raise ValueError("handwheel(): d, rim_w and bore_d must be positive")
    spokes = int(round(spokes))
    if spokes < 2:
        raise ValueError("handwheel(): need at least 2 spokes")
    if rim_t is None:
        rim_t = max(6.0, 0.08 * d)
    bore_r = (bore_d + clear) / 2.0
    hub_d = bore_d + clear + 8.0
    rim_in_r = d / 2.0 - rim_t
    if rim_t < 4.0:
        raise ValueError("handwheel(): rim_t must be at least 4.0 mm")
    if rim_in_r - hub_d / 2.0 < 2.0:
        raise ValueError("handwheel(): no room for spokes between hub and "
                         "rim")
    spoke_w = max(4.0, 0.5 * rim_w)
    rim_gap = (2.0 * math.pi * rim_in_r / spokes) - spoke_w
    if rim_gap < 1.0:
        raise ValueError("handwheel(): %d spokes leave no opening at the rim"
                         % spokes)

    rim = trimesh.creation.annulus(rim_in_r, d / 2.0, rim_w,
                                   sections=int(sections))
    rim.apply_translation((0.0, 0.0, rim_w / 2.0))
    hub = cyl(hub_d / 2.0, rim_w, center=(0, 0, rim_w / 2.0),
              sections=int(sections))
    parts = [rim, hub]
    span = rim_in_r - hub_d / 2.0
    import trimesh.transformations as tf
    for k in range(spokes):
        sp = boxc((span + 2.0, spoke_w, rim_w),
                  center=(hub_d / 2.0 + span / 2.0, 0.0, rim_w / 2.0))
        sp.apply_transform(tf.rotation_matrix(
            2.0 * math.pi * k / spokes, (0, 0, 1)))
        parts.append(sp)
    wheel = uni(parts)

    meta = {
        "d": float(d),
        "spokes": int(spokes),
        "rim_w": float(rim_w),
        "bore_d": float(bore_d + clear),
        "crank": bool(crank),
    }

    if not crank:
        wheel = sub(wheel, cyl(bore_r, rim_w + 4.0,
                               center=(0, 0, rim_w / 2.0),
                               sections=int(sections)))
        wheel.metadata.update(meta)
        return wheel

    if pin_d <= 0 or grip_d < pin_d + 4.0 or grip_len < pin_d * 3.0:
        raise ValueError("handwheel(): bad crank grip dimensions")
    grip_clear = 0.3
    pin_len = grip_len * 0.7
    head_r = pin_d / 2.0 + 1.0
    head_h = head_r - pin_d / 2.0  # 45 degree snap head
    r_pin = d / 2.0 - rim_t / 2.0
    pin = cyl(pin_d / 2.0, pin_len,
              center=(r_pin, 0.0, rim_w + pin_len / 2.0), sections=48)
    head = frustum(pin_d / 2.0, head_r, head_h,
                   z0=rim_w + pin_len, sections=48)
    head.apply_translation((r_pin, 0.0, 0.0))
    wheel = uni([wheel, pin, head])
    wheel = sub(wheel, cyl(bore_r, rim_w + 4.0,
                           center=(0, 0, rim_w / 2.0),
                           sections=int(sections)))

    # Grip: closed-end tube snapped over the pin head, posed in place.
    bore_gr = pin_d / 2.0 + grip_clear / 2.0
    cavity_d = pin_len + head_h + 0.5
    grip = cyl(grip_d / 2.0, grip_len, center=(0, 0, grip_len / 2.0),
               sections=int(sections))
    mouth = frustum(bore_gr + 0.6, bore_gr, 0.6, z0=0.0, sections=48)
    bore = cyl(bore_gr, cavity_d, center=(0, 0, cavity_d / 2.0),
               sections=48)
    socket = frustum(bore_gr, head_r + 0.15, head_h,
                     z0=cavity_d - head_h, sections=48)
    grip = sub(grip, uni([mouth, bore, socket]))
    grip.apply_translation((r_pin, 0.0, rim_w))
    wheel.metadata.update(meta)
    grip.metadata.update(meta)
    return {"wheel": wheel, "grip": grip}
