"""Pure gear generators.

``spur_gear_2d`` is the canonical 2D involute profile,
``spur_gear_mesh`` is the simple extrude-and-bore solid, and ``spur_gear`` is
the full-featured 3D helical, sector, and hub generator.
"""
import math, numpy as np


def spur_gear_2d(N, m, sector=None, relief=2.5, center_at=-90.0, entry_drop=0.0, entry_end=-1,
                 bl=None, t_relief=None, pa=20.0):
    """INVOLUTE spur-gear 2D profile (shapely). N teeth, module m, `pressure_angle`°, `backlash` tooth
    thinning → true conjugate meshing. Teeth are centred at k·(2π/N) so mesh_phase() still applies. If
    `sector` is an arc width (deg), only teeth whose centre is within that arc (about `center_at`°) are
    kept and the off-arc rim is recessed by `relief` so a meshing follower spins free there.
    entry_drop > 0 (sector only): the ENTRY tooth, the arc-edge tooth on the `entry_end` side (−1 = the
    low-angle edge, which leads when the driver rotates in −θ = the opening direction), has its tip cut
    down radially by entry_drop, the classic relieved-entry-tooth for gentle re-engagement.
    bl defaults to 0.35, t_relief to 0.10, and pa to 20.0.
    Defaults mirror the Klonk values (params.py pressure_angle=20.0, backlash=0.35, tip_relief=0.10);
    always pass explicitly in projects. Explicit, no hidden globals."""
    import shapely.geometry as sg
    from shapely.ops import unary_union
    a  = math.radians(pa)
    _bl = 0.35 if bl is None else bl                # per-gear backlash override (planetary runs tighter)
    _tr = 0.10 if t_relief is None else t_relief
    rp = m*N/2.0; rb = rp*math.cos(a); rt = rp + m; rr = max(rp - 1.25*m, 0.5)   # pitch, base, tip, root
    inv = lambda ang: math.tan(ang) - ang
    half_p = math.pi/(2*N) - (_bl/2.0)/rp               # half tooth-thickness ANGLE at the pitch circle
    half_b = half_p + inv(a)                            # ... extrapolated to the base circle
    pa = 2*math.pi/N
    P = lambda r,ang: (r*math.cos(ang), r*math.sin(ang))
    r_relief = rp + (rt - rp)*0.34                      # relieve the outer ~1/3 of the addendum (tip side)
    def flank(sign):                                    # one involute flank, root→tip (sign = ±1)
        pts = []
        if rr < rb: pts.append((rr, sign*half_b))       # radial stub below the base circle (low-N undercut)
        for r in np.linspace(max(rr, rb), rt, 16):
            ar = math.acos(min(1.0, rb/r))              # 0 at the base circle, = pressure_angle at the pitch
            ang = half_b - inv(ar)                      # → ±half_p at the pitch circle
            if _tr > 0 and r > r_relief:                # TIP RELIEF: back the flank off toward the tip so it
                ang -= (_tr/rp) * (r - r_relief)/(rt - r_relief)   # can't gouge under a slight axis tilt
            pts.append((r, sign*ang))
        return [P(r, ang) for (r, ang) in pts]
    tooth = flank(-1) + list(reversed(flank(+1)))       # CCW: −flank up, across the tip, +flank down
    c0 = math.radians(center_at)
    half = math.radians(sector)/2.0 if sector is not None else math.pi
    teeth = []; _dv = []
    for k in range(N):
        base = k*pa
        d = (base - c0 + math.pi) % (2*math.pi) - math.pi      # signed angle to sector centre
        if sector is not None and abs(d) > half: continue
        cs, sn = math.cos(base), math.sin(base)
        teeth.append(sg.Polygon([(x*cs - y*sn, x*sn + y*cs) for (x, y) in tooth])); _dv.append(d)
    if sector is not None and entry_drop > 0 and teeth:
        _i = int(np.argmin(_dv)) if entry_end < 0 else int(np.argmax(_dv))
        teeth[_i] = teeth[_i].intersection(sg.Point(0, 0).buffer(rt - entry_drop, 128))   # relieved entry tooth
    if sector is None:
        base = sg.Point(0,0).buffer(rr + 0.01, resolution=128)
    else:
        base = sg.Point(0,0).buffer(rr - relief, resolution=128)            # recessed disc off-sector
        aa = np.linspace(c0-half-pa, c0+half+pa, 64)
        base = unary_union([base, sg.Polygon([(0,0)]+[P(rr+0.01, ang) for ang in aa])])  # raised rim under the sector
    g = unary_union([base]+teeth).buffer(0)
    return g.buffer(0.02).buffer(-0.02)        # clean tooth-foot vertices → watertight extrude

def mesh_phase(N_drv, N_drn, phi_deg):
    """Rotation (degrees, about its own axis) to PHASE a driven gear (N_drn teeth) so a tooth sits in the
    driver's (N_drv teeth, unrotated) gap at the line of centres. phi_deg = direction from driver→driven.
    Both gears generated with a tooth centred at angle 0."""
    pad, pan = 360.0/N_drv, 360.0/N_drn
    fA  = (phi_deg / pad) % 1.0                 # driver tooth-phase at the contact (0=tooth, .5=gap)
    fB0 = ((phi_deg + 180.0) / pan) % 1.0       # driven tooth-phase at the point facing the driver
    fB  = (fA + 0.5) % 1.0                       # want the complement there (tooth↔gap)
    return ((fB - fB0 + 0.5) % 1.0 - 0.5) * pan  # smallest signed rotation


def spur_gear_mesh(N, m, width, bore_d=0.0, pa=20.0, bl=0.35,
                   t_relief=0.10, sections=96):
    """Extrude an involute spur gear and optionally cut a round axial bore.

    This unifies the finnish-windows polygon-extrusion helper and parviz's
    printable involute spur use case. ``sections`` controls bore resolution.
    origin: finnish-windows tools/gearbox.py:96
    origin: parviz src/gears.py:51
    """
    import trimesh

    from .meshutil import sub
    from .prim import cyl

    if width <= 0 or bore_d < 0:
        raise ValueError("spur_gear_mesh(): width must be positive and bore_d non-negative")
    poly = spur_gear_2d(N=N, m=m, pa=pa, bl=bl, t_relief=t_relief)
    gear = trimesh.creation.extrude_polygon(poly, width)
    if bore_d > 0:
        bore = cyl(bore_d / 2.0, width + 2.0, sections=sections)
        bore.apply_translation((0, 0, width / 2.0))
        gear = sub(gear, bore)
    return gear


def roller_sprocket_2d(n_teeth, pitch, pin_d, clear=0.0, outer_d=None):
    """Build a conjugate swept-envelope roller or track-pin sprocket profile.

    The pitch radius is ``pitch / (2 sin(pi/n))``. A circular pin envelope is
    swept through rack motion in half-degree steps and copied around the blank.
    ``clear`` is radial pin running clearance. If omitted, ``outer_d`` uses a
    0.12-pitch addendum, matching the proportions of parviz's 14-tooth profile.
    origin: parviz src/tracks.py:85
    origin: parviz src/tracks.py:335
    """
    import shapely.affinity as sa
    import shapely.geometry as sg
    from shapely.ops import unary_union

    if n_teeth < 3 or pitch <= 0 or pin_d <= 0 or clear < 0:
        raise ValueError("roller_sprocket_2d(): invalid tooth, pitch, pin, or clearance value")
    rp = pitch / (2.0 * math.sin(math.pi / n_teeth))
    env_r = pin_d / 2.0 + clear
    if outer_d is None:
        outer_d = 2.0 * (rp + 0.12 * pitch)
    if outer_d <= 2.0 * (rp - env_r):
        raise ValueError("roller_sprocket_2d(): outer_d is too small for the pin envelope")
    blank = sg.Point(0, 0).buffer(outer_d / 2.0, resolution=96)
    swept = []
    for th in np.arange(-40.0, 40.01, 0.5) * (np.pi / 180.0):
        c, s = np.cos(th), np.sin(th)
        u, v = rp, rp * th
        swept.append(sg.Point(c * u + s * v, -s * u + c * v)
                     .buffer(env_r, resolution=24))
    gap = unary_union(swept)
    gaps = unary_union([sa.rotate(gap, 360.0 * k / n_teeth, origin=(0, 0))
                        for k in range(n_teeth)])
    return blank.difference(gaps).simplify(0.01).buffer(0)


def _involute_points(rb, r_start, r_end, n=14):
    """Return points on an involute of base circle ``rb``."""
    pts = []
    rs = max(r_start, rb + 1e-6)
    for r in np.linspace(rs, r_end, n):
        a = np.sqrt((r / rb) ** 2 - 1.0)
        inv = a - np.arctan(a)
        pts.append((r * np.cos(inv), r * np.sin(inv)))
    return np.array(pts)


def spur_gear(module, teeth, width, bore=0.0, pressure_angle=20.0,
              sector_deg=360.0, backlash=0.06, hub_d=0.0, full_disc=True,
              helix_deg=0.0):
    """Build a full-featured external involute gear.

    ``sector_deg`` can select a toothed arc, while ``hub_d`` and ``helix_deg``
    add a hub and helical tooth twist respectively.
    origin: dual-axis-turntable src/gears.py:26
    """
    import trimesh
    from shapely.geometry import Polygon
    from shapely.ops import unary_union

    m = module
    z = teeth
    a = np.radians(pressure_angle)
    r_pitch = m * z / 2.0
    r_base = r_pitch * np.cos(a)
    r_add = r_pitch + m
    r_ded = r_pitch - 1.25 * m
    r_root = max(r_ded, 0.5)

    t_pitch = np.pi * m / 2.0 - backlash
    inv_a = np.tan(a) - a
    half_ang_pitch = t_pitch / (2.0 * r_pitch)
    beta = half_ang_pitch + inv_a

    flank = _involute_points(r_base, r_root, r_add, n=16)

    def rot(pts, ang):
        c, s = np.cos(ang), np.sin(ang)
        return np.column_stack([pts[:, 0] * c - pts[:, 1] * s,
                                pts[:, 0] * s + pts[:, 1] * c])

    right = rot(flank, -beta)
    left = rot(flank * [1, -1], beta)
    root_r = np.array([r_root * np.cos(-beta), r_root * np.sin(-beta)])
    root_l = np.array([r_root * np.cos(beta), r_root * np.sin(beta)])
    th_r = np.arctan2(right[-1, 1], right[-1, 0])
    th_l = np.arctan2(left[-1, 1], left[-1, 0])
    tip_pts = np.array([(r_add * np.cos(t), r_add * np.sin(t))
                        for t in np.linspace(th_r, th_l, 4)])
    tooth = np.vstack([root_r, right, tip_pts, left[::-1], root_l])

    ring = []
    for k in range(z):
        ang = 2 * np.pi * k / z
        c, s = np.cos(ang), np.sin(ang)
        tp = np.column_stack([tooth[:, 0] * c - tooth[:, 1] * s,
                              tooth[:, 0] * s + tooth[:, 1] * c])
        ring.append(tp)
    pts = np.vstack(ring)
    keep = [pts[0]]
    for p in pts[1:]:
        if np.linalg.norm(p - keep[-1]) > 1e-6:
            keep.append(p)
    prof = Polygon(keep).buffer(0)

    if sector_deg < 360:
        half = np.radians(sector_deg) / 2.0
        big = r_add * 1.3
        wedge = Polygon([(0, 0)] +
                        [(big * np.cos(t), big * np.sin(t))
                         for t in np.linspace(-half, half, 60)] + [(0, 0)])
        prof = prof.intersection(wedge).buffer(0)
        back_r = r_root if full_disc else max(bore / 2 + 5.0, 9.0)
        disc = Polygon([(back_r * np.cos(t), back_r * np.sin(t))
                        for t in np.linspace(0, 2 * np.pi, 180, endpoint=False)])
        prof = unary_union([prof, disc]).buffer(0)
    if not isinstance(prof, Polygon):
        prof = max(prof.geoms, key=lambda g: g.area)

    mesh = trimesh.creation.extrude_polygon(prof, height=width)
    mesh.apply_translation([0, 0, -width / 2.0])

    if abs(helix_deg) > 1e-6:
        twist = width * np.tan(np.radians(helix_deg)) / r_pitch
        vertices = mesh.vertices
        ang = twist * (vertices[:, 2] / width)
        c, s = np.cos(ang), np.sin(ang)
        mesh.vertices = np.column_stack([
            vertices[:, 0] * c - vertices[:, 1] * s,
            vertices[:, 0] * s + vertices[:, 1] * c,
            vertices[:, 2],
        ])

    if hub_d > 0:
        hub = trimesh.creation.cylinder(radius=hub_d / 2, height=width, sections=64)
        mesh = trimesh.boolean.union([mesh, hub], engine="manifold")
    if bore > 0:
        cutter = trimesh.creation.cylinder(radius=bore / 2, height=width * 3, sections=48)
        mesh = trimesh.boolean.difference([mesh, cutter], engine="manifold")
    return mesh


def worm(module, length, pitch_d, starts=1, pressure_angle=20.0, bore=0.0):
    """Build a helical worm along positive Z and return it with its lead angle.

    The lead is ``starts * pi * module``. ``pressure_angle`` is retained as an
    explicit conjugacy parameter from the source implementation.
    origin: dual-axis-turntable src/gears.py:127
    """
    import trimesh

    m = module
    lead = starts * np.pi * m
    r_pitch = pitch_d / 2.0
    r_out = r_pitch + m
    r_root = r_pitch - 1.25 * m
    lead_angle = np.degrees(np.arctan(lead / (np.pi * pitch_d)))

    core = trimesh.creation.cylinder(radius=r_root, height=length, sections=64)
    w_root = lead * 0.55
    w_crest = lead * 0.22
    prof = np.array([(r_root, -w_root / 2), (r_out, -w_crest / 2),
                     (r_out, +w_crest / 2), (r_root, +w_root / 2)])
    turns = length / lead
    nseg = max(60, int(turns * 60))
    phis = np.linspace(0, 2 * np.pi * turns, nseg + 1)
    z0 = -length / 2
    rings = []
    for ph in phis:
        zc = z0 + lead * ph / (2 * np.pi)
        c, s = np.cos(ph), np.sin(ph)
        ring = np.array([[rr * c, rr * s, zc + zz] for rr, zz in prof])
        rings.append(ring)
    rings = np.array(rings)
    vertices = rings.reshape(-1, 3)
    faces = []
    np_ = 4
    for i in range(nseg):
        b0, b1 = i * np_, (i + 1) * np_
        for j in range(np_):
            a, b = j, (j + 1) % np_
            faces.append([b0 + a, b0 + b, b1 + b])
            faces.append([b0 + a, b1 + b, b1 + a])
    s0 = 0
    faces.append([s0 + 0, s0 + 1, s0 + 2])
    faces.append([s0 + 0, s0 + 2, s0 + 3])
    e0 = nseg * np_
    faces.append([e0 + 2, e0 + 1, e0 + 0])
    faces.append([e0 + 3, e0 + 2, e0 + 0])
    thread = trimesh.Trimesh(vertices=vertices, faces=np.array(faces), process=True)
    thread.fix_normals()
    worm_mesh = trimesh.boolean.union([core, thread], engine="manifold")
    if bore > 0:
        cutter = trimesh.creation.cylinder(radius=bore / 2, height=length * 3, sections=32)
        worm_mesh = trimesh.boolean.difference([worm_mesh, cutter], engine="manifold")
    worm_mesh.metadata["lead_angle"] = lead_angle
    return worm_mesh, lead_angle
