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


def rack_2d(n_teeth, module, pa_deg=20.0, backlash=0.0,
            addendum=None, dedendum=None, base_height=None,
            end_margin=0.0, centered=True):
    """Build a finite trapezoidal-tooth rack profile as a Shapely polygon.

    Tooth centers are spaced at the circular pitch ``pi * module`` along X,
    with the pitch line at Y zero and teeth pointing toward positive Y. The
    pitch-line tooth thickness is half a pitch minus ``backlash``. It meshes
    with ``spur_gear_2d`` using the same module and pressure angle. ``addendum``
    and ``dedendum`` default to ``module`` and ``1.25 * module``;
    ``base_height`` defaults to ``2 * module`` below the tooth roots.

    origin: finnish-doors src/intercom/fixture.py ``_rack_tooth_gaps``
    """
    import shapely.geometry as sg
    from shapely.ops import unary_union

    if int(n_teeth) != n_teeth or n_teeth < 1 or module <= 0:
        raise ValueError("rack_2d(): n_teeth must be positive and module must be positive")
    if backlash < 0 or end_margin < 0 or not 0 < pa_deg < 45:
        raise ValueError("rack_2d(): invalid pressure angle, backlash, or end margin")
    addendum = module if addendum is None else addendum
    dedendum = 1.25 * module if dedendum is None else dedendum
    base_height = 2.0 * module if base_height is None else base_height
    if addendum <= 0 or dedendum <= 0 or base_height <= 0:
        raise ValueError("rack_2d(): addendum, dedendum, and base_height must be positive")

    pitch = math.pi * module
    pitch_thickness = pitch / 2.0 - backlash
    slope = math.tan(math.radians(pa_deg))
    tip_width = pitch_thickness - 2.0 * addendum * slope
    root_width = pitch_thickness + 2.0 * dedendum * slope
    if tip_width <= 0 or root_width >= pitch:
        raise ValueError("rack_2d(): tooth proportions overlap or collapse")

    length = n_teeth * pitch + 2.0 * end_margin
    x0 = -length / 2.0 if centered else 0.0
    base = sg.box(x0, -dedendum - base_height,
                  x0 + length, -dedendum + 0.01)
    first_center = x0 + end_margin + pitch / 2.0
    teeth = []
    for index in range(n_teeth):
        center = first_center + index * pitch
        teeth.append(sg.Polygon([
            (center - root_width / 2.0, -dedendum),
            (center + root_width / 2.0, -dedendum),
            (center + tip_width / 2.0, addendum),
            (center - tip_width / 2.0, addendum),
        ]))
    return unary_union([base] + teeth).buffer(0)


def _involute_points(rb, r_start, r_end, n=14):
    """Return points on an involute of base circle ``rb``."""
    pts = []
    rs = max(r_start, rb + 1e-6)
    for r in np.linspace(rs, r_end, n):
        a = np.sqrt((r / rb) ** 2 - 1.0)
        inv = a - np.arctan(a)
        pts.append((r * np.cos(inv), r * np.sin(inv)))
    return np.array(pts)


def _drop_degenerate_vertices(poly, tol=1e-7):
    """Strip consecutive coincident ring vertices from a shapely polygon.

    A boolean between a sampled circle and a radial cut edge emits an
    intersection vertex that can land exactly on an existing sample point (it
    happens whenever the sector half-angle is a multiple of the disc sampling
    step). The polygon stays ``is_valid`` and keeps its area, but the
    zero-length edge triangulates into a non-manifold wall, so
    ``extrude_polygon`` returns a mesh that is not a volume and the following
    boolean dies with "Not all meshes are volumes!". Dropping the duplicate
    vertices is exact, it removes no area, and it is deterministic.
    """
    from shapely.geometry import Polygon

    def clean(coords):
        pts = [tuple(p) for p in coords]
        if len(pts) > 1 and pts[0] == pts[-1]:
            pts = pts[:-1]
        out = []
        for p in pts:
            if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > tol:
                out.append(p)
        while len(out) > 1 and math.hypot(out[0][0] - out[-1][0],
                                          out[0][1] - out[-1][1]) <= tol:
            out.pop()
        return out

    shell = clean(poly.exterior.coords)
    if len(shell) < 3:
        raise ValueError("_drop_degenerate_vertices(): profile collapsed")
    holes = [h for h in (clean(i.coords) for i in poly.interiors) if len(h) >= 3]
    cleaned = Polygon(shell, holes)
    if not cleaned.is_valid:
        cleaned = cleaned.buffer(0)
    if cleaned.geom_type != "Polygon":
        cleaned = max(cleaned.geoms, key=lambda g: g.area)
    return cleaned


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
    prof = _drop_degenerate_vertices(prof)

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


def herringbone_gear(m=1.5, z=24, h=10.0, helix_deg=25.0, bore_d=0.0,
                     pa=20.0, bl=0.35, t_relief=0.10, hand=1):
    """Double-helical (herringbone) involute gear, fused watertight.

    Two mirrored opposite-twist halves of the ``spur_gear_2d`` profile are
    extruded to ``h``/2 each and unioned: tooth rotation is zero at the
    mid-plane and grows to ±``(h/2)·tan(helix_deg)/pitch_r`` at the faces, so
    the teeth form a chevron whose axial thrusts cancel. ``m`` module (mm),
    ``z`` teeth, ``h`` face width (mm), ``helix_deg`` helix angle (degrees),
    ``bore_d`` axial bore diameter (mm, 0 = solid), ``pa`` pressure angle
    (degrees), ``bl`` backlash tooth thinning (mm), ``t_relief`` tip relief
    (mm). ``hand`` selects the chevron direction: a meshing pair on parallel
    shafts needs one ``hand=+1`` and one ``hand=-1`` gear (mirrored chevrons,
    like any parallel helical pair). Because the mid-plane — the meshing
    plane — is untwisted in both hands, the pair is phased with
    ``mesh_phase`` exactly like spur gears. Prints vertically, no supports.
    """
    import trimesh

    from .meshutil import sub, uni
    from .prim import cyl

    if (m <= 0 or z < 6 or h <= 0 or not 0.0 < helix_deg < 45.0 or
            bore_d < 0 or not 0.0 < pa < 45.0 or bl < 0 or t_relief < 0 or
            hand not in (-1, 1)):
        raise ValueError("herringbone_gear(): invalid gear dimensions")
    profile = spur_gear_2d(z, m, pa=pa, bl=bl, t_relief=t_relief)
    rp = m * z / 2.0
    half = h / 2.0
    sweep = half * math.tan(math.radians(helix_deg)) / rp    # face rotation, rad
    parts = []
    for z0, sign in ((0.0, -1.0), (half, 1.0)):
        part = trimesh.creation.extrude_polygon(profile, half)
        part.apply_translation((0.0, 0.0, z0))
        v = part.vertices
        ang = hand * sign * sweep * (v[:, 2] - (z0 + half)) / half
        c, s = np.cos(ang), np.sin(ang)
        part.vertices = np.column_stack([
            v[:, 0] * c - v[:, 1] * s,
            v[:, 0] * s + v[:, 1] * c,
            v[:, 2],
        ])
        parts.append(part)
    gear = uni(parts)
    if bore_d > 0:
        gear = sub(gear, cyl(bore_d / 2.0, h + 2.0, (0.0, 0.0, half)))
    gear.metadata.update({
        "module": m, "teeth": z, "helix_deg": helix_deg, "pitch_r": rp,
        "hand": hand,
    })
    return gear


def trochoid_profile_2d(pins, pin_circle_r, pin_d, ecc, clearance=0.0,
                        samples=None):
    """Shortened-epitrochoid cycloidal disc profile (closed form, shapely).

    In the disc frame each roller centre traces the shortened epitrochoid
    ``E(u) = (R·cos u − e·cos(N·u), R·sin u − e·sin(N·u))``; the disc profile
    is its inner equidistant, offset by the roller radius along the curve
    normal. ``pins`` stationary rollers of diameter ``pin_d`` sit on
    ``pin_circle_r`` with eccentricity ``ecc`` (all mm); the disc carries
    ``pins - 1`` lobes and every roller stays tangent to it. ``clearance``
    shrinks the disc radially for running clearance against the rollers.
    """
    import shapely.geometry as sg

    n = pins
    R = pin_circle_r
    rr = pin_d / 2.0
    if samples is None:
        samples = max(240, 44 * n)
    u = np.linspace(0.0, 2.0 * math.pi, int(samples), endpoint=False)
    ex = R * np.cos(u) - ecc * np.cos(n * u)
    ey = R * np.sin(u) - ecc * np.sin(n * u)
    dx = -R * np.sin(u) + ecc * n * np.sin(n * u)
    dy = R * np.cos(u) - ecc * n * np.cos(n * u)
    norm = np.hypot(dx, dy)
    pts = np.stack([ex - rr * dy / norm, ey + rr * dx / norm], axis=1)
    poly = sg.Polygon(pts).buffer(0)
    if poly.geom_type != "Polygon":
        poly = max(poly.geoms, key=lambda g: g.area)
    if clearance > 0:
        poly = poly.buffer(-clearance / 2.0).buffer(0)
    return poly



# Kept as the historical private name: cycloidal_drive used this before the
# gerotor generator in mechlib.fluid became its second caller and the curve
# earned a public name. Same function, no behaviour change.
_cycloidal_disc_2d = trochoid_profile_2d

def cycloidal_drive(pins=12, pin_d=4.0, pin_circle_r=22.0, ecc=1.5,
                    disc_thickness=6.0, out_pins=6, out_pin_d=4.0,
                    out_circle_r=10.0, cam_d=8.0, shaft_d=5.0,
                    housing_t=4.0, rim_w=4.0, clearance=0.25,
                    shaft_tail=6.0, samples=None):
    """Single-stage cycloidal speed reducer as four printable parts.

    Returns ``{"housing", "disc", "input", "output"}`` posed in assembled
    coordinates (common axis +Z, disc orbiting at (+``ecc``, 0)). The housing
    is a base plate carrying ``pins`` roller pins on ``pin_circle_r``; the disc
    is the closed-form shortened-epitrochoid with ``pins - 1`` lobes and
    reduction ratio ``pins - 1``:1; the input is a shaft with an eccentric cam
    of ``cam_d`` offset by ``ecc``; the output plate carries ``out_pins`` pins
    of ``out_pin_d`` on ``out_circle_r`` engaging disc holes of
    ``hole_d = out_pin_d + 2*ecc + clearance`` (the classic oversize that
    absorbs the disc's orbit, plus print clearance). All dimensions mm.
    ``samples`` controls disc-curve resolution (modest by default).
    """
    import trimesh

    from .meshutil import sub, uni
    from .patterns import polar_ring
    from .prim import cyl

    if (pins < 4 or pin_d <= 0 or pin_circle_r <= 0 or ecc <= 0 or
            disc_thickness <= 0 or out_pins < 3 or out_pin_d <= 0 or
            out_circle_r <= 0 or cam_d <= 0 or shaft_d <= 0 or
            housing_t <= 0 or rim_w <= 0 or clearance < 0 or shaft_tail < 0):
        raise ValueError("cycloidal_drive(): invalid drive dimensions")
    if ecc * pins >= pin_circle_r:
        raise ValueError("cycloidal_drive(): ecc*pins must stay below pin_circle_r")
    if pin_d / 2.0 + ecc >= pin_circle_r * math.sin(math.pi / pins):
        raise ValueError("cycloidal_drive(): rollers undercut the disc curve")
    hole_d = out_pin_d + 2.0 * ecc + clearance
    if cam_d / 2.0 + ecc + clearance >= out_circle_r - hole_d / 2.0:
        raise ValueError("cycloidal_drive(): cam collides with output holes")

    disc2d = _cycloidal_disc_2d(pins, pin_circle_r, pin_d, ecc,
                                clearance=clearance, samples=samples)
    import shapely.geometry as sg
    for hx, hy in polar_ring(out_pins, out_circle_r):
        if not disc2d.covers(sg.Point(hx, hy).buffer(hole_d / 2.0 + 0.8)):
            raise ValueError("cycloidal_drive(): output holes break the disc edge")
    if not disc2d.covers(sg.Point(0.0, 0.0).buffer((cam_d + clearance) / 2.0 + 0.8)):
        raise ValueError("cycloidal_drive(): cam bore breaks the disc edge")

    disc = trimesh.creation.extrude_polygon(disc2d, disc_thickness)
    cutters = [cyl((cam_d + clearance) / 2.0, disc_thickness + 2.0,
                   (0.0, 0.0, disc_thickness / 2.0))]
    for hx, hy in polar_ring(out_pins, out_circle_r):
        cutters.append(cyl(hole_d / 2.0, disc_thickness + 2.0,
                           (hx, hy, disc_thickness / 2.0), sections=48))
    disc = sub(disc, uni(cutters))
    disc_z0 = housing_t + clearance
    disc.apply_translation((ecc, 0.0, disc_z0))

    plate_r = pin_circle_r + pin_d / 2.0 + rim_w
    plate = cyl(plate_r, housing_t, (0.0, 0.0, housing_t / 2.0))
    rollers = [cyl(pin_d / 2.0, disc_thickness + clearance,
                   (px, py, housing_t + (disc_thickness + clearance) / 2.0))
               for px, py in polar_ring(pins, pin_circle_r)]
    housing = uni([plate] + rollers)
    housing = sub(housing, cyl((shaft_d + clearance) / 2.0, housing_t + 2.0,
                               (0.0, 0.0, housing_t / 2.0)))

    shaft_top = disc_z0 + disc_thickness + clearance
    shaft = cyl(shaft_d / 2.0, shaft_top + shaft_tail,
                (0.0, 0.0, (shaft_top - shaft_tail) / 2.0))
    cam = cyl(cam_d / 2.0, disc_thickness,
              (ecc, 0.0, disc_z0 + disc_thickness / 2.0))
    input_shaft = uni([shaft, cam])

    out_z0 = disc_z0 + disc_thickness + clearance
    out_t = max(2.0, housing_t / 2.0)
    out_plate_r = out_circle_r + out_pin_d / 2.0 + rim_w / 2.0
    out_plate = cyl(out_plate_r, out_t, (0.0, 0.0, out_z0 + out_t / 2.0))
    out_pin_bottom = disc_z0 + 0.5
    out_rod = [cyl(out_pin_d / 2.0, out_z0 - out_pin_bottom,
                   (px, py, (out_z0 + out_pin_bottom) / 2.0))
               for px, py in polar_ring(out_pins, out_circle_r)]
    output = uni([out_plate] + out_rod)

    metadata = {"ratio": float(pins - 1), "lobes": pins - 1, "ecc": ecc,
                "hole_d": hole_d}
    for mesh in (housing, disc, input_shaft, output):
        mesh.metadata.update(metadata)
    return {"housing": housing, "disc": disc,
            "input": input_shaft, "output": output}


def bevel_gear_pair(m=1.5, z1=16, z2=24, face_w=None, pa=20.0, bl=0.35,
                    bore1_d=0.0, bore2_d=0.0, layers=10):
    """Straight bevel gear pair on 90-degree intersecting axes.

    Tredgold (back-cone equivalent) approximation: the ``spur_gear_2d``
    involute profile at the outer cone is lofted toward the pitch apex,
    scaling linearly with cone distance, so teeth taper straight at the apex.
    ``m`` outer module (mm), ``z1``/``z2`` tooth counts (pinion/gear),
    ``face_w`` face width along the cone (mm, default a quarter of the cone
    distance), ``pa`` pressure angle (degrees), ``bl`` backlash (mm),
    ``bore1_d``/``bore2_d`` axial bores (mm), ``layers`` loft resolution.
    Returns ``{"pinion", "gear"}`` posed meshed: pinion axis +Z, gear axis
    +Y, both pitch apexes coincident at the origin; the gear is spun with
    ``mesh_phase`` at the contact generator. Pitch cone half-angles follow
    tan(delta1) = z1/z2, delta1 + delta2 = 90 deg.
    """
    import trimesh
    import trimesh.transformations as tf

    from .meshutil import sub
    from .prim import cyl
    from .sweep import loft

    if (m <= 0 or z1 < 6 or z2 < 6 or not 0.0 < pa < 45.0 or bl < 0 or
            bore1_d < 0 or bore2_d < 0 or layers < 2):
        raise ValueError("bevel_gear_pair(): invalid gear dimensions")
    delta1 = math.atan2(z1, z2)
    delta2 = math.pi / 2.0 - delta1
    r1 = m * z1 / 2.0
    r2 = m * z2 / 2.0
    cone = math.hypot(r1, r2)                       # pitch cone distance, mm
    if face_w is None:
        face_w = cone / 4.0
    if not 0.0 < face_w < cone / 2.0:
        raise ValueError("bevel_gear_pair(): face_w must be within (0, cone/2)")

    def one_gear(z, delta, bore_d):
        profile = spur_gear_2d(z, m, pa=pa, bl=bl)
        coords = list(profile.exterior.coords)[:-1]
        rings = []
        for k in range(layers + 1):
            rho = cone - face_w + face_w * k / float(layers)
            s = rho / cone
            zz = rho * math.cos(delta)
            rings.append(np.array([[x * s, y * s, zz] for x, y in coords]))
        gear = loft(rings)
        if bore_d > 0:
            z_top = cone * math.cos(delta)
            gear = sub(gear, cyl(bore_d / 2.0, z_top + 2.0,
                                 (0.0, 0.0, z_top / 2.0)))
        return gear

    pinion = one_gear(z1, delta1, bore1_d)
    gear = one_gear(z2, delta2, bore2_d)
    gear.apply_transform(tf.rotation_matrix(
        math.radians(mesh_phase(z1, z2, 90.0)), (0.0, 0.0, 1.0)))
    gear.apply_transform(tf.rotation_matrix(-math.pi / 2.0, (1.0, 0.0, 0.0)))

    metadata = {"module": m, "ratio": z2 / float(z1),
                "delta1_deg": math.degrees(delta1),
                "delta2_deg": math.degrees(delta2),
                "cone_distance": cone}
    pinion.metadata.update(metadata)
    gear.metadata.update(metadata)
    return {"pinion": pinion, "gear": gear}


def _cutter_teeth_2d(z, m, pa_deg, bl, add, ded, n=14):
    """Return the tooth polygons of a shaper cutter (no hub disc).

    Same involute construction as ``spur_gear_2d`` -- same base circle, same
    pitch-line half-thickness angle -- but with an explicit ``add`` addendum
    and ``ded`` dedendum, and no tip relief. ``bl`` follows the
    ``spur_gear_2d`` sign convention (positive thins the tooth at the pitch
    circle), so a NEGATIVE ``bl`` thickens the cutter tooth and that is what
    opens backlash in the ring it generates. Only the teeth are returned:
    the cutter body never reaches the ring blank, so unioning a hub disc into
    the swept region would only slow the sweep down.
    """
    import shapely.geometry as sg

    a = math.radians(pa_deg)
    rp = m * z / 2.0
    rb = rp * math.cos(a)
    rr = max(rp - ded, 0.5)
    inv = lambda ang: math.tan(ang) - ang
    half_p = math.pi / (2 * z) - (bl / 2.0) / rp
    half_b = half_p + inv(a)
    if half_p <= 0:
        raise ValueError("_cutter_teeth_2d(): backlash swallows the tooth")

    def half_angle(r):
        return half_b - inv(math.acos(min(1.0, rb / r)))

    rt = rp + add
    if half_angle(rt) <= 1e-4:                  # pointed tooth: truncate the tip
        lo, hi = max(rr, rb) + 1e-9, rt
        for _ in range(48):
            mid = 0.5 * (lo + hi)
            if half_angle(mid) > 1e-4:
                lo = mid
            else:
                hi = mid
        rt = lo
    P = lambda r, ang: (r * math.cos(ang), r * math.sin(ang))

    def flank(sign):
        pts = []
        if rr < rb:
            pts.append((rr, sign * half_b))     # radial stub below the base circle
        for r in np.linspace(max(rr, rb), rt, n):
            pts.append((r, sign * half_angle(r)))
        return [P(r, ang) for r, ang in pts]

    tooth = flank(-1) + list(reversed(flank(+1)))
    out = []
    for k in range(z):
        base = 2.0 * math.pi * k / z
        c, s = math.cos(base), math.sin(base)
        out.append(sg.Polygon([(x * c - y * s, x * s + y * c)
                               for x, y in tooth]))
    return out, rt


def internal_mesh_phase(z_ring, z_pinion, phi_deg=0.0):
    """Rotation (degrees) that phases a pinion into an internal ring gear.

    ``mesh_phase`` does NOT apply to an internal mesh; it differs in two ways,
    and both matter. First, for two external gears the contact on the line of
    centres is seen at azimuth ``phi`` from the driver and at ``phi + 180``
    from the driven, which is where the 180 in ``mesh_phase`` comes from. In
    an internal mesh the pinion sits INSIDE the ring, so the contact lies
    beyond the pinion centre on the same ray and both bodies see it at azimuth
    ``phi``. Second, the sign flips: at an external contact the two bodies'
    CCW tangential directions oppose each other (the contact is on opposite
    sides of the two centres), so their tooth phases run backwards relative to
    one another; at an internal contact both centres are on the same side and
    the phases run together. The rule itself is unchanged -- a pinion tooth
    has to land in a ring tooth space. Substituting ``mesh_phase`` here is off
    by half a tooth whenever ``z_pinion`` is odd (with an even pinion the
    180 degree term happens to be a whole number of pitches, which is exactly
    the sort of coincidence that hides the error).

    ``z_ring`` and ``z_pinion`` are tooth counts, ``phi_deg`` is the azimuth of
    the pinion centre measured at the ring centre. Both members are assumed
    drawn with a tooth centred at their own angle zero, which is what
    ``internal_gear_2d`` and ``spur_gear_2d`` both do. Units are degrees.
    """
    if z_ring < 3 or z_pinion < 3:
        raise ValueError("internal_mesh_phase(): tooth counts must be >= 3")
    par, pap = 360.0 / z_ring, 360.0 / z_pinion
    fA = (phi_deg / par) % 1.0                  # ring tooth-phase at the contact
    fB0 = (phi_deg / pap) % 1.0                 # pinion tooth-phase at the contact
    fB = (fA + 0.5) % 1.0                       # want tooth against space
    return ((fB0 - fB + 0.5) % 1.0 - 0.5) * pap


def internal_gear_2d(z=40, m=1.5, pa_deg=20.0, rim=3.0, bl=0.35,
                     z_pinion=None, add=None, ded=None, steps=18,
                     simplify=0.005):
    """Build a true internal (annular) involute ring-gear profile (shapely).

    Returns one shapely ``Polygon``: a disc of outer radius
    ``m*z/2 + ded + rim`` with a single toothed hole, so it extrudes straight
    into a ring gear the same way ``spur_gear_2d``'s profile extrudes into a
    spur gear. Addendum and dedendum swap relative to an external gear: the
    teeth point INWARD, the tip circle sits at ``m*z/2 - add`` (INSIDE the
    pitch circle) and the roots at ``m*z/2 + ded`` (OUTSIDE it). Teeth are
    centred at ``k*(360/z)`` degrees, the same convention ``spur_gear_2d``
    uses, so ``internal_mesh_phase`` places a mating pinion.

    The flanks are generated, not drawn: a shaper cutter shaped like the
    mating pinion (``z_pinion`` teeth, cutting addendum ``ded`` so it also
    cuts the root clearance) is rolled through the conjugate internal motion
    and the swept region is subtracted from the blank, the same
    swept-envelope technique ``roller_sprocket_2d`` uses. Rolling one ring
    pitch and copying the result ``z`` times is exact: advancing the roll by
    one ring pitch is the same configuration rotated by one ring pitch, with
    the cutter one tooth on. ``steps`` samples per ring pitch controls flank
    fidelity; the default is deliberately modest because this runs in a
    WASM playground, and the residual scallop is already far below the
    backlash (raising 18 to 60 moves the profile area by 0.01%).

    ``bl`` follows ``spur_gear_2d``'s convention: it thins the ring tooth at
    the pitch circle, i.e. widens the tooth space by ``bl``, giving a flank
    clearance of ``(bl/2)*cos(pa)`` against a pinion drawn with zero
    backlash. ``z_pinion`` defaults to the tightest legal mate, ``z - 8``;
    generating against the largest allowed pinion means any smaller one also
    clears. ``add`` and ``ded`` default to ``m`` and ``1.25*m``.

    Units are mm and degrees.
    """
    import shapely.affinity as sa
    import shapely.geometry as sg
    from shapely.ops import unary_union

    from .meshutil import largest_poly

    if m <= 0 or not 0.0 < pa_deg < 45.0 or bl < 0 or rim <= 0 or steps < 4:
        raise ValueError("internal_gear_2d(): invalid module, pressure angle, "
                         "backlash, rim, or step count")
    if int(z) != z or z < 14:
        raise ValueError("internal_gear_2d(): z must be a whole number >= 14")
    z = int(z)
    z_pinion = z - 8 if z_pinion is None else int(z_pinion)
    if z_pinion < 6:
        raise ValueError("internal_gear_2d(): z_pinion must be >= 6")
    if z - z_pinion < 8:
        raise ValueError(
            "internal_gear_2d(): z - z_pinion is %d; an internal involute pair "
            "needs at least 8 teeth of difference or the pinion tips foul the "
            "ring tips" % (z - z_pinion))
    add = m if add is None else add
    ded = 1.25 * m if ded is None else ded
    if add <= 0 or ded <= add:
        raise ValueError("internal_gear_2d(): need 0 < add < ded "
                         "(the dedendum carries the tip clearance)")
    if rim < 0.8:
        raise ValueError("internal_gear_2d(): rim %.2f mm is thinner than one "
                         "printable wall; the tooth roots would break through "
                         "the back of the ring" % rim)

    rp = m * z / 2.0
    tip_r = rp - add
    root_r = rp + ded
    outer_r = root_r + rim
    rp_pinion = m * z_pinion / 2.0
    centre_dist = rp - rp_pinion

    # Cutter: the mating pinion with the cutting addendum that reaches the ring
    # root circle (a + r_tip = rp + ded), thickened by bl to open the backlash.
    teeth, _rt = _cutter_teeth_2d(z_pinion, m, pa_deg, -bl, ded, ded)
    cutter = unary_union(teeth)

    # Only the band the teeth live in can be cut, so clip the swept region to it
    # before making the z copies. The cutter body (its root circle) stays clear
    # of the ring tip circle by ded - add, which IS the tip clearance.
    band = sg.Point(0, 0).buffer(root_r + 0.5, resolution=128).difference(
        sg.Point(0, 0).buffer(max(tip_r - 0.5, 0.05), resolution=128))
    # Conjugate internal roll: the cutter centre orbits at psi and the cutter
    # spins at -(a/rp_pinion)*psi (pitch circles rolling inside one another).
    # The +180/z_pinion seed faces a cutter TOOTH SPACE at psi=0, which is what
    # leaves a ring tooth centred on angle zero.
    spin0 = math.pi / z_pinion
    swept = []
    for psi in np.linspace(-math.pi / z, math.pi / z, int(steps)):
        spin = spin0 - (centre_dist / rp_pinion) * psi
        placed = sa.rotate(cutter, math.degrees(spin), origin=(0, 0))
        swept.append(sa.translate(placed, centre_dist * math.cos(psi),
                                  centre_dist * math.sin(psi)))
    sweep = unary_union(swept).intersection(band)
    gaps = unary_union([sa.rotate(sweep, 360.0 * k / z, origin=(0, 0))
                        for k in range(z)])

    blank = sg.Point(0, 0).buffer(outer_r, resolution=192).difference(
        sg.Point(0, 0).buffer(tip_r, resolution=192))
    profile = blank.difference(gaps).buffer(0)
    if simplify > 0:
        profile = profile.simplify(simplify).buffer(0)
    profile = largest_poly(profile)
    if len(profile.interiors) != 1:
        raise ValueError("internal_gear_2d(): the tooth spaces broke the ring "
                         "into %d loops; increase rim" % len(profile.interiors))
    return profile


def ring_gear(z=40, m=1.5, width=8.0, pa_deg=20.0, rim=3.0, bl=0.35,
              z_pinion=None, add=None, ded=None, steps=18):
    """Extrude an internal involute ring gear as one printable solid.

    The teeth point inward from an annular rim, so the part is a ring whose
    bore is toothed: tip diameter ``m*z - 2*add`` (smaller than the pitch
    diameter), root diameter ``m*z + 2*ded`` (larger), outer diameter
    ``m*z + 2*(ded + rim)``. It sits on z=0 and runs to ``z = width``, axis
    +Z. Print it flat on the plate with the axis vertical: every tooth flank
    is then a vertical wall, no overhang and no supports, and the layer lines
    run around the teeth rather than across them.

    ``metadata`` carries ``pitch_d``, ``tip_d``, ``root_d``, ``outer_d``,
    ``z``, ``module``, and ``z_pinion`` (the pinion it was generated against).
    See ``internal_gear_2d`` for the profile arguments.
    Units are mm and degrees.
    """
    import trimesh

    if width <= 0:
        raise ValueError("ring_gear(): width must be positive")
    profile = internal_gear_2d(z=z, m=m, pa_deg=pa_deg, rim=rim, bl=bl,
                               z_pinion=z_pinion, add=add, ded=ded,
                               steps=steps)
    gear = trimesh.creation.extrude_polygon(profile, width)
    add = m if add is None else add
    ded = 1.25 * m if ded is None else ded
    gear.metadata.update({
        "pitch_d": m * z,
        "tip_d": m * z - 2.0 * add,
        "root_d": m * z + 2.0 * ded,
        "outer_d": m * z + 2.0 * (ded + rim),
        "z": int(z),
        "module": m,
        "z_pinion": (z - 8) if z_pinion is None else int(z_pinion),
        "width": width,
    })
    return gear


def ring_gear_mesh(z=40, z_pinion=20, m=1.5, width=8.0, pa_deg=20.0,
                   rim=3.0, bl=0.35, bore_d=5.0, phi_deg=0.0, steps=18):
    """Build an internal gear pair posed in mesh: pinion inside ring gear.

    Returns ``{"ring", "pinion"}``. The ring is centred on the origin with its
    axis +Z, the pinion centre sits at azimuth ``phi_deg`` and radius
    ``m*(z - z_pinion)/2`` (the internal centre distance, the DIFFERENCE of
    the pitch radii, not the sum), and the pinion is spun by
    ``internal_mesh_phase`` so its teeth drop into the ring's tooth spaces.
    Both parts span z=0 to ``width``. The ring is cut against this exact
    pinion, so the flanks are conjugate and the running clearance is
    ``bl*cos(pa)`` split between the two members.

    Internal gears turn the same way as their pinion (external pairs
    counter-rotate) and the ratio is ``z / z_pinion``. Print both flat on the
    plate, axes vertical, no supports; they are separate parts, not
    print-in-place. Units are mm and degrees.
    """
    import trimesh.transformations as tf

    if bore_d < 0:
        raise ValueError("ring_gear_mesh(): bore_d must be non-negative")
    ring = ring_gear(z=z, m=m, width=width, pa_deg=pa_deg, rim=rim, bl=bl,
                     z_pinion=z_pinion, steps=steps)
    pinion = spur_gear_mesh(z_pinion, m, width, bore_d=bore_d, pa=pa_deg,
                            bl=bl)
    centre_dist = m * (z - z_pinion) / 2.0
    pinion.apply_transform(tf.rotation_matrix(
        math.radians(internal_mesh_phase(z, z_pinion, phi_deg)),
        (0.0, 0.0, 1.0)))
    pinion.apply_translation((centre_dist * math.cos(math.radians(phi_deg)),
                              centre_dist * math.sin(math.radians(phi_deg)),
                              0.0))
    metadata = {"ratio": z / float(z_pinion), "centre_distance": centre_dist,
                "z": int(z), "z_pinion": int(z_pinion), "module": m}
    ring.metadata.update(metadata)
    pinion.metadata.update(metadata)
    return {"ring": ring, "pinion": pinion}
