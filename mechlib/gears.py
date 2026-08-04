"""Pure two-dimensional gear generators."""
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
