"""Project-agnostic fluid-handling generators: pumps, barbs, and valves."""

import math

import numpy as np
import shapely.affinity as affinity
import shapely.geometry as sg
import trimesh
import trimesh.transformations as tf
from shapely.ops import unary_union

from .gears import trochoid_profile_2d
from .meshutil import extrude_poly_z, sub, uni
from .patterns import directed_holes, polar_ring
from .prim import cyl, frustum, hex_poly, sector2d
from .sweep import swept_keyed_bore


def _trochoid_curvature_r(lobe_circle_r, ecc, outer_lobes, samples=1440):
    """Return the smallest convex radius of curvature of the generating trochoid.

    The gerotor inner rotor is the inner equidistant of the shortened
    epitrochoid traced by the outer rotor's tooth centres. Offsetting a curve
    inward by more than its convex radius of curvature folds the offset back on
    itself, so this number is the hard upper bound on the outer tooth radius.
    """
    n = float(outer_lobes)
    u = np.linspace(0.0, 2.0 * math.pi, int(samples), endpoint=False)
    dx = -lobe_circle_r * np.sin(u) + ecc * n * np.sin(n * u)
    dy = lobe_circle_r * np.cos(u) - ecc * n * np.cos(n * u)
    ddx = -lobe_circle_r * np.cos(u) + ecc * n * n * np.cos(n * u)
    ddy = -lobe_circle_r * np.sin(u) + ecc * n * n * np.sin(n * u)
    cross = dx * ddy - dy * ddx
    speed = np.hypot(dx, dy)
    with np.errstate(divide="ignore", invalid="ignore"):
        rho = speed ** 3 / cross
    convex = rho[np.isfinite(rho) & (rho > 0.0)]
    return float(convex.min()) if convex.size else float("inf")


def _gerotor_contacts(lobe_circle_r, ecc, outer_lobes, tooth_r, phase_deg):
    """Return the outer-frame contact points of the rotor pair at one phase.

    One contact per outer tooth: the inner rotor's profile is by construction
    tangent to every tooth circle, and those ``outer_lobes`` tangency points are
    exactly the walls that separate the pumping chambers from each other.
    """
    n = outer_lobes
    inner_lobes = n - 1
    phase = math.radians(phase_deg)
    outer_phase = phase * inner_lobes / n
    points = []
    for k in range(n):
        u = (2.0 * math.pi * k - phase) / n
        ex = lobe_circle_r * math.cos(u) - ecc * math.cos(n * u)
        ey = lobe_circle_r * math.sin(u) - ecc * math.sin(n * u)
        dx = -lobe_circle_r * math.sin(u) + ecc * n * math.sin(n * u)
        dy = lobe_circle_r * math.cos(u) - ecc * n * math.cos(n * u)
        norm = math.hypot(dx, dy)
        px, py = ex - tooth_r * dy / norm, ey + tooth_r * dx / norm
        cos_i, sin_i = math.cos(phase), math.sin(phase)
        wx, wy = px * cos_i - py * sin_i + ecc, px * sin_i + py * cos_i
        cos_o, sin_o = math.cos(-outer_phase), math.sin(-outer_phase)
        points.append((wx * cos_o - wy * sin_o, wx * sin_o + wy * cos_o))
    return points


def _gerotor_pose_2d(inner_2d, inner_lobes, outer_lobes, ecc, phase_deg):
    """Place the inner rotor profile into the outer rotor's own frame."""
    posed = affinity.rotate(inner_2d, phase_deg, origin=(0.0, 0.0))
    posed = affinity.translate(posed, ecc, 0.0)
    return affinity.rotate(posed, -phase_deg * inner_lobes / outer_lobes,
                           origin=(0.0, 0.0))


def _gerotor_chambers(cavity_2d, inner_2d, inner_lobes, outer_lobes, ecc,
                      lobe_circle_r, tooth_r, phase_deg, knife=0.15):
    """Return the sorted chamber areas of the rotor pair at one phase.

    The chambers meet at the tangency points, where a Shapely difference of two
    touching profiles cannot be trusted to split. Nicking a small disc out of
    each contact separates them positively; the nicks cost well under
    0.05 mm^2 because the free region is a cusp exactly there.
    """
    posed = _gerotor_pose_2d(inner_2d, inner_lobes, outer_lobes, ecc, phase_deg)
    free = cavity_2d.difference(posed)
    nicks = unary_union([sg.Point(p).buffer(knife, resolution=6) for p in
                         _gerotor_contacts(lobe_circle_r, ecc, outer_lobes,
                                           tooth_r, phase_deg)])
    free = free.difference(nicks)
    geoms = free.geoms if free.geom_type == "MultiPolygon" else [free]
    areas = sorted((g.area for g in geoms), reverse=True)
    return areas[:outer_lobes]


def gerotor_pump(lobes=6, lobe_circle_r=16.0, ecc=1.6, tooth_d=None,
                 rotor_h=8.0, clear=0.3, rim_w=4.0, wall=3.0, base_t=4.0,
                 cap_t=4.0, shaft_d=6.0, shaft_flat=0.0, ports=True,
                 port_land_deg=12.0, port_seal=0.6, phase_deg=0.0,
                 profile_samples=None, phase_samples=12):
    """Build an internal-gear (gerotor) pump as four printable parts.

    Returns ``{"inner", "outer", "housing", "cap"}`` posed in assembled
    coordinates. The inner rotor carries ``lobes`` trochoidal teeth and turns
    about an axis at ``(ecc, 0)``; the outer rotor carries ``lobes + 1``
    circular teeth of radius ``tooth_d/2`` on ``lobe_circle_r`` and turns about
    the origin, so the pair runs at ``(lobes + 1) / lobes``. The sealed
    crescents between them expand from zero on the ``+X`` side round to full on
    the ``-X`` side and back, which is what carries fluid from the inlet kidney
    port to the outlet kidney port in the cap. The inner profile is the inner
    equidistant of the shortened epitrochoid (shared with
    ``gears.cycloidal_drive``); the outer cavity is the tooth circles cut from a
    root circle at ``lobe_circle_r + 2*ecc - tooth_d/2``, which is the exact
    radius the inner rotor's tips reach, so the trapped dead volume at the roots
    is about 2% of the cavity. ``tooth_d`` defaults to 70% of the smaller of the
    two hard limits (profile undercut and tooth merging).

    ``phase_deg`` turns the inner rotor and reposes the outer at the conjugate
    rate; both parts are rigid bodies at every phase. ``displacement_per_rev``
    in the metadata is ``lobes * (chamber_a_max - chamber_a_min) * rotor_h``,
    measured from the real swept chamber polygons rather than assumed.

    z=0 is the housing's bottom face and +Z is up. Print all four parts flat on
    that face: every wall is vertical, the only horizontal spans are the housing
    floor and the cap, and the kidney ports are through-cuts. A printed gerotor
    leaks past the rotor faces, so it suits low-head liquid transfer rather than
    pressure; a round ``shaft_flat=0`` bore cannot transmit torque on its own,
    so either set ``shaft_flat`` for a D-bore or bond the shaft. The cap carries
    no fastening features; clamp it with your own screws. Units are mm and
    degrees.
    """
    if lobes < 3:
        raise ValueError("gerotor_pump(): lobes must be at least 3")
    if lobe_circle_r <= 0 or ecc <= 0 or rotor_h <= 0 or clear < 0:
        raise ValueError("gerotor_pump(): lobe_circle_r, ecc and rotor_h must "
                         "be positive and clear non-negative")
    if rim_w <= 0 or wall <= 0 or base_t <= 0 or cap_t <= 0 or shaft_d <= 0:
        raise ValueError("gerotor_pump(): rim_w, wall, base_t, cap_t and "
                         "shaft_d must be positive")
    if not 0.0 < port_land_deg < 60.0:
        raise ValueError("gerotor_pump(): port_land_deg must be in (0, 60)")

    n = lobes + 1
    R = float(lobe_circle_r)
    e = float(ecc)
    merge_limit = R * math.sin(math.pi / n)
    undercut_limit = _trochoid_curvature_r(R, e, n)
    if tooth_d is None:
        tooth_r = 0.7 * min(merge_limit, undercut_limit)
    else:
        if tooth_d <= 0:
            raise ValueError("gerotor_pump(): tooth_d must be positive")
        tooth_r = float(tooth_d) / 2.0
    if tooth_r >= undercut_limit:
        raise ValueError(
            "gerotor_pump(): tooth_d %.2f undercuts the inner profile "
            "(limit %.2f); reduce tooth_d or ecc" % (2.0 * tooth_r,
                                                     2.0 * undercut_limit))
    if tooth_r >= merge_limit:
        raise ValueError(
            "gerotor_pump(): tooth_d %.2f merges adjacent outer teeth "
            "(limit %.2f); reduce tooth_d or raise lobe_circle_r"
            % (2.0 * tooth_r, 2.0 * merge_limit))
    if e >= tooth_r:
        raise ValueError(
            "gerotor_pump(): ecc %.2f must stay under the outer tooth radius "
            "%.2f or the teeth become islands in the cavity instead of "
            "protrusions; reduce ecc or raise lobe_circle_r" % (e, tooth_r))
    root_r = R + 2.0 * e - tooth_r
    tip_r = R - tooth_r
    if tip_r <= 0.0 or root_r - tip_r < 1.0:
        raise ValueError("gerotor_pump(): chamber band %.2f mm is too thin; "
                         "raise ecc" % (root_r - tip_r))
    inner_root_r = R - e - tooth_r
    if inner_root_r <= 0.0:
        raise ValueError("gerotor_pump(): inner rotor has no material; "
                         "reduce tooth_d")
    if shaft_d + clear >= 2.0 * (inner_root_r - 1.2):
        raise ValueError("gerotor_pump(): shaft_d %.2f leaves under 1.2 mm of "
                         "inner rotor wall" % shaft_d)

    # The profile and the circle tessellations are locked to multiples of the
    # lobe counts so both rotors are EXACTLY symmetric under one tooth pitch.
    # That is what lets the pair repose as rigid bodies over a short cycle.
    samples = (int(profile_samples) if profile_samples
               else max(6 * lobes * n, 180))
    samples -= samples % lobes
    inner_nom = trochoid_profile_2d(n, R, 2.0 * tooth_r, e, clearance=0.0,
                                   samples=samples)
    inner_run = trochoid_profile_2d(n, R, 2.0 * tooth_r, e, clearance=clear,
                                   samples=samples)

    def _cavity(grow):
        teeth = unary_union([sg.Point(px, py).buffer(tooth_r - grow,
                                                     resolution=4 * n)
                             for px, py in polar_ring(n, R)])
        return sg.Point(0.0, 0.0).buffer(root_r + grow,
                                         resolution=12 * n).difference(teeth)

    cavity_nom = _cavity(0.0)
    cavity_run = _cavity(clear / 2.0)
    if cavity_nom.geom_type != "Polygon" or cavity_nom.interiors:
        raise ValueError("gerotor_pump(): outer cavity is not a simple "
                         "pocket; check lobe_circle_r, ecc and tooth_d")

    # Chamber cycle. One phase already samples all n chambers at n evenly
    # spaced points of the same cycle, so phase_samples phases over one inner
    # tooth pitch give phase_samples*n evenly spaced samples of the full cycle.
    if phase_samples < 4:
        raise ValueError("gerotor_pump(): phase_samples must be at least 4")
    cycle = []
    for k in range(int(phase_samples)):
        cycle.extend(_gerotor_chambers(cavity_nom, inner_nom, lobes, n, e, R,
                                       tooth_r, 360.0 * k / (lobes * phase_samples)))
    a_max, a_min = max(cycle), min(cycle)
    displacement = lobes * (a_max - a_min) * rotor_h

    outer_r = root_r + rim_w
    bore_r = outer_r + clear
    housing_r = bore_r + wall
    z_rotor = base_t + clear
    z_top = z_rotor + rotor_h
    z_wall = z_top + clear

    inner = trimesh.creation.extrude_polygon(inner_run, rotor_h)
    bore = cyl((shaft_d + clear) / 2.0, rotor_h + 4.0, (0.0, 0.0, rotor_h / 2.0),
               sections=6 * lobes)
    if shaft_flat > 0.0:
        flat_r = (shaft_d + clear) / 2.0 - float(shaft_flat)
        if flat_r <= 0.4:
            raise ValueError("gerotor_pump(): shaft_flat removes the bore")
        keep = trimesh.creation.box(extents=(shaft_d + 8.0, shaft_d + 8.0,
                                             rotor_h + 6.0))
        keep.apply_translation((0.0, -(shaft_d + 8.0) / 2.0 + flat_r,
                                rotor_h / 2.0))
        bore = sub(bore, keep)
    inner = sub(inner, bore)
    inner.apply_transform(tf.rotation_matrix(math.radians(phase_deg), (0, 0, 1)))
    inner.apply_translation((e, 0.0, z_rotor))

    outer_2d = sg.Point(0.0, 0.0).buffer(outer_r, resolution=12 * n).difference(
        cavity_run)
    outer = trimesh.creation.extrude_polygon(outer_2d, rotor_h)
    outer.apply_transform(tf.rotation_matrix(
        math.radians(phase_deg * lobes / n), (0, 0, 1)))
    outer.apply_translation((0.0, 0.0, z_rotor))

    housing = uni([
        cyl(housing_r, base_t, (0.0, 0.0, base_t / 2.0), sections=96),
        sub(cyl(housing_r, z_wall - base_t, (0.0, 0.0, (z_wall + base_t) / 2.0),
                sections=96),
            cyl(bore_r, z_wall - base_t + 2.0,
                (0.0, 0.0, (z_wall + base_t) / 2.0), sections=96)),
    ])
    housing = sub(housing, cyl((shaft_d + clear) / 2.0, base_t + 4.0,
                               (e, 0.0, base_t / 2.0), sections=48))

    cap = cyl(housing_r, cap_t, (0.0, 0.0, z_wall + cap_t / 2.0), sections=96)
    port_r0 = tip_r + port_seal
    port_r1 = root_r - port_seal
    if ports:
        if port_r1 - port_r0 < 0.8:
            raise ValueError("gerotor_pump(): kidney ports would be %.2f mm "
                             "wide; reduce port_seal" % (port_r1 - port_r0))
        cuts = []
        for a0, a1 in ((port_land_deg, 180.0 - port_land_deg),
                       (180.0 + port_land_deg, 360.0 - port_land_deg)):
            kidney = sector2d(a0, a1, port_r1, n=64).difference(
                sg.Point(0.0, 0.0).buffer(port_r0, resolution=48))
            cuts.append(extrude_poly_z(kidney, z_wall - 1.0,
                                       z_wall + cap_t + 1.0))
        cap = sub(cap, uni(cuts))

    meta = {
        "lobes": lobes,
        "outer_lobes": n,
        "ratio": n / float(lobes),
        "ecc": e,
        "lobe_circle_r": R,
        "tooth_d": 2.0 * tooth_r,
        "tooth_d_undercut_limit": 2.0 * undercut_limit,
        "tooth_d_merge_limit": 2.0 * merge_limit,
        "cavity_root_r": root_r,
        "cavity_tip_r": tip_r,
        "rotor_h": rotor_h,
        "clear": clear,
        "chamber_a_max": a_max,
        "chamber_a_min": a_min,
        "displacement_per_rev": displacement,
        "port_r0": port_r0,
        "port_r1": port_r1,
        "outer_d": 2.0 * outer_r,
        "housing_d": 2.0 * housing_r,
        "phase_deg": float(phase_deg),
    }
    parts = {"inner": inner, "outer": outer, "housing": housing, "cap": cap}
    for mesh in parts.values():
        mesh.metadata.update(meta)
    return parts


def hose_barb(tube_id=6.0, barbs=3, interference=0.6, root_relief=0.8,
              ramp_deg=25.0, barb_gap=1.0, bore_d=None, boss_d=None,
              boss_h=4.0, foot="flange", foot_d=None, foot_t=3.0,
              thread_d=None, thread_pitch=None, lead_in=0.8, sections=64):
    """Build a stacked-frustum hose barb (tail) for soft tubing.

    ``barbs`` sawtooth rings sit on a stop boss above a mounting foot. Each ring
    starts at the root diameter ``tube_id - root_relief``, jumps straight out to
    the crest diameter ``tube_id + interference``, then tapers back down at
    ``ramp_deg`` from the axis: the taper lets the tube walk on, the square
    shoulder facing the foot stops it walking back off. ``interference`` is the
    designed grip, so the tube is stretched over every crest and a worm clip
    over the barb band is still worth fitting for anything above a trickle.
    ``foot`` is ``"flange"`` (a plain disc), ``"thread"`` (an external printed
    thread from ``mechanisms.thread_solid``) or ``"none"``.

    z=0 is the bottom of the foot and the barbs point up +Z. Print in that
    orientation: the tapers are ``ramp_deg`` from vertical and every other
    surface either shrinks with height or is flat, so the only unsupported
    feature is each retaining shoulder, a ring ledge
    ``(interference + root_relief)/2`` wide that FDM spans without support. The
    tubing itself is not printed. Units are mm and degrees.
    """
    if tube_id <= 0 or barbs < 1:
        raise ValueError("hose_barb(): tube_id must be positive and barbs >= 1")
    if interference <= 0 or root_relief < 0:
        raise ValueError("hose_barb(): interference must be positive and "
                         "root_relief non-negative")
    if not 5.0 <= ramp_deg <= 45.0:
        raise ValueError("hose_barb(): ramp_deg must be in [5, 45] to stay "
                         "printable and still grip")
    if barb_gap < 0 or boss_h < 0 or foot_t <= 0 or lead_in < 0:
        raise ValueError("hose_barb(): barb_gap, boss_h, foot_t and lead_in "
                         "must be non-negative (foot_t positive)")
    if foot not in ("flange", "thread", "none"):
        raise ValueError("hose_barb(): foot must be 'flange', 'thread' or "
                         "'none'")

    crest_r = (tube_id + interference) / 2.0
    root_r = (tube_id - root_relief) / 2.0
    if root_r <= 0.6:
        raise ValueError("hose_barb(): root_relief leaves no barb root")
    rise = crest_r - root_r
    ramp_len = rise / math.tan(math.radians(ramp_deg))
    pitch = ramp_len + barb_gap
    if bore_d is None:
        bore_d = 2.0 * root_r - 2.0
    if bore_d < 0.8:
        raise ValueError("hose_barb(): bore_d %.2f is under one nozzle pair; "
                         "raise tube_id or drop root_relief" % bore_d)
    if bore_d >= 2.0 * root_r - 1.6:
        raise ValueError("hose_barb(): bore_d %.2f leaves under 0.8 mm wall at "
                         "the barb root" % bore_d)
    if boss_d is None:
        boss_d = 2.0 * crest_r + 2.4
    if boss_d < 2.0 * crest_r:
        raise ValueError("hose_barb(): boss_d must be at least the crest "
                         "diameter %.2f" % (2.0 * crest_r))

    z_foot = foot_t if foot != "none" else 0.0
    z_barb = z_foot + boss_h
    solids = []
    if foot == "flange":
        if foot_d is None:
            foot_d = boss_d + 8.0
        if foot_d <= boss_d:
            raise ValueError("hose_barb(): foot_d must exceed boss_d")
        solids.append(cyl(foot_d / 2.0, foot_t, (0.0, 0.0, foot_t / 2.0),
                          sections=sections))
    elif foot == "thread":
        from .mechanisms import coarse_pitch, thread_solid
        if thread_d is None:
            thread_d = 8.0
        pitch_mm = thread_pitch if thread_pitch else coarse_pitch(thread_d)
        if thread_d <= bore_d + 1.6:
            raise ValueError("hose_barb(): thread_d must clear the bore by "
                             "0.8 mm of wall")
        solids.append(thread_solid(thread_d, foot_t, pitch=pitch_mm))
    if boss_h > 0:
        solids.append(cyl(boss_d / 2.0, boss_h, (0.0, 0.0, z_foot + boss_h / 2.0),
                          sections=sections))

    z = z_barb
    for _ in range(int(barbs)):
        if barb_gap > 0:
            solids.append(cyl(root_r, barb_gap, (0.0, 0.0, z + barb_gap / 2.0),
                              sections=sections))
        z += barb_gap
        solids.append(frustum(crest_r, root_r, ramp_len, z0=z, sections=sections))
        z += ramp_len
    if lead_in > 0:
        solids.append(frustum(root_r, max(0.5, root_r - lead_in), lead_in, z0=z,
                              sections=sections))
        z += lead_in

    mesh = uni(solids)
    mesh = sub(mesh, cyl(bore_d / 2.0, z + 4.0, (0.0, 0.0, (z - 2.0) / 2.0),
                         sections=sections))
    mesh.metadata.update({
        "tube_id": float(tube_id),
        "crest_d": 2.0 * crest_r,
        "root_d": 2.0 * root_r,
        "interference": float(interference),
        "bore_d": float(bore_d),
        "barbs": int(barbs),
        "barb_pitch": pitch,
        "ramp_deg": float(ramp_deg),
        "barb_z0": z_barb,
        "barb_z1": z_barb + barbs * pitch,
        "total_h": z,
        "boss_d": float(boss_d),
        "foot": foot,
    })
    return mesh


def _valve_routing(port_angles, passages, plug_r, port_d, passage_d):
    """Derive the detent angles and the ports each one connects.

    The routing table is not hand written: every plug angle that lines a
    passage leg up with a body port is a candidate detent, and the ports a
    detent joins are read back off the same alignment test. ``tol_deg`` is the
    angular half-overlap of a port bore and a passage bore at the plug surface.
    """
    tol = math.degrees(0.25 * (port_d + passage_d) / plug_r)
    candidates = set()
    for port in port_angles:
        for passage in passages:
            for leg in passage:
                candidates.add(round((port - leg) % 360.0, 6))
    table = []
    for theta in sorted(candidates):
        groups = []
        for passage in passages:
            joined = []
            for index, port in enumerate(port_angles):
                for leg in passage:
                    delta = abs(((leg + theta - port) + 180.0) % 360.0 - 180.0)
                    if delta <= tol:
                        joined.append(index)
                        break
            if len(joined) >= 2:
                groups.append(tuple(sorted(set(joined))))
        if groups:
            table.append((theta, tuple(sorted(set(groups)))))
    return tol, tuple(table)


def _valve_closed_angle(table, port_angles, passages, tol):
    """Return a plug angle at which no passage leg reaches any body port."""
    opens = sorted(theta for theta, _groups in table)
    if not opens:
        return 0.0
    best, best_gap = None, -1.0
    for index, theta in enumerate(opens):
        nxt = opens[(index + 1) % len(opens)] + (360.0 if index + 1 >= len(opens)
                                                 else 0.0)
        gap = nxt - theta
        if gap > best_gap:
            best, best_gap = (theta + gap / 2.0) % 360.0, gap
    for port in port_angles:
        for passage in passages:
            for leg in passage:
                delta = abs(((leg + best - port) + 180.0) % 360.0 - 180.0)
                if delta <= 2.0 * tol:
                    return None
    return best


def rotary_spool_valve(ports=3, body_d=34.0, plug_d=16.0, port_d=5.0,
                       passage_d=4.5, passages=((0.0, 120.0),),
                       port_angles_deg=None, plug_deg=0.0, seat_h=None,
                       floor_t=2.4, clear=0.3, detents=True, detent_d=2.4,
                       detent_lift=0.5, oring_groove=False, oring_cs=1.8,
                       collar_t=3.0, collar_d=None, stem_d=9.0, stem_h=10.0,
                       socket_af=5.5, socket_depth=6.0, handle_free_deg=4.0,
                       cap_t=3.0, sections=64):
    """Build a rotary spool (plug) valve as three printable parts.

    Returns ``{"body", "plug", "cap"}``. The body is a cylinder with ``ports``
    radial bores drilled in to a central seat; the plug is a solid cylinder
    cross-drilled with ``passages``, each one a set of plug-frame angles whose
    radial bores all meet on the plug axis. Turning the plug lines a passage's
    legs up with a chosen subset of the body ports, so each detent position
    routes flow between exactly those ports and blocks the rest. The detent
    angles and the port groups they connect are DERIVED from the port angles,
    the passage angles and the two bore diameters, and land in
    ``metadata["routing"]``; ``metadata["closed_deg"]`` is a plug angle at which
    no passage reaches any port.

    This leaks. A printed plug in a printed bore with ``clear`` mm of running
    fit is a throttle, not a seal: it is fine for routing low-pressure air or
    gravity-fed liquid and useless above a metre or so of head. Set
    ``oring_groove=True`` for two glands flanking the port band; the O-rings and
    the grease that makes them turn are not printed parts, and neither is any
    handle you put in the stem socket. ``handle_free_deg`` sweeps the hex handle
    socket into a fan, which buys print-orientation freedom at the cost of that
    much lost motion between handle and plug.

    z=0 is the body's bottom face and +Z is up. Print all three parts on that
    face. The plug seat and the stem socket are vertical bores; the radial port
    and passage bores are horizontal and print with a slightly sagged crown,
    which is acceptable on a flow bore but means the printed diameter runs a
    little under nominal. The detent is a cone pip on the plug collar dropping
    into a countersink in the body's top face, which clicks weakly on its own;
    add a wave washer under the cap for a real click. Units are mm and degrees.
    """
    if ports < 2:
        raise ValueError("rotary_spool_valve(): ports must be at least 2")
    if body_d <= 0 or plug_d <= 0 or port_d <= 0 or passage_d <= 0:
        raise ValueError("rotary_spool_valve(): diameters must be positive")
    if plug_d + 2.0 * clear >= body_d - 3.2:
        raise ValueError("rotary_spool_valve(): body wall under 1.6 mm; raise "
                         "body_d or drop plug_d")
    if floor_t <= 0 or collar_t <= 0 or cap_t <= 0 or stem_h <= 0:
        raise ValueError("rotary_spool_valve(): floor_t, collar_t, cap_t and "
                         "stem_h must be positive")
    if not passages:
        raise ValueError("rotary_spool_valve(): give at least one passage")
    for passage in passages:
        if len(passage) < 2:
            raise ValueError("rotary_spool_valve(): a passage needs at least "
                             "two angles to connect anything")

    plug_r = plug_d / 2.0
    if passage_d >= plug_d - 2.4:
        raise ValueError("rotary_spool_valve(): passage_d %.2f leaves under "
                         "1.2 mm of plug wall" % passage_d)
    if port_angles_deg is None:
        port_angles = [360.0 * k / ports for k in range(ports)]
    else:
        port_angles = [float(a) % 360.0 for a in port_angles_deg]
        if len(port_angles) != ports:
            raise ValueError("rotary_spool_valve(): port_angles_deg must have "
                             "one angle per port")
    seat_pitch = 2.0 * plug_r * math.sin(math.pi / ports)
    if port_d >= seat_pitch - 0.8:
        raise ValueError("rotary_spool_valve(): port_d %.2f merges adjacent "
                         "ports at the plug seat (pitch %.2f)"
                         % (port_d, seat_pitch))

    band = max(port_d, passage_d)
    groove_w = oring_cs * 1.2
    groove_off = band / 2.0 + oring_cs
    if seat_h is None:
        seat_h = (2.0 * groove_off + 2.0 * groove_w + 3.0 if oring_groove
                  else band + 6.0)
    if seat_h < band + 2.0:
        raise ValueError("rotary_spool_valve(): seat_h %.2f cannot hold the "
                         "port band" % seat_h)
    z_port = floor_t + seat_h / 2.0
    z_body = floor_t + seat_h
    if collar_d is None:
        collar_d = min(body_d, plug_d + 8.0)
    if collar_d <= plug_d or collar_d > body_d:
        raise ValueError("rotary_spool_valve(): collar_d must sit between "
                         "plug_d and body_d")
    if stem_d <= 0 or stem_d > collar_d:
        raise ValueError("rotary_spool_valve(): stem_d must fit inside the "
                         "collar")

    tol, table = _valve_routing(port_angles, passages, plug_r, port_d,
                                passage_d)
    if not table:
        raise ValueError("rotary_spool_valve(): no plug angle connects two "
                         "ports; check passages against port_angles_deg")
    closed_deg = _valve_closed_angle(table, port_angles, passages, tol)

    # Body: seat bore, radial ports, detent countersinks.
    body = cyl(body_d / 2.0, z_body, (0.0, 0.0, z_body / 2.0), sections=sections)
    body = sub(body, cyl(plug_r + clear, seat_h + 1.0,
                         (0.0, 0.0, floor_t + (seat_h + 1.0) / 2.0),
                         sections=sections))
    starts, vectors = [], []
    for angle in port_angles:
        a = math.radians(angle)
        starts.append((body_d * math.cos(a), body_d * math.sin(a), z_port))
        vectors.append((-math.cos(a), -math.sin(a), 0.0))
    body = sub(body, directed_holes(starts, vectors, port_d, body_d))
    if detents:
        if detent_d <= 0 or detent_lift < 0:
            raise ValueError("rotary_spool_valve(): detent_d must be positive")
        pip_h = detent_d / 2.0
        seat_ring_r = (collar_d + plug_d) / 4.0
        sinks = []
        for theta, _groups in table:
            a = math.radians(theta)
            sinks.append(frustum(0.15, detent_d / 2.0, pip_h,
                                 z0=z_body - pip_h, sections=24))
            sinks[-1].apply_translation((seat_ring_r * math.cos(a),
                                         seat_ring_r * math.sin(a), 0.0))
        body = sub(body, uni(sinks))

    # Plug: cross-drilled passages, collar, detent pip, keyed stem socket.
    plug_h = seat_h - clear
    plug = cyl(plug_r, plug_h, (0.0, 0.0, floor_t + clear + plug_h / 2.0),
               sections=sections)
    starts, vectors = [], []
    for passage in passages:
        for leg in passage:
            a = math.radians(leg)
            # Start each leg a bore radius BEHIND the axis so every leg of a
            # passage overlaps solidly at the centre: butting flat end caps
            # together there leaves the axis point on a boolean boundary and
            # the passage reads as blocked at some plug angles and open at
            # others purely from rounding.
            starts.append((-passage_d * math.cos(a), -passage_d * math.sin(a),
                           z_port))
            vectors.append((math.cos(a), math.sin(a), 0.0))
    plug = sub(plug, directed_holes(starts, vectors, passage_d,
                                    plug_d + 2.0 * passage_d))
    if oring_groove:
        for sign in (-1.0, 1.0):
            zc = z_port + sign * groove_off
            if not floor_t + clear + groove_w / 2.0 + 0.6 < zc < z_body - groove_w / 2.0 - 0.6:
                raise ValueError("rotary_spool_valve(): O-ring glands fall "
                                 "outside the plug; raise seat_h")
            gland = sub(cyl(plug_r + 1.0, groove_w, (0.0, 0.0, zc),
                            sections=sections),
                        cyl(plug_r - oring_cs * 0.75, groove_w + 1.0,
                            (0.0, 0.0, zc), sections=sections))
            plug = sub(plug, gland)
    collar_z = z_body
    collar = cyl(collar_d / 2.0, collar_t,
                 (0.0, 0.0, collar_z + collar_t / 2.0), sections=sections)
    stem = cyl(stem_d / 2.0, stem_h,
               (0.0, 0.0, collar_z + collar_t + stem_h / 2.0), sections=sections)
    plug = uni([plug, collar, stem])
    if detents:
        pip_h = detent_d / 2.0
        seat_ring_r = (collar_d + plug_d) / 4.0
        pip = frustum(0.15, detent_d / 2.0 - 0.15, pip_h, z0=collar_z - pip_h,
                      sections=24)
        pip.apply_translation((seat_ring_r, 0.0, 0.0))
        plug = uni([plug, pip])
    socket_top = collar_z + collar_t + stem_h
    if socket_depth > 0:
        if socket_af + 1.6 > stem_d:
            raise ValueError("rotary_spool_valve(): socket_af leaves under "
                             "0.8 mm of stem wall")
        if socket_depth >= stem_h + collar_t:
            raise ValueError("rotary_spool_valve(): socket_depth cuts through "
                             "the collar")
        socket_2d = swept_keyed_bore(hex_poly(socket_af + clear),
                                     float(handle_free_deg), steps=16)
        socket = extrude_poly_z(socket_2d, socket_top - socket_depth,
                                socket_top + 1.0)
        plug = sub(plug, socket)
    plug.apply_transform(tf.rotation_matrix(math.radians(plug_deg), (0, 0, 1)))

    # Cap: traps the collar with detent_lift of axial float.
    cap_z0 = z_body
    cap_z1 = collar_z + collar_t + detent_lift + cap_t
    cap = cyl(body_d / 2.0, cap_z1 - cap_z0, (0.0, 0.0, (cap_z0 + cap_z1) / 2.0),
              sections=sections)
    cap = sub(cap, cyl(collar_d / 2.0 + clear,
                       collar_t + detent_lift + 1.0,
                       (0.0, 0.0, cap_z0 - 0.5 + (collar_t + detent_lift + 1.0) / 2.0),
                       sections=sections))
    cap = sub(cap, cyl(stem_d / 2.0 + clear, cap_t + 2.0,
                       (0.0, 0.0, cap_z1 - cap_t / 2.0), sections=sections))

    meta = {
        "ports": int(ports),
        "port_angles_deg": tuple(port_angles),
        "port_d": float(port_d),
        "passage_d": float(passage_d),
        "passages": tuple(tuple(float(a) for a in p) for p in passages),
        "align_tol_deg": tol,
        "routing": table,
        "detent_angles_deg": tuple(theta for theta, _g in table),
        "closed_deg": closed_deg,
        "plug_deg": float(plug_deg),
        "port_z": z_port,
        "body_d": float(body_d),
        "plug_d": float(plug_d),
        "seat_h": float(seat_h),
        "body_h": z_body,
        "total_h": cap_z1,
        "clear": float(clear),
        "oring_cs": float(oring_cs) if oring_groove else 0.0,
    }
    parts = {"body": body, "plug": plug, "cap": cap}
    for mesh in parts.values():
        mesh.metadata.update(meta)
    return parts


def peristaltic_pump_head(rollers=3, tube_od=6.0, tube_wall=1.5,
                          occlusion=0.9, race_r=18.0, roller_d=8.0,
                          wrap_deg=240.0, rotor_h=None, wall=4.0, floor_t=3.0,
                          cap_t=3.0, shaft_d=5.0, clear=0.3, sections=96):
    """Build a roller peristaltic pump head as three printable parts.

    Returns ``{"body", "rotor", "cap"}``. ``rollers`` posts on the rotor pinch a
    flexible tube against the circular race wall, so each post drags a sealed
    slug of fluid round the wrap and the tube's own elasticity refills behind
    it. The squeeze gap is set from ``occlusion``: 1.0 closes the tube to twice
    its wall, 0 leaves it round. The tube enters and leaves through two slots
    whose outer edges are TANGENT to the race wall by construction, so the tube
    peels off the wall along a continuous curve instead of over a step edge,
    which is what stops a post shearing it at the race entry.

    The tube is not a printed part. Silicone or PVC of ``tube_od`` outside and
    ``tube_od - 2*tube_wall`` bore is what makes this a pump, and it is a
    consumable: peristaltic tubing work-hardens and eventually splits. The posts
    are integral and SLIDE on the tube rather than rolling, which costs torque
    and tube life; drill them out for bearings on pins if you are running it for
    more than a few minutes. ``wrap_deg`` must exceed one roller pitch or no
    post is occluding at some angles and the pump back-feeds, which is checked.

    z=0 is the body's bottom face and +Z is up. Print all three flat on that
    face: the race and the tube slots are open-topped notches with vertical
    walls, so nothing overhangs. Units are mm and degrees.
    """
    if rollers < 2:
        raise ValueError("peristaltic_pump_head(): rollers must be at least 2")
    if tube_od <= 0 or tube_wall <= 0 or 2.0 * tube_wall >= tube_od:
        raise ValueError("peristaltic_pump_head(): tube_wall must be under "
                         "half of tube_od")
    if not 0.0 < occlusion <= 1.0:
        raise ValueError("peristaltic_pump_head(): occlusion must be in (0, 1]")
    if race_r <= 0 or roller_d <= 0 or wall <= 0 or floor_t <= 0 or cap_t <= 0:
        raise ValueError("peristaltic_pump_head(): race_r, roller_d, wall, "
                         "floor_t and cap_t must be positive")
    pitch_deg = 360.0 / rollers
    if wrap_deg <= pitch_deg or wrap_deg >= 355.0:
        raise ValueError(
            "peristaltic_pump_head(): wrap_deg %.1f must sit between one "
            "roller pitch (%.1f) and 355 or the tube is unoccluded at some "
            "angles" % (wrap_deg, pitch_deg))

    tube_id = tube_od - 2.0 * tube_wall
    squeeze = tube_od - occlusion * (tube_od - 2.0 * tube_wall)
    race_outer_r = race_r + tube_od / 2.0 + clear / 2.0
    roller_r = race_outer_r - squeeze - roller_d / 2.0
    if roller_r - roller_d / 2.0 <= shaft_d / 2.0 + 1.2:
        raise ValueError("peristaltic_pump_head(): roller circle collapses "
                         "onto the shaft; raise race_r or drop roller_d")
    if 2.0 * roller_r * math.sin(math.pi / rollers) <= roller_d + 0.8:
        raise ValueError("peristaltic_pump_head(): roller posts collide; "
                         "reduce rollers or roller_d")
    chan_h = tube_od + clear
    if rotor_h is None:
        rotor_h = chan_h - 2.0 * clear
    if not 0 < rotor_h <= chan_h - 2.0 * clear:
        raise ValueError("peristaltic_pump_head(): rotor_h must fit the "
                         "%.2f mm race channel with running clearance"
                         % chan_h)
    body_r = race_outer_r + wall
    z0, z1 = floor_t, floor_t + chan_h

    slot_w = tube_od + clear
    open_half = (360.0 - wrap_deg) / 2.0
    voids = [sg.Point(0.0, 0.0).buffer(race_outer_r, resolution=sections // 4)]
    for angle, sense in ((open_half, -1.0), (360.0 - open_half, 1.0)):
        a = math.radians(angle)
        nx, ny = math.cos(a), math.sin(a)
        dx, dy = -sense * math.sin(a), sense * math.cos(a)
        px, py = race_r * nx, race_r * ny
        reach = 2.0 * body_r
        voids.append(sg.Polygon([
            (px + nx * slot_w / 2.0, py + ny * slot_w / 2.0),
            (px + nx * slot_w / 2.0 + dx * reach,
             py + ny * slot_w / 2.0 + dy * reach),
            (px - nx * slot_w / 2.0 + dx * reach,
             py - ny * slot_w / 2.0 + dy * reach),
            (px - nx * slot_w / 2.0, py - ny * slot_w / 2.0)]))
    void = unary_union(voids)

    body = cyl(body_r, z1, (0.0, 0.0, z1 / 2.0), sections=sections)
    body = sub(body, extrude_poly_z(void, z0, z1 + 1.0))
    body = sub(body, cyl((shaft_d + clear) / 2.0, floor_t + 2.0,
                         (0.0, 0.0, floor_t / 2.0), sections=48))

    rotor_z = z0 + clear
    posts = [cyl(roller_d / 2.0, rotor_h,
                 (px, py, rotor_z + rotor_h / 2.0), sections=48)
             for px, py in polar_ring(rollers, roller_r)]
    rotor = uni([cyl(roller_r, rotor_h, (0.0, 0.0, rotor_z + rotor_h / 2.0),
                     sections=sections)] + posts)
    rotor = sub(rotor, cyl((shaft_d + clear) / 2.0, rotor_h + 2.0,
                           (0.0, 0.0, rotor_z + rotor_h / 2.0), sections=48))

    cap = cyl(body_r, cap_t, (0.0, 0.0, z1 + cap_t / 2.0), sections=sections)
    cap = sub(cap, cyl((shaft_d + clear) / 2.0, cap_t + 2.0,
                       (0.0, 0.0, z1 + cap_t / 2.0), sections=48))

    meta = {
        "rollers": int(rollers),
        "tube_od": float(tube_od),
        "tube_id": tube_id,
        "tube_wall": float(tube_wall),
        "occlusion": float(occlusion),
        "squeeze_gap": squeeze,
        "race_r": float(race_r),
        "race_outer_r": race_outer_r,
        "roller_r": roller_r,
        "roller_d": float(roller_d),
        "wrap_deg": float(wrap_deg),
        "roller_pitch_deg": pitch_deg,
        "channel_h": chan_h,
        "rotor_h": float(rotor_h),
        "body_d": 2.0 * body_r,
        "clear": float(clear),
        # Ideal displacement: one roller sweeps the whole race each turn and
        # the tube bore refills behind it.
        "displacement_per_rev": (math.pi / 4.0) * tube_id ** 2 * 2.0 * math.pi
                                * race_r,
    }
    parts = {"body": body, "rotor": rotor, "cap": cap}
    for mesh in parts.values():
        mesh.metadata.update(meta)
    return parts


def external_gear_pump(teeth=12, module=1.5, width=8.0, pressure_angle=20.0,
                       backlash=0.35, wall=3.0, base_t=4.0, cap_t=4.0,
                       shaft_d=5.0, clearance=0.3, phase_deg=0.0):
    """Build an external spur-gear pump with a close-fitting housing.

    Two identical involute gears of ``teeth`` / ``module`` mesh on centres
    spaced one pitch diameter apart inside a figure-eight bore. Inlet and
    outlet pockets sit on the mesh line so fluid is carried around the
    outside of each gear from one port to the other. ``phase_deg`` rotates
    the driving gear; the driven gear counter-rotates by the same amount
    after the fixed ``mesh_phase`` tooth-to-gap offset (conjugate ratio -1
    for equal tooth counts). Gears are extruded once and reposed by rigid
    transforms so a phase sweep is a pure rigid motion. Returns
    ``{"body", "gear_a", "gear_b", "cap", "displacement_per_rev"}``.
    Units mm / degrees.
    """
    from .gears import mesh_phase, spur_gear_2d

    if (teeth < 8 or module <= 0 or width < 1.2 or
            not 14.0 <= pressure_angle <= 30.0 or backlash < 0 or
            wall < 1.2 or base_t < 1.2 or cap_t < 1.2 or
            shaft_d < 0 or clearance < 0):
        raise ValueError("external_gear_pump(): invalid pump dimensions")
    pitch_r = module * teeth / 2.0
    tip_r = pitch_r + module
    centre_dist = 2.0 * pitch_r
    # Tip circles almost touch the housing with clearance.
    cavity_r = tip_r + clearance
    body_r = cavity_r + wall

    # Extrude a single blank, then rigid-transform both members. Rebuilding
    # the 2d profile per phase re-facetises the bores and fails the gallery
    # rigid-recovery bake.
    gear_poly = spur_gear_2d(N=teeth, m=module, pa=pressure_angle,
                             bl=backlash)
    z0 = base_t + clearance
    blank = extrude_poly_z(gear_poly, z0, z0 + width)
    if shaft_d > 0:
        blank = sub(blank, cyl((shaft_d + clearance) / 2.0, width + 2.0,
                               (0.0, 0.0, z0 + width / 2.0)))
    # mesh_phase's third arg is the line-of-centres azimuth, not driver spin.
    # Centres lie on +X, so the fixed tooth-to-gap offset is at phi=0.
    mesh_off = mesh_phase(teeth, teeth, 0.0)
    gear_a = blank.copy()
    gear_a.apply_transform(tf.rotation_matrix(math.radians(phase_deg),
                                              (0.0, 0.0, 1.0)))
    gear_b = blank.copy()
    gear_b.apply_transform(tf.rotation_matrix(
        math.radians(mesh_off - phase_deg), (0.0, 0.0, 1.0)))
    gear_b.apply_translation((centre_dist, 0.0, 0.0))

    # Figure-eight cavity: two overlapping discs.
    c1 = sg.Point(0.0, 0.0).buffer(cavity_r, resolution=64)
    c2 = sg.Point(centre_dist, 0.0).buffer(cavity_r, resolution=64)
    cavity = c1.union(c2)
    # Port pockets top and bottom on the mesh line.
    port = sg.box(pitch_r - module, cavity_r - 0.5,
                  pitch_r + module, cavity_r + wall * 0.6)
    port2 = affinity.translate(port, 0.0, -2.0 * (cavity_r + wall * 0.3))
    void = cavity.union(port).union(port2)

    body_outer = sg.Point(centre_dist / 2.0, 0.0).buffer(
        body_r + centre_dist / 2.0, resolution=64)
    # Tighter rectangular envelope with rounded ends.
    body_outer = c1.buffer(wall).union(c2.buffer(wall)).buffer(0)
    z1 = base_t + width + 2.0 * clearance
    body = extrude_poly_z(body_outer, 0.0, z1)
    body = sub(body, extrude_poly_z(void, base_t, z1 + 0.5))
    # Shaft bores through the base.
    if shaft_d > 0:
        for cx in (0.0, centre_dist):
            body = sub(body, cyl((shaft_d + clearance) / 2.0, base_t + 2.0,
                                 (cx, 0.0, base_t / 2.0)))

    cap = extrude_poly_z(body_outer, z1, z1 + cap_t)
    if shaft_d > 0:
        for cx in (0.0, centre_dist):
            cap = sub(cap, cyl((shaft_d + clearance) / 2.0, cap_t + 2.0,
                               (cx, 0.0, z1 + cap_t / 2.0)))

    # Approximate displacement: volume of tooth spaces carried per rev.
    # Classic estimate: 2 * pi * pitch_r * module * width (both gears).
    displacement = 2.0 * math.pi * pitch_r * module * width
    meta = {"teeth": teeth, "module": module, "width": width,
            "centre_dist": centre_dist, "displacement_per_rev": displacement}
    parts = {"body": body, "gear_a": gear_a, "gear_b": gear_b, "cap": cap}
    for mesh in parts.values():
        mesh.metadata.update(meta)
    parts["displacement_per_rev"] = displacement
    return parts
