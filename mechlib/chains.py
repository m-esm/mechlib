"""Link-train generators: print-in-place drag chains and roller-chain sets."""

import math

import numpy as np
import shapely.affinity as sa
import shapely.geometry as sg
from shapely.ops import unary_union
import trimesh
import trimesh.transformations as tf

from .meshutil import from_manifold, to_manifold, sub, uni
from .prim import cyl, sector2d


def _extrude(poly, z0, z1):
    """Extrude a Shapely Polygon or MultiPolygon between two Z planes.

    Repairs each part via manifold3d if trimesh leaves it non-watertight.
    """
    h = float(z1 - z0)
    if h <= 0:
        raise ValueError("extrusion height must be positive")
    poly = poly.buffer(0)
    geoms = poly.geoms if poly.geom_type == "MultiPolygon" else [poly]
    parts = []
    for g in geoms:
        if g.is_empty or g.area < 1e-6:
            continue
        m = trimesh.creation.extrude_polygon(g, h)
        if not m.is_watertight:
            m = from_manifold(to_manifold(m))
        parts.append(m)
    if not parts:
        raise ValueError("_extrude(): polygon is empty")
    mesh = parts[0] if len(parts) == 1 else uni(parts)
    if z0:
        mesh.apply_translation((0.0, 0.0, z0))
    return mesh


def _capsule_2d(p0, p1, width, sections=48):
    line = sg.LineString([p0, p1])
    return line.buffer(width / 2.0, resolution=max(4, sections // 4),
                       cap_style=1, join_style=1)


def _pose(mesh, pos_xy, heading_deg):
    """Copy ``mesh`` and place it at ``pos_xy`` rotated ``heading_deg`` about Z."""
    m = mesh.copy()
    T = tf.rotation_matrix(math.radians(heading_deg), (0, 0, 1))
    T[0, 3] = pos_xy[0]
    T[1, 3] = pos_xy[1]
    m.apply_transform(T)
    return m


def _y_taper_box(x0, x1, y_w0, y_w1, z0, z1):
    """Box that is ``y_w0`` wide (Y) at ``z0`` and ``y_w1`` wide at ``z1``.

    Built as a convex hull of the two end rectangles, so the Y walls taper
    linearly (a flat-roofed box degenerates to ``y_w0 == y_w1``).
    """
    pts = []
    for yw, z in ((y_w0, z0), (y_w1, z1)):
        for x in (x0, x1):
            for sy in (-1.0, 1.0):
                pts.append((x, sy * yw / 2.0, z))
    return trimesh.Trimesh(vertices=np.array(pts)).convex_hull


def drag_chain_link(pitch=20.0, width=9.0, height=8.0, bend_deg=30.0,
                    pin_d=2.4, clear=0.3, wall=1.6,
                    tab_hw_deg=4.0, stop_clear_deg=2.5,
                    roof_overhang=1.0, roof_h=1.2, lid=True,
                    lid_t=1.4, sections=48):
    """Build one link of a print-in-place cable-carrier (energy/drag) chain.

    The link is a flat open-top trough: two side walls of thickness ``wall``
    stand ``height`` mm tall around a channel ``width`` mm wide that carries a
    cable, printed with the trough floor on the bed and the walls vertical.
    One end (local x=0) is a FEMALE socket — the outer collar bored out to a
    ring, inner radius ``sock_r`` — and the other (local x=``pitch``) is a
    round MALE boss post of radius ``boss_r = sock_r - clear`` that nests
    inside the neighbour's ring with ``clear`` mm radial running clearance.
    Both features run the link's FULL height (never a partial-depth pocket),
    so the pivot is a plain Z-axis through-bore-and-post pair: every layer of
    every hole is a complete supported ring and every layer of the post is a
    complete supported disc, so the hinge itself needs no bridging support at
    any diameter (unlike a horizontal pin bore, which would).

    The socket also carries a keyway-shaped stop GROOVE cut into its ring,
    and the boss a matching stop TAB projecting from its post — posed so the
    tab travels freely through the groove for relative rotation in
    ``[-stop_clear_deg, bend_deg + stop_clear_deg]`` and jams against a
    groove wall just outside that window: reverse bending is blocked within
    about ``stop_clear_deg`` of straight (drag chains stay rigid the wrong
    way), forward bending is capped ``stop_clear_deg`` past ``bend_deg``.
    Both are Z-prismatic cuts/posts too — no overhang.

    The trough opening itself narrows at the very top: over the last
    ``roof_h`` mm the walls lean inward by ``roof_overhang`` mm per side
    (``atan(roof_overhang / roof_h)`` from vertical, kept under 45 degrees by
    default), an angled roof lip that partially retains the cable even
    without a lid — the FDM-correct alternative to a flat horizontal roof,
    which would need bridging support to close over the channel width.

    If ``lid=True`` the dict also carries a friction-fit ("press-on", not a
    true cantilever snap — FDM plastic across a 1.6 mm wall is not reliable
    enough for a repeatable snap) cable-retention lid: a flat cap plus an
    underside plug sized ``width - 0.3`` mm so it presses lightly into the
    open channel and holds by friction alone.

    Returns ``{"link", "pin"}`` plus ``"lid"`` when requested. ``link.metadata``
    carries ``pitch``, ``bend_deg``, ``min_bend_radius`` (mm — the radius of
    the circle a fully-curled run of these links traces, ``pitch / (2 *
    sin(bend_deg/2))``), ``boss_r``, ``sock_r``, and ``bore_r``.

    Print the link flat (as generated, z=0 on the bed) with no supports; print
    the pin standing on end. Units are mm and degrees.
    """
    if (pitch <= 0 or width < 6.0 or height <= 0 or wall < 0.8 or
            pin_d <= 0 or clear < 0.05 or
            tab_hw_deg <= 0 or stop_clear_deg < 0 or lid_t <= 0 or
            roof_overhang < 0 or roof_h <= 0 or sections < 12):
        raise ValueError("drag_chain_link(): invalid link dimensions")
    if not 5.0 <= bend_deg <= 120.0:
        raise ValueError(
            "drag_chain_link(): bend_deg must be between 5 and 120 degrees")
    if roof_overhang > roof_h or 2.0 * roof_overhang > width - 1.0:
        raise ValueError(
            "drag_chain_link(): roof_overhang too large for width or too "
            "steep for roof_h (must stay within 45 degrees of vertical)")
    if roof_h >= height - wall:
        raise ValueError(
            "drag_chain_link(): roof_h too tall for the wall height")

    boss_r = max(1.6, width * 0.26)
    tab_len = max(0.8, boss_r * 0.5)
    sock_r = boss_r + clear
    collar_r = width / 2.0 + wall
    outer_groove_r = sock_r + tab_len + clear
    # neck_r bounds BOTH pivots' local radius (the socket ring's OD and the
    # circle the boss end is carved down to) — deliberately narrower than
    # collar_r. A capsule's rounded end cap is full collar_r width for its
    # WHOLE angular sweep (not just near the tip), so if only the boss end
    # were narrowed, the socket end's still-full-width wall would graze the
    # boss link's wall at any nonzero bend angle. Carving BOTH ends down to
    # neck_r (via the same collar_r-radius carve, so the transition back to
    # full collar_r width happens at the same radius on both sides) confines
    # all near-pivot material for both links to radius neck_r, so it is the
    # stop tab that decides contact, not the wall edges.
    neck_r = outer_groove_r + 0.8
    if neck_r + 0.8 > collar_r:
        raise ValueError(
            "drag_chain_link(): width too small for the boss/stop geometry; "
            "increase width or reduce wall")
    bore_r = (pin_d + clear) / 2.0
    if bore_r + 0.6 > boss_r:
        raise ValueError(
            "drag_chain_link(): pin_d too large for the boss radius")
    ch_margin = collar_r * 0.95
    if pitch - 2.0 * ch_margin < 1.0:
        raise ValueError(
            "drag_chain_link(): pitch too short for width/wall; the cable "
            "trough would vanish")

    body2d = _capsule_2d((0.0, 0.0), (pitch, 0.0), 2.0 * collar_r, sections)
    lx0, lx1 = ch_margin, pitch - ch_margin

    lo = -(bend_deg + tab_hw_deg + stop_clear_deg)
    hi = tab_hw_deg + stop_clear_deg
    n_arc = max(8, sections // 4)
    pocket = sg.Point(0.0, 0.0).buffer(sock_r, resolution=sections)
    groove = sector2d(lo, hi, outer_groove_r, n=n_arc).difference(
        sg.Point(0.0, 0.0).buffer(max(0.1, sock_r - 0.05), resolution=sections))
    socket_keep2d = sg.Point(0.0, 0.0).buffer(
        neck_r, resolution=sections).difference(
        unary_union([pocket, groove]).buffer(0))
    socket_carve2d = sg.Point(0.0, 0.0).buffer(
        collar_r + 1.0, resolution=sections).difference(socket_keep2d)

    boss2d = sg.Point(pitch, 0.0).buffer(boss_r, resolution=sections)
    tab_local = sector2d(-tab_hw_deg, tab_hw_deg, boss_r + tab_len, n=n_arc).difference(
        sg.Point(0.0, 0.0).buffer(max(0.1, boss_r - 0.05), resolution=sections))
    tab2d = sa.translate(tab_local, xoff=pitch)
    boss_keep2d = unary_union([boss2d, tab2d]).buffer(0)
    # Everything within collar_r of the boss end OUTSIDE the boss/tab shape is
    # carved away, turning the rounded full-width end cap into a narrow post
    # (mirrors the socket carve above, at the same collar_r carve radius).
    boss_carve2d = sg.Point(pitch, 0.0).buffer(
        collar_r + 1.0, resolution=sections).difference(boss_keep2d)

    link = _extrude(body2d, 0.0, height)
    link = sub(link, _extrude(boss_carve2d, 0.0, height))
    link = sub(link, _extrude(socket_carve2d, 0.0, height))
    for cx, cy in ((0.0, 0.0), (pitch, 0.0)):
        bore = cyl(bore_r, height + 2.0, center=(cx, cy, height / 2.0),
                  sections=sections)
        link = sub(link, bore)
    # Angled roof: the trough opening narrows over the top roof_h mm instead
    # of a flat (bridging) lid-height shelf.
    roof_cut = _y_taper_box(lx0, lx1, width, width - 2.0 * roof_overhang,
                            height - roof_h, height + 0.5)
    lower_cut = trimesh.creation.box(
        extents=(lx1 - lx0, width, height - roof_h - wall))
    lower_cut.apply_translation(((lx0 + lx1) / 2.0, 0.0,
                                 wall + (height - roof_h - wall) / 2.0))
    channel_cut = uni([lower_cut, roof_cut])
    link = sub(link, channel_cut)

    min_r = pitch / (2.0 * math.sin(math.radians(bend_deg / 2.0)))
    link.metadata.update({
        "pitch": pitch, "bend_deg": bend_deg, "min_bend_radius": min_r,
        "boss_r": boss_r, "sock_r": sock_r, "bore_r": bore_r,
    })

    pin_extra = 1.0
    pin = cyl(pin_d / 2.0, height + 2.0 * pin_extra,
             center=(0.0, 0.0, height / 2.0), sections=sections)

    out = {"link": link, "pin": pin}
    if lid:
        plate2d = sg.box(lx0 - wall * 0.6, -collar_r + 0.4,
                         lx1 + wall * 0.6, collar_r - 0.4)
        cap = _extrude(plate2d, 0.0, lid_t)
        plug_w = max(1.0, width - 2.0 * roof_overhang - 0.3)
        plug_h = min(1.5, max(0.0, roof_h - 0.4))
        if plug_h > 0.4:
            plug2d = sg.box(lx0, -plug_w / 2.0, lx1, plug_w / 2.0)
            plug = _extrude(plug2d, -plug_h, 0.0)
            cap = uni([cap, plug])
        out["lid"] = cap
    return out


def drag_chain(links=8, bend_deg=30.0, straight_links=3, pitch=20.0,
              width=9.0, height=8.0, pin_d=2.4, clear=0.3, wall=1.6,
              lid=True):
    """Pose an N-link run of ``drag_chain_link`` with a straight run and a bend.

    The first ``straight_links`` joints stay unbent (a straight run); the
    remaining joints are posed at each link's ``bend_deg`` stop, so the tail
    curls into the classic cable-carrier U-turn at its designed minimum bend
    radius (see ``drag_chain_link``'s ``min_bend_radius`` metadata). All links
    share one XY plane (Z is the print-up / trough-depth axis for every
    link), so the whole run is a flat top-down layout — the same view a
    desktop cable chain guiding a wire along a horizontal rail would present.

    Returns a dict of posed meshes named ``link_NN``, ``pin_NN``, and (when
    ``lid=True``) ``lid_NN`` for ``NN`` in ``00..links-1``. Units are mm and
    degrees.
    """
    if links < 3:
        raise ValueError("drag_chain(): links must be at least 3")
    if not 0 <= straight_links < links:
        raise ValueError(
            "drag_chain(): straight_links must be within [0, links)")
    base = drag_chain_link(pitch=pitch, width=width, height=height,
                           bend_deg=bend_deg, pin_d=pin_d, clear=clear,
                           wall=wall, lid=lid)
    link_mesh, pin_mesh, lid_mesh = base["link"], base["pin"], base.get("lid")

    parts = {}
    pos = np.array([0.0, 0.0])
    heading = 0.0
    for i in range(links):
        parts["link_%02d" % i] = _pose(link_mesh, pos, heading)
        parts["pin_%02d" % i] = _pose(pin_mesh, pos, heading)
        if lid_mesh is not None:
            parts["lid_%02d" % i] = _pose(lid_mesh, pos, heading)
        step_bend = 0.0 if i < straight_links else bend_deg
        rad = math.radians(heading)
        pos = pos + pitch * np.array([math.cos(rad), math.sin(rad)])
        heading += step_bend
    return parts


def roller_chain_link(pitch=12.7, roller_d=7.75, plate_t=1.6, plate_w=None,
                      pin_d=3.0, bushing_wall=0.8, roller_wall=1.1,
                      clear=0.3, gap=0.3, roller_len=None, sections=48):
    """Build one pitch-length segment of a printable bush roller chain.

    Mates with a sprocket built by ``gears.roller_sprocket_2d(n_teeth, pitch,
    pin_d=roller_d, clear=clear)`` — pass THIS function's ``pitch``, its
    ``roller_d`` as that call's ``pin_d``, and the same ``clear``; the pitch
    circle radius ``pitch / (2 * sin(pi / n_teeth))`` and the tooth-gap
    envelope radius ``roller_d / 2 + clear`` then line up exactly.

    All mating surfaces are running-clearance slip fits rather than the
    press fits of a metal chain (pin-in-bushing, bushing-in-inner-plate,
    pin-in-outer-plate all get ``clear`` or ``gap`` mm) — printable and
    serviceable, at the cost of the rigidity a press fit would give the
    pin/outer-plate and bushing/inner-plate pairs in a real chain.

    Layout along Z (the sprocket's axis), bottom to top: ``outer_plate``,
    then a ``gap``, then ``inner_plate``, then a ``gap``, then the
    ``roller``/``bushing`` pair. Along X: both plates carry bored holes at
    local x=0 and x=``pitch``; the ``roller``, ``bushing``, and ``pin`` sit at
    x=0 only — tile a run by translating copies by ``pitch`` along X, as
    ``roller_chain`` does. ``bushing`` is a tube (OD, ID=``pin_d + clear``)
    the ``pin`` runs through; ``roller`` is a free ring (ID=bushing OD +
    ``clear``, OD=``roller_d``) around the bushing.

    Returns ``{"inner_plate", "outer_plate", "roller", "bushing", "pin"}``.
    ``roller.metadata["roller_d"]`` and ``["pitch"]`` restate the mating
    dimensions. Print plates and the roller/bushing flat (as generated); print
    the pin standing on end. Units are mm and degrees.
    """
    if (pitch <= 0 or roller_d <= 0 or plate_t <= 0 or pin_d <= 0 or
            bushing_wall <= 0 or roller_wall <= 0 or clear < 0.05 or
            gap < 0.05 or sections < 12):
        raise ValueError("roller_chain_link(): invalid chain dimensions")
    if plate_w is None:
        plate_w = roller_d * 0.95
    if roller_len is None:
        roller_len = plate_t * 2.0

    bushing_id = pin_d + clear
    bushing_od = bushing_id + 2.0 * bushing_wall
    roller_id = bushing_od + clear
    if roller_id + 2.0 * roller_wall > roller_d:
        raise ValueError(
            "roller_chain_link(): roller_d too small for pin_d/bushing_wall/"
            "roller_wall; increase roller_d")

    outer_hole_d = pin_d + clear
    inner_hole_d = bushing_od + clear
    if plate_w < max(outer_hole_d, inner_hole_d) + 1.6:
        raise ValueError(
            "roller_chain_link(): plate_w too small for the pin/bushing holes")

    def _plate2d(hole_d):
        bar = _capsule_2d((0.0, 0.0), (pitch, 0.0), plate_w, sections)
        for cx in (0.0, pitch):
            bar = bar.difference(
                sg.Point(cx, 0.0).buffer(hole_d / 2.0, resolution=sections))
        return bar.buffer(0)

    outer_z0 = 0.0
    outer_z1 = plate_t
    inner_z0 = outer_z1 + gap
    inner_z1 = inner_z0 + plate_t
    roll_z0 = inner_z1 + gap
    roll_z1 = roll_z0 + roller_len

    outer_plate = _extrude(_plate2d(outer_hole_d), outer_z0, outer_z1)
    inner_plate = _extrude(_plate2d(inner_hole_d), inner_z0, inner_z1)

    bushing_outer = cyl(bushing_od / 2.0, roll_z1 - roll_z0,
                        center=(0.0, 0.0, (roll_z0 + roll_z1) / 2.0),
                        sections=sections)
    bushing_bore = cyl(bushing_id / 2.0, roll_z1 - roll_z0 + 2.0,
                       center=(0.0, 0.0, (roll_z0 + roll_z1) / 2.0),
                       sections=sections)
    bushing = sub(bushing_outer, bushing_bore)

    roller_outer = cyl(roller_d / 2.0, roll_z1 - roll_z0,
                       center=(0.0, 0.0, (roll_z0 + roll_z1) / 2.0),
                       sections=sections)
    roller_bore = cyl(roller_id / 2.0, roll_z1 - roll_z0 + 2.0,
                      center=(0.0, 0.0, (roll_z0 + roll_z1) / 2.0),
                      sections=sections)
    roller = sub(roller_outer, roller_bore)
    roller.metadata.update({"roller_d": roller_d, "pitch": pitch,
                            "bushing_od": bushing_od})

    total_h = roll_z1 + inner_z1 - inner_z0 + gap + outer_z1 - outer_z0
    pin = cyl(pin_d / 2.0, total_h,
             center=(0.0, 0.0, total_h / 2.0 - outer_z0), sections=sections)

    return {"inner_plate": inner_plate, "outer_plate": outer_plate,
            "roller": roller, "bushing": bushing, "pin": pin}


def roller_chain(n_teeth=14, pitch=9.525, roller_d=6.0, clear=0.3,
                 wrap_deg=200.0, sprocket_width=6.0, outer_d=None):
    """Wrap a short roller-chain run around a sprocket built to match it.

    Builds ``gears.roller_sprocket_2d(n_teeth, pitch, pin_d=roller_d, clear,
    outer_d)`` extruded to ``sprocket_width``, then poses one bare roller (no
    plates — this is a visual/verification aid, not a full driveline part;
    build ``roller_chain_link`` runs separately for the load path) at every
    pitch station across ``wrap_deg`` of the pitch circle, each seated with
    ``clear`` mm running clearance in its tooth gap by construction.

    Returns ``{"sprocket", "roller_00", "roller_01", ...}``. Units are mm and
    degrees.
    """
    from .gears import roller_sprocket_2d
    from .meshutil import extrude_poly_z

    if n_teeth < 5 or pitch <= 0 or roller_d <= 0 or clear < 0.05:
        raise ValueError("roller_chain(): invalid sprocket or roller dimensions")
    if not 10.0 <= wrap_deg <= 360.0:
        raise ValueError("roller_chain(): wrap_deg must be between 10 and 360")
    if sprocket_width <= 0:
        raise ValueError("roller_chain(): sprocket_width must be positive")

    profile = roller_sprocket_2d(n_teeth, pitch, roller_d, clear=clear,
                                 outer_d=outer_d)
    sprocket = extrude_poly_z(profile, 0.0, sprocket_width)

    rp = pitch / (2.0 * math.sin(math.pi / n_teeth))
    step_deg = 360.0 / n_teeth
    n_rollers = int(round(wrap_deg / step_deg)) + 1
    roller_h = min(sprocket_width, roller_d)
    z0 = (sprocket_width - roller_h) / 2.0
    out = {"sprocket": sprocket}
    # roller_sprocket_2d() centres a tooth GAP at every k * step_deg (k
    # integer); snapping the run to the same integer lattice (rather than a
    # -wrap_deg/2 offset that generally isn't a multiple of step_deg) is what
    # actually seats every roller in a gap instead of on a tooth.
    k0 = -(n_rollers // 2)
    for i in range(n_rollers):
        ang = math.radians((k0 + i) * step_deg)
        c = (rp * math.cos(ang), rp * math.sin(ang), z0 + roller_h / 2.0)
        outer = cyl(roller_d / 2.0, roller_h, center=c, sections=48)
        bore = cyl(max(0.6, roller_d / 2.0 - 1.2), roller_h + 2.0,
                  center=(c[0], c[1], c[2]), sections=48)
        out["roller_%02d" % i] = sub(outer, bore)
    return out


__all__ = (
    "drag_chain_link",
    "drag_chain",
    "roller_chain_link",
    "roller_chain",
)
