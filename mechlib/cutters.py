"""Mechanical cutter solids and print-aware pocket features.

``slot_neg`` is an obround adjustment slot. ``slot_cutter`` is an FDM-ready
rectangular slot with square-corner and elephant-foot relief.
"""

import math

import numpy as np
import shapely.geometry as sg
import trimesh
import trimesh.transformations as tf

from .meshutil import extrude_poly_z, inter, orient, sub, uni
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


def slot_cutter(w, t, z0, z1, cx=0.0, cy=0.0, foot_z=0.0,
                dogbone_r=0.6, foot_relief=0.3, eps=0.5):
    """Build an FDM-ready rectangular slot with dog-bones and foot relief.

    Unlike the obround ``slot_neg``, this preserves a rectangular blade seat.
    Returns a list of cutter volumes.
    origin: torque-lever build.py:168
    """
    from shapely.geometry import Point, box
    from shapely.ops import unary_union

    from .meshutil import extrude_snapped

    rect = box(cx - w / 2, cy - t / 2, cx + w / 2, cy + t / 2)
    bones = [Point(cx + sx * w / 2, cy + sy * t / 2).buffer(dogbone_r, resolution=16)
             for sx in (-1, 1) for sy in (-1, 1)]
    outline = unary_union([rect] + bones)
    cut = extrude_snapped(outline, z0, z1)
    relief = extrude_snapped(outline.buffer(foot_relief, join_style=2),
                             foot_z - eps, foot_z + foot_relief)
    return cut + relief


def lobe_cavity_polys(section2d, wall=1.2, rib_w=1.6, n_rib=0):
    """Return hollow lobe cores after optional internal rib crosses.

    origin: tripod lighten_legs.py:56
    """
    from shapely.geometry import box
    from shapely.ops import unary_union

    cutters = []
    for poly in section2d.polygons_full:
        core = poly.buffer(-wall)
        if core.is_empty or core.area < 4:
            continue
        cx, cy = poly.centroid.x, poly.centroid.y
        minx, miny, maxx, maxy = poly.bounds
        ribs = []
        if n_rib >= 1:
            ribs.append(box(cx - rib_w / 2, miny - 1, cx + rib_w / 2, maxy + 1))
        if n_rib >= 2:
            ribs.append(box(minx - 1, cy - rib_w / 2, maxx + 1, cy + rib_w / 2))
        cut = core.difference(unary_union(ribs)) if ribs else core
        if not cut.is_empty:
            cutters.append(cut)
    return cutters


def tapered_cavity(g, zlo, height, taper_h=11.0, taper_step=0.6):
    """Build a cavity whose roof steps closed at about 45 degrees.

    Returns a list of watertight cutter slabs.
    origin: tripod lighten_legs.py:76
    """
    if taper_h <= 0:
        prism = trimesh.creation.extrude_polygon(g, height)
        prism.apply_translation([0, 0, zlo])
        return [prism]
    main_h = max(height - taper_h, 1.0)
    parts = []
    base = trimesh.creation.extrude_polygon(g, main_h)
    base.apply_translation([0, 0, zlo])
    parts.append(base)
    z = zlo + main_h
    k = 1
    while z < zlo + height:
        gk = g.buffer(-taper_step * k)
        if gk.is_empty or gk.area < 2:
            break
        for piece in (gk.geoms if gk.geom_type == "MultiPolygon" else [gk]):
            if piece.area < 2:
                continue
            slab = trimesh.creation.extrude_polygon(piece, taper_step)
            slab.apply_translation([0, 0, z])
            parts.append(slab)
        z += taper_step
        k += 1
    return parts


def u_channel_between(p0, p1, channel_w, z_floor, body_h):
    """Build an open-top rounded U cutter between arbitrary XY points.

    Returns the round floor bore and open-top slot as separate cutter meshes.
    origin: jumper-wire-sockets src/build.py:1386
    """
    x0, y0 = p0
    x1, y1 = p1
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    angle = math.atan2(dy, dx)
    r = channel_w / 2
    z_axis = z_floor + r

    bore = cyl(r, length + 0.4)
    bore.apply_transform(tf.rotation_matrix(math.pi / 2, [0, 1, 0]))
    bore.apply_transform(tf.rotation_matrix(angle, [0, 0, 1]))
    bore.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, z_axis])

    slot_h = body_h - z_axis + 0.5
    slot = boxc((length + 0.4, channel_w, slot_h))
    slot.apply_transform(tf.rotation_matrix(angle, [0, 0, 1]))
    slot.apply_translation(
        [(x0 + x1) / 2, (y0 + y1) / 2, z_axis + slot_h / 2]
    )
    return [bore, slot]


def revolved_gable_cavity(r_in, r_out, z0, h, roof_angle=45.0, sections=128):
    """Build a revolved annular cavity with a self-supporting gable roof.

    This generalizes the shower-head chamber while retaining its roof idea.
    origin: massage-shower-head build.py:117
    """
    half = (r_out - r_in) / 2.0
    r_mid = (r_in + r_out) / 2.0
    z_peak = z0 + h
    z_eave = z_peak - half * math.tan(math.radians(roof_angle))
    prof = np.array([
        [r_in, z0],
        [r_out, z0],
        [r_out, z_eave],
        [r_mid, z_peak],
        [r_in, z_eave],
        [r_in, z0],
    ])
    return trimesh.creation.revolve(prof, sections=sections)


# AS568/ISO 3601 standard O-ring cross-sections in mm. Documented here as a
# reference convenience only -- ``oring_groove`` never looks values up in
# this tuple, every groove dimension below is derived from whatever ``cs``
# is passed, so this is not a hand-maintained size table.
AS568_CS_MM = (1.78, 2.62, 3.53, 5.33, 6.99)


def oring_groove(bore_d=None, face_pcd=None, cs=2.62, squeeze=0.20, fill=0.78,
                 mode="face", width=None, depth=None, z0=0.0, sections=128):
    """Build an O-ring gland cutter sized from AS568/ISO 3601 gland relations.

    ``cs`` is the O-ring cross-section diameter (AS568 standard sizes are
    listed in ``AS568_CS_MM`` for reference; ``cs`` itself stays a free float
    and every dimension below is derived from it). ``squeeze`` is the target
    radial compression fraction of the cross-section (0.20 = 20%, the usual
    static-seal figure) and must stay in [0.05, 0.35] -- looser leaks, tighter
    overstresses the rubber and the print. The groove DEPTH follows the
    standard gland relation ``depth = cs * (1 - squeeze)``.

    The groove WIDTH is sized so the compressed ring fills ``fill`` (default
    78%) of the groove's cross-sectional area, leaving the remainder as fill
    allowance for the ring to bulge into rather than being hydraulically
    locked and extruded out of the joint under pressure. ``fill`` must stay
    in [0.70, 0.85]: this is the single most useful check the function makes,
    since eyeballing gland fill is the most common O-ring mistake (too tight
    and the ring can't compress or blows out; too loose and it rolls in the
    groove and never seals). Pass explicit ``width``/``depth`` to override
    the derived values (e.g. to match a hand gland chart); the achieved
    squeeze and fill are recomputed from whatever you pass and validated
    against the same bands, so a bad override still raises.

    ``mode="face"`` cuts an annular trench into a flat face at pitch diameter
    ``face_pcd`` for an axial (flange) seal; the face sits at ``z=z0`` and
    the groove opens downward into the material below it. ``mode="bore"``
    cuts a groove straddling ``bore_d`` (radial span ``bore_d/2 - depth`` to
    ``bore_d/2 + depth``, centered at ``z0``) for a radial (piston/shaft)
    seal -- the SAME cutter works whether you subtract it from a shaft OD or
    a bore ID, because whichever half of the straddle falls outside the
    solid is a no-op subtraction, so you never need to say which surface you
    are sealing.

    FDM note: an O-ring seals against layer lines, so print with the groove
    OPENING UP -- the groove floor and side walls should be finished by the
    nozzle's flat top layers, never by a vertical wall that crosses every
    layer line. For ``mode="face"`` that means printing the part flat with
    the sealing face as the top of the print. For ``mode="bore"`` on a
    shaft, print with the shaft axis vertical so the groove is a
    flat-bottomed annular pocket cut into the top layers, not a horizontal
    groove running across every layer line.
    Units are mm and degrees.
    """
    if cs <= 0:
        raise ValueError("oring_groove(): cs must be positive")
    if mode not in ("face", "bore"):
        raise ValueError("oring_groove(): mode must be 'face' or 'bore'")
    if mode == "face" and (face_pcd is None or face_pcd <= 0):
        raise ValueError("oring_groove(): mode='face' requires a positive face_pcd")
    if mode == "bore" and (bore_d is None or bore_d <= 0):
        raise ValueError("oring_groove(): mode='bore' requires a positive bore_d")
    if not 0.05 <= squeeze <= 0.35:
        raise ValueError(
            "oring_groove(): squeeze must be in [0.05, 0.35] (static-seal practice)")
    if not 0.70 <= fill <= 0.85:
        raise ValueError(
            "oring_groove(): fill target must be in [0.70, 0.85] gland-fill band")

    depth = cs * (1.0 - squeeze) if depth is None else float(depth)
    if depth <= 0 or depth >= cs:
        raise ValueError("oring_groove(): depth must be between 0 and cs")

    ring_area = math.pi * (cs / 2.0) ** 2
    width = ring_area / (fill * depth) if width is None else float(width)
    if width <= 0:
        raise ValueError("oring_groove(): width must be positive")

    achieved_fill = ring_area / (width * depth)
    if not 0.70 - 1e-6 <= achieved_fill <= 0.85 + 1e-6:
        raise ValueError(
            "oring_groove(): achieved gland fill %.1f%% outside the 70-85%% "
            "band; adjust width/depth" % (achieved_fill * 100.0))
    achieved_squeeze = 1.0 - depth / cs
    if not 0.05 - 1e-6 <= achieved_squeeze <= 0.35 + 1e-6:
        raise ValueError(
            "oring_groove(): achieved squeeze %.1f%% outside the 5-35%% band"
            % (achieved_squeeze * 100.0))

    if mode == "face":
        inner_r = face_pcd / 2.0 - width / 2.0
        if inner_r <= 0:
            raise ValueError("oring_groove(): width too large for face_pcd")
        outer_r = face_pcd / 2.0 + width / 2.0
        m = trimesh.creation.annulus(inner_r, outer_r, depth, sections=sections)
        m.apply_translation((0.0, 0.0, z0 - depth / 2.0))
    else:
        if bore_d / 2.0 - depth <= 0.05:
            raise ValueError("oring_groove(): depth too large relative to bore_d")
        inner_r = bore_d / 2.0 - depth
        outer_r = bore_d / 2.0 + depth
        m = trimesh.creation.annulus(inner_r, outer_r, width, sections=sections)
        m.apply_translation((0.0, 0.0, z0))

    m.metadata.update({
        "mode": mode,
        "cs": cs,
        "groove_width": width,
        "groove_depth": depth,
        "squeeze_pct": achieved_squeeze * 100.0,
        "gland_fill_pct": achieved_fill * 100.0,
    })
    return m


def labyrinth_seal(shaft_d=8.0, teeth=4, tooth_t=1.2, gap=0.3, pitch=None,
                   fin_h=2.0, hub_wall=1.6, stator_wall=1.6, end_margin=None,
                   sections=96):
    """Build an interleaved-comb labyrinth seal: a non-contact rotary seal.

    This is the correct PRINTABLE seal for a rotating joint: it needs no
    elastomer and has no rubbing surface, so it never wears and never needs
    replacing, at the cost of throttling rather than fully stopping flow (use
    it for dust, splash, or low-pressure gas, not for holding real pressure).

    The rotor is a hub (through-bore ``shaft_d`` for mounting on a shaft)
    carrying ``teeth`` outward-facing fin rings spaced ``pitch`` apart along
    the axis. The stator is a surrounding sleeve carrying ``teeth - 1``
    inward-facing counter-teeth, each centered in the axial gap BETWEEN two
    rotor fins -- offset by half a pitch, so the two combs interleave.
    Flow crossing the seal must thread past every rotor fin tip (radial
    clearance ``gap`` to the stator sleeve bore) and every stator tooth tip
    (radial clearance ``gap`` to the rotor hub) in turn: ``2 * teeth - 1``
    throttling stages, reported as ``metadata["stages"]``. Neither part ever
    touches the other -- both the radial and axial clearance are ``gap``.
    Assemble by sliding the rotor into the stator along the shared axis; the
    rotor's Z=0 is its left end face and the stator spans the same Z range.

    Print both parts with the axis vertical: every radial face then prints
    as a horizontal top or bottom layer instead of a bridge or overhang, and
    the ``gap`` clearance (keep it in the usual 0.25-0.35 mm FDM running-
    clearance range) prints open rather than fusing shut.
    Units are mm and degrees.
    """
    if teeth < 2:
        raise ValueError("labyrinth_seal(): teeth must be >= 2")
    if shaft_d <= 0 or tooth_t <= 0 or gap <= 0 or fin_h <= 0:
        raise ValueError(
            "labyrinth_seal(): shaft_d, tooth_t, gap, and fin_h must be positive")
    if hub_wall <= 0 or stator_wall <= 0:
        raise ValueError("labyrinth_seal(): hub_wall and stator_wall must be positive")
    if pitch is None:
        pitch = 2.0 * tooth_t + 4.0 * gap
    if pitch < 2.0 * tooth_t + 2.0 * gap:
        raise ValueError(
            "labyrinth_seal(): pitch too small for tooth_t/gap; need "
            "pitch >= 2*tooth_t + 2*gap to keep the rotor fins and stator "
            "teeth from touching axially")
    if end_margin is None:
        end_margin = tooth_t
    if end_margin <= 0:
        raise ValueError("labyrinth_seal(): end_margin must be positive")

    r0 = shaft_d / 2.0 + hub_wall
    fin_tip_r = r0 + fin_h
    r_big = fin_tip_r + gap
    r_small = r0 + gap
    stator_od = r_big + stator_wall

    total_len = 2.0 * end_margin + tooth_t + (teeth - 1) * pitch
    fin_centers = [end_margin + tooth_t / 2.0 + i * pitch for i in range(teeth)]
    stator_tooth_centers = [fc + pitch / 2.0 for fc in fin_centers[:-1]]

    hub = cyl(r0, total_len)
    hub.apply_translation((0.0, 0.0, total_len / 2.0))
    fins = []
    for zc in fin_centers:
        fin = trimesh.creation.annulus(r0, fin_tip_r, tooth_t, sections=sections)
        fin.apply_translation((0.0, 0.0, zc))
        fins.append(fin)
    rotor = uni([hub] + fins)
    bore = cyl(shaft_d / 2.0, total_len + 2.0)
    bore.apply_translation((0.0, 0.0, total_len / 2.0))
    rotor = sub(rotor, bore)

    sleeve = trimesh.creation.annulus(r_big, stator_od, total_len, sections=sections)
    sleeve.apply_translation((0.0, 0.0, total_len / 2.0))
    stator_teeth = []
    for zc in stator_tooth_centers:
        tooth = trimesh.creation.annulus(r_small, r_big, tooth_t, sections=sections)
        tooth.apply_translation((0.0, 0.0, zc))
        stator_teeth.append(tooth)
    stator = uni([sleeve] + stator_teeth)

    metadata = {
        "teeth": teeth,
        "stages": teeth + len(stator_tooth_centers),
        "radial_gap": gap,
        "pitch": pitch,
        "total_len": total_len,
        "fin_tip_r": fin_tip_r,
        "rotor_root_r": r0,
    }
    rotor.metadata.update(metadata)
    stator.metadata.update(metadata)
    return {"rotor": rotor, "stator": stator}


def gasket_channel(path=None, width=3.0, depth=1.5, z0=0.0, join_style=1):
    """Build a cord-stock gasket groove cutter following an arbitrary closed path.

    ``path`` is a Shapely ``LinearRing``/``LineString``/``Polygon`` or a
    sequence of ``(x, y)`` points describing a closed loop -- a rectangular
    or kidney-shaped lid perimeter, for example; it is closed automatically
    if the first and last points differ. The cutter is the band of points
    within ``width / 2`` of that path, extruded ``depth`` mm downward from a
    flat face at ``z=z0``, so subtracting it from a lid or box rim leaves a
    gasket channel that follows the outline exactly, for shapes an O-ring
    can't reach.

    This generalizes ``oring_groove(mode="face")`` to non-circular paths; for
    an actual circular seal prefer ``oring_groove``, since it derives width
    and depth from the O-ring cross-section with squeeze/fill validation --
    cord stock is sold by length rather than a matched cross-section
    standard, so there is no equivalent fill check here. Size ``width`` and
    ``depth`` from the cord diameter yourself using the same gland logic:
    roughly ``depth = cord_d * 0.80`` for a 20% squeeze and
    ``width = cord_d * 1.5`` for a comparable fill allowance.

    ``join_style`` follows Shapely's buffer convention (1=round, 2=mitre,
    3=bevel) for how path corners render in the channel outline; round is
    the printable default and avoids a sharp inner corner that would trap
    the cord.

    Print with the sealing face (containing the channel opening) as the top
    of the print, same as ``oring_groove``, so the groove floor and walls
    are finished by flat top layers rather than a vertical wall.
    Units are mm and degrees.
    """
    if path is None:
        raise ValueError("gasket_channel(): path is required")
    if width <= 0 or depth <= 0:
        raise ValueError("gasket_channel(): width and depth must be positive")

    if isinstance(path, sg.Polygon):
        ring = path.exterior
    elif isinstance(path, (sg.LinearRing, sg.LineString)):
        ring = path
    else:
        pts = [tuple(p) for p in path]
        if len(pts) < 3:
            raise ValueError("gasket_channel(): path needs at least 3 points")
        if pts[0] != pts[-1]:
            pts = pts + [pts[0]]
        ring = sg.LinearRing(pts)
    if ring.is_empty or ring.length <= 0:
        raise ValueError("gasket_channel(): degenerate path")

    band = ring.buffer(width / 2.0, join_style=join_style)
    if band.is_empty:
        raise ValueError("gasket_channel(): buffered path produced no area")
    m = extrude_poly_z(band, z0 - depth, z0)
    if m is None:
        raise ValueError("gasket_channel(): failed to extrude the channel")
    m.metadata.update({
        "groove_width": width,
        "groove_depth": depth,
        "path_length": float(ring.length),
    })
    return m
