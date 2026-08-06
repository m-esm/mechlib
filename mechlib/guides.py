"""Project-agnostic prismatic guideway and telescoping-stage generators."""

import math

import numpy as np
import shapely.geometry as sg
import trimesh
from shapely.ops import unary_union

from .cutters import teardrop
from .meshutil import extrude_poly_z, from_manifold, sub, to_manifold, uni
from .prim import boxc

_PROFILES = ("dovetail", "vee", "tslot")


def _solidify(mesh):
    """Return a watertight copy of an extruded mesh."""
    if mesh is None:
        raise ValueError("extrusion produced no geometry")
    if not mesh.is_watertight:
        mesh = from_manifold(to_manifold(mesh))
    return mesh


def _extrude(poly2d, z0, z1):
    """Extrude a shapely polygon between two Z planes into a solid."""
    return _solidify(extrude_poly_z(poly2d, z0, z1))


def _ccw(coords):
    """Return a ring's coordinates in counter-clockwise order."""
    pts = list(coords)
    if pts and pts[0] == pts[-1]:
        pts = pts[:-1]
    area = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        area += x0 * y1 - x1 * y0
    return pts if area > 0.0 else pts[::-1]


def _reflex_points(poly):
    """Return the concave (reflex) vertices of a polygon's outer ring.

    These are the inside corners of the solid: the places where an FDM first
    layer's elephant foot and over-extrusion pile up and jam a sliding fit, so
    they are exactly where a relief groove belongs.
    """
    ring = _ccw(poly.exterior.coords)
    n = len(ring)
    out = []
    for i in range(n):
        x0, y0 = ring[i - 1]
        x1, y1 = ring[i]
        x2, y2 = ring[(i + 1) % n]
        cross = (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1)
        if cross < -1e-9:
            out.append((x1, y1))
    return out


def _relieve(poly, radius, label):
    """Cut a chip-relief circle out of every inside corner of a polygon."""
    if radius <= 0.0:
        return poly
    pts = _reflex_points(poly)
    if not pts:
        return poly
    holes = unary_union([sg.Point(p).buffer(radius, 8) for p in pts])
    out = poly.difference(holes)
    if out.is_empty or out.geom_type != "Polygon":
        raise ValueError(
            "linear_way(): relief_r=%.2f splits the %s section; reduce it"
            % (radius, label))
    return out


def _way_section(profile, section_w, section_h, angle_deg, head_frac):
    """Return the rail cross-section polygon and its +X gib flank segment."""
    half = section_w / 2.0
    undercut = section_h / math.tan(math.radians(angle_deg))
    if profile == "dovetail":
        foot = half - undercut
        if foot < 1.5:
            raise ValueError(
                "linear_way(): dovetail foot is %.2f mm; widen section_w or "
                "raise angle_deg" % foot)
        poly = sg.Polygon([(-foot, 0.0), (foot, 0.0),
                           (half, section_h), (-half, section_h)])
        flank = ((foot, 0.0), (half, section_h))
    elif profile == "vee":
        top = half - undercut
        if top < 1.0:
            raise ValueError(
                "linear_way(): vee crest is %.2f mm wide; widen section_w or "
                "raise angle_deg" % (2.0 * top))
        poly = sg.Polygon([(-half, 0.0), (half, 0.0),
                           (top, section_h), (-top, section_h)])
        flank = ((half, 0.0), (top, section_h))
    else:
        stem = half - undercut
        if stem < 1.5:
            raise ValueError(
                "linear_way(): tslot stem half-width is %.2f mm; widen "
                "section_w or raise angle_deg" % stem)
        head_h = section_h * head_frac
        stem_h = section_h - head_h
        if head_h < 1.2 or stem_h < 1.2:
            raise ValueError(
                "linear_way(): tslot head and stem must each be at least "
                "1.2 mm tall; adjust section_h or head_frac")
        poly = sg.Polygon([(-stem, 0.0), (stem, 0.0), (stem, stem_h),
                           (half, stem_h), (half, section_h),
                           (-half, section_h), (-half, stem_h),
                           (-stem, stem_h)])
        flank = ((half, stem_h), (half, section_h))
    return poly, flank, undercut


def _poly_width_at(poly, y):
    """Return the total X extent of a polygon on a horizontal line."""
    lo, hi = poly.bounds[0] - 1.0, poly.bounds[2] + 1.0
    cut = poly.intersection(sg.LineString([(lo, y), (hi, y)]))
    return float(cut.length) if not cut.is_empty else 0.0


def _taper_prism(anchor, nvec, vvec, q0, t0, t1, z0, z1, p0, p1):
    """Build a prism on a flank whose thickness ramps linearly with world Z.

    ``anchor`` is the flank start in XY, ``vvec`` the unit vector along the
    flank, ``nvec`` the unit outward flank normal. The solid covers
    ``p0..p1`` along the flank, ``z0..z1`` in Z, and ``q0`` to ``q0+t(z)``
    along the normal with ``t`` running from ``t0`` at ``z0`` to ``t1`` at
    ``z1``. The varying dimension lives in the extruded 2D profile, so the
    result is an exact prism rather than a sheared approximation.
    """
    span = z1 - z0
    poly = sg.Polygon([(q0, 0.0), (q0 + t0, 0.0),
                       (q0 + t1, -span), (q0, -span)])
    mesh = trimesh.creation.extrude_polygon(poly, p1 - p0)
    matrix = np.eye(4)
    matrix[:3, 0] = (nvec[0], nvec[1], 0.0)
    matrix[:3, 1] = (0.0, 0.0, -1.0)
    matrix[:3, 2] = (vvec[0], vvec[1], 0.0)
    matrix[:3, 3] = (anchor[0] + vvec[0] * p0,
                     anchor[1] + vvec[1] * p0, z0)
    mesh.apply_transform(matrix)
    return _solidify(mesh)


def _end_ramp(x0, x1, height, z_at, top):
    """Build a 45-degree end-stop ramp on the rail base plate.

    The ramp rises toward the rail end so that, printed standing, every layer
    is supported by the one below it; the carriage runs onto the 45-degree
    face and wedges to a halt.
    """
    if top:
        tri = sg.Polygon([(0.0, 0.0), (0.0, height), (height, height)])
    else:
        tri = sg.Polygon([(0.0, 0.0), (0.0, height), (height, 0.0)])
    # Sink the ramp into the base plate so the union is a solid overlap
    # rather than a zero-thickness face contact.
    tri = unary_union([tri, sg.box(-0.5, 0.0, 0.0, height)]).buffer(0)
    mesh = _solidify(trimesh.creation.extrude_polygon(tri, x1 - x0))
    matrix = np.eye(4)
    matrix[:3, 0] = (0.0, 1.0, 0.0)
    matrix[:3, 1] = (0.0, 0.0, 1.0)
    matrix[:3, 2] = (1.0, 0.0, 0.0)
    matrix[:3, 3] = (x0, 0.0, z_at)
    mesh.apply_transform(matrix)
    return mesh


def linear_way(profile="dovetail", length=70.0, section_w=26.0,
               section_h=6.0, angle_deg=55.0, clear=0.25, gib=True,
               gib_t=2.0, gib_taper=0.8, wall=4.5, roof=3.0,
               carriage_len=28.0, base_t=3.0, base_ext=6.0, stop_h=3.0,
               relief_r=0.6, head_frac=0.4, mount_d=3.4, mount_pitch=25.0,
               gib_screw=False, gib_screw_d=3.0, gib_tab_t=5.0):
    """Build a prismatic linear guideway: rail, carriage, and adjusting gib.

    A cross-section chosen by ``profile`` (``"dovetail"``, ``"vee"``, or
    ``"tslot"``) is swept along +Z for ``length``. The rail stands on a base
    plate ``base_t`` thick that reaches ``base_ext`` beyond the section on each
    side, so the sliding datum plane is z-parallel at y=0 and the section
    occupies y=0..``section_h``. ``angle_deg`` is the flank angle from that
    datum and sets the undercut ``section_h/tan(angle_deg)`` for all three
    profiles: the dovetail widens upward and captures the carriage, the vee
    narrows upward and does not (it needs gravity or an external hold-down),
    and the tslot captures on a square head of height ``head_frac*section_h``.
    The carriage is the same section opened out by the running clearance
    ``clear`` on every sliding face, wrapped in ``wall``-thick sides and a
    ``roof``-thick top, and is posed centred on the rail. Every inside corner
    of both parts carries a ``relief_r`` chip-relief groove, because a sharp
    inside corner collects the first-layer elephant foot and jams the slide.
    45-degree end-stop ramps ``stop_h`` tall sit on the base plate at both
    ends of travel; metadata reports the resulting ``travel``.

    With ``gib=True`` the +X flank pocket is opened by an extra
    ``gib_t`` and filled by a tapered gib whose thickness ramps ``gib_taper``
    over the carriage length. Sliding the gib axially takes up
    ``gib_taper/carriage_len`` mm of clearance per mm of slide (metadata
    ``gib_preload_per_mm``); that plain wedge needs no hardware. On ``tslot``
    the gib bears on a vertical face, so it removes lateral play only and the
    vertical capture clearance stays at ``clear``. ``gib_screw=True`` adds a
    tab across the pocket mouth tapped for a ``gib_screw_d`` grub screw
    (printed thread, via ``mechanisms.thread_solid``) that drives the gib.
    ``mount_d`` teardrop through-holes are spaced about ``mount_pitch`` apart
    down both base-plate flanges; pass ``mount_d=0`` to leave the plate blank.

    FDM constraint, stated plainly: print the rail exactly as generated,
    standing on its z=0 end, section in XY, swept along Z. Lying the rail down
    turns the dovetail's undercut faces into 45-60 degree overhangs that need
    support inside the sliding surface, which is the one place support scars
    cannot be tolerated. Standing, every sliding face is a vertical wall and
    the only overhangs are the deliberate 45-degree end ramps and the
    teardrop mount holes. A standing rail has a small footprint, so use a
    brim. The carriage and gib print the same way, standing on an end face.
    Units are mm and degrees.
    """
    if profile not in _PROFILES:
        raise ValueError("linear_way(): profile must be one of %s, got %r"
                         % (", ".join(_PROFILES), profile))
    if length <= 0 or section_w <= 0 or section_h < 1.5:
        raise ValueError("linear_way(): invalid rail section or length")
    if not 25.0 <= angle_deg <= 80.0:
        raise ValueError("linear_way(): angle_deg must lie in 25..80 degrees")
    if clear <= 0.0:
        raise ValueError("linear_way(): running clearance must be positive")
    if clear >= section_h / 2.0:
        raise ValueError(
            "linear_way(): clear=%.2f is not a running clearance next to a "
            "%.2f mm section height" % (clear, section_h))
    if wall < 2.0 or roof < 1.2 or base_t < 1.2 or base_ext < 1.0:
        raise ValueError("linear_way(): wall, roof, or base too thin")
    if carriage_len <= 0 or stop_h < 0:
        raise ValueError("linear_way(): invalid carriage length or stop height")
    if carriage_len + 2.0 * stop_h + 2.0 > length:
        raise ValueError(
            "linear_way(): carriage plus end stops do not fit in length")
    if relief_r < 0 or relief_r > 0.35 * section_h or relief_r > 0.5 * roof:
        raise ValueError("linear_way(): relief_r out of range for this section")
    if not 0.2 <= head_frac <= 0.8:
        raise ValueError("linear_way(): head_frac must lie in 0.2..0.8")
    if mount_d < 0 or mount_d >= base_ext - 1.0:
        raise ValueError("linear_way(): mount_d does not fit the base flange")
    ramp_w = wall - clear - 1.0
    if ramp_w < 0.6:
        raise ValueError(
            "linear_way(): wall=%.2f leaves no room for an end-stop ramp"
            % wall)

    half = section_w / 2.0
    rail2d, flank, undercut = _way_section(profile, section_w, section_h,
                                           angle_deg, head_frac)
    ax, ay = flank[0]
    bx, by = flank[1]
    flank_len = math.hypot(bx - ax, by - ay)
    vvec = ((bx - ax) / flank_len, (by - ay) / flank_len)
    nvec = (vvec[1], -vvec[0])

    if gib:
        if gib_t < 1.2 or gib_taper <= 0.0:
            raise ValueError("linear_way(): gib too thin or untapered")
        if gib_t - gib_taper / 2.0 < 0.8:
            raise ValueError(
                "linear_way(): the thin end of the gib falls below 0.8 mm")
        depth = 2.0 * clear + gib_t + gib_taper / 2.0
        if wall < abs(nvec[0]) * depth + 0.8:
            raise ValueError(
                "linear_way(): the gib pocket would break through a %.2f mm "
                "carriage wall; raise wall or lower gib_t" % wall)
        if gib_taper >= flank_len:
            raise ValueError("linear_way(): gib_taper exceeds the flank")

    # Rail: section plus base plate, relieved at every inside corner.
    base = sg.box(-(half + base_ext), -base_t, half + base_ext, 0.0)
    rail_poly = _relieve(unary_union([base, rail2d]).buffer(0), relief_r,
                         "rail")
    rail = _extrude(rail_poly, 0.0, length)

    if stop_h > 0.0:
        x0 = half + clear + 1.0
        x1 = half + wall
        ramps = [_end_ramp(x0, x1, stop_h, 0.0, False),
                 _end_ramp(-x1, -x0, stop_h, 0.0, False),
                 _end_ramp(x0, x1, stop_h, length - stop_h, True),
                 _end_ramp(-x1, -x0, stop_h, length - stop_h, True)]
        rail = uni([rail] + ramps)

    if mount_d > 0.0:
        inset = max(6.0, stop_h + 3.0)
        span = length - 2.0 * inset
        count = max(2, int(round(span / max(mount_pitch, 1.0))) + 1)
        x_m = half + base_ext / 2.0
        holes = []
        for z_m in np.linspace(inset, length - inset, count):
            for sign in (1.0, -1.0):
                cut = teardrop(mount_d / 2.0, base_t + 4.0, axis="y",
                               up=(0, 0, 1))
                cut.apply_translation((sign * x_m, -base_t / 2.0, float(z_m)))
                holes.append(cut)
        rail = sub(rail, uni(holes))

    # Carriage: a block with the offset section subtracted out of its underside.
    cutter2d = rail2d.buffer(clear, join_style=2, mitre_limit=8.0)
    car_w = section_w + 2.0 * wall
    block = sg.box(-car_w / 2.0, clear, car_w / 2.0,
                   section_h + clear + roof)
    car_poly = block.difference(cutter2d)
    if car_poly.is_empty or car_poly.geom_type != "Polygon":
        raise ValueError("linear_way(): the carriage section is not a "
                         "single body; check clear and wall")
    car_poly = _relieve(car_poly, relief_r, "carriage")
    cz0 = (length - carriage_len) / 2.0
    cz1 = cz0 + carriage_len
    carriage = _extrude(car_poly, cz0, cz1)

    parts = {}
    slope = gib_taper / carriage_len if gib else 0.0
    zmid = (cz0 + cz1) / 2.0
    if gib:
        # The pocket runs past the gib on every free edge by more than one
        # clearance, so the gib's own end faces never set the running fit.
        over = clear + 0.4
        pocket = _taper_prism(
            (ax, ay), nvec, vvec, 0.0,
            2.0 * clear + gib_t + slope * (cz0 - over - zmid),
            2.0 * clear + gib_t + slope * (cz1 + over - zmid),
            cz0 - over, cz1 + over, -over, flank_len + over)
        carriage = sub(carriage, pocket)
        # With a screw abutment the gib stops one clearance short of it, so
        # the screw drives the gib across a real gap instead of the two parts
        # printing fused together.
        gib_z1 = cz1 - (clear if gib_screw else 0.0)
        gib_mesh = _taper_prism(
            (ax, ay), nvec, vvec, clear,
            gib_t - gib_taper / 2.0, gib_t + slope * (gib_z1 - zmid),
            cz0, gib_z1, 0.0, flank_len)
        # An undercut flank pushes the gib's normal partly downward, so trim
        # it back to the carriage's own underside plane at y=clear.
        gib_mesh = sub(gib_mesh, boxc(
            (4.0 * car_w, 4.0 * section_h, length + 4.0),
            center=(0.0, clear - 2.0 * section_h, length / 2.0)))
        if gib_screw:
            if gib_screw_d < 2.0 or gib_tab_t < 2.0:
                raise ValueError("linear_way(): gib screw or tab too small")
            if gib_t < gib_screw_d + 0.8:
                raise ValueError(
                    "linear_way(): a coaxial gib screw of %.1f mm needs "
                    "gib_t >= %.1f mm so the tapped hole keeps 0.4 mm of "
                    "material off the rail clearance line"
                    % (gib_screw_d, gib_screw_d + 0.8))
            # The tab is the carriage's own gib-side cross-section carried
            # past its end face, so it clears the rail by construction and
            # lands squarely over the gib's end.
            tab_poly = car_poly.intersection(
                sg.box(0.0, -length, car_w, section_h + clear + roof + 1.0))
            if tab_poly.is_empty or tab_poly.geom_type != "Polygon":
                raise ValueError("linear_way(): the gib screw tab is not a "
                                 "single body")
            carriage = uni([carriage,
                            _extrude(tab_poly, cz1, cz1 + gib_tab_t)])
            mid_t = clear + gib_t / 2.0
            px = ax + vvec[0] * flank_len / 2.0 + nvec[0] * mid_t
            py = ay + vvec[1] * flank_len / 2.0 + nvec[1] * mid_t
            from .mechanisms import tap
            carriage = tap(carriage, gib_screw_d,
                           (px, py, cz1 - 0.5), gib_tab_t + 1.5,
                           clear=clear, axis="z")
        parts["gib"] = gib_mesh

    parts["rail"] = rail
    parts["carriage"] = carriage

    throat_w = _poly_width_at(cutter2d, clear + 1e-6)
    widest_w = max(_poly_width_at(cutter2d, y) for y in
                   np.linspace(clear + 1e-6, section_h + clear - 1e-6, 41))
    car_span = carriage_len + (gib_tab_t if (gib and gib_screw) else 0.0)
    travel = max(0.0, length - car_span - 2.0 * max(0.0, stop_h - clear))
    meta = {
        "profile": profile,
        "clear": clear,
        "undercut": undercut,
        "travel": travel,
        "throat_w": throat_w,
        "widest_w": widest_w,
        "rail_max_w": section_w,
        "captures": bool(throat_w < widest_w - 0.05),
        "carriage_len": carriage_len,
        "length": length,
    }
    if gib:
        meta["gib_preload_per_mm"] = slope
    for mesh in parts.values():
        mesh.metadata.update(meta)
    return parts


def _sq_frustum(w0, w1, z0, z1):
    """Build a square frustum between two Z planes."""
    h0, h1 = w0 / 2.0, w1 / 2.0
    verts = np.array([
        (-h0, -h0, z0), (h0, -h0, z0), (h0, h0, z0), (-h0, h0, z0),
        (-h1, -h1, z1), (h1, -h1, z1), (h1, h1, z1), (-h1, h1, z1)])
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7)]
    for i in range(4):
        j = (i + 1) % 4
        faces.append((i, j, j + 4))
        faces.append((i, j + 4, i + 4))
    return trimesh.Trimesh(vertices=verts, faces=np.array(faces),
                           process=False)


def telescoping_stage(sections=3, length=60.0, outer_w=36.0, wall=2.0,
                      clear=0.3, lip_t=1.2, collar_h=4.0, extend=0.6,
                      min_bore=4.0):
    """Build a nested telescoping stage with anti-pullout stops at each joint.

    ``sections`` square tubes of ``length`` nest inside one another along +Z.
    The outermost tube is ``outer_w`` across with ``wall``-thick sides; each
    inner tube's bore drops by ``2*(clear + lip_t + wall)``, so every joint
    runs on a ``clear`` sliding fit. Square sections also key the stage
    against rotation, which round tubes do not.

    Each joint is stopped both ways. The parent carries a 45-degree inward
    lip at its top end that necks the bore down by ``lip_t`` per side; the
    child carries a ``collar_h``-tall external collar at its bottom end sized
    to the parent bore less ``clear``. Extending drives the collar into the
    lip and the stage cannot come apart, because the collar is ``lip_t-clear``
    wider per side than the lip opening (``lip_t`` must exceed ``clear`` or
    the stop does nothing). The child's plain body is ``clear`` narrower per
    side than that same opening, so it passes freely. Assembly is from the
    parent's open bottom end.

    Free travel per joint is ``length - lip_t - collar_h``; metadata carries
    ``travel_per_joint``, ``retracted_length`` (one ``length``) and
    ``extended_length``. ``extend`` is the fraction of travel each joint is
    posed at, 0 fully retracted and 1 at the stops. Returns
    ``{"section_0": ..., "section_1": ...}`` with section 0 outermost.

    Print every tube standing on its open bottom end. The retaining lip is a
    45-degree internal taper and the collar steps inward going up, so nothing
    overhangs and no support goes anywhere near a sliding face. Units are mm.
    """
    sections = int(round(sections))
    if sections < 2:
        raise ValueError("telescoping_stage(): needs at least 2 sections")
    if length <= 0 or outer_w <= 0 or wall < 1.2 or collar_h <= 0:
        raise ValueError("telescoping_stage(): invalid tube dimensions")
    if clear <= 0.0:
        raise ValueError("telescoping_stage(): clearance must be positive")
    if lip_t <= clear:
        raise ValueError(
            "telescoping_stage(): lip_t=%.2f must exceed clear=%.2f or the "
            "anti-pullout stop passes straight through" % (lip_t, clear))
    if not 0.0 <= extend <= 1.0:
        raise ValueError("telescoping_stage(): extend must lie in 0..1")
    travel = length - lip_t - collar_h
    if travel <= 0.0:
        raise ValueError(
            "telescoping_stage(): lip and collar consume the whole length")

    step = 2.0 * (clear + lip_t + wall)
    bore0 = outer_w - 2.0 * wall
    bores = [bore0 - i * step for i in range(sections)]
    if bores[-1] < min_bore:
        raise ValueError(
            "telescoping_stage(): section %d bore falls to %.2f mm; widen "
            "outer_w or drop a section" % (sections - 1, bores[-1]))

    parts = {}
    for i in range(sections):
        bore = bores[i]
        body_w = bore + 2.0 * wall
        solids = [boxc((body_w, body_w, length), center=(0, 0, length / 2.0))]
        if i > 0:
            collar_w = bores[i - 1] - 2.0 * clear
            solids.append(boxc((collar_w, collar_w, collar_h),
                               center=(0, 0, collar_h / 2.0)))
        tube = uni(solids) if len(solids) > 1 else solids[0]

        if i < sections - 1:
            top = length - lip_t
            cuts = [boxc((bore, bore, top + 2.0),
                         center=(0, 0, (top - 2.0) / 2.0)),
                    _sq_frustum(bore, bore - 2.0 * lip_t, top - 0.01, length),
                    boxc((bore - 2.0 * lip_t, bore - 2.0 * lip_t, 2.0),
                         center=(0, 0, length + 0.99))]
        else:
            cuts = [boxc((bore, bore, length + 4.0),
                         center=(0, 0, length / 2.0))]
        tube = sub(tube, uni(cuts))
        tube.apply_translation((0.0, 0.0, i * extend * travel))
        parts["section_%d" % i] = tube

    meta = {
        "sections": sections,
        "clear": clear,
        "travel_per_joint": travel,
        "retracted_length": length,
        "extended_length": length + (sections - 1) * travel,
        "lip_opening": bore0 - 2.0 * lip_t,
        "collar_w": bore0 - 2.0 * clear,
        "extend": extend,
    }
    for mesh in parts.values():
        mesh.metadata.update(meta)
    return parts


__all__ = (
    "linear_way",
    "telescoping_stage",
)
