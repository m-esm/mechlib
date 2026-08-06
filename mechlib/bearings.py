"""Project-agnostic printed journal, thrust, and rolling-element bearings."""

import math

import numpy as np
import shapely.geometry as sg
import trimesh
import trimesh.transformations as tf
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

from .meshutil import extrude_poly_z, sub, uni
from .prim import boxc, cyl, frustum, sector2d

_MIN_WALL = 0.8
_SQRT_HALF = math.sqrt(0.5)


def _revolve(poly, sections=96):
    """Revolve a closed (r, z) profile polygon about the Z axis."""
    ring = orient(poly, 1.0)
    return trimesh.creation.revolve(np.asarray(ring.exterior.coords),
                                    sections=int(sections))


def _ring_poly(r_in, r_out, sections=96):
    """Return an annular shapely polygon centred on the origin."""
    quad = max(4, int(sections) // 4)
    outer = sg.Point(0.0, 0.0).buffer(r_out, resolution=quad)
    if r_in <= 0.0:
        return outer
    return outer.difference(sg.Point(0.0, 0.0).buffer(r_in, resolution=quad))


def _truncated_ball(radius, flat_z, sections=96):
    """Return a sphere centred at the origin with its bottom cap removed.

    ``flat_z`` is the cut height relative to the centre (negative). The cut at
    ``-radius/sqrt(2)`` leaves every remaining surface at or above 45 degrees
    from the build plate, which is what makes the ball print in place.
    """
    quad = max(4, int(sections) // 4)
    circle = sg.Point(0.0, 0.0).buffer(radius, resolution=quad)
    keep = sg.box(-radius * 2.0, flat_z, radius * 2.0, radius * 2.0)
    half = circle.intersection(keep).intersection(
        sg.box(0.0, -radius * 2.0, radius * 2.0, radius * 2.0))
    return _revolve(half, sections=sections)


def plain_bushing(bore_d=8.0, wall=2.0, length=12.0, clear=0.25,
                  flange=True, flange_d=None, flange_t=1.6,
                  relief_grooves=4, groove_style="axial", groove_w=1.2,
                  groove_depth=0.6, lead_in=0.5, sections=96):
    """Build a sleeve or flanged plain bushing (printed journal bearing).

    The workhorse printed bearing: a plastic sleeve that a steel shaft turns
    directly inside. The bore is printed at ``bore_d + clear`` so the running
    fit is explicit rather than left to slicer compensation, and both ends get
    a 45 degree ``lead_in`` chamfer so the shaft starts square. ``wall`` sets
    the sleeve thickness, so the outside diameter is ``bore_d + clear +
    2*wall``. An optional flange of diameter ``flange_d`` and thickness
    ``flange_t`` sits at the z=0 end and carries the light axial load a plain
    journal can take. ``relief_grooves`` cuts grease and debris channels into
    the bore: ``groove_style='axial'`` runs them the full length (the FDM
    friendly choice, every wall stays vertical), ``'circumferential'`` cuts
    45 degree V rings at even spacing. The bushing sits with its flange face on
    z=0 and its axis along +Z; print it in exactly that orientation, flange
    down, no supports, so the layer lines run around the bore rather than along
    it. Units are mm and degrees.
    """
    if bore_d <= 0.0:
        raise ValueError("plain_bushing(): bore_d must be positive")
    if wall < _MIN_WALL:
        raise ValueError("plain_bushing(): wall must be at least %.1f mm"
                         % _MIN_WALL)
    if clear < 0.0:
        raise ValueError("plain_bushing(): clear must be non-negative")
    if length <= 0.0:
        raise ValueError("plain_bushing(): length must be positive")
    if groove_style not in ("axial", "circumferential"):
        raise ValueError("plain_bushing(): groove_style must be 'axial' or "
                         "'circumferential'")
    relief_grooves = int(round(relief_grooves))
    if relief_grooves < 0:
        raise ValueError("plain_bushing(): relief_grooves must be "
                         "non-negative")
    if lead_in < 0.0 or 2.0 * lead_in >= length:
        raise ValueError("plain_bushing(): lead_in must fit twice in length")

    bore_r = (bore_d + clear) / 2.0
    outer_d = bore_d + clear + 2.0 * wall
    outer_r = outer_d / 2.0
    if flange_d is None:
        flange_d = outer_d + 2.0 * max(2.0, wall)
    if flange:
        if flange_t < _MIN_WALL:
            raise ValueError("plain_bushing(): flange_t must be at least "
                             "%.1f mm" % _MIN_WALL)
        if flange_d <= outer_d:
            raise ValueError("plain_bushing(): flange_d must exceed the "
                             "bushing outside diameter")
        if flange_t >= length:
            raise ValueError("plain_bushing(): flange_t must be shorter than "
                             "the bushing length")

    if relief_grooves:
        if groove_w <= 0.0 or groove_depth <= 0.0:
            raise ValueError("plain_bushing(): groove_w and groove_depth must "
                             "be positive")
        if groove_depth > wall - _MIN_WALL:
            raise ValueError("plain_bushing(): groove_depth leaves less than "
                             "%.1f mm of wall" % _MIN_WALL)
        if groove_style == "axial":
            if groove_depth < groove_w / 2.0:
                raise ValueError("plain_bushing(): an axial groove needs "
                                 "groove_depth >= groove_w/2")
            spacing = (2.0 * bore_r * math.sin(math.pi / relief_grooves)
                       if relief_grooves > 1 else 2.0 * math.pi * bore_r)
            land = spacing - groove_w
            if land < _MIN_WALL:
                raise ValueError("plain_bushing(): %d axial grooves leave only "
                                 "%.2f mm of land between them"
                                 % (relief_grooves, land))
        else:
            if groove_w < 2.0 * groove_depth:
                raise ValueError("plain_bushing(): a circumferential groove "
                                 "needs groove_w >= 2*groove_depth to keep "
                                 "its flanks at 45 degrees")
            if (relief_grooves + 1) * groove_w >= length:
                raise ValueError("plain_bushing(): %d circumferential grooves "
                                 "do not fit in the length" % relief_grooves)

    body = cyl(outer_r, length, center=(0, 0, length / 2.0),
               sections=int(sections))
    if flange:
        body = uni([body, cyl(flange_d / 2.0, flange_t,
                              center=(0, 0, flange_t / 2.0),
                              sections=int(sections))])

    cutters = [cyl(bore_r, length + 4.0, center=(0, 0, length / 2.0),
                   sections=int(sections))]
    if lead_in > 0.0:
        cutters.append(frustum(bore_r + lead_in, bore_r, lead_in, z0=0.0,
                               sections=int(sections)))
        cutters.append(frustum(bore_r, bore_r + lead_in, lead_in,
                               z0=length - lead_in, sections=int(sections)))

    if relief_grooves and groove_style == "axial":
        quad = max(4, int(sections) // 4)
        centre_r = bore_r + groove_depth - groove_w / 2.0
        discs = []
        for k in range(relief_grooves):
            ang = 2.0 * math.pi * k / relief_grooves
            discs.append(sg.Point(centre_r * math.cos(ang),
                                  centre_r * math.sin(ang)).buffer(
                                      groove_w / 2.0, resolution=quad))
        cutters.append(extrude_poly_z(unary_union(discs), -2.0, length + 2.0))
    elif relief_grooves:
        step = length / (relief_grooves + 1.0)
        for k in range(1, relief_grooves + 1):
            zc = k * step
            cutters.append(frustum(bore_r, bore_r + groove_depth,
                                   groove_w / 2.0, z0=zc - groove_w / 2.0,
                                   sections=int(sections)))
            cutters.append(frustum(bore_r + groove_depth, bore_r,
                                   groove_w / 2.0, z0=zc,
                                   sections=int(sections)))

    mesh = sub(body, uni(cutters))
    mesh.metadata.update({
        "nominal_bore_d": float(bore_d),
        "bore_d": float(bore_d + clear),
        "outer_d": float(outer_d),
        "length": float(length),
        "wall": float(wall),
        "clear": float(clear),
        "flange_d": float(flange_d) if flange else 0.0,
        "flange_t": float(flange_t) if flange else 0.0,
        "relief_grooves": int(relief_grooves),
        "groove_style": groove_style if relief_grooves else "none",
        "bearing_area": float(bore_d * length),
    })
    return mesh


def thrust_washer(bore_d=8.0, outer_d=24.0, thickness=2.4, clear=0.3,
                  face="pockets", pockets=6, pocket_d=None, relief=0.6,
                  pad_gap_deg=14.0, pair=False, balls=6, ball_d=3.0,
                  groove_depth=None, cage_wall=1.0, sections=96):
    """Build a printed thrust washer, or a ball-race thrust pair.

    A thrust washer carries axial load between a rotor and its housing so the
    two never rub directly. With ``pair=False`` this returns a single flat
    annulus of ``thickness``, bored ``bore_d + clear``, whose upper face is
    relieved for lubricant: ``face='pockets'`` sinks ``pockets`` blind
    cylindrical grease reservoirs of depth ``relief`` at the mean radius,
    ``face='pads'`` leaves ``pockets`` raised sectors separated by
    ``pad_gap_deg`` wide channels so the real contact area (and therefore the
    PV product) drops, and ``face='flat'`` leaves it plain. The washer lies on
    z=0 with the relieved face up; print it in that orientation, flat on the
    plate, no supports.

    With ``pair=True`` this returns a dict of ``housing_washer``, ``cage``,
    ``balls`` and ``rotor_washer`` posed as an assembled axial ball thrust
    bearing: both washers get a toroidal raceway of depth ``groove_depth``
    facing each other, and a printed cage of wall ``cage_wall`` spaces
    ``balls`` rolling elements of diameter ``ball_d`` around the mean radius
    (the ``face``, ``pockets``, ``pocket_d``, ``relief`` and ``pad_gap_deg``
    arguments are ignored in that mode). The balls are BOUGHT hardware (steel
    bearing balls or ceramic); only the two washers and the cage are printed.
    Print the rotor washer flipped so that its raceway also faces up.
    Units are mm and degrees.
    """
    if bore_d <= 0.0 or outer_d <= 0.0:
        raise ValueError("thrust_washer(): bore_d and outer_d must be "
                         "positive")
    if clear < 0.0:
        raise ValueError("thrust_washer(): clear must be non-negative")
    if thickness < 1.2:
        raise ValueError("thrust_washer(): thickness must be at least 1.2 mm")
    if face not in ("flat", "pockets", "pads"):
        raise ValueError("thrust_washer(): face must be 'flat', 'pockets' or "
                         "'pads'")
    bore_r = (bore_d + clear) / 2.0
    outer_r = outer_d / 2.0
    if outer_r - bore_r < 2.0 * _MIN_WALL:
        raise ValueError("thrust_washer(): annulus is narrower than %.1f mm"
                         % (2.0 * _MIN_WALL))
    mean_r = (bore_r + outer_r) / 2.0

    if not pair:
        pockets = int(round(pockets))
        if face != "flat" and pockets < 2:
            raise ValueError("thrust_washer(): need at least 2 pockets or "
                             "pads")
        if face != "flat" and not 0.0 < relief <= thickness - _MIN_WALL:
            raise ValueError("thrust_washer(): relief must leave at least "
                             "%.1f mm under the face" % _MIN_WALL)
        ring = _ring_poly(bore_r, outer_r, sections)
        if face == "pads":
            if not 0.0 < pad_gap_deg < 360.0 / pockets:
                raise ValueError("thrust_washer(): pad_gap_deg must be "
                                 "positive and smaller than the pad pitch")
            base = trimesh.creation.extrude_polygon(ring, thickness - relief)
            pads = []
            span = 360.0 / pockets - pad_gap_deg
            for k in range(pockets):
                a0 = 360.0 * k / pockets
                wedge = sector2d(a0, a0 + span, outer_r * 1.5,
                                 n=max(6, int(sections) // 8))
                pads.append(wedge.intersection(ring))
            top = extrude_poly_z(unary_union(pads), thickness - relief,
                                 thickness)
            mesh = uni([base, top])
            contact_ratio = float(span * pockets / 360.0)
        else:
            mesh = trimesh.creation.extrude_polygon(ring, thickness)
            contact_ratio = 1.0
            if face == "pockets":
                if pocket_d is None:
                    pocket_d = 0.5 * (outer_r - bore_r)
                if pocket_d <= 0.0:
                    raise ValueError("thrust_washer(): pocket_d must be "
                                     "positive")
                if pocket_d / 2.0 > (outer_r - mean_r) - _MIN_WALL:
                    raise ValueError("thrust_washer(): pockets of %.2f mm do "
                                     "not fit in the annulus" % pocket_d)
                land = (2.0 * mean_r * math.sin(math.pi / pockets) - pocket_d)
                if land < _MIN_WALL:
                    raise ValueError("thrust_washer(): %d pockets leave only "
                                     "%.2f mm between them" % (pockets, land))
                cutters = []
                for k in range(pockets):
                    ang = 2.0 * math.pi * k / pockets
                    cutters.append(cyl(
                        pocket_d / 2.0, relief * 2.0,
                        center=(mean_r * math.cos(ang), mean_r * math.sin(ang),
                                thickness),
                        sections=int(sections)))
                mesh = sub(mesh, uni(cutters))
                contact_ratio = float(
                    1.0 - pockets * (pocket_d / 2.0) ** 2
                    / (outer_r ** 2 - bore_r ** 2))
        mesh.metadata.update({
            "bore_d": float(bore_d + clear),
            "outer_d": float(outer_d),
            "thickness": float(thickness),
            "mean_r": float(mean_r),
            "face": face,
            "contact_ratio": contact_ratio,
            "bearing_area": float(math.pi * (outer_r ** 2 - bore_r ** 2)
                                  * contact_ratio),
        })
        return mesh

    balls = int(round(balls))
    if balls < 3:
        raise ValueError("thrust_washer(): a ball pair needs at least 3 balls")
    if ball_d <= 0.0:
        raise ValueError("thrust_washer(): ball_d must be positive")
    if groove_depth is None:
        groove_depth = 0.25 * ball_d
    if not 0.0 < groove_depth < ball_d / 2.0:
        raise ValueError("thrust_washer(): groove_depth must be positive and "
                         "under the ball radius")
    if thickness - groove_depth < _MIN_WALL:
        raise ValueError("thrust_washer(): raceway leaves less than %.1f mm "
                         "under the washer" % _MIN_WALL)
    tube_r = ball_d / 2.0 + clear
    if mean_r - tube_r - bore_r < _MIN_WALL:
        raise ValueError("thrust_washer(): raceway leaves less than %.1f mm "
                         "of inner rim" % _MIN_WALL)
    if outer_r - (mean_r + tube_r) < _MIN_WALL:
        raise ValueError("thrust_washer(): raceway leaves less than %.1f mm "
                         "of outer rim" % _MIN_WALL)
    web = 2.0 * mean_r * math.sin(math.pi / balls) - ball_d - 2.0 * clear
    if web < _MIN_WALL:
        raise ValueError("thrust_washer(): %d balls leave only %.2f mm of "
                         "cage web between pockets" % (balls, web))
    if cage_wall < _MIN_WALL:
        raise ValueError("thrust_washer(): cage_wall must be at least %.1f mm"
                         % _MIN_WALL)

    ring = _ring_poly(bore_r, outer_r, sections)
    quad = max(4, int(sections) // 4)
    ball_z = thickness - groove_depth + tube_r
    face_gap = 2.0 * (tube_r - groove_depth)
    rotor_z0 = thickness + face_gap

    housing = trimesh.creation.extrude_polygon(ring, thickness)
    torus_lo = _revolve(sg.Point(mean_r, ball_z).buffer(tube_r,
                                                        resolution=quad),
                        sections=sections)
    housing = sub(housing, torus_lo)

    rotor = trimesh.creation.extrude_polygon(ring, thickness)
    rotor.apply_translation((0.0, 0.0, rotor_z0))
    torus_hi = _revolve(sg.Point(mean_r, ball_z).buffer(tube_r,
                                                        resolution=quad),
                        sections=sections)
    rotor = sub(rotor, torus_hi)

    cage_t = min(0.6 * ball_d, face_gap - 2.0 * clear)
    if cage_t < 0.6:
        raise ValueError("thrust_washer(): no room for a cage; raise ball_d "
                         "or lower groove_depth")
    cage_ring = _ring_poly(mean_r - ball_d / 2.0 - cage_wall,
                           mean_r + ball_d / 2.0 + cage_wall, sections)
    cage = trimesh.creation.extrude_polygon(cage_ring, cage_t)
    cage.apply_translation((0.0, 0.0, ball_z - cage_t / 2.0))
    holes = []
    ball_meshes = []
    for k in range(balls):
        ang = 2.0 * math.pi * k / balls
        cx, cy = mean_r * math.cos(ang), mean_r * math.sin(ang)
        holes.append(cyl(ball_d / 2.0 + clear, cage_t * 3.0,
                         center=(cx, cy, ball_z), sections=int(sections)))
        ball = trimesh.creation.icosphere(subdivisions=3, radius=ball_d / 2.0)
        ball.apply_translation((cx, cy, ball_z))
        ball_meshes.append(ball)
    cage = sub(cage, uni(holes))

    meta = {
        "bore_d": float(bore_d + clear),
        "outer_d": float(outer_d),
        "thickness": float(thickness),
        "mean_r": float(mean_r),
        "balls": int(balls),
        "ball_d": float(ball_d),
        "groove_depth": float(groove_depth),
        "face_gap": float(face_gap),
        "stack_height": float(rotor_z0 + thickness),
    }
    parts = {
        "housing_washer": housing,
        "cage": cage,
        "balls": uni(ball_meshes),
        "rotor_washer": rotor,
    }
    for part in parts.values():
        part.metadata.update(meta)
    return parts


def printed_ball_bearing(bore_d=8.0, outer_d=32.0, width=10.0, balls=6,
                         ball_d=7.0, clear=0.3, race_wall=2.0, floor_t=1.2,
                         cage=True, cage_wall=1.2, min_ball_d=6.0,
                         sections=96):
    """Build a print-in-place radial ball bearing (inner race, balls, outer race).

    Everything prints in one go, already assembled: an inner race with a
    bottom flange, ``balls`` rolling elements, an optional spacer cage, and an
    outer race. The geometry is shaped by what FDM can actually do, which is
    the whole difficulty of a printed ball bearing:

    * Each raceway is a circular groove of radius ``ball_d/2 + clear``
      TRUNCATED at plus and minus 45 degrees of ball latitude. Past 45 degrees
      the groove wall would be a shallower-than-45-degree overhang, so instead
      the race continues as a plain vertical shoulder. Truncating there still
      leaves a shoulder gap of ``sqrt(2)*(ball_d/2 + clear)``, which is
      narrower than the ball, so the balls stay captured axially and radially
      and lock the two races together. Nothing bridges over the balls.
    * Each ball is a sphere with its bottom cap cut off at 45 degrees of
      latitude, giving a flat first layer instead of a point. That flat prints
      ``clear`` above the cage rim (or above the inner race flange when
      ``cage=False``), the way any print-in-place part does; every remaining
      ball surface is at or above 45 degrees.
    * The ``cage`` is a free-floating spacer ring that keeps the balls from
      rubbing each other. It is not a retainer: the raceway shoulders are what
      hold the balls in. ``cage=False`` gives the full-complement variant.
    * Below ``min_ball_d`` (6 mm at a 0.4 mm nozzle) a printed ball is mostly
      seam and stair-stepping and will not roll, so it raises rather than
      pretending. A 608 footprint (8 x 22 x 7) cannot hold a printable ball at
      all: the radial budget only leaves about 2.4 mm. Printed ball bearings
      have to be physically bigger than their steel equivalents.

    The bearing sits with its bottom face on z=0 and its axis along +Z. Print
    it in exactly that orientation, no supports, and break it free by hand
    before first use. It is a low-speed, low-load part: for anything loaded or
    fast, press a steel bearing into ``cutters.bearing_seat`` instead.
    Units are mm and degrees.
    """
    if bore_d <= 0.0 or outer_d <= 0.0 or width <= 0.0:
        raise ValueError("printed_ball_bearing(): bore_d, outer_d and width "
                         "must be positive")
    if clear < 0.15:
        raise ValueError("printed_ball_bearing(): clear below 0.15 mm fuses "
                         "the print-in-place gaps")
    if race_wall < _MIN_WALL:
        raise ValueError("printed_ball_bearing(): race_wall must be at least "
                         "%.1f mm" % _MIN_WALL)
    if floor_t < _MIN_WALL:
        raise ValueError("printed_ball_bearing(): floor_t must be at least "
                         "%.1f mm" % _MIN_WALL)
    balls = int(round(balls))
    if balls < 3:
        raise ValueError("printed_ball_bearing(): need at least 3 balls")

    bore_r = bore_d / 2.0
    outer_r = outer_d / 2.0
    if ball_d is None:
        ball_d = (outer_r - bore_r) - 2.0 * race_wall - 2.0 * clear
    if ball_d < min_ball_d:
        raise ValueError(
            "printed_ball_bearing(): ball_d %.2f mm is below the %.1f mm "
            "printable floor; a printed ball smaller than that will not roll. "
            "Increase outer_d or drop the bore." % (ball_d, min_ball_d))
    a = ball_d / 2.0
    pitch_r = bore_r + race_wall + clear + a
    if outer_r - (pitch_r + a + clear) < _MIN_WALL:
        raise ValueError(
            "printed_ball_bearing(): outer_d %.2f mm leaves %.2f mm of outer "
            "race wall around a %.2f mm ball; it needs at least %.1f mm"
            % (outer_d, outer_r - (pitch_r + a + clear), ball_d, _MIN_WALL))

    shoulder = _SQRT_HALF * (a + clear)
    r_sh_i = pitch_r - shoulder
    r_sh_o = pitch_r + shoulder
    r_floor = r_sh_o - clear

    if cage:
        if cage_wall < _MIN_WALL:
            raise ValueError("printed_ball_bearing(): cage_wall must be at "
                             "least %.1f mm" % _MIN_WALL)
        cage_rim_t = max(_MIN_WALL, 0.8)
        ball_z0 = floor_t + 2.0 * clear + cage_rim_t
    else:
        cage_rim_t = 0.0
        ball_z0 = floor_t + clear
    ball_zc = ball_z0 + _SQRT_HALF * a
    groove_lo = ball_zc - shoulder
    groove_hi = ball_zc + shoulder
    ball_top = ball_zc + a
    if width < ball_top + 0.4:
        raise ValueError(
            "printed_ball_bearing(): width %.2f mm is too short; the ball "
            "stack alone needs %.2f mm" % (width, ball_top + 0.4))

    pitch_gap = 2.0 * pitch_r * math.sin(math.pi / balls) - ball_d
    needed = (cage_wall + 2.0 * clear) if cage else 0.4
    if pitch_gap < needed:
        raise ValueError(
            "printed_ball_bearing(): %d balls of %.2f mm on a %.2f mm pitch "
            "radius leave only %.2f mm between them; %.2f mm is needed"
            % (balls, ball_d, pitch_r, pitch_gap, needed))

    quad = max(4, int(sections) // 4)
    groove = sg.Point(pitch_r, ball_zc).buffer(a + clear, resolution=quad)
    groove = groove.intersection(
        sg.box(0.0, groove_lo, outer_r * 3.0, groove_hi))

    inner_prof = unary_union([
        sg.box(bore_r, 0.0, r_sh_i, width),
        sg.box(bore_r, 0.0, r_floor, floor_t),
    ]).difference(groove)
    outer_prof = sg.box(r_sh_o, 0.0, outer_r, width).difference(groove)
    inner_race = _revolve(inner_prof, sections=sections)
    outer_race = _revolve(outer_prof, sections=sections)

    template = _truncated_ball(a, -_SQRT_HALF * a, sections=sections)
    template.apply_translation((pitch_r, 0.0, ball_zc))
    ball_meshes = []
    for k in range(balls):
        ball = template.copy()
        ball.apply_transform(tf.rotation_matrix(
            2.0 * math.pi * k / balls, (0, 0, 1)))
        ball_meshes.append(ball)

    parts = {"inner_race": inner_race, "outer_race": outer_race}
    if cage:
        rim_z0 = floor_t + clear
        rim = trimesh.creation.extrude_polygon(
            _ring_poly(r_sh_i + clear, r_sh_o - clear, sections), cage_rim_t)
        rim.apply_translation((0.0, 0.0, rim_z0))
        span = (r_sh_o - clear) - (r_sh_i + clear)
        webs = []
        for k in range(balls):
            web = boxc((span, cage_wall, ball_zc - rim_z0),
                       center=(pitch_r, 0.0, (ball_zc + rim_z0) / 2.0))
            web.apply_transform(tf.rotation_matrix(
                2.0 * math.pi * (k + 0.5) / balls, (0, 0, 1)))
            webs.append(web)
        envelope = _truncated_ball(a + clear, -_SQRT_HALF * a - clear,
                                   sections=sections)
        envelope.apply_translation((pitch_r, 0.0, ball_zc))
        envelopes = []
        for k in range(balls):
            env = envelope.copy()
            env.apply_transform(tf.rotation_matrix(
                2.0 * math.pi * k / balls, (0, 0, 1)))
            envelopes.append(env)
        parts["cage"] = sub(uni([rim] + webs), uni(envelopes))
    parts["balls"] = uni(ball_meshes)

    meta = {
        "bore_d": float(bore_d),
        "outer_d": float(outer_d),
        "width": float(width),
        "balls": int(balls),
        "ball_d": float(ball_d),
        "pitch_r": float(pitch_r),
        "clear": float(clear),
        "shoulder_gap": float(2.0 * shoulder),
        "ball_pitch_gap": float(pitch_gap),
        "groove_span": (float(groove_lo), float(groove_hi)),
        "caged": bool(cage),
    }
    for part in parts.values():
        part.metadata.update(meta)
    return parts


__all__ = (
    "plain_bushing",
    "thrust_washer",
    "printed_ball_bearing",
)
