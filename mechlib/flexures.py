"""Project-agnostic compliant-mechanism generators (print-in-place parts)."""

import math

import numpy as np
import shapely.geometry as sg
from shapely.ops import unary_union
import trimesh
import trimesh.transformations as tf

from .meshutil import from_manifold, sub, to_manifold, uni
from .prim import boxc, cyl
from .sweep import loft


def _extrude(poly, height, z0=0.0):
    if height <= 0:
        raise ValueError("extrusion height must be positive")
    mesh = trimesh.creation.extrude_polygon(poly, height)
    if not mesh.is_watertight:
        mesh = from_manifold(to_manifold(mesh))
    if z0:
        mesh.apply_translation((0.0, 0.0, z0))
    return mesh


def cross_flexure(block_w=22.0, block_d=16.0, block_h=6.0, gap=10.0,
                  blade_t=0.8, blade_w=6.0, blade_angle_deg=45.0,
                  blade_gap=1.0, embed=1.5):
    """Build a monolithic cross-axis flexural pivot (cross-spring pivot).

    Two rigid blocks are joined only by two thin flat blades crossing at
    ``blade_angle_deg`` to the axis, so the top block rotates a limited angle
    about the crossing point with zero friction and zero backlash. The two
    blades are offset ``blade_gap`` apart along the blade width so they pass
    each other without intersecting, and each blade is embedded ``embed`` mm
    into both blocks; the result is a single printable part. The bottom block
    top face sits at z=0 and the blades span the ``gap`` to the top block.
    Print with the blades vertical so layer lines run along the blades.
    Units are mm and degrees.
    """
    if (block_w <= 0 or block_d <= 0 or block_h < 1.2 or gap <= 0 or
            blade_t <= 0 or blade_w <= 0 or
            not 10.0 <= blade_angle_deg <= 75.0 or blade_gap <= 0 or
            embed <= 0):
        raise ValueError("invalid cross flexure dimensions")
    half_span = (gap / 2.0 + embed) * math.tan(math.radians(blade_angle_deg))
    if half_span > block_w / 2.0 - 0.8:
        raise ValueError("blades overhang the blocks; reduce angle or gap")
    if 2.0 * blade_w + blade_gap > block_d:
        raise ValueError("blades wider than the block depth")
    blade_len = 2.0 * (gap / 2.0 + embed) / math.cos(
        math.radians(blade_angle_deg))

    parts = [
        boxc((block_w, block_d, block_h), center=(0, 0, -block_h / 2.0)),
        boxc((block_w, block_d, block_h),
             center=(0, 0, gap + block_h / 2.0)),
    ]
    for sign, y_off in ((1.0, (blade_w + blade_gap) / 2.0),
                        (-1.0, -(blade_w + blade_gap) / 2.0)):
        blade = boxc((blade_t, blade_w, blade_len))
        blade.apply_transform(tf.rotation_matrix(
            sign * math.radians(blade_angle_deg), (0, 1, 0)))
        blade.apply_translation((0, y_off, gap / 2.0))
        parts.append(blade)
    return uni(parts)


def wave_spring(d=30.0, waves=3, turns=2, strip_w=3.0, strip_t=0.8,
                amplitude=1.0, crest_fuse=0.15, sections=96):
    """Build a crest-to-crest wave spring (Smalley-style) around the Z axis.

    Each turn is an annular strip of radial width ``strip_w`` and axial
    thickness ``strip_t`` whose centerline follows a sinusoid of ``waves``
    lobes and ``amplitude`` around the circumference. Consecutive turns are
    phase-shifted by half a wave, so the upward crests of one turn bear on
    the downward crests of the next (Smalley crest-to-crest style) and are
    fused by ``crest_fuse`` mm of interference at the contact points; set
    ``crest_fuse=0`` for a stack of separate wave washers. The turn pitch is
    ``2*amplitude + strip_t - crest_fuse`` so the crests overlap into one
    printable body. Units are mm; ``sections`` is the per-turn sampling.
    """
    if (d <= 0 or waves < 2 or turns < 1 or strip_w <= 0 or strip_t <= 0 or
            amplitude <= 0 or crest_fuse < 0 or crest_fuse >= strip_t or
            sections < 24 or d <= strip_w + 1.0):
        raise ValueError("invalid wave spring dimensions")
    waves = int(round(waves))
    turns = int(round(turns))
    sections = int(round(sections))
    mean_r = d / 2.0
    turn_pitch = 2.0 * amplitude + strip_t - crest_fuse
    theta = np.linspace(0.0, 2.0 * np.pi, sections, endpoint=False)
    corners = ((mean_r - strip_w / 2.0, -strip_t / 2.0),
               (mean_r + strip_w / 2.0, -strip_t / 2.0),
               (mean_r + strip_w / 2.0, strip_t / 2.0),
               (mean_r - strip_w / 2.0, strip_t / 2.0))
    meshes = []
    for turn in range(turns):
        z_center = (turn * turn_pitch
                    + amplitude * np.sin(waves * theta + turn * np.pi))
        verts = np.empty((sections * 4, 3))
        for k, (radius, z_off) in enumerate(corners):
            verts[k::4, 0] = radius * np.cos(theta)
            verts[k::4, 1] = radius * np.sin(theta)
            verts[k::4, 2] = z_center + z_off
        faces = []
        for i in range(sections):
            j = (i + 1) % sections
            for k in range(4):
                k2 = (k + 1) % 4
                a0, b0 = i * 4 + k, i * 4 + k2
                a1, b1 = j * 4 + k, j * 4 + k2
                faces.append((a0, a1, b1))
                faces.append((a0, b1, b0))
        mesh = trimesh.Trimesh(vertices=verts, faces=np.array(faces),
                               process=False)
        mesh.fix_normals()
        meshes.append(mesh)
    if len(meshes) == 1:
        return meshes[0]
    return uni(meshes)


def bistable_beam(span=44.0, frame_h=24.0, wall=2.0, beams=2, beam_t=1.0,
                  beam_gap=8.0, apex=3.0, shuttle_w=6.0, travel=2.5,
                  clearance=0.3, thickness=3.0, sections=48):
    """Build a flat-printable buckled-beam bistable switch (snap-through).

    ``beams`` pre-curved beams are clamped between the two end walls of a
    rectangular frame and carry a central shuttle; each beam follows the
    cosine buckling-mode shape ``y0 - sign(y0) * apex * (1-cos(2*pi*x/span))/2``
    toward the mid-plane, so pushing the shuttle past the snap-through point
    flips it into a second stable position. Beams sit at evenly spaced base
    offsets inside ``beam_gap`` and fuse into the shuttle; the frame side
    rails keep ``travel + clearance`` of free space around the shuttle. The
    whole mechanism is one monolithic part extruded ``thickness`` mm, printed
    flat with in-plane motion. Units are mm.
    """
    if (span <= 0 or frame_h <= 0 or wall < 1.2 or beams < 2 or
            beam_t <= 0 or beam_gap <= 0 or apex <= 0 or shuttle_w <= 0 or
            travel <= 0 or clearance < 0 or thickness < 1.2 or sections < 16):
        raise ValueError("invalid bistable beam dimensions")
    if apex >= beam_gap / 2.0:
        raise ValueError("apex must stay below half the beam gap")
    shuttle_half = beam_gap / 2.0 - apex + beam_t
    if frame_h / 2.0 - wall < shuttle_half + travel + clearance:
        raise ValueError("frame too low for shuttle travel and clearance")
    if shuttle_w / 2.0 > span / 2.0 - wall - clearance:
        raise ValueError("shuttle too wide for the frame")
    beams = int(round(beams))
    sections = int(round(sections))

    half = frame_h / 2.0
    end_walls = [
        sg.box(0.0, -half, wall, half),
        sg.box(span - wall, -half, span, half),
    ]
    rails = [
        sg.box(0.0, half - wall, span, half),
        sg.box(0.0, -half, span, -half + wall),
    ]
    xs = np.linspace(wall * 0.4, span - wall * 0.4, sections)
    mode = 0.5 * (1.0 - np.cos(2.0 * np.pi * xs / span))
    beam_polys = []
    base_ys = np.linspace(-beam_gap / 2.0, beam_gap / 2.0, beams)
    for y0 in base_ys:
        direction = -math.copysign(1.0, y0) if y0 else 0.0
        curve = np.c_[xs, y0 + direction * apex * mode]
        beam_polys.append(sg.LineString(curve).buffer(
            beam_t / 2.0, cap_style=2, join_style=1))
    shuttle = sg.box(span / 2.0 - shuttle_w / 2.0, -shuttle_half,
                     span / 2.0 + shuttle_w / 2.0, shuttle_half)
    profile = unary_union(end_walls + rails + beam_polys + [shuttle]).buffer(0)
    return _extrude(profile, thickness)


def _coned_annulus(r_out, r_in, cone_h, thickness, sections):
    """Revolve one disc-spring section (a parallelogram) about +Z.

    The section is bounded by two parallel cones ``thickness`` apart measured
    axially, a vertical outer rim and a vertical bore. The outer rim bottom
    sits at z=0 and the bore top at ``cone_h + thickness``.
    """
    ang = np.linspace(0.0, 2.0 * np.pi, sections, endpoint=False)

    def ring(radius, z):
        return np.c_[radius * np.cos(ang), radius * np.sin(ang),
                     np.full(sections, float(z))]

    slope = cone_h / (r_out - r_in)
    over = min(0.5, 0.5 * r_in)          # inward overshoot, trimmed by the bore
    pad = thickness + 1.0
    lower = loft([ring(r_out, -pad), ring(r_out, 0.0),
                  ring(r_in - over, cone_h + over * slope)])
    upper = lower.copy()
    upper.apply_translation((0.0, 0.0, thickness))
    disc = sub(upper, lower)
    bore = cyl(r_in, 4.0 * (cone_h + thickness + pad), sections=sections)
    return sub(disc, bore)


def belleville_washer(outer_d=20.0, inner_d=10.2, thickness=1.0, free_h=1.6,
                      stack=1, arrangement="series", stack_gap=0.4,
                      sections=96):
    """Build a coned disc spring (Belleville washer), optionally as a stack.

    A single disc is an annulus of axial ``thickness`` whose faces are two
    parallel cones, so the free height ``free_h`` of the disc exceeds its
    thickness by the cone height ``h0 = free_h - thickness``. The disc spring
    carries the highest force per unit volume of any printable spring, and it
    is the only one whose load curve turns bistable purely by proportion:
    above ``h0/thickness = sqrt(2)`` the load-deflection curve has a negative
    slope region, so the disc snaps through instead of resisting smoothly.
    ``h0_over_t`` and the ``snap_through`` flag are stored in ``metadata``.

    ``stack`` discs can be combined: ``"series"`` alternates the cone
    direction rim to rim (travel adds, load stays), ``"parallel"`` nests them
    facing the same way (load adds, travel stays), and ``"alternating"``
    nests them in opposed pairs (an even ``stack`` is required) for the
    classic mixed stack. Stacked discs are separated by ``stack_gap`` so the
    slicer keeps them as distinct bodies; the resulting ``stack_height`` is
    stored in ``metadata``. The lowest rim sits at z=0 and the axis is +Z.

    Print with the axis vertical: every layer is then an annulus whose beads
    run circumferentially, which is the direction of the hoop stress that
    dominates a disc spring, so the load runs along the extrusions rather than
    across layer boundaries. The catch is the coned underside: ``metadata``
    reports ``cone_angle_deg``, and below 45 degrees (which is nearly always,
    for useful proportions) that underside needs support or a very slow first
    few layers. PLA cracks after a handful of cycles; use PETG for occasional
    travel and TPU for anything genuinely cyclic. Keep ``thickness`` at or
    above 0.8 mm. Units are mm and degrees.
    """
    if outer_d <= 0 or inner_d <= 0 or inner_d >= outer_d:
        raise ValueError("belleville: inner_d must be positive and below outer_d")
    if outer_d - inner_d < 3.2:
        raise ValueError("belleville: annulus narrower than 2 walls (1.6 mm radial)")
    if thickness < 0.8:
        raise ValueError("belleville: thickness below the 0.8 mm printable minimum")
    if free_h <= thickness:
        raise ValueError("belleville: free_h must exceed thickness (cone height > 0)")
    if arrangement not in ("series", "parallel", "alternating"):
        raise ValueError(
            "belleville: arrangement must be series, parallel or alternating")
    stack = int(round(stack))
    if stack < 1:
        raise ValueError("belleville: stack must be at least 1")
    if stack > 1 and stack_gap < 0.4:
        raise ValueError("belleville: stack_gap below the 0.4 mm printable kerf")
    if arrangement == "alternating" and stack % 2:
        raise ValueError("belleville: alternating stacks need an even disc count")
    if sections < 24:
        raise ValueError("belleville: sections must be at least 24")

    r_out, r_in = outer_d / 2.0, inner_d / 2.0
    cone_h = free_h - thickness
    disc = _coned_annulus(r_out, r_in, cone_h, thickness, int(round(sections)))

    placed, z = [], 0.0
    for index in range(stack):
        if arrangement == "series":
            flip, advance = bool(index % 2), free_h + stack_gap
        elif arrangement == "parallel":
            flip, advance = False, thickness + stack_gap
        else:
            flip = bool((index // 2) % 2)
            advance = (thickness + stack_gap) if index % 2 == 0 else free_h + stack_gap
        piece = disc.copy()
        if flip:
            piece.apply_transform(tf.rotation_matrix(math.pi, (1, 0, 0)))
            piece.apply_translation((0.0, 0.0, free_h))
        piece.apply_translation((0.0, 0.0, z))
        placed.append(piece)
        z += advance
    stack_height = z - advance + free_h

    mesh = placed[0] if stack == 1 else uni(placed)
    cone_angle_deg = math.degrees(math.atan2(cone_h, r_out - r_in))
    mesh.metadata.update({
        "outer_d": float(outer_d),
        "inner_d": float(inner_d),
        "thickness": float(thickness),
        "free_h": float(free_h),
        "h0": float(cone_h),
        "h0_over_t": float(cone_h / thickness),
        "snap_through": bool(cone_h / thickness > math.sqrt(2.0)),
        "cone_angle_deg": float(cone_angle_deg),
        "self_supporting": bool(cone_angle_deg >= 45.0),
        "stack": int(stack),
        "arrangement": arrangement,
        "stack_height": float(stack_height),
    })
    return mesh


def _sweep_section(centerline, section):
    """Sweep a closed 2D section along a 3D centerline with a radial frame.

    ``section`` is a list of ``(radial, axial)`` offsets applied in a frame
    whose normal points at the Z axis and whose binormal completes the
    right-handed set, the same twist-free construction ``mechanisms.helix_tube``
    uses. Both ends are closed with a fan cap.
    """
    tangent = np.gradient(centerline, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
    radial = centerline.copy()
    radial[:, 2] = 0.0
    normal = -radial / np.linalg.norm(radial, axis=1, keepdims=True)
    binormal = np.cross(tangent, normal)
    binormal /= np.linalg.norm(binormal, axis=1, keepdims=True)
    normal = np.cross(binormal, tangent)

    sect = np.asarray(section, float)
    count, sides = len(centerline), len(sect)
    rings = (normal[:, None, :] * sect[None, :, 0, None]
             + binormal[:, None, :] * sect[None, :, 1, None]
             + centerline[:, None, :])
    vertices = np.vstack([rings.reshape(-1, 3),
                          centerline[0], centerline[-1]])
    # Consistent grid winding, built without a graph pass so the module stays
    # free of scipy and networkx (the playground has neither).
    i = np.arange(count - 1)[:, None]
    j = np.arange(sides)[None, :]
    a = i * sides + j
    b = i * sides + (j + 1) % sides
    faces = np.concatenate([
        np.stack([a, b, b + sides], axis=-1).reshape(-1, 3),
        np.stack([a, b + sides, a + sides], axis=-1).reshape(-1, 3),
    ])
    hub0, hub1 = count * sides, count * sides + 1
    base = (count - 1) * sides
    js = np.arange(sides)
    caps = np.concatenate([
        np.stack([np.full(sides, hub0), (js + 1) % sides, js], axis=-1),
        np.stack([np.full(sides, hub1), base + js,
                  base + (js + 1) % sides], axis=-1),
    ])
    mesh = trimesh.Trimesh(vertices=vertices,
                           faces=np.concatenate([faces, caps]), process=False)
    if mesh.volume < 0:
        mesh.invert()
    return mesh


def coil_spring(coil_d=12.0, wire_d=2.0, turns=6.0, pitch=None, ends="closed",
                section="round", wire_w=None, coil_gap=0.4, min_helix_deg=4.0,
                shear_modulus_mpa=900.0, facets=16, steps_per_turn=48):
    """Build a helical compression spring swept along a cylinder about +Z.

    The wire centerline follows a helix of mean diameter ``coil_d``; the
    active coils advance by ``pitch`` per turn while one dead coil at each end
    advances by only ``wire_d`` so the end coils close up into a flat seat
    (``ends="closed"``), are ground flat against a plane (``ends="ground"``),
    or are left open at full pitch (``ends="open"``). ``section="round"``
    sweeps a circular wire of diameter ``wire_d``; ``section="rect"`` sweeps a
    rectangle ``wire_w`` wide radially by ``wire_d`` tall axially, which
    raises the rate for the same envelope and gives the wire flat faces
    instead of a round underside. The
    bottom of the spring sits at z=0 and ``free_length``, ``active_turns``,
    ``spring_index``, ``helix_angle_deg`` and an idealised
    ``rate_n_per_mm`` are stored in ``metadata``.

    Print with the spring axis vertical. Two FDM floors are enforced: the
    axial gap between coils must be at least ``coil_gap`` (below that the
    slicer fuses the turns into a tube), so ``pitch >= wire_d + coil_gap``,
    and the helix angle must reach ``min_helix_deg`` or the wire lies so flat
    that its underside becomes a long unsupported bridge. Both raise
    ValueError. A printed spring has poor fatigue life next to wound wire and
    the rate estimate assumes an isotropic solid section, so treat this as
    geometry for demos, light detents and compliant mechanisms, not as a
    load-bearing spring. Units are mm and degrees.
    """
    if coil_d <= 0 or wire_d <= 0:
        raise ValueError("coil_spring: coil_d and wire_d must be positive")
    if ends not in ("open", "closed", "ground"):
        raise ValueError("coil_spring: ends must be open, closed or ground")
    if section not in ("round", "rect"):
        raise ValueError("coil_spring: section must be round or rect")
    if coil_gap < 0.4:
        raise ValueError("coil_spring: coil_gap below the 0.4 mm printable kerf")
    if wire_d < 0.8:
        raise ValueError("coil_spring: wire_d below the 0.8 mm printable minimum")
    pitch = 2.0 * wire_d if pitch is None else float(pitch)
    wire_w = wire_d if wire_w is None else float(wire_w)
    index = coil_d / wire_d
    if index < 3.0:
        raise ValueError("coil_spring: spring index coil_d/wire_d below 3")
    if pitch < wire_d + coil_gap:
        raise ValueError(
            "coil_spring: pitch %.2f below the %.2f mm floor (wire_d + coil_gap); "
            "the coils would fuse" % (pitch, wire_d + coil_gap))
    helix_deg = math.degrees(math.atan2(pitch, math.pi * coil_d))
    if helix_deg < min_helix_deg:
        raise ValueError(
            "coil_spring: helix angle %.1f deg below the %.1f deg self-support "
            "floor; raise pitch or shrink coil_d" % (helix_deg, min_helix_deg))
    dead = 1.0 if ends in ("closed", "ground") else 0.0
    active = float(turns) - 2.0 * dead
    if active < 1.0:
        raise ValueError("coil_spring: fewer than one active coil after dead ends")
    if section == "rect" and (wire_w < 0.8 or wire_w > coil_d / 2.0):
        raise ValueError("coil_spring: wire_w must be 0.8 mm .. coil_d/2")
    facets = max(8, int(round(facets)))

    steps = max(64, int(round(steps_per_turn * float(turns))) + 1)
    turn = np.linspace(0.0, float(turns), steps)
    rise = np.where(
        turn <= dead, turn * wire_d,
        np.where(turn <= float(turns) - dead,
                 dead * wire_d + (turn - dead) * pitch,
                 dead * wire_d + active * pitch
                 + (turn - (float(turns) - dead)) * wire_d))
    theta = 2.0 * np.pi * turn
    radius = coil_d / 2.0
    centerline = np.c_[radius * np.cos(theta), radius * np.sin(theta),
                       rise + wire_d / 2.0]
    if section == "round":
        phi = np.linspace(0.0, 2.0 * np.pi, facets, endpoint=False)
        sect = np.c_[np.cos(phi), np.sin(phi)] * (wire_d / 2.0)
    else:
        sect = np.array([(-wire_w / 2.0, -wire_d / 2.0),
                         (wire_w / 2.0, -wire_d / 2.0),
                         (wire_w / 2.0, wire_d / 2.0),
                         (-wire_w / 2.0, wire_d / 2.0)])
    mesh = _sweep_section(centerline, sect)

    grind = 0.25 * wire_d if ends == "ground" else 0.0
    span = float(rise[-1]) + wire_d
    if grind:
        big = 4.0 * coil_d
        mesh = sub(mesh, boxc((big, big, big), center=(0, 0, grind - big / 2.0)))
        mesh = sub(mesh, boxc((big, big, big),
                              center=(0, 0, span - grind + big / 2.0)))
    mesh.apply_translation((0.0, 0.0, -grind))
    free_length = span - 2.0 * grind
    inertia_d = wire_d if section == "round" else min(wire_d, wire_w)
    mesh.metadata.update({
        "coil_d": float(coil_d),
        "wire_d": float(wire_d),
        "pitch": float(pitch),
        "turns": float(turns),
        "active_turns": float(active),
        "dead_turns": float(2.0 * dead),
        "ends": ends,
        "section": section,
        "free_length": float(free_length),
        "solid_length": float((float(turns) + 1.0) * wire_d),
        "spring_index": float(index),
        "helix_angle_deg": float(helix_deg),
        "coil_gap": float(pitch - wire_d),
        "rate_n_per_mm": float(shear_modulus_mpa * inertia_d ** 4
                               / (8.0 * coil_d ** 3 * active)),
    })
    return mesh


def _spiral_length(r_inner, pitch, turns):
    """Return the arc length of an Archimedean spiral by sampling."""
    theta = np.linspace(0.0, 2.0 * np.pi * turns, max(256, int(200 * turns)))
    r = r_inner + pitch * theta / (2.0 * np.pi)
    pts = np.c_[r * np.cos(theta), r * np.sin(theta)]
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())


def _packed_turns(length, radius, pitch, inward):
    """Return the turn count of a strip of ``length`` packed at ``pitch``.

    ``inward`` packs the coil inward from ``radius`` (against a barrel wall);
    otherwise it packs outward from ``radius`` (around an arbor).
    """
    if inward:
        disc = radius ** 2 - pitch * length / math.pi
        if disc <= 0:
            raise ValueError("spiral spring: strip too long to pack in the barrel")
        return (radius - math.sqrt(disc)) / pitch
    return (-radius + math.sqrt(radius ** 2 + pitch * length / math.pi)) / pitch


def spiral_power_spring(barrel_d=60.0, arbor_d=16.0, strip_t=1.0, gap=0.5,
                        turns=6.0, height=6.0, wall=4.0, bore_d=6.0,
                        clearance=0.3, max_fill=0.5, sections=720):
    """Build a flat spiral power spring (clock mainspring) as three parts.

    The strip follows the Archimedean spiral ``r = r0 + k*theta`` with
    ``k = (strip_t + gap) / 2pi``, so consecutive turns keep a clear ``gap``
    when the spring is relaxed. Its inner end runs radially into a T slot in
    the ``arbor`` and its outer end into a matching T slot in the ``barrel``
    wall, both with ``clearance``: the flat parts are printed separately and
    the spring drops into both slots along +Z, then winding the arbor stores
    energy. Returns ``{"barrel", "spring", "arbor"}`` posed in assembly with
    all three sitting on z=0.

    Torque falls as the spring unwinds, which is exactly what
    ``pulleys.grooved_drum(radius_law="fusee")`` compensates: pair the two to
    get a constant output torque. ``metadata`` carries ``arc_length``, the
    ``wound_turns`` on the arbor, the ``unwound_turns`` packed against the
    barrel, their difference as ``stored_turns``, and ``fill_fraction``. A
    mainspring cannot exceed roughly half the free annular area or it has no
    room to wind, so a strip above ``max_fill`` raises ValueError, as does a
    spiral whose outer turn does not fit inside the barrel.

    Print all three parts flat on the bed so the layer plane coincides with
    the spring's in-plane bending plane; that is the only orientation where a
    printed spiral survives more than a few winds. Keep ``strip_t`` between
    0.8 and 1.2 mm and ``gap`` at 0.4 mm or more. PETG or TPU, not PLA.
    Units are mm and degrees.
    """
    if barrel_d <= 0 or arbor_d <= 0 or arbor_d >= barrel_d:
        raise ValueError("spiral spring: arbor_d must be positive and below barrel_d")
    if strip_t < 0.8:
        raise ValueError("spiral spring: strip_t below the 0.8 mm printable minimum")
    if gap < 0.4:
        raise ValueError("spiral spring: gap below the 0.4 mm printable kerf")
    if turns < 1.0:
        raise ValueError("spiral spring: turns must be at least 1")
    if height < 1.2 or wall < 1.2 or clearance < 0.15:
        raise ValueError("spiral spring: height, wall or clearance too small")
    if not 0.05 < max_fill <= 0.8:
        raise ValueError("spiral spring: max_fill must be within (0.05, 0.8]")

    barrel_r, arbor_r = barrel_d / 2.0, arbor_d / 2.0
    pitch = strip_t + gap
    r0 = arbor_r + clearance + strip_t / 2.0
    r_end = r0 + pitch * turns
    if r_end + strip_t / 2.0 + clearance > barrel_r:
        raise ValueError(
            "spiral spring: outer turn at r=%.1f does not fit the %.1f mm barrel bore"
            % (r_end + strip_t / 2.0, barrel_r))

    # T-slot anchor depths, derived so 1.2 mm of material survives past the
    # deepest slot corner in both the arbor and the barrel wall.
    slot_r = strip_t / 2.0 + clearance
    bar = 3.0 * strip_t
    corner = bar / 2.0 + slot_r
    if arbor_r - 1.2 <= corner:
        raise ValueError("spiral spring: arbor too small for the hook T slot")
    arbor_anchor = arbor_r + slot_r - math.sqrt((arbor_r - 1.2) ** 2 - corner ** 2)
    if arbor_anchor < slot_r + 0.5:
        raise ValueError("spiral spring: arbor hook slot has no depth")
    if arbor_r - arbor_anchor - slot_r - (bore_d + clearance) / 2.0 < 1.2:
        raise ValueError(
            "spiral spring: arbor bore leaves under 1.2 mm to the hook slot")
    barrel_anchor = (math.sqrt((barrel_r + wall - 1.2) ** 2 - corner ** 2)
                     - barrel_r - slot_r)
    if barrel_anchor < slot_r + 0.5:
        raise ValueError(
            "spiral spring: barrel wall %.1f mm too thin for the anchor T slot" % wall)

    length = _spiral_length(r0, pitch, turns)
    annulus_area = math.pi * (barrel_r ** 2 - arbor_r ** 2)
    fill = length * strip_t / annulus_area
    if fill > max_fill:
        raise ValueError(
            "spiral spring: strip fills %.0f%% of the barrel annulus (max %.0f%%); "
            "shorten the strip or grow the barrel" % (100 * fill, 100 * max_fill))
    wound = _packed_turns(length, arbor_r + strip_t / 2.0, strip_t, False)
    unwound = _packed_turns(length, barrel_r - strip_t / 2.0, strip_t, True)

    steps = max(180, int(round(sections)))
    theta = np.linspace(0.0, 2.0 * np.pi * turns, steps)
    radii = r0 + pitch * theta / (2.0 * np.pi)
    spine = [(float(r * math.cos(t)), float(r * math.sin(t)))
             for r, t in zip(radii, theta)]
    inner_tip = (arbor_r - arbor_anchor, 0.0)
    outer_tip = (barrel_r + barrel_anchor, 0.0)
    stub_line = sg.LineString([inner_tip] + spine[:1])
    end_line = sg.LineString([spine[-1], (outer_tip[0] * math.cos(theta[-1]),
                                          outer_tip[0] * math.sin(theta[-1]))])
    inner_bar = sg.LineString([(inner_tip[0], -bar / 2.0),
                               (inner_tip[0], bar / 2.0)])
    end_dir = np.array([math.cos(theta[-1]), math.sin(theta[-1])])
    perp = np.array([-end_dir[1], end_dir[0]])
    outer_center = end_dir * outer_tip[0]
    outer_bar = sg.LineString([tuple(outer_center - perp * bar / 2.0),
                               tuple(outer_center + perp * bar / 2.0)])

    strip = unary_union([
        sg.LineString(spine).buffer(strip_t / 2.0, cap_style=2, join_style=1),
        stub_line.buffer(strip_t / 2.0, cap_style=2, join_style=1),
        end_line.buffer(strip_t / 2.0, cap_style=2, join_style=1),
        inner_bar.buffer(strip_t / 2.0, cap_style=2, join_style=1),
        outer_bar.buffer(strip_t / 2.0, cap_style=2, join_style=1),
    ]).buffer(0)

    # Round caps so the slot also clears the flat-capped ends of the T anchor.
    arbor_slot = unary_union([
        stub_line.buffer(slot_r, cap_style=1, join_style=1),
        inner_bar.buffer(slot_r, cap_style=1, join_style=1)]).buffer(0)
    barrel_slot = unary_union([
        end_line.buffer(slot_r, cap_style=1, join_style=1),
        outer_bar.buffer(slot_r, cap_style=1, join_style=1)]).buffer(0)

    origin = sg.Point(0.0, 0.0)
    barrel_poly = origin.buffer(barrel_r + wall, resolution=32).difference(
        origin.buffer(barrel_r, resolution=32)).difference(barrel_slot)
    arbor_poly = origin.buffer(arbor_r, resolution=32).difference(arbor_slot)
    if bore_d > 0:
        arbor_poly = arbor_poly.difference(
            origin.buffer((bore_d + clearance) / 2.0, resolution=24))

    meta = {
        "barrel_d": float(barrel_d),
        "arbor_d": float(arbor_d),
        "strip_t": float(strip_t),
        "gap": float(gap),
        "turns": float(turns),
        "arc_length": float(length),
        "wound_turns": float(wound),
        "unwound_turns": float(unwound),
        "stored_turns": float(wound - unwound),
        "fill_fraction": float(fill),
        "spiral_pitch": float(pitch),
    }
    parts = {
        "barrel": _extrude(barrel_poly, height),
        "spring": _extrude(strip, height),
        "arbor": _extrude(arbor_poly, height),
    }
    for part in parts.values():
        part.metadata.update(meta)
    return parts


def leaf_spring(span=90.0, leaves=3, leaf_t=2.0, width=10.0, camber=9.0,
                leaf_gap=0.4, taper=0.6, clamp_w=14.0, clamp_wall=2.4,
                clamp_clearance=0.3, eye_d=4.0, eye_wall=1.5,
                modulus_mpa=2000.0, sections=64):
    """Build a semi-elliptic multi-leaf spring plus its centre clamp band.

    ``leaves`` circular-arc leaves of thickness ``leaf_t`` are stacked on a
    common centre, each shorter than the one above it (the shortest is
    ``taper`` of the full ``span``) and separated by ``leaf_gap`` so the
    slicer keeps them apart. The longest leaf carries a rolled eye of bore
    ``eye_d`` at each end; the arc is set so those two eyes sit at y=0 and the
    crown rises to ``camber`` at x=0. The ``clamp`` is a closed rectangular
    band that slides over the pack at the centre and holds the leaves
    together, so the build refuses geometry whose clamp window cannot pass
    over the eye. Returns ``{"leaf_1" .. "leaf_n", "clamp"}`` posed in
    assembly, leaves extruded ``width`` deep along +Z from z=0.

    ``metadata`` carries the free ``camber``, ``deflection_to_flat`` (the
    travel before the pack goes flat), the estimated ``rate_n_per_mm`` from
    the classic semi-elliptic formula ``n*E*b*t^3 / (6*L^3)`` at the supplied
    ``modulus_mpa``, and the resulting ``load_to_flat_n``. Those numbers
    assume an isotropic solid; a printed leaf is weaker across layers.

    Print the leaves flat, profile down on the bed, so the layer plane is the
    bending plane and the eye bores come out vertical with no overhang. Print
    the clamp on its end, band axis vertical. PETG or TPU; PLA leaves crack.
    Units are mm and degrees.
    """
    leaves = int(round(leaves))
    if span <= 0 or leaves < 1 or width <= 0:
        raise ValueError("leaf spring: span, leaves and width must be positive")
    if leaf_t < 0.8:
        raise ValueError("leaf spring: leaf_t below the 0.8 mm printable minimum")
    if leaf_gap < 0.4:
        raise ValueError("leaf spring: leaf_gap below the 0.4 mm printable kerf")
    if not 0.0 < camber < span / 4.0:
        raise ValueError("leaf spring: camber must be within (0, span/4)")
    if not 0.2 <= taper <= 1.0:
        raise ValueError("leaf spring: taper must be within [0.2, 1.0]")
    if clamp_w <= 0 or clamp_wall < 1.2 or clamp_clearance < 0.2:
        raise ValueError("leaf spring: clamp dimensions too small")
    if eye_d <= 0 or eye_wall < 1.2:
        raise ValueError("leaf spring: eye bore or eye wall too small")
    sections = max(24, int(round(sections)))

    half = span / 2.0
    radius = (half ** 2 + camber ** 2) / (2.0 * camber)
    if radius - (leaves - 1) * (leaf_t + leaf_gap) <= half:
        raise ValueError("leaf spring: too many leaves for this camber")
    phi = math.asin(half / radius)
    centre_y = camber - radius

    polys = []
    for k in range(leaves):
        r_k = radius - k * (leaf_t + leaf_gap)
        f_k = 1.0 if leaves == 1 else 1.0 - (1.0 - taper) * k / (leaves - 1.0)
        ang = np.linspace(-phi * f_k, phi * f_k, sections)
        curve = np.c_[r_k * np.sin(ang), centre_y + r_k * np.cos(ang)]
        if abs(curve[-1, 0]) < clamp_w / 2.0 + 2.0:
            raise ValueError("leaf spring: shortest leaf does not clear the clamp")
        polys.append(sg.LineString(curve).buffer(leaf_t / 2.0, cap_style=2,
                                                 join_style=1))
    eye_r = eye_d / 2.0 + eye_wall
    main = unary_union([polys[0],
                        sg.Point(-half, 0.0).buffer(eye_r, resolution=24),
                        sg.Point(half, 0.0).buffer(eye_r, resolution=24)])
    polys[0] = main.difference(unary_union([
        sg.Point(-half, 0.0).buffer(eye_d / 2.0, resolution=24),
        sg.Point(half, 0.0).buffer(eye_d / 2.0, resolution=24)])).buffer(0)

    band = sg.box(-clamp_w / 2.0, -1e4, clamp_w / 2.0, 1e4)
    pack = unary_union(polys).intersection(band)
    _, pack_lo, _, pack_hi = pack.bounds
    win_lo, win_hi = pack_lo - clamp_clearance, pack_hi + clamp_clearance
    if win_hi - win_lo < 2.0 * eye_r:
        raise ValueError(
            "leaf spring: clamp window %.1f mm cannot pass over the %.1f mm eye; "
            "shrink eye_d/eye_wall or add leaves" % (win_hi - win_lo, 2.0 * eye_r))
    window = sg.box(-clamp_clearance, win_lo, width + clamp_clearance, win_hi)
    clamp_poly = sg.box(-clamp_clearance - clamp_wall, win_lo - clamp_wall,
                        width + clamp_clearance + clamp_wall,
                        win_hi + clamp_wall).difference(window)
    clamp = _extrude(clamp_poly, clamp_w)
    clamp.apply_transform(tf.rotation_matrix(-math.pi / 2.0, (0, 1, 0)))
    clamp.apply_translation((clamp_w / 2.0, 0.0, 0.0))

    rate = (leaves * modulus_mpa * width * leaf_t ** 3) / (6.0 * span ** 3)
    meta = {
        "span": float(span),
        "leaves": int(leaves),
        "leaf_t": float(leaf_t),
        "camber": float(camber),
        "arc_radius": float(radius),
        "deflection_to_flat": float(camber),
        "rate_n_per_mm": float(rate),
        "load_to_flat_n": float(rate * camber),
        "leaf_gap": float(leaf_gap),
    }
    parts = {"clamp": clamp}
    for k, poly in enumerate(polys):
        parts["leaf_%d" % (k + 1)] = _extrude(poly, width)
    for part in parts.values():
        part.metadata.update(meta)
    return parts


def flexure_stage(travel=2.0, blade_t=1.0, blade_len=25.0, width=60.0,
                  compound=True, stage_w=16.0, stage_h=12.0, wall=4.0,
                  clearance=1.0, thickness=4.0, root_fillet=0.6,
                  max_strain=0.01, modulus_mpa=2000.0):
    """Build a monolithic compound parallelogram flexure stage (straight line).

    Two rigid ground columns rise from a base rail and carry the outer blade
    pair up to a secondary bar; a second blade pair of the same ``blade_len``
    hangs back down from that bar to the motion stage, which floats between
    the columns. Because the second parallelogram is reversed, the parasitic
    arc drop of the inner pair cancels the drop of the outer pair, so the
    motion stage translates along X in a straight line with zero backlash and
    zero friction instead of following an arc. ``compound=False`` builds the
    single parallelogram for comparison and reports its uncancelled parasitic
    motion. The base rail bottom sits at y=0 and the part is extruded
    ``thickness`` deep from z=0.

    Blades are the failure point, so the peak bending strain
    ``3 * blade_t * delta / blade_len^2`` for the per-stage deflection is
    computed, stored in ``metadata`` as ``peak_strain`` and compared against
    ``max_strain`` (0.01 is a conservative repeated-flexure limit for PLA and
    PETG; TPU tolerates far more). Above the limit the build raises
    ValueError rather than shipping a stage that snaps on its first stroke.
    ``root_fillet`` rounds the blade roots and is clamped so it can never
    close the moving gaps.

    Print flat, profile down on the bed, with in-plane motion: the layer plane
    is then the bending plane and no blade sees an interlayer tensile load.
    Keep ``blade_t`` at or above 0.8 mm. Units are mm and degrees.
    """
    if travel <= 0 or blade_len <= 0 or width <= 0 or stage_w <= 0 or stage_h <= 0:
        raise ValueError(
            "flexure stage: travel, blade and stage sizes must be positive")
    if blade_t < 0.8:
        raise ValueError("flexure stage: blade_t below the 0.8 mm printable minimum")
    if thickness < 1.2 or wall < 1.2:
        raise ValueError("flexure stage: thickness or wall below 1.2 mm")
    if clearance < 0.4:
        raise ValueError("flexure stage: clearance below the 0.4 mm printable kerf")
    if root_fillet < 0:
        raise ValueError("flexure stage: root_fillet must not be negative")
    if compound and stage_w / 2.0 + travel + clearance > width / 2.0 - wall:
        raise ValueError(
            "flexure stage: stage plus %.1f mm travel does not clear the ground "
            "columns; widen `width` or shrink `stage_w`" % travel)

    stages = 2 if compound else 1
    stage_travel = travel / float(stages)
    peak_strain = 3.0 * blade_t * stage_travel / blade_len ** 2
    if peak_strain > max_strain:
        raise ValueError(
            "flexure stage: peak blade strain %.4f exceeds the %.4f limit; "
            "lengthen blade_len, thin blade_t or cut travel"
            % (peak_strain, max_strain))

    outer_x = width / 2.0 - wall / 2.0
    inner_x = stage_w / 2.0 - blade_t / 2.0
    if outer_x - blade_t / 2.0 < inner_x + blade_t / 2.0 + 2.0:
        raise ValueError("flexure stage: blade pairs collide; widen `width`")
    col_h = wall + stage_h + clearance
    top_y = col_h + blade_len

    solids = [
        sg.box(-width / 2.0, 0.0, width / 2.0, wall),
        sg.box(-width / 2.0, 0.0, -width / 2.0 + wall, col_h),
        sg.box(width / 2.0 - wall, 0.0, width / 2.0, col_h),
    ]
    for sign in (-1.0, 1.0):
        solids.append(sg.box(sign * outer_x - blade_t / 2.0, col_h,
                             sign * outer_x + blade_t / 2.0, top_y))
    if compound:
        solids.append(sg.box(-width / 2.0, top_y, width / 2.0, top_y + wall))
        for sign in (-1.0, 1.0):
            solids.append(sg.box(sign * inner_x - blade_t / 2.0, col_h,
                                 sign * inner_x + blade_t / 2.0, top_y))
        solids.append(sg.box(-stage_w / 2.0, col_h - stage_h,
                             stage_w / 2.0, col_h))
        stage_box = (-stage_w / 2.0, col_h - stage_h, stage_w / 2.0, col_h)
    else:
        solids.append(sg.box(-width / 2.0, top_y, width / 2.0,
                             top_y + stage_h))
        stage_box = (-width / 2.0, top_y, width / 2.0, top_y + stage_h)

    fillet = min(root_fillet, 0.4 * clearance)
    profile = unary_union(solids).buffer(0)
    if fillet > 0.01:
        profile = profile.buffer(fillet, join_style=1).buffer(
            -fillet, join_style=1).buffer(0)
    mesh = _extrude(profile, thickness)
    inertia = thickness * blade_t ** 3 / 12.0
    stage_rate = 24.0 * modulus_mpa * inertia / blade_len ** 3
    mesh.metadata.update({
        "travel": float(travel),
        "stage_travel": float(stage_travel),
        "stages": int(stages),
        "compound": bool(compound),
        "blade_t": float(blade_t),
        "blade_len": float(blade_len),
        "blades": int(4 if compound else 2),
        "peak_strain": float(peak_strain),
        "max_strain": float(max_strain),
        "parasitic_mm": float(0.0 if compound
                              else 3.0 * stage_travel ** 2 / (5.0 * blade_len)),
        "rate_n_per_mm": float(stage_rate / stages),
        "root_fillet": float(fillet),
        "stage_box": tuple(float(v) for v in stage_box),
    })
    return mesh


__all__ = (
    "cross_flexure",
    "wave_spring",
    "bistable_beam",
    "belleville_washer",
    "coil_spring",
    "spiral_power_spring",
    "leaf_spring",
    "flexure_stage",
)
