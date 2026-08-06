"""Project-agnostic compliant-mechanism generators (print-in-place parts)."""

import math

import numpy as np
import shapely.geometry as sg
from shapely.ops import unary_union
import trimesh
import trimesh.transformations as tf

from .meshutil import from_manifold, to_manifold, uni
from .prim import boxc


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


__all__ = (
    "cross_flexure",
    "wave_spring",
    "bistable_beam",
)
