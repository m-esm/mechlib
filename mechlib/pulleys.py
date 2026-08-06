"""Project-agnostic belt-pulley and grooved cable-drum generators."""

import math

import numpy as np
import shapely.geometry as sg
from shapely.ops import unary_union
import trimesh

from .meshutil import from_manifold, sub, to_manifold, uni
from .prim import cyl
from .sweep import loft


def _polar(r, angle):
    return r * math.cos(angle), r * math.sin(angle)


def _extrude(poly, height, z0=0.0):
    if height <= 0:
        raise ValueError("extrusion height must be positive")
    mesh = trimesh.creation.extrude_polygon(poly, height)
    if not mesh.is_watertight:
        mesh = from_manifold(to_manifold(mesh))
    if z0:
        mesh.apply_translation((0.0, 0.0, z0))
    return mesh


def _sweep_tube(points, wire_r, ring=10):
    """Sweep a circular-section capped tube along a 3D point path.

    The moving frame is derived from the path tangent and the radial-outward
    direction, so it never twists around the axis of a drum-style helix.
    """
    path = np.asarray(points, dtype=float)
    n = len(path)
    if n < 3 or wire_r <= 0:
        raise ValueError("invalid tube path or wire radius")
    tangent = np.gradient(path, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    outward = path.copy()
    outward[:, 2] = 0.0
    short = np.linalg.norm(outward, axis=1) < 1e-9
    outward[short] = (1.0, 0.0, 0.0)
    normal = outward - (outward * tangent).sum(axis=1)[:, None] * tangent
    normal /= np.linalg.norm(normal, axis=1)[:, None]
    binormal = np.cross(tangent, normal)
    angles = np.linspace(0.0, 2.0 * np.pi, ring, endpoint=False)
    verts = (path[:, None, :]
             + wire_r * (np.cos(angles)[None, :, None] * normal[:, None, :]
                         + np.sin(angles)[None, :, None] * binormal[:, None, :]))
    verts = verts.reshape(n * ring, 3)
    faces = []
    for i in range(n - 1):
        for j in range(ring):
            j2 = (j + 1) % ring
            a0, b0 = i * ring + j, i * ring + j2
            a1, b1 = (i + 1) * ring + j, (i + 1) * ring + j2
            faces.append((a0, b0, b1))
            faces.append((a0, b1, a1))
    fan = [(0, j + 1, j) for j in range(1, ring - 1)]
    faces += fan
    off = (n - 1) * ring
    faces += [(off, off + j, off + j + 1) for j in range(1, ring - 1)]
    mesh = trimesh.Trimesh(vertices=verts, faces=np.array(faces), process=False)
    mesh.fix_normals()
    return mesh


def timing_pulley(teeth=20, pitch=2.0, belt_w=6.0, bore_d=5.0,
                  hub_d=12.0, hub_len=7.0, flanges=True, flange_t=1.2,
                  flange_extra=1.5, setscrew_boss=False, setscrew_d=3.0,
                  clearance=0.25, sections=96):
    """Build a GT2-style synchronous belt pulley (axis along +Z).

    The tooth form uses the circular-arc tooth-space approximation of the GT2
    curvilinear profile: each tooth space is a circular arc of radius
    ``0.555 * pitch/2`` cutting ``0.75 * pitch/2`` below the pitch line, scaled
    from the GT2 2M specification. The pitch diameter is exactly
    ``teeth * pitch / pi`` and is stored in ``mesh.metadata["pitch_d"]`` along
    with ``tip_d``. With ``flanges`` the belt is retained between two discs of
    ``flange_t`` thickness; ``hub_d``/``hub_len`` add a one-sided hub and
    ``setscrew_boss`` adds a radial boss pierced with a ``setscrew_d`` hole
    into the bore. The bore is printed at ``bore_d + clearance`` for an
    easy shaft fit. Units are mm; ``sections`` is the circle resolution.
    """
    if (teeth < 8 or pitch <= 0 or belt_w < 3.0 or bore_d <= 0 or
            hub_d <= 0 or hub_len < 0 or flange_t < 1.2 or
            flange_extra <= 0 or setscrew_d <= 0 or clearance < 0 or
            sections < 24):
        raise ValueError("invalid timing pulley dimensions")
    if hub_len > 0 and hub_d < bore_d + clearance + 2.4:
        raise ValueError("hub wall below 1.2 mm around the bore")
    scale = pitch / 2.0
    pitch_r = teeth * pitch / (2.0 * math.pi)
    space_r = 0.555 * scale
    depth = 0.75 * scale
    tip_r = pitch_r - 0.254 * scale
    if tip_r - depth <= (bore_d + clearance) / 2.0 + 1.0:
        raise ValueError("tooth root too close to the bore")
    teeth = int(round(teeth))
    sections = int(round(sections))

    blank = sg.Point(0, 0).buffer(tip_r, resolution=sections)
    center_r = pitch_r - depth + space_r
    spaces = unary_union([
        sg.Point(*_polar(center_r, 2.0 * math.pi * k / teeth)).buffer(
            space_r, resolution=24)
        for k in range(teeth)
    ])
    profile = blank.difference(spaces).buffer(0)

    band_z0 = flange_t if flanges else 0.0
    parts = [_extrude(profile, belt_w, band_z0)]
    if flanges:
        flange_r = tip_r + flange_extra
        parts.append(cyl(flange_r, flange_t,
                         center=(0, 0, flange_t / 2.0), sections=sections))
        parts.append(cyl(flange_r, flange_t,
                         center=(0, 0, band_z0 + belt_w + flange_t / 2.0),
                         sections=sections))
    if hub_len > 0:
        parts.append(cyl(hub_d / 2.0, hub_len + 0.4,
                         center=(0, 0, -hub_len / 2.0 + 0.2),
                         sections=sections))
    holes = []
    if setscrew_boss:
        if hub_len <= 0:
            raise ValueError("setscrew_boss requires hub_len > 0")
        boss_r = setscrew_d / 2.0 + 1.2
        boss_len = 3.0
        boss = cyl(boss_r, hub_d / 2.0 + boss_len + 1.0, axis="x",
                   center=(hub_d / 4.0 + boss_len / 2.0 - 0.5, 0,
                           -hub_len / 2.0), sections=32)
        parts.append(boss)
        holes.append(cyl(setscrew_d / 2.0, hub_d / 2.0 + boss_len + 2.0,
                         axis="x",
                         center=(hub_d / 4.0 + boss_len / 2.0, 0,
                                 -hub_len / 2.0), sections=24))
    solid = uni(parts)
    total_h = band_z0 + belt_w + (flange_t if flanges else 0.0) + hub_len
    holes.append(cyl((bore_d + clearance) / 2.0, total_h + 4.0,
                     center=(0, 0, (band_z0 + belt_w +
                                    (flange_t if flanges else 0.0) -
                                    hub_len) / 2.0),
                     sections=sections))
    for hole in holes:
        solid = sub(solid, hole)
    solid.metadata["pitch_d"] = teeth * pitch / math.pi
    solid.metadata["tip_d"] = 2.0 * tip_r
    return solid


def _winding_radius(radius_law, fraction, core_r, radius_rise):
    """Cable-centerline winding radius at winding fraction ``fraction``."""
    if radius_law == "cylinder":
        return core_r
    if radius_law == "cone":
        return core_r + radius_rise * fraction
    if radius_law == "fusee":
        end_r = core_r + radius_rise
        return core_r * end_r / (end_r - radius_rise * fraction)
    raise ValueError("radius_law must be 'cylinder', 'cone', or 'fusee'")


def grooved_drum(radius_law="cylinder", turns=8.0, cable_d=3.0,
                 core_r=10.0, radius_rise=6.0, groove_factor=1.25,
                 flange_extra=1.5, flange_t=1.6, bore_d=6.0,
                 clearance=0.25, sections=64):
    """Build a helically grooved cable drum (axis along +Z).

    ``radius_law`` sets how the winding (cable-centerline) radius varies from
    the first to the last turn: ``"cylinder"`` keeps it at ``core_r`` (plain
    winch drum), ``"cone"`` ramps it linearly by ``radius_rise``, and
    ``"fusee"`` follows the classic hyperbolic cone law (507 Mechanical
    Movements No. 46) that compensates a linearly weakening mainspring. The
    helical groove seats the cable 0.35*cable_d deep with a groove pitch of
    ``cable_d * groove_factor`` so adjacent turns keep a printable wall. The
    grooved body height is exactly ``turns * groove_pitch``; flanges of
    ``flange_t`` retain the cable at both ends and the bore is printed at
    ``bore_d + clearance``. Winding data (``groove_pitch``, ``body_height``,
    ``r_start``, ``r_end``) is stored in ``mesh.metadata``. Units are mm.
    """
    if (turns < 1.0 or cable_d <= 0 or core_r <= 0 or radius_rise < 0 or
            groove_factor < 1.15 or flange_extra <= 0 or flange_t < 1.2 or
            bore_d <= 0 or clearance < 0 or sections < 24):
        raise ValueError("invalid grooved drum dimensions")
    groove_pitch = cable_d * groove_factor
    height = turns * groove_pitch
    seat = 0.35 * cable_d
    bore_r = (bore_d + clearance) / 2.0
    r_end = _winding_radius(radius_law, 1.0, core_r, radius_rise)
    if core_r - seat - bore_r < 1.2:
        raise ValueError("drum wall below 1.2 mm at the first turn")
    sections = int(round(sections))

    # Body: lofted surface of revolution of the drum surface radius law.
    n_rings = max(9, int(turns * 4) + 1)
    angles = np.linspace(0.0, 2.0 * np.pi, sections, endpoint=False)
    rings = []
    for i in range(n_rings):
        fraction = i / (n_rings - 1.0)
        surface_r = (_winding_radius(radius_law, fraction, core_r, radius_rise)
                     - seat)
        z = fraction * height
        rings.append(np.c_[surface_r * np.cos(angles),
                           surface_r * np.sin(angles),
                           np.full(sections, z)])
    body = loft([np.asarray(ring) for ring in rings])

    # Groove cutter: a circular-section tube swept along the winding helix.
    wire_r = cable_d / 2.0 + 0.15
    n_pts = max(64, int(turns * sections))
    theta = np.linspace(0.0, turns * 2.0 * np.pi, n_pts)
    fractions = theta / (turns * 2.0 * np.pi)
    radii = np.array([_winding_radius(radius_law, f, core_r, radius_rise)
                      for f in fractions])
    helix = np.c_[radii * np.cos(theta), radii * np.sin(theta),
                  fractions * height]
    groove = _sweep_tube(helix, wire_r, ring=10)
    drum = sub(body, groove)

    flange_r = r_end + 0.85 * cable_d + flange_extra
    flanges = [
        cyl(flange_r, flange_t, center=(0, 0, -flange_t / 2.0),
            sections=sections),
        cyl(flange_r, flange_t, center=(0, 0, height + flange_t / 2.0),
            sections=sections),
    ]
    drum = uni([drum] + flanges)
    bore = cyl(bore_r, height + 2.0 * flange_t + 4.0,
               center=(0, 0, height / 2.0), sections=sections)
    drum = sub(drum, bore)
    drum.metadata["groove_pitch"] = groove_pitch
    drum.metadata["body_height"] = height
    drum.metadata["r_start"] = core_r
    drum.metadata["r_end"] = r_end
    return drum


__all__ = (
    "timing_pulley",
    "grooved_drum",
)
