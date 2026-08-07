"""Project-agnostic belt-pulley and grooved cable-drum generators."""

import math

import numpy as np
import shapely.geometry as sg
from shapely.geometry.polygon import orient
from shapely.ops import unary_union
import trimesh

from .closures import setscrew
from .cutters import bearing_seat
from .meshutil import from_manifold, sub, to_manifold, uni
from .prim import boxc, cyl, hex_poly
from .sweep import loft


def _polar(r, angle):
    return r * math.cos(angle), r * math.sin(angle)


def _revolve(poly, sections=96):
    """Revolve a closed (r, z) profile polygon about the Z axis."""
    ring = orient(poly, 1.0)
    return trimesh.creation.revolve(np.asarray(ring.exterior.coords),
                                    sections=int(sections))


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


def idler_pulley(od=16.0, width=8.0, bore_d=5.0, crown=0.15,
                 flanges=True, flange_t=1.2, flange_extra=1.5,
                 belt_clearance=0.3, toothed=False, teeth=20, pitch=2.0,
                 bearing=None, bearing_fit="press", clearance=0.25,
                 sections=64):
    """Build a smooth or toothed idler pulley for tensioning a belt (axis along +Z).

    The default smooth idler rides the back of a flat or toothed belt on a
    shallow parabolic ``crown`` (peak radius ``od/2 + crown`` at mid-width,
    tapering to ``od/2`` at the edges, the classic conveyor-idler crown that
    self-centres a flat belt under tension). With ``toothed=True`` the body
    is a real ``timing_pulley`` instead (its GT2 tooth generation is reused
    directly, not reimplemented), so the idler meshes with a synchronous
    belt rather than rubbing it; ``crown`` is ignored in that case. With
    ``flanges`` the belt is retained between two discs of ``flange_t``
    thickness; for the smooth idler their inner faces are spaced
    ``width + belt_clearance`` apart, for the toothed idler exactly
    ``width`` apart (pass ``width`` already inclusive of any desired float,
    matching ``timing_pulley``'s ``belt_w``). The bore is a plain
    ``bore_d + clearance`` hole by default; pass ``bearing`` as ``"608"``,
    ``"695"``, or ``"MR105"`` to cut a ``cutters.bearing_seat`` pocket
    through the idler instead (``bearing_fit`` selects ``"press"`` or
    ``"slip"``). The effective belt-contact diameter -- the crown peak for a
    smooth idler, the pitch diameter for a toothed one -- is stored in
    ``mesh.metadata["belt_contact_d"]``. Print with the bore axis vertical;
    the crown and teeth need no support. Units are mm.
    """
    if (od <= 0 or width <= 0 or bore_d <= 0 or crown < 0 or flange_t < 1.2 or
            flange_extra <= 0 or belt_clearance < 0 or teeth < 8 or pitch <= 0 or
            clearance < 0 or sections < 24):
        raise ValueError("invalid idler pulley dimensions")
    if not toothed:
        if crown <= 0 and not flanges:
            raise ValueError(
                "idler_pulley(): a flat idler needs flanges or a crown to "
                "retain the belt")
        if crown > od / 4.0:
            raise ValueError("idler_pulley(): crown too large for the outer diameter")
        if od / 2.0 - (bore_d + clearance) / 2.0 < 1.2:
            raise ValueError("idler_pulley(): wall below 1.2 mm around the bore")

    sections = int(round(sections))

    if toothed:
        body = timing_pulley(teeth=teeth, pitch=pitch, belt_w=width,
                             bore_d=bore_d, hub_d=max(bore_d + 4.0, 8.0),
                             hub_len=0.0, flanges=flanges, flange_t=flange_t,
                             flange_extra=flange_extra, clearance=clearance,
                             sections=sections)
        contact_d = body.metadata["pitch_d"]
        total_h = width + (2.0 * flange_t if flanges else 0.0)
    else:
        span = width + (belt_clearance if flanges else 0.0)
        z0 = flange_t if flanges else 0.0
        n_rings = 9
        angles = np.linspace(0.0, 2.0 * np.pi, sections, endpoint=False)
        rings = []
        for i in range(n_rings):
            f = i / (n_rings - 1.0)
            r = od / 2.0 + crown * (1.0 - (2.0 * f - 1.0) ** 2)
            z = z0 + f * span
            rings.append(np.c_[r * np.cos(angles), r * np.sin(angles),
                               np.full(sections, z)])
        body = loft(rings)
        parts = [body]
        if flanges:
            flange_r = od / 2.0 + crown + flange_extra
            parts.append(cyl(flange_r, flange_t,
                             center=(0, 0, flange_t / 2.0), sections=sections))
            parts.append(cyl(flange_r, flange_t,
                             center=(0, 0, z0 + span + flange_t / 2.0),
                             sections=sections))
        body = uni(parts)
        contact_d = od + 2.0 * crown
        total_h = z0 + span + (flange_t if flanges else 0.0)
        bore = cyl((bore_d + clearance) / 2.0, total_h + 4.0,
                   center=(0, 0, total_h / 2.0), sections=sections)
        body = sub(body, bore)

    pocket_d = None
    if bearing is not None:
        seat = bearing_seat(bearing, fit=bearing_fit, open_column=True)
        seat_h = seat.bounds[1][2] - seat.bounds[0][2]
        if seat_h < total_h:
            seat = bearing_seat(bearing, fit=bearing_fit, open_column=True,
                                extra_depth=total_h - seat_h)
        pocket_d = float(seat.bounds[1][0] - seat.bounds[0][0])
        body = sub(body, seat)

    body.metadata["belt_contact_d"] = float(contact_d)
    body.metadata["toothed"] = bool(toothed)
    if pocket_d is not None:
        body.metadata["bearing_pocket_d"] = pocket_d
    return body


def eccentric_idler_mount(eccentricity=1.5, bushing_od=14.0, post_d=5.0,
                          height=10.0, rotation_deg=0.0, drive_af=6.0,
                          setscrew_d=3.0, clearance=0.25, idler_od=16.0,
                          idler_width=8.0, sections=64):
    """Build an eccentric take-up bushing plus the idler pulley it carries.

    The bushing's outer cylinder (``bushing_od``) seats concentrically in a
    fixed frame bore; its through-bore for the idler's shaft or an
    ``idler_pulley`` post is offset ``eccentricity`` off that outer axis.
    Turning the bushing (a hex drive recess, ``drive_af`` across flats, is
    cut into its top face on the outer axis so a key can turn the whole part
    from the frame side) sweeps the offset bore -- and the ``idler_pulley``
    riding it -- through a circle of radius ``eccentricity``, so the total
    belt-tension adjustment is ``2 * eccentricity`` end to end; that range is
    stored in ``mesh.metadata["adjustment_range"]``. ``rotation_deg`` is the
    bushing's current turned angle. A radial ``closures.setscrew`` boss,
    aimed at the offset bore regardless of ``rotation_deg``, locks the
    idler's post once tension is set. Print with the outer axis vertical.
    Units are mm and degrees.
    """
    if (eccentricity <= 0 or bushing_od <= 0 or post_d <= 0 or height <= 0 or
            drive_af <= 0 or setscrew_d <= 0 or clearance < 0 or
            idler_od <= 0 or idler_width <= 0 or sections < 24):
        raise ValueError("invalid eccentric idler mount dimensions")
    post_r = (post_d + clearance) / 2.0
    if bushing_od / 2.0 - eccentricity - post_r < 1.5:
        raise ValueError(
            "eccentric_idler_mount(): eccentricity leaves too little wall "
            "around the offset post bore")
    if 2.0 * drive_af / math.sqrt(3.0) > bushing_od - 2.0:
        raise ValueError(
            "eccentric_idler_mount(): hex drive too large for the bushing OD")

    sections = int(round(sections))
    rot = math.radians(rotation_deg)
    off = (eccentricity * math.cos(rot), eccentricity * math.sin(rot))

    bushing = cyl(bushing_od / 2.0, height, center=(0, 0, height / 2.0),
                 sections=sections)
    hex_depth = min(4.0, height * 0.4)
    hex_cut = trimesh.creation.extrude_polygon(hex_poly(drive_af), hex_depth)
    hex_cut.apply_translation((0, 0, height - hex_depth))
    bushing = sub(bushing, hex_cut)
    post_bore = cyl(post_r, height + 4.0,
                    center=(off[0], off[1], height / 2.0), sections=sections)
    bushing = sub(bushing, post_bore)

    off_len = math.hypot(off[0], off[1])
    off_dir = ((off[0] / off_len, off[1] / off_len) if off_len > 1e-9
              else (1.0, 0.0))
    ss_point = np.array([off_dir[0] * bushing_od / 2.0,
                         off_dir[1] * bushing_od / 2.0, height / 2.0])
    ss_dir = np.array([-off_dir[0], -off_dir[1], 0.0])
    boss, hole = setscrew(ss_point, ss_dir,
                          into=(bushing_od / 2.0 - eccentricity) + post_r,
                          hole_d=setscrew_d, boss_d=setscrew_d + 5.0,
                          boss_h=3.0, sections=32)
    bushing = sub(uni([bushing, boss]), hole)

    pulley = idler_pulley(od=idler_od, width=idler_width, bore_d=post_d,
                          clearance=clearance, sections=sections)
    pulley.apply_translation((off[0], off[1], height))

    bushing.metadata["eccentricity"] = float(eccentricity)
    bushing.metadata["adjustment_range"] = float(2.0 * eccentricity)
    bushing.metadata["axis_offset"] = (float(off[0]), float(off[1]))
    return {"bushing": bushing, "pulley": pulley}


def belt_tensioner(arm_len=30.0, sweep_deg=50.0, beam_t=1.4, beam_w=6.0,
                   preload_mm=2.5, mount_w=14.0, mount_d=8.0,
                   idler_bore_d=5.0, boss_d=11.0, clearance=0.25,
                   max_strain=0.015, sections=48):
    """Build a compliant-arm belt tensioner: a curved cantilever spring idler mount.

    A rigid mounting block anchors one end of a curved cantilever blade of
    developed length ``arm_len`` and in-plane thickness ``beam_t`` (extruded
    ``beam_w`` mm out of plane, and swept through ``sweep_deg`` of arc so the
    tip lands off to the side rather than straight out), the same arc-beam
    idiom used by ``ratchets.arc_ratchet_2d``'s pawl arms and ``flexures``'
    blade construction. The tip carries a boss with an
    ``idler_bore_d + clearance`` through-bore for an idler shaft or an
    ``eccentric_idler_mount`` post. Mounted so its free tip must be pushed
    back ``preload_mm`` to reach the belt line, the beam preloads the idler
    against the belt with no metal spring and keeps re-deflecting to take up
    belt stretch. The estimated tip deflection and peak bending strain
    (``3 * beam_t * preload_mm / (2 * arm_len**2)``, the standard cantilever
    estimate) are stored in ``mesh.metadata``; this raises rather than
    shipping a part that snaps if that strain exceeds ``max_strain`` (about
    0.015 for PETG, closer to 0.01 for PLA -- pass a lower ``max_strain`` for
    PLA). Print flat with the mount face down: the beam bends in the print
    plane, so no support is needed. Units are mm.
    """
    if (arm_len <= 0 or not 5.0 <= sweep_deg <= 170.0 or beam_t <= 0 or
            beam_w <= 0 or preload_mm < 0 or mount_w <= 0 or mount_d <= 0 or
            idler_bore_d <= 0 or boss_d <= 0 or clearance < 0 or
            max_strain <= 0 or sections < 24):
        raise ValueError("invalid belt tensioner dimensions")
    if arm_len < boss_d:
        raise ValueError("belt_tensioner(): arm_len too short for the tip boss")
    if boss_d / 2.0 - (idler_bore_d + clearance) / 2.0 < 1.2:
        raise ValueError(
            "belt_tensioner(): boss wall below 1.2 mm around the idler bore")
    strain = 3.0 * beam_t * preload_mm / (2.0 * arm_len ** 2)
    if strain > max_strain:
        raise ValueError(
            "belt_tensioner(): preload_mm=%.2f over arm_len=%.1f needs "
            "%.2f%% strain, over the %.2f%% limit; lower preload_mm or "
            "lengthen arm_len" % (preload_mm, arm_len, 100.0 * strain,
                                  100.0 * max_strain))

    sections = int(round(sections))
    arc_r = arm_len / math.radians(sweep_deg)
    n = max(12, int(sweep_deg / 3.0))
    thetas = np.linspace(-90.0, -90.0 + sweep_deg, n)
    centerline = [(arc_r * math.cos(math.radians(t)),
                  arc_r + arc_r * math.sin(math.radians(t))) for t in thetas]
    beam = sg.LineString(centerline).buffer(
        beam_t / 2.0, cap_style=2, join_style=1)

    mount = sg.box(-mount_d, -mount_w / 2.0, beam_t / 2.0, mount_w / 2.0)
    tip_x, tip_y = centerline[-1]
    boss = sg.Point(tip_x, tip_y).buffer(boss_d / 2.0, resolution=32)

    profile = unary_union([mount, beam, boss]).buffer(0)
    body = _extrude(profile, beam_w)
    bore = cyl((idler_bore_d + clearance) / 2.0, beam_w + 2.0,
              center=(tip_x, tip_y, beam_w / 2.0), sections=sections)
    body = sub(body, bore)

    body.metadata["preload_deflection_mm"] = float(preload_mm)
    body.metadata["peak_strain"] = float(strain)
    body.metadata["tip_xy"] = (float(tip_x), float(tip_y))
    return body


# Belt top width, groove depth and total flank angle per classic V-belt
# section (3L, A, B); the pitch line sits PITCH_FRAC of the depth above the
# groove root.
_V_SECTIONS = {
    "3L": (9.7, 6.0, 36.0),
    "A": (12.7, 8.0, 34.0),
    "B": (16.3, 10.5, 34.0),
}
_V_PITCH_FRAC = 0.7


def v_belt_pulley(section="3L", pitch_d=60.0, grooves=1, bore_d=8.0,
                  clear=0.2, hub="B", hub_d=None, hub_len=8.0,
                  keyway=False, setscrew_d=3.0, sections=96):
    """Build a trapezoidal-groove V-belt pulley (sheave), axis along +Z.

    The classic wedge-belt pulley of washing machines, drill presses, lathes
    and HVAC blowers: a rim carrying ``grooves`` trapezoidal grooves for
    ``section`` belts (``"3L"``, ``"A"`` or ``"B"``; top width, depth and
    flank angle from the standard section table), a web, and a hub. ``hub``
    selects the stock hub style: ``"A"`` is a flat plate with no hub,
    ``"B"`` adds a one-sided hub of ``hub_d`` x ``hub_len`` below the rim,
    ``"C"`` carries the hub on both faces. The groove pitch line sits at
    ``pitch_d``; the outer diameter is ``pitch_d + 2*(1-PITCH_FRAC)*depth``.
    The bore is printed at ``bore_d + clear``. With ``keyway`` a 3 mm
    keyway slot is broached along the bore; with hub style ``"B"``/``"C"`` a
    radial ``setscrew_d`` boss (via ``closures.setscrew``) pierces the hub
    into the bore. The rim sits on z=0 with the hub extending below it for
    hub styles ``"B"``/``"C"``; print hub-face down. The standard wedge
    flank angle (34-38 degrees) is shallower than the 45 degree FDM rule on
    the grooves' upper flanks, so print at 0.12-0.2 mm layers; the short
    flanks bridge cleanly at those layer heights. Units are mm and
    degrees.
    """
    if section not in _V_SECTIONS:
        raise ValueError("v_belt_pulley(): section must be one of %s"
                         % (sorted(_V_SECTIONS),))
    if pitch_d <= 0 or bore_d <= 0 or clear < 0 or hub_len < 0:
        raise ValueError("v_belt_pulley(): pitch_d and bore_d must be "
                         "positive, clear and hub_len non-negative")
    if hub not in ("A", "B", "C"):
        raise ValueError("v_belt_pulley(): hub must be 'A', 'B' or 'C'")
    grooves = int(round(grooves))
    if grooves < 1:
        raise ValueError("v_belt_pulley(): grooves must be at least 1")
    if hub == "A":
        hub_len = 0.0

    belt_w, depth, angle = _V_SECTIONS[section]
    pitch_r = pitch_d / 2.0
    root_r = pitch_r - _V_PITCH_FRAC * depth
    outer_r = root_r + depth
    bore_r = (bore_d + clear) / 2.0
    if root_r - bore_r < 2.0:
        raise ValueError("v_belt_pulley(): groove root leaves under 2.0 mm "
                         "above the bore; raise pitch_d or lower bore_d")
    half_ang = math.radians(angle / 2.0)
    root_w = belt_w - 2.0 * depth * math.tan(half_ang)
    if root_w < 1.0:
        raise ValueError("v_belt_pulley(): %s groove closes up at the root "
                         "for this depth" % section)

    spacing = belt_w + 3.0
    edge = 2.0
    rim_w = (grooves - 1) * spacing + belt_w + 2.0 * edge

    if hub_d is None:
        hub_d = max(bore_d + clear + 4.8, 2.0 * bore_d)
    if hub in ("B", "C"):
        if hub_d < bore_d + clear + 2.4:
            raise ValueError("v_belt_pulley(): hub wall below 1.2 mm around "
                             "the bore")
        if hub_d / 2.0 >= root_r:
            raise ValueError("v_belt_pulley(): hub_d reaches the groove "
                             "roots")
    if hub_len > 0 and hub == "A":
        raise ValueError("v_belt_pulley(): hub='A' takes no hub_len")

    # Rim block from z=0 to rim_w, hub(s) outside it.
    parts = [cyl(outer_r, rim_w, center=(0, 0, rim_w / 2.0),
                 sections=int(sections))]
    if hub in ("B", "C") and hub_len > 0:
        parts.append(cyl(hub_d / 2.0, hub_len,
                         center=(0, 0, -hub_len / 2.0),
                         sections=int(sections)))
    if hub == "C" and hub_len > 0:
        parts.append(cyl(hub_d / 2.0, hub_len,
                         center=(0, 0, rim_w + hub_len / 2.0),
                         sections=int(sections)))
    body = uni(parts)

    # Groove cutters: revolved trapezoids opening outward.
    cutters = []
    z_c0 = edge + belt_w / 2.0
    top_w = belt_w + 2.0 * 2.0 * math.tan(half_ang)
    for k in range(grooves):
        zc = z_c0 + k * spacing
        trap = sg.Polygon([
            (root_r - 0.5, zc - root_w / 2.0 - 0.5 * math.tan(half_ang)),
            (root_r - 0.5, zc + root_w / 2.0 + 0.5 * math.tan(half_ang)),
            (outer_r + 2.0, zc + top_w / 2.0),
            (outer_r + 2.0, zc - top_w / 2.0),
        ])
        cutters.append(_revolve(trap, sections=sections))
    body = sub(body, uni(cutters))

    # Bore, optional keyway, optional setscrew into the lower hub.
    total_h = rim_w + (hub_len if hub in ("B", "C") else 0.0) + \
        (hub_len if hub == "C" else 0.0)
    bore_cuts = [cyl(bore_r, total_h + 4.0,
                     center=(0, 0, total_h / 2.0 - (hub_len if hub in ("B", "C") else 0.0)),
                     sections=int(sections))]
    if keyway:
        if root_r - (bore_r + 1.5) < 1.2:
            raise ValueError("v_belt_pulley(): keyway breaks through the "
                             "groove root; raise pitch_d")
        key = boxc((3.0, 3.0, total_h + 4.0),
                   center=(bore_r + 0.5, 0.0,
                           total_h / 2.0 - (hub_len if hub in ("B", "C") else 0.0)))
        bore_cuts.append(key)
    body = sub(body, uni(bore_cuts))

    if hub in ("B", "C") and hub_len > 0 and setscrew_d > 0:
        ss_point = np.array([hub_d / 2.0, 0.0, -hub_len / 2.0])
        ss_dir = np.array([-1.0, 0.0, 0.0])
        boss, hole = setscrew(ss_point, ss_dir,
                              into=hub_d / 2.0, hole_d=setscrew_d,
                              boss_d=setscrew_d + 5.0, boss_h=3.0,
                              sections=32)
        body = sub(uni([body, boss]), hole)

    body.metadata.update({
        "section": section,
        "pitch_d": float(pitch_d),
        "outer_d": float(2.0 * outer_r),
        "grooves": int(grooves),
        "bore_d": float(bore_d + clear),
        "groove_angle_deg": float(angle),
        "rim_w": float(rim_w),
        "belt_line_speed_per_rpm": float(math.pi * pitch_d),
    })
    return body


__all__ = (
    "timing_pulley",
    "grooved_drum",
    "idler_pulley",
    "eccentric_idler_mount",
    "belt_tensioner",
    "v_belt_pulley",
)
