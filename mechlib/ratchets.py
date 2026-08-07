"""Project-agnostic one-way ratchet and compliant-clutch generators."""

import math

import numpy as np
import shapely.affinity as affinity
import shapely.geometry as sg
from shapely.ops import unary_union
import trimesh


def _polar(r, angle):
    return r * math.cos(angle), r * math.sin(angle)


def _largest_polygon(geometry):
    if geometry.geom_type == "Polygon":
        return geometry
    return max(geometry.geoms, key=lambda item: item.area)


def _extrude(poly, height, z0=0.0):
    if height <= 0:
        raise ValueError("extrusion height must be positive")
    mesh = trimesh.creation.extrude_polygon(poly, height)
    if not mesh.is_watertight:
        # Thin accordion combs expose occasional open edges in trimesh's
        # dependency-free triangulator. manifold3d closes the identical solid
        # without requiring the optional ``triangle`` package.
        from .meshutil import from_manifold, to_manifold
        mesh = from_manifold(to_manifold(mesh))
    if z0:
        mesh.apply_translation((0.0, 0.0, z0))
    return mesh


def _ratchet_cavity_2d(teeth, tip_r, root_r, undercut_deg, phase_deg,
                       flip=False, ramp_steps=6):
    """Return the free region inside an undercut internal sawtooth ring."""
    if teeth < 3 or not (0 < tip_r < root_r) or undercut_deg < 0:
        raise ValueError("invalid ratchet tooth geometry")
    pitch = 360.0 / teeth
    dr = root_r - tip_r
    hook_off = math.degrees(
        dr * math.tan(math.radians(undercut_deg)) / ((tip_r + root_r) / 2.0)
    )
    points = []
    for k in range(teeth):
        a = phase_deg + k * pitch
        points.append(_polar(tip_r, math.radians(a)))
        root_a = a - hook_off
        points.append(_polar(root_r, math.radians(root_a)))
        for j in range(1, ramp_steps + 1):
            fraction = j / (ramp_steps + 1.0)
            radius = root_r + (tip_r - root_r) * fraction
            angle = root_a + (a + pitch - root_a) * fraction
            points.append(_polar(radius, math.radians(angle)))
    cavity = sg.Polygon(points).buffer(0)
    if flip:
        cavity = affinity.scale(cavity, xfact=-1, yfact=1, origin=(0, 0))
    return cavity


def _pip_phase(teeth, tip_r, root_r, undercut_deg, flip, pawl_az,
               pawl_width, nose_r, spring_x0, spring_t, spring_gap,
               spring_folds, clearance):
    tail_in = spring_x0 + spring_folds * (spring_t + spring_gap) + 0.3
    blank = sg.box(tail_in, -pawl_width / 2.0, nose_r + 1.0,
                   pawl_width / 2.0)
    blank = blank.intersection(sg.Point(0, 0).buffer(nose_r, 96)).buffer(0)
    blank = affinity.rotate(blank, pawl_az[0], origin=(0, 0))
    if flip:
        blank = affinity.scale(blank, xfact=-1, yfact=1, origin=(0, 0))
    best_phase, best_area = 0.0, -1.0
    for phase in np.linspace(0.0, 360.0 / teeth, 121):
        cavity = _ratchet_cavity_2d(
            teeth, tip_r, root_r, undercut_deg, phase, flip=flip)
        area = blank.intersection(cavity.buffer(-clearance)).area
        if area > best_area:
            best_phase, best_area = float(phase), area
    return best_phase


def ratchet_ring_2d(teeth=9, tip_r=15.0, root_r=17.0, outer_r=18.4,
                    undercut_deg=7.0, clearance=0.15, flip=False,
                    phase_deg=None, pawl_az=(90.0, 210.0, 330.0),
                    pawl_width=5.6, nose_r=16.7, spring_x0=5.7,
                    spring_t=0.8, spring_gap=0.9, spring_folds=4):
    """Build the internal sawtooth ring used by ``pip_ratchet_hub_2d``.

    ``phase_deg=None`` deterministically phases the teeth for the matching
    accordion-pawl defaults. The locking flank is undercut, so drive load seats
    each rigid pawl while its spring only supplies the small reseating force.
    All tooth, clearance, hand, and phasing arguments are shared with the hub
    generator so the two profiles agree by construction.
    """
    if outer_r <= root_r or clearance < 0 or not pawl_az:
        raise ValueError("invalid ratchet ring dimensions")
    if phase_deg is None:
        phase_deg = _pip_phase(
            teeth, tip_r, root_r, undercut_deg, flip, pawl_az,
            pawl_width, nose_r, spring_x0, spring_t, spring_gap,
            spring_folds, clearance)
    cavity = _ratchet_cavity_2d(
        teeth, tip_r, root_r, undercut_deg, phase_deg, flip=flip)
    return sg.Point(0, 0).buffer(outer_r, resolution=96).difference(cavity).buffer(0)


def ratchet_ring(height=3.62, teeth=9, tip_r=15.0, root_r=17.0,
                 outer_r=18.4, undercut_deg=7.0, clearance=0.15,
                 flip=False, phase_deg=None,
                 pawl_az=(90.0, 210.0, 330.0), pawl_width=5.6,
                 nose_r=16.7, spring_x0=5.7, spring_t=0.8,
                 spring_gap=0.9, spring_folds=4):
    """Extrude ``ratchet_ring_2d`` into a printable internal ratchet ring."""
    profile = ratchet_ring_2d(
        teeth=teeth, tip_r=tip_r, root_r=root_r, outer_r=outer_r,
        undercut_deg=undercut_deg, clearance=clearance, flip=flip,
        phase_deg=phase_deg, pawl_az=pawl_az, pawl_width=pawl_width,
        nose_r=nose_r, spring_x0=spring_x0, spring_t=spring_t,
        spring_gap=spring_gap, spring_folds=spring_folds)
    return _extrude(profile, height)


def _pip_serpentine_2d(pawl_width, spring_x0, spring_t, spring_gap,
                        spring_folds):
    pitch = spring_t + spring_gap
    spring_x1 = spring_x0 + spring_folds * pitch
    tail_in = spring_x1 + 0.3
    half_width = (pawl_width - 0.4) / 2.0
    points = []
    for i in range(spring_folds + 1):
        x = spring_x0 + i * pitch
        points.extend(((x, -half_width), (x, half_width)) if i % 2 == 0
                      else ((x, half_width), (x, -half_width)))
    beam = sg.LineString(points).buffer(
        spring_t / 2.0, cap_style=2, join_style=1)
    base = sg.box(spring_x0 - 0.6, -half_width - 0.2,
                  spring_x0 + spring_t, half_width + 0.2)
    top = sg.box(spring_x1 - spring_t, -half_width - 0.2,
                 tail_in + 0.2, half_width + 0.2)
    spring = unary_union([beam, base, top]).buffer(0)
    keep = sg.box(1.5, -half_width - 0.6,
                  spring_x0 + spring_t, half_width + 0.6)
    return spring, keep, tail_in


def pip_ratchet_hub_2d(
        teeth=9, tip_r=15.0, root_r=17.0, outer_r=18.4,
        undercut_deg=7.0, clearance=0.15, flip=False, phase_deg=None,
        pawl_az=(90.0, 210.0, 330.0), hub_radius=14.5,
        spoke_outer_r=13.5, pawl_width=5.6, nose_r=16.7,
        travel=1.9, sliding_clearance=0.45, boss_width=1.7,
        spoke_inner_r=2.0, spring_x0=5.7, spring_t=0.8,
        spring_gap=0.9, spring_folds=4, spring_clearance=0.45):
    """Build a monolithic print-in-place hub with three rigid captive pawls.

    Each pawl is preloaded by an integral serpentine compression spring. The
    spring only reseats the pawl: torque travels through the undercut ring flank,
    rigid nose, pawl body, guide boss, spoke, and center disc. The flat profile
    prints without support because every moving clearance is a vertical slot.
    Pair it with ``ratchet_ring_2d`` using the same tooth, radius, undercut,
    clearance, hand, and phase arguments.
    """
    del outer_r  # part of the shared ring interface; it does not alter the hub
    if (hub_radius <= 0 or spoke_outer_r <= 0 or pawl_width <= 0 or
            travel < 0 or sliding_clearance < 0 or spring_clearance < 0 or
            boss_width <= 0 or spring_t <= 0 or spring_gap < 0 or
            spring_folds < 2 or not pawl_az):
        raise ValueError("invalid print-in-place ratchet dimensions")
    if phase_deg is None:
        phase_deg = _pip_phase(
            teeth, tip_r, root_r, undercut_deg, flip, pawl_az,
            pawl_width, nose_r, spring_x0, spring_t, spring_gap,
            spring_folds, clearance)
    cavity = _ratchet_cavity_2d(
        teeth, tip_r, root_r, undercut_deg, phase_deg, flip=flip)
    spring_local, spring_keep_local, tail_in = _pip_serpentine_2d(
        pawl_width, spring_x0, spring_t, spring_gap, spring_folds)
    half_spoke = pawl_width / 2.0 + sliding_clearance + boss_width
    spoke = sg.box(spoke_inner_r, -half_spoke, spoke_outer_r, half_spoke)
    hub = sg.Point(0, 0).buffer(hub_radius, resolution=128)
    for azimuth in pawl_az:
        hub = hub.union(affinity.rotate(spoke, azimuth, origin=(0, 0)))

    moving = []
    for azimuth in pawl_az:
        body = sg.box(tail_in, -pawl_width / 2.0, nose_r + 1.0,
                      pawl_width / 2.0)
        body = body.intersection(sg.Point(0, 0).buffer(nose_r, 96)).buffer(0)
        pawl = affinity.rotate(body, azimuth, origin=(0, 0))
        pawl = _largest_polygon(pawl.intersection(
            cavity.buffer(-clearance)).buffer(0))
        spring = affinity.rotate(spring_local, azimuth, origin=(0, 0))
        spring_keep = affinity.rotate(
            spring_keep_local, azimuth, origin=(0, 0))
        ux = math.cos(math.radians(azimuth))
        uy = math.sin(math.radians(azimuth))
        retracted = affinity.translate(pawl, -travel * ux, -travel * uy)
        pawl_sweep = unary_union([pawl, retracted])
        # The accordion base is fixed. Its engaged footprint already covers all
        # compressed states, so sweeping it would remove its anchor.
        void = unary_union([
            pawl_sweep.buffer(sliding_clearance),
            spring.buffer(spring_clearance).difference(spring_keep),
        ]).buffer(0)
        hub = hub.difference(void)
        moving.append(unary_union([pawl, spring]).buffer(0))
    hub = unary_union([hub] + moving).buffer(0)
    return _largest_polygon(hub)


def pip_ratchet_hub(
        height=3.15, bore_w=7.2, bore_t=2.0, bore_clearance=0.30,
        teeth=9, tip_r=15.0, root_r=17.0, outer_r=18.4,
        undercut_deg=7.0, clearance=0.15, flip=False, phase_deg=None,
        pawl_az=(90.0, 210.0, 330.0), hub_radius=14.5,
        spoke_outer_r=13.5, pawl_width=5.6, nose_r=16.7,
        travel=1.9, sliding_clearance=0.45, boss_width=1.7,
        spoke_inner_r=2.0, spring_x0=5.7, spring_t=0.8,
        spring_gap=0.9, spring_folds=4, spring_clearance=0.45):
    """Extrude a print-in-place ratchet hub and cut its rectangular shaft bore."""
    if bore_w < 0 or bore_t < 0 or bore_clearance < 0:
        raise ValueError("invalid hub bore dimensions")
    profile = pip_ratchet_hub_2d(
        teeth=teeth, tip_r=tip_r, root_r=root_r, outer_r=outer_r,
        undercut_deg=undercut_deg, clearance=clearance, flip=flip,
        phase_deg=phase_deg, pawl_az=pawl_az, hub_radius=hub_radius,
        spoke_outer_r=spoke_outer_r, pawl_width=pawl_width,
        nose_r=nose_r, travel=travel, sliding_clearance=sliding_clearance,
        boss_width=boss_width, spoke_inner_r=spoke_inner_r,
        spring_x0=spring_x0, spring_t=spring_t, spring_gap=spring_gap,
        spring_folds=spring_folds, spring_clearance=spring_clearance)
    if bore_w > 0 and bore_t > 0:
        profile = profile.difference(sg.box(
            -(bore_w + bore_clearance) / 2.0,
            -(bore_t + bore_clearance) / 2.0,
            +(bore_w + bore_clearance) / 2.0,
            +(bore_t + bore_clearance) / 2.0,
        )).buffer(0)
    return _extrude(profile, height)


def _cartridge_cavity_2d(teeth, tip_r, root_r, hook_deg, phase_deg,
                         drive_ccw):
    # The structural-template revision used one straight ramp chord per tooth.
    cavity = _ratchet_cavity_2d(
        teeth, tip_r, root_r, hook_deg, phase_deg,
        flip=not drive_ccw, ramp_steps=0)
    return cavity


def _cartridge_pawl_local(tail_w, tail_x0, tail_x1, fork_w, fork_x1,
                          body_w, nose_x1):
    tail = sg.box(tail_x0, -tail_w / 2.0, tail_x1, tail_w / 2.0)
    fork = sg.box(tail_x0 - 0.5, -fork_w / 2.0, fork_x1, fork_w / 2.0)
    body = sg.box(tail_x1 - 0.6, -body_w / 2.0, nose_x1, body_w / 2.0)
    return unary_union([tail.difference(fork), body]).buffer(0)


def _cartridge_phase(teeth, tip_r, root_r, hook_deg, drive_ccw,
                     pawl_az, pawl_clear, pawl_local):
    blank = affinity.rotate(pawl_local, pawl_az[0], origin=(0, 0))
    best_phase, best_area = 0.0, -1.0
    for phase in np.linspace(0.0, 360.0 / teeth, 121):
        cavity = _cartridge_cavity_2d(
            teeth, tip_r, root_r, hook_deg, phase, drive_ccw)
        area = blank.intersection(cavity.buffer(-pawl_clear)).area
        if area > best_area:
            best_phase, best_area = float(phase), area
    return best_phase


def spring_cartridge_ratchet_2d(
        teeth=12, tip_r=13.2, root_r=15.2, outer_r=17.8,
        hook_deg=7.0, drive_ccw=True, phase_deg=None,
        hub_r=12.7, pawl_az=(90.0, 210.0, 330.0), pawl_clear=0.15,
        slot_w_wide=7.7, slot_w_mouth=5.4, slot_x_in=5.0,
        slot_x_step=12.15, slot_x_out=12.75, tail_w=7.3,
        tail_x0=7.2, tail_x1=12.05, fork_w=4.6, fork_x1=11.0,
        body_w=5.0, nose_x1=15.35, boss_sq=2.2, boss_len=1.2):
    """Build a rigid-pawl ratchet cartridge as ``(ring, hub, pawls)``.

    ``pawls`` is a tuple of three individually printable polygons. Purchased
    radial compression springs sit in their fork windows and push them into the
    undercut ring. The hub carries guide slots and spring bosses; a separate
    cover can retain the cartridge in a consumer design. Defaults reproduce the
    standalone spring-ratchet structural template without its project gear top.
    """
    if outer_r <= root_r or hub_r <= 0 or pawl_clear < 0 or not pawl_az:
        raise ValueError("invalid spring-cartridge ratchet dimensions")
    pawl_local = _cartridge_pawl_local(
        tail_w, tail_x0, tail_x1, fork_w, fork_x1, body_w, nose_x1)
    if phase_deg is None:
        phase_deg = _cartridge_phase(
            teeth, tip_r, root_r, hook_deg, drive_ccw,
            pawl_az, pawl_clear, pawl_local)
    cavity = _cartridge_cavity_2d(
        teeth, tip_r, root_r, hook_deg, phase_deg, drive_ccw)
    ring = sg.Point(0, 0).buffer(outer_r, resolution=96).difference(cavity).buffer(0)

    wide = sg.box(slot_x_in, -slot_w_wide / 2.0,
                  slot_x_step, slot_w_wide / 2.0)
    mouth = sg.box(slot_x_step, -slot_w_mouth / 2.0,
                   slot_x_out, slot_w_mouth / 2.0)
    slot = unary_union([wide, mouth])
    boss = sg.box(slot_x_in - 1.0, -boss_sq / 2.0,
                  slot_x_in + boss_len, boss_sq / 2.0)
    hub = sg.Point(0, 0).buffer(hub_r, resolution=96)
    for azimuth in pawl_az:
        hub = hub.difference(affinity.rotate(slot, azimuth, origin=(0, 0)))
    for azimuth in pawl_az:
        hub = hub.union(affinity.rotate(boss, azimuth, origin=(0, 0)))
    hub = _largest_polygon(hub.buffer(0))

    pawls = []
    for azimuth in pawl_az:
        blank = affinity.rotate(pawl_local, azimuth, origin=(0, 0))
        pawl = _largest_polygon(blank.intersection(
            cavity.buffer(-pawl_clear)).buffer(0))
        pawls.append(pawl)
    return ring, hub, tuple(pawls)


def spring_cartridge_ratchet(
        ring_height=3.2, hub_height=2.75, pawl_height=2.6,
        hub_z0=0.15, pawl_z0=0.15, bore_w=7.2, bore_t=2.0,
        bore_clearance=0.3, teeth=12, tip_r=13.2, root_r=15.2,
        outer_r=17.8, hook_deg=7.0, drive_ccw=True, phase_deg=None,
        hub_r=12.7, pawl_az=(90.0, 210.0, 330.0), pawl_clear=0.15,
        slot_w_wide=7.7, slot_w_mouth=5.4, slot_x_in=5.0,
        slot_x_step=12.15, slot_x_out=12.75, tail_w=7.3,
        tail_x0=7.2, tail_x1=12.05, fork_w=4.6, fork_x1=11.0,
        body_w=5.0, nose_x1=15.35, boss_sq=2.2, boss_len=1.2):
    """Extrude a cartridge into ``(ring_mesh, hub_mesh, pawl_meshes)``.

    The return pieces are intentionally decomposed for a consumer to add its
    own shaft interface, cover, gear, and spring-retention features.
    """
    ring2d, hub2d, pawl2ds = spring_cartridge_ratchet_2d(
        teeth=teeth, tip_r=tip_r, root_r=root_r, outer_r=outer_r,
        hook_deg=hook_deg, drive_ccw=drive_ccw, phase_deg=phase_deg,
        hub_r=hub_r, pawl_az=pawl_az, pawl_clear=pawl_clear,
        slot_w_wide=slot_w_wide, slot_w_mouth=slot_w_mouth,
        slot_x_in=slot_x_in, slot_x_step=slot_x_step,
        slot_x_out=slot_x_out, tail_w=tail_w, tail_x0=tail_x0,
        tail_x1=tail_x1, fork_w=fork_w, fork_x1=fork_x1,
        body_w=body_w, nose_x1=nose_x1, boss_sq=boss_sq,
        boss_len=boss_len)
    if bore_w > 0 and bore_t > 0:
        bore = sg.box(
            -(bore_w + bore_clearance) / 2.0,
            -(bore_t + bore_clearance) / 2.0,
            +(bore_w + bore_clearance) / 2.0,
            +(bore_t + bore_clearance) / 2.0,
        )
        hub2d = hub2d.difference(bore).buffer(0)
    return (
        _extrude(ring2d, ring_height),
        _extrude(hub2d, hub_height, hub_z0),
        tuple(_extrude(pawl, pawl_height, pawl_z0) for pawl in pawl2ds),
    )


def check_ratchet_sense_and_sweep(
        ring_2d, pawls_2d, pawl_az=(90.0, 210.0, 330.0),
        teeth=12, travel=1.9, drive_ccw=True, drive_probe_deg=2.0,
        area_epsilon=5e-3):
    """Validate rest clearance, drive engagement, and retracted free-wheel.

    This is the reusable form of the spring-cartridge experiment's
    ``check_sense_and_sweep`` gate. It raises ``AssertionError`` when the ring
    overlaps at rest, fails to engage in the requested sense, fails to cam in
    on overrun, or cannot clear one tooth pitch at full radial retraction. A
    metrics dictionary is returned for test logs and design reports.
    """
    if len(pawls_2d) != len(pawl_az) or teeth < 3 or travel < 0:
        raise ValueError("pawls, azimuths, teeth, or travel are invalid")
    pawls = unary_union(pawls_2d)
    sign = 1.0 if drive_ccw else -1.0
    at_rest = ring_2d.intersection(pawls).area
    drive_area = affinity.rotate(
        ring_2d, sign * drive_probe_deg, origin=(0, 0)).intersection(pawls).area
    assert at_rest < 1e-6, "pawl/ring overlap at rest: %.6f mm^2" % at_rest
    assert drive_area > 0.05, "drive rotation does not engage: %.6f mm^2" % drive_area

    def free_rotation(rotation_sign, retract, limit=40.0):
        moved = []
        for azimuth, pawl in zip(pawl_az, pawls_2d):
            ux = math.cos(math.radians(azimuth))
            uy = math.sin(math.radians(azimuth))
            moved.append(affinity.translate(
                pawl, -ux * retract, -uy * retract))
        moved_pawls = unary_union(moved)
        if affinity.rotate(
                ring_2d, rotation_sign * limit, origin=(0, 0)
        ).intersection(moved_pawls).area < area_epsilon:
            return limit
        lo, hi = 0.0, limit
        for _ in range(24):
            mid = (lo + hi) / 2.0
            overlap = affinity.rotate(
                ring_2d, rotation_sign * mid, origin=(0, 0)
            ).intersection(moved_pawls).area
            if overlap < area_epsilon:
                lo = mid
            else:
                hi = mid
        return lo

    def contact_normal_radial(rotation_sign):
        theta = free_rotation(rotation_sign, 0.0) + 0.15
        sliver = affinity.rotate(
            ring_2d, rotation_sign * theta, origin=(0, 0)
        ).intersection(pawls_2d[0])
        assert sliver.area > 1e-4, "no contact sliver found"
        geometry = max(sliver.geoms, key=lambda item: item.area) \
            if hasattr(sliver, "geoms") else sliver
        xy = np.asarray(geometry.exterior.coords) - np.asarray(
            geometry.exterior.coords).mean(axis=0)
        _, _, vectors = np.linalg.svd(xy, full_matrices=False)
        direction = vectors[0] / np.linalg.norm(vectors[0])
        normal = np.array([-direction[1], direction[0]])
        center = np.asarray(geometry.centroid.coords[0])
        radial = center / np.linalg.norm(center)
        tangent = np.array([-radial[1], radial[0]]) * rotation_sign
        if np.dot(normal, tangent) < 0:
            normal = -normal
        return float(np.dot(normal, radial))

    drive_normal = contact_normal_radial(sign)
    overrun_normal = contact_normal_radial(-sign)
    assert drive_normal > 0.04, "drive flank is not self-energising"
    assert overrun_normal < -0.15, "overrun ramp does not cam pawls inward"
    free_rest = free_rotation(-sign, 0.0)
    free_retracted = free_rotation(-sign, travel)
    pitch = 360.0 / teeth
    assert free_retracted >= pitch, "overrun remains blocked when retracted"
    assert free_rest < pitch, "no overrun ramp contact at rest"
    return {
        "rest_overlap_area": at_rest,
        "drive_overlap_area": drive_area,
        "drive_normal_radial": drive_normal,
        "overrun_normal_radial": overrun_normal,
        "overrun_free_deg": free_rest,
        "retracted_free_deg": free_retracted,
    }


def compliant_clutch_2d(
        teeth=18, pawls=3, race_outer_r=24.5, race_tip_r=14.0,
        tooth_depth=2.6, hub_r=8.0, flexure_w=1.4,
        flexure_arc_deg=150.0, flexure_clearance=0.45,
        preload=0.6, flip=False, lock_face_frac=0.40):
    """Build ``(race, hub)`` for a compliant flexure one-way clutch.

    ``lock_face_frac`` controls the angular locking-face width. Around 0.12 is
    a steep self-locking one-way face; 0.30 to 0.38 produces a torque limiter
    that cams out and slips. The threshold depends on flexure stiffness and
    must be bench-tuned. In the source project's PETG bench test, flexure arms
    in the torque path crushed at about 1.3 N m, so this is a light-torque part.
    """
    if (teeth < 3 or pawls < 1 or race_outer_r <= race_tip_r + tooth_depth or
            tooth_depth <= 0 or hub_r <= 0 or flexure_w <= 0 or
            flexure_clearance < 0 or preload < 0 or
            not 0.0 < lock_face_frac < 0.5):
        raise ValueError("invalid compliant clutch dimensions")
    hand = -1.0 if flip else 1.0
    pitch = 2.0 * math.pi / teeth
    valley_r = race_tip_r + tooth_depth
    points = []
    for i in range(teeth):
        center = i * pitch
        points.append(_polar(valley_r, center - hand * 0.5 * pitch))
        points.append(_polar(
            race_tip_r,
            center + hand * (0.5 - lock_face_frac) * pitch,
        ))
    inner = sg.Polygon(points).buffer(0)
    race = sg.Point(0, 0).buffer(race_outer_r, resolution=96).difference(inner).buffer(0)

    hub = sg.Point(0, 0).buffer(hub_r, resolution=72)
    nest_r = valley_r - preload
    sweep = hand * math.radians(flexure_arc_deg)
    arms = []
    for k in range(pawls):
        tip_angle = hand * (2.0 * math.pi * k / pawls) - hand * 0.5 * pitch
        base_angle = tip_angle - sweep
        points = []
        for j in range(10):
            fraction = j / 9.0
            radius = hub_r - 0.8 + (nest_r - hub_r + 0.8) * fraction
            points.append(_polar(radius, base_angle + sweep * fraction))
        arms.append(sg.LineString(points).buffer(
            flexure_w / 2.0, cap_style=1, join_style=1))
    hub = unary_union([hub] + arms).buffer(0)
    hub = _largest_polygon(hub.difference(
        race.buffer(flexure_clearance)).buffer(0))
    return race, hub


def compliant_clutch(
        height=3.0, teeth=18, pawls=3, race_outer_r=24.5,
        race_tip_r=14.0, tooth_depth=2.6, hub_r=8.0,
        flexure_w=1.4, flexure_arc_deg=150.0,
        flexure_clearance=0.45, preload=0.6, flip=False,
        lock_face_frac=0.40):
    """Extrude ``compliant_clutch_2d`` and return ``(race_mesh, hub_mesh)``."""
    race, hub = compliant_clutch_2d(
        teeth=teeth, pawls=pawls, race_outer_r=race_outer_r,
        race_tip_r=race_tip_r, tooth_depth=tooth_depth, hub_r=hub_r,
        flexure_w=flexure_w, flexure_arc_deg=flexure_arc_deg,
        flexure_clearance=flexure_clearance, preload=preload,
        flip=flip, lock_face_frac=lock_face_frac)
    return _extrude(race, height), _extrude(hub, height)


def _arc_arm_points(hand, core_r, arm_r, arm_sweep_deg):
    sweep = math.radians(arm_sweep_deg)
    points = []
    for fraction in np.linspace(0.0, 1.0, 17):
        if fraction < 0.5:
            radius = (core_r - 0.5 + (arm_r - core_r + 0.5) *
                      min(1.0, fraction / 0.5) ** 1.5)
        else:
            radius = arm_r
        points.append(_polar(radius, hand * fraction * sweep))
    return points


def arc_ratchet_2d(
        teeth=12, tip_r=14.2, root_r=16.0, ring_outer_r=18.0,
        undercut_deg=8.0, pawls=3, arm_w=2.0, arm_sweep_deg=113.0,
        arm_r=12.3, clearance=0.35, flip=False, core_r=6.5):
    """Build ``(ring, hub)`` for a tension-loaded compliant arc ratchet.

    Size each arm with ``strain ~= 3*delta*w / (2*L**2)`` and keep the peak
    below roughly 1.5 percent for PETG. The source project's drive-path arms
    crushed at about 1.3 N m, so this is a light-torque part. Its undercut
    locking face is self-energising and loads the trailing arms in tension.
    """
    if (teeth < 3 or pawls < 1 or not 0 < tip_r < root_r < ring_outer_r or
            arm_w <= 0 or arm_r <= core_r or clearance < 0):
        raise ValueError("invalid arc ratchet dimensions")
    hand = -1.0 if flip else 1.0
    pitch = 2.0 * math.pi / teeth
    overhang = ((root_r - tip_r) * math.tan(math.radians(undercut_deg)) /
                tip_r)
    backing = sg.Point(0, 0).buffer(ring_outer_r, resolution=128).difference(
        sg.Point(0, 0).buffer(root_r - 0.05, resolution=128))
    teeth_polys = []
    for i in range(teeth):
        center = hand * i * pitch
        ramp = _polar(root_r, center - hand * 0.85 * pitch)
        lock = _polar(root_r, center + hand * 0.10 * pitch)
        tip = _polar(tip_r, center + hand * (0.10 * pitch + overhang))
        teeth_polys.append(sg.Polygon([ramp, lock, tip]))
    ring = unary_union([backing] + teeth_polys).buffer(0)

    core = sg.Point(0, 0).buffer(core_r, resolution=96)
    arm = sg.LineString(_arc_arm_points(
        hand, core_r, arm_r, arm_sweep_deg)).buffer(
            arm_w / 2.0, cap_style=2, join_style=1)
    tip_angle = hand * math.radians(arm_sweep_deg)
    head = sg.Polygon([
        _polar(arm_r - 1.2, tip_angle - hand * 0.06),
        _polar(root_r - 0.15, tip_angle + hand * 0.10),
        _polar(root_r - 0.15, tip_angle + hand * 0.16),
        _polar(arm_r - 1.2, tip_angle + hand * 0.10),
    ]).buffer(0.2)
    one = unary_union([arm, head]).buffer(0)
    arms = unary_union([
        affinity.rotate(one, 100.0 + 360.0 * k / pawls, origin=(0, 0))
        for k in range(pawls)
    ]).buffer(0)
    hub = unary_union([core, arms]).buffer(0)
    hub = hub.difference(ring.buffer(clearance)).buffer(0)
    return ring, _largest_polygon(hub)


__all__ = (
    "ratchet_ring_2d",
    "ratchet_ring",
    "pip_ratchet_hub_2d",
    "pip_ratchet_hub",
    "spring_cartridge_ratchet_2d",
    "spring_cartridge_ratchet",
    "check_ratchet_sense_and_sweep",
    "compliant_clutch_2d",
    "compliant_clutch",
    "arc_ratchet_2d",
)


def ratchet_wheel_pawl(teeth=14, tip_r=20.0, tooth_h=4.0, thickness=6.0,
                       bore_d=8.0, pawl_len=24.0, pivot_d=4.0,
                       spring="leaf", clear=0.25, flat=False, sections=96):
    """Build the canonical external ratchet: sawtooth wheel plus pivoted pawl.

    The serviceable high-torque form used on winch drums, come-alongs,
    windlasses and webbing tensioners -- unlike the print-in-place hubs and
    rings elsewhere in this module, wheel and pawl are separate bolt-on
    parts. The wheel carries ``teeth`` radial sawteeth of height
    ``tooth_h`` on a ``tip_r`` tip circle (drive face nearly radial, slip
    face shallow; looking from +Z the wheel drives the pawl when rotating
    CCW), bored ``bore_d + clear`` with an optional D-flat (``flat=True``)
    for a set screw. The pawl pivots on a frame shaft of ``pivot_d`` at
    ``pawl_len`` from its tip and is posed engaged with the wheel. With
    ``spring='leaf'`` a third part -- a curved printed leaf spring with an
    anchor block -- preloads the pawl into the teeth; ``spring='coil'``
    instead seats a blind 4.2 mm pocket on the pawl's back for a
    ``flexures.coil_spring`` and returns no spring part. Returns
    ``{'wheel', 'pawl'}`` plus ``'spring'`` for the leaf variant. All parts
    print flat on z=0, no support. Units are mm and degrees.
    """
    from .meshutil import sub, uni
    from .prim import boxc, cyl

    if tip_r <= 0 or tooth_h <= 0 or thickness <= 0 or bore_d <= 0:
        raise ValueError("ratchet_wheel_pawl(): tip_r, tooth_h, thickness "
                         "and bore_d must be positive")
    teeth = int(round(teeth))
    if teeth < 6:
        raise ValueError("ratchet_wheel_pawl(): need at least 6 teeth")
    if pawl_len <= 0 or pivot_d <= 0 or clear < 0.15:
        raise ValueError("ratchet_wheel_pawl(): pawl_len and pivot_d must "
                         "be positive, clear at least 0.15 mm")
    if spring not in ("leaf", "coil"):
        raise ValueError("ratchet_wheel_pawl(): spring must be 'leaf' or "
                         "'coil'")
    root_r = tip_r - tooth_h
    bore_r = (bore_d + clear) / 2.0
    if root_r - bore_r < 2.0:
        raise ValueError("ratchet_wheel_pawl(): tooth roots break through "
                         "to the bore")
    if tooth_h > 0.35 * tip_r:
        raise ValueError("ratchet_wheel_pawl(): tooth_h too tall for the "
                         "tip radius")

    step = 2.0 * math.pi / teeth
    pts = []
    for k in range(teeth):
        a0 = k * step
        pts.append(_polar(root_r, a0))
        pts.append(_polar(tip_r, a0 + 0.2 * step))
    wheel_poly = sg.Polygon(pts).difference(
        sg.Point(0.0, 0.0).buffer(bore_r, resolution=int(sections) // 4))
    if flat:
        wheel_poly = wheel_poly.difference(box := sg.box(
            bore_r - 1.0, -bore_r - 1.0, bore_r + 2.0, -bore_r + 0.8))
    wheel = _extrude(_largest_polygon(wheel_poly.buffer(0)), thickness)

    # Pawl: pivot boss + tip boss joined by a web, posed seated in the tooth
    # valley just off the drive face (mid-valley at the root circle).
    tip = np.array(_polar(root_r + 2.6 + 0.3, 0.55 * step))
    ang = math.radians(120.0)
    pivot = tip + pawl_len * np.array([math.cos(ang), math.sin(ang)])
    quad = max(4, int(sections) // 4)
    boss_p = pivot_d / 2.0 + clear / 2.0 + 2.4
    boss_t = 2.6
    pawl_poly = unary_union([
        sg.Point(*pivot).buffer(boss_p, resolution=quad),
        sg.Point(*tip).buffer(boss_t, resolution=quad),
    ]).convex_hull
    pawl_poly = pawl_poly.difference(
        sg.Point(*pivot).buffer(pivot_d / 2.0 + clear / 2.0,
                                resolution=quad)).buffer(0)
    pawl = _extrude(_largest_polygon(pawl_poly), thickness)

    meta = {
        "teeth": int(teeth),
        "tip_r": float(tip_r),
        "tooth_h": float(tooth_h),
        "thickness": float(thickness),
        "circular_pitch": float(2.0 * math.pi * tip_r / teeth),
        "bore_d": float(bore_d + clear),
        "pawl_len": float(pawl_len),
        "pivot_d": float(pivot_d),
        "spring": spring,
        "drive_sense": "ccw",
    }
    parts = {"wheel": wheel, "pawl": pawl}

    if spring == "coil":
        back = pivot + 0.65 * (tip - pivot)
        pocket = cyl(2.1, 5.0,
                     center=(back[0], back[1], thickness + 1.0),
                     sections=24)
        parts["pawl"] = sub(pawl, pocket)
    else:
        # Leaf spring: arc beam from an anchor block onto the pawl's back.
        d_pt = tip - pivot
        d_pt = d_pt / np.linalg.norm(d_pt)
        n = np.array([-d_pt[1], d_pt[0]])
        contact = pivot + 0.6 * (tip - pivot)
        if np.dot(n, contact) < 0:
            n = -n
        anchor = contact + n * 14.0
        mid = (anchor + contact) / 2.0 + n * 2.5
        beam = sg.LineString([tuple(anchor), tuple(mid),
                              tuple(contact + d_pt * 1.0)]).buffer(
                                  0.7, cap_style=2, join_style=1)
        block = sg.box(anchor[0] - 4.0, anchor[1] - 4.0,
                       anchor[0] + 4.0, anchor[1] + 4.0)
        hole = sg.Point(*anchor).buffer(1.7, resolution=quad)
        spring_poly = unary_union([beam, block]).difference(hole).buffer(0)
        parts["spring"] = _extrude(_largest_polygon(spring_poly), 3.0)

    for part in parts.values():
        part.metadata.update(meta)
    return parts


__all__ += ("ratchet_wheel_pawl",)
