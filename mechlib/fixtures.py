"""Pure fixture geometry."""
import math

import numpy as np
import trimesh
import trimesh.transformations as tf

from mechlib.prim import boxc

from .cutters import blind_socket, counterbore
from .mechanisms import COARSE_PITCH, coarse_pitch, thread_solid
from .meshutil import inter, orient, sub, uni
from .patterns import polar_ring
from .prim import cyl, frustum


def board_cradle(rect, fl, standoff=4.0, clr=0.4, wlen=9.0, wt=1.6, cap_over=2.0):
    """Per-component HOUSING for one PCB: at each of its 4 corners a standoff POST under the board
    (lifts the underside `standoff` mm off the floor so pins/solder don't short) + an L of two thin
    walls hugging that corner from OUTSIDE (locates the board in XY and captures its top edge). The
    brackets sit at corners only, so mid-edge connectors (USB, screw terminals, pin headers) stay
    clear, and the press-in lid holds the board down in Z. `fl` = interior floor z; `rect` = world
    (x0, y0, w, d, comp_h) from elec_board_rects(). Self-supporting: solid post, then walls rise on it."""
    x0, y0, w, d, _ = rect
    pcb_t = 1.6
    cap = fl + standoff + pcb_t + cap_over                  # capture-wall top, just over the PCB
    parts = []
    for ox in (-1, 1):
        for oy in (-1, 1):
            cx = x0 + (w if ox > 0 else 0)                  # this board corner (world XY)
            cy = y0 + (d if oy > 0 else 0)
            parts.append(boxc([5, 5, standoff], (cx - ox*2.5, cy - oy*2.5, fl + standoff/2.0)))     # post (under PCB)
            parts.append(boxc([wlen, wt, cap - fl], (cx - ox*wlen/2.0, cy + oy*(clr + wt/2.0), (fl + cap)/2.0)))  # wall ∥X edge
            parts.append(boxc([wt, wlen, cap - fl], (cx + ox*(clr + wt/2.0), cy - oy*wlen/2.0, (fl + cap)/2.0)))  # wall ∥Y edge
    return trimesh.boolean.union(parts)


def saddle(axis0, axis1, r, yi, rib_t, interior, rise=0.0):
    """Build a shell-trimmed cradle rib for a cylindrical part.

    This complements ``board_cradle`` for flat circuit boards.
    origin: mini-powerbank pickle_build.py:109
    """
    import numpy as np

    from .prim import seg_cylinder

    a0 = np.array(axis0, float); a1 = np.array(axis1, float)
    t = (yi - a0[1]) / (a1[1] - a0[1]); cz = a0[2] + t*(a1[2]-a0[2]); cx = a0[0] + t*(a1[0]-a0[0])
    top = cz + rise
    block = boxc((80, rib_t, top + 60), (cx, yi, top - (top+60)/2))
    block = trimesh.boolean.intersection([block, interior], engine='manifold')
    notch = seg_cylinder([a0[0], yi-rib_t, a0[2]],
                         [a1[0], yi+rib_t, a1[2]], 2 * (r+0.6))
    return trimesh.boolean.difference([block, notch], engine='manifold')


# --- exactly constrained mounts ---------------------------------------------
#
# ``board_cradle`` and ``saddle`` above are OVER-constrained holders: they clamp
# an object with more contacts than its six degrees of freedom, so the object
# sits wherever the clamping forces and the print tolerances push it. Everything
# below is the opposite discipline. A kinematic mount touches the moving body at
# exactly six points, one per degree of freedom, so the body has one and only
# one resting pose and returns to it every time it is put back.

_SQ2 = math.sqrt(2.0)
_KINDS = ("maxwell", "kelvin")
_BALL_STYLES = ("hardware", "printed")


def _vee_cutter(depth, ball_r, length):
    """Return a 90 degree vee-groove negative, apex line on +X at z=-depth."""
    half_diag = depth + ball_r
    side = _SQ2 * half_diag
    groove = boxc((length, side, side))
    groove.apply_transform(tf.rotation_matrix(math.pi / 4.0, (1, 0, 0)))
    groove.apply_translation((0.0, 0.0, -depth + half_diag))
    return groove


def _trihedral_cutter(depth, ball_r, phase_deg):
    """Return a three-plane pit negative pointed at (0, 0, -depth) and its normals.

    Each face is tilted 45 degrees from horizontal, so a ball drops onto exactly
    three points instead of the contact circle a turned cone would give.
    """
    reach = 6.0 * (depth + ball_r)
    parts, normals = [], []
    for k in range(3):
        psi = math.radians(phase_deg + 120.0 * k)
        normal = (math.cos(psi) / _SQ2, math.sin(psi) / _SQ2, 1.0 / _SQ2)
        half = boxc((reach, reach, reach), center=(0.0, 0.0, reach / 2.0))
        orient(half, normal)
        parts.append(half)
        normals.append(normal)
    pit = inter(inter(parts[0], parts[1]), parts[2])
    pit.apply_translation((0.0, 0.0, -depth))
    return pit, normals


def _flat_cutter(depth, ball_r, overshoot=3.0):
    """Return a flat-floored circular pocket negative with its floor at z=-depth."""
    height = depth + overshoot
    pocket = cyl(ball_r + 1.5, height)
    pocket.apply_translation((0.0, 0.0, -depth + height / 2.0))
    return pocket


def _seat_layout(kind, pcd, ball_d, depth, groove_len, phase_deg):
    """Return (cutters, ball centres, contact points, ball centre height).

    Seats are laid out on a ``pcd`` seat circle at 120 degrees. ``maxwell`` cuts
    three radial vee grooves (2 contacts each); ``kelvin`` cuts one trihedral
    pit (3), one vee groove (2), and one flat pocket (1). Both add to six.
    """
    ball_r = ball_d / 2.0
    z_c = -depth + ball_r * _SQ2
    cutters, centres, contacts = [], [], []
    for i, (x, y) in enumerate(polar_ring(3, pcd / 2.0,
                                          phase=math.radians(phase_deg))):
        centres.append((x, y, z_c))
        theta = math.atan2(y, x)
        if kind == "maxwell" or i == 1:
            groove = _vee_cutter(depth, ball_r, groove_len)
            groove.apply_transform(tf.rotation_matrix(theta, (0, 0, 1)))
            groove.apply_translation((x, y, 0.0))
            cutters.append(groove)
            tx, ty = -math.sin(theta), math.cos(theta)
            for side in (1.0, -1.0):
                contacts.append((x + side * tx * ball_r / _SQ2,
                                 y + side * ty * ball_r / _SQ2,
                                 z_c - ball_r / _SQ2))
        elif i == 0:
            pit, normals = _trihedral_cutter(depth, ball_r,
                                             math.degrees(theta))
            pit.apply_translation((x, y, 0.0))
            cutters.append(pit)
            for nx, ny, nz in normals:
                contacts.append((x - ball_r * nx, y - ball_r * ny,
                                 z_c - ball_r * nz))
        else:
            flat = _flat_cutter(depth - ball_r * (_SQ2 - 1.0), ball_r)
            flat.apply_translation((x, y, 0.0))
            cutters.append(flat)
            contacts.append((x, y, z_c - ball_r))
    return cutters, centres, contacts, z_c


def _point_seg_dist(p, a, b):
    """Return the XY distance from a point to a finite segment."""
    p, a, b = np.asarray(p, float), np.asarray(a, float), np.asarray(b, float)
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / float(np.dot(ab, ab)), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def _balls(centres, ball_r, subdiv):
    """Return one sphere mesh per ball centre."""
    out = []
    for centre in centres:
        sphere = trimesh.creation.icosphere(subdivisions=int(subdiv),
                                            radius=ball_r)
        sphere.apply_translation(centre)
        out.append(sphere)
    return out


def _stamp(parts, meta):
    """Copy an engineering-metadata dict onto every mesh of an assembly."""
    for name, mesh in parts.items():
        mesh.metadata.update(dict(meta, part=name))
    return parts


def kinematic_coupling(kind="maxwell", pcd=40.0, ball_d=6.0, plate_d=None,
                       plate_t=6.0, groove_depth=None, groove_len=None,
                       ball="hardware", socket_depth=None, socket_clear=0.15,
                       phase_deg=90.0, ball_subdiv=3):
    """Build an exactly constrained two-plate mount (Maxwell or Kelvin clamp).

    Three balls on the top plate drop into three seats on the base plate and
    touch them at exactly six points, one per degree of freedom, so the joint is
    exactly constrained: no bolt pattern, dowel fit, or print tolerance decides
    where the top plate ends up, only the six contacts do. Lift the top plate
    off and put it back and it returns to the same pose to roughly the contact
    finish, which is what makes tool changers, swappable end effectors, probe
    mounts, and modular fixture plates repeatable. ``kind="maxwell"`` cuts three
    vee grooves pointing at the centre, giving two contacts each; it is
    symmetric and its thermal centre is the middle of the plate.
    ``kind="kelvin"`` cuts a trihedral pit, a vee groove, and a flat pocket,
    giving 3+2+1 contacts; the pit end is a fixed thermal centre the rest of the
    plate expands away from, and each seat is easier to make accurately.

    z=0 is the base plate's seating face. The base occupies z in
    ``[-plate_t, 0]`` with the seats cut down into it; the top plate underside
    sits at the derived ``seat_gap`` above z=0 and the balls bridge the gap. All
    six contact points, the three ball centres, and ``dof_constrained=6`` land
    in ``mesh.metadata`` on every returned part, because the contact set is the
    whole engineering claim of this part and a test should be able to check it.

    ``ball="hardware"`` is the honest default: the top plate gets three blind
    cylindrical sockets of radius ``ball_d/2 + socket_clear`` for BOUGHT
    ``ball_d`` bearing balls or dowel ends, epoxied in with the coupling
    assembled so they self-locate. Steel balls are not printed. ``ball="printed"`` unions
    hemispherical bosses into the top plate instead, which makes a single-part
    demo but costs most of the repeatability: an FDM sphere has poor sphericity,
    the layer seam lands on or near the contact band, and the polar region
    prints as a stack of shrinking discs, so expect tens of microns of scatter
    and visible wear after a few dozen cycles rather than the few microns a
    steel ball gives. Grooves are the printable half of this joint: a vee cut
    into an upward face has no overhang at all. Balls are the unprintable half.

    A kinematic coupling carries no preload of its own and falls apart the
    moment it is tipped; use ``repeatable_dock`` for a coupling with a magnet or
    screw holding it closed.

    Print the base seats-up as drawn and the top plate flipped, underside up, so
    the sockets are plain blind holes and the printed bosses are domes rather
    than overhangs. Units are mm and degrees.
    """
    if kind not in _KINDS:
        raise ValueError("kinematic_coupling(): kind must be one of %r" % (_KINDS,))
    if ball not in _BALL_STYLES:
        raise ValueError("kinematic_coupling(): ball must be one of %r"
                         % (_BALL_STYLES,))
    if ball_d <= 0 or pcd <= 0 or plate_t <= 0:
        raise ValueError("kinematic_coupling(): pcd, ball_d, and plate_t must be positive")
    ball_r = ball_d / 2.0
    if pcd < 3.0 * ball_d:
        raise ValueError("kinematic_coupling(): pcd %.2f is too small for %.2f mm "
                         "balls; the three seats collide (need pcd >= 3*ball_d)"
                         % (pcd, ball_d))
    depth = ball_r if groove_depth is None else float(groove_depth)
    length = 2.0 * ball_d if groove_len is None else float(groove_len)
    plate_d = pcd + 3.0 * ball_d if plate_d is None else float(plate_d)
    socket_depth = 0.35 * ball_d if socket_depth is None else float(socket_depth)
    if depth < ball_r / _SQ2 + 0.2:
        raise ValueError("kinematic_coupling(): groove_depth %.2f is too shallow; "
                         "the contact band would sit above the plate face "
                         "(need >= %.2f)" % (depth, ball_r / _SQ2 + 0.2))
    if depth > plate_t - 1.6:
        raise ValueError("kinematic_coupling(): groove_depth %.2f leaves less than "
                         "1.6 mm of plate floor under the seats" % depth)
    if kind == "kelvin" and depth - ball_r * (_SQ2 - 1.0) < 0.3:
        raise ValueError("kinematic_coupling(): groove_depth %.2f leaves no flat "
                         "pocket for the Kelvin one-point seat" % depth)
    if length < ball_d + 1.0:
        raise ValueError("kinematic_coupling(): groove_len must be at least ball_d + 1")
    if pcd / 2.0 + length / 2.0 > plate_d / 2.0 - 1.5:
        raise ValueError("kinematic_coupling(): plate_d %.2f is too small to hold the "
                         "seats at pcd %.2f" % (plate_d, pcd))
    if ball == "hardware" and not 0.5 <= socket_depth <= ball_r - 0.5:
        raise ValueError("kinematic_coupling(): socket_depth %.2f must lie in "
                         "[0.5, %.2f] so the ball is captured yet protrudes"
                         % (socket_depth, ball_r - 0.5))
    if ball == "hardware" and plate_t < socket_depth + 1.6:
        raise ValueError("kinematic_coupling(): plate_t %.2f is too thin for a "
                         "%.2f mm ball socket" % (plate_t, socket_depth))
    if socket_clear < 0:
        raise ValueError("kinematic_coupling(): socket_clear must be non-negative")

    cutters, centres, contacts, z_c = _seat_layout(
        kind, pcd, ball_d, depth, length, phase_deg)
    base = cyl(plate_d / 2.0, plate_t)
    base.apply_translation((0.0, 0.0, -plate_t / 2.0))
    for cutter in cutters:
        base = sub(base, cutter)

    seat_gap = z_c if ball == "printed" else z_c + ball_r - socket_depth
    top = cyl(plate_d / 2.0, plate_t)
    top.apply_translation((0.0, 0.0, seat_gap + plate_t / 2.0))
    spheres = _balls(centres, ball_r, ball_subdiv)
    if ball == "printed":
        top = uni([top] + spheres)
        parts = {"base": base, "top": top}
    else:
        for x, y, _z in centres:
            top = sub(top, blind_socket(ball_r + socket_clear, socket_depth,
                                        (0, 0, -1), (x, y, seat_gap)))
        parts = {"base": base, "top": top, "balls": uni(spheres)}
    return _stamp(parts, {
        "kind": kind,
        "ball_style": ball,
        "dof_constrained": 6,
        "contacts": [[float(v) for v in p] for p in contacts],
        "ball_centers": [[float(v) for v in c] for c in centres],
        "ball_center_z": float(z_c),
        "seat_gap": float(seat_gap),
        "groove_depth": float(depth),
        "pcd": float(pcd),
        "ball_d": float(ball_d),
        "plate_d": float(plate_d),
        "plate_t": float(plate_t),
    })


def repeatable_dock(kind="maxwell", pcd=40.0, ball_d=6.0, plate_d=None,
                    plate_t=6.0, ball="hardware", preload="magnet",
                    magnet_d=10.0, magnet_t=3.0, magnet_gap=0.3,
                    screw_d=5.0, screw_len=None, bolt_d=3.4, bolt_pcd=None,
                    n_bolt=3, groove_depth=None, phase_deg=90.0,
                    ball_subdiv=3):
    """Build a kinematic coupling plus the preload that keeps it closed.

    A bare kinematic coupling only resists motion INTO its seats; nothing holds
    the top plate down, so it falls off the moment the assembly is tipped or
    accelerated. This adds a single central preload feature along the coupling
    axis, which is the only place a preload can act without adding a seventh
    contact and destroying the exact constraint. ``preload="magnet"`` sinks a
    disc magnet flush into the base seating face and carries the mating magnet
    on a boss under the top plate that reaches down to within ``magnet_gap`` of
    the base; the boss must never touch, so ``magnet_gap`` is a clearance, not a
    fit. ``preload="screw"`` taps the base centre and puts a counterbored
    clearance hole through the top plate, so a cap screw pulls axially without
    locating anything laterally. Disc magnets and the screw are bought hardware,
    not printed. ``n_bolt`` through holes on ``bolt_pcd`` mount the base to the
    machine and the top to the tool; ``bolt_pcd`` defaults to the seat circle
    itself, phased 60 degrees off the seats into the empty sectors between them,
    and every hole is checked against the real seat footprints rather than a
    radius rule.

    Coordinates, seats, contact metadata, and the ball hardware note are exactly
    those of ``kinematic_coupling``; z=0 is the base seating face and the six
    contact points are in ``mesh.metadata``. The magnet air gap is real: the
    balls hold the plates ``seat_gap`` apart, and a magnet pair separated by
    even half a millimetre of air loses a large fraction of its pull, which is
    why the top magnet rides on a boss instead of sitting flush.

    Print the base as drawn and the top plate flipped, underside up. Units are
    mm and degrees.
    """
    if preload not in ("magnet", "screw"):
        raise ValueError("repeatable_dock(): preload must be 'magnet' or 'screw'")
    parts = kinematic_coupling(kind=kind, pcd=pcd, ball_d=ball_d,
                               plate_d=plate_d, plate_t=plate_t,
                               groove_depth=groove_depth, ball=ball,
                               phase_deg=phase_deg, ball_subdiv=ball_subdiv)
    meta = dict(parts["base"].metadata)
    plate_d = meta["plate_d"]
    seat_gap = meta["seat_gap"]
    base, top = parts["base"], parts["top"]

    if preload == "magnet":
        if magnet_d <= 0 or magnet_t <= 0:
            raise ValueError("repeatable_dock(): magnet_d and magnet_t must be positive")
        if magnet_gap < 0.2:
            raise ValueError("repeatable_dock(): magnet_gap must be at least 0.2 mm "
                             "so the preload boss never becomes a seventh contact")
        boss_r = magnet_d / 2.0 + 1.6
        if boss_r > pcd / 2.0 - ball_d:
            raise ValueError("repeatable_dock(): magnet_d %.2f makes a boss that "
                             "fouls the seats at pcd %.2f" % (magnet_d, pcd))
        if magnet_t + 0.15 > plate_t - 1.2:
            raise ValueError("repeatable_dock(): plate_t %.2f is too thin for a "
                             "%.2f mm magnet" % (plate_t, magnet_t))
        boss_h = seat_gap - magnet_gap
        if boss_h < 0.4:
            raise ValueError("repeatable_dock(): seat_gap %.2f leaves no room for a "
                             "preload boss above magnet_gap %.2f"
                             % (seat_gap, magnet_gap))
        pocket_h = magnet_t + 0.15
        base_pocket = cyl(magnet_d / 2.0 + 0.15, pocket_h)
        base_pocket.apply_translation((0.0, 0.0, -pocket_h / 2.0))
        base = sub(base, base_pocket)
        boss = cyl(boss_r, boss_h)
        boss.apply_translation((0.0, 0.0, magnet_gap + boss_h / 2.0))
        top = uni([top, boss])
        top_pocket = cyl(magnet_d / 2.0 + 0.15, pocket_h)
        top_pocket.apply_translation((0.0, 0.0, magnet_gap + pocket_h / 2.0))
        top = sub(top, top_pocket)
        preload_r = boss_r
    else:
        thread_len = plate_t - 0.6 if screw_len is None else float(screw_len)
        if float(screw_d) not in COARSE_PITCH:
            raise ValueError("repeatable_dock(): screw_d %r has no printable coarse "
                             "pitch; use one of %r"
                             % (screw_d, sorted(COARSE_PITCH)))
        if thread_len < 2.0 * coarse_pitch(screw_d):
            raise ValueError("repeatable_dock(): screw_len %.2f is under two thread "
                             "pitches" % thread_len)
        if thread_len > plate_t:
            raise ValueError("repeatable_dock(): screw_len %.2f exceeds the base "
                             "plate thickness" % thread_len)
        if screw_d + 4.0 > pcd - ball_d:
            raise ValueError("repeatable_dock(): screw_d %.2f fouls the seats at "
                             "pcd %.2f" % (screw_d, pcd))
        head_depth = 0.6 * screw_d
        if head_depth > plate_t - 1.2:
            raise ValueError("repeatable_dock(): plate_t %.2f is too thin to "
                             "counterbore an M%.0f head" % (plate_t, screw_d))
        tap_cut = thread_solid(screw_d, thread_len, internal=True)
        tap_cut.apply_translation((0.0, 0.0, -thread_len))
        base = sub(base, tap_cut)
        bore = counterbore(screw_d + 0.6, 1.9 * screw_d, head_depth,
                           plate_t + 2.0, head_end="top")
        bore.apply_translation((0.0, 0.0, seat_gap + plate_t / 2.0 - 1.0))
        top = sub(top, bore)
        preload_r = 0.95 * screw_d

    bolt_pcd = pcd if bolt_pcd is None else float(bolt_pcd)
    n_bolt = int(n_bolt)
    if n_bolt < 0:
        raise ValueError("repeatable_dock(): n_bolt must be non-negative")
    if n_bolt:
        if bolt_d <= 0:
            raise ValueError("repeatable_dock(): bolt_d must be positive")
        seat_len = 2.0 * ball_d
        keep_out = (max(meta["groove_depth"], ball_d / 2.0 + 1.5)
                    + bolt_d / 2.0 + 1.2)
        seat_segments = []
        for x, y, _z in meta["ball_centers"]:
            radius = math.hypot(x, y)
            unit = np.array([x, y]) / radius
            seat_segments.append((unit * (radius - seat_len / 2.0),
                                  unit * (radius + seat_len / 2.0)))
        for x, y in polar_ring(n_bolt, bolt_pcd / 2.0,
                               phase=math.radians(phase_deg + 60.0)):
            for a, b in seat_segments:
                if _point_seg_dist((x, y), a, b) < keep_out:
                    raise ValueError(
                        "repeatable_dock(): bolt at (%.2f, %.2f) on bolt_pcd %.2f "
                        "runs into a seat; move the bolt circle or drop to 3 bolts"
                        % (x, y, bolt_pcd))
        if bolt_pcd / 2.0 - bolt_d / 2.0 < preload_r + 1.0:
            raise ValueError("repeatable_dock(): bolt_pcd %.2f runs into the central "
                             "preload feature" % bolt_pcd)
        if bolt_pcd / 2.0 + bolt_d / 2.0 > plate_d / 2.0 - 1.5:
            raise ValueError("repeatable_dock(): bolt_pcd %.2f runs off the %.2f mm "
                             "plate" % (bolt_pcd, plate_d))
        for x, y in polar_ring(n_bolt, bolt_pcd / 2.0,
                               phase=math.radians(phase_deg + 60.0)):
            hole = cyl(bolt_d / 2.0, 4.0 * plate_t)
            hole.apply_translation((x, y, 0.0))
            base = sub(base, hole)
            hole2 = cyl(bolt_d / 2.0, 4.0 * plate_t)
            hole2.apply_translation((x, y, seat_gap + plate_t / 2.0))
            top = sub(top, hole2)

    out = {"base": base, "top": top}
    if "balls" in parts:
        out["balls"] = parts["balls"]
    meta.pop("part", None)
    meta.update({
        "preload": preload,
        "magnet_d": float(magnet_d) if preload == "magnet" else 0.0,
        "magnet_t": float(magnet_t) if preload == "magnet" else 0.0,
        "magnet_air_gap": float(magnet_gap) if preload == "magnet" else 0.0,
        "bolt_d": float(bolt_d) if n_bolt else 0.0,
        "bolt_pcd": float(bolt_pcd) if n_bolt else 0.0,
        "n_bolt": n_bolt,
    })
    return _stamp(out, meta)


def three_point_leveller(kind="kelvin", plate_d=None, table_d=None, plate_t=6.0,
                         base_t=6.0, screw_pcd=44.0, screw_d=6.0,
                         screw_len=18.0, tip_ratio=0.7, knob_d=16.0, knob_t=5.0,
                         lift=6.0, thread_clear=0.3, phase_deg=90.0,
                         ball_subdiv=3):
    """Build a three-screw kinematic levelling stage (tip, tilt, and height).

    The same exact-constraint idea as ``kinematic_coupling`` applied to
    alignment instead of repeatability. Three screws thread through a table
    plate at 120 degrees and rest their ball tips in kinematic seats on a base
    plate, so the table is held on exactly six contact points and has no
    residual freedom, yet turning the screws walks the tips up and down and
    moves the table in the only three directions the screws leave: height, tip,
    and tilt. Turning one screw one full turn raises its corner by one thread
    pitch (``mm_per_turn``) and tilts the table about the line through the other
    two screws, whose lever arm is 1.5 times the screw circle radius, giving
    ``tilt_per_turn_deg``. This is how optical mounts, printer beds, and surface
    plates are levelled without fighting an over-constrained bolt pattern.

    z=0 is the base plate's seating face, the base occupies ``[-base_t, 0]``,
    and the table underside sits ``lift`` mm above it in the nominal pose the
    seats define. ``plate_d`` sizes the base and ``table_d`` the moving table;
    leave ``table_d`` at None to match them, or make the table smaller so the
    base rim stays reachable. The screw is one body: a spherical tip, a 45 degree taper out
    to the thread root, the ISO thread, and a plain cylindrical knob at the top
    (run ``mechanisms.knurl`` over it if you want grip flutes). ``tip_ratio``
    sets the tip diameter as a fraction of ``screw_d``; keeping it below 1 stops
    the thread crest from touching the mouth of the seat.

    Bought set screws with ball ends and matching threaded inserts will always
    beat printed threads here; the printed screw is for prototyping and for
    parts where a few tenths of adjustment resolution is enough. Print the base
    seats-up, the table either way, and the screws tip-UP so the tip dome and
    the taper are self-supporting and the knob sits flat on the bed. Units are
    mm and degrees.
    """
    if kind not in _KINDS:
        raise ValueError("three_point_leveller(): kind must be one of %r" % (_KINDS,))
    if screw_len <= 0 or screw_pcd <= 0:
        raise ValueError("three_point_leveller(): screw_len and screw_pcd must be "
                         "positive")
    if float(screw_d) not in COARSE_PITCH:
        raise ValueError("three_point_leveller(): screw_d %r has no printable coarse "
                         "pitch; use one of %r" % (screw_d, sorted(COARSE_PITCH)))
    if not 0.35 <= tip_ratio <= 0.95:
        raise ValueError("three_point_leveller(): tip_ratio must lie in [0.35, 0.95]")
    if plate_t < 2.0 or base_t < 3.0:
        raise ValueError("three_point_leveller(): plate_t >= 2 and base_t >= 3 required")
    tip_d = tip_ratio * screw_d
    tip_r = tip_d / 2.0
    depth = tip_r
    if depth > base_t - 1.6:
        raise ValueError("three_point_leveller(): base_t %.2f is too thin for a "
                         "%.2f mm tip seat" % (base_t, tip_d))
    if screw_pcd < 3.0 * tip_d:
        raise ValueError("three_point_leveller(): screw_pcd %.2f is too small for "
                         "%.2f mm tips" % (screw_pcd, tip_d))
    plate_d = screw_pcd + 3.0 * screw_d if plate_d is None else float(plate_d)
    table_d = plate_d if table_d is None else float(table_d)
    if screw_pcd / 2.0 + screw_d > plate_d / 2.0 - 1.0:
        raise ValueError("three_point_leveller(): plate_d %.2f is too small for the "
                         "screw circle" % plate_d)
    if screw_pcd / 2.0 + screw_d > table_d / 2.0 - 1.0:
        raise ValueError("three_point_leveller(): table_d %.2f is too small to carry "
                         "the tapped holes at screw_pcd %.2f" % (table_d, screw_pcd))
    if lift < tip_r:
        raise ValueError("three_point_leveller(): lift %.2f must clear the ball tip "
                         "radius %.2f" % (lift, tip_r))
    pitch = coarse_pitch(screw_d)
    cutters, centres, contacts, z_c = _seat_layout(
        kind, screw_pcd, tip_d, depth, 2.0 * tip_d, phase_deg)
    if screw_len < lift - z_c + plate_t + 2.0 * pitch:
        raise ValueError("three_point_leveller(): screw_len %.2f is too short to "
                         "reach through the table at lift %.2f" % (screw_len, lift))

    base = cyl(plate_d / 2.0, base_t)
    base.apply_translation((0.0, 0.0, -base_t / 2.0))
    for cutter in cutters:
        base = sub(base, cutter)

    table = cyl(table_d / 2.0, plate_t)
    table.apply_translation((0.0, 0.0, lift + plate_t / 2.0))
    tap_cut = thread_solid(screw_d, plate_t + 2.0, internal=True,
                           clear=thread_clear)
    taper_h = (screw_d - tip_d) / 2.0
    rod = thread_solid(screw_d, screw_len)
    # Both helices start at their own local z=0, so the rod is turned about its
    # own axis by the phase its axial offset from the tapped hole implies.
    rod.apply_transform(tf.rotation_matrix(
        2.0 * math.pi * ((z_c + taper_h) - (lift - 1.0)) / pitch, (0, 0, 1)))
    screws = []
    for x, y, _z in centres:
        cut = tap_cut.copy()
        cut.apply_translation((x, y, lift - 1.0))
        table = sub(table, cut)
        tip = trimesh.creation.icosphere(subdivisions=int(ball_subdiv),
                                         radius=tip_r)
        tip.apply_translation((x, y, z_c))
        taper = frustum(tip_r, screw_d / 2.0, taper_h, z0=z_c)
        taper.apply_translation((x, y, 0.0))
        shaft = rod.copy()
        shaft.apply_translation((x, y, z_c + taper_h))
        knob = cyl(knob_d / 2.0, knob_t)
        knob.apply_translation((x, y, z_c + taper_h + screw_len + knob_t / 2.0))
        screws.append(uni([tip, taper, shaft, knob]))

    parts = {"base": base, "table": table, "screws": uni(screws)}
    return _stamp(parts, {
        "kind": kind,
        "dof_constrained": 6,
        "contacts": [[float(v) for v in p] for p in contacts],
        "tip_centers": [[float(v) for v in c] for c in centres],
        "tip_d": float(tip_d),
        "screw_pcd": float(screw_pcd),
        "thread_pitch": float(pitch),
        "mm_per_turn": float(pitch),
        # One turn of one screw tilts the table about the line through the other
        # two, which sits 1.5 screw-circle radii away.
        "tilt_per_turn_deg": float(math.degrees(
            math.atan2(pitch, 0.75 * screw_pcd))),
        "lift": float(lift),
        "plate_d": float(plate_d),
        "table_d": float(table_d),
    })
