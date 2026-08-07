"""Hardware stand-ins and display-quality oriented fasteners."""

import math

import numpy as np
import shapely.geometry as sg
import trimesh

from .meshutil import sub, uni
from .prim import boxc, cyl, frustum, hex_poly


def zmin0(m):
    """Translate a mesh so its minimum Z coordinate is zero.

    origin: parviz src/standins/_common.py:13
    """
    m.apply_translation((0, 0, -m.bounds[0][2]))
    return m


def bolt_mesh(shank_r, shank_l, head_r, head_h):
    """Build a head-down vertical bolt with its shank above the head.

    origin: parviz src/standins/_common.py:18
    """
    hd = cyl(head_r, head_h)
    hd.apply_translation((0, 0, head_h / 2))
    sh = cyl(shank_r, shank_l)
    sh.apply_translation((0, 0, head_h + shank_l / 2))
    return uni([hd, sh])


def hex_nut_mesh(af, h, bore_d):
    """Build a hex-nut stand-in from across-flats, height, and bore diameter.

    origin: parviz src/standins/_common.py:27
    """
    nt = cyl(af / math.sqrt(3.0), h, sections=6)
    nt = sub(nt, cyl(bore_d / 2, h + 2))
    return zmin0(nt)


def washer_mesh(od, id_, h):
    """Build a flat annular washer stand-in.

    origin: parviz src/standins/_common.py:33
    """
    r = sub(cyl(od / 2, h), cyl(id_ / 2, h + 2))
    return zmin0(r)


def _aligned(mesh, direction, origin):
    direction = np.asarray(direction, float)
    direction /= np.linalg.norm(direction)
    T = trimesh.geometry.align_vectors([0, 0, 1], direction)
    T[:3, 3] = origin
    mesh.apply_transform(T)
    return mesh


def fastener_mesh(d, L, style="pan", axis="z", at=(0.0, 0.0, 0.0), sections=32):
    """Build and orient a pan, socket-head, or countersunk fastener stand-in.

    The origin is the head bearing plane and the shank extends along ``axis``,
    which may be x, y, z, or a direction vector. Pan and SHCS heads sit proud
    opposite the axis; the CSK cone sinks along it. Dimensions scale from ``d``.
    origin: finnish-windows tools/add_screws_glb.py:33
    origin: finnish-windows tools/add_screws_glb2.py:27
    origin: finnish-doors src/shared/fasteners.py:115
    """
    if style not in ("pan", "shcs", "csk"):
        raise ValueError("fastener_mesh(): style must be 'pan', 'shcs', or 'csk'")
    if d <= 0 or L <= 0:
        raise ValueError("fastener_mesh(): d and L must be positive")
    direction = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0),
                 "z": (0.0, 0.0, 1.0)}.get(axis, axis)
    shaft = cyl(d / 2.0, L, sections=sections)
    shaft.apply_translation((0, 0, L / 2.0))
    if style == "csk":
        hh = 0.57 * d
        head = trimesh.creation.cone(radius=d, height=hh, sections=sections)
        head.apply_translation((0, 0, hh / 2.0))
    else:
        if style == "pan":
            rh, hh = 0.92 * d, 0.70 * d
        else:
            rh, hh = 0.92 * d, 1.00 * d
        head = cyl(rh, hh, sections=sections)
        head.apply_translation((0, 0, -hh / 2.0))
        if style == "shcs":
            socket_d = 0.55 * d
            socket_h = 0.25 * d
            socket = trimesh.creation.extrude_polygon(hex_poly(socket_d), socket_h + 0.1)
            socket.apply_translation((0, 0, -hh - 0.05))
            head = sub(head, socket)
    screw = uni([shaft, head])
    return _aligned(screw, direction, np.asarray(at, float))


def pick_length(span, std_lengths=(6, 8, 10, 12, 16, 20, 25, 30)):
    """Pick the first standard screw length no more than 0.6 mm short.

    origin: finnish-windows tools/add_screws_glb.py:19
    """
    for L in std_lengths:
        if L >= span - 0.6:
            return L
    return std_lengths[-1]


# Insert OD and standard lengths (mm) of heat-set threaded inserts,
# McMaster 94180A-series style.
_INSERT_TABLE = {
    "M2": (3.2, (3.0, 4.0)),
    "M2.5": (4.0, (4.0, 5.7)),
    "M3": (4.6, (4.0, 5.7, 8.0)),
    "M4": (6.3, (6.0, 8.1)),
    "M5": (7.1, (9.5,)),
    "M6": (9.5, (12.7,)),
}


def thread_insert(d="M3", length=None, boss=True, wall=2.4, clear=0.15,
                  sections=96):
    """Build a heat-set threaded insert, its receiving boss, and the cavity.

    The standard way to put durable machine-screw threads in FDM parts --
    enclosure lids that are opened repeatedly, motor mounts, adjustment
    points: a fluted brass-style insert is melted into a printed boss with
    a soldering iron. Returns ``{'insert', 'boss', 'cavity'}``: ``insert``
    is the insert body with straight axial flutes, a pilot taper at the
    z=0 (insertion) end and a printed internal ISO thread of designation
    ``d`` (``'M2'``..``'M6'``); ``boss`` is the solid receiving cylinder of
    ``wall`` around the insert to union into a part (dropped when
    ``boss=False``); ``cavity`` is the tapered cutter to subtract -- the
    taper self-pilots the insert during heat-setting and an entry chamfer
    eases starting. ``length`` defaults to the short standard size for the
    designation. All three are z=0 based and concentric. Print the insert
    thread-up; the cavity prints as a plain tapered bore with no overhang.
    Units are mm.
    """
    from .mechanisms import tap

    key = str(d).upper()
    if key not in _INSERT_TABLE:
        raise ValueError("thread_insert(): d must be one of %s"
                         % (sorted(_INSERT_TABLE),))
    insert_od, std_lengths = _INSERT_TABLE[key]
    if length is None:
        length = std_lengths[0]
    if length <= 0 or length > 2.0 * std_lengths[-1]:
        raise ValueError("thread_insert(): length must be in (0, %.1f] mm"
                         % (2.0 * std_lengths[-1],))
    if wall < 1.6:
        raise ValueError("thread_insert(): wall must be at least 1.6 mm")
    if clear < 0.0:
        raise ValueError("thread_insert(): clear must be non-negative")
    d_nom = float(key.lstrip("M"))

    r = insert_od / 2.0
    pilot = min(1.5, 0.25 * length)
    insert = uni([
        frustum(r - 0.5, r, pilot, z0=0.0, sections=int(sections)),
        cyl(r, length - pilot, center=(0, 0, (length + pilot) / 2.0),
            sections=int(sections)),
    ])
    flutes = []
    n_flutes = max(8, int(insert_od * 2.0))
    for k in range(n_flutes):
        a = 2.0 * math.pi * k / n_flutes
        flutes.append(cyl(0.35, length - pilot - 1.0,
                          center=((r - 0.25) * math.cos(a),
                                  (r - 0.25) * math.sin(a),
                                  (length + pilot) / 2.0), sections=8))
    insert = sub(insert, uni(flutes))
    insert = tap(insert, d_nom, (0.0, 0.0, 0.0), length, axis="z")

    cavity = uni([
        cyl(r + clear / 2.0, length, center=(0, 0, 2.0 + length / 2.0),
            sections=int(sections)),
        frustum(r + clear / 2.0 + 0.4, r + clear / 2.0, 0.4,
                z0=2.0 + length, sections=int(sections)),
        cyl(r - 0.5 + clear / 2.0, 2.4, center=(0, 0, 1.2),
            sections=int(sections)),
    ])

    meta = {
        "d": key,
        "insert_od": float(insert_od),
        "length": float(length),
        "wall": float(wall),
        "clear": float(clear),
        "std_lengths": tuple(float(v) for v in std_lengths),
    }
    parts = {"insert": insert, "cavity": cavity}
    if boss:
        parts["boss"] = cyl(r + wall, length + 2.0,
                            center=(0, 0, (length + 2.0) / 2.0),
                            sections=int(sections))
    for part in parts.values():
        part.metadata.update(meta)
    return parts


# Slot width (mm) of common T-slot aluminium extrusion profiles.
_TSLOT_TABLE = {"2020": 6.0, "3030": 8.0, "4040": 8.0}


def tslot_nut(profile="2020", thread_d="M4", style="drop_in",
              spring_leaf=True, sections=96):
    """Build a printed nut that keys into T-slotted aluminium extrusion.

    The standard way to bolt printed parts to 2020/3030/4040 printer frames
    and CNC gantries (distinct from ``closures.nut_slot``, which embeds a
    hex nut -- this IS the nut): a plate of ``wing_w`` span drops into the
    slot and rotates 90 degrees so its wings catch under the slot lips,
    with a central printed ISO thread of designation ``thread_d``
    (``'M3'``..``'M6'``). All four corners are chamfered so the plate
    rotates in the slot. With ``spring_leaf`` a low compliant ridge across
    the top face preloads the plate against the slot roof so it does not
    rattle before the screw is driven. Prints flat on its back, no
    support. Units are mm.
    """
    from .mechanisms import tap

    if profile not in _TSLOT_TABLE:
        raise ValueError("tslot_nut(): profile must be one of %s"
                         % (sorted(_TSLOT_TABLE),))
    if style not in ("drop_in", "slide_in"):
        raise ValueError("tslot_nut(): style must be 'drop_in' or "
                         "'slide_in'")
    key = str(thread_d).upper()
    d_nom = float(key.lstrip("M")) if key.startswith("M") else 0.0
    if not 3.0 <= d_nom <= 6.0:
        raise ValueError("tslot_nut(): thread_d must be 'M3'..'M6'")
    slot_w = _TSLOT_TABLE[profile]
    nut_t = 0.65 * slot_w
    wing_w = slot_w + 5.5
    length = wing_w if style == "drop_in" else 2.0 * wing_w
    if nut_t < d_nom * 0.6:
        raise ValueError("tslot_nut(): %s thread too coarse for a %.1f mm "
                         "plate" % (key, nut_t))

    cham = 1.5
    hw, hl = wing_w / 2.0, length / 2.0
    plate = sg.Polygon([
        (-hw + cham, -hl), (hw - cham, -hl), (hw, -hl + cham),
        (hw, hl - cham), (hw - cham, hl), (-hw + cham, hl),
        (-hw, hl - cham), (-hw, -hl + cham),
    ])
    nut = trimesh.creation.extrude_polygon(plate, nut_t)
    if spring_leaf:
        ridge = boxc((1.2, length - 2.0 * cham, 0.8),
                     center=(0.0, 0.0, nut_t - 0.2))
        nut = uni([nut, ridge])
    nut = tap(nut, d_nom, (0.0, 0.0, 0.0), nut_t, axis="z")

    nut.metadata.update({
        "profile": profile,
        "slot_w": float(slot_w),
        "thread_d": key,
        "style": style,
        "spring_leaf": bool(spring_leaf),
        "wing_w": float(wing_w),
        "nut_t": float(nut_t),
    })
    return nut


# DIN 6885 parallel-key cross sections (w x h, mm) by shaft diameter range.
_KEY_TABLE = [
    (8.0, 2.0, 2.0), (10.0, 3.0, 3.0), (12.0, 4.0, 4.0),
    (17.0, 5.0, 5.0), (22.0, 6.0, 6.0), (30.0, 8.0, 7.0),
]


def shaft_key(shaft_d=12.0, length=None, style="parallel", clear=0.1,
              sections=64):
    """Build a DIN 6885 machine key with its shaft and hub keyway cutters.

    Sunk keys transmit torque between a shaft and its hub where a setscrew
    or D-bore slips: gear hubs, hand cranks, pulley drives, motor
    couplings. The key cross section comes from the DIN 6885 table for
    ``shaft_d``; ``length`` defaults to twice the shaft diameter. With
    ``style='parallel'`` the key is a plain rectangular bar and both
    keyways are rectangular pockets (shaft depth 60% of the key height,
    hub depth the remainder plus 0.3 mm); ``style='woodruff'`` is the
    half-moon variant for tapered or lightly loaded shafts, with a
    matching half-disc shaft seat. Returns ``{'key', 'shaft_keyway',
    'hub_keyway'}``; the cutters are positioned for a shaft along +Z
    centred at the origin with the keyseat at +X, and are slightly
    over-length so they subtract cleanly. Print the key flat. Units
    are mm.
    """
    if shaft_d <= 0 or clear < 0:
        raise ValueError("shaft_key(): shaft_d must be positive, clear "
                         "non-negative")
    if style not in ("parallel", "woodruff"):
        raise ValueError("shaft_key(): style must be 'parallel' or "
                         "'woodruff'")
    row = next((r for r in _KEY_TABLE if shaft_d <= r[0]), None)
    if row is None:
        raise ValueError("shaft_key(): shaft_d over 30 mm is outside the "
                         "table")
    _, key_w, key_h = row
    if length is None:
        length = 2.0 * shaft_d
    if length < 2.0 * key_w:
        raise ValueError("shaft_key(): length must be at least 2 key "
                         "widths")

    t1 = 0.6 * key_h
    t2 = key_h - t1 + 0.3
    r_shaft = shaft_d / 2.0
    meta = {
        "shaft_d": float(shaft_d),
        "key_w": float(key_w),
        "key_h": float(key_h),
        "length": float(length),
        "style": style,
    }

    if style == "parallel":
        key = boxc((key_h - 0.2, key_w, length),
                   center=(r_shaft - t1 + (key_h - 0.2) / 2.0, 0.0, 0.0))
        shaft_kw = boxc((t1 + 0.5, key_w + clear, length + 2.0),
                        center=(r_shaft - t1 / 2.0 + 0.25, 0.0, 0.0))
        hub_kw = boxc((t2 + 0.8, key_w + clear, length + 2.0),
                      center=(r_shaft + t2 / 2.0, 0.0, 0.0))
    else:
        disc_r = 0.75 * shaft_d
        key = cyl(key_w / 2.0, 2.0 * disc_r, axis="y", sections=int(sections))
        key = sub(key, boxc((2.2 * disc_r, 2.0 * key_w, 2.2 * disc_r),
                            center=(0.0, 0.0, -disc_r)))
        key.apply_translation((r_shaft + key_w / 2.0 - t1, 0.0, 0.0))
        shaft_kw = cyl(key_w / 2.0 + clear / 2.0, 2.0 * disc_r, axis="y",
                       sections=int(sections))
        shaft_kw = sub(shaft_kw, boxc((2.2 * disc_r, 2.0 * key_w,
                                       2.2 * disc_r),
                                      center=(0.0, 0.0, -disc_r)))
        shaft_kw.apply_translation((r_shaft + key_w / 2.0 - t1 + 0.1,
                                    0.0, 0.0))
        hub_kw = boxc((t2 + 0.8, key_w + clear, 2.0 * disc_r),
                      center=(r_shaft + t2 / 2.0, 0.0, 0.0))
        meta["disc_r"] = float(disc_r)

    parts = {"key": key, "shaft_keyway": shaft_kw, "hub_keyway": hub_kw}
    for part in parts.values():
        part.metadata.update(meta)
    return parts


__all__ = (
    "zmin0",
    "bolt_mesh",
    "hex_nut_mesh",
    "washer_mesh",
    "fastener_mesh",
    "pick_length",
    "thread_insert",
    "tslot_nut",
    "shaft_key",
)
