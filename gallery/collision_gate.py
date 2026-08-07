#!/usr/bin/env python3
"""Gallery multi-body collision gate.

For every gallery demo:

1. Build the default pose.
2. If multi-body, measure pairwise solid overlap volumes.
3. If the demo is in ``ANIMATE``, rebuild across a handful of phase samples
   and fail when a body pair that was clear at rest gains solid overlap
   mid-cycle (free-moving parts colliding).

Designed contacts (gear mesh, cam-follower, clutch engagement, threaded
fits, pin-in-slot) are listed in ``ALLOW_CONTACT`` so the gate does not
flag intentional mating. Bonded display pairs that always overlap at rest
are auto-allowed: the gate only cares about *new* collisions.

Run::

    python3 gallery/collision_gate.py            # all demos
    python3 gallery/collision_gate.py demo_four_bar
    python3 -m pytest tests/test_gallery_collisions.py -q

Exit code 0 = clean, 1 = collisions (or build errors).
"""

from __future__ import annotations

import argparse
import importlib.util
import inspect
import itertools
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from mechlib.meshutil import bbox_gap, overlap_volume  # noqa: E402

# ---------------------------------------------------------------------------
# Thresholds
# ---------------------------------------------------------------------------

# Solid overlap above this (mm^3) counts as a collision. Tiny numerical
# mesh intersections from tessellated running fits sit well under it.
OVERLAP_MM3 = 0.5

# Pairs whose AABB gap exceeds this (mm) are treated as far apart.
BBOX_SKIP_MM = 0.15

# Animation samples per ANIMATE demo (including the rest pose).
ANIM_SAMPLES = 8

# ---------------------------------------------------------------------------
# Designed contacts: pairs that may acquire or keep solid overlap mid-cycle.
# Keys are demo function names; values are frozensets of body-name pairs.
# ---------------------------------------------------------------------------

ALLOW_CONTACT = {
    # Dog teeth drive through each other as engage_frac rises.
    "demo_dog_clutch": {
        frozenset(("hub_a", "hub_b")),
    },
    # Escapement impulse: pallet kisses a tooth twice per period.
    "demo_escapement": {
        frozenset(("anchor", "escape_wheel")),
    },
    # Cam / follower rolling or sliding contact.
    "demo_face_cam": {
        frozenset(("cam", "follower_pin")),
    },
    "demo_heart_cam": {
        frozenset(("heart_cam", "roller")),
        frozenset(("follower_stem", "roller")),
    },
    "demo_plate_cam": {
        frozenset(("plate_cam", "roller")),
        frozenset(("follower_stem", "roller")),
    },
    "demo_snail_cam": {
        frozenset(("snail_cam", "roller")),
        frozenset(("follower_stem", "roller")),
    },
    "demo_barrel_cam": {
        frozenset(("barrel_cam", "follower_pin")),
    },
    # Crank pin is fixed in the disc (press-fit display) and rides the slot.
    "demo_quick_return": {
        frozenset(("pin_crank", "slotted_lever")),
        frozenset(("crank_pin", "slotted_lever")),
        frozenset(("crank_disc", "pin_crank")),
        frozenset(("crank_disc", "crank_pin")),
    },
    "demo_scotch_yoke": {
        frozenset(("crank_pin", "slotted_yoke")),
        frozenset(("crank_disc", "crank_pin")),
    },
    # pin_1 is the throw pin pressed into the crank disc.
    "demo_slider_crank": {
        frozenset(("crank_disc", "pin_1")),
    },
    # Running fit of plug in body (tessellation noise ~1 mm^3).
    "demo_rotary_spool_valve": {
        frozenset(("spool_valve_body", "spool_valve_plug")),
    },
    # Threads / pad on the screw as lift changes.
    "demo_screw_jack": {
        frozenset(("base", "screw")),
        frozenset(("pad", "screw")),
    },
    # Swash shoes and shaft pass through the plate body by design of the
    # simplified demo (shoes are blocks, not bearing pads on the face).
    "demo_swash_plate": {
        frozenset(("plate", "shaft")),
        frozenset(("plate", "shoe_0")),
        frozenset(("plate", "shoe_1")),
        frozenset(("plate", "shoe_2")),
        frozenset(("plate", "shoe_3")),
    },
    # Strain-wave flex spline is forced into the circular spline.
    "demo_harmonic_drive": {
        frozenset(("circular_spline", "flex_spline")),
    },
    # Engaged face teeth / worm mesh (display pose is already interpenetrating).
    "demo_hirth_coupling": {
        frozenset(("hub_a", "hub_b")),
    },
    "demo_worm": {
        frozenset(("helical_wheel", "worm")),
    },
    # Press-fit demo intentionally overlaps the ribs into the reference body.
    "demo_crush_ribs": {
        frozenset(("component_reference", "tapered_ribs")),
    },
    # Chain lid snaps into the link body.
    "demo_drag_chain": {
        # lids seat into their own link; allow any lid_N/link_N and neighbours
    },
    "demo_drag_chain_link": {
        frozenset(("lid", "link")),
    },
    "demo_nut_slot": {
        frozenset(("nut_standin", "trap_cutaway")),
    },
    "demo_torque_limiter": {
        frozenset(("pocket_driven", "preload_spring")),
    },
}


def _load_demos():
    path = ROOT / "gallery" / "demos.py"
    spec = importlib.util.spec_from_file_location("gallery_demos_gate", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _load_animate():
    path = ROOT / "gallery" / "build_gallery.py"
    spec = importlib.util.spec_from_file_location("gallery_build_gate", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return dict(module.ANIMATE)


def _bodies(mesh_list):
    return {name: mesh for name, mesh, _color in mesh_list}


def _pair_volumes(parts):
    """Return {(a, b): overlap_mm3} for nearby body pairs (a < b)."""
    out = {}
    for a, b in itertools.combinations(sorted(parts), 2):
        if bbox_gap(parts[a], parts[b]) > BBOX_SKIP_MM:
            out[(a, b)] = 0.0
            continue
        try:
            out[(a, b)] = float(overlap_volume(parts[a], parts[b]))
        except Exception as exc:  # noqa: BLE001 - surface as gate failure
            out[(a, b)] = float("nan")
            out[(a, b, "error")] = str(exc)
    return out


def _allowed(demo_name, a, b):
    pair = frozenset((a, b))
    if pair in ALLOW_CONTACT.get(demo_name, ()):
        return True
    # Drag chain: any lid_* vs link_* is a snap-fit seat.
    if demo_name == "demo_drag_chain":
        names = {a, b}
        if any(n.startswith("lid_") for n in names) and any(
                n.startswith("link_") for n in names):
            return True
    return False


def check_demo(demo_name, demo_fn, animate=None, samples=ANIM_SAMPLES):
    """Return a list of failure strings (empty means pass)."""
    failures = []
    try:
        rest_list = demo_fn()
    except Exception as exc:  # noqa: BLE001
        return ["%s: build failed at rest: %s: %s"
                % (demo_name, type(exc).__name__, exc)]

    if not isinstance(rest_list, list) or not rest_list:
        return ["%s: demo returned empty / non-list" % demo_name]

    rest = _bodies(rest_list)
    if len(rest) < 2:
        return []  # single body: nothing can collide

    v0 = _pair_volumes(rest)
    for key, vol in v0.items():
        if not isinstance(key, tuple) or len(key) != 2:
            continue
        a, b = key
        if vol != vol:  # NaN
            failures.append(
                "%s: boolean failed for %s vs %s at rest" % (demo_name, a, b))
            continue
        if vol > OVERLAP_MM3 and not _allowed(demo_name, a, b):
            failures.append(
                "%s: rest overlap %.3g mm^3 between %s and %s "
                "(add to ALLOW_CONTACT if intentional)"
                % (demo_name, vol, a, b))

    # Pairs that are clear at rest (and not allow-listed) must stay clear.
    clear_pairs = [
        (a, b) for (a, b), vol in v0.items()
        if isinstance((a, b), tuple) and len((a, b)) == 2
        and vol == vol and vol <= OVERLAP_MM3 and not _allowed(demo_name, a, b)
    ]

    if not animate or demo_name not in animate:
        return failures

    param, cycle_deg, _frames, _closed = animate[demo_name]
    sig = inspect.signature(demo_fn).parameters
    if param not in sig:
        failures.append(
            "%s: ANIMATE param %r missing from signature" % (demo_name, param))
        return failures
    base = sig[param].default
    if base is inspect.Parameter.empty:
        failures.append(
            "%s: ANIMATE param %r has no default" % (demo_name, param))
        return failures
    base = float(base)

    n = max(2, int(samples))
    for i in range(n):
        phase = base + (i * float(cycle_deg) / n)
        try:
            posed = _bodies(demo_fn(**{param: phase}))
        except Exception as exc:  # noqa: BLE001
            failures.append(
                "%s: build failed at %s=%g: %s: %s"
                % (demo_name, param, phase, type(exc).__name__, exc))
            continue
        if set(posed) != set(rest):
            failures.append(
                "%s: body set changed at %s=%g" % (demo_name, param, phase))
            continue
        vv = _pair_volumes(posed)
        for a, b in clear_pairs:
            if _allowed(demo_name, a, b):
                continue
            vol = vv.get((a, b), 0.0)
            if vol != vol:
                failures.append(
                    "%s: boolean failed for %s vs %s at %s=%g"
                    % (demo_name, a, b, param, phase))
                continue
            if vol > OVERLAP_MM3:
                failures.append(
                    "%s: collision %.3g mm^3 between %s and %s at %s=%g"
                    % (demo_name, vol, a, b, param, phase))
    return failures


def all_demo_names(demos_module):
    return sorted(n for n in dir(demos_module) if n.startswith("demo_"))


def run_gate(only=None, samples=ANIM_SAMPLES, quiet=False):
    demos = _load_demos()
    animate = _load_animate()
    names = list(only) if only else all_demo_names(demos)
    t0 = time.time()
    failed = []
    passed = 0
    skipped_single = 0
    for name in names:
        fn = getattr(demos, name, None)
        if fn is None or not callable(fn):
            failed.append("%s: not a callable demo" % name)
            continue
        result = check_demo(name, fn, animate=animate, samples=samples)
        # Count single-body as pass without noise.
        try:
            n_bodies = len(_bodies(fn()))
        except Exception:
            n_bodies = -1
        if not result:
            passed += 1
            if n_bodies < 2:
                skipped_single += 1
            elif not quiet:
                tag = "anim" if name in animate else "static"
                print("  OK  %-32s (%s, %d bodies)" % (name, tag, n_bodies))
        else:
            failed.extend(result)
            for line in result:
                print("  FAIL %s" % line)

    elapsed = time.time() - t0
    print()
    print("gallery collision gate: %d ok (%d single-body), %d failure line(s), "
          "%.1fs" % (passed, skipped_single, len(failed), elapsed))
    return 0 if not failed else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "demos", nargs="*",
        help="optional demo_* names to check (default: all)")
    parser.add_argument(
        "--samples", type=int, default=ANIM_SAMPLES,
        help="animation samples per ANIMATE demo (default %d)" % ANIM_SAMPLES)
    parser.add_argument(
        "-q", "--quiet", action="store_true",
        help="only print failures and the summary line")
    args = parser.parse_args(argv)
    only = None
    if args.demos:
        only = []
        for name in args.demos:
            if not name.startswith("demo_"):
                name = "demo_" + name
            only.append(name)
    return run_gate(only=only, samples=args.samples, quiet=args.quiet)


if __name__ == "__main__":
    sys.exit(main())
