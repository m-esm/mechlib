#!/usr/bin/env python3
"""Derive the overnight work lists from live sources.

Motion queue comes from gallery ANIMATE + movement-group demos.
Gap queue is a first-principles research agenda (kinematic pairs,
textbook families, machine-element classes), not a restatement of
the existing module list. Existing APIs are recorded so researchers
can mark a candidate covered rather than invent a duplicate.
"""

from __future__ import annotations

import importlib.util
import inspect
import json
import re
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from paths import CATALOG, GAP_QUEUE, MOTION_QUEUE, ROOT, ensure_dirs
from workqueue import dump_json, load_json, now_iso

# First-principles research slices. These are families of mechanisms, not
# mechlib modules. The overnight gap loop must survey the class, then
# subtract what the live catalog already covers.
RESEARCH_SLICES = (
    {
        "id": "kinematic-pairs",
        "title": "Lower and higher kinematic pairs",
        "prompt": (
            "Reuleaux / IFToMM pairs: revolute, prismatic, helical, cylindrical, "
            "spherical, planar, plus higher pairs (cam-follower, gear mesh, "
            "rolling contact). Which pairs have a mechlib semi-primitive, which "
            "are only implied by a larger assembly, and which pair-level "
            "generators are missing?"
        ),
    },
    {
        "id": "planar-four-bar",
        "title": "Planar four-bar and cognates",
        "prompt": (
            "Crank-rocker, double-crank, double-rocker, parallelogram, deltoid, "
            "change-point, Grashof vs non-Grashof, Roberts/Chebyshev cognates, "
            "Hoecken. What does four_bar plus the named linkages already cover, "
            "and what named 4-bar still needs its own pose+kit?"
        ),
    },
    {
        "id": "six-bar-and-walking",
        "title": "Six-bar, walking, and animal-motion linkages",
        "prompt": (
            "Watt and Stephenson six-bar topologies, Klann, Theo Jansen, "
            "strandbeest-class, walking-beam / pumpjack, plantigrade. These are "
            "the usual 'missing from a 4-bar kit' request. Which are "
            "semi-primitive enough for mechlib vs a finished walking robot?"
        ),
    },
    {
        "id": "straight-line",
        "title": "Straight-line and inversor linkages",
        "prompt": (
            "Peaucellier, Hart, Watt, Chebyshev, Scott-Russell, Hoecken, Roberts, "
            "Evans, grasshopper, Tusi couple, Cardanic motion, trammel of "
            "Archimedes, Sarrus (already spatial). Catalog each; mark exact vs "
            "approximate; say which pose APIs exist."
        ),
    },
    {
        "id": "spatial-spherical-parallel",
        "title": "Spatial, spherical, and parallel mechanisms",
        "prompt": (
            "Bennett, spherical four-bar, 7R, Agile Eye, delta, Stewart/Gough, "
            "5-bar planar robot, serial RRR arm primitive, roll-pitch-roll wrist, "
            "Hoberman / deployable. Filter hard: finished robot chassis is out of "
            "scope; a parametric delta-joint kit or spherical 4-bar pose may not be."
        ),
    },
    {
        "id": "cams-indexing",
        "title": "Cam types and intermittent motion",
        "prompt": (
            "Plate, face, barrel, heart, snail already exist. Still in the class: "
            "globoidal / Ferguson indexing cam, conjugate / desmodromic pairs, "
            "inverse cam, linear / wedge cam, cylindrical face-groove, barrel "
            "indexer, Geneva variants (internal, spherical), star wheel vs "
            "Maltese, intermittent gears, ratchet-index tables."
        ),
    },
    {
        "id": "gear-families",
        "title": "Gear and reduction families",
        "prompt": (
            "Spur, internal, herringbone, bevel, worm, cycloidal, harmonic, rack, "
            "sector, planetary already exist. Survey helical (single), face/crown, "
            "hypoid, spiroid, crossed-helical, lantern/pin, noncircular/elliptical, "
            "magnetic, nutating/wobble, compound trains, automotive differential, "
            "sun-and-planet, hypoid-ish printable stand-ins. Which are FDM-real?"
        ),
    },
    {
        "id": "clutches-brakes-overrunning",
        "title": "Clutches, brakes, and overrunning",
        "prompt": (
            "Dog, freewheel, torque limiter, compliant clutch, Hirth exist. "
            "Missing class? Cone, multiplate, wrap-spring, sprag, roller-ramp, "
            "centrifugal, magnetic, fluid coupling. Brakes as a category: band, "
            "shoe, disc/caliper, over-center park. Which are printable "
            "semi-primitives vs bought hardware?"
        ),
    },
    {
        "id": "couplings-cv",
        "title": "Shaft couplings and CV joints",
        "prompt": (
            "Oldham, jaw, beam, Hooke, double Cardan, tripod exist. Survey "
            "Rzeppa/ball CV, Tracta, Thompson, Schmidt, bellows, disc/membrane, "
            "grid, chain coupling, homokinetic joints. Note which have pose laws "
            "in-module (needed to animate) vs display-only kits."
        ),
    },
    {
        "id": "screws-linear-actuators",
        "title": "Screws, jacks, and linear actuators",
        "prompt": (
            "Lead screw, differential screw, screw jack, scroll, archimedes, "
            "telescoping stage, linear way exist. Survey ball screw, roller "
            "screw, scotch-yoke (have), rack (have), wedge/inclined plane, "
            "hydraulic/pneumatic cylinder geometry, voice-coil stand-in, "
            "tape measure / constant-force Negator, cable cylinder."
        ),
    },
    {
        "id": "flexure-compliant",
        "title": "Compliant mechanisms and flexure joints",
        "prompt": (
            "Cross flexure, bistable beam, flexure stage, leaf/coil/wave/torsion/"
            "spiral springs, kerf, auxetic exist. Survey cartwheel hinge, "
            "living hinge as API, lamina emergent, constant-force compliant, "
            "contact-aided, ORBITAL / remote-center, notch hinge primitives, "
            "split-tube flexure, tape-spring hinge."
        ),
    },
    {
        "id": "clockwork-escapements",
        "title": "Clockwork, escapements, and energy storage",
        "prompt": (
            "Escapement, fusee, spiral power spring, ratchet exist. Survey "
            "deadbeat vs recoil vs detent vs coaxial vs gravity, going barrel, "
            "remontoire, maintaining power, count wheel, striking (probably "
            "product), stackfreed, constant-force mechanisms, gravity arm."
        ),
    },
    {
        "id": "pulleys-cables-tackle",
        "title": "Pulleys, cables, tackle, and textile hardware",
        "prompt": (
            "Timing, V-belt, idler, tensioner, winch, fusee, grooved drum exist. "
            "Survey block-and-tackle, Weston differential pulley, capstan, "
            "self-tailing winch, cam/clam cleat, rope clutch, fairlead, turning "
            "block, Spanish windlass, belt CVT variable pulley, cable carrier "
            "is already drag_chain."
        ),
    },
    {
        "id": "fluid-pumps-valves",
        "title": "Pumps, valves, and fluid machines",
        "prompt": (
            "Gerotor, external gear, peristaltic, spool, check, hose barb exist. "
            "Survey vane, lobe, screw, piston/swash (swash_plate exists), "
            "diaphragm, centrifugal impeller, needle/ball/gate/butterfly valves, "
            "pressure relief, shuttle, manifold block, Tesla valve, pulsatile."
        ),
    },
    {
        "id": "grippers-chucks-workholding",
        "title": "Grippers, chucks, latches, and workholding",
        "prompt": (
            "Toggle clamp, eccentric cam clamp, collet, bellows cup, iris, "
            "kinematic coupling, dock, leveller exist. Survey parallel jaw "
            "gripper as semi-primitive, angular gripper, latch/pawl catch, "
            "over-center hood latch, draw latch, vises, soft jaw, magnetic "
            "chuck, vacuum cup families, tool-changer plate (if primitive)."
        ),
    },
    {
        "id": "bearings-ways-seals",
        "title": "Bearings, ways, and seals",
        "prompt": (
            "Plain bushing, printed ball, thrust, linear way, oring, labyrinth, "
            "gasket exist. Survey crossed-roller, dovetail way (ydovetail is a "
            "cutter), V-way, air bearing (out?), magnetic bearing (out?), "
            "needle roller stand-in, shaft seal / lip, piston ring, wiper."
        ),
    },
    {
        "id": "missing-categories",
        "title": "Whole categories mechlib does not name",
        "prompt": (
            "Look at the live CATEGORIES table and name mechanism classes that "
            "are not a current shelf: brakes, dampers/dashpots, differentials, "
            "parallel robots, walking, cable/tendon, constant-force, magnetic "
            "drives, traction/CVT, brakes vs clutches split, end-effectors vs "
            "grippers, tooling/ATC, heat? (out). For each proposed NEW category: "
            "is it a real machinery class, FDM-printable, semi-primitive, and "
            "not a finished product? Rank the ones that should become a module."
        ),
    },
    {
        "id": "animate-coverage",
        "title": "Mechanisms that exist but cannot play",
        "prompt": (
            "Read gallery/build_gallery.py ANIMATE and the 'deliberately left "
            "out' comment. For every multi-body movement demo with no animation: "
            "is the reason still true, or can a pose law be added so the gallery "
            "can play it? Rank the highest-value pose-law additions. Do not "
            "propose animating static adjustments (eccentric_idler_mount)."
        ),
    },
)


def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def extract_left_out(build_src: str) -> str:
    match = re.search(
        r"Mechanisms deliberately left out.*?(?=\n_MAX_STEP_DEG)",
        build_src,
        flags=re.S,
    )
    return match.group(0).strip() if match else ""


def demo_to_api(demo_name: str) -> str:
    return demo_name[5:] if demo_name.startswith("demo_") else demo_name


def main() -> int:
    ensure_dirs()
    import mechlib
    from mechlib.usecases import USE_CASES

    demos_mod = _load_module(ROOT / "gallery" / "demos.py", "gallery_demos_inventory")
    build_mod = _load_module(ROOT / "gallery" / "build_gallery.py", "gallery_build_inventory")
    index = json.loads((ROOT / "docs" / "models" / "index.json").read_text())
    build_src = (ROOT / "gallery" / "build_gallery.py").read_text()

    demo_fns = {
        name: fn
        for name, fn in inspect.getmembers(demos_mod, inspect.isfunction)
        if name.startswith("demo_")
    }
    animate = {
        name: {
            "param": spec[0],
            "cycle_deg": spec[1],
            "frames": spec[2],
            "closed": spec[3],
        }
        for name, spec in build_mod.ANIMATE.items()
    }

    models_by_demo = {}
    for model in index.get("models") or []:
        play = model.get("play") or {}
        demo_name = play.get("demo") or ("demo_" + model.get("name", ""))
        models_by_demo[demo_name] = model

    demos_info = {}
    for demo_name, fn in sorted(demo_fns.items()):
        model = models_by_demo.get(demo_name, {})
        api = demo_to_api(demo_name)
        demos_info[demo_name] = {
            "api": api,
            "doc": (inspect.getdoc(fn) or "").split("\n")[0],
            "signature": "%s%s" % (demo_name, inspect.signature(fn)),
            "animated": demo_name in animate,
            "animate": animate.get(demo_name),
            "file": model.get("file"),
            "category": model.get("category"),
            "group": model.get("group"),
            "parts": model.get("parts"),
            "description": model.get("description"),
            "applications": model.get("applications"),
            "usecase": USE_CASES.get(api),
            "has_usecase": api in USE_CASES,
        }

    public = list(mechlib.__all__)
    usecase_keys = sorted(USE_CASES)
    catalog = {
        "built_at": now_iso(),
        "version": getattr(mechlib, "__version__", None),
        "public_api": public,
        "public_api_count": len(public),
        "usecases": usecase_keys,
        "usecases_missing_api": [k for k in usecase_keys if k not in set(public)],
        "api_missing_usecase": [k for k in public if k not in USE_CASES],
        "categories": {
            key: {"group": val[0], "title": val[1], "blurb": val[2]}
            for key, val in build_mod.CATEGORIES.items()
        },
        "category_groups": [
            {"key": k, "title": t, "blurb": b}
            for k, t, b in build_mod.CATEGORY_GROUPS
        ],
        "animate": animate,
        "left_out_notes": extract_left_out(build_src),
        "demos": demos_info,
        "demo_count": len(demos_info),
        "animated_count": len(animate),
        "research_slices": [dict(s) for s in RESEARCH_SLICES],
    }
    dump_json(CATALOG, catalog)

    # Motion queue: ANIMATE first, then other movement-group multi-body demos,
    # then remaining movement-group singles (rest pose only).
    existing = {
        item["id"]: item
        for item in load_json(MOTION_QUEUE, {"items": []}).get("items") or []
    }
    motion_items = []

    def push(demo_name: str, priority: int, kind: str):
        prev = existing.get(demo_name, {})
        info = demos_info[demo_name]
        motion_items.append({
            "id": demo_name,
            "priority": priority,
            "kind": kind,
            "category": info.get("category"),
            "group": info.get("group"),
            "parts": info.get("parts"),
            "animated": info.get("animated"),
            "rendered": prev.get("rendered", False),
            "reviewed": prev.get("reviewed", False),
            "verdict": prev.get("verdict"),
            "sheet": prev.get("sheet"),
            "render_error": prev.get("render_error"),
        })

    for name in sorted(animate):
        if name in demo_fns:
            push(name, 0, "animate")
    for name, info in sorted(demos_info.items()):
        if name in animate:
            continue
        if info.get("group") != "movements":
            continue
        parts = info.get("parts") or 1
        push(name, 1 if parts and parts >= 2 else 2, "movement")

    motion_items.sort(key=lambda i: (i["priority"], i["id"]))
    dump_json(MOTION_QUEUE, {"updated": now_iso(), "items": motion_items})

    existing_gap = {
        item["id"]: item
        for item in load_json(GAP_QUEUE, {"items": []}).get("items") or []
    }
    gap_items = []
    for spec in RESEARCH_SLICES:
        prev = existing_gap.get(spec["id"], {})
        item = dict(spec)
        item["status"] = prev.get("status", "pending")
        item["report"] = prev.get("report")
        item["claimed_at"] = prev.get("claimed_at")
        item["completed_at"] = prev.get("completed_at")
        gap_items.append(item)
    dump_json(GAP_QUEUE, {"updated": now_iso(), "items": gap_items})

    print("catalog   %s  (%d APIs, %d demos, %d animated)"
          % (CATALOG, catalog["public_api_count"], catalog["demo_count"],
             catalog["animated_count"]))
    print("motion    %s  (%d items)" % (MOTION_QUEUE, len(motion_items)))
    print("gap       %s  (%d slices)" % (GAP_QUEUE, len(gap_items)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
