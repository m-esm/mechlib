"""Per-demo gallery collision gate.

Each multi-body gallery entry is built at rest and, when listed in
``ANIMATE``, across a phase sweep. A pair that was clear at rest and later
gains solid overlap fails the test. Designed contacts live in
``gallery.collision_gate.ALLOW_CONTACT``.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

_GATE_PATH = ROOT / "gallery" / "collision_gate.py"
_spec = importlib.util.spec_from_file_location("gallery_collision_gate", _GATE_PATH)
_gate = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
_spec.loader.exec_module(_gate)

_demos = _gate._load_demos()
_animate = _gate._load_animate()
_DEMO_NAMES = _gate.all_demo_names(_demos)


@pytest.mark.parametrize("demo_name", _DEMO_NAMES)
def test_gallery_demo_no_new_collisions(demo_name):
    fn = getattr(_demos, demo_name)
    failures = _gate.check_demo(
        demo_name, fn, animate=_animate, samples=_gate.ANIM_SAMPLES)
    assert not failures, "\n".join(failures)
