"""Sweep playground demo defaults and PLAY param extremes."""

from __future__ import annotations

import importlib.util
import inspect
import math
import sys
from pathlib import Path

import numpy as np
import pytest
import trimesh

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

_DEMOS_PATH = ROOT / "gallery" / "demos.py"
_spec = importlib.util.spec_from_file_location("gallery_demos", _DEMOS_PATH)
demos = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(demos)

PLAY = demos.PLAY


def _assert_mesh_list(result, label):
    assert isinstance(result, list), "%s: expected list, got %r" % (label, type(result))
    assert result, "%s: empty mesh list" % label
    for item in result:
        assert isinstance(item, tuple) and len(item) == 3, (
            "%s: entry must be (name, mesh, color), got %r" % (label, item)
        )
        name, mesh, color = item
        assert isinstance(name, str) and name, "%s: bad name %r" % (label, name)
        assert isinstance(mesh, trimesh.Trimesh), (
            "%s/%s: not a Trimesh" % (label, name)
        )
        assert isinstance(color, tuple) and len(color) in (3, 4), (
            "%s/%s: bad color %r" % (label, name, color)
        )
        bounds = np.asarray(mesh.bounds, dtype=float)
        assert np.all(np.isfinite(bounds)), (
            "%s/%s: non-finite bounds %s" % (label, name, bounds)
        )
        assert mesh.vertices.size > 0, "%s/%s: empty vertices" % (label, name)


def _param_values(pname, pmin, pmax, default):
    """Return (min_value, max_value) coerced to the default's type."""
    if isinstance(default, bool):
        return bool(int(pmin)), bool(int(pmax))
    if isinstance(default, int) and not isinstance(default, bool):
        return int(pmin), int(pmax)
    return float(pmin), float(pmax)


@pytest.mark.parametrize("demo_name", sorted(PLAY.keys()))
def test_demo_play_defaults_and_extremes(demo_name):
    demo_fn = getattr(demos, demo_name)
    sig = inspect.signature(demo_fn)
    result = demo_fn()
    _assert_mesh_list(result, "%s()" % demo_name)

    for pname, (pmin, pmax, _pstep) in PLAY[demo_name].items():
        assert pname in sig.parameters, "%s missing param %s" % (demo_name, pname)
        default = sig.parameters[pname].default
        assert default is not inspect.Parameter.empty
        vmin, vmax = _param_values(pname, pmin, pmax, default)
        for edge_name, value in (("min", vmin), ("max", vmax)):
            kwargs = {pname: value}
            label = "%s(%s=%r)" % (demo_name, pname, value)
            try:
                edge_result = demo_fn(**kwargs)
            except Exception as exc:  # noqa: BLE001, surface as test failure
                pytest.fail("%s raised %s: %s" % (label, type(exc).__name__, exc))
            _assert_mesh_list(edge_result, label)
