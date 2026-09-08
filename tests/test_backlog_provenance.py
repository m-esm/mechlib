"""The pattern/lattice/kerf backlog must resolve to code plus commits.

Guards ``scripts/overnight/pattern-lattice-kerf-backlog.md`` against the drift
that ``design/roadmap/pattern-lattice-kerf-provenance.md`` measured: rows that
claim ``shipped`` while naming an API mechlib does not export, or carrying prose
like ``THIS HOUR`` where a commit belongs.
"""

import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "backlog_provenance.py"


def _shallow():
    out = subprocess.run(
        ["git", "rev-parse", "--is-shallow-repository"],
        cwd=ROOT, capture_output=True, text=True,
    )
    return out.stdout.strip() == "true"


def test_every_shipped_backlog_row_names_an_api_and_a_commit():
    if _shallow():
        pytest.skip("shallow clone cannot resolve historic SHAs")
    proc = subprocess.run(
        [sys.executable, str(SCRIPT), "-v"],
        cwd=ROOT, capture_output=True, text=True,
    )
    assert proc.returncode == 0, (
        "unverifiable backlog rows (id / api / reason):\n" + proc.stderr
    )
    assert proc.stdout.strip() == "0", proc.stdout
