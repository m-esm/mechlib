"""A shipped pattern/lattice/kerf API must cite a retrievable source.

Locks slice 3 of ``design/roadmap/gallery-hides-the-citations.md``: once the
measure reaches 0 it must stay there. The probe reads the *visitor* path only
(the gallery card in ``docs/models/index.json`` plus
``mechlib.usecases.USE_CASES``), so a docstring reference does not satisfy it —
that is the whole point of the metric.

``backlog_row_cited.py`` guards the research table behind the same rule.
"""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "design" / "roadmap" / "evidence"


def _run(script):
    return subprocess.run(
        [sys.executable, str(EVIDENCE / script), "-v"],
        cwd=ROOT, capture_output=True, text=True,
    )


def test_every_shipped_api_is_citable_from_the_gallery():
    proc = _run("gallery_row_cited.py")
    assert proc.returncode == 0, (
        "APIs a gallery visitor cannot trace:\n" + proc.stderr
    )
    assert proc.stdout.split()[-1].startswith("0/"), proc.stdout


def test_every_shipped_backlog_row_cites_a_source():
    proc = _run("backlog_row_cited.py")
    assert proc.returncode == 0, (
        "backlog rows with no retrievable source:\n" + proc.stderr
    )
    assert proc.stdout.split()[-1].startswith("0/"), proc.stdout
