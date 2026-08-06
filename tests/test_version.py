"""Guard the one version invariant worth testing.

The version lives in two places (``pyproject.toml`` and ``mechlib/__init__``)
and the project workflow says to bump them together. Pinning the literal in
test files instead just spreads a third and fourth copy that go stale on every
release, so this compares the two real sources against each other.
"""

import re
from pathlib import Path

import mechlib

ROOT = Path(__file__).resolve().parents[1]


def test_package_version_matches_pyproject():
    text = (ROOT / "pyproject.toml").read_text(encoding="utf-8")
    match = re.search(r'^version\s*=\s*"([^"]+)"', text, re.MULTILINE)
    assert match, "pyproject.toml has no top-level version field"
    assert mechlib.__version__ == match.group(1), (
        "mechlib.__version__ is %r but pyproject.toml says %r; bump both together"
        % (mechlib.__version__, match.group(1))
    )
