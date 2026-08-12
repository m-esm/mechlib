#!/usr/bin/env python3
"""Commit overnight loop progress on the feature branch and push it.

Refuses to commit on main/master. Stages scripts, reports, reviews, and
queues. Leaves PNG renders untracked.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from paths import GAP_REPORT, MOTION_REPORT, OVERNIGHT, ROOT
from workqueue import summarize

BRANCH = "overnight/visual-and-gaps"
FORBIDDEN = {"main", "master"}

TRACKED = [
    "scripts/overnight/",
    ".gitignore",
    ".claude/overnight/MOTION_REPORT.md",
    ".claude/overnight/GAP_REPORT.md",
    ".claude/overnight/reviews/",
    ".claude/overnight/gaps/",
    ".claude/overnight/catalog.json",
    ".claude/overnight/motion_queue.json",
    ".claude/overnight/gap_queue.json",
]


def run(args, check=True):
    return subprocess.run(
        args,
        cwd=str(ROOT),
        check=check,
        capture_output=True,
        text=True,
    )


def current_branch() -> str:
    return run(["git", "rev-parse", "--abbrev-ref", "HEAD"]).stdout.strip()


def ensure_branch() -> str:
    branch = current_branch()
    if branch == BRANCH:
        return branch
    if branch in FORBIDDEN:
        exists = run(["git", "rev-parse", "--verify", BRANCH], check=False)
        if exists.returncode == 0:
            run(["git", "checkout", BRANCH])
        else:
            run(["git", "checkout", "-b", BRANCH])
        return current_branch()
    raise SystemExit("overnight commit refused on branch %r (want %s)" % (branch, BRANCH))


def stage() -> None:
    for path in TRACKED:
        full = ROOT / path
        if full.exists():
            run(["git", "add", "-A", "--", path])


def has_staged() -> bool:
    diff = run(["git", "diff", "--cached", "--quiet"], check=False)
    return diff.returncode != 0


def message(loop: str) -> str:
    stats = summarize()
    motion = stats.get("motion") or {}
    gap = stats.get("gap") or {}
    return (
        "Overnight %s: motion %s/%s reviewed, gaps %s/%s\n\n"
        "Visual fails=%s unclear=%s. Gap pending=%s.\n"
        % (
            loop,
            motion.get("reviewed", 0),
            motion.get("total", 0),
            gap.get("done", 0),
            gap.get("total", 0),
            motion.get("failed_review", 0),
            motion.get("unclear", 0),
            gap.get("pending", 0),
        )
    )


def push() -> str:
    result = run(["git", "push", "-u", "origin", "HEAD"], check=False)
    if result.returncode != 0:
        return "push failed: %s" % ((result.stderr or result.stdout).strip()[:400],)
    return "pushed"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--loop", default="progress")
    parser.add_argument("--no-push", action="store_true")
    args = parser.parse_args(argv)
    branch = ensure_branch()
    if branch in FORBIDDEN:
        raise SystemExit("refusing to commit on %s" % branch)
    stage()
    if not has_staged():
        print("no overnight changes to commit")
        return 0
    msg = message(args.loop)
    run(["git", "commit", "-m", msg])
    sha = run(["git", "rev-parse", "--short", "HEAD"]).stdout.strip()
    extra = "local only"
    if not args.no_push:
        extra = push()
    print("committed %s on %s (%s)" % (sha, branch, extra))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
