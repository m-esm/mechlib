"""Shared paths for the overnight motion-review and gap-research loops."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OVERNIGHT = ROOT / ".claude" / "overnight"
RENDERS = OVERNIGHT / "renders"
REVIEWS = OVERNIGHT / "reviews"
GAPS = OVERNIGHT / "gaps"

MOTION_QUEUE = OVERNIGHT / "motion_queue.json"
GAP_QUEUE = OVERNIGHT / "gap_queue.json"
CATALOG = OVERNIGHT / "catalog.json"
MOTION_REPORT = OVERNIGHT / "MOTION_REPORT.md"
GAP_REPORT = OVERNIGHT / "GAP_REPORT.md"
HEARTBEAT = OVERNIGHT / "heartbeat.json"
STOP = OVERNIGHT / "STOP"

MORNING_CUTOFF = "2026-08-13T07:30:00+03:00"  # EEST, do not stop before this


def ensure_dirs():
    for path in (OVERNIGHT, RENDERS, REVIEWS, GAPS):
        path.mkdir(parents=True, exist_ok=True)
