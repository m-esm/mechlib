"""Use-case catalogue stays complete and queryable for agents."""

import json
from pathlib import Path

from mechlib.usecases import (
    GALLERY_FILE_TO_API,
    USE_CASES,
    applications_for_file,
    search_use_cases,
    use_case,
)


def test_use_case_known_api():
    text = use_case("four_bar")
    assert "robot" in text.lower() or "walking" in text.lower()


def test_use_case_alias():
    assert use_case("slider_crank_pose") == use_case("slider_crank")


def test_use_case_unknown_raises():
    try:
        use_case("not_a_real_api_zzz")
    except KeyError:
        return
    raise AssertionError("expected KeyError")


def test_search_use_cases_finds_robot_parts():
    hits = search_use_cases("robot")
    assert hits
    names = {n for n, _ in hits}
    assert names & {"cycloidal_drive", "harmonic_drive", "four_bar",
                    "planet_stage", "ball_socket_joint"}


def test_every_gallery_file_has_applications():
    for glb, api in GALLERY_FILE_TO_API.items():
        text = applications_for_file(glb)
        assert text and len(text) > 20, glb
        # Override or mapped API must resolve.
        assert api in USE_CASES, (glb, api)


def test_every_mapped_gallery_file_exists_and_is_catalogued():
    models_dir = Path(__file__).resolve().parents[1] / "docs" / "models"
    index = json.loads((models_dir / "index.json").read_text(encoding="utf-8"))
    listed = {model["file"] for model in index["models"]}
    missing_files = sorted(
        file for file in GALLERY_FILE_TO_API if not (models_dir / file).is_file()
    )
    missing_rows = sorted(set(GALLERY_FILE_TO_API) - listed)
    assert not missing_files and not missing_rows, (
        f"Missing GLBs: {missing_files}; missing catalog rows: {missing_rows}"
    )


def test_use_cases_nonempty_and_concrete():
    assert len(USE_CASES) >= 100
    vague = ("useful", "various", "etc", "general purpose")
    for name, text in USE_CASES.items():
        assert len(text) > 24, name
        # Prefer concrete machines over pure filler.
        assert not text.lower().startswith("general"), name
