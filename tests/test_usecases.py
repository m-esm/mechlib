"""Use-case catalogue stays complete and queryable for agents."""

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


def test_use_cases_nonempty_and_concrete():
    assert len(USE_CASES) >= 100
    vague = ("useful", "various", "etc", "general purpose")
    for name, text in USE_CASES.items():
        assert len(text) > 24, name
        # Prefer concrete machines over pure filler.
        assert not text.lower().startswith("general"), name
