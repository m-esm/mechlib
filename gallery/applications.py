"""Gallery adapter for machinery use-case text.

Source of truth is ``mechlib.usecases`` (public API names for AI agents and
humans). This module only maps gallery GLB filenames onto that catalogue.
"""

from mechlib.usecases import (  # noqa: F401 — re-export for build_gallery
    GALLERY_FILE_OVERRIDES,
    GALLERY_FILE_TO_API,
    USE_CASES,
    applications_for_file,
    search_use_cases,
    use_case,
)

# Back-compat name used by build_gallery / older scripts.
APPLICATIONS = {
    f: applications_for_file(f)
    for f in list(GALLERY_FILE_TO_API) + list(GALLERY_FILE_OVERRIDES)
}


def applications_for(file_name, description=""):
    """Return the applications line for a gallery GLB."""
    return applications_for_file(file_name, description)
