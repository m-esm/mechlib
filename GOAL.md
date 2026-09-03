# GOAL (draft by Pawl 2026-09-03, edit freely)

For Python-fluent makers and AI coding agents building FDM-printable mechanisms (robot joints, fixtures, kinetic products) who need parametric semi-primitive parts (gears, cams, linkages, flexures, ratchets, snaps, bought-part envelopes) without hand-modeling primitives or guessing dimensions. Done: an agent can `search_use_cases("job")`, call one mechlib function with explicit args, get a watertight mesh that prints, and see it in the live gallery, without inventing geometry from boxes and cylinders. It is explicitly NOT a full parametric CAD system, a GUI modeler, or a home for finished or branded product assemblies; those stay in consumer projects.

## Numbers that prove it
- public API covered by a use case: `python3 -c "import mechlib; from mechlib.usecases import USE_CASES, ALIASES; a=set(mechlib.__all__); print(len(a & (set(USE_CASES)|set(ALIASES))), len(a))"` - today: 190/263; target: 263/263
- use cases with a gallery demo GLB: `python3 -c "from mechlib.usecases import USE_CASES, GALLERY_FILE_TO_API as g; print(len(set(USE_CASES)&set(g.values())), len(USE_CASES))"` - today: 175/175; target: stays 100% as USE_CASES grows
- printed-and-verified parts: demos with a `design/critiques/` look that states it was sliced or printed - today: 0; target: 20

source: README.md, CLAUDE.md, AGENTS.md, mechlib/usecases.py, gallery/build_gallery.py
