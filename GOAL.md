# GOAL (edit freely)

A growing mechanical pattern library for Python-fluent makers and AI coding agents who need researched FDM-printable pattern, lattice, and kerf cells as parametric semi-primitives (explicit args, watertight mesh, live gallery). Done: at least 20 researched pattern/lattice/kerf items in the list, one new implemented every hour, sortable by newest. It is explicitly NOT a full parametric CAD system, a GUI modeler, or a home for finished or branded product assemblies; those stay in consumer projects.

## Numbers that prove it
- researched pattern/lattice/kerf items in the list: `python3 -c "import re; from pathlib import Path; print(sum(1 for l in Path('scripts/overnight/pattern-lattice-kerf-backlog.md').read_text().splitlines() if re.match(r'\|\\s*\\d\\d\\s*\\|', l)))"` - today: 22; target: 20
- of those, shipped (implemented): `python3 -c "import re; from pathlib import Path; print(sum(1 for l in Path('scripts/overnight/pattern-lattice-kerf-backlog.md').read_text().splitlines() if re.match(r'\|\\s*\\d\\d\\s*\\|', l) and '| shipped |' in l))"` - today: 21; target: 20, then one pending row per hour until the queue is empty
- sortable by newest: live gallery Latest-added chip; every card carries `added` in `docs/models/index.json`

source: scripts/overnight/pattern-lattice-kerf-backlog.md, gallery/build_gallery.py, docs/models/index.json, mechlib/lattices.py, mechlib/flexures.py
