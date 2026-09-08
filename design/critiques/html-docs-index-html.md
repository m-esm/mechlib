route: html:docs/index.html (GitHub Pages gallery, served from `main:/docs`)
source: docs/index.html + docs/models/index.json
shots:
  2026-09-07 WebGL (uiwalk via SwiftShader wrapper, CHROME_BIN=/tmp/chrome-gl.sh) against live https://m-esm.github.io/mechlib:
    ~/.hermes/design/shots/html-docs-index-html-20260907/desktop/03-hero.png
    ~/.hermes/design/shots/html-docs-index-html-20260907/desktop/06-gallery.png
    ~/.hermes/design/shots/html-docs-index-html-20260907/desktop/09-in-motion.png
    ~/.hermes/design/shots/html-docs-index-html-20260907/desktop/12-search-four-bar.png
    ~/.hermes/design/shots/html-docs-index-html-20260907/desktop/16-tune.png
    ~/.hermes/design/shots/html-docs-index-html-20260907/mobile/03-hero.png
    ~/.hermes/design/shots/html-docs-index-html-20260907/mobile/06-gallery.png
    ~/.hermes/design/shots/html-docs-index-html-20260907/mobile/16-tune.png
walked: 2026-09-07 `python3 ~/.hermes/scripts/uiwalk.py --flow /tmp/mechlib-review-20260907.json` viewports 1440×900 and 390×844. Flow: load, wait `.model-card`, hero, scroll, In motion, search `four_bar`, Tune. Console errors none; failed requests none (favicon not requested by this walk; curl `/favicon.ico` still HTTP 404).

Looked at every PNG with vision analysis.

WebGL desktop hero: VERSION **v0.11.0**, PARTS **190 + 32 utils**, **190 OF 190 PARTS**. Install `pip3 install git+https://github.com/m-esm/mechlib` is fully visible. Shelf All / Mechanical movements / Machine elements / Building blocks. Category chips Linkages 14 through Worm & planetary 6, with **Pumps & valves** cut at the right edge (`Pumps & valves ↵`). LINKAGES 14 parts. First row FOUR_BAR, TOGGLE_CLAMP, SCOTCH_YOKE, QUICK_RETURN with live-params / bodies / mm / animated badges and pastel 3D meshes in the card viewports. No WebGL-fail banner.

WebGL desktop after scroll: sticky toolbar holds. PEAUCELLIER_LINKAGE, WATT_LINKAGE, SARRUS_LINKAGE, PANTOGRAPH_LINKAGE meshes rendered. USED IN situations readable. COPY IMPORT / GLB / SOURCE / LINK under each card. TUNE ⚙ on the viewport.

WebGL desktop In motion: chip outlined; count **37 OF 190 PARTS**. Category chips re-count (Grippers 1, Gears 6, Ratchets 0, Worm & planetary 0). four_bar / toggle_clamp / scotch_yoke / quick_return meshes still render.

WebGL desktop search `four_bar` (In motion still on from the previous click): **1 OF 190 PARTS**, FOUR_BAR card with mesh. Empty grid to the right is the filter, not a crash.

WebGL desktop Tune on FOUR_BAR: modal title **FOUR_BAR**, CLOSE, signature `four_bar(crank_angle_deg=60, l_crank=12.5)`, sliders CRANK ANGLE DEG · MOTION 60 deg and L Crank 12.5 mm, PLAY / COPY CODE / DOWNLOAD STL / RESET DEFAULTS. Left pane is empty navy — no mesh. Footer in red: **runtime boot failed** … `ImportError: cannot import name 'cubic_lattice' from 'mechlib.lattices' … Did you mean: 'bcc_lattice'?`

WebGL mobile 390×844 first screen: Install clips at `github.com/m-esm/mecl`. Use wraps mid-token (`axis:`). Search placeholder clips at `descriptio` with a `press /` hint. Filter chips wrap. Category row clips `Grippers & cla`. No part mesh is on the first screen. VERSION/PARTS still **v0.11.0** / **190 + 32 utils**.

WebGL mobile after scroll: sticky search/shelf/chips remain. FOUR_BAR USED IN + Copy import / GLB / Source / Link; Tune visible. TOGGLE_CLAMP heading starts below; mesh still below the fold.

WebGL mobile Tune: FOUR_BAR / CLOSE, empty preview, signature line, then CRANK ANGLE slider overlapped by PLAY / COPY CODE / DOWNLOAD STL / RESET DEFAULTS. Same **runtime boot failed** `cubic_lattice` ImportError fills the lower half.

## verdict

Desktop catalog still earns its place: hero, live counts, shelf, cards with meshes and machinery “Used in” text. Search `four_bar` returns the one card. Tune does not: the playground boots into a traceback because the Pyodide `mechlib.lattices` has no `cubic_lattice`, so “retune it live” / Download STL is dead for this walk (opened on FOUR_BAR, not on a lattice). Mobile is still a chrome stack that clips install and search and hides the first mesh.

Looked for and holds (WebGL path): live Pages serves the gallery; v0.11.0 and 190+32; All/Movements/Elements/Blocks shelf; In motion reduces 190→37 and restyles the chip; four_bar-class meshes are recognizable; sticky toolbar survives scroll; card actions Copy import / GLB / Source / Link are visible on desktop; Tune modal opens.

Not seen in these shots: Latest added, Utility API rows, STL that actually downloads, a category with zero animated parts after filter alone, or lattice_flexure / honeycomb_core cards (those 404s are already a pending proposal, not re-filed here).

## debt

- [x] SEV=high `new THREE.WebGLRenderer` runs before `fetch("./models/index.json")`. A failed WebGL context leaves VERSION “loading”, PARTS “-”, an empty `#gallery`, and an empty Utility API, with no on-page error. Confirmed by uiwalk `--disable-gpu` (desktop+mobile 02-hero). Fixed: constructor is try/caught; catalog fetch still runs; on-page banner + card status when WebGL is missing.
- [x] SEV=high Tune playground runtime boot failed on live Pages: `ImportError: cannot import name 'cubic_lattice' from 'mechlib.lattices'` (Did you mean: `bcc_lattice`?). Cause: `docs/playground/demos.py` imported `cubic_lattice` while `docs/wheels/mechlib-0.11.0-py3-none-any.whl` was still the 2026-09-02 bcc-only build. Rebuilt the wheel from current `mechlib/` so Tune exec of demos.py can import.
- [ ] SEV=med At 390×844 the install line clips at `github.com/m-esm/mecl` with no wrap or overflow cue; Copy is the only way to recover the URL.
- [ ] SEV=med At 390×844 the search placeholder clips (`descriptio`) and `press /` occupies the field on a phone that has no slash shortcut.
- [ ] SEV=med The category chip row clips with no overflow hint: desktop cuts `Pumps & valves`, mobile cuts `Grippers & cla`.
- [ ] SEV=med At 390×844 the first viewport is only hero + toolbar; no part mesh is visible, so “every part below” is below the fold after a tall chrome stack.
- [ ] SEV=med At 390×844 the Tune modal stacks PLAY / COPY CODE / DOWNLOAD STL over the CRANK ANGLE slider, so even a successful boot would fight the chrome.
- [ ] SEV=low `/favicon.ico` returns 404.
