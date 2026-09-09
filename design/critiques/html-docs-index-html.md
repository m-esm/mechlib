route: html:docs/index.html (GitHub Pages gallery, served from `main:/docs`)
source: docs/index.html + docs/models/index.json
shots:
  2026-09-09 WebGL (uiwalk via SwiftShader wrapper, CHROME_BIN=/tmp/chrome-gl.sh) against live https://m-esm.github.io/mechlib/:
    ~/.hermes/design/shots/html-docs-index-html-20260909/desktop/03-hero.png
    ~/.hermes/design/shots/html-docs-index-html-20260909/desktop/06-latest.png
    ~/.hermes/design/shots/html-docs-index-html-20260909/desktop/08-latest-scroll.png
    ~/.hermes/design/shots/html-docs-index-html-20260909/desktop/12-in-motion.png
    ~/.hermes/design/shots/html-docs-index-html-20260909/desktop/17-search-flexure.png
    ~/.hermes/design/shots/html-docs-index-html-20260909/desktop/22-tune.png
    ~/.hermes/design/shots/html-docs-index-html-20260909/mobile/03-hero.png
    ~/.hermes/design/shots/html-docs-index-html-20260909/mobile/06-latest.png
    ~/.hermes/design/shots/html-docs-index-html-20260909/mobile/08-latest-scroll.png
    ~/.hermes/design/shots/html-docs-index-html-20260909/mobile/17-search-flexure.png
    ~/.hermes/design/shots/html-docs-index-html-20260909/mobile/22-tune.png
walked: 2026-09-09 `python3 ~/.hermes/scripts/uiwalk.py --flow /tmp/mechlib-review-20260909.json` viewports 1440×900 and 390×844. Flow: load, wait `.model-card`, hero, `#latest-added`, scroll, In motion, search `lattice_flexure`, Tune. Console errors none; failed requests none. curl `/favicon.ico` still HTTP 404.

Looked at every PNG with vision analysis.

WebGL desktop hero: VERSION **v0.11.0**, PARTS **195 + 32 utils**, **195 OF 195 PARTS**. Install `pip3 install git+https://github.com/m-esm/mechlib` is fully visible. Shelf All / Mechanical movements / Machine elements / Building blocks. Category chips Linkages 14 through Worm & planetary 6, with **Pumps & valves** cut at the right edge (`Pumps & valves ↵`). LINKAGES 14 parts. First row FOUR_BAR, TOGGLE_CLAMP, SCOTCH_YOKE, QUICK_RETURN with live-params / bodies / mm / animated badges and pastel 3D meshes in the card viewports. No WebGL-fail banner. Toolbar now shows **In motion**, **Tunable only**, and **Latest added** as sibling chips.

WebGL desktop Latest added: chip outlined; count stays **195 OF 195 PARTS** (sort, not filter). Section **LATEST ADDED** 195 parts, subtitle “Newest gallery cards first.” First row **HONEYCOMB_CORE**, **LATTICE_FLEXURE**, **BOLT_MESH**, **CAM_PROFILE_2D** with meshes. No per-card `added` date on the face — recency is order only. After scroll: sticky toolbar holds; honeycomb_core / lattice_flexure signatures + USED IN copy + COPY IMPORT / GLB / SOURCE / LINK / TUNE; next row CHAMFER_CUTTER, COUNTERSINK, EXTRUDE_DOWN, EXTRUDE_POLY_Z meshes render. No empty viewports.

WebGL desktop In motion: chip outlined; count **37 OF 195 PARTS**. Category chips re-count (Grippers 1, Gears 6, Ratchets 0, Worm & planetary 0). four_bar / toggle_clamp / scotch_yoke / quick_return meshes still render.

WebGL desktop search `lattice_flexure`: **1 OF 195 PARTS**, FLEXURES 1 part, LATTICE_FLEXURE card with two-block mesh. Empty grid to the right is the filter, not a crash.

WebGL desktop Tune: modal title **FOUR_BAR**, CLOSE, signature `four_bar(crank_angle_deg=60, l_crank=12.5)`, sliders CRANK ANGLE DEG · MOTION 60 deg and L Crank 12.5 mm, PLAY / COPY CODE / DOWNLOAD STL / RESET DEFAULTS. Left pane shows the pastel four-bar mesh (not empty). Footer **regenerated in 82 ms**. No traceback.

WebGL mobile 390×844 first screen: Install clips at `github.com/m-esm/mecl`. Use wraps mid-token (`axis:`). Search placeholder clips at `descriptio` with a `press /` hint. Filter chips wrap. Category row clips `Grippers & cla`. No part mesh is on the first screen. VERSION/PARTS still **v0.11.0** / **195 + 32 utils**.

WebGL mobile Latest added: chip outlined; heading **LATEST ADDED** 195 parts and HONEYCOMB_CORE title sit at the bottom of the first viewport; mesh still below the fold. After scroll: honeycomb_core hex-core mesh + Tune + USED IN; sticky search/shelf/chips remain; category still clips `Grippers & cla`.

WebGL mobile search `lattice_flexure`: **1 OF 195 PARTS**, FLEXURES heading + LATTICE_FLEXURE title on the first screen; mesh still below the fold.

WebGL mobile Tune: FOUR_BAR / CLOSE, four-bar mesh in the preview, signature, CRANK ANGLE and L Crank sliders, then PLAY / COPY CODE / DOWNLOAD STL / RESET DEFAULTS stacked **below** the sliders (not over them). Footer **regenerated in 78 ms**. No traceback.

## verdict

Desktop catalog still earns its place: hero, live counts, shelf, cards with meshes and machinery “Used in” text. **Latest added** now exists and actually reorders — honeycomb_core and lattice_flexure lead, which they did not on the 2026-09-07 look. Search `lattice_flexure` returns the one card. Tune boots: **regenerated in 82 ms** with a mesh, which it did not on 2026-09-07 (`cubic_lattice` ImportError). Mobile is still a chrome stack that clips install and search and hides the first mesh; Tune on a phone is now usable.

Looked for and holds (WebGL path): live Pages serves the gallery; v0.11.0 and 195+32; All/Movements/Elements/Blocks shelf; Latest added chip + LATEST ADDED section; In motion reduces 195→37 and restyles the chip; four_bar-class and honeycomb_core / lattice_flexure meshes are recognizable; sticky toolbar survives scroll; card actions Copy import / GLB / Source / Link are visible on desktop; Tune modal opens and regenerates.

Not seen in these shots: per-card added dates, Utility API rows, STL that actually downloads, a category with zero animated parts after filter alone.

## debt

- [x] SEV=high `new THREE.WebGLRenderer` runs before `fetch("./models/index.json")`. A failed WebGL context leaves VERSION “loading”, PARTS “-”, an empty `#gallery`, and an empty Utility API, with no on-page error. Confirmed by uiwalk `--disable-gpu` (desktop+mobile 02-hero). Fixed: constructor is try/caught; catalog fetch still runs; on-page banner + card status when WebGL is missing.
- [x] SEV=high Tune playground runtime boot failed on live Pages: `ImportError: cannot import name 'cubic_lattice' from 'mechlib.lattices'` (Did you mean: `bcc_lattice`?). Cause: `docs/playground/demos.py` imported `cubic_lattice` while `docs/wheels/mechlib-0.11.0-py3-none-any.whl` was still the 2026-09-02 bcc-only build. Rebuilt the wheel from current `mechlib/` so Tune exec of demos.py can import. Re-look 2026-09-09: desktop **regenerated in 82 ms** with mesh; mobile **78 ms** with mesh.
- [x] SEV=med At 390×844 the Tune modal stacks PLAY / COPY CODE / DOWNLOAD STL over the CRANK ANGLE slider, so even a successful boot would fight the chrome. Re-look 2026-09-09 mobile/22-tune.png: actions sit below the sliders; mesh and PLAY are both usable.
- [ ] SEV=med At 390×844 the install line clips at `github.com/m-esm/mecl` with no wrap or overflow cue; Copy is the only way to recover the URL.
- [ ] SEV=med At 390×844 the search placeholder clips (`descriptio`) and `press /` occupies the field on a phone that has no slash shortcut.
- [ ] SEV=med The category chip row clips with no overflow hint: desktop cuts `Pumps & valves`, mobile cuts `Grippers & cla`.
- [ ] SEV=med At 390×844 the first viewport is only hero + toolbar; no part mesh is visible, so “every part below” is below the fold after a tall chrome stack. Latest added puts HONEYCOMB_CORE’s title on the first screen but the mesh still needs a scroll.
- [ ] SEV=low Latest-added reorders (honeycomb_core, lattice_flexure first) but cards show no `added` date, so “newest” is only implied by position.
- [ ] SEV=low `/favicon.ico` returns 404.
