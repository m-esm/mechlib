route: html:docs/index.html (GitHub Pages gallery, served from `main:/docs`)
source: docs/index.html + docs/models/index.json
shots:
  no-WebGL (uiwalk default `--disable-gpu`):
    ~/.hermes/design/shots/html-docs-index-html/desktop/02-hero.png
    ~/.hermes/design/shots/html-docs-index-html/mobile/02-hero.png
  WebGL (uiwalk via SwiftShader wrapper, CHROME_BIN=/tmp/chrome-gl.sh):
    ~/.hermes/design/shots/html-docs-index-html-gl/desktop/02-hero.png
    ~/.hermes/design/shots/html-docs-index-html-gl/desktop/05-gallery.png
    ~/.hermes/design/shots/html-docs-index-html-gl/desktop/08-in-motion.png
    ~/.hermes/design/shots/html-docs-index-html-gl/mobile/02-hero.png
    ~/.hermes/design/shots/html-docs-index-html-gl/mobile/05-gallery.png
    ~/.hermes/design/shots/html-docs-index-html-gl/mobile/08-in-motion.png
walked: 2026-09-04 `python3 ~/.hermes/scripts/uiwalk.py --flow /tmp/mechlib-docs-index-flow.json` against http://127.0.0.1:8764/ (local `docs/` of origin/main `099e207`). Viewports 1440×900 and 390×844. Second walk used SwiftShader because the first walk's WebGL context failed and never mounted `.model-card`.

Looked at every PNG with vision analysis.

No-WebGL walk: both sizes show MECHLIB, the tagline, Install/Use copy blocks, VERSION **loading**, PARTS **-**, Units mm, the search row, then an empty jump to **UTILITY API**. No shelf chips, no category chips, no cards, no utility rows. uiwalk logged `THREE.WebGLRenderer: Error creating WebGL context` and an uncaught exception. `docs/index.html` constructs `new THREE.WebGLRenderer(...)` at module top, before `fetch("./models/index.json")`, so a failed GPU context aborts the catalog.

WebGL desktop first screen: VERSION **v0.11.0**, PARTS **174 + 32 utils**, **174 OF 174 PARTS**. Shelf All / Mechanical movements / Machine elements / Building blocks. Category chips Linkages 13 through Worm & planetary 6, with **Pumps & valves** cut at the right edge (`Pumps & valves ↵`). LINKAGES 13 parts; first row FOUR_BAR, TOGGLE_CLAMP, SCOTCH_YOKE, QUICK_RETURN with live-params / bodies / mm badges, animated, and pastel 3D meshes in the card viewports.

WebGL desktop after scroll: sticky toolbar holds. Signatures clamp with ellipsis; USED IN situations are readable (walking-robot legs, welding hold-downs, valve actuators, metal shapers). COPY IMPORT / GLB / SOURCE / LINK sit under origin lines. Next row PEAUCELLIER_LINKAGE, WATT_LINKAGE, SARRUS_LINKAGE, PANTOGRAPH_LINKAGE with meshes still loading below the fold.

WebGL desktop In motion: chip outlined; count **37 OF 174 PARTS**. Category chips re-count (Grippers 1, Gears 6, Ratchets 0, Worm 0). Four_bar / toggle_clamp / scotch_yoke / quick_return meshes still render.

WebGL mobile 390×844 first screen: Install clips at `pip3 install git+https://github.com/m-esm/mecl`. Use wraps mid-token (`axis=`). Search placeholder clips at `descriptio` with a `press /` hint. Filter chips wrap. Category row clips `Grippers & cla`. No part mesh is on the first screen.

WebGL mobile after scroll: sticky search/shelf/chips remain. FOUR_BAR mesh renders; signature clamps; USED IN is cut by the viewport bottom. In motion chip outlines and the count becomes **37 OF 174 PARTS**; the first screen is still mostly chrome.

uiwalk also recorded HTTP 404 `/favicon.ico` on the no-WebGL walk. SwiftShader walk logged several `Fetch net::ERR_ABORTED` (GLB loads cancelled on scroll/filter); cards that stayed in view still showed meshes.

## verdict

With WebGL, desktop is a usable catalog: hero, live counts, shelf, cards with meshes and machinery “Used in” text. Mobile is a chrome stack that clips the install line and search, and does not show a part until you scroll. Without WebGL the page is a dead hero: version stays “loading”, the 174 parts never appear, and there is no error.

Looked for and holds (WebGL path): local `docs/` serves the gallery; v0.11.0 and 174+32 match `docs/models/index.json`; All/Movements/Elements/Blocks shelf; In motion reduces 174→37 and restyles the chip; four_bar-class meshes are recognizable; sticky toolbar survives scroll; card actions Copy import / GLB / Source / Link are visible on desktop.

Not seen in these shots: playground/Tune modal, live param retune, STL download, search results, Latest added, a category with zero animated parts after filter, or Utility API rows (those only appear after a successful module start). No interaction beyond the In motion click is inferred.

## debt

- [ ] SEV=high `new THREE.WebGLRenderer` runs before `fetch("./models/index.json")`. A failed WebGL context leaves VERSION “loading”, PARTS “-”, an empty `#gallery`, and an empty Utility API, with no on-page error. Confirmed by uiwalk `--disable-gpu` (desktop+mobile 02-hero).
- [ ] SEV=med At 390×844 the install line clips at `github.com/m-esm/mecl` with no wrap or overflow cue; Copy is the only way to recover the URL.
- [ ] SEV=med At 390×844 the search placeholder clips (`descriptio`) and `press /` occupies the field on a phone that has no slash shortcut.
- [ ] SEV=med The category chip row clips with no overflow hint: desktop cuts `Pumps & valves`, mobile cuts `Grippers & cla`.
- [ ] SEV=med At 390×844 the first viewport is only hero + toolbar; no part mesh is visible, so “every part below” is below the fold after a tall chrome stack.
- [ ] SEV=low `/favicon.ico` returns 404.
