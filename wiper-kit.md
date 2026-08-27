# wiper_kit

Use case in `mechlib/usecases.py` (`use_case("wiper_kit")`). Not a mechlib
mesh API — no printed geometry lives here, and none should be added.

## Situation

Wall-button single-pivot wiper kits: printed **arm**, **zn** and **zp** frame
halves, and an aim **stencil**. The bought servo is rebound from
`vitamin()` instead of a forked envelope table.

## Printed kit (consumer project, not this repo)

| Body | Role |
| --- | --- |
| arm | printed wiper blade on the servo horn |
| zn | frame half (wall / −Z) |
| zp | frame half (+Z) |
| stencil | aim / tape template for the press point |

Finished solids stay in the product. Do not add CadQuery, OpenSCAD, or
NopSCADlib, and do not grow a second dim table here.

## Print order
Print **arm**, then **zn**, then **zp**, then **stencil**. Consumer project; not this repo.
1. **arm** — wiper blade on the servo horn.
2. **zn** — frame half (wall / −Z).
3. **zp** — frame half (+Z).
4. **stencil** — aim / tape template for the press point.
No printed geometry lives here, and none should be added.
Do not add CadQuery, OpenSCAD, or NopSCADlib.

## Post-print assembly
Sandwich **zn**+**zp**, then **arm**, **stencil** last. Consumer project; not this repo.
1. **zn** — frame half (wall / −Z), first face of the sandwich.
2. **zp** — frame half (+Z), close the sandwich on zn.
3. **arm** — wiper blade on the servo horn, after the frame sandwich.
4. **stencil** — aim / tape template last (press-point layout).
No printed geometry lives here, and none should be added.
Do not add CadQuery, OpenSCAD, or NopSCADlib.

## Post-print inspection
Inspect **flash**, **layer**, and **zn**/**zp** fit before the sandwich. Consumer project; not this repo.
1. **flash** — peel brim, elephants-foot, and stringing off zn/zp faces and the arm horn seat.
2. **layer** — reject delam, missing walls, or a smeared first layer that closes the sandwich.
3. **zn/zp fit** — dry-mate −Z and +Z; they must close flush with no crush or gap.
Stencil is aim-only; it is not a fit part. Arm horn seat must stay free after cleanup.
No printed geometry lives here, and none should be added.
Do not add CadQuery, OpenSCAD, or NopSCADlib.

## `vitamin()` slugs

```python
from mechlib import vitamin

sg90 = vitamin("servo/sg90")    # SG90 9g micro, datasheet envelope
mg90s = vitamin("servo/mg90s")  # MG90S metal-gear 9g, calipered
```

Addresses are `family/slug`. Envelopes are display meshes, not printed STLs.
Consumers rebind product params from the address. Port facts into
`mechlib/vitamins/tables.py`.

| Address | Title | Source |
| --- | --- | --- |
| `servo/sg90` | SG90 9g micro servo (published envelope) | datasheet |
| `servo/mg90s` | MG90S metal-gear 9g servo (calipered) | caliper |

## Lookup

```python
from mechlib.usecases import use_case, search_use_cases

use_case("wiper_kit")
search_use_cases("wiper")
```
