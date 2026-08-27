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

## Filament (PETG vs PLA)
Prefer **PETG** for **arm**/**zn**/**zp**; **PLA** is fine for **stencil**. Consumer project; not this repo.
1. **arm** — PETG: heat and creep at the horn; PLA can sag on a warm wall.
2. **zn** — PETG: layer weld on the wall half; PLA is brittle at the sandwich.
3. **zp** — PETG: same as zn so the sandwich matches shrinkage.
4. **stencil** — PLA: aim/tape template only; PETG stringing blurs the press point.
No printed geometry lives here, and none should be added.
Do not add CadQuery, OpenSCAD, or NopSCADlib.

## First layer / bed
Print on **PEI**; bed **PETG ~70C**, **PLA ~60C**. Consumer project; not this repo.
1. **PEI**: wipe the sheet, no glue; first layer must stick on zn/zp faces.
2. **PETG ~70C**: arm/zn/zp bed; too cold and the sandwich warps off PEI.
3. **PLA ~60C**: stencil bed; hotter PEI elephants-foot the aim cut.
4. **first layer**: reject a smeared or unstuck skirt before committing the kit.
No printed geometry lives here, and none should be added.
Do not add CadQuery, OpenSCAD, or NopSCADlib.

## Retraction / stringing
Keep **PETG** retraction **short**; **PLA stencil** can retract more. Consumer project; not this repo.
1. **arm** — PETG: short retract so the horn seat does not jam; wipe stringing off the blade.
2. **zn** — PETG: same short retract; strings on the sandwich face fail the dry-mate.
3. **zp** — PETG: match zn so both halves string the same; peel before sandwich.
4. **stencil** — PLA: more retract is fine; leftover strings blur the press-point cut.
No printed geometry lives here, and none should be added.
Do not add CadQuery, OpenSCAD, or NopSCADlib.

## Cooling / fan
**PETG** low fan on **arm**; **PLA stencil** fan **100%**. Consumer project; not this repo.
1. **arm**: PETG low fan so layers weld at the horn; high fan makes the blade brittle.
2. **zn**: PETG low fan; a cold sandwich face delams under wall load.
3. **zp**: match zn so both halves shrink the same.
4. **stencil**: PLA 100% fan; low fan blurs the press-point cut.
No printed geometry lives here, and none should be added.
Do not add CadQuery, OpenSCAD, or NopSCADlib.

## Bed / nozzle
**PETG arm** bed **85** / nozzle **250**; **PLA stencil** **60**/**210**. Consumer project; not this repo.
1. **arm** — PETG: bed 85, nozzle 250; cooler PETG fails the horn weld.
2. **zn** — PETG: same 85/250 so the wall half matches the arm shrinkage.
3. **zp** — PETG: match zn; a colder half gaps the sandwich.
4. **stencil** — PLA: bed 60, nozzle 210; hotter PLA blurs the press-point cut.
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
