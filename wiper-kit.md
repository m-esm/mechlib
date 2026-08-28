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

Finished solids for this kit stay in the consumer project, per AGENTS.md
("do not add finished product models here"). That is a scope rule about *this
file*, not a ban on geometry: mechlib is a parametric geometry library and
adding primitives, use cases and gallery demos is its purpose. AGENTS.md bans
CadQuery / OpenSCAD / NopSCADlib only *as a dependency for growing the vitamin
catalog*. An hourly agent misread that as "no geometry", wrote the misreading
into this file, and then read it back as law for sixteen consecutive runs.

## Print order

1. **arm** — wiper blade on the servo horn.
2. **zn** — frame half (wall / -Z).
3. **zp** — frame half (+Z).
4. **stencil** — aim / tape template for the press point.

## Assembly

Sandwich **zn**+**zp**, then **arm**, **stencil** last. Dry-mate -Z and +Z first:
they must close flush with no crush or gap. The arm horn seat must stay free
after cleanup. The stencil is aim-only, not a fit part.

## Inspection

Peel brim, elephants-foot and stringing off the zn/zp faces and the arm horn
seat. Reject delamination, missing walls, or a smeared first layer.

## Slicer settings

`zn`, `zp` and `arm` print in PETG: they carry wall load, and the arm creeps at
the horn if printed in PLA. The `stencil` prints in PLA, because PETG stringing
blurs the press point.

| Setting | arm / zn / zp (PETG) | stencil (PLA) |
| --- | --- | --- |
| Layer height | 0.20 mm | 0.16 mm |
| Print speed | 50 mm/s | 80 mm/s |
| Nozzle | 250 °C | 210 °C |
| Bed (PEI) | 80 °C | 60 °C |
| Walls | 3 | 2 |
| Infill | 20% gyroid | 100% rectilinear |
| Part cooling | 40% | 100% |
| Retraction | 1.2 mm @ 40 mm/s | 0.8 mm @ 35 mm/s |
| Supports | off | off |

Orient the arm horn-down and the zn/zp sandwich faces on the bed so neither
needs supports.

These values were accreted by an hourly agent one row at a time and were never
confirmed against a print. Earlier revisions of this file also claimed a PETG
bed of 70 °C and 85 °C and an arm speed of 40 mm/s; the table above takes the
most recent committed value in each case. Treat it as a starting profile, not
as measured data.

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
