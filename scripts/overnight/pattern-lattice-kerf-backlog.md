# mechlib pattern / lattice / kerf backlog

Hourly Pawl cron (`mechlib-pattern-lattice-hourly` `444870752857`) ships one pending row per tick.
nbg1 owns research (add/replace/veto). Pawl implements only; do not invent slugs.
Semi-primitives. No CadQuery. FDM: nozzle-multiple struts, wall>=0.8, kerf>=nozzle.

## Already in library (do not re-ship)

- patterns: polar_ring, lighten_cell_poly (rect|hex), lighten_grid_centres, directed_holes
- auxetic_panel: reentrant, rotating_squares, arrowhead, star, chiral, anti_tetrachiral, houndstooth (hexachiral is a dup, vetoed)
- kerf_bend_cutter: lattice, diagonal, spiral, wave, hex, cross, chevron, diamond, fishbone
- honeycomb_panel (47e3460)
- isogrid_panel (3df5609)

## Queue

| id | api | kind | status | note |
| --- | --- | --- | --- | --- |
| 01 | honeycomb_panel | 2d-lattice | shipped | SHA 47e3460 |
| 02 | isogrid_panel | 2d-lattice | shipped | NASA triangular rib sheet. SHA 3df5609 |
| 03 | kerf_bend_cutter(mode="hex") | kerf | shipped | SHA 7d0cb2a hex living-hinge edge slits |
| 04 | kerf_bend_cutter(mode="cross") | kerf | shipped | X-lattice bars + ~30° arms. THIS HOUR. |
| 05 | kerf_bend_cutter(mode="chevron") | kerf | shipped | nested 45° arrowheads. SHA 2b34903 |
| 06 | kerf_bend_cutter(mode="diamond") | kerf | shipped | elongated diamond-outline brick-wall slits. |
| 07 | kerf_bend_cutter(mode="fishbone") | kerf | shipped | herringbone 45/135° rib pairs. SHA 6bd5243 |
| 08 | kerf_bend_cutter(mode="meander") | kerf | shipped | MDPI meander |
| 09 | kerf_bend_cutter(mode="biaxial") | kerf | shipped | 2-axis wrap. SHA dca0357dbb6a0658a43507a9f8b5a896521cdbe3 |
| 10 | auxetic_panel(mode="arrowhead") | auxetic | shipped | Grima double-arrowhead NPR cells. THIS HOUR. |
| 11 | auxetic_panel(mode="star") | auxetic | shipped | Grima star-shaped honeycomb NPR cells. SHA 9bad3d1 |
| 12 | auxetic_panel(mode="anti_tetrachiral") | auxetic | shipped | Opposite-sense square-grid NPR cells. THIS HOUR. |
| 13 | auxetic_panel(mode="houndstooth") | auxetic | shipped | interlocking L / broken-chevron NPR cells. THIS HOUR. |
| 14 | kagome_lattice | 2d-lattice | pending | |
| 15 | bcc_lattice | 3d-strut | pending | |
| 16 | octet_truss | 3d-strut | pending | |
| 17 | kelvin_cell | 3d-strut | pending | |
| 18 | cubic_lattice | 3d-strut | pending | |
| 19 | gyroid_lattice | tpms | pending | keep; wall>=1.2 cell>=8 |
| 20 | honeycomb_core | 2.5d | pending | != honeycomb_panel sheet |
| 21 | lattice_flexure(kind="x") | flexure | pending | |
| 22 | lattice_flexure(kind="v") | flexure | pending | |

## Vetoed (do not ship)

hexachiral (dup of chiral), Schwarz P, Schwarz D, bezier, fabric, circle, snake, Voronoi, Miura, Yoshimura, rotating_triangles, living_hinge_panel (= mode=lattice already).

Sources: LivingHingeGenerator, MDPI meander, FFF review, NASA isogrid PA+CF, BCC FFF truss.
