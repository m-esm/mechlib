# mechlib pattern / lattice / kerf backlog

Hourly Pawl cron (`mechlib-pattern-lattice-hourly` `444870752857`) ships one pending row per tick.
nbg1 owns research (add/replace/veto). Pawl implements only; do not invent slugs.
Semi-primitives. No CadQuery. FDM: nozzle-multiple struts, wall>=0.8, kerf>=nozzle.

**Marking a row shipped:** the `api` column must name a function on
`mechlib.__all__`, and the note must carry `SHA <hex>` of the commit that added
it (`git log --oneline -S'<mode-string>' -- mechlib/<module>.py`). Never write
`THIS HOUR`. `python3 scripts/backlog_provenance.py` prints the number of rows
that fail this and CI fails on anything above 0.

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
| 04 | kerf_bend_cutter(mode="cross") | kerf | shipped | X-lattice bars + ~30° arms. SHA 3694df1 |
| 05 | kerf_bend_cutter(mode="chevron") | kerf | shipped | nested 45° arrowheads. SHA 2b34903 |
| 06 | kerf_bend_cutter(mode="diamond") | kerf | shipped | elongated diamond-outline brick-wall slits. SHA 53204ba |
| 07 | kerf_bend_cutter(mode="fishbone") | kerf | shipped | herringbone 45/135° rib pairs. SHA 6bd5243 |
| 08 | kerf_bend_cutter(mode="meander") | kerf | shipped | MDPI meander square-wave labyrinth. SHA d1882e5 |
| 09 | kerf_bend_cutter(mode="biaxial") | kerf | shipped | 2-axis wrap. SHA dca0357dbb6a0658a43507a9f8b5a896521cdbe3 |
| 10 | auxetic_panel(mode="arrowhead") | auxetic | shipped | Grima double-arrowhead NPR cells. SHA 16184d5 |
| 11 | auxetic_panel(mode="star") | auxetic | shipped | Grima star-shaped honeycomb NPR cells. SHA 9bad3d1 |
| 12 | auxetic_panel(mode="anti_tetrachiral") | auxetic | shipped | Opposite-sense square-grid NPR cells. SHA e9833c9 |
| 13 | auxetic_panel(mode="houndstooth") | auxetic | shipped | interlocking L / broken-chevron NPR cells. SHA d122a6bdc5c1335650aac7979a2cd1ac63c0dc4d |
| 14 | kagome_panel | 2d-lattice | shipped | Trihexagonal Kagome tri+hex lightening sheet (researched as kagome_lattice). SHA 434302b |
| 15 | bcc_lattice | 3d-strut | shipped | Body-centred-cubic strut truss, 8 half-diagonals/cell to shared centre node. SHA 479856a986f90ebcaa8c55c8fa80f5d4db1a9070 |
| 16 | octet_truss | 3d-strut | shipped | FCC face-diagonal tetrahedral/octahedral strut network. SHA 01fdddb138d5a2df2ba228e2f1390686ac978e83 |
| 17 | kelvin_cell | 3d-strut | shipped | Truncated-octahedron 24-node, 36-strut open-cell frame. SHA 55251cbc23acfe612cbc0b3e6e6d71e224e04566 |
| 18 | cubic_lattice | 3d-strut | shipped | Simple-cubic X/Y/Z edge truss with shared grid nodes. SHA e8f12a895686ebb2f139bd0d2d91491ae6a77df2 |
| 19 | gyroid_lattice | tpms | shipped | TPMS gyroid sheet; wall>=1.2 cell>=8. SHA cf1b37fdb8caddf5bf3eff30d32ed34e6d4e95e5 |
| 20 | honeycomb_core | 2.5d | shipped | SHA b08ccb9 tall open-cell hex-tube sandwich core, single-wall rim + optional bond skins; != honeycomb_panel sheet |
| 21 | lattice_flexure(kind="x") | flexure | shipped | Howell distributed-compliance crossed-truss pivot. SHA 5563753 |
| 22 | lattice_flexure(kind="v") | flexure | shipped | Howell V-fold accordion distributed-compliance sheets. SHA fe201be82a9372a7d4204b510aa3fafaf1c30287 |

## Vetoed (do not ship)

hexachiral (dup of chiral), Schwarz P, Schwarz D, bezier, fabric, circle, snake, Voronoi, Miura, Yoshimura, rotating_triangles, living_hinge_panel (= mode=lattice already).

Sources: LivingHingeGenerator, MDPI meander, FFF review, NASA isogrid PA+CF, BCC FFF truss.
