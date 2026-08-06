# mechlib

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

`mechlib` is a standalone collection of pure parametric geometry primitives for FDM-printed mechanical design using trimesh, Shapely, and manifold3d. The primitives were mined from finnish-doors (the Klonk door CAD project), finnish-windows, and parviz, which contained three or more independent reimplementations of the same geometry helpers.

## Scope

mechlib holds semi-primitive parts only: reusable, project-agnostic building blocks (primitives, sweeps, cutters, gear generators, fasteners, closures, patterns, text, mesh utilities, packing, STEP export). Finished designed parts, product models, and assemblies never live here; they belong to consumer projects. The gallery demos API usage, one minimal entry per function, and is not a parts showcase.

## Install

```bash
# from a consumer project on this machine (editable, tracks the checkout)
pip3 install -e ~/Desktop/myprojects/mechlib

# or straight from GitHub
pip3 install git+https://github.com/m-esm/mechlib
```

```python
import mechlib as ml

body = ml.rbox((40, 24, 12), r=3)                              # rounded block
body = ml.sub(body, ml.teardrop(4.1, 50, axis="x", up=(0, 0, 1)))  # print-safe bore
ml.export_stl(body, "bracket.stl")
```

## API

### `mechlib.prim`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `cyl` | Create and orient a cylinder mesh. | finnish-doors `src/shared/geom_util.py` |
| `boxc` | Create a box centered at a point. | finnish-doors `src/shared/geom_util.py` |
| `rbox` | Extrude a box with rounded vertical corners. | finnish-doors `src/shared/geom_util.py` |
| `frustum` | Create a truncated cone mesh. | finnish-doors `src/shared/geom_util.py` |
| `sector2d` | Create a circular-sector polygon. | finnish-doors `src/shared/geom_util.py` |
| `rot2` | Rotate two-dimensional points by degrees. | finnish-doors `src/shared/geom_util.py` |
| `ideg` | Compute the involute-angle function in degrees. | finnish-doors `src/shared/geom_util.py` |
| `mesh_from_tris` | Build and repair a mesh from triangle tuples. | finnish-doors `src/shared/geom_util.py` |
| `largest` | Select the largest connected mesh component. | finnish-doors `src/shared/geom_util.py` |
| `extrude_down` | Extrude Polygon or MultiPolygon downward. | finnish-doors `src/shared/geom_util.py` |
| `hex_poly` | Create a regular hexagon from across-flats width. | finnish-doors `src/projects/klonk/gears2d.py` |
| `chamfer_prism` | Build a rounded prism with a hull-chamfered top. | dual-axis-turntable `src/build.py` |
| `seg_cylinder` | Build a cylinder between arbitrary 3D points. | massage-shower-head `build.py` |

### `mechlib.sweep` and `mechlib.fixtures`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `extrude_twist` | Extrude a profile through an explicit angular sweep. | finnish-doors `src/shared/geom_util.py` |
| `swept_keyed_bore` | Sweep a keyed bore polygon through free rotation. | finnish-doors `src/projects/klonk/shaft.py` |
| `ring_pts` | Resample a polygon boundary into a 3D point ring. | dual-axis-turntable `src/build.py` |
| `loft` | Build a capped solid through equal-count point rings. | dual-axis-turntable `src/build.py` |
| `board_cradle` | Build four corner standoffs and capture walls for a PCB. | finnish-doors `src/projects/klonk/system_layout.py` |
| `saddle` | Build a shell-trimmed cradle rib for a cylindrical part. | mini-powerbank `pickle_build.py` |
| `kinematic_coupling` | Build an exactly constrained Maxwell or Kelvin ball-and-groove mount. | New, gap-analysis wave v0.8.0 |
| `repeatable_dock` | Build a kinematic coupling with magnet or screw preload and a bolt circle. | New, gap-analysis wave v0.8.0 |
| `three_point_leveller` | Build a three-screw kinematic levelling stage for tip, tilt, and height. | New, gap-analysis wave v0.8.0 |

### `mechlib.meshutil`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `to_manifold` | Convert a trimesh mesh to manifold3d. | parviz `src/geo.py` |
| `from_manifold` | Convert manifold3d output to trimesh. | parviz `src/geo.py` |
| `sub` | Subtract one watertight mesh from another. | parviz `src/geo.py` |
| `uni` | Union a sequence of watertight meshes. | parviz `src/geo.py` |
| `inter` | Intersect two watertight meshes. | parviz `src/geo.py` |
| `export_stl` | Export STL through repair and geometry guards. | parviz `src/geo.py` |
| `inflate` | Offset vertices outward along vertex normals. | finnish-windows `src/build_combined.py` |
| `bbox_overlap` | Test axis-aligned bounding-box overlap. | parviz `src/assembly_check.py` |
| `overlap_volume` | Measure exact manifold intersection volume. | parviz `src/assembly_check.py` |
| `inside` | Check that all probes lie inside a solid. | parviz `src/checks.py` |
| `clear` | Check that no probes lie inside a solid. | parviz `src/checks.py` |
| `bore_pierces` | Probe bore clearance along its real axis. | parviz `src/checks.py` |
| `void_cube` | Confirm a local void by cube intersection. | parviz `src/checks.py` |
| `solid_cube` | Confirm local material by cube intersection. | parviz `src/checks.py` |
| `self_thickness` | Sample ray-based material thickness. | parviz `src/wallcheck.py` |
| `cube_rotations` | Return all 24 proper cube rotations. | parviz `src/refparts.py` |
| `fit_transform` | Rigidly fit a real mesh to a placeholder. | parviz `src/refparts.py` |
| `decimate` | Cluster mesh vertices on a regular grid. | parviz `src/refparts.py` |
| `orient` | Rotate positive Z onto a normal. | parviz `src/geo.py` |
| `extrude_poly_z` | Extrude Polygon or MultiPolygon between Z planes. | finnish-doors `src/projects/klonk/housings.py` |
| `largest_poly` | Select the largest polygon in a geometry. | finnish-doors `src/projects/klonk/housings.py` |
| `min_distance` | Approximate surface distance from sampled points. | torque-lever `assembly_check.py` |
| `audit` | Audit pairwise overlap, clearance, aliases, and allowlists. | torque-lever `assembly_check.py` |
| `approach_clear` | Measure free travel toward an opening. | jumper-wire-sockets `src/checks.py` |
| `slicer_area` | Predict per-layer perimeter and infill extrusion area. | tripod `lighten_legs.py` |
| `extrude_snapped` | Snap tangent vertices before polygon extrusion. | torque-lever `build.py` |

### `mechlib.cutters`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `teardrop` | Build a print-aware bore cutter with an arbitrary up vector. | parviz `src/head.py` |
| `ss_bore` | Build a support-light clamshell bore. | finnish-doors `src/shared/geom_util.py` |
| `dbore` | Build a unified explicit-dimension double-D bore. | parviz and finnish-windows |
| `dbore_hub` | Build a cylindrical hub with a double-D socket. | parviz `src/geo.py` |
| `chamfer_cutter` | Build a 45 degree end-chamfer cutter. | parviz `src/standins/foot_pin.py` |
| `hex_corner_chamfer` | Chamfer hex corners while preserving flats. | parviz `src/standins/m8_nut.py` |
| `countersink` | Cut a bore lead-in down to a thread crest. | parviz `src/standins/m8_nut.py` |
| `slot_neg` | Build an adjustable pedestal obround negative. | parviz `src/chassis.py` |
| `blind_socket` | Build a blind locating-pin socket. | parviz `src/geo.py` |
| `gable_roof` | Build a two-slope support-light opening roof. | finnish-doors `src/projects/klonk/housings.py` |
| `counterbore` | Build a through-hole plus cylindrical head pocket. | New, inspired by finnish-doors fastener seats |
| `bearing_seat` | Build 608, 695, or MR105 pocket cutters. | New, inspired by finnish-doors 608 seats |
| `crush_ribs` | Build tapered ribs on opposing pocket walls. | New, generalized from finnish-doors `motors.py` |
| `slot_cutter` | Build an FDM rectangular slot with dog-bone and foot relief. | torque-lever `build.py` |
| `lobe_cavity_polys` | Build hollow lobe cores around optional ribs. | tripod `lighten_legs.py` |
| `tapered_cavity` | Build a stepped, self-supporting hollow cutter. | tripod `lighten_legs.py` |
| `u_channel_between` | Build an open rounded U channel at any XY angle. | jumper-wire-sockets `src/build.py` |
| `revolved_gable_cavity` | Build an annular cavity with a self-supporting roof. | massage-shower-head `build.py` |
| `AS568_CS_MM` | The AS568/ISO 3601 O-ring cross-section sizes in millimetres. | New, gap-analysis wave v0.8.0 |
| `oring_groove` | Build an AS568/ISO 3601 O-ring gland cutter (face or bore mode), squeeze and fill validated. | New, gap-analysis wave v0.8.0 |
| `labyrinth_seal` | Build an interleaved-comb non-contact rotary seal (rotor plus stator). | New, gap-analysis wave v0.8.0 |
| `gasket_channel` | Build a cord-stock gasket groove cutter following an arbitrary closed path. | New, gap-analysis wave v0.8.0 |

### `mechlib.closures`

| Function or type | Purpose | Origin project |
| --- | --- | --- |
| `press_lid` | Build a plate and hollow friction plug. | finnish-doors `src/projects/klonk/system_layout.py` |
| `clamshell_shiplap` | Build a perimeter lip and matching lid slot. | finnish-doors `src/projects/klonk/housings.py` |
| `ydovetail` | Build a self-supporting Y-axis dovetail prism. | finnish-doors `src/projects/klonk/system_layout.py` |
| `SnapSpec` | Hold unified snap-catch dimensions. | finnish-doors `system_layout.py` and `build_powerbank.py` |
| `snap_catch` | Build a ramped outer-wall catch. | finnish-doors `system_layout.py` and `build_powerbank.py` |
| `snap_finger` | Build a cantilever finger and hook. | finnish-doors `system_layout.py` and `build_powerbank.py` |
| `nut_ac` | Convert nut across-flats to across-corners. | parviz `src/geo.py` |
| `nut_slot` | Build a seated captive-nut trap negative. | parviz `src/geo.py` |
| `screw_post` | Build a cylindrical boss along a normal. | parviz `src/geo.py` |
| `fix_pin` | Build a locating pin with a buried root. | parviz `src/geo.py` |
| `setscrew` | Build a locking boss and inward set-screw pilot. | wall-shelf-clamp `lib.py` |
| `push_pin` | Build a barbed printed press pin with lead-in. | dual-axis-turntable `src/build.py` |

### `mechlib.gears`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `spur_gear_2d` | Generate an involute spur or sector gear polygon. | finnish-doors `src/projects/klonk/gears2d.py` |
| `mesh_phase` | Compute driven-gear tooth phase. | finnish-doors `src/projects/klonk/gears2d.py` |
| `spur_gear_mesh` | Extrude a spur gear and cut a round bore. | New, unifying finnish-windows and parviz |
| `roller_sprocket_2d` | Generate a conjugate pin-envelope sprocket. | New, generalized from parviz `src/tracks.py` |
| `rack_2d` | Generate a finite pressure-angle rack matching `spur_gear_2d`. | finnish-doors `src/intercom/fixture.py` |
| `spur_gear` | Build a full 3D helical, sector, or hubbed gear. | dual-axis-turntable `src/gears.py` |
| `worm` | Build a true helical worm and report its lead angle. | dual-axis-turntable `src/gears.py` |
| `herringbone_gear` | Build a double-helical (herringbone) involute gear. | New, mechanical-movements wave v0.6.0 |
| `cycloidal_drive` | Build a single-stage cycloidal reducer stack. | New, mechanical-movements wave v0.6.0 |
| `bevel_gear_pair` | Build a straight bevel pair on 90-degree axes (Tredgold approximation). | New, mechanical-movements wave v0.6.0 |
| `internal_gear_2d` | Generate a true internal (annular) involute ring-gear polygon. | New, gap-analysis wave v0.8.0 |
| `internal_mesh_phase` | Compute pinion tooth phase for an INTERNAL mesh (not `mesh_phase`). | New, gap-analysis wave v0.8.0 |
| `ring_gear` | Extrude an internal ring gear with an outer rim. | New, gap-analysis wave v0.8.0 |
| `ring_gear_mesh` | Build a pinion posed in mesh inside a ring gear. | New, gap-analysis wave v0.8.0 |
| `trochoid_profile_2d` | Generate the shortened-epitrochoid inner equidistant shared by cycloidal discs and gerotor rotors. | New, gap-analysis wave v0.8.0 |

### `mechlib.drives`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `printed_worm` | Build a journalled printed worm with a keyed bore and runout threads. | finnish-doors Klonk `worm.py` |
| `flat_worm` | Build the bench-proven three-start flat-drive input worm. | finnish-doors Klonk `worm.py` |
| `worm_wheel_band` | Build the lead-angle-matched helical wheel band for `flat_worm`. | finnish-doors Klonk `worm.py` |
| `worm_coupon` | Build the inexpensive worm and wheel-band pair used to bench-test mesh quality. | finnish-doors Klonk `worm.py` |
| `planet_stage` | Build an assembled top-loading fixed-ring planetary stage with a hex-output carrier. | finnish-doors Klonk `parts_drive.py` |

### `mechlib.mechanisms`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `coarse_pitch` | Look up a printable coarse metric pitch. | parviz `src/threads.py` |
| `helix_solid` | Sweep and cap a watertight helical profile. | parviz `src/threads.py` |
| `thread_solid` | Build a printable external thread or internal cutter. | parviz `src/threads.py` |
| `tap` | Cut an internal thread into a solid. | parviz `src/threads.py` |
| `knurl` | Cut vertical grip flutes around a cylinder. | parviz `src/standins/m4_bolt.py` |
| `torsion_spring_mesh` | Build a torsion-spring assembly preview. | finnish-doors `src/projects/klonk/shaft.py` |
| `helix_tube` | Sweep a capped solid tube along a helix. | finnish-doors `wrap_demo.py` |
| `dog_slot_coupling` | Build a slotted boss and dog collar for angular lost motion. | finnish-doors `coupling_variants/build_coupling.py` |
| `threaded_rod` | Build a fast radial-grid external thread. | wall-shelf-clamp `lib.py` |

### `mechlib.ratchets`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `ratchet_ring_2d` | Generate the shared phased internal undercut ring profile. | finnish-doors Klonk `gears2d.py` |
| `ratchet_ring` | Extrude the shared internal ratchet ring. | finnish-doors Klonk `gears2d.py` |
| `pip_ratchet_hub_2d` | Generate a monolithic hub with captive rigid pawls and accordion springs. | finnish-doors Klonk `gears2d.py` |
| `pip_ratchet_hub` | Extrude and bore the print-in-place accordion ratchet hub. | finnish-doors Klonk `parts_drive.py` |
| `spring_cartridge_ratchet_2d` | Generate a slotted hub, matching ring, and separate spring-loaded pawls. | finnish-doors `experiments/spring_ratchet_fable/design.py` |
| `spring_cartridge_ratchet` | Extrude the spring-cartridge ring, hub, and pawl pieces. | finnish-doors `experiments/spring_ratchet_fable/design.py` |
| `check_ratchet_sense_and_sweep` | Validate drive sense, self-energising contact, cam-out, and retracted clearance. | finnish-doors `experiments/spring_ratchet_fable/design.py` |
| `compliant_clutch_2d` | Generate a compliant one-way or torque-limiting race and hub. | finnish-doors Klonk `gen_compliant_2d` |
| `compliant_clutch` | Extrude the compliant clutch race and flexure hub. | finnish-doors Klonk `gen_compliant_2d` |
| `arc_ratchet_2d` | Generate a tension-loaded compliant arc-arm ratchet. | finnish-doors historical Klonk follower ratchet |

### `mechlib.linkages`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `link_bar` | Build a flat link bar with bored pivot holes. | New, mechanical-movements wave v0.6.0 |
| `four_bar_pose` | Solve four-bar joint positions by circle-circle intersection. | New, mechanical-movements wave v0.6.0 |
| `four_bar` | Build an assembled four-bar linkage kit with printed pivot pins. | New, mechanical-movements wave v0.6.0 |
| `toggle_clamp` | Build an over-center knee toggle clamp posed near self-lock. | New, mechanical-movements wave v0.6.0 |
| `scotch_yoke` | Build a crank-and-pin slotted yoke with guide rails. | New, mechanical-movements wave v0.6.0 |
| `quick_return_ratio` | Compute the working:return time ratio of a crank quick-return. | New, mechanical-movements wave v0.6.0 |
| `quick_return` | Build a slotted-lever (Whitworth) quick-return mechanism. | New, mechanical-movements wave v0.6.0 |
| `peaucellier_pose` | Solve the Peaucellier-Lipkin inversor pose by geometric inversion. | New, gap-analysis wave v0.8.0 |
| `peaucellier_linkage` | Build the exact straight-line Peaucellier-Lipkin cell. | New, gap-analysis wave v0.8.0 |
| `watt_pose` | Solve Watt's parallel-motion pose for one lever angle. | New, gap-analysis wave v0.8.0 |
| `watt_linkage` | Build Watt's parallel motion and measure its straight-line error. | New, gap-analysis wave v0.8.0 |
| `sarrus_pose` | Solve the Sarrus platform lift from the fold angle. | New, gap-analysis wave v0.8.0 |
| `sarrus_linkage` | Build the spatial Sarrus pure-translation linkage. | New, gap-analysis wave v0.8.0 |
| `pantograph_pose` | Solve the pantograph pose and its exact scale ratio. | New, gap-analysis wave v0.8.0 |
| `pantograph_linkage` | Build a parallelogram pantograph that copies a shape to scale. | New, gap-analysis wave v0.8.0 |
| `lazy_tongs_pose` | Solve the Nuremberg-scissors span, height, and stroke gain. | New, gap-analysis wave v0.8.0 |
| `lazy_tongs` | Build a lazy-tongs extension chain with guided end yokes. | New, gap-analysis wave v0.8.0 |

### `mechlib.grippers`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `iris_diaphragm` | Build a stacked-plane iris diaphragm posed at one drive-ring angle. | New, gap-analysis wave v0.8.0 |
| `iris_control_range` | Return an iris drive ring's travel from wide open to its minimum aperture. | New, gap-analysis wave v0.8.0 |
| `collet_chuck` | Build an ER-style split collet with its taper nut and spindle nose. | New, gap-analysis wave v0.8.0 |
| `eccentric_cam_clamp` | Build an over-centre eccentric cam clamp posed at one handle angle. | New, gap-analysis wave v0.8.0 |

### `mechlib.cams`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `MOTION_LAWS` | The motion-law names usable in cam segments (dwell, linear, shm, cycloidal). | New, mechanical-movements wave v0.6.0 |
| `DEFAULT_SEGMENTS` | A stock rise-dwell-return-dwell cam segment program. | New, mechanical-movements wave v0.6.0 |
| `cam_lift` | Evaluate follower lift at an angle over motion-law segments. | New, mechanical-movements wave v0.6.0 |
| `cam_profile_2d` | Synthesize a roller-compensated cam profile polygon from segments. | New, mechanical-movements wave v0.6.0 |
| `plate_cam` | Extrude a plate cam with hub and D-flat or keyway bore. | New, mechanical-movements wave v0.6.0 |
| `snail_cam` | Build a snail drop cam with a single radial drop face. | New, mechanical-movements wave v0.6.0 |
| `heart_cam` | Build a constant-velocity heart cam with linear rise-fall. | New, mechanical-movements wave v0.6.0 |
| `barrel_cam` | Build a barrel cam with a closed motion-law groove and follower pin. | New, mechanical-movements wave v0.6.0 |

### `mechlib.indexing`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `geneva_pair` | Build an external Geneva driver and slotted wheel posed mid-engagement. | New, mechanical-movements wave v0.6.0 |
| `geneva_wheel_angle` | Return the exact crank-to-wheel angle relation of an external Geneva drive (dwell included). | New, mechanical-movements wave v0.6.0 |
| `escapement` | Build an escape wheel and anchor/deadbeat pallets with one pallet engaged. | New, mechanical-movements wave v0.6.0 |
| `intermittent_gear_pair` | Build a mutilated-gear intermittent pair with locking segments, posed meshed. | New, mechanical-movements wave v0.6.0 |

### `mechlib.fluid`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `gerotor_pump` | Build a trochoidal internal-gear pump with kidney ports. | New, gap-analysis wave v0.8.0 |
| `hose_barb` | Build a stacked-frustum hose tail with a flange or threaded foot. | New, gap-analysis wave v0.8.0 |
| `rotary_spool_valve` | Build a cross-drilled rotary plug valve with derived routing. | New, gap-analysis wave v0.8.0 |
| `peristaltic_pump_head` | Build a roller peristaltic head with a tangential tube race. | New, gap-analysis wave v0.8.0 |

### `mechlib.linear`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `scroll_drive` | Build a lathe-chuck scroll plate with three self-centering jaws. | New, mechanical-movements wave v0.6.0 |
| `differential_screw` | Build a two-pitch differential screw with its nut blocks. | New, mechanical-movements wave v0.6.0 |
| `archimedes_screw` | Build an inclined helical water screw in a half-pipe trough. | New, mechanical-movements wave v0.6.0 |

### `mechlib.guides`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `linear_way` | Build a prismatic linear guideway: rail, carriage, and tapered gib. | New, gap-analysis wave v0.8.0 |
| `telescoping_stage` | Build nested telescoping sections with anti-pullout stops. | New, gap-analysis wave v0.8.0 |

### `mechlib.couplings` and `mechlib.clutches`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `oldham_coupling` | Build a cross-slotted Oldham coupling for offset parallel shafts. | New, mechanical-movements wave v0.6.0 |
| `universal_joint` | Build a Cardan universal joint posed at a bend angle. | New, mechanical-movements wave v0.6.0 |
| `jaw_coupling` | Build interleaved jaw hubs with an elastomer spider. | New, mechanical-movements wave v0.6.0 |
| `torque_limiter` | Build a spring-detent slip clutch with geometry-set trip torque. | New, mechanical-movements wave v0.6.0 |
| `freewheel_clutch` | Build a roller-ramp one-way overrunning clutch. | New, mechanical-movements wave v0.6.0 |
| `tripod_cv_joint` | Build a plunging tripod constant-velocity joint posed at a shaft angle. | New, gap-analysis wave v0.8.0 |
| `double_cardan_joint` | Build two Hooke joints in series, intermediate yokes 90 degrees apart. | New, gap-analysis wave v0.8.0 |
| `tripod_pose` | Return the tripod's exact pose: housing angle, spider centre, orbit radius. | New, gap-analysis wave v0.8.0 |
| `cv_velocity_ratio` | Return the instantaneous output/input speed ratio of a Hooke or CV joint. | New, gap-analysis wave v0.8.0 |
| `cv_velocity_fluctuation` | Return the peak-to-peak Cardan speed error at a shaft angle. | New, gap-analysis wave v0.8.0 |

### `mechlib.joints`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `ball_socket_joint` | Build a snap-together spherical joint (ball stud plus split-finger socket). | New, gap-analysis wave v0.8.0 |
| `knuckle_hinge` | Build a print-in-place knuckle hinge with an integral hard stop. | New, gap-analysis wave v0.8.0 |
| `gimbal_rings` | Build nested print-in-place gimbal rings on alternating axes. | New, gap-analysis wave v0.8.0 |

### `mechlib.pulleys` and `mechlib.flexures`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `timing_pulley` | Build a GT2-style toothed belt pulley with flanges and hub. | New, mechanical-movements wave v0.6.0 |
| `grooved_drum` | Build a helically grooved cable drum (cylinder, cone, or fusee). | New, mechanical-movements wave v0.6.0 |
| `cross_flexure` | Build a monolithic cross-axis flexural pivot. | New, mechanical-movements wave v0.6.0 |
| `wave_spring` | Build a crest-to-crest annular wave spring. | New, mechanical-movements wave v0.6.0 |
| `bistable_beam` | Build a buckled-beam bistable switch with a central shuttle. | New, mechanical-movements wave v0.6.0 |
| `idler_pulley` | Build a crowned or toothed idler pulley with plain or bearing bore. | New, gap-analysis wave v0.8.0 |
| `eccentric_idler_mount` | Build an eccentric take-up bushing plus the idler pulley it carries. | New, gap-analysis wave v0.8.0 |
| `belt_tensioner` | Build a compliant-arm cantilever spring that self-preloads an idler. | New, gap-analysis wave v0.8.0 |
| `belleville_washer` | Build a coned disc spring, optionally as a series, parallel or alternating stack. | New, gap-analysis wave v0.8.0 |
| `coil_spring` | Build a helical compression spring with dead end coils and a round or rectangular wire. | New, gap-analysis wave v0.8.0 |
| `spiral_power_spring` | Build a flat spiral mainspring with its barrel and arbor. | New, gap-analysis wave v0.8.0 |
| `leaf_spring` | Build a semi-elliptic multi-leaf spring with a centre clamp band. | New, gap-analysis wave v0.8.0 |
| `flexure_stage` | Build a monolithic compound parallelogram straight-line flexure stage. | New, gap-analysis wave v0.8.0 |

### `mechlib.chains`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `drag_chain_link` | Build one print-in-place cable-carrier link with a one-sided stop. | New, gap-analysis wave v0.8.0 |
| `drag_chain` | Pose an N-link drag-chain run with a straight section and a bend. | New, gap-analysis wave v0.8.0 |
| `roller_chain_link` | Build one pitch segment of a bush roller chain matching `roller_sprocket_2d`. | New, gap-analysis wave v0.8.0 |
| `roller_chain` | Wrap a roller-chain run around a matching sprocket. | New, gap-analysis wave v0.8.0 |

### `mechlib.fasteners`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `zmin0` | Move a mesh onto the Z zero plane. | parviz `src/standins/_common.py` |
| `bolt_mesh` | Build a head-down bolt stand-in. | parviz `src/standins/_common.py` |
| `hex_nut_mesh` | Build a bored hex-nut stand-in. | parviz `src/standins/_common.py` |
| `washer_mesh` | Build an annular washer stand-in. | parviz `src/standins/_common.py` |
| `fastener_mesh` | Build and orient pan, SHCS, or CSK fasteners. | Unified from all three source projects |
| `pick_length` | Select the next usable standard screw length. | finnish-windows `tools/add_screws_glb.py` |

### `mechlib.bearings`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `plain_bushing` | Build a sleeve or flanged plain bushing with bore relief grooves. | New, gap-analysis wave v0.8.0 |
| `thrust_washer` | Build a relieved thrust washer, or a caged ball thrust pair. | New, gap-analysis wave v0.8.0 |
| `printed_ball_bearing` | Build a print-in-place radial ball bearing. | New, gap-analysis wave v0.8.0 |

### `mechlib.patterns` and `mechlib.text`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `polar_ring` | Return evenly spaced XY ring points. | Unified from finnish-windows `tools/add_teeth.py` |
| `lighten_cell_poly` | Build one rectangular or hex lightening cell. | finnish-doors `src/projects/klonk/housings.py` |
| `lighten_grid_centres` | Yield rect or hex lattice centers over a box. | finnish-doors `src/projects/klonk/housings.py` |
| `directed_holes` | Union bores along point-and-vector specifications. | massage-shower-head `build.py` |
| `text_polygon` | Convert text to counter-preserving polygons. | finnish-doors `src/projects/klonk/housings.py` |
| `place` | Center a 2D geometry at a point. | torque-lever `build.py` |
| `place_right` | Right-align a 2D geometry at a point. | torque-lever `build.py` |
| `text_block` | Stack centered multi-line text polygons. | torque-lever `build.py` |

### `mechlib.lattices`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `auxetic_panel` | Build a flat negative-Poisson's-ratio panel (reentrant, rotating-squares, or chiral cells). | New, gap-analysis wave v0.8.0 |
| `kerf_bend_cutter` | Build slit-array cutters that make a flat slab bend, twist, or roll (also the single-axis living-hinge case). | New, gap-analysis wave v0.8.0 |

### `mechlib.packing` and `mechlib.stepio`

| Function | Purpose | Origin project |
| --- | --- | --- |
| `shelf_pack` | Pack brim-grown footprints across multiple plates. | wall-shelf-clamp `tools/export_bambu.py` |
| `pack_by_category` | Group parts by category before plate packing. | wall-shelf-clamp `tools/export_bambu.py` |
| `export_assembly` | Export positioned triangle meshes as one STEP assembly. | dual-axis-turntable `src/step_export.py` |

## DECOUPLING CONTRACT

mechlib functions take explicit arguments only. They never read project parameter modules, never import from consuming projects, and never hold mutable module state. Consumers keep thin facade modules that bind their own params.

Current consumer: finnish-doors (Klonk). Planned migrations: finnish-windows, parviz, dual-axis-turntable, wall-shelf-clamp.

## Gallery

Explore every visual component in the [interactive 3D gallery](https://m-esm.github.io/mechlib/). Parts are shelved by what they do, movements first and primitives last, and are searchable by name, module, or description. Mechanisms with a real motion law run through a full cycle in their viewport. Anything marked tunable opens a parameter playground that re-runs mechlib itself in the browser through Pyodide, so you can drag a slider, watch the geometry rebuild, then copy runnable code or download the STL.

Run `python3 gallery/build_gallery.py` from the repository root to regenerate `docs/models/`. The build derives every category, badge, slider range, build cost, and animation track from the code itself and fails loudly rather than shipping stale metadata: an uncategorised module, a slider default outside its own range or off its step grid, and a body that does not move rigidly all stop the build.

Licensed under the MIT License.
