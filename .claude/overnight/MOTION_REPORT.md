# Overnight motion review

Visual confirmation of gallery mechanisms. Generated overnight; not a hand-maintained registry.

Cutoff: 2026-08-13 07:30 EEST.

## demo_barrel_cam — pass

- reviewed: 2026-08-12T23:54:18+03:00
- kind: animate
- motion: Follower pin and blue guide orbit the stationary barrel while the pin stays seated in the helical groove and climbs/descends with the groove program.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: barrel_cam

Barrel is correctly stationary; pin_phase_deg walks the follower around. Last column at 340 deg continues toward the 40 deg start.

**Issues**
- none

## demo_bell_crank — pass

- reviewed: 2026-08-12T23:54:18+03:00
- kind: animate
- motion: Pink two-arm crank rotates on the fixed base pivot; orange and green links stay pinned to the arms and sweep a full pose cycle without detaching.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: base, pin_0

pose_deg is swept through nearly 360 rather than a limited rocker stroke; that matches the demo param. Ground plate stays put.

**Issues**
- none

## demo_bevel_gear_pair — fail

- reviewed: 2026-08-12T23:54:18+03:00
- kind: animate
- motion: A 90-degree bevel pair is posed correctly but tooth orientation is identical across drive_deg 0 through 900; neither wheel rotates.
- cycle_closes: True
- looks_like_intended: False
- frozen_that_should_move: bevel_gear, bevel_pinion

Rest pose looks like a Tredgold bevel pair. Animation is the failure. meta.moving is empty and both bodies are listed stationary.

**Issues**
- **high**: drive_deg advances 0/180/360/540/720/900 but both gears are visually frozen; mesh does not turn. Centroid travel is ~0 as expected for spin-in-place, so this is a real animation miss not a travel-metric artifact.

## demo_face_cam — pass

- reviewed: 2026-08-12T23:54:56+03:00
- kind: animate
- motion: Brown follower pin walks around the stationary pink face disc and rises then falls on the axial track, tallest near 180 deg and lowest near 0/300.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: cam

Cam disc stays put; pin_phase_deg is the moving coordinate. Front row makes the lift program obvious.

**Issues**
- none

## demo_four_bar — pass

- reviewed: 2026-08-12T23:54:56+03:00
- kind: animate
- motion: Pink crank rotates on the fixed blue ground; yellow coupler and green rocker follow without breaking joints, tracer sweeping a coupler curve.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: ground_link, pin_0, pin_1

Sample starts at 60 deg and ends at 360 (=0), so last column is not a visual match to first but is the next step back toward 60. Ground bar stays put.

**Issues**
- none

## demo_chebyshev — pass

- reviewed: 2026-08-12T23:58:06+03:00
- kind: animate
- motion: Lambda four-bar rocks through a sine stroke: mid poses at 0/180 match, the two extremes pair as 60/120 and 240/300, coupler and orange tracer travel while the blue ground stays put.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: ground, pin_0, pin_1

drive_deg is a closed sine swing about the mid angle, not a full crank turn. Last column sits on the same extreme as 240 and would return to mid at 360. Joints stay assembled; no jumps.

**Issues**
- none

## demo_cycloidal_drive — pass

- reviewed: 2026-08-12T23:58:06+03:00
- kind: animate
- motion: Red cycloidal disc walks and precesses around the fixed blue pin ring while the yellow eccentric orbits; housing stays put across the 3960 deg input sweep.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: housing_ring

meta lists output_plate stationary with ~0 centroid travel, which is expected for spin-in-place. 3300 deg last column is still approaching 3960=0, not a snap-back. Red showing through output holes is designed pin-hole contact.

**Issues**
- **low**: Output-plate spin is not visually confirmable: 6-hole symmetry matches the 60 deg output step per column, and no unique landmark on the green plate walks. Disc-to-pin walk is the readable motion.

## demo_dog_clutch — pass

- reviewed: 2026-08-12T23:58:06+03:00
- kind: animate
- motion: Green hub_b slides axially onto the fixed magenta hub: stack is tallest at 0/300 (withdrawn) and shortest at 180 (dogs fully nested), then opens again.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: hub_a

drive_deg is a cosine engage_frac (0 withdrawn, 180 seated). 300 continues toward the withdrawn start. Hubs stay coaxial; no relative spin through locked dogs.

**Issues**
- **low**: Mid-engagement columns (120-240) show a mushy sawtooth band at the dog interface, likely z-fighting as the teeth nest, not parts flying apart.

## demo_eccentric_cam_clamp — pass

- reviewed: 2026-08-12T23:58:06+03:00
- kind: animate
- motion: Red eccentric handle turns a full circle on the fixed orange pivot; the green follower is pressed and released as the offset disc sweeps past, blue base stays put.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: clamp_base, pivot_pin

Handle clock: ~1 o'clock at 0, then 11, 9, 7, 4, 3, heading back to 1. 180 is the clamp-overlap pose. Last column continues the rotation.

**Issues**
- none

## demo_escapement — fail

- reviewed: 2026-08-12T23:58:06+03:00
- kind: animate
- motion: Pink anchor rocks a few degrees (mostly 180 to 240/300) against a yellow escape wheel whose teeth and hub marks stay in the same clocking in every column.
- cycle_closes: True
- looks_like_intended: False
- frozen_that_should_move: escape_wheel

Rest pose looks like an anchor escapement. meta.stationary lists the wheel and centroid travel is ~0 (spin-in-place would also read 0), but the teeth themselves do not walk. Last column is still the rocked pose, so the sequence would close on the return to 360=0.

**Issues**
- **high**: Escape wheel does not step. Demo is supposed to advance two teeth over 360 deg of phase (~24 deg, ~20 deg by the last column). Hub marks and perimeter teeth are identical from 0 through 300. A rocking pallet on a locked wheel is not an escapement tick.
- **med**: Anchor motion is lumped into the last two columns; 0/60/120/180 look nearly the same rest pose rather than a beat-and-beat swing.

## demo_external_gear_pump — pass

- reviewed: 2026-08-12T23:58:06+03:00
- kind: animate
- motion: Figure-eight body and cap stay put while red and green flashes in the cap ports walk around the holes, showing the two internal gears turning in place.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: body, cap

Port flashes repeat every 180 deg (0~180, 60~240, 120~300), consistent with an even-tooth gear spinning. Housing never drifts. Last column continues the port walk.

**Issues**
- **low**: Side and front rows are visually identical across phase; mesh rotation is only readable through the iso cap ports. meta.moving is empty because centroid travel is ~0.

## note — phase aliasing

First bevel_gear_pair fail was a sheet bug: equal cycle/N samples hit tooth-pitch identity (180 deg on a 16-tooth pinion). Sheets were rebuilt with a one-frame step plus uneven fractions. Gear-family reviews were queued again.

## demo_herringbone_gear — pass

- reviewed: 2026-08-13T00:04:28+03:00
- kind: animate
- motion: Orange driver and blue driven herringbone pair stay in mesh while tooth and spoke highlights rotate across drive_deg 0/5/79/148/227/302. Centroids stay put as expected for spin-in-place gears.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Anti-alias resample: 0 vs 5 deg are nearly identical (5 deg of mesh) and later columns show clear chevron rotation on both wheels. Not a frozen-from-symmetry miss. Remaining 58 deg would return toward the 0 pose.

**Issues**
- none

## demo_lazy_tongs — pass

- reviewed: 2026-08-13T00:04:28+03:00
- kind: animate
- motion: Red frame stays put. Yellow scissor bars fold from an open zigzag into a compact stack around 80-150 deg, then flatten and shoot the blue output yoke out along the axis by 300 deg.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: frame, pin_0

Nuremberg-scissors stroke: retract then extend in one 360 drive. Front row shows the stack thinning as the rhombs flatten. Pins travel with the joints; nothing detaches.

**Issues**
- none

## demo_pantograph — pass

- reviewed: 2026-08-13T00:04:28+03:00
- kind: animate
- motion: Blue base pad stays fixed. Orange pivot bar, yellow/purple/green parallelogram, stylus pin and output point sweep a closed pose cycle while the parallelogram stays closed.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: base_pad, pin_f

Last column at 302 deg is already near the 0 deg pose (orange bar left, output toward the pad). Scale-copy geometry holds; no broken pins.

**Issues**
- none

## demo_peaucellier — pass

- reviewed: 2026-08-13T00:04:28+03:00
- kind: animate
- motion: Green ground stays put. Orange crank orbits, purple rhombus opens and folds, red/yellow anchor links rock, and the tracer end travels as the inversor works through the drive.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: ground_link, pin_c, pin_o

Cell stays assembled across 0-302 deg. Side row shows the orange crank walking left-right while the rhombus inverts. Last column is still opening, consistent with 58 deg left in the cycle.

**Issues**
- none

## demo_plate_cam — pass

- reviewed: 2026-08-13T00:04:28+03:00
- kind: animate
- motion: Blue plate cam spins in place while the orange roller stays on the profile and the green follower stem walks clockwise around the cam with changing lift (short at 60 deg, longer at 139 deg).
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Demo param is follower_deg starting at 60. Follower orbits rather than sliding on a fixed axis: kinematic inversion of a radial cam, lift still reads off the stem length. Last column at 362.4 continues clockwise toward the 60 deg start.

**Issues**
- none

## demo_quick_return — pass

- reviewed: 2026-08-13T00:04:28+03:00
- kind: animate
- motion: Yellow crank pin rides the orange disc and slides along the green slotted lever. Lever stays nearly flat through 40-188 deg, then rocks up sharply by 267 deg and starts back down at 342 deg (quick return).
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: base, pin_crank, pin_lever

Crank disc is a featureless orange circle so rotation is read from the pin orbit, not disc markings. Blue base and both ground pins stay put. Last column at 342 deg is heading back toward the 40 deg start.

**Issues**
- none

## demo_rack_pinion — pass

- reviewed: 2026-08-13T00:05:40+03:00
- kind: animate
- motion: Red pinion spins in place (tooth orientation walks) and the green rack translates along the pitch line, staying in mesh. Rack left end walks toward the pinion from 0 to 302 deg.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Pinion centroid is fixed as expected. 0 vs 10 deg are close; later columns show both rotation and ~17 mm of rack travel. Last column still short of a full turn back to 0.

**Issues**
- none

## demo_ring_gear_mesh — pass

- reviewed: 2026-08-13T00:05:40+03:00
- kind: animate
- motion: Orange pinion stays centered on the left of the blue internal ring and its face spokes rotate through drive_deg 0/7.5/158/295/454/605. Mesh at the ring ID holds. Ring teeth walk slowly (high ratio).
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Anti-alias resample: 0 vs 7.5 deg look nearly identical; later columns are not a frozen copy. Cycle is 720 deg so last column at 605 is still short of close. Both listed stationary because travel is spin-in-place.

**Issues**
- none

## demo_rotary_spool_valve — pass

- reviewed: 2026-08-13T00:05:40+03:00
- kind: animate
- motion: Blue body and orange cap stay put. Brown plug rotates in the bore: hex socket walks, the stem slot appears and disappears in the side row, and the L-passage lines up with the body port around 227 deg (see-through in the side view).
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: spool_valve_body, spool_valve_cap

Centroids are all ~0 as expected for a rotary valve. Plug_deg 0-302 is a partial turn; last column is heading back toward the 0 hex pose.

**Issues**
- none

## demo_sarrus — pass

- reviewed: 2026-08-13T00:05:40+03:00
- kind: animate
- motion: Purple base stays put. Orthogonal yellow/pink hinge chains fold and the cyan platform rises (0-148 deg, bars near vertical) then drops (227-302 deg) with no visible twist or sideways drift.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: base_plate, hinge_pin_0, hinge_pin_1

Side and front rows confirm pure vertical travel. 302 deg is still low, consistent with rising again toward the mid-height 0 deg pose.

**Issues**
- none

## demo_bevel_gear_pair — pass

- reviewed: 2026-08-13T00:06:44+03:00
- kind: animate
- motion: Blue pinion and green crown rotate in place on 90-degree axes; tooth clocking and the pinion hole pattern walk across drive_deg 0 / 7.5 / 238 / 443 / 680 / 907.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Prior fail was 180 deg aliasing on a 16/24 pair. Anti-alias phases make the mesh turn obvious in iso and front. Centroid travel stays ~0 as expected for spin-in-place.

**Issues**
- none

## demo_cycloidal_drive — pass

- reviewed: 2026-08-13T00:06:44+03:00
- kind: animate
- motion: Red cycloidal disc walks and precesses around the fixed blue pin ring while the gold eccentric orbits; housing stays put across the 3960 deg input sweep.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: housing_ring

meta lists output_plate stationary with ~0 centroid travel, expected for spin-in-place. Last column at 3326 deg is still approaching 3960=0. Red showing through output holes is designed pin-hole contact.

**Issues**
- **low**: Output-plate spin is not visually confirmable: 6-hole symmetry plus a weak landmark on the green plate. Disc-to-pin walk is the readable motion.

## demo_escapement — fail

- reviewed: 2026-08-13T00:06:44+03:00
- kind: animate
- motion: Pink anchor rocks through the phase sweep against a yellow escape wheel whose hub ticks and perimeter teeth stay in the same clocking in every column.
- cycle_closes: True
- looks_like_intended: False
- frozen_that_should_move: escape_wheel

Rest pose looks like an anchor escapement. Anti-alias phases (0/7.5/79/148/227/302) now show a more gradual anchor rock than the old 60 deg samples; the wheel still never advances. Centroid travel ~0 is expected for spin-in-place, but the teeth themselves do not walk.

**Issues**
- **high**: Escape wheel does not step. Demo should advance two teeth over 360 deg of phase. Hub marks (three ticks at 11/12/1) and perimeter teeth are identical from 0 through 302. A rocking pallet on a locked wheel is not an escapement tick.

## demo_external_gear_pump — pass

- reviewed: 2026-08-13T00:06:44+03:00
- kind: animate
- motion: Figure-eight body and cap stay put while red and green flashes in the cap ports walk around the holes, showing the two internal gears turning in place.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: body, cap

Port flashes walk continuously across 0/7.5/79/148/227/302, consistent with gears spinning in the bore. Housing never drifts. Last column continues the port walk toward 360=0.

**Issues**
- **low**: Side and front rows are visually identical across phase; mesh rotation is only readable through the iso cap ports. meta.moving is empty because centroid travel is ~0.

## demo_geneva_pair — fail

- reviewed: 2026-08-13T00:06:44+03:00
- kind: animate
- motion: Orange pin-driver rotates through dwell and engagement poses, but the blue 6-slot wheel keeps the same arm clocking in every column.
- cycle_closes: True
- looks_like_intended: False
- frozen_that_should_move: slotted_wheel

Driver motion is real (crescent and pin walk around). Wheel centroid travel ~0 is expected for in-place indexing, so this is an orientation miss. Last column at 1814 deg is another in-window pose and still matches the 0 deg star.

**Issues**
- **high**: Slotted wheel does not index. Columns 0 (crank 0, mid-engagement) and 1 (crank 20, still in the 60 deg window) should differ by ~18 deg of wheel rotation; the star silhouette is identical. Dwell columns cannot prove a 60 deg step on a 6-slot wheel, but the in-window pair can and it does not move.

## demo_gerotor_pump — pass

- reviewed: 2026-08-13T00:06:44+03:00
- kind: animate
- motion: Housing and kidney-port cap stay put while red/green/yellow lobe flashes walk along both ports, showing the inner and outer rotors turning in place through the 60 deg pitch cycle.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: gerotor_housing, gerotor_port_cap

Side and front rows hide the rotors, same as the external gear pump. Iso port colors at 50.4 deg are returning toward the 0 deg pattern, as expected for a 60 deg closed cycle. Centroid travel ~0 is spin-in-place.

**Issues**
- none

## demo_gimbal_rings — pass

- reviewed: 2026-08-13T00:06:44+03:00
- kind: animate
- motion: Blue outer ring stays level while the red middle ring swings from nearly flat through vertical and back, and the pink inner ring rides its trunnions through the same tilt_deg sweep.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: gimbal_ring_0

Single-parameter tilt swings the nested rings rather than holding a payload level against a moving frame; that matches tilt_deg on a fixed outer ring. Last column at 322 deg is heading back toward the 20 deg start pose. Near-zero centroid travel is rotation about the trunnion axes.

**Issues**
- none

## demo_heart_cam — pass

- reviewed: 2026-08-13T00:06:44+03:00
- kind: animate
- motion: Green follower stem and orange roller orbit the stationary pink heart disc, staying on the rim while radial reach grows near 180 deg and shrinks toward 0/360.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: heart_cam

Same stationary-cam / walking-follower convention as barrel_cam and face_cam. Heart outline reads almost circular at this scale; the lift program is the changing stem length, longest near follower_deg 169 and shortest near 32/90. Last column at 392=32 continues toward the 90 deg start.

**Issues**
- none

## demo_scotch_yoke — pass

- reviewed: 2026-08-13T00:08:37+03:00
- kind: animate
- motion: Gold crank pin orbits on the orange disc while the blue slotted yoke reciprocates in the purple rails, long at 35/337 deg and shortest near 183 deg.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: crank_disc, rail_a, rail_b

Front row makes the SHM stroke obvious. Disc has no orientation landmark so spin-with-pin is not confirmable; the pin orbit is the driving motion. Last column at 337 deg continues toward the 35 deg start.

**Issues**
- none

## demo_scott_russell — pass

- reviewed: 2026-08-13T00:08:37+03:00
- kind: animate
- motion: Green inversor bar rocks on the yellow crank while the slider rides the base slot and the orange tracer stays on the guide line through the sine-mapped swing.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: base, pin_0

drive_deg is a closed sine around mid-angle, not a full crank turn, so the bar rocks rather than windmills. Joints stay pinned. Last column at 302 deg is heading back toward the 0 deg pose.

**Issues**
- none

## demo_screw_jack — pass

- reviewed: 2026-08-13T00:08:38+03:00
- kind: animate
- motion: Orange pad rides the pink screw from down at 0/15 deg to full height near 148/227 deg and back down by 302 deg; blue base stays put.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: base, screw

Demo is a cosine lift_frac out-and-back so the cycle can close (0=down, 180=up). Screw does not spin; the pad translates on the thread. That matches the demo, not a missed rotation.

**Issues**
- none

## demo_slider_crank — pass

- reviewed: 2026-08-13T00:08:38+03:00
- kind: animate
- motion: Orange crank pin orbits the red disc; green conrod stays pinned and the cyan slider reciprocates in the base slot, inboard near 183 deg and outboard near 35/337 deg.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: base, pin_0

Front row is the readable piston stroke. Crank disc has little landmark and ~0 centroid travel; the pin orbit carries the rotation. Last column at 337 deg continues toward the 35 deg start.

**Issues**
- none

## demo_snail_cam — pass

- reviewed: 2026-08-13T00:23:14+03:00
- kind: animate
- motion: Yellow snail stays put while the orange roller and green stem orbit it, riding a slow Archimedean rise then jumping inward at the radial drop face.
- cycle_closes: False
- looks_like_intended: True
- frozen_that_should_move: none

Kinematic inversion: follower_deg walks the stem around a fixed cam. Drop-face jump at mid phases is expected and not a fail. Sample ends at 552.4 deg, still closing toward 610 (=250+360).

**Issues**
- none

## demo_spur_gear_mesh — pass

- reviewed: 2026-08-13T00:23:16+03:00
- kind: animate
- motion: Single 20-tooth involute gear spins in place. Top-face hatch and tooth silhouettes advance clockwise across the 18 deg pitch cycle.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Centroid travel is ~0 as expected for spin-in-place. Tooth orientation, not centroid, shows the rotation. Last column 15.1 deg is approaching the 18 deg close. Face hatching is a render alias, not a mesh defect.

**Issues**
- none

## demo_spur_gear_pair — pass

- reviewed: 2026-08-13T00:23:19+03:00
- kind: animate
- motion: Orange 18-tooth driver and green 28-tooth driven stay in mesh and spin in place. Driver face hatch and both tooth rows advance; contact stays clash-free.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Centroids frozen (spin-in-place). 16.8 deg of driver is most of the 20 deg cycle. Driven rotation is slower (18/28) but visible on the green teeth in iso and front.

**Issues**
- none

## demo_swash_plate — pass

- reviewed: 2026-08-13T00:23:21+03:00
- kind: animate
- motion: Tilted pink plate spins under four colored shoes. Shoes stay on their angular stations and stroke axially, high side walking around the plate as phase advances.
- cycle_closes: False
- looks_like_intended: True
- frozen_that_should_move: none

Shaft stays on axis. Plate is catalog-stationary because its centroid barely translates; the tilt orientation rotating is the intended spin. Last column 302.4 deg is approaching the 360 start. Shoe heights at 0 vs 302 are consistent with a continuing cycle.

**Issues**
- none

## demo_toggle_clamp — pass

- reviewed: 2026-08-13T00:23:24+03:00
- kind: animate
- motion: Pink handle folds over the yellow connecting link, the knee pin travels, and the brown clamp arm rises then settles as the toggle goes through and past center. Blue base and its two fixed pins stay put.
- cycle_closes: False
- looks_like_intended: True
- frozen_that_should_move: none

Closed=false; 302.4 deg is heading back toward the upright handle pose of column 0. Compact travels (~5-8 mm) match the short links. Mid columns (147-226) sit near the flattened/locked pose.

**Issues**
- none

## demo_tripod_cv_joint — pass

- reviewed: 2026-08-13T00:23:26+03:00
- kind: animate
- motion: Blue tulip stays fixed at a shaft angle. Orange input shaft and pink crowned barrels rotate inside the three tracks: the shaft cross-hole walks left to front to right, and the barrels progress around the housing.
- cycle_closes: False
- looks_like_intended: True
- frozen_that_should_move: none

Catalog lists every body stationary because centroid travel is only ~0.6 mm (spin about a near-center axis plus a little plunge). Orientation, not centroid, shows the CV rotation. Housing does not spin. Last column 302.4 deg is approaching 360.

**Issues**
- none

## demo_watt — pass

- reviewed: 2026-08-13T00:24:23+03:00
- kind: animate
- motion: Pink and green levers rock on the fixed blue ground while the yellow coupler and orange tracer sweep an approximate straight line. Front row keeps the tracer near a level stroke; mid columns show the expected bow at the ends.
- cycle_closes: False
- looks_like_intended: True
- frozen_that_should_move: none

Ground and its two cyan pivots stay put. Floating pins travel with the joints. 302.4 deg is folding back toward the 0 pose. No broken joints or explosions.

**Issues**
- none

## demo_arc_ratchet — pass

- reviewed: 2026-08-13T00:24:25+03:00
- kind: movement
- motion: Rest pose: cyan hub with three trailing arc flexures seated inside a pink internally toothed ring.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Non-animated. Arms trail in the self-energising direction and sit in the undercut teeth. Side/front show a flat two-body disc. Sane rest assembly.

**Issues**
- none

## demo_archimedes_screw — pass

- reviewed: 2026-08-13T00:24:28+03:00
- kind: movement
- motion: Rest pose: cyan helical flight on a shaft lying in a teal half-pipe trough, inclined.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Non-animated. Front row is the trough end-on with the shaft centered. Flight stays inside the trough; no detached bodies. Sane rest assembly of a water/grain screw.

**Issues**
- none

## demo_ball_socket_joint — pass

- reviewed: 2026-08-13T00:24:30+03:00
- kind: movement
- motion: Rest pose: pink ball stud snapped into a blue split-finger socket, stem leaving the mouth at an angle inside the cone.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Non-animated. Side row shows the slots and the lip past the ball equator. Stud is retained, not exploded out of the cup. Sane rest assembly.

**Issues**
- none

## demo_belt_tensioner — pass

- reviewed: 2026-08-13T00:41:42+03:00
- kind: movement
- motion: Rest pose: purple cantilever arm with a cyan idler pulley on the tip, mount block at the other end.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Non-animated. Idler sits on the arm axis; front row shows the slight curve of the spring arm. Two bodies, no explode. Sane rest assembly.

**Issues**
- none

## demo_check_valve — pass

- reviewed: 2026-08-13T00:41:45+03:00
- kind: movement
- motion: Rest pose: green snap-on cap with a hose barb stacked on a blue body with a matching barb, on one axis.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Non-animated. Ball is a bought part so only body+cap print. Cap seats on the body; barbs on both ports. Sane rest assembly.

**Issues**
- none

## demo_clevis — pass

- reviewed: 2026-08-13T00:41:47+03:00
- kind: movement
- motion: Rest pose: magenta U-fork, green eye between the ears, cyan pin through all three.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Non-animated. Side row is the pin axis; front is the fork face. Clearance looks designed, not exploded. Sane rest assembly of a pin joint.

**Issues**
- none

## demo_collet_chuck — pass

- reviewed: 2026-08-13T00:41:50+03:00
- kind: movement
- motion: Rest pose: green knurled taper nut on a blue threaded spindle nose, orange collet bore visible on axis.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Non-animated. Three bodies stacked concentrically. Threads and knurl read; nothing floating off-axis. Sane rest assembly of an ER-style chuck.

**Issues**
- none

## demo_compliant_clutch — pass

- reviewed: 2026-08-13T00:41:52+03:00
- kind: movement
- motion: Rest pose: cyan spiral flexure hub seated inside an orange internal-sawtooth race.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Non-animated. Flexure tips sit in the teeth; side/front are a flat two-body disc. Sane rest assembly of a torque-limit clutch.

**Issues**
- none

## demo_detent_pair — pass

- reviewed: 2026-08-13T00:41:55+03:00
- kind: movement
- motion: Rest pose: green plunger in a blue housing clicking into a notch of the pink dial, coil spring visible in the pocket.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Non-animated. Front row shows the plunger tip in a tooth gap. Housing, plunger, and wheel stay assembled. Sane rest assembly.

**Issues**
- none

## demo_differential_screw — pass

- reviewed: 2026-08-13T00:42:20+03:00
- kind: movement
- motion: Rest pose: gold twin-pitch shaft through a purple frame nut and a cyan moving nut stacked on one axis.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Non-animated. Shaft threads show at both ends; nuts sit concentric, not exploded. Sane rest assembly of a differential screw.

**Issues**
- none

## demo_double_cardan_joint — pass

- reviewed: 2026-08-13T00:42:22+03:00
- kind: movement
- motion: Rest pose: two Hooke joints in series, purple yokes and an angled intermediate, green/yellow spiders, cyan output yoke.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Non-animated. Intermediate sits at a working angle; both crosses are captured in the forks. No exploded spiders. Sane rest assembly of a double Cardan.

**Issues**
- none

## demo_drag_chain — pass

- reviewed: 2026-08-13T00:42:25+03:00
- kind: movement
- motion: Rest pose: eight-link cable carrier with a straight run and a U-bend to the designed min radius, lids on, pins in.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Non-animated. Links stay chained; bend is one-way as designed. Side/front show the trough and lids. Sane rest assembly of an energy-chain run.

**Issues**
- none

## demo_drag_chain_link — pass

- reviewed: 2026-08-13T00:42:27+03:00
- kind: movement
- motion: Rest pose: one trough link with a press-fit lid and a red pivot pin in the male boss.
- cycle_closes: True
- looks_like_intended: True
- frozen_that_should_move: none

Non-animated. Lid caps the cable channel; pin is seated. Sane rest assembly of a single energy-chain link.

**Issues**
- none
