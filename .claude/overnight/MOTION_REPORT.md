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
