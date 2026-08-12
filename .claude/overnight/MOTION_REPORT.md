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
