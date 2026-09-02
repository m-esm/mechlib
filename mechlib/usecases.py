"""Real-world machinery use cases for mechlib APIs.

This is the source of truth for "where would I use this part?" — for
humans, the gallery cards, and AI agents that pick geometry.

Keys are public function names (what you ``from mechlib import …``).
Gallery demos look up text by GLB file via ``applications_for_file``.

AI agents: when choosing a mechlib part for a design task, read this
module (or call ``use_case("four_bar")`` / ``search_use_cases("…")``)
and match the job to a concrete machinery situation rather than
inventing one-off geometry from boxes and cylinders.
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# Use cases by public API name
# ---------------------------------------------------------------------------

USE_CASES = {
    'vitamin': (
        "Bought bearings, ISO fasteners, motors, servos, cells, and sensors "
        "looked up by address (bearing/608-2rs) so a printed pocket rebinds "
        "from one catalog instead of a forked dimension table."
    ),
    'find_vitamin': (
        "Search the bought-part catalog by family or title when picking a "
        "608 vs 695, an M3 SHCS, or a GA12-N20 envelope for a new product."
    ),
    'vitamin_addresses': (
        "List every catalog address so a staleness gate can prove a posed "
        "assembly node still resolves after a catalog refresh."
    ),
    'annular_snap': (
        "Bottle-cap and filter-housing lids, pipe couplings, and round "
        "enclosure closures that snap together on a circumferential "
        "ridge."
    ),
    'arc_ratchet_2d': (
        "Tension-loaded one-way clutches, winch handles, and compact ratchets "
        "that use flexure arms instead of rigid pawls."
    ),
    'archimedes_screw': (
        "Irrigation and drainage lifts, bulk-material conveyors, and "
        "demonstration models of the classical water screw."
    ),
    'auxetic_panel': (
        "Impact pads, expandable meshes, medical and sports structures that "
        "widen when stretched, including Grima arrowhead and star-shaped "
        "honeycomb NPR cells, anti-tetrachiral opposite-sense square-grid "
        "NPR cells, and houndstooth interlocking-L / broken-chevron NPR cells."
    ),
    'ball_socket_joint': (
        "Control arms, camera gimbals, robotic wrists, and snap-together "
        "spherical joints for multi-axis motion."
    ),
    'barrel_cam': (
        "Fishing-reel level winds, tool-changer drums, textile traverse "
        "mechanisms, and cylindrical groove cams for axial programs."
    ),
    'bcc_lattice': (
        "3D-printed lightweight cores, energy-absorbing crush structures, and "
        "stiffness-tuned infill blocks that want a volumetric body-centred-"
        "cubic strut truss, not a flat 2D lightening sheet like honeycomb_panel "
        "or isogrid_panel."
    ),
    'octet_truss': (
        "Lightweight sandwich cores, stiff robotic frames, and load-bearing "
        "metamaterial blocks that need a true octet/FCC face-diagonal network "
        "of tetrahedral and octahedral cells rather than BCC body diagonals or "
        "a cubic edge grid."
    ),
    'kelvin_cell': (
        "Open-cell foam coupons, isotropic energy absorbers, and lightweight "
        "cores that need the 24-node, 36-edge Kelvin tetrakaidecahedral cell "
        "with six square and eight hexagonal faces rather than BCC or FCC "
        "connectivity."
    ),
    'beam_coupling': (
        "3D-printer Z axes, CNC builds, and motor-to-leadscrew links "
        "that need one cheap part forgiving angular, parallel, and "
        "axial misalignment."
    ),
    'bearing_seat': (
        "608 and similar skate-bearing pockets in robots, printers, and "
        "rollers that need a retained race."
    ),
    'bell_crank': (
        "Brake and clutch pedals, throttle and shift linkages, print-bed "
        "levellers, and any force redirect through a fixed angle."
    ),
    'belleville_washer': (
        "Bolted-joint preload, clutch packs, and high-load short-stroke "
        "stacks of disc springs."
    ),
    'bellows_suction_cup': (
        "TPU pick-and-place end effectors and vacuum handling of smooth "
        "parts, with a hose-barb stem for the vacuum line."
    ),
    'belt_tensioner': (
        "Automatic belt tensioners on engines and printers, compliant arms "
        "that keep constant belt load as the belt stretches."
    ),
    'bevel_gear_pair': (
        "Differentials, hand drills, right-angle gearboxes, and any "
        "intersecting-axis mesh at 90 degrees."
    ),
    'bistable_beam': (
        "Snap switches, tactile buttons, and mechanisms that click between "
        "two stable positions."
    ),
    'board_cradle': (
        "PCB mounting in instruments and printers: corner standoffs and "
        "capture walls around a board outline."
    ),
    'boxc': (
        "Blocks, spacers, and axis-aligned volumes that start most bracket "
        "and housing models."
    ),
    'chain_dual_output': (
        "Jackshafts and conveyor lines needing simultaneous forward and "
        "reverse take-offs from one chain run: a sprocket inside the loop "
        "turns with the driver, an idler on the back of a span turns opposite."
    ),
    'chain_reverse': (
        "Reverse-rotation takeoffs on conveyors, agricultural machinery, and "
        "machine drives: an idler sprocket bearing on the back of a chain "
        "span turns opposite to the driver."
    ),
    'chain_s_wrap': (
        "S-wrap reversing drives on conveyors, mixers, and farm machinery: "
        "the chain snakes over the driver and around the outside of the "
        "driven sprocket, so the output shaft turns opposite without gears."
    ),
    'chamfer_prism': (
        "Enclosures with a soft top edge, control knobs, and prisms that need "
        "a clean hull chamfer."
    ),
    'chebyshev_linkage': (
        "Straight-line walking and leg mechanisms, Russian school teaching "
        "models, and approximate linear guides with only revolute joints."
    ),
    'check_valve': (
        "Pump outlet lines, siphon breaks, and air lines that need one- "
        "way flow: a bought bearing ball on a conical seat."
    ),
    'clamshell_shiplap': (
        "Split enclosures, clamshell housings, and lid-to-base seams that "
        "interlock with a shiplap lip."
    ),
    'clevis': (
        "Actuator rod ends, turnbuckles, control links, and any pin joint "
        "that couples a fork to an eye."
    ),
    'coil_spring': (
        "Suspension and return springs, battery contacts, and general helical "
        "compression springs in machines."
    ),
    'collet_chuck': (
        "CNC spindle and router collets, lathe workholding, and any on-axis "
        "grip that must close concentrically on a round tool or stock."
    ),
    'compliant_clutch_2d': (
        "Torque-limiting knobs, safety clutches on feeders, and printed "
        "drives that should slip before something breaks."
    ),
    'counterbore': (
        "Socket-head and pan-head screw seats, flush fastener pockets in "
        "plates and brackets."
    ),
    'cross_flexure': (
        "Precision instrument pivots, watch balances, and monolithic hinges "
        "that flex instead of using a pin."
    ),
    'crush_ribs': (
        "Press-fit captures for PCBs, sensors, and rectangular inserts that "
        "should grip without glue."
    ),
    'cycloidal_drive': (
        "Robot joint reducers, cobot arms, and compact high-ratio drives "
        "where cycloidal geometry packs more torque than a planetary."
    ),
    'cyl': (
        "Bosses, pins, rollers, and any cylindrical stock or cutter axis in a "
        "parametric assembly."
    ),
    'dbore': (
        "Double-D motor and pot shafts, keyed hubs, and sockets that must "
        "transmit torque without a set screw."
    ),
    'detent_pair': (
        "Pan-tilt heads, adjustable arm joints, rotary selectors, and "
        "folding-leg locks that click positively into position."
    ),
    'differential_screw': (
        "Fine-adjustment stages, micrometers, and slow linear motion from two "
        "slightly different pitches on one shaft."
    ),
    'directed_holes': (
        "Shower heads, spray nozzles, and multi-axis hole patterns aimed "
        "along arbitrary vectors."
    ),
    'dog_clutch': (
        "Motorcycle and gearbox dog rings, PTO engagements, and positive-lock "
        "couplings that must not slip once seated."
    ),
    'dog_slot_coupling': (
        "Lost-motion couplings, indexed drives with backlash on purpose, and "
        "angular play between a dog and a slot."
    ),
    'double_cardan_joint': (
        "Steering columns and industrial drives that cancel single-U-joint "
        "speed fluctuation with two Hooke joints phased 90 degrees."
    ),
    'drag_chain': (
        "Full cable-carrier runs protecting wires and hoses on moving axes of "
        "machines and 3D printers; reverse_bend/s_bend_at pose RBR S-bend "
        "runs that snake between levels."
    ),
    'drag_chain_link': (
        "Cable carriers on CNC and robots; single link of an energy chain "
        "that flexes only one way (reverse_bend=True for RBR links that "
        "articulate both ways)."
    ),
    'eccentric_cam_clamp': (
        "Quick-release workholding on mills and fixtures, bicycle seat and "
        "stem clamps, and over-centre cam locks on jigs."
    ),
    'eccentric_idler_mount': (
        "Belt tension take-up on mills and printers: turn the eccentric to "
        "set belt preload without a sliding slot."
    ),
    'escapement': (
        "Longcase and printed clocks, metronomes, and any build that must "
        "release one tooth at a time under a pendulum or balance."
    ),
    'external_gear_pump': (
        "Hydraulic power units, oil transfer pumps, and compact "
        "positive-displacement gear pumps for viscous fluids."
    ),
    'extrude_twist': (
        "Twisted columns, augers, and decorative or functional solids swept "
        "with continuous rotation."
    ),
    'face_cam': (
        "Automotive and textile end cams, axial followers on face tracks, and "
        "compact lift programs normal to a disc face."
    ),
    'fastener_mesh': (
        "Assembly visualization: pan, socket, and countersunk screws with "
        "nuts and washers in CAD layouts."
    ),
    'flat_worm': (
        "Bench-proven multi-start flat worms for FDM gearboxes and high-ratio "
        "printed drives that must actually mesh."
    ),
    'flexure_stage': (
        "Optical and nanopositioning stages, vibration-isolated mounts, and "
        "printed straight-line stages without bearings."
    ),
    'flywheel': (
        "Punch presses, engines, spin-casters, and gyro demos that "
        "smooth shaft speed with a rim-heavy inertia wheel."
    ),
    'four_bar': (
        "Walking-robot legs, windshield-wiper kinematics, folding-furniture "
        "hinges, and any hobby build that needs a guided coupler curve."
    ),
    'freewheel_clutch': (
        "Bicycle freehubs, starter motors, and overrunning clutches that "
        "drive one way and freewheel the other."
    ),
    'frustum': (
        "Tapers, funnels, draft on molds, and stepped cones between two "
        "diameters."
    ),
    'gasket_channel': (
        "Enclosure lids and irregular flanges that seal with cord-stock "
        "gasket rather than a circular O-ring."
    ),
    'geneva_pair': (
        "Movie and film projectors, assembly turrets, indexing tables, and "
        "any station that needs dwell then a fixed step per revolution."
    ),
    'gerotor_pump': (
        "Oil pumps in engines and transmissions, compact hydraulic power "
        "packs, and positive-displacement lubrication circuits."
    ),
    'gimbal_rings': (
        "Camera and sensor gimbals, gyroscope mounts, and nested rings that "
        "free two or three rotation axes."
    ),
    'grooved_drum': (
        "Winches, cranes, fishing reels, and cable drums that need a helical "
        "groove so the rope stacks cleanly."
    ),
    'handwheel': (
        "Manual leadscrew and valve adjustment on screw jacks, indexing "
        "tables, and machine axes; the crank variant drives winches."
    ),
    'harmonic_drive': (
        "Cobot and industrial robot joints, pan-tilt heads, and anywhere you "
        "need huge ratio and near-zero backlash in a short package."
    ),
    'heart_cam': (
        "Chronograph hand reset (heart piece), sewing-machine take-up, and "
        "any cam that must return a pointer to a unique zero."
    ),
    'helix_tube': (
        "Spiral cable wraps, decorative helices, and swept tubes along a "
        "helical path."
    ),
    'herringbone_gear': (
        "Quiet high-load gearboxes, marine and industrial herringbone pairs, "
        "and FDM gears that cancel axial thrust from helix angle."
    ),
    'hex_poly': (
        "Nut and bolt blanks, hex columns, and across-flats hex stock for "
        "wrench-driven parts."
    ),
    'honeycomb_panel': (
        "Sandwich skins, drone-frame ribs, and lightened covers that need a "
        "regular hex core with a solid rim, not an auxetic bowtie."
    ),
    'hirth_coupling': (
        "Machine-tool spindles, indexing tables, and face couplings that must "
        "locate torque and centerline with radial teeth."
    ),
    'hose_barb': (
        "Coolant and air tubing on printers and lab gear, barbed hose tails "
        "that grip soft tube without a clamp."
    ),
    'idler_pulley': (
        "Belt path redirects on printers and conveyors, free-spinning pulleys "
        "that only change direction or take up slack."
    ),
    'intermittent_gear_pair': (
        "Mechanical counters, washing-machine timers, odometers, and "
        "digit-advance mechanisms that step once per input turn."
    ),
    'iris_diaphragm': (
        "Camera and projector apertures, laser beam expanders, soft-robot "
        "grippers, and printed iris valves for light or air."
    ),
    'isogrid_panel': (
        "Satellite payload skins, aircraft isogrid tanks, and printed "
        "lightened sheets that want triangular through-cells with 0/60/120 "
        "ribs and a solid rim, not a hex honeycomb."
    ),
    'kagome_panel': (
        "Lightweight printed skins and drone/robot ribs that want a Kagome "
        "trihexagonal lattice with both triangular and hexagonal through-cells "
        "and a solid rim, not a plain hex honeycomb or a triangle-only isogrid."
    ),
    'jaw_coupling': (
        "Servo and stepper motor couplings, pump shafts, and general flexible "
        "jaw (Lovejoy-style) connections with a spider."
    ),
    'kerf_bend_cutter': (
        "Living hinges, foldable enclosures, and kerf-bent panels that roll "
        "or twist from a flat print, including sinusoidal wave slits, "
        "hexagonal living-hinge edge slits, cross X-lattice living-hinge "
        "slits, chevron nested-arrowhead living-hinge slits, and "
        "diamond-outline brick-wall and fishbone herringbone living-hinge "
        "slits, plus a meander-labyrinth continuous square-wave kerf and "
        "biaxial orthogonal slits for a 2-axis wrap."
    ),
    'kinematic_coupling': (
        "Optical mounts, metrology fixtures, and any plate pair that must "
        "reseat to the same six-point pose every time."
    ),
    'knuckle_hinge': (
        "Print-in-place lids and doors, laptop-style hinges, and living "
        "hinges that need a hard stop at open or closed."
    ),
    'knurl': (
        "Thumb screws, adjustment knobs, and grip surfaces that need knurling "
        "without a lathe."
    ),
    'labyrinth_seal': (
        "Dust and splash seals on printed rotating shafts, non-contact comb "
        "seals where elastomer is unwanted."
    ),
    'lazy_tongs': (
        "Lazy-tongs riveters, scissor lifts, folding gates, and extendable "
        "booms that multiply a short squeeze into a long straight stroke."
    ),
    'lead_screw': (
        "Vices, C-clamps, Z-axis stages, and press screws: trapezoidal "
        "power threads turning rotation into linear force."
    ),
    'leaf_spring': (
        "Vehicle suspensions, clamp arms, and multi-leaf springs that carry "
        "load in bending."
    ),
    'lighten_grid_centres': (
        "Weight-saving panels on robots and drones, hex lattices that keep "
        "stiffness while cutting plastic."
    ),
    'linear_way': (
        "Machine-tool and 3D-printer linear guides, dovetail slides, and "
        "adjustable gibbed carriages."
    ),
    'loft': (
        "Organic housings, fairings, and solids that blend unequal "
        "cross-sections along a path."
    ),
    'nut_slot': (
        "Captive hex nuts in printed parts, T-nut slots, and any fastener "
        "seat that must not spin."
    ),
    'oldham_coupling': (
        "Misaligned parallel shafts on pumps and encoders, and printed "
        "couplings that tolerate offset without side load."
    ),
    'oldham_pose': (
        "Posing an Oldham coupling through a turn so the floating disc "
        "orbits at twice shaft speed: printer stepper-to-screw couplers "
        "and scroll-compressor drive trains."
    ),
    'oring_groove': (
        "Face-seal glands on lids and flanges, AS568 O-ring seats designed "
        "for correct squeeze and fill."
    ),
    'pantograph_linkage': (
        "Engraving and sign-cutting pantographs, scale-copying arms, and any "
        "setup that traces a shape at a fixed scale ratio."
    ),
    'peaucellier_linkage': (
        "Straight-line guides without a slideway: instrument mechanisms, "
        "historical drafting machines, and teaching models of exact motion."
    ),
    'peristaltic_pump_head': (
        "Lab and medical dosing pumps, food-safe fluid transfer, and any pump "
        "that must never touch the fluid with gears or seals."
    ),
    'plain_bushing': (
        "Journal bearings in printed machines, flanged sleeve bushings for "
        "shafts that only need sliding support."
    ),
    'planet_stage': (
        "Robot gearboxes, cordless-tool reducers, and multi-stage planetary "
        "transmissions with a fixed ring."
    ),
    'plate_cam': (
        "Model engines and automata valve timing, packaging-machine motion "
        "programs, and any radial cam that must follow a prescribed lift law."
    ),
    'press_lid': (
        "Battery and electronics boxes, snap-fit instrument lids, and "
        "friction-plug covers that press closed without screws."
    ),
    'printed_ball_bearing': (
        "Fully printed radial bearings for light-duty rollers, demo models, "
        "and places steel bearings will not fit the budget."
    ),
    'printed_worm': (
        "Fully printed worm reducers for robots and turntables, when metal "
        "worms are overkill and self-locking is welcome."
    ),
    'push_pin': (
        "Barbed press-fit pins for plastics, axles that push in and stay, and "
        "printed rivets with a lead-in."
    ),
    'quick_return': (
        "Metal shapers and slotters, packing machines, and press feeders "
        "where the return stroke should run faster than the working stroke."
    ),
    'rack_2d': (
        "Linear stages, rack-and-pinion steering, and CNC axes that convert "
        "rotation into straight travel along a pitch line."
    ),
    'rack_pinion': (
        "Steering racks, CNC and plotter axes, camera sliders, and linear "
        "stages driven by a spur pinion."
    ),
    'ratchet_ring_2d': (
        "Print-in-place freewheels, one-way knobs, and socket adapters that "
        "click when turning the drive way only."
    ),
    'ratchet_wheel_pawl': (
        "Winch drums, come-alongs, windlasses, and webbing tensioners "
        "that need a serviceable, high-torque one-way drive."
    ),
    'rbox': (
        "Enclosure walls and buttons with rounded corners that print cleanly "
        "and feel finished."
    ),
    'repeatable_dock': (
        "Tool changers, probe docks, and magnetic kinematic mounts that need "
        "preload so they do not fall apart when tipped."
    ),
    'revolved_gable_cavity': (
        "Annular chambers under self-supporting roofs, plumbing voids, and "
        "lightened rings."
    ),
    'ring_gear': (
        "Planetary ring gears, internal mesh housings, and turntable drives "
        "that run a pinion inside an annulus."
    ),
    'ring_gear_mesh': (
        "Epicyclic stages, slewing drives, and internal gear pairs where "
        "pinion and ring turn the same way."
    ),
    'roller_chain': (
        "Conveyors, motorcycles, and machine drives that wrap a roller chain "
        "around a sprocket."
    ),
    'roller_chain_link': (
        "Bicycle and industrial roller-chain pitches, printable chain "
        "segments matched to a sprocket."
    ),
    'roller_follower': (
        "Valve lifters, pump diaphragm drives, and automata: the "
        "pivoted roller lever that rides plate, heart, and snail cams."
    ),
    'roller_sprocket_2d': (
        "Bicycle and conveyor chain drives, 3D-printer motion systems, and "
        "roller-chain sprockets matched to a pin envelope."
    ),
    'rotary_spool_valve': (
        "Pneumatic and hydraulic direction valves, multi-port manifolds, and "
        "rotary selectors that route flow by plug angle."
    ),
    'saddle': (
        "Cradles for tubes and batteries, ribs that hug a cylinder without a "
        "full bore."
    ),
    'sarrus_linkage': (
        "Lift platforms and parallel stages that must rise without twisting; "
        "FDM-friendly alternative to linear rails for pure vertical travel."
    ),
    'scotch_yoke': (
        "Pneumatic and hydraulic valve actuators, shaper tables, and any "
        "drive that wants pure simple-harmonic stroke from a rotating shaft."
    ),
    'scott_russell_linkage': (
        "Compact exact straight-line guides, early steam and marine parallel "
        "motions, and instruments that can afford one sliding joint."
    ),
    'screw_jack': (
        "Car jacks, theater stage lifts, machine levelling feet, and "
        "self-locking screw actuators for slow heavy lifts."
    ),
    'screw_post': (
        "PCB standoffs, locating pins, screw bosses, and alignment features "
        "between mating plastic parts."
    ),
    'scroll_drive': (
        "Lathe chuck scroll plates, self-centering three-jaw mechanisms, and "
        "any drive that closes jaws equally from one rotation."
    ),
    'sector2d': (
        "Pie slices, partial flanges, and sector plates used as 2D profiles "
        "before extrusion."
    ),
    'seg_cylinder': (
        "Struts and links between skew points, space-frame bars, and "
        "cylinders that do not share an axis-aligned path."
    ),
    'setscrew': (
        "Collar and hub locks on shafts, pulley set-screw seats, and printed "
        "bosses that take a radial screw."
    ),
    'shaft_collar': (
        "Axially locating bearings, gears, and sprockets on a shaft "
        "without machining a shoulder into it."
    ),
    'shaft_key': (
        "Gear hubs, hand cranks, and pulley drives that transmit torque "
        "through a DIN 6885 sunk key where a D-bore would slip."
    ),
    'slider_crank': (
        "Piston engines and compressors, bicycle pumps, shapers, and every "
        "classic rotary-to-reciprocating conversion."
    ),
    'slot_cutter': (
        "Blade and tab slots in FDM parts, dog-bone relief so square inserts "
        "seat fully."
    ),
    'snail_cam': (
        "Clock strike trains, trip hammers, and slow-wind sudden-release "
        "mechanisms that need one drop per revolution."
    ),
    'snap_catch': (
        "Battery doors, access panels, and plastic enclosures that click shut "
        "with a catch and finger."
    ),
    'spiral_power_spring': (
        "Clock and toy mainsprings, retractable reels, and flat spiral power "
        "springs in a barrel."
    ),
    'spring_cartridge_ratchet_2d': (
        "Serviceable ratchet wrenches and winch freewheels where pawls and "
        "springs are separate replaceable parts."
    ),
    'spur_gear': (
        "Steering sectors, limited-travel instruments, and any place a full "
        "gear would waste space when only a few teeth are needed."
    ),
    'spur_gear_2d': (
        "Clockwork and instrument trains, printer extruder gears, and any "
        "parallel-shaft reduction that must mesh without clash."
    ),
    'spur_gear_mesh': (
        "Gearboxes, robot joints, and FDM-printed power transmission where you "
        "need a single involute gear blank."
    ),
    'ss_bore': (
        "Clamshell shaft cradles and support-light upper bores in split "
        "housings."
    ),
    'star_knob': (
        "Jigs, fixtures, machine adjustment points, and camera rigs "
        "that are tightened and loosened by hand."
    ),
    'star_wheel': (
        "Filling, capping, and sorting machines that meter bottles, "
        "cans, or bearings at a fixed pitch."
    ),
    'swash_plate': (
        "Axial piston pumps and motors, helicopter cyclic control analogues, "
        "and multi-piston rotary-to-linear conversion."
    ),
    'swept_keyed_bore': (
        "D-shaft hubs, keyed bores with free-rotation clearance, and sockets "
        "that must drive a shaft only over a limited angle."
    ),
    'tapered_cavity': (
        "Lightened legs and hollow structures whose roofs print without "
        "bridging failures."
    ),
    'teardrop': (
        "Horizontal bores in FDM parts that would otherwise need support: "
        "axles, cable holes, and fastener clearances."
    ),
    'telescoping_stage': (
        "Drawer slides, boom extensions, camera columns, and nested tubes "
        "that travel farther than any single section."
    ),
    'text_block': (
        "Multi-line plaques, version labels, and stacked raised text on "
        "enclosures and tools."
    ),
    'text_polygon': (
        "Raised labels on parts, nameplates, and logos that must print with "
        "counters preserved."
    ),
    'thread_insert': (
        "Enclosure lids opened repeatedly, motor mounts, and adjustment "
        "points that need durable heat-set machine-screw threads."
    ),
    'thread_solid': (
        "Printed screws and nuts, lead screws, and any FDM thread that must "
        "mate with a real or printed counterpart."
    ),
    'threaded_rod': (
        "Lead screws, all-thread stand-offs, and display rods where a fast "
        "radial-grid thread is enough."
    ),
    'three_point_leveller': (
        "Optical tables, printer beds, and surface plates that level with "
        "three screws without fighting an over-constrained bolt pattern."
    ),
    'thrust_washer': (
        "Axial load washers under gears and pulleys, thrust ball cages "
        "between rotating faces."
    ),
    'timing_pulley': (
        "GT2 and similar belt drives on printers and CNC, synchronous shaft "
        "coupling without slip."
    ),
    'tslot_nut': (
        "Bolting printed parts onto 2020/3030/4040 aluminium-extrusion "
        "frames and machine-table T-slots."
    ),
    'toggle_clamp': (
        "Welding and woodworking hold-downs, drill-press fixtures, and CNC "
        "fixture clamps that must lock solid without continuous force."
    ),
    'torque_limiter': (
        "Drill and screwdriver clutches, conveyor overload protection, and "
        "any shaft that must slip at a set torque."
    ),
    'torsion_spring_mesh': (
        "Clothes-peg and clip springs, hinge returns, and mechanisms that "
        "store energy in twist."
    ),
    'tripod_cv_joint': (
        "Front-wheel-drive halfshafts, plunging CV joints, and "
        "constant-velocity drives that also allow axial travel."
    ),
    'u_channel_between': (
        "Wire ways, cable channels, and open U runs that snake between "
        "arbitrary points on a panel."
    ),
    'universal_joint': (
        "Driveshafts, steering columns, and any angled shaft pair that can "
        "live with Cardan speed variation."
    ),
    'hooke_pose': (
        "Animating a Cardan joint through a turn so the output lags and "
        "leads twice per revolution: driveshafts, steering columns, and "
        "socket-wrench extensions at a known bend."
    ),
    'v_belt_pulley': (
        "Washing machines, drill presses, lathes, and HVAC blowers: "
        "wedge-belt power transmission between parallel shafts."
    ),
    'watt_linkage': (
        "Beam-engine parallel motion (historical), solid-axle rear suspension "
        "links, and approximate straight-line guides in vehicles."
    ),
    'wave_spring': (
        "Compact axial preload in bearings and seals, crest-to-crest springs "
        "where a coil spring is too tall."
    ),
    'wiper_kit': (
        "Wall-button single-pivot wiper kits (printed arm, zn and zp frame "
        "halves, aim stencil) that rebind the bought servo from "
        "vitamin(\"servo/sg90\") and vitamin(\"servo/mg90s\") instead of a "
        "forked envelope table."
    ),
    'worm': (
        "High-ratio right-angle reducers, tuning knobs, winches, and "
        "self-locking stages that should not back-drive."
    ),
    'worm_coupon': (
        "Quick mesh test coupons before committing a full gearbox print; "
        "iterate backlash and lead angle on a small pair."
    ),
    'ydovetail': (
        "Printable drawer slides, modular plate joins, and self-supporting "
        "dovetail tongues that assemble along Y."
    ),
}

# Related public names that share the same use-case text as a primary API.
ALIASES = {
    'arc_ratchet': 'arc_ratchet_2d',
    'blind_socket': 'screw_post',
    'chebyshev_pose': 'chebyshev_linkage',
    'compliant_clutch': 'compliant_clutch_2d',
    'dbore_hub': 'dbore',
    'fix_pin': 'screw_post',
    'lighten_cell_poly': 'lighten_grid_centres',
    'lighten_grid': 'lighten_grid_centres',
    'mesh_phase': 'spur_gear_2d',
    'pantograph_pose': 'pantograph_linkage',
    'peaucellier_pose': 'peaucellier_linkage',
    'pip_ratchet_hub': 'ratchet_ring_2d',
    'pip_ratchet_hub_2d': 'ratchet_ring_2d',
    'ratchet_ring': 'ratchet_ring_2d',
    'roller_sprocket': 'roller_sprocket_2d',
    'sarrus_pose': 'sarrus_linkage',
    'scott_russell_pose': 'scott_russell_linkage',
    'slider_crank_pose': 'slider_crank',
    'snap_finger': 'snap_catch',
    'spring_cartridge_ratchet': 'spring_cartridge_ratchet_2d',
    'tap': 'thread_solid',
    'watt_pose': 'watt_linkage',
    'worm_wheel_band': 'flat_worm',
}

# Gallery demos that share one API but need distinct copy (e.g. winch vs fusee).
GALLERY_FILE_OVERRIDES = {
    'fusee_demo.glb': (
        "Historical clock and watch fusees that equalize spring torque as the "
        "mainspring runs down."
    ),
    'winch_drum_demo.glb': (
        "Winches, cranes, fishing reels, and cable drums that need a helical "
        "groove so the rope stacks cleanly."
    ),
}

# GLB filename -> primary API (for gallery index generation).
GALLERY_FILE_TO_API = {
    'annular_snap_demo.glb': 'annular_snap',
    'beam_coupling_demo.glb': 'beam_coupling',
    'bellows_suction_cup_demo.glb': 'bellows_suction_cup',
    'check_valve_demo.glb': 'check_valve',
    'detent_pair_demo.glb': 'detent_pair',
    'flywheel_demo.glb': 'flywheel',
    'handwheel_demo.glb': 'handwheel',
    'lead_screw_demo.glb': 'lead_screw',
    'ratchet_wheel_pawl_demo.glb': 'ratchet_wheel_pawl',
    'roller_follower_demo.glb': 'roller_follower',
    'shaft_collar_demo.glb': 'shaft_collar',
    'shaft_key_demo.glb': 'shaft_key',
    'star_knob_demo.glb': 'star_knob',
    'star_wheel_demo.glb': 'star_wheel',
    'thread_insert_demo.glb': 'thread_insert',
    'tslot_nut_demo.glb': 'tslot_nut',
    'v_belt_pulley_demo.glb': 'v_belt_pulley',
    'arc_ratchet_2d_demo.glb': 'arc_ratchet_2d',
    'arc_ratchet_demo.glb': 'arc_ratchet_2d',
    'archimedes_screw_demo.glb': 'archimedes_screw',
    'auxetic_panel_demo.glb': 'auxetic_panel',
    'ball_socket_joint_demo.glb': 'ball_socket_joint',
    'barrel_cam_demo.glb': 'barrel_cam',
    'bcc_lattice_demo.glb': 'bcc_lattice',
    'octet_truss_demo.glb': 'octet_truss',
    'kelvin_cell_demo.glb': 'kelvin_cell',
    'bearing_seat_demo.glb': 'bearing_seat',
    'bell_crank_demo.glb': 'bell_crank',
    'belleville_washer_demo.glb': 'belleville_washer',
    'belt_tensioner_demo.glb': 'belt_tensioner',
    'bevel_gear_pair_demo.glb': 'bevel_gear_pair',
    'bistable_beam_demo.glb': 'bistable_beam',
    'board_cradle_demo.glb': 'board_cradle',
    'boxc_demo.glb': 'boxc',
    'chain_dual_output_demo.glb': 'chain_dual_output',
    'chain_reverse_demo.glb': 'chain_reverse',
    'chain_s_wrap_demo.glb': 'chain_s_wrap',
    'chamfer_prism_demo.glb': 'chamfer_prism',
    'chebyshev_linkage_demo.glb': 'chebyshev_linkage',
    'clamshell_shiplap_demo.glb': 'clamshell_shiplap',
    'clevis_demo.glb': 'clevis',
    'coil_spring_demo.glb': 'coil_spring',
    'collet_chuck_demo.glb': 'collet_chuck',
    'compliant_clutch_demo.glb': 'compliant_clutch_2d',
    'counterbore_demo.glb': 'counterbore',
    'cross_flexure_demo.glb': 'cross_flexure',
    'crush_ribs_demo.glb': 'crush_ribs',
    'cycloidal_drive_demo.glb': 'cycloidal_drive',
    'cyl_demo.glb': 'cyl',
    'dbore_demo.glb': 'dbore',
    'differential_screw_demo.glb': 'differential_screw',
    'directed_holes_demo.glb': 'directed_holes',
    'dog_clutch_demo.glb': 'dog_clutch',
    'dog_slot_coupling_demo.glb': 'dog_slot_coupling',
    'double_cardan_joint_demo.glb': 'double_cardan_joint',
    'drag_chain_demo.glb': 'drag_chain',
    'drag_chain_link_demo.glb': 'drag_chain_link',
    'eccentric_cam_clamp_demo.glb': 'eccentric_cam_clamp',
    'eccentric_idler_mount_demo.glb': 'eccentric_idler_mount',
    'escapement_demo.glb': 'escapement',
    'external_gear_pump_demo.glb': 'external_gear_pump',
    'extrude_twist_demo.glb': 'extrude_twist',
    'face_cam_demo.glb': 'face_cam',
    'fastener_trio_demo.glb': 'fastener_mesh',
    'find_vitamin_demo.glb': 'find_vitamin',
    'fix_pin_demo.glb': 'screw_post',
    'flat_worm_pair_demo.glb': 'flat_worm',
    'flexure_stage_demo.glb': 'flexure_stage',
    'four_bar_demo.glb': 'four_bar',
    'freewheel_clutch_demo.glb': 'freewheel_clutch',
    'frustum_demo.glb': 'frustum',
    'fusee_demo.glb': 'grooved_drum',
    'gasket_channel_demo.glb': 'gasket_channel',
    'geneva_pair_demo.glb': 'geneva_pair',
    'gerotor_pump_demo.glb': 'gerotor_pump',
    'gimbal_rings_demo.glb': 'gimbal_rings',
    'harmonic_drive_demo.glb': 'harmonic_drive',
    'heart_cam_demo.glb': 'heart_cam',
    'helix_tube_demo.glb': 'helix_tube',
    'herringbone_gear_demo.glb': 'herringbone_gear',
    'hex_poly_demo.glb': 'hex_poly',
    'honeycomb_panel_demo.glb': 'honeycomb_panel',
    'hirth_coupling_demo.glb': 'hirth_coupling',
    'hooke_pose_demo.glb': 'hooke_pose',
    'hose_barb_demo.glb': 'hose_barb',
    'idler_pulley_demo.glb': 'idler_pulley',
    'intermittent_gear_pair_demo.glb': 'intermittent_gear_pair',
    'iris_diaphragm_demo.glb': 'iris_diaphragm',
    'isogrid_panel_demo.glb': 'isogrid_panel',
    'kagome_panel_demo.glb': 'kagome_panel',
    'jaw_coupling_demo.glb': 'jaw_coupling',
    'kerf_bend_cutter_demo.glb': 'kerf_bend_cutter',
    'kinematic_coupling_demo.glb': 'kinematic_coupling',
    'knuckle_hinge_demo.glb': 'knuckle_hinge',
    'knurl_demo.glb': 'knurl',
    'labyrinth_seal_demo.glb': 'labyrinth_seal',
    'lazy_tongs_demo.glb': 'lazy_tongs',
    'leaf_spring_demo.glb': 'leaf_spring',
    'lighten_grid_demo.glb': 'lighten_grid_centres',
    'linear_way_demo.glb': 'linear_way',
    'loft_demo.glb': 'loft',
    'nut_slot_demo.glb': 'nut_slot',
    'oldham_coupling_demo.glb': 'oldham_coupling',
    'oldham_pose_demo.glb': 'oldham_pose',
    'oring_groove_demo.glb': 'oring_groove',
    'pantograph_linkage_demo.glb': 'pantograph_linkage',
    'peaucellier_linkage_demo.glb': 'peaucellier_linkage',
    'peristaltic_pump_head_demo.glb': 'peristaltic_pump_head',
    'pins_and_posts_demo.glb': 'screw_post',
    'pip_ratchet_demo.glb': 'ratchet_ring_2d',
    'plain_bushing_demo.glb': 'plain_bushing',
    'planet_stage_demo.glb': 'planet_stage',
    'plate_cam_demo.glb': 'plate_cam',
    'press_lid_demo.glb': 'press_lid',
    'printed_ball_bearing_demo.glb': 'printed_ball_bearing',
    'printed_worm_demo.glb': 'printed_worm',
    'push_pin_demo.glb': 'push_pin',
    'quick_return_demo.glb': 'quick_return',
    'rack_2d_demo.glb': 'rack_2d',
    'rack_pinion_demo.glb': 'rack_pinion',
    'rbox_demo.glb': 'rbox',
    'repeatable_dock_demo.glb': 'repeatable_dock',
    'revolved_gable_cavity_demo.glb': 'revolved_gable_cavity',
    'ring_gear_demo.glb': 'ring_gear',
    'ring_gear_mesh_demo.glb': 'ring_gear_mesh',
    'roller_chain_demo.glb': 'roller_chain',
    'roller_chain_link_demo.glb': 'roller_chain_link',
    'roller_sprocket_demo.glb': 'roller_sprocket_2d',
    'rotary_spool_valve_demo.glb': 'rotary_spool_valve',
    'saddle_demo.glb': 'saddle',
    'sarrus_linkage_demo.glb': 'sarrus_linkage',
    'scotch_yoke_demo.glb': 'scotch_yoke',
    'scott_russell_linkage_demo.glb': 'scott_russell_linkage',
    'screw_jack_demo.glb': 'screw_jack',
    'scroll_drive_demo.glb': 'scroll_drive',
    'sector2d_demo.glb': 'sector2d',
    'seg_cylinder_demo.glb': 'seg_cylinder',
    'setscrew_demo.glb': 'setscrew',
    'slider_crank_demo.glb': 'slider_crank',
    'slot_cutter_demo.glb': 'slot_cutter',
    'snail_cam_demo.glb': 'snail_cam',
    'snap_pair_demo.glb': 'snap_catch',
    'spiral_power_spring_demo.glb': 'spiral_power_spring',
    'spring_cartridge_ratchet_demo.glb': 'spring_cartridge_ratchet_2d',
    'spur_gear_mesh_demo.glb': 'spur_gear_mesh',
    'spur_gear_pair_demo.glb': 'spur_gear_2d',
    'spur_gear_sector_demo.glb': 'spur_gear',
    'ss_bore_demo.glb': 'ss_bore',
    'swash_plate_demo.glb': 'swash_plate',
    'swept_keyed_bore_demo.glb': 'swept_keyed_bore',
    'tapered_cavity_demo.glb': 'tapered_cavity',
    'teardrop_demo.glb': 'teardrop',
    'telescoping_stage_demo.glb': 'telescoping_stage',
    'text_block_demo.glb': 'text_block',
    'text_polygon_demo.glb': 'text_polygon',
    'thread_demo.glb': 'thread_solid',
    'threaded_rod_demo.glb': 'threaded_rod',
    'three_point_leveller_demo.glb': 'three_point_leveller',
    'thrust_washer_demo.glb': 'thrust_washer',
    'timing_pulley_demo.glb': 'timing_pulley',
    'toggle_clamp_demo.glb': 'toggle_clamp',
    'torque_limiter_demo.glb': 'torque_limiter',
    'torsion_spring_demo.glb': 'torsion_spring_mesh',
    'tripod_cv_joint_demo.glb': 'tripod_cv_joint',
    'u_channel_between_demo.glb': 'u_channel_between',
    'universal_joint_demo.glb': 'universal_joint',
    'vitamin_addresses_demo.glb': 'vitamin_addresses',
    'vitamin_demo.glb': 'vitamin',
    'watt_linkage_demo.glb': 'watt_linkage',
    'wave_spring_demo.glb': 'wave_spring',
    'winch_drum_demo.glb': 'grooved_drum',
    'worm_coupon_demo.glb': 'worm_coupon',
    'worm_demo.glb': 'worm',
    'ydovetail_demo.glb': 'ydovetail',
}


def use_case(name: str) -> str:
    """Return the machinery use-case text for a public API name.

    Raises ``KeyError`` when the name is unknown (after alias resolution).
    """
    key = ALIASES.get(name, name)
    if key not in USE_CASES:
        raise KeyError(
            "no use case for %r; add it to mechlib/usecases.py USE_CASES"
            % name)
    return USE_CASES[key]


def search_use_cases(query: str, limit: int = 12):
    """Return ``[(api_name, text), ...]`` whose use case mentions ``query``.

    Case-insensitive substring match over API name and use-case text.
    Intended for agents that need to pick a part from a job description.
    """
    q = (query or "").strip().lower()
    if not q:
        return []
    hits = []
    for name, text in sorted(USE_CASES.items()):
        blob = ("%s %s" % (name, text)).lower()
        if q in blob:
            hits.append((name, text))
    return hits[: max(1, int(limit))]


def applications_for_file(file_name: str, description: str = "") -> str:
    """Gallery helper: use-case text for a demo GLB filename."""
    if file_name in GALLERY_FILE_OVERRIDES:
        return GALLERY_FILE_OVERRIDES[file_name]
    api = GALLERY_FILE_TO_API.get(file_name)
    if api and api in USE_CASES:
        return USE_CASES[api]
    # Fallback: last sentence of the description.
    desc = (description or "").strip()
    if not desc:
        return "General mechanical design and FDM prototyping."
    parts = [p.strip() for p in desc.replace("—", ". ").split(".") if p.strip()]
    if len(parts) >= 2:
        return parts[-1] + "."
    return desc


def all_use_cases():
    """Sorted ``(api_name, text)`` pairs — full catalogue for agents."""
    return sorted(USE_CASES.items())
