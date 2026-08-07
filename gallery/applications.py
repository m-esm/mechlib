"""Real-world machinery situations for each gallery demo.

Keyed by GLB filename so two demos of the same function (winch vs fusee
drums) can differ. Text is "where you would meet this part in the wild",
not API usage. Consumed by build_gallery when writing index.json.
"""

# Short "used in …" lines. Keep them concrete: machines, not abstractions.
APPLICATIONS = {
    # Linkages & clamps
    "four_bar_demo.glb": (
        "Walking-robot legs, windshield-wiper kinematics, folding-furniture "
        "hinges, and any hobby build that needs a guided coupler curve."
    ),
    "toggle_clamp_demo.glb": (
        "Welding and woodworking hold-downs, drill-press fixtures, and CNC "
        "fixture clamps that must lock solid without continuous force."
    ),
    "scotch_yoke_demo.glb": (
        "Pneumatic and hydraulic valve actuators, shaper tables, and any "
        "drive that wants pure simple-harmonic stroke from a rotating shaft."
    ),
    "quick_return_demo.glb": (
        "Metal shapers and slotters, packing machines, and press feeders "
        "where the return stroke should run faster than the working stroke."
    ),
    "peaucellier_linkage_demo.glb": (
        "Straight-line guides without a slideway: instrument mechanisms, "
        "historical drafting machines, and teaching models of exact motion."
    ),
    "watt_linkage_demo.glb": (
        "Beam-engine parallel motion (historical), solid-axle rear "
        "suspension links, and approximate straight-line guides in vehicles."
    ),
    "sarrus_linkage_demo.glb": (
        "Lift platforms and parallel stages that must rise without twisting; "
        "FDM-friendly alternative to linear rails for pure vertical travel."
    ),
    "pantograph_linkage_demo.glb": (
        "Engraving and sign-cutting pantographs, scale-copying arms, and "
        "any setup that traces a shape at a fixed scale ratio."
    ),
    "lazy_tongs_demo.glb": (
        "Lazy-tongs riveters, scissor lifts, folding gates, and extendable "
        "booms that multiply a short squeeze into a long straight stroke."
    ),
    "slider_crank_demo.glb": (
        "Piston engines and compressors, bicycle pumps, shapers, and every "
        "classic rotary-to-reciprocating conversion."
    ),
    "chebyshev_linkage_demo.glb": (
        "Straight-line walking and leg mechanisms, Russian school teaching "
        "models, and approximate linear guides with only revolute joints."
    ),
    "scott_russell_linkage_demo.glb": (
        "Compact exact straight-line guides, early steam and marine "
        "parallel motions, and instruments that can afford one sliding joint."
    ),
    "bell_crank_demo.glb": (
        "Brake and clutch pedals, throttle and shift linkages, print-bed "
        "levellers, and any force redirect through a fixed angle."
    ),
    "iris_diaphragm_demo.glb": (
        "Camera and projector apertures, laser beam expanders, soft-robot "
        "grippers, and printed iris valves for light or air."
    ),
    "collet_chuck_demo.glb": (
        "CNC spindle and router collets, lathe workholding, and any "
        "on-axis grip that must close concentrically on a round tool or stock."
    ),
    "eccentric_cam_clamp_demo.glb": (
        "Quick-release workholding on mills and fixtures, bicycle seat and "
        "stem clamps, and over-centre cam locks on jigs."
    ),

    # Cams & indexing
    "plate_cam_demo.glb": (
        "Model engines and automata valve timing, packaging-machine motion "
        "programs, and any radial cam that must follow a prescribed lift law."
    ),
    "snail_cam_demo.glb": (
        "Clock strike trains, trip hammers, and slow-wind sudden-release "
        "mechanisms that need one drop per revolution."
    ),
    "heart_cam_demo.glb": (
        "Chronograph hand reset (heart piece), sewing-machine take-up, and "
        "any cam that must return a pointer to a unique zero."
    ),
    "barrel_cam_demo.glb": (
        "Fishing-reel level winds, tool-changer drums, textile traverse "
        "mechanisms, and cylindrical groove cams for axial programs."
    ),
    "face_cam_demo.glb": (
        "Automotive and textile end cams, axial followers on face tracks, "
        "and compact lift programs normal to a disc face."
    ),
    "geneva_pair_demo.glb": (
        "Movie and film projectors, assembly turrets, indexing tables, and "
        "any station that needs dwell then a fixed step per revolution."
    ),
    "escapement_demo.glb": (
        "Longcase and printed clocks, metronomes, and any build that must "
        "release one tooth at a time under a pendulum or balance."
    ),
    "intermittent_gear_pair_demo.glb": (
        "Mechanical counters, washing-machine timers, odometers, and "
        "digit-advance mechanisms that step once per input turn."
    ),

    # Gears & drives
    "spur_gear_pair_demo.glb": (
        "Clockwork and instrument trains, printer extruder gears, and any "
        "parallel-shaft reduction that must mesh without clash."
    ),
    "spur_gear_mesh_demo.glb": (
        "General spur gearing: gearboxes, robot joints, and FDM-printed "
        "power transmission where you need a single involute gear blank."
    ),
    "roller_sprocket_demo.glb": (
        "Bicycle and conveyor chain drives, 3D-printer motion systems, and "
        "roller-chain sprockets matched to a pin envelope."
    ),
    "worm_demo.glb": (
        "High-ratio right-angle reducers, tuning knobs, winches, and "
        "self-locking stages that should not back-drive."
    ),
    "spur_gear_sector_demo.glb": (
        "Steering sectors, limited-travel instruments, and any place a full "
        "gear would waste space when only a few teeth are needed."
    ),
    "rack_2d_demo.glb": (
        "Linear stages, rack-and-pinion steering, and CNC axes that convert "
        "rotation into straight travel along a pitch line."
    ),
    "herringbone_gear_demo.glb": (
        "Quiet high-load gearboxes, marine and industrial herringbone "
        "pairs, and FDM gears that cancel axial thrust from helix angle."
    ),
    "cycloidal_drive_demo.glb": (
        "Robot joint reducers, cobot arms, and compact high-ratio drives "
        "where cycloidal geometry packs more torque than a planetary."
    ),
    "bevel_gear_pair_demo.glb": (
        "Differentials, hand drills, right-angle gearboxes, and any "
        "intersecting-axis mesh at 90 degrees."
    ),
    "ring_gear_demo.glb": (
        "Planetary ring gears, internal mesh housings, and turntable "
        "drives that run a pinion inside an annulus."
    ),
    "ring_gear_mesh_demo.glb": (
        "Epicyclic stages, slewing drives, and internal gear pairs where "
        "pinion and ring turn the same way."
    ),
    "printed_worm_demo.glb": (
        "Fully printed worm reducers for robots and turntables, when metal "
        "worms are overkill and self-locking is welcome."
    ),
    "flat_worm_pair_demo.glb": (
        "Bench-proven multi-start flat worms for FDM gearboxes and "
        "high-ratio printed drives that must actually mesh."
    ),
    "worm_coupon_demo.glb": (
        "Quick mesh test coupons before committing a full gearbox print; "
        "iterate backlash and lead angle on a small pair."
    ),
    "planet_stage_demo.glb": (
        "Robot gearboxes, cordless-tool reducers, and multi-stage "
        "planetary transmissions with a fixed ring."
    ),
    "harmonic_drive_demo.glb": (
        "Cobot and industrial robot joints, pan-tilt heads, and anywhere "
        "you need huge ratio and near-zero backlash in a short package."
    ),

    # Ratchets & clutches
    "pip_ratchet_demo.glb": (
        "Print-in-place freewheels, one-way knobs, and socket adapters "
        "that click when turning the drive way only."
    ),
    "spring_cartridge_ratchet_demo.glb": (
        "Serviceable ratchet wrenches and winch freewheels where pawls "
        "and springs are separate replaceable parts."
    ),
    "compliant_clutch_demo.glb": (
        "Torque-limiting knobs, safety clutches on feeders, and printed "
        "drives that should slip before something breaks."
    ),
    "arc_ratchet_demo.glb": (
        "Tension-loaded one-way clutches, winch handles, and compact "
        "ratchets that use flexure arms instead of rigid pawls."
    ),
    "torque_limiter_demo.glb": (
        "Drill and screwdriver clutches, conveyor overload protection, "
        "and any shaft that must slip at a set torque."
    ),
    "freewheel_clutch_demo.glb": (
        "Bicycle freehubs, starter motors, and overrunning clutches that "
        "drive one way and freewheel the other."
    ),
    "dog_clutch_demo.glb": (
        "Motorcycle and gearbox dog rings, PTO engagements, and "
        "positive-lock couplings that must not slip once seated."
    ),

    # Couplings & joints
    "oldham_coupling_demo.glb": (
        "Misaligned parallel shafts on pumps and encoders, and printed "
        "couplings that tolerate offset without side load."
    ),
    "universal_joint_demo.glb": (
        "Driveshafts, steering columns, and any angled shaft pair that "
        "can live with Cardan speed variation."
    ),
    "jaw_coupling_demo.glb": (
        "Servo and stepper motor couplings, pump shafts, and general "
        "flexible jaw (Lovejoy-style) connections with a spider."
    ),
    "tripod_cv_joint_demo.glb": (
        "Front-wheel-drive halfshafts, plunging CV joints, and constant-"
        "velocity drives that also allow axial travel."
    ),
    "double_cardan_joint_demo.glb": (
        "Steering columns and industrial drives that cancel single-U-joint "
        "speed fluctuation with two Hooke joints phased 90 degrees."
    ),
    "hirth_coupling_demo.glb": (
        "Machine-tool spindles, indexing tables, and face couplings that "
        "must locate torque and centerline with radial teeth."
    ),
    "ball_socket_joint_demo.glb": (
        "Control arms, camera gimbals, robotic wrists, and snap-together "
        "spherical joints for multi-axis motion."
    ),
    "knuckle_hinge_demo.glb": (
        "Print-in-place lids and doors, laptop-style hinges, and living "
        "hinges that need a hard stop at open or closed."
    ),
    "gimbal_rings_demo.glb": (
        "Camera and sensor gimbals, gyroscope mounts, and nested rings "
        "that free two or three rotation axes."
    ),
    "clevis_demo.glb": (
        "Actuator rod ends, turnbuckles, control links, and any pin joint "
        "that couples a fork to an eye."
    ),

    # Fluid
    "gerotor_pump_demo.glb": (
        "Oil pumps in engines and transmissions, compact hydraulic "
        "power packs, and positive-displacement lubrication circuits."
    ),
    "hose_barb_demo.glb": (
        "Coolant and air tubing on printers and lab gear, barbed hose "
        "tails that grip soft tube without a clamp."
    ),
    "rotary_spool_valve_demo.glb": (
        "Pneumatic and hydraulic direction valves, multi-port manifolds, "
        "and rotary selectors that route flow by plug angle."
    ),
    "peristaltic_pump_head_demo.glb": (
        "Lab and medical dosing pumps, food-safe fluid transfer, and "
        "any pump that must never touch the fluid with gears or seals."
    ),
    "external_gear_pump_demo.glb": (
        "Hydraulic power units, oil transfer pumps, and compact "
        "positive-displacement gear pumps for viscous fluids."
    ),

    # Linear & screw
    "scroll_drive_demo.glb": (
        "Lathe chuck scroll plates, self-centering three-jaw mechanisms, "
        "and any drive that closes jaws equally from one rotation."
    ),
    "differential_screw_demo.glb": (
        "Fine-adjustment stages, micrometers, and slow linear motion from "
        "two slightly different pitches on one shaft."
    ),
    "archimedes_screw_demo.glb": (
        "Irrigation and drainage lifts, bulk-material conveyors, and "
        "demonstration models of the classical water screw."
    ),
    "swash_plate_demo.glb": (
        "Axial piston pumps and motors, helicopter cyclic control "
        "analogues, and multi-piston rotary-to-linear conversion."
    ),
    "screw_jack_demo.glb": (
        "Car jacks, theater stage lifts, machine levelling feet, and "
        "self-locking screw actuators for slow heavy lifts."
    ),
    "rack_pinion_demo.glb": (
        "Steering racks, CNC and plotter axes, camera sliders, and "
        "linear stages driven by a spur pinion."
    ),
    "linear_way_demo.glb": (
        "Machine-tool and 3D-printer linear guides, dovetail slides, "
        "and adjustable gibbed carriages."
    ),
    "telescoping_stage_demo.glb": (
        "Drawer slides, boom extensions, camera columns, and nested "
        "tubes that travel farther than any single section."
    ),

    # Pulleys & chain
    "timing_pulley_demo.glb": (
        "GT2 and similar belt drives on printers and CNC, synchronous "
        "shaft coupling without slip."
    ),
    "winch_drum_demo.glb": (
        "Winches, cranes, fishing reels, and cable drums that need a "
        "helical groove so the rope stacks cleanly."
    ),
    "fusee_demo.glb": (
        "Historical clock and watch fusees that equalize spring torque "
        "as the mainspring runs down."
    ),
    "idler_pulley_demo.glb": (
        "Belt path redirects on printers and conveyors, free-spinning "
        "pulleys that only change direction or take up slack."
    ),
    "eccentric_idler_mount_demo.glb": (
        "Belt tension take-up on mills and printers: turn the eccentric "
        "to set belt preload without a sliding slot."
    ),
    "belt_tensioner_demo.glb": (
        "Automatic belt tensioners on engines and printers, compliant "
        "arms that keep constant belt load as the belt stretches."
    ),
    "drag_chain_link_demo.glb": (
        "Cable carriers on CNC and robots; single link of an energy "
        "chain that flexes only one way."
    ),
    "drag_chain_demo.glb": (
        "Full cable-carrier runs protecting wires and hoses on moving "
        "axes of machines and 3D printers."
    ),
    "roller_chain_link_demo.glb": (
        "Bicycle and industrial roller-chain pitches, printable chain "
        "segments matched to a sprocket."
    ),
    "roller_chain_demo.glb": (
        "Conveyors, motorcycles, and machine drives that wrap a "
        "roller chain around a sprocket."
    ),

    # Flexures & springs
    "cross_flexure_demo.glb": (
        "Precision instrument pivots, watch balances, and monolithic "
        "hinges that flex instead of using a pin."
    ),
    "wave_spring_demo.glb": (
        "Compact axial preload in bearings and seals, crest-to-crest "
        "springs where a coil spring is too tall."
    ),
    "bistable_beam_demo.glb": (
        "Snap switches, tactile buttons, and mechanisms that click "
        "between two stable positions."
    ),
    "belleville_washer_demo.glb": (
        "Bolted-joint preload, clutch packs, and high-load short-stroke "
        "stacks of disc springs."
    ),
    "coil_spring_demo.glb": (
        "Suspension and return springs, battery contacts, and general "
        "helical compression springs in machines."
    ),
    "spiral_power_spring_demo.glb": (
        "Clock and toy mainsprings, retractable reels, and flat spiral "
        "power springs in a barrel."
    ),
    "leaf_spring_demo.glb": (
        "Vehicle suspensions, clamp arms, and multi-leaf springs that "
        "carry load in bending."
    ),
    "flexure_stage_demo.glb": (
        "Optical and nanopositioning stages, vibration-isolated mounts, "
        "and printed straight-line stages without bearings."
    ),

    # Threads & fasteners
    "thread_demo.glb": (
        "Printed screws and nuts, lead screws, and any FDM thread that "
        "must mate with a real or printed counterpart."
    ),
    "knurl_demo.glb": (
        "Thumb screws, adjustment knobs, and grip surfaces that need "
        "knurling without a lathe."
    ),
    "torsion_spring_demo.glb": (
        "Clothes-peg and clip springs, hinge returns, and mechanisms "
        "that store energy in twist."
    ),
    "threaded_rod_demo.glb": (
        "Lead screws, all-thread stand-offs, and display rods where a "
        "fast radial-grid thread is enough."
    ),
    "helix_tube_demo.glb": (
        "Spiral cable wraps, decorative helices, and swept tubes along "
        "a helical path."
    ),
    "dog_slot_coupling_demo.glb": (
        "Lost-motion couplings, indexed drives with backlash on purpose, "
        "and angular play between a dog and a slot."
    ),
    "fastener_trio_demo.glb": (
        "Assembly visualization: pan, socket, and countersunk screws "
        "with nuts and washers in CAD layouts."
    ),
    "plain_bushing_demo.glb": (
        "Journal bearings in printed machines, flanged sleeve bushings "
        "for shafts that only need sliding support."
    ),
    "thrust_washer_demo.glb": (
        "Axial load washers under gears and pulleys, thrust ball cages "
        "between rotating faces."
    ),
    "printed_ball_bearing_demo.glb": (
        "Fully printed radial bearings for light-duty rollers, demo "
        "models, and places steel bearings will not fit the budget."
    ),

    # Closures & fixtures
    "press_lid_demo.glb": (
        "Battery and electronics boxes, snap-fit instrument lids, and "
        "friction-plug covers that press closed without screws."
    ),
    "clamshell_shiplap_demo.glb": (
        "Split enclosures, clamshell housings, and lid-to-base seams "
        "that interlock with a shiplap lip."
    ),
    "ydovetail_demo.glb": (
        "Printable drawer slides, modular plate joins, and self-"
        "supporting dovetail tongues that assemble along Y."
    ),
    "snap_pair_demo.glb": (
        "Battery doors, access panels, and plastic enclosures that "
        "click shut with a catch and finger."
    ),
    "nut_slot_demo.glb": (
        "Captive hex nuts in printed parts, T-nut slots, and any "
        "fastener seat that must not spin."
    ),
    "pins_and_posts_demo.glb": (
        "PCB standoffs, locating pins, screw bosses, and alignment "
        "features between mating plastic parts."
    ),
    "push_pin_demo.glb": (
        "Barbed press-fit pins for plastics, axles that push in and "
        "stay, and printed rivets with a lead-in."
    ),
    "setscrew_demo.glb": (
        "Collar and hub locks on shafts, pulley set-screw seats, and "
        "printed bosses that take a radial screw."
    ),
    "board_cradle_demo.glb": (
        "PCB mounting in instruments and printers: corner standoffs "
        "and capture walls around a board outline."
    ),
    "saddle_demo.glb": (
        "Cradles for tubes and batteries, ribs that hug a cylinder "
        "without a full bore."
    ),
    "kinematic_coupling_demo.glb": (
        "Optical mounts, metrology fixtures, and any plate pair that "
        "must reseat to the same six-point pose every time."
    ),
    "repeatable_dock_demo.glb": (
        "Tool changers, probe docks, and magnetic kinematic mounts "
        "that need preload so they do not fall apart when tipped."
    ),
    "three_point_leveller_demo.glb": (
        "Optical tables, printer beds, and surface plates that level "
        "with three screws without fighting an over-constrained bolt pattern."
    ),

    # Primitives & cutters
    "cyl_demo.glb": (
        "Bosses, pins, rollers, and any cylindrical stock or cutter "
        "axis in a parametric assembly."
    ),
    "boxc_demo.glb": (
        "Blocks, spacers, and axis-aligned volumes that start most "
        "bracket and housing models."
    ),
    "rbox_demo.glb": (
        "Enclosure walls and buttons with rounded corners that print "
        "cleanly and feel finished."
    ),
    "frustum_demo.glb": (
        "Tapers, funnels, draft on molds, and stepped cones between "
        "two diameters."
    ),
    "sector2d_demo.glb": (
        "Pie slices, partial flanges, and sector plates used as 2D "
        "profiles before extrusion."
    ),
    "hex_poly_demo.glb": (
        "Nut and bolt blanks, hex columns, and across-flats hex stock "
        "for wrench-driven parts."
    ),
    "chamfer_prism_demo.glb": (
        "Enclosures with a soft top edge, control knobs, and prisms "
        "that need a clean hull chamfer."
    ),
    "seg_cylinder_demo.glb": (
        "Struts and links between skew points, space-frame bars, and "
        "cylinders that do not share an axis-aligned path."
    ),
    "extrude_twist_demo.glb": (
        "Twisted columns, augers, and decorative or functional solids "
        "swept with continuous rotation."
    ),
    "swept_keyed_bore_demo.glb": (
        "D-shaft hubs, keyed bores with free-rotation clearance, and "
        "sockets that must drive a shaft only over a limited angle."
    ),
    "loft_demo.glb": (
        "Organic housings, fairings, and solids that blend unequal "
        "cross-sections along a path."
    ),
    "teardrop_demo.glb": (
        "Horizontal bores in FDM parts that would otherwise need "
        "support: axles, cable holes, and fastener clearances."
    ),
    "ss_bore_demo.glb": (
        "Clamshell shaft cradles and support-light upper bores in "
        "split housings."
    ),
    "dbore_demo.glb": (
        "Double-D motor and pot shafts, keyed hubs, and sockets that "
        "must transmit torque without a set screw."
    ),
    "counterbore_demo.glb": (
        "Socket-head and pan-head screw seats, flush fastener pockets "
        "in plates and brackets."
    ),
    "bearing_seat_demo.glb": (
        "608 and similar skate-bearing pockets in robots, printers, "
        "and rollers that need a retained race."
    ),
    "crush_ribs_demo.glb": (
        "Press-fit captures for PCBs, sensors, and rectangular "
        "inserts that should grip without glue."
    ),
    "slot_cutter_demo.glb": (
        "Blade and tab slots in FDM parts, dog-bone relief so square "
        "inserts seat fully."
    ),
    "tapered_cavity_demo.glb": (
        "Lightened legs and hollow structures whose roofs print "
        "without bridging failures."
    ),
    "u_channel_between_demo.glb": (
        "Wire ways, cable channels, and open U runs that snake "
        "between arbitrary points on a panel."
    ),
    "revolved_gable_cavity_demo.glb": (
        "Annular chambers under self-supporting roofs, plumbing "
        "voids, and lightened rings."
    ),
    "oring_groove_demo.glb": (
        "Face-seal glands on lids and flanges, AS568 O-ring seats "
        "designed for correct squeeze and fill."
    ),
    "labyrinth_seal_demo.glb": (
        "Dust and splash seals on printed rotating shafts, non-"
        "contact comb seals where elastomer is unwanted."
    ),
    "gasket_channel_demo.glb": (
        "Enclosure lids and irregular flanges that seal with cord-"
        "stock gasket rather than a circular O-ring."
    ),
    "lighten_grid_demo.glb": (
        "Weight-saving panels on robots and drones, hex lattices "
        "that keep stiffness while cutting plastic."
    ),
    "directed_holes_demo.glb": (
        "Shower heads, spray nozzles, and multi-axis hole patterns "
        "aimed along arbitrary vectors."
    ),
    "auxetic_panel_demo.glb": (
        "Impact pads, expandable meshes, medical and sports "
        "structures that widen when stretched."
    ),
    "kerf_bend_cutter_demo.glb": (
        "Living hinges, foldable enclosures, and kerf-bent panels "
        "that roll or twist from a flat print."
    ),
    "text_polygon_demo.glb": (
        "Raised labels on parts, nameplates, and logos that must "
        "print with counters preserved."
    ),
    "text_block_demo.glb": (
        "Multi-line plaques, version labels, and stacked raised "
        "text on enclosures and tools."
    ),
}


def applications_for(file_name, description=""):
    """Return the applications line for a gallery GLB, or a description fallback."""
    text = APPLICATIONS.get(file_name)
    if text:
        return text
    # Fallback: last sentence of the description if no map entry yet.
    desc = (description or "").strip()
    if not desc:
        return "General mechanical design and FDM prototyping."
    parts = [p.strip() for p in desc.replace("—", ". ").split(".") if p.strip()]
    if len(parts) >= 2:
        return parts[-1] + "."
    return desc
