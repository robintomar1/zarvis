# Zarvis v1 requirements

This document collects the requirements agreed in the first grilling session (2026-10-07). Terms are defined in [CONTEXT.md](../CONTEXT.md).

Every item is **provisional**, because each one is still to be reviewed individually. Items tagged **revisit** were flagged during the session as needing more information.

## Mission

- **R0**: **v1 is a Teleop platform with no autonomy.** v1 is done when a person can carry out Tidy-up and Fetch entirely through Teleop, and every run is recorded as a Demonstration. Autonomy comes later, from Policies trained on those Demonstrations.
- **R1**: Zarvis's long-term job is **Tidy-up**: it picks up Clutter from the floor and puts it where it belongs. Where things belong is not programmed. It is learned from Demonstrations, by watching where the operator puts things.
- **R2**: Zarvis's long-term job is also **Fetch**: it carries a requested object from one place in the Home to another.
- **R3**: Zarvis is developed through Teleop: Demonstrations are recorded and Policies are trained from them.
- **R4**: Out of scope for v1:
  - surface tasks such as wiping or loading the dishwasher
  - opening doors, drawers and cabinets
  - reaching under furniture

## Home

- **R5**: Zarvis is built for one Home: tile floors and thin rugs, all on a single level.
- **R6**: Zarvis crosses edges (rug edges, thresholds) up to 10 mm.
- **R7**: Zarvis passes through 70 cm gaps and can turn in place inside them. **(revisit)**

## Reach and payload

- **R8**: The Reach Envelope runs from the floor up to the top of a 90 cm kitchen counter, reaching about 30 cm in from the counter edge.
- **R9**: The Gripper lifts at least 0.5 kg.
- **R10**: The Carry Bin holds at least 3 kg.
- **R11**: Zarvis has a Lift with about 65 cm of stroke, so the Arm's mounting plate moves from about 20 cm to about 85–90 cm above the floor. This allows top-down grasps on the counter (see [arm reachability](research/arm-reachability.md)). Whether to buy or build it is still open; the builder is researching options. **(revisit)**
- **R29**: The Lift holds its position when it has no power, for example through a self-locking lead screw.
- **R30**: The Lift carries at least 10 kg, travels its full stroke in 10 s or less, and knows its absolute height at power-up.
  - **Note for choosing a Lift:** a 65 cm stroke in 10 s needs about 6.5 cm/s. Off-the-shelf standing-desk columns move at 3–4 cm/s, which is 16–22 s for the full stroke.
  - Desk columns are also typically 55–65 cm tall when retracted, so reaching R32's 20 cm mount height needs a bracket that hangs the Arm below the carriage.
  - Either relax the 10 s, or pick a lead-screw carriage.

## Layout

- **R12**: The footprint is at most 45 × 45 cm, *including* the Lift and the Arm in its Stowed Pose. Zarvis needs this to turn in place in a 70 cm gap (R7). Mounting the Lift on the front face (R32) eats into it. **(revisit)**
- **R13**: The Base has four mecanum wheels driven by Unitree IM6014s (see [ADR-0001](adr/0001-mecanum-base-on-im6014-actuators.md)). How the wheels are supported on bearings is decided in the chassis design. **(revisit)**
- **R14**: All four wheels stay in contact with the floor when crossing a 10 mm edge.
- **R15**: The Lift and Arm sit at the front of the Base, and the Carry Bin sits low at the back.
- **R32**: The Lift runs down the front face of the Base, so its carriage can lower the Arm's mounting plate to about 20 cm above the floor, below the deck. The arm axis sits at the Base's front edge, so the mounting plate overhangs the Base by about 7 cm. For top-down counter grasps, the Base stops about 9 cm short of the counter edge. The Arm needs this to reach the floor (see [arm reachability](research/arm-reachability.md)).
- **R39**: The real lower limit of the Arm's shoulder joint (J2) must be confirmed with Seeed. The URDF and the vendor parts list ("Upper Arm Horizontal Limit Block") both say the upper arm can't point below horizontal. If the hardware allows more, R32's 20 cm minimum mount height can be relaxed. **(revisit)**
- **R33**: The Carry Bin is split into two compartments, one on each side of the Lift column. The Arm drops objects into them with the Gripper tilted at least 30° down.
- **R16**: The Arm can drop objects into the front two-thirds of both Carry Bin compartments, with the Lift at 0.20–0.65 m. With the bins as currently laid out, the Arm **cannot pick objects back out** at any Lift height (see [arm reachability](research/arm-reachability.md)). Either redesign the bins (move them outboard or forward, or lower the walls) or accept bins the Arm can only drop into. **(revisit)**
- **R17**: Zarvis is at most about 1.4 m tall with the Lift fully raised.

## Stability and safety

- **R18**: Zarvis weighs at most 30 kg, with the battery mounted at the bottom of the Base.
- **R19**: Zarvis must not tip in any direction when all of these happen together:
  - the Lift is fully raised
  - the Arm is fully extended, holding 0.5 kg
  - the Base brakes from its speed limit

  The design keeps a 1.5× safety margin on this case. If it can't, Zarvis slows down automatically when the Lift is up.
- **R20**: The Base speed limit is 0.5 m/s near people. The Arm and Lift have a reduced-speed mode. **(revisit)**
- **R21**: A physical E-stop stops Zarvis in two stages. First it commands a controlled stop, then a relay cuts power to the Base and Arm motors about 0.5 s later. The computer stays powered.
- **R31**: Zarvis drives between places with the Arm folded into the Stowed Pose.

## Power

- **R22**: Zarvis runs at least 2 h of mixed use on one charge, which works out to about 500 Wh.
- **R23**: The battery is a 10S Li-ion pack (36 V nominal) with a battery management system. It supplies:
  - the Base motors, directly
  - the Arm, through a regulated 24 V converter rated for 400 W or more
  - the computer and sensors, through a separate converter
- **R24**: Zarvis is plugged in by hand to charge. The back of the Base keeps room for docking contacts.

## Compute and software

- **R25**: The onboard computer is a Jetson Xavier NX, to be upgraded to an Orin later. Training happens on a separate machine. Its memory size is not yet confirmed; if it is 8 GB, Policies run on another machine from day one. **(revisit)**
- **R26**: The software runs ROS 2 Humble in Docker on JetPack 5. After v1, Nav2 drives the Base and learned Policies drive the Arm. If the NX runs out of capacity, Policies run on another machine over Wi-Fi.

## Sensing

- **R27**: The sensors are:
  - a 2D lidar, mounted low on the Base (already owned)
  - an IMU
  - an RGB-D head camera that rides on the Lift carriage, tilted down at a fixed angle
  - a wrist camera on the Arm

## Interaction

- **R34**: During development, Zarvis takes commands from a gamepad or a terminal. After v1, it also serves a simple web UI where you pick a Fetch or start a Tidy-up from lists of named places and objects. The UI moved after v1 because v1 has no autonomy. Voice control is deferred.

## Teleop

- **R28**: A Demonstration covers the whole robot. One Operator holds the Leader Arm in one hand and a gamepad in the other; the gamepad drives the Base and the Lift.
- **R35**: In v1 the Operator is in the same room as Zarvis and watches it directly. Remote Teleop through the cameras comes later.
- **R36**: Each Demonstration records:
  - Head Camera images, colour and depth
  - Wrist Camera images
  - Arm and Gripper joint positions
  - Lift height
  - Base velocity commands and odometry
  - lidar scans
  - the Leader Arm and gamepad commands, recorded as the actions

  Joint data and actions are recorded at 30 Hz or faster.
- **R40**: The Leader Arm is not mounted on Zarvis. Teleop commands reach Zarvis wirelessly. Zarvis records actions and observations itself, with its own timestamps, so wireless lag doesn't misalign a Demonstration. How the Operator keeps up with Zarvis between rooms is still to be worked out. **(revisit)**
- **R37**: Demonstrations are saved in the LeRobot dataset format to an onboard SSD, then copied to the training machine.

## v1 acceptance test

- **R38**: v1 passes when one Operator, using only Teleop:
  1. tidies 10 scattered items, on both tile and rug, into the Carry Bin and then puts them away. If picking from the bin turns out not to work (R16), "putting them away" means emptying the bin by hand or tipping it.
  2. fetches one object, grasped top-down from the counter, to another room
  3. gets every run saved as a valid Demonstration that LeRobot can load and replay
  4. triggers the E-stop in the middle of a task, and Zarvis stops safely
