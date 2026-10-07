# Zarvis

Zarvis is a personal mobile manipulator that tidies and fetches things around one specific home. This glossary pins down the words used for its parts, its jobs and how it is taught.

## Language

### The robot

**Zarvis**:
The whole robot: Base, Arm and everything mounted on them.
_Avoid_: the bot, the platform

**Base**:
The holonomic wheeled body that moves Zarvis around the Home and carries everything else.
_Avoid_: chassis, platform, drivetrain (the drivetrain is only part of the Base)

**Lift**:
The vertical axis on the Base that raises and lowers the Arm.
_Avoid_: mast, elevator, column, torso, telescope

**Arm**:
The 6-joint manipulator that rides on the Lift, not counting its Gripper.
_Avoid_: manipulator, robot arm

**Gripper**:
The end effector on the Arm that grasps objects.
_Avoid_: hand, end effector, claw

**Stowed Pose**:
The folded, low position the Arm holds while the Base drives.
_Avoid_: home pose, rest position, park

**Carry Bin**:
A container on the Base that holds objects while Zarvis drives, so carrying doesn't depend on the Arm.
_Avoid_: basket, tray, cargo

**Head Camera**:
The depth camera that rides on the Lift and looks down at the Arm's work area.
_Avoid_: main camera, RGB-D (that names the sensor type, not its role)

**Wrist Camera**:
The camera on the Arm next to the Gripper.
_Avoid_: hand camera, gripper camera

**E-stop**:
A physical switch that stops Zarvis and then cuts power to its actuators. The power cut works even if the software has failed.
_Avoid_: kill switch, emergency button

### Where it works

**Home**:
The one household Zarvis is built for. Its floors are tile and thin rugs, all on a single level.
_Avoid_: environment, house, deployment site

**Reach Envelope**:
The region the Gripper must be able to reach, from the floor up to the top of the kitchen counter.
_Avoid_: workspace (ambiguous with the arm's kinematic workspace)

### What it does

**Tidy-up**:
A job in which Zarvis picks up Clutter from the floor and puts it where it belongs.
_Avoid_: cleaning, decluttering

**Clutter**:
A loose, light object lying out of place on the floor.
_Avoid_: mess, debris, items

**Fetch**:
A job in which Zarvis carries a requested object from one place in the Home to another.
_Avoid_: delivery, bring

### How it learns

**Teleop**:
The Operator controlling all of Zarvis directly, with the Arm following the Leader Arm.
_Avoid_: remote control, puppeteering

**Operator**:
The person controlling Zarvis during Teleop.
_Avoid_: user, pilot, teleoperator

**Leader Arm**:
A smaller arm that a person moves by hand; the Arm copies its motion during Teleop.
_Avoid_: master arm, teleop arm

**Demonstration**:
A recording of one Teleop run of a task, kept so a Policy can be trained on it.
_Avoid_: episode (only when talking about datasets), recording

**Policy**:
A learned model that controls Zarvis from what its sensors see, trained from Demonstrations.
_Avoid_: model, brain, controller
