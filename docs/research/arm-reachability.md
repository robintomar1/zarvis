# Arm reachability vs Lift height (2026-10-07)

This study asks what mounting heights let the B601-DM Arm, sitting on the Base, reach both the floor and a 90 cm counter.

## Method

- **Arm model**: the official URDF, `Seeed-Projects/reBot-DevArm` at commit `171ed82`, file `Rebot_Arm_description/DM/urdf/ReBot_Arm_DM.urdf`.
- **Collisions**: checked against the URDF's collision meshes, with 1 cm clearance.
- **Joint margin**: every joint stays at least 5° away from its limit.
- **Search**: IK solved from 43 seeds per target.

The terms used below:

| Term | Meaning |
|---|---|
| H | Height of the arm's mounting plate above the floor. The shoulder joint (J2) sits 14 cm higher. |
| s | How far the mounting plate sits back from the Base's front edge. |
| TCP | The grasp point. It sits 2 cm behind the fingertips. |
| Comfortable | The arm is extended no more than 70% of its reach, which is the condition its 1.5 kg payload rating assumes. |

## Key findings

- **The shoulder can't point below horizontal.** In the URDF, joint J2 is limited to [-180°, 0°], so the upper arm can't reach downward. This limits floor pickup to mounting heights of about 0.35 m or lower.
  - The real mechanical stop is not yet confirmed with Seeed.
- **No single fixed height works.** Reaching the floor 5–30 cm out and the counter 25 cm deep from one height is only possible around H ≈ 0.32 m. Even there the arm is beyond 70% extension, and the wrist clears the counter edge by only 1.2 cm.
- **Comfortable range of H needed**, with s between 0.08 and 0.12 m:

  | Target | H needed |
  |---|---|
  | Floor, top-down grasp, 5–30 cm out | ≤ 0.20–0.25 m |
  | Counter, 30 cm deep, side grasp | 0.45–0.60 m. Side reach gets worse again above about 0.65 m. |
  | Counter, 30 cm deep, top-down grasp | ≥ 0.80–0.85 m |

- **Lift stroke**:
  - about **30–35 cm** if the counter is reached with side grasps
  - about **60–70 cm** if it must also be reached with top-down grasps
- **Carry Bin**, tested with an 18 × 30 cm bin on a 25–30 cm high deck at the lowest Lift position:
  - With nothing in the way, the Arm can drop objects into the bin with the gripper tilted at least 30° down.
  - A Lift column directly behind the Arm blocks the centre of the bin. Points beside the column, about 20 cm to either side, stay reachable.

## Caveats

- **Base shape**: modelled as an infinitely wide box.
- **Not modelled**: the Arm colliding with itself, and joint torques and dynamics.
- **Lateral positions**: side grasps were checked only straight ahead.
- **Counter clearance**: the TCP target was 5 cm above the counter. Aim for 6 cm or more.


## Re-run for the chosen layout

This re-run models the layout Zarvis will actually use:

- **Lift**: on the Base's front face.
- **Arm setback**: s = 0 m, so the arm axis sits at the Base's front edge and the mounting plate overhangs it by 7 cm.
- **Column**: 10 × 10 cm, directly behind the Arm.
- **Deck**: 25–30 cm high.
- **Carry Bin**: two 15 × 25 cm compartments beside the column, with 12 cm walls.

### Results

**Floor, top-down, at H = 0.20–0.25 m**: reach covers 5–40 cm in front of the Base and ±20 cm to the sides, mostly within 70% extension. Moving the mount further forward (s = −0.05) blocks targets close in front of the Base, so **s ≈ 0** is better.

**Counter, top-down**: the overhanging mount hits the counter slab if the Base is flush against the counter. Two working options:

| Base position | Lift height (H) | Reach into counter |
|---|---|---|
| Stops about 9 cm short of the counter edge | 0.85–0.90 m | 5–35 cm, within 70% extension |
| Flush, with the mount overhanging the countertop | about 0.92 m | 7.5–35 cm |

**Carry Bin**:
- **Dropping objects in** works with the Lift at 0.20–0.65 m (0.20–0.70 m with a 30 cm deck), but only into the front two-thirds of each compartment. The column blocks the rear third.
- **Picking objects out fails at every Lift height.** The column blocks the arm, and the bin walls block the 18 cm-wide gripper.
- Possible fixes: move the compartments outboard or forward of the column, use lower walls, or accept drop-only bins.

**Stowed Pose**: a valid pose exists. The arm turns about 90° to one side and folds upright beside the column, at roughly q ≈ [−98, −43, −8, 25, −23, −81]°.
- It stays within the 45 cm width.
- It sits over one bin compartment.
- Its top is about 0.59 m above the floor when the mount is at H = 0.20 m.

**Shoulder (J2) stop**: the vendor parts list calls `01_Upper_Arm_Limit.step` the "Upper Arm Horizontal Limit Block". That matches the URDF's limit at horizontal, but the real stop is still unconfirmed with Seeed.

## Scripts

The scripts are in [`reach/`](reach/):

| Script | Purpose |
|---|---|
| `sweep.py` | General sweep over mounting heights |
| `chosen.py` | The re-run for the chosen layout |
| `chosen_sum.py` | Summarises the chosen-layout results |
| `pickdeep.py` | Deeper search for picking out of the bin |
| `comfort.py` | Checks targets against the 70% extension limit |
| `bin*.py` | Earlier Carry Bin checks |

Running them needs:
- `pinocchio`, `coal` and `scipy`
- the official URDF from `Seeed-Projects/reBot-DevArm` at commit `171ed82`

Run them from inside `reach/`, because `chosen.py` loads `sweep.py` from the current directory:

```
python -I chosen.py <URDF> <s> floor|counter|bins|stow
```
