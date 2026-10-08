# Mecanum Base driven by Unitree IM6014 joint actuators

The Base rolls on four mecanum wheels, and each wheel is driven by a Unitree IM6014. We chose mecanum over 3-wheel kiwi and 4-wheel X-drive omni because the builder prefers it. We chose it even though rollers on mecanum wheels lose more traction on rugs than omni wheels do. We chose the IM6014 over purpose-built hub motors because a discount makes it cheap for us.

## Consequences

- **Wheel bearings**: the IM6014 is a geared joint actuator with no published radial load rating. Each wheel will likely need its own bearings so the motor supplies torque but doesn't carry the robot's weight; this gets decided in the chassis design.
- **Floor contact**: mecanum only steers correctly when all four wheels press on the floor evenly, so the Base needs suspension (likely a centre-pivot rear axle).
- **Odometry**: mecanum rollers slip, worst on rugs, so the Base's position comes from wheel encoders fused with the IMU and lidar, not encoders alone.
- **Driver**: there is no ROS 2 driver for the IM6014, so we write an RS-485 driver from Unitree's protocol.
