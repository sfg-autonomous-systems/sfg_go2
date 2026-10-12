# Go2 Vicon PID controller

`trajectory_controller` reads the Go2 pose from TF and publishes bounded
body-frame velocity commands (`TwistStamped`) to the locomotion controller's
`cmd_vel` interface. By default, it looks up `map -> sfg_go2_01/base_link`,
published by the Vicon bridge.

Start the Go2 locomotion controller first, then launch this package:

```bash
ros2 launch sfg_go2_trajectory_controller trajectory_controller.launch.py
```

The target coordinates are expressed in `target_frame` (meters), and
`target_yaw` is in radians. With `target_from_start: true` (the safe default),
the first valid TF pose becomes the target, so the robot holds its starting
position and heading. Set it to `false` to use `target_x`, `target_y`, and
`target_yaw` as a fixed target.

PID gains, command limits, update rate, and pose timeout are configurable in
`config/trajectory_controller.yaml`. If no pose has arrived or tracking becomes
stale, the controller publishes a zero command and resets its integrators. The
Vicon bridge must publish the `target_frame -> base_frame` transform and its
timestamp must be within `pose_timeout`.
