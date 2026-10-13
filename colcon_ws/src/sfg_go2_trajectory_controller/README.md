# Go2 timed trajectory controller

`trajectory_controller` reads the Go2 pose from TF and publishes bounded
body-frame velocity commands (`TwistStamped`) to the locomotion controller's
`cmd_vel` interface. It provides the `follow_trajectory` action by default and
tracks the timed waypoints in each goal's frame. Segment velocities are used as
feed-forward commands, with the configured PID gains correcting position and
heading tracking error. Goals must use `target_frame` and have finite poses
with strictly increasing, non-negative waypoint times.

Start the Go2 locomotion controller first, then launch this package:

```bash
ros2 launch sfg_go2_trajectory_controller trajectory_controller.launch.py
```

The controller is also registered as the
`sfg_go2_trajectory_controller::TrajectoryController` component and can be
loaded into a component container.

Trajectory coordinates are expressed in `target_frame` (meters); yaw is in
radians. The action succeeds after the final waypoint time once the robot is
within `position_tolerance` and `yaw_tolerance` of the final pose. A new goal
supersedes the current trajectory, and canceling a goal stops it and holds the
current pose.

When no action goal is active, the controller retains its single-pose hold
behavior. With `target_from_start: true` (the safe default), the first valid TF
pose becomes the target. Set it to `false` to use `target_x`, `target_y`, and
`target_yaw` as a fixed target.

PID gains, command limits, update rate, action name, completion tolerances, and
pose timeout are configurable in `config/trajectory_controller.yaml`. If no
pose has arrived or tracking becomes stale, the controller publishes a zero
command and resets its integrators. The Vicon bridge must publish the
`target_frame -> base_frame` transform and its timestamp must be within
`pose_timeout`.
