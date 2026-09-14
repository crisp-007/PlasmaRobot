# Real Robot Full Dry-Run Result - 2026-07-25

## Goal

Verify the complete robot motion chain before calibration accuracy tuning:

1. Current robot pose to the reviewed pre-entry pose.
2. Enter along the reconstructed cavity entry normal.
3. Execute the complete reviewed in-cavity spray joint trajectory.

This commissioning run used 5% velocity and acceleration scaling, RealMan
collision stage 8, and plasma output remained disabled for the entire run.

## Result

The complete real-robot dry-run succeeded.

- Approach trajectory: 133 joint points, completed in about 67 seconds.
- Reviewed after-entry trajectory: 1479 joint points, completed in about 725
  seconds.
- MoveIt and `rm_control` both reported `SUCCEEDED` for both trajectories.
- Final controller status: `arm_current_status=0` (idle).
- Final joint error codes: `[0, 0, 0, 0, 0, 0]`.
- Final joint feedback, radians:
  `[-0.119620, -0.070638, -1.897286, 0.635005, 1.791941, 2.591238]`.
- Plasma output was never enabled.

Planning selected a complete collision-checked solution across eight nearby-IK
attempts:

- Approach total joint travel: 199.396678 degrees.
- Approach maximum absolute joint step: 1.122997 degrees on joint6.
- Spray maximum absolute joint step: 1.222507 degrees on joint4.
- Cartesian after-entry interpolation followed 100% of 298 requested poses and
  produced 1479 reviewed joint points.

## Commissioning Configuration

The hand-eye and TCP files are loaded but their formal acceptance flags are not
complete. The robot-motion test therefore used the explicit
`allow_unvalidated_dry_run:=true` commissioning switch. This switch defaults to
`false`; it does not modify calibration files or claim that calibration passed.
The entry planner has no plasma-output command and this override applies only to
dry robot motion.

Launch command:

```bash
ros2 launch plasma_path_executor automatic_entry_motion.launch.py \
  motion_enabled:=true \
  allow_unvalidated_dry_run:=true \
  velocity_scaling:=0.05 \
  acceleration_scaling:=0.05 \
  max_plan_age_sec:=600.0 \
  rviz:=true \
  start_rm_control:=true
```

Execution services:

```bash
ros2 service call /plasma_entry_motion_planner/plan_to_pre_entry \
  std_srvs/srv/Trigger "{}"

ros2 service call /plasma_entry_motion_planner/execute_to_pre_entry \
  std_srvs/srv/Trigger "{}"

ros2 service call /plasma_entry_motion_planner/execute_reviewed_dry_run \
  std_srvs/srv/Trigger "{}"
```

## Execution-Layer Fixes Used By This Run

- `rm_control` waits for real joint feedback after a blocking MoveJ is
  accepted instead of treating command acceptance as arrival.
- An action cancellation now clears pending waypoint and settle state before
  returning, so a canceled MoveIt action cannot continue dispatching points.
- MoveIt's allowed execution-duration scaling is 300, avoiding the previous
  false timeout at about 535 seconds during a valid 5% trajectory.
- The reviewed-plan age is 600 seconds, allowing the approach to finish before
  the separately confirmed in-cavity execution starts.

## Important Operating Constraints

- Keep only one `rm_driver`; the GUI-managed driver was used for this run.
- Do not use `/rm_driver/move_stop_cmd` as a normal pause on this generation of
  controller. It enters `STOP(9)` and requires a controller power cycle.
- `allow_unvalidated_dry_run` must remain false for normal operation. Remove the
  commissioning override after hand-eye and TCP validation are accepted.
- Accuracy, cable routing, joint6 winding, and clearance margins still require
  final validation. This result proves end-to-end motion flow, not clinical or
  production accuracy.

## Next Work

1. Validate the hand-eye matrix and update its acceptance status.
2. Validate the physical TCP and side outlet geometry against measurements.
3. Re-run collision review with the accepted calibration and the commissioning
   override disabled.
4. Tune accuracy and motion quality without changing the now-proven workflow.

## Follow-up Speed Setting

After the successful 5% run, the automatic-entry launch defaults were changed
to 10% velocity, 10% acceleration scaling, and 10% blocking MoveJ speed. This
is approximately twice the commissioned motion speed. Geometry, collision
stage 8, dry-run behavior, and the default-off calibration override were not
changed. The 10% setting still requires a new reviewed plan before execution.
