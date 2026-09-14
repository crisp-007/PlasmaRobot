# Two-layer dry-run result (2026-07-24)

## Test configuration

```text
robot: ECO65-BI generation 3
path_id: layer_commission_2x_1784879468990848141
speed_percent: 5
angular_step_deg: 5
layer_count: 2
layer_step_m: 0.010
total_points: 297
dry_run: true
```

## Completed motion

```text
layer 0: 0 -> +180 -> 0 -> -180 -> 0
transition: tool-local +X toward the bottom by 10 mm
layer 1: 0 -> +180 -> 0 -> -180 -> 0
```

Final executor status:

```text
state: 4 (STATE_COMPLETED)
current_index: 296
total_points: 297
layer_index: 1
motion_phase: 3
motion_enabled: true
dry_run: true
path_execution_permitted: true
driver_ready: true
command_in_flight: false
desired_plasma_enabled: false
plasma_output_enabled: false
message: after-entry spray path dry-run completed
```

The final `entry_pose_valid: false` is expected because the robot finishes at
the second layer, 10 mm away from the original first-layer entry pose.

The operator confirmed that the side-outlet motion, positive and negative
sweeps, layer transition, joint behavior, and cable clearance were normal.
This validates the real driver and executor for fixed-TCP layer rotation plus
one axial transition. Real plasma output remained disabled.

## Next stage

Replace the commissioning-generated circles with the GUI's actual sliced path.
First run the complete camera-to-base transform and executor structural check
with `motion_enabled=false`; do not approve real motion solely from this
commissioning result.
