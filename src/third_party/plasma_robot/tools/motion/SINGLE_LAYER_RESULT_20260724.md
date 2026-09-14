# Single-layer dry-run result (2026-07-24)

> Historical result: this run used the former 302 mm TCP / 310 mm rounded-tip
> model. The active 2026-07-30 geometry is 310/318 mm, so this path must not be
> reused for execution.

## Test configuration

```text
robot: ECO65-BI generation 3
path_id: single_layer_commission_1784878211186471095
speed_percent: 5
angular_step_deg: 5
total_points: 148
sequence: 0 -> +180 -> 0 -> -180 -> 0
dry_run: true
```

The path was generated from the live `baselink -> Link6` pose after the
operator manually placed the robot at the test entry pose. The nominal
`3_4 + 200_1x10` side-outlet TCP was held fixed in all generated targets.

## Controller result

The executor accepted the explicit start request and completed the complete
single-layer sequence:

```text
state: 4 (STATE_COMPLETED)
current_index: 147
total_points: 148
layer_index: 0
motion_phase: 3
motion_enabled: true
dry_run: true
path_execution_permitted: true
driver_ready: true
entry_pose_valid: true
command_in_flight: false
desired_plasma_enabled: false
plasma_output_enabled: false
message: after-entry spray path dry-run completed
```

This confirms that the real driver, current-pose path preparation, four-stage
layer contract, MoveL command/result loop, reached-pose verification, and
completion state work end to end. Real plasma output remained disabled.

## Corrected TCP regression run

The complete single-layer motion was repeated after installing the corrected
physical tool dimensions:

```text
test time: 2026-07-24 18:16 Asia/Shanghai
path_id: layer_commission_1x_1784888160533063293
Link6 to side-outlet motion TCP: [0.002, 0.0, 0.302] m
Link6 to rounded nozzle end: 0.310 m axial
motion TCP to rounded nozzle end: [+0.008, 0.0, -0.002] m
speed_percent: 5
total_points: 148
final_state: 4 (STATE_COMPLETED)
final_index: 147
entry_pose_valid: true
plasma_output_enabled: false
```

Every MoveL command reported success and passed the reached-flange-pose check.
The phase sequence and requested plasma states were also observed as expected:

```text
0 -> +180 deg: desired plasma on
+180 -> 0 deg: desired plasma off
0 -> -180 deg: desired plasma on
-180 -> 0 deg: desired plasma off
```

The physical plasma output remained off for the entire regression run.

## Remaining physical observation

`STATE_COMPLETED` proves that all commanded flange targets were reached. It
does not by itself prove the nominal tool geometry. Record the operator's
observation of side-outlet TCP drift, cable behavior, joint-limit margin, and
clearance before proceeding to a multi-layer axial-transition test.
