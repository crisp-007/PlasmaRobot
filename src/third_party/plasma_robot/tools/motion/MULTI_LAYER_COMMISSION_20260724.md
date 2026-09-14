# Two-layer robot commissioning

This test follows the successful single-layer dry run. It verifies one axial
layer transition and a second complete spray sweep. Real plasma output remains
disabled.

## Motion contract

```text
layer 0: 0 -> +180 -> 0 -> -180 -> 0
transition: plasma off, TCP +X by 10 mm toward the bottom
layer 1: 0 -> +180 -> 0 -> -180 -> 0
```

At a 5-degree angular step the path contains 297 poses. The operator manually
places the robot at the first entry pose; the software does not perform the
initial-to-entry motion.

## Before launch

- Keep the existing `rm_driver` running.
- Stop the previous `single_layer_commission.launch.py` with `Ctrl+C`.
- Confirm `/plasma_path_executor` no longer appears before starting this test.
- Plasma power and gas remain off; speed is 5% or lower.
- Confirm 10 mm of free space in tool-local `+X`, toward the cavity bottom.

## Start the two-layer nodes

```bash
cd /home/larusxu/CodeSpace/PlasmaRobot
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 launch plasma_path_executor multi_layer_commission.launch.py \
  commissioning_enabled:=true \
  motion_enabled:=true \
  speed_percent:=5 \
  layer_count:=2 \
  layer_step_m:=0.010
```

Starting the launch does not move the robot.

## Monitor status

```bash
ros2 topic echo /plasma/path_executor/status \
  --qos-reliability reliable \
  --qos-durability transient_local
```

## Prepare

```bash
ros2 service call /plasma_multi_layer_commission/prepare \
  std_srvs/srv/Trigger "{}"
```

Preparation does not move the robot. Before starting, require:

```text
state: 1
total_points: 297
motion_enabled: true
path_execution_permitted: true
driver_ready: true
entry_pose_valid: true
plasma_output_enabled: false
```

## Start real robot dry-run motion

```bash
ros2 service call /plasma_path_executor/start \
  plasma_robot_interfaces/srv/StartSprayPath \
  "{path_id: '', dry_run: true}"
```

Successful completion reports `state: 4`, `current_index: 296`, and
`plasma_output_enabled: false`.

## Stop

```bash
ros2 service call /plasma_path_executor/stop std_srvs/srv/Trigger "{}"
```

Use the physical emergency stop for an immediate collision risk.
