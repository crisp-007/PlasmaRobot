# Single-layer robot commissioning

This procedure verifies one complete plasma spray layer after the operator has
manually placed the robot at the test entry pose. It does not move the robot
from its initial pose to the entry. Real plasma output remains disabled.

## Required state

- Plasma power and gas are off.
- Robot speed is 5% or lower.
- The emergency stop is immediately accessible.
- The tool has clearance for the full `0 -> +180 -> 0 -> -180 -> 0` motion.
- No previous `rm_driver`, `plasma_path_executor`, or commissioning node is
  running.

## Terminal 1: robot driver

```bash
cd /home/larusxu/CodeSpace/PlasmaRobot
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 launch rm_driver rm_eco65_driver.launch.py
```

Leave this terminal running. Never start a second driver while it is active.

## Verify feedback

In another terminal:

```bash
cd /home/larusxu/CodeSpace/PlasmaRobot
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 node list
ros2 topic echo /rm_driver/udp_arm_position
```

After several changing pose messages appear, press `Ctrl+C` only in this
feedback terminal. The driver terminal remains running.

## Manual entry positioning

Use the teach pendant at no more than 5% speed to place the robot at the test
entry pose. Confirm full wrist rotation clearance, cable clearance, and joint
margin. The software does not perform this positioning.

## Terminal 2: commissioning and executor

```bash
cd /home/larusxu/CodeSpace/PlasmaRobot
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 launch plasma_path_executor single_layer_commission.launch.py \
  commissioning_enabled:=true \
  motion_enabled:=true \
  speed_percent:=5
```

Starting this launch does not move the robot.

## Terminal 3: status monitor

```bash
cd /home/larusxu/CodeSpace/PlasmaRobot
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 topic echo /plasma/path_executor/status \
  --qos-reliability reliable \
  --qos-durability transient_local
```

Leave this monitor running.

## Terminal 4: prepare the current-pose path

```bash
cd /home/larusxu/CodeSpace/PlasmaRobot
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 service call /plasma_single_layer_commission/prepare \
  std_srvs/srv/Trigger "{}"
```

Preparation publishes 148 poses and does not move the robot. Before starting,
the status monitor must report:

```text
state: 1
total_points: 148
motion_enabled: true
path_execution_permitted: true
driver_ready: true
entry_pose_valid: true
plasma_output_enabled: false
message: path structurally valid; waiting for explicit start
```

Do not start if any required boolean is false.

## Start the dry run

The next command starts real robot motion:

```bash
ros2 service call /plasma_path_executor/start \
  plasma_robot_interfaces/srv/StartSprayPath \
  "{path_id: '', dry_run: true}"
```

Keep a hand at the emergency stop. The status may show
`desired_plasma_enabled: true` during simulated spray phases, but
`plasma_output_enabled` must remain `false` at all times.

Successful completion reports:

```text
state: 4
current_index: 147
plasma_output_enabled: false
message: after-entry spray path dry-run completed
```

## Stop

For an unexpected but non-emergency motion:

```bash
ros2 service call /plasma_path_executor/stop std_srvs/srv/Trigger "{}"
```

Use the physical emergency stop immediately for a collision risk or hazardous
motion. Preserve terminal output after any failure before shutting nodes down.

After a normal completion, press `Ctrl+C` in Terminal 3, then Terminal 2, then
Terminal 1.
