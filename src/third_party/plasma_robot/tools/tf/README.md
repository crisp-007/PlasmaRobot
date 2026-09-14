# TF and path transform tools

This directory owns robot-coordinate utilities for PlasmaRobot.

## Packages

- `plasma_path_transform/`: converts the GUI spray path from the capture
  camera frame into `baselink` using capture-time robot state and hand-eye
  calibration.

## Calibration inputs

- Hand-eye: installed by the sibling ROS package `plasma_eye_hand` from
  `../eye_hand/config/camera_to_gripper.yaml`.
- Tool geometry: passed as `tcp_yaml`; it contains `T_gripper_to_tcp`,
  `T_tcp_to_nozzle_tip`, and `T_tcp_to_spray_outlet`. The current real plasma
  tool has not yet been approved for execution.

The current L515 hand-eye matrix is provisional. The node may publish preview
coordinates, but `execution_permitted` must remain false until both hand-eye
and TCP calibrations are approved.
