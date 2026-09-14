# PlasmaRobot mechanical-arm tools

This directory groups project-owned mechanical-arm utilities without mixing
them into the RealMan vendor packages.

## Layout

```text
tools/
├── eye_hand/                  # Hand-eye calibration data package
├── motion/
│   └── plasma_path_executor/  # Validated base-frame path executor
└── tf/
    └── plasma_path_transform/ # Camera-path to robot-base C++ transform node
```

Shared ROS messages remain in the sibling package
`../plasma_robot_interfaces/`. The end-effector model is kept in the sibling
description package `../plasma_tool_description/`, not under calibration or TF.

## Commissioning baseline

The first successful real-robot end-to-end dry run is preserved in
`motion/FIRST_END_TO_END_DRY_RUN_BASELINE_20260725.md`. It covers the complete
motion from the current pose to pre-entry, normal entry, and the reviewed
in-cavity trajectory with plasma disabled. Treat it as a workflow regression
baseline, not as evidence that hand-eye or TCP accuracy has passed acceptance.

## Current calibration

- Camera: Intel RealSense L515, serial `f1423110`.
- Active source file: `eye_hand/config/camera_to_gripper.yaml`.
- Matrix meaning: `^Link6 T_camera` (`gripper_T_camera`).
- Translation units: meters.
- Status: `user_approved_provisional_use`; strict offline validation is false.

The transform node loads this matrix by default. It can publish preview
coordinates. It also loads the provisional `3_4 + 200_1x10` motion TCP,
safety point, and side-outlet transforms from
`../plasma_tool_description/config/tcp_3_4_200_1x10.yaml`. Both calibrations
remain unvalidated, so it must report `execution_permitted=false`.

## Build and test

```bash
source /opt/ros/galactic/setup.bash
colcon build --packages-select plasma_eye_hand plasma_tool_description plasma_path_transform plasma_path_executor
colcon test --packages-select plasma_eye_hand plasma_path_transform plasma_path_executor
colcon test-result --test-result-base build/plasma_path_transform --verbose
```

## Replacing the hand-eye matrix

1. Preserve the original measured YAML and its quality report.
2. Set `frame_convention.matrix`, `meaning`, and `units` explicitly.
3. Keep a provisional deployment status until strict validation passes.
4. Replace `eye_hand/config/camera_to_gripper.yaml`, rebuild
   `plasma_eye_hand`, and call `/plasma_path_transform/reload_calibration`.
5. Check known points before considering any downstream execution.

Do not store the end-effector URDF in `eye_hand/` or `tf/`. It belongs in a
separate description package after the physical plasma tool is confirmed.
