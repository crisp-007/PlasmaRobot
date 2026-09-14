# L515 depth-frame path conversion

Date: 2026-07-24

## Background

The L515 initially publishes a color-textured point cloud in
`camera_color_optical_frame`. Its RGB endpoint can stop after startup while the
depth stream remains healthy. The operational fallback publishes untextured
XYZ points in `camera_depth_optical_frame`.

The hand-eye matrix in `tools/eye_hand/config/camera_to_gripper.yaml` was
calibrated with the color camera and therefore remains tied to
`camera_color_optical_frame`. Relabelling depth points as color points would
introduce an incorrect rigid offset.

## Implementation

`plasma_path_transform` now accepts a camera-frame input that differs from the
calibrated camera frame only when TF can resolve the relationship. For the XYZ
fallback it applies:

```text
T_base_input = T_base_gripper(capture)
             * T_gripper_color(hand-eye)
             * T_color_depth(RealSense static TF)
```

The node rejects the path if the static TF is absent or invalid. It never
guesses a camera transform or rewrites the input frame ID.

## On-machine verification

- RealSense static TF resolved between `camera_color_optical_frame` and
  `camera_depth_optical_frame`.
- Package `plasma_path_transform` built successfully.
- The captured GUI path `重建1路径` was accepted and transformed:
  `446 poses`, output frame `baselink`.
- Output remained `execution_permitted=false` because the hand-eye and TCP
  YAML files are still marked unvalidated.
- The GUI executor remained `motion_enabled=false`, speed 5%, so no MoveL or
  plasma command was sent.
