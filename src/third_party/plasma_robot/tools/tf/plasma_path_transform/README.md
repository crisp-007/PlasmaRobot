# Plasma path transform

This ROS 2 C++ node converts the Plasma GUI nozzle pose sequence from the
capture camera frame into `baselink`.

## Scope

This package does not plan or command motion from the robot's initial pose to
the cavity opening or spray entry. The first output point is the first spray
pose after entry. Every output point tells a downstream controller:

- the desired motion TCP pose and, when tool calibration is loaded, flange pose;
- the physical nozzle-end clearance point and side-outlet center pose;
- sequence and layer indices;
- whether the motion is processing, returning, or changing layers;
- whether plasma is enabled at that point.

The node transforms and validates this contract; it does not publish MoveJ or
MoveL commands.

The eye-in-hand transform chain is:

```text
T_base_tcp = T_base_gripper(capture) * T_gripper_calibrated_camera
             * T_calibrated_camera_input_camera * T_input_camera_tcp
```

`T_gripper_camera` is loaded from the YAML key `T_camera_to_gripper`. The name
is retained for compatibility with the existing calibration workflow; its
actual matrix semantics are `^gripper T_camera`.

The hand-eye matrix was calibrated in `camera_color_optical_frame`. When the
L515 publishes an untextured XYZ cloud in `camera_depth_optical_frame`, the
node resolves the fixed depth-to-color transform from RealSense `/tf_static`.
It rejects the path if that transform is unavailable; frame IDs are never
relabelled or treated as interchangeable.

The input topic is `/plasma/planned_spray_path/camera`; the transformed path is
published on `/plasma/planned_spray_path/base`. The node also provides:

```text
/plasma_path_transform/transform_path
/plasma_path_transform/reload_calibration
/plasma/path_transform/status
```

The input must carry either the capture-time six-joint state or a direct
`^base T_gripper` pose. Current robot state is deliberately not substituted for
capture state because path planning can finish long after the point cloud was
captured.

Without a hand-eye matrix the node rejects camera-frame conversion. A rigid but unvalidated
matrix may produce preview output when `publish_unvalidated_preview=true`, but
the result is marked `execution_permitted=false`. Execution permission also
requires a loaded and validated `T_gripper_to_tcp` calibration plus valid
`T_tcp_to_nozzle_tip` and `T_tcp_to_spray_outlet` process geometry.

The default launch loads the installed L515 matrix from:

```text
plasma_eye_hand/config/camera_to_gripper.yaml
```

It also loads the current nominal tool TCP from:

```text
plasma_tool_description/config/tcp_3_4_200_1x10.yaml
```

The selected trial tool is `3_4 + 200_1x10`. Its motion TCP is the physical
side-outlet center, at `[0.002, 0, 0.310] m` in `Link6`. The rounded end is
318 mm from Link6; the outlet near edge starts 3 mm behind it and its 10 mm
axial length places the center 8 mm behind the end. In the motion TCP frame,
the rounded nozzle end is at `[0.008, 0, -0.002] m`, and the
side-outlet transform is identity. Motion TCP `+X` points deeper into the
cavity and `+Z` points through the side outlet. This geometry is provisional and intentionally
has `validated: false`, so flange and process geometry are available for
preview while `execution_permitted` remains false.
The controller currently stores `Arm_Tip` as a zero tool coincident with the
flange. Its axes match ROS `Link6`, but it does not contain the process TCP
offset; this node therefore must apply `T_gripper_to_tcp` when producing
flange poses.

The `3_4` and `6_8` adapters share the same axial positions for an equal rod
length, but their shaft radii and therefore side-outlet TCP offsets differ.

For cavity entry, the physical constraint is the rounded nozzle end outside the
detected mouth by `entry_tip_standoff_m` (30 mm by default). The node derives
the required motion-TCP entry pose as
`T_source_safe_tip * inverse(T_tcp_to_nozzle_tip)`. Spray waypoints continue to
use the side-outlet motion TCP. This gives the visual mouth estimate an explicit
axial safety margin before spraying begins.

The optional `cavity_axis_outward_compensation_m` parameter translates the
complete path along the detected cavity outward axis after hand-eye conversion.
It moves the mouth marker, pre-entry, safe entry, every spray/safety/surface
point, and therefore the reviewed retreat by exactly the same distance. It does
not alter the hand-eye matrix, TCP geometry, entry standoff, or layer spacing.
Positive values move the complete path outward. Any non-zero value is identified
in `calibration_id` and keeps normal execution permission false until the
measured compensation is formally validated; the explicit low-speed
commissioning override is still required.

The independent `spray_axis_outward_compensation_m` parameter translates only
the spray poses, their surface targets, rounded-tip safety points, side-outlet
poses, and derived Link6 poses. It deliberately leaves the detected mouth,
safe entry, and pre-entry unchanged. Use it only when entrance registration is
accepted but the measured safe-entry-to-layer insertion is too deep. The first
compensated spray layer must remain inside the safe-entry plane; otherwise the
path is rejected. Like complete-path compensation, any non-zero value is
recorded in `calibration_id` and forces normal execution permission off.

That matrix currently has `deployment.status: user_approved_provisional_use`.
It is intentionally loaded as unvalidated: preview is available, but execution
permission remains false. Pass `handeye_yaml:=...` only to test another
explicit calibration file.

An entry-reach training candidate can be applied after the live camera-frame TF without
replacing the hand-eye YAML:

```bash
ros2 launch plasma_path_transform plasma_path_transform.launch.py \
  source_refinement_yaml:=/absolute/path/to/entry_reach_refinement_candidate.yaml
```

The YAML must contain `source_frame` and `T_source_correction`, with the convention
`T_base_source_corrected = T_base_source_current * T_source_correction`. When this optional
parameter is active, the node always forces `execution_permitted=false` and identifies the
training refinement in `calibration_id`, even if other calibration files are approved. It is
only for independent preview and blind reach validation.

The loader recognizes these approval fields, in precedence order:

```yaml
validated: true
# or: status: validated
# or: quality: {status: approved}
# or: deployment: {quality_accepted: true}
# or: deployment: {status: production_ready}
```

Set an approval field only after the measured transform has passed the real
calibration quality checks. Loading a rigid matrix is not, by itself,
validation.

Launch with the current default hand-eye matrix:

```bash
ros2 launch plasma_path_transform plasma_path_transform.launch.py
```

Launch after a replacement calibration:

```bash
ros2 launch plasma_path_transform plasma_path_transform.launch.py \
  handeye_yaml:=/absolute/path/to/camera_to_gripper.yaml \
  tcp_yaml:=/absolute/path/to/tool_tcp.yaml
```

Files in `config/*.example.yaml` document the schema only. Their identity
matrices must never be used for real motion.
