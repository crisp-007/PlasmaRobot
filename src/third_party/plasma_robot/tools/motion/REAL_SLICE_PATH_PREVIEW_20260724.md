# Real slice path preview result (2026-07-24)

> 自动入口、MoveIt 预入口规划和 5 mm 分段进入已继续实现，见
> `AUTOMATIC_ENTRY_WORKFLOW_20260724.md`。

## Scope

This record covers the first complete path generated from the live GUI capture
after the camera stream was restored. No robot command or plasma output was
sent during this check.

## Result

- GUI path: `重建1路径`
- Source frame: `camera_depth_optical_frame`
- Calibration frame: `camera_color_optical_frame`
- Output frame: `baselink`
- Closed layers: 2
- Open layers skipped: 2
- Total poses: 297
- Angular step: 5 deg
- Sequence per layer: `0 -> +180 -> 0 -> -180 -> 0`
- Coordinate transform: successful
- Structural path validation: successful
- Execution permission: false

The transform node used the live camera TF
`camera_color_optical_frame <- camera_depth_optical_frame` before applying the
eye-hand matrix.

## First entry pose in `baselink`

The values below are inspection targets only. They must not be sent directly
until the non-contact on-site check and calibration acceptance are complete.

```text
motion TCP position (m): [-0.508256236, -0.080095355, 0.061397666]
Link6 position (m):      [-0.538638337, -0.123507598, 0.358719554]
Link6 quaternion xyzw:   [ 0.992969453, -0.080601263, 0.041185678, -0.076281328]
Link6 Euler xyz (rad):   [-2.982408700, -0.069551560, -0.167538430]
safety tip position (m): [-0.509470752, -0.078603843, 0.053378917]
surface target (m):      [-0.491717645, -0.082892425, 0.062566186]
spray distance (mm):     16.814
```

The second layer advances approximately 10 mm along the planned cavity axis.
The motion TCP remains fixed within each layer while Link6 rotates around it.

## Remaining gates

The path is intentionally preview-only because both installed calibration
files truthfully remain unvalidated:

- Eye-hand: `deployment.status=user_approved_provisional_use`
- TCP: `validated=false`, `deployment.status=dimensional_verification_required`

Before opening the 5 percent motion gate, use the pendant to place the tool at
the first entry without contact. Check that the side-outlet center is aligned
with the intended surface point at the planned stand-off, the outlet direction
faces the surface, and the rounded tip has bottom clearance. Compare the live
Link6 pose with the entry pose above. Do not mark either YAML as validated from
software output alone.

Real plasma output is not implemented in the executor and remains disabled.

## Confirmed cavity-entry check

After the operator manually placed the tool at the cavity opening, the live
Link6 pose was checked against the buffered 297-pose path. No MoveL command was
published. The check reported:

```text
entry TCP to first layer:            94.149 mm
approach/current-tool-axis error:     7.368 deg
current-tool/planned-cavity-axis:     9.812 deg
entry TCP to planned cavity axis:    27.575 mm
```

The configured limits are 5 deg for tool/cavity-axis alignment and 20 mm for
entry TCP offset, so this entry pose was correctly rejected. The planned
cavity direction toward the first layer, expressed in `baselink`, is
approximately `[+0.0941, +0.1449, -0.9850]`. Using the provisional calibration,
the entry TCP projection correction is approximately `[-25.5, -9.9, -3.9] mm`
in `baselink` XYZ. These values are pendant-jog references only; the operator
must first align the physical shaft with the cavity and preserve clearance.

The start-service checks were reordered so confirmed-entry geometry is
reported before the calibration acceptance gate. Passing the geometry check
still cannot move the robot while `execution_permitted=false`.
