# 3_4 + 200_1x10 verification record (updated 2026-07-24)

> Historical record only. The 2026-07-30 physical stack measurement supersedes
> the intermediate 302/310 mm values: the active motion TCP is
> `[0.002, 0, 0.310] m` and the rounded-end length is 318 mm. See
> `TOOL_AXIAL_REMEASUREMENT_20260730.md`.

This record covers drawing-derived process geometry and software integration.
It does not approve automatic robot motion.

## Drawing result

The three DWG drawings and the confirmed assembly establish:

```text
Link6 to physical safety tip            318 mm
side outlet starts behind tip             3 mm
side outlet axial length                  10 mm
tip to side-outlet center                  8 mm
Link6 to motion TCP                      310 mm
3_4 shaft outer radius                     2 mm
```

The rod is physically 210 mm long and inserts 10 mm into the gun body, leaving
200 mm exposed. The superseded 304 mm value mixed the insertion start with a
temporary TCP and is not used.

## Frame contract

```text
Link6 -> plasma_motion_tcp:   translation [0, 0, 0.310] m
motion_tcp -> safety_tip:     translation [+0.008, 0, 0] m
motion_tcp -> spray_outlet:   translation [0, 0, +0.002] m
```

`plasma_motion_tcp +X` points toward the physical tip/deeper into the cavity.
`+Z` points through the side outlet. Layer rotations are about `+X`.

Path planning uses asymmetric axial occupancy:

```text
opening side: outlet rear half 5 mm + configured opening margin
bottom side:  safety point lead 8 mm + configured bottom margin
```

The 10 mm outlet length is not a shaft diameter. The `3_4` shaft radius used
for radial geometry is 2 mm.

## Software verification

- `plasma_robot_interfaces`, `plasma_tool_description`,
  `plasma_path_transform`, `plasma_path_executor`, and `plasma_gui` built
  successfully on ROS 2 Galactic.
- `plasma_path_transform`: 8 tests passed.
- `plasma_path_executor`: 6 tests passed.
- A read-only transform service check with identity motion TCP produced
  `safety_point=[0.008, 0, 0]` and side-outlet position `[0, 0, 0.002]`, with
  both validity flags true.
- The same check produced the expected flange transform using the 310 mm
  motion TCP offset.
- The output remained `execution_permitted=false` because hand-eye and tool
  calibration are still unvalidated.
- No MoveJ, MoveL, stop, or plasma-output command was sent during these checks.

## Remaining acceptance work

1. Verify the installed tool's full six-degree-of-freedom motion TCP.
2. Verify the physical safety point and side-outlet location, including radial
   installation offsets.
3. Complete non-contact known-point validation of the hand-eye transform.
4. Validate the complete camera-to-process geometry chain and first spray
   pose before changing any deployment approval flag.

Until those checks pass, all converted poses are preview only.
