# 2 mm reinforcement plate trial (withdrawn 2026-07-29)

> This option is not installed. The operator returned to the original tool on
> 2026-07-29. The active URDF therefore has no reinforcement-plate visual or
> collision mesh and `mount_xyz` is `[0, 0, 0]`. The active base-tool geometry
> was remeasured on 2026-07-30 as a 318 mm rounded-tip distance and a
> `[0.002, 0, 0.310] m` side-outlet TCP.

## Trial geometry

The evaluated reinforcement plate would add a 2 mm axial offset between `Link6`
and the complete existing plasma-tool CAD stack. The original internal CAD
stack remains unchanged:

```text
historical stack used by this withdrawn trial: 78 + 32 + 200 = 310 mm
reinforcement plate axial offset:                         2 mm
trial Link6-to-rounded-tip distance:                    312 mm
trial Link6-to-side-outlet-center distance:             304 mm
side-outlet radial offset:                                2 mm
```

During the trial, the plate was represented by `mount_xyz="0 0 0.002"`. That
runtime change has been removed.

## CAD and collision geometry

The received SolidWorks source and export are archived at
`reference/cad/加固板.SLDPRT` and `reference/cad/加固板.STL`. The binary
STL contains 1568 triangles, is expressed in millimetres, and has an
approximately `59 x 2 x 144.5 mm` bounding box.

The temporary runtime mesh used the following proposed placement:

```text
mesh scale:  [0.001, 0.001, 0.001]
mesh origin: [-0.0295, -0.002, -0.0295] m
```

This would place the supplied plate at local `Y=-2..0 mm`, immediately behind the
old base mounting face. The original base begins at local `Y=0`; after the
2 mm Link6 mount offset, the plate spans the new Link6-to-tool gap.

The runtime mesh copy and its URDF references were removed during rollback.
The source CAD remains archived for traceability only.

## Validation state

The rollback restores the geometry used by the existing validation records.
The hand-eye transform is unchanged. Existing deployment gates remain under
the control of their calibration files; this record does not enable motion.
