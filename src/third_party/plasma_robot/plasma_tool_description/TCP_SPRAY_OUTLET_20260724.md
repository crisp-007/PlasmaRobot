# Side-outlet TCP correction (updated 2026-07-30)

## Confirmed physical meaning

The robot trajectory point is the center of the physical side spray outlet.
The rounded distal end of the nozzle is not a spraying point and is not a
separate physical "safety point" to be located during TCP calibration.

The drawing and operator confirmation establish:

```text
Link6 to rounded nozzle end, axial             318 mm
rounded end to outlet near edge, axial           3 mm
side-outlet axial length                        10 mm
rounded end to outlet center, axial              8 mm
Link6 to outlet center, axial                  310 mm
3_4 rod axis to outlet surface, radial           2 mm
```

Nominal transforms are therefore:

```text
Link6 -> plasma_motion_tcp translation: [0.002, 0, 0.310] m
motion_tcp -> plasma_nozzle_tip:          [0.008, 0, -0.002] m
motion_tcp -> plasma_spray_outlet:        [0, 0, 0] m
```

`plasma_motion_tcp +X` follows the rod toward the rounded end. `+Z` is the
side spray direction. A layer's positive and negative 180-degree sweeps must
keep the `plasma_motion_tcp` position fixed, so rotation occurs at the actual
spray point.

The 2026-07-24 `302/310 mm` software values omitted the 8 mm flange end plate.
The 2026-07-30 physical stack is `8 + 70 + 40 + 200 = 318 mm`. The 3 mm
dimension reaches only the outlet near edge; half of the 10 mm outlet length
must be added to locate its center, 310 mm axially from Link6.

## Optional pivot procedure

The pivot solver returns the position of whichever physical point is held
fixed. This is a temporary metrology operation, not the spraying motion. Direct
side-outlet pivot is permitted only with a purpose-built spherical or
articulated locator that cannot bind or damage the hole. A fixed long pin must
not be inserted into the side hole while changing robot orientation.

The result field is `result.flange_to_tcp_m`. Pivot does not determine the
rotation about the rod axis, so the side-outlet direction still requires an
independent orientation check.

Without such a fixture, retain the drawing-derived nominal TCP and perform a
non-contact target check. During spraying, the outlet center stays on the
cavity axis for the current layer, its `+Z` axis points to the corresponding
wall target, and the distance between those two points is the spray distance.
The outlet never contacts the workpiece.

## Deployment state

This correction uses the installed axial measurement and confirmed outlet
edge semantics. The current YAML remains
`validated: false`, `quality_accepted: false`, and `execution_allowed: false`.
No real motion or plasma output is enabled by this change.
