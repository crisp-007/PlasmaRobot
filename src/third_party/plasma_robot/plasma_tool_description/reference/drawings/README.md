# Current plasma-tool drawings

These three AutoCAD 2018 drawings are the dimensional source for the current
`3_4 + 200_1x10` trial tool:

```text
1X固定罩1.DWG
2X枪体4-6转接.DWG
喷杆X3+护套X1.DWG
```

## Axial dimensions used by the model

- The fixed-cover drawing contains the `70 mm` and `23 mm` axial sections;
  its STL spans `93 mm`.
- The adapter STL is mounted at `78 mm` from Link6, representing the `8 mm`
  flange end plate plus `70 mm` fixed cover. The `40 mm` gun head then places
  the effective gun-body exit at `118 mm` from Link6.
- The rod drawing specifies a physical rod length of `210 mm` and a `10 mm`
  insertion section. The exposed length outside the gun body is therefore
  `200 mm`.
- The rod STL models the visible `200 mm` section and is placed at the
  gun-body exit. The hidden insertion section is not drawn as a separate link.

The current flange-to-tip transform is consequently:

```text
Link6 to adapter origin      78 mm
adapter origin to gun exit  40 mm
exposed rod                 200 mm
                            ------
Link6 to nozzle tip         318 mm (installed stack, 2026-07-30)
```

The old `78 + 30 + 196 = 304 mm` result mixed the insertion-start origin with
an internal provisional TCP point. The later `302/310 mm` software model used
only 32 mm after the adapter origin and omitted 8 mm of the gun-head stack.
The active model is now `78 + 40 + 200 = 318 mm`; the discrepancy is not hidden
in the hand-eye matrix or a world-coordinate compensation.

This drawing check establishes the axial geometry. Radial installation error
and the complete hand-eye chain still require known-point validation before
automatic execution can be enabled.
