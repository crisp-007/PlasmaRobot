# CAD source files

This directory stores archived source CAD. ROS, RViz, and MoveIt do not load
these files directly.

## Reinforcement plate

- Source: `加固板.SLDPRT`
- Export: `加固板.STL`
- Received: 2026-07-28
- SLDPRT SHA-256: `4d245fe596337352ad8b352664c138e2f89bdf412e122d3b67a693782ab4def1`
- STL SHA-256: `bed9c65fb8309f031160a0622c29b8cbae56afb3880dd44ce533144e03b348ea`
- Trial thickness: 2 mm
- Status: not installed; withdrawn by the operator on 2026-07-29

The binary STL uses millimetres and measures approximately
`59 x 2 x 144.5 mm`. The runtime copy and all URDF references were removed
when the tool returned to its original configuration. The later 2026-07-30
physical stack measurement supersedes the historical values here: the active
tool is 318 mm from Link6 to the rounded end and 310 mm axially to the outlet
centre. See `../../TOOL_AXIAL_REMEASUREMENT_20260730.md`.
