# Plasma L515 color bridge

This package leaves the vendor `realsense2_camera` package unchanged. The
vendor node publishes its stable depth-driven XYZ cloud, while
`colored_pointcloud_node` projects the latest RGB image onto that geometry
using the factory `depth_to_color` extrinsics and color `CameraInfo`. The
wrapper forces the CUDA pointcloud texture selector to `0` (no vendor texture)
so split Jetson V4L2 framesets cannot stall XYZ publication. The vendor XYZ
publisher also uses `SENSOR_DATA` QoS to avoid reliable delivery backpressure
from its approximately 5 MB frames.

Input topics:

- `/camera/depth/color/points` (vendor XYZ cloud)
- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/camera/extrinsics/depth_to_color`

Output topic:

- `/plasma/camera/colored_points` (`x`, `y`, `z`, `rgb`, SensorData QoS)

Points outside the RGB field of view remain visible as neutral gray. Color is
for display and reconstruction only; XYZ coordinates and the source frame are
copied from the vendor depth cloud.

Run the complete camera chain with:

```bash
ros2 launch plasma_camera_bridge plasma_l515.launch.py
```
