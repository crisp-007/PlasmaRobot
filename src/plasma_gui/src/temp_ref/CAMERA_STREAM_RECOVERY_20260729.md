# L515 camera stream recovery (2026-07-29)

## 2026-07-30 independent color bridge (current active design)

This section supersedes every older active-state section below it. The vendor
package remains unchanged:

```text
src/third_party/realsense2/realsense2_camera
```

The project-owned camera integration is now isolated under:

```text
src/third_party/plasma_robot/tools/camera/plasma_camera_bridge
```

`plasma_l515.launch.py` starts the stock driver and the independent C++
`colored_pointcloud_node`. The stock CUDA pointcloud filter publishes stable
depth-driven XYZ with these two exact runtime options:

```text
pointcloud__cuda_.stream_filter=0
pointcloud__cuda_.pointcloud_qos=SENSOR_DATA
```

The first option prevents the split Jetson V4L2 depth/color framesets from
starving the pointcloud. The second prevents approximately 5 MB reliable DDS
samples from creating backpressure and delaying new subscribers. CUDA depth
alignment remains disabled. Resolution, frame rate, exposure, gain, and the
GUI-selected visual preset are not changed by the bridge.

The C++ bridge subscribes to the stable XYZ cloud, the independent RGB image,
Color CameraInfo, and the factory `depth_to_color` extrinsics. It projects RGB
onto the depth geometry and publishes:

```text
/plasma/camera/colored_points
```

The output contains packed `x`, `y`, `z`, and `rgb` fields and keeps
`camera_depth_optical_frame`. Points outside the RGB field of view remain
neutral gray. Therefore hand-eye conversion, path generation, and robot
coordinates still use the original depth geometry; coloring is a display and
reconstruction attribute only.

The GUI now:

- registers `plasma_camera_bridge/plasma_l515.launch.py` as its camera launch;
- subscribes to `/plasma/camera/colored_points` with SensorData QoS;
- sets the actual CUDA stream filter to `0` before restarting Color and Depth;
- keeps the existing automatic mode mapped to Short Range preset 5;
- does not rewrite the vendor pointcloud QoS after startup.

Validation on the physical L515 (`f1423110`) on 2026-07-30:

- stock XYZ and bridge output remained near 306,000 valid points per frame;
- one sampled output contained 306,336 points and 5,631 distinct RGB values in
  about 20,000 sampled points;
- bridge output accumulated 60 frames every 12.6-14.4 seconds, approximately
  4.2-4.8 Hz under Jetson/GUI load;
- after DDS discovery, a continuity probe received successive frames at
  0.14-0.61 second intervals without a stream stop;
- GUI self-check launched the wrapper with 640x480 at 30 FPS for both streams,
  applied automatic preset 5, restarted both streams, and completed;
- after that restart, five successive GUI-side probe frames each contained
  about 306,400 points with `x/y/z/rgb`, and the live VTK view visibly showed
  real teal, skin-tone, gray, and red regions;
- `git diff` for the vendor `realsense2_camera` package was empty.

Build and standalone diagnostics:

```bash
cd /home/larusxu/CodeSpace/PlasmaRobot
source /opt/ros/galactic/setup.bash
source install/setup.bash
colcon build --packages-select plasma_camera_bridge plasma_gui
ros2 launch plasma_camera_bridge plasma_l515.launch.py
```

## 2026-07-30 stable rollback (supersedes the active-state text below)

The RealSense ROS source was restored byte-for-byte to repository `HEAD` for:

- `include/base_realsense_node.h`;
- `include/named_filter.h`;
- `src/base_realsense_node.cpp`;
- `src/named_filter.cpp`;
- `launch/rs_launch.py`.

The GUI no longer passes the custom `pointcloud__cuda_.stream_filter=0` or
`pointcloud__cuda_.allow_no_texture_points=true` launch arguments. The cached
cross-frame RGB/depth projection implementation was removed because the camera
process consumed roughly 77% CPU and the full pointcloud reached only about
4.5-6.5 Hz under GUI load.

The independent VTK fix in `point_v2/main_gl.cpp` remains: an existing
`m_pointsActor` is not added to the renderer again on every frame. This prevents
the renderer from accumulating duplicate references and eventually appearing
frozen.

Stock-driver validation on the Jetson/L515 produced continuously increasing
timestamps with about 293k points per frame. With the GUI rendering and an
additional diagnostic subscriber, measured delivery was about 4.4-5.9 Hz.
GUI memory remained stable during the observation. The stable stock message has
`x`, `y`, and `z` fields only.

Real RGB pointcloud coloring is deliberately not enabled in this rollback.
The V4L2 backend delivers depth and color in separate framesets. Selecting Color
as the stock pointcloud texture source reproduces:

```text
No stream match for pointcloud chosen texture Process - Color
```

Enabling the stock CUDA-named alignment filter also starves the pointcloud.
Do not restore the per-point cached RGB projection merely to add color; it
reintroduces the high-CPU path. Any future RGB solution must preserve the
depth-driven geometry path and be benchmarked separately from reconstruction.

## Current active state: Jetson V4L2 backend restored on 2026-07-30

The active Jetson runtime uses the locally installed V4L2 librealsense 2.50.0:

```text
/home/larusxu/.local/librealsense-v4l2-2.50.0
```

`realsense2_camera` is linked to `librealsense2.so.2.50`. The GUI prepends the
same SDK `lib` directory before starting ROS. This avoids the RSUSB video
endpoint failure reproduced with both 2.51.1 and 2.54.2 on this Jetson.

The rest of the active camera path remains stock:

- stock `realsense2_camera` pointcloud implementation;
- standard `rs_launch.py` instead of `rs_launch_cuda.py`;
- no GUI camera watchdog or automatic camera restart state machine;
- no cached cross-frame color/depth remapping.

Two launch arguments are exposed for the existing CUDA-named stock filter:

```text
pointcloud__cuda_.stream_filter=0
pointcloud__cuda_.allow_no_texture_points=true
depth_module.visual_preset=5
```

The first removes the synchronized-color requirement from pointcloud
generation. On this Jetson, debug capture showed that the V4L2 backend emits a
three-frame depth set (`Depth`, `Infrared`, `Confidence`) and a separate color
set. With `stream_filter=2`, the pointcloud filter rejected every valid depth
set because it did not contain `Color`. With `stream_filter=0`, the same live
stream published about 306,000 XYZ points per frame. The independent RGB image
topic remains enabled. These options do not alter resolution, frame rate,
exposure, gain, or reconstruction parameters. The third makes the GUI's
selected `Short Range` preset effective in the driver.

Failure isolation on 2026-07-30 showed:

- the L515 streamed RGB and depth at 640x480 at 30 FPS on a laptop using SDK
  2.54.2;
- Jetson RSUSB 2.51.1 repeatedly reported `control_transfer` EAGAIN and UVC
  endpoint watchdog failures, then stopped every image and pointcloud topic;
- a direct Jetson 2.54.2 callback received IMU frames but no RGB or depth
  frames, proving that changing RSUSB versions did not restore video;
- Jetson V4L2 2.50.0 then published continuously increasing depth and RGB
  metadata at about 30 FPS and colored pointclouds containing about 132,000
  points per frame with XYZ and RGB fields;
- the remaining central pointcloud gap was traced to a GUI/runtime mismatch:
  the GUI displayed `Short Range`, but the driver was actually using
  `Max Range (4)`, which returned no depth below about 490 mm;
- setting `Short Range (5)` increased raw depth validity from 16.1% to 100%,
  center-region validity from 0% to 100%, and colored pointcloud size from
  about 132,000 to about 295,500 points per frame.

The custom 2026-07-29 cached cross-frame implementation described later in
this document is not active. It produced a partial cloud because it paired the
latest cached color frame with a different depth frame and rejected points
without valid texture coordinates. The historical sections below are retained
for diagnosis and must not be treated as the active runtime design.

## Scope

This recovery only changes the RealSense transport, ROS pointcloud delivery,
and GUI frame queueing. The configured depth/color resolution, frame rate,
visual preset, exposure, gain, and reconstruction filters are unchanged.

Camera used for validation:

- Model: Intel RealSense L515
- Serial: `f1423110`
- Firmware: `01.05.08.01`
- USB: 3.2, 5 Gbit/s

## Root causes

1. The locally installed RSUSB SDK lost the L515 UVC interfaces after the
   first frames on this Jetson host.
2. V4L2 video is stable, but this kernel does not provide reliable L515 UVC
   metadata synchronization. Requiring a color texture frame or running the
   extra depth-to-color alignment therefore causes intermittent pointcloud
   starvation.
3. Pointclouds were published with reliable QoS while the GUI subscribed as
   sensor data. Large stale messages could queue behind the live stream.
4. The GUI queued every incoming cloud for VTK conversion without a bound.

## Installed SDK

Source:

```text
/home/larusxu/share/librealsense-v2.50.0
```

Build:

```text
/home/larusxu/CodeSpace/PlasmaRobot/build/librealsense_v4l2_2_50
```

Install:

```text
/home/larusxu/.local/librealsense-v4l2-2.50.0
```

Important SDK build settings:

```text
RS2_USE_V4L2_BACKEND
BUILD_WITH_CUDA=ON
HWM_OVER_XU=OFF
```

`realsense2_camera` is pinned to this SDK through
`PLASMA_REALSENSE2_ROOT` and has an install RPATH to its `lib` directory.

## Final runtime settings

The GUI starts `rs_launch_cuda.py` with:

```text
pointcloud__cuda_.enable=true
pointcloud__cuda_.pointcloud_qos=SENSOR_DATA
pointcloud__cuda_.stream_filter=0
align_depth.enable=false
enable_confidence=false
```

`stream_filter=0` publishes geometry in `camera_depth_optical_frame` without
waiting for the L515 metadata-based depth/color synchronization that caused
stream starvation. This is the coordinate frame already used by hand-eye
conversion and path generation.

Color is restored without re-enabling that synchronization path. The camera
node caches the latest `RGB8` color frame from either a frameset or a standalone
video-frame callback. For every XYZ point it uses librealsense's calibrated
depth/color profiles, `get_extrinsics_to()`, `rs2_transform_point_to_point()`,
and `rs2_project_point_to_pixel()` to sample that independent color image. This
avoids the synchronized-frameset side effect of `rs2::pointcloud::map_to()`. A
250 ms host-monotonic freshness limit prevents a stopped color stream from
painting new geometry with an old image. The
published `/camera/depth/color/points` message therefore contains packed
`x`, `y`, `z`, and `rgb` fields while retaining the stable depth-driven publish
path. The GUI mapper explicitly renders the point-data colors as direct RGB
scalars.

The GUI conversion queue now keeps at most one cloud being converted and one
latest pending cloud. Older display-only frames are dropped.

## 2026-08-14 runtime preset correction

The GUI runtime preset sequence no longer tries to set
`pointcloud__cuda_.stream_filter` through `/camera/camera`. The CUDA pointcloud
filter is configured at launch time and the running camera node does not declare
that parameter. The failed runtime write previously aborted the sequence before
the color and depth streams could be restarted. Runtime preset changes now set
`depth_module.visual_preset` and then complete the existing serialized
color/depth stream restart without modifying the vendor RealSense package.

The parameter client also waits up to approximately 15 seconds for
`depth_module.visual_preset` to be declared. The system-check process can see
the ROS node several seconds before the L515 finishes device discovery and
parameter registration; treating the first `not declared` response as final
previously produced a false preset failure during normal startup.

## Validation

- Direct V4L2 SDK record: 19 seconds continuous, 1.7 GB before cleanup.
- ROS color stream: sustained near 25-30 Hz.
- Final RGB pointcloud stream with a real subscriber: sustained approximately
  18-20 Hz under normal load, including a run longer than two minutes. During
  a simultaneous GUI rebuild the rate temporarily fell to 12-15 Hz but the
  stream did not stop.
- RGB payload check on 2026-07-30: one cloud contained 306,485 points; 25,541
  sampled points contained 6,328 distinct RGB values, with zero white samples.
  Points outside the RGB camera field of view remain as neutral gray geometry.
  This
  confirms that the field is populated with real image colors rather than a
  constant fallback color.
- Final GUI check: the existing GUI received the rebuilt manually launched
  camera stream and displayed visibly distinct teal, skin-tone, and brown/red
  regions in the live pointcloud. The GUI and camera were left running after
  validation.
- GUI live-render freeze fixed on 2026-07-30: `MainOpengl` previously called
  `AddActor(m_pointsActor)` for every converted frame. VTK retained duplicate
  references to the same actor, so each render redrew an ever-growing number
  of copies and eventually appeared stuck on one frame. Both live and file
  display paths now call `HasViewProp()` before adding the actor. After a clean
  GUI restart, a 15-second probe received 68 successive clouds with changing
  timestamps and approximately 297k-298k points per frame.
- `realsense2_camera` links to:
  `/home/larusxu/.local/librealsense-v4l2-2.50.0/lib/librealsense2.so.2.50`.

## Rebuild

```bash
cd /home/larusxu/CodeSpace/PlasmaRobot
colcon build --packages-select realsense2_camera plasma_gui \
  --cmake-args \
  -DPLASMA_REALSENSE2_ROOT=/home/larusxu/.local/librealsense-v4l2-2.50.0
```

## GUI stream watchdog (2026-07-30)

The L515 can remain enumerated and leave `realsense2_camera_node` alive after
its UVC video stream has stopped. In that state all image and pointcloud topics
stop advancing, while VTK previously kept the last partial cloud on screen.
The kernel reported:

```text
uvcvideo: Failed to query (SET_CUR) UVC control 2 on unit 3: -32
```

The GUI now monitors both the raw main-window subscription and the pointcloud
that actually completes VTK conversion. Runtime behavior is:

- allow 15 seconds for the first pointcloud after launching the camera;
- treat 5 seconds without a new rendered cloud as a stream stall;
- immediately mark the camera disconnected and hide the stale live actor;
- restart only the GUI-managed `camera` launch process group;
- show the live actor again only after a new cloud is converted;
- retry at most three consecutive times, then request a USB reconnect instead
  of entering an infinite restart loop;
- do not run the display watchdog while the operator intentionally pauses the
  live view to select a frame.

This does not change resolution, frame rate, visual preset, exposure, gain,
pointcloud registration, or reconstruction parameters. Saved reconstruction
clouds are not hidden or deleted.

Validation on 2026-07-30:

- `plasma_gui` rebuilt successfully;
- GUI self-check started the original camera launch arguments unchanged;
- forcing the camera launch process group to stop caused the GUI log to report
  `显示点云已连续 5 秒无新帧，正在自动重启相机（第 1/3 次）`;
- the old process group was terminated and a new `realsense2_camera_node` PID
  was created;
- the GUI then reported `实时点云数据流已恢复`;
- `/camera/depth/color/points` resumed with increasing timestamps at about
  30 Hz.
