# L515 Hand-Eye Matrix

当前暂用矩阵：`config/camera_to_gripper.yaml`

## 设备与坐标约定

- 相机：Intel RealSense L515，序列号 `f1423110`
- 图像：`640x480`
- 标定板：ChArUco `9x7`，方格 `20.0 mm`，Marker `15.0 mm`，`DICT_4X4_50`
- 样本：稳定段 `img_001` 至 `img_058`，共 58 张，方法：TSAI（AUTO选择）
- `T_camera_to_gripper` 表示 `^gripper T_camera`，平移单位为米
- 平移分量：`[-36.678083, -61.625162, 29.686178] mm`

## 使用状态

该矩阵在 2026-07-29 经用户决定作为当前设备的暂用工作矩阵保存。它来自 L515 重新安装
后的稳定数据段 `img_001` 至 `img_058`；`img_059` 至 `img_062` 因设备空间关系发生约
`36--48 mm` 离散变化而排除。该矩阵没有通过项目原定的全部严格旋转验收门槛，因此文件
同时保留：

```text
deployment.status: user_approved_provisional_use
validation.strict_offline_accepted: false
```

仓库中的矩阵由程序显式加载；`pointcloud_get/calib_data/handeye_matrices/current` 已切换到
`provisional_tsai_20260729_stable_001_058`。

本目录是 ROS 2 数据包 `plasma_eye_hand`。构建后矩阵安装到：

```text
share/plasma_eye_hand/config/camera_to_gripper.yaml
```

入口端到端数据由 `reach_validation_recorder` 记录；
`reach_refinement_analyzer` 对至少三条完整记录做 C++ Kabsch 刚体拟合，只生成不可执行的
训练候选。详细流程和当前结果分别见 `ENTRY_REACH_VALIDATION_20260725.md` 与
`ENTRY_REACH_RESULT_20260725.md`。

固定偏移假设、现有数据的方差计算以及允许启用补偿的盲测条件见
`CONSTANT_OFFSET_ASSESSMENT_20260726.md`。当前五次误差不满足恒定偏移条件，禁止把
误差均值直接写入手眼、TCP 或 baselink 路径点。

只测试固定平移、不允许拟合旋转时，使用：

```bash
ros2 run plasma_eye_hand reach_refinement_analyzer \
  records/entry_reach_20260725.yaml \
  records/entry_reach_translation_candidate_20260726.yaml \
  3 --translation-only
```

该模式仍固定输出 `validated: false` 和 `execution_allowed: false`。当前候选为相机源
坐标平移 `[-17.893, -51.329, -7.984] mm`，训练 RMS 为 `10.294 mm`；只能加载到
预览节点做独立圆头盲验证。

`plasma_path_transform.launch.py` 默认加载该安装路径。当前矩阵只允许坐标预览；严格验收完成前，不得把 `deployment.status` 改为 `validated`、`approved` 或 `production_ready`。

## 已知误差

| 检查 | 平移 | 旋转/其他 |
|---|---:|---:|
| 固定板一致性（中位） | 2.54 mm | 0.65 deg |
| 固定板一致性（最大） | 6.92 mm | 1.47 deg |
| 5折验证（中位） | 2.68 mm | 0.71 deg |
| 5折验证（最大） | 7.00 mm | 1.57 deg |
| 留一法最大影响 | 1.43 mm | 0.13 deg |
| 新安装独立姿态验证 | 待完成 | 待完成 |

新数据中 TSAI/PARK/HORAUD 的最大互差为 `0.368 mm / 0.352 deg`。固定板旋转中位和
五折旋转仍高于 `0.500 deg` 严格门限，因此保持暂用状态。

L515 静态深度平面诊断结果：

- 角点 Depth-PnP：中位 `9.79 mm`，最大 `11.15 mm`
- 深度平面相对 PnP 平面的统一偏移：`-9.50 mm`
- 两平面法向差：`0.34 deg`
- 深度平面自身拟合残差中位：`0.49 mm`

因此当前链路不能假设具有亚毫米或稳定小于 `3 mm` 的端到端精度。涉及实际接触、靠近工件或精密测量时，必须把上述误差计入安全余量并先做非接触验证。

## 来源

```text
/home/larusxu/Data/script/pointcloud_get/calibration/calib_data/handeye_l515_20260729_161944_precision_v1_stable_001_058
```

本次严格报告保存在该数据目录的 `result/` 下。求解与排除记录见
`HANDEYE_STABLE_SEGMENT_20260729.md`。

## 鲁棒手眼离线重估

`robust_handeye_analyzer` 使用 C++ 在 SE(3) 上同时优化
`T_camera_to_gripper` 和固定标定板位姿，并使用 Huber 权重、逐步异常样本剔除及五折留出
比较。由于本机 C++ OpenCV 未安装 ChArUco 模块，`export_charuco_samples.py` 只负责复用
现有 Python OpenCV 从原始图片导出 PnP 位姿；求解、评估和候选写出均由 C++ 完成。

2026-07-27 的离线结果见：

```text
records/camera_to_gripper_robust_candidate_20260727.yaml
ROBUST_HANDEYE_REESTIMATION_20260727.md
```

候选保留 36/40 组，剔除 `img_002/img_008/img_027/img_038`。五折平移中位/最大由原报告的
`3.208/7.317 mm` 降至 `2.012/5.172 mm`，但五折旋转中位仍为 `0.522 deg`，未通过
`0.500 deg` 门限。文件固定保存 `validated: false`、`execution_allowed: false`，没有写入
`config/camera_to_gripper.yaml`，也不得直接给真实机械臂使用。

## 深度/彩色几何采集

`collect_depth_charuco_ros.py` 只读订阅现有 RealSense ROS 节点的彩色图、原始深度图和
CameraInfo。它不会打开相机 SDK、不会设置相机参数、不会连接机械臂。每组数据使用最近
15 帧深度中值，并保存 ChArUco PnP 平面及板面内部原始深度射线，用于后续 C++ 离线拟合
深度比例、偏置和小量彩深刚体修正。

完整步骤见 `DEPTH_ALIGNMENT_CALIBRATION_20260727.md`。至少采集 9 组：约
`0.35/0.50/0.65 m` 三个距离，每个距离覆盖画面左/中/右并改变板面倾角。数据不足时不
允许生成校正候选。

2026-07-27 首轮九组的 C++ 联合拟合和整组留一验证已经完成。虽然全数据误差中位由
`15.068 mm` 降到 `6.005 mm`，整组留一误差中位仍为 `5.904 mm`，外参平移修正还触及
`30 mm` 上限；候选已拒绝并保持不可执行。逐组深度平面自身约 `0.37--1.05 mm`，但相对
PnP 法向差为 `1.95--4.02 deg`。下一轮必须先使用尺寸核对过的刚性平整标定板，并满足
不少于 `35` 个角点、彩深时间差不超过 `50 ms`。完整结果和重采条件见
`DEPTH_ALIGNMENT_CALIBRATION_20260727.md`。

## 当前末端 TCP pivot 工具

`tcp_pivot_calibrator` 用于求真实 `Link6 -> 侧喷口中心 TCP` 三维平移，尤其是
图纸不能给出的横向安装偏差。它是 C++ 只读采集程序，仅订阅
`/rm_driver/udp_arm_position`，不会发布 MoveJ、MoveL 或其他运动指令，也不会
自动修改 `plasma_tool_description` 中的 TCP YAML。

使用前必须确认控制器该话题仍表示 `baselink -> Link6` 法兰零工具。当前
ECO65-BI 已完成这项确认；一旦在控制器中写入非零工具坐标，必须停止使用该
默认话题或先换成明确的法兰位姿来源。

现场步骤：

1. 启动真实机械臂驱动，但不要启动自动轨迹。
2. 只有具备不会卡住或损坏侧孔的专用球形/万向定位夹具时，才将喷口中心保持
   在同一个空间位置。普通长定位销不能固定插入侧孔后强行改变姿态。
3. 只用示教器低速改变腕部姿态，至少采 6 个姿态，且要绕多个轴改变方向。
4. 每个姿态稳定后在本工具终端按 Enter；采完输入 `s` 求解。

```bash
source /opt/ros/galactic/setup.bash
source /home/larusxu/CodeSpace/PlasmaRobot/install/setup.bash

ros2 run plasma_eye_hand tcp_pivot_calibrator --ros-args \
  -p output_yaml:=/home/larusxu/CodeSpace/PlasmaRobot/src/third_party/plasma_robot/tools/eye_hand/records/tcp_pivot_3_4_200_1x10.yaml
```

输出包含原始 `T_base_to_flange` 样本、`flange_to_tcp_m`、固定点、矩阵秩、
条件数和残差，并固定写入 `validated: false`、`execution_allowed: false`。先检查
样本姿态分布、残差和重复试验一致性，再人工更新当前 TCP YAML；不能把手眼、
点云或人工对点误差作为 TCP 补偿写进去。

单点 pivot 只能可靠求被固定点的三维位置，不能单独确定绕喷杆轴的旋转角。
如果选择这种方法，必须由专用夹具固定真实侧喷口中心，并另外检查喷口朝向。当前正式过程帧为
`plasma_motion_tcp`：原点就是喷口中心，`+X` 沿喷杆朝圆头末端，`+Z` 指向
侧面喷涂方向；完整坐标链还需要独立的相机已知点验证。

Pivot 是可选的计量手段，不是喷涂动作。没有专用夹具时应保留图纸名义 TCP，
改用非接触已知点/标靶验证。实际喷涂中，喷口中心与内壁目标点保持喷涂距离，
两者绝不接触。

## 入口端到端验收工具

`reach_validation_recorder` 是 C++ 只读节点，用于记录最新转换后的几何开口中心与
真实圆头人工对准位姿之间的误差。它按腔体法向分别输出轴向和横向误差，并要求至少三个
不同真实采集时刻，不能用同一状态重复采样伪造验收。

完整现场步骤和门限见：

```text
ENTRY_REACH_VALIDATION_20260725.md
ENTRY_REACH_RESULT_20260725.md
```

该工具只建立可追溯的量化依据。报告通过前不会修改当前手眼/TCP YAML，报告通过后也
必须人工审查相机姿态分布、工具规格和现场对准方法，才能解除严格执行门。
