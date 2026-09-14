# L515 深度/彩色几何离线校正（2026-07-27）

## 目的

现有静态检查显示深度平面自身残差中位约 `0.495 mm`，但相对彩色 ChArUco PnP 平面有
`-9.503 mm` 系统偏移。单帧无法区分固定偏置、距离比例误差和彩深外参误差，因此需要多
距离、多画面位置和多倾角数据。

## 采集器边界

`collect_depth_charuco_ros.py`：

- 只读订阅 `/camera/color/image_raw`、`/camera/depth/image_rect_raw`、彩色/深度 CameraInfo
  和 `/camera/extrinsics/depth_to_color`；采集器用工厂外参离线投影原始深度，不依赖当前
  无帧的对齐深度话题；
- 不调用 RealSense SDK，不修改 `visual_preset`、分辨率、帧率或其他相机参数；
- 不连接机械臂，不订阅或发布运动命令；
- 每次保存最近 15 帧深度中值、彩色原图、原始深度图、板内掩膜、PnP 位姿和最多 800
  个板面深度射线；
- 当前默认角点不足 35 或有效深度不足 250 时拒绝保存。

2026-07-27 首轮数据诊断后，默认质量门已收紧为：角点不少于 `35`、当前彩色帧与最新
深度帧时间差不超过 `50 ms`、有效深度点不少于 `250`。采集窗口只有全部满足时才显示
`READY`。这些门只控制离线数据保存，不修改 RealSense 参数。

## 启动

先使用软件原有的相机启动命令，确认彩色和对齐深度话题都有实时帧。然后运行：

```bash
source /opt/ros/galactic/setup.bash
source /home/larusxu/CodeSpace/PlasmaRobot/install/setup.bash

ros2 run plasma_eye_hand collect_depth_charuco_ros.py \
  --output /home/larusxu/CodeSpace/PlasmaRobot/src/third_party/plasma_robot/tools/eye_hand/records/depth_alignment_20260727 \
  --samples 9 --median-frames 15
```

采集窗口中，`SPACE` 或 `C` 保存，`Q` 结束。保存时标定板必须静止；板面不需要接触任何
物体，机械臂也不需要运动。

## 九组数据

1. 约 0.35 m，画面中央，基本正视。
2. 约 0.35 m，画面左侧，轻微左右倾斜。
3. 约 0.35 m，画面右侧，轻微上下倾斜。
4. 约 0.50 m，画面中央，轻微左右倾斜。
5. 约 0.50 m，画面左侧，轻微上下倾斜。
6. 约 0.50 m，画面右侧，基本正视。
7. 约 0.65 m，画面中央，轻微上下倾斜。
8. 约 0.65 m，画面左侧，基本正视。
9. 约 0.65 m，画面右侧，轻微左右倾斜。

实际距离由 PnP 自动记录，不要求人工量到精确数值。倾角约 `10--20 deg` 即可，不要让
反光、遮挡或过大倾角导致角点/深度缺失。

## 2026-07-27 九组结果

首轮九组已保存到：

```text
records/depth_alignment_20260727/dataset.yaml
```

使用 C++ `depth_alignment_analyzer` 联合拟合深度比例、射线固定偏置及工厂彩深外参的
小量 SE(3) 修正，并执行九次整组留一验证。候选报告为：

```text
records/depth_alignment_candidate_20260727.yaml
```

结果明确不通过：

| 指标 | 结果 |
|---|---:|
| 工厂模型平面绝对误差中位 / RMS | `15.068 / 15.098 mm` |
| 全九组拟合绝对误差中位 / RMS | `6.005 / 6.237 mm` |
| 整组留一绝对误差中位 / 最差组 | `5.904 / 8.546 mm` |
| 深度比例 | `0.99666869` |
| 射线偏置 | `-2.117 mm` |
| 外参修正平移 / 旋转 | `30.000 mm（触及上限） / 1.955 deg` |
| 法方程条件数 | `4.421e9` |

候选的外参平移直接触及保守 `30 mm` 边界，整组留一参数变化最大达到
`25.129 mm / 3.732 deg`。因此这些参数不能解释为真实相机标定量，也不能作为固定补偿。
报告固定保存 `validated: false`、`execution_allowed: false`，没有修改相机驱动、正式 TF、
GUI 或机械臂运动许可。

逐组诊断显示：每组深度点自身仍能形成平面，平面自拟合绝对残差中位为
`0.372--1.045 mm`；但深度平面与 ChArUco PnP 平面的法向差达到
`1.945--4.018 deg`，各组工厂模型有符号偏移约为 `-5.955--18.336 mm`。这不是一个
恒定比例、恒定偏置或固定彩深刚体修正能够同时消除的问题。

外参数组解释已经从源码核对：ROS 驱动原样复制 `rs2_extrinsics.rotation`，librealsense
将其定义为列主序。故采集器按列主序解析是正确的。故意转置旋转后的诊断误差中位仍有
`14.078 mm`，不能解释当前问题。

首轮数据还存在以下质量问题：

- `sample_02` 彩深时间差 `305.7 ms`，`sample_05` 为 `132.7 ms`，`sample_09` 为
  `66.3 ms`；
- `sample_08/09` 仅检测到 `27/26` 个角点，板内深度采样区域明显缩小；
- 当前打印板/支撑面的局部平整度没有计量保证。深度自身平面稳定、但相对 PnP 法向随
  采样区域变化，最需要优先排查标定板翘曲、贴合和平面尺度。

## 下一轮采集条件

1. 将同一 ChArUco 图案无气泡、无翘曲地贴合到刚性、哑光平板上；不使用玻璃或高反光
   表面。用卡尺核对实际方格边长，若不是 `20.0 mm`，先修改板参数再采集。
2. 新建数据目录，不覆盖本次失败数据。每个姿态必须看到至少 `35` 个角点并等待窗口显示
   `READY`。
3. 保持板和相机静止后再保存。若 RGB 或深度流停顿，等待恢复或按原参数重启驱动，不在
   停顿期间采集。
4. 新九组先运行离线分析器；只有整组留一门全部通过，才能安排独立入口盲验证。不能从
   本轮候选直接进入实机运动。

离线分析命令：

```bash
source /opt/ros/galactic/setup.bash
source /home/larusxu/CodeSpace/PlasmaRobot/install/setup.bash

ros2 run plasma_eye_hand depth_alignment_analyzer \
  --dataset /path/to/new_dataset/dataset.yaml \
  --output /path/to/depth_alignment_candidate.yaml
```

## 历史状态

2026-07-27 首次准备采集时，相机节点持续报告 USB
`Resource temporarily unavailable`，并触发 `uvc streamer watchdog`；彩色和对齐深度
话题均无帧。失败进程已停止，没有更改相机参数。需要重新插拔 L515 USB 后，仍用软件
原启动命令恢复话题，再开始上述九组采集。

该 USB 故障后来通过原参数重启和必要的重新插拔恢复，完成了上述首轮九组采集。相机
参数未修改。
