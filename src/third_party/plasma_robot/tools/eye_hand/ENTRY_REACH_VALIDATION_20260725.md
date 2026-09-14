# 入口手眼与 TCP 组合验收

## 目的

本流程量化验证以下完整坐标链在真实腔体入口处的误差：

```text
相机点云 -> 手眼矩阵 -> baselink -> Link6 -> 侧喷口中心 TCP
```

记录器是只读 C++ ROS 2 节点，只订阅转换后的喷涂路径和机械臂法兰位姿，绝不发布
运动命令。它不会自动修改手眼或 TCP YAML，也不会绕过 `execution_permitted=false`。

新版记录同时订阅 `/plasma/planned_spray_path/camera`，并要求它与 base 路径具有相同
`path_id` 和采集时间。每条记录额外保存相机坐标系原始入口/入口轴和采集六关节，便于
离线区分入口提取漂移与手眼变换漂移；该订阅同样不会发布命令。

## 验收对象

- TCP 是侧喷口中心，用于腔内喷涂旋转，不是入口对准的可见基准。
- 入口验收使用圆头末端和喷杆中心轴。圆头与入口不接触；将圆头对准实际开口中心，
  并使喷杆轴线与软件入口轴一致。
- 当前工具必须是 `3_4 + 200_1x10`，`Link6 -> TCP` 名义轴向长度为 `302 mm`。

## 启动

先正常启动 GUI、自检、机械臂驱动并生成一次完整喷涂轨迹。确认
`/plasma/planned_spray_path/base` 已有最新路径后，在单独终端运行：

```bash
source /opt/ros/galactic/setup.bash
source /home/larusxu/CodeSpace/PlasmaRobot/install/setup.bash

ros2 run plasma_eye_hand reach_validation_recorder --ros-args \
  -p output_yaml:=/home/larusxu/CodeSpace/PlasmaRobot/src/third_party/plasma_robot/tools/eye_hand/records/entry_reach_20260725.yaml
```

## 每次采集

1. 等离子保持关闭，速度保持低速，急停可触及。
2. 使用经过碰撞检查的点动/规划，把圆头移动到软件识别的几何开口中心。
3. 保持圆头与工件不接触；从两个方向观察喷杆轴线是否横向居中。
4. 停稳后在记录器终端按 `Enter`，保存一条观测。
5. 改变相机采集姿态，重新采集点云并生成新的 `path_id`，重复以上步骤。

至少需要 3 个不同真实采集时刻。记录器使用路径消息头的采集时间生成
`acquisition_id`，因此 GUI 重启后显示名称重新从“重建1路径”开始也不会混淆样本。
同一次采集连续按三次不能通过验收，因为它不能暴露手眼矩阵随采集姿态变化的误差。

终端命令：

```text
Enter 或 o   记录当前人工确认的入口对准状态
r             显示当前统计和验收结果
q             退出
```

## 默认门限

```text
独立路径数                 >= 3
三维位置误差最大值         <= 5 mm
入口法向轴向误差最大值     <= 5 mm
入口横向误差最大值         <= 3 mm
喷杆轴线夹角最大值         <= 2 deg
```

报告中的 `signed_axial_mm` 以腔体向外法向为正，`lateral_mm` 是垂直于入口法向的
横向偏差。`axis_alignment_deg` 是喷杆轴线与软件入口轴的夹角，用于入口验收；
`full_orientation_deg` 还包含绕喷杆自身的旋转，仅用于侧喷方向诊断，不参与圆头入口
验收。提高插入精度时优先压低横向误差；轴向误差必须与 L515 已知约 9.5 mm 深度
偏差分开分析。

## 验收与修正原则

- `accepted: false` 时不得直接把手眼/TCP YAML 标成已验收。
- 多个采集姿态的偏差方向随相机姿态变化：优先检查/重做手眼和深度链。
- 偏差在 Link6 局部坐标中稳定：优先复测 TCP 横向安装和喷口朝向。
- 偏差只在 baselink 中近似常量：可建立独立的安装组合补偿，但不能污染手眼矩阵
  或 TCP 名义尺寸。
- 报告通过后仍需人工审查记录、现场照片和工具规格，之后才更新两个 YAML 的部署
  状态并关闭 `allow_unvalidated_dry_run` 做严格低速复验。

## 生成位置修正候选

至少三条记录包含 `source_geometry` 后，可用只读 C++ 分析器拟合训练候选：

```bash
ros2 run plasma_eye_hand reach_refinement_analyzer \
  records/entry_reach_20260725.yaml \
  records/entry_reach_refinement_candidate_20260725.yaml \
  3
```

候选矩阵语义为：

```text
T_base_source_corrected = T_base_source_current * T_source_correction
```

分析器不发布 ROS 话题、不修改正式手眼/TCP YAML，并固定输出
`validated: false`、`execution_allowed: false`。拟合使用过的记录只能算训练集；至少需要
一个未参与拟合的新采集姿态做盲验证，不能用训练残差批准运动。

当前 L515 离线报告仍有 `9.79 mm` Depth-PnP 中位误差，因此仅创建工具并不代表
已经通过验收；必须完成上述真实观测。
