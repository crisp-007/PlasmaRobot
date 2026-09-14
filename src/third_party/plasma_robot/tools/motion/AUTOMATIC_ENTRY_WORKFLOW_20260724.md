# 机械臂自动到入口与分段进入流程（2026-07-24）

> 防碰撞实现和安全门见同目录 `COLLISION_SAFETY_20260724.md`。当前入口规划会同时检查
> “当前位置到预入口”和“预入口到全部喷涂点”，不再只检查第一段。

## 1. 本次实现的目标

当前流程不再要求操作者精确手动找到入口：

```text
机械臂任意当前位置
  -> MoveIt 规划到入口外法线方向 100 mm 的预入口点
  -> RViz 预览和人工确认
  -> 执行规划轨迹到预入口
  -> 最多 5 mm/步沿腔体轴线到入口
  -> 最多 5 mm/步到第一喷涂层
  -> 每层 0 -> +180 -> 0 -> -180 -> 0
```

完整笛卡尔预览保持环境碰撞检查，并对六轴相邻采样点执行 10 度绝对步差限制；
这允许设计中的 5 度喷涂旋转，同时拒绝 IK 分支切换或真实整圈反转。旋转关节
经过正负 180 度数值边界时，会先在 URDF 关节限位内改写为与上一点最近的等价角，
并将连续角度写回 RViz 轨迹；无法在限位内连续化时仍会被 10 度限制拒绝。
由于 MoveIt 的预入口姿态目标可能返回不同 IK 支路，规划器默认最多尝试 8 个完整
候选。一个候选只有在入口接近和全部喷涂点同时通过后才会结束重试并发布到 RViz；
中间失败候选不会获得碰撞许可，也不能执行。

2026-07-25 现场重新生成 `重建1路径` 后确认：预入口距离为 `100.0 mm`，转换后
包含 446 个喷涂位姿；完整规划在第 3 个候选支路通过，最大相邻关节步差为
`1.512857 deg`（joint4）。操作者从 RViz 确认预入口到入口沿外法向直线进入，
不再出现靠近入口后从下方横向进入的路径。

等离子输出仍保持关闭。本流程只完成机械臂入口接近和喷涂路径干运行。

## 2. 入口如何产生

GUI 在喷嘴位姿生成阶段读取拟合切片中的 `ClosedLayer` 和 `SliceIndex`：

1. 闭合层用于生成喷涂层 TCP。
2. 开口层不再丢弃，而是用于判定腔体开口方向。
3. 比较开口层和闭合层在蓝色直轴上的分布，确定从腔内指向自由空间的外向轴。
4. 取最靠外的开口层质心，并投影到蓝轴，得到 `cavity_entry_tcp_pose`。
5. 入口沿外向轴退出 100 mm，得到 `pre_entry_tcp_pose`。该距离沿用旧机械臂 SOP
   的默认法向接近距离，使 MoveIt 的自由关节规划段停在入口区域之外。
6. 入口和预入口姿态与第一喷涂层的零度姿态一致。

路径同时携带 TCP 位姿、Link6 位姿、外向轴、入口来源切片和预入口距离。转换节点负责把相机坐标统一转换到 `baselink`，不会由执行器猜入口。

## 3. 人工摆放不准时的处理

推荐始终使用 MoveIt 自动到预入口。若已经人工放到入口附近，执行器允许小范围线性纠正：

```text
到入口的总平移误差 <= 30 mm
离腔体轴线的横向误差 <= 10 mm
Link6 姿态误差 <= 10 deg
```

纠正过程同样最多 5 mm/步，并逐步检查实时 Link6 到位误差。超过上述范围时会拒绝 MoveL 纠正，要求重新使用 MoveIt 规划到预入口，避免从任意位置直接做笛卡尔直线运动。

## 4. 当前 MoveIt 模型

当前模型使用：

```text
ECO65 本体
3_4 转接件
200 mm 喷管
1x10 侧喷口
Link6 -> TCP = [0.002, 0, 0.310] m（2026-07-30 复测）
Link6 -> 圆头末端轴向长度 = 0.318 m（2026-07-30 复测）
TCP -> 圆头末端 = [0.008, 0, -0.002] m
```

碰撞模型包含真实工具 mesh 和白色机柜的保守方盒：

```text
尺寸: 0.995 x 0.603 x 0.954 m
中心（baselink）: [0.403, 0, -0.403] m
```

患者、床和实时腔体 mesh 尚未作为 MoveIt 世界碰撞物体接入，因此每次仍必须在 RViz 检查轨迹和现场障碍物。

## 5. GUI 操作顺序

更新程序后，旧内存中的喷嘴位姿没有入口字段，必须重新点击“完整喷涂轨迹”生成一次。

进入“机械臂执行”后重复点击执行按钮，每次只推进一个阶段：

1. 第一次：启动当前工具的 MoveIt 和 RViz。
2. 第二次：生成当前位置到预入口，并继续验证预入口到全部喷涂点的完整碰撞检查轨迹。
3. 在 RViz 检查两段轨迹以及机械臂、工具和机柜的间隙。
4. 第三次：二次确认后执行到预入口。
5. 到达预入口后再次点击：确认直线通道，分段进入并运行喷涂路径。

停止按钮会同时停止入口 MoveIt 运动和正式路径执行器，并额外发送 RealMan 停止命令。

## 6. 命令行预览启动

GUI 会按需启动；命令行调试可使用：

```bash
source /opt/ros/galactic/setup.bash
source /home/larusxu/CodeSpace/PlasmaRobot/install/setup.bash

ros2 launch plasma_path_executor automatic_entry_motion.launch.py \
  motion_enabled:=false \
  rviz:=true \
  start_rm_control:=true
```

规划和状态服务：

```bash
ros2 service call /plasma_entry_motion_planner/plan_to_pre_entry \
  std_srvs/srv/Trigger "{}"

ros2 service call /plasma_entry_motion_planner/execute_to_pre_entry \
  std_srvs/srv/Trigger "{}"

ros2 service call /plasma_entry_motion_planner/stop \
  std_srvs/srv/Trigger "{}"
```

入口坐标可直接查看：

```bash
ros2 topic echo /plasma/planned_spray_path/base \
  plasma_robot_interfaces/msg/SprayPath
```

关注字段：

```text
cavity_entry_tcp_pose
pre_entry_tcp_pose
cavity_entry_flange_pose
pre_entry_flange_pose
cavity_axis
pre_entry_distance_m
```

## 7. 当前安全锁（不能绕过）

当前两处运动门都保持关闭：

```text
entry_motion motion_enabled=false
path_executor motion_enabled=false
```

而且当前标定文件仍报告：

```text
hand-eye deployment.status=user_approved_provisional_use
hand-eye strict_quality_accepted=false
TCP validated=false
TCP deployment.status=dimensional_verification_required
```

因此转换后的路径为 `execution_permitted=false`。现在可以生成入口、坐标转换、MoveIt 规划、RViz 预览和结构校验，但不能真实自动移动。必须先完成手眼盲验证和 TCP 尺寸/实测验收，再同时打开两个运动门。

## 8. 本次新增或修改的文件

入口数据和 GUI：

```text
src/plasma_gui/src/gui/work_space/opengl/point_v2/point_deal.cpp
src/plasma_gui/src/gui/mainwindow/mainwindow.cpp
src/plasma_gui/src/gui/mainwindow/mainwindow.h
src/plasma_gui/src/gui/work_space/opengl/point_v2/ros_worker.h
```

ROS 消息：

```text
src/third_party/plasma_robot/plasma_robot_interfaces/msg/SprayPath.msg
src/third_party/plasma_robot/plasma_robot_interfaces/msg/EntryMotionStatus.msg
src/third_party/plasma_robot/plasma_robot_interfaces/srv/StartSprayPath.srv
```

坐标转换和运动：

```text
src/third_party/plasma_robot/tools/tf/plasma_path_transform/src/path_transform_node.cpp
src/third_party/plasma_robot/tools/motion/plasma_path_executor/src/entry_motion_planner_node.cpp
src/third_party/plasma_robot/tools/motion/plasma_path_executor/src/path_executor_node.cpp
src/third_party/plasma_robot/tools/motion/plasma_path_executor/src/path_validation.cpp
src/third_party/plasma_robot/tools/motion/plasma_path_executor/launch/automatic_entry_motion.launch.py
```

当前工具 MoveIt 模型：

```text
src/third_party/plasma_robot/plasma_tool_description/urdf/rm_eco65_with_plasma_tool.urdf.xacro
src/third_party/plasma_robot/plasma_tool_description/config/rm_eco65_plasma.srdf
```

测试：

```text
src/third_party/plasma_robot/tools/motion/plasma_path_executor/test/test_path_validation.cpp
```

## 9. 2026-07-24 验证结果

```text
plasma_robot_interfaces: build passed
plasma_path_transform: build passed, 8 tests passed
plasma_path_executor: build passed, 18 tests passed
plasma_gui: build passed
automatic_entry_motion.launch.py: startup passed with motion_enabled=false
当前实时关节状态 + 工具 + 机柜碰撞检查: valid=true, contacts=[]
```

全工作区历史 `rm_bringup` 包仍有既有 flake8 问题；与本次目标包的构建和测试无关。

## 10. 2026-07-25 首次实机执行结果

已在 `5%` 速度、等离子关闭条件下，完成真实机械臂从当时当前位置到预入口的
MoveIt 执行，MoveIt 和控制器均报告成功，操作者已现场确认机械臂真实运动。

本次尚未执行预入口后的分段法向进入和 446 点完整喷涂干运行。现场数据、安全边界、
临时调试授权、状态 QoS 诊断和明天续接步骤见：

```text
REAL_ROBOT_PRE_ENTRY_RESULT_20260725.md
```
