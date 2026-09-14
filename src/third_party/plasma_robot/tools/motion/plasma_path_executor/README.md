# Plasma path executor

该 ROS 2 C++ 包负责用 MoveIt 从当前位姿规划到预入口，并在完整碰撞预检后，按转换后的
法兰位姿进入第一喷涂层和执行腔内喷涂路径。

## 当前喷涂顺序

每一层只有一个喷涂旋转点，TCP 位置保持不变：

```text
0 -> +180 deg    喷涂开启
+180 -> 0 deg    喷涂关闭，返回入口角
0 -> -180 deg    喷涂开启
-180 -> 0 deg    喷涂关闭，返回入口角
移动到下一层   喷涂关闭
```

执行器会检查四个阶段、角度单调性、正负 180 度终点、层内 TCP 固定，以及
返回和换层阶段必须关闭喷涂。它还检查每个内壁喷涂点必须位于 TCP 的 `+Z`
喷射射线上，默认非接触距离至少 `5 mm`、方向误差不超过 `2 deg`。任何字段
不一致都会整条拒绝。

## 安全边界

- 输入：`/plasma/planned_spray_path/base`
- 当前法兰：`/rm_driver/udp_arm_position`
- MoveL：`/rm_driver/movel_cmd`、`/rm_driver/movel_result`
- 停止：`/rm_driver/move_stop_cmd`、`/rm_driver/move_stop_result`
- 碰撞等级：`/rm_driver/set_collision_stage_cmd`、`/rm_driver/collision_stage`
- 关节电流：`/rm_driver/udp_joint_current`
- 全路径碰撞许可：`/plasma/entry_motion/collision_checked_path`
- 状态：`/plasma/path_executor/status`
- 开始服务：`/plasma_path_executor/start`
- 停止服务：`/plasma_path_executor/stop`

节点收到路径只会缓存和检查，不会自动运动。开始前必须同时满足：

1. launch 参数 `motion_enabled=true`；默认值为 `false`。
2. 路径已完成坐标转换且 `execution_permitted=true`。
3. 所有点都有有效 `Link6` 法兰位姿、圆头末端间隙点和侧喷口位姿。
4. 机械臂实时法兰位姿新鲜，并已到达第一喷涂点，或由操作者明确确认已在腔口。
5. MoveIt 已对“当前到预入口”和“预入口到全部喷涂点”完成无碰撞检查，许可未超过 120 秒。
6. RealMan SDK 碰撞等级已设置并回读为 8，实时关节电流新鲜。
7. MoveL 和停止命令/结果话题均已连接。
8. 操作者显式调用开始服务。

若机械臂已经位于第一喷涂点，执行器从第二个点开始下发 MoveL。若操作者在
开始服务中明确确认机械臂已位于腔口，且 launch 显式开启
`entry_approach_enabled`，执行器先以等离子关闭状态移动到第一喷涂点，到位
验证后再继续喷涂路径。该段还受 200 mm 最大距离、20 度接近方向夹角、
5 度喷杆与规划腔体轴夹角以及 20 mm 入口轴线偏差限制；路径必须包含至少
两个不同层 TCP 以建立腔体轴。初始位置到预入口由同包的
`entry_motion_planner_node` 负责 MoveIt 规划、RViz 预览和二次确认。

入口规划器等待新鲜 MoveIt 关节状态的时间由 `joint_state_wait_sec` 控制，默认
为 5 秒，用于吸收驱动和 MoveIt 状态监视器的启动竞态。超时后仍会保持安全锁，
不会用旧关节角规划。安全入口和喷涂轨迹拼接、以及喷涂完成后的严格反向退出，都会
合并回零或换段产生的相邻同关节位置冗余点；不同位置的非递增时间戳仍会拒绝。拼接
规则和现场故障记录见 `../TRAJECTORY_TIMESTAMP_JOIN_FIX_20260730.md`。

三段审核窗口由 `max_plan_age_sec` 控制，默认 600 秒。完整规划成功时开始第一段
计时；机械臂成功到达并验证预入口后，为安全入口段重新计时；成功到达并验证安全入口
后，为腔内喷涂段重新计时。这样现场观察和测量不会消耗后续阶段的确认时间。每一段
开始前仍检查同一路径 ID、审核起点关节、实时 Link6 位姿、机械臂空闲状态和碰撞等级，
超时也仍会锁住运动。安全入口确认后，腔内全部喷涂层作为一条审核轨迹连续执行，
不再在第一层停车或要求继续确认。详见 `../STAGED_REVIEW_FRESHNESS_20260730.md`。

腔内运动期间红色停止会同时请求入口规划器停止、正式路径执行器停止，并直接发布
RealMan 停止命令。入口规划器至少等待 500 ms，且只有在控制器空闲、Link6 位姿和
六轴关节状态都新鲜时才进入 `STATE_STOPPED_IN_CAVITY`。随后“从当前停车点安全退出”
会根据实际停车位姿重新生成 MoveIt 轨迹：保持当前末端姿态，只沿腔体外向轴移动到
预入口平面，不先回到 0 度、不横向修正。重新规划或安全检查失败时机械臂保持停车。
详见 `../CONTINUOUS_SPRAY_AND_STOP_RETREAT_20260731.md`。

当前位置到预入口以及预入口到安全入口使用独立的 `approach_velocity_scaling` 和
`approach_acceleration_scaling`，由 GUI 的“移动速度”10%/20% 档直接控制。
安全入口后的腔内喷涂使用 `velocity_scaling`/`acceleration_scaling`，由“喷涂速度”
5%/10% 档直接控制；反向退出保持较低的喷涂档。两个参数不再自动联动。
入口规划器在每段执行前通过 ROS 参数服务设置并确认
`rm_control.final_settle_speed_percent`；`rm_control` 在新轨迹开始时锁存速度，
运动中的参数变化只对下一条轨迹生效。参数服务不可用或拒绝设置时禁止执行。
改变任一档位后入口规划器会重启，必须重新检查 RViz 并重新规划。

每个直接下发的 MoveL 小步都记录该步开始前的关节电流。任一关节电流幅值增加超过
`1500 mA` 会发送停止命令。完整防碰撞链路见 `../COLLISION_SAFETY_20260724.md`。

## 当前限制

真实等离子输出尚未接入安全联锁。因此本版本只接受 `dry_run=true`，状态消息
保留路径要求的 `desired_plasma_enabled`，但
`plasma_output_enabled` 始终为 `false`。这能验证机械臂运动顺序，又不会打开
真实等离子输出。

运动 TCP 已定义为真实侧喷口中心。每层正负 180 度旋转期间，该点的位置必须
保持不变；圆头末端只作为 `TCP +X 8 mm、-Z 2 mm` 的底部间隙检查几何，
并不是另一个需要现场寻找的“安全点”。转换节点会同时生成圆头末端位置和
侧喷口位姿；本执行器会拒绝有效标志缺失的旧路径。手眼和完整工具标定尚未
验收，因此上游路径仍保持 `execution_permitted=false`，本执行器不会运动。

## 启动

默认检查模式：

```bash
ros2 launch plasma_path_executor plasma_path_executor.launch.py
```

完成全部现场验收后，才可由操作者明确打开低速运动门：

```bash
ros2 launch plasma_path_executor plasma_path_executor.launch.py \
  motion_enabled:=true entry_approach_enabled:=true speed_percent:=5
```

开始请求仍必须明确声明干运行：

```bash
ros2 service call /plasma_path_executor/start \
  plasma_robot_interfaces/srv/StartSprayPath \
  "{path_id: '', dry_run: true, approach_from_confirmed_entry: true}"
```

禁止通过手工伪造 `execution_permitted=true` 绕过标定验收。

## 单层实机调试

`single_layer_commission_node` 用于验收机械臂能否从人工放置的当前入口姿态，
完整执行一层：

```text
0 -> +180 -> 0 -> -180 -> 0
```

它读取当前 `Link6` 位姿和末端 YAML，以当前侧喷口 TCP 作为固定旋转点，按
`5 deg` 间隔计算法兰目标。准备服务只发布路径，不发送运动；正式执行仍通过
本包执行器的入口位姿、驱动话题、停止、超时和到位检查。调试路径会显式标记
喷涂阶段，但执行器只接受 `dry_run=true`，因此真实等离子输出始终关闭。

所有门默认关闭。现场确认后才使用：

```bash
ros2 launch plasma_path_executor single_layer_commission.launch.py \
  commissioning_enabled:=true motion_enabled:=true speed_percent:=5
```

机械臂由人工放到测试入口姿态并确认周围有完整旋转空间后，先准备路径：

```bash
ros2 service call /plasma_single_layer_commission/prepare std_srvs/srv/Trigger "{}"
```

准备不会移动机械臂。核对执行器状态为路径就绪后，再单独开始：

```bash
ros2 service call /plasma_path_executor/start \
  plasma_robot_interfaces/srv/StartSprayPath "{path_id: '', dry_run: true}"
```

随时停止：

```bash
ros2 service call /plasma_path_executor/stop std_srvs/srv/Trigger "{}"
```

该入口仅用于当前姿态的单层调试，不替代相机路径、手眼验收或正式 TCP 验收，
也不负责机械臂初始位置到入口点。

2026-07-24 的首次真实机械臂完整单层干运行结果记录在：

```text
../SINGLE_LAYER_RESULT_20260724.md
```

单层通过后的两层轴向推进测试入口为：

```text
launch/multi_layer_commission.launch.py
../MULTI_LAYER_COMMISSION_20260724.md
```

首次两层真实机械臂干运行的通过记录为：

```text
../MULTI_LAYER_RESULT_20260724.md
```

## GUI 接入

`plasma_gui` 启动时同时启动坐标转换和本执行器，默认明确传入
`motion_enabled:=false`。机械臂执行页订阅状态话题，开始按钮调用正式开始
服务，停止按钮调用正式停止服务；主急停还会直接发布 RealMan 停止命令作为
冗余。旧 `DemoRotationExecutor` 不再由 GUI 创建或调用；其诊断源码仍保留在
`src/module/RobotArm/`，但已从 GUI 构建列表排除。

在手眼和完整工具链验收前，不得把 GUI 注册 launch 中的运动门改为 true。
验收后开启运动门也不会绕过路径许可、入口位姿、驱动就绪和 dry-run 检查。

2026-07-31 起，GUI 实机流程为：当前位置到预入口、预入口到圆头保持在视觉开口外
的安全入口、安全入口后连续执行全部喷涂层。安全入口通过 `execute_reviewed_entry`
独立执行并进入 `STATE_AT_ENTRY`；只有在该状态下，连续喷涂服务才接受完整腔内轨迹。
当前默认圆头入口外停距离为 30 mm，详见 `../AXIAL_ENTRY_STANDOFF_20260730.md`。
