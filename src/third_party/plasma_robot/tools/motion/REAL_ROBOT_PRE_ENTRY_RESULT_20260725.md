# 机械臂整体运动首次实机记录（2026-07-25）

## 1. 本次目标和结论

本次只验证等离子软件生成的轨迹能否经过坐标转换、MoveIt 规划和碰撞检查后，
驱动真实机械臂运动。精度验收和真实等离子输出不在本次范围内。

结论：整体链路已经走到真实机械臂并成功执行到预入口，操作者已在现场亲眼确认
机械臂真实运动。已验证链路为：

```text
GUI 生成重建1路径
  -> 相机坐标转换到 baselink
  -> 当前位姿到预入口的 MoveIt 规划
  -> 预入口到全部喷涂点的 IK、关节步差和碰撞检查
  -> RViz 动画确认
  -> 5% 速度执行到预入口
  -> MoveIt 和控制器均报告 SUCCEEDED
```

这证明软件到机械臂的整体运动通路可用，但不代表手眼精度、TCP 精度或完整喷涂
运动已经验收。

## 2. 本次路径和规划数据

```text
路径名称: 重建1路径
喷涂位姿数: 446
路径源坐标系: camera_depth_optical_frame
预入口方向: 入口外法向
预入口距离: 100 mm
运动速度: 5%
等离子输出: 始终关闭
```

完整碰撞规划结果：

```text
首次完整规划: 候选 1 通过，最大相邻关节步差 joint4 = 1.546846 deg
到达预入口后复核规划: 候选 3 通过，最大相邻关节步差 joint4 = 1.519167 deg
```

预入口到入口的方向已经由操作者在 RViz 中确认是沿开口外法向进入，不再从下方
平移接近。

## 3. 已完成和未完成的边界

已经完成：

1. `重建1路径` 的坐标转换。
2. 当前真实关节状态参与的完整 IK 和碰撞规划。
3. RViz 中完整轨迹动画检查。
4. 真实机械臂从当时当前位置运动到预入口。
5. MoveIt 与真实控制器的成功反馈核对。

尚未执行：

1. 从预入口到入口的最多 5 mm/步法向进入。
2. 从入口到第一喷涂层的分段进入。
3. 446 点完整喷涂轨迹的实机干运行。
4. 真实等离子启停和带等离子喷涂。
5. 手眼矩阵盲验证、TCP 实测验收和最终精度调整。

明天的第一目标不是打开等离子，而是在等离子关闭状态下完成第 1 至第 3 项，
由操作者亲眼确认整段机械运动可以走通。

## 4. 停止前的机械臂快照

本次执行结束时，机械臂停在预入口附近。记录到的 Link6 位姿为：

```text
position baselink [m]: [-0.439740, -0.012684, 0.386711]
orientation [x,y,z,w]: [0.995184, -0.013719, 0.005368, 0.096904]
joint [deg]: [-8.029, -1.098, -77.809, -13.129, 100.954, 83.355]
controller state: idle
system error: 0
joint error: 0
```

以上数据仅是 2026-07-25 停止前快照。明天必须重新读取实时关节和 Link6 位姿，
不能直接假定机械臂仍在该位置。停止时不发送回撤命令，机械臂保持预入口位置。

## 5. 本次临时调试授权

为了先验证整体运动通路，本次通过显式启动参数临时放宽未验收标定限制：

```text
path_transform require_validated_handeye=false
path_transform require_validated_tcp=false
path_executor motion_enabled=true
path_executor speed_percent=5
entry planner motion_enabled=true（执行到预入口时）
```

手眼和 TCP 的 YAML 验收状态没有修改，GUI 中的默认运动门也没有修改：

```text
hand-eye strict_quality_accepted=false
TCP validated=false
GUI path_executor motion_enabled=false
GUI entry_motion motion_enabled=false
```

正式路径执行器仍只接受 `dry_run=true`，本次以及明天的完整运动验证都必须保持
`plasma_output_enabled=false`。

## 6. 状态反馈诊断记录

入口执行成功后，入口规划器曾返回：

```text
controller reported success but current Link6 pose is missing or stale
```

控制器和 MoveIt 实际均明确报告 `SUCCEEDED`，独立 SDK 也能读取当前位置。现场进一步
确认厂家 ROS 状态话题使用 `RELIABLE` QoS；默认命令行订阅方式可能收不到数据，
从而造成 UDP 状态停止更新的误判。诊断时应显式使用：

```bash
ros2 topic echo \
  --qos-profile system_default \
  --qos-reliability reliable \
  /rm_driver/udp_arm_position
```

同样的 QoS 参数适用于：

```text
/joint_states
/rm_driver/udp_joint_current
```

显式使用 `RELIABLE` 后，位姿、关节和电流均能约 200 Hz 连续更新。后续应修正节点
内部状态订阅的 QoS 或状态新鲜度判断，但该反馈问题不改变本次控制器执行成功结论。

## 7. 明天续接步骤

1. 确认现场急停、使能和周围障碍物状态，等离子保持关闭。
2. 启动 GUI 并完成自检，不修改原作者调好的相机参数。
3. 重新生成 `重建1路径`，核对 446 个位姿和 100 mm 预入口距离。
4. 使用与本次相同的显式调试授权，速度保持 5%。
5. 重新读取实时关节和 Link6 位姿，确认机械臂是否仍在预入口允许误差内。
6. 重新执行完整 IK、关节步差和碰撞规划，并在 RViz 检查法向进入路径。
7. 操作者现场确认后，调用正式路径执行器执行 `dry_run=true`。
8. 观察最多 5 mm/步的法向进入、进入第一层以及完整 446 点运动。
9. 全程监控 Link6 状态新鲜度、六关节电流、碰撞等级 8，并确认
   `plasma_output_enabled=false`。
10. 完成后记录最终状态、实际执行点数和任何被拒绝的点。

只有完整干运行走通后，才进入手眼矩阵、TCP、喷涂距离和轨迹精度调整。真实等离子
输出必须另行验收和授权，不能因为本次机械臂运动成功而自动开启。

## 8. 相关文档

```text
tools/motion/AUTOMATIC_ENTRY_WORKFLOW_20260724.md
tools/motion/COLLISION_SAFETY_20260724.md
tools/motion/REAL_SLICE_PATH_PREVIEW_20260724.md
src/plasma_gui/src/temp_ref/HANDOFF_NEW_CHAT_20260722.md
```

## 9. 当晚结束状态

记录完成后已正常关闭以下临时调试进程：

```text
motion_enabled=true 的 plasma_path_executor
require_validated_handeye=false / require_validated_tcp=false 的 path_transform
motion_enabled=false 的入口规划器及其 MoveIt、RViz
独立启动的 rm_driver
临时 ROS 状态话题 echo
```

关闭过程只向软件进程发送正常退出信号，没有向机械臂下发回撤、入口进入或其他
新运动命令。机械臂保持停止在预入口位置。GUI 和相机进程未改动，明天开始前仍应
重新启动 GUI，并重新执行自检和实时状态核对。

## 10. 2026-07-25 完整干运行续试

正式路径执行器增加了“预入口附近分段纠正”能力并重新构建：当前位置只有在距
预入口不超过 30 mm、相对入口轴横向偏差不超过 10 mm、姿态偏差不超过 10 度时，
才允许先以最多 5 mm/步纠正到预入口，再沿入口法向进入。原有 2 mm/3 度逐步到位
验收、电流增量、碰撞等级和状态新鲜度保护均保持不变。

构建和测试结果：

```text
plasma_path_executor: build passed
tests: 23
errors: 0
failures: 0
skipped: 0
```

执行前重新进行全路径碰撞规划，结果为：

```text
path_id: 重建1路径
spray poses: 446
branch attempt: 1
max joint step: joint4 = 1.248246 deg
speed: 5%
dry_run: true
plasma_output_enabled: false
```

真实机械臂随后完成：

```text
预入口 -> 腔体入口: 20/20 个分段完成
腔体入口 -> 第一喷涂层: 6/6 个分段完成
第一喷涂层到位验收: 通过
446 点喷涂轨迹: 发送第 1 个点后被电流保护停止
```

停止原因：

```text
joint 3 current magnitude rose 1508 mA; possible collision
configured threshold: 1500 mA
executor state: 5 (error)
current_index: 0
RealMan stop result: success
plasma_output_enabled: false
```

停止后 Link6 位姿稳定在：

```text
position baselink [m]: [-0.449254, -0.038357, 0.262549]
orientation [x,y,z,w]: [0.993344, -0.018206, -0.034793, 0.108284]
joint current after stop [mA], approximately:
[-30 to 10, -780 to -720, 1650 to 1700, -330 to -300, -165 to -130, 90 to 125]
```

本次不能据日志单独判断 1508 mA 增量是真实接触，还是第一喷涂点运动产生的瞬态
负载。继续前必须由现场操作者确认末端、喷杆和外壳是否发生接触。不得直接提高
1500 mA 阈值或关闭电流保护；若现场确认无接触，应先记录首个喷涂小步的完整电流
波形和命令前基线，再决定是否改为带持续时间/多样本滤波的保护判据。

## 11. 整臂异常绕转及旧执行链路停用

继续实机干运行时，机械臂在喷涂轨迹附近出现了未经 RViz 审核的整臂绕转。该姿态
和运动均不正确，现场已停止机械臂，等离子始终关闭。停止后六轴仅有传感器分辨率
范围内波动，确认机械臂已经静止。停止时关节约为：

```text
joint [rad]: [-0.08489, -0.08029, -1.86481, 0.43065, 1.79150, 1.52850]
```

根因不是 RViz 中已审核关节轨迹本身，而是旧执行链路没有执行这条轨迹：

```text
EntryMotionPlanner 生成并检查 MoveIt 连续关节轨迹
  -> 旧 PathExecutor 丢弃关节轨迹
  -> 逐点向 RealMan 下发 Cartesian MoveL 末端位姿
  -> 厂家控制器再次逆解并可能选择另一关节支路
  -> 实机姿态和 RViz 审核姿态不一致
```

因此，第 1 节和第 10 节所述“整体通路可用”只证明命令能够到达控制器，不再作为
完整喷涂运动通过验收的依据。旧 `PathExecutor` 的真实 Cartesian MoveL 路径已硬锁：
只要 `motion_enabled=true` 就拒绝启动，不能再用于真实运动。

新增的安全执行方式为：由 `EntryMotionPlanner` 保存 RViz 显示的
`after_entry_preview_`，通过以下服务直接执行同一条 MoveIt 关节轨迹：

```text
/plasma_entry_motion_planner/execute_reviewed_dry_run
```

执行前必须同时满足：规划未过期、路径许可通过、运动门开启、RealMan 碰撞等级 8
反馈新鲜、Link6 位姿新鲜、机械臂位于审核过的预入口，以及当前六轴和审核轨迹起点
逐轴绝对误差不超过 2 度。这里不把相差 360 度视为同一关节位置，避免绕转状态被
错误放行。真实等离子仍不接入。

GUI 已改为在到达预入口后调用上述审核关节轨迹服务，不再调用旧
`/plasma_path_executor/start`。新增状态 `EntryMotionStatus.STATE_COMPLETED=7` 用于报告
完整关节轨迹干运行完成。

修复后的构建和测试结果：

```text
plasma_robot_interfaces: build passed
plasma_path_executor: build passed
plasma_gui: build passed
plasma_path_executor tests: 23, errors: 0, failures: 0, skipped: 0
```

修复后尚未重新执行任何实机运动。已用 `motion_enabled=false` 从当前真实关节状态
重新规划两段 RViz 动画：候选 2 通过，喷涂段最大相邻步差为 joint4 的 1.248246 度。
返回预入口段各轴变化连续；喷涂段 joint1 至 joint5 范围连续，joint6 按每层
“正 180 度/回零/负 180 度/回零”累计运动约 2165 度。操作者必须确认返回预入口和
后续完整轨迹的整臂姿态连续、只有预期的末端旋转，并且不碰外壳、机柜及其他结构，
之后才能另行授权低速实机验证。

## 12. 审核关节轨迹首次实机干运行及持续报警

操作者确认两段 RViz 动画后，机械臂以 5% 速度执行到预入口并通过现场确认。随后
直接执行同一条审核关节轨迹，等离子全程关闭。控制链路记录为：

```text
path_id: 重建2路径
reviewed trajectory points: 1477
controller duration: 17.514807 s
rm_group_controller result: SUCCEEDED
MoveIt execution result: SUCCEEDED
```

实机确实发生了运动，但运动结束后机械臂持续发出报警声。执行器的最终 Link6 到位
验收失败：

```text
translation_error_mm: 10.422599
rotation_error_deg: 58.744314
```

操作者随后关闭机械臂电源。关机前已调用入口规划器停止服务并使规划失效；关机后也
关闭了 `automatic_entry_motion.launch.py` 启动的 `rm_control`、MoveIt、入口规划器和
RViz，避免残留运动节点在下次上电时重连。`rm_driver` 在机械臂断电后因连接断开以
`-11` 退出，这是断电后的软件退出结果，不足以说明报警原因。

本次没有在报警发生前持久记录 `/rm_driver/udp_rm_err` 和
`/rm_driver/udp_joint_error_code`，因此不能把报警准确归类为碰撞、过流、位置跟踪、
指令阶跃或其他控制器错误。另已确认厂家 `rm_control` 的 `SUCCEEDED` 只表示全部透传
关节点已发送完成；它没有根据 UDP 控制器错误、关节错误和最终到位反馈决定 action
结果。因此上述 `SUCCEEDED` 不能作为实机正确完成的依据。

再次上电前必须先加入被动错误记录，并在不开启运动门的状态下读取控制器与六轴错误
码。未确定报警代码、未修正 action 成功判据、未解释 10.4 mm/58.7 度最终偏差之前，
不得再次执行完整腔内轨迹，也不得直接清错后继续。

## 13. 重启后的错误预览与入口分支修复

再次上电后确认了三个独立问题：

1. 持续蜂鸣来自软件每 5 秒重复设置碰撞等级 8，现已改为连接后只设置并验收一次。
2. 厂家 `rm_control` 原先使用 `follow=false`，会丢弃来不及跟随的透传点，现已改为
   `follow=true`。
3. 重启驱动后 RealMan 实时 UDP 推送曾中断，`/joint_states` 没有新鲜数据；MoveIt
   因此可能用无效起点生成明显折叠的入口轨迹。现场发现后立即下发 `move_stop`，没有
   执行腔内轨迹，等离子保持关闭。

入口规划器现增加以下硬约束：

- 规划前 1 秒内必须取得新鲜 MoveIt 关节状态，否则拒绝规划；
- 预入口位姿先以当前真实六轴作为 IK 种子求最近关节解，不再把自由位姿目标交给
  OMPL 随机选择远端逆解分支；
- 入口段和腔内段均检查逐点绝对关节步进；
- 对全部完整可行候选计算入口累计关节运动量，只保留运动量最小的候选。

重新下发 RealMan 实时 UDP 推送配置后，`/joint_states` 恢复约 200 Hz。新版本以
`motion_enabled=false` 生成预览成功：

```text
path_id: 重建2路径
nearby-IK attempts: 8
approach accumulated joint travel: 93.636690 joint-deg
approach max adjacent step: joint6, 1.122997 deg
spray max adjacent step: joint4, 1.273863 deg
collision check: passed
```

构建通过，`plasma_path_executor` 测试结果为 23 tests、0 errors、0 failures、0 skipped。
该轨迹目前仅供 RViz 审核；在操作者确认整臂姿态和入口方向正确前，运动门保持关闭。

## 14. 正确入口轨迹确认与控制器跟随方式验证

操作者已确认近邻 IK 修复后的 RViz 入口轨迹正确。随后以 5% 速度进行了多次仅到
预入口的实机验证，等离子始终关闭，腔内轨迹未下发。机械臂每次都沿正确方向移动，
剩余入口累计关节运动由 81.85 度逐步下降到 45.38 度，说明轨迹和实机命令方向一致。

厂家 CANFD 透传不能用“重复同一点直到到位”的方式闭环：无论 `follow=true` 还是
`follow=false`，控制器都会在目标不再变化后保留约 0.5 至 1.1 度的 joint6 跟随差，
重复发送相同目标不会继续收敛。已验证并停用该实验方式。`rm_control` 目前增加了一个
默认关闭的 `blocking_movej_mode`；仅本项目的 `automatic_entry_motion.launch.py`
启用它，以 5% 速度逐个执行 MoveIt 已审核的原始关节路点。已在 0.5 度内的重复起点
会跳过，每个实际路点等待厂家返回，反馈误差超过 2 度、单点超过 30 秒或厂家返回
失败都会中止 action。厂家原始 ECO65 control launch 未修改。

本轮还补齐了 `rm_control` 对 `sensor_msgs` 的构建依赖，并加入：

- `/joint_states` 新鲜度门（200 ms）；
- CANFD 实验模式的审核点、前瞻点和最终 5% MoveJ 收敛保护；
- 阻塞 MoveJ 逐路点执行模式；
- MoveJ 结果和最终反馈验收；
- MoveIt 执行超时放宽，以容纳真实低速执行。

构建和回归测试结果：

```text
rm_control: build passed
plasma_path_executor: build passed
plasma_path_executor tests: 23, errors: 0, failures: 0, skipped: 0
```

当前实机阻塞不是轨迹问题。为结束一次卡住的 CANFD 试验，软件发送了厂家
`/rm_driver/move_stop_cmd`，控制器随后持续上报：

```text
arm_current_status: 9  # RM_STOP_E
joint_en_flag: [true, true, true, true, true, true]
joint_error: [0, 0, 0, 0, 0, 0]
MoveJ result: false
MoveJ SDK error: 1
```

`set_arm_continue` 对状态 9 无效；四代控制器的 `emergency_stop state=false` 在本机
返回 SDK 错误 `-4`；`clear_system_err` 后状态仍为 9。必须在示教器上解除停止并重新
使能，使 `arm_current_status` 回到 0，之后从当前真实关节状态重新规划预入口。不要在
状态 9 下反复发送运动命令。

入口规划器随后增加了 RealMan 运行状态硬门：订阅
`/rm_driver/udp_arm_current_status`，只在 500 ms 内收到状态 `0 (RM_IDLE_E)` 时允许
执行预入口或审核后的腔内轨迹。状态 9、状态反馈过期或话题缺失都会在调用 MoveIt
之前拒绝执行并报告当前状态。修改后构建通过，测试仍为 23/23 通过。

## 15. STOP(9) 恢复流程作为后续优化项

2026-07-25 再次重连厂家驱动后确认：实时关节反馈、六轴使能状态和碰撞等级 8 均
正常，但控制器仍保持 `arm_current_status=9`。厂家 SDK 文档说明
`rm_set_arm_emergency_stop(false)` 是四代控制器接口；当前三代 ECO65 返回 `-4` 表示
不支持该接口。因此本阶段不继续改动轨迹，也不绕过入口规划器的状态硬门。

当前固定恢复步骤为：在示教器解除停止、清除报警并重新使能；确认状态回到
`0 (RM_IDLE_E)`；重新读取真实关节；重新规划预入口和腔内轨迹；最后才允许 5% 速度
干运行。后续单独优化三代控制器的停止方式和恢复操作，避免正常调试再次进入不可由
ROS 恢复的 `STOP(9)`。在优化完成前，普通可恢复中断应优先使用暂停/继续语义，
`move_stop` 只作为需要人工复位的紧急保护。
