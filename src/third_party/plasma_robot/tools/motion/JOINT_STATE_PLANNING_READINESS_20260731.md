# 入口规划关节状态与轨迹时间修正（2026-07-31）

## 现象

入口规划器冷启动后的第一次规划经常失败：

```text
fresh joint state remained unavailable after 5 seconds
Failed to fetch current robot state
```

再次点击有时能够进入规划，但随后可能在分段轨迹拼接时报错：

```text
reviewed entry trajectory timestamps are not strictly increasing
```

## 诊断证据

故障发生时确认：

```text
/joint_states 发布者：1
发布频率：约 200 Hz
关节名称：joint1 ... joint6
消息时间戳：与系统 ROS 时间一致
use_sim_time：false
```

机械臂驱动、GUI 和 `rm_control` 都持续收到关节数据。问题不是机械臂断线或系统
时钟错误，而是入口规划器依赖的 MoveGroupInterface 内部 CurrentStateMonitor 在
冷启动后的可靠订阅端点已经被发现、但首帧没有及时交付。服务已经可见不代表该内部
监视器已经可用，因此旧代码会先失败一次，等待一段时间后重试才可能成功。

第二个问题来自 `computeCartesianPath()` 返回的分段轨迹。部分相邻点具有重复或不严格
递增的 `time_from_start`，直接拼接第一层和剩余层会被审核器正确拒绝。

## 修正

入口规划器现在：

1. 使用与 `rm_control` 相同的 `SensorDataQoS` 直接订阅 `/joint_states`。
2. 使用单调时钟记录本机实际接收时间，要求关节状态在 500 ms 内更新。
3. 检查规划组所需的六个变量均存在、数值有限且在 MoveIt 关节范围内。
4. 从该快照构造显式 `moveit::core::RobotState`，用于预入口 IK、OMPL 起点、笛卡尔
   路径起点以及执行前审核关节核对。
5. 不再调用 MoveGroupInterface 的 `getCurrentState()` 或 `getCurrentPose()`；法兰位姿
   使用已有的实时 `/rm_driver/udp_arm_position` 安全订阅。
6. 每段笛卡尔轨迹先删除严格相同的相邻关节点，再使用 MoveIt
   `IterativeParabolicTimeParameterization` 按当前移动/喷涂速度重新计算速度、加速度和
   时间戳，最后验证所有时间戳严格递增后才允许拼接。

等待参数 `joint_state_wait_sec` 从 5 秒调整为 15 秒。它只用于冷启动时等待规划器自己的
关节订阅收到首帧，不会放宽 500 ms 的运行时新鲜度门限。

## 验收

在停止全部入口规划子进程、保留真实 `rm_driver` 后重新冷启动，第一次调用规划服务
直接成功，无需预热重试：

```text
approach/safe-entry/first-layer/remaining-spray/direct-retreats collision-checked
approach max joint step: 4.491989 deg on joint6
cavity max joint step: 1.936232 deg on joint3
first-layer retreat waypoints: 2
full retreat waypoints: 3
```

测试实例使用 `motion_enabled=false`，本次验收只进行了规划，没有向机械臂发送运动。
`plasma_path_executor` 的 39 项单元测试全部通过。

