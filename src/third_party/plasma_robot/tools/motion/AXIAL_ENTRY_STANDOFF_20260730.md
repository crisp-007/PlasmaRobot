# 入口轴向外停与三段执行修正（2026-07-30）

## 现场问题

入口的横向位置和方向已经基本准确，但安装真实喷管后，旧流程在进入喷涂前存在
轴向过深甚至圆头接近底部的风险。问题不能通过修改手眼矩阵或相机参数掩盖。

当前工具几何保持不变：

```text
Link6 -> 圆头末端：318 mm
Link6 -> 侧喷口运动 TCP：轴向 310 mm，径向 2 mm
侧喷口运动 TCP -> 圆头：局部 [+8, 0, -2] mm
```

旧入口让圆头位于视觉开口平面，且执行到预入口后会将“进入入口”和“全部喷涂点”
作为一条连续轨迹执行。这样既没有额外轴向误差余量，也没有在入口处停下检查的机会。

## 修正后的入口定义

新增独立参数 `entry_tip_standoff_m`，默认 `0.030 m`。它只控制安全入口，不改变
318 mm 工具长度，也不改变“圆头末端-底部额外距离”。定义如下：

```text
安全入口圆头目标 = 视觉物理开口 + 入口外向轴 * 30 mm
安全入口 TCP      = 安全入口圆头目标 * inverse(T_tcp_to_nozzle_tip)
安全入口 Link6    = 安全入口 TCP * inverse(T_Link6_to_tcp)
预入口             = 安全入口 + 入口外向轴 * 100 mm
```

以视觉物理开口为轴向零点时，当前工具的名义位置为：

```text
安全入口圆头：开口外 30 mm
安全入口 Link6：开口外约 348 mm
预入口 Link6：开口外约 448 mm
```

其中 340/440 mm 只用于说明轴向关系；真实三维位姿仍由完整工具矩阵、入口姿态和
手眼矩阵计算，不能在控制器里手工写成单轴坐标。

GUI 的“路径参数 -> 连续执行 -> 圆头入口外停距离”可在 `0-150 mm` 调整。
首次现场复验必须保持默认 30 mm。只有实测安全入口圆头确实仍在开口外后，才允许
讨论减小该数值。

## 三段实机状态机

完整规划仍一次完成碰撞、IK、关节限位和步差检查，但实际运动拆成三段：

1. 当前位姿到预入口，只走自由空间接近轨迹。
2. 预入口沿入口法向到安全入口，随后强制停车，状态进入 `STATE_AT_ENTRY`。
3. 操作者目视确认圆头仍在真实开口外后，才允许从安全入口执行全部喷涂点。

GUI 的同一个机械臂执行按钮按当前状态依次触发三段，每段都有独立确认弹窗。
第二段服务为：

```text
/plasma_entry_motion_planner/execute_reviewed_entry
```

第三段仍使用：

```text
/plasma_entry_motion_planner/execute_reviewed_dry_run
```

第三段现在只接受 `STATE_AT_ENTRY`，不能再从预入口直接开始。喷涂完成后的第四段退出
保持最终 `0 deg` 回零姿态，沿各层中心由深到浅直接退出，再经过安全入口沿入口法向到
预入口；不得倒放腔内 `+180/-180 deg` 喷涂旋转。真实等离子输出保持关闭。

## 首次现场复验

旧 GUI 中已经生成的路径不包含新入口外停字段，必须作废。重新启动 GUI 后：

1. 保持等离子关闭、物理急停可触达，移动速度使用 10% 档，喷涂速度使用 5% 档。
2. 重新采集、重建并生成“完整喷涂轨迹”。日志必须出现
   `圆头停在物理开口外 30.0mm`。
3. 在 RViz 检查三段轨迹，不是旧版两段轨迹。
4. 第一次点击规划，审核完成后再点击，只执行到预入口。
5. 再次点击只执行到安全入口。此时必须停车，不能继续喷涂轨迹。
6. 目视或用直尺确认圆头仍在真实开口外，记录实际外停毫米数。
7. 本次先不要执行第三段。如果名义外停 30 mm 时圆头仍进入开口，立即停止并记录
   偏内量，优先诊断视觉开口轴向偏差，不得继续增加喷涂深度。

只有第 6 步通过后，下一轮才可确认第三段，并先以等离子关闭方式检查最近喷涂层。

## 代码分布

```text
plasma_robot_interfaces/msg/SprayPath.msg
    入口外停结构化字段
plasma_robot_interfaces/msg/EntryMotionStatus.msg
    STATE_AT_ENTRY 独立停车状态
plasma_gui/.../point_v2/point_deal.cpp
    视觉开口、安全入口和预入口几何生成
plasma_gui/.../mainwindow/mainwindow.cpp
    参数控件、路径发布和三段确认流程
plasma_gui/.../point_v2/ros_worker.h
    安全入口服务客户端
tools/tf/plasma_path_transform/src/path_transform_node.cpp
    根据工具 YAML 反算安全入口 TCP 和 Link6
tools/motion/plasma_path_executor/src/entry_motion_planner_node.cpp
    三段轨迹规划、执行和反向退出
```

## 构建与测试

2026-07-30 已完成：

```text
plasma_robot_interfaces、plasma_tool_description、plasma_path_transform、
plasma_path_executor、plasma_gui：构建通过
plasma_path_transform：11/11 测试通过
plasma_path_executor：22/22 测试通过
```

工作区历史测试报告中 RealMan 供应商原始包仍有格式检查失败，与本次入口修正无关。
