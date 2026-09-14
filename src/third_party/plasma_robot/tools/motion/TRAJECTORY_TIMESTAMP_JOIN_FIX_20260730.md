# 审核轨迹拼接时间戳修正（2026-07-30）

## 现场现象

完整入口规划在 8 个关节分支尝试后全部失败，最后错误为：

```text
cannot combine reviewed safe-entry and spray trajectories:
reviewed spray trajectory timestamps are not strictly increasing
```

规划在轨迹拼接阶段中止，没有向机械臂下发运动。入口位置、入口法向、碰撞规划和
速度档位不是本次错误的原因。

## 原因

MoveIt 生成的喷涂轨迹在回零或分段交界处可能包含相邻的同关节姿态点，两个点还可能
具有相同的 `time_from_start`。旧拼接逻辑只检查时间戳，因此把这种不产生机械臂运动的
冗余点误判为危险时间序列。

## 修正规则

拼接逻辑已经提取到：

```text
plasma_path_executor/include/plasma_path_executor/reviewed_trajectory.hpp
plasma_path_executor/src/reviewed_trajectory.cpp
```

安全规则如下：

1. 安全入口末点和喷涂起点仍必须关节连续，误差不得超过 `1e-6 rad`。
2. 相邻点所有关节位置误差不超过 `1e-9 rad` 时，只删除后一个冗余点。
3. 关节位置不同而时间戳不递增时仍严格拒绝，不自动修改或伪造时间。
4. 所有关节位置必须完整且为有限值；拼接后的时间戳必须严格递增。
5. 日志会报告删除的相邻重复点数；拒绝日志会包含喷涂点索引和前后纳秒时间。

回归测试覆盖相同姿态同时间、入口边界重复点、不同姿态同时间、边界不连续和最终
时间严格递增。此修改不改变路径几何、手眼矩阵、TCP、相机参数或机械臂速度。

## 重新验证

构建完成后必须重启 GUI，使入口规划器加载新二进制。旧审核轨迹必须作废并重新规划。
保持等离子关闭，先检查 RViz 三段动画，再按“预入口、安全入口、腔内干运行”逐段确认。
