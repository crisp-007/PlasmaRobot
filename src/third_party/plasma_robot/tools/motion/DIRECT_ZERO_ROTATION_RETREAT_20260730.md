# 腔体直接退出修正记录（2026-07-30）

## 现场问题

旧版“退出腔体”直接倒放安全入口和完整喷涂关节轨迹。由于每个喷涂层包含
`0 -> +180 -> 0 -> -180 -> 0 deg`，倒放会让机械臂在腔体内重新完成这些旋转，之后
才退出。这不符合安全退出要求。

## 当前退出定义

喷涂轨迹的最后一点必须是最后一层的
`PHASE_RETURN_FROM_NEGATIVE + joint6_geometric_deg=0`。退出轨迹按以下顺序生成：

```text
当前最深层 0 deg 终点
  -> 前一层 0 deg 中心
  -> ...
  -> 最浅喷涂层 0 deg 中心
  -> 安全入口
  -> 沿入口外法向到预入口
```

当前最深层位姿是退出规划的起点，不重复加入 waypoint。所有退出 waypoint 使用喷涂
最终点的同一个 Link6 四元数，因此不包含任何喷口轴向旋转。`+180/-180 deg` 的处理点
不会进入退出 waypoint。

## 规划和运动门

退出轨迹不是在点击退出按钮时临时生成。完整规划时，规划器从喷涂轨迹的最终关节状态
出发，对第四段调用 MoveIt 笛卡尔规划，并完成：

1. 连续 IK 和可达性检查。
2. PlanningScene 环境及机器人自碰撞检查。
3. 每个绝对关节步差不超过配置上限。
4. 时间参数和关节轨迹完整性检查。
5. RViz 第四段动画审核。

实际退出前继续检查：完整喷涂已执行完成、机械臂处于喷涂审核终点、当前关节与第四段
审核起点一致、RealMan 空闲、碰撞等级 8、Link6 位姿和关节状态新鲜、运动门已开启。
退出保持喷涂速度档，真实等离子输出始终关闭。

## GUI 操作要求

完整规划成功后，RViz 正常显示四段：

1. 当前位姿到预入口。
2. 预入口到安全入口。
3. 完整腔内喷涂。
4. 保持 `0 deg` 姿态的直接退出。

实机前必须重点观看第四段：腔内只能沿层中心向开口平移，不得出现喷杆轴向旋转。
只有第三段真实干运行完成并停在回零终点后，“退出腔体（退到预入口）”按钮才启用。

## 文件分布

- `plasma_path_executor/include/plasma_path_executor/retreat_path.hpp`：退出几何接口。
- `plasma_path_executor/src/retreat_path.cpp`：逐层回零位姿提取。
- `plasma_path_executor/src/entry_motion_planner_node.cpp`：第四段 MoveIt 规划、审核和执行。
- `plasma_path_executor/test/test_retreat_path.cpp`：退出几何回归测试。
- `plasma_gui/src/gui/mainwindow/mainwindow.cpp`：退出按钮和确认说明。

## 验证结果

```text
colcon build --packages-select plasma_path_executor plasma_gui --symlink-install
结果：通过

colcon test --packages-select plasma_path_executor --event-handlers console_direct+
结果：4 个测试程序、32 项测试全部通过
```

新增测试覆盖：每层只选一个最终回零点、不包含正负 180 度点、层序反向、结尾为安全
入口和预入口、全段姿态恒定、未在最后一层回零或任一层缺少回零时拒绝生成退出轨迹。
