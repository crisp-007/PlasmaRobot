# 连续多层喷涂与任意停车安全退出（2026-07-31）

## 当前行为

安全入口仍是进入腔体前的最后一个现场确认点。操作者确认后，入口规划器一次执行
完整的腔内审核轨迹：

```text
安全入口
  -> 第一喷涂层 0 / +180 / 0 / -180 / 0 度
  -> 后续全部喷涂层
  -> 最后一层 0 度停车
```

第一层不再强制停车，也不再弹出“继续剩余层”的确认。正常完成后，“退出腔体”使用
规划阶段已碰撞检查的 0 度退出轨迹，沿各层中心和入口法向退到预入口。

## 红色停止后的退出

腔内运动期间可随时按 GUI 红色停止按钮。停止链路依次关闭等离子控制输出，并同时
请求入口规划器、正式路径执行器和 RealMan 底层停止。入口规划器进入：

```text
STATE_STOPPING_IN_CAVITY
  -> 至少等待 500 ms
  -> 优先确认新鲜的 arm_current_status=0
  -> 若控制器停止后不再刷新该可选字段，则确认六轴在 0.05 度范围连续稳定 1 秒
  -> 确认 Link6 位姿和六轴关节状态新鲜
STATE_STOPPED_IN_CAVITY
```

只有进入 `STATE_STOPPED_IN_CAVITY` 后，GUI 才启用“从当前停车点安全退出”。退出轨迹
从实际 Link6 停车位姿重新计算，遵守以下规则：

1. 保持停车时的末端四元数和 joint6 喷涂角度，不先在腔内旋转回 0 度。
2. 根据当前姿态恢复侧喷口 TCP，只沿 `cavity_axis` 外向移动到预入口平面。
3. 保留当前横向偏移，不在腔内作横向纠偏；轴线偏离超过 20 mm 时拒绝退出规划。
4. 外退距离超过 500 mm 或已经位于预入口平面外时拒绝规划。
5. 重新检查实时起始关节、MoveIt IK、环境和自碰撞、关节跳变及严格递增时间戳。
6. 规划、起点复核或执行验证失败时保持停车，不继续下发运动。
7. 红色停止即使发生在执行前的实时关节起点复核期间，也优先保留停稳状态；速度设置
   失败同样不会清除停车退出资格。

退出成功后状态回到 `STATE_AT_PRE_ENTRY`。该操作只退出到预入口，不自动返回机械臂
初始位置。

## GUI 操作

```text
规划并审核 RViz 动画
-> 执行到预入口
-> 执行到安全入口
-> 确认后连续执行全部喷涂层
```

正常完成时点击“退出腔体（退到预入口）”。中途需要停止时：

```text
按红色停止
-> 等待按钮显示“从当前停车点安全退出”
-> 确认机械臂已完全停稳且入口外向通道无障碍
-> 点击该按钮
```

不要在状态仍为“等待机械臂完全停稳...”时尝试退出，也不要手动旋转 joint6 后复用
旧轨迹。

## 2026-07-31 现场停稳修复

首次任意停车验收中，机械臂已经停止且 `/joint_states`、Link6 位姿持续更新，但 RealMan
的 `/rm_driver/udp_arm_current_status` 停止刷新，旧实现要求该消息必须在 500 ms 内为 0，
因此 GUI 永久停在“等待机械臂完全停稳”。现场关节速度仅有约 0.00 至 0.04 度每秒的
量化抖动。

现改为：新鲜控制器状态仍具有最高优先级；只有该状态缺失或过期时，才使用按关节名
匹配的连续稳定窗口作为回退。稳定窗口默认 1000 ms、最大位置漂移 0.05 度，同时仍
要求 Link6 位姿和关节消息新鲜。退出前的轴线、距离、碰撞、起点和关节跳变检查均未
删除。人工拖动会重置稳定窗口，并使旧审核轨迹失效。

## 文件分布

- `plasma_robot_interfaces/msg/EntryMotionStatus.msg`：增加停稳确认状态。
- `plasma_path_executor/src/entry_motion_planner_node.cpp`：连续多层执行、停稳门和动态退出。
- `plasma_path_executor/include/plasma_path_executor/retreat_path.hpp`：动态退出几何接口。
- `plasma_path_executor/src/retreat_path.cpp`：保持姿态的轴向外退目标计算。
- `plasma_path_executor/test/test_retreat_path.cpp`：任意停车退出几何测试。
- `plasma_gui/src/gui/mainwindow/mainwindow.cpp`：连续执行提示、停止状态和退出按钮。

## 许可边界

当前仍是低速、真实等离子关闭的实机干运行。`allow_unvalidated_dry_run=true` 只允许
验证运动流程，不表示手眼、TCP、真实点火或生产安全许可已经完成。首次现场验收使用
喷涂速度 5%，并分别验证正常完整退出和至少一次腔内中途停车退出。

## 2026-07-31 完整实机干运行验收

现场使用新采集和重建的轨迹完成了连续全层实机干运行。入口规划器最终报告：

```text
all reviewed spray layers completed continuously and stopped at zero degrees;
plasma remained disabled; direct retreat is ready
```

随后从 GUI 执行“退出腔体（退到预入口）”，机械臂保持喷口 0 度姿态直接沿腔体轴
退出，没有在腔内增加旋转，最终报告：

```text
direct zero-rotation cavity retreat completed and pre-entry verified;
plasma remained disabled
```

本次确认通过的范围为：预入口、法向进入、连续执行全部喷涂层、末层 0 度停车、直接
轴向退出和预入口到位验证。全过程真实等离子保持关闭，因此这是完整机械臂运动流程
验收，不是实际放电喷涂验收。下一阶段需单独确认轴向间隙和补偿，再在具备现场点火
条件后验收等离子输出联动。
