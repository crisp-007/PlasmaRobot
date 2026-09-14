# Gazebo 调试修改记录

针对 `ros2 launch rm_bringup rm_eco65_gazebo.launch.py` 命令的调试修改汇总。

---

## 1. `use_sim_time` 参数不一致

**文件**：`src/plasma_robot/rm_moveit2_config/rm_eco65_config/launch/gazebo_moveit_demo.launch.py`

**问题**：`move_group` 和 `rviz2` 节点未设置 `use_sim_time`，默认为 `False`；而 `robot_state_publisher` 设置了 `use_sim_time: True`。时间不一致导致 `move_group` 认为 `/joint_states` 时间戳过期（超过 1 秒），拒绝执行轨迹。

**原来**：`move_group` 和 `rviz2` 的 `parameters` 列表中没有 `use_sim_time` 条目。

**改为**：
```python
# move_group
{"use_sim_time": True}

# rviz2
{"use_sim_time": True}
```

同时放宽轨迹执行超时参数（防止 sim time 偏差导致提前中止）：
```python
trajectory_execution = {
    "moveit_manage_controllers": True,
    "trajectory_execution.allowed_execution_duration_scaling": 5.0,
    "trajectory_execution.allowed_goal_duration_margin": 5.0,
    "trajectory_execution.allowed_start_tolerance": 0.0,
}
```

---

## 2. `gzserver` 崩溃（Segmentation fault）/ Gazebo 窗口卡死

**文件**：`src/plasma_robot/rm_gazebo/config/gazebo_eco65_description.urdf.xacro`

**问题**：URDF 中同时存在 `<is_static>true</is_static>` 和 `gazebo_ros2_control` 插件。`is_static=true` 让 Gazebo 物理引擎冻结所有关节，而 `gazebo_ros2_control` 需要驱动这些关节，二者直接冲突，导致 gzserver 在加载插件时 Segmentation fault（exit code 139）。

> 注意：`is_static` 最终被保留，因为去掉后关节在某些姿态下因重力问题到不了目标位姿（见第 5 条修复前的临时状态）。

---

## 3. Gazebo 窗口黑屏

**文件**：`src/plasma_robot/rm_gazebo/launch/gazebo_eco65_demo.launch.py`

**问题**：`GAZEBO_MODEL_PATH` 未设置，gzclient 尝试从 `fuel.gazebosim.org` 下载 `ground_plane` 和 `sun` 模型，网络不通导致黑屏。本机 `/usr/share/gazebo-11/models/` 下已有这两个模型。

**原来**：launch 文件中没有设置 `GAZEBO_MODEL_PATH`。

**改为**：在 launch 文件中添加：
```python
from launch.actions import SetEnvironmentVariable

set_gazebo_model_path = SetEnvironmentVariable(
    name='GAZEBO_MODEL_PATH',
    value='/usr/share/gazebo-11/models:' + os.environ.get('GAZEBO_MODEL_PATH', '')
)
```

---

## 4. 重复启动 `robot_state_publisher` 导致 invalid sequence

**文件**：`src/plasma_robot/rm_moveit2_config/rm_eco65_config/launch/gazebo_moveit_demo.launch.py`

**问题**：`gazebo_eco65_demo.launch.py` 已经启动了一个带 `use_sim_time: True` 的 `robot_state_publisher`（节点编号 `-3`），`gazebo_moveit_demo.launch.py` 又启动了第二个不带 `use_sim_time` 的同名节点（编号 `-9`）。两个节点同时发布 `robot_description`，导致 `gazebo_ros2_control` 收到 "invalid sequence number"，进而触发 gzserver segfault，controller_manager 消失。

**原来**：
```python
robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    output="both",
    parameters=[robot_description],
)
# LaunchDescription 中包含 robot_state_publisher
```

**改为**：注释掉该节点，并从 `LaunchDescription` 中移除：
```python
# NOTE: robot_state_publisher is already started by gazebo_eco65_demo.launch.py
# robot_state_publisher = Node(...)
```

---

## 5. `<gazebo reference>` link 名大小写错误

**文件**：`src/plasma_robot/rm_gazebo/config/gazebo_eco65_description.urdf.xacro`

**问题**：URDF 中 link 名为 `Link1`~`Link6`（首字母大写），但 `<gazebo reference>` 标签中写的是 `link1`~`link6`（全小写），Gazebo 找不到对应 link，导致：
- `<gravity>false</gravity>` 未生效 → 重力持续作用 → 机械臂缓慢下坠
- `<material>Gazebo/xxx</material>` 未生效 → Gazebo 中看不到颜色
- 下垂导致下次运动起始位置错误 → 到不了目标位姿

**原来**：
```xml
<gazebo reference="link1">...</gazebo>
<gazebo reference="link2">...</gazebo>
...（link3~link6 同理）
```

**改为**：
```xml
<gazebo reference="Link1">...</gazebo>
<gazebo reference="Link2">...</gazebo>
...（Link3~Link6 同理）
```

---

## 6. `static_transform_publisher` 的 child frame 名写错

**文件**：`src/plasma_robot/rm_moveit2_config/rm_eco65_config/launch/gazebo_moveit_demo.launch.py`

**问题**：所有 URDF 和 SRDF 中根 link 名均为 `baselink`（无下划线），但 `static_transform_publisher` 发布的 TF 是 `world → base_link`（有下划线），导致 TF 树中多了一个孤立的 `base_link` frame，MoveIt 规划坐标系偏移，执行后位姿差一点。

**原来**：
```python
arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
```

**改为**：
```python
arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "baselink"],
```

---

## 遗留问题

**`rm_eco65_gazebo.urdf` 与 `rm_eco65.urdf` 关节限位不一致**：两个文件中 joint1~joint6 的 `lower`/`upper` 值不同，Gazebo 执行时会将超出自身 limit 的目标角度夹在边界，导致位姿偏差。需确认哪份数据为准后统一。

| 关节 | `rm_eco65_gazebo.urdf` (Gazebo) | `rm_eco65.urdf` (MoveIt) |
|------|------|------|
| joint1 | -3.1923 ~ 3.1923 | -3.1067 ~ 3.1067 |
| joint2 | -2.61666 ~ 2.442 | -3.1067 ~ 2.3562 |
| joint3 | -2.61666 ~ 2.61666 | -2.7925 ~ 2.5307 |
| joint4 | -2.61666 ~ 2.61666 | -3.1067 ~ 3.1067 |
| joint5 | -2.96555 ~ 2.96555 | -3.1067 ~ 3.1067 |
| joint6 | 待确认 | -6.2832 ~ 6.2832 |
