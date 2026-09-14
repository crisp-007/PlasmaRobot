# 新对话交接：整机流程融合

交接时间：2026-07-23 17:43 CST

## 使用规则

1. 本文档记录交接时刻的代码和现场状态。
2. 新对话中用户提供的更新信息具有更高优先级；设备位置、连接状态、工具方向或任务目标发生变化时，必须按最新信息修正文档假设。
3. 任何任务中断后，从本文档和 `INTEGRATED_WORKFLOW.md` 中最后一个未完成项继续，不能静默跳过。
4. 工作区包含大量用户历史修改和未跟踪文件，禁止 `git reset --hard`、`git checkout --`、清理未跟踪文件或回滚无关内容。

## 机械臂模块搭建规则（用户于 2026-07-22 确认）

1. 当前目标是让等离子软件中的机械臂部分能够正常使用。
2. 正式机械臂代码统一放在 `src/third_party/plasma_robot/`，保持该目录结构清晰、内容精简。
3. `/home/larusxu/CodeSpace/robot_arm_handoff_20260721/` 仅作为旧机械臂实现、数据和文档的参考源。
4. 禁止把旧交接目录整体复制进 `src/third_party/plasma_robot/`；每次只筛选当前任务实际需要的代码、配置或资源。
5. 机械臂能力必须按用户布置的任务逐部分搭建、接入和验证。完成当前部分前，不提前批量迁移后续模块。
6. 每次迁移都要明确该部分的职责、依赖、运行入口和验证结果，并避免带入历史数据、构建产物或无关实验脚本。
7. 正式集成代码的实现语言保持一致，优先使用 C 或 C++；旧交接包中的 Python 脚本只作为算法、接口和实机验证参考，不直接作为等离子软件的正式运行依赖，除非用户针对某一部分另行确认。
8. 不负责机械臂从初始位置移动到腔口或喷涂入口。转换后路径的第一个点定义为入口后的喷涂起始位姿，如何到达该点由外部流程负责。
9. 本模块负责给出入口后的连续 TCP/法兰位姿，以及每个点的顺序、层号、运动阶段和等离子开关语义，让下游机械臂控制模块知道“怎么运动、何时喷涂”。
10. 机械臂工具功能统一归档到 `src/third_party/plasma_robot/tools/`：手眼采集、求解、验证和矩阵配置放入 `tools/eye_hand/`；TF 发布、坐标变换、检查及路径变换功能放入 `tools/tf/`。
11. `plasma_robot_interfaces` 属于跨模块共享消息接口，保留为独立 ROS 包，不塞入工具目录。`plasma_path_transform` 已迁入 `tools/tf/`，并已同步验证 colcon 发现、launch、GUI 包名依赖和构建结果。

## 当前主任务

当前阶段已完成机械臂真实连接自检，以及相机路径到机械臂基坐标系的 C++ 坐标转换基础链路。主线仍是打通整机操作者流程：

```text
系统自检
-> 相机采集
-> 残腔重建与术区选择
-> 路径规划
-> 机械臂执行
-> 等离子处理
-> 安全结束
```

L515 暂用手眼矩阵已经接入；下一步是严格验收该矩阵并确认真实等离子喷枪 TCP/末端 URDF。在两项均验收前只允许转换预览，不允许把路径交给运动执行器。

详细任务清单见：`src/plasma_gui/src/temp_ref/INTEGRATED_WORKFLOW.md`。

## 2026-07-30 入口规划启动竞态与退出轨迹修复

- 现场曾出现 `fresh joint state is unavailable`。检查确认 RealMan 驱动随后以
  约 `197 Hz` 发布完整六轴 `/joint_states`，消息年龄约 `4 ms`；问题是
  MoveIt 状态监视器刚启动时原先只等待硬编码的 `1 s`。
- `entry_motion_planner_node` 新增 `joint_state_wait_sec`，默认 `5.0 s`，同时
  用于开始规划和执行前核对审核轨迹起点。超过 5 秒仍没有新状态时继续拒绝
  规划/运动，不会使用旧关节角。
- 完整规划随后暴露退出轨迹误判：喷涂的回零/换段位置允许相邻的同关节位置点，
  这些冗余点可以具有相同时间戳。旧反向函数要求原始每一点都严格增时，因此
  错误报告 `reviewed spray trajectory cannot be safely reversed`。
- 反向退出生成现在只合并相邻且六轴位置在 `1e-9 rad` 内完全相同的冗余点；
  不同关节位置若时间戳不严格递增仍立即拒绝，并在状态中给出具体失败原因。
- 2026-07-30 实机数据重新规划通过：路径
  `重建1路径_1785406650_181041282`，审核轨迹共 `1620` 点，8 个附近 IK
  分支均完成检查，接近段最大步长 `1.122997 deg (joint6)`，喷涂段最大步长
  `1.136349 deg (joint4)`。当前状态为 `STATE_PREVIEW_READY`，只发布 RViz
  动画，未下发机械臂运动。
- `plasma_path_executor` 重新构建成功，现有测试结果为 `23 tests, 0 failures`。

## 2026-07-30 预入口段独立提速

- 本节旧的“喷涂档自动乘二”规则已经作废。GUI 现在显示两个独立下拉框：移动速度
  10%/20%，喷涂速度 5%/10%。
- 移动速度控制“当前位置 -> 预入口 -> 30 mm 外停安全入口”；喷涂速度控制
  “安全入口 -> 全部腔内喷涂点”。腔内反向退出保持喷涂档。
- MoveIt 使用独立的 `approach_velocity_scaling` 与
  `approach_acceleration_scaling` 生成移动轨迹，使用 `velocity_scaling` 与
  `acceleration_scaling` 生成喷涂轨迹。由于现场 `rm_control` 使用逐点阻塞 MoveJ，
  入口规划器还会在每段执行前设置并确认对应的控制器速度。设置失败时保持运动锁定。
- `rm_control` 在每条新轨迹开始时读取并锁存速度，运动中修改参数只影响下一条
  轨迹，避免一条轨迹中途突然变速。

## 2026-07-23 完成：真实机械臂自检与坐标转换

### 2026-07-23 末端描述与试验 TCP

- 新增独立 ROS 2 描述包：
  `src/third_party/plasma_robot/plasma_tool_description/`，不修改 RealMan
  原厂 `rm_description`。
- 正式装配链为：
  `Link6 -> plasma_tool_base_link -> plasma_adapter_link ->`
  `plasma_spray_rod_link -> Arm_Tip`。
- 已迁入当前确认需要的公共底座、2 种转接件和 12 种喷杆 STL；没有迁入
  `260612end.STL` 或 `End piece.STL`。
- `config/tool_variants.yaml` 支持 `3_4/6_8`、`150/200` 和
  `1x5/1x10/1x20` 组合；12 个组合均通过 Xacro 生成检查。
- 用户确认现场实际转接规格为 `3_4`；当前默认试验型号修正为
  `3_4 + 200_1x10`。
- 用户确认 `3_4/6_8` 不改变轴向长度。同一喷杆/喷嘴规格共用轴向 TCP；
  GUI 固定内部 `3_4`，不显示转接件或完整工具型号，只显示喷嘴总长度。
- GUI 启动时从 `tcp_3_4_200_1x10.yaml` 的 `T_gripper_to_tcp` 读取
  `Arm_Tip +Z` 方向上的喷嘴总长度；后续实测只需更新同一 YAML，GUI 与
  路径转换节点会同步，避免两处硬编码。
- 2026-07-23 新增的三张现行 DWG 已归档到
  `plasma_tool_description/reference/drawings/`。喷杆图纸标注总长 `210 mm`，
  装配时插入枪体 `10 mm`，外露仍为 `200 mm`；枪口距 Link6 为
  `78 + 40 = 118 mm`，所以正确轴向 TCP 为
  `0.118 + 0.200 = 0.318 m`。旧的 `0.304 m` 计算混用了插入起点和临时
  TCP，已经作废。TCP YAML 状态更新为 `drawing_confirmed_geometry`，完整
  安装横向偏差/姿态尚未验收，所以继续保持 `validated=false` 和仅预览状态。
- CAD 几何继续沿局部 `+Y` 装配；整套 CAD 组件相对 Link6 使用
  `Rx(+90 deg)`，将局部 `+Y` 映射到 `Link6 +Z`，末端再用
  `Rx(-90 deg)`，使 `Arm_Tip` 与 Link6 同向。
- 2026-07-23 已在真实 `ECO65-BI` 三代控制器上以 5% 速度完成
  `Arm_Tip +Z 2 mm -> -Z 2 mm` 往返点动。两次 MoveL 均成功，正向实际
  位移约 `1.98 mm`，返回位置误差约 `0.033 mm`；操作者目视确认 `+Z`
  沿喷杆轴线朝喷嘴尖端。该结果只验收工具轴方向，完整 TCP 和手眼精度尚未
  验收，`execution_permitted` 继续保持 false。
- 新增只读详细工具系结果话题
  `/rm_driver/get_current_tool_frame_data_result`。实机读回控制器 `Arm_Tip`
  的平移和欧拉角均为零；同关节角下控制器 UDP 零工具位姿与 ROS
  `baselink -> Link6` 完全一致（四元数仅整体反号）。因此控制器当前
  `Arm_Tip` 是法兰零工具，未包含 318 mm 喷嘴长度；软件继续通过
  `T_gripper_to_tcp` 反算法兰位姿，未向控制器写入 TCP。
- 名义矩阵保存为
  `plasma_tool_description/config/tcp_3_4_200_1x10.yaml`，并由
  `plasma_path_transform.launch.py` 默认加载。
- 该 TCP 仅来自 CAD，安装原点和真实喷嘴尖端未实测，保持
  `validated=false`。实测运行状态为 `tcp_loaded=true`、
  `ready_for_preview=true`、`ready_for_execution=false`；未发送运动命令。
- 型号选择、文件职责和后续实测步骤见
  `plasma_tool_description/README.md`。

### 真实机械臂自检

- GUI 注册并自动启动 `rm_driver/rm_eco65_driver.launch.py`。
- 成功条件同时要求 ROS 图存在 `/rm_driver`，且 `/joint_states` 在 1.5 秒有效期内持续提供健康的六关节数据。
- 最长等待 15 秒；失败时只停止本次自检自动启动的驱动，不停止外部已有驱动。
- 默认不再跳过机械臂；只有显式设置 `PLASMA_GUI_DEBUG_MODE=1` 才启用调试跳过。
- 2026-07-23 已在正式 GUI 中点击系统自检，自动启动相机和 ECO65 驱动，界面显示成功，日志输出“系统自检通过”。全程未发送 MoveJ、MoveL 或其他运动命令。

### C++ 坐标转换节点

正式代码均位于 `src/third_party/plasma_robot/`：

```text
plasma_robot_interfaces
tools/eye_hand (ROS 包名 plasma_eye_hand)
tools/tf/plasma_path_transform
```

固定变换链：

```text
T_base_tcp =
T_base_gripper(capture)
* T_gripper_camera(handeye)
* T_camera_tcp(path)
```

- 输入必须携带采集点云时的六关节状态，或直接携带采集时 `T_base_gripper`；禁止使用规划完成后的当前机械臂姿态替代。
- 六关节输入通过 ECO65 URDF 和 KDL 正解得到 `T_base_gripper`。
- 手眼 YAML 兼容键 `T_camera_to_gripper`，实际语义固定为 `^gripper T_camera`；TCP 键为 `T_gripper_to_tcp`。
- 标定加载支持 `validated`、文本 `status`、`quality.status`、`deployment.quality_accepted` 和 `deployment.status`。
- 没有手眼矩阵时拒绝转换；未验收矩阵只能生成 `execution_permitted=false` 的预览。
- 示例 identity 文件只说明格式，禁止用于实机运动。

ROS 接口：

```text
输入  /plasma/planned_spray_path/camera
输出  /plasma/planned_spray_path/base
状态  /plasma/path_transform/status
服务  /plasma_path_transform/transform_path
服务  /plasma_path_transform/reload_calibration
```

GUI 在暂停采集时冻结点云 `frame_id`、时间戳和当时六关节状态；规划完成后将全部路径点、姿态、阶段和开关元数据发布给转换节点。缺少采集元数据时拒绝发布。

### 文件归属与脚本清单

- 本轮正式新增的 ROS 2 包：共享接口 `plasma_robot_interfaces/`、手眼数据包 `tools/eye_hand/`、坐标转换包 `tools/tf/plasma_path_transform/`。
- 本轮新增的启动脚本：`tools/tf/plasma_path_transform/launch/plasma_path_transform.launch.py`，只启动 C++ 坐标转换节点。
- 已融入 GUI、但不是本轮生成的原厂脚本：`rm_driver/launch/rm_eco65_driver.launch.py`，用于真实机械臂自检连接。
- 原厂机械臂本体和接口：`rm_driver/`、`rm_description/`、`rm_ros_interfaces/` 等 `rm_*` 包，不属于本轮生成代码。
- `src/plasma_gui/scripts/mock_arm_joint_states.py` 只是模拟六关节数据的测试工具。
- `src/plasma_gui/scripts/single_layer_rotation_demo.py` 和 `multi_layer_rotation_demo.py` 是上一阶段实机诊断/干运行脚本，不属于正式坐标转换或喷涂位姿输出链路；正式功能不依赖它们。
- GUI 中正式接入代码仍放在 GUI 自己的职责目录：路径发布位于 `gui/mainwindow` 和 `gui/work_space/opengl/point_v2`，原有机械臂 ROS 适配位于 `module/RobotArm`。

后续目标目录结构：

```text
src/third_party/plasma_robot/
├── plasma_robot_interfaces/       # 跨模块共享 ROS 消息/服务
└── tools/
    ├── eye_hand/                  # 手眼采集、求解、验证、矩阵配置
    └── tf/                        # TF、坐标转换、路径变换工具
```

`tools/eye_hand/` 已保存 L515 `f1423110` 的 HORAUD 暂用矩阵，并整理为 ROS 数据包 `plasma_eye_hand`。`plasma_path_transform` 已迁入 `tools/tf/`，默认加载该矩阵。矩阵严格验收未通过，因此只允许预览，`execution_permitted` 保持为 false。

`plasma_eye_hand` 现已增加 C++ 只读工具 `tcp_pivot_calibrator`。它只订阅
`/rm_driver/udp_arm_position` 并由操作者按键采集法兰姿态，不发布任何运动
命令；用于从 6-10 个手动低速、多轴变化的 pivot 姿态求
`Link6 -> 喷嘴尖端` 三维平移和横向安装偏差。结果独立保存为 YAML，固定保持
`validated: false`、`execution_allowed: false`，不会自动覆盖当前 318 mm TCP。
使用步骤和输出路径见 `tools/eye_hand/README.md`。

### 构建、测试和运行时结果

- `plasma_robot_interfaces`、`plasma_path_transform`、`plasma_gui` 构建成功。
- 转换包限定结果：`8 tests, 0 errors, 0 failures, 0 skipped`。
- `plasma_eye_hand` 与迁移后的 `plasma_path_transform` 从新目录构建成功，源矩阵和安装矩阵 SHA-256 一致。
- 默认 launch 已实际加载 `l515_handeye_20260723_152410_horaud`；单位法兰姿态下，相机原点转换为 `[-0.0400500, -0.0619060, 0.0242366] m`，与 YAML 完全一致。
- 修正后 URDF 实测 `Link6 -> Arm_Tip` 为平移 `[0, 0, 0.318] m`、单位旋转。路径转换服务使用 `z=0.500 m` 的 TCP 测试点，正确得到 `z=0.182 m` 的法兰位姿；结果为 `transform_valid=true`、`flange_pose_valid=true`、`execution_permitted=false`。
- 状态话题确认机械臂模型、手眼矩阵和 TCP 均已加载，`ready_for_preview=true`、`ready_for_execution=false`；图纸推导喷嘴总长度为 `318.0 mm`，但手眼矩阵仍为暂用状态，且安装横向偏差和完整六自由度 TCP 尚未验收。
- 最新 GUI 已重新构建并实际启动，日志确认 `已加载喷嘴总长度：318.0 mm`；启动和验证过程中未发送新的 MoveJ 或 MoveL。
- `plasma_eye_hand` 的 pivot 合成数据和无驱动安全拒绝测试通过；尚未采集当前喷嘴的现场 pivot 样本。
- GUI 启动时发现并修复 Galactic 对进程内通信加 `transient_local` QoS 的不兼容。
- GUI 关闭时增加 ROS worker 和三个受管 launch 的停止等待；复测 GUI 返回码为 0，关闭后 ROS 图及设备进程全部清空。
- 当前没有运行中的 `plasma_gui`、`rm_driver`、相机或 `path_transform_node` 进程。

## 2026-07-22 上一阶段完成内容

### 独立机械臂执行步骤

- `WizardStep` 从 4 步扩展为 5 步，在 `PathPlanning` 后增加 `RobotExecution`。
- 新增“机械臂执行”流程页。页面在 `MainWindow::initStepWizard()` 中动态创建，避免直接扩充庞大的 `.ui` XML。
- 页面显示：
  - 机械臂在线状态
  - 当前选择的轨迹节点
  - 闭合层数
  - 规划层间距
  - 喷嘴总长度和当前人工入口干运行模式
  - 独立执行进度条
- 路径规划页只负责参数配置和完整轨迹生成，不再通过 `btn1` 启动机械臂。
- `btn1` 长按确认已经迁移到 `RobotExecution`；长按走满后调用已有执行确认弹窗。
- `btn2` 在机械臂页是停止按钮，会中止执行器或直接发布 `move_stop`。

### 流程状态

- 开始重新生成完整轨迹时，`PathPlanning` 和 `RobotExecution` 都重置为未完成。
- 完整轨迹的六个阶段全部成功后，才将 `PathPlanning` 标记完成并允许正常进入机械臂步骤。
- 机械臂执行成功后单独完成 `RobotExecution`。
- 机械臂执行失败、中止或急停后，`RobotExecution` 保持未完成。
- 路径参数改变后，必须重新生成完整轨迹。

### 安全入口

- 主界面急停现在会同时：
  - 关闭等离子 UI 输出状态
  - 将气体流量置零
  - 调用串口 `emergencyStop()`
  - 中止正在运行的机械臂执行器，或直接发布 `/rm_driver/move_stop_cmd`
  - 清空机械臂执行进度并将该步骤标记为未完成
- 真实等离子自动启停尚未接入机械臂执行阶段，当前仍保持关闭。

## 构建与启动结果

已执行：

```bash
colcon build --packages-select plasma_gui --event-handlers console_direct+
```

结果：成功。只有 Open3D `CMP0072` 的既有 CMake 弃用警告。

新构建 GUI 已启动，启动日志确认：

```text
StepWizard initialized with 5 steps
```

启动时没有下发 MoveJ 或 MoveL。第五步页面交互检查仍属于后续整机流程任务，不优先于当前手眼矩阵接入。

## 交接时正在运行的进程

当前没有运行中的 GUI、相机、机械臂驱动或坐标转换节点。运行时仍会看到两个同名 `/plasma_gui_node` 和 rosout 警告，这是已有节点命名问题，后续单独整理。

## 最后确认的实机状态

以下是最后一次动作后的确认值，新对话开始时必须向用户确认或读取实时状态：

- 机械臂：RealMan ECO65-BI。
- 控制器工具系：`Arm_Tip`。
- 现场已确认控制器工具 `+Z` 沿喷杆朝喷嘴尖端方向。
- 单层旋转执行成功：J6 左转半圈、回零、右转半圈、回零。
- 随后执行了两段工具 `+Z 5 mm` 的新增分层演示。
- 最后确认位置相对人工入口累计工具 `+Z 10 mm`，位于演示第三层。
- 最后确认 J6 回到末层入口角，约 `7.7 deg`。
- GUI 重启后没有再发送机械臂运动命令。
- 动作速度为 `5%`。
- 真实等离子输出关闭；用户此前说明等离子部分没有通电。

重要：不得直接重复运行多层脚本。每次运行都会继续沿工具 `+Z` 累计深入，必须先确认当前位置和剩余安全距离。

## 当前机械臂执行方式

正式 GUI 当前复用：

- `src/plasma_gui/src/module/RobotArm/arm_ros_adapter.*`
- `src/plasma_gui/src/module/RobotArm/demo_rotation_executor.*`

执行方式仍是经过实机演示验证的临时方案：

```text
人工对准第一层入口
-> 每层 J6 +180 deg
-> J6 回入口角
-> J6 -180 deg
-> J6 回入口角
-> 沿现场验证的工具轴移动到下一层
```

限制：

- 这不是模型坐标系离线位姿到机械臂基坐标系的完整转换。
- `Joint6GeometricDeg` 仍是保守几何代理，不是完整逆解结果。
- 没有完成机械臂模型碰撞检测、自动入口定位或实际等离子剂量控制。
- 当前用途是整机流程融合和低速断电干运行。

临时终端脚本仍保留，只用于诊断和现场简化演示，不作为正式 GUI 流程入口：

- `src/plasma_gui/scripts/single_layer_rotation_demo.py`
- `src/plasma_gui/scripts/multi_layer_rotation_demo.py`

## 下一步必须继续的任务

1. L515 `f1423110` 暂用矩阵已保存到 `tools/eye_hand/config/camera_to_gripper.yaml`，方向确认为 `^Link6 T_camera`，单位米。
2. 当前矩阵状态为 `user_approved_provisional_use`，严格离线验收为 false；继续进行非接触已知点验证，不得提升为正式执行状态。
3. 图纸轴向总长已修正为 `318.0 mm`，`Arm_Tip +Z` 方向已经确认；继续实测 `3_4 + 200_1x10` 的安装横向偏差和完整六自由度 TCP，修正 `T_gripper_to_tcp`。
4. TCP 实测验收后更新对应型号 YAML 并调用 `reload_calibration`，再次检查 `execution_permitted`。
5. 标定和 TCP 均验收后，确认输出从第一个喷涂起始位姿开始，且全部点的运动阶段和 `plasma_enabled` 正确。
6. 坐标链验收完成后，继续补齐机械臂故障与等离子输出的统一安全联锁。

## 本轮修改入口

- `src/third_party/plasma_robot/plasma_robot_interfaces/`
- `src/third_party/plasma_robot/tools/eye_hand/`
- `src/third_party/plasma_robot/tools/tf/plasma_path_transform/`
- `src/plasma_gui/src/gui/panel/left_panel/step_wizard/step/system_check/SystemCheckManager.*`
- `src/plasma_gui/src/gui/panel/left_panel/step_wizard/ros_launch_manager/ros_launch_manager.*`
- `src/plasma_gui/src/gui/work_space/opengl/point_v2/ros_worker.h`
- `src/plasma_gui/src/gui/panel/left_panel/dbtree/NodeCloudData.h`
- `src/plasma_gui/src/gui/mainwindow/mainwindow.*`
- `src/plasma_gui/CMakeLists.txt`
- `src/plasma_gui/package.xml`

- `src/plasma_gui/src/gui/panel/left_panel/step_wizard/StepWizardModel.h`
- `src/plasma_gui/src/gui/panel/left_panel/step_wizard/StepWizardModel.cpp`
- `src/plasma_gui/src/gui/panel/left_panel/step_wizard/StepWizardView.h`
- `src/plasma_gui/src/gui/panel/left_panel/step_wizard/StepWizardView.cpp`
- `src/plasma_gui/src/gui/mainwindow/mainwindow.h`
- `src/plasma_gui/src/gui/mainwindow/mainwindow.cpp`
- `src/plasma_gui/src/temp_ref/INTEGRATED_WORKFLOW.md`
- `src/plasma_gui/src/temp_ref/HANDOFF.md`

## 工作区警告

当前仓库非常脏，大量 `src/plasma_gui`、`src/plasma_robot` 和资源文件尚未跟踪，另有历史删除和修改。下一位处理时：

- 只修改当前任务涉及的文件。
- 不要以 `git status` 中的未跟踪状态为理由删除文件。
- 不要恢复看似“被删除”的历史文件，除非用户明确要求。
- `mainwindow.cpp/.h` 原本就包含大量用户修改并混用 CRLF/LF，避免进行整文件格式化或行尾统一。

## 新对话开场检查命令

```bash
cd /home/larusxu/CodeSpace/PlasmaRobot
sed -n '1,260p' src/plasma_gui/src/temp_ref/HANDOFF_NEW_CHAT_20260722.md
sed -n '1,220p' src/plasma_gui/src/temp_ref/INTEGRATED_WORKFLOW.md
git status --short
pgrep -af 'plasma_gui|rm_driver'
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 node list | sort
```

新对话首先询问用户最新现场变化；如果最新信息与本文档冲突，以用户最新信息为准。当前从“导入并核对实测手眼矩阵”继续，不得先使用示例矩阵或启动自动路径运动。

## 2026-07-24 末端三对象与正式路径接口更新

本节覆盖本文前面所有把 `Arm_Tip 318 mm` 直接称为运动 TCP、把 10 mm
喷口对称套在 TCP 两端、或把喷杆半径写成 5 mm 的旧说明。

当前 `3_4 + 200_1x10` 正式几何：

```text
plasma_motion_tcp:
  Link6 轴向距离 310 mm
  +X 沿喷杆朝底部/物理前端
  +Z 指向侧面喷涂方向

plasma_safety_tip:
  motion TCP +X 8 mm
  Link6 轴向距离 318 mm

plasma_spray_outlet:
  motion TCP +Z 2 mm
  代表 3_4 喷杆侧喷口表面中心
```

路径有效范围使用非对称末端占用：

```text
开口端 = 侧喷口后半长 5 mm + 开口额外安全距离
底部端 = TCP 到安全点 8 mm + 底部额外安全距离
喷杆半径 = 2 mm
```

GUI 已改为读取 TCP YAML 的 `measurement` 字段，不再从旋转后的矩阵投影猜
工具长度；`PathPlanningTask` 和 `SlicePlanningTask` 都显式接收 5 mm/8 mm。
`NodeCloudData` 保存运动 TCP、安全点、喷口后向占用和喷杆半径元数据。

`SprayPathPoint.msg` 新增：

```text
safety_point + safety_point_valid
spray_outlet_pose + spray_outlet_pose_valid
```

`plasma_path_transform` 从同一工具 YAML 的 `T_tcp_to_safety_tip` 和
`T_tcp_to_spray_outlet` 为每个基坐标路径点生成这两项。正式
`plasma_path_executor` 会拒绝缺少有效安全点或侧喷口位姿的旧路径。

验证结果：

- 五个目标包均构建成功：`plasma_robot_interfaces`、
  `plasma_tool_description`、`plasma_path_transform`、
  `plasma_path_executor`、`plasma_gui`。
- 转换测试 8 项、执行器测试 6 项全部通过。
- 只读 ROS 服务以单位运动 TCP 检查得到安全点 `[0.008, 0, 0]`、侧喷口
  `[0, 0, 0.002]`，两项 valid 均为 true。
- 输出仍为 `execution_permitted=false`；真实运动和真实等离子输出均未开启。

正式 `plasma_path_executor` 已接入 GUI：执行页开始/停止和主急停均走正式
执行器状态/服务，旧 `DemoRotationExecutor` 不再由 GUI 创建或调用。GUI 自动
启动执行器时固定 `motion_enabled=false`，因此当前只能缓存、校验和显示路径，
不能运动。软件仍不负责机械臂初始位置到入口点的运动；第一点只用于核对人工
已经到入口。

下一步优先级：完成现场手眼、运动 TCP、安全点和侧喷口的已知点验收。只有
验收并更新 YAML 状态后，才讨论把 GUI 注册 launch 中的运动门改为 true；
即使开启，正式执行器仍只接受 `dry_run=true`，真实等离子输出保持关闭。

补充验证：已向正式执行器发布一条完整的 8 点单层合成路径，顺序为
`0 -> +180 -> 0 -> -180 -> 0`，安全点和侧喷口位姿均有效。执行器状态为
`STATE_PATH_READY`、`total_points=8`，随后因 `motion_enabled=false` 明确拒绝
开始请求；全程 `plasma_output_enabled=false`。临时执行器代码已从
`MainWindow` 删除，并从 GUI 构建源列表排除，仅保留诊断源码文件。

## 2026-07-24 末端安装尺寸最终修正

本节覆盖本文前面所有 `318/315 mm` 结论以及把圆头后方 `3 mm` 直接当成
喷口中心的结论。现场最终确认：

```text
Link6 安装基准面到圆头末端       310 mm
圆头末端到喷口近侧边               3 mm
1x10 喷口轴向长度                  10 mm
圆头末端到喷口中心                  8 mm = 3 + 10 / 2
Link6 到喷口中心轴向               302 mm
3_4 喷口表面相对喷杆轴线径向         2 mm
```

正式 `plasma_motion_tcp` 仍位于真实喷口中心，因此
`Link6 -> TCP = [0.002, 0, 0.302] m`，`TCP -> 圆头末端 =
[+0.008, 0, -0.002] m`。径向 2 mm 对远距离喷涂距离影响很小，但保留该项以
保证正负 180 度旋转围绕真实喷口中心。旧的 `重建1路径` 由错误末端几何生成，
必须作废并在新 GUI 加载修正后的 YAML 后重新规划。

## 2026-07-24 自动入口与防碰撞更新（覆盖前文旧入口说明）

当前软件已经负责从机械臂任意初始位姿到入口外法线方向 100 mm 预入口点，不再要求人工精确
摆到入口。MoveIt 先规划当前位置到预入口；从该规划的末端关节状态继续检查
`预入口 -> 入口 -> 全部喷涂点`，5 mm 笛卡尔采样必须 100% 可解且无碰撞，
否则不会签发当前路径的执行许可。GUI 会在 RViz 显示两段轨迹并要求二次确认。

下方白色机柜已经作为当前专用 URDF 的碰撞 Link：

```text
尺寸: 0.995 x 0.603 x 0.954 m
中心(baselink): [0.403, 0, -0.403] m
```

SRDF 只关闭 `baselink <-> plasma_cabinet_collision_link`，机械臂其他 Link 和
等离子末端与机柜的碰撞检查保持开启。除此之外，正式运动还增加两层运行保护：

1. RealMan 驱动设置碰撞灵敏度后立即回读，实际值必须为 8；设置或回读失败会发布
   无效值并锁住运动。
2. 每个直接 MoveL 小步开始前记录六关节电流，运动中任一关节电流幅值相对该步基准
   增加超过 1500 mA，会立即发送停止命令。电流数据或 Link6 位姿超时同样停止。

完整说明：

```text
src/third_party/plasma_robot/tools/motion/AUTOMATIC_ENTRY_WORKFLOW_20260724.md
src/third_party/plasma_robot/tools/motion/COLLISION_SAFETY_20260724.md
```

安全锁仍保持：GUI 中入口规划器和路径执行器均为 `motion_enabled=false`，手眼
`strict_quality_accepted=false`，TCP `validated=false`，因此目前只允许全路径规划、
碰撞检查和 RViz 预览，不允许绕过验收下发真实运动。

## 2026-07-25 完整路径首次现场 MoveIt 预览

`重建1路径` 的当前位置到预入口规划成功。后半段最初被 MoveIt 相对关节跳变
系数误判；探测计算显示启用或关闭碰撞均只能完成 `0.000342`，关闭相对跳变后
完成率为 `1.0`，因此不是碰撞或 IK 不可达。

入口规划器现改为保持全程碰撞检查，并对生成后的六轴轨迹执行 10 度绝对相邻
步差限制。现场重新规划结果：596 个目标位姿、3107 个关节轨迹点，最大步差
`1.900827 deg`，发生在 `joint3`，完整预览通过并进入 `STATE_PREVIEW_READY`。
当时机械臂距离预入口约 `117.9 mm`、姿态差约 `148.9 deg`；运动门和标定许可
仍为 false，未发送任何真实运动。

后续一次规划在 `joint6` 第 489 点报告 `358.999981 deg`，但 RViz 已能播放动画。
确认该值来自正负 180 度边界的等价角表示，而非真实整圈反转。入口规划器现会
依据 MoveIt/URDF 的旋转关节上下限，为每一点选择最接近上一点的等价角，并把
连续化结果写回预览轨迹，再执行 10 度绝对步差检查。若限位内没有连续等价角，
仍会拒绝路径。`plasma_path_executor` 已重新构建，本包 23 项测试全部通过；运行中
的旧 GUI/规划器必须重启后才会加载该修复。

继续实机诊断确认存在第二类 359 度问题：失败点实际为
`joint6 -359.962702 -> -0.962572 deg`。这表示 MoveIt 随机选择的预入口 IK 支路
贴近机械下限，后续负向 180 度路径没有剩余关节行程，不能用局部角度回绕修正。
同一路径换一个预入口支路即可完整通过，最大步差约 1.29 度。入口规划器已改为
自动尝试最多 8 个“入口接近 + 全部喷涂点”候选，每个候选独立执行碰撞、IK、
关节限位和 10 度步差检查，只发布完整通过的支路。现场无运动模式连续验证 4 次
全部成功，最大步差为 `1.285716-1.682803 deg`；所有真实运动锁保持关闭。

2026-07-25 将预入口由 60 mm 调整为入口外法向 100 mm，并重新生成现场
`重建1路径`：446 个位姿，第 3 个完整分支通过，最大步差 `1.512857 deg`
（joint4）。操作者已在 RViz 确认预入口到入口为正常法向直线进入，不再从下方
横移靠近组织。该确认只验收路径形状；手眼 `strict_quality_accepted=false`、TCP
`validated=false` 和两个 `motion_enabled=false` 仍保持真实运动锁定。

## 2026-07-25 首次真实机械臂运动结果

通过显式的现场调试授权，在 `5%` 速度、等离子关闭条件下，已完成
`重建1路径` 的完整规划，并让真实机械臂成功运动到入口外法向 100 mm 的预入口。
MoveIt 和控制器均报告成功，操作者已亲眼确认机械臂运动。手眼/TCP 的 YAML 验收
状态和 GUI 默认运动门没有修改。

尚未执行预入口到入口、入口到第一喷涂层以及 446 点完整路径的实机干运行。
明天应从重新读取实时机械臂位姿、重新规划和等离子关闭的完整干运行继续。

结束前已正常关闭临时的运动执行器、宽松标定转换节点、MoveIt/RViz、入口规划器和
独立 `rm_driver`，未发送回撤或其他新运动命令。GUI 和相机仍保持运行。详细记录：

```text
src/third_party/plasma_robot/tools/motion/REAL_ROBOT_PRE_ENTRY_RESULT_20260725.md
```

## 2026-07-30 入口轴向过深安全修正

现场确认入口横向位置和方向基本准确，但安装真实喷管后，旧入口轴向过深，尚未开始
喷涂就存在圆头接近底部的风险。工具尺寸没有改动，仍为 Link6 到圆头 310 mm、
Link6 到侧喷口 TCP 轴向 302 mm、TCP 到圆头轴向 8 mm。

已新增独立的圆头入口外停距离，默认 30 mm；安全入口现在让圆头停在视觉物理开口外
30 mm，而不是开口平面。真实执行拆成“当前位置到预入口、预入口到安全入口并停车、
安全入口到喷涂点”三段。GUI 必须在第二段结束后目视确认圆头仍在真实开口外，才允许
第三段。旧路径必须作废并由新 GUI 重新生成。

相关五个包构建通过，坐标转换 11 项和轨迹执行 22 项测试全部通过。详细设计、文件
分布和首次现场复验步骤见：

```text
src/third_party/plasma_robot/tools/motion/AXIAL_ENTRY_STANDOFF_20260730.md
```

## 2026-07-30 末端轴向实测再次覆盖

本节覆盖本文所有旧的 `Link6 -> TCP 302 mm / 圆头 310 mm` 活动结论。操作者重新
确认实际装配栈为：

```text
端板 8 mm + 固定罩 70 mm + 喷头本体 40 mm + 外露喷管 200 mm = 318 mm
喷管起点/喷头本体末端 = 118 mm（不是运动 TCP）
真实侧喷口中心 TCP = 318 - (3 + 10 / 2) = 310 mm
```

活动工具配置已改为 `Link6 -> TCP=[0.002, 0, 0.310] m`、圆头 `318 mm`，
`TCP -> 圆头=[+0.008, 0, -0.002] m` 保持不变。完整记录在
`plasma_tool_description/TOOL_AXIAL_REMEASUREMENT_20260730.md`。

操作者在旧 302/310 模型生成的安全入口实测到“真实侧喷口恰好位于第一喷涂层”。
该结果必须保留，但旧路径已经失效；修正后同一视觉目标对应的 Link6 位姿会沿工具轴
向外移动约 8 mm，必须重新生成路径并分别复测圆头入口位置和第一层侧喷口位置。
