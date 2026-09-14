# Plasma GUI Handoff

本文档记录当前代码中的流程卡片、DB 树、点云/mesh/路径规划数据流，以及下一位程序员继续开发必须知道的入口。只描述当前最终状态。

> 当前最高优先级已经切换为整机流程融合：相机采集与重建 -> 机械臂执行 -> 等离子处理。实施顺序、暂停项和完成标准统一记录在 [INTEGRATED_WORKFLOW.md](./INTEGRATED_WORKFLOW.md)。恢复开发时必须先继续该文档中最后一个未完成项。
>
> 新对话请优先阅读 [HANDOFF_NEW_CHAT_20260722.md](./HANDOFF_NEW_CHAT_20260722.md)，其中包含最新代码改动、运行进程、实机最后位置和下一步检查。

## 任务连续性约定

- 任何进行中的任务如果因上下文切换、工具会话结束、程序异常或其他原因中断，恢复后必须先核对上次进度，并从中断处继续完成该任务。
- 不得把未完成的任务标记为完成，不得静默跳过未完成任务，也不得在未说明原因的情况下直接开始后续任务。
- 恢复工作时必须检查已修改文件、尚未完成的步骤、最近一次构建/测试结果，以及是否仍有命令或程序正在运行。
- 若中断前的实现只完成了几何预览或占位逻辑，恢复时必须保留其“不可执行”状态说明，直到机械臂位姿、逆解、碰撞检查和实际执行链路全部验证完成。
- 当前已经生成“左转半圈 -> 断能回零 -> 右转半圈 -> 断能回零”的离线几何位姿序列，但仍不得误写为可下发的机械臂轨迹；后续手眼标定、工具坐标变换、逆解、退刀和碰撞检查同样受上述中断恢复规则约束。

## 算法参考文件

外部 MATLAB 示例和测试数据已统一归档到 `src/plasma_gui/src/temp_ref/`：

- `cavity_reference_axis_matlab.m`：残腔中心轴计算参考。
- `cavity_slicing_planes_matlab_fixed.m`：切片平面生成参考。
- `cavity_section_contour_bspline_fit_v8.m`：切片轮廓 B 样条拟合参考。
- `cavity_equal_dose_and_path_from_v8_manual_region_overlay_v6.m`：等剂量面、分区和喷杆路径生成参考。
- `u1.txt`：MATLAB 示例的历史分区点云测试数据，仅保留作离线对照；当前 GUI 不再加载或显示它。

## 当前操作流程（快速查看）

下面是从程序启动到完整喷涂轨迹预览及第一层旋转演示的操作顺序。

1. 系统自检
   - 在 `系统自检` 流程卡片中点击 `btn1` 开始自检。
   - 当前 debug mode 可以跳过设备自检；点击下一步时会提示当前处于调试模式。

2. 残腔采集
   - 进入 `残腔采集` 页面后，实时点云默认持续刷新。
   - 点击 `btn1` 暂停实时点云，再使用点云裁剪工具选择需要保留的区域。
   - 裁剪确认后点击 `btn2`，将当前结果保存为 `残腔采集` 下的新关键帧。
   - 保存完成后自动恢复实时点云显示，可继续采集下一帧。

3. 残腔重建
   - 点击下一步会停止实时点云显示并进入 `残腔重建`。
   - 右键 `残腔采集` 下的关键帧，选择 `重建`。
   - 等待流程卡片进度条完成；重建 mesh 会加入 `残腔重建` 节点。
   - 右键新生成的重建节点，选择 `术区选择`。
   - 使用矩形或多边形框选核心术区，点击裁剪工具条中的勾确认。
   - 术区确认后，选中 mesh 使用淡橙色显示，并允许进入下一步。

4. 路径规划参数配置
   - 点击下一步进入 `路径规划` 页面。
   - 点击流程卡片中的 `btn2` 打开参数配置窗口。
   - 参数窗口按 `术区与喷嘴`、`切片范围`、`喷嘴路径`、`连续执行` 分组，只保留操作者需要的参数；算法细节使用内部默认值。

5. 生成参考轴
   - 右键 `残腔重建` 下已经完成术区选择的节点，选择 `路径规划`。
   - 线程池完成计算后，结果加入 `轨迹生成`，命名为 `重建节点名 + 路径`。
   - 勾选该轨迹节点后显示蓝色加粗直参考轴；其两端按当前喷嘴端部轴向尺寸和底部/开口安全距离裁剪，红色弯曲中心线默认隐藏，可通过右键菜单切换。

6. 生成切片平面
   - 右键 `轨迹生成` 子节点，选择 `生成切片平面`。
   - 切片起止范围、平面尺寸和绿色定向包围盒均以已选择的核心术区 mesh 为准，不再覆盖完整重建 mesh。
   - 第一个切片平面显示为半透明红色，其余切片平面显示为半透明蓝色，定向包围盒显示为绿色。

7. 生成原始切片轮廓
   - 右键同一个 `轨迹生成` 子节点，选择 `生成切片轮廓`。
   - 程序计算各切片平面与术区 mesh 的交线，结果显示为黄色轮廓。

8. 拟合切片轮廓
   - 右键同一个 `轨迹生成` 子节点，选择 `拟合切片轮廓`。
   - 线程池逐层完成闭合/开口判断、角度分箱、平滑、B 样条拟合和开口端曲率清理。
   - 拟合结果显示为青色加粗轮廓，并叠加在黄色原始轮廓上。
   - 每个闭合拟合轮廓会显示一个洋红色旋转起点：蓝色直参考轴的 `+u` 法线与该层轨迹的交点。
   - 路径规划卡片底部的进度条实时更新，主日志框输出各层拟合状态。

9. 生成喷杆几何路径预览
   - 普通流程无需单独执行；`生成完整喷涂轨迹` 会自动进入该阶段。需要单步调试时，在 `高级处理` 中选择 `生成喷杆几何路径`。
   - 线程池根据青色拟合轮廓生成分区等剂量带状面、红色逐层喷杆路径和绿色层间连接线。
   - 可在右键菜单中分别显示或隐藏等剂量面和喷杆路径。
   - 当前结果是几何路径预览，尚不能直接下发机械臂。

10. 生成分段连续路径
   - 右键同一个 `轨迹生成` 子节点，选择 `生成分段连续路径`。
   - 机械臂末端存在线缆缠绕约束，关节 6 每个执行段最多只能转半圈，因此闭合 360 度路径会至少拆成两个执行段。
   - 线程池按局部周向累计转角拆分路径，并在精确的 180 度边界插值；分段处显示白色关节 6 复位标记。
   - 洋红色粗线为处理段，黄色线为正常层间移动，红色线为超过配置阈值的异常层间移动。
   - 生成后原始红色喷杆路径自动隐藏，可通过右键菜单显示或隐藏整套分段连续路径。
   - 当前只生成分段几何和复位标记，不包含安全退刀、关节 6 回绕动作、喷杆姿态、逆解或碰撞检查，不能直接执行。

11. 生成喷嘴位姿
   - 右键同一个 `轨迹生成` 子节点，选择 `生成喷嘴位姿`。
   - 每个闭合层以洋红色旋转起点为零度，按 `左转 0→180° -> 断能回零 -> 右转 0→-180° -> 断能回零` 生成离线几何位姿；开口层暂不套用完整旋转并会跳过。
   - 每个位姿保存蓝轴上的 TCP、喷嘴方向、轴向、四元数、层号、几何转角、运动阶段和等离子启停状态；层间以零度姿态沿蓝轴移动。
   - 视图稀疏显示每层四个唯一方向的彩色喷嘴箭头：`0°/+90°/±180°共用方向/-90°`，可通过右键菜单隐藏或显示。
   - 当前位姿仍在模型坐标系中，未经过机器人基坐标变换、真实 TCP 标定、逆解或碰撞检查，不能直接执行。

12. 手动入口的逐层旋转干运行
   - 本功能只用于实机演示，不等同于将模型坐标系中的喷嘴位姿直接下发机械臂。
   - 在 DB 树中单击已经生成喷嘴位姿的 `轨迹生成` 子节点，再长按路径规划卡片的 `btn1`，直至进度条走满。
   - 软件从 `nozzlePoseSequence` 的每个闭合层提取左右半圈角度和蓝轴上的 TCP；没有喷嘴位姿、没有有效左右半圈或没有选中轨迹节点时禁止执行。
   - 操作者必须先通过示教器将机械臂移动到第一层入口，并让喷嘴对准洋红色旋转起点。软件不负责从初始姿态移动到入口。
   - 确认弹窗显示轨迹生成时保存的等离子工具型号、法兰到喷嘴 TCP、喷嘴端部轴向尺寸和模型深入轴，并自动查询控制器当前工具坐标系。工具系查询、MoveL 接口或结果话题未就绪时禁止进入执行。
   - 控制器层移工具 X/Y/Z 轴及同向/反向均不设默认值。选择候选轴后，使用弹窗中的 `+1mm / -1mm` 按钮完成一次往返点动；第一次点动后只能反向返回，回到测试起点后该轴才被标记为已验证。
   - 点动速度固定为 `5%`，下发前同样要求 `/joint_states` 在 `1.5s` 内有效。点动失败或 `15s` 超时会发送 `/rm_driver/move_stop_cmd`，将位置标记为未知并禁止本次继续执行。点动尚未返回测试起点时，不能切换轴或确认执行。
   - 每层以最新 `/joint_states` 为入口零位，保持该层 `J1-J5` 不变，按 `J6 左转 -> 回零 -> J6 右转 -> 回零` 执行；层间断能并通过 `/rm_driver/movel_offset_cmd` 沿工具坐标轴平移，MoveJ/MoveL 速度均固定为 `5%`。
   - MoveL 成功后必须再收到一帧新的 `/joint_states`，该状态才会成为下一层基线。每一层开始前都重新校验 J6 左右目标是否在 `+-360°` 内。
   - 真实等离子输出始终关闭，日志中的喷涂启停只是阶段提示。
   - 执行前要求 `/joint_states` 在 `1.5s` 内有效，且真实驱动同时存在 `/rm_driver/movej_cmd` 订阅端和 `/rm_driver/movej_result` 发布端。模拟关节状态不能单独触发实机动作。
   - 多层执行还要求发现 `/rm_driver/movel_offset_cmd` 和 `/rm_driver/movel_offset_result`。四代控制器使用原生 `rm_movel_offset`；三代控制器不支持该 SDK 接口，驱动会读取当前位姿，通过 `rm_algo_pose_move` 计算工具/工作坐标偏移目标，再用 `rm_movel` 执行兼容直线运动。每段失败、超过 `90s`、MoveL 后 `2.5s` 内没有新关节状态或执行期间关节状态中断时，向 `/rm_driver/move_stop_cmd` 发送停止命令。
   - 2026-07-22 实机只读查询已确认当前设备为 `ECO65-BI`、三代控制器（controller version/generation 均为 `3`），当前控制器工具系为 `Arm_Tip`。原生 `rm_movel_offset` 的首次 1 mm 验证被控制器拒绝后，已采用上述三代兼容实现；重新构建后的驱动启动、控制器代际识别和工具系查询均已验证，兼容 MoveL 的实机 1 mm 往返仍需在开阔位置低速确认。
   - 当前 Eco65 的 J6 软件限位按 URDF 的 `+-360°` 检查，每个方向的单段旋转不允许超过 `180°`。
   - 轨迹节点必须保存生成时的工具型号、法兰到喷嘴 TCP、喷嘴端部轴向尺寸、工具深入轴、底部安全距离和切片间距。蓝轴生成时已从底面退让 `喷嘴端部轴向尺寸/2 + 底部安全距离`；执行前还会确认每层 TCP 同时满足轴向范围和径向贴轴要求，任一层不满足即禁止实机执行。
   - 层间距离必须大于 `1 um` 且不超过路径参数中的层间距离阈值，执行器另有 `50 mm` 硬上限。当前默认阈值为 `15 mm`。

13. 首次多层实机验证
   - 保持等离子电源物理禁用，先只选择一层，确认 `左半圈 -> 回零 -> 右半圈 -> 回零` 与线缆约束一致。
   - 单层通过后，在喷嘴远离工件的位置打开执行确认弹窗。确认自动显示的控制器工具系名称，选择候选轴并通过 `+1mm / -1mm` 往返点动确定与蓝轴重合的轴和正负方向。
   - 轴向确认后再选择两层，只验证一次层间移动；不要第一次就执行全部层。
   - 用示教器将第一层 TCP 和喷嘴零方向对准后，确认弹窗中的首个层间指令约等于规划切片间距，通常为 `5 mm`。
   - 当前动态 URDF 的喷杆深入轴是 `tcp_link` 局部 `+Y`，但 MoveL 使用的是机械臂控制器当前激活的工具坐标系。交接包没有启动时创建或切换喷杆工具坐标系的步骤，因此二者不能默认视为一致；旧固定模型中的 TCP 链接名是 `spray_tcp_link`。
   - 物理急停必须可立即触达。两层方向和距离验证正确后，才逐步增加执行层数。
   - 当前手眼矩阵尚未通过固定标定板一致性验证，不得用于自动绝对入口定位；现阶段仅采用手动首层对准加工具坐标相对层移。

控制器工具坐标系查询命令：

```bash
ros2 topic echo /rm_driver/get_current_tool_frame_result
ros2 topic pub --once /rm_driver/get_current_tool_frame_cmd std_msgs/msg/Empty "{}"
```

执行模块位于：

- `src/plasma_gui/src/module/RobotArm/arm_ros_adapter.*`：封装 RealMan MoveJ、工具坐标 MoveL、当前工具系查询、结果和运动停止话题。
- `src/plasma_gui/src/module/RobotArm/demo_rotation_executor.*`：逐层四段旋转、层间移动、关节基线更新、新鲜度监控、超时和进度。
- `MainWindow::onRequestPathExecution()`：轨迹数据检查和执行确认弹窗。
- `MainWindow::initArmMotionExecution()`：GUI 日志、进度和 ROS 适配器连接。

轨迹节点的普通右键菜单为：

```text
生成完整喷涂轨迹
显示内容 >
高级处理 >
删除
```

`生成完整喷涂轨迹` 会自动串行执行切片平面、切片轮廓、轮廓拟合、喷杆几何路径、分段连续路径和喷嘴位姿。流程卡片进度条显示六个阶段的总进度；中途失败会停在对应阶段。完成后默认隐藏中间结果，只保留蓝色参考轴和喷嘴位姿。

当前结果的颜色含义：

- 淡橙色半透明表面：选中的核心术区 mesh。
- 蓝色粗直线：按喷嘴端部轴向半尺寸和两端安全距离裁剪后的喷嘴中心安全轴段。
- 红色细曲线：弯曲中心线，默认隐藏。
- 红色半透明平面：第一个切片平面。
- 蓝色半透明平面：其余切片平面。
- 绿色线框：定向包围盒。
- 黄色轮廓：原始 mesh/切片平面交线。
- 青色粗轮廓：B 样条拟合结果。
- 洋红色大点：每层左转/右转半圈共用的旋转起点。
- 六色半透明带状面：按手动角度范围划分的喷涂分区预览，不代表物理等剂量。
- 红色粗线：各切片层的喷杆路径。
- 绿色粗线：相邻切片层之间的连接线。
- 洋红色粗线：受关节 6 半圈约束的分段处理路径。
- 黄色/红色连接线：正常/超长层间移动。
- 白色大点：关节 6 需要复位的分段位置。
- 洋红色箭头：每层零度喷嘴方向。
- 青绿色箭头：左转方向的 `+90°` 关键姿态。
- 橙色箭头：右转方向的 `-90°` 关键姿态。
- 浅蓝色箭头：空间上重合的 `+180/-180°` 共用方向。

结果失效规则：

- 重新生成切片平面时，旧黄色轮廓和旧青色拟合轮廓会被清除。
- 重新生成原始切片轮廓时，旧青色拟合轮廓会被清除。
- 重新生成切片平面、原始轮廓或拟合轮廓时，旧喷杆几何路径结果会被清除。
- 重新生成切片、轮廓、喷杆几何路径或分段连续路径时，旧喷嘴位姿会被清除。
- 删除正在计算的节点后，任务完成回调不会重新创建该节点的渲染数据。

## 流程卡片

流程由 `StepWizardModel` 管理，枚举在：

- `src/plasma_gui/src/gui/panel/left_panel/step_wizard/StepWizardModel.h`
- `src/plasma_gui/src/gui/panel/left_panel/step_wizard/StepWizardModel.cpp`

当前步骤：

1. `SystemCheck`：系统自检
   - `btn1` 是自检启动按钮。
   - 当前 MainWindow 初始化中开启了 debug mode，可跳过系统自检。

2. `CloudCapture`：残腔采集
   - 提示：请将末端相机移动至残腔上方合适的位置，以便采集残腔空间信息。
   - `btn1` 用于暂停/恢复实时点云帧显示。
   - `btn2` 用于确认裁剪后的关键帧。
   - 实时帧暂停/恢复使用 ROS topic 显示暂停逻辑，不应停止 camera 进程。

3. `CloudRebuild`：残腔重建
   - 提示：请在残腔采集序列中，右键选择合适的关键帧进行重建及术区选择。
   - 页面下方有重建进度条。
   - 右键 `残腔采集` 子节点：`重建 / 裁剪 / 删除`。
   - 右键 `残腔重建` 子节点：进入路径规划步骤前只显示 `术区选择 / 删除`；进入路径规划步骤后显示 `路径规划 / 术区选择 / 删除`。
   - 术区选择完成后，`CloudRebuild` 被标记为完成，允许进入下一步。

4. `PathPlanning`：路径规划
   - 提示：右键重建帧进行路径规划，并配置等离子参数，检查无误后即可执行！！！
   - 页面下方有路径规划进度条。
   - `btn1` 是执行确认按钮，需要长按，进度条走满后输出测试执行日志。
   - `btn2` 是路径参数配置按钮，点击弹出参数窗口。注意：`btn2` 只应在路径规划页可点击，采集页另有自己的确认逻辑。

`StepWizardView` 负责 UI 切换：

- `updateSystemCheckPage()`
- `updateCloudCapturePage()`
- `updateCloudRebuildPage()`
- `updatePathPlanningPage()`

## DB 树结构

DB 树相关文件：

- `src/plasma_gui/src/gui/panel/left_panel/dbtree/DbtreeModel.*`
- `src/plasma_gui/src/gui/panel/left_panel/dbtree/DbtreeView.*`
- `src/plasma_gui/src/gui/panel/left_panel/dbtree/NodeCloudData.h`

当前顶层节点：

- `残腔采集`
- `残腔重建`
- `轨迹生成`

`NodeCloudData` 是核心数据结构，当前包含：

- `polyData`：原点云
- `meshPolyData` / `meshFilePath`：重建 mesh
- `surgicalPointCloud` / `surgicalMesh`：术区选择结果
- `pathSampleCloud`：路径规划重采样点云
- `curvedAxis`：弯曲中心线
- `straightAxis`：按喷嘴端部轴向半尺寸和两端安全距离裁剪后的直参考轴
- `slicePlanes / firstSlicePlane / sliceBoundingBox`：切片平面与定向包围盒
- `sliceContours`：切片平面与术区 mesh 的原始交线
- `fittedSliceContours`：按层 B 样条拟合后的平滑轮廓
- `rotationStartMarkers`：蓝色直参考轴 `+u` 法线与各闭合拟合轨迹的交点
- `equalDoseSurface`：带 `RegionId` 和 cell color 的分区等剂量面
- `sprayPath`：各层喷杆路径
- `sprayPathConnections`：相邻层连接线
- `continuousSprayPath`：带 `SegmentId / SliceIndex / ClosedLayer / Joint6SweepDeg` 的分段处理路径
- `continuousPathTransitions`：带距离、起止切片和异常标记的层间移动线
- `joint6ResetMarkers`：同层路径分段处的关节 6 复位标记
- `nozzlePoseSequence`：离线六维几何位姿；点数据包含 `SequenceIndex / SliceIndex / MotionPhase / PlasmaEnabled / Joint6GeometricDeg / NozzleDirection / AxisDirection / QuaternionXYZW / TargetPoint`
- `nozzlePosePreview`：每层四个唯一方向的稀疏彩色喷嘴方向箭头输入
- `showCurvedAxis`：轨迹节点下是否显示弯曲中心线，默认 false
- `showSliceLayers`：统一控制切片平面、切片定向框、黄色节点框和黄色原始轮廓
- `showFittedContours`：统一控制拟合轮廓和洋红色旋转起点
- `showEqualDoseSurface / showSprayPath`：喷涂分区面和喷杆几何路径显隐状态
- `showContinuousPath`：分段路径、层间移动和复位标记的统一显隐状态
- `showNozzlePoses`：喷嘴位姿箭头显隐状态
- `surgicalMeshOpacity`：术区 mesh 高亮透明度
- `plannedNozzleLength`：规划时使用的喷嘴端部轴向尺寸，不是整根喷杆或法兰到 TCP 的长度
- `plannedToolVariant / plannedToolTcpOffset / plannedToolAxis`：轨迹生成时固化的等离子工具组合、法兰到喷嘴 TCP 和喷杆深入轴

路径规划完成后，不在原 `残腔重建` 子节点上显示路径。会复制一份数据加入 `轨迹生成` 子节点，命名规则：

```text
重建节点名 + 路径
```

例如：`重建1路径`。

`轨迹生成` 子节点右键菜单：

- `生成完整喷涂轨迹`：按固定依赖顺序自动生成全部结果
- `显示内容`：带勾选状态的二级菜单，可分别控制切片层与原始轮廓、拟合轮廓与旋转起点、喷涂分区面、喷杆几何路径、分段连续路径、喷嘴位姿和弯曲中心线
- `高级处理`：保留六个单步算法入口，供调试和局部重新生成使用
- `删除`

完整生成结束后默认只显示蓝色加粗直参考轴和喷嘴位姿；其他结果可从 `显示内容` 重新打开。

## 点云显示与裁剪

OpenGL 入口：

- `src/plasma_gui/src/gui/work_space/opengl/point_v2/main_gl.*`

关键接口：

- `showPointCloud(std::shared_ptr<NodeCloudData>)`
- `setPointCloudVisible(std::shared_ptr<NodeCloudData>, bool)`
- `setCurvedAxisVisible(std::shared_ptr<NodeCloudData>, bool)`
- `startSurgicalAreaSelection(std::shared_ptr<NodeCloudData>)`
- `showPathPlanningResult(...)`

当前显示约定：

- 术区 mesh：淡橙色，轨迹节点中透明度更低，便于突出路径轴线。
- 直参考轴：蓝色，加粗。
- 弯曲中心线：红色，细线，默认隐藏。

## 残腔重建

重建由 `PointDeal` 的线程池执行：

- `CloudRebuildTask`
- `PointDeal::requestCloudRebuild(...)`

MainWindow 入口：

- `onRequestCloudRebuild(...)`
- `onReconstructionProgress(...)`
- `onReconstructionLog(...)`
- `onReconstructionFinished(...)`

重建完成后通过 `DbtreeView::addRebuildMesh(...)` 加入 `残腔重建` 节点。

## 术区选择

术区选择入口：

- DB 树右键 `术区选择`
- `MainWindow::onRequestSurgicalAreaSelection(...)`
- `MainOpengl::startSurgicalAreaSelection(...)`

选择确认后：

- 同时裁剪原点云和重建 mesh。
- 当前界面只高亮术区 mesh。
- 术区 mesh 存入 `NodeCloudData::surgicalMesh`。
- 原重建节点保留术区数据，后续路径规划以 `surgicalMesh` 为输入。

## 路径参数配置

路径规划步骤下，点击 `btn2` 调用：

- `MainWindow::showPathParamDialog()`

窗口当前只显示以下功能参数：

- `术区与喷嘴`：术区处理等级；转接件 `3/4 / 6/8`；喷杆标称长度 `150 / 200mm`；喷嘴端部规格 `1x5 / 1x10 / 1x20mm`。
- `切片范围`：自动计算的切片间距；可分别配置喷嘴到底部、喷嘴到开口的安全距离。首次实机演示默认底部 `10mm`、开口 `5mm`，界面范围 `0.5-50mm`；若有效深度不足，算法拒绝生成，不会自动缩小安全距离。
- `喷嘴路径`：自动计算的喷嘴端部轴向尺寸、路径半径、法兰到喷嘴 TCP；喷杆深入轴固定显示为工具局部 `+Y`。
- `连续执行`：关节 6 单段最大转角、最大层间连接距离、切片执行顺序。

工具规格来自交接包：

```text
/home/larusxu/CodeSpace/robot_arm_handoff_20260721/
  plasma_ws/src/plasma_urdf_v2/config/tool_variants.yaml
```

默认组合为 `3_4_150_1x5`。当前工程的动态启动链为 `rm_eco65_bringup -> real_moveit_demo.launch.py -> tool_variants.yaml`，该组合加载 `78 + 30 + 143.5 = 251.5mm`，并带有对应工具的显示和碰撞 mesh。因此 GUI 默认实机工具固化为 `3_4_150_1x5 / 251.5mm / 工具局部 +Y`；其他组合可以生成路径预览，但启动参数和执行校验同步前禁止实机执行。

交接包中的旧固定启动链使用 `rm_eco65_with_shell.urdf.xacro`，其中几何偏移为 `78 + 30 + 150 = 258mm`，而且部分惯性字段与默认工具名称不一致。旧固定链与当前动态模型不能混用。实机演示应在机械臂侧使用当前工程源码并显式启动：

```bash
ros2 launch rm_bringup rm_eco65_bringup.launch.py \
  tool_enabled:=true \
  use_printed_part:=false \
  adapter_variant:=3_4 \
  rod_variant:=3_4_150_1x5
```

正式 L515 交接文档中的 pivot TCP `106.830mm` 属于当时的标定笔尖，不是等离子喷杆 TCP。代码规格不能替代现场量具确认；首次运动前必须核对实物组合和法兰到喷嘴尖端的实际距离。

喷嘴端部规格是几何主参数。选择宽度 `d` 后自动设置：

- 切片间距和喷嘴端部轴向包络为 `d`。
- 喷嘴外半径为 `d / 2`。
- 蓝色直参考轴和切片中心的开口端、底部端均让出 `d / 2 + 对应端安全距离`；蓝线表示喷嘴中心允许经过的安全轴段。
- 等剂量路径使用两端安全距离的较小值进行径向检查：`术区轮廓半径 - 路径半径 >= 检查间隙`，不满足时停止生成并输出实际最小间隙。
- 修改 btn2 参数不会原地篡改已有轨迹节点；需要重新右键重建节点执行 `路径规划`，以重新生成蓝色安全轴段及其后续派生结果。旧轨迹没有工具规格元数据时禁止实机执行。

不再显示在窗口中的算法参数继续使用代码默认值：中心轴体素 `0.001m`、候选百分位 `75`、分层 `60`、截面半宽 `0.002m`、中心线点数 `120`、平滑窗口 `7`；切片平面缩放 `1.08`；B 样条点数 `320`、闭合阈值 `0.80`、轮廓平滑窗口 `7`、角度分箱 `1度`；闭合/开口路径采样 `180/120`；连续路径最小点间距 `0.0001m`，开口层自动反向保持开启。角度分区参数沿用 MATLAB 示例值。

如果直接右键重建节点执行路径规划但还没配置参数，会提示：

```text
请先配置路径参数！
```

## 路径规划算法

算法在线程池中执行：

- `PathPlanningTask`
- `PointDeal::requestPathPlanning(...)`

当前算法实现参考 MATLAB 示例 `cavity_reference_axis_matlab.m`，主要流程：

1. 从 `surgicalMesh` 提取三角面和边界边。
2. 按三角面面积进行表面重采样，生成 `pathSampleCloud`。
3. 对 mesh 表面进行体素化。
4. 对表面体素做一圈膨胀，增强闭合性。
5. 从体素网格外边界 flood fill 外部空间。
6. 将非外部空间视为实体体素，完成内部填充。
7. 对实体体素计算欧氏近似距离场。
8. 按中心候选百分位提取中心候选点。
9. 从 mesh 边界边中提取最大开口边界。
10. 拟合开口平面法向，并根据 mesh 中心确定深度方向。
11. 剔除开口附近不稳定中心候选点。
12. 沿深度方向分层提取中心点。
13. 对弯曲中心线做移动平均和平滑重采样。
14. 对平滑弯曲中心线做 PCA，得到指向残腔内部的直参考轴方向。
15. 将术区 mesh 投影到该方向，并在开口端、底部端分别扣除 `喷嘴半长 + 对应安全距离`，得到蓝色安全轴段。
16. 输出弯曲中心线和裁剪后的直参考轴；若安全参数占满术区深度则终止并提示减小参数。

注意：

- 如果体素尺寸过小导致体素数量过大，代码会自动放大体素尺寸，并在日志中打印调整后的尺寸。
- 当前距离场是 C++ 中的欧氏近似实现，不依赖 MATLAB Image Processing Toolbox。

## 路径规划结果

路径规划完成回调：

- `MainWindow::onPathPlanningFinished(...)`

结果处理：

1. 从 `m_pathPlanningSources` 找到原重建节点数据。
2. 新建一份 `NodeCloudData`。
3. 复制原重建 mesh、术区 mesh、路径采样点云、弯曲线、直轴。
4. 设置：
   - `showCurvedAxis = false`
   - `surgicalMeshOpacity = 0.28`
5. 调用 `DbtreeView::addTrajectoryPath(sourceName, pathData)` 加入 `轨迹生成`。

只有勾选 `轨迹生成` 下的路径节点，才会显示路径结果。

## 切片平面生成

参考算法：

- `src/plasma_gui/src/temp_ref/cavity_slicing_planes_matlab_fixed.m`

入口：

- `轨迹生成` 子节点右键 `生成切片平面`
- `MainWindow::onRequestSlicePlanning(...)`
- `MainOpengl::requestSlicePlanning(...)`
- `PointDeal::requestSlicePlanning(...)`
- `SlicePlanningTask`

当前 C++ 实现：

1. 从直参考轴得到残腔轴向 `a`。
2. 用弯曲中心线首尾方向统一 `a` 为指向残腔内部。
3. 在垂直于 `a` 的横截面上对 mesh 投影点做 PCA，获得 `u/v` 局部基。
4. 将 mesh 投影到 `u/v/a` 局部坐标系，得到定向包围盒范围。
5. 根据开口和底部安全距离生成有效轴向区间。
6. 首个切片位于 `bottomLimit - 0.5 * sliceSpacing`。
7. 其余切片按固定间距向开口方向排列。
8. 生成普通切片、首个切片和定向包围盒 VTK 几何。

显示约定：

- 普通切片：半透明蓝色。
- 首个切片：半透明红色。
- 定向包围盒：绿色线框。
- 切片结果存入当前轨迹节点的 `slicePlanes / firstSlicePlane / sliceBoundingBox`。
- 切片显隐跟随轨迹节点复选框。

## 切片轮廓

生成切片平面后，右键同一轨迹节点选择 `生成切片轮廓`。

算法在 `SliceContourTask` 中执行：

1. 遍历首切片和其余切片平面。
2. 从每个平面四边形取原点和法向。
3. 用 VTK Cutter 计算平面与 `surgicalMesh` 的交线。
4. 用 VTK Stripper 拼接连续线段。
5. 汇总所有有效切片轮廓到 `NodeCloudData::sliceContours`。
6. 轮廓在 OpenGL 中显示为亮黄色线。

入口：

- `MainWindow::onRequestSliceContours(...)`
- `MainOpengl::requestSliceContours(...)`
- `PointDeal::requestSliceContours(...)`
- `SliceContourTask`

## B 样条轮廓拟合

生成黄色切片轮廓后，右键同一 `轨迹生成` 子节点选择 `拟合切片轮廓`。

算法参考：

- `src/plasma_gui/src/temp_ref/cavity_section_contour_bspline_fit_v8.m`

算法在线程池的 `ContourFittingTask` 中执行：

1. 按原始轮廓 cell data 中的 `SliceIndex` 分组，每层保留最长的连续轮廓。
2. 根据直参考轴和弯曲轴方向建立局部轴向坐标系。
3. 将每层点按极角排序，并根据最大角度空隙判断该层为闭合层或开口层。
4. 开口层从最大角度空隙之后开始排序，避免跨越 0/360 度时顺序断裂。
5. 对候选点进行角度分箱取均值，再做闭合环形或开口移动平均。
6. 使用 `vtkParametricSpline` 分别执行闭合或开放 B 样条拟合。
7. 开口层按 MATLAB 示例计算离散曲率，检测端部峰值并寻找曲率恢复点，限制最大裁剪量和最少保留点数。
8. 输出轮廓写入 `NodeCloudData::fittedSliceContours`，并保留 `SliceIndex` 和 `ClosedLayer` cell data。
9. 对每个闭合层，以蓝色直参考轴为轴线建立稳定的 `u/v` 横向基，计算 `+u` 法线射线与拟合轮廓的最近正向交点，写入 `rotationStartMarkers`。开口层不生成完整旋转起点。

入口：

- `MainWindow::onRequestContourFitting(...)`
- `MainOpengl::requestContourFitting(...)`
- `PointDeal::requestContourFitting(...)`
- `ContourFittingTask`

显示约定：

- 黄色粗线：原始 mesh/切片平面交线。
- 青色加粗线：B 样条拟合后的分层轮廓。
- 洋红色大点：闭合层的统一旋转起点；左转半圈和回到起点后的右转半圈均以此点为基准。
- 重新生成切片平面会清除旧原始轮廓和拟合轮廓；重新生成原始轮廓会清除旧拟合轮廓。
- 拟合进度复用路径规划流程卡片底部的进度条，关键阶段和逐层结果写入主日志框。

## 喷杆几何路径与分区预览

普通流程由 `生成完整喷涂轨迹` 自动调用。单步调试时，完成青色 B 样条轮廓后，在 `高级处理` 中选择 `生成喷杆几何路径`。

算法参考：

- `src/plasma_gui/src/temp_ref/cavity_equal_dose_and_path_from_v8_manual_region_overlay_v6.m`

算法在线程池的 `EqualDosePathTask` 中执行：

1. 按 `SliceIndex` 读取拟合轮廓，并通过 `ClosedLayer` 区分闭合层和开口层。
2. 使用与轮廓拟合一致的直参考轴局部坐标系。
3. 闭合层按周向角周期 PCHIP 重采样，开口层展开角度后 PCHIP 重采样。
4. 根据喷杆外半径在每个角度生成喷杆路径点。
5. 将拟合轮廓沿轴向上下各扩展半个喷嘴长度，生成每层带状面。
6. 将手动角度范围按观察方向、增长方向和零度偏移转换，为面片写入 `RegionId` 和 MATLAB `lines` 风格颜色。
7. 第一有效开口层可使用首端或末端作为连接基准，其余层选择最接近参考角的路径点。
8. 将相邻层基准点连接成绿色层间连接线。

入口：

- `MainWindow::onRequestEqualDosePath(...)`
- `MainOpengl::requestEqualDosePath(...)`
- `PointDeal::requestEqualDosePath(...)`
- `EqualDosePathTask`

结果写入当前 `NodeCloudData`，不会新建 DB 树子节点。六色半透明表面是手动角度分区预览，红色线为喷杆几何路径，绿色线为层间连接；三者可在 `显示内容` 中控制。

当前限制：

- 当前“等剂量”来自几何带状面和手动角度分区，并没有物理剂量场模型。
- 当前路径只有位置点，没有喷杆姿态。
- 已进行基于局部轮廓和圆柱喷嘴外半径的径向安全间隙检查，但尚未进行完整喷嘴姿态、机械臂模型碰撞、越界、速度、停留时间和功率映射。
- 该结果只能用于预览，不能直接下发机械臂。

## 分段连续路径与关节 6 限制

完成等剂量路径后，右键同一 `轨迹生成` 子节点选择 `生成分段连续路径`。算法在线程池的 `ContinuousPathTask` 中执行，输入为 `NodeCloudData::sprayPath` 和直参考轴。

路径规划页 `btn2` 参数窗口中的 `连续执行` 分组包含：

- `关节6单段最大转角`：默认且最大为 180 度，可向下调整。
- `最大层间连接距离`：默认 `0.015m`，超过后连接线标红。
- `切片执行顺序`：切片号升序或降序。

`连续路径最小点间距` 默认 `0.0001m`、`自动反转开口层` 默认开启，二者作为内部算法参数，不再显示在操作窗口中。

任务处理过程：

1. 按 `SliceIndex` 排列喷杆路径层，并按参数决定升序或降序。
2. 闭合层从最靠近上一层终点的位置开始；开口层可自动反向。
3. 在直参考轴局部坐标系计算连续展开的周向角。
4. 按累计周向转动量拆分，每段不超过配置值且绝不超过 180 度；跨越边界时插入精确边界点。
5. 每个同层分割处写入关节 6 复位标记，相邻层之间单独生成过渡线并检查距离。
6. 主日志输出执行段数、复位点数、处理长度、层间移动长度、最大点间距和异常连接数。

入口：

- `MainWindow::onRequestContinuousPath(...)`
- `MainOpengl::requestContinuousPath(...)`
- `PointDeal::requestContinuousPath(...)`
- `ContinuousPathTask`

重要限制：局部残腔周向角只是当前阶段对关节 6 转角的保守几何代理，不等于机械臂真实关节角。后续必须先生成完整喷杆位姿并做机械臂逆解，再逐点验证实际关节 6 角度。复位标记目前只表达“此处必须中断并复位”，尚未生成退刀、回绕、重新接近等动作。

## 喷嘴离线几何位姿

完成分段连续路径后，右键同一 `轨迹生成` 子节点选择 `生成喷嘴位姿`。算法在线程池的 `NozzlePoseTask` 中执行。

入口：

- `MainWindow::onRequestNozzlePoses(...)`
- `MainOpengl::requestNozzlePoses(...)`
- `PointDeal::requestNozzlePoses(...)`
- `NozzlePoseTask`

当前处理过程：

1. 只读取带 `ClosedLayer` 的闭合拟合轮廓，开口层跳过。
2. 通过 `rotationStartMarkers` 的 `SliceIndex` 找到每层洋红色共同起点，并以该方向为零度。
3. 将轮廓中心投影到有限蓝色安全轴段，得到该层 TCP 位置。
4. 从 TCP 向每个周向角发射射线，与真实拟合轮廓求最近正向交点，得到喷嘴指向；内部采样步长固定为 `5°`。
5. 定义几何工具坐标：局部 `+Z` 为喷嘴朝向，局部 `+X` 为蓝轴方向，局部 `+Y` 保持右手系，并输出 `QuaternionXYZW`。
6. 每层依次写入 `左转处理(0→180°)`、`断能回零(180→0°)`、`右转处理(0→-180°)`、`断能回零(-180→0°)`；层间移动写为 `MotionPhase=4` 且 `PlasmaEnabled=0`。
7. 另生成四个唯一方向的稀疏箭头；`+180/-180°` 在空间中重合，合并为一支浅蓝箭头，避免颜色重叠和遮挡术区。

`MotionPhase` 含义：

- `0`：左半圈处理，等离子开启。
- `1`：左半圈结束后回零，等离子关闭。
- `2`：右半圈处理，等离子开启。
- `3`：右半圈结束后回零，等离子关闭。
- `4`：层间移动到下一层零度 TCP，等离子关闭。

重要限制：这里的 `Joint6GeometricDeg` 是绕残腔蓝轴的几何角，不是机械臂逆解得到的真实关节 6 角度。断能回零当前只描述原位转回零度，没有生成径向退刀、避障回绕或重新接近路径。四元数位于当前模型坐标系且使用临时几何工具轴约定；在手眼标定和真实 TCP 坐标约定完成前不能下发机械臂。

## ROS Launch 与系统自检

ROS launch 管理：

- `src/plasma_gui/src/gui/panel/left_panel/step_wizard/ros_launch_manager/ros_launch_manager.*`

`stop()` / `stopAll()` 使用 SIGINT 方式停止进程，且已经避免阻塞 UI 的等待逻辑。

系统自检：

- `SystemCheckManager`
- 入口：`MainWindow::onRequestSystemCheck()`

当前相机日志不再直接刷主日志大段输出，状态和相机日志窗口由状态栏按钮控制。

## 常用构建命令

```bash
colcon build --packages-select plasma_gui
```

当前构建可能出现 Open3D CMake policy 警告，和业务逻辑无关。

## 后续开发注意点

- 不要让 `setCloudCaptureConfirmEnabled(false)` 影响路径规划页的 `btn2`。
- 路径结果应加入 `轨迹生成` 节点，不应直接显示在 `残腔重建` 节点上。
- 轨迹节点默认只显示蓝色直参考轴，红色弯曲中心线由右键菜单控制。
- 若继续提高路径算法精度，应优先验证体素尺寸、mesh 是否闭合、开口边界是否可靠。
- 下一阶段应将 `NodeCloudData::nozzlePoseSequence` 从模型坐标系转换到机械臂基坐标系，接入真实 TCP 工具坐标和机械臂逆解；随后在回零阶段设计安全退刀/回绕/重新接近动作，再完成碰撞、越界、真实关节 6 转角、剂量、速度和功率检查。当前结果不能直接下发机械臂。
