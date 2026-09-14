# PlasmaRobot

面向等离子处理机器人研究的 ROS 2 + Qt 6 工程。项目包含上位机界面、RealSense 点云采集与处理、残腔重建及路径可视化、睿尔曼机械臂 ROS 2 接口、等离子控制器串口通信，以及相关标定和实验工具。

> 本仓库是科研与工程验证原型，不是经过认证的医疗设备软件。涉及机械臂或等离子设备的操作必须由熟悉设备的人员在急停可用、低速、断能或干运行条件下分阶段验证，不得直接用于临床或无人值守运行。

## 主要功能

- Qt 6 上位机流程：系统自检、残腔采集、残腔重建、术区选择、路径规划与状态显示。
- 点云链路：`ROS 2 PointCloud2 -> RosWorker -> PointDeal -> MainOpengl -> VTK`。
- RealSense L515 相机接入、实时点云显示、裁剪、关键帧保存和 mesh 重建。
- 基于 PCL、VTK、Open3D 的点云与网格处理及三维交互显示。
- 切片平面、轮廓拟合、喷杆几何路径、分段连续路径和喷嘴位姿预览。
- 睿尔曼机械臂驱动、MoveJ/MoveL 控制适配、状态监测和停止保护。
- 等离子控制器串口协议与 STM32/MFC 相关测试工程。
- 手眼标定、深度对齐、TCP/路径坐标变换及离线分析工具。

## 系统结构

```text
RealSense L515
      |
      v
ROS 2 PointCloud2 --> 点云解析/处理 --> VTK 三维显示
                                      |
                                      v
                         重建/术区选择/路径预览
                                      |
                 +--------------------+--------------------+
                 v                                         v
        睿尔曼机械臂 ROS 2 接口                    等离子控制器串口接口
```

## 目录说明

```text
PlasmaRobot/
├─ src/plasma_gui/                 # Qt 6 上位机与点云可视化主程序
├─ src/third_party/plasma_robot/   # 机械臂、标定、坐标变换和路径执行包
├─ src/third_party/realsense2/     # RealSense ROS 2 相关包
├─ src/third_party/plasma_urdf_v2/ # 机器人末端与喷杆 URDF/mesh
├─ plasma_test/                    # STM32、MFC 与通信测试资料
├─ func_real/                      # RealSense/点云实现参考资料
├─ Testing/                        # 历史测试记录
└─ PlasmaRobot.workspace           # ROS 工作区配置
```

大型 `.ply`、`.pcd`、`.stl` 和 CAD 文件由 Git LFS 管理。克隆前请先安装并启用 Git LFS：

```bash
git lfs install
git clone https://github.com/crisp-007/PlasmaRobot.git
```

## 当前环境基线

当前工程文件指向以下开发环境：

- Linux / Jetson ARM64
- ROS 2 Galactic
- Qt 6.7.2
- CMake 3.16+、C++17、colcon、ament_cmake
- PCL、VTK、Open3D、Qhull
- librealsense 2.50 运行时
- 睿尔曼机械臂 ROS 2 驱动及消息包

第三方机械臂文档中还存在 Foxy/Humble 相关内容，因此不能仅凭其 README 选择 ROS 发行版；构建前应以本机依赖、`PlasmaRobot.workspace` 和实际启动脚本为准。

## 构建

在已安装上述依赖的 Linux/Jetson 环境中执行：

```bash
cd ~/CodeSpace/PlasmaRobot
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只重新构建上位机包时可使用：

```bash
colcon build --packages-select plasma_gui --symlink-install
source install/setup.bash
```

## 运行上位机

```bash
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 run plasma_gui plasma_gui
```

相机、机械臂和其他 ROS 2 节点应根据测试目标分别启动。首次连接实机前，应先核对实际话题名称、消息类型、控制器代际、工具坐标系和设备急停状态。

## 已知限制

- `src/plasma_gui/CMakeLists.txt` 当前包含 Qt ARM64 安装路径 `/home/larusxu/Qt/6.7.2/gcc_arm64`。
- GUI 的部分资源、数据输出和 ROS 启动代码仍引用 `/home/larusxu/CodeSpace/PlasmaRobot`。
- `launch_plasma_gui.sh` 使用固定的 ROS 2 Galactic 和 librealsense 2.50 路径。
- 当前手眼矩阵不能默认视为已通过固定标定板一致性验证，不应直接用于自动绝对入口定位。
- 路径规划中的部分结果属于几何预览；在坐标变换、真实 TCP、逆解、碰撞检查和实机验证完成前，不能直接作为机械臂可执行轨迹。
- 不同 ROS 2 发行版、Qt/PCL/VTK/Open3D 版本及图形环境可能需要重新配置和编译。

## 进一步阅读

- [上位机开发交接说明](src/plasma_gui/src/temp_ref/HANDOFF.md)
- [集成工作流](src/plasma_gui/src/temp_ref/INTEGRATED_WORKFLOW.md)
- [相机流恢复记录](src/plasma_gui/src/temp_ref/CAMERA_STREAM_RECOVERY_20260729.md)
- [路径执行模块说明](src/third_party/plasma_robot/tools/motion/plasma_path_executor/README.md)
- [坐标变换模块说明](src/third_party/plasma_robot/tools/tf/plasma_path_transform/README.md)
- [手眼标定模块说明](src/third_party/plasma_robot/tools/eye_hand/README.md)

## License

本仓库根目录代码采用 [MIT License](LICENSE)。第三方组件和资源仍分别受其原始许可证约束。
