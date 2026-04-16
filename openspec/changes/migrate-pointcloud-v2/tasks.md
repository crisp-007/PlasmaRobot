# Tasks: Migrate PointCloud Display to V2

## 1. Preparation & Baseline
- [x] 确认 ROS2 相机节点发布 `/camera/depth/color/points` 及 QoS 设置 (Phase 0)
- [x] 建立 V2 基础文件结构与类封装 (Phase 1)
- [x] 实现基础 PointCloud2 订阅与显示链路 (`RosWorker` -> `PointDeal` -> `MainOpengl`) (Phase 1)

## 2. Performance Optimization (Core)
- [x] 实现 VTK 容器与 Mapper/Filter 的类成员化与复用机制 (Phase 2)
- [x] 实现 `sensor_msgs::PointCloud2Iterator` 直写逻辑 (Phase 2)
- [x] 实现 DirectScalars 颜色渲染配置 (Phase 2)

## 3. Stability Optimization
- [x] 实现离群点（NaN/Inf/Zero）过滤逻辑 (Phase 2/3)
- [x] 实现基于有效 Bounds 的手动 Clipping Range 更新 (Phase 2/3)
- [x] 优化渲染调用，使用 `widget->update()` 替代直接 `Render()` (Phase 2/3)
- [x] 验证 QVTK `QSurfaceFormat` 设置 (Phase 2/3)

## 4. Interaction & Finalization
- [ ] 移植 V1 的裁剪（Crop）、移动、标签交互逻辑 (Phase 3)
- [ ] 验证并修复交互过程中的状态同步问题 (Phase 3)
- [ ] 执行性能验收测试（FPS, Latency, CPU/GPU Usage） (Phase 4)
- [ ] 完善监控日志（转换耗时、渲染耗时） (Phase 4)
