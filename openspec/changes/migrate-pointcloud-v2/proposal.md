# Change: Migrate PointCloud Display to V2

## Reason
当前 V1 方案在处理高分辨率（1280x720）、高帧率（30FPS+）的点云流时存在明显的性能瓶颈（CPU 占用高、内存分配频繁），且在交互（旋转/缩放）时因 Z-buffer 精度和渲染同步问题导致画面闪烁（花屏）。

## Description
本提案旨在实施 Point V2 方案，以 PointCloud2 为基线，通过以下核心技术升级实现高性能与高稳定性：
1.  **性能优化**：
    *   **迭代器直写**：使用 `sensor_msgs::PointCloud2Iterator` 直接填充 VTK 容器，跳过 PCL 中间转换。
    *   **容器复用**：复用 VTK 对象（Points, Cells, Colors），避免每帧内存分配。
    *   **颜色直传**：使用 DirectScalars 模式，移除 LUT 计算。
2.  **稳定性优化**：
    *   **离群点过滤与 Clipping Range 管理**：解决 Z-fighting 导致的花屏。
    *   **渲染调度优化**：使用 `widget->update()` 接管渲染循环，解决画面撕裂与同步问题。

## Impact
*   **性能**：CPU 占用显著降低，帧率稳定在 30FPS+。
*   **体验**：交互操作（旋转、缩放）流畅无花屏。
*   **架构**：建立可扩展的渲染管线，支持未来进一步 GPU 优化。
