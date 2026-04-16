# Design: Point V2 Architecture

## Context
V1 方案采用 `PCL -> VTK` 的传统转换路径，且每帧重建渲染管线，导致在高负载下性能不足。同时，VTK 默认的 Clipping Range 策略在处理含噪点的深度相机数据时会导致 Z-buffer 精度坍塌，引发交互花屏。

## Goals
1.  **高性能**：最小化 CPU/内存开销，最大化渲染帧率。
2.  **高稳定性**：消除交互时的花屏与卡顿。
3.  **兼容性**：优先复用 PointCloud2 标准接口。

## Decisions
### 1. 数据流优化
*   **决策**：放弃 `pcl::fromROSMsg`，采用 `PointCloud2Iterator` 直写 VTK。
*   **理由**：减少一次全量数据拷贝与中间结构（`pcl::PointCloud`）的构造，降低内存带宽压力。

### 2. 内存管理
*   **决策**：引入对象池/双缓冲或成员变量复用机制。
*   **理由**：避免高频（30Hz）的大块内存分配（`new`/`delete`），减少内存碎片与分配器开销。

### 3. 渲染稳定性
*   **决策**：手动管理 Clipping Range 与离群点过滤。
*   **理由**：自动 Clipping Range 会因噪点导致视锥极长，降低 Z-buffer 有效精度；手动管理可确保精度集中在有效物体区域。
*   **决策**：Qt 事件驱动渲染。
*   **理由**：避免多线程直接调用 `Render()` 导致的 OpenGL 上下文冲突与撕裂，利用 VSync 实现平滑显示。
