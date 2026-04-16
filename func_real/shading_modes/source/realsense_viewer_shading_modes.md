# RealSense Viewer 3D 着色模式实现流程分析

本文档详细分析了 RealSense SDK v2.51.1 中 `realsense-viewer` 的 3D 视图着色模式实现，包括 "Raw Point-Cloud"（原始点云）、"Flat-Shaded Mesh"（平面着色网格）和 "With Diffuse Lighting"（漫反射光照）三种模式。

## 1. 概述与模式映射

在 `realsense-viewer` 的 UI 中，三种模式对应代码中的 `shader_type` 枚举。

**文件位置**: `common/viewer.h`

```cpp
enum class shader_type {
    points,  // 对应 "Raw Point-Cloud"
    flat,    // 对应 "Flat-Shaded Mesh"
    diffuse  // 对应 "With Diffuse Lighting"
};
```

## 2. UI 交互与状态切换

用户点击界面上的 Shading 按钮时，会触发 ImGui 的菜单逻辑，更新 `selected_shader` 变量。

**文件位置**: `common/viewer.cpp`

关键代码片段（逻辑示意）：

```cpp
// 在 viewer_model 类的绘制逻辑中
if (ImGui::MenuItem("Raw Point-Cloud", nullptr, &selected))
{
    if (selected) selected_shader = shader_type::points;
}
if (ImGui::MenuItem("Flat-Shaded Mesh", nullptr, &selected))
{
    if (selected) selected_shader = shader_type::flat;
}
if (ImGui::MenuItem("With Diffuse Lighting", nullptr, &selected, glsl_available))
{
    if (selected) selected_shader = shader_type::diffuse;
}
```

## 3. 渲染参数设置 (Render 3D View)

`viewer_model::render_3d_view` 函数负责根据当前的 `selected_shader` 状态，配置点云渲染器 (`_pc_renderer`) 的选项。

**文件位置**: `common/viewer.cpp`
**函数**: `viewer_model::render_3d_view`

该函数通过设置两个关键选项来控制渲染器的行为：
1.  `OPTION_FILLED`: 控制是画点（Points）还是画填充三角形（Mesh）。
2.  `OPTION_SHADED`: 控制是否启用着色器中的光照计算。

```cpp
// 核心逻辑
_pc_renderer.set_option(gl::pointcloud_renderer::OPTION_FILLED, selected_shader != shader_type::points ? 1.f : 0.f);
_pc_renderer.set_option(gl::pointcloud_renderer::OPTION_SHADED, selected_shader == shader_type::diffuse ? 1.f : 0.f);
```

| 模式名称 | shader_type | OPTION_FILLED | OPTION_SHADED | 效果 |
| :--- | :--- | :--- | :--- | :--- |
| Raw Point-Cloud | `points` | 0 (False) | 0 (False) | 绘制离散点，无光照 |
| Flat-Shaded Mesh | `flat` | 1 (True) | 0 (False) | 绘制三角网格，使用纹理颜色，无光照 |
| With Diffuse Lighting | `diffuse` | 1 (True) | 1 (True) | 绘制三角网格，叠加漫反射光照计算 |

## 4. 渲染器实现 (Pointcloud Renderer)

`pointcloud_renderer` 类负责执行具体的 OpenGL 绘制命令。它位于 `src/gl` 目录下，属于 OpenGL 功能模块。

**文件位置**: `src/gl/pc-shader.cpp`
**类**: `pointcloud_renderer`

在 `process_frame` 或绘制相关的方法中，根据 `OPTION_FILLED` 决定绘制方式：

```cpp
// 伪代码逻辑示意
if (_filled_opt->query() > 0.f)
    _model->draw();        // 绘制三角形 (GL_TRIANGLES) - 对应 Mesh 模式
else
    _model->draw_points(); // 绘制点 (GL_POINTS) - 对应 Point-Cloud 模式
```

同时，它会将 `OPTION_SHADED` 的值传递给 GLSL 着色器。

```cpp
shader->set_shaded(_shaded_opt->query());
```

## 5. GLSL 着色器实现 (Shader Implementation)

核心的光照计算逻辑位于 GLSL 片段着色器代码中。

**文件位置**: `src/gl/pc-shader.cpp`

在代码中，着色器源码通常以字符串形式存储（如 `fragment_shader_text`）。

### 漫反射光照算法 (Diffuse Lighting)

当 `shaded` 参数大于 0 时，启用光照计算。代码定义了三个固定的光源方向，计算法线与光源方向的点积（Lambertian reflectance），混合得到最终亮度。

```glsl
// 片段着色器核心逻辑
if (shaded > 0.0) {
    // 定义三个光源方向
    vec3 light0 = vec3(0.0, 1.0, 0.0);
    vec3 light1 = vec3(0.5, -1.0, 0.0);
    vec3 light2 = vec3(-0.5, -1.0, 0.0);

    // 计算当前片元到光源的方向
    vec3 light_dir0 = light0 - vec3(outPos);
    vec3 light_dir1 = light1 - vec3(outPos);
    vec3 light_dir2 = light2 - vec3(outPos);

    // 计算漫反射系数 (Normal dot LightDir)
    float diffuse_factor0 = max(dot(normal,light_dir0), 0.0);
    float diffuse_factor1 = max(dot(normal,light_dir1), 0.0);
    float diffuse_factor2 = max(dot(normal,light_dir2), 0.0);

    // 混合光照强度
    float diffuse_factor = 0.6 + diffuse_factor0 * 0.2 + diffuse_factor1 * 0.2 + diffuse_factor2 * 0.2;

    // 应用光照到纹理颜色
    color = clamp(diffuse_factor, 0.0, 1.0) * color;
}

## 5. 多线程与并发实现分析

在功能实现过程中，RealSense Viewer 采用了典型的**主线程渲染 + 工作线程处理**的模式：

1.  **UI交互与渲染 (主线程)**:
    *   用户在界面上选择 "Shading" 模式（`shader_type`）的操作完全在主 UI 线程（Main Thread）处理。
    *   `viewer_model::render_3d_view` 函数由主渲染循环调用，负责设置 OpenGL 状态和 Uniform 变量。
    *   OpenGL 的绘制调用（Draw Calls）必须在持有 GL Context 的线程（通常是主线程）执行。

2.  **数据生成 (工作线程)**:
    *   虽然 Shading 模式主要涉及渲染阶段，但其依赖的点云数据（Geometry）是由后台工作线程生成的。
    *   `post_processing_filters` 类中持有一个 `processing_block`，通过 `start()` 方法启动一个独立的工作线程。
    *   在该线程中，`pointcloud::process_frame`（或 `gl::pointcloud`）负责将深度图转换为点云数据（如果是 CPU 模式）或准备 GL 纹理（如果是 GPU 模式）。

3.  **线程同步**:
    *   主线程和工作线程之间通过 `rs2::frame_queue` 进行通信。工作线程生成数据后放入队列，主线程在渲染前从队列取出最新帧。
    *   这种设计保证了 UI 的高响应性，即使点云计算耗时较高，也不会卡顿界面。

### 总结

要在自己的项目中复现该功能，你需要：
1.  **构建点云网格**：不仅需要顶点位置（XYZ），还需要生成网格索引（Indices）以构成三角形（用于 Mesh 模式）。
2.  **计算法线**：光照计算依赖于顶点或面法线（Normal），需要在几何着色器或预处理阶段计算好。
3.  **移植 Shader**：将 `src/gl/pc-shader.cpp` 中的 GLSL 代码移植到你的渲染管线中。
4.  **控制逻辑**：实现类似的 Uniform 变量控制（`filled` 和 `shaded` 开关）。
