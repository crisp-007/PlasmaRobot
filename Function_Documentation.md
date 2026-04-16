# 点云交互功能文档 (Point Cloud Interaction Documentation)

本文档描述了 PlasmaRobot 项目中点云交互功能的实现细节和使用说明。

## 1. 功能概述

在主界面右侧布局中，移除了原有的 `pointcloud_gl` 和 `robotarm_gl` 控件，替换为一组点云交互按钮。主要提供以下三个功能：

1.  **裁剪 (Crop)**: 使用 3D 包围盒 (Box Widget) 对点云进行裁剪。
2.  **移动 (Move)**: 使用 3D 包围盒对点云进行平移和旋转操作。
3.  **标注 (Label)**: 点击点云上的点，显示该点的坐标信息。

## 2. 实现细节

### 2.1 UI 更改 (`mainwindow.ui`)

-   **移除**: `rightLayout` 中的 `PointCloudgl` 和 `RobotArmgl`。
-   **新增**: `QGroupBox` (标题 "点云交互")，包含三个 `QPushButton`:
    -   `btnCrop`: 开启裁剪模式。
    -   `btnMove`: 开启移动模式。
    -   `btnLabel`: 开启标注模式。

### 2.2 核心逻辑 (`main_gl.cpp` / `main_gl.h`)

`MainOpengl` 类（继承自 `QVTKOpenGLNativeWidget`）负责具体的 VTK 交互实现。

#### 引入的 VTK 组件
-   `vtkBoxWidget2` / `vtkBoxRepresentation`: 用于裁剪和移动的交互框。
-   `vtkPointPicker`: 用于拾取点云上的点。
-   `vtkCaptionActor2D`: 用于显示标注文本。
-   `vtkExtractGeometry` / `vtkPlanes`: 用于裁剪计算（目前实现为视觉裁剪）。

#### 主要接口
-   `void EnableCrop(bool enable)`: 启用/禁用裁剪模式。
-   `void EnableMove(bool enable)`: 启用/禁用移动模式。
-   `void EnableLabel(bool enable)`: 启用/禁用标注模式。

#### 回调函数实现
1.  **CropCallback**:
    -   监听 `vtkBoxWidget2` 的交互事件。
    -   获取 Box 的平面 (`GetPlanes`)。
    -   设置 Actor Mapper 的裁剪平面 (`SetClippingPlanes`)，实现实时视觉裁剪。

2.  **MoveCallback**:
    -   监听 `vtkBoxWidget2` 的交互事件。
    -   获取 Box 的变换矩阵 (`GetTransform`)。
    -   应用变换到点云 Actor (`SetUserTransform`)，实现点云随框移动。

3.  **LabelCallback**:
    -   监听鼠标左键点击事件 (`LeftButtonPressEvent`)。
    -   使用 `vtkPointPicker` 获取点击位置的世界坐标。
    -   创建 `vtkCaptionActor2D` 在点击位置显示坐标 (X, Y, Z)。

### 2.3 信号与槽 (`mainwindow.cpp`)

在 `MainWindow` 中连接了 UI 按钮的 `toggled` 信号到相应的处理函数：
-   `OnCropToggled`: 互斥地开启裁剪模式。
-   `OnMoveToggled`: 互斥地开启移动模式。
-   `OnLabelToggled`: 互斥地开启标注模式。

## 3. 使用说明

1.  **加载点云**: 程序启动时会自动加载默认点云（或通过代码指定）。
2.  **裁剪**:
    -   点击右侧面板的“裁剪”按钮。
    -   主视图中会出现一个白色线框盒。
    -   拖动盒子的面或控制点来调整裁剪范围，点云会被实时裁剪。
    -   再次点击按钮取消模式（裁剪效果保留）。
3.  **移动**:
    -   点击“移动”按钮。
    -   出现控制盒，拖动盒子可平移或旋转整个点云。
4.  **标注**:
    -   点击“标注”按钮。
    -   在点云上点击任意位置，会生成一个红色标签，显示该点的坐标。

## 4. 依赖库

-   VTK (Visualization Toolkit) 8.x/9.x
-   PCL (Point Cloud Library)
-   Qt 5/6

