#include "main_gl.h"
#include "ros_worker.h"
#include "point_deal.h"
#include <QDebug>
#include <QDir>
#include <QRegularExpression>
#include <vtkRenderWindow.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkVertexGlyphFilter.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkLookupTable.h>
#include <vtkCommand.h>
#include <vtkTransform.h>
#include <vtkBox.h>
#include <vtkPlanes.h>
#include <vtkExtractGeometry.h>
#include <vtkCallbackCommand.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextProperty.h>
#include <vtkPointPicker.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRendererCollection.h>
#include <vtkInteractorStyleDrawPolygon.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkAreaPicker.h>
#include <vtkExtractPolyDataGeometry.h>
#include <vtkImplicitSelectionLoop.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCamera.h>
#include <vtkPolyDataMapper2D.h>
#include <vtkProperty2D.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkSelectPolyData.h>
#include <vtkClipPolyData.h>
#include <vtkPlaneSource.h>
#include <vtkPolygon.h>
#include <vtkLine.h>
#include <vtkPolyLine.h>
#include <vtkGeometryFilter.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <QResizeEvent>
#include <vtkCellData.h>
#include <vtkMath.h>

// ==========================================
// 交互样式工厂宏
// ==========================================
vtkStandardNewMacro(RectDrawStyle);
vtkStandardNewMacro(PolygonDrawStyle);

// ==========================================
// 1. RectDrawStyle 实现 (矩形绘制)
// ==========================================

// 左键按下事件：开始绘制矩形
void RectDrawStyle::OnLeftButtonDown() {
    if (!m_gl) return;
    m_isDrawing = true;
    
    // 获取当前鼠标位置
    int* pos = this->Interactor->GetEventPosition();
    m_startPos[0] = pos[0];
    m_startPos[1] = pos[1];
    
    // 初始化矩形显示（起点和终点相同）
    m_gl->UpdateRectSelection(pos[0], pos[1], pos[0], pos[1]);
    
    // 注意：不调用父类OnLeftButtonDown，避免触发默认的旋转视角行为
}

// 鼠标移动事件：更新矩形形状 或 平移点云
void RectDrawStyle::OnMouseMove() {
    if (m_isDrawing && m_gl) {
        // 正在绘制矩形：更新矩形终点
        int* pos = this->Interactor->GetEventPosition();
        m_gl->UpdateRectSelection(m_startPos[0], m_startPos[1], pos[0], pos[1]);
        
        // 刷新渲染窗口以显示动态更新
        if (m_gl && m_gl->renderWindow()) {
            m_gl->renderWindow()->Render();
        }
    } else if (m_isMoving && m_gl) {
        
    } else {
        // 其他情况：调用父类默认行为（如悬停交互）
        vtkInteractorStyleTrackballCamera::OnMouseMove();
    }
}

// 左键抬起事件：结束矩形绘制
void RectDrawStyle::OnLeftButtonUp() {
    if (m_isDrawing) {
        m_isDrawing = false;
        // 确保渲染窗口最后更新一次
        if (m_gl && m_gl->renderWindow()) {
            m_gl->renderWindow()->Render();
        }
    }
}

// 右键按下事件：保留给平移/扩展交互，当前不处理
void RectDrawStyle::OnRightButtonDown() {
}

// 右键抬起事件：保留给平移/扩展交互，当前不处理
void RectDrawStyle::OnRightButtonUp() {
}
// ==========================================
// 2. PolygonDrawStyle 实现 (多边形绘制)
// ==========================================

// 左键按下事件：添加多边形顶点
void PolygonDrawStyle::OnLeftButtonDown() {
    if (!m_gl) return;
    int* pos = this->Interactor->GetEventPosition();
    
    // 严格校验：避免无效点（如窗口边缘误触）
    if (pos[0] <= 1 && pos[1] <= 1) return;

    // 左键点击添加顶点到多边形
    m_gl->AddPolyPoint(pos[0], pos[1]);
}

// 鼠标移动事件：更新多边形动态预览线 或 平移点云
void PolygonDrawStyle::OnMouseMove() {
    if (m_gl) {
         int* pos = this->Interactor->GetEventPosition();
         
         // 严格校验
         if (pos[0] <= 1 && pos[1] <= 1) return;

         // 如果多边形已开始绘制，更新从最后一个点到当前鼠标位置的动态连线
         m_gl->UpdatePolyDynamicLine(pos[0], pos[1]);
         if (m_gl->renderWindow()) {
             m_gl->renderWindow()->Render();
         }
    }
}

// 左键抬起事件：空操作
void PolygonDrawStyle::OnLeftButtonUp() {
    // 释放不做任何事，等待下一次点击添加新点
}

// 右键按下事件：结束多边形绘制 或 开始平移
void PolygonDrawStyle::OnRightButtonDown() {
    if (m_gl) {
        // 策略：右键按下时，尝试闭合多边形
        m_gl->FinishPolySelection();
        if (m_gl->renderWindow()) {
            m_gl->renderWindow()->Render();
        }
    }
}

// 右键抬起事件：结束平移
void PolygonDrawStyle::OnRightButtonUp() {

}

// ==========================================
// MainOpengl: 主渲染
// ==========================================

// 构造函数
MainOpengl::MainOpengl(QWidget *parent)
    : QVTKOpenGLNativeWidget(parent), m_currentCropMode(Crop_None), m_cropOverlay(nullptr), m_pointDeal(nullptr)
{
    // 初始化VTK环境
    InitVTK();

    // 初始化点云处理类
    m_pointDeal = new PointDeal(this);
    connect(m_pointDeal, &PointDeal::cropFinished, this, &MainOpengl::onCropFinished);
    connect(m_pointDeal, &PointDeal::errorOccurred, this, &MainOpengl::onErrorOccurred);
    
    // 创建裁剪功能悬浮窗，并初始化位置
    m_cropOverlay = new CropOverlayWidget(this);
    m_cropOverlay->hide();
    UpdateCropOverlayPosition();

    // --- 信号连接 ---
    // 裁剪模式切换（矩形/多边形）
    connect(m_cropOverlay, &CropOverlayWidget::modeChanged, this, [this](int mode){
        if (mode == 0) EnableCrop(true, Crop_Rect);
        else EnableCrop(true, Crop_Poly);
    });
    
    // 内部/外部裁剪模式切换
    connect(m_cropOverlay, &CropOverlayWidget::typeChanged, this, [this](bool inside){
        m_cropInside = inside;
        qDebug() << "Crop type changed to:" << (inside ? "Inside" : "Outside");
    });

    // 确认裁剪按钮点击
    connect(m_cropOverlay, &CropOverlayWidget::confirmClicked, this, &MainOpengl::DoCrop);
    
    // 取消裁剪按钮点击
    connect(m_cropOverlay, &CropOverlayWidget::cancelClicked, this, [this](){
        EnableCrop(false);
    });

    // 初始化 PointDeal 信号 (用于 ROS 点云显示)
    connect(m_pointDeal, &PointDeal::rosCloudFinished, this, &MainOpengl::onRosCloudFinished);

    // 初始化 ROS Worker 线程
    m_workerThread = new QThread(this);
    m_rosWorker = new RosWorker(); // 注意：parent 不能是 this，因为要 moveToThread
    m_rosWorker->moveToThread(m_workerThread);
    
    // 线程启动时，启动 Worker
    connect(m_workerThread, &QThread::started, m_rosWorker, &RosWorker::start);
    // 线程结束时，销毁 Worker
    connect(m_workerThread, &QThread::finished, m_rosWorker, &QObject::deleteLater);
    
    // ROS 数据 -> PointDeal 转换
    connect(m_rosWorker, &RosWorker::cloudReceived, m_pointDeal, &PointDeal::requestConvertRosToVtk);

    // 启动 ROS 线程
    m_workerThread->start();
}

// 析构函数
MainOpengl::~MainOpengl()
{
    // 停止并清理 ROS 线程
    if (m_rosWorker) {
        // 直接调用 stop 以触发 rclcpp::shutdown()，中断 spin() 的阻塞
        // 注意：不要使用 invokeMethod(BlockingQueuedConnection)，因为 worker 线程
        // 正阻塞在 spin() 中，无法响应信号槽事件，会导致死锁或无限等待。
        m_rosWorker->stop();
    }
    if (m_workerThread) {
        m_workerThread->quit();
        m_workerThread->wait();
    }
}

// 初始化VTK相关组件
void MainOpengl::InitVTK()
{
    // 1. 初始化VTK渲染器和窗口
    m_renderer = vtkSmartPointer<vtkRenderer>::New(); // 渲染器：管理场景中的Actor和相机
    m_renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New(); // 渲染窗口：OpenGL上下文
    setRenderWindow(m_renderWindow); // 将渲染窗口绑定到Qt控件
    
    if (renderWindow()) {
        renderWindow()->AddRenderer(m_renderer);
        
        // 设置背景颜色（深色背景，增加对比度）-深石板灰到黑色的渐变
        SetBackgroundColor(QColor(50, 50, 60), QColor(10, 10, 15));
        
        // 初始化并显示默认网格（m）
        // 默认较大的网格 (24m)，网格颜色: 深灰(100,100,100), 中心线: 浅灰(200,200,200)
        SetGridProperties(24.0, 1.0, QColor(100, 100, 100), QColor(200, 200, 200));
        
        // 设置初始相机视角 (RealSense 风格)
        SetCamera();

        // 2. 设置默认交互样式
        // 使用默认的InteractorStyleTrackballCamera，支持右键缩放等操作
        vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
        m_defaultStyle = style;
        renderWindow()->GetInteractor()->SetInteractorStyle(m_defaultStyle);

        // 3. 初始化坐标轴 Widget
        vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
        m_axesWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
        m_axesWidget->SetOrientationMarker(axes);
        m_axesWidget->SetInteractor(renderWindow()->GetInteractor());
        m_axesWidget->SetViewport(0.0, 0.0, 0.2, 0.2); // 左下角显示
        m_axesWidget->SetEnabled(1);
        m_axesWidget->InteractiveOff(); // 禁止拖动坐标轴本身
    }

    // 初始化点云Actor容器
    m_pointsActor = vtkSmartPointer<vtkActor>::New();

    // --- V2 优化：初始化复用对象 ---
    // 1. vtkVertexGlyphFilter: 顶点符号化滤波器
    // 作用: 将输入的点数据(vtkPolyData)中的每个点，转换成一个可视化的几何图元(默认为单个像素的点)。
    // 为什么需要: 在某些旧版本 VTK 或特定显卡驱动下，直接渲染点云可能不可见或效率低，
    // 使用 GlyphFilter 可以确保点云以标准的 GL_POINTS 或其他形式正确渲染。
    // 它也常用于将点渲染为球体、立方体等，但这里默认用法是将其“符号化”为可渲染的顶点。
    m_glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();

    // 2. vtkPolyDataMapper: 多边形数据映射器
    // 作用: 将几何数据(vtkPolyData) 映射为 图形库(OpenGL) 的图元。
    // 它负责把数据(坐标、颜色、法向量等)转换为显卡能理解的绘制指令。
    // 它是连接数据处理管道(Pipeline)和渲染系统(Rendering System)的桥梁。
    m_cloudMapper = vtkSmartPointer<vtkPolyDataMapper>::New();

    // 3. 建立静态连接管道 (Pipeline Connection)
    // 数据流向: [输入数据] -> m_glyphFilter -> m_cloudMapper -> m_pointsActor
    // 这里建立了静态连接，后续只要调用 m_glyphFilter->SetInputData(newData) 并渲染，
    // 整个管道会自动更新，无需每次都重新创建 Mapper 或 Actor。
    m_cloudMapper->SetInputConnection(m_glyphFilter->GetOutputPort());
    
    // 4. 将 Mapper 设置给 Actor
    // Actor 负责在场景中表示这个物体，包含位置、旋转、缩放等变换信息，以及材质属性。
    m_pointsActor->SetMapper(m_cloudMapper);
    
    // 4. 初始化裁剪相关的辅助显示Actor
    // 矩形选择框 Actor
    m_rectActor = vtkSmartPointer<vtkActor2D>::New();
    vtkSmartPointer<vtkPolyDataMapper2D> rectMapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    // 初始化一个空的 PolyData 以避免 "No input" 错误
    vtkSmartPointer<vtkPolyData> emptyPoly = vtkSmartPointer<vtkPolyData>::New();
    rectMapper->SetInputData(emptyPoly);
    m_rectActor->SetMapper(rectMapper);
    m_rectActor->GetProperty()->SetColor(0.0, 1.0, 0.0); // 绿色线条
    m_rectActor->GetProperty()->SetLineWidth(2.0);
    m_rectActor->VisibilityOff(); // 默认隐藏
    m_renderer->AddActor(m_rectActor);
    
    // 多边形动态连线 Actor
    m_polyLineActor = vtkSmartPointer<vtkActor2D>::New();
    vtkSmartPointer<vtkPolyDataMapper2D> polyMapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    polyMapper->SetInputData(emptyPoly); // 同样设置空的 Input
    m_polyLineActor->SetMapper(polyMapper);
    m_polyLineActor->GetProperty()->SetColor(0.0, 1.0, 0.0);
    m_polyLineActor->GetProperty()->SetLineWidth(2.0);
    m_renderer->AddActor(m_polyLineActor);
}

// --------------------------------------------------------
// 数据加载
// --------------------------------------------------------

// 加载PCD文件
void MainOpengl::LoadPCDPoint(const QString &filePath)
{
    if (filePath.isEmpty()) return;

    // 1. 读取PCD文件（兼容 RGB 点云与纯 XYZ 点云）
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filePath.toStdString(), *cloudRgb) != -1) {
        for (const auto &point : cloudRgb->points) {
            points->InsertNextPoint(point.x * 1000, point.y * 1000, point.z * 1000); // m -> mm
            colors->InsertNextTuple3(point.r, point.g, point.b);
        }
    } else {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXyz(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filePath.toStdString(), *cloudXyz) == -1) {
            qDebug() << "Couldn't read file " << filePath;
            return;
        }

        for (const auto &point : cloudXyz->points) {
            points->InsertNextPoint(point.x * 1000, point.y * 1000, point.z * 1000); // m -> mm
            // 无颜色字段时使用浅灰色
            colors->InsertNextTuple3(220, 220, 220);
        }
    }

    // 2. 转换为VTK PolyData
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->GetPointData()->SetScalars(colors);

    // 3. 渲染管线更新
    // 不再重新创建Mapper和Actor，而是更新现有管线的输入
    
    // V2 优化：使用 VertexGlyphFilter 将点渲染为顶点
    m_glyphFilter->SetInputData(polyData);
    m_glyphFilter->Update();

    // 4. 将Actor添加到场景（如果还没添加）
    if (m_pointsActor) {
        m_renderer->AddActor(m_pointsActor);
    }
    
    // 5. 自动调整相机视角和网格
    if (renderWindow()) {
        // 恢复到默认相机视角
        SetCamera();
        
        renderWindow()->Render();
    }
}

// 处理 ROS 点云转换完成信号
void MainOpengl::onRosCloudFinished(vtkSmartPointer<vtkPolyData> polyData)
{
    if (!polyData) return;

    // 1. 更新渲染管线输入
    m_glyphFilter->SetInputData(polyData);
    m_glyphFilter->Update();

    // 2. 确保Actor在场景中
    if (m_pointsActor) {
        m_renderer->AddActor(m_pointsActor);
    }

    // 3. 刷新渲染
    if (renderWindow()) {
        // ROS 实时流通常不自动重置相机，以免视角乱跳
        // 但可以根据需要更新网格中心等
        
        // 首次接收时，进行自适应调整
        if (m_isFirstRosFrame) {            
            // 使用标准 RealSense 视角
            SetCamera();
            m_isFirstRosFrame = false;
        }
        
        renderWindow()->Render();
    }
}

// 暂停/恢复 ROS 数据更新
void MainOpengl::SetRosPaused(bool paused)
{
    if (m_rosWorker) {
        m_rosWorker->setPaused(paused);
        qDebug() << (paused ? "ROS Subscription Paused" : "ROS Subscription Resumed");
    }
}

// --------------------------------------------------------
// 可视化设置
// --------------------------------------------------------

// 设置网格属性
void MainOpengl::SetGridProperties(double gridSize, double step, const QColor &gridColor, const QColor &centerColor)
{
    // 移除旧网格
    if (m_gridActor) {
        m_renderer->RemoveActor(m_gridActor);
    }
    
    // 创建新网格    
    vtkSmartPointer<vtkPolyData> gridPoly = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");

    // 网格参数
    double size = gridSize;      // 总边长
    double cellSize = step;      // 单元格边长
    int n = size / cellSize;     // 分割数
    double offset = -size / 2.0; // 起始偏移

    // 颜色定义
    unsigned char gColor[3] = { (unsigned char)gridColor.red(), (unsigned char)gridColor.green(), (unsigned char)gridColor.blue() };
    unsigned char cColor[3] = { (unsigned char)centerColor.red(), (unsigned char)centerColor.green(), (unsigned char)centerColor.blue() };

    // 生成网格线
    for (int i = 0; i <= n; i++) {
        double pos = offset + i * cellSize;
        
        // 平行于 X 轴的线
        vtkIdType p1 = points->InsertNextPoint(offset, 1.0, pos);
        vtkIdType p2 = points->InsertNextPoint(offset + size, 1.0, pos);
        
        vtkSmartPointer<vtkLine> lineX = vtkSmartPointer<vtkLine>::New();
        lineX->GetPointIds()->SetId(0, p1);
        lineX->GetPointIds()->SetId(1, p2);
        lines->InsertNextCell(lineX);
        
        if (std::abs(pos) < 0.001) colors->InsertNextTuple3(cColor[0], cColor[1], cColor[2]);
        else colors->InsertNextTuple3(gColor[0], gColor[1], gColor[2]);

        // 平行于 Z 轴的线
        vtkIdType p3 = points->InsertNextPoint(pos, 1.0, offset);
        vtkIdType p4 = points->InsertNextPoint(pos, 1.0, offset + size);
        
        vtkSmartPointer<vtkLine> lineZ = vtkSmartPointer<vtkLine>::New();
        lineZ->GetPointIds()->SetId(0, p3);
        lineZ->GetPointIds()->SetId(1, p4);
        lines->InsertNextCell(lineZ);
        
        if (std::abs(pos) < 0.001) colors->InsertNextTuple3(cColor[0], cColor[1], cColor[2]);
        else colors->InsertNextTuple3(gColor[0], gColor[1], gColor[2]);
    }

    gridPoly->SetPoints(points);
    gridPoly->SetLines(lines);
    gridPoly->GetCellData()->SetScalars(colors); // 使用 CellData 给线段着色

    // 创建 Mapper 和 Actor
    vtkSmartPointer<vtkPolyDataMapper> gridMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    gridMapper->SetInputData(gridPoly);
    
    m_gridActor = vtkSmartPointer<vtkActor>::New();
    m_gridActor->SetMapper(gridMapper);
    m_gridActor->GetProperty()->SetLineWidth(1.0);
    
    // 添加到场景
    m_renderer->AddActor(m_gridActor);
}

// 设置相机参数
void MainOpengl::SetCamera()
{
    if (!m_renderer) return;
    vtkCamera* camera = m_renderer->GetActiveCamera();
    if (!camera) return;
    // 相机位置（默认俯视图）
    // 注意：系统单位为 m
    camera->SetPosition(0.0, 0.0, -0.5);  // 0.5m
    // 观察目标点（固定在原点，确保点云居中）
    camera->SetFocalPoint(0.0, 0.0, 0.0);
    // 上方向向量（VTK 自动计算，也可手动指定）
    camera->SetViewUp(0.0, -1.0, 0.0);
    // 视场角（45 度标准视角）
    camera->SetViewAngle(45.0);
    // 裁剪面（米 -> 毫米）
    camera->SetClippingRange(0.001, 100.0); // 0.001m,100m
}

// 设置背景颜色
void MainOpengl::SetBackgroundColor(const QColor &topColor, const QColor &bottomColor)
{
    if (m_renderer) {
        m_renderer->SetBackground(topColor.redF(), topColor.greenF(), topColor.blueF());
        m_renderer->SetBackground2(bottomColor.redF(), bottomColor.greenF(), bottomColor.blueF());
        m_renderer->GradientBackgroundOn();
    }
}

// --------------------------------------------------------
// 裁剪系统
// --------------------------------------------------------

// 启用/禁用裁剪功能
void MainOpengl::EnableCrop(bool enable, CropMode mode)
{
    if (enable) {
        m_currentCropMode = mode;
        m_cropOverlay->show();
        UpdateCropOverlayPosition();
        
        if (mode == Crop_Rect) {
            // 设置交互样式为矩形绘制
            vtkSmartPointer<RectDrawStyle> style = vtkSmartPointer<RectDrawStyle>::New();
            style->SetMainOpengl(this);
            renderWindow()->GetInteractor()->SetInteractorStyle(style);
        } else if (mode == Crop_Poly) {
            // 设置交互样式为多边形绘制
            vtkSmartPointer<PolygonDrawStyle> style = vtkSmartPointer<PolygonDrawStyle>::New();
            style->SetMainOpengl(this);
            renderWindow()->GetInteractor()->SetInteractorStyle(style);
            
            // 准备多边形选择
            StartPolySelection();
        }
    } else {
        m_currentCropMode = Crop_None;
        m_cropOverlay->hide();
        
        // 隐藏所有选择框
        m_rectActor->VisibilityOff();
        
        // 恢复默认交互样式 (平移/旋转)
        renderWindow()->GetInteractor()->SetInteractorStyle(m_defaultStyle);
        
        renderWindow()->Render();
    }
}

// 执行裁剪操作
void MainOpengl::DoCrop()
{
    if (!m_pointsActor) return;
    
    vtkSmartPointer<vtkPolyData> inputCloud = vtkPolyData::SafeDownCast(m_pointsActor->GetMapper()->GetInput());
    if (!inputCloud) return;

    if (m_currentCropMode == Crop_Rect) {
        // 矩形裁剪逻辑
        // 1. 获取屏幕选择区域 (x1, y1, x2, y2)
        // 注意：VTK坐标系原点在左下角
        
        if (!m_renderer) return;
        
        // 将屏幕坐标转换为世界坐标的视锥体
        vtkSmartPointer<vtkAreaPicker> areaPicker = vtkSmartPointer<vtkAreaPicker>::New();
        areaPicker->AreaPick(m_rectSelection[0], m_rectSelection[1], 
                             m_rectSelection[2], m_rectSelection[3], m_renderer);
        
        vtkSmartPointer<vtkImplicitFunction> frustum = areaPicker->GetFrustum();
        
        // 2. 使用 ExtractGeometry 提取
        vtkSmartPointer<vtkExtractGeometry> extract = vtkSmartPointer<vtkExtractGeometry>::New();
        extract->SetImplicitFunction(frustum);
        extract->SetInputData(inputCloud);
        extract->SetExtractInside(m_cropInside); // true=保留内部, false=保留外部
        extract->Update();
        
        // vtkExtractGeometry 输出的是 vtkUnstructuredGrid，需要转换为 vtkPolyData
        vtkSmartPointer<vtkGeometryFilter> geometryFilter = vtkSmartPointer<vtkGeometryFilter>::New();
        geometryFilter->SetInputConnection(extract->GetOutputPort());
        geometryFilter->Update();

        vtkSmartPointer<vtkPolyData> result = vtkSmartPointer<vtkPolyData>::New();
        result->ShallowCopy(geometryFilter->GetOutput());
        
        // 3. 触发完成信号
        onCropFinished(result);
        
    } else if (m_currentCropMode == Crop_Poly) {
        // 多边形裁剪逻辑
        if (m_polyPoints->GetNumberOfPoints() < 3) return;
        
        // 1. 构建多边形选择回路
        vtkSmartPointer<vtkImplicitSelectionLoop> loop = vtkSmartPointer<vtkImplicitSelectionLoop>::New();
        loop->SetLoop(m_polyPoints);
        
        // 注意：ImplicitSelectionLoop 默认是无限延伸的柱状体
        // 需要结合相机方向来确定投影
        vtkCamera* camera = m_renderer->GetActiveCamera();
        double normal[3];
        camera->GetViewPlaneNormal(normal);
        loop->SetNormal(normal);
        
        // 2. 使用 ExtractPolyDataGeometry (专门针对PolyData，比ExtractGeometry快)
        vtkSmartPointer<vtkExtractPolyDataGeometry> extract = vtkSmartPointer<vtkExtractPolyDataGeometry>::New();
        extract->SetInputData(inputCloud);
        extract->SetImplicitFunction(loop);
        extract->SetExtractInside(m_cropInside);
        extract->Update();
        
        vtkSmartPointer<vtkPolyData> result = vtkSmartPointer<vtkPolyData>::New();
        result->ShallowCopy(extract->GetOutput());
        
        onCropFinished(result);
    }
}

// 裁剪完成回调
void MainOpengl::onCropFinished(vtkSmartPointer<vtkPolyData> resultCloud)
{
    if (!resultCloud || resultCloud->GetNumberOfPoints() == 0) {
        qDebug() << "Crop result is empty!";
        return;
    }
    
    // 更新显示
    m_glyphFilter->SetInputData(resultCloud);
    m_glyphFilter->Update();
    
    renderWindow()->Render();
    
    // 退出裁剪模式
    EnableCrop(false);
}

// --------------------------------------------------------
// 交互辅助
// --------------------------------------------------------

void MainOpengl::UpdateRectSelection(int x1, int y1, int x2, int y2)
{
    // 记录选择区域
    m_rectSelection[0] = std::min(x1, x2);
    m_rectSelection[1] = std::min(y1, y2);
    m_rectSelection[2] = std::max(x1, x2);
    m_rectSelection[3] = std::max(y1, y2);
    
    // 更新矩形Actor
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(x1, y1, 0);
    points->InsertNextPoint(x2, y1, 0);
    points->InsertNextPoint(x2, y2, 0);
    points->InsertNextPoint(x1, y2, 0);
    // 闭合
    points->InsertNextPoint(x1, y1, 0); 
    
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();
    polyLine->GetPointIds()->SetNumberOfIds(5);
    for(int i=0; i<5; i++) polyLine->GetPointIds()->SetId(i, i);
    lines->InsertNextCell(polyLine);
    
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetLines(lines);
    
    // 使用 Coordinate 系统将 2D 像素坐标映射到 Overlay 平面
    vtkSmartPointer<vtkCoordinate> coordinate = vtkSmartPointer<vtkCoordinate>::New();
    coordinate->SetCoordinateSystemToDisplay();
    
    vtkSmartPointer<vtkPolyDataMapper2D> mapper = vtkPolyDataMapper2D::SafeDownCast(m_rectActor->GetMapper());
    mapper->SetTransformCoordinate(coordinate);
    mapper->SetInputData(polyData);
    
    m_rectActor->VisibilityOn();
}

void MainOpengl::StartPolySelection()
{
    m_polyPoints = vtkSmartPointer<vtkPoints>::New();
    m_polyLineActor->VisibilityOn();
}

void MainOpengl::AddPolyPoint(int x, int y)
{
    if (!m_polyPoints) return;
    
    // 将屏幕坐标(x,y) 转换为世界坐标，或者直接存储屏幕坐标用于后续处理
    // 这里我们存储屏幕坐标，在 DoCrop 时再生成视锥或投影
    // 但为了显示连线，我们需要屏幕坐标
    
    m_polyPoints->InsertNextPoint(x, y, 0);
    
    // 更新连线显示
    // (逻辑类似 UpdateRectSelection)
}

void MainOpengl::UpdatePolyDynamicLine(int x, int y)
{
    // 绘制从最后一个顶点到当前鼠标位置 (x,y) 的线
}

void MainOpengl::FinishPolySelection()
{
    // 触发裁剪
    DoCrop();
}

void MainOpengl::resizeEvent(QResizeEvent *event)
{
    QVTKOpenGLNativeWidget::resizeEvent(event);
    UpdateCropOverlayPosition();
}

void MainOpengl::UpdateCropOverlayPosition()
{
    if (m_cropOverlay) {
        // 将悬浮窗定位在窗口顶部中间
        int w = width();
        int h = height();
        int ow = m_cropOverlay->width();
        int oh = m_cropOverlay->height();
        
        m_cropOverlay->move((w - ow) / 2, 20); // 顶部 20px 边距
    }
}

void MainOpengl::onErrorOccurred(const QString &errorMsg)
{
    qDebug() << "PointDeal Error:" << errorMsg;
}

void MainOpengl::EnableMove(bool enable)
{
    if (enable) {
        // 切换到移动模式
        // 这里可以实现类似 PointcloudMoveStyle 的逻辑
        // 简单起见，我们假设默认模式就是 TrackballCamera (左键旋转，Shift+左键平移)
        // 如果要专门的平移模式（如右键平移），需要切换 InteractorStyle
        
        // 恢复默认样式 (PointcloudMoveStyle 支持右键平移)
        renderWindow()->GetInteractor()->SetInteractorStyle(m_defaultStyle);
    }
}

void MainOpengl::EnableLabel(bool enable)
{
    // 标签功能暂未实现
    // 可以切换到 vtkInteractorStyleRubberBandPick 进行点选
    if (enable) {
        // TODO: 实现标签交互
    } else {
        renderWindow()->GetInteractor()->SetInteractorStyle(m_defaultStyle);
    }
}

void MainOpengl::MoveActor(double dx, double dy)
{
    // 由 PointcloudMoveStyle 调用
}

void MainOpengl::UpdateActorMatrix(vtkMatrix4x4* matrix)
{
    // 可以在这里通知外部（如 PointDeal）矩阵发生了变化
    // m_pointDeal->SetTransform(matrix);
}
