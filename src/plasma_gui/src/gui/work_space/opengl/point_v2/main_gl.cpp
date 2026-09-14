#include "main_gl.h"
#include "ros_worker.h"
#include "point_deal.h"
#include "NodeCloudData.h"
#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QRegularExpression>
#include <vtkRenderWindow.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkArrowSource.h>
#include <vtkGlyph3D.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
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
#include <vtkImplicitBoolean.h>
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
#include <vtkPLYReader.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <QResizeEvent>
#include <vtkCellData.h>
#include <vtkIntArray.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkCommand.h>
#include <vtkPlane.h>
#include <vector>
#include <algorithm>
#include <limits>

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
    connect(m_pointDeal, &PointDeal::reconstructionProgress,
            this, &MainOpengl::reconstructionProgress);
    connect(m_pointDeal, &PointDeal::reconstructionLog,
            this, &MainOpengl::reconstructionLog);
    connect(m_pointDeal, &PointDeal::reconstructionFinished,
            this, &MainOpengl::reconstructionFinished);
    connect(m_pointDeal, &PointDeal::pathPlanningProgress,
            this, &MainOpengl::pathPlanningProgress);
    connect(m_pointDeal, &PointDeal::pathPlanningLog,
            this, &MainOpengl::pathPlanningLog);
    connect(m_pointDeal, &PointDeal::pathPlanningFinished,
            this, &MainOpengl::pathPlanningFinished);
    connect(m_pointDeal, &PointDeal::slicePlanningProgress,
            this, &MainOpengl::slicePlanningProgress);
    connect(m_pointDeal, &PointDeal::slicePlanningLog,
            this, &MainOpengl::slicePlanningLog);
    connect(m_pointDeal, &PointDeal::slicePlanningFinished,
            this, &MainOpengl::slicePlanningFinished);
    connect(m_pointDeal, &PointDeal::sliceContourProgress,
            this, &MainOpengl::sliceContourProgress);
    connect(m_pointDeal, &PointDeal::sliceContourLog,
            this, &MainOpengl::sliceContourLog);
    connect(m_pointDeal, &PointDeal::sliceContourFinished,
            this, &MainOpengl::sliceContourFinished);
    connect(m_pointDeal, &PointDeal::contourFittingProgress,
            this, &MainOpengl::contourFittingProgress);
    connect(m_pointDeal, &PointDeal::contourFittingLog,
            this, &MainOpengl::contourFittingLog);
    connect(m_pointDeal, &PointDeal::contourFittingFinished,
            this, &MainOpengl::contourFittingFinished);
    connect(m_pointDeal, &PointDeal::equalDosePathProgress,
            this, &MainOpengl::equalDosePathProgress);
    connect(m_pointDeal, &PointDeal::equalDosePathLog,
            this, &MainOpengl::equalDosePathLog);
    connect(m_pointDeal, &PointDeal::equalDosePathFinished,
            this, &MainOpengl::equalDosePathFinished);
    connect(m_pointDeal, &PointDeal::continuousPathProgress,
            this, &MainOpengl::continuousPathProgress);
    connect(m_pointDeal, &PointDeal::continuousPathLog,
            this, &MainOpengl::continuousPathLog);
    connect(m_pointDeal, &PointDeal::continuousPathFinished,
            this, &MainOpengl::continuousPathFinished);
    connect(m_pointDeal, &PointDeal::nozzlePoseProgress,
            this, &MainOpengl::nozzlePoseProgress);
    connect(m_pointDeal, &PointDeal::nozzlePoseLog,
            this, &MainOpengl::nozzlePoseLog);
    connect(m_pointDeal, &PointDeal::nozzlePoseFinished,
            this, &MainOpengl::nozzlePoseFinished);
    
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
        if (m_surgicalAreaTarget) {
            auto target = m_surgicalAreaTarget;
            m_surgicalAreaTarget.reset();
            EnableCrop(false);
            emit surgicalAreaSelectionFinished(target, false);
            return;
        }
        EnableCrop(false);
        emit cropCancelled();
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

        vtkSmartPointer<vtkCallbackCommand> hideBboxCallback = vtkSmartPointer<vtkCallbackCommand>::New();
        hideBboxCallback->SetClientData(this);
        hideBboxCallback->SetCallback([](vtkObject *, unsigned long, void *clientData, void *) {
            auto *self = static_cast<MainOpengl *>(clientData);
            if (self)
                self->HideActiveBoundingBox();
        });
        renderWindow()->GetInteractor()->AddObserver(vtkCommand::LeftButtonPressEvent, hideBboxCallback);
        renderWindow()->GetInteractor()->AddObserver(vtkCommand::MiddleButtonPressEvent, hideBboxCallback);
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
    m_cloudMapper->SetScalarModeToUsePointData();
    m_cloudMapper->SetColorModeToDirectScalars();
    m_cloudMapper->ScalarVisibilityOn();

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
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(filePath.toStdString(), *cloudRgb) != -1) {
        for (const auto &point : cloudRgb->points) {
            points->InsertNextPoint(point.x, point.y, point.z); // 单位: m
            colors->InsertNextTuple3(point.r, point.g, point.b);
        }
    } else {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXyz(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(filePath.toStdString(), *cloudXyz) == -1) {
            qDebug() << "Couldn't read file " << filePath;
            return;
        }

        for (const auto &point : cloudXyz->points) {
            points->InsertNextPoint(point.x, point.y, point.z); // 单位: m
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
    if (m_pointsActor && !m_renderer->HasViewProp(m_pointsActor)) {
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
    if (m_pointsActor && !m_renderer->HasViewProp(m_pointsActor)) {
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
        qDebug() << (paused ? "PointCloud topic display paused" : "PointCloud topic display resumed");
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
    // 裁剪面（米）
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
        m_isPolyFinished = false;
        m_hasPolySelection = false;
        m_hasRectSelection = false;
        m_rectSelection[0] = m_rectSelection[1] = m_rectSelection[2] = m_rectSelection[3] = 0;
        m_cropOverlay->show();
        UpdateCropOverlayPosition();
        
        // 切换模式时清除上一次的选择可视化
        m_rectActor->VisibilityOff();
        m_polyLineActor->VisibilityOff();
        
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
        m_isPolyFinished = false;
        m_hasPolySelection = false;
        m_hasRectSelection = false;
        m_polyPoints = nullptr;
        m_polyWorldPoints = nullptr;
        m_cropOverlay->hide();
        
        // 隐藏所有选择框
        m_rectActor->VisibilityOff();
        m_polyLineActor->VisibilityOff();
        if (auto *polyMapper = vtkPolyDataMapper2D::SafeDownCast(m_polyLineActor->GetMapper())) {
            vtkSmartPointer<vtkPolyData> empty = vtkSmartPointer<vtkPolyData>::New();
            polyMapper->SetInputData(empty);
        }
        
        // 恢复默认交互样式 (平移/旋转)
        renderWindow()->GetInteractor()->SetInteractorStyle(m_defaultStyle);
        
        renderWindow()->Render();
        
    }
}

// 执行裁剪操作
void MainOpengl::DoCrop()
{
    if (m_surgicalAreaTarget) {
        vtkSmartPointer<vtkPolyData> selectedPoints =
            cropPolyDataWithCurrentSelection(m_surgicalAreaTarget->polyData);
        vtkSmartPointer<vtkPolyData> mesh = loadMeshPolyData(m_surgicalAreaTarget);
        vtkSmartPointer<vtkPolyData> selectedMesh =
            cropPolyDataWithCurrentSelection(mesh);

        const bool pointsOk = selectedPoints && selectedPoints->GetNumberOfPoints() > 0;
        const bool meshOk = selectedMesh && selectedMesh->GetNumberOfPoints() > 0;
        const bool ok = meshOk;
        if (ok) {
            if (pointsOk) {
                m_surgicalAreaTarget->surgicalPointCloud = vtkSmartPointer<vtkPolyData>::New();
                m_surgicalAreaTarget->surgicalPointCloud->DeepCopy(selectedPoints);
            } else {
                m_surgicalAreaTarget->surgicalPointCloud = nullptr;
            }
            m_surgicalAreaTarget->surgicalMesh = vtkSmartPointer<vtkPolyData>::New();
            m_surgicalAreaTarget->surgicalMesh->DeepCopy(selectedMesh);
            updateSurgicalOverlay(m_surgicalAreaTarget);
        }

        auto finishedTarget = m_surgicalAreaTarget;
        m_surgicalAreaTarget.reset();
        EnableCrop(false);
        emit surgicalAreaSelectionFinished(finishedTarget, ok);
        return;
    }

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
        if (!m_polyWorldPoints || m_polyWorldPoints->GetNumberOfPoints() < 3) return;
        
        // 1. 构建多边形选择回路（使用世界坐标，在 FinishPolySelection 时已锁定）
        vtkSmartPointer<vtkImplicitSelectionLoop> loop = vtkSmartPointer<vtkImplicitSelectionLoop>::New();
        loop->SetLoop(m_polyWorldPoints);
        
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
        EnableCrop(false);
        return;
    }
    
    // 更新显示
    m_latestCroppedCloud = vtkSmartPointer<vtkPolyData>::New();
    m_latestCroppedCloud->DeepCopy(resultCloud);

    m_glyphFilter->SetInputData(resultCloud);
    m_glyphFilter->Update();
    
    renderWindow()->Render();
    
    // 裁剪完成，退出裁剪模式
    EnableCrop(false);
    emit cropFinishedForWorkflow();
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
    m_hasRectSelection = (m_rectSelection[2] - m_rectSelection[0] >= 3)
                      && (m_rectSelection[3] - m_rectSelection[1] >= 3);
    
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
    m_polyWorldPoints = vtkSmartPointer<vtkPoints>::New();
    m_isPolyFinished = false;
    m_hasPolySelection = false;
    
    // 清除旧的多边形可视化，避免旧数据残留
    vtkPolyDataMapper2D* mapper = vtkPolyDataMapper2D::SafeDownCast(m_polyLineActor->GetMapper());
    vtkSmartPointer<vtkPolyData> empty = vtkSmartPointer<vtkPolyData>::New();
    if (mapper)
        mapper->SetInputData(empty);
    m_polyLineActor->VisibilityOn();
}

void MainOpengl::AddPolyPoint(int x, int y)
{
    if (!m_polyPoints || m_isPolyFinished) return;
    
    // 存储屏幕坐标点（用于可视化）
    m_polyPoints->InsertNextPoint(x, y, 0);
    
    // 更新闭合预览折线
    UpdatePolyDynamicLine(x, y);
    
    // 强制刷新
    if (renderWindow()) renderWindow()->Render();
}

void MainOpengl::UpdatePolyDynamicLine(int x, int y)
{
    if (!m_polyPoints || m_polyPoints->GetNumberOfPoints() == 0) return;
    
    // 构建点集
    vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
    pts->DeepCopy(m_polyPoints);
    
    if (!m_isPolyFinished) {
        // 未完成：添加鼠标位置点，形成实时闭合预览
        pts->InsertNextPoint(x, y, 0);
    }
    
    // 闭合回第一个点，显示完整的环形预览
    // 点序: 0 → 1 → 2 → ... → N-1 → 0
    vtkIdType numPts = pts->GetNumberOfPoints();
    if (numPts < 2) return;
    
    vtkIdType numSegments = numPts + 1; // 最后回到第0点
    vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();
    polyLine->GetPointIds()->SetNumberOfIds(numSegments);
    for (vtkIdType i = 0; i < numPts; ++i)
        polyLine->GetPointIds()->SetId(i, i);
    polyLine->GetPointIds()->SetId(numSegments - 1, 0); // 回到起点闭合
    
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    lines->InsertNextCell(polyLine);
    
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(pts);
    polyData->SetLines(lines);
    
    // 使用屏幕坐标系
    vtkSmartPointer<vtkCoordinate> coord = vtkSmartPointer<vtkCoordinate>::New();
    coord->SetCoordinateSystemToDisplay();
    
    vtkPolyDataMapper2D* mapper = vtkPolyDataMapper2D::SafeDownCast(m_polyLineActor->GetMapper());
    if (!mapper)
        return;
    mapper->SetTransformCoordinate(coord);
    mapper->SetInputData(polyData);
    m_polyLineActor->VisibilityOn();
}

void MainOpengl::FinishPolySelection()
{
    if (!m_renderer || !m_polyPoints || m_polyPoints->GetNumberOfPoints() < 3)
        return;
    
    m_isPolyFinished = true;
    m_hasPolySelection = true;
    
    // --- 将屏幕坐标转成世界坐标，锁定当前视角下的多边形 ---
    // 取点云中心在 display 空间的 z 深度作为转换参考深度
    m_polyWorldPoints = vtkSmartPointer<vtkPoints>::New();
    
    double refDepth = 0.5; // 默认中点深度
    if (m_pointsActor && m_pointsActor->GetMapper()) {
        vtkPolyData* poly = vtkPolyData::SafeDownCast(m_pointsActor->GetMapper()->GetInputDataObject(0, 0));
        if (poly && poly->GetNumberOfPoints() > 0) {
            double bounds[6];
            poly->GetBounds(bounds);
            double center[3] = {
                (bounds[0] + bounds[1]) / 2.0,
                (bounds[2] + bounds[3]) / 2.0,
                (bounds[4] + bounds[5]) / 2.0
            };
            m_renderer->SetWorldPoint(center[0], center[1], center[2], 1.0);
            m_renderer->WorldToDisplay();
            double dp[3];
            m_renderer->GetDisplayPoint(dp);
            refDepth = dp[2];
        }
    }
    
    for (vtkIdType i = 0; i < m_polyPoints->GetNumberOfPoints(); i++) {
        double sp[3];
        m_polyPoints->GetPoint(i, sp);
        double world[4];
        m_renderer->SetDisplayPoint(sp[0], sp[1], refDepth);
        m_renderer->DisplayToWorld();
        m_renderer->GetWorldPoint(world);
        if (world[3] != 0.0) {
            world[0] /= world[3];
            world[1] /= world[3];
            world[2] /= world[3];
        }
        m_polyWorldPoints->InsertNextPoint(world);
    }
    
    // 绘制最终闭合多边形（不含鼠标位置），只基于已确定的顶点
    UpdatePolyDynamicLine(0, 0);
    
    // 注意：不要在 PolygonDrawStyle 的右键回调中切换 InteractorStyle。
    // 当前样式可能会在回调返回前被释放，容易导致 VTK 闪退。
    if (renderWindow()) {
        renderWindow()->Render();
    }
    
    qDebug() << "Polygon selection finished with" << m_polyPoints->GetNumberOfPoints() << "points";
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

// --------------------------------------------------------
//  DB树节点点云切换（预加载模式）
// --------------------------------------------------------

vtkSmartPointer<vtkActor> MainOpengl::createBboxActor(const double min[3], const double max[3])
{
    // 8 个顶点（AABB 角点）
    const double pts[8][3] = {
        {min[0], min[1], min[2]},  // 0
        {max[0], min[1], min[2]},  // 1
        {max[0], max[1], min[2]},  // 2
        {min[0], max[1], min[2]},  // 3
        {min[0], min[1], max[2]},  // 4
        {max[0], min[1], max[2]},  // 5
        {max[0], max[1], max[2]},  // 6
        {min[0], max[1], max[2]},  // 7
    };
    // 12 条棱的顶点索引
    const int edges[12][2] = {
        {0,1}, {1,2}, {2,3}, {3,0},   // 底面
        {4,5}, {5,6}, {6,7}, {7,4},   // 顶面
        {0,4}, {1,5}, {2,6}, {3,7},   // 竖直棱
    };

    vtkNew<vtkPoints> points;
    for (int i = 0; i < 8; ++i)
        points->InsertNextPoint(pts[i]);

    vtkNew<vtkCellArray> lines;
    for (int i = 0; i < 12; ++i) {
        vtkNew<vtkLine> line;
        line->GetPointIds()->SetId(0, edges[i][0]);
        line->GetPointIds()->SetId(1, edges[i][1]);
        lines->InsertNextCell(line);
    }

    vtkNew<vtkPolyData> polyData;
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(polyData);

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 1.0, 0.0);   // 黄色
    actor->GetProperty()->SetLineWidth(2.0);
    actor->GetProperty()->SetRepresentationToWireframe();
    actor->PickableOff();

    return actor;
}

vtkSmartPointer<vtkActor> MainOpengl::createLineActor(vtkPolyData *polyData, const QColor &color, double lineWidth)
{
    if (!polyData || !polyData->GetLines() || polyData->GetNumberOfPoints() < 2)
        return nullptr;

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(polyData);
    mapper->ScalarVisibilityOff();

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color.redF(), color.greenF(), color.blueF());
    actor->GetProperty()->SetLineWidth(lineWidth);
    actor->PickableOff();
    return actor;
}

vtkSmartPointer<vtkActor> MainOpengl::createSurfaceActor(vtkPolyData *polyData,
                                                          const QColor &color,
                                                          double opacity)
{
    if (!polyData || !polyData->GetPolys() || polyData->GetNumberOfCells() == 0)
        return nullptr;

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(polyData);
    mapper->ScalarVisibilityOff();

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color.redF(), color.greenF(), color.blueF());
    actor->GetProperty()->SetOpacity(opacity);
    actor->GetProperty()->SetRepresentationToSurface();
    actor->PickableOff();
    return actor;
}

vtkSmartPointer<vtkActor> MainOpengl::createColoredSurfaceActor(vtkPolyData *polyData,
                                                                double opacity)
{
    if (!polyData || !polyData->GetPolys() || polyData->GetNumberOfCells() == 0)
        return nullptr;

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(polyData);
    mapper->SetScalarModeToUseCellData();
    mapper->SetColorModeToDirectScalars();
    mapper->ScalarVisibilityOn();

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetOpacity(opacity);
    actor->GetProperty()->SetRepresentationToSurface();
    actor->PickableOff();
    return actor;
}

vtkSmartPointer<vtkActor> MainOpengl::createColoredLineActor(vtkPolyData *polyData,
                                                             double lineWidth)
{
    if (!polyData || !polyData->GetLines() || polyData->GetNumberOfLines() == 0)
        return nullptr;

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(polyData);
    mapper->SetScalarModeToUseCellData();
    mapper->SetColorModeToDirectScalars();
    mapper->ScalarVisibilityOn();

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetLineWidth(lineWidth);
    actor->PickableOff();
    return actor;
}

vtkSmartPointer<vtkActor> MainOpengl::createColoredPointActor(vtkPolyData *polyData,
                                                              double pointSize)
{
    if (!polyData || !polyData->GetVerts() || polyData->GetNumberOfPoints() == 0)
        return nullptr;

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(polyData);
    mapper->SetScalarModeToUsePointData();
    mapper->SetColorModeToDirectScalars();
    mapper->ScalarVisibilityOn();

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(pointSize);
    actor->PickableOff();
    return actor;
}

vtkSmartPointer<vtkActor> MainOpengl::createNozzlePoseActor(vtkPolyData *polyData)
{
    if (!polyData || polyData->GetNumberOfPoints() == 0 ||
        !polyData->GetPointData()->GetVectors("PreviewVector")) {
        return nullptr;
    }

    vtkNew<vtkArrowSource> arrow;
    arrow->SetTipResolution(12);
    arrow->SetShaftResolution(12);
    arrow->SetTipLength(0.28);
    arrow->SetTipRadius(0.09);
    arrow->SetShaftRadius(0.025);

    vtkNew<vtkGlyph3D> glyph;
    glyph->SetInputData(polyData);
    glyph->SetSourceConnection(arrow->GetOutputPort());
    glyph->SetVectorModeToUseVector();
    glyph->SetScaleModeToScaleByVector();
    glyph->SetColorModeToColorByScalar();
    glyph->OrientOn();
    glyph->SetScaleFactor(1.0);
    glyph->Update();

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(glyph->GetOutputPort());
    mapper->SetScalarModeToUsePointData();
    mapper->SetColorModeToDirectScalars();
    mapper->ScalarVisibilityOn();

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->PickableOff();
    return actor;
}

vtkSmartPointer<vtkPolyData> MainOpengl::buildRotationStartMarkers(
    vtkPolyData *trajectories,
    vtkPolyData *straightAxis,
    vtkPolyData *curvedAxis) const
{
    vtkSmartPointer<vtkPolyData> result = vtkSmartPointer<vtkPolyData>::New();
    if (!trajectories || trajectories->GetNumberOfLines() == 0 ||
        !straightAxis || straightAxis->GetNumberOfPoints() < 2) {
        return result;
    }

    double axisStart[3];
    double axisEnd[3];
    straightAxis->GetPoint(0, axisStart);
    straightAxis->GetPoint(straightAxis->GetNumberOfPoints() - 1, axisEnd);
    double axis[3] = {
        axisEnd[0] - axisStart[0],
        axisEnd[1] - axisStart[1],
        axisEnd[2] - axisStart[2]
    };
    if (vtkMath::Normalize(axis) <= 1e-12)
        return result;

    if (curvedAxis && curvedAxis->GetNumberOfPoints() >= 2) {
        double curvedStart[3];
        double curvedEnd[3];
        curvedAxis->GetPoint(0, curvedStart);
        curvedAxis->GetPoint(curvedAxis->GetNumberOfPoints() - 1, curvedEnd);
        const double curvedDirection[3] = {
            curvedEnd[0] - curvedStart[0],
            curvedEnd[1] - curvedStart[1],
            curvedEnd[2] - curvedStart[2]
        };
        if (vtkMath::Dot(axis, curvedDirection) < 0.0) {
            axis[0] = -axis[0];
            axis[1] = -axis[1];
            axis[2] = -axis[2];
        }
    }

    const double origin[3] = {
        0.5 * (axisStart[0] + axisEnd[0]),
        0.5 * (axisStart[1] + axisEnd[1]),
        0.5 * (axisStart[2] + axisEnd[2])
    };
    const double reference[3] = {
        0.0,
        std::abs(axis[2]) < 0.9 ? 0.0 : 1.0,
        std::abs(axis[2]) < 0.9 ? 1.0 : 0.0
    };
    double u[3];
    double v[3];
    vtkMath::Cross(reference, axis, u);
    if (vtkMath::Normalize(u) <= 1e-12)
        return result;
    vtkMath::Cross(axis, u, v);
    vtkMath::Normalize(v);

    vtkIntArray *sliceIds = vtkIntArray::SafeDownCast(
        trajectories->GetCellData()->GetArray("SliceIndex"));
    vtkIntArray *closedFlags = vtkIntArray::SafeDownCast(
        trajectories->GetCellData()->GetArray("ClosedLayer"));

    vtkNew<vtkPoints> markerPoints;
    vtkNew<vtkCellArray> markerVertices;
    vtkNew<vtkIntArray> markerSliceIds;
    markerSliceIds->SetName("SliceIndex");
    vtkNew<vtkUnsignedCharArray> markerColors;
    markerColors->SetName("Colors");
    markerColors->SetNumberOfComponents(3);
    const unsigned char markerColor[3] = {255, 64, 200};

    auto localCoordinates = [&](const double point[3], double &x, double &y) {
        const double relative[3] = {
            point[0] - origin[0], point[1] - origin[1], point[2] - origin[2]
        };
        x = vtkMath::Dot(relative, u);
        y = vtkMath::Dot(relative, v);
    };

    for (vtkIdType cellId = 0; cellId < trajectories->GetNumberOfCells(); ++cellId) {
        vtkCell *cell = trajectories->GetCell(cellId);
        if (!cell || cell->GetNumberOfPoints() < 3)
            continue;

        bool closed = closedFlags && closedFlags->GetNumberOfTuples() > cellId
            ? closedFlags->GetValue(cellId) != 0 : false;
        if (!closed) {
            double first[3];
            double last[3];
            trajectories->GetPoint(cell->GetPointId(0), first);
            trajectories->GetPoint(cell->GetPointId(cell->GetNumberOfPoints() - 1), last);
            closed = vtkMath::Distance2BetweenPoints(first, last) <= 1e-12;
        }
        if (!closed)
            continue;

        bool foundIntersection = false;
        double bestDistance = std::numeric_limits<double>::max();
        double bestPoint[3] = {0.0, 0.0, 0.0};
        double fallbackScore = std::numeric_limits<double>::max();
        double fallbackPoint[3] = {0.0, 0.0, 0.0};

        const vtkIdType pointCount = cell->GetNumberOfPoints();
        for (vtkIdType i = 0; i < pointCount; ++i) {
            const vtkIdType next = (i + 1) % pointCount;
            double first[3];
            double second[3];
            trajectories->GetPoint(cell->GetPointId(i), first);
            trajectories->GetPoint(cell->GetPointId(next), second);
            double x1 = 0.0;
            double y1 = 0.0;
            double x2 = 0.0;
            double y2 = 0.0;
            localCoordinates(first, x1, y1);
            localCoordinates(second, x2, y2);

            if (x1 >= 0.0) {
                const double score = std::abs(std::atan2(y1, x1));
                if (score < fallbackScore) {
                    fallbackScore = score;
                    std::copy(first, first + 3, fallbackPoint);
                }
            }

            auto considerIntersection = [&](double ratio) {
                const double x = x1 + ratio * (x2 - x1);
                if (x < 0.0 || x >= bestDistance)
                    return;
                bestDistance = x;
                for (int component = 0; component < 3; ++component)
                    bestPoint[component] = first[component] +
                        ratio * (second[component] - first[component]);
                foundIntersection = true;
            };

            constexpr double epsilon = 1e-10;
            if (std::abs(y1) <= epsilon)
                considerIntersection(0.0);
            if ((y1 < -epsilon && y2 > epsilon) ||
                (y1 > epsilon && y2 < -epsilon) || std::abs(y2) <= epsilon) {
                const double denominator = y2 - y1;
                if (std::abs(denominator) > epsilon)
                    considerIntersection(-y1 / denominator);
            }
        }

        const double *marker = foundIntersection ? bestPoint : fallbackPoint;
        if (!foundIntersection && fallbackScore == std::numeric_limits<double>::max())
            continue;
        const vtkIdType id = markerPoints->InsertNextPoint(marker);
        markerVertices->InsertNextCell(1, &id);
        markerSliceIds->InsertNextValue(
            sliceIds && sliceIds->GetNumberOfTuples() > cellId
                ? sliceIds->GetValue(cellId) : static_cast<int>(cellId));
        markerColors->InsertNextTypedTuple(markerColor);
    }

    result->SetPoints(markerPoints);
    result->SetVerts(markerVertices);
    result->GetPointData()->AddArray(markerSliceIds);
    result->GetPointData()->SetScalars(markerColors);
    return result;
}

vtkSmartPointer<vtkPolyData> MainOpengl::loadMeshPolyData(std::shared_ptr<NodeCloudData> data)
{
    if (!data || data->meshFilePath.isEmpty())
        return nullptr;

    if (data->meshPolyData && data->meshPolyData->GetNumberOfPoints() > 0)
        return data->meshPolyData;

    vtkNew<vtkPLYReader> reader;
    reader->SetFileName(data->meshFilePath.toStdString().c_str());
    reader->Update();
    vtkPolyData *output = reader->GetOutput();
    if (!output || output->GetNumberOfPoints() == 0)
        return nullptr;

    data->meshPolyData = vtkSmartPointer<vtkPolyData>::New();
    data->meshPolyData->DeepCopy(output);
    return data->meshPolyData;
}

vtkSmartPointer<vtkPolyData> MainOpengl::cropPolyDataWithCurrentSelection(vtkPolyData *input)
{
    if (!input || input->GetNumberOfPoints() == 0 || !m_renderer)
        return nullptr;

    if (m_currentCropMode == Crop_Rect) {
        if (!m_hasRectSelection)
            return nullptr;

        vtkNew<vtkAreaPicker> areaPicker;
        areaPicker->AreaPick(m_rectSelection[0], m_rectSelection[1],
                             m_rectSelection[2], m_rectSelection[3], m_renderer);

        vtkNew<vtkExtractPolyDataGeometry> extract;
        extract->SetImplicitFunction(areaPicker->GetFrustum());
        extract->SetInputData(input);
        extract->SetExtractInside(m_cropInside);
        extract->Update();

        vtkSmartPointer<vtkPolyData> result = vtkSmartPointer<vtkPolyData>::New();
        result->DeepCopy(extract->GetOutput());
        return result;
    }

    if (m_currentCropMode == Crop_Poly) {
        if (!m_hasPolySelection || !m_polyPoints || m_polyPoints->GetNumberOfPoints() < 3)
            return nullptr;

        vtkCamera *camera = m_renderer->GetActiveCamera();
        if (!camera)
            return nullptr;

        double cameraPosition[3];
        camera->GetPosition(cameraPosition);

        double centerDisplay[3] = {0.0, 0.0, 0.5};
        for (vtkIdType i = 0; i < m_polyPoints->GetNumberOfPoints(); ++i) {
            double p[3];
            m_polyPoints->GetPoint(i, p);
            centerDisplay[0] += p[0];
            centerDisplay[1] += p[1];
        }
        centerDisplay[0] /= m_polyPoints->GetNumberOfPoints();
        centerDisplay[1] /= m_polyPoints->GetNumberOfPoints();

        m_renderer->SetDisplayPoint(centerDisplay);
        m_renderer->DisplayToWorld();
        double centerWorld4[4];
        m_renderer->GetWorldPoint(centerWorld4);
        double centerWorld[3] = {centerWorld4[0], centerWorld4[1], centerWorld4[2]};
        if (centerWorld4[3] != 0.0) {
            centerWorld[0] /= centerWorld4[3];
            centerWorld[1] /= centerWorld4[3];
            centerWorld[2] /= centerWorld4[3];
        }

        vtkNew<vtkImplicitBoolean> polyFrustum;
        polyFrustum->SetOperationTypeToIntersection();

        std::vector<vtkSmartPointer<vtkPlane>> sidePlanes;
        sidePlanes.reserve(m_polyPoints->GetNumberOfPoints());

        for (vtkIdType i = 0; i < m_polyPoints->GetNumberOfPoints(); ++i) {
            const vtkIdType j = (i + 1) % m_polyPoints->GetNumberOfPoints();
            double p1[3];
            double p2[3];
            m_polyPoints->GetPoint(i, p1);
            m_polyPoints->GetPoint(j, p2);

            double w1Display[3] = {p1[0], p1[1], 0.0};
            double w2Display[3] = {p2[0], p2[1], 0.0};
            m_renderer->SetDisplayPoint(w1Display);
            m_renderer->DisplayToWorld();
            double w14[4];
            m_renderer->GetWorldPoint(w14);
            m_renderer->SetDisplayPoint(w2Display);
            m_renderer->DisplayToWorld();
            double w24[4];
            m_renderer->GetWorldPoint(w24);

            double w1[3] = {w14[0], w14[1], w14[2]};
            double w2[3] = {w24[0], w24[1], w24[2]};
            if (w14[3] != 0.0) {
                w1[0] /= w14[3];
                w1[1] /= w14[3];
                w1[2] /= w14[3];
            }
            if (w24[3] != 0.0) {
                w2[0] /= w24[3];
                w2[1] /= w24[3];
                w2[2] /= w24[3];
            }

            double ray1[3];
            double ray2[3];
            vtkMath::Subtract(w1, cameraPosition, ray1);
            vtkMath::Subtract(w2, cameraPosition, ray2);

            double normal[3];
            vtkMath::Cross(ray2, ray1, normal);
            if (vtkMath::Normalize(normal) == 0.0)
                continue;

            double toCenter[3];
            vtkMath::Subtract(centerWorld, cameraPosition, toCenter);
            if (vtkMath::Dot(normal, toCenter) > 0.0) {
                normal[0] = -normal[0];
                normal[1] = -normal[1];
                normal[2] = -normal[2];
            }

            vtkSmartPointer<vtkPlane> plane = vtkSmartPointer<vtkPlane>::New();
            plane->SetOrigin(cameraPosition);
            plane->SetNormal(normal);
            sidePlanes.push_back(plane);
            polyFrustum->AddFunction(plane);
        }

        if (sidePlanes.size() < 3)
            return nullptr;

        vtkNew<vtkExtractPolyDataGeometry> extract;
        extract->SetInputData(input);
        extract->SetImplicitFunction(polyFrustum);
        extract->SetExtractInside(m_cropInside);
        extract->Update();

        vtkSmartPointer<vtkPolyData> result = vtkSmartPointer<vtkPolyData>::New();
        result->DeepCopy(extract->GetOutput());
        return result;
    }

    return nullptr;
}

vtkSmartPointer<vtkActor> MainOpengl::createSurgicalMeshActor(vtkPolyData *polyData, double opacity)
{
    if (!polyData || polyData->GetNumberOfPoints() == 0)
        return nullptr;

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(polyData);
    mapper->ScalarVisibilityOff();

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 0.78, 0.56);
    actor->GetProperty()->SetOpacity(opacity);
    actor->GetProperty()->SetAmbient(0.35);
    actor->GetProperty()->SetDiffuse(0.8);
    actor->GetProperty()->SetSpecular(0.15);
    actor->GetProperty()->SetRepresentationToSurface();
    actor->PickableOff();
    return actor;
}

void MainOpengl::updateSurgicalOverlay(std::shared_ptr<NodeCloudData> data)
{
    const QString nodeName = cloudKey(data);
    if (nodeName.isEmpty())
        return;

    ensureCloudRenderData(data);
    if (!m_cloudCache.contains(nodeName))
        return;

    auto &crd = m_cloudCache[nodeName];
    if (crd.surgicalMeshActor)
        m_renderer->RemoveActor(crd.surgicalMeshActor);

    crd.surgicalMeshActor = createSurgicalMeshActor(data->surgicalMesh, data->surgicalMeshOpacity);
    if (crd.surgicalMeshActor)
        m_renderer->AddActor(crd.surgicalMeshActor);

    const bool visible = crd.cloudActor && crd.cloudActor->GetVisibility();
    if (crd.surgicalMeshActor)
        crd.surgicalMeshActor->SetVisibility(visible);
    if (crd.cloudActor)
        crd.cloudActor->GetProperty()->SetOpacity(visible ? 0.45 : 1.0);
    renderWindow()->Render();
}

void MainOpengl::clearCloudActors()
{
    for (auto it = m_cloudCache.begin(); it != m_cloudCache.end(); ++it) {
        m_renderer->RemoveActor(it->cloudActor);
        m_renderer->RemoveActor(it->bboxActor);
        if (it->surgicalMeshActor)
            m_renderer->RemoveActor(it->surgicalMeshActor);
        if (it->curvedAxisActor)
            m_renderer->RemoveActor(it->curvedAxisActor);
        if (it->straightAxisActor)
            m_renderer->RemoveActor(it->straightAxisActor);
        if (it->slicePlanesActor)
            m_renderer->RemoveActor(it->slicePlanesActor);
        if (it->firstSlicePlaneActor)
            m_renderer->RemoveActor(it->firstSlicePlaneActor);
        if (it->sliceBoundingBoxActor)
            m_renderer->RemoveActor(it->sliceBoundingBoxActor);
        if (it->sliceContoursActor)
            m_renderer->RemoveActor(it->sliceContoursActor);
        if (it->fittedSliceContoursActor)
            m_renderer->RemoveActor(it->fittedSliceContoursActor);
        if (it->rotationStartMarkersActor)
            m_renderer->RemoveActor(it->rotationStartMarkersActor);
        if (it->equalDoseSurfaceActor)
            m_renderer->RemoveActor(it->equalDoseSurfaceActor);
        if (it->sprayPathActor)
            m_renderer->RemoveActor(it->sprayPathActor);
        if (it->sprayPathConnectionsActor)
            m_renderer->RemoveActor(it->sprayPathConnectionsActor);
        if (it->continuousSprayPathActor)
            m_renderer->RemoveActor(it->continuousSprayPathActor);
        if (it->continuousPathTransitionsActor)
            m_renderer->RemoveActor(it->continuousPathTransitionsActor);
        if (it->joint6ResetMarkersActor)
            m_renderer->RemoveActor(it->joint6ResetMarkersActor);
        if (it->nozzlePosePreviewActor)
            m_renderer->RemoveActor(it->nozzlePosePreviewActor);
    }
    m_cloudCache.clear();
    m_activeCloudNode.clear();
}

QString MainOpengl::cloudKey(std::shared_ptr<NodeCloudData> data) const
{
    if (!data)
        return {};
    if (!data->displayName.isEmpty())
        return data->displayName;
    return QFileInfo(data->filePath).completeBaseName();
}

void MainOpengl::ensureCloudRenderData(std::shared_ptr<NodeCloudData> data)
{
    if (!data || !data->polyData)
        return;

    const QString nodeName = cloudKey(data);
    if (nodeName.isEmpty() || m_cloudCache.contains(nodeName))
        return;

    m_pointsActor->VisibilityOff();

    vtkNew<vtkPolyDataMapper> mapper;
    if (!data->meshFilePath.isEmpty()) {
        vtkSmartPointer<vtkPolyData> mesh = loadMeshPolyData(data);
        if (!mesh)
            return;
        mapper->SetInputData(mesh);
    } else {
        vtkNew<vtkVertexGlyphFilter> glyph;
        glyph->SetInputData(data->polyData);
        glyph->Update();
        mapper->SetInputConnection(glyph->GetOutputPort());
    }

    vtkSmartPointer<vtkActor> cloudActor = vtkSmartPointer<vtkActor>::New();
    cloudActor->SetMapper(mapper);
    if (!data->meshFilePath.isEmpty()) {
        cloudActor->GetProperty()->SetRepresentationToSurface();
        cloudActor->GetProperty()->SetInterpolationToPhong();
        cloudActor->GetProperty()->SetOpacity(data->surgicalMesh ? 0.45 : 1.0);
    }

    vtkSmartPointer<vtkActor> bboxActor = createBboxActor(data->bboxMin, data->bboxMax);
    bboxActor->VisibilityOff();

    CloudRenderData crd;
    crd.cloudActor = cloudActor;
    crd.bboxActor  = bboxActor;

    m_renderer->AddActor(cloudActor);
    m_renderer->AddActor(bboxActor);
    if (data->surgicalMesh && data->surgicalMesh->GetNumberOfPoints() > 0) {
        crd.surgicalMeshActor = createSurgicalMeshActor(data->surgicalMesh, data->surgicalMeshOpacity);
        if (crd.surgicalMeshActor) {
            crd.surgicalMeshActor->VisibilityOff();
            m_renderer->AddActor(crd.surgicalMeshActor);
        }
    }
    if (data->curvedAxis && data->curvedAxis->GetNumberOfPoints() >= 2) {
        crd.curvedAxisActor = createLineActor(data->curvedAxis, QColor(220, 38, 38), 2.0);
        if (crd.curvedAxisActor) {
            crd.curvedAxisActor->VisibilityOff();
            m_renderer->AddActor(crd.curvedAxisActor);
        }
    }
    if (data->straightAxis && data->straightAxis->GetNumberOfPoints() >= 2) {
        crd.straightAxisActor = createLineActor(data->straightAxis, QColor(0, 102, 255), 7.0);
        if (crd.straightAxisActor) {
            crd.straightAxisActor->VisibilityOff();
            m_renderer->AddActor(crd.straightAxisActor);
        }
    }
    if (data->slicePlanes && data->slicePlanes->GetNumberOfCells() > 0) {
        crd.slicePlanesActor = createSurfaceActor(data->slicePlanes, QColor(64, 153, 255), 0.20);
        if (crd.slicePlanesActor) {
            crd.slicePlanesActor->VisibilityOff();
            m_renderer->AddActor(crd.slicePlanesActor);
        }
    }
    if (data->firstSlicePlane && data->firstSlicePlane->GetNumberOfCells() > 0) {
        crd.firstSlicePlaneActor = createSurfaceActor(data->firstSlicePlane, QColor(255, 0, 0), 0.30);
        if (crd.firstSlicePlaneActor) {
            crd.firstSlicePlaneActor->VisibilityOff();
            m_renderer->AddActor(crd.firstSlicePlaneActor);
        }
    }
    if (data->sliceBoundingBox && data->sliceBoundingBox->GetNumberOfLines() > 0) {
        crd.sliceBoundingBoxActor = createLineActor(data->sliceBoundingBox, QColor(26, 178, 51), 2.0);
        if (crd.sliceBoundingBoxActor) {
            crd.sliceBoundingBoxActor->VisibilityOff();
            m_renderer->AddActor(crd.sliceBoundingBoxActor);
        }
    }
    if (data->sliceContours && data->sliceContours->GetNumberOfLines() > 0) {
        crd.sliceContoursActor = createLineActor(data->sliceContours, QColor(255, 214, 10), 3.0);
        if (crd.sliceContoursActor) {
            crd.sliceContoursActor->VisibilityOff();
            m_renderer->AddActor(crd.sliceContoursActor);
        }
    }
    if (data->fittedSliceContours && data->fittedSliceContours->GetNumberOfLines() > 0) {
        crd.fittedSliceContoursActor = createLineActor(data->fittedSliceContours, QColor(0, 230, 230), 4.0);
        if (crd.fittedSliceContoursActor) {
            crd.fittedSliceContoursActor->VisibilityOff();
            m_renderer->AddActor(crd.fittedSliceContoursActor);
        }
    }
    if (data->rotationStartMarkers && data->rotationStartMarkers->GetNumberOfPoints() > 0) {
        crd.rotationStartMarkersActor = createColoredPointActor(data->rotationStartMarkers, 14.0);
        if (crd.rotationStartMarkersActor) {
            crd.rotationStartMarkersActor->GetProperty()->SetRenderPointsAsSpheres(true);
            crd.rotationStartMarkersActor->VisibilityOff();
            m_renderer->AddActor(crd.rotationStartMarkersActor);
        }
    }
    if (data->equalDoseSurface && data->equalDoseSurface->GetNumberOfCells() > 0) {
        crd.equalDoseSurfaceActor = createColoredSurfaceActor(data->equalDoseSurface, 0.28);
        if (crd.equalDoseSurfaceActor) {
            crd.equalDoseSurfaceActor->VisibilityOff();
            m_renderer->AddActor(crd.equalDoseSurfaceActor);
        }
    }
    if (data->sprayPath && data->sprayPath->GetNumberOfLines() > 0) {
        crd.sprayPathActor = createLineActor(data->sprayPath, QColor(217, 26, 26), 4.0);
        if (crd.sprayPathActor) {
            crd.sprayPathActor->VisibilityOff();
            m_renderer->AddActor(crd.sprayPathActor);
        }
    }
    if (data->sprayPathConnections && data->sprayPathConnections->GetNumberOfLines() > 0) {
        crd.sprayPathConnectionsActor = createLineActor(
            data->sprayPathConnections, QColor(26, 166, 51), 4.5);
        if (crd.sprayPathConnectionsActor) {
            crd.sprayPathConnectionsActor->VisibilityOff();
            m_renderer->AddActor(crd.sprayPathConnectionsActor);
        }
    }
    if (data->continuousSprayPath && data->continuousSprayPath->GetNumberOfLines() > 0) {
        crd.continuousSprayPathActor = createLineActor(
            data->continuousSprayPath, QColor(255, 0, 110), 5.5);
        if (crd.continuousSprayPathActor) {
            crd.continuousSprayPathActor->VisibilityOff();
            m_renderer->AddActor(crd.continuousSprayPathActor);
        }
    }
    if (data->continuousPathTransitions &&
        data->continuousPathTransitions->GetNumberOfLines() > 0) {
        crd.continuousPathTransitionsActor = createColoredLineActor(
            data->continuousPathTransitions, 4.0);
        if (crd.continuousPathTransitionsActor) {
            crd.continuousPathTransitionsActor->VisibilityOff();
            m_renderer->AddActor(crd.continuousPathTransitionsActor);
        }
    }
    if (data->joint6ResetMarkers && data->joint6ResetMarkers->GetNumberOfPoints() > 0) {
        crd.joint6ResetMarkersActor = createColoredPointActor(data->joint6ResetMarkers, 12.0);
        if (crd.joint6ResetMarkersActor) {
            crd.joint6ResetMarkersActor->VisibilityOff();
            m_renderer->AddActor(crd.joint6ResetMarkersActor);
        }
    }
    if (data->nozzlePosePreview && data->nozzlePosePreview->GetNumberOfPoints() > 0) {
        crd.nozzlePosePreviewActor = createNozzlePoseActor(data->nozzlePosePreview);
        if (crd.nozzlePosePreviewActor) {
            crd.nozzlePosePreviewActor->VisibilityOff();
            m_renderer->AddActor(crd.nozzlePosePreviewActor);
        }
    }
    m_cloudCache[nodeName] = crd;

    vtkCamera *cam = m_renderer->GetActiveCamera();
    cam->SetFocalPoint(data->center);
    double dx = data->bboxMax[0] - data->bboxMin[0];
    double dy = data->bboxMax[1] - data->bboxMin[1];
    double dz = data->bboxMax[2] - data->bboxMin[2];
    double diag = std::sqrt(dx*dx + dy*dy + dz*dz);
    double dist = (diag < 1e-6) ? 1.0 : diag * 1.5;
    cam->SetPosition(data->center[0],
                     data->center[1],
                     data->center[2] - dist);
    cam->SetViewUp(0.0, -1.0, 0.0);
    m_renderer->ResetCameraClippingRange();
}

void MainOpengl::showPointCloud(std::shared_ptr<NodeCloudData> data)
{
    if (!data || !data->polyData) return;

    const QString nodeName = cloudKey(data);
    if (nodeName.isEmpty()) return;

    ensureCloudRenderData(data);
    if (!m_cloudCache.contains(nodeName))
        return;

    if (!m_activeCloudNode.isEmpty() && m_cloudCache.contains(m_activeCloudNode))
        m_cloudCache[m_activeCloudNode].bboxActor->VisibilityOff();

    auto &cur = m_cloudCache[nodeName];
    cur.cloudActor->VisibilityOn();
    cur.cloudActor->GetProperty()->SetOpacity(data->surgicalMesh ? 0.45 : 1.0);
    if (cur.surgicalMeshActor)
        cur.surgicalMeshActor->VisibilityOn();
    if (cur.curvedAxisActor)
        cur.curvedAxisActor->SetVisibility(data->showCurvedAxis);
    if (cur.straightAxisActor)
        cur.straightAxisActor->VisibilityOn();
    if (cur.slicePlanesActor)
        cur.slicePlanesActor->SetVisibility(data->showSliceLayers);
    if (cur.firstSlicePlaneActor)
        cur.firstSlicePlaneActor->SetVisibility(data->showSliceLayers);
    if (cur.sliceBoundingBoxActor)
        cur.sliceBoundingBoxActor->SetVisibility(data->showSliceLayers);
    if (cur.sliceContoursActor)
        cur.sliceContoursActor->SetVisibility(data->showSliceLayers);
    if (cur.fittedSliceContoursActor)
        cur.fittedSliceContoursActor->SetVisibility(data->showFittedContours);
    if (cur.rotationStartMarkersActor)
        cur.rotationStartMarkersActor->SetVisibility(data->showFittedContours);
    if (cur.equalDoseSurfaceActor)
        cur.equalDoseSurfaceActor->SetVisibility(data->showEqualDoseSurface);
    if (cur.sprayPathActor)
        cur.sprayPathActor->SetVisibility(data->showSprayPath);
    if (cur.sprayPathConnectionsActor)
        cur.sprayPathConnectionsActor->SetVisibility(data->showSprayPath);
    if (cur.continuousSprayPathActor)
        cur.continuousSprayPathActor->SetVisibility(data->showContinuousPath);
    if (cur.continuousPathTransitionsActor)
        cur.continuousPathTransitionsActor->SetVisibility(data->showContinuousPath);
    if (cur.joint6ResetMarkersActor)
        cur.joint6ResetMarkersActor->SetVisibility(data->showContinuousPath);
    if (cur.nozzlePosePreviewActor)
        cur.nozzlePosePreviewActor->SetVisibility(data->showNozzlePoses);
    cur.bboxActor->SetVisibility(data->showSliceLayers);
    m_activeCloudNode = nodeName;

    renderWindow()->Render();
}

void MainOpengl::setPointCloudVisible(std::shared_ptr<NodeCloudData> data, bool visible)
{
    if (!data || !data->polyData)
        return;

    const QString nodeName = cloudKey(data);
    if (nodeName.isEmpty())
        return;

    ensureCloudRenderData(data);
    if (!m_cloudCache.contains(nodeName))
        return;

    auto &cur = m_cloudCache[nodeName];
    cur.cloudActor->SetVisibility(visible);
    cur.cloudActor->GetProperty()->SetOpacity((visible && data->surgicalMesh) ? 0.45 : 1.0);
    if (cur.surgicalMeshActor)
        cur.surgicalMeshActor->SetVisibility(visible);
    if (cur.curvedAxisActor)
        cur.curvedAxisActor->SetVisibility(visible && data->showCurvedAxis);
    if (cur.straightAxisActor)
        cur.straightAxisActor->SetVisibility(visible);
    if (cur.slicePlanesActor)
        cur.slicePlanesActor->SetVisibility(visible && data->showSliceLayers);
    if (cur.firstSlicePlaneActor)
        cur.firstSlicePlaneActor->SetVisibility(visible && data->showSliceLayers);
    if (cur.sliceBoundingBoxActor)
        cur.sliceBoundingBoxActor->SetVisibility(visible && data->showSliceLayers);
    if (cur.sliceContoursActor)
        cur.sliceContoursActor->SetVisibility(visible && data->showSliceLayers);
    if (cur.fittedSliceContoursActor)
        cur.fittedSliceContoursActor->SetVisibility(visible && data->showFittedContours);
    if (cur.rotationStartMarkersActor)
        cur.rotationStartMarkersActor->SetVisibility(visible && data->showFittedContours);
    if (cur.equalDoseSurfaceActor)
        cur.equalDoseSurfaceActor->SetVisibility(visible && data->showEqualDoseSurface);
    if (cur.sprayPathActor)
        cur.sprayPathActor->SetVisibility(visible && data->showSprayPath);
    if (cur.sprayPathConnectionsActor)
        cur.sprayPathConnectionsActor->SetVisibility(visible && data->showSprayPath);
    if (cur.continuousSprayPathActor)
        cur.continuousSprayPathActor->SetVisibility(visible && data->showContinuousPath);
    if (cur.continuousPathTransitionsActor)
        cur.continuousPathTransitionsActor->SetVisibility(visible && data->showContinuousPath);
    if (cur.joint6ResetMarkersActor)
        cur.joint6ResetMarkersActor->SetVisibility(visible && data->showContinuousPath);
    if (cur.nozzlePosePreviewActor)
        cur.nozzlePosePreviewActor->SetVisibility(visible && data->showNozzlePoses);
    if (!visible) {
        cur.bboxActor->VisibilityOff();
        if (m_activeCloudNode == nodeName)
            m_activeCloudNode.clear();
    }
    renderWindow()->Render();
}

void MainOpengl::setSliceLayersVisible(std::shared_ptr<NodeCloudData> data, bool visible)
{
    if (!data)
        return;
    data->showSliceLayers = visible;
    const QString nodeName = cloudKey(data);
    if (!m_cloudCache.contains(nodeName))
        return;

    auto &crd = m_cloudCache[nodeName];
    const bool nodeVisible = crd.cloudActor && crd.cloudActor->GetVisibility();
    if (crd.slicePlanesActor)
        crd.slicePlanesActor->SetVisibility(nodeVisible && visible);
    if (crd.firstSlicePlaneActor)
        crd.firstSlicePlaneActor->SetVisibility(nodeVisible && visible);
    if (crd.sliceBoundingBoxActor)
        crd.sliceBoundingBoxActor->SetVisibility(nodeVisible && visible);
    if (crd.sliceContoursActor)
        crd.sliceContoursActor->SetVisibility(nodeVisible && visible);
    if (crd.bboxActor) {
        const bool active = m_activeCloudNode == nodeName;
        crd.bboxActor->SetVisibility(nodeVisible && active && visible);
    }
    renderWindow()->Render();
}

void MainOpengl::setFittedContoursVisible(std::shared_ptr<NodeCloudData> data, bool visible)
{
    if (!data)
        return;
    data->showFittedContours = visible;
    const QString nodeName = cloudKey(data);
    if (!m_cloudCache.contains(nodeName))
        return;

    auto &crd = m_cloudCache[nodeName];
    const bool nodeVisible = crd.cloudActor && crd.cloudActor->GetVisibility();
    if (crd.fittedSliceContoursActor)
        crd.fittedSliceContoursActor->SetVisibility(nodeVisible && visible);
    if (crd.rotationStartMarkersActor)
        crd.rotationStartMarkersActor->SetVisibility(nodeVisible && visible);
    renderWindow()->Render();
}

void MainOpengl::setCurvedAxisVisible(std::shared_ptr<NodeCloudData> data, bool visible)
{
    if (!data)
        return;

    data->showCurvedAxis = visible;
    const QString nodeName = cloudKey(data);
    if (nodeName.isEmpty() || !m_cloudCache.contains(nodeName))
        return;

    auto &cur = m_cloudCache[nodeName];
    const bool nodeVisible = cur.cloudActor && cur.cloudActor->GetVisibility();
    if (cur.curvedAxisActor)
        cur.curvedAxisActor->SetVisibility(nodeVisible && visible);
    renderWindow()->Render();
}

void MainOpengl::loadCloudForCropping(std::shared_ptr<NodeCloudData> data)
{
    if (!data || !data->polyData)
        return;

    HideActiveBoundingBox();
    SetRosPaused(true);
    m_pointsActor->VisibilityOn();
    m_glyphFilter->SetInputData(data->polyData);
    m_glyphFilter->Update();

    vtkCamera *cam = m_renderer->GetActiveCamera();
    cam->SetFocalPoint(data->center);
    double dx = data->bboxMax[0] - data->bboxMin[0];
    double dy = data->bboxMax[1] - data->bboxMin[1];
    double dz = data->bboxMax[2] - data->bboxMin[2];
    double diag = std::sqrt(dx*dx + dy*dy + dz*dz);
    double dist = (diag < 1e-6) ? 1.0 : diag * 1.5;
    cam->SetPosition(data->center[0], data->center[1], data->center[2] - dist);
    cam->SetViewUp(0.0, -1.0, 0.0);
    m_renderer->ResetCameraClippingRange();
    renderWindow()->Render();
}

void MainOpengl::removeCachedCloud(std::shared_ptr<NodeCloudData> data)
{
    const QString nodeName = cloudKey(data);
    if (nodeName.isEmpty() || !m_cloudCache.contains(nodeName))
        return;

    auto crd = m_cloudCache.take(nodeName);
    m_renderer->RemoveActor(crd.cloudActor);
    m_renderer->RemoveActor(crd.bboxActor);
    if (crd.surgicalMeshActor)
        m_renderer->RemoveActor(crd.surgicalMeshActor);
    if (crd.curvedAxisActor)
        m_renderer->RemoveActor(crd.curvedAxisActor);
    if (crd.straightAxisActor)
        m_renderer->RemoveActor(crd.straightAxisActor);
    if (crd.slicePlanesActor)
        m_renderer->RemoveActor(crd.slicePlanesActor);
    if (crd.firstSlicePlaneActor)
        m_renderer->RemoveActor(crd.firstSlicePlaneActor);
    if (crd.sliceBoundingBoxActor)
        m_renderer->RemoveActor(crd.sliceBoundingBoxActor);
    if (crd.sliceContoursActor)
        m_renderer->RemoveActor(crd.sliceContoursActor);
    if (crd.fittedSliceContoursActor)
        m_renderer->RemoveActor(crd.fittedSliceContoursActor);
    if (crd.rotationStartMarkersActor)
        m_renderer->RemoveActor(crd.rotationStartMarkersActor);
    if (crd.equalDoseSurfaceActor)
        m_renderer->RemoveActor(crd.equalDoseSurfaceActor);
    if (crd.sprayPathActor)
        m_renderer->RemoveActor(crd.sprayPathActor);
    if (crd.sprayPathConnectionsActor)
        m_renderer->RemoveActor(crd.sprayPathConnectionsActor);
    if (crd.continuousSprayPathActor)
        m_renderer->RemoveActor(crd.continuousSprayPathActor);
    if (crd.continuousPathTransitionsActor)
        m_renderer->RemoveActor(crd.continuousPathTransitionsActor);
    if (crd.joint6ResetMarkersActor)
        m_renderer->RemoveActor(crd.joint6ResetMarkersActor);
    if (crd.nozzlePosePreviewActor)
        m_renderer->RemoveActor(crd.nozzlePosePreviewActor);
    if (m_activeCloudNode == nodeName)
        m_activeCloudNode.clear();
    renderWindow()->Render();
}

void MainOpengl::requestCloudRebuild(std::shared_ptr<NodeCloudData> data, const QString &outputDir)
{
    if (!data || !data->polyData || !m_pointDeal)
        return;

    m_pointDeal->requestCloudRebuild(cloudKey(data), data->polyData, outputDir);
}

void MainOpengl::requestPathPlanning(std::shared_ptr<NodeCloudData> data,
                                     int sampleCount,
                                     const QString &rodType,
                                     double voxelSize,
                                     double medialPercentile,
                                     int sectionCount,
                                     double sectionHalfWidth,
                                     int smoothPointsNum,
                                     int smoothWindow,
                                     double excludeOpeningDistance,
                                     int excludeOpeningLayers,
                                     double outletRearExtent,
                                     double safetyForwardExtent,
                                     double safeMarginBottom,
                                     double safeMarginTop)
{
    if (!data || !data->surgicalMesh || !m_pointDeal)
        return;

    vtkSmartPointer<vtkPolyData> referenceMesh = loadMeshPolyData(data);
    m_pointDeal->requestPathPlanning(cloudKey(data), data->surgicalMesh, referenceMesh,
                                     sampleCount,
                                     rodType, voxelSize, medialPercentile, sectionCount,
                                     sectionHalfWidth, smoothPointsNum, smoothWindow,
                                     excludeOpeningDistance, excludeOpeningLayers,
                                     outletRearExtent, safetyForwardExtent,
                                     safeMarginBottom, safeMarginTop);
}

void MainOpengl::showPathPlanningResult(std::shared_ptr<NodeCloudData> data,
                                        vtkSmartPointer<vtkPolyData> sampledCloud,
                                        vtkSmartPointer<vtkPolyData> curvedAxis,
                                        vtkSmartPointer<vtkPolyData> straightAxis)
{
    if (!data)
        return;

    data->pathSampleCloud = sampledCloud;
    data->curvedAxis = curvedAxis;
    data->straightAxis = straightAxis;

    const QString nodeName = cloudKey(data);
    if (nodeName.isEmpty())
        return;

    ensureCloudRenderData(data);
    if (!m_cloudCache.contains(nodeName))
        return;

    auto &crd = m_cloudCache[nodeName];
    if (crd.curvedAxisActor)
        m_renderer->RemoveActor(crd.curvedAxisActor);
    if (crd.straightAxisActor)
        m_renderer->RemoveActor(crd.straightAxisActor);

    crd.curvedAxisActor = createLineActor(curvedAxis, QColor(220, 38, 38), 2.0);
    crd.straightAxisActor = createLineActor(straightAxis, QColor(0, 102, 255), 7.0);
    if (crd.curvedAxisActor)
        m_renderer->AddActor(crd.curvedAxisActor);
    if (crd.straightAxisActor)
        m_renderer->AddActor(crd.straightAxisActor);

    const bool visible = crd.cloudActor && crd.cloudActor->GetVisibility();
    if (crd.curvedAxisActor)
        crd.curvedAxisActor->SetVisibility(visible && data->showCurvedAxis);
    if (crd.straightAxisActor)
        crd.straightAxisActor->SetVisibility(visible);
    renderWindow()->Render();
}

void MainOpengl::requestSlicePlanning(std::shared_ptr<NodeCloudData> data,
                                      double sliceSpacing,
                                      double outletRearExtent,
                                      double safetyForwardExtent,
                                      double safeMarginBottom,
                                      double safeMarginTop,
                                      double planeScaleRatio)
{
    if (!data || !m_pointDeal || !data->straightAxis || !data->surgicalMesh ||
        data->surgicalMesh->GetNumberOfPoints() == 0)
        return;

    m_pointDeal->requestSlicePlanning(cloudKey(data), data->surgicalMesh, data->straightAxis,
                                      data->curvedAxis, sliceSpacing, outletRearExtent,
                                      safetyForwardExtent, safeMarginBottom, safeMarginTop,
                                      planeScaleRatio);
}

void MainOpengl::showSlicePlanningResult(std::shared_ptr<NodeCloudData> data,
                                         vtkSmartPointer<vtkPolyData> slicePlanes,
                                         vtkSmartPointer<vtkPolyData> firstSlicePlane,
                                         vtkSmartPointer<vtkPolyData> boundingBox)
{
    if (!data)
        return;

    data->slicePlanes = slicePlanes;
    data->firstSlicePlane = firstSlicePlane;
    data->sliceBoundingBox = boundingBox;
    data->showSliceLayers = true;
    data->sliceContours = nullptr;
    data->fittedSliceContours = nullptr;
    data->rotationStartMarkers = nullptr;
    data->equalDoseSurface = nullptr;
    data->sprayPath = nullptr;
    data->sprayPathConnections = nullptr;
    data->continuousSprayPath = nullptr;
    data->continuousPathTransitions = nullptr;
    data->joint6ResetMarkers = nullptr;
    data->nozzlePoseSequence = nullptr;
    data->nozzlePosePreview = nullptr;

    const QString nodeName = cloudKey(data);
    if (nodeName.isEmpty())
        return;
    ensureCloudRenderData(data);
    if (!m_cloudCache.contains(nodeName))
        return;

    auto &crd = m_cloudCache[nodeName];
    if (crd.slicePlanesActor)
        m_renderer->RemoveActor(crd.slicePlanesActor);
    if (crd.firstSlicePlaneActor)
        m_renderer->RemoveActor(crd.firstSlicePlaneActor);
    if (crd.sliceBoundingBoxActor)
        m_renderer->RemoveActor(crd.sliceBoundingBoxActor);
    if (crd.sliceContoursActor)
        m_renderer->RemoveActor(crd.sliceContoursActor);
    if (crd.fittedSliceContoursActor)
        m_renderer->RemoveActor(crd.fittedSliceContoursActor);
    if (crd.rotationStartMarkersActor)
        m_renderer->RemoveActor(crd.rotationStartMarkersActor);
    if (crd.equalDoseSurfaceActor)
        m_renderer->RemoveActor(crd.equalDoseSurfaceActor);
    if (crd.sprayPathActor)
        m_renderer->RemoveActor(crd.sprayPathActor);
    if (crd.sprayPathConnectionsActor)
        m_renderer->RemoveActor(crd.sprayPathConnectionsActor);
    if (crd.continuousSprayPathActor)
        m_renderer->RemoveActor(crd.continuousSprayPathActor);
    if (crd.continuousPathTransitionsActor)
        m_renderer->RemoveActor(crd.continuousPathTransitionsActor);
    if (crd.joint6ResetMarkersActor)
        m_renderer->RemoveActor(crd.joint6ResetMarkersActor);
    if (crd.nozzlePosePreviewActor)
        m_renderer->RemoveActor(crd.nozzlePosePreviewActor);
    crd.sliceContoursActor = nullptr;
    crd.fittedSliceContoursActor = nullptr;
    crd.rotationStartMarkersActor = nullptr;
    crd.equalDoseSurfaceActor = nullptr;
    crd.sprayPathActor = nullptr;
    crd.sprayPathConnectionsActor = nullptr;
    crd.continuousSprayPathActor = nullptr;
    crd.continuousPathTransitionsActor = nullptr;
    crd.joint6ResetMarkersActor = nullptr;
    crd.nozzlePosePreviewActor = nullptr;

    crd.slicePlanesActor = createSurfaceActor(slicePlanes, QColor(64, 153, 255), 0.20);
    crd.firstSlicePlaneActor = createSurfaceActor(firstSlicePlane, QColor(255, 0, 0), 0.30);
    crd.sliceBoundingBoxActor = createLineActor(boundingBox, QColor(26, 178, 51), 2.0);
    if (crd.slicePlanesActor)
        m_renderer->AddActor(crd.slicePlanesActor);
    if (crd.firstSlicePlaneActor)
        m_renderer->AddActor(crd.firstSlicePlaneActor);
    if (crd.sliceBoundingBoxActor)
        m_renderer->AddActor(crd.sliceBoundingBoxActor);

    const bool visible = crd.cloudActor && crd.cloudActor->GetVisibility();
    if (crd.slicePlanesActor)
        crd.slicePlanesActor->SetVisibility(visible);
    if (crd.firstSlicePlaneActor)
        crd.firstSlicePlaneActor->SetVisibility(visible);
    if (crd.sliceBoundingBoxActor)
        crd.sliceBoundingBoxActor->SetVisibility(visible);
    renderWindow()->Render();
}

void MainOpengl::requestSliceContours(std::shared_ptr<NodeCloudData> data)
{
    if (!data || !m_pointDeal || !data->surgicalMesh ||
        (!data->slicePlanes && !data->firstSlicePlane)) {
        return;
    }

    m_pointDeal->requestSliceContours(cloudKey(data), data->surgicalMesh,
                                      data->slicePlanes, data->firstSlicePlane);
}

void MainOpengl::showSliceContours(std::shared_ptr<NodeCloudData> data,
                                   vtkSmartPointer<vtkPolyData> contours)
{
    if (!data)
        return;
    data->sliceContours = contours;
    data->fittedSliceContours = nullptr;
    data->rotationStartMarkers = nullptr;
    data->equalDoseSurface = nullptr;
    data->sprayPath = nullptr;
    data->sprayPathConnections = nullptr;
    data->continuousSprayPath = nullptr;
    data->continuousPathTransitions = nullptr;
    data->joint6ResetMarkers = nullptr;
    data->nozzlePoseSequence = nullptr;
    data->nozzlePosePreview = nullptr;

    const QString nodeName = cloudKey(data);
    if (nodeName.isEmpty())
        return;
    ensureCloudRenderData(data);
    if (!m_cloudCache.contains(nodeName))
        return;

    auto &crd = m_cloudCache[nodeName];
    if (crd.sliceContoursActor)
        m_renderer->RemoveActor(crd.sliceContoursActor);
    if (crd.fittedSliceContoursActor)
        m_renderer->RemoveActor(crd.fittedSliceContoursActor);
    if (crd.rotationStartMarkersActor)
        m_renderer->RemoveActor(crd.rotationStartMarkersActor);
    if (crd.equalDoseSurfaceActor)
        m_renderer->RemoveActor(crd.equalDoseSurfaceActor);
    if (crd.sprayPathActor)
        m_renderer->RemoveActor(crd.sprayPathActor);
    if (crd.sprayPathConnectionsActor)
        m_renderer->RemoveActor(crd.sprayPathConnectionsActor);
    if (crd.continuousSprayPathActor)
        m_renderer->RemoveActor(crd.continuousSprayPathActor);
    if (crd.continuousPathTransitionsActor)
        m_renderer->RemoveActor(crd.continuousPathTransitionsActor);
    if (crd.joint6ResetMarkersActor)
        m_renderer->RemoveActor(crd.joint6ResetMarkersActor);
    if (crd.nozzlePosePreviewActor)
        m_renderer->RemoveActor(crd.nozzlePosePreviewActor);
    crd.fittedSliceContoursActor = nullptr;
    crd.rotationStartMarkersActor = nullptr;
    crd.equalDoseSurfaceActor = nullptr;
    crd.sprayPathActor = nullptr;
    crd.sprayPathConnectionsActor = nullptr;
    crd.continuousSprayPathActor = nullptr;
    crd.continuousPathTransitionsActor = nullptr;
    crd.joint6ResetMarkersActor = nullptr;
    crd.nozzlePosePreviewActor = nullptr;
    crd.sliceContoursActor = createLineActor(contours, QColor(255, 214, 10), 3.0);
    if (crd.sliceContoursActor)
        m_renderer->AddActor(crd.sliceContoursActor);

    const bool visible = crd.cloudActor && crd.cloudActor->GetVisibility();
    if (crd.sliceContoursActor)
        crd.sliceContoursActor->SetVisibility(visible && data->showSliceLayers);
    renderWindow()->Render();
}

void MainOpengl::requestContourFitting(std::shared_ptr<NodeCloudData> data,
                                       int fitPointCount,
                                       double coverageThreshold,
                                       int smoothWindow,
                                       double angleBinDeg,
                                       bool trimOpenEnds,
                                       int endCheckCount,
                                       double curvaturePeakRatio,
                                       double curvatureRecoverRatio,
                                       int maxTrimCount,
                                       int minKeepPointCount)
{
    if (!data || !m_pointDeal || !data->sliceContours || !data->straightAxis)
        return;
    m_pointDeal->requestContourFitting(cloudKey(data), data->sliceContours,
                                       data->straightAxis, data->curvedAxis,
                                       fitPointCount, coverageThreshold,
                                       smoothWindow, angleBinDeg, trimOpenEnds,
                                       endCheckCount, curvaturePeakRatio,
                                       curvatureRecoverRatio, maxTrimCount,
                                       minKeepPointCount);
}

void MainOpengl::showFittedContours(std::shared_ptr<NodeCloudData> data,
                                    vtkSmartPointer<vtkPolyData> fittedContours)
{
    if (!data)
        return;
    data->fittedSliceContours = fittedContours;
    data->rotationStartMarkers = buildRotationStartMarkers(
        fittedContours, data->straightAxis, data->curvedAxis);
    data->equalDoseSurface = nullptr;
    data->sprayPath = nullptr;
    data->sprayPathConnections = nullptr;
    data->continuousSprayPath = nullptr;
    data->continuousPathTransitions = nullptr;
    data->joint6ResetMarkers = nullptr;
    data->nozzlePoseSequence = nullptr;
    data->nozzlePosePreview = nullptr;

    const QString nodeName = cloudKey(data);
    if (nodeName.isEmpty())
        return;
    ensureCloudRenderData(data);
    if (!m_cloudCache.contains(nodeName))
        return;

    auto &crd = m_cloudCache[nodeName];
    if (crd.fittedSliceContoursActor)
        m_renderer->RemoveActor(crd.fittedSliceContoursActor);
    if (crd.rotationStartMarkersActor)
        m_renderer->RemoveActor(crd.rotationStartMarkersActor);
    if (crd.equalDoseSurfaceActor)
        m_renderer->RemoveActor(crd.equalDoseSurfaceActor);
    if (crd.sprayPathActor)
        m_renderer->RemoveActor(crd.sprayPathActor);
    if (crd.sprayPathConnectionsActor)
        m_renderer->RemoveActor(crd.sprayPathConnectionsActor);
    if (crd.continuousSprayPathActor)
        m_renderer->RemoveActor(crd.continuousSprayPathActor);
    if (crd.continuousPathTransitionsActor)
        m_renderer->RemoveActor(crd.continuousPathTransitionsActor);
    if (crd.joint6ResetMarkersActor)
        m_renderer->RemoveActor(crd.joint6ResetMarkersActor);
    if (crd.nozzlePosePreviewActor)
        m_renderer->RemoveActor(crd.nozzlePosePreviewActor);
    crd.equalDoseSurfaceActor = nullptr;
    crd.sprayPathActor = nullptr;
    crd.sprayPathConnectionsActor = nullptr;
    crd.continuousSprayPathActor = nullptr;
    crd.continuousPathTransitionsActor = nullptr;
    crd.joint6ResetMarkersActor = nullptr;
    crd.nozzlePosePreviewActor = nullptr;
    crd.fittedSliceContoursActor = createLineActor(fittedContours, QColor(0, 230, 230), 4.0);
    crd.rotationStartMarkersActor = createColoredPointActor(data->rotationStartMarkers, 14.0);
    if (crd.fittedSliceContoursActor)
        m_renderer->AddActor(crd.fittedSliceContoursActor);
    if (crd.rotationStartMarkersActor) {
        crd.rotationStartMarkersActor->GetProperty()->SetRenderPointsAsSpheres(true);
        m_renderer->AddActor(crd.rotationStartMarkersActor);
    }
    const bool visible = crd.cloudActor && crd.cloudActor->GetVisibility();
    if (crd.fittedSliceContoursActor)
        crd.fittedSliceContoursActor->SetVisibility(visible && data->showFittedContours);
    if (crd.rotationStartMarkersActor)
        crd.rotationStartMarkersActor->SetVisibility(visible && data->showFittedContours);
    renderWindow()->Render();
}

void MainOpengl::requestEqualDosePath(std::shared_ptr<NodeCloudData> data,
                                      double nozzleVerticalLength,
                                      double sprayRodRadius,
                                      double safetyClearance,
                                      int closedSamples,
                                      int openSamples,
                                      double connectionRefAngleDeg,
                                      bool useTopOpenEndpoint,
                                      const QString &openEndpointMode,
                                      const QString &manualRegionRanges,
                                      double manualZeroOffsetDeg,
                                      const QString &angleViewDirection,
                                      const QString &angleIncreaseDirection)
{
    if (!data || !m_pointDeal || !data->fittedSliceContours || !data->straightAxis)
        return;
    m_pointDeal->requestEqualDosePath(
        cloudKey(data), data->fittedSliceContours, data->straightAxis, data->curvedAxis,
        nozzleVerticalLength, sprayRodRadius, safetyClearance, closedSamples, openSamples,
        connectionRefAngleDeg, useTopOpenEndpoint, openEndpointMode,
        manualRegionRanges, manualZeroOffsetDeg,
        angleViewDirection, angleIncreaseDirection);
}

void MainOpengl::showEqualDosePathResult(std::shared_ptr<NodeCloudData> data,
                                         vtkSmartPointer<vtkPolyData> equalDoseSurface,
                                         vtkSmartPointer<vtkPolyData> sprayPath,
                                         vtkSmartPointer<vtkPolyData> pathConnections)
{
    if (!data)
        return;
    data->equalDoseSurface = equalDoseSurface;
    data->sprayPath = sprayPath;
    data->sprayPathConnections = pathConnections;
    data->continuousSprayPath = nullptr;
    data->continuousPathTransitions = nullptr;
    data->joint6ResetMarkers = nullptr;
    data->nozzlePoseSequence = nullptr;
    data->nozzlePosePreview = nullptr;

    const QString nodeName = cloudKey(data);
    if (nodeName.isEmpty())
        return;
    ensureCloudRenderData(data);
    if (!m_cloudCache.contains(nodeName))
        return;

    auto &crd = m_cloudCache[nodeName];
    if (crd.equalDoseSurfaceActor)
        m_renderer->RemoveActor(crd.equalDoseSurfaceActor);
    if (crd.sprayPathActor)
        m_renderer->RemoveActor(crd.sprayPathActor);
    if (crd.sprayPathConnectionsActor)
        m_renderer->RemoveActor(crd.sprayPathConnectionsActor);
    if (crd.continuousSprayPathActor)
        m_renderer->RemoveActor(crd.continuousSprayPathActor);
    if (crd.continuousPathTransitionsActor)
        m_renderer->RemoveActor(crd.continuousPathTransitionsActor);
    if (crd.joint6ResetMarkersActor)
        m_renderer->RemoveActor(crd.joint6ResetMarkersActor);
    if (crd.nozzlePosePreviewActor)
        m_renderer->RemoveActor(crd.nozzlePosePreviewActor);

    crd.continuousSprayPathActor = nullptr;
    crd.continuousPathTransitionsActor = nullptr;
    crd.joint6ResetMarkersActor = nullptr;
    crd.nozzlePosePreviewActor = nullptr;

    crd.equalDoseSurfaceActor = createColoredSurfaceActor(equalDoseSurface, 0.28);
    crd.sprayPathActor = createLineActor(sprayPath, QColor(217, 26, 26), 4.0);
    crd.sprayPathConnectionsActor = createLineActor(
        pathConnections, QColor(26, 166, 51), 4.5);
    if (crd.equalDoseSurfaceActor)
        m_renderer->AddActor(crd.equalDoseSurfaceActor);
    if (crd.sprayPathActor)
        m_renderer->AddActor(crd.sprayPathActor);
    if (crd.sprayPathConnectionsActor)
        m_renderer->AddActor(crd.sprayPathConnectionsActor);

    const bool nodeVisible = crd.cloudActor && crd.cloudActor->GetVisibility();
    if (crd.equalDoseSurfaceActor)
        crd.equalDoseSurfaceActor->SetVisibility(nodeVisible && data->showEqualDoseSurface);
    if (crd.sprayPathActor)
        crd.sprayPathActor->SetVisibility(nodeVisible && data->showSprayPath);
    if (crd.sprayPathConnectionsActor)
        crd.sprayPathConnectionsActor->SetVisibility(nodeVisible && data->showSprayPath);
    renderWindow()->Render();
}

void MainOpengl::setEqualDoseSurfaceVisible(std::shared_ptr<NodeCloudData> data, bool visible)
{
    if (!data)
        return;
    data->showEqualDoseSurface = visible;
    const QString nodeName = cloudKey(data);
    if (m_cloudCache.contains(nodeName) && m_cloudCache[nodeName].equalDoseSurfaceActor) {
        const bool nodeVisible = m_cloudCache[nodeName].cloudActor &&
                                 m_cloudCache[nodeName].cloudActor->GetVisibility();
        m_cloudCache[nodeName].equalDoseSurfaceActor->SetVisibility(nodeVisible && visible);
        renderWindow()->Render();
    }
}

void MainOpengl::setSprayPathVisible(std::shared_ptr<NodeCloudData> data, bool visible)
{
    if (!data)
        return;
    data->showSprayPath = visible;
    const QString nodeName = cloudKey(data);
    if (!m_cloudCache.contains(nodeName))
        return;
    auto &crd = m_cloudCache[nodeName];
    const bool nodeVisible = crd.cloudActor && crd.cloudActor->GetVisibility();
    if (crd.sprayPathActor)
        crd.sprayPathActor->SetVisibility(nodeVisible && visible);
    if (crd.sprayPathConnectionsActor)
        crd.sprayPathConnectionsActor->SetVisibility(nodeVisible && visible);
    renderWindow()->Render();
}

void MainOpengl::requestContinuousPath(std::shared_ptr<NodeCloudData> data,
                                       double maxJoint6SweepDeg,
                                       double minPointSpacing,
                                       double maxTransitionDistance,
                                       bool reverseLayerOrder,
                                       bool autoReverseOpenLayers)
{
    if (!data || !m_pointDeal || !data->sprayPath || !data->straightAxis)
        return;
    m_pointDeal->requestContinuousPath(
        cloudKey(data), data->sprayPath, data->straightAxis,
        maxJoint6SweepDeg, minPointSpacing, maxTransitionDistance,
        reverseLayerOrder, autoReverseOpenLayers);
}

void MainOpengl::showContinuousPathResult(
    std::shared_ptr<NodeCloudData> data,
    vtkSmartPointer<vtkPolyData> continuousPath,
    vtkSmartPointer<vtkPolyData> transitions,
    vtkSmartPointer<vtkPolyData> resetMarkers)
{
    if (!data)
        return;
    data->continuousSprayPath = continuousPath;
    data->continuousPathTransitions = transitions;
    data->joint6ResetMarkers = resetMarkers;
    data->nozzlePoseSequence = nullptr;
    data->nozzlePosePreview = nullptr;

    const QString nodeName = cloudKey(data);
    if (nodeName.isEmpty())
        return;
    ensureCloudRenderData(data);
    if (!m_cloudCache.contains(nodeName))
        return;

    auto &crd = m_cloudCache[nodeName];
    if (crd.continuousSprayPathActor)
        m_renderer->RemoveActor(crd.continuousSprayPathActor);
    if (crd.continuousPathTransitionsActor)
        m_renderer->RemoveActor(crd.continuousPathTransitionsActor);
    if (crd.joint6ResetMarkersActor)
        m_renderer->RemoveActor(crd.joint6ResetMarkersActor);
    if (crd.nozzlePosePreviewActor)
        m_renderer->RemoveActor(crd.nozzlePosePreviewActor);

    crd.continuousSprayPathActor = createLineActor(
        continuousPath, QColor(255, 0, 110), 5.5);
    crd.continuousPathTransitionsActor = createColoredLineActor(transitions, 4.0);
    crd.joint6ResetMarkersActor = createColoredPointActor(resetMarkers, 12.0);
    crd.nozzlePosePreviewActor = nullptr;
    if (crd.continuousSprayPathActor)
        m_renderer->AddActor(crd.continuousSprayPathActor);
    if (crd.continuousPathTransitionsActor)
        m_renderer->AddActor(crd.continuousPathTransitionsActor);
    if (crd.joint6ResetMarkersActor)
        m_renderer->AddActor(crd.joint6ResetMarkersActor);

    const bool nodeVisible = crd.cloudActor && crd.cloudActor->GetVisibility();
    if (crd.continuousSprayPathActor)
        crd.continuousSprayPathActor->SetVisibility(nodeVisible && data->showContinuousPath);
    if (crd.continuousPathTransitionsActor)
        crd.continuousPathTransitionsActor->SetVisibility(nodeVisible && data->showContinuousPath);
    if (crd.joint6ResetMarkersActor)
        crd.joint6ResetMarkersActor->SetVisibility(nodeVisible && data->showContinuousPath);
    renderWindow()->Render();
}

void MainOpengl::setContinuousPathVisible(std::shared_ptr<NodeCloudData> data, bool visible)
{
    if (!data)
        return;
    data->showContinuousPath = visible;
    const QString nodeName = cloudKey(data);
    if (!m_cloudCache.contains(nodeName))
        return;
    auto &crd = m_cloudCache[nodeName];
    const bool nodeVisible = crd.cloudActor && crd.cloudActor->GetVisibility();
    if (crd.continuousSprayPathActor)
        crd.continuousSprayPathActor->SetVisibility(nodeVisible && visible);
    if (crd.continuousPathTransitionsActor)
        crd.continuousPathTransitionsActor->SetVisibility(nodeVisible && visible);
    if (crd.joint6ResetMarkersActor)
        crd.joint6ResetMarkersActor->SetVisibility(nodeVisible && visible);
    renderWindow()->Render();
}

void MainOpengl::requestNozzlePoses(std::shared_ptr<NodeCloudData> data,
                                    bool reverseLayerOrder)
{
    if (!data || !m_pointDeal || !data->fittedSliceContours ||
        !data->rotationStartMarkers || !data->straightAxis) {
        return;
    }
    m_pointDeal->requestNozzlePoses(
        cloudKey(data), data->fittedSliceContours, data->rotationStartMarkers,
        data->straightAxis, data->curvedAxis, reverseLayerOrder,
        data->plannedSafetyForwardExtent, data->plannedEntryTipStandoff);
}

void MainOpengl::showNozzlePoseResult(
    std::shared_ptr<NodeCloudData> data,
    vtkSmartPointer<vtkPolyData> poseSequence,
    vtkSmartPointer<vtkPolyData> posePreview)
{
    if (!data)
        return;
    data->nozzlePoseSequence = poseSequence;
    data->nozzlePosePreview = posePreview;
    data->showNozzlePoses = true;

    const QString nodeName = cloudKey(data);
    if (nodeName.isEmpty())
        return;
    ensureCloudRenderData(data);
    if (!m_cloudCache.contains(nodeName))
        return;

    auto &crd = m_cloudCache[nodeName];
    if (crd.nozzlePosePreviewActor)
        m_renderer->RemoveActor(crd.nozzlePosePreviewActor);
    crd.nozzlePosePreviewActor = createNozzlePoseActor(posePreview);
    if (crd.nozzlePosePreviewActor)
        m_renderer->AddActor(crd.nozzlePosePreviewActor);

    const bool nodeVisible = crd.cloudActor && crd.cloudActor->GetVisibility();
    if (crd.nozzlePosePreviewActor)
        crd.nozzlePosePreviewActor->SetVisibility(nodeVisible && data->showNozzlePoses);
    renderWindow()->Render();
}

void MainOpengl::setNozzlePosesVisible(std::shared_ptr<NodeCloudData> data, bool visible)
{
    if (!data)
        return;
    data->showNozzlePoses = visible;
    const QString nodeName = cloudKey(data);
    if (!m_cloudCache.contains(nodeName))
        return;
    auto &crd = m_cloudCache[nodeName];
    const bool nodeVisible = crd.cloudActor && crd.cloudActor->GetVisibility();
    if (crd.nozzlePosePreviewActor)
        crd.nozzlePosePreviewActor->SetVisibility(nodeVisible && visible);
    renderWindow()->Render();
}

void MainOpengl::startSurgicalAreaSelection(std::shared_ptr<NodeCloudData> data)
{
    if (!data || !data->polyData || data->meshFilePath.isEmpty())
        return;

    vtkSmartPointer<vtkPolyData> mesh = loadMeshPolyData(data);
    if (!mesh || mesh->GetNumberOfPoints() == 0)
        return;

    EnableCrop(false);
    m_surgicalAreaTarget = data;
    showPointCloud(data);
    m_glyphFilter->SetInputData(data->polyData);
    m_glyphFilter->Update();
    m_pointsActor->VisibilityOff();
    EnableCrop(true, Crop_Rect);
}

void MainOpengl::HideActiveBoundingBox()
{
    if (m_activeCloudNode.isEmpty() || !m_cloudCache.contains(m_activeCloudNode))
        return;

    m_cloudCache[m_activeCloudNode].bboxActor->VisibilityOff();
    renderWindow()->Render();
}
