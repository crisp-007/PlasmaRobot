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
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <QResizeEvent>
#include <vtkMath.h>

// ==========================================
// 交互样式工厂宏
// ==========================================
vtkStandardNewMacro(RectDrawStyle);
vtkStandardNewMacro(PolygonDrawStyle);
vtkStandardNewMacro(PointcloudMoveStyle);

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

// 右键按下事件：开始平移操作
void RectDrawStyle::OnRightButtonDown() {

}

// 右键抬起事件：结束平移操作
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
// 3. PointcloudMoveStyle 实现 (纯移动模式)
// ==========================================

// 右键按下事件：开始平移
void PointcloudMoveStyle::OnRightButtonDown() {
    if (m_actor) {
        m_isMoving = true;
        int* pos = this->Interactor->GetEventPosition();
        m_lastPos[0] = pos[0];
        m_lastPos[1] = pos[1];
    }
}

// 右键抬起事件：结束平移并结算矩阵
void PointcloudMoveStyle::OnRightButtonUp() {
    m_isMoving = false;
    
    // 移动结束后，获取Actor当前的变换矩阵并更新到MainOpengl
    if (m_actor && m_gl) {
        vtkMatrix4x4* matrix = m_actor->GetMatrix();
        m_gl->UpdateActorMatrix(matrix);
        qDebug() << "Matrix updated after move.";
    }
}

// 鼠标移动事件：执行平移逻辑
void PointcloudMoveStyle::OnMouseMove() {
    if (m_isMoving && m_actor) {
        vtkCamera* camera = this->CurrentRenderer ? this->CurrentRenderer->GetActiveCamera() : nullptr;
        if (camera) {
             int* pos = this->Interactor->GetEventPosition();
             double dx = pos[0] - m_lastPos[0];
             double dy = pos[1] - m_lastPos[1];
             m_lastPos[0] = pos[0];
             m_lastPos[1] = pos[1];
             
             // 计算屏幕空间移动对应的世界空间位移
             // 根据相机距离和视角计算比例因子
             double dist = camera->GetDistance();
             double angle = camera->GetViewAngle();
             double height = this->Interactor->GetRenderWindow()->GetSize()[1]; 
             double scale = 2.0 * dist * tan(angle * 0.5 * 0.0174532925) / height;
             
             double moveX = dx * scale;
             double moveY = dy * scale;
             
             // 获取相机坐标系方向
             double viewFocus[3], viewPos[3], viewUp[3];
             camera->GetFocalPoint(viewFocus);
             camera->GetPosition(viewPos);
             camera->GetViewUp(viewUp);
             
             double dir[3];
             vtkMath::Subtract(viewFocus, viewPos, dir);
             vtkMath::Normalize(dir);
             
             double right[3];
             vtkMath::Cross(dir, viewUp, right);
             vtkMath::Normalize(right);
             
             vtkMath::Cross(right, dir, viewUp);
             vtkMath::Normalize(viewUp);
             
             // 计算世界坐标系下的位移向量
             double displacement[3];
             for(int i=0; i<3; i++) {
                 displacement[i] = right[i] * moveX + viewUp[i] * moveY;
             }
             
             // 应用位移到Actor
             double* pos3d = m_actor->GetPosition();
             m_actor->SetPosition(pos3d[0] + displacement[0], 
                                  pos3d[1] + displacement[1], 
                                  pos3d[2] + displacement[2]);
             
             this->Interactor->Render();
        }
    } else {
        // 非移动状态下，保持默认交互（如左键旋转）
        vtkInteractorStyleTrackballCamera::OnMouseMove();
    }
}


// ==========================================
// MainOpengl: 生命周期与初始化
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
        // 通过 metaObject 异步调用 stop，确保在 worker 线程执行
        QMetaObject::invokeMethod(m_rosWorker, "stop", Qt::BlockingQueuedConnection);
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
        
        // 设置背景颜色（渐变天蓝色）
        SetBackgroundColor(QColor(135, 206, 235), QColor(255, 255, 255));
        
        // 初始化并显示默认网格（mm）
        SetGridProperties(4.0, 0.5, QColor(200, 200, 200));

        // 2. 设置默认交互样式
        // 使用自定义的PointcloudMoveStyle，支持右键平移点云
        vtkSmartPointer<PointcloudMoveStyle> style = vtkSmartPointer<PointcloudMoveStyle>::New();
        style->SetActor(m_pcdActor); 
        style->SetMainOpengl(this);
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
    m_pcdActor = vtkSmartPointer<vtkActor>::New();
    
    // 4. 初始化裁剪相关的辅助显示Actor
    // 矩形选择框 Actor
    m_rectActor = vtkSmartPointer<vtkActor2D>::New();
    vtkSmartPointer<vtkPolyDataMapper2D> rectMapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    m_rectActor->SetMapper(rectMapper);
    m_rectActor->GetProperty()->SetColor(0.0, 1.0, 0.0); // 绿色线条
    m_rectActor->GetProperty()->SetLineWidth(2.0);
    m_rectActor->VisibilityOff(); // 默认隐藏
    m_renderer->AddActor(m_rectActor);

    // 多边形选择框 Actor
    m_polyActor = vtkSmartPointer<vtkActor2D>::New();
    vtkSmartPointer<vtkPolyDataMapper2D> polyMapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    m_polyActor->SetMapper(polyMapper);
    m_polyActor->GetProperty()->SetColor(1.0, 0.0, 0.0); // 红色线条
    m_polyActor->GetProperty()->SetLineWidth(2.0);
    m_polyActor->VisibilityOff(); // 默认隐藏
    m_renderer->AddActor(m_polyActor);
    
    // 多边形顶点容器
    m_polyPoints = vtkSmartPointer<vtkPoints>::New();

    // 5. 初始化其他Widget (备用/保留代码)
    // 盒式裁剪Widget
    m_cropWidget = vtkSmartPointer<vtkBoxWidget2>::New();
    m_cropWidget->SetInteractor(renderWindow()->GetInteractor());
    m_cropWidget->CreateDefaultRepresentation();
    
    vtkSmartPointer<vtkCallbackCommand> cropCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    cropCallback->SetCallback(BoxCropCallback);
    cropCallback->SetClientData(this);
    m_cropWidget->AddObserver(vtkCommand::InteractionEvent, cropCallback);

    // 移动Widget
    m_moveWidget = vtkSmartPointer<vtkBoxWidget2>::New();
    m_moveWidget->SetInteractor(renderWindow()->GetInteractor());
    m_moveWidget->CreateDefaultRepresentation();
    
    vtkSmartPointer<vtkCallbackCommand> moveCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    moveCallback->SetCallback(MoveCallback);
    moveCallback->SetClientData(this);
    m_moveWidget->AddObserver(vtkCommand::InteractionEvent, moveCallback);

    // 标签回调
    m_labelCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    m_labelCallback->SetCallback(LabelCallback);
    m_labelCallback->SetClientData(this);
}

// ==========================================
// MainOpengl: 数据加载与可视化
// ==========================================

// 加载PCD点云文件
void MainOpengl::LoadPCDPoint(const QString &filePath)
{
    if (filePath.isEmpty()) return;
    
    // 使用PCL库加载点云文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filePath.toStdString(), *cloud) == -1) {
        qDebug() << "Failed to load PCD file:" << filePath;
        return;
    }
    
    // 转换为VTK格式
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    // 创建标量数组用于颜色映射（这里使用Z轴高度）
    vtkSmartPointer<vtkFloatArray> scalars = vtkSmartPointer<vtkFloatArray>::New();
    scalars->SetNumberOfComponents(1);
    scalars->SetName("Elevation");

    for (const auto& p : *cloud) {
        points->InsertNextPoint(p.x, p.y, p.z);
        scalars->InsertNextValue(p.z);
    }
    
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->GetPointData()->SetScalars(scalars);
    
    // 使用VertexGlyphFilter优化点云渲染性能
    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputData(polyData);
    glyphFilter->Update();
    
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(glyphFilter->GetOutputPort());
    
    // 设置颜色映射范围（根据Z轴最小最大值）
    double bounds[6];
    polyData->GetBounds(bounds);
    mapper->SetScalarRange(bounds[4], bounds[5]);
    
    // 更新或创建Actor
    if (!m_pcdActor) {
        m_pcdActor = vtkSmartPointer<vtkActor>::New();
    }
    m_pcdActor->SetMapper(mapper);
    m_renderer->AddActor(m_pcdActor);
    
    // 更新默认交互样式中的Actor引用，确保移动功能可用
    if (m_defaultStyle) {
        PointcloudMoveStyle::SafeDownCast(m_defaultStyle)->SetActor(m_pcdActor);
    }
    
    // 重置相机视角以包含所有点云
    m_renderer->ResetCamera();
    if (renderWindow()) renderWindow()->Render();
}

// 设置网格属性并绘制
void MainOpengl::SetGridProperties(double gridSize, double step, const QColor &color)
{
    // 如果已存在网格，先移除
    if (m_gridActor) {
        m_renderer->RemoveActor(m_gridActor);
    }
    
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

    // 计算网格参数
    double halfSize = gridSize / 2.0;
    int numSteps = static_cast<int>(gridSize / step);
    
    // 构建平行于X轴的线
    for (int i = 0; i <= numSteps; ++i) {
        double y = -halfSize + i * step;
        vtkIdType p1 = points->InsertNextPoint(-halfSize, y, 0.0);
        vtkIdType p2 = points->InsertNextPoint(halfSize, y, 0.0);
        vtkIdType line[] = {p1, p2};
        lines->InsertNextCell(2, line);
    }

    // 构建平行于Y轴的线
    for (int i = 0; i <= numSteps; ++i) {
        double x = -halfSize + i * step;
        vtkIdType p1 = points->InsertNextPoint(x, -halfSize, 0.0);
        vtkIdType p2 = points->InsertNextPoint(x, halfSize, 0.0);
        vtkIdType line[] = {p1, p2};
        lines->InsertNextCell(2, line);
    }

    vtkSmartPointer<vtkPolyData> gridPoly = vtkSmartPointer<vtkPolyData>::New();
    gridPoly->SetPoints(points);
    gridPoly->SetLines(lines);

    vtkSmartPointer<vtkPolyDataMapper> gridMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    gridMapper->SetInputData(gridPoly);

    m_gridActor = vtkSmartPointer<vtkActor>::New();
    m_gridActor->SetMapper(gridMapper);
    m_gridActor->GetProperty()->SetColor(color.redF(), color.greenF(), color.blueF());
    m_gridActor->GetProperty()->SetLighting(false); // 关闭光照，确保网格颜色恒定

    m_renderer->AddActor(m_gridActor);
    if (renderWindow()) renderWindow()->Render();
}

// 设置背景渐变色
void MainOpengl::SetBackgroundColor(const QColor &topColor, const QColor &bottomColor)
{
    if (m_renderer) {
        m_renderer->SetBackground(topColor.redF(), topColor.greenF(), topColor.blueF());
        m_renderer->SetBackground2(bottomColor.redF(), bottomColor.greenF(), bottomColor.blueF());
        m_renderer->GradientBackgroundOn(); // 开启渐变背景
    }
}

// ==========================================
// MainOpengl: 交互模式控制
// ==========================================

// 启用/禁用裁剪模式
void MainOpengl::EnableCrop(bool enable, CropMode mode)
{
    if (enable) {
        // 启用裁剪前，禁用其他互斥模式
        EnableMove(false);
        EnableLabel(false);
        
        m_currentCropMode = mode;
        
        // 显示裁剪工具悬浮窗
        if (m_cropOverlay) {
            m_cropOverlay->show();
            UpdateCropOverlayPosition();
        }

        // 重置所有选择可视化状态
        m_hasRectSelection = false;
        m_rectActor->VisibilityOff();
        m_hasPolySelection = false;
        m_polyActor->VisibilityOff();
        if (m_polyPoints) m_polyPoints->Reset();

        // 根据模式设置交互样式
        if (mode == Crop_Box) {
             // 盒式裁剪（备用逻辑）
             if (renderWindow()->GetInteractor()) {
                renderWindow()->GetInteractor()->SetInteractorStyle(m_defaultStyle);
            }
            if (m_pcdActor && m_pcdActor->GetMapper()) {
                m_cropWidget->SetEnabled(1);
                double bounds[6];
                m_pcdActor->GetBounds(bounds);
                vtkBoxRepresentation* rep = vtkBoxRepresentation::SafeDownCast(m_cropWidget->GetRepresentation());
                if (rep) {
                    rep->PlaceWidget(bounds);
                    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkPolyDataMapper::SafeDownCast(m_pcdActor->GetMapper());
                    if (mapper) mapper->RemoveAllClippingPlanes();
                }
            }
        } else if (mode == Crop_Rect) {
            // 矩形裁剪：切换到 RectDrawStyle
            m_cropWidget->SetEnabled(0);
            // 清除之前可能已经设置的裁剪平面
            if (m_pcdActor && m_pcdActor->GetMapper()) {
                vtkPolyDataMapper::SafeDownCast(m_pcdActor->GetMapper())->RemoveAllClippingPlanes();
            }
            // 切换到矩形绘制样式
            vtkSmartPointer<RectDrawStyle> style = vtkSmartPointer<RectDrawStyle>::New();
            style->SetMainOpengl(this);
            // 切换到矩形绘制样式
            if (renderWindow()->GetInteractor()) 
            {
                renderWindow()->GetInteractor()->SetInteractorStyle(style);
            }
        } else if (mode == Crop_Poly) {
            // 多边形裁剪：切换到 PolygonDrawStyle
            m_cropWidget->SetEnabled(0);
            if (m_pcdActor && m_pcdActor->GetMapper()) {
                vtkPolyDataMapper::SafeDownCast(m_pcdActor->GetMapper())->RemoveAllClippingPlanes();
            }
            //重置多边形裁剪状态
            StartPolySelection();

            vtkSmartPointer<PolygonDrawStyle> style = vtkSmartPointer<PolygonDrawStyle>::New();
            style->SetMainOpengl(this);
            if (renderWindow()->GetInteractor()) {
                renderWindow()->GetInteractor()->SetInteractorStyle(style);
            }
        }
    } else {
        // 禁用裁剪：恢复默认状态
        m_currentCropMode = Crop_None;
        m_cropWidget->SetEnabled(0);
        
        if (m_cropOverlay) {
            m_cropOverlay->hide();
        }
        
        m_rectActor->VisibilityOff();
        m_polyActor->VisibilityOff();

        // 恢复默认交互样式（旋转/缩放）
        if (renderWindow()->GetInteractor()) {
            renderWindow()->GetInteractor()->SetInteractorStyle(m_defaultStyle);
        }
        
        if (m_pcdActor && m_pcdActor->GetMapper()) {
            vtkPolyDataMapper::SafeDownCast(m_pcdActor->GetMapper())->RemoveAllClippingPlanes();
        }
    }
    
    if (renderWindow()) renderWindow()->Render();
}

// 启用/禁用移动模式
void MainOpengl::EnableMove(bool enable)
{
    if (enable) {
        EnableCrop(false);
        EnableLabel(false);
        m_moveWidget->SetEnabled(1);
    } else {
        m_moveWidget->SetEnabled(0);
    }
}

// 启用/禁用标签模式
void MainOpengl::EnableLabel(bool enable)
{
    if (enable) {
        EnableCrop(false);
        EnableMove(false);
        // 开启标签逻辑（如添加Observer）
    } else {
        // 禁用标签逻辑
    }
}

// ==========================================
// MainOpengl: 选择辅助函数
// ==========================================

// 更新矩形选择框的可视化
void MainOpengl::UpdateRectSelection(int x1, int y1, int x2, int y2)
{
    // 规范化坐标：确保min和max正确
    int minX = std::min(x1, x2);
    int maxX = std::max(x1, x2);
    int minY = std::min(y1, y2);
    int maxY = std::max(y1, y2);
    
    //设置矩形的顶点坐标
    m_selectedRect = vtkRecti(minX, minY, maxX - minX, maxY - minY);
    m_hasRectSelection = true;
    
    // 构建矩形边框的几何数据 (PolyData)
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(minX, minY, 0);
    points->InsertNextPoint(maxX, minY, 0);
    points->InsertNextPoint(maxX, maxY, 0);
    points->InsertNextPoint(minX, maxY, 0);
    
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType line0[] = {0, 1};
    vtkIdType line1[] = {1, 2};
    vtkIdType line2[] = {2, 3};
    vtkIdType line3[] = {3, 0};
    lines->InsertNextCell(2, line0);
    lines->InsertNextCell(2, line1);
    lines->InsertNextCell(2, line2);
    lines->InsertNextCell(2, line3);
    
    vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
    poly->SetPoints(points);
    poly->SetLines(lines);
    
    // 更新Mapper并显示
    vtkPolyDataMapper2D* mapper = vtkPolyDataMapper2D::SafeDownCast(m_rectActor->GetMapper());
    if (mapper) {
        mapper->SetInputData(poly);
        m_rectActor->VisibilityOn();
    }
}

// 开始新的多边形选择（重置状态）
void MainOpengl::StartPolySelection()
{
    m_polyPoints->Reset();
    m_hasPolySelection = false;
    m_isPolyFinished = false;
    m_polyActor->VisibilityOff();
}

// 添加多边形顶点
void MainOpengl::AddPolyPoint(int x, int y)
{
    if (m_isPolyFinished) return;
    
    m_polyPoints->InsertNextPoint(x, y, 0.0);
    m_hasPolySelection = true;
    
    // 立即更新显示
    UpdatePolyDynamicLine(x, y);
    if (renderWindow()) renderWindow()->Render();
}

// 更新多边形动态预览线
void MainOpengl::UpdatePolyDynamicLine(int x, int y)
{
    // 如果点数过少，无法构成线段
    if (m_polyPoints->GetNumberOfPoints() < 1) return;
    
    // 复制已确定的点
    vtkSmartPointer<vtkPoints> displayPoints = vtkSmartPointer<vtkPoints>::New();
    displayPoints->DeepCopy(m_polyPoints);
    
    // 如果选择未完成，将当前鼠标位置作为临时点加入，形成动态跟随效果
    if (!m_isPolyFinished) {
        displayPoints->InsertNextPoint(x, y, 0.0);
    }
    
    // 构建线段连接所有点
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    int num = displayPoints->GetNumberOfPoints();
    for (int i = 0; i < num - 1; ++i) {
        vtkIdType line[] = {i, i+1};
        lines->InsertNextCell(2, line);
    }
    
    // 连接最后一个点和第一个点，形成闭合多边形（不管有没有选择完成，都适用）
    vtkIdType line[] = {num-1, 0};
    lines->InsertNextCell(2, line);
    
    vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
    poly->SetPoints(displayPoints);
    poly->SetLines(lines);
    
    vtkPolyDataMapper2D* mapper = vtkPolyDataMapper2D::SafeDownCast(m_polyActor->GetMapper());
    if (mapper) {
        mapper->SetInputData(poly);
        m_polyActor->VisibilityOn();
    }
}

// 结束多边形选择
void MainOpengl::FinishPolySelection()
{
    if (m_polyPoints->GetNumberOfPoints() < 3) return; // 至少3个点
    m_isPolyFinished = true;
    // 刷新显示以闭合多边形
    UpdatePolyDynamicLine(0, 0); 
}

// 辅助函数：移动点云Actor
void MainOpengl::MoveActor(double dx, double dy)
{
    // 实现平移逻辑：将屏幕空间的dx, dy转换为世界空间的位移
    if (m_pcdActor) {
         vtkCamera* camera = m_renderer->GetActiveCamera();
         if (!camera) return;
         
         // 计算缩放比例：将像素距离转换为世界单位距离
         double dist = camera->GetDistance();
         double angle = camera->GetViewAngle();
         double height = m_renderWindow->GetSize()[1]; 
         double scale = 2.0 * dist * tan(angle * 0.5 * 0.0174532925) / height;
         
         double moveX = dx * scale;
         double moveY = dy * scale;
         
         // 获取相机视角方向向量
         double viewFocus[3], viewPos[3], viewUp[3];
         camera->GetFocalPoint(viewFocus);
         camera->GetPosition(viewPos);
         camera->GetViewUp(viewUp);
         
         double dir[3];
         vtkMath::Subtract(viewFocus, viewPos, dir);
         vtkMath::Normalize(dir);
         
         double right[3];
         vtkMath::Cross(dir, viewUp, right);
         vtkMath::Normalize(right);
         
         vtkMath::Cross(right, dir, viewUp);
         vtkMath::Normalize(viewUp);
         
         // 计算在相机平面上的位移向量
         double displacement[3];
         for(int i=0; i<3; i++) {
             displacement[i] = right[i] * moveX + viewUp[i] * moveY;
         }
         
         // 应用位移
         double* pos3d = m_pcdActor->GetPosition();
         m_pcdActor->SetPosition(pos3d[0] + displacement[0], 
                              pos3d[1] + displacement[1], 
                              pos3d[2] + displacement[2]);
    }
}

// ==========================================
// MainOpengl: 裁剪执行核心
// ==========================================

// 执行裁剪操作
void MainOpengl::DoCrop()
{
    if (!m_pointsActor || !m_pointsActor->GetMapper() || !m_pointsActor->GetMapper()->GetInput()) return;
    if (!m_pointDeal) return;

    qDebug() << "Requesting Crop via PointDeal. Mode:" << m_currentCropMode << " Inside:" << m_cropInside;

    // 1. 准备参数
    CropParams params;
    
    // 视口参数
    int* size = m_renderer->GetSize();
    params.screenWidth = (double)size[0];
    params.screenHeight = (double)size[1];
    double aspect = params.screenWidth / params.screenHeight;
    
    // 投影矩阵
    vtkCamera* cam = m_renderer->GetActiveCamera();
    double range[2];
    cam->GetClippingRange(range);
    vtkMatrix4x4* mat = cam->GetCompositeProjectionTransformMatrix(aspect, range[0], range[1]);
    params.compositeMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
    params.compositeMatrix->DeepCopy(mat);

    // 输入点云 (深拷贝以保证线程安全)
    vtkPolyData* input = vtkPolyData::SafeDownCast(m_pcdActor->GetMapper()->GetInput());
    params.inputCloud = vtkSmartPointer<vtkPolyData>::New();
    params.inputCloud->DeepCopy(input);

    // 裁剪设置
    params.mode = (m_currentCropMode == Crop_Rect) ? 1 : (m_currentCropMode == Crop_Poly ? 2 : 0);
    params.cropInside = m_cropInside;

    // Actor位置
    double* actorPos = m_pcdActor->GetPosition();
    params.actorPos[0] = actorPos[0];
    params.actorPos[1] = actorPos[1];
    params.actorPos[2] = actorPos[2];

    // 矩形/多边形参数
    if (m_currentCropMode == Crop_Rect && m_hasRectSelection) {
        params.rectX = m_selectedRect.GetX();
        params.rectY = m_selectedRect.GetY();
        params.rectW = m_selectedRect.GetWidth();
        params.rectH = m_selectedRect.GetHeight();
    } else if (m_currentCropMode == Crop_Poly && m_polyPoints->GetNumberOfPoints() >= 3) {
        for (int i = 0; i < m_polyPoints->GetNumberOfPoints(); i++) {
            double p[3];
            m_polyPoints->GetPoint(i, p);
            params.polyPoints.push_back({p[0], p[1]});
        }
    } else {
        qDebug() << "Invalid crop selection";
        return;
    }

    // 2. 发送请求
    m_pointDeal->requestCrop(params);
}

// 裁剪完成回调
void MainOpengl::onCropFinished(vtkSmartPointer<vtkPolyData> resultCloud)
{
    if (resultCloud && resultCloud->GetNumberOfPoints() > 0) {
        // 重建Mapper
        vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
        glyphFilter->SetInputData(resultCloud);
        glyphFilter->Update();
        
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(glyphFilter->GetOutputPort());
        
        // 保持颜色范围一致
        double bounds[6];
        resultCloud->GetBounds(bounds);
        mapper->SetScalarRange(bounds[4], bounds[5]); 
        
        // 更新Actor的Mapper
        m_pcdActor->SetMapper(mapper);
        if (renderWindow()) renderWindow()->Render();
        
        qDebug() << "Crop finished. New points:" << resultCloud->GetNumberOfPoints();
    } else {
        qDebug() << "Crop result empty!";
    }

    // 3. 裁剪完成后，隐藏选择框并重置选择状态
    m_rectActor->VisibilityOff();
    m_polyActor->VisibilityOff();
    m_hasRectSelection = false;
    m_hasPolySelection = false;
    if (m_polyPoints) m_polyPoints->Reset();
    m_isPolyFinished = false; 
    
    // 强制刷新渲染窗口
    if (renderWindow()) renderWindow()->Render();
}

// 更新Actor矩阵
void MainOpengl::UpdateActorMatrix(vtkMatrix4x4* matrix)
{
    if (!m_actorMatrix) {
        m_actorMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
    }
    m_actorMatrix->DeepCopy(matrix);
}

// ROS 点云更新回调
void MainOpengl::onRosCloudFinished(vtkSmartPointer<vtkPolyData> resultCloud)
{
    if (!resultCloud || resultCloud->GetNumberOfPoints() == 0) return;

    // 重建Mapper (或者更新现有Mapper的Input，视情况而定)
    // 为了颜色显示正确，这里使用 VertexGlyphFilter
    // 注意：如果是高频刷新，频繁 new Filter 可能会有性能开销，可以考虑复用成员变量
    
    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputData(resultCloud);
    glyphFilter->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(glyphFilter->GetOutputPort());

    // 自动调整颜色范围 (如果有点云颜色)
    if (resultCloud->GetPointData()->GetScalars()) {
        // vtkUnsignedCharArray 不需要 SetScalarRange，它直接作为 RGB 颜色使用
        mapper->SetScalarModeToUsePointFieldData();
        mapper->SelectColorArray("Colors");
        // 确保使用直接颜色映射
        mapper->SetColorModeToDirectScalars(); 
    }

    // 更新Actor
    // 确保Actor已被创建（InitVTK中已创建）并添加到渲染器
    if (m_pcdActor) {
        // 检查是否已经添加到渲染器，如果没有则添加
        if (!m_renderer->HasViewProp(m_pcdActor)) {
            m_renderer->AddActor(m_pcdActor);
        }
    }
    
    // 设置Mapper
    m_pcdActor->SetMapper(mapper);
    // 设置点大小为3，方便观察
    m_pcdActor->GetProperty()->SetPointSize(3.0);
    
    // 如果是第一次接收到点云，重置相机以适应点云范围
    // 使用点云的Bounds进行重置，忽略Grid等其他Actor，解决显示太小的问题
    if (m_isFirstCloudReceived) {
        m_renderer->ResetCamera(m_pcdActor->GetBounds());
        m_isFirstCloudReceived = false;
    }

    // 刷新渲染
    if (renderWindow()) renderWindow()->Render();
}

// 错误处理回调
void MainOpengl::onErrorOccurred(QString msg)
{
    qDebug() << "Error during point cloud operation:" << msg;
}

// ==========================================
// MainOpengl: 事件与回调
// ==========================================

// 窗口大小改变事件
void MainOpengl::resizeEvent(QResizeEvent *event)
{
    QVTKOpenGLNativeWidget::resizeEvent(event);
    UpdateCropOverlayPosition();
}

// 更新裁剪功能条位置
void MainOpengl::UpdateCropOverlayPosition()
{
    if (m_cropOverlay && m_cropOverlay->isVisible()) {
        int w = m_cropOverlay->width();
        int h = m_cropOverlay->height();
        // 放置在右上角
        m_cropOverlay->move(this->width() - w, 0);
    }
}

// 盒式裁剪回调（暂未实现）
void MainOpengl::BoxCropCallback(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
{
}

// 移动回调（暂未实现）
void MainOpengl::MoveCallback(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
{
}

// 标签回调（暂未实现）
void MainOpengl::LabelCallback(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
{
}

// 矩形裁剪回调（逻辑已移至Style，此处暂留空）
void MainOpengl::RectCropCallback(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
{
}
