#ifndef MAINOPENGL_H
#define MAINOPENGL_H

#include <QVTKOpenGLNativeWidget.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkPlaneSource.h>
#include <vtkProperty.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkBoxWidget2.h>
#include <vtkBoxRepresentation.h>
#include <vtkPointPicker.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyle.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkVector.h>
#include <vtkRect.h>
#include <vtkActor2D.h>
#include <vtkVertexGlyphFilter.h>
#include <QThread>
#include <QMap>
#include <memory>
#include "crop_overlay.h"

#include "point_deal.h"

class RosWorker; // 前置声明
struct NodeCloudData;   // 前置声明

// 前向声明
class MainOpengl;

// ==========================================
// MainOpengl 类定义
// ==========================================
class MainOpengl : public QVTKOpenGLNativeWidget
{
    Q_OBJECT

public:
    explicit MainOpengl(QWidget *parent = nullptr);
    ~MainOpengl();

    // --- 数据加载与管理 ---
    // 加载PCD点云文件
    void LoadPCDPoint(const QString &filePath);
    // 获取点云Actor
    vtkActor* GetPCDActor() { return m_pointsActor; }
    vtkSmartPointer<vtkPolyData> latestCroppedCloud() const { return m_latestCroppedCloud; }

    // DB树节点切换显示（预加载模式）
    void showPointCloud(std::shared_ptr<NodeCloudData> data);
    void setPointCloudVisible(std::shared_ptr<NodeCloudData> data, bool visible);
    void setCurvedAxisVisible(std::shared_ptr<NodeCloudData> data, bool visible);
    void loadCloudForCropping(std::shared_ptr<NodeCloudData> data);
    void removeCachedCloud(std::shared_ptr<NodeCloudData> data);
    void requestCloudRebuild(std::shared_ptr<NodeCloudData> data, const QString &outputDir);
    void requestPathPlanning(std::shared_ptr<NodeCloudData> data,
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
                             double safeMarginTop);
    void showPathPlanningResult(std::shared_ptr<NodeCloudData> data,
                                vtkSmartPointer<vtkPolyData> sampledCloud,
                                vtkSmartPointer<vtkPolyData> curvedAxis,
                                vtkSmartPointer<vtkPolyData> straightAxis);
    void requestSlicePlanning(std::shared_ptr<NodeCloudData> data,
                              double sliceSpacing,
                              double outletRearExtent,
                              double safetyForwardExtent,
                              double safeMarginBottom,
                              double safeMarginTop,
                              double planeScaleRatio);
    void showSlicePlanningResult(std::shared_ptr<NodeCloudData> data,
                                 vtkSmartPointer<vtkPolyData> slicePlanes,
                                 vtkSmartPointer<vtkPolyData> firstSlicePlane,
                                 vtkSmartPointer<vtkPolyData> boundingBox);
    void requestSliceContours(std::shared_ptr<NodeCloudData> data);
    void showSliceContours(std::shared_ptr<NodeCloudData> data,
                           vtkSmartPointer<vtkPolyData> contours);
    void requestContourFitting(std::shared_ptr<NodeCloudData> data,
                               int fitPointCount,
                               double coverageThreshold,
                               int smoothWindow,
                               double angleBinDeg,
                               bool trimOpenEnds,
                               int endCheckCount,
                               double curvaturePeakRatio,
                               double curvatureRecoverRatio,
                               int maxTrimCount,
                               int minKeepPointCount);
    void showFittedContours(std::shared_ptr<NodeCloudData> data,
                            vtkSmartPointer<vtkPolyData> fittedContours);
    void requestEqualDosePath(std::shared_ptr<NodeCloudData> data,
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
                              const QString &angleIncreaseDirection);
    void showEqualDosePathResult(std::shared_ptr<NodeCloudData> data,
                                 vtkSmartPointer<vtkPolyData> equalDoseSurface,
                                 vtkSmartPointer<vtkPolyData> sprayPath,
                                 vtkSmartPointer<vtkPolyData> pathConnections);
    void setEqualDoseSurfaceVisible(std::shared_ptr<NodeCloudData> data, bool visible);
    void setSprayPathVisible(std::shared_ptr<NodeCloudData> data, bool visible);
    void setSliceLayersVisible(std::shared_ptr<NodeCloudData> data, bool visible);
    void setFittedContoursVisible(std::shared_ptr<NodeCloudData> data, bool visible);
    void requestContinuousPath(std::shared_ptr<NodeCloudData> data,
                               double maxJoint6SweepDeg,
                               double minPointSpacing,
                               double maxTransitionDistance,
                               bool reverseLayerOrder,
                               bool autoReverseOpenLayers);
    void showContinuousPathResult(std::shared_ptr<NodeCloudData> data,
                                  vtkSmartPointer<vtkPolyData> continuousPath,
                                  vtkSmartPointer<vtkPolyData> transitions,
                                  vtkSmartPointer<vtkPolyData> resetMarkers);
    void setContinuousPathVisible(std::shared_ptr<NodeCloudData> data, bool visible);
    void requestNozzlePoses(std::shared_ptr<NodeCloudData> data, bool reverseLayerOrder);
    void showNozzlePoseResult(std::shared_ptr<NodeCloudData> data,
                              vtkSmartPointer<vtkPolyData> poseSequence,
                              vtkSmartPointer<vtkPolyData> posePreview);
    void setNozzlePosesVisible(std::shared_ptr<NodeCloudData> data, bool visible);
    void startSurgicalAreaSelection(std::shared_ptr<NodeCloudData> data);

    // --- 可视化设置 ---
    // 设置网格属性：gridSize为网格总边长，step为单个格子边长，gridColor为网格线颜色，centerColor为中心十字线颜色
    void SetGridProperties(double gridSize, double step, const QColor &gridColor, const QColor &centerColor);
    // 设置背景颜色（渐变）
    void SetBackgroundColor(const QColor &topColor, const QColor &bottomColor);

    // --- 裁剪系统 ---
    // 裁剪模式枚举
    enum CropMode {
        Crop_None, // 无裁剪模式
        Crop_Box,  // 盒式裁剪（暂不使用）
        Crop_Rect, // 矩形裁剪
        Crop_Poly  // 多边形裁剪
    };

    // 启用/禁用裁剪功能
    void EnableCrop(bool enable, CropMode mode = Crop_Rect);
    // 执行裁剪操作
    void DoCrop();

    // --- 交互模式控制 ---
    // 启用/禁用移动功能
    void EnableMove(bool enable);
    // 启用/禁用标签功能
    void EnableLabel(bool enable);
    
    // 暂停/恢复 ROS 数据更新
    void SetRosPaused(bool paused);

    // --- 交互辅助函数 (供样式调用) ---
    // 更新矩形选择框可视化
    void UpdateRectSelection(int x1, int y1, int x2, int y2);
    // 开始多边形选择
    void StartPolySelection();
    // 添加多边形顶点
    void AddPolyPoint(int x, int y);
    // 更新多边形动态预览线
    void UpdatePolyDynamicLine(int x, int y); 
    // 结束多边形选择
    void FinishPolySelection(); 
    // 移动点云Actor
    void MoveActor(double dx, double dy);
    void UpdateActorMatrix(vtkMatrix4x4* matrix); // 更新矩阵的接口
    void HideActiveBoundingBox();

signals:
    // 裁剪模式被退出（取消/确认后清理）
    void cropCancelled();
    void cropFinishedForWorkflow();
    void reconstructionProgress(const QString &sourceName, int progress);
    void reconstructionLog(const QString &sourceName, const QString &message);
    void reconstructionFinished(const QString &sourceName,
                                const QString &meshFilePath,
                                bool ok,
                                const QString &message);
    void pathPlanningProgress(const QString &sourceName, int progress);
    void pathPlanningLog(const QString &sourceName, const QString &message);
    void pathPlanningFinished(const QString &sourceName,
                              vtkSmartPointer<vtkPolyData> sampledCloud,
                              vtkSmartPointer<vtkPolyData> curvedAxis,
                              vtkSmartPointer<vtkPolyData> straightAxis,
                              bool ok,
                              const QString &message);
    void slicePlanningProgress(const QString &sourceName, int progress);
    void slicePlanningLog(const QString &sourceName, const QString &message);
    void slicePlanningFinished(const QString &sourceName,
                               vtkSmartPointer<vtkPolyData> slicePlanes,
                               vtkSmartPointer<vtkPolyData> firstSlicePlane,
                               vtkSmartPointer<vtkPolyData> boundingBox,
                               bool ok,
                               const QString &message);
    void sliceContourProgress(const QString &sourceName, int progress);
    void sliceContourLog(const QString &sourceName, const QString &message);
    void sliceContourFinished(const QString &sourceName,
                              vtkSmartPointer<vtkPolyData> contours,
                              bool ok,
                              const QString &message);
    void contourFittingProgress(const QString &sourceName, int progress);
    void contourFittingLog(const QString &sourceName, const QString &message);
    void contourFittingFinished(const QString &sourceName,
                                vtkSmartPointer<vtkPolyData> fittedContours,
                                bool ok,
                                const QString &message);
    void equalDosePathProgress(const QString &sourceName, int progress);
    void equalDosePathLog(const QString &sourceName, const QString &message);
    void equalDosePathFinished(const QString &sourceName,
                               vtkSmartPointer<vtkPolyData> equalDoseSurface,
                               vtkSmartPointer<vtkPolyData> sprayPath,
                               vtkSmartPointer<vtkPolyData> pathConnections,
                               bool ok,
                               const QString &message);
    void continuousPathProgress(const QString &sourceName, int progress);
    void continuousPathLog(const QString &sourceName, const QString &message);
    void continuousPathFinished(const QString &sourceName,
                                vtkSmartPointer<vtkPolyData> continuousPath,
                                vtkSmartPointer<vtkPolyData> transitions,
                                vtkSmartPointer<vtkPolyData> resetMarkers,
                                bool ok,
                                const QString &message);
    void nozzlePoseProgress(const QString &sourceName, int progress);
    void nozzlePoseLog(const QString &sourceName, const QString &message);
    void nozzlePoseFinished(const QString &sourceName,
                            vtkSmartPointer<vtkPolyData> poseSequence,
                            vtkSmartPointer<vtkPolyData> posePreview,
                            bool ok,
                            const QString &message);
    void surgicalAreaSelectionFinished(std::shared_ptr<NodeCloudData> data, bool ok);

protected:
    // --- 事件处理 ---
    void resizeEvent(QResizeEvent *event) override;

private slots:
    // 处理裁剪完成信号
    void onCropFinished(vtkSmartPointer<vtkPolyData> resultCloud);
    void onRosCloudFinished(vtkSmartPointer<vtkPolyData> resultCloud);
    // 处理错误信号
    void onErrorOccurred(const QString &msg);

private:
    // --- 初始化与内部设置 ---
    void InitVTK();
    void SetupGrid();
    void UpdateCropOverlayPosition();

    // --- 静态回调函数 ---
    static void BoxCropCallback(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData);
    static void MoveCallback(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData);
    static void LabelCallback(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData);
    static void RectCropCallback(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData);

    // --- 成员变量: 点云处理 ---
    PointDeal* m_pointDeal; // 点云处理管理器
    vtkSmartPointer<vtkPolyData> m_latestCroppedCloud;

    // --- 成员变量: VTK核心 ---
    vtkSmartPointer<vtkRenderer> m_renderer;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderWindow; // 通常通过GetRenderWindow获取，但也可能保留引用
    
    // --- 成员变量: 场景对象 ---
    vtkSmartPointer<vtkActor> m_gridActor;
    vtkSmartPointer<vtkPlaneSource> m_gridPlane;
    vtkSmartPointer<vtkActor> m_pointsActor; // 点云Actor
    vtkSmartPointer<vtkOrientationMarkerWidget> m_axesWidget; // 坐标轴挂件

    // --- 成员变量: 交互部件 ---
    vtkSmartPointer<vtkBoxWidget2> m_cropWidget;
    vtkSmartPointer<vtkBoxWidget2> m_moveWidget;
    vtkSmartPointer<vtkCallbackCommand> m_labelCallback;

    // --- 成员变量: 裁剪控制 ---
    CropMode m_currentCropMode;
    vtkSmartPointer<vtkInteractorObserver> m_defaultStyle; // 保存默认交互样式
    CropOverlayWidget *m_cropOverlay; // 裁剪悬浮工具条
    bool m_cropInside = true; // true: 保留内部, false: 保留外部
    
    // --- 成员变量: 状态控制 ---
    bool m_isFirstRosFrame = true;

    // --- 成员变量: 矩形裁剪可视化 ---
    vtkSmartPointer<vtkActor2D> m_rectActor;
    int m_rectSelection[4];
    bool m_hasRectSelection = false;

    // --- 成员变量: 多边形裁剪可视化 ---
    vtkSmartPointer<vtkActor2D> m_polyLineActor;
    vtkSmartPointer<vtkPoints> m_polyPoints;          // 屏幕坐标顶点
    vtkSmartPointer<vtkPoints> m_polyWorldPoints;     // 世界坐标顶点（在 FinishPolySelection 时转换）
    bool m_hasPolySelection = false;
    bool m_isPolyFinished = false; // 标记多边形绘制是否完成
    vtkSmartPointer<vtkCallbackCommand> m_polyCropObserver; // 用于监听绘制结束

    // --- 相机/Actor变换矩阵 ---
    vtkSmartPointer<vtkMatrix4x4> m_actorMatrix; // 存储Actor当前的变换矩阵
    

    // 辅助函数：设置相机 (基于 RealSense Viewer)
    void SetCamera();

    // --- Ros Worker ---
    RosWorker* m_rosWorker = nullptr;
    QThread* m_workerThread = nullptr;

    // --- 渲染对象复用 (V2 优化) ---
    vtkSmartPointer<vtkVertexGlyphFilter> m_glyphFilter;
    vtkSmartPointer<vtkPolyDataMapper> m_cloudMapper;

    // --- 内部状态 ---
    bool m_isFirstCloudReceived = true; // 标志位：是否是第一次接收到ROS点云

    // --- 渲染对象缓存 (DB树预加载) ---
    struct CloudRenderData {
        vtkSmartPointer<vtkActor> cloudActor;
        vtkSmartPointer<vtkActor> bboxActor;
        vtkSmartPointer<vtkActor> surgicalMeshActor;
        vtkSmartPointer<vtkActor> curvedAxisActor;
        vtkSmartPointer<vtkActor> straightAxisActor;
        vtkSmartPointer<vtkActor> slicePlanesActor;
        vtkSmartPointer<vtkActor> firstSlicePlaneActor;
        vtkSmartPointer<vtkActor> sliceBoundingBoxActor;
        vtkSmartPointer<vtkActor> sliceContoursActor;
        vtkSmartPointer<vtkActor> fittedSliceContoursActor;
        vtkSmartPointer<vtkActor> rotationStartMarkersActor;
        vtkSmartPointer<vtkActor> equalDoseSurfaceActor;
        vtkSmartPointer<vtkActor> sprayPathActor;
        vtkSmartPointer<vtkActor> sprayPathConnectionsActor;
        vtkSmartPointer<vtkActor> continuousSprayPathActor;
        vtkSmartPointer<vtkActor> continuousPathTransitionsActor;
        vtkSmartPointer<vtkActor> joint6ResetMarkersActor;
        vtkSmartPointer<vtkActor> nozzlePosePreviewActor;
    };
    QMap<QString, CloudRenderData> m_cloudCache;   // key = 节点名
    QString m_activeCloudNode;                      // 当前显示的节点名

    vtkSmartPointer<vtkActor> createBboxActor(const double min[3], const double max[3]);
    vtkSmartPointer<vtkActor> createLineActor(vtkPolyData *polyData, const QColor &color, double lineWidth);
    vtkSmartPointer<vtkActor> createSurfaceActor(vtkPolyData *polyData, const QColor &color, double opacity);
    vtkSmartPointer<vtkActor> createColoredSurfaceActor(vtkPolyData *polyData, double opacity);
    vtkSmartPointer<vtkActor> createColoredLineActor(vtkPolyData *polyData, double lineWidth);
    vtkSmartPointer<vtkActor> createColoredPointActor(vtkPolyData *polyData, double pointSize);
    vtkSmartPointer<vtkActor> createNozzlePoseActor(vtkPolyData *polyData);
    vtkSmartPointer<vtkPolyData> buildRotationStartMarkers(
        vtkPolyData *trajectories,
        vtkPolyData *straightAxis,
        vtkPolyData *curvedAxis) const;
    vtkSmartPointer<vtkPolyData> loadMeshPolyData(std::shared_ptr<NodeCloudData> data);
    vtkSmartPointer<vtkPolyData> cropPolyDataWithCurrentSelection(vtkPolyData *input);
    vtkSmartPointer<vtkActor> createSurgicalMeshActor(vtkPolyData *polyData, double opacity = 0.92);
    void updateSurgicalOverlay(std::shared_ptr<NodeCloudData> data);
    void clearCloudActors();
    QString cloudKey(std::shared_ptr<NodeCloudData> data) const;
    void ensureCloudRenderData(std::shared_ptr<NodeCloudData> data);
    std::shared_ptr<NodeCloudData> m_surgicalAreaTarget;
};

// ==========================================
// 自定义交互样式定义
// ==========================================

// 1. 多边形绘制样式 (左键点击添加点，右键结束/平移)
class PolygonDrawStyle : public vtkInteractorStyleTrackballCamera {
public:
    static PolygonDrawStyle* New();
    vtkTypeMacro(PolygonDrawStyle, vtkInteractorStyleTrackballCamera);
    
    void OnLeftButtonDown() override;
    void OnLeftButtonUp() override;
    void OnMouseMove() override;
    void OnRightButtonDown() override;
    void OnRightButtonUp() override;
    
    void SetMainOpengl(MainOpengl* gl) { m_gl = gl; }

private:
    bool m_isDrawing = false;
    bool m_isMoving = false;
    MainOpengl* m_gl = nullptr;
};

// 3. 矩形绘制样式 (左键拖动绘制)
class RectDrawStyle : public vtkInteractorStyleTrackballCamera {
public:
    static RectDrawStyle* New();
    vtkTypeMacro(RectDrawStyle, vtkInteractorStyleTrackballCamera);
    
    void OnLeftButtonDown() override;
    void OnLeftButtonUp() override;
    void OnMouseMove() override;
    
    // 右键平移支持
    void OnRightButtonDown() override;
    void OnRightButtonUp() override;
    
    void SetMainOpengl(MainOpengl* gl) { m_gl = gl; }

private:
    bool m_isDrawing = false;
    bool m_isMoving = false;
    MainOpengl* m_gl = nullptr;
    int m_startPos[2];
    int m_lastPos[2];
};

#endif // MAINOPENGL_H
