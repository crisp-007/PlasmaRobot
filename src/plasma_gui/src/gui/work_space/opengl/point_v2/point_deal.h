#ifndef POINT_DEAL_H
#define POINT_DEAL_H

#include <QObject>
#include <QThreadPool>
#include <QRunnable>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkMatrix4x4.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <QString>
#include <vector>
#include <cmath>

// 预定义 CropMode 枚举 (需与 MainOpengl 一致)
enum CropMode {
    Crop_None = 0,
    Crop_Rect = 1,
    Crop_Poly = 2,
    Crop_Box = 3
};

struct CropParams {
    vtkPolyData* inputCloud = nullptr;
    vtkMatrix4x4* compositeMatrix = nullptr;
    double screenWidth = 0;
    double screenHeight = 0;
    
    // 裁剪参数
    int mode = 0; // 1=Rect, 2=Poly
    bool cropInside = false;
    
    // 矩形参数
    double rectX = 0, rectY = 0, rectW = 0, rectH = 0;
    
    // 多边形参数
    std::vector<std::pair<double, double>> polyPoints;

    // Actor 位置
    double actorPos[3] = {0.0, 0.0, 0.0};
};

// ==========================================
// 任务类定义
// ==========================================

class CropTask : public QObject, public QRunnable
{
    Q_OBJECT
public:
    CropTask(const CropParams& params, QObject* receiver);
    void run() override;

private:
    CropParams m_params;
    QObject* m_receiver;
};

class RosToVtkTask : public QObject, public QRunnable
{
    Q_OBJECT
public:
    RosToVtkTask(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg, QObject* receiver);
    void run() override;

private:
    sensor_msgs::msg::PointCloud2::ConstSharedPtr m_msg;
    QObject* m_receiver;
};

class CloudRebuildTask : public QObject, public QRunnable
{
    Q_OBJECT
public:
    CloudRebuildTask(const QString &sourceName,
                     vtkSmartPointer<vtkPolyData> inputCloud,
                     const QString &outputDir,
                     QObject *receiver);
    void run() override;

private:
    void reportProgress(int progress);
    void reportLog(const QString &message);

    QString m_sourceName;
    vtkSmartPointer<vtkPolyData> m_inputCloud;
    QString m_outputDir;
    QObject *m_receiver;
};

class PathPlanningTask : public QObject, public QRunnable
{
    Q_OBJECT
public:
    PathPlanningTask(const QString &sourceName,
                     vtkSmartPointer<vtkPolyData> surgicalMesh,
                     vtkSmartPointer<vtkPolyData> referenceMesh,
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
                     double safeMarginTop,
                     QObject *receiver);
    void run() override;

private:
    void reportProgress(int progress);
    void reportLog(const QString &message);

    QString m_sourceName;
    vtkSmartPointer<vtkPolyData> m_surgicalMesh;
    vtkSmartPointer<vtkPolyData> m_referenceMesh;
    int m_sampleCount = 5000;
    QString m_rodType;
    double m_voxelSize = 0.001;
    double m_medialPercentile = 75.0;
    int m_sectionCount = 60;
    double m_sectionHalfWidth = 0.002;
    int m_smoothPointsNum = 120;
    int m_smoothWindow = 7;
    double m_excludeOpeningDistance = 0.003;
    int m_excludeOpeningLayers = 0;
    double m_outletRearExtent = 0.005;
    double m_safetyForwardExtent = 0.008;
    double m_safeMarginBottom = 0.002;
    double m_safeMarginTop = 0.002;
    QObject *m_receiver;
};

class SlicePlanningTask : public QObject, public QRunnable
{
    Q_OBJECT
public:
    SlicePlanningTask(const QString &sourceName,
                      vtkSmartPointer<vtkPolyData> mesh,
                      vtkSmartPointer<vtkPolyData> straightAxis,
                      vtkSmartPointer<vtkPolyData> curvedAxis,
                      double sliceSpacing,
                      double outletRearExtent,
                      double safetyForwardExtent,
                      double safeMarginBottom,
                      double safeMarginTop,
                      double planeScaleRatio,
                      QObject *receiver);
    void run() override;

private:
    void reportProgress(int progress);
    void reportLog(const QString &message);

    QString m_sourceName;
    vtkSmartPointer<vtkPolyData> m_mesh;
    vtkSmartPointer<vtkPolyData> m_straightAxis;
    vtkSmartPointer<vtkPolyData> m_curvedAxis;
    double m_sliceSpacing = 0.005;
    double m_outletRearExtent = 0.005;
    double m_safetyForwardExtent = 0.008;
    double m_safeMarginBottom = 0.002;
    double m_safeMarginTop = 0.002;
    double m_planeScaleRatio = 1.08;
    QObject *m_receiver;
};

class SliceContourTask : public QObject, public QRunnable
{
    Q_OBJECT
public:
    SliceContourTask(const QString &sourceName,
                     vtkSmartPointer<vtkPolyData> surgicalMesh,
                     vtkSmartPointer<vtkPolyData> slicePlanes,
                     vtkSmartPointer<vtkPolyData> firstSlicePlane,
                     QObject *receiver);
    void run() override;

private:
    void reportProgress(int progress);
    void reportLog(const QString &message);

    QString m_sourceName;
    vtkSmartPointer<vtkPolyData> m_surgicalMesh;
    vtkSmartPointer<vtkPolyData> m_slicePlanes;
    vtkSmartPointer<vtkPolyData> m_firstSlicePlane;
    QObject *m_receiver;
};

class ContourFittingTask : public QObject, public QRunnable
{
    Q_OBJECT
public:
    ContourFittingTask(const QString &sourceName,
                       vtkSmartPointer<vtkPolyData> contours,
                       vtkSmartPointer<vtkPolyData> straightAxis,
                       vtkSmartPointer<vtkPolyData> curvedAxis,
                       int fitPointCount,
                       double coverageThreshold,
                       int smoothWindow,
                       double angleBinDeg,
                       bool trimOpenEnds,
                       int endCheckCount,
                       double curvaturePeakRatio,
                       double curvatureRecoverRatio,
                       int maxTrimCount,
                       int minKeepPointCount,
                       QObject *receiver);
    void run() override;

private:
    void reportProgress(int progress);
    void reportLog(const QString &message);

    QString m_sourceName;
    vtkSmartPointer<vtkPolyData> m_contours;
    vtkSmartPointer<vtkPolyData> m_straightAxis;
    vtkSmartPointer<vtkPolyData> m_curvedAxis;
    int m_fitPointCount = 320;
    double m_coverageThreshold = 0.85;
    int m_smoothWindow = 7;
    double m_angleBinDeg = 1.0;
    bool m_trimOpenEnds = true;
    int m_endCheckCount = 500;
    double m_curvaturePeakRatio = 1.0;
    double m_curvatureRecoverRatio = 0.01;
    int m_maxTrimCount = 50;
    int m_minKeepPointCount = 60;
    QObject *m_receiver;
};

class EqualDosePathTask : public QObject, public QRunnable
{
    Q_OBJECT
public:
    EqualDosePathTask(const QString &sourceName,
                      vtkSmartPointer<vtkPolyData> fittedContours,
                      vtkSmartPointer<vtkPolyData> straightAxis,
                      vtkSmartPointer<vtkPolyData> curvedAxis,
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
                      const QString &angleIncreaseDirection,
                      QObject *receiver);
    void run() override;

private:
    void reportProgress(int progress);
    void reportLog(const QString &message);

    QString m_sourceName;
    vtkSmartPointer<vtkPolyData> m_fittedContours;
    vtkSmartPointer<vtkPolyData> m_straightAxis;
    vtkSmartPointer<vtkPolyData> m_curvedAxis;
    double m_nozzleVerticalLength = 0.005;
    double m_sprayRodRadius = 0.002;
    double m_safetyClearance = 0.002;
    int m_closedSamples = 180;
    int m_openSamples = 120;
    double m_connectionRefAngleDeg = 0.0;
    bool m_useTopOpenEndpoint = true;
    QString m_openEndpointMode = QStringLiteral("end");
    QString m_manualRegionRanges;
    double m_manualZeroOffsetDeg = 175.0;
    QString m_angleViewDirection = QStringLiteral("-a");
    QString m_angleIncreaseDirection = QStringLiteral("ccw");
    QObject *m_receiver;
};

class ContinuousPathTask : public QObject, public QRunnable
{
    Q_OBJECT
public:
    ContinuousPathTask(const QString &sourceName,
                       vtkSmartPointer<vtkPolyData> sprayPath,
                       vtkSmartPointer<vtkPolyData> straightAxis,
                       double maxJoint6SweepDeg,
                       double minPointSpacing,
                       double maxTransitionDistance,
                       bool reverseLayerOrder,
                       bool autoReverseOpenLayers,
                       QObject *receiver);
    void run() override;

private:
    void reportProgress(int progress);
    void reportLog(const QString &message);

    QString m_sourceName;
    vtkSmartPointer<vtkPolyData> m_sprayPath;
    vtkSmartPointer<vtkPolyData> m_straightAxis;
    double m_maxJoint6SweepDeg = 180.0;
    double m_minPointSpacing = 0.0001;
    double m_maxTransitionDistance = 0.015;
    bool m_reverseLayerOrder = false;
    bool m_autoReverseOpenLayers = true;
    QObject *m_receiver;
};

class NozzlePoseTask : public QObject, public QRunnable
{
    Q_OBJECT
public:
    NozzlePoseTask(const QString &sourceName,
                   vtkSmartPointer<vtkPolyData> fittedContours,
                   vtkSmartPointer<vtkPolyData> rotationStartMarkers,
                   vtkSmartPointer<vtkPolyData> straightAxis,
                   vtkSmartPointer<vtkPolyData> curvedAxis,
                   bool reverseLayerOrder,
                   double motionTcpToRoundedTipAxial,
                   double entryTipStandoff,
                   QObject *receiver);
    void run() override;

private:
    void reportProgress(int progress);
    void reportLog(const QString &message);

    QString m_sourceName;
    vtkSmartPointer<vtkPolyData> m_fittedContours;
    vtkSmartPointer<vtkPolyData> m_rotationStartMarkers;
    vtkSmartPointer<vtkPolyData> m_straightAxis;
    vtkSmartPointer<vtkPolyData> m_curvedAxis;
    bool m_reverseLayerOrder = false;
    double m_motionTcpToRoundedTipAxial = 0.0;
    double m_entryTipStandoff = 0.030;
    QObject *m_receiver;
};

// ==========================================
// PointDeal 类定义
// ==========================================

class PointDeal : public QObject
{
    Q_OBJECT
public:
    explicit PointDeal(QObject *parent = nullptr);
    ~PointDeal();

    // 外部调用接口
    void requestCrop(const CropParams& params);
    void requestConvertRosToVtk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void requestCloudRebuild(const QString &sourceName,
                             vtkSmartPointer<vtkPolyData> inputCloud,
                             const QString &outputDir);
    void requestPathPlanning(const QString &sourceName,
                             vtkSmartPointer<vtkPolyData> surgicalMesh,
                             vtkSmartPointer<vtkPolyData> referenceMesh,
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
    void requestSlicePlanning(const QString &sourceName,
                              vtkSmartPointer<vtkPolyData> mesh,
                              vtkSmartPointer<vtkPolyData> straightAxis,
                              vtkSmartPointer<vtkPolyData> curvedAxis,
                              double sliceSpacing,
                              double outletRearExtent,
                              double safetyForwardExtent,
                              double safeMarginBottom,
                              double safeMarginTop,
                              double planeScaleRatio);
    void requestSliceContours(const QString &sourceName,
                              vtkSmartPointer<vtkPolyData> surgicalMesh,
                              vtkSmartPointer<vtkPolyData> slicePlanes,
                              vtkSmartPointer<vtkPolyData> firstSlicePlane);
    void requestContourFitting(const QString &sourceName,
                               vtkSmartPointer<vtkPolyData> contours,
                               vtkSmartPointer<vtkPolyData> straightAxis,
                               vtkSmartPointer<vtkPolyData> curvedAxis,
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
    void requestEqualDosePath(const QString &sourceName,
                              vtkSmartPointer<vtkPolyData> fittedContours,
                              vtkSmartPointer<vtkPolyData> straightAxis,
                              vtkSmartPointer<vtkPolyData> curvedAxis,
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
    void requestContinuousPath(const QString &sourceName,
                               vtkSmartPointer<vtkPolyData> sprayPath,
                               vtkSmartPointer<vtkPolyData> straightAxis,
                               double maxJoint6SweepDeg,
                               double minPointSpacing,
                               double maxTransitionDistance,
                               bool reverseLayerOrder,
                               bool autoReverseOpenLayers);
    void requestNozzlePoses(const QString &sourceName,
                            vtkSmartPointer<vtkPolyData> fittedContours,
                            vtkSmartPointer<vtkPolyData> rotationStartMarkers,
                            vtkSmartPointer<vtkPolyData> straightAxis,
                            vtkSmartPointer<vtkPolyData> curvedAxis,
                            bool reverseLayerOrder,
                            double motionTcpToRoundedTipAxial,
                            double entryTipStandoff);

signals:
    // 处理完成信号
    void cropFinished(vtkSmartPointer<vtkPolyData> result);
    void rosCloudFinished(vtkSmartPointer<vtkPolyData> result);
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
    void errorOccurred(QString msg);

private:
    void startRosConversion(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

    QThreadPool m_threadPool;
    bool m_rosConversionInFlight = false;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr m_latestPendingRosCloud;

    // 允许 CropTask 访问私有成员以触发信号（如果需要，或者通过 QMetaObject::invokeMethod）
    friend class CropTask;
    friend class RosToVtkTask;
    friend class CloudRebuildTask;
    friend class PathPlanningTask;
    friend class SlicePlanningTask;
    friend class SliceContourTask;
    friend class ContourFittingTask;
    friend class EqualDosePathTask;
    friend class ContinuousPathTask;
    friend class NozzlePoseTask;
    
    // 内部处理任务完成的槽函数（通过 invokeMethod 调用）
    Q_INVOKABLE void onTaskFinished(vtkSmartPointer<vtkPolyData> result);
    Q_INVOKABLE void onRosTaskFinished(vtkSmartPointer<vtkPolyData> result);
    Q_INVOKABLE void onReconstructionProgress(QString sourceName, int progress);
    Q_INVOKABLE void onReconstructionLog(QString sourceName, QString message);
    Q_INVOKABLE void onReconstructionFinished(QString sourceName,
                                              QString meshFilePath,
                                              bool ok,
                                              QString message);
    Q_INVOKABLE void onPathPlanningProgress(QString sourceName, int progress);
    Q_INVOKABLE void onPathPlanningLog(QString sourceName, QString message);
    Q_INVOKABLE void onPathPlanningFinished(QString sourceName,
                                            vtkSmartPointer<vtkPolyData> sampledCloud,
                                            vtkSmartPointer<vtkPolyData> curvedAxis,
                                            vtkSmartPointer<vtkPolyData> straightAxis,
                                            bool ok,
                                            QString message);
    Q_INVOKABLE void onSlicePlanningProgress(QString sourceName, int progress);
    Q_INVOKABLE void onSlicePlanningLog(QString sourceName, QString message);
    Q_INVOKABLE void onSlicePlanningFinished(QString sourceName,
                                             vtkSmartPointer<vtkPolyData> slicePlanes,
                                             vtkSmartPointer<vtkPolyData> firstSlicePlane,
                                             vtkSmartPointer<vtkPolyData> boundingBox,
                                             bool ok,
                                             QString message);
    Q_INVOKABLE void onSliceContourProgress(QString sourceName, int progress);
    Q_INVOKABLE void onSliceContourLog(QString sourceName, QString message);
    Q_INVOKABLE void onSliceContourFinished(QString sourceName,
                                            vtkSmartPointer<vtkPolyData> contours,
                                            bool ok,
                                            QString message);
    Q_INVOKABLE void onContourFittingProgress(QString sourceName, int progress);
    Q_INVOKABLE void onContourFittingLog(QString sourceName, QString message);
    Q_INVOKABLE void onContourFittingFinished(QString sourceName,
                                              vtkSmartPointer<vtkPolyData> fittedContours,
                                              bool ok,
                                              QString message);
    Q_INVOKABLE void onEqualDosePathProgress(QString sourceName, int progress);
    Q_INVOKABLE void onEqualDosePathLog(QString sourceName, QString message);
    Q_INVOKABLE void onEqualDosePathFinished(QString sourceName,
                                             vtkSmartPointer<vtkPolyData> equalDoseSurface,
                                             vtkSmartPointer<vtkPolyData> sprayPath,
                                             vtkSmartPointer<vtkPolyData> pathConnections,
                                             bool ok,
                                             QString message);
    Q_INVOKABLE void onContinuousPathProgress(QString sourceName, int progress);
    Q_INVOKABLE void onContinuousPathLog(QString sourceName, QString message);
    Q_INVOKABLE void onContinuousPathFinished(QString sourceName,
                                              vtkSmartPointer<vtkPolyData> continuousPath,
                                              vtkSmartPointer<vtkPolyData> transitions,
                                              vtkSmartPointer<vtkPolyData> resetMarkers,
                                              bool ok,
                                              QString message);
    Q_INVOKABLE void onNozzlePoseProgress(QString sourceName, int progress);
    Q_INVOKABLE void onNozzlePoseLog(QString sourceName, QString message);
    Q_INVOKABLE void onNozzlePoseFinished(QString sourceName,
                                          vtkSmartPointer<vtkPolyData> poseSequence,
                                          vtkSmartPointer<vtkPolyData> posePreview,
                                          bool ok,
                                          QString message);
    Q_INVOKABLE void onTaskError(QString msg);
};

#endif // POINT_DEAL_H
