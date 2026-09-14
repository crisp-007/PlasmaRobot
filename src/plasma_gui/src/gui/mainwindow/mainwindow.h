#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QElapsedTimer>
#include <QLabel>
#include <QMovie>
#include <QOpenGLWidget>
#include <QMap>
#include <QStringList>
#include <QDateTime>
#include <memory>

#include "gasprogresswidget.h"
#include "flowdisplaywidget.h"
#include "StepWizardModel.h"
#include "StepWizardView.h"
//opengl窗口
#include "main_gl.h"
#include "processing_time_widget.h"//处理时间
#include "serial.h"//串口
#include "state_btn.h"//状态图标按钮
#include "tip_manager.h"//顶层提示窗
#include "patient.h"//患者信息卡
#include "DbtreeView.h"//DB树控件
#include "logview.h"//日志界面控制器
#include "ros_launch_manager.h"
#include "SystemCheckManager.h"
#include "arm_status_widget.h"   // 机械臂状态面板
#include "arm_config.h"
#include "serial_status_widget.h"
#include "device_health_model.h"
#include "ros_worker.h"          // ROS2 节点 & 线程
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/header.hpp>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class QDialog;
class QComboBox;
class QPlainTextEdit;
class QPushButton;
class QTabWidget;
struct NodeCloudData;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    
    // 获取等离子处理状态
    bool GetPlasmaDealState() const;

private slots:
    void OnSerialPortButtonClicked();
    
    // 等离子控制相关槽函数
    void OnPlasmaEnableToggled(bool enabled);
    void OnGasModeChanged(int index);
    void OnSingleGasChanged(int index);
    void OnArgonRatioChanged(int value);
    void OnHeliumRatioChanged(int value);
    void OnArgonEnableToggled(bool enabled);
    void OnArgonFlowChanged(double value);
    void OnHeliumEnableToggled(bool enabled);
    void OnHeliumFlowChanged(double value);
    void OnBlendGasEnableToggled(bool enabled);
    void OnBlendGasFlowChanged(double value);
    void OnEmergencyStopClicked();
    
    // 压力值更新槽函数
    void OnPressureValuesUpdated(double heliumPressureMpa, double argonPressureMpa);
    void OnSerialConnectionStatusChanged(bool connected);
    
    // 气体当前值更新槽函数
    void OnHeliumCurrentValueChanged(quint16 heValue);
    void OnArgonCurrentValueChanged(quint16 arValue);
    
    // 混合气体流量复选框联动槽函数
    void OnBlendGasFlowCheckBoxToggled(bool checked);
    
    // 点云交互槽函数
    void OnCropToggled(bool checked);
    void OnLabelToggled(bool checked);
    void OnSelectFrameToggled(bool checked);

    // 测试提示框
    void ShowWelcomeTip();

    // ---- StepWizard 系统引导 ----
    void initStepWizard();

    void onCloudCaptureFinished(bool ok);
    void onCloudRebuildFinished(bool ok);

    // 步骤导航信号
signals:
    void StepChanged(int stepIndex);

protected:
    //void showEvent(QShowEvent *event) override;
    bool eventFilter(QObject *obj, QEvent *event) override;

private:
    Ui::MainWindow *ui;
    FlowDisplayWidget *flow_display_widget;
    GasProgressWidget *he_progress_bar;
    GasProgressWidget *ar_progress_bar;
    ProcessingTimeWidget *processing_time_widget;
    Serial *serial_window;
    PatientCard  *patient_card_;   // 患者信息卡（leftPanel 顶部）
    DbtreeView   *dbtree_view_;     // DB树（leftPanel 中部）
    bool if_plasma_running;  // 等离子运行状态标志位
    TipManager *tip_manager;   // 提示框管理器
    
    // 操作步骤当前索引（StepWizard 同步更新）
    int currentStepIndex = 0;

    // StepWizard 系统引导
    StepWizardModel *m_stepWizardModel = nullptr;
    StepWizardView  *m_stepWizardView  = nullptr;

    // ROS Launch 管理 & 设备自检
    RosLaunchManager *m_rosLaunchManager   = nullptr;
    SystemCheckManager *m_systemCheckManager = nullptr;

    // ---- 机械臂状态面板 ----
    ArmStatusWidget *m_armStatusWidget = nullptr;
    ArmConfig m_armConfig;
    int m_armSpraySpeedPercent = 5;
    int m_armApproachSpeedPercent = 10;
    DeviceHealthModel *m_armHealthModel = nullptr;
    QTimer          *m_armHeartbeatTimer = nullptr;
    RosWorker       *m_rosWorker       = nullptr;
    QThread         *m_rosThread       = nullptr;
    std_msgs::msg::Header m_latestCloudHeader;
    sensor_msgs::msg::JointState m_latestArmJointState;
    std_msgs::msg::Header m_pendingCaptureHeader;
    sensor_msgs::msg::JointState m_pendingCaptureJointState;
    bool m_hasPendingCaptureJointState = false;

    // 日志界面控制器
    LogView *m_logView = nullptr;

    void SetupGasStatusDisplay();
    void UpdateGasStatus();
    void SetupDarkTitleBar();
    void SetupProcessingTimeWidget();
    void SetupPlasmaControlConnections();
    void UpdatePlasmaConnectionStatus(bool connected);
    double ConvertHeliumFlowToOutputValue(double flowValue);
    double ConvertArgonFlowToOutputValue(double flowValue);

    // 系统引导初始化
    void initStepWizardConnections();
    void initRosLaunchManager();
    void initSystemCheckManager();

    // 机械臂状态面板初始化
    void initArmStatusWidget();
    void applyArmMotionSpeedLaunchParams();
    void handleArmSpeedSelection(QComboBox *combo,
                                 int selectedSpeed,
                                 int &currentSpeed,
                                 const QString &settingsKey,
                                 const QString &speedName);
    void initRosWorker();
    void initArmMotionExecution();
    void updateRobotExecutionSummary();
    void updateRobotExecutionReadiness();
    void resetRobotExecutionConfirmation();
    void requestCavityRetreat();
    bool robotExecutionReady(QString *reason = nullptr,
                             bool includeOperatorConfirmation = true) const;
    void publishPathForTransform(const std::shared_ptr<NodeCloudData> &data);
    void requestPathExecutorStop(const QString &reason, bool sendDirectStop);

private slots:
    void onRequestSystemCheck();
    void onSystemCheckFinished(bool ok, const QString &message);
    void onRequestCloudCapture();
    void onRequestCloudRebuild(std::shared_ptr<NodeCloudData> cloudData);
    void onRequestCloudRecrop(std::shared_ptr<NodeCloudData> cloudData);
    void onRequestPathPlanning(std::shared_ptr<NodeCloudData> cloudData);
    void onRequestFullTrajectory(std::shared_ptr<NodeCloudData> cloudData);
    void onRequestSlicePlanning(std::shared_ptr<NodeCloudData> cloudData);
    void onRequestSliceContours(std::shared_ptr<NodeCloudData> cloudData);
    void onRequestContourFitting(std::shared_ptr<NodeCloudData> cloudData);
    void onRequestEqualDosePath(std::shared_ptr<NodeCloudData> cloudData);
    void onRequestSurgicalAreaSelection(std::shared_ptr<NodeCloudData> cloudData);
    void onCloudDeleted(std::shared_ptr<NodeCloudData> cloudData);
    void onCloudCropCancelled();
    void onCloudCropFinished();
    void onConfirmCloudCapture();
    void onReconstructionProgress(const QString &sourceName, int progress);
    void onReconstructionLog(const QString &sourceName, const QString &message);
    void onReconstructionFinished(const QString &sourceName,
                                  const QString &meshFilePath,
                                  bool ok,
                                  const QString &message);
    void onPathPlanningProgress(const QString &sourceName, int progress);
    void onPathPlanningLog(const QString &sourceName, const QString &message);
    void onPathPlanningFinished(const QString &sourceName,
                                vtkSmartPointer<vtkPolyData> sampledCloud,
                                vtkSmartPointer<vtkPolyData> curvedAxis,
                                vtkSmartPointer<vtkPolyData> straightAxis,
                                bool ok,
                                const QString &message);
    void onSlicePlanningProgress(const QString &sourceName, int progress);
    void onSlicePlanningLog(const QString &sourceName, const QString &message);
    void onSlicePlanningFinished(const QString &sourceName,
                                 vtkSmartPointer<vtkPolyData> slicePlanes,
                                 vtkSmartPointer<vtkPolyData> firstSlicePlane,
                                 vtkSmartPointer<vtkPolyData> boundingBox,
                                 bool ok,
                                 const QString &message);
    void onSliceContourProgress(const QString &sourceName, int progress);
    void onSliceContourLog(const QString &sourceName, const QString &message);
    void onSliceContourFinished(const QString &sourceName,
                                vtkSmartPointer<vtkPolyData> contours,
                                bool ok,
                                const QString &message);
    void onContourFittingProgress(const QString &sourceName, int progress);
    void onContourFittingLog(const QString &sourceName, const QString &message);
    void onContourFittingFinished(const QString &sourceName,
                                  vtkSmartPointer<vtkPolyData> fittedContours,
                                  bool ok,
                                  const QString &message);
    void onEqualDosePathProgress(const QString &sourceName, int progress);
    void onEqualDosePathLog(const QString &sourceName, const QString &message);
    void onEqualDosePathFinished(const QString &sourceName,
                                 vtkSmartPointer<vtkPolyData> equalDoseSurface,
                                 vtkSmartPointer<vtkPolyData> sprayPath,
                                 vtkSmartPointer<vtkPolyData> pathConnections,
                                 bool ok,
                                 const QString &message);
    void onRequestContinuousPath(std::shared_ptr<NodeCloudData> cloudData);
    void onContinuousPathProgress(const QString &sourceName, int progress);
    void onContinuousPathLog(const QString &sourceName, const QString &message);
    void onContinuousPathFinished(const QString &sourceName,
                                  vtkSmartPointer<vtkPolyData> continuousPath,
                                  vtkSmartPointer<vtkPolyData> transitions,
                                  vtkSmartPointer<vtkPolyData> resetMarkers,
                                  bool ok,
                                  const QString &message);
    void onRequestNozzlePoses(std::shared_ptr<NodeCloudData> cloudData);
    void onRequestPathExecution();
    void onNozzlePoseProgress(const QString &sourceName, int progress);
    void onNozzlePoseLog(const QString &sourceName, const QString &message);
    void onNozzlePoseFinished(const QString &sourceName,
                              vtkSmartPointer<vtkPolyData> poseSequence,
                              vtkSmartPointer<vtkPolyData> posePreview,
                              bool ok,
                              const QString &message);
    void onSurgicalAreaSelectionFinished(std::shared_ptr<NodeCloudData> cloudData, bool ok);

    // ROS 机械臂数据槽
    void onJointStateReceived(const sensor_msgs::msg::JointState::ConstSharedPtr &msg);
    void onArmEmergencyStop();
    void onArmSelfCheckRequest();
    void onLaunchOutput(const QString &name, const QString &line);
    void showStatusDialog(int tabIndex);
    void toggleSerialLogEnabled();
    void clearSerialLog();
    void onSerialDataReceived(const QByteArray &data);
    void onSerialDataSent(const QByteArray &data);
    void onSerialStatisticsUpdated(qint64 receivedBytes, qint64 sentBytes);
    void onSerialErrorOccurred(const QString &message);
    void onSerialDeviceStatusUpdated(bool emergencyStop,
                                     bool plasmaRelay,
                                     bool voltageRelay,
                                     bool heliumFlowRelay,
                                     bool heliumValve,
                                     bool argonFlowRelay,
                                     bool argonValve,
                                     bool controlAuxFan,
                                     bool deviceFan,
                                     bool controlMainFan);
    void onSerialOutputValuesUpdated(quint16 voltageOutput,
                                     quint16 heliumOutput,
                                     quint16 argonOutput);
    void onPlasmaFeedbackUpdated(double feedbackVpp);
    void toggleArmLogEnabled();
    void toggleCameraLogEnabled();
    void onArmHeartbeatCheck();

private:
    void initStatusDialogConnections();
    void ensureStatusDialog();
    void refreshSerialStatusView();
    void appendSerialLog(const QString &line);
    void applyArmHealthState();
    void appendArmLog(const QString &line);
    void updateCameraParamView();
    void appendCameraLog(const QString &line);
    void loadPlasmaToolConfig();
    void showPathParamDialog();
    void updateFullTrajectoryProgress(const QString &sourceName,
                                      int stage, int stageProgress);
    void continueFullTrajectory(std::shared_ptr<NodeCloudData> source,
                                int completedStage);
    void failFullTrajectory(std::shared_ptr<NodeCloudData> source, int failedStage,
                            const QString &message);

    QDialog *m_statusDialog = nullptr;
    QTabWidget *m_statusTabs = nullptr;
    SerialStatusWidget *m_serialStatusWidget = nullptr;
    QPlainTextEdit *m_serialLogView = nullptr;
    QPushButton *m_serialLogToggleButton = nullptr;
    QPushButton *m_serialLogClearButton = nullptr;
    QStringList m_serialLogBuffer;
    QDateTime m_serialLastPacketTime;
    bool m_serialLogEnabled = true;
    bool m_hasLivePressureData = false;
    bool m_controllerStatusReceived = false;
    QDateTime m_controllerLastStatusTime;
    bool m_controllerEmergencyStop = false;
    bool m_controllerPlasmaRelay = false;
    bool m_controllerVoltageRelay = false;
    bool m_controllerHeliumFlowRelay = false;
    bool m_controllerHeliumValve = false;
    bool m_controllerArgonFlowRelay = false;
    bool m_controllerArgonValve = false;
    bool m_controllerAuxFan = false;
    bool m_controllerDeviceFan = false;
    bool m_controllerMainFan = false;
    quint16 m_controllerVoltageOutput = 0;
    quint16 m_controllerHeliumOutput = 0;
    quint16 m_controllerArgonOutput = 0;
    double m_plasmaFeedbackVpp = 0.0;
    bool m_plasmaStartPending = false;
    bool m_plasmaWorkpointQualified = false;
    quint64 m_plasmaStartSequence = 0;
    QCheckBox *m_robotSafetyConfirmation = nullptr;
    QPlainTextEdit *m_armLogView = nullptr;
    QPushButton *m_armLogToggleButton = nullptr;
    QStringList m_armLogBuffer;
    bool m_armLogEnabled = true;
    QPlainTextEdit *m_cameraParamView = nullptr;
    QPlainTextEdit *m_cameraLogView = nullptr;
    QPushButton *m_cameraLogToggleButton = nullptr;
    QStringList m_cameraLogBuffer;
    bool m_cameraLogEnabled = true;
    bool m_cloudCapturePaused = false;
    bool m_cloudCaptureCropped = false;
    bool m_pathParamsConfigured = false;
    int m_pathSampleCount = 5000;
    double m_pathVoxelSize = 0.001;
    double m_pathMedialPercentile = 75.0;
    int m_pathSectionCount = 60;
    double m_pathSectionHalfWidth = 0.002;
    int m_pathSmoothPointsNum = 120;
    int m_pathSmoothWindow = 7;
    double m_pathExcludeOpeningDistance = 0.003;
    int m_pathExcludeOpeningLayers = 0;
    QString m_pathRodType = QStringLiteral("3_4_200_1x10");
    QString m_toolAdapterType = QStringLiteral("3_4");
    int m_toolRodLengthMm = 200;
    int m_toolNozzleWidthMm = 10;
    double m_configuredToolTcpOffset = 0.310;
    double m_toolTcpOffset = 0.310;
    double m_toolSafetyTipOffset = 0.318;
    double m_toolSafetyForwardExtent = 0.008;
    double m_toolOutletRearExtent = 0.005;
    double m_sliceSpacing = 0.010;
    double m_sliceSafeMarginBottom = 0.010;
    double m_sliceSafeMarginTop = 0.005;
    double m_entryTipStandoff = 0.030;
    double m_slicePlaneScaleRatio = 1.08;
    int m_contourFitPointCount = 320;
    double m_contourCoverageThreshold = 0.85;
    int m_contourSmoothWindow = 7;
    double m_contourAngleBinDeg = 1.0;
    bool m_contourTrimOpenEnds = true;
    int m_contourEndCheckCount = 500;
    double m_contourCurvaturePeakRatio = 1.0;
    double m_contourCurvatureRecoverRatio = 0.01;
    int m_contourMaxTrimCount = 50;
    int m_contourMinKeepPointCount = 60;
    double m_equalDoseNozzleLength = 0.010; // 侧喷口沿腔体轴向的总长度
    double m_equalDoseRodRadius = 0.002;
    double m_nozzleSafetyClearance = 0.005;
    int m_equalDoseClosedSamples = 180;
    int m_equalDoseOpenSamples = 120;
    double m_equalDoseConnectionAngleDeg = 0.0;
    bool m_equalDoseUseTopOpenEndpoint = true;
    QString m_equalDoseOpenEndpointMode = QStringLiteral("end");
    QString m_equalDoseRegionRanges = QStringLiteral(
        "357,17.8;17.8,89.3;89.3,113.7;113.7,198.6;198.6,249.7;249.7,357");
    double m_equalDoseZeroOffsetDeg = 175.0;
    QString m_equalDoseViewDirection = QStringLiteral("-a");
    QString m_equalDoseIncreaseDirection = QStringLiteral("ccw");
    double m_continuousMaxJoint6SweepDeg = 180.0;
    double m_continuousMinPointSpacing = 0.0001;
    double m_continuousMaxTransitionDistance = 0.015;
    bool m_continuousReverseLayerOrder = false;
    bool m_continuousAutoReverseOpenLayers = true;
    int m_pathExecutorState = 0;
    QString m_pathExecutorPathId;
    QString m_pathExecutorMessage;
    int m_pathExecutorCurrentIndex = -1;
    int m_pathExecutorTotalPoints = 0;
    bool m_pathExecutorStatusReceived = false;
    bool m_pathExecutorMotionEnabled = false;
    bool m_pathExecutorPathPermitted = false;
    bool m_pathExecutorDriverReady = false;
    bool m_pathExecutorEntryPoseValid = false;
    int m_entryMotionState = 0;
    QString m_entryMotionPathId;
    QString m_entryMotionMessage;
    bool m_entryMotionStatusReceived = false;
    bool m_entryMotionEnabled = false;
    bool m_entryMotionPathPermitted = false;
    bool m_entryMotionPlanAvailable = false;
    int m_entryMotionTrajectoryPoints = 0;
    QPushButton *m_cavityRetreatButton = nullptr;
    std::shared_ptr<NodeCloudData> m_selectedCloudData;
    std::shared_ptr<NodeCloudData> m_recropTarget;
    QMap<QString, std::shared_ptr<NodeCloudData>> m_rebuildSources;
    QMap<QString, std::shared_ptr<NodeCloudData>> m_pathPlanningSources;
    QMap<QString, std::shared_ptr<NodeCloudData>> m_slicePlanningSources;
    QMap<QString, std::shared_ptr<NodeCloudData>> m_sliceContourSources;
    QMap<QString, std::shared_ptr<NodeCloudData>> m_contourFittingSources;
    QMap<QString, std::shared_ptr<NodeCloudData>> m_equalDosePathSources;
    QMap<QString, std::shared_ptr<NodeCloudData>> m_continuousPathSources;
    QMap<QString, std::shared_ptr<NodeCloudData>> m_nozzlePoseSources;
    QMap<QString, int> m_fullTrajectoryStages;
};
#endif // MAINWINDOW_H
