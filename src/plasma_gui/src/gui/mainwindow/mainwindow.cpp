#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "state_btn.h"
#include "logmanager.h"
#include "logview.h"
#include "NodeCloudData.h"
#include "../serial/plasma_controller_protocol.h"
#include <plasma_robot_interfaces/msg/spray_path.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <QPainter>
#include <QPixmap>
#include <QTransform>
#include <QGraphicsDropShadowEffect>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QTimer>
#include <QOperatingSystemVersion>
#include <QShowEvent>
#include <QEvent>
#include <QApplication>
#include <QDateTime>
#include <chrono>
#include <QFileDialog>
#include <QFileInfo>
#include <QDir>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <QComboBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QSet>
#include <QSettings>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTabWidget>
#include <QThread>
#include <vtkDoubleArray.h>
#include <vtkDataArray.h>
#include <vtkFieldData.h>
#include <vtkIntArray.h>
#include <vtkPLYWriter.h>
#include <vtkPointData.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#ifdef Q_OS_WIN
#include <windows.h>
#include <dwmapi.h>
#endif

namespace {

enum FullTrajectoryStage
{
    FullSlicePlanning = 0,
    FullSliceContours,
    FullContourFitting,
    FullSprayGeometry,
    FullContinuousPath,
    FullNozzlePoses,
    FullTrajectoryStageCount
};

constexpr double kToolBaseToSprayRodMeters = 0.118;
constexpr double kTipToOutletNearEdgeMeters = 0.003;
constexpr double kActiveShaftRadiusMeters = 0.002;
constexpr qint64 kArmHeartbeatTimeoutMs = 1500;
constexpr qint64 kControllerStatusTimeoutMs = 1500;
constexpr double kPlasmaTargetVppKv = 7.5;
constexpr double kPlasmaTargetToleranceVppKv = 0.2;
constexpr double kPlasmaOverVoltageTripVppKv = 8.8;

bool isActiveDemoTool(int rodLengthMm,
                      int nozzleWidthMm)
{
    return rodLengthMm == 200 && nozzleWidthMm == 10;
}

double nominalSafetyTipOffsetMeters(int rodLengthMm)
{
    return kToolBaseToSprayRodMeters + rodLengthMm / 1000.0;
}

double nominalSafetyForwardExtentMeters(int nozzleWidthMm)
{
    return kTipToOutletNearEdgeMeters + nozzleWidthMm / 2000.0;
}

double nominalMotionTcpOffsetMeters(int rodLengthMm, int nozzleWidthMm)
{
    return nominalSafetyTipOffsetMeters(rodLengthMm) -
        nominalSafetyForwardExtentMeters(nozzleWidthMm);
}

QString plasmaToolVariant(const QString &adapterType,
                          int rodLengthMm,
                          int nozzleWidthMm)
{
    return QStringLiteral("%1_%2_1x%3")
        .arg(adapterType)
        .arg(rodLengthMm)
        .arg(nozzleWidthMm);
}

QString serialPayloadPreview(const QByteArray &data)
{
    constexpr int kPreviewBytes = 24;
    QString preview = QString::fromLatin1(data.left(kPreviewBytes).toHex(' ').toUpper());
    if (data.size() > kPreviewBytes)
        preview += QStringLiteral(" ...");
    return preview;
}

} // namespace

// =========初始化与生命周期===========
/** 主窗口构造：初始化UI、子控件、信号连接与提示 */
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), 
    ui(new Ui::MainWindow), 
    flow_display_widget(nullptr), 
    he_progress_bar(nullptr), 
    ar_progress_bar(nullptr), 
    processing_time_widget(nullptr), 
    serial_window(nullptr),
    if_plasma_running(false), 
    tip_manager(nullptr),
    patient_card_(nullptr),
    dbtree_view_(nullptr)
{
    ui->setupUi(this);

    //全屏显示
    //showFullScreen();
    // 启动时最大化显示
    showMaximized();

    //1、系统准备
    //1-1、暗色标题栏将在窗口显示后设置

    //1-2、初始化日志界面控制器，接管 ui->log (QPlainTextEdit)
    m_logView = new LogView(ui->log, this);
    connect(&LogManager::instance(),
            &LogManager::logReceived,
            m_logView,
            &LogView::appendLog,
            Qt::QueuedConnection);

    LOG_INFO("System", "系统启动，开始进行初始化！");

    loadPlasmaToolConfig();

    

    //1-3、加载状态栏图标（在 C++ 中加载一次，避免样式表每帧重复解析图片）
    ui->SerialStatus->setIconPath(
        QStringLiteral("/home/larusxu/CodeSpace/PlasmaRobot/src/plasma_gui/src/gui/ui_source/plasma/plasma-26.png"));
    ui->RobotArm->setIconPath(
        QStringLiteral("/home/larusxu/CodeSpace/PlasmaRobot/src/plasma_gui/src/gui/ui_source/plasma/robotarm1-26.png"));
    ui->CameraStatus->setIconPath(
        QStringLiteral("/home/larusxu/CodeSpace/PlasmaRobot/src/plasma_gui/src/gui/ui_source/plasma/camera-26.png"));

    //所有设备默认为未就绪
    ui->SerialStatus->setStatus(StateBtn::Disconnected);
    ui->RobotArm->setStatus(StateBtn::Disconnected);
    ui->CameraStatus->setStatus(StateBtn::Disconnected);


    // 获取患者信息卡
    patient_card_ = ui->patientCard;

    // 获取 DB 树控件并关联外部属性标签
    dbtree_view_ = ui->dbtreeView;
    if (dbtree_view_) {
        dbtree_view_->setPropLabel(ui->tree_mes);

        // DB树节点点击 → 切换 MainOpengl 显示的点云
        connect(dbtree_view_, &DbtreeView::nodeSelected,
                ui->main_gl,   &MainOpengl::showPointCloud);
        connect(dbtree_view_, &DbtreeView::nodeSelected,
                this, [this](const std::shared_ptr<NodeCloudData> &data) {
            if (m_selectedCloudData != data)
                resetRobotExecutionConfirmation();
            m_selectedCloudData = data;
            updateRobotExecutionSummary();
        });
        connect(dbtree_view_, &DbtreeView::cloudVisibilityChanged,
                ui->main_gl,   &MainOpengl::setPointCloudVisible);
        connect(dbtree_view_, &DbtreeView::curvedAxisVisibilityChanged,
                ui->main_gl,   &MainOpengl::setCurvedAxisVisible);
        connect(dbtree_view_, &DbtreeView::rebuildRequested,
                this, &MainWindow::onRequestCloudRebuild);
        connect(dbtree_view_, &DbtreeView::recropRequested,
                this, &MainWindow::onRequestCloudRecrop);
        connect(dbtree_view_, &DbtreeView::pathPlanningRequested,
                this, &MainWindow::onRequestPathPlanning);
        connect(dbtree_view_, &DbtreeView::fullTrajectoryRequested,
                this, &MainWindow::onRequestFullTrajectory);
        connect(dbtree_view_, &DbtreeView::slicePlanningRequested,
                this, &MainWindow::onRequestSlicePlanning);
        connect(dbtree_view_, &DbtreeView::sliceContourRequested,
                this, &MainWindow::onRequestSliceContours);
        connect(dbtree_view_, &DbtreeView::contourFittingRequested,
                this, &MainWindow::onRequestContourFitting);
        connect(dbtree_view_, &DbtreeView::equalDosePlanningRequested,
                this, &MainWindow::onRequestEqualDosePath);
        connect(dbtree_view_, &DbtreeView::continuousPathRequested,
                this, &MainWindow::onRequestContinuousPath);
        connect(dbtree_view_, &DbtreeView::nozzlePoseRequested,
                this, &MainWindow::onRequestNozzlePoses);
        connect(dbtree_view_, &DbtreeView::equalDoseSurfaceVisibilityChanged,
                ui->main_gl, &MainOpengl::setEqualDoseSurfaceVisible);
        connect(dbtree_view_, &DbtreeView::sprayPathVisibilityChanged,
                ui->main_gl, &MainOpengl::setSprayPathVisible);
        connect(dbtree_view_, &DbtreeView::sliceLayersVisibilityChanged,
                ui->main_gl, &MainOpengl::setSliceLayersVisible);
        connect(dbtree_view_, &DbtreeView::fittedContoursVisibilityChanged,
                ui->main_gl, &MainOpengl::setFittedContoursVisible);
        connect(dbtree_view_, &DbtreeView::continuousPathVisibilityChanged,
                ui->main_gl, &MainOpengl::setContinuousPathVisible);
        connect(dbtree_view_, &DbtreeView::nozzlePoseVisibilityChanged,
                ui->main_gl, &MainOpengl::setNozzlePosesVisible);
        connect(dbtree_view_, &DbtreeView::surgicalAreaRequested,
                this, &MainWindow::onRequestSurgicalAreaSelection);
        connect(dbtree_view_, &DbtreeView::cloudDeleted,
                this, &MainWindow::onCloudDeleted);

    }


    // 连接点云交互按钮
    ui->btnCrop->setEnabled(false);
    connect(ui->btnCrop, &QPushButton::toggled, this, &MainWindow::OnCropToggled);
    // 裁剪弹窗关闭时取消 btnCrop 的 checked 状态
    connect(ui->main_gl, &MainOpengl::cropCancelled, this, [this]() {
        onCloudCropCancelled();
    });
    connect(ui->main_gl, &MainOpengl::cropFinishedForWorkflow,
            this, &MainWindow::onCloudCropFinished);
    connect(ui->main_gl, &MainOpengl::reconstructionProgress,
            this, &MainWindow::onReconstructionProgress);
    connect(ui->main_gl, &MainOpengl::reconstructionLog,
            this, &MainWindow::onReconstructionLog);
    connect(ui->main_gl, &MainOpengl::reconstructionFinished,
            this, &MainWindow::onReconstructionFinished);
    connect(ui->main_gl, &MainOpengl::pathPlanningProgress,
            this, &MainWindow::onPathPlanningProgress);
    connect(ui->main_gl, &MainOpengl::pathPlanningLog,
            this, &MainWindow::onPathPlanningLog);
    connect(ui->main_gl, &MainOpengl::pathPlanningFinished,
            this, &MainWindow::onPathPlanningFinished);
    connect(ui->main_gl, &MainOpengl::slicePlanningProgress,
            this, &MainWindow::onSlicePlanningProgress);
    connect(ui->main_gl, &MainOpengl::slicePlanningLog,
            this, &MainWindow::onSlicePlanningLog);
    connect(ui->main_gl, &MainOpengl::slicePlanningFinished,
            this, &MainWindow::onSlicePlanningFinished);
    connect(ui->main_gl, &MainOpengl::sliceContourProgress,
            this, &MainWindow::onSliceContourProgress);
    connect(ui->main_gl, &MainOpengl::sliceContourLog,
            this, &MainWindow::onSliceContourLog);
    connect(ui->main_gl, &MainOpengl::sliceContourFinished,
            this, &MainWindow::onSliceContourFinished);
    connect(ui->main_gl, &MainOpengl::contourFittingProgress,
            this, &MainWindow::onContourFittingProgress);
    connect(ui->main_gl, &MainOpengl::contourFittingLog,
            this, &MainWindow::onContourFittingLog);
    connect(ui->main_gl, &MainOpengl::contourFittingFinished,
            this, &MainWindow::onContourFittingFinished);
    connect(ui->main_gl, &MainOpengl::equalDosePathProgress,
            this, &MainWindow::onEqualDosePathProgress);
    connect(ui->main_gl, &MainOpengl::equalDosePathLog,
            this, &MainWindow::onEqualDosePathLog);
    connect(ui->main_gl, &MainOpengl::equalDosePathFinished,
            this, &MainWindow::onEqualDosePathFinished);
    connect(ui->main_gl, &MainOpengl::continuousPathProgress,
            this, &MainWindow::onContinuousPathProgress);
    connect(ui->main_gl, &MainOpengl::continuousPathLog,
            this, &MainWindow::onContinuousPathLog);
    connect(ui->main_gl, &MainOpengl::continuousPathFinished,
            this, &MainWindow::onContinuousPathFinished);
    connect(ui->main_gl, &MainOpengl::nozzlePoseProgress,
            this, &MainWindow::onNozzlePoseProgress);
    connect(ui->main_gl, &MainOpengl::nozzlePoseLog,
            this, &MainWindow::onNozzlePoseLog);
    connect(ui->main_gl, &MainOpengl::nozzlePoseFinished,
            this, &MainWindow::onNozzlePoseFinished);
    connect(ui->main_gl, &MainOpengl::surgicalAreaSelectionFinished,
            this, &MainWindow::onSurgicalAreaSelectionFinished);
    //标注按钮:点击切换"标注模式"
    connect(ui->btnLabel, &QPushButton::toggled, this, &MainWindow::OnLabelToggled);
    // 选帧按钮:点击切换"暂停/恢复实时画面"
    connect(ui->btnSelectFrame, &QPushButton::toggled, this, &MainWindow::OnSelectFrameToggled);


    // 初始化提示框管理器
    tip_manager = new TipManager(this);
    
    //延迟显示欢迎提示框
    QTimer::singleShot(1600, this, &MainWindow::ShowWelcomeTip);


    //2、模块初始化
    //2-1、初始化串口窗口（作为独立窗口）
    serial_window = new Serial(this);
    //2-2、等离子处理时间
    SetupProcessingTimeWidget();
    //2-3、气体状态卡片（氦气、氩气）
    SetupGasStatusDisplay();
    //2-4、状态按钮信号连接
    initStatusDialogConnections();
    //连接串口状态改变信号
    connect(serial_window, &Serial::connectionStatusChanged, this, &MainWindow::OnSerialConnectionStatusChanged);
    connect(serial_window, &Serial::dataReceived,
            this, &MainWindow::onSerialDataReceived);
    connect(serial_window, &Serial::dataSent,
            this, &MainWindow::onSerialDataSent);
    connect(serial_window, &Serial::statisticsUpdated,
            this, &MainWindow::onSerialStatisticsUpdated);
    connect(serial_window, &Serial::serialErrorOccurred,
            this, &MainWindow::onSerialErrorOccurred);
    connect(serial_window, &Serial::settingsChanged,
            this, &MainWindow::refreshSerialStatusView);
    connect(serial_window, &Serial::deviceStatusUpdated,
            this, &MainWindow::onSerialDeviceStatusUpdated);
    connect(serial_window, &Serial::outputValuesUpdated,
            this, &MainWindow::onSerialOutputValuesUpdated);
    connect(serial_window, &Serial::plasmaFeedbackUpdated,
            this, &MainWindow::onPlasmaFeedbackUpdated);
    appendSerialLog(QStringLiteral("串口状态监控已就绪。"));

    

    //连接等离子控制相关的信号槽
    SetupPlasmaControlConnections();
    //TODO：自动连接下位机串口
    
    // 启动时切换离线调试模式
    //ui->main_gl->SetRosPaused(false);
    //ui->btnSelectFrame->setChecked(false);

    
    QThread::msleep(1000);
    LOG_SUCCESS("系统初始化完成。");

    // 1-3、初始化 ROS Launch 管理器并注册 launch 包
    initRosLaunchManager();
    // 1-4、初始化机械臂状态面板
    initArmStatusWidget();
    // 1-5、初始化系统自检管理器
    initSystemCheckManager();
    // 1-6、初始化一键式系统引导
    initStepWizard();
    // 1-7、启动 ROS2 订阅
    initRosWorker();

}

/** 析构：关闭并释放子窗口与UI资源 */
MainWindow::~MainWindow()
{
    using Status = plasma_robot_interfaces::msg::PathExecutionStatus;
    if (m_pathExecutorState == Status::STATE_EXECUTING ||
        m_pathExecutorState == Status::STATE_STOPPING) {
        if (m_rosWorker && m_rosWorker->node())
            m_rosWorker->node()->publishMoveStop();
    }

    if (m_rosWorker)
        m_rosWorker->stop();
    if (m_rosThread && m_rosThread->isRunning())
    {
        m_rosThread->quit();
        if (!m_rosThread->wait(5000))
            LOG_ERROR("System", "ROS2 worker 线程未能在 5 秒内退出");
    }

    // 停止并等待所有由 GUI 启动的 ROS2 launch 进程。
    if (m_rosLaunchManager)
    {
        m_rosLaunchManager->stopAllAndWait();
    }

    // 清理资源并释放内存
    if (serial_window)
    {
        serial_window->close();
        delete serial_window;
        serial_window = nullptr;
    }

    delete ui;
}

//=================0-系统级别初始化======================

// //窗口显示事件：首次显示时应用暗色标题栏
// void MainWindow::showEvent(QShowEvent *event)
// {
//     QMainWindow::showEvent(event);
//     // 窗口首次显示时设置暗色标题栏并全屏
//     static bool first_show = true;
//     if (first_show)
//     {
//         showFullScreen();
//         SetupDarkTitleBar();
//         first_show = false;
//     }
// }

// /** 跨平台暗色标题栏:Windows 10+ / Linux(X11/Wayland) */
// void MainWindow::SetupDarkTitleBar()
// {
// #ifdef Q_OS_WIN
//     if (QOperatingSystemVersion::current() >= QOperatingSystemVersion::Windows10)
//     {
//         HWND hwnd = (HWND)winId();
//         BOOL value = TRUE;
//         ::DwmSetWindowAttribute(hwnd, 19, &value, sizeof(value)); // DWMWA_USE_IMMERSIVE_DARK_MODE_BEFORE_20H1
//     }
// #elif defined(Q_OS_LINUX)
//     // 通知窗口管理器使用暗色标题栏装饰
//     // _NET_WM_THEME_VARIANT: GTK 系窗口管理器(Mutter/Marco)识别此 hint
//     // _KDE_NET_WM_THEME_VARIANT: KDE Plasma 识别此 hint
//     if (QWindow *wh = windowHandle()) {
//         wh->setProperty("_NET_WM_THEME_VARIANT", QByteArray("dark"));
//         wh->setProperty("_KDE_NET_WM_THEME_VARIANT", QByteArray("dark"));
//     }
// #endif
// }

// =========右侧面板初始化===========
// =========1-串口连接相关===========
//1-1、点击串口状态按钮，弹出串口设置窗口
void MainWindow::OnSerialPortButtonClicked()
{
    // serial_window 在构造时已创建，直接显示
    serial_window->show();
    serial_window->raise();
    serial_window->activateWindow();
}

void MainWindow::loadPlasmaToolConfig()
{
    try {
        const std::string packageShare =
            ament_index_cpp::get_package_share_directory("plasma_tool_description");
        const std::string configPath =
            packageShare + "/config/tcp_3_4_200_1x10.yaml";
        const YAML::Node root = YAML::LoadFile(configPath);
        const YAML::Node measurement = root["measurement"];
        if (!measurement) {
            throw std::runtime_error("缺少 measurement 末端实测几何");
        }
        const double motionTcpOffset = measurement["flange_to_motion_tcp_m"].as<double>();
        const double safetyTipOffset = measurement["flange_to_tip_m"].as<double>();
        const double safetyForwardExtent =
            measurement["motion_tcp_to_safety_tip_m"].as<double>();
        const double outletLength = measurement["side_outlet_axial_length_m"].as<double>();
        const double shaftRadius = measurement["shaft_outer_radius_m"].as<double>();
        const double outletRearExtent = 0.5 * outletLength;
        if (!std::isfinite(motionTcpOffset) || motionTcpOffset <= 0.0 ||
            !std::isfinite(safetyTipOffset) || safetyTipOffset <= motionTcpOffset ||
            !std::isfinite(safetyForwardExtent) || safetyForwardExtent <= 0.0 ||
            !std::isfinite(outletLength) || outletLength <= 0.0 ||
            !std::isfinite(shaftRadius) || shaftRadius <= 0.0 ||
            std::abs((safetyTipOffset - motionTcpOffset) - safetyForwardExtent) > 1e-6) {
            throw std::runtime_error("measurement 末端几何无效或相互不一致");
        }

        const YAML::Node variant = root["tool_variant"];
        if (variant) {
            if (variant["adapter_type"])
                m_toolAdapterType = QString::fromStdString(
                    variant["adapter_type"].as<std::string>());
            if (variant["spray_rod_type"]) {
                const QString rodType = QString::fromStdString(
                    variant["spray_rod_type"].as<std::string>());
                const QStringList parts = rodType.split(QStringLiteral("_1x"));
                if (parts.size() == 2) {
                    bool lengthOk = false;
                    bool nozzleOk = false;
                    const int rodLength = parts[0].toInt(&lengthOk);
                    const int nozzleWidth = parts[1].toInt(&nozzleOk);
                    if (lengthOk && nozzleOk) {
                        m_toolRodLengthMm = rodLength;
                        m_toolNozzleWidthMm = nozzleWidth;
                    }
                }
            }
        }

        m_configuredToolTcpOffset = motionTcpOffset;
        m_toolTcpOffset = motionTcpOffset;
        m_toolSafetyTipOffset = safetyTipOffset;
        m_toolSafetyForwardExtent = safetyForwardExtent;
        m_toolOutletRearExtent = outletRearExtent;
        m_pathRodType = plasmaToolVariant(
            m_toolAdapterType, m_toolRodLengthMm, m_toolNozzleWidthMm);
        m_sliceSpacing = outletLength;
        m_equalDoseNozzleLength = outletLength;
        m_equalDoseRodRadius = shaftRadius;
        LOG_INFO("RobotArm", QStringLiteral(
            "已加载末端几何：喷口 TCP 轴向=%1 mm，圆头末端=%2 mm，TCP 到圆头轴向=%3 mm，喷口规格=%4 mm，喷杆半径=%5 mm")
            .arg(motionTcpOffset * 1000.0, 0, 'f', 1)
            .arg(safetyTipOffset * 1000.0, 0, 'f', 1)
            .arg(safetyForwardExtent * 1000.0, 0, 'f', 1)
            .arg(outletLength * 1000.0, 0, 'f', 1)
            .arg(shaftRadius * 1000.0, 0, 'f', 1));
    } catch (const std::exception &error) {
        LOG_WARN("RobotArm", QStringLiteral(
            "读取末端 TCP 配置失败，保留内置喷口 TCP 310.0 mm/圆头末端 318.0 mm：%1")
            .arg(QString::fromUtf8(error.what())));
    }
}

void MainWindow::initStatusDialogConnections()
{
    connect(ui->SerialStatus, &StateBtn::clicked, this, [this]() { showStatusDialog(0); });
    connect(ui->RobotArm, &StateBtn::clicked, this, [this]() { showStatusDialog(1); });
    connect(ui->CameraStatus, &StateBtn::clicked, this, [this]() { showStatusDialog(2); });
}

void MainWindow::showStatusDialog(int tabIndex)
{
    ensureStatusDialog();
    refreshSerialStatusView();
    updateCameraParamView();
    if (m_armStatusWidget) {
        m_armStatusWidget->setArmInfo(ui->armModelComboBox->currentText(),
                                      ui->armIpValue->text(),
                                      ui->armPortValue->text());
    }

    if (m_statusTabs && tabIndex >= 0 && tabIndex < m_statusTabs->count())
        m_statusTabs->setCurrentIndex(tabIndex);

    m_statusDialog->show();
    m_statusDialog->raise();
    m_statusDialog->activateWindow();
}

void MainWindow::ensureStatusDialog()
{
    if (m_statusDialog)
        return;

    m_statusDialog = new QDialog(this);
    m_statusDialog->setWindowTitle(QStringLiteral("设备状态"));
    m_statusDialog->resize(820, 680);

    auto *dialogLayout = new QVBoxLayout(m_statusDialog);
    m_statusTabs = new QTabWidget(m_statusDialog);
    dialogLayout->addWidget(m_statusTabs);

    auto *serialTab = new QWidget(m_statusTabs);
    auto *serialLayout = new QVBoxLayout(serialTab);
    serialLayout->setContentsMargins(10, 10, 10, 10);
    serialLayout->setSpacing(8);

    auto *serialStatusTitle = new QLabel(QStringLiteral("串口实时状态"), serialTab);
    m_serialStatusWidget = new SerialStatusWidget(serialTab);

    auto *serialLogHeaderLayout = new QHBoxLayout();
    auto *serialLogTitle = new QLabel(QStringLiteral("串口实时日志"), serialTab);
    auto *serialSettingsButton = new QPushButton(QStringLiteral("串口设置"), serialTab);
    m_serialLogClearButton = new QPushButton(QStringLiteral("清空日志"), serialTab);
    m_serialLogToggleButton = new QPushButton(serialTab);
    connect(serialSettingsButton, &QPushButton::clicked,
            this, &MainWindow::OnSerialPortButtonClicked);
    connect(m_serialLogClearButton, &QPushButton::clicked,
            this, &MainWindow::clearSerialLog);
    connect(m_serialLogToggleButton, &QPushButton::clicked,
            this, &MainWindow::toggleSerialLogEnabled);
    serialLogHeaderLayout->addWidget(serialLogTitle);
    serialLogHeaderLayout->addStretch();
    serialLogHeaderLayout->addWidget(serialSettingsButton);
    serialLogHeaderLayout->addWidget(m_serialLogClearButton);
    serialLogHeaderLayout->addWidget(m_serialLogToggleButton);

    m_serialLogView = new QPlainTextEdit(serialTab);
    m_serialLogView->setReadOnly(true);
    m_serialLogView->setMinimumHeight(140);

    serialLayout->addWidget(serialStatusTitle);
    serialLayout->addWidget(m_serialStatusWidget);
    serialLayout->addLayout(serialLogHeaderLayout);
    serialLayout->addWidget(m_serialLogView, 1);
    m_statusTabs->addTab(serialTab, QStringLiteral("串口"));

    auto *armTab = new QWidget(m_statusTabs);
    auto *armLayout = new QVBoxLayout(armTab);
    armLayout->setContentsMargins(10, 10, 10, 10);
    armLayout->setSpacing(8);

    auto *armStatusTitle = new QLabel(QStringLiteral("机械臂实时状态"), armTab);
    m_armStatusWidget = new ArmStatusWidget(armTab);

    auto *armLogHeaderLayout = new QHBoxLayout();
    auto *armLogTitle = new QLabel(QStringLiteral("机械臂实时日志"), armTab);
    m_armLogToggleButton = new QPushButton(armTab);
    connect(m_armLogToggleButton, &QPushButton::clicked,
            this, &MainWindow::toggleArmLogEnabled);
    armLogHeaderLayout->addWidget(armLogTitle);
    armLogHeaderLayout->addStretch();
    armLogHeaderLayout->addWidget(m_armLogToggleButton);

    m_armLogView = new QPlainTextEdit(armTab);
    m_armLogView->setReadOnly(true);
    m_armLogView->setMinimumHeight(150);

    armLayout->addWidget(armStatusTitle);
    armLayout->addWidget(m_armStatusWidget);
    armLayout->addLayout(armLogHeaderLayout);
    armLayout->addWidget(m_armLogView, 1);
    m_statusTabs->addTab(armTab, QStringLiteral("机械臂"));

    auto *cameraTab = new QWidget(m_statusTabs);
    auto *cameraLayout = new QVBoxLayout(cameraTab);
    cameraLayout->setContentsMargins(10, 10, 10, 10);
    cameraLayout->setSpacing(8);

    auto *paramTitle = new QLabel(QStringLiteral("当前相机启动参数"), cameraTab);
    m_cameraParamView = new QPlainTextEdit(cameraTab);
    m_cameraParamView->setReadOnly(true);
    m_cameraParamView->setMinimumHeight(130);
    m_cameraParamView->setMaximumHeight(180);

    auto *logHeaderLayout = new QHBoxLayout();
    auto *logTitle = new QLabel(QStringLiteral("相机实时日志"), cameraTab);
    m_cameraLogToggleButton = new QPushButton(cameraTab);
    connect(m_cameraLogToggleButton, &QPushButton::clicked,
            this, &MainWindow::toggleCameraLogEnabled);
    logHeaderLayout->addWidget(logTitle);
    logHeaderLayout->addStretch();
    logHeaderLayout->addWidget(m_cameraLogToggleButton);

    m_cameraLogView = new QPlainTextEdit(cameraTab);
    m_cameraLogView->setReadOnly(true);
    m_cameraLogView->setMinimumHeight(220);

    cameraLayout->addWidget(paramTitle);
    cameraLayout->addWidget(m_cameraParamView);
    cameraLayout->addLayout(logHeaderLayout);
    cameraLayout->addWidget(m_cameraLogView, 1);
    m_statusTabs->addTab(cameraTab, QStringLiteral("相机"));

    m_serialLogToggleButton->setText(m_serialLogEnabled
        ? QStringLiteral("停止打印串口日志")
        : QStringLiteral("开启打印串口日志"));
    m_armLogToggleButton->setText(m_armLogEnabled
        ? QStringLiteral("停止打印机械臂日志")
        : QStringLiteral("开启打印机械臂日志"));
    m_cameraLogToggleButton->setText(m_cameraLogEnabled
        ? QStringLiteral("停止打印相机日志")
        : QStringLiteral("开启打印相机日志"));
    updateCameraParamView();
    refreshSerialStatusView();

    for (const QString &line : m_serialLogBuffer)
        m_serialLogView->appendPlainText(line);
    for (const QString &line : m_armLogBuffer)
        m_armLogView->appendPlainText(line);
    for (const QString &line : m_cameraLogBuffer)
        m_cameraLogView->appendPlainText(line);
}

void MainWindow::toggleSerialLogEnabled()
{
    m_serialLogEnabled = !m_serialLogEnabled;
    if (m_serialLogToggleButton) {
        m_serialLogToggleButton->setText(m_serialLogEnabled
            ? QStringLiteral("停止打印串口日志")
            : QStringLiteral("开启打印串口日志"));
    }
}

void MainWindow::clearSerialLog()
{
    m_serialLogBuffer.clear();
    if (m_serialLogView)
        m_serialLogView->clear();
}

void MainWindow::refreshSerialStatusView()
{
    if (!m_serialStatusWidget || !serial_window)
        return;

    m_serialStatusWidget->setConnectionState(serial_window->isConnected());
    m_serialStatusWidget->setSerialSettings(
        serial_window->getCurrentPortName(),
        serial_window->getCurrentBaudRate(),
        serial_window->getCurrentFrameFormat());
    m_serialStatusWidget->setStatistics(
        serial_window->getBytesReceived(),
        serial_window->getBytesSent(),
        m_serialLastPacketTime);
    if (serial_window->isConnected() && m_hasLivePressureData) {
        m_serialStatusWidget->setPressureValues(
            serial_window->getHeliumPressure(),
            serial_window->getArgonPressure());
    } else {
        m_serialStatusWidget->clearPressureValues();
    }
}

void MainWindow::appendSerialLog(const QString &line)
{
    if (!m_serialLogEnabled)
        return;

    const QString stamped = QStringLiteral("[%1] %2")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz")), line);
    m_serialLogBuffer.append(stamped);
    while (m_serialLogBuffer.size() > 1000)
        m_serialLogBuffer.removeFirst();

    if (m_serialLogView)
        m_serialLogView->appendPlainText(stamped);
}

void MainWindow::onSerialDataReceived(const QByteArray &data)
{
    m_serialLastPacketTime = QDateTime::currentDateTime();
    appendSerialLog(QStringLiteral("接收 %1 B | %2")
        .arg(data.size())
        .arg(serialPayloadPreview(data)));
    refreshSerialStatusView();
}

void MainWindow::onSerialDataSent(const QByteArray &data)
{
    appendSerialLog(QStringLiteral("发送 %1 B | %2")
        .arg(data.size())
        .arg(serialPayloadPreview(data)));
}

void MainWindow::onSerialStatisticsUpdated(qint64 receivedBytes, qint64 sentBytes)
{
    if (m_serialStatusWidget) {
        m_serialStatusWidget->setStatistics(
            receivedBytes, sentBytes, m_serialLastPacketTime);
    }
}

void MainWindow::onSerialErrorOccurred(const QString &message)
{
    appendSerialLog(QStringLiteral("错误：%1").arg(message));
    ui->SerialStatus->setStatus(StateBtn::Error, true);
}

void MainWindow::onSerialDeviceStatusUpdated(bool emergencyStop,
                                             bool plasmaRelay,
                                             bool voltageRelay,
                                             bool heliumFlowRelay,
                                             bool heliumValve,
                                             bool argonFlowRelay,
                                             bool argonValve,
                                             bool controlAuxFan,
                                             bool deviceFan,
                                             bool controlMainFan)
{
    const bool previousEmergencyStop = m_controllerEmergencyStop;
    const bool previousPlasmaRelay = m_controllerPlasmaRelay;
    const bool previousVoltageRelay = m_controllerVoltageRelay;
    m_controllerStatusReceived = true;
    m_controllerLastStatusTime = QDateTime::currentDateTime();
    m_controllerEmergencyStop = emergencyStop;
    m_controllerPlasmaRelay = plasmaRelay;
    m_controllerVoltageRelay = voltageRelay;
    m_controllerHeliumFlowRelay = heliumFlowRelay;
    m_controllerHeliumValve = heliumValve;
    m_controllerArgonFlowRelay = argonFlowRelay;
    m_controllerArgonValve = argonValve;
    m_controllerAuxFan = controlAuxFan;
    m_controllerDeviceFan = deviceFan;
    m_controllerMainFan = controlMainFan;

    if ((m_plasmaStartPending || if_plasma_running) &&
        (previousEmergencyStop != emergencyStop ||
         previousPlasmaRelay != plasmaRelay ||
         previousVoltageRelay != voltageRelay)) {
        LOG_WARN(QStringLiteral(
            "点火状态变化：急停=%1，调压器继电器=%2，等离子继电器=%3，Vpp=%4")
            .arg(emergencyStop ? QStringLiteral("触发") : QStringLiteral("正常"))
            .arg(voltageRelay ? QStringLiteral("开") : QStringLiteral("关"))
            .arg(plasmaRelay ? QStringLiteral("开") : QStringLiteral("关"))
            .arg(m_plasmaFeedbackVpp, 0, 'f', 2));
    }

    if (emergencyStop && (if_plasma_running || m_plasmaStartPending)) {
        ++m_plasmaStartSequence;
        m_plasmaStartPending = false;
        m_plasmaWorkpointQualified = false;
        if_plasma_running = false;
        serial_window->setPlasmaControl(false, false, false);
        {
            const QSignalBlocker blocker(ui->plasma_start_btn);
            ui->plasma_start_btn->setChecked(false);
        }
        ui->plasma_start_btn->setText(QStringLiteral("启动等离子"));
        ui->plasma_start_btn->setEnabled(true);
        ui->gasModeComboBox->setEnabled(true);
        ui->singleGasComboBox->setEnabled(true);
        processing_time_widget->StopTiming();
        flow_display_widget->StopAnimation();
        LOG_ERROR("Safety", "控制板急停触发，等离子启动/运行状态已清除");
    }

    if (m_serialStatusWidget) {
        m_serialStatusWidget->setDeviceStatus(
            emergencyStop,
            plasmaRelay,
            voltageRelay,
            heliumFlowRelay,
            heliumValve,
            argonFlowRelay,
            argonValve);
    }
    updateRobotExecutionReadiness();
}

void MainWindow::onSerialOutputValuesUpdated(quint16 voltageOutput,
                                             quint16 heliumOutput,
                                             quint16 argonOutput)
{
    m_controllerVoltageOutput = voltageOutput;
    m_controllerHeliumOutput = heliumOutput;
    m_controllerArgonOutput = argonOutput;
    if (m_serialStatusWidget) {
        m_serialStatusWidget->setOutputValues(
            voltageOutput, heliumOutput, argonOutput);
    }
}

void MainWindow::onPlasmaFeedbackUpdated(double feedbackVpp)
{
    if (!std::isfinite(feedbackVpp) || feedbackVpp < 0.0)
        return;

    m_plasmaFeedbackVpp = feedbackVpp;
    // The controller reports feedback volts and the power-supply divider is
    // calibrated as 1 V feedback = 1 kV high voltage, so the number is equal.
    ui->plasmaVoltageValue->setText(QString::number(feedbackVpp, 'f', 2));

    if (if_plasma_running && !m_plasmaWorkpointQualified &&
        feedbackVpp >= kPlasmaTargetVppKv - kPlasmaTargetToleranceVppKv &&
        feedbackVpp <= kPlasmaTargetVppKv + kPlasmaTargetToleranceVppKv) {
        m_plasmaWorkpointQualified = true;
        LOG_SUCCESS(QStringLiteral(
            "已到保守调试停止点：%1 kV Vpp，目标=%2 kV Vpp；请立即手动停止等离子")
            .arg(feedbackVpp, 0, 'f', 2)
            .arg(kPlasmaTargetVppKv, 0, 'f', 1));
        if (tip_manager) {
            tip_manager->showTip(
                QStringLiteral("已到 7.5 kV Vpp 调试停止点，请立即停止等离子"),
                TipWidget::Warning);
        }
    }

    if ((if_plasma_running || m_plasmaStartPending) &&
        feedbackVpp > kPlasmaOverVoltageTripVppKv) {
        LOG_ERROR("Safety", QStringLiteral(
            "高压反馈超过调试上限：%1 kV Vpp > %2 kV Vpp，正在关断")
            .arg(feedbackVpp, 0, 'f', 2)
            .arg(kPlasmaOverVoltageTripVppKv, 0, 'f', 1));
        m_plasmaStartPending = false;
        m_plasmaWorkpointQualified = false;
        ++m_plasmaStartSequence;
        if_plasma_running = false;
        if (serial_window)
            serial_window->setPlasmaControl(false, false, false);
        {
            const QSignalBlocker blocker(ui->plasma_start_btn);
            ui->plasma_start_btn->setChecked(false);
        }
        ui->plasma_start_btn->setText(QStringLiteral("启动等离子"));
        ui->plasma_start_btn->setEnabled(true);
        ui->gasModeComboBox->setEnabled(true);
        ui->singleGasComboBox->setEnabled(true);
        processing_time_widget->StopTiming();
        flow_display_widget->StopAnimation();
        if (tip_manager) {
            tip_manager->showTip(
                QStringLiteral("高压反馈超过 8.8 kV Vpp，等离子已关闭"),
                TipWidget::Dangerous);
        }
    }
}

void MainWindow::toggleArmLogEnabled()
{
    m_armLogEnabled = !m_armLogEnabled;
    if (m_armLogToggleButton) {
        m_armLogToggleButton->setText(m_armLogEnabled
            ? QStringLiteral("停止打印机械臂日志")
            : QStringLiteral("开启打印机械臂日志"));
    }
}

void MainWindow::toggleCameraLogEnabled()
{
    m_cameraLogEnabled = !m_cameraLogEnabled;
    if (m_cameraLogToggleButton) {
        m_cameraLogToggleButton->setText(m_cameraLogEnabled
            ? QStringLiteral("停止打印相机日志")
            : QStringLiteral("开启打印相机日志"));
    }
}

void MainWindow::updateCameraParamView()
{
    if (!m_cameraParamView)
        return;

    if (!m_rosLaunchManager || !m_rosLaunchManager->m_launches.contains(QStringLiteral("camera")))
    {
        m_cameraParamView->setPlainText(QStringLiteral("相机 launch 尚未注册。"));
        return;
    }

    const LaunchInstance &camera = m_rosLaunchManager->m_launches[QStringLiteral("camera")];
    QStringList lines;
    lines << QStringLiteral("package: %1").arg(camera.package);
    lines << QStringLiteral("launch: %1").arg(camera.launchFile);
    lines << QStringLiteral("command: %1").arg(m_rosLaunchManager->buildCommand(camera));
    lines << QStringLiteral("");
    lines << QStringLiteral("参数:");
    if (camera.params.isEmpty())
    {
        lines << QStringLiteral("  暂无参数");
    }
    else
    {
        for (auto it = camera.params.cbegin(); it != camera.params.cend(); ++it)
            lines << QStringLiteral("  %1 := %2").arg(it.key(), it.value());
    }

    m_cameraParamView->setPlainText(lines.join('\n'));
}

void MainWindow::appendCameraLog(const QString &line)
{
    if (!m_cameraLogEnabled)
        return;

    const QString stamped = QStringLiteral("[%1] %2")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz")), line);
    m_cameraLogBuffer.append(stamped);
    while (m_cameraLogBuffer.size() > 1000)
        m_cameraLogBuffer.removeFirst();

    if (m_cameraLogView)
        m_cameraLogView->appendPlainText(stamped);
}

void MainWindow::appendArmLog(const QString &line)
{
    if (!m_armLogEnabled)
        return;

    const QString stamped = QStringLiteral("[%1] %2")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz")), line);
    m_armLogBuffer.append(stamped);
    while (m_armLogBuffer.size() > 1000)
        m_armLogBuffer.removeFirst();

    if (m_armLogView)
        m_armLogView->appendPlainText(stamped);
}

void MainWindow::onLaunchOutput(const QString &name, const QString &line)
{
    if (name == QStringLiteral("camera"))
        appendCameraLog(line);
}
//2-1、初始化等离子处理计时控件：从UI占位转换或替换为组件实例
void MainWindow::SetupProcessingTimeWidget()
{
    // 直接使用 UI 文件中的 processingTimeWidget
    QWidget *ui_widget = ui->processingTimeWidget;
    if (ui_widget)
    {
        // 将 UI widget 转换为 ProcessingTimeWidget 类型
        processing_time_widget = qobject_cast<ProcessingTimeWidget *>(ui_widget);
        if (!processing_time_widget)
        {
            // 如果转换失败，说明 UI 中的 widget 不是 ProcessingTimeWidget 类型
            // 如果需要创建一个新的ProcessingTimeWidget来替换
            processing_time_widget = new ProcessingTimeWidget(ui_widget->parentWidget());

            // 为新建的 UI widget 设置属性
            processing_time_widget->setObjectName("processingTimeWidget");
            processing_time_widget->setMinimumSize(ui_widget->minimumSize());
            processing_time_widget->setMaximumSize(ui_widget->maximumSize());
            processing_time_widget->setStyleSheet(ui_widget->styleSheet());

            // 获取父布局并替换widget
            QVBoxLayout *parent_layout = qobject_cast<QVBoxLayout *>(ui_widget->parentWidget()->layout());
            if (parent_layout)
            {
                int index = parent_layout->indexOf(ui_widget);
                if (index >= 0)
                {
                    parent_layout->removeWidget(ui_widget);
                    parent_layout->insertWidget(index, processing_time_widget);
                    ui_widget->setParent(nullptr);
                    ui_widget->deleteLater();
                }
            }
        }

        qDebug() << "处理时间窗口设置成功！";
    }
    else
    {
        qDebug() << "在ui文件中未找到processingTimeWidget控件！";
    }
}

// =========2-气体状态卡片===========
/** 初始化气体状态区域：替换占位控件并设置相关控件的初始值 */
void MainWindow::SetupGasStatusDisplay()
{
    //控件：混合模式右侧动画显示+流量文字显示
    flow_display_widget = new FlowDisplayWidget(ui->gasStatusWidget);
    flow_display_widget->SetFlowValue(0.0);

    QVBoxLayout *gasStatusLayout = qobject_cast<QVBoxLayout *>(ui->gasStatusWidget->layout());
    if (gasStatusLayout && gasStatusLayout->count() > 0)
    {
        QLayoutItem *firstItem = gasStatusLayout->itemAt(0);
        if (firstItem && firstItem->layout())
        {
            QHBoxLayout *topLayout = qobject_cast<QHBoxLayout *>(firstItem->layout());
            if (topLayout)
            {
                for (int i = 0; i < topLayout->count(); ++i)
                {
                    QLayoutItem *item = topLayout->itemAt(i);
                    if (item && item->widget() && item->widget()->objectName() == "flowDisplayPlaceholder")
                    {
                        topLayout->replaceWidget(item->widget(), flow_display_widget);
                        item->widget()->setParent(nullptr);
                        break;
                    }
                }
            }
        }
    }
    //控件：气体进度条
    he_progress_bar = new GasProgressWidget(ui->gasStatusWidget);
    he_progress_bar->SetPercentage(0.0);

    ar_progress_bar = new GasProgressWidget(ui->gasStatusWidget);
    ar_progress_bar->SetPercentage(0.0);

    QHBoxLayout *heLayout = qobject_cast<QHBoxLayout *>(ui->he_gas_widget->layout());
    if (heLayout)
    {
        for (int i = 0; i < heLayout->count(); ++i)
        {
            QLayoutItem *item = heLayout->itemAt(i);
            if (item && item->widget() && item->widget()->objectName() == "heProgressWidget")
            {
                heLayout->replaceWidget(item->widget(), he_progress_bar);
                item->widget()->setParent(nullptr);
                break;
            }
        }
    }

    QHBoxLayout *arLayout = qobject_cast<QHBoxLayout *>(ui->ar_gas_widget->layout());
    if (arLayout)
    {
        for (int i = 0; i < arLayout->count(); ++i)
        {
            QLayoutItem *item = arLayout->itemAt(i);
            if (item && item->widget() && item->widget()->objectName() == "arProgressWidget")
            {
                arLayout->replaceWidget(item->widget(), ar_progress_bar);
                item->widget()->setParent(nullptr);
                break;
            }
        }
    }

    ui->gasModeLabel->setText("混合模式");

    ui->he_pressure_lab->setText(QStringLiteral("压力: -- MPa"));
    ui->he_volume_lab->setText(QStringLiteral("余量: -- %"));
    ui->he_flowrate_lab->setText(QStringLiteral("流量: -- L/min"));
    ui->ar_pressure_lab->setText(QStringLiteral("压力: -- MPa"));
    ui->ar_volume_lab->setText(QStringLiteral("余量: -- %"));
    ui->ar_flowrate_lab->setText(QStringLiteral("流量: -- L/min"));
}

// =========3-等离子标签页===========
// =========3-1、等离子相关===========
// 连接等离子控制相关的UI信号与初始化联动逻辑
void MainWindow::SetupPlasmaControlConnections()
{
    // 等离子启动按钮
    connect(ui->plasma_start_btn, &QPushButton::toggled, this, &MainWindow::OnPlasmaEnableToggled);

    // 气体模式控制
    connect(ui->gasModeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::OnGasModeChanged);
    

    // 混合气体流量复选框联动
    connect(ui->blendGasEnableCheckBox, &QCheckBox::toggled, this, &MainWindow::OnBlendGasFlowCheckBoxToggled);

    // 气体比例控制
    connect(ui->argonRatioSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::OnArgonRatioChanged);
    connect(ui->heliumRatioSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::OnHeliumRatioChanged);


    connect(ui->singleGasComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::OnSingleGasChanged);
    // 氩气
    // 氩气滑动条和数值框同步
    connect(ui->argonFlowSlider, &QSlider::valueChanged, this, [this](int value)
            {
        ui->argonFlowSpinBox->blockSignals(true);
        ui->argonFlowSpinBox->setValue(value * 0.01);
        OnArgonFlowChanged(ui->argonFlowSpinBox->value());
        ui->argonFlowSpinBox->blockSignals(false); });
    connect(ui->argonFlowSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value)
            {
        ui->argonFlowSlider->blockSignals(true);
        ui->argonFlowSlider->setValue(static_cast<int>(value*100));
        ui->argonFlowSlider->blockSignals(false); });

    // 氩气->串口更新
    connect(ui->argonEnableCheckBox, &QCheckBox::toggled, this, &MainWindow::OnArgonEnableToggled);
    connect(ui->argonFlowSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::OnArgonFlowChanged);

    // 氦气
    // 氦气滑动条和数值框同步
    connect(ui->heliumFlowSlider, &QSlider::valueChanged, this, [this](int value)
            {
        ui->heliumFlowSpinBox->blockSignals(true);
        ui->heliumFlowSpinBox->setValue(value * 0.01);
        OnHeliumFlowChanged(ui->heliumFlowSpinBox->value());
        ui->heliumFlowSpinBox->blockSignals(false); });
    connect(ui->heliumFlowSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value)
            {
        ui->heliumFlowSlider->blockSignals(true);
        ui->heliumFlowSlider->setValue(static_cast<int>(value * 100));
        ui->heliumFlowSlider->blockSignals(false); });
    // 串口更新
    connect(ui->heliumEnableCheckBox, &QCheckBox::toggled, this, &MainWindow::OnHeliumEnableToggled);
    connect(ui->heliumFlowSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::OnHeliumFlowChanged);

    // 混合气体滑动条和数值框同步
    connect(ui->blendGasFlowSlider, &QSlider::valueChanged, this, [this](int value)
            {
        ui->blendGasFlowSpinBox->blockSignals(true);
        ui->blendGasFlowSpinBox->setValue(value * 0.1);
        ui->blendGasFlowSpinBox->blockSignals(false); });
    connect(ui->blendGasFlowSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value)
            {
        ui->blendGasFlowSlider->blockSignals(true);
        qDebug() << "Blend gas flow spin box value changed to " << value;
        ui->blendGasFlowSlider->setValue(static_cast<int>(value * 10));
        ui->blendGasFlowSlider->blockSignals(false); });
    // 混合气->串口更新
    connect(ui->blendGasEnableCheckBox, &QCheckBox::toggled, this, &MainWindow::OnBlendGasEnableToggled);
    connect(ui->blendGasFlowSlider, &QSlider::valueChanged, this, &MainWindow::OnBlendGasFlowChanged);
    connect(ui->blendGasFlowSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::OnBlendGasFlowChanged);

    // 初始化气体模式
    OnGasModeChanged(ui->gasModeComboBox->currentIndex());

    // 初始化气体比例
    OnArgonRatioChanged(ui->argonRatioSpinBox->value());
    OnSingleGasChanged(ui->singleGasComboBox->currentIndex());

    // 紧急停止按钮
    connect(ui->emergencyStopButton1, &QPushButton::clicked, this, &MainWindow::OnEmergencyStopClicked);

    // 为ComboBox控件安装事件过滤器以禁用滚轮事件
    ui->gasModeComboBox->installEventFilter(this);
    ui->singleGasComboBox->installEventFilter(this);

    // 初始化滑动条和数值框的同步
    ui->argonFlowSlider->setValue(static_cast<int>(ui->argonFlowSpinBox->value() * 100));
    ui->heliumFlowSlider->setValue(static_cast<int>(ui->heliumFlowSpinBox->value() * 100));
    ui->blendGasFlowSlider->setValue(static_cast<int>(ui->blendGasFlowSpinBox->value() * 10));

    // 连接压力值和气体当前流量值更新信号
    if (serial_window) {
        connect(serial_window, &Serial::pressureValuesUpdated, this, &MainWindow::OnPressureValuesUpdated);
        connect(serial_window, &Serial::heliumCurrentValueChanged, this, &MainWindow::OnHeliumCurrentValueChanged);
        connect(serial_window, &Serial::argonCurrentValueChanged, this, &MainWindow::OnArgonCurrentValueChanged);
    }

    // 初始化等离子状态为离线
    UpdatePlasmaConnectionStatus(false);    
}

//启停等离子：校验气体条件，联动串口与UI状态
void MainWindow::OnPlasmaEnableToggled(bool enabled)
{
    // 检测气体流量条件
    if (enabled)
    {
        // 检查氦气和氩气checkbox的选中状态
        bool heliumChecked = ui->heliumEnableCheckBox->isChecked();
        bool argonChecked = ui->argonEnableCheckBox->isChecked();

        // 检查对应slider的值是否大于0
        bool heliumFlowValid = ui->heliumFlowSlider->value() > 0;
        bool argonFlowValid = ui->argonFlowSlider->value() > 0;

        // 在单气体模式下，如果只选中了一个checkbox，检查其对应的slider值
        if (ui->gasModeComboBox->currentIndex() == 0) // 单气体模式
        {

            // 检查氦气和氩气checkbox的选中状态
            bool heliumChecked = ui->heliumEnableCheckBox->isChecked();
            bool argonChecked = ui->argonEnableCheckBox->isChecked();
            // 检查对应slider的值是否大于0
            bool heliumFlowValid = ui->heliumFlowSlider->value() > 0;
            bool argonFlowValid = ui->argonFlowSlider->value() > 0;

            // 根据选择的气体自动设置对应的复选框为选中状态
            if (ui->singleGasComboBox->currentIndex() == 0) // 选择了氩气
            {
                if (!argonFlowValid)
                {
                    // 阻断信号，防止触发其他槽函数
                    ui->plasma_start_btn->blockSignals(true);
                    ui->plasma_start_btn->setChecked(false); // 重置按钮状态
                    ui->plasma_start_btn->blockSignals(false); // 恢复信号
                    // MessageBoxW(NULL, L"请设置氩气流量", L"错误", MB_OK | MB_ICONERROR);
                    if (tip_manager) {
                        tip_manager->showTip("请设置氩气流量！", TipWidget::Warning);
                    }
                    return; // 阻止启动
                }
                ui->argonEnableCheckBox->setChecked(true);
            }
            else if (ui->singleGasComboBox->currentIndex() == 1) // 选择了氦气
            {
                if (!heliumFlowValid)
                {
                    // 阻断信号，防止触发其他槽函数
                    ui->plasma_start_btn->blockSignals(true);
                    ui->plasma_start_btn->setChecked(false); // 重置按钮状态
                    ui->plasma_start_btn->blockSignals(false); // 恢复信号
                    // MessageBoxW(NULL, L"请设置氦气流量", L"错误", MB_OK | MB_ICONERROR);
                    if (tip_manager) {
                        tip_manager->showTip("请设置氦气流量！", TipWidget::Warning);
                    }
                    return; // 阻止启动
                }
                ui->heliumEnableCheckBox->setChecked(true);
            }
            // 如果两个checkbox都没有选中，则无法启动等离子
            // if (!heliumChecked && !argonChecked)
            // {
            //     ui->plasma_start_btn->setChecked(false); // 重置按钮状态
            //     MessageBoxW(NULL, L"请至少选择一个激发气体", L"错误", MB_OK | MB_ICONERROR);
            //     return; // 阻止启动
            // }
        }
        else
        {
            // 混合气体模式
            // 检查混合气体流量滑动条的值是否大于0
            bool blendGasFlowValid = ui->blendGasFlowSlider->value() > 0;
            // 如果混合气体流量为0，则无法启动
            if (!blendGasFlowValid)
            {
                // 阻断信号，防止触发其他槽函数
                ui->plasma_start_btn->blockSignals(true);
                ui->plasma_start_btn->setChecked(false); // 重置按钮状态
                ui->plasma_start_btn->blockSignals(false); // 恢复信号
                // MessageBoxW(NULL, L"请设置混合气体流量", L"错误", MB_OK | MB_ICONERROR);
                if (tip_manager) {
                    tip_manager->showTip("请设置混合气体流量！", TipWidget::Warning);
                }
                return; // 阻止启动
            }

            // 如果选中的checkbox对应的slider值都小于等于0，则无法启动
            // 检查比例值
            int argonRatio = ui->argonRatioSpinBox->value();
            int heliumRatio = ui->heliumRatioSpinBox->value();
            if (argonRatio + heliumRatio != 100)
            {
                // 阻断信号，防止触发其他槽函数
                ui->plasma_start_btn->blockSignals(true);
                ui->plasma_start_btn->setChecked(false); // 重置按钮状态
                ui->plasma_start_btn->blockSignals(false); // 恢复信号
                // MessageBoxW(NULL, L"请设置正确的气体比例", L"错误", MB_OK | MB_ICONERROR);
                if (tip_manager) {
                    tip_manager->showTip("请设置正确的气体比例！", TipWidget::Warning);
                }
                return; // 阻止启动
            }

            // 检查对应slider的值是否大于0
            bool heliumFlowValid = ui->heliumFlowSlider->value() > 0;
            bool argonFlowValid = ui->argonFlowSlider->value() > 0;
            if ((heliumRatio > 0 && !heliumFlowValid) || (argonRatio > 0 && !argonFlowValid))
            {
                // 阻断信号，防止触发其他槽函数
                ui->plasma_start_btn->blockSignals(true);
                ui->plasma_start_btn->setChecked(false); // 重置按钮状态
                ui->plasma_start_btn->blockSignals(false); // 恢复信号
                // MessageBoxW(NULL, L"请设置气体流量", L"错误", MB_OK | MB_ICONERROR);
                if (tip_manager) {
                    tip_manager->showTip("请设置气体流量！", TipWidget::Warning);
                }
                return; // 阻止启动
            }

            ui->blendGasEnableCheckBox->setChecked(true);
        }
    }

    if (enabled)
    {
        const bool controllerFresh = m_controllerStatusReceived &&
            m_controllerLastStatusTime.msecsTo(QDateTime::currentDateTime()) <=
                kControllerStatusTimeoutMs;
        QString blockedReason;
        if (!serial_window || !serial_window->isConnected())
            blockedReason = QStringLiteral("控制板串口尚未连接");
        else if (!controllerFresh)
            blockedReason = QStringLiteral("控制板状态回读已超时");
        else if (m_controllerEmergencyStop)
            blockedReason = QStringLiteral("控制板急停仍处于触发状态");
        else if (m_controllerPlasmaRelay || m_controllerVoltageRelay)
            blockedReason = QStringLiteral("启动前电源继电器未处于全关状态");
        else if (ui->heliumEnableCheckBox->isChecked() &&
                 (!m_controllerHeliumFlowRelay || !m_controllerHeliumValve ||
                  std::abs(static_cast<int>(m_controllerHeliumOutput) -
                           static_cast<int>(PlasmaControllerProtocol::flowToCentiLitersPerMinute(
                               ui->heliumFlowSpinBox->value(), 30.0))) > 2))
            blockedReason = QStringLiteral("氦气流量计、电磁阀或目标流量回读尚未就绪");
        else if (ui->argonEnableCheckBox->isChecked() &&
                 (!m_controllerArgonFlowRelay || !m_controllerArgonValve ||
                  std::abs(static_cast<int>(m_controllerArgonOutput) -
                           static_cast<int>(PlasmaControllerProtocol::flowToCentiLitersPerMinute(
                               ui->argonFlowSpinBox->value(), 10.0))) > 2))
            blockedReason = QStringLiteral("氩气流量计、电磁阀或目标流量回读尚未就绪");

        if (!blockedReason.isEmpty()) {
            const QSignalBlocker blocker(ui->plasma_start_btn);
            ui->plasma_start_btn->setChecked(false);
            LOG_ERROR("Safety", QStringLiteral("等离子启动被阻止：%1").arg(blockedReason));
            if (tip_manager)
                tip_manager->showTip(blockedReason, TipWidget::Dangerous);
            return;
        }

        const auto answer = QMessageBox::warning(
            this,
            QStringLiteral("腔外静止点火确认"),
            QStringLiteral(
                "本次仅允许机械臂静止、腔外短时点火。\n\n"
                "喷头周围无人员和可燃物，保护接地及高压线连接可靠，物理急停可立即触达。\n"
                "当前物理调压器位置暂定为本轮正确位置。启动后不要转动调压器，只观察放电状态；达到认可状态后立即手动停止。\n"
                "高压反馈超过 8.8 kV Vpp 时仍将自动关断。"),
            QMessageBox::Ok | QMessageBox::Cancel,
            QMessageBox::Cancel);
        if (answer != QMessageBox::Ok) {
            const QSignalBlocker blocker(ui->plasma_start_btn);
            ui->plasma_start_btn->setChecked(false);
            return;
        }

        m_plasmaStartPending = true;
        m_plasmaWorkpointQualified = false;
        const quint64 startSequence = ++m_plasmaStartSequence;
        ui->plasma_start_btn->setText(QStringLiteral("正在确认调压器..."));
        ui->plasma_start_btn->setEnabled(false);
        ui->gasModeComboBox->setEnabled(false);
        ui->singleGasComboBox->setEnabled(false);

        // First energize only the regulator relay. Plasma is allowed only
        // after the controller reports that exact state back.
        serial_window->setPlasmaControl(false, true, false);
        QTimer::singleShot(900, this, [this, startSequence]() {
            if (!m_plasmaStartPending || startSequence != m_plasmaStartSequence)
                return;
            const bool fresh = m_controllerStatusReceived &&
                m_controllerLastStatusTime.msecsTo(QDateTime::currentDateTime()) <=
                    kControllerStatusTimeoutMs;
            if (!fresh || m_controllerEmergencyStop ||
                !m_controllerVoltageRelay || m_controllerPlasmaRelay) {
                const qint64 statusAgeMs = m_controllerStatusReceived
                    ? m_controllerLastStatusTime.msecsTo(QDateTime::currentDateTime())
                    : -1;
                m_plasmaStartPending = false;
                serial_window->setPlasmaControl(false, false, false);
                {
                    const QSignalBlocker blocker(ui->plasma_start_btn);
                    ui->plasma_start_btn->setChecked(false);
                }
                ui->plasma_start_btn->setText(QStringLiteral("启动等离子"));
                ui->plasma_start_btn->setEnabled(true);
                ui->gasModeComboBox->setEnabled(true);
                ui->singleGasComboBox->setEnabled(true);
                LOG_ERROR("Safety", QStringLiteral(
                    "调压器继电器状态确认失败：fresh=%1, status_age_ms=%2, "
                    "emergency_stop=%3, voltage_relay=%4, plasma_relay=%5, "
                    "vpp=%6；已取消等离子启动")
                    .arg(fresh ? QStringLiteral("true") : QStringLiteral("false"))
                    .arg(statusAgeMs)
                    .arg(m_controllerEmergencyStop ? QStringLiteral("true") : QStringLiteral("false"))
                    .arg(m_controllerVoltageRelay ? QStringLiteral("true") : QStringLiteral("false"))
                    .arg(m_controllerPlasmaRelay ? QStringLiteral("true") : QStringLiteral("false"))
                    .arg(m_plasmaFeedbackVpp, 0, 'f', 2));
                if (tip_manager)
                    tip_manager->showTip(
                        QStringLiteral("调压器继电器状态确认失败，已关断"),
                        TipWidget::Dangerous);
                return;
            }

            ui->plasma_start_btn->setText(QStringLiteral("正在确认等离子电源..."));
            serial_window->setPlasmaControl(true, true, false);
            QTimer::singleShot(900, this, [this, startSequence]() {
                if (!m_plasmaStartPending || startSequence != m_plasmaStartSequence)
                    return;
                const bool fresh = m_controllerStatusReceived &&
                    m_controllerLastStatusTime.msecsTo(QDateTime::currentDateTime()) <=
                        kControllerStatusTimeoutMs;
                if (!fresh || m_controllerEmergencyStop ||
                    !m_controllerVoltageRelay || !m_controllerPlasmaRelay) {
                    const qint64 statusAgeMs = m_controllerStatusReceived
                        ? m_controllerLastStatusTime.msecsTo(QDateTime::currentDateTime())
                        : -1;
                    m_plasmaStartPending = false;
                    serial_window->setPlasmaControl(false, false, false);
                    {
                        const QSignalBlocker blocker(ui->plasma_start_btn);
                        ui->plasma_start_btn->setChecked(false);
                    }
                    ui->plasma_start_btn->setText(QStringLiteral("启动等离子"));
                    ui->plasma_start_btn->setEnabled(true);
                    ui->gasModeComboBox->setEnabled(true);
                    ui->singleGasComboBox->setEnabled(true);
                    LOG_ERROR("Safety", QStringLiteral(
                        "等离子继电器状态确认失败：fresh=%1, status_age_ms=%2, "
                        "emergency_stop=%3, voltage_relay=%4, plasma_relay=%5, "
                        "vpp=%6；已执行关断")
                        .arg(fresh ? QStringLiteral("true") : QStringLiteral("false"))
                        .arg(statusAgeMs)
                        .arg(m_controllerEmergencyStop ? QStringLiteral("true") : QStringLiteral("false"))
                        .arg(m_controllerVoltageRelay ? QStringLiteral("true") : QStringLiteral("false"))
                        .arg(m_controllerPlasmaRelay ? QStringLiteral("true") : QStringLiteral("false"))
                        .arg(m_plasmaFeedbackVpp, 0, 'f', 2));
                    if (tip_manager)
                        tip_manager->showTip(
                            QStringLiteral("等离子继电器状态确认失败，已关断"),
                            TipWidget::Dangerous);
                    return;
                }

                m_plasmaStartPending = false;
                if_plasma_running = true;
                m_plasmaWorkpointQualified = false;
                ui->plasma_start_btn->setText(QStringLiteral("停止等离子"));
                ui->plasma_start_btn->setEnabled(true);
                processing_time_widget->ResetTiming();
                processing_time_widget->StartTiming();
                flow_display_widget->StartAnimation();
                LOG_WARN("等离子继电器已确认；保持当前调压器位置不动，观察放电状态后由操作人员手动停止");
                if (tip_manager)
                    tip_manager->showTip(
                        QStringLiteral("保持当前调压器位置不动，观察放电状态后手动停止"),
                        TipWidget::Warning);
            });
        });
    }
    else
    {
        ++m_plasmaStartSequence;
        m_plasmaStartPending = false;
        m_plasmaWorkpointQualified = false;
        ui->plasma_start_btn->setText("启动等离子");
        ui->plasma_start_btn->setEnabled(true);
        if_plasma_running = false;  // 设置等离子运行标志位为false
        
        // 恢复气体模式和单气体类型选择
        ui->gasModeComboBox->setEnabled(true);
        ui->singleGasComboBox->setEnabled(true);
        
        // 清除提示信息
        ui->gasModeComboBox->setToolTip("");
        ui->singleGasComboBox->setToolTip("");
        // Immediately remove both power enables. Gas remains under its own
        // controls so the operator can keep a short post-flow if required.
        if (serial_window)
        {
            serial_window->setPlasmaControl(false, false, false);
            flow_display_widget->StopAnimation();
            processing_time_widget->StopTiming();
        }
        UpdatePlasmaConnectionStatus(serial_window && serial_window->isConnected());
        
        // 显示等离子停止提示
        if (tip_manager) {
            tip_manager->showTip("等离子处理已停止！", TipWidget::Succeed);
            LOG_WARN("等离子输出已关闭");
        }
    }
}

//紧急停止按钮
void MainWindow::OnEmergencyStopClicked()
{
    // 紧急停止所有操作
    ui->plasma_start_btn->setChecked(false);
    ui->argonEnableCheckBox->setChecked(false);
    ui->heliumEnableCheckBox->setChecked(false);

    // 重置所有流量为0
    ui->argonFlowSlider->setValue(0);
    ui->heliumFlowSlider->setValue(0);

    requestPathExecutorStop(QStringLiteral("主界面急停"), true);
    if (m_stepWizardView)
        m_stepWizardView->setRobotExecutionProgress(0);
    if (m_stepWizardModel)
        m_stepWizardModel->setStepFinished(WizardStep::RobotExecution, false);
    LOG_WARN("Safety", "主界面急停：已关闭等离子并发送机械臂停止命令");

    // 通过串口发送紧急停止命令
    if (serial_window)
    {
        serial_window->emergencyStop();
    }

    qDebug() << "Emergency stop activated!";
}


/** 切换氩气启用状态并同步串口输出 */
void MainWindow::OnArgonEnableToggled(bool enabled)
{
    // 移除滑动条的启用/禁用控制，允许在未选中checkbox时也能拖动滑动条
    // ui->argonFlowSlider->setEnabled(enabled);
    // ui->argonFlowSpinBox->setEnabled(enabled);

    if (serial_window)
    {
        if (enabled)
        {
            // 打开氩气流量计和电磁阀，并设置当前流量值
            double flowValue = ui->argonFlowSpinBox->value();
            int outputValue = static_cast<int>(ConvertArgonFlowToOutputValue(flowValue));
            serial_window->setArgonControl(true, true, outputValue);
        }
        else
        {
            // 关闭氩气流量计和电磁阀
            serial_window->setArgonControl(false, false, 0);
        }
    }
}

/** 氩气流量变化时（启用状态）更新串口输出值 */
void MainWindow::OnArgonFlowChanged(double value)
{
    // 同步滑块和数值框的值已经通过信号槽自动处理
    if (serial_window && ui->argonEnableCheckBox->isChecked())
    {
        // 如果氩气已启用，更新串口窗口中的输出值
        int outputValue = static_cast<int>(ConvertArgonFlowToOutputValue(value));
        serial_window->setArgonControl(true, true, outputValue);
    }
}

/** 切换氦气启用状态并同步串口输出 */
void MainWindow::OnHeliumEnableToggled(bool enabled)
{
    // 移除滑动条的启用/禁用控制，允许在未选中checkbox时也能拖动滑动条
    // ui->heliumFlowSlider->setEnabled(enabled);
    // ui->heliumFlowSpinBox->setEnabled(enabled);

    if (serial_window)
    {
        if (enabled)
        {
            // 打开氦气流量计和电磁阀，并设置当前流量值
            double flowValue = ui->heliumFlowSpinBox->value();
            int outputValue = static_cast<int>(ConvertHeliumFlowToOutputValue(flowValue));
            serial_window->setHeliumControl(true, true, outputValue);
        }
        else
        {
            // 关闭氦气流量计和电磁阀
            serial_window->setHeliumControl(false, false, 0);
        }
    }
}

/** 氦气流量变化时（启用状态）更新串口输出值 */
void MainWindow::OnHeliumFlowChanged(double value)
{
    // 同步滑块和数值框的值已经通过信号槽自动处理
    if (serial_window && ui->heliumEnableCheckBox->isChecked())
    {
        // 如果氦气已启用，更新串口窗口中的输出值
        int outputValue = static_cast<int>(ConvertHeliumFlowToOutputValue(value));
        serial_window->setHeliumControl(true, true, outputValue);
    }
}

/** 切换气体模式（单气体/混合）并调整控件可见性与启用 */
void MainWindow::OnGasModeChanged(int index)
{
    if (index == 0)
    { // 单气体模式
        ui->singleGasComboBox->setVisible(true);
        ui->gasRatioLabel->setVisible(false);
        ui->argonRatioLabel->setVisible(false);
        ui->argonRatioSpinBox->setVisible(false);
        ui->heliumRatioLabel->setVisible(false);
        ui->heliumRatioSpinBox->setVisible(false);

        // 隐藏混合气体控件
        ui->blendGasEnableCheckBox->setVisible(false);
        ui->blendGasFlowSlider->setVisible(false);
        ui->blendGasFlowSpinBox->setVisible(false);

        // 根据选择的单气体类型设置控制状态
        OnSingleGasChanged(ui->singleGasComboBox->currentIndex());
    }
    else
    { // 混合模式
        ui->singleGasComboBox->setVisible(false);
        ui->gasRatioLabel->setVisible(true);
        ui->argonRatioLabel->setVisible(true);
        ui->argonRatioSpinBox->setVisible(true);
        ui->heliumRatioLabel->setVisible(true);
        ui->heliumRatioSpinBox->setVisible(true);

        // 显示混合气体控件
        ui->blendGasEnableCheckBox->setVisible(true);
        ui->blendGasFlowSlider->setVisible(true);
        ui->blendGasFlowSpinBox->setVisible(true);

        // 启用所有气体控制
        ui->argonEnableCheckBox->setEnabled(true);
        ui->argonFlowSlider->setEnabled(true);
        ui->argonFlowSpinBox->setEnabled(true);
        ui->heliumEnableCheckBox->setEnabled(true);
        ui->heliumFlowSlider->setEnabled(true);
        ui->heliumFlowSpinBox->setEnabled(true);

        // 恢复所有控件的正常样式
        ui->argonEnableCheckBox->setStyleSheet("");
        ui->argonFlowSpinBox->setStyleSheet("border-radius:5px;");
        ui->heliumEnableCheckBox->setStyleSheet("");
        ui->heliumFlowSpinBox->setStyleSheet("border-radius:5px;");

        // 初始化气体比例
        OnArgonRatioChanged(ui->argonRatioSpinBox->value());
        
    }
}

/** 单气体模式下切换类型（氩/氦），联动启用与比例显示 */
void MainWindow::OnSingleGasChanged(int index)
{
    if (ui->gasModeComboBox->currentIndex() == 0)
    { // 只在单气体模式下生效
        if (index == 0)
        { // 选择氩气
            // 启用氩气控制，禁用氦气控制
            ui->argonEnableCheckBox->setEnabled(true);
            ui->argonFlowSlider->setEnabled(true);
            ui->argonFlowSpinBox->setEnabled(true);

            // 恢复氩气控件的正常样式
            ui->argonEnableCheckBox->setStyleSheet("");
            ui->argonFlowSpinBox->setStyleSheet("border-radius:5px;");

            //更新比例显示
            ui->ar_ratio_lab->setText(QString("比例: %1 %").arg(100));
            ui->he_ratio_lab->setText(QString("比例: %1 %").arg(0));
            //禁用氦气
            ui->heliumEnableCheckBox->setChecked(false);
            ui->heliumEnableCheckBox->setEnabled(false);
            ui->heliumFlowSlider->setValue(0);
            ui->heliumFlowSlider->setEnabled(false);
            ui->heliumFlowSpinBox->setValue(0);
            ui->heliumFlowSpinBox->setEnabled(false);

            // 设置氦气控件为灰色样式
            ui->heliumEnableCheckBox->setStyleSheet("QCheckBox { color: #888888; }");
            ui->heliumFlowSpinBox->setStyleSheet("QDoubleSpinBox { background-color: #3a3a3a; color: #888888; border: 1px solid #555555; border-radius: 5px; }");
        }
        else
        { // 选择氦气
            // 启用氦气控制，禁用氩气控制
            ui->heliumEnableCheckBox->setEnabled(true);
            ui->heliumFlowSlider->setEnabled(true);
            ui->heliumFlowSpinBox->setEnabled(true);

            // 恢复氦气控件的正常样式
            ui->heliumEnableCheckBox->setStyleSheet("");
            ui->heliumFlowSpinBox->setStyleSheet("border-radius:5px;");

            //更新比例显示
            ui->ar_ratio_lab->setText(QString("比例: %1 %").arg(0));
            ui->he_ratio_lab->setText(QString("比例: %1 %").arg(100));
            //禁用氩气
            ui->argonEnableCheckBox->setChecked(false);
            ui->argonEnableCheckBox->setEnabled(false);
            ui->argonFlowSlider->setValue(0);
            ui->argonFlowSlider->setEnabled(false);
            ui->argonFlowSpinBox->setValue(0);
            ui->argonFlowSpinBox->setEnabled(false);

            // 设置氩气控件为灰色样式
            ui->argonEnableCheckBox->setStyleSheet("QCheckBox { color: #888888; }");
            ui->argonFlowSpinBox->setStyleSheet("QDoubleSpinBox { background-color: #3a3a3a; color: #888888; border: 1px solid #555555; border-radius: 5px; }");
        }
    }
}

/** 调整氩气比例，保持总和为100%，并在混合模式联动流量 */
void MainWindow::OnArgonRatioChanged(int value)
{
    // 自动调整氦气比例，确保总和为100%
    int heliumRatio = 100 - value;
    ui->heliumRatioSpinBox->blockSignals(true);
    ui->heliumRatioSpinBox->setValue(heliumRatio);
    ui->heliumRatioSpinBox->blockSignals(false);

    //更新状态显示
    ui->ar_ratio_lab->setText(QString("比例: %1 %").arg(value));
    ui->he_ratio_lab->setText(QString("比例: %1 %").arg(heliumRatio));
    // 标签颜色已在UI文件中设置为固定颜色

    // 如果是混合模式，根据比例更新各气体流量
    if (ui->gasModeComboBox->currentIndex() == 1)
    { // 混合模式
        double blend_flow = ui->blendGasFlowSlider->value() * 10.00;
        double argon_flow = blend_flow * value / 100.00;
        double helium_flow = blend_flow - helium_flow;

        // 只更新滑动条，数值框会通过信号槽自动更新
        int blend_flow_int = ui->blendGasFlowSlider->value() * 10;
        ui->argonFlowSlider->setValue(static_cast<int>(argon_flow));
        ui->heliumFlowSlider->setValue(blend_flow_int - static_cast<int>(argon_flow));
    }
}

/** 调整氦气比例，保持总和为100%，并在混合模式联动流量 */
void MainWindow::OnHeliumRatioChanged(int value)
{
    // 自动调整氩气比例，确保总和为100%
    int argonRatio = 100 - value;
    ui->argonRatioSpinBox->blockSignals(true);
    ui->argonRatioSpinBox->setValue(argonRatio);
    ui->argonRatioSpinBox->blockSignals(false);

    // 更新显示
    //更新状态显示
    ui->ar_ratio_lab->setText(QString("比例: %1 %").arg(argonRatio));
    ui->he_ratio_lab->setText(QString("比例: %1 %").arg(value));

    // 标签颜色已在UI文件中设置为固定颜色

    // 如果是混合模式，根据比例更新各气体流量
    if (ui->gasModeComboBox->currentIndex() == 1)
    { // 混合模式
        double blend_flow = ui->blendGasFlowSlider->value() * 10.00;
        double helium_flow = blend_flow * value / 100.00;
        double argon_flow = blend_flow - helium_flow;

        // 只更新滑动条，数值框会通过信号槽自动更新
        int blend_flow_int = ui->blendGasFlowSlider->value() * 10;
        ui->heliumFlowSlider->setValue(static_cast<int>(helium_flow));
        ui->argonFlowSlider->setValue(blend_flow_int - static_cast<int>(helium_flow));
    }
}

/** 混合气体启用状态切换（当前保留控件状态，不禁用滑条） */
void MainWindow::OnBlendGasEnableToggled(bool enabled)
{
    // 混合气体启用状态改变
    // 注释掉滑动条的启用/禁用控制，允许在未选中checkbox时也能拖动滑动条
    // ui->blendGasFlowSlider->setEnabled(enabled);
    // ui->blendGasFlowSpinBox->setEnabled(enabled);
}

/** 混合气体总流量变化时按比例分配氩/氦流量并更新滑条 */
void MainWindow::OnBlendGasFlowChanged(double value)
{
    // 如果是混合模式，根据比例更新各气体流量
    if (ui->gasModeComboBox->currentIndex() == 1)
    { // 混合模式
        int blend_flow = ui->blendGasFlowSlider->value() * 10;
        int helium_ratio = ui->heliumRatioSpinBox->value();
        int argon_ratio = ui->argonRatioSpinBox->value();

        double helium_flow = blend_flow * helium_ratio / 100.0;
        double argon_flow = blend_flow * argon_ratio / 100.0;

        // 只更新滑动条，数值框会通过信号槽自动更新
        ui->heliumFlowSlider->setValue(static_cast<int>(helium_flow));
        ui->argonFlowSlider->setValue(blend_flow - static_cast<int>(helium_flow));
        qDebug() << "Blend Flow:" << blend_flow;
        qDebug("argon slider: %d helium: %d", blend_flow - static_cast<int>(helium_flow), static_cast<int>(helium_flow));
    }
}

//过滤ComboBox滚轮与点击事件；等离子运行时禁止模式/气体切换
bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    // 禁用ComboBox控件的滚轮事件，但将事件传递给父控件以保持页面滚动
    if ((obj == ui->gasModeComboBox || obj == ui->singleGasComboBox) && event->type() == QEvent::Wheel)
    {
        // 将滚轮事件传递给父控件
        QWidget *parent = qobject_cast<QWidget *>(obj)->parentWidget();
        if (parent)
        {
            QApplication::sendEvent(parent, event);
        }
        return true; // 阻止ComboBox处理滚轮事件
    }
    
    // 检查等离子运行状态下的ComboBox点击事件
    if ((obj == ui->gasModeComboBox || obj == ui->singleGasComboBox) && 
        event->type() == QEvent::MouseButtonPress && if_plasma_running)
    {
        // 根据控件类型显示相应的提示信息
        if (obj == ui->gasModeComboBox)
        {
            if (tip_manager) {
                tip_manager->showTip("等离子运行时无法切换气体模式！", TipWidget::Warning);
            }
        }
        else if (obj == ui->singleGasComboBox)
        {
            if (tip_manager) {
                tip_manager->showTip("等离子运行时无法切换气体类型！", TipWidget::Warning);
            }
        }
        return true; // 阻止ComboBox处理点击事件
    }
    
    return QMainWindow::eventFilter(obj, event);
}


//串口连接状态变化时，更新等离子在线/离线显示
void MainWindow::OnSerialConnectionStatusChanged(bool connected)
{
    // 更新串口状态按钮
    ui->SerialStatus->setStatus(connected ? StateBtn::Connected : StateBtn::Disconnected);
    UpdatePlasmaConnectionStatus(connected);
    if (!connected) {
        if (if_plasma_running || m_plasmaStartPending) {
            ++m_plasmaStartSequence;
            m_plasmaStartPending = false;
            m_plasmaWorkpointQualified = false;
            if_plasma_running = false;
            {
                const QSignalBlocker blocker(ui->plasma_start_btn);
                ui->plasma_start_btn->setChecked(false);
            }
            ui->plasma_start_btn->setText(QStringLiteral("启动等离子"));
            ui->plasma_start_btn->setEnabled(true);
            processing_time_widget->StopTiming();
            flow_display_widget->StopAnimation();
            LOG_ERROR("Safety", "等离子运行期间串口断开；请立即按下物理急停确认断能");
            if (tip_manager) {
                tip_manager->showTip(
                    QStringLiteral("串口断开，无法确认软件关断，请立即按物理急停"),
                    TipWidget::Dangerous);
            }
        }
        m_controllerStatusReceived = false;
        m_controllerLastStatusTime = QDateTime();
        m_hasLivePressureData = false;
        ui->he_pressure_lab->setText(QStringLiteral("压力: -- MPa"));
        ui->ar_pressure_lab->setText(QStringLiteral("压力: -- MPa"));
        ui->he_flowrate_lab->setText(QStringLiteral("流量: -- L/min"));
        ui->ar_flowrate_lab->setText(QStringLiteral("流量: -- L/min"));
        if (flow_display_widget)
            flow_display_widget->SetFlowValue(0.0);
        if (m_serialStatusWidget)
            m_serialStatusWidget->clearPressureValues();
    }
    refreshSerialStatusView();
    appendSerialLog(connected
        ? QStringLiteral("串口已连接：%1").arg(serial_window->getCurrentPortName())
        : QStringLiteral("串口已断开。"));
    updateRobotExecutionReadiness();
}

//根据串口连接状态更新等离子状态标签与样式
void MainWindow::UpdatePlasmaConnectionStatus(bool connected)
{
    if (connected)
    {
        ui->plasmaStateValue->setText("在线");
        ui->plasmaStateValue->setStyleSheet("QLabel {\n    background-color: #4a4a4a;\n    border: 1px solid #666666;\n    border-radius: 4px;\n    padding: 5px;\n    color: #00ff00;\n    font-weight: bold;\n}");
    }
    else
    {
        ui->plasmaStateValue->setText("离线");
        ui->plasmaStateValue->setStyleSheet("QLabel {\n    background-color: #4a4a4a;\n    border: 1px solid #666666;\n    border-radius: 4px;\n    padding: 5px;\n    color: #ff4444;\n    font-weight: bold;\n}");
    }
}

// 将氦气流量转换为控制板协议单位 0.01 L/min。
double MainWindow::ConvertHeliumFlowToOutputValue(double flowValue)
{
    return PlasmaControllerProtocol::flowToCentiLitersPerMinute(flowValue, 30.0);
}

// 将氩气流量转换为控制板协议单位 0.01 L/min。
double MainWindow::ConvertArgonFlowToOutputValue(double flowValue)
{
    return PlasmaControllerProtocol::flowToCentiLitersPerMinute(flowValue, 10.0);
}

/**
 * @brief 压力值更新槽函数
 * @param heliumPressureMpa 氦气压力值(MPa)
 * @param argonPressureMpa 氩气压力值(MPa)
 */
// =========串口相关===========
void MainWindow::OnPressureValuesUpdated(double heliumPressureMpa, double argonPressureMpa)
{
    m_hasLivePressureData = true;
    if (m_serialStatusWidget)
        m_serialStatusWidget->setPressureValues(heliumPressureMpa, argonPressureMpa);
    ui->he_pressure_lab->setText(
        QStringLiteral("压力: %1 MPa").arg(heliumPressureMpa, 0, 'f', 3));
    ui->ar_pressure_lab->setText(
        QStringLiteral("压力: %1 MPa").arg(argonPressureMpa, 0, 'f', 3));
}


/** 混合流量复选框联动：勾选时启用氦/氩，取消时同时关闭 */
void MainWindow::OnBlendGasFlowCheckBoxToggled(bool checked)
{
    if (checked)
    {
        // 当混合气体流量复选框被选中时，同时选中氦气流量和氩气流量复选框
        ui->heliumEnableCheckBox->setChecked(true);
        ui->argonEnableCheckBox->setChecked(true);
    }
    else
    {
        // 当取消选中混合气体流量复选框时，同时取消氦气和氩气的选中状态
        ui->heliumEnableCheckBox->setChecked(false);
        ui->argonEnableCheckBox->setChecked(false);
    }
}


/** 串口回读氦气目标流量，协议单位为 0.01 L/min。 */
void MainWindow::OnHeliumCurrentValueChanged(quint16 value)
{
    const double flow_value = static_cast<double>(value) / 100.0;
    ui->he_flowrate_lab->setText(QString("流量: %1 L/min").arg(flow_value, 0, 'f', 2));

    const double argon_flow =
        static_cast<double>(serial_window->getArgonCurrentValue()) / 100.0;

    flow_display_widget->SetFlowValue(argon_flow + flow_value);
}

/** 串口回读氩气目标流量，协议单位为 0.01 L/min。 */
void MainWindow::OnArgonCurrentValueChanged(quint16 value)
{
    const double flow_value = static_cast<double>(value) / 100.0;
    ui->ar_flowrate_lab->setText(QString("流量: %1 L/min").arg(flow_value, 0, 'f', 2));

    const double helium_flow =
        static_cast<double>(serial_window->getHeliumCurrentValue()) / 100.0;

    flow_display_widget->SetFlowValue(helium_flow + flow_value);
}

/** 获取等离子处理是否正在运行的状态标志 */
bool MainWindow::GetPlasmaDealState() const
{
    return if_plasma_running;
}

/** 显示欢迎使用的提示信息 */
void MainWindow::ShowWelcomeTip()
{
    if (tip_manager) {
        tip_manager->showTip("欢迎使用等离子处理系统！", TipWidget::Succeed);
    }
}

//==================4-一键式系统引导=======================






//===============裁剪设置====================
void MainWindow::OnCropToggled(bool checked)
{
    if (checked) {
        if (!m_cloudCapturePaused && !m_recropTarget) {
            ui->btnCrop->setChecked(false);
            if (tip_manager)
                tip_manager->showTip(QStringLiteral("请先暂停实时点云帧后再裁剪"), TipWidget::Warning);
            return;
        }
        ui->btnLabel->setChecked(false);
        // 默认启用矩形裁剪
        if (ui->main_gl) {
            ui->main_gl->EnableCrop(true, MainOpengl::Crop_Rect);
        }
    } else {
        // 关闭裁剪
        if (ui->main_gl) {
            ui->main_gl->EnableCrop(false);
        }
    }
}

void MainWindow::OnLabelToggled(bool checked)
{
    if (checked) {
        ui->btnCrop->setChecked(false);
    }
    if (ui->main_gl) {
        ui->main_gl->EnableLabel(checked);
    }
}

/**
 * @brief 选帧/暂停按钮切换槽函数
 * @param checked true:暂停实时画面并保留当前帧; false:恢复实时画面
 */
void MainWindow::OnSelectFrameToggled(bool checked)
{
    if (checked) {
        if (ui->main_gl) {
            ui->main_gl->SetRosPaused(true);
        }
        if (tip_manager) {
                    tip_manager->showTip("已暂停实时画面，保留当前帧", TipWidget::Succeed);
                }
            } else {
                if (ui->main_gl) {
                    ui->main_gl->SetRosPaused(false);
                }
                if (tip_manager) {
                    tip_manager->showTip("恢复实时画面", TipWidget::Succeed);
                }
            }
}

// =========StepWizard 系统引导===========
/**
 * initStepWizard - 初始化一键下一步式系统引导
 *
 * 创建 StepWizardModel 和 StepWizardView，绑定 UI 控件，连接信号槽。
 * 此函数替代原有的 SetupStepNavigation()，统一管理步骤流程。
 */
void MainWindow::initStepWizard()
{
    // ---- 1. 创建 Model ----
    m_stepWizardModel = new StepWizardModel(this);
    bool debugMode = false;
    if (qEnvironmentVariableIsSet("PLASMA_GUI_DEBUG_MODE"))
        debugMode = qEnvironmentVariableIntValue("PLASMA_GUI_DEBUG_MODE") != 0;
    m_stepWizardModel->setDebugMode(debugMode);

    // 机械臂执行是正式流程步骤，页面在代码中创建以复用现有流程卡片容器。
    auto *robotExecutionPage = new QWidget(ui->optionStep);
    robotExecutionPage->setObjectName(QStringLiteral("page4_robotExecution"));
    robotExecutionPage->setStyleSheet(QStringLiteral(
        "QWidget#page4_robotExecution { background-color: #2b2b2b; }"));
    auto *robotPageLayout = new QVBoxLayout(robotExecutionPage);
    robotPageLayout->setContentsMargins(10, 5, 10, 5);
    robotPageLayout->setSpacing(3);

    auto *robotTitle = new QLabel(QStringLiteral("机械臂执行  |  执行前检查"), robotExecutionPage);
    robotTitle->setObjectName(QStringLiteral("robotExecutionTitleLabel"));
    robotTitle->setStyleSheet(QStringLiteral(
        "color: #f3f4f6; font-size: 14px; font-weight: 600;"));
    robotPageLayout->addWidget(robotTitle);

    auto *readinessLayout = new QGridLayout();
    readinessLayout->setContentsMargins(0, 0, 0, 0);
    readinessLayout->setHorizontalSpacing(6);
    readinessLayout->setVerticalSpacing(1);
    int readinessIndex = 0;
    auto addReadinessItem = [robotExecutionPage, readinessLayout,
                             &readinessIndex](const QString &objectName) {
        auto *label = new QLabel(robotExecutionPage);
        label->setObjectName(objectName);
        label->setStyleSheet(QStringLiteral("color: #f59e0b; font-size: 11px;"));
        readinessLayout->addWidget(label, readinessIndex / 2, readinessIndex % 2);
        ++readinessIndex;
    };
    addReadinessItem(QStringLiteral("robotReadyWorkflow"));
    addReadinessItem(QStringLiteral("robotReadyArmConfig"));
    addReadinessItem(QStringLiteral("robotReadyArmOnline"));
    addReadinessItem(QStringLiteral("robotReadyController"));
    addReadinessItem(QStringLiteral("robotReadyControllerOutputs"));
    addReadinessItem(QStringLiteral("robotReadyPathParams"));
    addReadinessItem(QStringLiteral("robotReadyTrajectory"));
    robotPageLayout->addLayout(readinessLayout);

    m_robotSafetyConfirmation = new QCheckBox(
        QStringLiteral("已核对机械臂、控制板和工艺参数，现场安全"),
        robotExecutionPage);
    m_robotSafetyConfirmation->setObjectName(QStringLiteral("robotSafetyConfirmation"));
    m_robotSafetyConfirmation->setStyleSheet(QStringLiteral(
        "QCheckBox { color: #f3f4f6; font-size: 11px; spacing: 6px; }"
        "QCheckBox:disabled { color: #6b7280; }"));
    robotPageLayout->addWidget(m_robotSafetyConfirmation);

    m_cavityRetreatButton = new QPushButton(
        QStringLiteral("退出腔体（退到预入口）"), robotExecutionPage);
    m_cavityRetreatButton->setObjectName(QStringLiteral("cavityRetreatButton"));
    m_cavityRetreatButton->setToolTip(
        QStringLiteral("喷涂完成并回零后，保持末端姿态沿层中心和入口法向退出到预入口"));
    m_cavityRetreatButton->setEnabled(false);
    m_cavityRetreatButton->setMinimumHeight(28);
    m_cavityRetreatButton->setStyleSheet(QStringLiteral(
        "QPushButton { color: #f3f4f6; background: #2563eb; border: 1px solid #3b82f6; padding: 4px 10px; }"
        "QPushButton:hover { background: #1d4ed8; }"
        "QPushButton:disabled { color: #6b7280; background: #303030; border-color: #454545; }"));
    connect(m_cavityRetreatButton, &QPushButton::clicked,
            this, &MainWindow::requestCavityRetreat);
    robotPageLayout->addWidget(m_cavityRetreatButton);
    robotPageLayout->addStretch(1);

    auto *robotExecutionProgress = new QProgressBar(robotExecutionPage);
    robotExecutionProgress->setObjectName(QStringLiteral("robotExecutionProgressBar"));
    robotExecutionProgress->setMinimumSize(0, 8);
    robotExecutionProgress->setMaximumHeight(8);
    robotExecutionProgress->setTextVisible(false);
    robotExecutionProgress->setRange(0, 100);
    robotExecutionProgress->setValue(0);
    robotPageLayout->addWidget(robotExecutionProgress);
    ui->optionStep->addWidget(robotExecutionPage);

    // ---- 2. 创建 View ----
    m_stepWizardView = new StepWizardView(this);

    // ---- 3. 绑定导航按钮和功能按钮 ----
    auto *cloudRebuildProgress = new QProgressBar(ui->page2);
    cloudRebuildProgress->setObjectName(QStringLiteral("cloudRebuildProgressBar"));
    cloudRebuildProgress->setMinimumSize(0, 8);
    cloudRebuildProgress->setMaximumHeight(8);
    cloudRebuildProgress->setTextVisible(false);
    cloudRebuildProgress->setRange(0, 100);
    cloudRebuildProgress->setValue(0);
    ui->page2Layout->addWidget(cloudRebuildProgress);
    auto *pathPlanningProgress = new QProgressBar(ui->page3);
    pathPlanningProgress->setObjectName(QStringLiteral("pathPlanningProgressBar"));
    pathPlanningProgress->setMinimumSize(0, 8);
    pathPlanningProgress->setMaximumHeight(8);
    pathPlanningProgress->setTextVisible(false);
    pathPlanningProgress->setRange(0, 100);
    pathPlanningProgress->setValue(0);
    ui->page3Layout->addWidget(pathPlanningProgress);
    m_stepWizardView->bindWidgets(nullptr,
                                  nullptr,
                                  ui->opprocessStepIndicator,
                                  cloudRebuildProgress);
    m_stepWizardView->bindPathPlanningProgress(pathPlanningProgress);
    m_stepWizardView->bindRobotExecutionProgress(robotExecutionProgress);
    m_stepWizardView->bindBtns(
        ui->optionStep,
        ui->btnStepPrev,
        ui->btnStepNext,
        ui->btn1,
        ui->btn2,
        ui->btn3,
        ui->btn4,
        ui->opprocessStepIndicator
    );

    // ---- 4. Model 与 View 绑定 ----
    m_stepWizardView->setModel(m_stepWizardModel);
    connect(m_robotSafetyConfirmation, &QCheckBox::toggled,
            this, &MainWindow::updateRobotExecutionReadiness);

    // ---- 5. 连接 Model 信号到 MainWindow 槽 ----
    // 上一步 / 下一步按钮已通过 View 绑定，此处额外连接业务逻辑

    // 警告消息 → 弹窗提示
    connect(m_stepWizardModel, &StepWizardModel::warningMessage,
            this, [this](const QString &msg) {
        if (tip_manager) {
            tip_manager->showTip(msg, TipWidget::Warning);
        }
    });

    // 流程完成 → 提示用户
    connect(m_stepWizardModel, &StepWizardModel::workflowFinished,
            this, [this]() {
        if (tip_manager) {
            tip_manager->showTip(QStringLiteral("🎉 所有步骤已完成！流程结束。"), TipWidget::Succeed);
        }
    });

    // 步骤切换 → 同步加载对应点云
    connect(m_stepWizardModel, &StepWizardModel::stepChanged,
            this, [this](int index) {
        // 用旧接口兼容：更新 currentStepIndex 并加载点云
        currentStepIndex = index;
        if (index != static_cast<int>(WizardStep::RobotExecution))
            resetRobotExecutionConfirmation();
        m_cloudCapturePaused = false;
        m_cloudCaptureCropped = false;
        ui->btnCrop->setChecked(false);
        ui->btnCrop->setEnabled(false);
        if (ui->main_gl) {
            const bool enteringCloudRebuild = (index == static_cast<int>(WizardStep::CloudRebuild));
            const bool enteringPostCapture = (index >= static_cast<int>(WizardStep::CloudRebuild));
            ui->main_gl->SetRosPaused(enteringPostCapture);
            if (enteringCloudRebuild && dbtree_view_)
                dbtree_view_->syncCheckedCloudVisibility();
        }
        if (dbtree_view_)
            dbtree_view_->setPathPlanningMenuEnabled(index == static_cast<int>(WizardStep::PathPlanning));
        if (m_stepWizardView && index == static_cast<int>(WizardStep::CloudCapture))
            m_stepWizardView->setCloudCaptureConfirmEnabled(false);
        updateRobotExecutionSummary();
        emit StepChanged(index);
    });
    connect(m_stepWizardModel, &StepWizardModel::stepFinishedChanged,
            this, [this](int, bool) { updateRobotExecutionReadiness(); });

    qDebug() << "StepWizard initialized with" << m_stepWizardModel->totalSteps() << "steps";

    // ---- 6. 连接 View 请求信号 → MainWindow 槽 ----
    initStepWizardConnections();
    updateRobotExecutionSummary();
}

void MainWindow::updateRobotExecutionSummary()
{
    if (!ui || !ui->optionStep)
        return;

    auto setText = [this](const QString &objectName, const QString &text) {
        if (auto *label = ui->optionStep->findChild<QLabel *>(objectName))
            label->setText(text);
    };

    const bool armOnline = m_armHealthModel && m_armHealthModel->dataActive() &&
        !m_armHealthModel->isDataStale(kArmHeartbeatTimeoutMs) &&
        m_armHealthModel->state() == DeviceHealthModel::State::Healthy;
    setText(QStringLiteral("robotExecutionArmState"),
            armOnline ? QStringLiteral("在线，关节状态实时更新")
                      : QStringLiteral("未就绪"));

    const auto data = m_selectedCloudData;
    const bool trajectoryReady = data && data->nozzlePoseSequence &&
        data->nozzlePoseSequence->GetNumberOfPoints() > 0 &&
        data->straightAxis && data->straightAxis->GetNumberOfPoints() >= 2;
    setText(QStringLiteral("robotExecutionTrajectory"),
            trajectoryReady ? data->displayName
                            : QStringLiteral("请选择已生成完整喷涂轨迹的子节点"));

    int layerCount = 0;
    if (trajectoryReady) {
        if (auto *sliceIds = vtkIntArray::SafeDownCast(
                data->nozzlePoseSequence->GetPointData()->GetArray("SliceIndex"))) {
            QSet<int> layers;
            for (vtkIdType i = 0; i < sliceIds->GetNumberOfTuples(); ++i)
                layers.insert(sliceIds->GetValue(i));
            layerCount = layers.size();
        }
    }
    setText(QStringLiteral("robotExecutionLayers"),
            layerCount > 0 ? QStringLiteral("%1 个闭合层").arg(layerCount)
                           : QStringLiteral("--"));
    setText(QStringLiteral("robotExecutionSpacing"),
            trajectoryReady && data->plannedSliceSpacing > 0.0
                ? QStringLiteral("%1 mm").arg(
                    data->plannedSliceSpacing * 1000.0, 0, 'f', 1)
                : QStringLiteral("--"));
    QString executorMode = !m_pathExecutorStatusReceived
        ? QStringLiteral("等待执行器状态")
        : (m_pathExecutorMotionEnabled
            ? QStringLiteral("运动许可已开启；等离子输出关闭")
            : QStringLiteral("预览模式；运动许可关闭"));
    if (m_entryMotionStatusReceived) {
        executorMode += QStringLiteral("；入口规划器：%1")
            .arg(m_entryMotionMessage);
    }
    if (trajectoryReady) {
        executorMode = QStringLiteral("运动 TCP %1 mm；圆头安全入口外停 %2 mm；%3")
            .arg(data->plannedToolTcpOffset * 1000.0, 0, 'f', 1)
            .arg(data->plannedEntryTipStandoff * 1000.0, 0, 'f', 1)
            .arg(executorMode);
    }
    setText(QStringLiteral("robotExecutionMode"), executorMode);
    if (m_cavityRetreatButton) {
        const bool retreatReady = m_entryMotionStatusReceived &&
            (m_entryMotionState ==
                plasma_robot_interfaces::msg::EntryMotionStatus::STATE_COMPLETED ||
             m_entryMotionState ==
                plasma_robot_interfaces::msg::EntryMotionStatus::STATE_AT_FIRST_LAYER ||
             m_entryMotionState ==
                plasma_robot_interfaces::msg::EntryMotionStatus::STATE_STOPPED_IN_CAVITY) &&
            m_rosWorker && m_rosWorker->node() &&
            m_rosWorker->node()->entryRetreatReady();
        m_cavityRetreatButton->setEnabled(retreatReady);
        if (m_entryMotionState ==
            plasma_robot_interfaces::msg::EntryMotionStatus::STATE_STOPPED_IN_CAVITY) {
            m_cavityRetreatButton->setText(
                QStringLiteral("从当前停车点安全退出"));
            m_cavityRetreatButton->setToolTip(
                QStringLiteral("保持当前末端姿态，重新碰撞规划后沿腔体轴向退出到预入口"));
        } else if (m_entryMotionState ==
                   plasma_robot_interfaces::msg::EntryMotionStatus::STATE_STOPPING_IN_CAVITY) {
            m_cavityRetreatButton->setText(
                QStringLiteral("等待机械臂完全停稳..."));
            m_cavityRetreatButton->setToolTip(
                QStringLiteral("控制器确认空闲并收到新鲜位姿后才允许退出"));
        } else {
            m_cavityRetreatButton->setText(
                QStringLiteral("退出腔体（退到预入口）"));
            m_cavityRetreatButton->setToolTip(
                QStringLiteral("喷涂完成后保持末端姿态沿入口法向退出到预入口"));
        }
    }
    if (auto *title = ui->optionStep->findChild<QLabel *>(
            QStringLiteral("robotExecutionTitleLabel"))) {
        title->setToolTip(QStringLiteral(
            "机械臂：%1\n轨迹：%2\n层数：%3\n层间距：%4\n执行配置：%5")
            .arg(armOnline ? QStringLiteral("在线") : QStringLiteral("未就绪"))
            .arg(trajectoryReady ? data->displayName : QStringLiteral("未选择"))
            .arg(layerCount > 0 ? QString::number(layerCount) : QStringLiteral("--"))
            .arg(trajectoryReady && data->plannedSliceSpacing > 0.0
                ? QStringLiteral("%1 mm").arg(
                    data->plannedSliceSpacing * 1000.0, 0, 'f', 1)
                : QStringLiteral("--"))
            .arg(executorMode));
    }
    updateRobotExecutionReadiness();
}

void MainWindow::requestCavityRetreat()
{
    using EntryStatus = plasma_robot_interfaces::msg::EntryMotionStatus;
    if (!m_rosWorker || !m_rosWorker->node() ||
        !m_rosWorker->node()->entryRetreatReady()) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("机械臂退出服务未就绪"),
                                 TipWidget::Dangerous);
        return;
    }
    if (!m_entryMotionStatusReceived ||
        (m_entryMotionState != EntryStatus::STATE_COMPLETED &&
         m_entryMotionState != EntryStatus::STATE_AT_FIRST_LAYER &&
         m_entryMotionState != EntryStatus::STATE_STOPPED_IN_CAVITY)) {
        if (tip_manager)
            tip_manager->showTip(
                m_entryMotionState == EntryStatus::STATE_STOPPING_IN_CAVITY
                    ? QStringLiteral("机械臂正在停稳，暂时不能启动退出运动")
                    : QStringLiteral("当前状态没有可执行的腔体退出轨迹"),
                TipWidget::Warning);
        return;
    }

    const bool fromFirstLayer =
        m_entryMotionState == EntryStatus::STATE_AT_FIRST_LAYER;
    const bool fromArbitraryStop =
        m_entryMotionState == EntryStatus::STATE_STOPPED_IN_CAVITY;
    const QString confirmation = fromArbitraryStop
        ? QStringLiteral(
            "机械臂将从红色停止后的实际停车位姿重新计算退出轨迹。保持当前末端姿态，不会先在腔内回到 0°；TCP 只沿腔体外向轴移动到预入口平面。执行前会重新检查实时关节、IK、碰撞和关节跳变。\n\n"
            "真实等离子输出保持关闭。确认机械臂已经完全停稳，腔内及入口附近无人、无新增障碍物，物理急停可触达。")
        : QStringLiteral(
            "%1，保持喷口 0° 回零姿态向开口直接退出，再沿入口法向到达物理开口外的预入口。退出过程不会重演正负 180° 喷涂旋转。\n\n"
            "此操作不会返回机械臂初始位置，真实等离子输出保持关闭。确认 RViz 退出动画无腔内旋转，腔内及入口附近无人、无新增障碍物，物理急停可触达。")
            .arg(fromFirstLayer ? QStringLiteral("机械臂将从第一层停车点")
                                : QStringLiteral("机械臂将从最深层停车点沿各层中心"));
    if (QMessageBox::warning(this, QStringLiteral("确认退出腔体"), confirmation,
                             QMessageBox::Ok | QMessageBox::Cancel,
                             QMessageBox::Cancel) != QMessageBox::Ok) {
        return;
    }

    m_cavityRetreatButton->setEnabled(false);
    const bool sent = m_rosWorker->node()->callEntryRetreat(
        [this](bool ok, const std::string &responseMessage) {
            const QString message = QString::fromStdString(responseMessage);
            QMetaObject::invokeMethod(this, [this, ok, message]() {
                if (ok) {
                    LOG_INFO("RobotArm", QStringLiteral("机械臂已退出腔体：%1").arg(message));
                    if (tip_manager)
                        tip_manager->showTip(
                            QStringLiteral("机械臂已沿入口法向退出到预入口"),
                            TipWidget::Succeed);
                } else {
                    LOG_ERROR("RobotArm", QStringLiteral("机械臂退出失败：%1").arg(message));
                    if (tip_manager)
                        tip_manager->showTip(
                            QStringLiteral("机械臂退出失败：%1").arg(message),
                            TipWidget::Dangerous);
                }
                updateRobotExecutionSummary();
            }, Qt::QueuedConnection);
        });
    if (!sent) {
        updateRobotExecutionSummary();
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("机械臂退出服务已断开"),
                                 TipWidget::Dangerous);
    }
}

bool MainWindow::robotExecutionReady(QString *reason,
                                     bool includeOperatorConfirmation) const
{
    auto fail = [reason](const QString &message) {
        if (reason)
            *reason = message;
        return false;
    };

    const bool workflowReady = m_stepWizardModel &&
        m_stepWizardModel->isStepFinished(static_cast<int>(WizardStep::SystemCheck)) &&
        m_stepWizardModel->isStepFinished(static_cast<int>(WizardStep::CloudCapture)) &&
        m_stepWizardModel->isStepFinished(static_cast<int>(WizardStep::CloudRebuild));
    if (!workflowReady)
        return fail(QStringLiteral("请按顺序完成系统自检、残腔采集和残腔重建"));

    if (!m_armConfig.valid)
        return fail(QStringLiteral("机械臂型号、IP 和端口参数尚未有效加载"));

    const bool armOnline = m_armHealthModel && m_armHealthModel->dataActive() &&
        !m_armHealthModel->isDataStale(kArmHeartbeatTimeoutMs) &&
        m_armHealthModel->state() == DeviceHealthModel::State::Healthy;
    if (!armOnline)
        return fail(QStringLiteral("机械臂未在线或六轴关节状态已过期"));

    const bool controllerStatusFresh = m_controllerStatusReceived &&
        m_controllerLastStatusTime.isValid() &&
        m_controllerLastStatusTime.msecsTo(QDateTime::currentDateTime()) <=
            kControllerStatusTimeoutMs;
    if (!serial_window || !serial_window->isConnected() || !controllerStatusFresh)
        return fail(QStringLiteral("请连接控制板并等待有效状态数据"));
    if (m_controllerEmergencyStop)
        return fail(QStringLiteral("控制板急停处于触发状态，请排除后复位"));

    const bool dangerousOutputsOff = !m_controllerPlasmaRelay &&
        !m_controllerVoltageRelay && !m_controllerHeliumFlowRelay &&
        !m_controllerHeliumValve && !m_controllerArgonFlowRelay &&
        !m_controllerArgonValve;
    if (!dangerousOutputsOff)
        return fail(QStringLiteral("运动前请关闭等离子、调压器、流量计和电磁阀"));

    if (!m_pathParamsConfigured)
        return fail(QStringLiteral("请先保存工具和路径参数"));

    const bool pathPlanningReady = m_stepWizardModel &&
        m_stepWizardModel->isStepFinished(static_cast<int>(WizardStep::PathPlanning));
    const auto data = m_selectedCloudData;
    const bool trajectoryReady = pathPlanningReady && data && data->nozzlePoseSequence &&
        data->nozzlePoseSequence->GetNumberOfPoints() > 0 && data->straightAxis &&
        data->straightAxis->GetNumberOfPoints() >= 2;
    if (!trajectoryReady)
        return fail(QStringLiteral("请选择已完成规划的完整喷涂轨迹"));

    if (includeOperatorConfirmation &&
        (!m_robotSafetyConfirmation || !m_robotSafetyConfirmation->isChecked())) {
        return fail(QStringLiteral("请完成参数核对和现场安全确认"));
    }

    if (reason)
        reason->clear();
    return true;
}

void MainWindow::resetRobotExecutionConfirmation()
{
    if (!m_robotSafetyConfirmation || !m_robotSafetyConfirmation->isChecked())
        return;

    m_robotSafetyConfirmation->blockSignals(true);
    m_robotSafetyConfirmation->setChecked(false);
    m_robotSafetyConfirmation->blockSignals(false);
}

void MainWindow::updateRobotExecutionReadiness()
{
    if (!ui || !ui->optionStep)
        return;

    auto setCheck = [this](const QString &objectName, int order,
                           bool ready, const QString &text) {
        if (auto *label = ui->optionStep->findChild<QLabel *>(objectName)) {
            label->setText(QStringLiteral("%1  [%2] %3")
                .arg(order)
                .arg(ready ? QStringLiteral("通过") : QStringLiteral("等待"), text));
            label->setStyleSheet(ready
                ? QStringLiteral("color: #22c55e; font-size: 11px;")
                : QStringLiteral("color: #f59e0b; font-size: 11px;"));
        }
    };

    const bool workflowReady = m_stepWizardModel &&
        m_stepWizardModel->isStepFinished(static_cast<int>(WizardStep::SystemCheck)) &&
        m_stepWizardModel->isStepFinished(static_cast<int>(WizardStep::CloudCapture)) &&
        m_stepWizardModel->isStepFinished(static_cast<int>(WizardStep::CloudRebuild));
    const bool armConfigReady = m_armConfig.valid;
    const bool armOnline = m_armHealthModel && m_armHealthModel->dataActive() &&
        !m_armHealthModel->isDataStale(kArmHeartbeatTimeoutMs) &&
        m_armHealthModel->state() == DeviceHealthModel::State::Healthy;
    const bool controllerStatusFresh = m_controllerStatusReceived &&
        m_controllerLastStatusTime.isValid() &&
        m_controllerLastStatusTime.msecsTo(QDateTime::currentDateTime()) <=
            kControllerStatusTimeoutMs;
    const bool controllerReady = serial_window && serial_window->isConnected() &&
        controllerStatusFresh && !m_controllerEmergencyStop;
    const bool controllerOutputsSafe = controllerReady &&
        !m_controllerPlasmaRelay && !m_controllerVoltageRelay &&
        !m_controllerHeliumFlowRelay && !m_controllerHeliumValve &&
        !m_controllerArgonFlowRelay && !m_controllerArgonValve;
    const bool pathParamsReady = m_pathParamsConfigured;
    const bool pathPlanningReady = m_stepWizardModel &&
        m_stepWizardModel->isStepFinished(static_cast<int>(WizardStep::PathPlanning));
    const auto data = m_selectedCloudData;
    const bool trajectoryReady = pathPlanningReady && data && data->nozzlePoseSequence &&
        data->nozzlePoseSequence->GetNumberOfPoints() > 0 && data->straightAxis &&
        data->straightAxis->GetNumberOfPoints() >= 2;

    setCheck(QStringLiteral("robotReadyWorkflow"), 1, workflowReady,
             QStringLiteral("系统流程"));
    setCheck(QStringLiteral("robotReadyArmConfig"), 2, armConfigReady,
             QStringLiteral("机械臂参数"));
    setCheck(QStringLiteral("robotReadyArmOnline"), 3, armOnline,
             QStringLiteral("机械臂在线"));
    setCheck(QStringLiteral("robotReadyController"), 4, controllerReady,
             QStringLiteral("控制板/急停"));
    setCheck(QStringLiteral("robotReadyControllerOutputs"), 5, controllerOutputsSafe,
             QStringLiteral("控制板输出安全"));
    setCheck(QStringLiteral("robotReadyPathParams"), 6, pathParamsReady,
             QStringLiteral("工具/路径参数"));
    setCheck(QStringLiteral("robotReadyTrajectory"), 7, trajectoryReady,
             QStringLiteral("完整喷涂轨迹"));

    QString automaticReason;
    const bool automaticReady = robotExecutionReady(&automaticReason, false);
    if (!automaticReady)
        resetRobotExecutionConfirmation();
    if (m_robotSafetyConfirmation)
        m_robotSafetyConfirmation->setEnabled(automaticReady);

    QString reason;
    const bool ready = robotExecutionReady(&reason, true);
    if (m_stepWizardView)
        m_stepWizardView->setRobotExecutionEnabled(ready, reason);
}

// =============================================
//  initStepWizardConnections - 连接 View 请求信号
// =============================================
void MainWindow::initStepWizardConnections()
{
    connect(m_stepWizardView, &StepWizardView::requestSystemCheck,
            this, &MainWindow::onRequestSystemCheck);
    connect(m_stepWizardView, &StepWizardView::requestCloudCapture,
            this, &MainWindow::onRequestCloudCapture);
    connect(m_stepWizardView, &StepWizardView::requestCloudCaptureConfirm,
            this, &MainWindow::onConfirmCloudCapture);
    connect(m_stepWizardView, &StepWizardView::requestPathExecution,
            this, &MainWindow::onRequestPathExecution);
    connect(m_stepWizardView, &StepWizardView::requestRobotStop,
            this, [this]() {
        requestPathExecutorStop(QStringLiteral("操作者停止"), true);
        if (m_stepWizardView)
            m_stepWizardView->setRobotExecutionProgress(0);
    });
    connect(m_stepWizardView, &StepWizardView::requestPathParamConfig,
            this, &MainWindow::showPathParamDialog);
}

void MainWindow::requestPathExecutorStop(const QString &reason, bool sendDirectStop)
{
    bool requestSent = false;
    if (m_rosWorker && m_rosWorker->node()) {
        m_rosWorker->node()->callEntryStop(
            [this, reason](bool ok, const std::string &responseMessage) {
                const QString message = QString::fromStdString(responseMessage);
                QMetaObject::invokeMethod(this, [this, ok, reason, message]() {
                    const QString log = QStringLiteral("%1（入口规划器）：%2")
                        .arg(reason, message);
                    if (ok)
                        LOG_INFO("RobotArm", log);
                    else
                        LOG_WARN("RobotArm", log);
                }, Qt::QueuedConnection);
            });
        requestSent = m_rosWorker->node()->stopSprayPath(
            [this, reason](bool ok, const std::string &responseMessage) {
                const QString message = QString::fromStdString(responseMessage);
                QMetaObject::invokeMethod(this, [this, ok, reason, message]() {
                    const QString log = QStringLiteral("%1：%2").arg(reason, message);
                    appendArmLog(log);
                    if (ok)
                        LOG_INFO("RobotArm", log);
                    else
                        LOG_WARN("RobotArm", log);
                }, Qt::QueuedConnection);
            });
        if (sendDirectStop)
            m_rosWorker->node()->publishMoveStop();
    }
    if (!requestSent)
        LOG_WARN("RobotArm", QStringLiteral("%1：正式停止服务未就绪").arg(reason));
    if (m_stepWizardModel)
        m_stepWizardModel->setStepFinished(WizardStep::RobotExecution, false);
}

void MainWindow::onRequestPathExecution()
{
    if (currentStepIndex != static_cast<int>(WizardStep::RobotExecution))
        return;

    QString readinessReason;
    if (!robotExecutionReady(&readinessReason, true)) {
        if (tip_manager)
            tip_manager->showTip(readinessReason, TipWidget::Warning);
        updateRobotExecutionReadiness();
        return;
    }
    if (m_stepWizardView)
        m_stepWizardView->setRobotExecutionProgress(0);

    const auto data = m_selectedCloudData;
    if (!data || !data->nozzlePoseSequence ||
        data->nozzlePoseSequence->GetNumberOfPoints() == 0) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先选择已生成完整喷涂位姿的轨迹节点"),
                                 TipWidget::Warning);
        return;
    }
    if (!m_rosWorker || !m_rosWorker->node()) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("机械臂 ROS 服务未就绪"),
                                 TipWidget::Dangerous);
        return;
    }

    using EntryStatus = plasma_robot_interfaces::msg::EntryMotionStatus;
    if (!m_rosWorker->node()->entryPlanReady()) {
        if (m_rosLaunchManager && !m_rosLaunchManager->isRunning("entry_motion"))
            m_rosLaunchManager->start("entry_motion");
        const QString message = QStringLiteral(
            "正在启动当前 304 mm 等离子末端的 MoveIt 入口规划器和 RViz，请等待就绪后再次点击");
        LOG_INFO("RobotArm", message);
        if (tip_manager)
            tip_manager->showTip(message, TipWidget::Warning);
        return;
    }
    if (!m_entryMotionStatusReceived || data->motionPathId.isEmpty() ||
        m_entryMotionPathId != data->motionPathId) {
        publishPathForTransform(data);
        const QString message = QStringLiteral(
            "已把入口、预入口和喷涂路径重新提交给入口规划器，请等待状态更新后再次点击");
        LOG_INFO("RobotArm", message);
        if (tip_manager)
            tip_manager->showTip(message, TipWidget::Warning);
        return;
    }
    if (m_entryMotionState == EntryStatus::STATE_READY_TO_PLAN ||
        m_entryMotionState == EntryStatus::STATE_ERROR) {
        const bool sent = m_rosWorker->node()->callEntryPlan(
            [this](bool ok, const std::string &responseMessage) {
                const QString message = QString::fromStdString(responseMessage);
                QMetaObject::invokeMethod(this, [this, ok, message]() {
                    if (ok) {
                        LOG_INFO("RobotArm", QStringLiteral("完整机械臂轨迹碰撞检查完成：%1").arg(message));
                        if (tip_manager)
                            tip_manager->showTip(
                                QStringLiteral("机械臂审核轨迹已生成，请按 RViz 当前显示内容检查后再次点击"),
                                TipWidget::Succeed);
                    } else {
                        LOG_ERROR("RobotArm", QStringLiteral("入口轨迹规划失败：%1").arg(message));
                        if (tip_manager)
                            tip_manager->showTip(
                                QStringLiteral("完整轨迹规划或碰撞检查失败：%1").arg(message),
                                TipWidget::Dangerous);
                    }
                }, Qt::QueuedConnection);
            });
        if (!sent && tip_manager)
            tip_manager->showTip(QStringLiteral("入口规划服务未就绪"), TipWidget::Dangerous);
        return;
    }
    if (m_entryMotionState == EntryStatus::STATE_PLANNING ||
        m_entryMotionState == EntryStatus::STATE_EXECUTING) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("入口规划或运动正在进行，请等待"),
                                 TipWidget::Warning);
        return;
    }
    if (m_entryMotionState == EntryStatus::STATE_PREVIEW_READY) {
        if (!m_entryMotionEnabled) {
            const QString message = QStringLiteral(
                "入口规划已完成，但入口规划器 motion_enabled=false；当前只能在 RViz 预览");
            LOG_WARN("RobotArm", message);
            if (tip_manager)
                tip_manager->showTip(message, TipWidget::Dangerous);
            return;
        }
        if (!m_entryMotionPathPermitted) {
            const QString message = QStringLiteral(
                "入口规划已完成，但当前路径标定许可未通过，禁止执行到预入口");
            LOG_WARN("RobotArm", message);
            if (tip_manager)
                tip_manager->showTip(message, TipWidget::Dangerous);
            return;
        }
        const QString confirmation = QStringLiteral(
            "确认已在 RViz 检查连续主流程：当前位置到预入口、预入口到安全入口、全部连续喷涂层，以及完成后沿入口法向退出。轨迹不穿过机柜、患者或其他障碍物，现场无人且物理急停可触达。\n\n"
            "红色停止后的任意停车点退出将根据实际停车位姿另行重新规划。确认后机械臂仅移动到入口外法线方向 100 mm 的预入口点，不会进入腔体，也不会开启等离子。");
        if (QMessageBox::warning(this, QStringLiteral("执行预入口轨迹确认"), confirmation,
                                 QMessageBox::Ok | QMessageBox::Cancel,
                                 QMessageBox::Cancel) != QMessageBox::Ok) {
            return;
        }
        const bool sent = m_rosWorker->node()->callEntryExecute(
            [this](bool ok, const std::string &responseMessage) {
                const QString message = QString::fromStdString(responseMessage);
                QMetaObject::invokeMethod(this, [this, ok, message]() {
                    if (ok)
                        LOG_INFO("RobotArm", QStringLiteral("已到预入口：%1").arg(message));
                    else
                        LOG_ERROR("RobotArm", QStringLiteral("预入口运动失败：%1").arg(message));
                    if (tip_manager)
                        tip_manager->showTip(
                            ok ? QStringLiteral("机械臂已到预入口，请再次点击确认分段进入")
                               : QStringLiteral("预入口运动失败：%1").arg(message),
                            ok ? TipWidget::Succeed : TipWidget::Dangerous);
                }, Qt::QueuedConnection);
            });
        if (!sent && tip_manager)
            tip_manager->showTip(QStringLiteral("预入口执行服务未就绪"), TipWidget::Dangerous);
        return;
    }
    if (m_entryMotionState == EntryStatus::STATE_COMPLETED) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("全部喷涂层已完成，请点击退出腔体（等离子输出关闭）"),
                                 TipWidget::Succeed);
        return;
    }
    if (m_entryMotionState == EntryStatus::STATE_AT_PRE_ENTRY) {
        if (!m_rosWorker->node()->entryReviewedEntryReady()) {
            if (tip_manager)
                tip_manager->showTip(QStringLiteral("安全入口执行服务未就绪"),
                                     TipWidget::Dangerous);
            return;
        }
        const double standoffMm = data->plannedEntryTipStandoff * 1000.0;
        const QString confirmation = QStringLiteral(
            "机械臂已经验证到达预入口。下一段只沿入口法向移动到安全入口，然后强制停车，不会继续执行喷涂路径，也不会开启等离子。\n\n"
            "安全入口的目标是让圆头停在视觉物理开口外 %1 mm。确认入口轴向无遮挡、现场无人且物理急停可触达。")
            .arg(standoffMm, 0, 'f', 1);
        if (QMessageBox::warning(this, QStringLiteral("执行安全入口轨迹确认"), confirmation,
                                 QMessageBox::Ok | QMessageBox::Cancel,
                                 QMessageBox::Cancel) != QMessageBox::Ok) {
            return;
        }
        const bool sent = m_rosWorker->node()->callEntryReviewedEntry(
            [this, standoffMm](bool ok, const std::string &responseMessage) {
                const QString message = QString::fromStdString(responseMessage);
                QMetaObject::invokeMethod(this, [this, ok, message, standoffMm]() {
                    if (ok)
                        LOG_INFO("RobotArm", QStringLiteral("已到安全入口：%1").arg(message));
                    else
                        LOG_ERROR("RobotArm", QStringLiteral("安全入口运动失败：%1").arg(message));
                    if (tip_manager) {
                        tip_manager->showTip(
                            ok ? QStringLiteral("已停在安全入口，请确认圆头仍在开口外 %1 mm 后再次点击")
                                     .arg(standoffMm, 0, 'f', 1)
                               : QStringLiteral("安全入口运动失败：%1").arg(message),
                            ok ? TipWidget::Succeed : TipWidget::Dangerous);
                    }
                }, Qt::QueuedConnection);
            });
        if (!sent && tip_manager)
            tip_manager->showTip(QStringLiteral("安全入口执行服务已断开"),
                                 TipWidget::Dangerous);
        return;
    }
    if (m_entryMotionState == EntryStatus::STATE_AT_ENTRY) {
        if (!m_rosWorker->node()->entryReviewedFirstLayerReady()) {
            if (tip_manager)
                tip_manager->showTip(QStringLiteral("连续喷涂轨迹执行服务未就绪"),
                                     TipWidget::Dangerous);
            return;
        }
        const double standoffMm = data->plannedEntryTipStandoff * 1000.0;
        const QString confirmation = QStringLiteral(
            "机械臂已停在安全入口。请现场确认圆头仍位于真实开口外，名义外停距离为 %1 mm；如果圆头已进入开口或接近底部，请取消并停止。\n\n"
            "确认后将按已审核轨迹连续执行全部喷涂层，不再在第一层要求二次确认。真实等离子保持关闭；运动期间可随时按红色停止按钮，完全停稳后再点击安全退出。")
            .arg(standoffMm, 0, 'f', 1);
        if (QMessageBox::warning(this, QStringLiteral("连续执行全部喷涂层"), confirmation,
                                 QMessageBox::Ok | QMessageBox::Cancel,
                                 QMessageBox::Cancel) != QMessageBox::Ok) {
            return;
        }

        const bool sent = m_rosWorker->node()->callEntryReviewedFirstLayer(
            [this](bool accepted, const std::string &responseMessage) {
                const QString message = QString::fromStdString(responseMessage);
                QMetaObject::invokeMethod(this, [this, accepted, message]() {
                    if (accepted) {
                        LOG_INFO("RobotArm", QStringLiteral("全部腔内审核轨迹执行完成：%1").arg(message));
                        if (tip_manager)
                            tip_manager->showTip(
                                QStringLiteral("全部喷涂层已完成，请点击退出腔体"),
                                TipWidget::Succeed);
                    } else {
                        LOG_ERROR("RobotArm", QStringLiteral("腔内审核轨迹执行失败或已停止：%1").arg(message));
                        if (tip_manager)
                            tip_manager->showTip(
                                QStringLiteral("腔内轨迹已停止；等待状态更新后可安全退出：%1").arg(message),
                                TipWidget::Dangerous);
                    }
                    updateRobotExecutionSummary();
                }, Qt::QueuedConnection);
            });
        if (!sent && tip_manager)
            tip_manager->showTip(QStringLiteral("连续喷涂轨迹执行服务已断开"),
                                 TipWidget::Dangerous);
        return;
    }
    if (m_entryMotionState == EntryStatus::STATE_AT_FIRST_LAYER) {
        if (tip_manager)
            tip_manager->showTip(
                QStringLiteral("检测到旧版第一层停车状态；请直接安全退出并重新规划，不能继续旧轨迹"),
                TipWidget::Warning);
        return;
    }
    if (tip_manager)
        tip_manager->showTip(QStringLiteral("入口规划器尚未到达可执行状态"),
                             TipWidget::Warning);
    return;

}

// =============================================
//  initRosLaunchManager - 创建并注册 ROS Launch
// =============================================
void MainWindow::initRosLaunchManager()
{
    m_rosLaunchManager = new RosLaunchManager(this);
    connect(m_rosLaunchManager, &RosLaunchManager::launchOutput,
            this, &MainWindow::onLaunchOutput);

    // TODO: 根据实际 ROS2 包名和 launch 文件名修改
    m_rosLaunchManager->registerLaunch(
        "camera",
        "plasma_camera_bridge",
        "plasma_l515.launch.py"
    );
    m_rosLaunchManager->registerLaunch(
        "robot",
        "rm_driver",
        "rm_eco65_driver.launch.py"
    );
    m_rosLaunchManager->registerLaunch(
        "path_transform",
        "plasma_path_transform",
        "plasma_path_transform.launch.py"
    );
    m_rosLaunchManager->registerLaunch(
        "path_executor",
        "plasma_path_executor",
        "plasma_path_executor.launch.py"
    );
    m_rosLaunchManager->registerLaunch(
        "entry_motion",
        "plasma_path_executor",
        "automatic_entry_motion.launch.py"
    );
    QSettings settings;
    const int legacySpraySpeed = settings.value(
        QStringLiteral("robot/motion_speed_percent"), 5).toInt();
    m_armSpraySpeedPercent = settings.value(
        QStringLiteral("robot/spray_speed_percent"), legacySpraySpeed).toInt();
    if (m_armSpraySpeedPercent != 5 && m_armSpraySpeedPercent != 10)
        m_armSpraySpeedPercent = 5;
    m_armApproachSpeedPercent = settings.value(
        QStringLiteral("robot/approach_speed_percent"),
        m_armSpraySpeedPercent * 2).toInt();
    if (m_armApproachSpeedPercent != 10 && m_armApproachSpeedPercent != 20)
        m_armApproachSpeedPercent = 10;

    m_rosLaunchManager->setParam("path_executor", "motion_enabled", "false");
    m_rosLaunchManager->setParam("path_executor", "entry_approach_enabled", "true");
    // The complete camera-derived path measured about 30 mm too deep on
    // 2026-07-30. Keep this explicit and separate from hand-eye/TCP geometry.
    m_rosLaunchManager->setParam(
        "path_transform", "cavity_axis_outward_compensation_m", "0.030");
    // The reviewed entrance is accurate, but the first and deeper spray layers
    // measured too far inward. Shift only spray poses outward; keep pre-entry
    // and safe-entry geometry unchanged.
    m_rosLaunchManager->setParam(
        "path_transform", "spray_axis_outward_compensation_m", "0.020");
    // The GUI executes only the MoveIt-reviewed dry-run trajectory.  Calibration
    // acceptance flags remain truthful; this explicit commissioning override is
    // what permits the user-approved provisional hand-eye/TCP data to move at the
    // explicitly selected low-speed commissioning setting.
    m_rosLaunchManager->setParam("entry_motion", "motion_enabled", "true");
    m_rosLaunchManager->setParam("entry_motion", "allow_unvalidated_dry_run", "true");
    m_rosLaunchManager->setParam("entry_motion", "rviz", "true");
    m_rosLaunchManager->setParam("entry_motion", "start_rm_control", "true");
    m_rosLaunchManager->setParam("entry_motion", "joint_state_wait_sec", "15.0");
    applyArmMotionSpeedLaunchParams();
    LOG_WARN(
        "RobotArm",
        QStringLiteral(
            "实机干运行模式已启用：仅执行 MoveIt 审核轨迹，腔内喷涂/退出速度 %1%，"
            "当前位置到安全入口移动速度 %2%，整条轨迹沿腔体轴外向补偿 30 mm，"
            "喷涂层在入口不变的前提下额外外移 20 mm，"
            "等离子输出保持关闭；预入口和安全入口分段确认，进入后连续执行全部喷涂层；"
            "红色停止后停稳即可从当前姿态安全退出")
            .arg(m_armSpraySpeedPercent)
            .arg(m_armApproachSpeedPercent));
    const QString sourceRefinementYaml =
        qEnvironmentVariable("PLASMA_SOURCE_REFINEMENT_YAML");
    if (!sourceRefinementYaml.isEmpty()) {
        m_rosLaunchManager->setParam(
            "path_transform", "source_refinement_yaml", sourceRefinementYaml);
        LOG_INFO("RobotArm",
                 QStringLiteral("路径转换器已加载盲测补偿：%1；真实运动许可保持关闭")
                     .arg(sourceRefinementYaml));
    }
    m_rosLaunchManager->start("path_transform");
    m_rosLaunchManager->start("path_executor");
}

void MainWindow::applyArmMotionSpeedLaunchParams()
{
    if (!m_rosLaunchManager)
        return;

    const QString spraySpeedPercent = QString::number(m_armSpraySpeedPercent);
    const QString sprayScaling = QString::number(
        static_cast<double>(m_armSpraySpeedPercent) / 100.0, 'f', 2);
    const QString approachScaling = QString::number(
        static_cast<double>(m_armApproachSpeedPercent) / 100.0, 'f', 2);
    m_rosLaunchManager->setParam(
        "path_executor", "speed_percent", spraySpeedPercent);
    m_rosLaunchManager->setParam(
        "entry_motion", "velocity_scaling", sprayScaling);
    m_rosLaunchManager->setParam(
        "entry_motion", "acceleration_scaling", sprayScaling);
    m_rosLaunchManager->setParam(
        "entry_motion", "approach_velocity_scaling", approachScaling);
    m_rosLaunchManager->setParam(
        "entry_motion", "approach_acceleration_scaling", approachScaling);
    m_rosLaunchManager->setParam(
        "entry_motion", "final_settle_speed_percent", spraySpeedPercent);
}

// =============================================
//  initSystemCheckManager - 创建自检管理器并连接信号
// =============================================
void MainWindow::initSystemCheckManager()
{
    m_systemCheckManager = new SystemCheckManager(
        m_rosLaunchManager, m_armHealthModel, this);

    // 自检开始：主日志只保留高层进度，不打印启动命令和底层检测细节
    connect(m_systemCheckManager, &SystemCheckManager::checkStarted,
            this, []() {
                LOG_INFO("System", "开始系统自检...");
            });

    // 每项检查完成：只输出用户可读状态，底层 ros2 node list 细节由 RosLaunchManager 静默处理
    connect(m_systemCheckManager, &SystemCheckManager::checkItemUpdated,
            this, [this](const QString &itemName, bool ok, const QString &message) {
                const QString logText = QString("%1：%2").arg(itemName, message);
                if (ok || message.contains(QStringLiteral("正在"))) {
                    LOG_INFO(logText);
                } else {
                    LOG_ERROR(logText);
                }

                if (itemName == QStringLiteral("相机")) {
                    if (ok) {
                        ui->CameraStatus->setStatus(StateBtn::Connected, true);
                        if (m_rosWorker && m_rosWorker->node()) {
                            const int selectedIndex = ui->cameraModeComboBox
                                ? ui->cameraModeComboBox->currentIndex()
                                : 6;
                            const int visualPreset = selectedIndex == 6
                                ? 5
                                : std::clamp(selectedIndex, 0, 5);
                            m_rosWorker->node()->applyCameraPreset(
                                visualPreset,
                                [this, selectedIndex, visualPreset](
                                    bool applied, const std::string &reason) {
                                    QMetaObject::invokeMethod(
                                        this,
                                        [this, applied, selectedIndex,
                                         visualPreset, reason]() {
                                            const QString mode = selectedIndex == 6
                                                ? QStringLiteral("自动")
                                                : ui->cameraModeComboBox->itemText(
                                                      selectedIndex);
                                            const QString detail = QStringLiteral(
                                                "%1模式已应用预设 %2：%3")
                                                .arg(mode)
                                                .arg(visualPreset)
                                                .arg(QString::fromStdString(reason));
                                            if (applied) {
                                                appendCameraLog(detail);
                                                LOG_INFO("Camera", detail);
                                            } else {
                                                appendCameraLog(
                                                    QStringLiteral("相机预设应用失败：%1")
                                                        .arg(detail));
                                                LOG_ERROR("Camera", detail);
                                            }
                                        },
                                        Qt::QueuedConnection);
                                });
                        }
                    } else if (!message.contains(QStringLiteral("正在自动启动"))) {
                        ui->CameraStatus->setStatus(StateBtn::Disconnected, true);
                    }
                }
                else if (itemName == QStringLiteral("机械臂") && m_armStatusWidget)
                {
                    m_armStatusWidget->setSelfCheckStatus(message);
                }
            });

    connect(m_systemCheckManager, &SystemCheckManager::checkFinished,
            this, &MainWindow::onSystemCheckFinished);
}


// =============================================
//  系统自检信号槽
// =============================================
void MainWindow::onRequestSystemCheck()
{
    if (!m_systemCheckManager || !m_rosLaunchManager)
    {
        LOG_WARN("MainWindow", "m_systemCheckManager 或 m_rosLaunchManager 为空，跳过自检");
        return;
    }

    //1、加载launch包启动参数
    m_rosLaunchManager->setParam("camera", "pointcloud__cuda_.enable", "true");
    m_rosLaunchManager->setParam("camera", "align_depth.enable",       "false");

    // Index 6 is the application-level Auto mode. The fixed close-range
    // surgical workspace uses the L515 Short Range preset in that mode.
    if (ui->cameraModeComboBox)
    {
        const int selectedIndex = ui->cameraModeComboBox->currentIndex();
        const int visualPreset = selectedIndex == 6
            ? 5
            : std::clamp(selectedIndex, 0, 5);
        m_rosLaunchManager->setParam(
            "camera", "depth_module.visual_preset",
            QString::number(visualPreset));
    }

    //逐个安全读取 combo box 参数
    if (ui->depthResComboBox && ui->depthFpsComboBox)
    {
        QStringList tokens = ui->depthResComboBox->currentText().split(" ");
        const QString fps = ui->depthFpsComboBox->currentText().split(" ").value(0);
        m_rosLaunchManager->setParam(
            "camera", "depth_module.profile",
            QStringLiteral("%1,%2,%3").arg(tokens.value(0), tokens.value(2), fps));
    }
    if (ui->rgbResComboBox && ui->rgbFpsComboBox)
    {
        QStringList tokens = ui->rgbResComboBox->currentText().split(" ");
        const QString fps = ui->rgbFpsComboBox->currentText().split(" ").value(0);
        m_rosLaunchManager->setParam(
            "camera", "rgb_camera.profile",
            QStringLiteral("%1,%2,%3").arg(tokens.value(0), tokens.value(2), fps));
    }

    updateCameraParamView();
    m_systemCheckManager->setSkipRobotCheck(
        m_stepWizardModel && m_stepWizardModel->debugMode());
    m_systemCheckManager->startCheck();
}

void MainWindow::onSystemCheckFinished(bool ok, const QString &message)
{
    if (!m_stepWizardModel) return;

    m_stepWizardModel->setStepFinished(WizardStep::SystemCheck, ok);

    if (ok)
    {
        if (m_stepWizardView)
            m_stepWizardView->setSystemCheckButtonState(StepWizardView::SystemCheckButtonState::Success);
        LOG_INFO("System", message);
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("✅ 系统自检完成"), TipWidget::Succeed);
    }
    else
    {
        if (m_stepWizardView)
            m_stepWizardView->setSystemCheckButtonState(StepWizardView::SystemCheckButtonState::Ready);
        LOG_ERROR("System", message);
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("❌ 系统自检失败，请检查设备连接"), TipWidget::Dangerous);
        QMessageBox::warning(this, QStringLiteral("系统自检失败"), message);
    }
}

void MainWindow::onRequestCloudCapture()
{
    m_cloudCapturePaused = !m_cloudCapturePaused;

    if (ui->main_gl)
        ui->main_gl->SetRosPaused(m_cloudCapturePaused);

    if (m_cloudCapturePaused)
    {
        m_pendingCaptureHeader = m_latestCloudHeader;
        m_pendingCaptureJointState = m_latestArmJointState;
        m_hasPendingCaptureJointState =
            m_pendingCaptureJointState.position.size() >= 6 &&
            m_armHealthModel && m_armHealthModel->dataActive() &&
            !m_armHealthModel->isDataStale(kArmHeartbeatTimeoutMs);
        m_cloudCaptureCropped = false;
        ui->btnCrop->setEnabled(true);
        if (m_stepWizardView)
            m_stepWizardView->setCloudCaptureConfirmEnabled(false);
        LOG_INFO("残腔采集：已暂停实时点云，可开始裁剪。");
        if (m_pendingCaptureHeader.frame_id.empty())
            LOG_WARN("残腔采集：当前点云缺少 frame_id，后续无法进行机械臂坐标转换。");
        if (!m_hasPendingCaptureJointState)
            LOG_WARN("残腔采集：没有冻结到有效六关节状态，后续坐标转换将被拒绝。");
    }
    else
    {
        m_cloudCaptureCropped = false;
        ui->btnCrop->setChecked(false);
        ui->btnCrop->setEnabled(false);
        if (m_stepWizardView)
            m_stepWizardView->setCloudCaptureConfirmEnabled(false);
        LOG_INFO("残腔采集：已恢复实时点云显示。");
    }
}

void MainWindow::onRequestCloudRebuild(std::shared_ptr<NodeCloudData> cloudData)
{
    if (currentStepIndex != static_cast<int>(WizardStep::CloudRebuild))
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先进入残腔重建流程"), TipWidget::Warning);
        return;
    }

    if (!cloudData || !cloudData->polyData)
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("当前关键帧没有可重建的点云数据"), TipWidget::Warning);
        return;
    }

    const QString sourceName = cloudData->displayName.isEmpty()
        ? QFileInfo(cloudData->filePath).completeBaseName()
        : cloudData->displayName;
    if (sourceName.isEmpty())
        return;

    const QString outputDir = QStringLiteral("/home/larusxu/CodeSpace/PlasmaRobot/src/plasma_gui/src/gui/ui_source/rebuild_mesh");
    QDir().mkpath(outputDir);

    m_rebuildSources[sourceName] = cloudData;
    if (m_stepWizardView)
        m_stepWizardView->setRebuildProgress(0);

    LOG_INFO(QStringLiteral("残腔重建：开始重建 %1").arg(sourceName));
    if (ui->main_gl)
        ui->main_gl->requestCloudRebuild(cloudData, outputDir);
}

void MainWindow::onRequestCloudRecrop(std::shared_ptr<NodeCloudData> cloudData)
{
    if (currentStepIndex != static_cast<int>(WizardStep::CloudRebuild))
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先进入残腔重建流程"), TipWidget::Warning);
        return;
    }

    if (!cloudData || !cloudData->polyData)
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("当前关键帧没有可裁剪的点云数据"), TipWidget::Warning);
        return;
    }

    m_recropTarget = cloudData;
    m_cloudCapturePaused = true;
    if (ui->main_gl)
        ui->main_gl->loadCloudForCropping(cloudData);

    ui->btnCrop->setEnabled(true);
    ui->btnCrop->setChecked(true);
    LOG_INFO(QStringLiteral("残腔重建：正在重新裁剪 %1，点击裁剪工具条的勾以更新关键帧，点击 x 放弃。")
        .arg(cloudData->displayName));
}

void MainWindow::onRequestPathPlanning(std::shared_ptr<NodeCloudData> cloudData)
{
    if (currentStepIndex != static_cast<int>(WizardStep::PathPlanning))
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先进入路径规划流程"), TipWidget::Warning);
        return;
    }

    if (!m_pathParamsConfigured)
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先配置路径参数！"), TipWidget::Warning);
        LOG_INFO(QStringLiteral("路径规划：请先配置路径参数！"));
        return;
    }

    if (!cloudData || !cloudData->surgicalMesh || cloudData->surgicalMesh->GetNumberOfPoints() == 0)
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("当前重建帧还没有完成术区选择"), TipWidget::Warning);
        return;
    }

    const QString nodeName = cloudData ? cloudData->displayName : QString();
    const QString sourceName = nodeName.isEmpty() ? QStringLiteral("未命名节点") : nodeName;
    m_pathPlanningSources[sourceName] = cloudData;
    if (m_stepWizardView)
        m_stepWizardView->setPathPlanningProgress(0);

    LOG_INFO(QStringLiteral("路径规划：开始处理重建帧 %1，采样点数=%2，喷口 TCP=%3mm，喷口后向占用=%4mm，圆头前向占用=%5mm")
        .arg(sourceName)
        .arg(m_pathSampleCount)
        .arg(m_toolTcpOffset * 1000.0, 0, 'f', 1)
        .arg(m_toolOutletRearExtent * 1000.0, 0, 'f', 1)
        .arg(m_toolSafetyForwardExtent * 1000.0, 0, 'f', 1));
    if (ui->main_gl)
        ui->main_gl->requestPathPlanning(cloudData, m_pathSampleCount, m_pathRodType,
                                         m_pathVoxelSize, m_pathMedialPercentile,
                                         m_pathSectionCount, m_pathSectionHalfWidth,
                                         m_pathSmoothPointsNum, m_pathSmoothWindow,
                                         m_pathExcludeOpeningDistance, m_pathExcludeOpeningLayers,
                                         m_toolOutletRearExtent, m_toolSafetyForwardExtent,
                                         m_sliceSafeMarginBottom, m_sliceSafeMarginTop);
}

void MainWindow::showPathParamDialog()
{
    QDialog dialog(this);
    dialog.setWindowTitle(QStringLiteral("路径参数配置"));
    dialog.setModal(true);

    auto *layout = new QVBoxLayout(&dialog);
    auto *formContainer = new QWidget(&dialog);
    auto *groupLayout = new QVBoxLayout(formContainer);

    auto *levelCombo = new QComboBox(&dialog);
    levelCombo->addItem(QStringLiteral("1级"), 5000);
    levelCombo->addItem(QStringLiteral("2级"), 10000);
    levelCombo->addItem(QStringLiteral("3级"), 15000);
    const int currentLevel = levelCombo->findData(m_pathSampleCount);
    if (currentLevel >= 0)
        levelCombo->setCurrentIndex(currentLevel);

    auto *rodLengthCombo = new QComboBox(&dialog);
    rodLengthCombo->addItem(QStringLiteral("150 mm"), 150);
    rodLengthCombo->addItem(QStringLiteral("200 mm"), 200);
    const int currentRodLength = rodLengthCombo->findData(m_toolRodLengthMm);
    rodLengthCombo->setCurrentIndex(currentRodLength >= 0 ? currentRodLength : 0);

    auto *nozzleCombo = new QComboBox(&dialog);
    nozzleCombo->addItem(QStringLiteral("1 x 5 mm"), 5);
    nozzleCombo->addItem(QStringLiteral("1 x 10 mm"), 10);
    nozzleCombo->addItem(QStringLiteral("1 x 20 mm"), 20);
    const int currentNozzle = nozzleCombo->findData(m_toolNozzleWidthMm);
    nozzleCombo->setCurrentIndex(currentNozzle >= 0 ? currentNozzle : 0);

    auto *joint6SweepSpin = new QDoubleSpinBox(&dialog);
    joint6SweepSpin->setRange(30.0, 180.0);
    joint6SweepSpin->setDecimals(1);
    joint6SweepSpin->setSingleStep(10.0);
    joint6SweepSpin->setSuffix(QStringLiteral("°"));
    joint6SweepSpin->setValue(m_continuousMaxJoint6SweepDeg);

    auto makeSafetySpin = [&dialog](double valueMeters) {
        auto *spin = new QDoubleSpinBox(&dialog);
        spin->setRange(0.5, 50.0);
        spin->setDecimals(1);
        spin->setSingleStep(0.5);
        spin->setSuffix(QStringLiteral(" mm"));
        spin->setValue(valueMeters * 1000.0);
        return spin;
    };
    auto *bottomSafetySpin = makeSafetySpin(m_sliceSafeMarginBottom);
    auto *openingSafetySpin = makeSafetySpin(m_sliceSafeMarginTop);
    auto *entryTipStandoffSpin = makeSafetySpin(m_entryTipStandoff);
    entryTipStandoffSpin->setRange(0.0, 150.0);
    entryTipStandoffSpin->setSingleStep(5.0);

    auto *transitionDistanceSpin = new QDoubleSpinBox(&dialog);
    transitionDistanceSpin->setRange(1.0, 100.0);
    transitionDistanceSpin->setDecimals(1);
    transitionDistanceSpin->setSingleStep(1.0);
    transitionDistanceSpin->setSuffix(QStringLiteral(" mm"));
    transitionDistanceSpin->setValue(m_continuousMaxTransitionDistance * 1000.0);

    auto *layerOrderCombo = new QComboBox(&dialog);
    layerOrderCombo->addItem(QStringLiteral("入口 → 底部（推荐）"), false);
    layerOrderCombo->addItem(QStringLiteral("底部 → 入口"), true);
    layerOrderCombo->setCurrentIndex(m_continuousReverseLayerOrder ? 1 : 0);

    auto *sliceSpacingValue = new QLabel(&dialog);
    auto *nozzleAxialSizeValue = new QLabel(&dialog);
    auto *pathRadiusValue = new QLabel(&dialog);
    auto *toolTcpOffsetValue = new QLabel(&dialog);
    auto *executionCompatibilityValue = new QLabel(&dialog);
    auto updateNozzleGeometry = [this, rodLengthCombo, nozzleCombo,
                                 sliceSpacingValue, nozzleAxialSizeValue,
                                 pathRadiusValue,
                                 toolTcpOffsetValue,
                                 executionCompatibilityValue](int) {
        const int rodLengthMm = rodLengthCombo->currentData().toInt();
        const int nozzleSizeMm = nozzleCombo->currentData().toInt();
        const double nominalLength = nominalMotionTcpOffsetMeters(
            rodLengthMm, nozzleSizeMm);
        const double flangeToTcpMeters = isActiveDemoTool(rodLengthMm, nozzleSizeMm)
            ? m_configuredToolTcpOffset : nominalLength;
        sliceSpacingValue->setText(QStringLiteral("%1 mm").arg(nozzleSizeMm));
        nozzleAxialSizeValue->setText(QStringLiteral("%1 mm").arg(nozzleSizeMm));
        pathRadiusValue->setText(QStringLiteral("%1 mm")
            .arg(kActiveShaftRadiusMeters * 1000.0, 0, 'f', 1));
        toolTcpOffsetValue->setText(QStringLiteral("%1 mm")
            .arg(flangeToTcpMeters * 1000.0, 0, 'f', 1));
        executionCompatibilityValue->setText(
            isActiveDemoTool(rodLengthMm, nozzleSizeMm)
                ? QStringLiteral("当前末端参数匹配")
                : QStringLiteral("仅路径预览，实机模型未同步"));
    };
    connect(rodLengthCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            &dialog, updateNozzleGeometry);
    connect(nozzleCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            &dialog, updateNozzleGeometry);
    updateNozzleGeometry(nozzleCombo->currentIndex());

    auto makeGroup = [formContainer, groupLayout](const QString &title) {
        auto *group = new QGroupBox(title, formContainer);
        auto *form = new QFormLayout(group);
        form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        groupLayout->addWidget(group);
        return form;
    };

    auto *basicForm = makeGroup(QStringLiteral("术区与喷嘴"));
    basicForm->addRow(QStringLiteral("术区处理等级"), levelCombo);
    basicForm->addRow(QStringLiteral("喷杆标称长度"), rodLengthCombo);
    basicForm->addRow(QStringLiteral("喷嘴端部规格"), nozzleCombo);

    auto *sliceForm = makeGroup(QStringLiteral("切片范围"));
    sliceForm->addRow(QStringLiteral("切片间距"), sliceSpacingValue);
    sliceForm->addRow(QStringLiteral("圆头末端-底部额外距离"), bottomSafetySpin);
    sliceForm->addRow(QStringLiteral("喷口-开口额外距离"), openingSafetySpin);

    auto *nozzleForm = makeGroup(QStringLiteral("喷嘴路径"));
    nozzleForm->addRow(QStringLiteral("侧喷口轴向长度"), nozzleAxialSizeValue);
    nozzleForm->addRow(QStringLiteral("喷杆半径"), pathRadiusValue);
    nozzleForm->addRow(QStringLiteral("法兰到运动 TCP"), toolTcpOffsetValue);
    nozzleForm->addRow(QStringLiteral("喷杆深入轴"), new QLabel(QStringLiteral("plasma_motion_tcp 局部 +X"), &dialog));
    nozzleForm->addRow(QStringLiteral("实机执行状态"), executionCompatibilityValue);

    auto *executionForm = makeGroup(QStringLiteral("连续执行"));
    executionForm->addRow(QStringLiteral("圆头入口外停距离"), entryTipStandoffSpin);
    executionForm->addRow(QStringLiteral("关节6单段最大转角"), joint6SweepSpin);
    executionForm->addRow(QStringLiteral("最大层间连接距离"), transitionDistanceSpin);
    executionForm->addRow(QStringLiteral("喷涂层执行顺序"), layerOrderCombo);

    groupLayout->addStretch();
    layout->addWidget(formContainer);

    auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(buttons);
    dialog.resize(540, 740);

    if (dialog.exec() != QDialog::Accepted)
        return;

    m_pathSampleCount = levelCombo->currentData().toInt();
    m_toolRodLengthMm = rodLengthCombo->currentData().toInt();
    m_toolNozzleWidthMm = nozzleCombo->currentData().toInt();
    m_pathRodType = plasmaToolVariant(
        m_toolAdapterType, m_toolRodLengthMm, m_toolNozzleWidthMm);
    const double nozzleSizeMeters = m_toolNozzleWidthMm / 1000.0;
    const double nominalToolTcpOffset = nominalMotionTcpOffsetMeters(
        m_toolRodLengthMm, m_toolNozzleWidthMm);
    m_toolTcpOffset = isActiveDemoTool(m_toolRodLengthMm, m_toolNozzleWidthMm)
        ? m_configuredToolTcpOffset : nominalToolTcpOffset;
    m_toolSafetyTipOffset = nominalSafetyTipOffsetMeters(m_toolRodLengthMm);
    m_toolSafetyForwardExtent = nominalSafetyForwardExtentMeters(m_toolNozzleWidthMm);
    m_toolOutletRearExtent = 0.5 * nozzleSizeMeters;
    m_sliceSpacing = nozzleSizeMeters;
    m_equalDoseNozzleLength = nozzleSizeMeters;
    m_equalDoseRodRadius = kActiveShaftRadiusMeters;
    m_sliceSafeMarginBottom = bottomSafetySpin->value() / 1000.0;
    m_sliceSafeMarginTop = openingSafetySpin->value() / 1000.0;
    m_entryTipStandoff = entryTipStandoffSpin->value() / 1000.0;
    m_nozzleSafetyClearance = qMin(m_sliceSafeMarginBottom, m_sliceSafeMarginTop);
    m_continuousMaxJoint6SweepDeg = joint6SweepSpin->value();
    m_continuousMaxTransitionDistance = transitionDistanceSpin->value() / 1000.0;
    m_continuousReverseLayerOrder = layerOrderCombo->currentData().toBool();
    m_pathParamsConfigured = true;
    resetRobotExecutionConfirmation();
    if (m_stepWizardModel) {
        m_stepWizardModel->setStepFinished(WizardStep::PathPlanning, false);
        m_stepWizardModel->setStepFinished(WizardStep::RobotExecution, false);
    }
    if (m_stepWizardView)
        m_stepWizardView->setPathPlanningProgress(0);
    LOG_INFO(QStringLiteral("路径参数：术区处理等级=%1，采样点数=%2，运动 TCP=%3mm")
        .arg(levelCombo->currentText())
        .arg(m_pathSampleCount)
        .arg(m_toolTcpOffset * 1000.0, 0, 'f', 1));
    LOG_INFO(QStringLiteral("切片范围：间距=%1mm，底部安全距离=%2mm，开口安全距离=%3mm")
        .arg(m_sliceSpacing * 1000.0, 0, 'f', 1)
        .arg(m_sliceSafeMarginBottom * 1000.0, 0, 'f', 1)
        .arg(m_sliceSafeMarginTop * 1000.0, 0, 'f', 1));
    LOG_INFO(QStringLiteral("喷嘴路径：侧喷口轴向长度=%1mm，喷杆半径=%2mm，喷口 TCP=%3mm，圆头前向占用=%4mm，深入轴=plasma_motion_tcp +X，径向检查间隙=%5mm")
        .arg(m_equalDoseNozzleLength * 1000.0, 0, 'f', 1)
        .arg(m_equalDoseRodRadius * 1000.0, 0, 'f', 1)
        .arg(m_toolTcpOffset * 1000.0, 0, 'f', 1)
        .arg(m_toolSafetyForwardExtent * 1000.0, 0, 'f', 1)
        .arg(m_nozzleSafetyClearance * 1000.0, 0, 'f', 1));
    LOG_INFO(QStringLiteral("连续执行：圆头入口外停=%1mm，关节6单段限制=%2度，最大层间连接=%3mm，切片顺序=%4")
        .arg(m_entryTipStandoff * 1000.0, 0, 'f', 1)
        .arg(m_continuousMaxJoint6SweepDeg)
        .arg(m_continuousMaxTransitionDistance * 1000.0, 0, 'f', 1)
        .arg(m_continuousReverseLayerOrder
             ? QStringLiteral("底部到入口")
             : QStringLiteral("入口到底部")));
    if (tip_manager)
        tip_manager->showTip(QStringLiteral("路径参数已保存，重新执行路径规划后生效"), TipWidget::Succeed);
}

void MainWindow::onPathPlanningProgress(const QString &sourceName, int progress)
{
    if (m_stepWizardView)
        m_stepWizardView->setPathPlanningProgress(progress);
    LOG_INFO(QStringLiteral("路径规划：%1 进度 %2%").arg(sourceName).arg(progress));
}

void MainWindow::onPathPlanningLog(const QString &sourceName, const QString &message)
{
    LOG_INFO(QStringLiteral("路径规划：[%1] %2").arg(sourceName, message));
}

void MainWindow::onPathPlanningFinished(const QString &sourceName,
                                        vtkSmartPointer<vtkPolyData> sampledCloud,
                                        vtkSmartPointer<vtkPolyData> curvedAxis,
                                        vtkSmartPointer<vtkPolyData> straightAxis,
                                        bool ok,
                                        const QString &message)
{
    auto source = m_pathPlanningSources.take(sourceName);
    if (!ok || !source)
    {
        LOG_ERROR(QStringLiteral("路径规划：%1 失败，%2").arg(sourceName, message));
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("路径规划失败：%1").arg(sourceName), TipWidget::Dangerous);
        return;
    }

    auto pathData = std::make_shared<NodeCloudData>();
    pathData->filePath = source->filePath;
    pathData->meshFilePath = source->meshFilePath;
    pathData->polyData = source->polyData;
    pathData->meshPolyData = source->meshPolyData;
    pathData->surgicalPointCloud = source->surgicalPointCloud;
    pathData->surgicalMesh = source->surgicalMesh;
    pathData->pathSampleCloud = sampledCloud;
    pathData->curvedAxis = curvedAxis;
    pathData->straightAxis = straightAxis;
    pathData->showCurvedAxis = false;
    pathData->plannedNozzleLength = m_equalDoseNozzleLength;
    pathData->plannedOutletRearExtent = m_toolOutletRearExtent;
    pathData->plannedSafetyForwardExtent = m_toolSafetyForwardExtent;
    pathData->plannedShaftRadius = m_equalDoseRodRadius;
    pathData->plannedBottomSafetyMargin = m_sliceSafeMarginBottom;
    pathData->plannedOpeningSafetyMargin = m_sliceSafeMarginTop;
    pathData->plannedEntryTipStandoff = m_entryTipStandoff;
    pathData->plannedSliceSpacing = m_sliceSpacing;
    pathData->plannedToolVariant = m_pathRodType;
    pathData->plannedToolTcpOffset = m_toolTcpOffset;
    pathData->plannedSafetyTipOffset = m_toolSafetyTipOffset;
    pathData->plannedToolAxis = 0;
    pathData->sourceHeader = source->sourceHeader;
    pathData->captureJointState = source->captureJointState;
    pathData->hasCaptureJointState = source->hasCaptureJointState;
    pathData->surgicalMeshOpacity = 0.28;
    for (int i = 0; i < 3; ++i) {
        pathData->bboxMin[i] = source->bboxMin[i];
        pathData->bboxMax[i] = source->bboxMax[i];
        pathData->center[i] = source->center[i];
    }

    const QString pathNodeName = dbtree_view_ ? dbtree_view_->addTrajectoryPath(sourceName, pathData) : QString();
    if (pathNodeName.isEmpty())
    {
        LOG_ERROR(QStringLiteral("路径规划：%1 完成但写入轨迹生成节点失败。").arg(sourceName));
        return;
    }

    LOG_INFO(QStringLiteral("路径规划：%1 完成，已生成 %2，采样点=%3，弯曲线点=%4，直轴点=%5")
        .arg(sourceName, pathNodeName)
        .arg(sampledCloud ? sampledCloud->GetNumberOfPoints() : 0)
        .arg(curvedAxis ? curvedAxis->GetNumberOfPoints() : 0)
        .arg(straightAxis ? straightAxis->GetNumberOfPoints() : 0));
    if (tip_manager)
        tip_manager->showTip(QStringLiteral("路径规划参考轴已生成"), TipWidget::Succeed);
}

void MainWindow::onRequestFullTrajectory(std::shared_ptr<NodeCloudData> cloudData)
{
    if (currentStepIndex != static_cast<int>(WizardStep::PathPlanning)) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先进入路径规划流程"), TipWidget::Warning);
        return;
    }
    if (!m_pathParamsConfigured) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先点击 btn2 配置并保存路径参数"), TipWidget::Warning);
        return;
    }
    if (!cloudData || cloudData->displayName.isEmpty() ||
        !cloudData->straightAxis || cloudData->straightAxis->GetNumberOfPoints() < 2 ||
        !cloudData->surgicalMesh || cloudData->surgicalMesh->GetNumberOfPoints() == 0) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("当前轨迹节点缺少术区 mesh 或参考轴"), TipWidget::Warning);
        return;
    }

    const QString sourceName = cloudData->displayName;
    const bool taskBusy = m_fullTrajectoryStages.contains(sourceName) ||
        m_slicePlanningSources.contains(sourceName) ||
        m_sliceContourSources.contains(sourceName) ||
        m_contourFittingSources.contains(sourceName) ||
        m_equalDosePathSources.contains(sourceName) ||
        m_continuousPathSources.contains(sourceName) ||
        m_nozzlePoseSources.contains(sourceName);
    if (taskBusy) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("该轨迹节点正在处理，请等待当前任务完成"), TipWidget::Warning);
        return;
    }

    if (m_stepWizardModel) {
        m_stepWizardModel->setStepFinished(WizardStep::PathPlanning, false);
        m_stepWizardModel->setStepFinished(WizardStep::RobotExecution, false);
    }
    m_fullTrajectoryStages[sourceName] = FullSlicePlanning;
    cloudData->fullTrajectoryProcessing = true;
    updateFullTrajectoryProgress(sourceName, FullSlicePlanning, 0);
    LOG_INFO(QStringLiteral(
        "完整喷涂轨迹：开始处理 %1，将依次生成切片、轮廓、喷杆几何路径、分段路径和喷嘴位姿。")
        .arg(sourceName));
    onRequestSlicePlanning(cloudData);
}

void MainWindow::updateFullTrajectoryProgress(const QString &sourceName,
                                              int stage, int stageProgress)
{
    int progress = std::clamp(stageProgress, 0, 100);
    if (m_fullTrajectoryStages.value(sourceName, -1) == stage) {
        progress = std::clamp(
            (stage * 100 + progress) / static_cast<int>(FullTrajectoryStageCount),
            0, 100);
    }
    if (m_stepWizardView)
        m_stepWizardView->setPathPlanningProgress(progress);
}

void MainWindow::continueFullTrajectory(std::shared_ptr<NodeCloudData> source,
                                        int completedStage)
{
    if (!source || source->displayName.isEmpty())
        return;
    const QString sourceName = source->displayName;
    if (m_fullTrajectoryStages.value(sourceName, -1) != completedStage)
        return;

    const int nextStage = completedStage + 1;
    if (nextStage >= FullTrajectoryStageCount) {
        m_fullTrajectoryStages.remove(sourceName);
        source->fullTrajectoryProcessing = false;
        source->showSliceLayers = false;
        source->showFittedContours = false;
        source->showEqualDoseSurface = false;
        source->showSprayPath = false;
        source->showContinuousPath = false;
        source->showNozzlePoses = true;
        if (ui->main_gl) {
            ui->main_gl->setSliceLayersVisible(source, false);
            ui->main_gl->setFittedContoursVisible(source, false);
            ui->main_gl->setEqualDoseSurfaceVisible(source, false);
            ui->main_gl->setSprayPathVisible(source, false);
            ui->main_gl->setContinuousPathVisible(source, false);
            ui->main_gl->setNozzlePosesVisible(source, true);
        }
        if (m_stepWizardView)
            m_stepWizardView->setPathPlanningProgress(100);
        if (m_stepWizardModel) {
            m_stepWizardModel->setStepFinished(WizardStep::PathPlanning, true);
            m_stepWizardModel->setStepFinished(WizardStep::RobotExecution, false);
        }
        LOG_INFO(QStringLiteral(
            "完整喷涂轨迹：%1 全部生成完成，默认仅保留蓝色参考轴和喷嘴位姿。")
            .arg(sourceName));
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("完整喷涂轨迹已生成"), TipWidget::Succeed);
        return;
    }

    m_fullTrajectoryStages[sourceName] = nextStage;
    updateFullTrajectoryProgress(sourceName, nextStage, 0);
    bool launched = false;
    switch (nextStage) {
    case FullSliceContours:
        onRequestSliceContours(source);
        launched = m_sliceContourSources.contains(sourceName);
        break;
    case FullContourFitting:
        onRequestContourFitting(source);
        launched = m_contourFittingSources.contains(sourceName);
        break;
    case FullSprayGeometry:
        onRequestEqualDosePath(source);
        launched = m_equalDosePathSources.contains(sourceName);
        break;
    case FullContinuousPath:
        onRequestContinuousPath(source);
        launched = m_continuousPathSources.contains(sourceName);
        break;
    case FullNozzlePoses:
        onRequestNozzlePoses(source);
        launched = m_nozzlePoseSources.contains(sourceName);
        break;
    default:
        failFullTrajectory(source, nextStage, QStringLiteral("未知处理阶段"));
        return;
    }
    if (!launched)
        failFullTrajectory(source, nextStage, QStringLiteral("阶段启动条件不满足"));
}

void MainWindow::failFullTrajectory(std::shared_ptr<NodeCloudData> source, int failedStage,
                                    const QString &message)
{
    if (!source || source->displayName.isEmpty())
        return;
    const QString sourceName = source->displayName;
    if (m_fullTrajectoryStages.value(sourceName, -1) != failedStage)
        return;
    m_fullTrajectoryStages.remove(sourceName);
    source->fullTrajectoryProcessing = false;
    LOG_ERROR(QStringLiteral("完整喷涂轨迹：%1 在阶段 %2 中止，%3")
              .arg(sourceName).arg(failedStage + 1).arg(message));
}

void MainWindow::onRequestSlicePlanning(std::shared_ptr<NodeCloudData> cloudData)
{
    if (currentStepIndex != static_cast<int>(WizardStep::PathPlanning))
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先进入路径规划流程"), TipWidget::Warning);
        return;
    }
    if (!m_pathParamsConfigured)
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先配置路径参数！"), TipWidget::Warning);
        return;
    }
    if (!cloudData || !cloudData->straightAxis || cloudData->straightAxis->GetNumberOfPoints() < 2)
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("当前轨迹节点没有有效直参考轴"), TipWidget::Warning);
        return;
    }
    if (!cloudData->surgicalMesh || cloudData->surgicalMesh->GetNumberOfPoints() == 0)
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("当前轨迹节点没有有效术区，请先完成术区选择"), TipWidget::Warning);
        return;
    }

    const QString sourceName = cloudData->displayName;
    if (sourceName.isEmpty())
        return;
    m_slicePlanningSources[sourceName] = cloudData;
    updateFullTrajectoryProgress(sourceName, FullSlicePlanning, 0);

    LOG_INFO(QStringLiteral("切片规划：开始处理 %1，范围限定为已选择术区，间距=%2m，底部安全距离=%3m，开口安全距离=%4m")
        .arg(sourceName).arg(m_sliceSpacing).arg(m_sliceSafeMarginBottom).arg(m_sliceSafeMarginTop));
    if (ui->main_gl)
        ui->main_gl->requestSlicePlanning(cloudData, m_sliceSpacing,
                                          m_toolOutletRearExtent,
                                          m_toolSafetyForwardExtent,
                                          m_sliceSafeMarginBottom,
                                          m_sliceSafeMarginTop,
                                          m_slicePlaneScaleRatio);
}

void MainWindow::onSlicePlanningProgress(const QString &sourceName, int progress)
{
    updateFullTrajectoryProgress(sourceName, FullSlicePlanning, progress);
    LOG_INFO(QStringLiteral("切片规划：%1 进度 %2%").arg(sourceName).arg(progress));
}

void MainWindow::onSlicePlanningLog(const QString &sourceName, const QString &message)
{
    LOG_INFO(QStringLiteral("切片规划：[%1] %2").arg(sourceName, message));
}

void MainWindow::onSlicePlanningFinished(const QString &sourceName,
                                         vtkSmartPointer<vtkPolyData> slicePlanes,
                                         vtkSmartPointer<vtkPolyData> firstSlicePlane,
                                         vtkSmartPointer<vtkPolyData> boundingBox,
                                         bool ok,
                                         const QString &message)
{
    const bool fullPipeline =
        m_fullTrajectoryStages.value(sourceName, -1) == FullSlicePlanning;
    auto source = m_slicePlanningSources.take(sourceName);
    if (!ok || !source)
    {
        if (fullPipeline)
            failFullTrajectory(source, FullSlicePlanning, message);
        LOG_ERROR(QStringLiteral("切片规划：%1 失败，%2").arg(sourceName, message));
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("切片平面生成失败"), TipWidget::Dangerous);
        return;
    }

    source->slicePlanes = slicePlanes;
    source->firstSlicePlane = firstSlicePlane;
    source->sliceBoundingBox = boundingBox;
    if (ui->main_gl)
        ui->main_gl->showSlicePlanningResult(source, slicePlanes, firstSlicePlane, boundingBox);

    const vtkIdType otherCount = slicePlanes ? slicePlanes->GetNumberOfCells() : 0;
    const vtkIdType firstCount = firstSlicePlane ? firstSlicePlane->GetNumberOfCells() : 0;
    LOG_INFO(QStringLiteral("切片规划：%1 完成，切片平面数量=%2")
        .arg(sourceName).arg(otherCount + firstCount));
    if (tip_manager && !fullPipeline)
        tip_manager->showTip(QStringLiteral("切片平面已生成"), TipWidget::Succeed);
    if (fullPipeline)
        continueFullTrajectory(source, FullSlicePlanning);
}

void MainWindow::onRequestSliceContours(std::shared_ptr<NodeCloudData> cloudData)
{
    if (currentStepIndex != static_cast<int>(WizardStep::PathPlanning))
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先进入路径规划流程"), TipWidget::Warning);
        return;
    }
    if (!cloudData || !cloudData->surgicalMesh ||
        (!cloudData->slicePlanes && !cloudData->firstSlicePlane))
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先生成切片平面"), TipWidget::Warning);
        return;
    }

    const QString sourceName = cloudData->displayName;
    if (sourceName.isEmpty())
        return;
    m_sliceContourSources[sourceName] = cloudData;
    updateFullTrajectoryProgress(sourceName, FullSliceContours, 0);
    LOG_INFO(QStringLiteral("切片轮廓：开始计算 %1 的切片交线").arg(sourceName));
    if (ui->main_gl)
        ui->main_gl->requestSliceContours(cloudData);
}

void MainWindow::onSliceContourProgress(const QString &sourceName, int progress)
{
    updateFullTrajectoryProgress(sourceName, FullSliceContours, progress);
    LOG_INFO(QStringLiteral("切片轮廓：%1 进度 %2%").arg(sourceName).arg(progress));
}

void MainWindow::onSliceContourLog(const QString &sourceName, const QString &message)
{
    LOG_INFO(QStringLiteral("切片轮廓：[%1] %2").arg(sourceName, message));
}

void MainWindow::onSliceContourFinished(const QString &sourceName,
                                        vtkSmartPointer<vtkPolyData> contours,
                                        bool ok,
                                        const QString &message)
{
    const bool fullPipeline =
        m_fullTrajectoryStages.value(sourceName, -1) == FullSliceContours;
    auto source = m_sliceContourSources.take(sourceName);
    if (!ok || !source)
    {
        if (fullPipeline)
            failFullTrajectory(source, FullSliceContours, message);
        LOG_ERROR(QStringLiteral("切片轮廓：%1 失败，%2").arg(sourceName, message));
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("切片轮廓生成失败"), TipWidget::Dangerous);
        return;
    }

    source->sliceContours = contours;
    if (ui->main_gl)
        ui->main_gl->showSliceContours(source, contours);
    LOG_INFO(QStringLiteral("切片轮廓：%1 完成，轮廓点=%2，线单元=%3")
        .arg(sourceName)
        .arg(contours ? contours->GetNumberOfPoints() : 0)
        .arg(contours ? contours->GetNumberOfLines() : 0));
    if (tip_manager && !fullPipeline)
        tip_manager->showTip(QStringLiteral("切片轮廓已生成"), TipWidget::Succeed);
    if (fullPipeline)
        continueFullTrajectory(source, FullSliceContours);
}

void MainWindow::onRequestContourFitting(std::shared_ptr<NodeCloudData> cloudData)
{
    if (currentStepIndex != static_cast<int>(WizardStep::PathPlanning))
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先进入路径规划流程"), TipWidget::Warning);
        return;
    }
    if (!cloudData || !cloudData->sliceContours ||
        cloudData->sliceContours->GetNumberOfLines() == 0)
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先生成切片轮廓"), TipWidget::Warning);
        return;
    }
    if (!cloudData->straightAxis || cloudData->straightAxis->GetNumberOfPoints() < 2)
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("当前轨迹节点没有有效直参考轴"), TipWidget::Warning);
        return;
    }

    const QString sourceName = cloudData->displayName;
    if (sourceName.isEmpty())
        return;
    m_contourFittingSources[sourceName] = cloudData;
    updateFullTrajectoryProgress(sourceName, FullContourFitting, 0);

    LOG_INFO(QStringLiteral("轮廓拟合：开始处理 %1，原始轮廓点=%2，线单元=%3")
        .arg(sourceName)
        .arg(cloudData->sliceContours->GetNumberOfPoints())
        .arg(cloudData->sliceContours->GetNumberOfLines()));
    if (ui->main_gl) {
        ui->main_gl->requestContourFitting(
            cloudData, m_contourFitPointCount, m_contourCoverageThreshold,
            m_contourSmoothWindow, m_contourAngleBinDeg, m_contourTrimOpenEnds,
            m_contourEndCheckCount, m_contourCurvaturePeakRatio,
            m_contourCurvatureRecoverRatio, m_contourMaxTrimCount,
            m_contourMinKeepPointCount);
    }
}

void MainWindow::onContourFittingProgress(const QString &sourceName, int progress)
{
    updateFullTrajectoryProgress(sourceName, FullContourFitting, progress);
    LOG_INFO(QStringLiteral("轮廓拟合：%1 进度 %2%").arg(sourceName).arg(progress));
}

void MainWindow::onContourFittingLog(const QString &sourceName, const QString &message)
{
    LOG_INFO(QStringLiteral("轮廓拟合：[%1] %2").arg(sourceName, message));
}

void MainWindow::onContourFittingFinished(const QString &sourceName,
                                          vtkSmartPointer<vtkPolyData> fittedContours,
                                          bool ok,
                                          const QString &message)
{
    const bool fullPipeline =
        m_fullTrajectoryStages.value(sourceName, -1) == FullContourFitting;
    auto source = m_contourFittingSources.take(sourceName);
    if (!ok || !source || !fittedContours || fittedContours->GetNumberOfLines() == 0)
    {
        if (fullPipeline)
            failFullTrajectory(source, FullContourFitting, message);
        LOG_ERROR(QStringLiteral("轮廓拟合：%1 失败，%2").arg(sourceName, message));
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("切片轮廓拟合失败"), TipWidget::Dangerous);
        return;
    }

    source->fittedSliceContours = fittedContours;
    if (ui->main_gl)
        ui->main_gl->showFittedContours(source, fittedContours);
    LOG_INFO(QStringLiteral("轮廓拟合：%1 完成，拟合点=%2，拟合层=%3，旋转起点=%4")
        .arg(sourceName)
        .arg(fittedContours->GetNumberOfPoints())
        .arg(fittedContours->GetNumberOfLines())
        .arg(source->rotationStartMarkers
            ? source->rotationStartMarkers->GetNumberOfPoints() : 0));
    if (tip_manager && !fullPipeline)
        tip_manager->showTip(QStringLiteral("B 样条轮廓拟合完成"), TipWidget::Succeed);
    if (fullPipeline)
        continueFullTrajectory(source, FullContourFitting);
}

void MainWindow::onRequestEqualDosePath(std::shared_ptr<NodeCloudData> cloudData)
{
    if (currentStepIndex != static_cast<int>(WizardStep::PathPlanning)) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先进入路径规划流程"), TipWidget::Warning);
        return;
    }
    if (!m_pathParamsConfigured) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先点击参数按钮并保存路径参数"), TipWidget::Warning);
        return;
    }
    if (!cloudData || !cloudData->fittedSliceContours ||
        cloudData->fittedSliceContours->GetNumberOfLines() == 0) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先拟合切片轮廓"), TipWidget::Warning);
        return;
    }
    if (!cloudData->straightAxis || cloudData->straightAxis->GetNumberOfPoints() < 2) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("当前轨迹节点没有有效直参考轴"), TipWidget::Warning);
        return;
    }

    const QString sourceName = cloudData->displayName;
    if (sourceName.isEmpty())
        return;
    m_equalDosePathSources[sourceName] = cloudData;
    updateFullTrajectoryProgress(sourceName, FullSprayGeometry, 0);
    LOG_INFO(QStringLiteral("喷杆几何路径：开始处理 %1，拟合层=%2")
        .arg(sourceName)
        .arg(cloudData->fittedSliceContours->GetNumberOfLines()));
    if (ui->main_gl) {
        ui->main_gl->requestEqualDosePath(
            cloudData, m_equalDoseNozzleLength, m_equalDoseRodRadius,
            m_nozzleSafetyClearance,
            m_equalDoseClosedSamples, m_equalDoseOpenSamples,
            m_equalDoseConnectionAngleDeg, m_equalDoseUseTopOpenEndpoint,
            m_equalDoseOpenEndpointMode, m_equalDoseRegionRanges,
            m_equalDoseZeroOffsetDeg, m_equalDoseViewDirection,
            m_equalDoseIncreaseDirection);
    }
}

void MainWindow::onEqualDosePathProgress(const QString &sourceName, int progress)
{
    updateFullTrajectoryProgress(sourceName, FullSprayGeometry, progress);
    LOG_INFO(QStringLiteral("喷杆几何路径：%1 进度 %2%").arg(sourceName).arg(progress));
}

void MainWindow::onEqualDosePathLog(const QString &sourceName, const QString &message)
{
    LOG_INFO(QStringLiteral("喷杆几何路径：[%1] %2").arg(sourceName, message));
}

void MainWindow::onEqualDosePathFinished(
    const QString &sourceName,
    vtkSmartPointer<vtkPolyData> equalDoseSurface,
    vtkSmartPointer<vtkPolyData> sprayPath,
    vtkSmartPointer<vtkPolyData> pathConnections,
    bool ok,
    const QString &message)
{
    const bool fullPipeline =
        m_fullTrajectoryStages.value(sourceName, -1) == FullSprayGeometry;
    auto source = m_equalDosePathSources.take(sourceName);
    if (!ok || !source || !equalDoseSurface || equalDoseSurface->GetNumberOfCells() == 0 ||
        !sprayPath || sprayPath->GetNumberOfLines() == 0) {
        if (fullPipeline)
            failFullTrajectory(source, FullSprayGeometry, message);
        LOG_ERROR(QStringLiteral("喷杆几何路径：%1 失败，%2").arg(sourceName, message));
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("喷杆几何路径生成失败"), TipWidget::Dangerous);
        return;
    }

    source->equalDoseSurface = equalDoseSurface;
    source->sprayPath = sprayPath;
    source->sprayPathConnections = pathConnections;
    source->showEqualDoseSurface = true;
    source->showSprayPath = true;
    if (ui->main_gl) {
        ui->main_gl->showEqualDosePathResult(
            source, equalDoseSurface, sprayPath, pathConnections);
    }
    LOG_INFO(QStringLiteral("喷杆几何路径：%1 完成，分区面片=%2，路径层=%3，层间连接=%4")
        .arg(sourceName)
        .arg(equalDoseSurface->GetNumberOfCells())
        .arg(sprayPath->GetNumberOfLines())
        .arg(pathConnections ? pathConnections->GetNumberOfLines() : 0));
    if (tip_manager && !fullPipeline)
        tip_manager->showTip(QStringLiteral("喷杆几何路径已生成"), TipWidget::Succeed);
    if (fullPipeline)
        continueFullTrajectory(source, FullSprayGeometry);
}

void MainWindow::onRequestContinuousPath(std::shared_ptr<NodeCloudData> cloudData)
{
    if (currentStepIndex != static_cast<int>(WizardStep::PathPlanning)) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先进入路径规划流程"), TipWidget::Warning);
        return;
    }
    if (!m_pathParamsConfigured) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先点击参数按钮并保存路径参数"), TipWidget::Warning);
        return;
    }
    if (!cloudData || !cloudData->sprayPath ||
        cloudData->sprayPath->GetNumberOfLines() == 0) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先生成喷杆几何路径"), TipWidget::Warning);
        return;
    }
    if (!cloudData->straightAxis || cloudData->straightAxis->GetNumberOfPoints() < 2) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("当前轨迹节点没有有效直参考轴"), TipWidget::Warning);
        return;
    }

    const QString sourceName = cloudData->displayName;
    if (sourceName.isEmpty())
        return;
    m_continuousPathSources[sourceName] = cloudData;
    updateFullTrajectoryProgress(sourceName, FullContinuousPath, 0);
    LOG_INFO(QStringLiteral("分段连续路径：开始处理 %1，关节6单段最大转角=%2度")
        .arg(sourceName).arg(m_continuousMaxJoint6SweepDeg));
    LOG_INFO(QStringLiteral("分段连续路径：当前使用局部周向角进行保守约束，实际关节6角度需在机械臂逆解后再次检查。"));
    if (ui->main_gl) {
        ui->main_gl->requestContinuousPath(
            cloudData, m_continuousMaxJoint6SweepDeg,
            m_continuousMinPointSpacing, m_continuousMaxTransitionDistance,
            !m_continuousReverseLayerOrder, m_continuousAutoReverseOpenLayers);
    }
}

void MainWindow::onContinuousPathProgress(const QString &sourceName, int progress)
{
    updateFullTrajectoryProgress(sourceName, FullContinuousPath, progress);
    LOG_INFO(QStringLiteral("分段连续路径：%1 进度 %2%").arg(sourceName).arg(progress));
}

void MainWindow::onContinuousPathLog(const QString &sourceName, const QString &message)
{
    LOG_INFO(QStringLiteral("分段连续路径：[%1] %2").arg(sourceName, message));
}

void MainWindow::onContinuousPathFinished(
    const QString &sourceName,
    vtkSmartPointer<vtkPolyData> continuousPath,
    vtkSmartPointer<vtkPolyData> transitions,
    vtkSmartPointer<vtkPolyData> resetMarkers,
    bool ok,
    const QString &message)
{
    const bool fullPipeline =
        m_fullTrajectoryStages.value(sourceName, -1) == FullContinuousPath;
    auto source = m_continuousPathSources.take(sourceName);
    if (!ok || !source || !continuousPath || continuousPath->GetNumberOfLines() == 0) {
        if (fullPipeline)
            failFullTrajectory(source, FullContinuousPath, message);
        LOG_ERROR(QStringLiteral("分段连续路径：%1 失败，%2").arg(sourceName, message));
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("分段连续路径生成失败"), TipWidget::Dangerous);
        return;
    }

    source->continuousSprayPath = continuousPath;
    source->continuousPathTransitions = transitions;
    source->joint6ResetMarkers = resetMarkers;
    source->showContinuousPath = true;
    source->showSprayPath = false;
    if (ui->main_gl) {
        ui->main_gl->setSprayPathVisible(source, false);
        ui->main_gl->showContinuousPathResult(
            source, continuousPath, transitions, resetMarkers);
    }
    LOG_INFO(QStringLiteral("分段连续路径：%1 完成，执行段=%2，层间连接=%3，关节6复位点=%4")
        .arg(sourceName)
        .arg(continuousPath->GetNumberOfLines())
        .arg(transitions ? transitions->GetNumberOfLines() : 0)
        .arg(resetMarkers ? resetMarkers->GetNumberOfPoints() : 0));
    LOG_INFO(QStringLiteral("分段连续路径：仅完成几何分段；复位动作、机械臂逆解、碰撞检测尚未生成，当前结果不可直接执行。"));
    if (tip_manager && !fullPipeline)
        tip_manager->showTip(QStringLiteral("分段连续路径及关节6复位点已生成"), TipWidget::Succeed);
    if (fullPipeline)
        continueFullTrajectory(source, FullContinuousPath);
}

void MainWindow::onRequestNozzlePoses(std::shared_ptr<NodeCloudData> cloudData)
{
    if (currentStepIndex != static_cast<int>(WizardStep::PathPlanning)) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先进入路径规划流程"), TipWidget::Warning);
        return;
    }
    if (!m_pathParamsConfigured) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先点击参数按钮并保存路径参数"), TipWidget::Warning);
        return;
    }
    if (!cloudData || !cloudData->continuousSprayPath ||
        cloudData->continuousSprayPath->GetNumberOfLines() == 0) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先生成分段连续路径"), TipWidget::Warning);
        return;
    }
    if (!cloudData->fittedSliceContours ||
        cloudData->fittedSliceContours->GetNumberOfLines() == 0 ||
        !cloudData->rotationStartMarkers ||
        cloudData->rotationStartMarkers->GetNumberOfPoints() == 0) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("缺少拟合轮廓或旋转起点，请重新拟合切片轮廓"), TipWidget::Warning);
        return;
    }
    if (!cloudData->straightAxis || cloudData->straightAxis->GetNumberOfPoints() < 2) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("当前轨迹节点没有有效蓝色参考轴"), TipWidget::Warning);
        return;
    }

    const QString sourceName = cloudData->displayName;
    if (sourceName.isEmpty())
        return;
    m_nozzlePoseSources[sourceName] = cloudData;
    updateFullTrajectoryProgress(sourceName, FullNozzlePoses, 0);
    LOG_INFO(QStringLiteral("喷嘴位姿：开始处理 %1，顺序=左半圈/回零/右半圈/回零")
        .arg(sourceName));
    LOG_INFO(QStringLiteral("喷嘴位姿：当前仅生成模型坐标系中的离线几何位姿，不会向机械臂下发命令。"));
    if (ui->main_gl)
        ui->main_gl->requestNozzlePoses(cloudData, m_continuousReverseLayerOrder);
}

void MainWindow::onNozzlePoseProgress(const QString &sourceName, int progress)
{
    updateFullTrajectoryProgress(sourceName, FullNozzlePoses, progress);
    LOG_INFO(QStringLiteral("喷嘴位姿：%1 进度 %2%").arg(sourceName).arg(progress));
}

void MainWindow::onNozzlePoseLog(const QString &sourceName, const QString &message)
{
    LOG_INFO(QStringLiteral("喷嘴位姿：[%1] %2").arg(sourceName, message));
}

void MainWindow::onNozzlePoseFinished(
    const QString &sourceName,
    vtkSmartPointer<vtkPolyData> poseSequence,
    vtkSmartPointer<vtkPolyData> posePreview,
    bool ok,
    const QString &message)
{
    const bool fullPipeline =
        m_fullTrajectoryStages.value(sourceName, -1) == FullNozzlePoses;
    auto source = m_nozzlePoseSources.take(sourceName);
    if (!ok || !source || !poseSequence || poseSequence->GetNumberOfPoints() == 0 ||
        !posePreview || posePreview->GetNumberOfPoints() == 0) {
        if (fullPipeline)
            failFullTrajectory(source, FullNozzlePoses, message);
        LOG_ERROR(QStringLiteral("喷嘴位姿：%1 失败，%2").arg(sourceName, message));
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("喷嘴位姿生成失败"), TipWidget::Dangerous);
        return;
    }

    source->nozzlePoseSequence = poseSequence;
    source->nozzlePosePreview = posePreview;
    source->showNozzlePoses = true;
    if (ui->main_gl)
        ui->main_gl->showNozzlePoseResult(source, poseSequence, posePreview);
    publishPathForTransform(source);
    LOG_INFO(QStringLiteral("喷嘴位姿：%1 完成，离线位姿=%2，预览箭头=%3")
        .arg(sourceName)
        .arg(poseSequence->GetNumberOfPoints())
        .arg(posePreview->GetNumberOfPoints()));
    LOG_INFO(QStringLiteral("喷嘴位姿：尚未进行手眼标定、TCP 工具变换、机械臂逆解和碰撞检查，当前不可执行。"));
    if (tip_manager && !fullPipeline)
        tip_manager->showTip(QStringLiteral("喷嘴离线位姿已生成"), TipWidget::Succeed);
    if (fullPipeline)
        continueFullTrajectory(source, FullNozzlePoses);
}

void MainWindow::publishPathForTransform(
    const std::shared_ptr<NodeCloudData> &data)
{
    if (!data || !data->nozzlePoseSequence ||
        data->nozzlePoseSequence->GetNumberOfPoints() == 0) {
        return;
    }
    if (!m_rosWorker || !m_rosWorker->node()) {
        LOG_ERROR("喷嘴位姿：ROS 发布节点未就绪，未提交坐标转换。");
        return;
    }
    if (data->sourceHeader.frame_id.empty()) {
        LOG_ERROR("喷嘴位姿：原始点云 frame_id 缺失，禁止猜测相机坐标系。");
        return;
    }
    if (!data->hasCaptureJointState || data->captureJointState.position.size() < 6) {
        LOG_ERROR("喷嘴位姿：缺少点云采集时六关节状态，禁止使用当前姿态替代。");
        return;
    }

    vtkPolyData *sequence = data->nozzlePoseSequence;
    vtkPointData *pointData = sequence->GetPointData();
    vtkDataArray *sequenceIndices = pointData->GetArray("SequenceIndex");
    vtkDataArray *sliceIds = pointData->GetArray("SliceIndex");
    vtkDataArray *motionPhases = pointData->GetArray("MotionPhase");
    vtkDataArray *plasmaEnabled = pointData->GetArray("PlasmaEnabled");
    vtkDataArray *joint6Angles = pointData->GetArray("Joint6GeometricDeg");
    vtkDataArray *quaternions = pointData->GetArray("QuaternionXYZW");
    vtkDataArray *targetPoints = pointData->GetArray("TargetPoint");
    vtkFieldData *fieldData = sequence->GetFieldData();
    vtkDataArray *mouthTipPose = fieldData ?
        fieldData->GetArray("CavityMouthTipPoseXYZXYZW") : nullptr;
    vtkDataArray *entryPose = fieldData ?
        fieldData->GetArray("CavityEntryTcpPoseXYZXYZW") : nullptr;
    vtkDataArray *preEntryPose = fieldData ?
        fieldData->GetArray("PreEntryTcpPoseXYZXYZW") : nullptr;
    vtkDataArray *cavityAxis = fieldData ?
        fieldData->GetArray("CavityAxisOutward") : nullptr;
    vtkDataArray *entrySliceIndex = fieldData ?
        fieldData->GetArray("CavityEntrySliceIndex") : nullptr;
    vtkDataArray *preEntryDistance = fieldData ?
        fieldData->GetArray("PreEntryDistanceM") : nullptr;
    vtkDataArray *entryTipStandoff = fieldData ?
        fieldData->GetArray("EntryTipStandoffM") : nullptr;
    const vtkIdType pointCount = sequence->GetNumberOfPoints();

    auto hasTuples = [pointCount](vtkDataArray *array, int components) {
        return array && array->GetNumberOfTuples() == pointCount &&
               array->GetNumberOfComponents() >= components;
    };
    if (!hasTuples(sequenceIndices, 1) || !hasTuples(sliceIds, 1) ||
        !hasTuples(motionPhases, 1) || !hasTuples(plasmaEnabled, 1) ||
        !hasTuples(joint6Angles, 1) || !hasTuples(quaternions, 4) ||
        !hasTuples(targetPoints, 3)) {
        LOG_ERROR("喷嘴位姿：路径元数据不完整，未提交坐标转换。");
        return;
    }
    if (!mouthTipPose || mouthTipPose->GetNumberOfTuples() != 1 ||
        mouthTipPose->GetNumberOfComponents() < 7 ||
        !entryPose || entryPose->GetNumberOfTuples() != 1 ||
        entryPose->GetNumberOfComponents() < 7 ||
        !preEntryPose || preEntryPose->GetNumberOfTuples() != 1 ||
        preEntryPose->GetNumberOfComponents() < 7 ||
        !cavityAxis || cavityAxis->GetNumberOfTuples() != 1 ||
        cavityAxis->GetNumberOfComponents() < 3 ||
        !entrySliceIndex || entrySliceIndex->GetNumberOfTuples() != 1 ||
        !preEntryDistance || preEntryDistance->GetNumberOfTuples() != 1 ||
        !entryTipStandoff || entryTipStandoff->GetNumberOfTuples() != 1) {
        LOG_ERROR("喷嘴位姿：缺少入口/预入口结构化数据，未提交坐标转换。");
        return;
    }

    plasma_robot_interfaces::msg::SprayPath path;
    path.header = data->sourceHeader;
    if (data->motionPathId.isEmpty()) {
        const auto &stamp = data->sourceHeader.stamp;
        if (stamp.sec != 0 || stamp.nanosec != 0) {
            data->motionPathId = QStringLiteral("%1_%2_%3")
                .arg(data->displayName)
                .arg(stamp.sec)
                .arg(stamp.nanosec, 9, 10, QLatin1Char('0'));
        } else {
            data->motionPathId = QStringLiteral("%1_%2")
                .arg(data->displayName)
                .arg(QDateTime::currentMSecsSinceEpoch());
        }
    }
    path.path_id = data->motionPathId.toStdString();
    path.tool_frame = "plasma_motion_tcp";
    path.capture_joint_state = data->captureJointState;
    path.capture_gripper_pose_valid = false;
    path.base_frame = "baselink";
    path.gripper_frame = "Link6";
    path.transform_valid = false;
    path.execution_permitted = false;
    path.status_message = "camera-frame path awaiting calibrated transform";
    auto readPose = [](vtkDataArray *array, geometry_msgs::msg::Pose *pose) {
        pose->position.x = array->GetComponent(0, 0);
        pose->position.y = array->GetComponent(0, 1);
        pose->position.z = array->GetComponent(0, 2);
        pose->orientation.x = array->GetComponent(0, 3);
        pose->orientation.y = array->GetComponent(0, 4);
        pose->orientation.z = array->GetComponent(0, 5);
        pose->orientation.w = array->GetComponent(0, 6);
    };
    readPose(mouthTipPose, &path.cavity_mouth_tip_pose);
    readPose(entryPose, &path.cavity_entry_tcp_pose);
    readPose(preEntryPose, &path.pre_entry_tcp_pose);
    path.cavity_mouth_tip_pose_valid = true;
    path.cavity_entry_tcp_pose_valid = true;
    path.pre_entry_tcp_pose_valid = true;
    path.cavity_axis.x = cavityAxis->GetComponent(0, 0);
    path.cavity_axis.y = cavityAxis->GetComponent(0, 1);
    path.cavity_axis.z = cavityAxis->GetComponent(0, 2);
    path.cavity_axis_valid = true;
    path.cavity_entry_slice_index = static_cast<std::int32_t>(
        entrySliceIndex->GetComponent(0, 0));
    path.pre_entry_distance_m = preEntryDistance->GetComponent(0, 0);
    path.entry_tip_standoff_m = entryTipStandoff->GetComponent(0, 0);
    path.cavity_entry_flange_pose_valid = false;
    path.pre_entry_flange_pose_valid = false;
    path.points.reserve(static_cast<std::size_t>(pointCount));

    for (vtkIdType index = 0; index < pointCount; ++index) {
        plasma_robot_interfaces::msg::SprayPathPoint point;
        double position[3] = {0.0, 0.0, 0.0};
        sequence->GetPoint(index, position);
        point.tcp_pose.position.x = position[0];
        point.tcp_pose.position.y = position[1];
        point.tcp_pose.position.z = position[2];
        point.tcp_pose.orientation.x = quaternions->GetComponent(index, 0);
        point.tcp_pose.orientation.y = quaternions->GetComponent(index, 1);
        point.tcp_pose.orientation.z = quaternions->GetComponent(index, 2);
        point.tcp_pose.orientation.w = quaternions->GetComponent(index, 3);
        point.flange_pose_valid = false;
        point.sequence_index = static_cast<std::int32_t>(
            sequenceIndices->GetComponent(index, 0));
        point.layer_index = static_cast<std::int32_t>(
            sliceIds->GetComponent(index, 0));
        point.motion_phase = static_cast<std::uint8_t>(
            motionPhases->GetComponent(index, 0));
        point.plasma_enabled = plasmaEnabled->GetComponent(index, 0) > 0.5;
        point.joint6_geometric_deg = joint6Angles->GetComponent(index, 0);
        point.surface_target.x = targetPoints->GetComponent(index, 0);
        point.surface_target.y = targetPoints->GetComponent(index, 1);
        point.surface_target.z = targetPoints->GetComponent(index, 2);
        path.points.push_back(std::move(point));
    }

    m_rosWorker->node()->publishSprayPath(path);
    LOG_INFO(QStringLiteral(
        "喷嘴位姿：已提交坐标转换，显示路径=%1，唯一ID=%2，源坐标系=%3，位姿=%4，采集关节=%5。")
        .arg(data->displayName)
        .arg(data->motionPathId)
        .arg(QString::fromStdString(path.header.frame_id))
        .arg(path.points.size())
        .arg(path.capture_joint_state.position.size()));
}

void MainWindow::onRequestSurgicalAreaSelection(std::shared_ptr<NodeCloudData> cloudData)
{
    if (currentStepIndex != static_cast<int>(WizardStep::CloudRebuild))
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先进入残腔重建流程"), TipWidget::Warning);
        return;
    }

    if (!cloudData || !cloudData->polyData || cloudData->meshFilePath.isEmpty())
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("当前重建节点缺少术区选择所需的数据"), TipWidget::Warning);
        return;
    }

    if (ui->main_gl)
        ui->main_gl->startSurgicalAreaSelection(cloudData);
    LOG_INFO(QStringLiteral("残腔重建：开始选择 %1 的核心术区，点击裁剪工具条的勾保存术区，点击 x 放弃。")
        .arg(cloudData->displayName));
}

void MainWindow::onCloudDeleted(std::shared_ptr<NodeCloudData> cloudData)
{
    if (m_selectedCloudData == cloudData) {
        resetRobotExecutionConfirmation();
        m_selectedCloudData.reset();
    }
    if (ui->main_gl)
        ui->main_gl->removeCachedCloud(cloudData);
    if (cloudData) {
        const QString sourceName = cloudData->displayName;
        cloudData->fullTrajectoryProcessing = false;
        m_fullTrajectoryStages.remove(sourceName);
        m_rebuildSources.remove(sourceName);
        m_pathPlanningSources.remove(sourceName);
        m_slicePlanningSources.remove(sourceName);
        m_sliceContourSources.remove(sourceName);
        m_contourFittingSources.remove(sourceName);
        m_equalDosePathSources.remove(sourceName);
        m_continuousPathSources.remove(sourceName);
        m_nozzlePoseSources.remove(sourceName);
    }
    LOG_INFO(QStringLiteral("数据节点已删除。"));
    updateRobotExecutionSummary();
}

void MainWindow::onCloudCropCancelled()
{
    ui->btnCrop->setChecked(false);
    if (m_recropTarget)
    {
        LOG_INFO(QStringLiteral("残腔重建：已取消重新裁剪 %1，关键帧保持不变。")
            .arg(m_recropTarget->displayName));
        m_recropTarget.reset();
        m_cloudCapturePaused = false;
        ui->btnCrop->setEnabled(false);
    }
}

void MainWindow::onCloudCropFinished()
{
    if (m_recropTarget)
    {
        vtkSmartPointer<vtkPolyData> cropped = ui->main_gl ? ui->main_gl->latestCroppedCloud() : nullptr;
        if (!cropped || cropped->GetNumberOfPoints() == 0)
        {
            LOG_ERROR(QStringLiteral("残腔重建：重新裁剪结果为空，关键帧保持不变。"));
            m_recropTarget.reset();
            return;
        }

        m_recropTarget->polyData = vtkSmartPointer<vtkPolyData>::New();
        m_recropTarget->polyData->DeepCopy(cropped);

        double bounds[6] = {0, 0, 0, 0, 0, 0};
        m_recropTarget->polyData->GetBounds(bounds);
        m_recropTarget->bboxMin[0] = bounds[0];
        m_recropTarget->bboxMax[0] = bounds[1];
        m_recropTarget->bboxMin[1] = bounds[2];
        m_recropTarget->bboxMax[1] = bounds[3];
        m_recropTarget->bboxMin[2] = bounds[4];
        m_recropTarget->bboxMax[2] = bounds[5];
        for (int i = 0; i < 3; ++i)
            m_recropTarget->center[i] = (m_recropTarget->bboxMin[i] + m_recropTarget->bboxMax[i]) * 0.5;

        if (ui->main_gl) {
            ui->main_gl->removeCachedCloud(m_recropTarget);
            ui->main_gl->setPointCloudVisible(m_recropTarget, true);
        }
        if (dbtree_view_)
            dbtree_view_->updateCloudNode(m_recropTarget);

        LOG_INFO(QStringLiteral("残腔重建：已用重新裁剪结果更新 %1。").arg(m_recropTarget->displayName));
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("关键帧已更新"), TipWidget::Succeed);

        m_recropTarget.reset();
        m_cloudCapturePaused = false;
        ui->btnCrop->setChecked(false);
        ui->btnCrop->setEnabled(false);
        return;
    }

    if (currentStepIndex != static_cast<int>(WizardStep::CloudCapture) || !m_cloudCapturePaused)
        return;

    m_cloudCaptureCropped = true;
    if (m_stepWizardView)
        m_stepWizardView->setCloudCaptureConfirmEnabled(true);
    LOG_INFO("残腔采集：点云裁剪完成，请点击确认保存。");
}

void MainWindow::onReconstructionProgress(const QString &sourceName, int progress)
{
    if (m_stepWizardView)
        m_stepWizardView->setRebuildProgress(progress);
    LOG_INFO(QStringLiteral("残腔重建：%1 进度 %2%").arg(sourceName).arg(progress));
}

void MainWindow::onReconstructionLog(const QString &sourceName, const QString &message)
{
    LOG_INFO(QStringLiteral("残腔重建：[%1] %2").arg(sourceName, message));
}

void MainWindow::onReconstructionFinished(const QString &sourceName,
                                          const QString &meshFilePath,
                                          bool ok,
                                          const QString &message)
{
    auto source = m_rebuildSources.value(sourceName);
    m_rebuildSources.remove(sourceName);
    if (m_stepWizardView)
        m_stepWizardView->setRebuildProgress(m_rebuildSources.isEmpty() ? (ok ? 100 : 0) : 99);

    if (!ok || !source)
    {
        LOG_ERROR(QStringLiteral("残腔重建：%1 失败，%2").arg(sourceName, message));
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("重建失败：%1").arg(sourceName), TipWidget::Dangerous);
        return;
    }

    auto result = std::make_shared<NodeCloudData>();
    result->filePath = source->filePath;
    result->meshFilePath = meshFilePath;
    result->polyData = vtkSmartPointer<vtkPolyData>::New();
    result->polyData->DeepCopy(source->polyData);
    result->sourceHeader = source->sourceHeader;
    result->captureJointState = source->captureJointState;
    result->hasCaptureJointState = source->hasCaptureJointState;
    for (int i = 0; i < 3; ++i) {
        result->bboxMin[i] = source->bboxMin[i];
        result->bboxMax[i] = source->bboxMax[i];
        result->center[i] = source->center[i];
    }

    const QString nodeName = dbtree_view_ ? dbtree_view_->addRebuildMesh(result) : QString();
    if (nodeName.isEmpty())
    {
        LOG_ERROR(QStringLiteral("残腔重建：%1 完成但写入 DB 树失败。").arg(sourceName));
        return;
    }

    LOG_INFO(QStringLiteral("残腔重建：%1 完成，已生成 %2，mesh=%3")
        .arg(sourceName, nodeName, meshFilePath));
    if (m_stepWizardModel)
        m_stepWizardModel->setStepFinished(WizardStep::CloudRebuild, true);
    if (tip_manager)
        tip_manager->showTip(QStringLiteral("%1 重建完成").arg(sourceName), TipWidget::Succeed);
}

void MainWindow::onSurgicalAreaSelectionFinished(std::shared_ptr<NodeCloudData> cloudData, bool ok)
{
    if (!cloudData)
        return;

    if (!ok)
    {
        LOG_INFO(QStringLiteral("残腔重建：已取消 %1 的术区选择。").arg(cloudData->displayName));
        return;
    }

    if (dbtree_view_)
        dbtree_view_->updateCloudNode(cloudData);
    if (m_stepWizardModel)
        m_stepWizardModel->setStepFinished(WizardStep::CloudRebuild, true);

    LOG_INFO(QStringLiteral("残腔重建：已保存 %1 的核心术区，点云点数=%2，mesh点数=%3；界面仅高亮显示术区 mesh。")
        .arg(cloudData->displayName)
        .arg(cloudData->surgicalPointCloud ? cloudData->surgicalPointCloud->GetNumberOfPoints() : 0)
        .arg(cloudData->surgicalMesh ? cloudData->surgicalMesh->GetNumberOfPoints() : 0));
    if (cloudData->surgicalMesh && cloudData->surgicalMesh->GetNumberOfPoints() > 0) {
        QDir recordDir(QStringLiteral(PLASMA_GUI_SOURCE_DIR));
        const QString relativeRecordDir = QStringLiteral(
            "../third_party/plasma_robot/tools/eye_hand/records/entry_geometry");
        if (recordDir.mkpath(relativeRecordDir) && recordDir.cd(relativeRecordDir)) {
            const QString recordPath = recordDir.filePath(QStringLiteral(
                "surgical_mesh_%1.ply").arg(
                    QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz")));
            vtkNew<vtkPLYWriter> writer;
            writer->SetFileName(recordPath.toUtf8().constData());
            writer->SetInputData(cloudData->surgicalMesh);
            writer->SetFileTypeToBinary();
            writer->Write();
            if (writer->GetErrorCode() == 0) {
                LOG_INFO(QStringLiteral("入口几何诊断：已保存核心术区 mesh=%1，来源=%2")
                    .arg(recordPath, cloudData->displayName));
            } else {
                LOG_WARN(QStringLiteral("入口几何诊断：核心术区 mesh 保存失败，错误码=%1")
                    .arg(writer->GetErrorCode()));
            }
        } else {
            LOG_WARN(QStringLiteral("入口几何诊断：无法创建核心术区记录目录"));
        }
    }
    if (tip_manager)
        tip_manager->showTip(QStringLiteral("核心术区已保存"), TipWidget::Succeed);
}

void MainWindow::onConfirmCloudCapture()
{
    if (currentStepIndex != static_cast<int>(WizardStep::CloudCapture))
        return;

    if (!m_cloudCapturePaused || !m_cloudCaptureCropped)
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("请先暂停实时点云并完成裁剪"), TipWidget::Warning);
        return;
    }

    vtkSmartPointer<vtkPolyData> cropped = ui->main_gl ? ui->main_gl->latestCroppedCloud() : nullptr;
    if (!cropped || cropped->GetNumberOfPoints() == 0)
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("当前没有可保存的裁剪点云"), TipWidget::Warning);
        return;
    }

    auto data = std::make_shared<NodeCloudData>();
    data->polyData = vtkSmartPointer<vtkPolyData>::New();
    data->polyData->DeepCopy(cropped);
    data->sourceHeader = m_pendingCaptureHeader;
    data->captureJointState = m_pendingCaptureJointState;
    data->hasCaptureJointState = m_hasPendingCaptureJointState;

    double bounds[6] = {0, 0, 0, 0, 0, 0};
    data->polyData->GetBounds(bounds);
    data->bboxMin[0] = bounds[0];
    data->bboxMax[0] = bounds[1];
    data->bboxMin[1] = bounds[2];
    data->bboxMax[1] = bounds[3];
    data->bboxMin[2] = bounds[4];
    data->bboxMax[2] = bounds[5];
    for (int i = 0; i < 3; ++i)
        data->center[i] = (data->bboxMin[i] + data->bboxMax[i]) * 0.5;

    const QString nodeName = dbtree_view_ ? dbtree_view_->addCaptureCloud(data) : QString();
    if (nodeName.isEmpty())
    {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("保存裁剪点云失败"), TipWidget::Dangerous);
        return;
    }

    LOG_INFO(QString("残腔采集：已保存裁剪点云到 %1").arg(nodeName));
    if (tip_manager)
        tip_manager->showTip(QStringLiteral("已保存裁剪点云到 %1").arg(nodeName), TipWidget::Succeed);

    if (m_stepWizardModel)
        m_stepWizardModel->setStepFinished(WizardStep::CloudCapture, true);
    if (m_stepWizardView)
        m_stepWizardView->setCloudCaptureConfirmEnabled(false);

    m_cloudCapturePaused = false;
    m_cloudCaptureCropped = false;
    ui->btnCrop->setChecked(false);
    ui->btnCrop->setEnabled(false);
    if (ui->main_gl)
        ui->main_gl->SetRosPaused(false);
    LOG_INFO("残腔采集：已恢复实时点云显示。");
}

/**
 * 点云采集完成回调
 * @param ok  true=采集成功, false=采集失败
 */
void MainWindow::onCloudCaptureFinished(bool ok)
{
    if (!m_stepWizardModel) return;

    m_stepWizardModel->setStepFinished(WizardStep::CloudCapture, ok);

    if (ok) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("✅ 点云采集完成"), TipWidget::Succeed);
    } else {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("❌ 点云采集失败"), TipWidget::Dangerous);
    }
}

/**
 * 点云重建完成回调
 * @param ok  true=重建成功, false=重建失败
 */
void MainWindow::onCloudRebuildFinished(bool ok)
{
    if (!m_stepWizardModel) return;

    m_stepWizardModel->setStepFinished(WizardStep::CloudRebuild, ok);

    if (ok) {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("✅ 点云重建完成"), TipWidget::Succeed);
    } else {
        if (tip_manager)
            tip_manager->showTip(QStringLiteral("❌ 点云重建失败"), TipWidget::Dangerous);
    }
}

// ============================================================
// 机械臂状态面板 — Realman Eco65-B
// ============================================================

void MainWindow::handleArmSpeedSelection(QComboBox *combo,
                                         int selectedSpeed,
                                         int &currentSpeed,
                                         const QString &settingsKey,
                                         const QString &speedName)
{
    if (!combo || selectedSpeed == currentSpeed)
        return;

    using EntryStatus = plasma_robot_interfaces::msg::EntryMotionStatus;
    if (m_entryMotionState == EntryStatus::STATE_PLANNING ||
        m_entryMotionState == EntryStatus::STATE_EXECUTING) {
        const QSignalBlocker blocker(combo);
        const int oldIndex = combo->findData(currentSpeed);
        combo->setCurrentIndex(oldIndex);
        if (tip_manager) {
            tip_manager->showTip(
                QStringLiteral("机械臂正在规划或运动，不能切换%1").arg(speedName),
                TipWidget::Warning);
        }
        return;
    }

    currentSpeed = selectedSpeed;
    QSettings settings;
    settings.setValue(settingsKey, currentSpeed);
    if (settingsKey == QStringLiteral("robot/spray_speed_percent")) {
        settings.setValue(QStringLiteral("robot/motion_speed_percent"), currentSpeed);
    }
    applyArmMotionSpeedLaunchParams();
    resetRobotExecutionConfirmation();

    const bool plannerRunning = m_rosLaunchManager &&
        m_rosLaunchManager->isRunning("entry_motion");
    if (plannerRunning) {
        m_rosLaunchManager->restart("entry_motion");
        m_entryMotionStatusReceived = false;
        m_entryMotionPlanAvailable = false;
        m_entryMotionTrajectoryPoints = 0;
    }

    const QString message = plannerRunning
        ? QStringLiteral("%1已切换为 %2%，入口规划器正在重启，请重新规划轨迹")
            .arg(speedName).arg(currentSpeed)
        : QStringLiteral("%1已切换为 %2%，将在下一次轨迹规划时生效")
            .arg(speedName).arg(currentSpeed);
    appendArmLog(message);
    LOG_INFO("RobotArm", message);
    if (tip_manager)
        tip_manager->showTip(message, TipWidget::Succeed);
    updateRobotExecutionSummary();
}

/**
 * 初始化机械臂设备状态页和心跳监控。
 */
void MainWindow::initArmStatusWidget()
{
    ensureStatusDialog();
    if (!m_armStatusWidget)
        return;

    // 参数页只保留配置项，实时连接状态统一在设备状态弹窗中展示。
    ui->armStatusLabel->hide();
    ui->armStatusValue->hide();

    ui->armApproachSpeedComboBox->setStyleSheet(
        ui->armMotionSpeedComboBox->styleSheet());
    ui->armApproachSpeedComboBox->clear();
    ui->armApproachSpeedComboBox->addItem(QStringLiteral("低速 10%"), 10);
    ui->armApproachSpeedComboBox->addItem(QStringLiteral("标准 20%"), 20);
    const int approachSpeedIndex = ui->armApproachSpeedComboBox->findData(
        m_armApproachSpeedPercent);
    ui->armApproachSpeedComboBox->setCurrentIndex(
        approachSpeedIndex >= 0 ? approachSpeedIndex : 0);
    ui->armApproachSpeedComboBox->setToolTip(
        QStringLiteral(
            "仅作用于当前位置到预入口、再沿法向到安全入口的移动段；"
            "不改变安全入口后的腔内喷涂速度"));
    connect(ui->armApproachSpeedComboBox,
            qOverload<int>(&QComboBox::currentIndexChanged),
            this, [this](int index) {
        const int selectedSpeed =
            ui->armApproachSpeedComboBox->itemData(index).toInt();
        if (selectedSpeed != 10 && selectedSpeed != 20)
            return;
        handleArmSpeedSelection(
            ui->armApproachSpeedComboBox, selectedSpeed,
            m_armApproachSpeedPercent,
            QStringLiteral("robot/approach_speed_percent"),
            QStringLiteral("移动速度"));
    });

    ui->armMotionSpeedComboBox->clear();
    ui->armMotionSpeedComboBox->addItem(QStringLiteral("低速 5%"), 5);
    ui->armMotionSpeedComboBox->addItem(QStringLiteral("标准 10%"), 10);
    const int speedIndex = ui->armMotionSpeedComboBox->findData(
        m_armSpraySpeedPercent);
    ui->armMotionSpeedComboBox->setCurrentIndex(speedIndex >= 0 ? speedIndex : 0);
    ui->armMotionSpeedComboBox->setToolTip(
        QStringLiteral(
            "仅作用于安全入口后的腔内喷涂轨迹和腔内反向退出；"
            "不改变当前位置到安全入口的移动速度"));
    connect(ui->armMotionSpeedComboBox,
            qOverload<int>(&QComboBox::currentIndexChanged),
            this, [this](int index) {
        const int selectedSpeed = ui->armMotionSpeedComboBox->itemData(index).toInt();
        if (selectedSpeed != 5 && selectedSpeed != 10)
            return;
        handleArmSpeedSelection(
            ui->armMotionSpeedComboBox, selectedSpeed,
            m_armSpraySpeedPercent,
            QStringLiteral("robot/spray_speed_percent"),
            QStringLiteral("喷涂速度"));
    });

    m_armConfig = ArmConfig::loadDefault();
    if (m_armConfig.valid) {
        ui->armModelComboBox->clear();
        ui->armModelComboBox->addItem(m_armConfig.displayModel(), m_armConfig.armType);
        ui->armModelComboBox->setEnabled(false);
        ui->armModelComboBox->setToolTip(
            QStringLiteral("由驱动配置加载: %1").arg(m_armConfig.sourcePath));
        ui->armIpValue->setText(m_armConfig.controllerIp);
        ui->armPortValue->setText(QString::number(m_armConfig.tcpPort));
        appendArmLog(QStringLiteral("已加载驱动配置: %1").arg(m_armConfig.sourcePath));
        appendArmLog(QStringLiteral("机械臂 %1，TCP %2:%3，UDP %4:%5 / %6 ms。")
            .arg(m_armConfig.armType,
                 m_armConfig.controllerIp,
                 QString::number(m_armConfig.tcpPort),
                 m_armConfig.udpIp,
                 QString::number(m_armConfig.udpPort),
                 QString::number(m_armConfig.udpCycleMs)));
        LOG_INFO("RobotArm", QStringLiteral("已加载驱动配置 %1，控制器 %2:%3")
            .arg(m_armConfig.armType,
                 m_armConfig.controllerIp,
                 QString::number(m_armConfig.tcpPort)));
    } else {
        appendArmLog(QStringLiteral("机械臂配置加载失败: %1").arg(m_armConfig.error));
        LOG_WARN("RobotArm", m_armConfig.error);
    }

    m_armStatusWidget->setArmInfo(ui->armModelComboBox->currentText(),
                                  ui->armIpValue->text(),
                                  ui->armPortValue->text());
    m_armStatusWidget->setSelfCheckStatus(QStringLiteral("待连接真实机械臂"));
    connect(ui->armModelComboBox, &QComboBox::currentTextChanged,
            this, [this](const QString &model) {
        if (m_armStatusWidget) {
            m_armStatusWidget->setArmInfo(model,
                                          ui->armIpValue->text(),
                                          ui->armPortValue->text());
        }
    });

    m_armHealthModel = new DeviceHealthModel(this);
    connect(m_armHealthModel, &DeviceHealthModel::changed,
            this, &MainWindow::applyArmHealthState);
    m_armHealthModel->markDataInactive(
        DeviceHealthModel::State::Disconnected,
        QStringLiteral("等待 /joint_states 数据"));

    m_armHeartbeatTimer = new QTimer(this);
    m_armHeartbeatTimer->setInterval(250);
    connect(m_armHeartbeatTimer, &QTimer::timeout,
            this, &MainWindow::onArmHeartbeatCheck);
    m_armHeartbeatTimer->start();

    appendArmLog(QStringLiteral("机械臂状态框架已就绪，等待 /joint_states。"));
    appendArmLog(QStringLiteral(
        "MoveJ 和工具坐标 MoveL 执行接口已接入；TCP 实时位姿和碰撞检测尚未接入。"));
    LOG_INFO("MainWindow", "机械臂设备状态框架已加载");
}

void MainWindow::applyArmHealthState()
{
    if (!m_armHealthModel)
        return;

    if (m_armStatusWidget) {
        m_armStatusWidget->setHealthState(m_armHealthModel->state(),
                                           m_armHealthModel->statusMessage());
        m_armStatusWidget->setLastUpdate(m_armHealthModel->lastUpdate());
    }

    switch (m_armHealthModel->state()) {
    case DeviceHealthModel::State::Healthy:
        ui->RobotArm->setStatus(StateBtn::Connected, true);
        break;
    case DeviceHealthModel::State::Starting:
    case DeviceHealthModel::State::Warning:
    case DeviceHealthModel::State::Error:
    case DeviceHealthModel::State::Stale:
        ui->RobotArm->setStatus(StateBtn::Error, true);
        break;
    case DeviceHealthModel::State::Disconnected:
    default:
        ui->RobotArm->setStatus(StateBtn::Disconnected, true);
        break;
    }
    updateRobotExecutionSummary();
}

void MainWindow::onArmHeartbeatCheck()
{
    updateRobotExecutionReadiness();
    if (!m_armHealthModel
        || !m_armHealthModel->isDataStale(kArmHeartbeatTimeoutMs)) {
        return;
    }

    m_armHealthModel->markDataInactive(
        DeviceHealthModel::State::Stale,
        QStringLiteral("超过 %1 ms 未收到 /joint_states")
            .arg(kArmHeartbeatTimeoutMs));
    appendArmLog(QStringLiteral("关节数据已中断，机械臂状态标记为过期。"));
    LOG_WARN("RobotArm", "关节数据已中断，机械臂状态标记为过期");
}

/**
 * 初始化 ROS2 Worker 线程，订阅 /joint_states 并更新面板
 */
void MainWindow::initRosWorker()
{
    m_rosThread   = new QThread(this);
    m_rosWorker   = new RosWorker();
    m_rosWorker->moveToThread(m_rosThread);
    initArmMotionExecution();

    // 线程启动 → ros worker 开始 spin
    connect(m_rosThread, &QThread::started,  m_rosWorker, &RosWorker::start);
    connect(m_rosThread, &QThread::finished, m_rosWorker, &QObject::deleteLater);

    // 关节状态 → 更新面板
    connect(m_rosWorker, &RosWorker::jointStateReceived,
            this, &MainWindow::onJointStateReceived);
    connect(m_rosWorker, &RosWorker::cloudReceived,
            this, [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
                if (!msg)
                    return;
                m_latestCloudHeader = msg->header;
            });

    m_rosThread->start();

    appendArmLog(QStringLiteral("ROS2 关节状态订阅已启动。"));
    LOG_INFO("MainWindow", "ROS2 机械臂订阅线程已启动");
}

void MainWindow::initArmMotionExecution()
{
    if (!m_rosWorker)
        return;

    connect(m_rosWorker, &RosWorker::pathExecutionStatusReceived,
            this, [this](int state, const QString &pathId,
                         int currentIndex, int totalPoints,
                         bool motionEnabled, bool pathPermitted,
                         bool driverReady, bool entryPoseValid,
                         bool plasmaOutputEnabled, const QString &message) {
        const int previousState = m_pathExecutorState;
        const QString previousMessage = m_pathExecutorMessage;
        m_pathExecutorState = state;
        m_pathExecutorPathId = pathId;
        m_pathExecutorCurrentIndex = currentIndex;
        m_pathExecutorTotalPoints = totalPoints;
        m_pathExecutorStatusReceived = true;
        m_pathExecutorMotionEnabled = motionEnabled;
        m_pathExecutorPathPermitted = pathPermitted;
        m_pathExecutorDriverReady = driverReady;
        m_pathExecutorEntryPoseValid = entryPoseValid;
        m_pathExecutorMessage = message;

        if (plasmaOutputEnabled) {
            LOG_ERROR("Safety", "正式路径执行器报告真实等离子输出开启，GUI 已请求停止");
            requestPathExecutorStop(QStringLiteral("检测到未授权等离子输出"), true);
            return;
        }

        using Status = plasma_robot_interfaces::msg::PathExecutionStatus;
        if (state == Status::STATE_EXECUTING && totalPoints > 0 &&
            m_stepWizardView) {
            const int progress = std::clamp(
                static_cast<int>(100.0 * (currentIndex + 1) / totalPoints), 0, 99);
            m_stepWizardView->setRobotExecutionProgress(progress);
        } else if (state == Status::STATE_COMPLETED) {
            if (m_stepWizardView)
                m_stepWizardView->setRobotExecutionProgress(100);
            if (m_stepWizardModel)
                m_stepWizardModel->setStepFinished(WizardStep::RobotExecution, true);
            if (previousState != state && tip_manager)
                tip_manager->showTip(QStringLiteral("机械臂喷涂轨迹执行完成（等离子输出关闭）"),
                                     TipWidget::Succeed);
        } else if (state == Status::STATE_ERROR) {
            if (m_stepWizardModel)
                m_stepWizardModel->setStepFinished(WizardStep::RobotExecution, false);
            if (previousState != state && m_stepWizardView)
                m_stepWizardView->setRobotExecutionProgress(0);
        }

        if (previousState != state || previousMessage != message) {
            appendArmLog(QStringLiteral("正式路径执行器：%1").arg(message));
            if (state == Status::STATE_ERROR)
                LOG_ERROR("RobotArm", QStringLiteral("正式路径执行器：%1").arg(message));
            else
                LOG_INFO("RobotArm", QStringLiteral("正式路径执行器：%1").arg(message));
        }
        updateRobotExecutionSummary();
    });
    connect(m_rosWorker, &RosWorker::entryMotionStatusReceived,
            this, [this](int state, const QString &pathId,
                         bool motionEnabled, bool pathPermitted,
                         bool planAvailable, int trajectoryPoints,
                         double translationError, double rotationError,
                         const QString &message) {
        Q_UNUSED(translationError)
        Q_UNUSED(rotationError)
        const int previousState = m_entryMotionState;
        const QString previousMessage = m_entryMotionMessage;
        m_entryMotionState = state;
        m_entryMotionPathId = pathId;
        m_entryMotionStatusReceived = true;
        m_entryMotionEnabled = motionEnabled;
        m_entryMotionPathPermitted = pathPermitted;
        m_entryMotionPlanAvailable = planAvailable;
        m_entryMotionTrajectoryPoints = trajectoryPoints;
        m_entryMotionMessage = message;
        if (previousState != state || previousMessage != message) {
            appendArmLog(QStringLiteral("入口规划器：%1").arg(message));
            using EntryStatus = plasma_robot_interfaces::msg::EntryMotionStatus;
            if (state == EntryStatus::STATE_ERROR)
                LOG_ERROR("RobotArm", QStringLiteral("入口规划器：%1").arg(message));
            else
                LOG_INFO("RobotArm", QStringLiteral("入口规划器：%1").arg(message));

            if (state == EntryStatus::STATE_COMPLETED) {
                if (m_stepWizardView)
                    m_stepWizardView->setRobotExecutionProgress(100);
                if (m_stepWizardModel)
                    m_stepWizardModel->setStepFinished(WizardStep::RobotExecution, true);
            } else if (state == EntryStatus::STATE_STOPPING_IN_CAVITY) {
                if (tip_manager)
                    tip_manager->showTip(
                        QStringLiteral("机械臂正在停稳，保持现场不动，退出按钮稍后开放"),
                        TipWidget::Warning);
            } else if (state == EntryStatus::STATE_STOPPED_IN_CAVITY) {
                if (tip_manager)
                    tip_manager->showTip(
                        QStringLiteral("机械臂已确认停稳，可以点击“从当前停车点安全退出”"),
                        TipWidget::Succeed);
            } else if (state == EntryStatus::STATE_ERROR) {
                if (m_stepWizardModel)
                    m_stepWizardModel->setStepFinished(WizardStep::RobotExecution, false);
            }
        }
        updateRobotExecutionSummary();
    });
    return;

}

/**
 * ROS2 /joint_states 回调槽：提取关节角度，更新 ArmStatusWidget
 */
void MainWindow::onJointStateReceived(const sensor_msgs::msg::JointState::ConstSharedPtr &msg)
{
    if (!msg)
        return;

    m_latestArmJointState = *msg;

    if (!m_armStatusWidget || !m_armHealthModel)
        return;

    const DeviceHealthModel::State previousState = m_armHealthModel->state();
    m_armHealthModel->markDataReceived(QStringLiteral("/joint_states 数据正常"));
    m_armStatusWidget->setLastUpdate(m_armHealthModel->lastUpdate());

    // Realman Eco65-B 有 6 个关节
    if (msg->position.size() < 6) {
        const QString error = QStringLiteral(
            "/joint_states 关节数量不足，期望 6，实际 %1")
            .arg(msg->position.size());
        m_armHealthModel->setLastError(error);
        m_armHealthModel->setState(DeviceHealthModel::State::Warning, error);
        if (previousState != DeviceHealthModel::State::Warning) {
            appendArmLog(error);
            LOG_WARN("RobotArm", error);
        }
        return;
    }

    if (previousState != DeviceHealthModel::State::Healthy) {
        appendArmLog(QStringLiteral("已收到有效关节数据，机械臂状态恢复正常。"));
        LOG_INFO("RobotArm", "已收到有效关节数据，机械臂状态恢复正常");
    }
    m_armHealthModel->setLastError(QString());

    // 将弧度转换为度。TCP 由 RealMan Armstate 接口接入，当前保持未提供状态。
    const double RAD2DEG = 180.0 / M_PI;
    m_armStatusWidget->updateJointAngles(
        msg->position[0] * RAD2DEG,
        msg->position[1] * RAD2DEG,
        msg->position[2] * RAD2DEG,
        msg->position[3] * RAD2DEG,
        msg->position[4] * RAD2DEG,
        msg->position[5] * RAD2DEG
    );
}

/**
 * 机械臂急停槽：调用 ROS2 服务
 */
void MainWindow::onArmEmergencyStop()
{
    if (!m_rosWorker || !m_rosWorker->node()) return;

    LOG_WARN("MainWindow", "⚠️ 机械臂急停请求已发送");

    m_rosWorker->node()->callEmergencyStop(
        [this](bool ok, const std::string &msg) {
            QMetaObject::invokeMethod(this, [this, ok, msg]() {
                if (ok) {
                    LOG_INFO("MainWindow", QString("机械臂急停成功: %1").arg(QString::fromStdString(msg)));
                    if (tip_manager)
                        tip_manager->showTip(QStringLiteral("🛑 机械臂已急停"), TipWidget::Warning);
                } else {
                    LOG_ERROR("MainWindow", QString("机械臂急停失败: %1").arg(QString::fromStdString(msg)));
                    if (tip_manager)
                        tip_manager->showTip(QStringLiteral("❌ 机械臂急停失败"), TipWidget::Dangerous);
                }
            });
        });
}

/**
 * 机械臂自检请求槽
 */
void MainWindow::onArmSelfCheckRequest()
{
    if (!m_rosWorker || !m_rosWorker->node()) return;

    LOG_INFO("MainWindow", "机械臂自检请求已发送");

    m_rosWorker->node()->callSelfCheck(
        [this](bool ok, const std::string &msg) {
            QMetaObject::invokeMethod(this, [this, ok, msg]() {
                QString text = QString::fromStdString(msg);
                if (m_armStatusWidget) {
                    m_armStatusWidget->setSelfCheckStatus(ok ? text : "自检失败");
                }
                if (ok) {
                    LOG_INFO("MainWindow", QString("机械臂自检: %1").arg(text));
                } else {
                    LOG_ERROR("MainWindow", QString("机械臂自检失败: %1").arg(text));
                }
            });
        });
}

