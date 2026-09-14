#include "SystemCheckManager.h"
#include "device_health_model.h"
#include "ros_launch_manager.h"
#include <QPointer>

// =============================================
//  占位 launch 名称（与 MainWindow 注册名保持一致）
// =============================================
static const QString kCameraLaunch     = "camera";
static const QString kRobotLaunch      = "robot";
static const QString kPointCloudLaunch  = "point_cloud";
static constexpr qint64 kRobotStateTimeoutMs = 1500;
static constexpr int kRobotPollIntervalMs = 500;
static constexpr int kRobotStartupTimeoutMs = 15000;

// =============================================
SystemCheckManager::SystemCheckManager(RosLaunchManager *launchManager,
                                       DeviceHealthModel *robotHealthModel,
                                       QObject *parent)
    : QObject(parent)
    , m_launchManager(launchManager)
    , m_robotHealthModel(robotHealthModel)
    , m_currentStep(-1)
{
    // 延迟验证定时器：单次触发，不在构造时启动
    m_verifyTimer.setSingleShot(true);
    connect(&m_verifyTimer, &QTimer::timeout,
            this, &SystemCheckManager::onLaunchVerifyTimer);
}

// =============================================
//  启动自检（全异步，不阻塞调用线程）
// =============================================
void SystemCheckManager::startCheck()
{
    if (m_checkRunning)
    {
        emit checkItemUpdated(QStringLiteral("系统自检"), false,
                              QStringLiteral("自检正在进行中，请稍候..."));
        return;
    }

    m_checkRunning = true;
    m_pendingLaunch.clear();
    m_verifyTimer.stop();
    m_startedRobotLaunch = false;

    emit checkStarted();
    m_currentStep = 0;   // 从 camera 开始
    checkNext();
}

// =============================================
//  checkNext - 依次推进各检查步骤
// =============================================
void SystemCheckManager::checkNext()
{
    // ---- 步骤 0: 相机 ----
    if (m_currentStep == 0)
    {
        if (!m_launchManager)
        {
            emit checkItemUpdated(QStringLiteral("相机"), false,
                                  QStringLiteral("RosLaunchManager 未初始化"));
            finishCheck(false, QStringLiteral("系统自检失败：RosLaunchManager 未初始化"));
            return;
        }

        QPointer<SystemCheckManager> self(this);
        m_launchManager->isRunningAsync(kCameraLaunch, [self](bool running)
        {
            if (!self || !self->m_checkRunning || self->m_currentStep != 0)
                return;

            if (running)
            {
                // 已在运行，直接通过
                emit self->checkItemUpdated(QStringLiteral("相机"), true, QStringLiteral("正常"));
                self->m_currentStep = 1;
                self->checkNext();
                return;
            }

            // 未运行 → 启动并启动延迟验证
            emit self->checkItemUpdated(QStringLiteral("相机"), false,
                                        QStringLiteral("未启动，正在自动启动..."));
            self->m_launchManager->start(kCameraLaunch);
            self->m_pendingLaunch = kCameraLaunch;
            self->m_verifyTimer.start(kLaunchStartupDelayMs);
        });
        return;   // 异步等待定时器回调
    }

    // ---- 步骤 1: 机械臂 ----
    if (m_currentStep == 1)
    {
        if (m_skipRobotCheck)
        {
            emit checkItemUpdated(QStringLiteral("机械臂"), true,
                                  QStringLiteral("调试模式，已跳过"));
            m_currentStep = 2;
            checkNext();
            return;
        }

        if (!m_launchManager)
        {
            emit checkItemUpdated(QStringLiteral("机械臂"), false,
                                  QStringLiteral("RosLaunchManager 未初始化"));
            finishCheck(false, QStringLiteral("系统自检失败：RosLaunchManager 未初始化"));
            return;
        }

        QPointer<SystemCheckManager> self(this);
        m_launchManager->hasExpectedNodeAsync(kRobotLaunch, [self](bool running)
        {
            if (!self || !self->m_checkRunning || self->m_currentStep != 1)
                return;
            self->beginRobotConnectionCheck(running);
        });
        return; // 异步等待真实驱动节点和关节数据
    }

    // ---- 全部完成 ----
    finishCheck(true, QStringLiteral("系统自检通过"));
}

// =============================================
//  onLaunchVerifyTimer - 延迟验证定时器回调
// =============================================
void SystemCheckManager::onLaunchVerifyTimer()
{
    if (m_pendingLaunch.isEmpty()) return;

    if (m_pendingLaunch == kRobotLaunch)
    {
        pollRobotConnection();
        return;
    }

    if (!m_launchManager)
    {
        finishCheck(false, QStringLiteral("系统自检失败：RosLaunchManager 未初始化"));
        return;
    }

    const QString pendingLaunch = m_pendingLaunch;
    QPointer<SystemCheckManager> self(this);
    m_launchManager->isRunningAsync(pendingLaunch, [self, pendingLaunch](bool ok)
    {
        if (!self || !self->m_checkRunning || self->m_pendingLaunch != pendingLaunch)
            return;

        QString displayName = (pendingLaunch == kCameraLaunch)
                                ? QStringLiteral("相机")
                                : pendingLaunch;

        if (ok)
        {
            emit self->checkItemUpdated(displayName, true, QStringLiteral("正常"));
        }
        else
        {
            emit self->checkItemUpdated(displayName, false,
                                        QStringLiteral("%1 启动失败").arg(displayName));
            self->finishCheck(false,
                              QStringLiteral("系统自检失败：%1 启动失败").arg(displayName));
            return;
        }

        self->m_pendingLaunch.clear();

        // 推进到下一步
        if (self->m_currentStep == 0)
        {
            self->m_currentStep = 1;
            self->checkNext();
        }
    });
}

void SystemCheckManager::beginRobotConnectionCheck(bool driverAlreadyRunning)
{
    if (!m_checkRunning || m_currentStep != 1)
        return;

    m_startedRobotLaunch = !driverAlreadyRunning;
    if (driverAlreadyRunning)
    {
        emit checkItemUpdated(QStringLiteral("机械臂"), false,
                              QStringLiteral("真实驱动已运行，正在验证六关节状态..."));
    }
    else
    {
        emit checkItemUpdated(QStringLiteral("机械臂"), false,
                              QStringLiteral("未连接，正在自动启动真实机械臂驱动..."));
        m_launchManager->start(kRobotLaunch);
    }

    m_pendingLaunch = kRobotLaunch;
    m_robotStartupElapsed.restart();
    m_verifyTimer.start(kRobotPollIntervalMs);
}

void SystemCheckManager::pollRobotConnection()
{
    if (!m_checkRunning || m_currentStep != 1 || !m_launchManager)
        return;

    QPointer<SystemCheckManager> self(this);
    m_launchManager->hasExpectedNodeAsync(kRobotLaunch, [self](bool nodeRunning)
    {
        if (!self || !self->m_checkRunning || self->m_currentStep != 1 ||
            self->m_pendingLaunch != kRobotLaunch)
            return;

        QString reason;
        if (nodeRunning && self->checkRobot(&reason))
        {
            emit self->checkItemUpdated(
                QStringLiteral("机械臂"), true,
                QStringLiteral("正常，已连接真实机械臂，六关节数据实时更新"));
            self->m_pendingLaunch.clear();
            self->m_startedRobotLaunch = false;
            self->m_currentStep = 2;
            self->checkNext();
            return;
        }

        if (self->m_robotStartupElapsed.elapsed() >= kRobotStartupTimeoutMs)
        {
            if (self->m_startedRobotLaunch)
                self->m_launchManager->stop(kRobotLaunch);

            const QString detail = !nodeRunning
                ? QStringLiteral("真实 rm_driver 节点未就绪")
                : (reason.isEmpty() ? QStringLiteral("未收到有效六关节状态") : reason);
            emit self->checkItemUpdated(QStringLiteral("机械臂"), false,
                                        QStringLiteral("连接超时：%1").arg(detail));
            self->finishCheck(false,
                QStringLiteral("系统自检失败：机械臂连接超时，%1").arg(detail));
            return;
        }

        self->m_verifyTimer.start(kRobotPollIntervalMs);
    });
}

void SystemCheckManager::finishCheck(bool ok, const QString &message)
{
    m_verifyTimer.stop();
    m_pendingLaunch.clear();
    m_startedRobotLaunch = false;
    m_currentStep = -1;
    m_checkRunning = false;
    emit checkFinished(ok, message);
}

// =============================================
//  单项检查辅助函数（当前仅 checkCamera 使用异步流程）
// =============================================
bool SystemCheckManager::checkRosLaunch(const QString &launchName, QString *reason)
{
    if (!m_launchManager)
    {
        if (reason) *reason = QStringLiteral("RosLaunchManager 未初始化");
        return false;
    }
    if (!m_launchManager->isRunning(launchName))
    {
        if (reason) *reason = QStringLiteral("%1 未运行").arg(launchName);
        return false;
    }
    return true;
}

bool SystemCheckManager::checkCamera(QString *reason)
{
    return checkRosLaunch(kCameraLaunch, reason);
}

bool SystemCheckManager::checkRobot(QString *reason)
{
    if (!m_robotHealthModel)
    {
        if (reason) *reason = QStringLiteral("状态监控未初始化");
        return false;
    }

    if (!m_robotHealthModel->dataActive()
        || !m_robotHealthModel->lastUpdate().isValid())
    {
        if (reason) *reason = QStringLiteral("未收到六关节状态数据");
        return false;
    }

    if (m_robotHealthModel->isDataStale(kRobotStateTimeoutMs))
    {
        if (reason) *reason = QStringLiteral("关节状态数据已超时");
        return false;
    }

    if (m_robotHealthModel->state() != DeviceHealthModel::State::Healthy)
    {
        if (reason) {
            *reason = m_robotHealthModel->statusMessage().isEmpty()
                ? QStringLiteral("状态异常")
                : m_robotHealthModel->statusMessage();
        }
        return false;
    }

    if (reason) *reason = QStringLiteral("正常，六关节数据实时更新");
    return true;
}

bool SystemCheckManager::checkPointCloud(QString *reason)
{
    return checkRosLaunch(kPointCloudLaunch, reason);
}
