#ifndef SYSTEMCHECKMANAGER_H
#define SYSTEMCHECKMANAGER_H

#include <QObject>
#include <QElapsedTimer>
#include <QString>
#include <QTimer>

// 启动 launch 后等待进程初始化的延迟时间（毫秒）
static constexpr int kLaunchStartupDelayMs = 5000;

class RosLaunchManager;
class DeviceHealthModel;

// =============================================
//  SystemCheckManager
//  - 全异步：startCheck() 不阻塞调用线程
//  - 启动 launch 后通过 QTimer 延迟验证，事件循环始终畅通
// =============================================
class SystemCheckManager : public QObject
{
    Q_OBJECT

public:
    explicit SystemCheckManager(RosLaunchManager *launchManager,
                                DeviceHealthModel *robotHealthModel,
                                QObject *parent = nullptr);

    void startCheck();
    void setSkipRobotCheck(bool skip) { m_skipRobotCheck = skip; }

signals:
    void checkStarted();
    void checkItemUpdated(const QString &itemName, bool ok, const QString &message);
    void checkFinished(bool allPassed, const QString &message);

private slots:
    void onLaunchVerifyTimer();

private:
    void checkNext();
    void finishCheck(bool ok, const QString &message);
    void beginRobotConnectionCheck(bool driverAlreadyRunning);
    void pollRobotConnection();
    bool checkRosLaunch(const QString &launchName, QString *reason);
    bool checkCamera(QString *reason);
    bool checkRobot(QString *reason);
    bool checkPointCloud(QString *reason);

    RosLaunchManager *m_launchManager = nullptr;
    DeviceHealthModel *m_robotHealthModel = nullptr;
    QTimer            m_verifyTimer;       // 异步延迟验证定时器
    QString           m_pendingLaunch;     // 正在等待验证的 launch 名称
    QElapsedTimer     m_robotStartupElapsed;
    int               m_currentStep = -1;  // 当前检查步骤（0=camera, 1=robot）
    bool              m_checkRunning = false;
    bool              m_skipRobotCheck = false;
    bool              m_startedRobotLaunch = false;
};

#endif // SYSTEMCHECKMANAGER_H
