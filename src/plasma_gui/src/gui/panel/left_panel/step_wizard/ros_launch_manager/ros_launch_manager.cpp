#include "ros_launch_manager.h"
#include "logmanager.h"
#include <QElapsedTimer>
#include <QTimer>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <memory>

namespace {

constexpr auto kRealSenseRuntimeLibraryPath =
    "/home/larusxu/.local/librealsense-v4l2-2.50.0/lib";

QString buildNodeListCommand()
{
    return QStringLiteral("export LD_LIBRARY_PATH=%1:$LD_LIBRARY_PATH && ")
        .arg(QString::fromLatin1(kRealSenseRuntimeLibraryPath)) + QStringLiteral(
        "source /opt/ros/galactic/setup.bash && "
        "source /home/larusxu/CodeSpace/PlasmaRobot/install/setup.bash && "
        "ros2 node list");
}

bool nodeMatchesLaunch(const QString &launchName, const QString &nodeName)
{
    QString trimmed = nodeName.trimmed();
    if (trimmed.isEmpty()) return false;

    if (launchName == QStringLiteral("camera"))
    {
        return trimmed.contains(QStringLiteral("/camera")) ||
               trimmed.contains(QStringLiteral("realsense2_camera"));
    }

    if (launchName == QStringLiteral("robot"))
        return trimmed == QStringLiteral("/rm_driver") ||
               trimmed.endsWith(QStringLiteral("/rm_driver"));

    return trimmed.contains(launchName);
}

bool outputContainsLaunchNode(const QString &launchName, const QString &output)
{
    const QStringList nodeLines = output.split('\n');
    for (const QString &node : nodeLines)
    {
        if (nodeMatchesLaunch(launchName, node))
        {
            if (launchName != QStringLiteral("camera")) {
                LOG_INFO("[RosLaunchManager]",
                         QString("%1 fallback 检测: ros2 节点 %2 正在运行")
                             .arg(launchName, node.trimmed()));
            }
            return true;
        }
    }

    return false;
}

} // namespace

// ============================
RosLaunchManager::RosLaunchManager(QObject *parent)
    : QObject(parent)
{
}

// ============================
// 注册 launch
// ============================
void RosLaunchManager::registerLaunch(const QString& name,
                                         const QString& package,
                                         const QString& launchFile)
{
    LaunchInstance ins;
    ins.name = name;
    ins.package = package;
    ins.launchFile = launchFile;
    ins.process = new QProcess(this);

    // 捕获子进程输出，便于在日志中查看错误原因
    ins.process->setProcessChannelMode(QProcess::MergedChannels);

    // 实时读取子进程输出（解决看不到启动失败原因的问题）
    connect(ins.process, &QProcess::readyReadStandardOutput, this,
            [this, process = ins.process, name]() {
        QByteArray data = process->readAllStandardOutput();
        QString text = QString::fromLocal8Bit(data).trimmed();
        if (!text.isEmpty()) {
            for (const QString &line : text.split('\n')) {
                const QString trimmed = line.trimmed();
                if (!trimmed.isEmpty()) {
                    emit launchOutput(name, trimmed);
                    if (name != QStringLiteral("camera"))
                        LOG_INFO("[RosLaunchManager]", QString("[%1] %2").arg(name, trimmed));
                }
            }
        }
    });

    // 连接进程信号（只连一次）
    connect(ins.process, &QProcess::errorOccurred, this, [name](QProcess::ProcessError err) {
        QString errStr;
        switch (err) {
        case QProcess::FailedToStart: errStr = "启动失败"; break;
        case QProcess::Crashed:       errStr = "进程崩溃"; break;
        case QProcess::Timedout:      errStr = "超时";     break;
        case QProcess::WriteError:    errStr = "写入错误"; break;
        case QProcess::ReadError:     errStr = "读取错误"; break;
        default:                      errStr = "未知错误"; break;
        }
        LOG_INFO("[RosLaunchManager]", QString("%1 进程错误: %2").arg(name, errStr));
    });
    connect(ins.process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this, name](int exitCode, QProcess::ExitStatus status) {
        QString statusStr = (status == QProcess::NormalExit) ? "正常退出" : "崩溃退出";
        LOG_INFO("[RosLaunchManager]", QString("%1 进程退出, exitCode=%2, %3")
                 .arg(name).arg(exitCode).arg(statusStr));

        if (m_launches.contains(name) && m_launches[name].stopping)
        {
            bool shouldRestart = m_launches[name].restartAfterStop;
            m_launches[name].stopping = false;
            m_launches[name].restartAfterStop = false;
            emit launchStopped(name);

            if (shouldRestart)
            {
                start(name);
            }
        }
    });

    m_launches[name] = ins;
}

// ============================
// 设置参数
// ============================
void RosLaunchManager::setParam(const QString& launchName,
                                   const QString& key,
                                   const QString& value)
{
    m_launches[launchName].params[key] = value;
}



// ============================
// 生成命令
// ============================
QString RosLaunchManager::buildCommand(const LaunchInstance& ins) const
{
    QString cmd = QStringLiteral("export LD_LIBRARY_PATH=%1:$LD_LIBRARY_PATH && ")
                      .arg(QString::fromLatin1(kRealSenseRuntimeLibraryPath));
    cmd += "source /opt/ros/galactic/setup.bash && ";
    cmd += "source /home/larusxu/CodeSpace/PlasmaRobot/install/setup.bash && ";
    cmd += "ros2 launch " + ins.package + " " + ins.launchFile + " ";

    for (auto it = ins.params.begin(); it != ins.params.end(); ++it)
    {
        cmd += it.key() + ":=" + it.value() + " ";
    }

    return cmd;
}

// ============================
// 启动单个 launch
// ============================
void RosLaunchManager::start(const QString& name)
{
    if (!m_launches.contains(name)) return;

    LaunchInstance &ins = m_launches[name];

    if (ins.running()) return;

    QString cmd = buildCommand(ins);

    if (name != QStringLiteral("camera"))
        LOG_INFO("[RosLaunchManager]", QString("启动 %1: %2").arg(name, cmd));
    // 使用 setsid 创建独立进程组，便于 stop() 时向整个进程组发送信号
    ins.process->start("setsid", QStringList() << "bash" << "-c" << cmd);

    emit launchStarted(name, cmd);
}

// ============================
// 停止
// ============================
void RosLaunchManager::stop(const QString& name)
{
    if (!m_launches.contains(name)) return;

    LaunchInstance &ins = m_launches[name];

    if (ins.process && ins.process->state() == QProcess::Running)
    {
        if (ins.stopping)
        {
            LOG_INFO("[RosLaunchManager]", QString("%1 正在停止中，忽略重复停止请求").arg(name));
            return;
        }

        qint64 pid = ins.process->processId();

        // 向进程组发送 SIGINT，等效于在终端按 Ctrl+C
        // 负 PID 表示发送给整个进程组（setsid 创建的独立进程组）
        if (::kill(-pid, SIGINT) == 0)
        {
            ins.stopping = true;
            LOG_INFO("[RosLaunchManager]", QString("%1 已发送 SIGINT 到进程组 (pgid=%2)").arg(name).arg(pid));
        }
        else
        {
            LOG_ERROR("[RosLaunchManager]",
                      QString("%1 发送 SIGINT 失败 (pgid=%2): %3")
                          .arg(name)
                          .arg(pid)
                          .arg(QString::fromLocal8Bit(std::strerror(errno))));
            return;
        }

        QTimer::singleShot(5000, this, [this, name, pid]()
        {
            if (!m_launches.contains(name)) return;

            LaunchInstance &timerIns = m_launches[name];
            if (!timerIns.process ||
                timerIns.process->state() != QProcess::Running ||
                timerIns.process->processId() != pid ||
                !timerIns.stopping)
            {
                return;
            }

            // SIGINT 未响应，异步强制杀死整个进程组
            if (::kill(-pid, SIGKILL) == 0)
            {
                LOG_INFO("[RosLaunchManager]", QString("%1 SIGINT 超时，已强制终止进程组").arg(name));
            }
            else
            {
                LOG_ERROR("[RosLaunchManager]",
                          QString("%1 强制发送 SIGKILL 失败 (pgid=%2): %3")
                              .arg(name)
                              .arg(pid)
                              .arg(QString::fromLocal8Bit(std::strerror(errno))));
                timerIns.stopping = false;
                timerIns.restartAfterStop = false;
            }
        });
    }
}

// ============================
// 重启
// ============================
void RosLaunchManager::restart(const QString& name)
{
    if (!m_launches.contains(name)) return;

    LaunchInstance &ins = m_launches[name];
    if (ins.running())
    {
        ins.restartAfterStop = true;
        stop(name);
        return;
    }

    start(name);
}

// ============================
// 启动全部
// ============================
void RosLaunchManager::startAll()
{
    for (auto it = m_launches.begin(); it != m_launches.end(); ++it)
        start(it.key());
}

// ============================
// 停止全部
// ============================
void RosLaunchManager::stopAll()
{
    for (auto it = m_launches.begin(); it != m_launches.end(); ++it)
        stop(it.key());
}

void RosLaunchManager::stopAllAndWait(int timeoutMs)
{
    stopAll();

    QElapsedTimer elapsed;
    elapsed.start();
    for (auto it = m_launches.begin(); it != m_launches.end(); ++it)
    {
        QProcess *process = it.value().process;
        if (!process || process->state() == QProcess::NotRunning)
            continue;

        const int remaining = qMax(0, timeoutMs - static_cast<int>(elapsed.elapsed()));
        if (remaining > 0 && process->waitForFinished(remaining))
            continue;

        const qint64 pid = process->processId();
        if (pid > 0)
            ::kill(-pid, SIGKILL);
        process->waitForFinished(1000);
    }
}

// ============================
bool RosLaunchManager::isRunning(const QString& name) const
{
    if (!m_launches.contains(name)) return false;

    // 优先检查 QProcess 状态（进程仍在运行时直接返回）
    if (m_launches[name].running()) return true;

    // fallback: exec 替换了 bash 后，QProcess 跟踪到的进程可能已退出，
    // 但 ros2 节点仍在后台运行，通过 ros2 node list 验证
    QProcess check;
    check.setProcessChannelMode(QProcess::SeparateChannels);
    check.start("bash", QStringList() << "-c" << buildNodeListCommand());
    if (!check.waitForFinished(5000))
    {
        if (name != QStringLiteral("camera"))
            LOG_INFO("[RosLaunchManager]", QString("%1 ros2 node list 超时").arg(name));
        check.kill();
        return false;
    }

    int exitCode = check.exitCode();
    QString output = check.readAllStandardOutput().trimmed();
    QString errOutput = check.readAllStandardError().trimmed();

    if (name != QStringLiteral("camera")) {
        LOG_INFO("[RosLaunchManager]", QString("%1 ros2 node list exitCode=%2, stdout=%3, stderr=%4")
                 .arg(name).arg(exitCode).arg(output.left(200)).arg(errOutput.left(200)));
    }

    return outputContainsLaunchNode(name, output);
}

void RosLaunchManager::isRunningAsync(const QString& name,
                                      std::function<void(bool)> callback) const
{
    if (!callback) return;

    if (!m_launches.contains(name))
    {
        QTimer::singleShot(0, const_cast<RosLaunchManager *>(this), [callback = std::move(callback)]() {
            callback(false);
        });
        return;
    }

    if (m_launches[name].running())
    {
        QTimer::singleShot(0, const_cast<RosLaunchManager *>(this), [callback = std::move(callback)]() {
            callback(true);
        });
        return;
    }

    QProcess *check = new QProcess(const_cast<RosLaunchManager *>(this));
    check->setProcessChannelMode(QProcess::SeparateChannels);

    QTimer *timeout = new QTimer(check);
    timeout->setSingleShot(true);

    auto completed = std::make_shared<bool>(false);
    auto finish = [this, name, callback = std::move(callback), check, timeout, completed](bool running) mutable {
        if (*completed) return;
        *completed = true;

        if (timeout) timeout->stop();
        callback(running);

        if (check)
            check->deleteLater();
    };

    connect(timeout, &QTimer::timeout, this, [name, check, finish]() mutable {
        if (name != QStringLiteral("camera"))
            LOG_INFO("[RosLaunchManager]", QString("%1 ros2 node list 异步检测超时").arg(name));
        if (check && check->state() != QProcess::NotRunning)
            check->kill();
        finish(false);
    });

    connect(check, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [name, check, finish](int exitCode, QProcess::ExitStatus) mutable {
        QString output = QString::fromLocal8Bit(check->readAllStandardOutput()).trimmed();
        QString errOutput = QString::fromLocal8Bit(check->readAllStandardError()).trimmed();

        if (name != QStringLiteral("camera")) {
            LOG_INFO("[RosLaunchManager]", QString("%1 ros2 node list async exitCode=%2, stdout=%3, stderr=%4")
                     .arg(name).arg(exitCode).arg(output.left(200)).arg(errOutput.left(200)));
        }

        finish(outputContainsLaunchNode(name, output));
    });

    check->start("bash", QStringList() << "-c" << buildNodeListCommand());
    timeout->start(5000);
}

void RosLaunchManager::hasExpectedNodeAsync(
    const QString &name, std::function<void(bool)> callback) const
{
    if (!callback) return;

    if (!m_launches.contains(name))
    {
        QTimer::singleShot(0, const_cast<RosLaunchManager *>(this),
                           [callback = std::move(callback)]() {
            callback(false);
        });
        return;
    }

    QProcess *check = new QProcess(const_cast<RosLaunchManager *>(this));
    check->setProcessChannelMode(QProcess::SeparateChannels);

    QTimer *timeout = new QTimer(check);
    timeout->setSingleShot(true);

    auto completed = std::make_shared<bool>(false);
    auto finish = [callback = std::move(callback), check, timeout, completed](bool running) mutable {
        if (*completed) return;
        *completed = true;
        timeout->stop();
        callback(running);
        check->deleteLater();
    };

    connect(timeout, &QTimer::timeout, this, [check, finish]() mutable {
        if (check->state() != QProcess::NotRunning)
            check->kill();
        finish(false);
    });

    connect(check, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [name, check, finish](int, QProcess::ExitStatus) mutable {
        const QString output = QString::fromLocal8Bit(
            check->readAllStandardOutput()).trimmed();
        finish(outputContainsLaunchNode(name, output));
    });

    check->start("bash", QStringList() << "-c" << buildNodeListCommand());
    timeout->start(5000);
}
