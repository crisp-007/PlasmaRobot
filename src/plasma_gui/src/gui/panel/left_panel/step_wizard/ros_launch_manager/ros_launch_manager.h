#ifndef ROSLAUNCHMANAGER_H
#define ROSLAUNCHMANAGER_H

#include <QObject>
#include <QProcess>
#include <QMap>
#include <QString>
#include <functional>

// ============================
// 单个 Launch 实例
// ============================
struct LaunchInstance
{
    QString name;        // camera / arm / planner
    QString package;
    QString launchFile;

    QMap<QString, QString> params;
    QProcess* process = nullptr;
    bool stopping = false;
    bool restartAfterStop = false;

    bool running() const {
        return process && process->state() == QProcess::Running;
    }
};

// ============================
// 多 Launch 管理器
// ============================
class RosLaunchManager : public QObject
{
    Q_OBJECT

public:
    explicit RosLaunchManager(QObject *parent = nullptr);

    // ===== 注册一个 launch =====
    void registerLaunch(const QString& name,
                        const QString& package,
                        const QString& launchFile);

    // ===== 参数设置 =====
    void setParam(const QString& launchName,
                  const QString& key,
                  const QString& value);

    // ===== 控制 launch =====
    void start(const QString& launchName);
    void stop(const QString& launchName);
    void restart(const QString& launchName);

    void startAll();
    void stopAll();
    void stopAllAndWait(int timeoutMs = 5000);

    // ===== 查询 =====
    bool isRunning(const QString& launchName) const;
    void isRunningAsync(const QString& launchName,
                        std::function<void(bool)> callback) const;
    void hasExpectedNodeAsync(const QString& launchName,
                              std::function<void(bool)> callback) const;

    QString buildCommand(const LaunchInstance& ins) const;

    QMap<QString, LaunchInstance> m_launches;
signals:
    void launchOutput(QString name, QString line);
    void launchStarted(QString name, QString cmd);
    void launchStopped(QString name);

private:



};

#endif // ROSLAUNCHMANAGER_H
