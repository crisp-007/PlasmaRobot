#ifndef DEVICE_HEALTH_MODEL_H
#define DEVICE_HEALTH_MODEL_H

#include <QDateTime>
#include <QObject>
#include <QString>

class DeviceHealthModel : public QObject
{
    Q_OBJECT

public:
    enum class State {
        Disconnected,
        Starting,
        Healthy,
        Warning,
        Error,
        Stale
    };
    Q_ENUM(State)

    explicit DeviceHealthModel(QObject *parent = nullptr);

    State state() const { return m_state; }
    QString statusMessage() const { return m_statusMessage; }
    QString lastError() const { return m_lastError; }
    QDateTime lastUpdate() const { return m_lastUpdate; }
    bool processRunning() const { return m_processRunning; }
    bool dataActive() const { return m_dataActive; }

    void setState(State state, const QString &message = QString());
    void markDataReceived(const QString &message = QString());
    void markDataInactive(State state, const QString &message);
    void setProcessRunning(bool running);
    void setLastError(const QString &error);
    bool isDataStale(qint64 timeoutMs,
                     const QDateTime &now = QDateTime::currentDateTime()) const;

signals:
    void changed();

private:
    State m_state = State::Disconnected;
    QString m_statusMessage;
    QString m_lastError;
    QDateTime m_lastUpdate;
    bool m_processRunning = false;
    bool m_dataActive = false;
};

#endif // DEVICE_HEALTH_MODEL_H
