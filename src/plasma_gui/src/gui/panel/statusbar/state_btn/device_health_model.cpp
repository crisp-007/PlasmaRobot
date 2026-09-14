#include "device_health_model.h"

DeviceHealthModel::DeviceHealthModel(QObject *parent)
    : QObject(parent)
{
}

void DeviceHealthModel::setState(State state, const QString &message)
{
    if (m_state == state && m_statusMessage == message)
        return;

    m_state = state;
    m_statusMessage = message;
    emit changed();
}

void DeviceHealthModel::markDataReceived(const QString &message)
{
    m_lastUpdate = QDateTime::currentDateTime();
    m_dataActive = true;

    const QString effectiveMessage = message.isEmpty()
        ? QStringLiteral("数据正常")
        : message;
    setState(State::Healthy, effectiveMessage);
}

void DeviceHealthModel::markDataInactive(State state, const QString &message)
{
    m_dataActive = false;
    setState(state, message);
}

void DeviceHealthModel::setProcessRunning(bool running)
{
    if (m_processRunning == running)
        return;

    m_processRunning = running;
    emit changed();
}

void DeviceHealthModel::setLastError(const QString &error)
{
    if (m_lastError == error)
        return;

    m_lastError = error;
    emit changed();
}

bool DeviceHealthModel::isDataStale(qint64 timeoutMs, const QDateTime &now) const
{
    return m_dataActive && m_lastUpdate.isValid()
        && m_lastUpdate.msecsTo(now) > timeoutMs;
}
