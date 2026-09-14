#ifndef SERIAL_STATUS_WIDGET_H
#define SERIAL_STATUS_WIDGET_H

#include <QDateTime>
#include <QLabel>
#include <QWidget>

class SerialStatusWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SerialStatusWidget(QWidget *parent = nullptr);

    void setConnectionState(bool connected);
    void setSerialSettings(const QString &portName,
                           qint32 baudRate,
                           const QString &frameFormat);
    void setStatistics(qint64 receivedBytes,
                       qint64 sentBytes,
                       const QDateTime &lastPacketTime);
    void setPressureValues(double heliumPressureMpa, double argonPressureMpa);
    void clearPressureValues();
    void setDeviceStatus(bool emergencyStop,
                         bool plasmaRelay,
                         bool voltageRelay,
                         bool heliumFlowRelay,
                         bool heliumValve,
                         bool argonFlowRelay,
                         bool argonValve);
    void setOutputValues(quint16 voltageOutput,
                         quint16 heliumOutput,
                         quint16 argonOutput);

private:
    void setupUi();
    QLabel *createValueLabel(const QString &text = QStringLiteral("--"));
    void setBinaryState(QLabel *label,
                        bool active,
                        const QString &activeText = QStringLiteral("开启"),
                        const QString &inactiveText = QStringLiteral("关闭"));

    QLabel *m_connectionDot = nullptr;
    QLabel *m_connectionValue = nullptr;
    QLabel *m_portValue = nullptr;
    QLabel *m_settingsValue = nullptr;
    QLabel *m_receivedValue = nullptr;
    QLabel *m_sentValue = nullptr;
    QLabel *m_lastPacketValue = nullptr;

    QLabel *m_emergencyStopValue = nullptr;
    QLabel *m_plasmaRelayValue = nullptr;
    QLabel *m_voltageRelayValue = nullptr;
    QLabel *m_heliumFlowRelayValue = nullptr;
    QLabel *m_heliumValveValue = nullptr;
    QLabel *m_argonFlowRelayValue = nullptr;
    QLabel *m_argonValveValue = nullptr;
    QLabel *m_heliumPressureValue = nullptr;
    QLabel *m_argonPressureValue = nullptr;
    QLabel *m_voltageOutputValue = nullptr;
    QLabel *m_heliumOutputValue = nullptr;
    QLabel *m_argonOutputValue = nullptr;
};

#endif // SERIAL_STATUS_WIDGET_H
