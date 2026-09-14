#include "serial_status_widget.h"

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace {

const QString kGroupStyle = QStringLiteral(R"(
    QGroupBox {
        background-color: #333333;
        border: 1px solid #4a4a4a;
        border-radius: 8px;
        margin-top: 10px;
        padding: 10px 5px 5px 5px;
        font-size: 13px;
        font-weight: 600;
        color: #5F9EA0;
    }
    QGroupBox::title {
        subcontrol-origin: margin;
        left: 14px;
        padding: 0 6px;
    }
)");

const QString kFieldStyle = QStringLiteral(
    "color: #c0c0c0; font-size: 12px; background: transparent;");

QString valueStyle(const QString &color = QStringLiteral("#ffffff"))
{
    return QStringLiteral(
        "QLabel { background-color: #4a4a4a; border: 1px solid #666666; "
        "border-radius: 4px; padding: 4px 6px; color: %1; "
        "font-family: 'Consolas', 'Monospace', monospace; font-size: 12px; }")
        .arg(color);
}

void addField(QGridLayout *layout,
              int row,
              int column,
              const QString &name,
              QWidget *value)
{
    auto *nameLabel = new QLabel(name);
    nameLabel->setStyleSheet(kFieldStyle);
    nameLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    layout->addWidget(nameLabel, row, column * 2);
    layout->addWidget(value, row, column * 2 + 1);
}

} // namespace

SerialStatusWidget::SerialStatusWidget(QWidget *parent)
    : QWidget(parent)
{
    setupUi();
}

QLabel *SerialStatusWidget::createValueLabel(const QString &text)
{
    auto *label = new QLabel(text);
    label->setStyleSheet(valueStyle());
    label->setAlignment(Qt::AlignCenter);
    label->setMinimumHeight(24);
    return label;
}

void SerialStatusWidget::setupUi()
{
    auto *mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(6);

    auto *connectionGroup = new QGroupBox(QStringLiteral("连接信息"));
    connectionGroup->setStyleSheet(kGroupStyle);
    auto *connectionLayout = new QGridLayout(connectionGroup);
    connectionLayout->setContentsMargins(8, 12, 8, 8);
    connectionLayout->setHorizontalSpacing(8);
    connectionLayout->setVerticalSpacing(4);

    auto *connectionContainer = new QWidget(connectionGroup);
    auto *connectionRow = new QHBoxLayout(connectionContainer);
    connectionRow->setContentsMargins(0, 0, 0, 0);
    connectionRow->setSpacing(6);
    m_connectionDot = new QLabel(connectionContainer);
    m_connectionDot->setFixedSize(12, 12);
    m_connectionValue = new QLabel(QStringLiteral("未连接"), connectionContainer);
    connectionRow->addWidget(m_connectionDot);
    connectionRow->addWidget(m_connectionValue);
    connectionRow->addStretch();

    m_portValue = createValueLabel();
    m_settingsValue = createValueLabel();
    addField(connectionLayout, 0, 0, QStringLiteral("连接状态"), connectionContainer);
    addField(connectionLayout, 0, 1, QStringLiteral("端口"), m_portValue);
    auto *settingsTitle = new QLabel(QStringLiteral("通信参数"));
    settingsTitle->setStyleSheet(kFieldStyle);
    settingsTitle->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    connectionLayout->addWidget(settingsTitle, 1, 0);
    connectionLayout->setColumnStretch(1, 1);
    connectionLayout->setColumnStretch(3, 1);
    connectionLayout->addWidget(m_settingsValue, 1, 1, 1, 3);
    mainLayout->addWidget(connectionGroup);

    auto *statisticsGroup = new QGroupBox(QStringLiteral("通信统计"));
    statisticsGroup->setStyleSheet(kGroupStyle);
    auto *statisticsLayout = new QGridLayout(statisticsGroup);
    statisticsLayout->setContentsMargins(8, 12, 8, 8);
    statisticsLayout->setHorizontalSpacing(8);
    statisticsLayout->setVerticalSpacing(4);
    m_receivedValue = createValueLabel(QStringLiteral("0 B"));
    m_sentValue = createValueLabel(QStringLiteral("0 B"));
    m_lastPacketValue = createValueLabel();
    addField(statisticsLayout, 0, 0, QStringLiteral("接收"), m_receivedValue);
    addField(statisticsLayout, 0, 1, QStringLiteral("发送"), m_sentValue);
    addField(statisticsLayout, 0, 2, QStringLiteral("最后收包"), m_lastPacketValue);
    statisticsLayout->setColumnStretch(1, 1);
    statisticsLayout->setColumnStretch(3, 1);
    statisticsLayout->setColumnStretch(5, 1);
    mainLayout->addWidget(statisticsGroup);

    auto *deviceGroup = new QGroupBox(QStringLiteral("下位机状态"));
    deviceGroup->setStyleSheet(kGroupStyle);
    auto *deviceLayout = new QGridLayout(deviceGroup);
    deviceLayout->setContentsMargins(8, 12, 8, 8);
    deviceLayout->setHorizontalSpacing(8);
    deviceLayout->setVerticalSpacing(4);

    m_emergencyStopValue = createValueLabel(QStringLiteral("未知"));
    m_plasmaRelayValue = createValueLabel(QStringLiteral("未知"));
    m_voltageRelayValue = createValueLabel(QStringLiteral("未知"));
    m_heliumFlowRelayValue = createValueLabel(QStringLiteral("未知"));
    m_heliumValveValue = createValueLabel(QStringLiteral("未知"));
    m_argonFlowRelayValue = createValueLabel(QStringLiteral("未知"));
    m_argonValveValue = createValueLabel(QStringLiteral("未知"));
    m_heliumPressureValue = createValueLabel(QStringLiteral("-- MPa"));
    m_argonPressureValue = createValueLabel(QStringLiteral("-- MPa"));
    m_voltageOutputValue = createValueLabel(QStringLiteral("0"));
    m_heliumOutputValue = createValueLabel(QStringLiteral("0"));
    m_argonOutputValue = createValueLabel(QStringLiteral("0"));

    addField(deviceLayout, 0, 0, QStringLiteral("急停"), m_emergencyStopValue);
    addField(deviceLayout, 0, 1, QStringLiteral("等离子继电器"), m_plasmaRelayValue);
    addField(deviceLayout, 0, 2, QStringLiteral("调压继电器"), m_voltageRelayValue);
    addField(deviceLayout, 1, 0, QStringLiteral("氦气流量计"), m_heliumFlowRelayValue);
    addField(deviceLayout, 1, 1, QStringLiteral("氦气阀"), m_heliumValveValue);
    addField(deviceLayout, 1, 2, QStringLiteral("氦气压力"), m_heliumPressureValue);
    addField(deviceLayout, 2, 0, QStringLiteral("氩气流量计"), m_argonFlowRelayValue);
    addField(deviceLayout, 2, 1, QStringLiteral("氩气阀"), m_argonValveValue);
    addField(deviceLayout, 2, 2, QStringLiteral("氩气压力"), m_argonPressureValue);
    addField(deviceLayout, 3, 0, QStringLiteral("调压输出"), m_voltageOutputValue);
    addField(deviceLayout, 3, 1, QStringLiteral("氦气输出"), m_heliumOutputValue);
    addField(deviceLayout, 3, 2, QStringLiteral("氩气输出"), m_argonOutputValue);
    deviceLayout->setColumnStretch(1, 1);
    deviceLayout->setColumnStretch(3, 1);
    deviceLayout->setColumnStretch(5, 1);
    mainLayout->addWidget(deviceGroup);

    setConnectionState(false);
}

void SerialStatusWidget::setConnectionState(bool connected)
{
    const QString color = connected ? QStringLiteral("#00ff88")
                                    : QStringLiteral("#ff4444");
    m_connectionDot->setStyleSheet(QStringLiteral(
        "background-color: %1; border-radius: 6px; border: 1px solid #333;")
        .arg(color));
    m_connectionValue->setText(connected ? QStringLiteral("已连接")
                                         : QStringLiteral("未连接"));
    m_connectionValue->setStyleSheet(QStringLiteral(
        "color: %1; font-size: 12px; font-weight: bold; background: transparent;")
        .arg(color));
}

void SerialStatusWidget::setSerialSettings(const QString &portName,
                                           qint32 baudRate,
                                           const QString &frameFormat)
{
    m_portValue->setText(portName.isEmpty() ? QStringLiteral("--") : portName);
    m_settingsValue->setText(QStringLiteral("%1 baud / %2")
        .arg(baudRate)
        .arg(frameFormat.isEmpty() ? QStringLiteral("--") : frameFormat));
}

void SerialStatusWidget::setStatistics(qint64 receivedBytes,
                                       qint64 sentBytes,
                                       const QDateTime &lastPacketTime)
{
    m_receivedValue->setText(QStringLiteral("%1 B").arg(receivedBytes));
    m_sentValue->setText(QStringLiteral("%1 B").arg(sentBytes));
    m_lastPacketValue->setText(lastPacketTime.isValid()
        ? lastPacketTime.toString(QStringLiteral("HH:mm:ss.zzz"))
        : QStringLiteral("--"));
}

void SerialStatusWidget::setPressureValues(double heliumPressureMpa, double argonPressureMpa)
{
    m_heliumPressureValue->setText(QStringLiteral("%1 MPa").arg(heliumPressureMpa, 0, 'f', 3));
    m_argonPressureValue->setText(QStringLiteral("%1 MPa").arg(argonPressureMpa, 0, 'f', 3));
}

void SerialStatusWidget::clearPressureValues()
{
    m_heliumPressureValue->setText(QStringLiteral("-- MPa"));
    m_argonPressureValue->setText(QStringLiteral("-- MPa"));
}

void SerialStatusWidget::setBinaryState(QLabel *label,
                                        bool active,
                                        const QString &activeText,
                                        const QString &inactiveText)
{
    label->setText(active ? activeText : inactiveText);
    label->setStyleSheet(valueStyle(active ? QStringLiteral("#00ff88")
                                           : QStringLiteral("#aaaaaa")));
}

void SerialStatusWidget::setDeviceStatus(bool emergencyStop,
                                         bool plasmaRelay,
                                         bool voltageRelay,
                                         bool heliumFlowRelay,
                                         bool heliumValve,
                                         bool argonFlowRelay,
                                         bool argonValve)
{
    m_emergencyStopValue->setText(emergencyStop ? QStringLiteral("已触发")
                                                : QStringLiteral("正常"));
    m_emergencyStopValue->setStyleSheet(valueStyle(
        emergencyStop ? QStringLiteral("#ff4444") : QStringLiteral("#00ff88")));
    setBinaryState(m_plasmaRelayValue, plasmaRelay);
    setBinaryState(m_voltageRelayValue, voltageRelay);
    setBinaryState(m_heliumFlowRelayValue, heliumFlowRelay);
    setBinaryState(m_heliumValveValue, heliumValve);
    setBinaryState(m_argonFlowRelayValue, argonFlowRelay);
    setBinaryState(m_argonValveValue, argonValve);
}

void SerialStatusWidget::setOutputValues(quint16 voltageOutput,
                                         quint16 heliumOutput,
                                         quint16 argonOutput)
{
    m_voltageOutputValue->setText(QString::number(voltageOutput));
    m_heliumOutputValue->setText(QString::number(heliumOutput));
    m_argonOutputValue->setText(QString::number(argonOutput));
}
