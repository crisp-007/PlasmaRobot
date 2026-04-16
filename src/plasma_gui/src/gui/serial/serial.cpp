#include "serial.h"
#include <QMessageBox>
#include <QDebug>
#include <QDateTime>
#include <QInputDialog>
#include <QOperatingSystemVersion>
#include <QShowEvent>

#ifdef Q_OS_WIN
#include <windows.h>
#include <dwmapi.h>
#endif

namespace {

bool isPermissionDeniedError(const QSerialPort *serialPort)
{
    if (!serialPort) {
        return false;
    }

    if (serialPort->error() == QSerialPort::PermissionError) {
        return true;
    }

    const QString errorText = serialPort->errorString();
    return errorText.contains("permission", Qt::CaseInsensitive)
           || errorText.contains("denied", Qt::CaseInsensitive)
           || errorText.contains(QStringLiteral("拒绝"));
}

QString buildOpenPortErrorMessage(const QString &portName, const QSerialPort *serialPort)
{
    QString message = QString("无法打开串口 %1\n错误: %2")
                          .arg(portName, serialPort ? serialPort->errorString() : QString("未知错误"));

#ifdef Q_OS_LINUX
    if (isPermissionDeniedError(serialPort)) {
        const QString devicePath = portName.startsWith("/dev/") ? portName : QString("/dev/%1").arg(portName);
        message += QString(
            "\n\n检测到权限不足。常见原因：\n"
            "1) 当前用户不在串口访问组（dialout/tty）\n"
            "2) 串口被其他进程占用\n\n"
            "建议在终端执行：\n"
            "- groups\n"
            "- sudo usermod -aG dialout $USER\n"
            "- sudo usermod -aG tty $USER\n"
            "- lsof %1\n\n"
            "完成加组后请重新登录系统，再重试连接。")
                       .arg(devicePath);
    }
#endif

    return message;
}

} // namespace

/**
 * @brief 构造函数
 * @param parent 父窗口指针
 */
Serial::Serial(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Serial)
    , m_serialPort(nullptr)
    , m_statusTimer(nullptr)
    , m_baudRate(115200)
    , m_dataBits(QSerialPort::Data8)
    , m_parity(QSerialPort::NoParity)
    , m_stopBits(QSerialPort::OneStop)
    , m_flowControl(QSerialPort::NoFlowControl)
    , m_bytesReceived(0)
    , m_bytesSent(0)
    , m_isConnected(false)
    , m_logModeEnabled(false)
    , m_heliumPressure(0)
    , m_argonPressure(0)
    , m_instantSendMode(false)
    , m_instantSendTimer(new QTimer(this))
{
    ui->setupUi(this);
    
    // 暗色标题栏将在窗口显示后设置
    
    // 设置即时发送定时器
    m_instantSendTimer->setInterval(1000); // 1秒间隔
    connect(m_instantSendTimer, &QTimer::timeout, this, &Serial::updateControlPacketDisplay);
    
    setupUI();
    initSerialPort();
    connectSignals();
    refreshPortList();
}

/**
 * @brief 析构函数
 */
Serial::~Serial()
{
    if (m_serialPort && m_serialPort->isOpen()) {
        m_serialPort->close();
    }
    delete m_serialPort;
    delete ui;
}

/**
 * @brief 获取当前串口连接状态
 */
bool Serial::isConnected() const
{
    return m_isConnected;
}

/**
 * @brief 获取当前串口名称
 */
QString Serial::getCurrentPortName() const
{
    return m_portName;
}

/**
 * @brief 初始化UI界面
 */
void Serial::setupUI()
{
    setWindowTitle("串口通信设置");
    setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint | Qt::WindowMinimizeButtonHint);
    
    // 设置窗口图标（如果有的话）
    // setWindowIcon(QIcon(":/icons/serial.png"));
    
    // 设置checkbox初始背景色为红色
    QString redStyle = "QCheckBox { background-color: #F44336; border-radius: 4px; padding: 2px; }";
    ui->plasmaRelayCheckBox->setStyleSheet(redStyle);
    ui->voltageRelayCheckBox->setStyleSheet(redStyle);
    ui->heliumFlowMeterCheckBox->setStyleSheet(redStyle);
    ui->heliumValveCheckBox->setStyleSheet(redStyle);
    ui->argonFlowMeterCheckBox->setStyleSheet(redStyle);
    ui->argonValveCheckBox->setStyleSheet(redStyle);
    
    // 设置十六进制显示和日志模式按钮的初始选中状态
    ui->hexReceiveCheckBox->setChecked(true);
    ui->logModeCheckBox->setChecked(true);
    
    // 初始化日志模式状态变量
    m_logModeEnabled = true;
    
    // 设置即时发送模式按钮的初始状态为即时发送模式
    m_instantSendMode = true;
    ui->instantSendModeButton->setText("即时发送模式");
    ui->instantSendModeButton->setChecked(true);
    ui->sendButton->setEnabled(false);
    ui->sendButton->setStyleSheet("QPushButton { background-color: #666666; color: #999999; }");
    
    // 启动即时发送定时器
    m_instantSendTimer->start();
}

/**
 * @brief 初始化串口参数
 */
void Serial::initSerialPort()
{
    m_serialPort = new QSerialPort(this);
    
    // 创建状态更新定时器
    m_statusTimer = new QTimer(this);
    m_statusTimer->setInterval(1000); // 每秒更新一次
    connect(m_statusTimer, &QTimer::timeout, this, &Serial::updateStatusInfo);
}

/**
 * @brief 连接信号槽
 */
void Serial::connectSignals()
{
    // 串口信号连接
    connect(m_serialPort, &QSerialPort::readyRead, this, &Serial::onDataReceived);
    connect(m_serialPort, QOverload<QSerialPort::SerialPortError>::of(&QSerialPort::errorOccurred),
            this, &Serial::onSerialError);
    
    // 基本按钮信号连接
    connect(ui->refreshButton, &QPushButton::clicked, this, &Serial::refreshPortList);
    connect(ui->connectButton, &QPushButton::clicked, this, &Serial::toggleConnection);
    connect(ui->sendButton, &QPushButton::clicked, this, &Serial::sendData);
    connect(ui->clearReceiveButton, &QPushButton::clicked, this, &Serial::clearReceiveArea);
    connect(ui->clearSendButton, &QPushButton::clicked, this, &Serial::clearSendArea);
    
    // 日志模式复选框信号连接
    connect(ui->logModeCheckBox, &QCheckBox::toggled, [this](bool checked) {
        m_logModeEnabled = checked;
    });
    
    // 等离子控制系统信号连接
    connect(ui->sendControlPacketButton, &QPushButton::clicked, this, &Serial::sendControlPacket);
    connect(ui->loadTestPacketButton, &QPushButton::clicked, this, &Serial::loadTestPacket);
    connect(ui->instantSendModeButton, &QPushButton::clicked, this, &Serial::toggleInstantSendMode);
    connect(ui->emergencyStopButton, &QPushButton::clicked, this, &Serial::emergencyStop);
    connect(ui->resetSystemButton, &QPushButton::clicked, this, &Serial::resetSystem);
    
    // 实时数据包生成连接
    connect(ui->plasmaRelayCheckBox, &QCheckBox::toggled, this, &Serial::updateControlPacketDisplay);
    connect(ui->voltageRelayCheckBox, &QCheckBox::toggled, this, &Serial::updateControlPacketDisplay);
    connect(ui->plasmaValueSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &Serial::updateControlPacketDisplay);
    connect(ui->heliumFlowMeterCheckBox, &QCheckBox::toggled, this, &Serial::updateControlPacketDisplay);
    connect(ui->heliumValveCheckBox, &QCheckBox::toggled, this, &Serial::updateControlPacketDisplay);
    connect(ui->heliumValueSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &Serial::updateControlPacketDisplay);
    connect(ui->argonFlowMeterCheckBox, &QCheckBox::toggled, this, &Serial::updateControlPacketDisplay);
    connect(ui->argonValveCheckBox, &QCheckBox::toggled, this, &Serial::updateControlPacketDisplay);
    connect(ui->argonValueSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &Serial::updateControlPacketDisplay);
    
    // UI联动信号连接
    connect(this, &Serial::deviceStatusUpdated, this, &Serial::updateDeviceStatus);
    connect(this, &Serial::outputValuesUpdated, this, &Serial::updateOutputValues);
    
    // 初始化时显示默认数据包
    updateControlPacketDisplay();
}

/**
 * @brief 刷新可用串口列表
 */
void Serial::refreshPortList()
{
    ui->portComboBox->clear();
    
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos) {
        QString portInfo = QString("%1 (%2)").arg(info.portName(), info.description());
        ui->portComboBox->addItem(portInfo, info.portName());
    }
    
    if (ui->portComboBox->count() == 0) {
        ui->portComboBox->addItem("无可用串口");
        ui->connectButton->setEnabled(false);
    } else {
        ui->connectButton->setEnabled(true);
    }
}

/**
 * @brief 连接/断开串口
 */
void Serial::toggleConnection()
{
    if (!m_isConnected) {
        // 连接串口
        if (ui->portComboBox->count() == 0 || ui->portComboBox->currentText() == "无可用串口") {
            QMessageBox::warning(this, "警告", "没有可用的串口！");
            return;
        }
        
        m_portName = ui->portComboBox->currentData().toString();
        applySerialSettings();
        
        m_serialPort->setPortName(m_portName);
        
        if (m_serialPort->open(QIODevice::ReadWrite)) {
            m_isConnected = true;
            ui->connectButton->setText("断开");
            ui->connectButton->setStyleSheet("QPushButton { background-color: #ff6b6b; }");
            ui->statusLabel->setText("状态: 已连接");
            ui->statusLabel->setStyleSheet("color: #4CAF50;");
            
            // 禁用参数设置控件
            ui->portComboBox->setEnabled(false);
            ui->baudRateComboBox->setEnabled(false);
            ui->dataBitsComboBox->setEnabled(false);
            ui->parityComboBox->setEnabled(false);
            ui->stopBitsComboBox->setEnabled(false);
            ui->flowControlComboBox->setEnabled(false);
            
            m_statusTimer->start();
            emit connectionStatusChanged(true);
        } else {
            QMessageBox::critical(this, "错误", buildOpenPortErrorMessage(m_portName, m_serialPort));
        }
    } else {
        // 断开串口
        m_serialPort->close();
        m_isConnected = false;
        ui->connectButton->setText("连接");
        ui->connectButton->setStyleSheet("");
        ui->statusLabel->setText("状态: 未连接");
        ui->statusLabel->setStyleSheet("color: #f44336;");
        
        // 启用参数设置控件
        ui->portComboBox->setEnabled(true);
        ui->baudRateComboBox->setEnabled(true);
        ui->dataBitsComboBox->setEnabled(true);
        ui->parityComboBox->setEnabled(true);
        ui->stopBitsComboBox->setEnabled(true);
        ui->flowControlComboBox->setEnabled(true);
        
        m_statusTimer->stop();
        emit connectionStatusChanged(false);
    }
    
    updateConnectionStatus();
}

/**
 * @brief 发送数据
 */
void Serial::sendData()
{
    if (!m_isConnected) {
        QMessageBox::warning(this, "警告", "串口未连接！");
        return;
    }
    
    QString text = ui->sendTextEdit->toPlainText();
    if (text.isEmpty()) {
        return;
    }
    
    QByteArray data;
    if (ui->hexSendCheckBox->isChecked()) {
        // 十六进制发送
        text = text.replace(" ", "").replace("\n", "").replace("\r", "");
        for (int i = 0; i < text.length(); i += 2) {
            bool ok;
            quint8 byte = text.mid(i, 2).toUInt(&ok, 16);
            if (ok) {
                data.append(byte);
            }
        }
    } else {
        // 文本发送
        data = text.toUtf8();
        if (ui->newLineCheckBox->isChecked()) {
            data.append("\r\n");
        }
    }
    
    qint64 bytesWritten = m_serialPort->write(data);
    if (bytesWritten != -1) {
        m_bytesSent += bytesWritten;
        if (ui->autoClearSendCheckBox->isChecked()) {
            ui->sendTextEdit->clear();
        }
    } else {
        QMessageBox::critical(this, "错误", "数据发送失败！");
    }
}

/**
 * @brief 清空接收区
 */
void Serial::clearReceiveArea()
{
    ui->receiveTextEdit->clear();
    m_bytesReceived = 0;
}

/**
 * @brief 清空发送区
 */
void Serial::clearSendArea()
{
    ui->sendTextEdit->clear();
}

/**
 * @brief 串口数据接收处理
 */
void Serial::onDataReceived()
{
    QByteArray data = m_serialPort->readAll();
    m_bytesReceived += data.size();
    
    // 将接收到的数据添加到缓冲区
    m_receiveBuffer.append(data);
    
    // 尝试解析数据包
    parseReceivedPacket(m_receiveBuffer);
    
    QString displayText;
    
    // 如果启用日志模式，添加时间戳
    if (m_logModeEnabled) {
        QDateTime currentTime = QDateTime::currentDateTime();
        QString timestamp = QString("[%1]\n").arg(currentTime.toString("hh:mm:ss:zzz"));
        displayText += timestamp;
    }
    
    QString contentText;
    if (ui->hexReceiveCheckBox->isChecked()) {
        // 十六进制显示
        for (char byte : data) {
            contentText += QString("%1 ").arg(static_cast<quint8>(byte), 2, 16, QChar('0')).toUpper();
        }
    } else {
        // 文本显示 - 处理系统编码（通常为GB2312/GBK）
        contentText = QString::fromLocal8Bit(data);
        // 如果显示异常，可能需要尝试UTF-8
        if (contentText.contains(QChar::ReplacementCharacter)) {
            contentText = QString::fromUtf8(data);
        }
    }
    
    displayText += contentText;
    
    // 添加换行（默认换行显示）
    if (!displayText.endsWith("\n")) {
        displayText += "\n";
    }
    
    ui->receiveTextEdit->insertPlainText(displayText);
    
    // 自动滚动到底部
    QTextCursor cursor = ui->receiveTextEdit->textCursor();
    cursor.movePosition(QTextCursor::End);
    ui->receiveTextEdit->setTextCursor(cursor);
    
    emit dataReceived(data);
}

/**
 * @brief 串口错误处理
 */
void Serial::onSerialError(QSerialPort::SerialPortError error)
{
    if (error != QSerialPort::NoError) {
        QString errorString = m_serialPort->errorString();
        ui->statusLabel->setText(QString("错误: %1").arg(errorString));
        ui->statusLabel->setStyleSheet("color: #f44336;");
        
        if (m_isConnected) {
            // 如果发生严重错误，自动断开连接
            toggleConnection();
        }
    }
}

/**
 * @brief 更新连接状态显示
 */
void Serial::updateConnectionStatus()
{
    // 这个函数可以用来更新其他状态相关的UI元素
}

/**
 * @brief 应用串口参数设置
 */
void Serial::applySerialSettings()
{
    // 波特率
    m_baudRate = ui->baudRateComboBox->currentText().toInt();
    m_serialPort->setBaudRate(m_baudRate);
    
    // 数据位
    switch (ui->dataBitsComboBox->currentIndex()) {
        case 0: m_dataBits = QSerialPort::Data5; break;
        case 1: m_dataBits = QSerialPort::Data6; break;
        case 2: m_dataBits = QSerialPort::Data7; break;
        case 3: m_dataBits = QSerialPort::Data8; break;
        default: m_dataBits = QSerialPort::Data8; break;
    }
    m_serialPort->setDataBits(m_dataBits);
    
    // 校验位
    switch (ui->parityComboBox->currentIndex()) {
        case 0: m_parity = QSerialPort::NoParity; break;
        case 1: m_parity = QSerialPort::EvenParity; break;
        case 2: m_parity = QSerialPort::OddParity; break;
        case 3: m_parity = QSerialPort::SpaceParity; break;
        case 4: m_parity = QSerialPort::MarkParity; break;
        default: m_parity = QSerialPort::NoParity; break;
    }
    m_serialPort->setParity(m_parity);
    
    // 停止位
    switch (ui->stopBitsComboBox->currentIndex()) {
        case 0: m_stopBits = QSerialPort::OneStop; break;
        case 1: m_stopBits = QSerialPort::OneAndHalfStop; break;
        case 2: m_stopBits = QSerialPort::TwoStop; break;
        default: m_stopBits = QSerialPort::OneStop; break;
    }
    m_serialPort->setStopBits(m_stopBits);
    
    // 流控制
    switch (ui->flowControlComboBox->currentIndex()) {
        case 0: m_flowControl = QSerialPort::NoFlowControl; break;
        case 1: m_flowControl = QSerialPort::HardwareControl; break;
        case 2: m_flowControl = QSerialPort::SoftwareControl; break;
        default: m_flowControl = QSerialPort::NoFlowControl; break;
    }
    m_serialPort->setFlowControl(m_flowControl);
}

/**
 * @brief 更新状态栏信息
 */
void Serial::updateStatusInfo()
{
    if (m_isConnected) {
        QString info = QString("接收: %1 字节 | 发送: %2 字节")
                      .arg(m_bytesReceived)
                      .arg(m_bytesSent);
        ui->statisticsLabel->setText(info);
    }
}

/**
 * @brief 发送等离子控制指令
 */
void Serial::sendControlPacket()
{
    QByteArray packet = buildControlPacket();
    
    // 将数据包转换为十六进制字符串显示在发送区
    QString hexString;
    for (int i = 0; i < packet.size(); ++i) {
        hexString += QString("%1 ").arg(static_cast<quint8>(packet[i]), 2, 16, QChar('0')).toUpper();
    }
    ui->sendTextEdit->setPlainText(hexString.trimmed());
    
    // 设置为十六进制发送模式
    ui->hexSendCheckBox->setChecked(true);
    
    if (!m_isConnected) {
        QMessageBox::warning(this, "警告", "串口未连接！数据包已生成并显示在发送区。");
        return;
    }
    
    // 发送数据包
    sendData();
}

/**
 * @brief 加载测试数据包
 */
void Serial::loadTestPacket()
{
    QStringList testPackets = {
        "FF FE 00 11 E8 03 11 F4 01 11 2C 01 3D 04 00 00", // 正常工作状态
        "FF FE F8 00 00 00 00 00 00 00 00 00 F5 02 00 00", // 紧急停止状态
        "FF FE 00 10 64 00 10 C8 00 00 00 00 49 03 00 00", // 部分设备开启
        "FF FE 00 11 FF 0F 11 FF 0F 11 FF 0F 5A 05 00 00", // 高输出值测试
        "FF FE 00 11 01 00 11 01 00 11 01 00 33 02 00 00"  // 最小输出值测试
    };
    
    QStringList descriptions = {
        "正常工作状态",
        "紧急停止状态", 
        "部分设备开启",
        "高输出值测试",
        "最小输出值测试"
    };
    
    bool ok;
    QString item = QInputDialog::getItem(this, "选择测试数据包", "请选择要加载的测试数据包:", descriptions, 0, false, &ok);
    
    if (ok && !item.isEmpty()) {
        int index = descriptions.indexOf(item);
        if (index >= 0 && index < testPackets.size()) {
            ui->sendTextEdit->setPlainText(testPackets[index]);
            ui->hexSendCheckBox->setChecked(true);
        }
    }
}

/**
 * @brief 紧急停止
 */
void Serial::emergencyStop()
{
    if (!m_isConnected) {
        QMessageBox::warning(this, "警告", "请先连接串口！");
        return;
    }
    resetSystem();
    // 构建紧急停止数据包
    QByteArray packet(16, 0);
    
    // 包头 0xFEFF (小端序)
    packet[0] = 0xFF;
    packet[1] = 0xFE;
    
    // 紧急停止标志
    packet[2] = 0xF8;
    
    // 其他字节保持为0
    
    // 计算校验和
    quint32 checksum = calculateChecksum(packet.left(12));
    packet[12] = checksum & 0xFF;
    packet[13] = (checksum >> 8) & 0xFF;
    packet[14] = (checksum >> 16) & 0xFF;
    packet[15] = (checksum >> 24) & 0xFF;
    
    // 转换为十六进制字符串
    QString hexString;
    for (int i = 0; i < packet.size(); ++i) {
        hexString += QString("%1 ").arg(static_cast<quint8>(packet[i]), 2, 16, QChar('0')).toUpper();
    }
    
    ui->sendTextEdit->setPlainText(hexString.trimmed());
    ui->hexSendCheckBox->setChecked(true);
    
    // 立即发送
    sendData();
    
    // QMessageBox::information(this, "紧急停止", "紧急停止指令已发送！");
}

/**
 * @brief 系统复位
 */
void Serial::resetSystem()
{
    // 重置所有控件到默认状态
    ui->plasmaRelayCheckBox->setChecked(false);
    ui->voltageRelayCheckBox->setChecked(false);
    ui->heliumFlowMeterCheckBox->setChecked(false);
    ui->heliumValveCheckBox->setChecked(false);
    ui->argonFlowMeterCheckBox->setChecked(false);
    ui->argonValveCheckBox->setChecked(false);
    
    ui->plasmaValueSpinBox->setValue(0);
    ui->heliumValueSpinBox->setValue(0);
    ui->argonValueSpinBox->setValue(0);
    
    // QMessageBox::information(this, "系统复位", "控制面板已重置到默认状态！");
}

/**
 * @brief 构建控制数据包
 * @return 16字节的控制数据包
 */
QByteArray Serial::buildControlPacket()
{
    QByteArray packet(16, 0);
    
    // 包头 0xFEFF (小端序发送: FF FE)
    packet[0] = 0xFF;
    packet[1] = 0xFE;
    
    // 紧急停止标志 (0x00=正常, 0xF8=紧急停止)
    packet[2] = 0x00; // 正常状态
    
    // 等离子电源继电器状态 (高4位=等离子电源, 低4位=调压器)
    quint8 volRelay = 0;
    if (ui->plasmaRelayCheckBox->isChecked()) {
        volRelay |= 0x10; // 高4位
    }
    if (ui->voltageRelayCheckBox->isChecked()) {
        volRelay |= 0x01; // 低4位
    }
    packet[3] = static_cast<char>(volRelay);
    
    // 等离子电源输出值 (小端序)
    quint16 plasmaValue = static_cast<quint16>(ui->plasmaValueSpinBox->value());
    packet[4] = plasmaValue & 0xFF;
    packet[5] = (plasmaValue >> 8) & 0xFF;
    
    // 氦气流量继电器状态 (高4位=流量计, 低4位=电磁阀)
    quint8 heRelay = 0;
    if (ui->heliumFlowMeterCheckBox->isChecked()) {
        heRelay |= 0x10; // 高4位
    }
    if (ui->heliumValveCheckBox->isChecked()) {
        heRelay |= 0x01; // 低4位
    }
    packet[6] = heRelay;
    
    // 氦气输出值 (小端序)
    quint16 heliumValue = static_cast<quint16>(ui->heliumValueSpinBox->value());
    packet[7] = heliumValue & 0xFF;
    packet[8] = (heliumValue >> 8) & 0xFF;
    
    // 氩气流量继电器状态 (高4位=流量计, 低4位=电磁阀)
    quint8 arRelay = 0;
    if (ui->argonFlowMeterCheckBox->isChecked()) {
        arRelay |= 0x10; // 高4位
    }
    if (ui->argonValveCheckBox->isChecked()) {
        arRelay |= 0x01; // 低4位
    }
    packet[9] = arRelay;
    
    // 氩气输出值 (小端序)
    quint16 argonValue = static_cast<quint16>(ui->argonValueSpinBox->value());
    packet[10] = argonValue & 0xFF;
    packet[11] = (argonValue >> 8) & 0xFF;
    
    // 计算校验和 (按照下位机的方式：各字节值相加，结果为32位footer)
    quint16 header = (packet[1]<<8|packet[0]);
    quint8 emergencyStop = 0x00;
    
    // 打印每个字节的值用于调试
    qDebug() << "Packet bytes for checksum calculation:";
    for (int i = 0; i < 12; ++i) {
        qDebug() << QString("packet[%1] = 0x%2 (%3)").arg(i).arg(static_cast<quint8>(packet[i]), 2, 16, QChar('0')).toUpper().arg(static_cast<quint8>(packet[i]));
    }
    
    quint32 footer = static_cast<quint8>(packet[0]) + static_cast<quint8>(packet[1]) + static_cast<quint8>(packet[2]) + static_cast<quint8>(packet[3]) + static_cast<quint8>(packet[4]) + static_cast<quint8>(packet[5]) + static_cast<quint8>(packet[6]) + static_cast<quint8>(packet[7]) + static_cast<quint8>(packet[8]) + static_cast<quint8>(packet[9]) + static_cast<quint8>(packet[10]) + static_cast<quint8>(packet[11]);
    qDebug() << "Calculated footer:" << footer << "(0x" << QString::number(footer, 16).toUpper() << ")";
    // 打印数据包前12字节内容
    QString packetHex;
    for (int i = 0; i < 12; ++i) {
        packetHex += QString("%1").arg(static_cast<quint8>(packet[i]), 2, 16, QChar('0')).toUpper();
        if (i < 11) packetHex += " ";
    }
    qDebug() << "Packet[0-11]:" << packetHex;
    qDebug() << "Footer (hex):" << QString("0x%1").arg(footer, 8, 16, QChar('0')).toUpper();
    packet[12] = footer & 0xFF;
    packet[13] = (footer >> 8) & 0xFF;
    packet[14] = (footer >> 16) & 0xFF;
    packet[15] = (footer >> 24) & 0xFF;
    
    return packet;
}

/**
 * @brief 计算校验和
 * @param data 数据字节
 * @return 校验和
 */
quint32 Serial::calculateChecksum(const QByteArray &data)
{
    quint32 checksum = 0;
    for (int i = 0; i < data.size(); ++i) {
        checksum += static_cast<quint8>(data[i]);
    }
    return checksum;
}

void Serial::updateControlPacketDisplay()
{
    QByteArray packet = buildControlPacket();
    
    // 在发送区显示生成的数据包
    QString hexString;
    for (int i = 0; i < packet.size(); ++i) {
        hexString += QString("%1 ").arg((unsigned char)packet[i], 2, 16, QChar('0')).toUpper();
    }
    ui->sendTextEdit->setPlainText(hexString.trimmed());
    
    // 确保十六进制发送模式开启
    ui->hexSendCheckBox->setChecked(true);
    
    // 如果处于即时发送模式，立即发送数据包
    if (m_instantSendMode && m_serialPort && m_serialPort->isOpen()) {
        qint64 bytesWritten = m_serialPort->write(packet);
        if (bytesWritten != -1) {
            m_bytesSent += bytesWritten;
            updateStatusInfo();
            
            // 在接收区显示发送的数据（如果启用了日志模式）
            if (m_logModeEnabled) {
                QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
                QString logEntry = QString("[%1] 发送: %2\n").arg(timestamp, hexString);
                ui->receiveTextEdit->insertPlainText(logEntry);
                ui->receiveTextEdit->moveCursor(QTextCursor::End);
            }
        }
    }
}

/**
 * @brief 设置氦气流量计和电磁阀状态
 */
void Serial::setHeliumControl(bool flowMeterEnabled, bool valveEnabled, int outputValue)
{
    ui->heliumFlowMeterCheckBox->setChecked(flowMeterEnabled);
    ui->heliumValveCheckBox->setChecked(valveEnabled);
    ui->heliumValueSpinBox->setValue(outputValue);
    updateControlPacketDisplay();
}

/**
 * @brief 设置氩气流量计和电磁阀状态
 */
void Serial::setArgonControl(bool flowMeterEnabled, bool valveEnabled, int outputValue)
{
    ui->argonFlowMeterCheckBox->setChecked(flowMeterEnabled);
    ui->argonValveCheckBox->setChecked(valveEnabled);
    ui->argonValueSpinBox->setValue(outputValue);
    updateControlPacketDisplay();
}

/**
 * @brief 设置等离子电源和调压器继电器状态
 */
void Serial::setPlasmaControl(bool plasmaEnabled, bool voltageEnabled,bool if_ctl)
{
    ui->plasmaRelayCheckBox->setChecked(plasmaEnabled);
    ui->voltageRelayCheckBox->setChecked(voltageEnabled);
    if(if_ctl)
    {
        ui->argonValueSpinBox->setValue(0);
        ui->heliumValueSpinBox->setValue(0);
    }
    
    // updateControlPacketDisplay();
}

void Serial::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);
    SetupDarkTitleBar();
}

void Serial::SetupDarkTitleBar()
{
#ifdef Q_OS_WIN
    if (QOperatingSystemVersion::current() >= QOperatingSystemVersion::Windows10) {
        HWND hwnd = reinterpret_cast<HWND>(winId());
        if (hwnd) {
            BOOL value = TRUE;
            DwmSetWindowAttribute(hwnd, 19, &value, sizeof(value));
        }
    }
#endif
}

/**
 * @brief 解析接收到的数据包
 * @param data 接收到的数据
 */
void Serial::parseReceivedPacket(const QByteArray &data)
{
    // 查找数据包头 0xFEFF (小端序: FF FE)
    int headerIndex = -1;
    for (int i = 0; i <= data.size()-16; ++i) {
        if (static_cast<quint8>(data[i]) == 0xFF && static_cast<quint8>(data[i + 1]) == 0xFE) {
            headerIndex = i;
            break;
        }
    }
    
    if (headerIndex == -1) {
        // 没有找到包头，保留末尾16个字节（可能包含不完整的包头），清除前面的数据
        if (m_receiveBuffer.size() > 16) {
            m_receiveBuffer = m_receiveBuffer.right(16);
        }
        return;
    }
    
    // 检查是否有完整的18字节数据包
    if (headerIndex + 18 > data.size()) {
        // 数据包不完整，等待更多数据
        // 移除包头之前的无效数据
        if (headerIndex > 0) {
            m_receiveBuffer.remove(0, headerIndex);
        }
        return;
    }
    
    // 提取完整的18字节数据包
    QByteArray packet = data.mid(headerIndex, 18);
    
    // 验证校验和
    if (verifyPacketChecksum(packet)) {
        // 解析数据包内容
        quint8 emerStop = static_cast<quint8>(packet[2]);
        quint8 volRelay = static_cast<quint8>(packet[3]);
        quint16 volOutValue = static_cast<quint8>(packet[4]) | (static_cast<quint8>(packet[5]) << 8);
        
        quint8 heFLOWRelay = static_cast<quint8>(packet[6]);
        quint8 hePress = static_cast<quint8>(packet[7]);
        quint16 heOutValue = static_cast<quint8>(packet[8]) | (static_cast<quint8>(packet[9]) << 8);
        
        quint8 arFLOWRelay = static_cast<quint8>(packet[10]);
        quint8 arPress = static_cast<quint8>(packet[11]);
        quint16 arOutValue = static_cast<quint8>(packet[12]) | (static_cast<quint8>(packet[13]) << 8);
        
        // 更新压力值
        if (m_heliumPressure != hePress || m_argonPressure != arPress) {
            m_heliumPressure = hePress;
            m_argonPressure = arPress;
            emit pressureValuesUpdated(m_heliumPressure, m_argonPressure);
        }
        
        // 发射设备状态更新信号
        bool emerStopActive = (emerStop == 0xF8);
        bool plasmaRelayActive = (volRelay & 0xF0) >> 4;  // 等离子电源（高4位）
        bool volRelayActive = (volRelay & 0x0F);          // 调压器（低4位）
        bool heFLOWRelayActive = (heFLOWRelay & 0xF0)>>4;  // 氦气流量计
        bool heValveActive = (heFLOWRelay & 0x0F);      // 氦气电磁阀
        bool arFLOWRelayActive = (arFLOWRelay & 0xF0)>>4;  // 氩气流量计
        bool arValveActive = (arFLOWRelay & 0x0F);      // 氩气电磁阀
        
        // 更新UI控件状态
        updateDeviceStatus(emerStopActive, plasmaRelayActive, volRelayActive, heFLOWRelayActive, heValveActive, arFLOWRelayActive, arValveActive);
        
        // 发射输出值更新信号
        emit outputValuesUpdated(volOutValue, heOutValue, arOutValue);
        
        // 输出解析结果到调试信息
        qDebug() << "数据包解析成功:";
        qDebug() << "  紧急停止:" << (emerStop == 0xF8 ? "是" : "否");
        qDebug() << "  等离子电源:" << ((volRelay & 0x10) ? "开" : "关");
        qDebug() << "  调压器:" << ((volRelay & 0x01) ? "开" : "关");
        qDebug() << "  等离子输出值:" << volOutValue;
        qDebug() << "  氦气流量计:" << ((heFLOWRelay & 0x10) ? "开" : "关");
        qDebug() << "  氦气电磁阀:" << ((heFLOWRelay & 0x01) ? "开" : "关");
        qDebug() << "  氦气压力:" << hePress;
        qDebug() << "  氦气输出值:" << heOutValue;
        qDebug() << "  氩气流量计:" << ((arFLOWRelay & 0x10) ? "开" : "关");
        qDebug() << "  氩气电磁阀:" << ((arFLOWRelay & 0x01) ? "开" : "关");
        qDebug() << "  氩气压力:" << arPress;
        qDebug() << "  氩气输出值:" << arOutValue;
    } else {
        qDebug() << "数据包校验和验证失败";
    }
    
    // 移除已处理的数据包
    m_receiveBuffer.remove(0, headerIndex + 18);
    
    // 如果缓冲区中还有数据，递归处理
    if (m_receiveBuffer.size() >= 18) {
        parseReceivedPacket(m_receiveBuffer);
    }
}

/**
 * @brief 验证数据包校验和
 * @param packet 完整的16字节数据包
 * @return true-校验通过，false-校验失败
 */
bool Serial::verifyPacketChecksum(const QByteArray &packet)
{
    if (packet.size() != 18) {
        return false;
    }
    
    // 按照下位机的方式计算校验和：各字段值相加
    //FF FE 00 10 00 08 00 00 00 00 00 00 15 02 00 00
    quint16 header = (static_cast<quint8>(packet[1]) << 8) | static_cast<quint8>(packet[0]);
    quint8 emerStop = static_cast<quint8>(packet[2]);
    quint8 volRelay = static_cast<quint8>(packet[3]);
    quint16 volOutValue = (static_cast<quint8>(packet[5]) << 8) | static_cast<quint8>(packet[4]);
    quint8 heFLOWRelay = static_cast<quint8>(packet[6]);
    quint8 hePress = static_cast<quint8>(packet[7]);
    quint16 heOutValue = (static_cast<quint8>(packet[9]) << 8) | static_cast<quint8>(packet[8]);
    quint8 arFLOWRelay = static_cast<quint8>(packet[10]);
    quint8 arPress = static_cast<quint8>(packet[11]);
    quint16 arOutValue = (static_cast<quint8>(packet[13]) << 8) | static_cast<quint8>(packet[12]);
    
    quint32 calculatedChecksum = header + emerStop + volRelay + volOutValue + 
                                heFLOWRelay + hePress + heOutValue + 
                                arFLOWRelay + arPress + arOutValue;
    
    // 提取数据包中的footer校验和 (32位，小端序，位于字节14-17)
    quint32 packetFooter = static_cast<quint8>(packet[14]) | 
                          (static_cast<quint8>(packet[15]) << 8) |
                          (static_cast<quint8>(packet[16]) << 16) |
                          (static_cast<quint8>(packet[17]) << 24);
    // 打印整个数据包为十六进制格式
    QString hexString;
    for (int i = 0; i < packet.size(); ++i) {
        hexString += QString("%1 ").arg(static_cast<quint8>(packet[i]), 2, 16, QChar('0')).toUpper();
    }
    qDebug() << "Received packet:" << hexString.trimmed();

    qDebug("calculatedChecksum: %2X , packetFooter: %2X",calculatedChecksum,packetFooter);

    return calculatedChecksum == packetFooter;
}

/**
 * @brief 更新UI控件状态
 * @param emerStop 紧急停止状态
 * @param volRelay 调压器继电器状态
 * @param heFLOWRelay 氦气流量计状态
 * @param heValve 氦气电磁阀状态
 * @param arFLOWRelay 氩气流量计状态
 * @param arValve 氩气电磁阀状态
 */
void Serial::updateDeviceStatus(bool emerStop, bool plasmaRelay, bool volRelay, bool heFLOWRelay, bool heValve, bool arFLOWRelay, bool arValve)
{
    // 定义checkbox背景颜色样式
    QString greenStyle = "QCheckBox { background-color: #4CAF50; border-radius: 4px; padding: 2px; }";
    QString redStyle = "QCheckBox { background-color: #F44336; border-radius: 4px; padding: 2px; }";
    
    // 更新等离子电源继电器checkbox
    ui->plasmaRelayCheckBox->setStyleSheet(plasmaRelay ? greenStyle : redStyle);
    
    // 更新调压器继电器checkbox
    ui->voltageRelayCheckBox->setStyleSheet(volRelay ? greenStyle : redStyle);
    
    // 更新氦气流量计checkbox
    ui->heliumFlowMeterCheckBox->setStyleSheet(heFLOWRelay ? greenStyle : redStyle);
    
    // 更新氦气电磁阀checkbox
    ui->heliumValveCheckBox->setStyleSheet(heValve ? greenStyle : redStyle);
    
    // 更新氩气流量计checkbox
    ui->argonFlowMeterCheckBox->setStyleSheet(arFLOWRelay ? greenStyle : redStyle);
    
    // 更新氩气电磁阀checkbox
    ui->argonValveCheckBox->setStyleSheet(arValve ? greenStyle : redStyle);
}

/**
 * @brief 更新输出值显示
 * @param volOutValue 调压器输出值
 * @param heOutValue 氦气输出值
 * @param arOutValue 氩气输出值
 */
void Serial::updateOutputValues(quint16 volOutValue, quint16 heOutValue, quint16 arOutValue)
{
    // 更新等离子电源当前值显示
    ui->plasmaCurrentValueDisplay->setText(QString::number(volOutValue));
    
    // 更新氦气当前值显示
    ui->heliumCurrentValueDisplay->setText(QString::number(heOutValue));
    // 发出氦气当前值变化信号
    emit heliumCurrentValueChanged(heOutValue);
    
    // 更新氩气当前值显示
    ui->argonCurrentValueDisplay->setText(QString::number(arOutValue));
    // 发出氩气当前值变化信号
    emit argonCurrentValueChanged(arOutValue);
}

void Serial::toggleInstantSendMode()
{
    m_instantSendMode = !m_instantSendMode;
    
    if (m_instantSendMode) {
        // 切换到即时发送模式
        ui->instantSendModeButton->setText("即时发送模式");
        ui->sendButton->setEnabled(false);  // 禁用发送按钮
        ui->sendButton->setStyleSheet("QPushButton { background-color: #666666; color: #999999; }");  // 设置为灰色
        
        // 启动定时器
        m_instantSendTimer->start();
    } else {
        // 切换到调试模式
        ui->instantSendModeButton->setText("调试模式");
        ui->sendButton->setEnabled(true);   // 启用发送按钮
        ui->sendButton->setStyleSheet("");  // 恢复默认样式
        
        // 停止定时器
        m_instantSendTimer->stop();
    }
    
    // 更新按钮选中状态
    ui->instantSendModeButton->setChecked(m_instantSendMode);
}

qint16 Serial::getHeliumCurrentValue() const
{
    return ui->heliumCurrentValueDisplay->text().toInt();
}

qint16 Serial::getArgonCurrentValue() const
{
    return ui->argonCurrentValueDisplay->text().toInt();
}
