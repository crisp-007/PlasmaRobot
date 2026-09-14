#include "serial.h"
#include "plasma_controller_protocol.h"
#include <QMessageBox>
#include <QDebug>
#include <QDateTime>
#include <QComboBox>
#include <QInputDialog>
#include <QOperatingSystemVersion>
#include <QPushButton>
#include <QSignalBlocker>
#include <QShowEvent>
#include <QSpinBox>
#include <QTabWidget>
#include <QVBoxLayout>

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

QString formatControlFrames(const QByteArray &frames)
{
    QString text;
    for (int i = 0; i < frames.size(); ++i) {
        text += QString("%1").arg(static_cast<quint8>(frames.at(i)), 2, 16, QChar('0')).toUpper();
        if (i + 1 < frames.size()) {
            text += ((i + 1) % PlasmaControllerProtocol::kCommandSize == 0) ? '\n' : ' ';
        }
    }
    return text;
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
    , m_handshakeTimer(nullptr)
    , m_baudRate(115200)
    , m_dataBits(QSerialPort::Data8)
    , m_parity(QSerialPort::NoParity)
    , m_stopBits(QSerialPort::OneStop)
    , m_flowControl(QSerialPort::NoFlowControl)
    , m_bytesReceived(0)
    , m_bytesSent(0)
    , m_isConnected(false)
    , m_handshakeComplete(false)
    , m_handshakeAttempts(0)
    , m_logModeEnabled(false)
    , m_heliumPressure(0)
    , m_argonPressure(0)
    , m_instantSendMode(false)
    , m_instantSendTimer(new QTimer(this))
{
    ui->setupUi(this);
    setupPageLayout();
    
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
    return m_isConnected && m_handshakeComplete;
}

/**
 * @brief 获取当前串口名称
 */
QString Serial::getCurrentPortName() const
{
    if (m_isConnected || !m_portName.isEmpty())
        return m_portName;
    return ui->portComboBox->currentData().toString();
}

void Serial::setupPageLayout()
{
    ui->mainLayout->removeItem(ui->leftLayout);
    ui->mainLayout->removeItem(ui->rightLayout);
    ui->leftLayout->removeWidget(ui->plasmaControlGroupBox);

    auto *tabWidget = new QTabWidget(this);
    tabWidget->setObjectName(QStringLiteral("serialTabWidget"));

    auto *serialPage = new QWidget(tabWidget);
    auto *serialPageLayout = new QVBoxLayout(serialPage);
    serialPageLayout->setContentsMargins(0, 0, 0, 0);
    serialPageLayout->addLayout(ui->rightLayout);
    tabWidget->addTab(serialPage, QStringLiteral("串口收发"));

    auto *controllerPage = new QWidget(tabWidget);
    auto *controllerPageLayout = new QVBoxLayout(controllerPage);
    controllerPageLayout->setContentsMargins(8, 8, 8, 8);
    ui->plasmaControlGroupBox->setMinimumWidth(0);
    ui->plasmaControlGroupBox->setMaximumWidth(QWIDGETSIZE_MAX);
    controllerPageLayout->addWidget(ui->plasmaControlGroupBox);
    tabWidget->addTab(controllerPage, QStringLiteral("控制板"));

    ui->leftLayout->addWidget(tabWidget, 1);
    ui->mainLayout->addLayout(ui->leftLayout, 1);
    ui->mainLayout->setStretch(0, 1);
}

qint32 Serial::getCurrentBaudRate() const
{
    return m_isConnected ? m_baudRate : ui->baudRateComboBox->currentText().toInt();
}

QString Serial::getCurrentFrameFormat() const
{
    return QStringLiteral("%1 / %2 / %3 / %4")
        .arg(ui->dataBitsComboBox->currentText(),
             ui->parityComboBox->currentText(),
             ui->stopBitsComboBox->currentText(),
             ui->flowControlComboBox->currentText());
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
    ui->controlAuxFanCheckBox->setStyleSheet(redStyle);
    ui->deviceFanCheckBox->setStyleSheet(redStyle);
    ui->controlMainFanCheckBox->setStyleSheet(redStyle);
    
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
    
    // 即时控制命令只会在控制板握手完成后发送。
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

    m_handshakeTimer = new QTimer(this);
    m_handshakeTimer->setSingleShot(true);
    m_handshakeTimer->setInterval(1000);
    connect(m_handshakeTimer, &QTimer::timeout, this, &Serial::onHandshakeTimeout);
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

    const QList<QComboBox *> settingBoxes = {
        ui->portComboBox,
        ui->baudRateComboBox,
        ui->dataBitsComboBox,
        ui->parityComboBox,
        ui->stopBitsComboBox,
        ui->flowControlComboBox
    };
    for (QComboBox *box : settingBoxes) {
        connect(box, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, [this]() { emit settingsChanged(); });
    }
    
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
    connect(ui->controlAuxFanCheckBox, &QCheckBox::toggled, this, &Serial::updateControlPacketDisplay);
    connect(ui->deviceFanCheckBox, &QCheckBox::toggled, this, &Serial::updateControlPacketDisplay);
    connect(ui->controlMainFanCheckBox, &QCheckBox::toggled, this, &Serial::updateControlPacketDisplay);
    connect(ui->controlAuxFanSpeedSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &Serial::updateControlPacketDisplay);
    connect(ui->deviceFanSpeedSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &Serial::updateControlPacketDisplay);
    connect(ui->controlMainFanSpeedSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &Serial::updateControlPacketDisplay);
    
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
    emit settingsChanged();
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
            m_handshakeComplete = false;
            m_handshakeAttempts = 0;
            m_receiveBuffer.clear();
            ui->connectButton->setText("断开");
            ui->connectButton->setStyleSheet("QPushButton { background-color: #ff6b6b; }");
            ui->statusLabel->setText("状态: 串口已打开，正在握手");
            ui->statusLabel->setStyleSheet("color: #FFC107;");
            
            // 禁用参数设置控件
            ui->portComboBox->setEnabled(false);
            ui->baudRateComboBox->setEnabled(false);
            ui->dataBitsComboBox->setEnabled(false);
            ui->parityComboBox->setEnabled(false);
            ui->stopBitsComboBox->setEnabled(false);
            ui->flowControlComboBox->setEnabled(false);
            
            m_statusTimer->start();
            emit connectionStatusChanged(false);
            sendHandshake();
        } else {
            const QString errorMessage = buildOpenPortErrorMessage(m_portName, m_serialPort);
            emit serialErrorOccurred(errorMessage);
            QMessageBox::critical(this, "错误", errorMessage);
        }
    } else {
        // 断开串口
        m_serialPort->close();
        m_isConnected = false;
        m_handshakeComplete = false;
        m_handshakeAttempts = 0;
        m_handshakeTimer->stop();
        m_receiveBuffer.clear();
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
    if (!m_handshakeComplete) {
        QMessageBox::warning(this, "警告", "控制板握手尚未完成，暂不发送控制命令。");
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
        emit dataSent(data.left(bytesWritten));
        emit statisticsUpdated(m_bytesReceived, m_bytesSent);
        if (ui->autoClearSendCheckBox->isChecked()) {
            ui->sendTextEdit->clear();
        }
    } else {
        emit serialErrorOccurred(QStringLiteral("数据发送失败"));
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
    emit statisticsUpdated(m_bytesReceived, m_bytesSent);
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
    emit statisticsUpdated(m_bytesReceived, m_bytesSent);
}

/**
 * @brief 串口错误处理
 */
void Serial::onSerialError(QSerialPort::SerialPortError error)
{
    if (error != QSerialPort::NoError) {
        QString errorString = m_serialPort->errorString();
        emit serialErrorOccurred(errorString);
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

void Serial::sendHandshake()
{
    if (!m_isConnected || !m_serialPort->isOpen() || m_handshakeComplete) {
        return;
    }

    const QByteArray request = PlasmaControllerProtocol::handshakeRequest();
    const qint64 written = m_serialPort->write(request);
    if (written != request.size()) {
        const QString message = QStringLiteral("控制板握手发送失败：%1")
                                    .arg(m_serialPort->errorString());
        emit serialErrorOccurred(message);
        ui->statusLabel->setText(QStringLiteral("状态: 握手发送失败"));
        ui->statusLabel->setStyleSheet(QStringLiteral("color: #f44336;"));
        return;
    }

    ++m_handshakeAttempts;
    m_bytesSent += written;
    emit dataSent(request);
    emit statisticsUpdated(m_bytesReceived, m_bytesSent);
    m_handshakeTimer->start();
}

void Serial::onHandshakeTimeout()
{
    constexpr int kMaximumAttempts = 3;
    if (!m_isConnected || m_handshakeComplete) {
        return;
    }
    if (m_handshakeAttempts < kMaximumAttempts) {
        sendHandshake();
        return;
    }

    const QString message = QStringLiteral("控制板握手超时，已尝试 %1 次")
                                .arg(m_handshakeAttempts);
    ui->statusLabel->setText(QStringLiteral("状态: 握手失败"));
    ui->statusLabel->setStyleSheet(QStringLiteral("color: #f44336;"));
    emit serialErrorOccurred(message);
    emit connectionStatusChanged(false);
}

void Serial::handleHandshakeAck()
{
    if (m_handshakeComplete) {
        return;
    }
    m_handshakeComplete = true;
    m_handshakeTimer->stop();
    ui->statusLabel->setText(QStringLiteral("状态: 控制板已连接"));
    ui->statusLabel->setStyleSheet(QStringLiteral("color: #4CAF50;"));
    emit connectionStatusChanged(true);
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
    const QByteArray packets = buildControlPacketBundle();
    ui->sendTextEdit->setPlainText(formatControlFrames(packets));
    
    // 设置为十六进制发送模式
    ui->hexSendCheckBox->setChecked(true);
    
    if (!m_isConnected || !m_handshakeComplete) {
        QMessageBox::warning(this, "警告", "控制板尚未连接或握手未完成，数据包仅生成未发送。");
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
    ui->controlAuxFanCheckBox->setChecked(false);
    ui->deviceFanCheckBox->setChecked(false);
    ui->controlMainFanCheckBox->setChecked(false);
    
    ui->plasmaValueSpinBox->setValue(0);
    ui->heliumValueSpinBox->setValue(0);
    ui->argonValueSpinBox->setValue(0);
    ui->controlAuxFanSpeedSpinBox->setValue(50);
    ui->deviceFanSpeedSpinBox->setValue(50);
    ui->controlMainFanSpeedSpinBox->setValue(50);
    
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
    if (ui->controlAuxFanCheckBox->isChecked()) {
        volRelay |= 0x20;
    }
    if (ui->deviceFanCheckBox->isChecked()) {
        volRelay |= 0x40;
    }
    if (ui->controlMainFanCheckBox->isChecked()) {
        volRelay |= 0x80;
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
    
    quint32 footer = static_cast<quint8>(packet[0]) + static_cast<quint8>(packet[1]) + static_cast<quint8>(packet[2]) + static_cast<quint8>(packet[3]) + static_cast<quint8>(packet[4]) + static_cast<quint8>(packet[5]) + static_cast<quint8>(packet[6]) + static_cast<quint8>(packet[7]) + static_cast<quint8>(packet[8]) + static_cast<quint8>(packet[9]) + static_cast<quint8>(packet[10]) + static_cast<quint8>(packet[11]);
    packet[12] = footer & 0xFF;
    packet[13] = (footer >> 8) & 0xFF;
    packet[14] = (footer >> 16) & 0xFF;
    packet[15] = (footer >> 24) & 0xFF;
    
    return packet;
}

QByteArray Serial::buildFanControlPacket() const
{
    return PlasmaControllerProtocol::buildFanDutyCommand(
        ui->controlAuxFanCheckBox->isChecked(),
        static_cast<quint16>(ui->controlAuxFanSpeedSpinBox->value() * 10),
        ui->deviceFanCheckBox->isChecked(),
        static_cast<quint16>(ui->deviceFanSpeedSpinBox->value() * 10),
        ui->controlMainFanCheckBox->isChecked(),
        static_cast<quint16>(ui->controlMainFanSpeedSpinBox->value() * 10));
}

QByteArray Serial::buildControlPacketBundle()
{
    QByteArray packets = buildControlPacket();
    packets.append(buildFanControlPacket());
    return packets;
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
    const QByteArray packets = buildControlPacketBundle();
    const QString hexString = formatControlFrames(packets);
    ui->sendTextEdit->setPlainText(hexString);
    
    // 确保十六进制发送模式开启
    ui->hexSendCheckBox->setChecked(true);
    
    // 如果处于即时发送模式，立即发送数据包
    if (m_instantSendMode && m_handshakeComplete
        && m_serialPort && m_serialPort->isOpen()) {
        qint64 bytesWritten = m_serialPort->write(packets);
        if (bytesWritten != -1) {
            m_bytesSent += bytesWritten;
            emit dataSent(packets.left(bytesWritten));
            emit statisticsUpdated(m_bytesReceived, m_bytesSent);
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
    const QSignalBlocker plasmaBlocker(ui->plasmaRelayCheckBox);
    const QSignalBlocker voltageBlocker(ui->voltageRelayCheckBox);
    const QSignalBlocker argonValueBlocker(ui->argonValueSpinBox);
    const QSignalBlocker heliumValueBlocker(ui->heliumValueSpinBox);

    ui->plasmaRelayCheckBox->setChecked(plasmaEnabled);
    ui->voltageRelayCheckBox->setChecked(voltageEnabled);
    if(if_ctl)
    {
        ui->argonValueSpinBox->setValue(0);
        ui->heliumValueSpinBox->setValue(0);
    }

    // Send one coherent frame. Emitting each checkbox signal separately can
    // briefly energize only one of the two power relays during a transition.
    updateControlPacketDisplay();
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
    Q_UNUSED(data);
    const QByteArray header = QByteArray::fromHex("FFFE");

    while (true) {
        const int headerIndex = m_receiveBuffer.indexOf(header);
        if (headerIndex < 0) {
            // A trailing 0xFF may be the first byte of a split header.
            const bool keepTrailingHeaderByte = !m_receiveBuffer.isEmpty()
                && static_cast<quint8>(m_receiveBuffer.back()) == 0xFF;
            m_receiveBuffer = keepTrailingHeaderByte
                ? QByteArray(1, static_cast<char>(0xFF))
                : QByteArray();
            return;
        }

        if (headerIndex > 0) {
            m_receiveBuffer.remove(0, headerIndex);
        }
        if (m_receiveBuffer.size() < PlasmaControllerProtocol::kCommandSize) {
            return;
        }

        const QByteArray baseFrame =
            m_receiveBuffer.left(PlasmaControllerProtocol::kCommandSize);
        if (PlasmaControllerProtocol::isHandshakeAck(baseFrame)) {
            m_receiveBuffer.remove(0, PlasmaControllerProtocol::kCommandSize);
            handleHandshakeAck();
            continue;
        }

        if (m_receiveBuffer.size() < PlasmaControllerProtocol::kStateSize) {
            return;
        }

        const QByteArray frame =
            m_receiveBuffer.left(PlasmaControllerProtocol::kStateSize);
        PlasmaControllerProtocol::State state;
        QString error;
        if (!PlasmaControllerProtocol::decodeState(frame, &state, &error)) {
            qWarning() << "控制板状态包解析失败:" << error;
            m_receiveBuffer.remove(0, 1);
            continue;
        }

        const bool emergencyStopActive = state.emergencyStop != 0U;
        const bool plasmaRelayActive = (state.voltageRelay & 0x10U) != 0U;
        const bool voltageRelayActive = (state.voltageRelay & 0x01U) != 0U;
        const bool controlAuxFanActive = (state.voltageRelay & 0x20U) != 0U;
        const bool deviceFanActive = (state.voltageRelay & 0x40U) != 0U;
        const bool controlMainFanActive = (state.voltageRelay & 0x80U) != 0U;
        const bool heliumFlowRelayActive = (state.heliumRelay & 0x10U) != 0U;
        const bool heliumValveActive = (state.heliumRelay & 0x01U) != 0U;
        const bool argonFlowRelayActive = (state.argonRelay & 0x10U) != 0U;
        const bool argonValveActive = (state.argonRelay & 0x01U) != 0U;

        m_heliumPressure = state.heliumPressureMpa;
        m_argonPressure = state.argonPressureMpa;
        emit pressureValuesUpdated(m_heliumPressure, m_argonPressure);
        emit plasmaFeedbackUpdated(state.plasmaFeedbackVpp);
        emit deviceStatusUpdated(emergencyStopActive,
                                 plasmaRelayActive,
                                 voltageRelayActive,
                                 heliumFlowRelayActive,
                                 heliumValveActive,
                                 argonFlowRelayActive,
                                 argonValveActive,
                                 controlAuxFanActive,
                                 deviceFanActive,
                                 controlMainFanActive);
        emit outputValuesUpdated(state.voltageOutput,
                                 state.heliumOutput,
                                 state.argonOutput);

        m_receiveBuffer.remove(0, PlasmaControllerProtocol::kStateSize);
    }
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
void Serial::updateDeviceStatus(bool emerStop, bool plasmaRelay, bool volRelay,
                                bool heFLOWRelay, bool heValve,
                                bool arFLOWRelay, bool arValve,
                                bool controlAuxFan, bool deviceFan, bool controlMainFan)
{
    // 定义checkbox背景颜色样式
    QString greenStyle = "QCheckBox { background-color: #4CAF50; border-radius: 4px; padding: 2px; }";
    QString redStyle = "QCheckBox { background-color: #F44336; border-radius: 4px; padding: 2px; }";
    
    // 更新等离子电源继电器checkbox
    ui->plasmaRelayCheckBox->setStyleSheet(plasmaRelay ? greenStyle : redStyle);
    
    // 更新调压器继电器checkbox
    ui->voltageRelayCheckBox->setStyleSheet(volRelay ? greenStyle : redStyle);

    ui->controlAuxFanCheckBox->setStyleSheet(controlAuxFan ? greenStyle : redStyle);
    ui->deviceFanCheckBox->setStyleSheet(deviceFan ? greenStyle : redStyle);
    ui->controlMainFanCheckBox->setStyleSheet(controlMainFan ? greenStyle : redStyle);
    
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
    ui->heliumCurrentValueDisplay->setText(
        QStringLiteral("%1 L/min (%2)")
            .arg(heOutValue / 100.0, 0, 'f', 2)
            .arg(heOutValue));
    // 发出氦气当前值变化信号
    emit heliumCurrentValueChanged(heOutValue);
    
    // 更新氩气当前值显示
    ui->argonCurrentValueDisplay->setText(
        QStringLiteral("%1 L/min (%2)")
            .arg(arOutValue / 100.0, 0, 'f', 2)
            .arg(arOutValue));
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
