#ifndef SERIAL_H
#define SERIAL_H

#include <QWidget>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QStringList>
#include "ui_serial.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class Serial;
}
QT_END_NAMESPACE
/**
 * @brief 串口通信窗口类
 * 
 * 提供串口参数设置、连接管理、数据收发等功能
 */
class Serial : public QWidget
{
    Q_OBJECT

public:
    explicit Serial(QWidget *parent = nullptr);
    ~Serial();

    /**
     * @brief 获取当前串口连接状态
     * @return true-已连接，false-未连接
     */
    bool isConnected() const;

    /**
     * @brief 获取当前串口名称
     * @return 串口名称
     */
    QString getCurrentPortName() const;
    qint32 getCurrentBaudRate() const;
    QString getCurrentFrameFormat() const;
    qint64 getBytesReceived() const { return m_bytesReceived; }
    qint64 getBytesSent() const { return m_bytesSent; }

public slots:
    /**
     * @brief 刷新可用串口列表
     */
    void refreshPortList();

    /**
     * @brief 连接/断开串口
     */
    void toggleConnection();

    /**
     * @brief 发送数据
     */
    void sendData();

    /**
     * @brief 清空接收区
     */
    void clearReceiveArea();

    /**
     * @brief 清空发送区域
     */
    void clearSendArea();

    /**
     * @brief 发送等离子控制指令
     */
    void sendControlPacket();

    /**
     * @brief 加载测试数据包
     */
    void loadTestPacket();

    /**
     * @brief 紧急停止
     */
    void emergencyStop();

    /**
     * @brief 系统复位
     */
    void resetSystem();

    /**
     * @brief 更新控制数据包显示
     */
    void updateControlPacketDisplay();
    
    /**
     * @brief 切换即时发送模式
     */
    void toggleInstantSendMode();
    
    /**
     * @brief 设置氦气流量计和电磁阀状态
     * @param flowMeterEnabled 流量计状态
     * @param valveEnabled 电磁阀状态
     * @param outputValue 目标流量，单位 0.01 L/min
     */
    void setHeliumControl(bool flowMeterEnabled, bool valveEnabled, int outputValue);
    
    /**
     * @brief 设置氩气流量计和电磁阀状态
     * @param flowMeterEnabled 流量计状态
     * @param valveEnabled 电磁阀状态
     * @param outputValue 目标流量，单位 0.01 L/min
     */
    void setArgonControl(bool flowMeterEnabled, bool valveEnabled, int outputValue);
    
    /**
     * @brief 设置等离子电源和调压器继电器状态
     * @param plasmaEnabled 等离子电源继电器状态
     * @param voltageEnabled 调压器继电器状态
     * @param if_ctl 是否需要控制
     */
    void setPlasmaControl(bool plasmaEnabled, bool voltageEnabled,bool if_ctl=false);

protected:
    void showEvent(QShowEvent *event) override;

signals:
    /**
     * @brief 串口连接状态改变信号
     * @param connected 连接状态
     */
    void connectionStatusChanged(bool connected);

    /**
     * @brief 数据接收信号
     * @param data 接收到的数据
     */
    void dataReceived(const QByteArray &data);
    void dataSent(const QByteArray &data);
    void statisticsUpdated(qint64 receivedBytes, qint64 sentBytes);
    void serialErrorOccurred(const QString &message);
    void settingsChanged();
    void plasmaFeedbackUpdated(double feedbackVpp);

private slots:
    /**
     * @brief 处理数据接收处理
     */
    void onDataReceived();

    /**
     * @brief 串口错误处理
     * @param error 错误类型
     */
    void onSerialError(QSerialPort::SerialPortError error);

    /**
     * @brief 更新连接状态显示
     */
    void updateConnectionStatus();
    void onHandshakeTimeout();

private:
    /**
     * @brief 初始化UI界面
     */
    void setupUI();
    void setupPageLayout();

    /**
     * @brief 初始化串口参数
     */
    void initSerialPort();

    /**
     * @brief 连接信号槽
     */
    void connectSignals();

    /**
     * @brief 应用串口参数设置
     */
    void applySerialSettings();

    /**
     * @brief 更新状态栏信息
     */
    void updateStatusInfo();
    void sendHandshake();
    void handleHandshakeAck();

private:
    Ui::Serial *ui;                    ///< UI界面指针
    QSerialPort *m_serialPort;         ///< 串口对象
    QTimer *m_statusTimer;             ///< 状态更新定时器
    QTimer *m_handshakeTimer;          ///< 控制板握手重试定时器
    
    // 串口参数
    QString m_portName;                ///< 串口名称
    qint32 m_baudRate;                 ///< 波特率
    QSerialPort::DataBits m_dataBits;  ///< 数据位
    QSerialPort::Parity m_parity;      ///< 校验位
    QSerialPort::StopBits m_stopBits;  ///< 停止位
    QSerialPort::FlowControl m_flowControl; ///< 流控制
    
    // 统计信息
    qint64 m_bytesReceived;            ///< 接收字节数
    qint64 m_bytesSent;                ///< 发送字节数
    bool m_isConnected;                ///< 连接状态
    bool m_handshakeComplete;          ///< 控制板握手状态
    int m_handshakeAttempts;           ///< 当前握手发送次数
    
    // 日志模式
    bool m_logModeEnabled;             ///< 日志模式是否启用
    
    // 等离子控制系统相关函数
    /**
     * @brief 构建控制数据包
     * @return 16字节的控制数据包
     */
    QByteArray buildControlPacket();
    QByteArray buildFanControlPacket() const;
    QByteArray buildControlPacketBundle();
    
    /**
     * @brief 计算校验和
     * @param data 前12字节数据
     * @return 校验和
     */
    quint32 calculateChecksum(const QByteArray &data);
    
    /**
     * @brief 加载预定义数据包
     * @param packetIndex 数据包索引
     */
    void loadPredefinedPacket(int packetIndex);
    
    /**
     * @brief 设置暗色标题栏
     */
    void SetupDarkTitleBar();
    
    /**
     * @brief 解析接收到的数据包
     * @param data 接收到的数据
     */
    void parseReceivedPacket(const QByteArray &data);
    
    // 接收缓冲区和数据包解析
    QByteArray m_receiveBuffer;        ///< 接收数据缓冲区

    // 控制板状态包中的压力单位为 MPa
    double m_heliumPressure;
    double m_argonPressure;
    bool m_instantSendMode;            ///< 即时发送模式标志
    QTimer *m_instantSendTimer;        ///< 即时发送模式定时器
    
public:
    /**
     * @brief 获取氦气压力值
     * @return 氦气压力值(MPa)
     */
    double getHeliumPressure() const { return m_heliumPressure; }
    
    /**
     * @brief 获取氩气压力值
     * @return 氩气压力值(MPa)
     */
    double getArgonPressure() const { return m_argonPressure; }
    
    /**
     * @brief 获取氦气当前值控件的值
     * @return 氦气当前值
     */
    qint16 getHeliumCurrentValue() const;
    
    /**
     * @brief 获取氩气当前值控件的值
     * @return 氩气当前值
     */
    qint16 getArgonCurrentValue() const;
    
signals:
    /**
     * @brief 压力值更新信号
     * @param heliumPressure 氦气压力值
     * @param argonPressure 氩气压力值
     */
    void pressureValuesUpdated(double heliumPressureMpa, double argonPressureMpa);
    
    /**
     * @brief 设备状态更新信号
     * @param emerStop 紧急停止状态
     * @param plasmaRelay 等离子电源继电器状态
     * @param volRelay 调压器继电器状态
     * @param heFLOWRelay 氦气流量计状态
     * @param heValve 氦气电磁阀状态
     * @param arFLOWRelay 氩气流量计状态
     * @param arValve 氩气电磁阀状态
     */
    void deviceStatusUpdated(bool emerStop, bool plasmaRelay, bool volRelay,
                             bool heFLOWRelay, bool heValve,
                             bool arFLOWRelay, bool arValve,
                             bool controlAuxFan, bool deviceFan, bool controlMainFan);
    
    /**
     * @brief 输出值更新信号
     * @param volOutValue 调压器输出值
     * @param heOutValue 氦气输出值
     * @param arOutValue 氩气输出值
     */
    void outputValuesUpdated(quint16 volOutValue, quint16 heOutValue, quint16 arOutValue);
    
    /**
     * @brief 氦气当前值变化信号
     * @param heValue 氦气当前值
     */
    void heliumCurrentValueChanged(quint16 heValue);
    
    /**
     * @brief 氩气当前值变化信号
     * @param arValue 氩气当前值
     */
    void argonCurrentValueChanged(quint16 arValue);

public slots:
    /**
     * @brief 更新设备状态UI
     * @param emerStop 紧急停止状态
     * @param plasmaRelay 等离子电源继电器状态
     * @param volRelay 调压器继电器状态
     * @param heFLOWRelay 氦气流量计状态
     * @param heValve 氦气电磁阀状态
     * @param arFLOWRelay 氩气流量计状态
     * @param arValve 氩气电磁阀状态
     */
    void updateDeviceStatus(bool emerStop, bool plasmaRelay, bool volRelay,
                            bool heFLOWRelay, bool heValve,
                            bool arFLOWRelay, bool arValve,
                            bool controlAuxFan, bool deviceFan, bool controlMainFan);
    
    /**
     * @brief 更新输出值显示
     * @param volOutValue 调压器输出值
     * @param heOutValue 氦气输出值
     * @param arOutValue 氩气输出值
     */
    void updateOutputValues(quint16 volOutValue, quint16 heOutValue, quint16 arOutValue);
};

#endif // SERIAL_H
