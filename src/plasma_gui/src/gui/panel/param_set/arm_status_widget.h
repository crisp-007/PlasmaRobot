#ifndef ARM_STATUS_WIDGET_H
#define ARM_STATUS_WIDGET_H

#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QFont>
#include <QFrame>
#include <QHBoxLayout>
#include <QDateTime>

#include "device_health_model.h"

// =============================================
//  ArmStatusWidget
//  显示 Realman Eco65-B 机械臂状态：
//  - 连接状态（颜色指示）
//  - 6 个关节角度（J1-J6）
//  - 末端位姿 TCP（x, y, z, rx, ry, rz）
//  真实 TCP、诊断和控制接口在驱动协议确认后接入。
// =============================================
class ArmStatusWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ArmStatusWidget(QWidget *parent = nullptr);
    ~ArmStatusWidget() override = default;

    // 外部更新接口（由 MainWindow 调用，传递 ROS 数据）
    void updateJointAngles(double j1, double j2, double j3,
                           double j4, double j5, double j6);
    void updateTcpPose(double x, double y, double z,
                       double rx, double ry, double rz);
    void setConnected(bool connected);
    void setHealthState(DeviceHealthModel::State state, const QString &message);
    void setLastUpdate(const QDateTime &time);
    void setArmInfo(const QString &model, const QString &ip, const QString &port);
    void setSelfCheckStatus(const QString &status);

signals:
    void emergencyStopRequested();

private:
    void setupUi();
    QWidget* createConnectionSection();
    QWidget* createJointSection();
    QWidget* createTcpSection();
    QWidget* createSelfCheckSection();

    QLabel* createValueLabel();

    // 连接状态
    QLabel *m_armNameLabel    = nullptr;
    QLabel *m_endpointLabel   = nullptr;
    QLabel *m_connStatusLabel = nullptr;
    QLabel *m_connDot         = nullptr;
    QLabel *m_lastUpdateLabel = nullptr;

    // 关节角度标签
    QLabel *m_j1Value = nullptr;
    QLabel *m_j2Value = nullptr;
    QLabel *m_j3Value = nullptr;
    QLabel *m_j4Value = nullptr;
    QLabel *m_j5Value = nullptr;
    QLabel *m_j6Value = nullptr;

    // TCP 位姿标签
    QLabel *m_txValue = nullptr;
    QLabel *m_tyValue = nullptr;
    QLabel *m_tzValue = nullptr;
    QLabel *m_trxValue = nullptr;
    QLabel *m_tryValue = nullptr;
    QLabel *m_trzValue = nullptr;

    // 自检状态
    QLabel *m_selfCheckValue = nullptr;

    bool m_connected = false;
};

#endif // ARM_STATUS_WIDGET_H
