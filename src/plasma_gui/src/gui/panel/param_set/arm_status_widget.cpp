#include "arm_status_widget.h"

ArmStatusWidget::ArmStatusWidget(QWidget *parent)
    : QWidget(parent)
{
    setupUi();
}

// ========= 样式常量 =========
static const QString kDarkBg     = "#2b2b2b";
static const QString kCardBg     = "#333333";
static const QString kValueBg    = "#4a4a4a";
static const QString kBorder     = "#555555";
static const QString kAccent     = "#5F9EA0";
static const QString kGreenDot   = "#00ff88";
static const QString kRedDot     = "#ff4444";
static const QString kAmberDot   = "#ffc107";
static const QString kTitleColor = "#e0e0e0";
static const QString kValueColor = "#ffffff";
static const QString kLabelColor = "#c0c0c0";

static const QString kGroupBoxStyle = R"(
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
)";

static const QString kValueLabelStyle = R"(
    QLabel {
        background-color: #4a4a4a;
        border: 1px solid #666666;
        border-radius: 4px;
        padding: 4px 6px;
        color: #ffffff;
        font-family: 'Consolas', 'Monospace', monospace;
        font-size: 12px;
    }
)";

static const QString kFieldNameStyle = R"(
    QLabel {
        color: #c0c0c0;
        font-size: 12px;
        background: transparent;
    }
)";

static QString valueLabelStyleForColor(const QString &color)
{
    QString style = kValueLabelStyle;
    style.replace(QStringLiteral("color: #ffffff;"),
                  QStringLiteral("color: %1;").arg(color));
    return style;
}

// ========= 辅助：创建标签 =========
QLabel* ArmStatusWidget::createValueLabel()
{
    QLabel *lbl = new QLabel("--");
    lbl->setStyleSheet(kValueLabelStyle);
    lbl->setAlignment(Qt::AlignCenter);
    lbl->setMinimumHeight(24);
    return lbl;
}

// ========= 构建连接状态区 =========
QWidget* ArmStatusWidget::createConnectionSection()
{
    QWidget *w = new QWidget;
    QVBoxLayout *lay = new QVBoxLayout(w);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->setSpacing(6);

    // 机械臂型号
    m_armNameLabel = new QLabel("Realman Eco65-B");
    m_armNameLabel->setStyleSheet(QString(
        "QLabel { color: %1; font-size: 15px; font-weight: bold; background: transparent; }").arg(kAccent));
    m_armNameLabel->setAlignment(Qt::AlignCenter);
    lay->addWidget(m_armNameLabel);

    m_endpointLabel = new QLabel(QStringLiteral("界面配置: --"));
    m_endpointLabel->setStyleSheet(kFieldNameStyle);
    m_endpointLabel->setAlignment(Qt::AlignCenter);
    lay->addWidget(m_endpointLabel);

    // 连接状态行
    QHBoxLayout *connRow = new QHBoxLayout;
    connRow->setSpacing(6);

    QLabel *connTitle = new QLabel("连接状态");
    connTitle->setStyleSheet(kFieldNameStyle);
    connRow->addWidget(connTitle);

    // 状态指示灯
    m_connDot = new QLabel;
    m_connDot->setFixedSize(12, 12);
    m_connDot->setStyleSheet(QString(
        "background-color: %1; border-radius: 6px; border: 1px solid #333;").arg(kRedDot));
    connRow->addWidget(m_connDot);

    // 状态文字
    m_connStatusLabel = new QLabel("未连接");
    m_connStatusLabel->setStyleSheet(QString(
        "QLabel { color: %1; font-size: 12px; font-weight: bold; background: transparent; }").arg(kRedDot));
    connRow->addWidget(m_connStatusLabel);
    connRow->addStretch();

    lay->addLayout(connRow);

    QGridLayout *dataGrid = new QGridLayout;
    dataGrid->setContentsMargins(0, 0, 0, 0);
    dataGrid->setHorizontalSpacing(8);
    dataGrid->setVerticalSpacing(4);

    QLabel *sourceTitle = new QLabel(QStringLiteral("关节数据"));
    sourceTitle->setStyleSheet(kFieldNameStyle);
    QLabel *sourceValue = createValueLabel();
    sourceValue->setText(QStringLiteral("/joint_states"));

    QLabel *updateTitle = new QLabel(QStringLiteral("最后更新"));
    updateTitle->setStyleSheet(kFieldNameStyle);
    m_lastUpdateLabel = createValueLabel();

    dataGrid->addWidget(sourceTitle, 0, 0);
    dataGrid->addWidget(sourceValue, 0, 1);
    dataGrid->addWidget(updateTitle, 0, 2);
    dataGrid->addWidget(m_lastUpdateLabel, 0, 3);
    dataGrid->setColumnStretch(1, 1);
    dataGrid->setColumnStretch(3, 1);
    lay->addLayout(dataGrid);
    return w;
}

// ========= 构建关节角度区 =========
QWidget* ArmStatusWidget::createJointSection()
{
    QWidget *w = new QWidget;
    QVBoxLayout *lay = new QVBoxLayout(w);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->setSpacing(4);

    QGridLayout *grid = new QGridLayout;
    grid->setSpacing(4);
    grid->setContentsMargins(0, 0, 0, 0);

    QLabel *joints[6] = {};
    QLabel *values[6] = {};

    m_j1Value = createValueLabel(); joints[0] = new QLabel("J1"); values[0] = m_j1Value;
    m_j2Value = createValueLabel(); joints[1] = new QLabel("J2"); values[1] = m_j2Value;
    m_j3Value = createValueLabel(); joints[2] = new QLabel("J3"); values[2] = m_j3Value;
    m_j4Value = createValueLabel(); joints[3] = new QLabel("J4"); values[3] = m_j4Value;
    m_j5Value = createValueLabel(); joints[4] = new QLabel("J5"); values[4] = m_j5Value;
    m_j6Value = createValueLabel(); joints[5] = new QLabel("J6"); values[5] = m_j6Value;

    for (int i = 0; i < 6; ++i) {
        int row = i / 3;
        int col = (i % 3) * 2;

        joints[i]->setStyleSheet(kFieldNameStyle);
        joints[i]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        joints[i]->setFixedWidth(24);

        grid->addWidget(joints[i],  row, col);
        grid->addWidget(values[i],  row, col + 1);
    }

    lay->addLayout(grid);
    return w;
}

// ========= 构建 TCP 位姿区 =========
QWidget* ArmStatusWidget::createTcpSection()
{
    QWidget *w = new QWidget;
    QVBoxLayout *lay = new QVBoxLayout(w);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->setSpacing(4);

    QGridLayout *grid = new QGridLayout;
    grid->setSpacing(4);
    grid->setContentsMargins(0, 0, 0, 0);

    struct Field { QLabel *&val; const char *name; };
    Field fields[] = {
        {m_txValue,  "X"},  {m_tyValue,  "Y"},  {m_tzValue,  "Z"},
        {m_trxValue, "Rx"}, {m_tryValue, "Ry"}, {m_trzValue, "Rz"}
    };

    for (int i = 0; i < 6; ++i) {
        int row = i / 3;
        int col = (i % 3) * 2;

        QLabel *name = new QLabel(fields[i].name);
        name->setStyleSheet(kFieldNameStyle);
        name->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        name->setFixedWidth(24);

        fields[i].val = createValueLabel();

        grid->addWidget(name,              row, col);
        grid->addWidget(fields[i].val,     row, col + 1);
    }

    lay->addLayout(grid);
    return w;
}

// ========= 构建自检状态区 =========
QWidget* ArmStatusWidget::createSelfCheckSection()
{
    QWidget *w = new QWidget;
    QVBoxLayout *lay = new QVBoxLayout(w);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->setSpacing(6);

    QHBoxLayout *row = new QHBoxLayout;
    row->setSpacing(8);

    QLabel *title = new QLabel("自检状态");
    title->setStyleSheet(kFieldNameStyle);
    row->addWidget(title);

    m_selfCheckValue = createValueLabel();
    m_selfCheckValue->setText(QStringLiteral("待接入"));
    m_selfCheckValue->setStyleSheet(valueLabelStyleForColor(QStringLiteral("#aaaaaa")));
    row->addWidget(m_selfCheckValue, 1);

    lay->addLayout(row);
    return w;
}

// ========= 组装整体界面 =========
void ArmStatusWidget::setupUi()
{
    setObjectName("armStatusWidget");

    auto *mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(6);

    // 1. 连接状态
    QGroupBox *connGroup = new QGroupBox("连接信息");
    connGroup->setStyleSheet(kGroupBoxStyle);
    QVBoxLayout *connLay = new QVBoxLayout(connGroup);
    connLay->setContentsMargins(8, 12, 8, 8);
    connLay->setSpacing(4);
    connLay->addWidget(createConnectionSection());
    mainLayout->addWidget(connGroup);

    // 2. 关节角度
    QGroupBox *jointGroup = new QGroupBox("关节角度 (°)");
    jointGroup->setStyleSheet(kGroupBoxStyle);
    QVBoxLayout *jointLay = new QVBoxLayout(jointGroup);
    jointLay->setContentsMargins(8, 12, 8, 8);
    jointLay->setSpacing(4);
    jointLay->addWidget(createJointSection());
    // 3. 末端位姿
    QGroupBox *tcpGroup = new QGroupBox("末端位姿 TCP (mm / °)");
    tcpGroup->setStyleSheet(kGroupBoxStyle);
    QVBoxLayout *tcpLay = new QVBoxLayout(tcpGroup);
    tcpLay->setContentsMargins(8, 12, 8, 8);
    tcpLay->setSpacing(4);
    tcpLay->addWidget(createTcpSection());
    QHBoxLayout *telemetryLayout = new QHBoxLayout;
    telemetryLayout->setContentsMargins(0, 0, 0, 0);
    telemetryLayout->setSpacing(6);
    telemetryLayout->addWidget(jointGroup, 1);
    telemetryLayout->addWidget(tcpGroup, 1);
    mainLayout->addLayout(telemetryLayout);

    // 4. 自检状态
    QGroupBox *selfCheckGroup = new QGroupBox("系统自检");
    selfCheckGroup->setStyleSheet(kGroupBoxStyle);
    QVBoxLayout *scLay = new QVBoxLayout(selfCheckGroup);
    scLay->setContentsMargins(8, 12, 8, 8);
    scLay->setSpacing(4);
    scLay->addWidget(createSelfCheckSection());
    mainLayout->addWidget(selfCheckGroup);
}

// ========= 外部更新接口 =========

void ArmStatusWidget::updateJointAngles(double j1, double j2, double j3,
                                        double j4, double j5, double j6)
{
    m_j1Value->setText(QString::number(j1, 'f', 2));
    m_j2Value->setText(QString::number(j2, 'f', 2));
    m_j3Value->setText(QString::number(j3, 'f', 2));
    m_j4Value->setText(QString::number(j4, 'f', 2));
    m_j5Value->setText(QString::number(j5, 'f', 2));
    m_j6Value->setText(QString::number(j6, 'f', 2));
}

void ArmStatusWidget::updateTcpPose(double x, double y, double z,
                                    double rx, double ry, double rz)
{
    m_txValue->setText(QString::number(x, 'f', 2));
    m_tyValue->setText(QString::number(y, 'f', 2));
    m_tzValue->setText(QString::number(z, 'f', 2));
    m_trxValue->setText(QString::number(rx, 'f', 2));
    m_tryValue->setText(QString::number(ry, 'f', 2));
    m_trzValue->setText(QString::number(rz, 'f', 2));
}

void ArmStatusWidget::setConnected(bool connected)
{
    setHealthState(connected ? DeviceHealthModel::State::Healthy
                             : DeviceHealthModel::State::Disconnected,
                   connected ? QStringLiteral("关节数据正常")
                             : QStringLiteral("尚未收到机械臂数据"));
}

void ArmStatusWidget::setHealthState(DeviceHealthModel::State state,
                                     const QString &message)
{
    QString color = kRedDot;
    QString text = QStringLiteral("未连接");

    switch (state) {
    case DeviceHealthModel::State::Starting:
        color = kAmberDot;
        text = QStringLiteral("启动中");
        break;
    case DeviceHealthModel::State::Healthy:
        color = kGreenDot;
        text = QStringLiteral("数据正常");
        break;
    case DeviceHealthModel::State::Warning:
        color = kAmberDot;
        text = QStringLiteral("状态警告");
        break;
    case DeviceHealthModel::State::Error:
        color = kRedDot;
        text = QStringLiteral("运行错误");
        break;
    case DeviceHealthModel::State::Stale:
        color = kAmberDot;
        text = QStringLiteral("数据中断");
        break;
    case DeviceHealthModel::State::Disconnected:
    default:
        break;
    }

    m_connected = state == DeviceHealthModel::State::Healthy;
    m_connDot->setStyleSheet(QString(
        "background-color: %1; border-radius: 6px; border: 1px solid #333;").arg(color));
    m_connStatusLabel->setText(text);
    m_connStatusLabel->setToolTip(message);
    m_connStatusLabel->setStyleSheet(QString(
        "QLabel { color: %1; font-size: 12px; font-weight: bold; background: transparent; }").arg(color));
}

void ArmStatusWidget::setLastUpdate(const QDateTime &time)
{
    if (m_lastUpdateLabel) {
        m_lastUpdateLabel->setText(time.isValid()
            ? time.toString(QStringLiteral("HH:mm:ss.zzz"))
            : QStringLiteral("--"));
    }
}

void ArmStatusWidget::setArmInfo(const QString &model,
                                 const QString &ip,
                                 const QString &port)
{
    if (m_armNameLabel)
        m_armNameLabel->setText(model.isEmpty() ? QStringLiteral("机械臂") : model);
    if (m_endpointLabel) {
        m_endpointLabel->setText(QStringLiteral("控制器: %1:%2")
            .arg(ip.isEmpty() ? QStringLiteral("--") : ip,
                 port.isEmpty() ? QStringLiteral("--") : port));
    }
}

void ArmStatusWidget::setSelfCheckStatus(const QString &status)
{
    m_selfCheckValue->setText(status);
    // 根据状态文字设置颜色
    if (status.contains("正常") || status.contains("OK")) {
        m_selfCheckValue->setStyleSheet(valueLabelStyleForColor(QStringLiteral("#00ff88")));
    } else if (status.contains("异常") || status.contains("失败")) {
        m_selfCheckValue->setStyleSheet(valueLabelStyleForColor(QStringLiteral("#ff4444")));
    } else {
        m_selfCheckValue->setStyleSheet(valueLabelStyleForColor(QStringLiteral("#aaaaaa")));
    }
}
