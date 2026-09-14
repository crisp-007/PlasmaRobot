#include "StepWizardView.h"
#include "StepWizardModel.h"
#include <QApplication>
#include <QIcon>
#include <QMovie>
#include <QPixmap>
#include <QStyle>
#include <algorithm>

namespace {
constexpr int kSystemCheckReadyIconSize = 40;
constexpr int kSystemCheckLoadingIconSize = 35;
constexpr int kSystemCheckSuccessIconSize = 40;
constexpr int kPathParamIconSize = 40;
const char *kSystemCheckStartIcon = ":/icons/ui_source/check/start-40.png";
const char *kSystemCheckLoadingIcon = ":/icons/ui_source/check/loading-50.gif";
const char *kSystemCheckSuccessIcon = ":/icons/ui_source/check/check-mark.gif";
const char *kPathParamIcon = ":/icons/ui_source/point/path-config-40.gif";
}

// =============================================
//  构造 / 析构
// =============================================
StepWizardView::StepWizardView(QObject *parent)
    : QObject(parent)
{
    m_rebuildProgressTimer = new QTimer(this);
    m_rebuildProgressTimer->setInterval(120);
    connect(m_rebuildProgressTimer, &QTimer::timeout, this, [this]() {
        m_rebuildProgressPulse = (m_rebuildProgressPulse + 1) % 12;
        applyRebuildProgressStyle();
    });

    m_robotExecutionHoldTimer = new QTimer(this);
    m_robotExecutionHoldTimer->setInterval(45);
    connect(m_robotExecutionHoldTimer, &QTimer::timeout, this, [this]() {
        if (!m_robotProgress)
            return;

        m_robotProgressPulse = (m_robotProgressPulse + 1) % 12;
        m_robotExecutionProgress = std::min(100, m_robotExecutionProgress + 2);
        m_robotProgress->setValue(m_robotExecutionProgress);
        applyRobotExecutionProgressStyle();

        if (m_robotExecutionProgress >= 100) {
            m_robotExecutionHoldTimer->stop();
            m_robotExecutionHoldCompleted = true;
            emit requestPathExecution();
        }
    });
}

StepWizardView::~StepWizardView() = default;

// =============================================
//  bindWidgets - 绑定显示控件（描述、图片、步骤文字、进度条）
// =============================================
void StepWizardView::bindWidgets(QLabel *step_dsc,
                                 QLabel *stepimg,
                                 QLabel *stepLabel,
                                 QProgressBar *step_progress)
{
    m_stepDsc      = step_dsc;
    m_stepImg      = stepimg;
    m_stepShow     = stepLabel;
    m_stepProgress = step_progress;
    if (m_stepProgress) {
        m_stepProgress->setRange(0, 100);
        m_stepProgress->setValue(0);
        m_stepProgress->hide();
        applyRebuildProgressStyle();
    }
}

// =============================================
//  bindBtns - 绑定导航按钮 + 步骤功能按钮
// =============================================
void StepWizardView::bindBtns(QStackedWidget *stackedWidget,
                              QPushButton *prevButton,
                              QPushButton *nextButton,
                              QPushButton *btn1,
                              QPushButton *btn2,
                              QPushButton *btn3,
                              QPushButton *btn4,
                              QLabel *stepShow,
                              QComboBox *stepComboBox)
{
    m_stackedWidget = stackedWidget;
    m_prevButton    = prevButton;
    m_nextButton    = nextButton;
    m_btn1          = btn1;
    m_btn2          = btn2;
    m_btn3          = btn3;
    m_btn4          = btn4;
    m_stepShow      = stepShow;
    m_stepComboBox  = stepComboBox;

    // 如果有 ComboBox，填充步骤列表
    populateComboBox();

    // 上一步按钮
    if (m_prevButton)
    {
        connect(m_prevButton, &QPushButton::clicked, this, [this]()
        {
            if (m_model) m_model->prevStep();
        });
    }

    // 下一步按钮
    if (m_nextButton)
    {
        connect(m_nextButton, &QPushButton::clicked, this, [this]()
        {
            if (m_model) m_model->nextStep();
        });
    }

    // ComboBox 步骤跳转（调试模式下）
    if (m_stepComboBox)
    {
        connect(m_stepComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, [this](int index)
        {
            if (m_model && index >= 0)
                m_model->gotoStep(index);
        });
    }

    // 步骤功能按钮 btn1 点击
    if (m_btn1)
    {
        connect(m_btn1, &QPushButton::clicked, this, &StepWizardView::onBtn1Clicked);
        connect(m_btn1, &QPushButton::pressed, this, [this]() {
            if (m_model && m_model->currentIndex() == static_cast<int>(WizardStep::RobotExecution))
                startRobotExecutionHold();
        });
        connect(m_btn1, &QPushButton::released, this, [this]() {
            if (m_model && m_model->currentIndex() == static_cast<int>(WizardStep::RobotExecution))
                stopRobotExecutionHold(!m_robotExecutionHoldCompleted);
        });
    }
    if (m_btn2)
    {
        connect(m_btn2, &QPushButton::clicked, this, [this]()
        {
            if (!m_model)
                return;

            if (m_model->currentIndex() == static_cast<int>(WizardStep::CloudCapture))
                emit requestCloudCaptureConfirm();
            else if (m_model->currentIndex() == static_cast<int>(WizardStep::PathPlanning))
                emit requestPathParamConfig();
            else if (m_model->currentIndex() == static_cast<int>(WizardStep::RobotExecution))
                emit requestRobotStop();
        });
    }
}

// =============================================
//  setModel - 与 Model 绑定并连接信号
// =============================================
void StepWizardView::setModel(StepWizardModel *model)
{
    if (m_model)
        disconnect(m_model, nullptr, this, nullptr);

    m_model = model;

    if (!m_model)
        return;

    connect(m_model, &StepWizardModel::stepChanged,
            this, &StepWizardView::onStepChanged);
    connect(m_model, &StepWizardModel::stepFinishedChanged,
            this, &StepWizardView::onStepFinishedChanged);
    connect(m_model, &StepWizardModel::debugModeChanged,
            this, &StepWizardView::onDebugModeChanged);

    refreshAll();
}

// =============================================
//  refreshAll - 全量刷新界面
// =============================================
void StepWizardView::refreshAll()
{
    if (!m_model)
        return;

    onStepChanged(m_model->currentIndex());
    onDebugModeChanged(m_model->debugMode());

    for (int i = 0; i < m_model->totalSteps(); ++i)
    {
        onStepFinishedChanged(i, m_model->isStepFinished(i));
    }
}

// =============================================
//  槽函数实现
// =============================================

void StepWizardView::onStepChanged(int index)
{
    if (m_stackedWidget)
        m_stackedWidget->setCurrentIndex(index);

    m_currentStep = index;
    updateUI(index);
    

    // 同步 ComboBox
    if (m_stepComboBox && index >= 0 && index < m_stepComboBox->count())
    {
        m_stepComboBox->blockSignals(true);
        m_stepComboBox->setCurrentIndex(index);
        m_stepComboBox->blockSignals(false);
    }
}

void StepWizardView::onStepFinishedChanged(int index, bool finished)
{
    Q_UNUSED(finished);
    if (!m_model)
        return;

    // 如果变化的是当前步骤，刷新整个页面
    if (index == m_model->currentIndex())
        updateUI(index);
}

void StepWizardView::onDebugModeChanged(bool enabled)
{
    if (m_debugCheckBox && m_debugCheckBox->isChecked() != enabled)
    {
        m_debugCheckBox->blockSignals(true);
        m_debugCheckBox->setChecked(enabled);
        m_debugCheckBox->blockSignals(false);
    }

    if (m_stepComboBox)
        m_stepComboBox->setEnabled(enabled);

    if (m_model)
        updateStepLabel(m_model->currentIndex());
}

// =============================================
//  私有辅助函数
// =============================================

void StepWizardView::updateStepLabel(int index)
{
    if (!m_stepShow || !m_model)
        return;

    int total = m_model->totalSteps();
    QString text = QStringLiteral("%1/%2")
                       .arg(index + 1)
                       .arg(total);
    m_stepShow->setText(text);
}

void StepWizardView::updateButtonStates(int index)
{
    if (!m_model)
        return;

    int total = m_model->totalSteps();

    if (m_prevButton)
        m_prevButton->setEnabled(index > 0);

    if (m_nextButton)
        m_nextButton->setEnabled(index < total - 1);
}

void StepWizardView::updateCommonUI(int stepIndex)
{
    if (!m_model)
        return;

    if (m_stackedWidget)
        m_stackedWidget->setCurrentIndex(stepIndex);

    updateStepLabel(stepIndex);
    updateButtonStates(stepIndex);
    //日志打印：开始xxx步骤
    if (!(m_model->isStepFinished(stepIndex)))
    {
        QString stepStartLog= QString("开始%1")
            .arg(m_model->stepTitle(stepIndex));
        LOG_INFO(stepStartLog);
    }
}

void StepWizardView::updateUI(int stepIndex)
{
    if (!m_model)
        return;

    updateCommonUI(stepIndex);

    WizardStep step = m_model->stepAt(stepIndex);

    switch (step)
    {
    case WizardStep::SystemCheck:
        updateSystemCheckPage();
        break;

    case WizardStep::CloudCapture:
        updateCloudCapturePage();
        break;

    case WizardStep::CloudRebuild:
        updateCloudRebuildPage();
        break;

    case WizardStep::PathPlanning:
        updatePathPlanningPage();
        break;

    case WizardStep::RobotExecution:
        updateRobotExecutionPage();
        break;

    default:
        break;
    }
}
void StepWizardView::updateSystemCheckPage()
{
    hideStepButtons();
    if (m_stepProgress) {
        setRebuildProgressActive(false);
        m_stepProgress->hide();
    }
    if (m_pathProgress) m_pathProgress->hide();
    if (m_robotProgress) m_robotProgress->hide();

    if (m_btn1)
    {
        clearBtn1Movie();
        m_btn1->setEnabled(true);
        m_btn1->setIcon(QIcon());
        m_btn1->setStyleSheet(QStringLiteral(
            "QPushButton {"
            "  background-color: white;"
            "  border: none;"
            "  border-radius: 8px;"
            "}"
            "QPushButton:hover { background-color: #3B82F6; }"
            "QPushButton:pressed { background-color: #87CEEB; }"
        ));
        if (m_model && m_model->isStepFinished(static_cast<int>(WizardStep::SystemCheck))) {
            m_systemCheckButtonState = SystemCheckButtonState::Success;
        } else if (m_systemCheckButtonState == SystemCheckButtonState::Success) {
            m_systemCheckButtonState = SystemCheckButtonState::Ready;
        }
        applySystemCheckButtonState();
        m_btn1->show();
    }
}

void StepWizardView::updateCloudCapturePage()
{
    hideStepButtons();
    if (m_stepProgress) {
        setRebuildProgressActive(false);
        m_stepProgress->hide();
    }
    if (m_pathProgress) m_pathProgress->hide();
    if (m_robotProgress) m_robotProgress->hide();
    if (m_btn1)
    {
        clearBtn1Movie();
        m_btn1->setEnabled(true);
        m_btn1->setIcon(QIcon());
        m_btn1->setStyleSheet(QStringLiteral(
            "QPushButton {"
            "  background-color: white;"
            "  image: url(:/icons/ui_source/point/capture-40.png);"
            "  background-repeat: no-repeat;"
            "  background-position: center;"
            "  border: none;"
            "  border-radius: 8px;"
            "}"
            "QPushButton:hover { background-color: #3B82F6; }"
            "QPushButton:pressed { background-color: #87CEEB; }"
        ));
        m_btn1->show();
    }
    if (m_btn2)
    {
        m_btn2->setToolTip(QStringLiteral("确认当前关键帧"));
        m_btn2->setStyleSheet(QStringLiteral(
            "QPushButton {"
            "  background-color: white;"
            "  image: url(:/icons/ui_source/check/check-mark-40.png);"
            "  background-repeat: no-repeat;"
            "  background-position: center;"
            "  border: none;"
            "  border-radius: 8px;"
            "}"
            "QPushButton:hover:enabled { background-color: #3B82F6; }"
            "QPushButton:pressed:enabled { background-color: #87CEEB; }"
            "QPushButton:disabled { background-color: #5c5c5c; }"
        ));
        m_btn2->setEnabled(false);
        m_btn2->show();
    }
}

void StepWizardView::updateCloudRebuildPage()
{
    hideStepButtons();
    if (m_stepProgress)
        m_stepProgress->show();
    if (m_pathProgress)
        m_pathProgress->hide();
    if (m_robotProgress)
        m_robotProgress->hide();
}

void StepWizardView::updatePathPlanningPage()
{
    hideStepButtons();

    if (auto *page = m_stackedWidget ? m_stackedWidget->widget(static_cast<int>(WizardStep::PathPlanning)) : nullptr) {
        if (auto *title = page->findChild<QLabel *>(QStringLiteral("stepTitleLabel_4")))
            title->setText(QStringLiteral("路径规划"));
        if (auto *desc = page->findChild<QLabel *>(QStringLiteral("stepDescLabel_4")))
            desc->setText(QStringLiteral("右键重建帧生成完整喷涂轨迹，检查规划结果后进入机械臂执行。"));
    }

    if (m_stepProgress)
        m_stepProgress->hide();

    if (m_pathProgress) {
        m_pathProgress->show();
        applyPathPlanningProgressStyle();
    }
    if (m_robotProgress) m_robotProgress->hide();
    if (m_btn2)
    {
        m_btn2->setEnabled(true);
        m_btn2->setToolTip(QStringLiteral("配置路径参数"));
        m_btn2->setStyleSheet(QStringLiteral(
            "QPushButton {"
            "  background-color: white;"
            "  border: none;"
            "  border-radius: 8px;"
            "}"
            "QPushButton:hover { background-color: #3B82F6; }"
            "QPushButton:pressed { background-color: #87CEEB; }"
        ));
        setBtn2MovieIcon(QString::fromLatin1(kPathParamIcon), kPathParamIconSize);
        m_btn2->show();
    }
}

void StepWizardView::updateRobotExecutionPage()
{
    hideStepButtons();
    if (m_stepProgress) m_stepProgress->hide();
    if (m_pathProgress) m_pathProgress->hide();
    if (m_robotProgress) {
        m_robotProgress->show();
        applyRobotExecutionProgressStyle();
    }

    if (m_btn1) {
        clearBtn1Movie();
        m_btn1->setEnabled(m_robotExecutionEnabled);
        m_btn1->setToolTip(m_robotExecutionEnabled
            ? QStringLiteral("长按确认机械臂轨迹执行")
            : m_robotExecutionDisabledReason);
        m_btn1->setIcon(QIcon(QStringLiteral(":/icons/ui_source/check/check-mark-40.png")));
        m_btn1->setIconSize(QSize(30, 30));
        m_btn1->setStyleSheet(QStringLiteral(
            "QPushButton { background-color: white; border: none; border-radius: 8px; }"
            "QPushButton:hover { background-color: #3B82F6; }"
            "QPushButton:pressed { background-color: #87CEEB; }"
            "QPushButton:disabled { background-color: #4b5563; }"));
        m_btn1->show();
    }
    if (m_btn2) {
        m_btn2->setEnabled(true);
        m_btn2->setToolTip(QStringLiteral("停止机械臂运动"));
        m_btn2->setIcon(QApplication::style()->standardIcon(QStyle::SP_MediaStop));
        m_btn2->setIconSize(QSize(24, 24));
        m_btn2->setStyleSheet(QStringLiteral(
            "QPushButton { background-color: #b91c1c; color: white; border: none; border-radius: 8px; }"
            "QPushButton:hover { background-color: #dc2626; }"
            "QPushButton:pressed { background-color: #7f1d1d; }"));
        m_btn2->show();
    }
}





void StepWizardView::hideStepButtons()
{
    // 先隐藏所有步骤功能按钮
    clearBtn2Movie();
    if (m_btn1) m_btn1->hide();
    if (m_btn2) {
        m_btn2->setIcon(QIcon());
        m_btn2->hide();
    }
    if (m_btn3) m_btn3->hide();
    if (m_btn4) m_btn4->hide();
}

// =============================================
//  bindStatusLabel - 绑定状态标签
// =============================================
void StepWizardView::bindStatusLabel(QLabel *statusLabel)
{
    m_statusLabel = statusLabel;
}

void StepWizardView::setSystemCheckButtonState(SystemCheckButtonState state)
{
    m_systemCheckButtonState = state;
    applySystemCheckButtonState();
}

void StepWizardView::setCloudCaptureConfirmEnabled(bool enabled)
{
    if (m_btn2 && m_model && m_model->currentIndex() == static_cast<int>(WizardStep::CloudCapture))
        m_btn2->setEnabled(enabled);
}

void StepWizardView::setRebuildProgress(int value)
{
    if (!m_stepProgress)
        return;

    m_stepProgress->show();
    m_stepProgress->setValue(std::clamp(value, 0, 100));
    setRebuildProgressActive(value > 0 && value < 100);
}

void StepWizardView::resetRebuildProgress()
{
    if (!m_stepProgress)
        return;

    m_stepProgress->setValue(0);
    setRebuildProgressActive(false);
    m_stepProgress->hide();
}

void StepWizardView::bindPathPlanningProgress(QProgressBar *progress)
{
    m_pathProgress = progress;
    if (!m_pathProgress)
        return;

    m_pathProgress->setRange(0, 100);
    m_pathProgress->setValue(0);
    m_pathProgress->setTextVisible(false);
    m_pathProgress->hide();
    applyPathPlanningProgressStyle();
}

void StepWizardView::setPathPlanningProgress(int value)
{
    if (!m_pathProgress)
        return;

    m_pathProgress->show();
    m_pathProgress->setValue(std::clamp(value, 0, 100));
    applyPathPlanningProgressStyle();
}

void StepWizardView::bindRobotExecutionProgress(QProgressBar *progress)
{
    m_robotProgress = progress;
    if (!m_robotProgress)
        return;

    m_robotProgress->setRange(0, 100);
    m_robotProgress->setValue(0);
    m_robotProgress->setTextVisible(false);
    m_robotProgress->hide();
    applyRobotExecutionProgressStyle();
}

void StepWizardView::setRobotExecutionProgress(int value)
{
    if (!m_robotProgress)
        return;

    m_robotProgress->show();
    m_robotExecutionProgress = std::clamp(value, 0, 100);
    m_robotProgress->setValue(m_robotExecutionProgress);
    applyRobotExecutionProgressStyle();
}

void StepWizardView::setRobotExecutionEnabled(bool enabled, const QString &reason)
{
    m_robotExecutionEnabled = enabled;
    m_robotExecutionDisabledReason = reason;

    if (!enabled)
        stopRobotExecutionHold(true);

    if (!m_btn1 || !m_model ||
        m_model->currentIndex() != static_cast<int>(WizardStep::RobotExecution)) {
        return;
    }

    m_btn1->setEnabled(enabled);
    m_btn1->setToolTip(enabled
        ? QStringLiteral("长按确认机械臂轨迹执行")
        : reason);
}

void StepWizardView::setRebuildProgressActive(bool active)
{
    if (!m_rebuildProgressTimer)
        return;

    if (active && !m_rebuildProgressTimer->isActive())
        m_rebuildProgressTimer->start();
    else if (!active && m_rebuildProgressTimer->isActive())
        m_rebuildProgressTimer->stop();

    applyRebuildProgressStyle();
}

void StepWizardView::applyRebuildProgressStyle()
{
    if (!m_stepProgress)
        return;

    const int pos = 10 + m_rebuildProgressPulse * 7;
    m_stepProgress->setStyleSheet(QStringLiteral(
        "QProgressBar {"
        "  background-color: #555555;"
        "  border: none;"
        "  border-radius: 0px;"
        "}"
        "QProgressBar::chunk {"
        "  background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,"
        "    stop:0 #0078d4,"
        "    stop:%1 #38bdf8,"
        "    stop:%2 #0078d4);"
        "  border-radius: 0px;"
        "}"
    ).arg(QString::number(pos / 100.0, 'f', 2),
          QString::number(std::min(1.0, (pos + 18) / 100.0), 'f', 2)));
}

void StepWizardView::applyPathPlanningProgressStyle()
{
    if (!m_pathProgress)
        return;

    const int pos = 10 + m_pathProgressPulse * 7;
    m_pathProgress->setStyleSheet(QStringLiteral(
        "QProgressBar {"
        "  background-color: #555555;"
        "  border: none;"
        "  border-radius: 0px;"
        "}"
        "QProgressBar::chunk {"
        "  background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,"
        "    stop:0 #22c55e,"
        "    stop:%1 #facc15,"
        "    stop:%2 #22c55e);"
        "  border-radius: 0px;"
        "}"
    ).arg(QString::number(pos / 100.0, 'f', 2),
          QString::number(std::min(1.0, (pos + 18) / 100.0), 'f', 2)));
}

void StepWizardView::applyRobotExecutionProgressStyle()
{
    if (!m_robotProgress)
        return;

    const int pos = 10 + m_robotProgressPulse * 7;
    m_robotProgress->setStyleSheet(QStringLiteral(
        "QProgressBar { background-color: #555555; border: none; border-radius: 0px; }"
        "QProgressBar::chunk {"
        "  background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,"
        "    stop:0 #2563eb, stop:%1 #93c5fd, stop:%2 #2563eb);"
        "  border-radius: 0px;"
        "}"
    ).arg(QString::number(pos / 100.0, 'f', 2),
          QString::number(std::min(1.0, (pos + 18) / 100.0), 'f', 2)));
}

void StepWizardView::startRobotExecutionHold()
{
    if (!m_robotExecutionEnabled || !m_robotProgress || !m_robotExecutionHoldTimer ||
        m_robotExecutionHoldTimer->isActive())
        return;

    m_robotExecutionProgress = 0;
    m_robotExecutionHoldCompleted = false;
    m_robotProgress->setValue(0);
    m_robotProgress->show();
    m_robotExecutionHoldTimer->start();
}

void StepWizardView::stopRobotExecutionHold(bool resetProgress)
{
    if (m_robotExecutionHoldTimer)
        m_robotExecutionHoldTimer->stop();

    if (resetProgress && m_robotProgress) {
        m_robotExecutionProgress = 0;
        m_robotProgress->setValue(0);
        applyRobotExecutionProgressStyle();
    }
}

// =============================================
//  onBtn1Clicked - 步骤功能按钮点击（按当前步骤分发信号）
// =============================================
void StepWizardView::onBtn1Clicked()
{
    if (!m_model)
        return;

    int index = m_model->currentIndex();

    if (index == static_cast<int>(WizardStep::SystemCheck))
    {
        setSystemCheckButtonState(SystemCheckButtonState::Loading);
        emit requestSystemCheck();
    }
    else if (index == static_cast<int>(WizardStep::CloudCapture))
    {
        emit requestCloudCapture();
    }
}

void StepWizardView::populateComboBox()
{
    if (!m_stepComboBox || !m_model)
        return;

    m_stepComboBox->clear();
    for (int i = 0; i < m_model->totalSteps(); ++i)
    {
        m_stepComboBox->addItem(m_model->stepTitle(i));
    }
}

void StepWizardView::applySystemCheckButtonState()
{
    if (!m_btn1)
        return;

    switch (m_systemCheckButtonState)
    {
    case SystemCheckButtonState::Ready:
        setBtn1StaticIcon(QString::fromLatin1(kSystemCheckStartIcon), kSystemCheckReadyIconSize);
        break;
    case SystemCheckButtonState::Loading:
        setBtn1MovieIcon(QString::fromLatin1(kSystemCheckLoadingIcon), kSystemCheckLoadingIconSize);
        break;
    case SystemCheckButtonState::Success:
        setBtn1MovieIcon(QString::fromLatin1(kSystemCheckSuccessIcon), kSystemCheckSuccessIconSize);
        break;
    }
}

void StepWizardView::setBtn1StaticIcon(const QString &resourcePath, int size)
{
    if (!m_btn1)
        return;

    clearBtn1Movie();
    m_btn1->setIcon(QIcon(resourcePath));
    m_btn1->setIconSize(QSize(size, size));
}

void StepWizardView::setBtn1MovieIcon(const QString &resourcePath, int size)
{
    if (!m_btn1)
        return;

    clearBtn1Movie();
    m_btn1Movie = new QMovie(resourcePath, QByteArray(), this);
    m_btn1->setIconSize(QSize(size, size));

    connect(m_btn1Movie, &QMovie::frameChanged, this, [this]() {
        if (m_btn1 && m_btn1Movie)
            m_btn1->setIcon(QIcon(m_btn1Movie->currentPixmap()));
    });

    m_btn1Movie->start();
}

void StepWizardView::clearBtn1Movie()
{
    if (!m_btn1Movie)
        return;

    m_btn1Movie->stop();
    m_btn1Movie->deleteLater();
    m_btn1Movie = nullptr;
}

void StepWizardView::setBtn2MovieIcon(const QString &resourcePath, int size)
{
    if (!m_btn2)
        return;

    clearBtn2Movie();
    m_btn2Movie = new QMovie(resourcePath, QByteArray(), this);
    m_btn2Movie->setScaledSize(QSize(size, size));
    m_btn2->setIconSize(QSize(size, size));

    connect(m_btn2Movie, &QMovie::frameChanged, this, [this]() {
        if (m_btn2 && m_btn2Movie)
            m_btn2->setIcon(QIcon(m_btn2Movie->currentPixmap()));
    });

    m_btn2Movie->start();
}

void StepWizardView::clearBtn2Movie()
{
    if (!m_btn2Movie)
        return;

    m_btn2Movie->stop();
    m_btn2Movie->deleteLater();
    m_btn2Movie = nullptr;
}
