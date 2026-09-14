#ifndef STEPWIZARDVIEW_H
#define STEPWIZARDVIEW_H

#include <QObject>
#include <QStackedWidget>
#include <QPushButton>
#include <QLabel>
#include <QCheckBox>
#include <QComboBox>
#include <QProgressBar>
#include <QTimer>

class StepWizardModel;
class QMovie;

// =============================================
//  StepWizardView
//  - 只负责更新界面显示
//  - 不执行任何业务逻辑
//  - bindWidgets()  → 绑定显示控件（描述、图片、步骤文字、进度条）
//  - bindBtns()     → 绑定导航按钮 + 步骤功能按钮
// =============================================
class StepWizardView : public QObject
{
    Q_OBJECT

public:
    enum class SystemCheckButtonState
    {
        Ready,
        Loading,
        Success
    };

    explicit StepWizardView(QObject *parent = nullptr);
    ~StepWizardView() override;

    // 绑定显示控件（由 MainWindow 调用）
    void bindWidgets(QLabel *step_dsc,
                     QLabel *stepimg,
                     QLabel *stepLabel,
                     QProgressBar *step_progress = nullptr);

    // 绑定导航按钮 + 步骤功能按钮
    void bindBtns(QStackedWidget *stackedWidget,
                  QPushButton *prevButton,
                  QPushButton *nextButton,
                  QPushButton *btn1,
                  QPushButton *btn2,
                  QPushButton *btn3,
                  QPushButton *btn4,
                  QLabel *stepShow,
                  QComboBox *stepComboBox = nullptr);

    // 与 Model 绑定
    void setModel(StepWizardModel *model);

    // 外部可调用的刷新接口
    void refreshAll();

    // 设置状态标签（用于显示自检结果文字）
    void bindStatusLabel(QLabel *statusLabel);
    void setSystemCheckButtonState(SystemCheckButtonState state);
    void setCloudCaptureConfirmEnabled(bool enabled);
    void setRebuildProgress(int value);
    void resetRebuildProgress();
    void bindPathPlanningProgress(QProgressBar *progress);
    void setPathPlanningProgress(int value);
    void bindRobotExecutionProgress(QProgressBar *progress);
    void setRobotExecutionProgress(int value);
    void setRobotExecutionEnabled(bool enabled, const QString &reason = QString());

signals:
    // 步骤功能按钮请求信号
    void requestSystemCheck();
    void requestCloudCapture();
    void requestCloudCaptureConfirm();
    void requestCloudRebuild();
    void requestPathExecution();
    void requestRobotStop();
    void requestPathParamConfig();

public slots:
    void onStepChanged(int index);
    void onStepFinishedChanged(int index, bool finished);
    void onDebugModeChanged(bool enabled);
    void onBtn1Clicked();

private:
    void updateStepLabel(int index);
    void updateButtonStates(int index);
    //void showStepButtons(int stepIndex);
    void updateCommonUI(int stepIndex);
    void updateUI(int stepIndex);
    void updateSystemCheckPage();
    void updateCloudCapturePage();
    void updateCloudRebuildPage();
    void updatePathPlanningPage();
    void updateRobotExecutionPage();
    void hideStepButtons();
    void populateComboBox();
    void applySystemCheckButtonState();
    void applyRebuildProgressStyle();
    void applyPathPlanningProgressStyle();
    void applyRobotExecutionProgressStyle();
    void setRebuildProgressActive(bool active);
    void startRobotExecutionHold();
    void stopRobotExecutionHold(bool resetProgress);
    void setBtn1StaticIcon(const QString &resourcePath, int size);
    void setBtn1MovieIcon(const QString &resourcePath, int size);
    void clearBtn1Movie();
    void setBtn2MovieIcon(const QString &resourcePath, int size);
    void clearBtn2Movie();

    // bindWidgets 绑定的显示控件
    QLabel         *m_stepDsc     = nullptr;
    QLabel         *m_stepImg     = nullptr;
    QLabel         *m_stepShow    = nullptr;
    QProgressBar   *m_stepProgress = nullptr;
    QProgressBar   *m_pathProgress = nullptr;
    QProgressBar   *m_robotProgress = nullptr;

    // bindBtns 绑定的导航控件
    QStackedWidget *m_stackedWidget  = nullptr;
    QPushButton    *m_prevButton     = nullptr;
    QPushButton    *m_nextButton     = nullptr;

    // 步骤功能按钮（按步骤上下文切换显示）
    QPushButton    *m_btn1 = nullptr;
    QPushButton    *m_btn2 = nullptr;
    QPushButton    *m_btn3 = nullptr;
    QPushButton    *m_btn4 = nullptr;

    QCheckBox      *m_debugCheckBox  = nullptr;
    QComboBox      *m_stepComboBox   = nullptr;
    QLabel         *m_statusLabel    = nullptr;

    int             m_currentStep = 0;
    int             m_rebuildProgressPulse = 0;
    int             m_pathProgressPulse = 0;
    int             m_robotProgressPulse = 0;
    int             m_robotExecutionProgress = 0;
    bool            m_robotExecutionHoldCompleted = false;
    bool            m_robotExecutionEnabled = false;
    QString         m_robotExecutionDisabledReason;
    StepWizardModel *m_model = nullptr;
    QMovie          *m_btn1Movie = nullptr;
    QMovie          *m_btn2Movie = nullptr;
    QTimer          *m_rebuildProgressTimer = nullptr;
    QTimer          *m_robotExecutionHoldTimer = nullptr;
    SystemCheckButtonState m_systemCheckButtonState = SystemCheckButtonState::Ready;
};

#endif // STEPWIZARDVIEW_H
