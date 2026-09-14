#ifndef STEPWIZARDMODEL_H
#define STEPWIZARDMODEL_H

#include "logmanager.h"
#include <QObject>
#include <QString>


// =============================================
//  流程步骤枚举
// =============================================
enum class WizardStep
{
    SystemCheck = 0,   // 系统自检
    CloudCapture,      // 点云采集
    CloudRebuild,      // 点云重建
    PathPlanning,      // 路径规划
    RobotExecution,    // 机械臂执行
    Count              // 步骤总数
};

// =============================================
//  StepWizardModel
//  - 只负责流程规则和状态管理
//  - 不直接操作 UI，不依赖 MainWindow
//  - 通过信号通知外部更新界面
// =============================================
class StepWizardModel : public QObject
{
    Q_OBJECT

public:
    explicit StepWizardModel(QObject *parent = nullptr);
    ~StepWizardModel() override;

    // ---------- 查询接口 ----------
    int currentIndex() const;
    int totalSteps() const;
    WizardStep stepAt(int index) const;
    QString stepTitle(int index) const;
    QString stepDescription(int index) const;
    bool isStepFinished(int index) const;
    bool debugMode() const;

    // ---------- 导航接口 ----------
    void nextStep();
    void prevStep();
    bool gotoStep(int index);

    // ---------- 状态设置 ----------
    void setDebugMode(bool enabled);
    void setStepFinished(WizardStep step, bool finished);
    void setCurrentStepFinished(bool finished);
    void resetWorkflow();

    // ---------- 内部判断 ----------
    bool canGoNext(QString *reason = nullptr) const;
    bool canEnterStep(int index, QString *reason = nullptr) const;

signals:
    // ---------- 信号 ----------
    void stepChanged(int index);
    void stepFinishedChanged(int index, bool finished);
    void debugModeChanged(bool enabled);
    void warningMessage(const QString &message);
    void workflowFinished();

private:
    int m_currentIndex = 0;
    bool m_debugMode = false;
    static constexpr int STEP_COUNT = static_cast<int>(WizardStep::Count);

    // 记录每个步骤是否完成
    bool m_stepFinished[STEP_COUNT] = { false };

    // 步骤标题与描述
    struct StepInfo
    {
        QString title;
        QString description;
    };
    static const StepInfo s_stepInfos[STEP_COUNT];
};

#endif // STEPWIZARDMODEL_H
