#include "StepWizardModel.h"

// =============================================
//  步骤静态信息表
// =============================================
const StepWizardModel::StepInfo StepWizardModel::s_stepInfos[STEP_COUNT] =
{
    { QStringLiteral("系统自检"),   QStringLiteral("正在检查设备连接状态、传感器数据及系统环境...") },
    { QStringLiteral("残腔采集"),   QStringLiteral("请将末端相机移动至残腔上方合适的位置，以便采集残腔空间信息。") },
    { QStringLiteral("残腔重建"),   QStringLiteral("请在残腔采集序列中，右键选择合适的关键帧进行重建及术区选择。") },
    { QStringLiteral("路径规划"),   QStringLiteral("右键重建帧生成完整喷涂轨迹，检查规划结果后进入机械臂执行。") },
    { QStringLiteral("机械臂执行"), QStringLiteral("系统将规划至预入口，经碰撞检查后沿入口法向进入，并执行已确认的喷涂轨迹。") },
};

// =============================================
//  构造 / 析构
// =============================================
StepWizardModel::StepWizardModel(QObject *parent)
    : QObject(parent)
    , m_currentIndex(0)
    , m_debugMode(false)
{
    // 初始化所有步骤为未完成
    for (int i = 0; i < STEP_COUNT; ++i)
        m_stepFinished[i] = false;
}

StepWizardModel::~StepWizardModel() = default;

// =============================================
//  查询接口
// =============================================
int StepWizardModel::currentIndex() const
{
    return m_currentIndex;
}

int StepWizardModel::totalSteps() const
{
    return STEP_COUNT;
}

WizardStep StepWizardModel::stepAt(int index) const
{
    if (index < 0 || index >= STEP_COUNT)
        return WizardStep::SystemCheck;
    return static_cast<WizardStep>(index);
}

QString StepWizardModel::stepTitle(int index) const
{
    if (index < 0 || index >= STEP_COUNT)
        return QString();
    return s_stepInfos[index].title;
}

QString StepWizardModel::stepDescription(int index) const
{
    if (index < 0 || index >= STEP_COUNT)
        return QString();
    return s_stepInfos[index].description;
}

bool StepWizardModel::isStepFinished(int index) const
{
    if (index < 0 || index >= STEP_COUNT)
        return false;
    return m_stepFinished[index];
}

bool StepWizardModel::debugMode() const
{
    return m_debugMode;
}

// =============================================
//  导航接口
// =============================================

/**
 * nextStep - 尝试进入下一步
 * 普通模式：当前步骤必须已完成，且不能超过最后一步
 * 调试模式：忽略步骤完成状态，任意跳转
 */
void StepWizardModel::nextStep()
{
    // 已经在最后一步，触发流程完成
    if (m_currentIndex >= STEP_COUNT - 1)
    {
        // 最后一步：检查是否已完成
        if (m_stepFinished[m_currentIndex] || m_debugMode)
        {
            emit workflowFinished();
        }
        else
        {
            emit warningMessage(QStringLiteral("请先完成当前步骤再结束流程。"));
        }
        return;
    }

    // 检查能否进入下一步
    QString reason;
    if (!canGoNext(&reason))
    {
        emit warningMessage(reason);
        return;
    }

    if (m_debugMode
        && m_currentIndex == static_cast<int>(WizardStep::SystemCheck)
        && !m_stepFinished[m_currentIndex])
    {
        emit warningMessage(QStringLiteral("当前处于调试模式，已跳过系统自检。"));
    }

    m_currentIndex++;
    emit stepChanged(m_currentIndex);
}

/**
 * prevStep - 返回上一步（始终允许）
 */
void StepWizardModel::prevStep()
{
    if (m_currentIndex > 0)
    {
        m_currentIndex--;
        emit stepChanged(m_currentIndex);
    }
}

/**
 * gotoStep - 跳转到指定步骤
 * 普通模式：不允许跳转
 * 调试模式：允许跳转到任意步骤
 * @return 是否成功跳转
 */
bool StepWizardModel::gotoStep(int index)
{
    if (index < 0 || index >= STEP_COUNT)
        return false;

    if (!m_debugMode)
    {
        emit warningMessage(QStringLiteral("非调试模式下不允许跳转步骤。"));
        return false;
    }

    // 调试模式：直接跳转，忽略步骤完成状态
    m_currentIndex = index;
    emit stepChanged(m_currentIndex);
    return true;
}

// =============================================
//  状态设置
// =============================================

void StepWizardModel::setDebugMode(bool enabled)
{
    if (m_debugMode != enabled)
    {
        m_debugMode = enabled;
        emit debugModeChanged(m_debugMode);
    }
}

void StepWizardModel::setStepFinished(WizardStep step, bool finished)
{
    int idx = static_cast<int>(step);
    if (idx < 0 || idx >= STEP_COUNT)
        return;

    if (m_stepFinished[idx] != finished)
    {
        m_stepFinished[idx] = finished;
        emit stepFinishedChanged(idx, finished);
    }
}

void StepWizardModel::setCurrentStepFinished(bool finished)
{
    setStepFinished(static_cast<WizardStep>(m_currentIndex), finished);
}

void StepWizardModel::resetWorkflow()
{
    // 重置所有步骤状态
    for (int i = 0; i < STEP_COUNT; ++i)
    {
        if (m_stepFinished[i])
        {
            m_stepFinished[i] = false;
            emit stepFinishedChanged(i, false);
        }
    }

    // 回到第一步
    if (m_currentIndex != 0)
    {
        m_currentIndex = 0;
        emit stepChanged(m_currentIndex);
    }
}

// =============================================
//  内部判断
// =============================================

/**
 * canGoNext - 判断是否可以进入下一步
 * 普通模式下，当前步骤必须已完成
 * 调试模式下，始终允许
 * @param reason  若不允许，填入提示原因
 */
bool StepWizardModel::canGoNext(QString *reason) const
{
    // 调试模式始终允许
    if (m_debugMode)
        return true;

    // 最后一步不允许再 next（由 nextStep 逻辑单独处理 workflowFinished）
    if (m_currentIndex >= STEP_COUNT - 1)
    {
        if (reason)
            *reason = QStringLiteral("当前已是最后一步。");
        return false;
    }

    // 普通模式：当前步骤必须已完成
    if (!m_stepFinished[m_currentIndex])
    {
        if (reason)
            *reason = QStringLiteral("请先完成当前步骤「%1」后再进入下一步。").arg(stepTitle(m_currentIndex));
        return false;
    }

    return true;
}

/**
 * canEnterStep - 判断是否允许进入指定步骤
 * 普通模式：目标步骤必须是当前步骤或前面已完成的所有步骤
 * 调试模式：始终允许
 * @param reason  若不允许，填入提示原因
 */
bool StepWizardModel::canEnterStep(int index, QString *reason) const
{
    if (index < 0 || index >= STEP_COUNT)
    {
        if (reason)
            *reason = QStringLiteral("无效的步骤索引。");
        return false;
    }

    // 调试模式始终允许
    if (m_debugMode)
        return true;

    // 允许留在当前步骤
    if (index == m_currentIndex)
        return true;

    // 允许返回已完成的前序步骤
    if (index < m_currentIndex)
    {
        // 检查目标步骤之前的所有步骤是否都已完成
        for (int i = 0; i < index; ++i)
        {
            if (!m_stepFinished[i])
            {
                if (reason)
                    *reason = QStringLiteral("步骤「%1」尚未完成，无法跳转到「%2」。")
                                  .arg(stepTitle(i), stepTitle(index));
                return false;
            }
        }
        return true;
    }

    // index > m_currentIndex：前进（由 nextStep 统一处理）
    if (reason)
        *reason = QStringLiteral("请通过「下一步」按钮依次推进流程。");
    return false;
}
