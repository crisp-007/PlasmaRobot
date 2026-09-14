#include "demo_rotation_executor.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr int kMoveSpeedPercent = 5;
constexpr int kJointStateFreshnessMs = 1500;
constexpr int kMoveResultTimeoutMs = 90000;
constexpr int kPostMoveJointStateTimeoutMs = 2500;
constexpr double kMaximumSweepDeg = 180.0;
constexpr double kJoint6LimitDeg = 360.0;
constexpr double kMaximumLayerOffsetMeters = 0.05;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
}

DemoRotationExecutor::DemoRotationExecutor(QObject *parent)
    : QObject(parent)
{
    m_resultTimeout.setSingleShot(true);
    m_resultTimeout.setInterval(kMoveResultTimeoutMs);
    connect(&m_resultTimeout, &QTimer::timeout, this, [this]() {
        const QString message = m_waitingForPostMoveJointState
            ? QStringLiteral("层间移动完成后未收到新的 /joint_states")
            : QStringLiteral("等待机械臂运动结果超时");
        fail(message, true);
    });

    m_jointStateWatchdog.setInterval(250);
    connect(&m_jointStateWatchdog, &QTimer::timeout, this, [this]() {
        if (!m_running)
            return;
        if (!m_lastJointStateAt.isValid() ||
            m_lastJointStateAt.msecsTo(QDateTime::currentDateTime()) > kJointStateFreshnessMs) {
            fail(QStringLiteral("执行期间 /joint_states 数据中断"), true);
        }
    });
}

void DemoRotationExecutor::updateJointState(const std::array<double, 6> &jointsRad,
                                            const QDateTime &receivedAt)
{
    m_latestJoints = jointsRad;
    m_lastJointStateAt = receivedAt;
    m_hasJointState = true;
    ++m_jointStateSequence;
    if (m_running && m_waitingForPostMoveJointState &&
        m_jointStateSequence > m_moveLResultJointStateSequence) {
        beginNextLayerAfterStateUpdate();
    }
}

bool DemoRotationExecutor::start(const QVector<LayerPlan> &layers,
                                 int toolAxis,
                                 double toolAxisDirection,
                                 QString *errorMessage)
{
    if (!validateStart(layers, toolAxis, toolAxisDirection, errorMessage))
        return false;

    m_layers = layers;
    m_toolAxis = toolAxis;
    m_toolAxisDirection = toolAxisDirection;
    m_currentLayer = 0;
    m_completedCommands = 0;
    const int layerCount = static_cast<int>(m_layers.size());
    m_totalCommands = layerCount * 4 + std::max(0, layerCount - 1);
    m_layerBaselineJoints = m_latestJoints;
    m_phase = Phase::LeftSweep;
    m_expectedResult = ExpectedResult::None;
    m_waitingForPostMoveJointState = false;
    m_moveLResultJointStateSequence = m_jointStateSequence;
    m_running = true;
    m_jointStateWatchdog.start();
    emit progressChanged(0);
    emit logMessage(QStringLiteral(
        "逐层旋转演示开始：层数=%1，层内锁定 J1-J5，层间使用工具坐标轴相对移动；真实等离子输出保持关闭。")
        .arg(m_layers.size()));
    sendCurrentCommand();
    return true;
}

bool DemoRotationExecutor::isRunning() const
{
    return m_running;
}

bool DemoRotationExecutor::hasFreshJointState(int maxAgeMs) const
{
    return m_hasJointState && m_lastJointStateAt.isValid() && maxAgeMs >= 0 &&
           m_lastJointStateAt.msecsTo(QDateTime::currentDateTime()) <= maxAgeMs;
}

void DemoRotationExecutor::handleMoveJResult(bool ok)
{
    if (!m_running || m_expectedResult != ExpectedResult::MoveJ)
        return;

    m_resultTimeout.stop();
    if (!ok) {
        fail(QStringLiteral("第 %1 层 MoveJ 失败").arg(m_currentLayer + 1), true);
        return;
    }

    switch (m_phase) {
    case Phase::LeftSweep:
        completeCurrentCommand(QStringLiteral("左半圈喷涂"));
        m_phase = Phase::LeftReturn;
        break;
    case Phase::LeftReturn:
        completeCurrentCommand(QStringLiteral("左半圈断能回零"));
        m_phase = Phase::RightSweep;
        break;
    case Phase::RightSweep:
        completeCurrentCommand(QStringLiteral("右半圈喷涂"));
        m_phase = Phase::RightReturn;
        break;
    case Phase::RightReturn:
        completeCurrentCommand(QStringLiteral("右半圈断能回零"));
        if (m_currentLayer + 1 >= m_layers.size()) {
            finishSuccessfully();
            return;
        }
        m_phase = Phase::LayerTransition;
        break;
    case Phase::LayerTransition:
        fail(QStringLiteral("内部状态错误：层间移动等待了 MoveJ 结果"), true);
        return;
    }

    sendCurrentCommand();
}

void DemoRotationExecutor::handleMoveLOffsetResult(bool ok)
{
    if (!m_running || m_expectedResult != ExpectedResult::MoveLOffset)
        return;

    m_resultTimeout.stop();
    if (!ok) {
        fail(QStringLiteral("第 %1 层到下一层的工具坐标移动失败")
             .arg(m_currentLayer + 1), true);
        return;
    }

    completeCurrentCommand(QStringLiteral("断能层间移动"));
    m_expectedResult = ExpectedResult::None;
    m_waitingForPostMoveJointState = true;
    m_moveLResultJointStateSequence = m_jointStateSequence;
    m_resultTimeout.setInterval(kPostMoveJointStateTimeoutMs);
    m_resultTimeout.start();
    emit logMessage(QStringLiteral(
        "逐层旋转演示：层间移动完成，等待新的关节状态后锁定下一层姿态。"));
}

void DemoRotationExecutor::abort(const QString &reason)
{
    if (!m_running)
        return;
    fail(reason, true);
}

void DemoRotationExecutor::sendCurrentCommand()
{
    if (!m_running || m_currentLayer < 0 || m_currentLayer >= m_layers.size())
        return;

    const LayerPlan &layer = m_layers[m_currentLayer];
    QString label;
    bool plasmaEnabled = false;
    if (m_phase == Phase::LayerTransition) {
        const double distance = layer.offsetToNextMeters * m_toolAxisDirection;
        if (!std::isfinite(distance) || std::abs(distance) > kMaximumLayerOffsetMeters) {
            fail(QStringLiteral("层间移动距离 %1m 超出执行限制")
                 .arg(distance, 0, 'f', 5), true);
            return;
        }
        label = QStringLiteral("断能移动到下一层");
        m_expectedResult = ExpectedResult::MoveLOffset;
        m_commandSentAt = QDateTime::currentDateTime();
        emit logMessage(QStringLiteral(
            "逐层旋转演示：第 %1/%2 层完成，沿工具轴移动 %3 mm，喷涂状态=关闭。")
            .arg(m_currentLayer + 1).arg(m_layers.size())
            .arg(distance * 1000.0, 0, 'f', 2));
        emit moveLOffsetRequested(m_toolAxis, distance, kMoveSpeedPercent);
    } else {
        auto target = m_layerBaselineJoints;
        switch (m_phase) {
        case Phase::LeftSweep:
            target[5] += layer.leftSweepDeg * kDegToRad;
            label = QStringLiteral("左半圈喷涂");
            plasmaEnabled = true;
            break;
        case Phase::LeftReturn:
            label = QStringLiteral("左半圈断能回零");
            break;
        case Phase::RightSweep:
            target[5] -= layer.rightSweepDeg * kDegToRad;
            label = QStringLiteral("右半圈喷涂");
            plasmaEnabled = true;
            break;
        case Phase::RightReturn:
            label = QStringLiteral("右半圈断能回零");
            break;
        case Phase::LayerTransition:
            break;
        }

        const double targetJoint6Deg = target[5] / kDegToRad;
        if (targetJoint6Deg < -kJoint6LimitDeg || targetJoint6Deg > kJoint6LimitDeg) {
            fail(QStringLiteral("第 %1 层 J6 目标 %2°超过 ±%3°限位")
                 .arg(m_currentLayer + 1)
                 .arg(targetJoint6Deg, 0, 'f', 1)
                 .arg(kJoint6LimitDeg, 0, 'f', 0), true);
            return;
        }
        m_expectedResult = ExpectedResult::MoveJ;
        m_commandSentAt = QDateTime::currentDateTime();
        emit logMessage(QStringLiteral(
            "逐层旋转演示：第 %1/%2 层开始%3，喷涂状态=%4（仅界面模拟）。")
            .arg(m_currentLayer + 1).arg(m_layers.size()).arg(label,
                 plasmaEnabled ? QStringLiteral("开启") : QStringLiteral("关闭")));
        emit moveJRequested(QVector<double>(target.begin(), target.end()), kMoveSpeedPercent);
    }

    m_resultTimeout.setInterval(kMoveResultTimeoutMs);
    if (m_running)
        m_resultTimeout.start();
}

void DemoRotationExecutor::completeCurrentCommand(const QString &label)
{
    ++m_completedCommands;
    emit progressChanged(m_totalCommands > 0
        ? std::clamp(100 * m_completedCommands / m_totalCommands, 0, 100) : 0);
    emit logMessage(QStringLiteral("逐层旋转演示：第 %1/%2 层%3完成。")
                    .arg(m_currentLayer + 1).arg(m_layers.size()).arg(label));
}

void DemoRotationExecutor::beginNextLayerAfterStateUpdate()
{
    if (!m_running || !m_waitingForPostMoveJointState)
        return;
    m_resultTimeout.stop();
    m_waitingForPostMoveJointState = false;
    ++m_currentLayer;
    if (m_currentLayer >= m_layers.size()) {
        finishSuccessfully();
        return;
    }
    m_layerBaselineJoints = m_latestJoints;
    QString joint6Error;
    if (!validateLayerJoint6Targets(m_currentLayer, &joint6Error)) {
        fail(joint6Error, true);
        return;
    }
    m_phase = Phase::LeftSweep;
    m_expectedResult = ExpectedResult::None;
    sendCurrentCommand();
}

bool DemoRotationExecutor::validateLayerJoint6Targets(int layerIndex,
                                                      QString *errorMessage) const
{
    if (layerIndex < 0 || layerIndex >= m_layers.size()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("机械臂执行层索引无效");
        return false;
    }

    const LayerPlan &layer = m_layers[layerIndex];
    const double baselineDeg = m_layerBaselineJoints[5] / kDegToRad;
    if (baselineDeg + layer.leftSweepDeg > kJoint6LimitDeg ||
        baselineDeg - layer.rightSweepDeg < -kJoint6LimitDeg) {
        if (errorMessage) {
            *errorMessage = QStringLiteral(
                "第 %1 层 J6 基线=%2°，左右旋转后将超过 ±%3°限位")
                .arg(layerIndex + 1)
                .arg(baselineDeg, 0, 'f', 1)
                .arg(kJoint6LimitDeg, 0, 'f', 0);
        }
        return false;
    }
    return true;
}

void DemoRotationExecutor::finishSuccessfully()
{
    m_running = false;
    m_expectedResult = ExpectedResult::None;
    m_waitingForPostMoveJointState = false;
    m_resultTimeout.stop();
    m_jointStateWatchdog.stop();
    emit progressChanged(100);
    emit logMessage(QStringLiteral("逐层旋转演示完成：末层 J6 已回到该层入口零位。"));
    emit finished(true, QStringLiteral("逐层左右半圈旋转演示完成"));
}

void DemoRotationExecutor::fail(const QString &message, bool requestStop)
{
    if (!m_running)
        return;

    m_running = false;
    m_expectedResult = ExpectedResult::None;
    m_waitingForPostMoveJointState = false;
    m_resultTimeout.stop();
    m_jointStateWatchdog.stop();
    if (requestStop)
        emit moveStopRequested();
    emit logMessage(QStringLiteral("预设旋转演示已停止：%1").arg(message));
    emit finished(false, message);
}

bool DemoRotationExecutor::validateStart(const QVector<LayerPlan> &layers,
                                         int toolAxis,
                                         double toolAxisDirection,
                                         QString *errorMessage) const
{
    auto reject = [errorMessage](const QString &message) {
        if (errorMessage)
            *errorMessage = message;
        return false;
    };

    if (m_running)
        return reject(QStringLiteral("已有机械臂演示动作正在执行"));
    if (!m_hasJointState || !m_lastJointStateAt.isValid())
        return reject(QStringLiteral("尚未收到有效的 /joint_states"));
    if (m_lastJointStateAt.msecsTo(QDateTime::currentDateTime()) > kJointStateFreshnessMs)
        return reject(QStringLiteral("/joint_states 已过期，请检查机械臂驱动连接"));
    if (layers.isEmpty())
        return reject(QStringLiteral("没有可执行的闭合切片层"));
    if (toolAxis < 0 || toolAxis > 2 ||
        (!qFuzzyCompare(toolAxisDirection, 1.0) && !qFuzzyCompare(toolAxisDirection, -1.0)))
        return reject(QStringLiteral("工具轴或轴向方向配置无效"));
    for (int layerIndex = 0; layerIndex < layers.size(); ++layerIndex) {
        const LayerPlan &layer = layers[layerIndex];
        if (!std::isfinite(layer.leftSweepDeg) || !std::isfinite(layer.rightSweepDeg) ||
            layer.leftSweepDeg <= 0.0 || layer.rightSweepDeg <= 0.0 ||
            layer.leftSweepDeg > kMaximumSweepDeg ||
            layer.rightSweepDeg > kMaximumSweepDeg) {
            return reject(QStringLiteral("第 %1 层旋转角不在 0° 到 180°之间")
                          .arg(layerIndex + 1));
        }
        if (layerIndex + 1 < layers.size() &&
            (!std::isfinite(layer.offsetToNextMeters) ||
             std::abs(layer.offsetToNextMeters) > kMaximumLayerOffsetMeters)) {
            return reject(QStringLiteral("第 %1 层的层间移动距离无效").arg(layerIndex + 1));
        }
    }
    if (!std::all_of(m_latestJoints.begin(), m_latestJoints.end(),
                     [](double value) { return std::isfinite(value); })) {
        return reject(QStringLiteral("当前关节状态包含无效数值"));
    }

    const double currentJoint6Deg = m_latestJoints[5] / kDegToRad;
    if (currentJoint6Deg + layers.front().leftSweepDeg > kJoint6LimitDeg ||
        currentJoint6Deg - layers.front().rightSweepDeg < -kJoint6LimitDeg) {
        return reject(QStringLiteral(
            "当前 J6=%1°，左右旋转后将超过 ±%2°限位，请先通过示教器调整入口零位")
            .arg(currentJoint6Deg, 0, 'f', 1).arg(kJoint6LimitDeg, 0, 'f', 0));
    }
    return true;
}
