#ifndef DEMO_ROTATION_EXECUTOR_H
#define DEMO_ROTATION_EXECUTOR_H

#include <array>
#include <cstdint>

#include <QDateTime>
#include <QObject>
#include <QTimer>
#include <QVector>

class DemoRotationExecutor : public QObject
{
    Q_OBJECT

public:
    struct LayerPlan
    {
        int sliceId = 0;
        double leftSweepDeg = 0.0;
        double rightSweepDeg = 0.0;
        double offsetToNextMeters = 0.0;
    };

    explicit DemoRotationExecutor(QObject *parent = nullptr);

    void updateJointState(const std::array<double, 6> &jointsRad,
                          const QDateTime &receivedAt = QDateTime::currentDateTime());
    bool start(const QVector<LayerPlan> &layers,
               int toolAxis,
               double toolAxisDirection,
               QString *errorMessage = nullptr);
    bool isRunning() const;
    bool hasFreshJointState(int maxAgeMs = 1500) const;

public slots:
    void handleMoveJResult(bool ok);
    void handleMoveLOffsetResult(bool ok);
    void abort(const QString &reason = QStringLiteral("操作者取消"));

signals:
    void moveJRequested(const QVector<double> &jointsRad, int speed);
    void moveLOffsetRequested(int toolAxis, double distanceMeters, int speed);
    void moveStopRequested();
    void logMessage(const QString &message);
    void progressChanged(int progress);
    void finished(bool ok, const QString &message);

private:
    enum class Phase
    {
        LeftSweep,
        LeftReturn,
        RightSweep,
        RightReturn,
        LayerTransition
    };

    enum class ExpectedResult { None, MoveJ, MoveLOffset };

    void sendCurrentCommand();
    void completeCurrentCommand(const QString &label);
    void beginNextLayerAfterStateUpdate();
    bool validateLayerJoint6Targets(int layerIndex, QString *errorMessage) const;
    void finishSuccessfully();
    void fail(const QString &message, bool requestStop);
    bool validateStart(const QVector<LayerPlan> &layers,
                       int toolAxis,
                       double toolAxisDirection,
                       QString *errorMessage) const;

    std::array<double, 6> m_latestJoints{};
    QDateTime m_lastJointStateAt;
    std::array<double, 6> m_layerBaselineJoints{};
    QVector<LayerPlan> m_layers;
    QTimer m_resultTimeout;
    QTimer m_jointStateWatchdog;
    QDateTime m_commandSentAt;
    std::uint64_t m_jointStateSequence = 0;
    std::uint64_t m_moveLResultJointStateSequence = 0;
    Phase m_phase = Phase::LeftSweep;
    ExpectedResult m_expectedResult = ExpectedResult::None;
    int m_currentLayer = 0;
    int m_completedCommands = 0;
    int m_totalCommands = 0;
    int m_toolAxis = 2;
    double m_toolAxisDirection = 1.0;
    bool m_hasJointState = false;
    bool m_running = false;
    bool m_waitingForPostMoveJointState = false;
};

#endif // DEMO_ROTATION_EXECUTOR_H
