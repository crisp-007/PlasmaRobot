#ifndef PLASMA_CONTROLLER_PROTOCOL_H
#define PLASMA_CONTROLLER_PROTOCOL_H

#include <QByteArray>
#include <QString>
#include <QtGlobal>

class PlasmaControllerProtocol
{
public:
    static constexpr int kCommandSize = 16;
    static constexpr int kStateSize = 28;

    struct State
    {
        quint8 emergencyStop = 0;
        quint8 voltageRelay = 0;
        quint16 voltageOutput = 0;
        quint8 heliumRelay = 0;
        quint16 heliumOutput = 0;
        quint8 argonRelay = 0;
        quint16 argonOutput = 0;
        float plasmaFeedbackVpp = 0.0f;
        float heliumPressureMpa = 0.0f;
        float argonPressureMpa = 0.0f;
    };

    static QByteArray handshakeRequest();
    static quint16 flowToCentiLitersPerMinute(double flowLpm, double maxFlowLpm);
    static QByteArray buildFanDutyCommand(bool controlAuxEnabled,
                                          quint16 controlAuxDutyPermille,
                                          bool deviceEnabled,
                                          quint16 deviceDutyPermille,
                                          bool controlMainEnabled,
                                          quint16 controlMainDutyPermille);
    static bool isHandshakeAck(const QByteArray &frame);
    static quint32 checksum(const QByteArray &frame);
    static bool hasValidBaseChecksum(const QByteArray &frame);
    static bool decodeState(const QByteArray &frame, State *state, QString *error = nullptr);

private:
    static quint16 readU16Le(const QByteArray &data, int offset);
    static quint32 readU32Le(const QByteArray &data, int offset);
    static float readFloatLe(const QByteArray &data, int offset);
};

#endif // PLASMA_CONTROLLER_PROTOCOL_H
