#include "plasma_controller_protocol.h"

#include <QtEndian>

#include <cmath>
#include <cstring>

namespace {

const QByteArray kHandshakeRequest = QByteArray::fromHex(
    "FFFEC5A5A55A3C3CC396966936070000");
const QByteArray kHandshakeAck = QByteArray::fromHex(
    "FFFECaa5a55a3c3cc39696693b070000");

} // namespace

QByteArray PlasmaControllerProtocol::handshakeRequest()
{
    return kHandshakeRequest;
}

quint16 PlasmaControllerProtocol::flowToCentiLitersPerMinute(double flowLpm,
                                                              double maxFlowLpm)
{
    if (!std::isfinite(flowLpm) || !std::isfinite(maxFlowLpm) || maxFlowLpm <= 0.0) {
        return 0;
    }

    const double boundedFlow = qBound(0.0, flowLpm, maxFlowLpm);
    return static_cast<quint16>(std::lround(boundedFlow * 100.0));
}

QByteArray PlasmaControllerProtocol::buildFanDutyCommand(
    bool controlAuxEnabled,
    quint16 controlAuxDutyPermille,
    bool deviceEnabled,
    quint16 deviceDutyPermille,
    bool controlMainEnabled,
    quint16 controlMainDutyPermille)
{
    QByteArray frame(kCommandSize, 0);
    frame[0] = static_cast<char>(0xFF);
    frame[1] = static_cast<char>(0xFE);
    frame[2] = static_cast<char>(0xA5);

    quint8 fanBits = 0;
    if (controlAuxEnabled) {
        fanBits |= 0x20;
    }
    if (deviceEnabled) {
        fanBits |= 0x40;
    }
    if (controlMainEnabled) {
        fanBits |= 0x80;
    }
    frame[3] = static_cast<char>(fanBits);

    qToLittleEndian(qMin<quint16>(controlAuxDutyPermille, 1000),
                    reinterpret_cast<uchar *>(frame.data() + 4));
    qToLittleEndian(qMin<quint16>(deviceDutyPermille, 1000),
                    reinterpret_cast<uchar *>(frame.data() + 7));
    qToLittleEndian(qMin<quint16>(controlMainDutyPermille, 1000),
                    reinterpret_cast<uchar *>(frame.data() + 10));
    qToLittleEndian(checksum(frame),
                    reinterpret_cast<uchar *>(frame.data() + 12));
    return frame;
}

bool PlasmaControllerProtocol::isHandshakeAck(const QByteArray &frame)
{
    return frame == kHandshakeAck && hasValidBaseChecksum(frame);
}

quint32 PlasmaControllerProtocol::checksum(const QByteArray &frame)
{
    if (frame.size() < 12) {
        return 0;
    }
    quint32 value = 0;
    for (int i = 0; i < 12; ++i) {
        value += static_cast<quint8>(frame.at(i));
    }
    return value;
}

bool PlasmaControllerProtocol::hasValidBaseChecksum(const QByteArray &frame)
{
    return frame.size() >= kCommandSize
        && static_cast<quint8>(frame.at(0)) == 0xFF
        && static_cast<quint8>(frame.at(1)) == 0xFE
        && readU32Le(frame, 12) == checksum(frame);
}

bool PlasmaControllerProtocol::decodeState(const QByteArray &frame,
                                            State *state,
                                            QString *error)
{
    const auto fail = [error](const QString &message) {
        if (error) {
            *error = message;
        }
        return false;
    };

    if (!state) {
        return fail(QStringLiteral("状态输出指针为空"));
    }
    if (frame.size() != kStateSize) {
        return fail(QStringLiteral("状态包长度应为 28 字节，实际为 %1").arg(frame.size()));
    }
    if (!hasValidBaseChecksum(frame)) {
        return fail(QStringLiteral("状态包头或校验和错误"));
    }

    State decoded;
    decoded.emergencyStop = static_cast<quint8>(frame.at(2));
    decoded.voltageRelay = static_cast<quint8>(frame.at(3));
    decoded.voltageOutput = readU16Le(frame, 4);
    decoded.heliumRelay = static_cast<quint8>(frame.at(6));
    decoded.heliumOutput = readU16Le(frame, 7);
    decoded.argonRelay = static_cast<quint8>(frame.at(9));
    decoded.argonOutput = readU16Le(frame, 10);
    decoded.plasmaFeedbackVpp = readFloatLe(frame, 16);
    decoded.heliumPressureMpa = readFloatLe(frame, 20);
    decoded.argonPressureMpa = readFloatLe(frame, 24);
    if (!std::isfinite(decoded.plasmaFeedbackVpp)
        || !std::isfinite(decoded.heliumPressureMpa)
        || !std::isfinite(decoded.argonPressureMpa)) {
        return fail(QStringLiteral("状态包包含非有限浮点数"));
    }
    *state = decoded;
    return true;
}

quint16 PlasmaControllerProtocol::readU16Le(const QByteArray &data, int offset)
{
    return qFromLittleEndian<quint16>(
        reinterpret_cast<const uchar *>(data.constData() + offset));
}

quint32 PlasmaControllerProtocol::readU32Le(const QByteArray &data, int offset)
{
    return qFromLittleEndian<quint32>(
        reinterpret_cast<const uchar *>(data.constData() + offset));
}

float PlasmaControllerProtocol::readFloatLe(const QByteArray &data, int offset)
{
    const quint32 bits = readU32Le(data, offset);
    float value = 0.0f;
    static_assert(sizeof(value) == sizeof(bits));
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}
