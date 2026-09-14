#include "plasma_controller_protocol.h"

#include <QtEndian>

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

namespace {

int failures = 0;

void expect(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        ++failures;
    }
}

void writeU16Le(QByteArray &data, int offset, quint16 value)
{
    qToLittleEndian(value, reinterpret_cast<uchar *>(data.data() + offset));
}

void writeU32Le(QByteArray &data, int offset, quint32 value)
{
    qToLittleEndian(value, reinterpret_cast<uchar *>(data.data() + offset));
}

void writeFloatLe(QByteArray &data, int offset, float value)
{
    quint32 bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    writeU32Le(data, offset, bits);
}

bool near(float left, float right)
{
    return std::fabs(left - right) < 1e-6f;
}

} // namespace

int main()
{
    const QByteArray handshake = PlasmaControllerProtocol::handshakeRequest();
    expect(handshake.toHex(' ').toUpper()
               == QByteArray("FF FE C5 A5 A5 5A 3C 3C C3 96 96 69 36 07 00 00"),
           "handshake request must match the controller documentation");
    expect(PlasmaControllerProtocol::isHandshakeAck(QByteArray::fromHex(
               "FFFECaa5a55a3c3cc39696693b070000")),
           "documented handshake ACK should be accepted");

    expect(PlasmaControllerProtocol::flowToCentiLitersPerMinute(4.0, 30.0) == 400,
           "4.00 L/min must be encoded as 400 centi-L/min");
    expect(PlasmaControllerProtocol::flowToCentiLitersPerMinute(-1.0, 30.0) == 0,
           "negative flow must be clamped to zero");
    expect(PlasmaControllerProtocol::flowToCentiLitersPerMinute(31.0, 30.0) == 3000,
           "helium flow must be clamped to the configured 30 L/min maximum");
    expect(PlasmaControllerProtocol::flowToCentiLitersPerMinute(
               std::numeric_limits<double>::quiet_NaN(), 30.0) == 0,
           "non-finite flow must fail closed");

    const QByteArray fansAtFiftyPercent =
        PlasmaControllerProtocol::buildFanDutyCommand(true, 500,
                                                       true, 500,
                                                       true, 500);
    expect(fansAtFiftyPercent.toHex(' ').toUpper()
               == QByteArray("FF FE A5 E0 F4 01 00 F4 01 00 F4 01 61 06 00 00"),
           "three enabled fans at 50 percent must match the controller protocol");
    expect(PlasmaControllerProtocol::hasValidBaseChecksum(fansAtFiftyPercent),
           "fan duty command must have a valid checksum");

    QByteArray packet(PlasmaControllerProtocol::kStateSize, 0);
    packet[0] = static_cast<char>(0xFF);
    packet[1] = static_cast<char>(0xFE);
    packet[2] = static_cast<char>(0x00);
    packet[3] = static_cast<char>(0x11);
    writeU16Le(packet, 4, 220);
    packet[6] = static_cast<char>(0x11);
    writeU16Le(packet, 7, 825);
    packet[9] = static_cast<char>(0x01);
    writeU16Le(packet, 10, 650);
    writeU32Le(packet, 12, PlasmaControllerProtocol::checksum(packet));
    writeFloatLe(packet, 16, 12.5f);
    writeFloatLe(packet, 20, 0.25f);
    writeFloatLe(packet, 24, 0.40f);

    PlasmaControllerProtocol::State state;
    QString error;
    expect(PlasmaControllerProtocol::decodeState(packet, &state, &error),
           "valid 28-byte state packet should decode");
    expect(state.voltageRelay == 0x11 && state.voltageOutput == 220,
           "voltage fields should use documented offsets");
    expect(state.heliumRelay == 0x11 && state.heliumOutput == 825,
           "helium fields should use documented offsets");
    expect(state.argonRelay == 0x01 && state.argonOutput == 650,
           "argon fields should use documented offsets");
    expect(near(state.plasmaFeedbackVpp, 12.5f), "plasma feedback float should decode");
    expect(near(state.heliumPressureMpa, 0.25f), "helium MPa float should decode");
    expect(near(state.argonPressureMpa, 0.40f), "argon MPa float should decode");

    writeFloatLe(packet, 20, std::numeric_limits<float>::quiet_NaN());
    expect(!PlasmaControllerProtocol::decodeState(packet, &state, &error),
           "non-finite sensor values must be rejected");
    writeFloatLe(packet, 20, 0.25f);

    packet[12] = static_cast<char>(packet.at(12) ^ 0x01);
    expect(!PlasmaControllerProtocol::decodeState(packet, &state, &error),
           "bad base checksum must be rejected");

    return failures == 0 ? 0 : 1;
}
