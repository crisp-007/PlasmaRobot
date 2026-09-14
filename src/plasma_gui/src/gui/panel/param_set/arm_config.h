#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H

#include <QString>

struct ArmConfig
{
    QString armType;
    QString controllerIp;
    int tcpPort = 0;
    int dof = 0;
    QString udpIp;
    int udpPort = 0;
    int udpCycleMs = 0;
    QString sourcePath;
    QString error;
    bool valid = false;

    QString displayModel() const;

    static ArmConfig loadDefault();
    static ArmConfig loadFromFile(const QString &filePath);
};

#endif // ARM_CONFIG_H
