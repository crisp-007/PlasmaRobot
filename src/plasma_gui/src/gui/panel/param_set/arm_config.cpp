#include "arm_config.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <QDir>
#include <QFileInfo>
#include <QStringList>

#include <exception>

QString ArmConfig::displayModel() const
{
    if (armType.compare(QStringLiteral("RM_eco65"), Qt::CaseInsensitive) == 0)
        return QStringLiteral("RealMan ECO65");
    if (armType.compare(QStringLiteral("RM_65"), Qt::CaseInsensitive) == 0)
        return QStringLiteral("RealMan RM65");
    return armType.isEmpty() ? QStringLiteral("机械臂") : armType;
}

ArmConfig ArmConfig::loadDefault()
{
    QStringList candidates;

    const QString overridePath = QString::fromLocal8Bit(qgetenv("PLASMA_ARM_CONFIG"));
    if (!overridePath.isEmpty())
        candidates.append(overridePath);

    try {
        const QString sharePath = QString::fromStdString(
            ament_index_cpp::get_package_share_directory("rm_driver"));
        candidates.append(QDir(sharePath).filePath(
            QStringLiteral("config/rm_eco65_config.yaml")));
    } catch (const std::exception &) {
        // The source-tree fallback below also supports launching from Qt Creator.
    }

#ifdef PLASMA_GUI_SOURCE_DIR
    candidates.append(QDir(QStringLiteral(PLASMA_GUI_SOURCE_DIR)).filePath(
        QStringLiteral("../plasma_robot/rm_driver/config/rm_eco65_config.yaml")));
#endif

    QStringList attemptedPaths;
    for (const QString &candidate : candidates) {
        const QString absolutePath = QFileInfo(candidate).absoluteFilePath();
        if (attemptedPaths.contains(absolutePath))
            continue;
        attemptedPaths.append(absolutePath);
        if (!QFileInfo::exists(absolutePath))
            continue;

        ArmConfig config = loadFromFile(absolutePath);
        if (config.valid)
            return config;
        return config;
    }

    ArmConfig config;
    config.error = QStringLiteral("未找到机械臂配置文件。已检查: %1")
        .arg(attemptedPaths.isEmpty()
            ? QStringLiteral("无可用路径")
            : attemptedPaths.join(QStringLiteral(", ")));
    return config;
}

ArmConfig ArmConfig::loadFromFile(const QString &filePath)
{
    ArmConfig config;
    config.sourcePath = QFileInfo(filePath).absoluteFilePath();

    try {
        const YAML::Node root = YAML::LoadFile(config.sourcePath.toStdString());
        const YAML::Node params = root["rm_driver"]["ros__parameters"];
        if (!params || !params.IsMap()) {
            config.error = QStringLiteral("机械臂配置缺少 rm_driver.ros__parameters");
            return config;
        }

        config.armType = QString::fromStdString(params["arm_type"].as<std::string>());
        config.controllerIp = QString::fromStdString(params["arm_ip"].as<std::string>());
        config.tcpPort = params["tcp_port"].as<int>();
        config.dof = params["arm_dof"].as<int>();
        config.udpIp = QString::fromStdString(params["udp_ip"].as<std::string>());
        config.udpPort = params["udp_port"].as<int>();
        config.udpCycleMs = params["udp_cycle"].as<int>();

        if (config.armType.isEmpty() || config.controllerIp.isEmpty()
            || config.tcpPort <= 0 || config.tcpPort > 65535
            || config.dof <= 0) {
            config.error = QStringLiteral("机械臂配置字段无效: %1")
                .arg(config.sourcePath);
            return config;
        }

        config.valid = true;
    } catch (const YAML::Exception &error) {
        config.error = QStringLiteral("解析机械臂配置失败: %1")
            .arg(QString::fromLocal8Bit(error.what()));
    }

    return config;
}
