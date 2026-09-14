#pragma once

#include <QString>
#include <QDateTime>
#include <QThread>
#include <QMetaType>
#include "logdefs.h"

/**
 * 日志消息结构体
 * 通过 Qt 信号槽跨线程传递，需要 Q_DECLARE_METATYPE
 */
struct LogMessage
{
    QDateTime  time;
    LogLevel   level;
    QString    module;
    QString    text;
    Qt::HANDLE threadId;
};

Q_DECLARE_METATYPE(LogMessage)
