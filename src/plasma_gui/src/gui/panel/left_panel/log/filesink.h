#pragma once

#include <QObject>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include "logmessage.h"

/**
 * FileSink - 文件日志输出端
 *
 * 运行在独立的日志线程中，不阻塞 UI。
 * 职责：
 *   1. 创建日志目录
 *   2. 按日期创建日志文件（yyyy-MM-dd.log）
 *   3. 写入格式化日志文本
 *   4. Error/Fatal 级别立即 flush
 *   5. close() 中 flush 并关闭文件
 */
class FileSink : public QObject
{
    Q_OBJECT

public:
    explicit FileSink(QObject* parent = nullptr);
    ~FileSink();

public slots:
    void init(const QString& logDir);
    void writeLog(const LogMessage& message);
    void close();

private:
    QString makeLogFilePath(const QString& logDir) const;
    QString formatMessage(const LogMessage& message) const;

private:
    QFile       m_file;
    QString     m_logDir;
    QTextStream m_stream;
};
