#include "filesink.h"
#include <QDir>
#include <QDateTime>
#include <QTextStream>

FileSink::FileSink(QObject* parent)
    : QObject(parent)
    , m_stream(&m_file)
{
}

FileSink::~FileSink()
{
    close();
}

void FileSink::init(const QString& logDir)
{
    m_logDir = logDir;

    QDir dir(m_logDir);
    if (!dir.exists())
    {
        dir.mkpath(".");
    }

    QString filePath = makeLogFilePath(m_logDir);

    m_file.setFileName(filePath);
    if (m_file.open(QIODevice::Append | QIODevice::Text))
    {
        m_stream.setDevice(&m_file);
        // Qt6 默认 UTF-8 编码，无需 setCodec
    }
}

void FileSink::writeLog(const LogMessage& message)
{
    if (!m_file.isOpen())
        return;

    m_stream << formatMessage(message) << "\n";

    // Error 和 Fatal 立即 flush，防止丢失
    if (message.level == LogLevel::Error ||
        message.level == LogLevel::Fatal)
    {
        m_stream.flush();
    }
}

void FileSink::close()
{
    if (m_file.isOpen())
    {
        m_stream.flush();
        m_file.close();
    }
}

QString FileSink::makeLogFilePath(const QString& logDir) const
{
    QString date = QDateTime::currentDateTime().toString("yyyy-MM-dd");
    return QString("%1/%2.log").arg(logDir, date);
}

QString FileSink::formatMessage(const LogMessage& message) const
{
    return QString("[%1] [%2] [%3] [tid:%4] %5")
        .arg(message.time.toString("yyyy-MM-dd hh:mm:ss.zzz"))
        .arg(logLevelToString(message.level))
        .arg(message.module)
        .arg(reinterpret_cast<quintptr>(message.threadId), 0, 16)
        .arg(message.text);
}
