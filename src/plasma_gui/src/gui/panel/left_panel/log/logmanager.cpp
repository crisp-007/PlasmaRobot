#include "logmanager.h"
#include "filesink.h"
#include <QCoreApplication>

LogManager::LogManager(QObject* parent)
    : QObject(parent)
{
    qRegisterMetaType<LogMessage>("LogMessage");
}

LogManager::~LogManager()
{
    shutdown();
}

LogManager& LogManager::instance()
{
    static LogManager manager;
    return manager;
}

void LogManager::init(const QString& logDir)
{
    if (m_initialized)
        return;

    // 创建日志线程
    m_fileThread = new QThread();
    m_fileSink = new FileSink();
    m_fileSink->moveToThread(m_fileThread);

    // 线程结束时自动删除 FileSink
    connect(m_fileThread, &QThread::finished,
            m_fileSink, &QObject::deleteLater);

    // 日志分发 → FileSink 写入（跨线程排队）
    connect(this, &LogManager::logReceived,
            m_fileSink, &FileSink::writeLog,
            Qt::QueuedConnection);

    // 初始化请求（跨线程排队）
    connect(this, &LogManager::requestFileSinkInit,
            m_fileSink, &FileSink::init,
            Qt::QueuedConnection);

    // 关闭请求（跨线程阻塞，确保文件写完）
    connect(this, &LogManager::requestFileSinkClose,
            m_fileSink, &FileSink::close,
            Qt::BlockingQueuedConnection);

    m_fileThread->start();

    // 发送初始化请求，使用绝对路径避免工作目录变化
    QString absoluteDir = QCoreApplication::applicationDirPath() + "/" + logDir;
    emit requestFileSinkInit(absoluteDir);

    m_initialized = true;
}

void LogManager::shutdown()
{
    if (!m_initialized)
        return;

    // 1. 通知 FileSink 关闭文件（阻塞等待完成）
    if (m_fileSink)
    {
        emit requestFileSinkClose();
    }

    // 2. 停止线程
    if (m_fileThread)
    {
        m_fileThread->quit();
        m_fileThread->wait();
        delete m_fileThread;
        m_fileThread = nullptr;
    }

    m_fileSink = nullptr;
    m_initialized = false;
}

void LogManager::log(LogLevel level, const QString& module, const QString& text)
{
    LogMessage message;
    message.time     = QDateTime::currentDateTime();
    message.level    = level;
    message.module   = module;
    message.text     = text;
    message.threadId = QThread::currentThreadId();

    emit logReceived(message);
}

void LogManager::debug(const QString& module, const QString& text)
{
    log(LogLevel::Debug, module, text);
}

void LogManager::info(const QString& module, const QString& text)
{
    log(LogLevel::Info, module, text);
}

void LogManager::success(const QString& module, const QString& text)
{
    log(LogLevel::Success, module, text);
}

void LogManager::warning(const QString& module, const QString& text)
{
    log(LogLevel::Warning, module, text);
}

void LogManager::error(const QString& module, const QString& text)
{
    log(LogLevel::Error, module, text);
}

void LogManager::fatal(const QString& module, const QString& text)
{
    log(LogLevel::Fatal, module, text);
}

// ========= 无模块名重载 =========

void LogManager::log(LogLevel level, const QString& text)
{
    log(level, QString(), text);
}

void LogManager::debug(const QString& text)   { log(LogLevel::Debug,   text); }
void LogManager::info(const QString& text)    { log(LogLevel::Info,    text); }
void LogManager::success(const QString& text) { log(LogLevel::Success, text); }
void LogManager::warning(const QString& text) { log(LogLevel::Warning, text); }
void LogManager::error(const QString& text)   { log(LogLevel::Error,   text); }
void LogManager::fatal(const QString& text)   { log(LogLevel::Fatal,   text); }
