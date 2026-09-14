#pragma once

#include <QObject>
#include <QThread>
#include "logmessage.h"

class FileSink;

/**
 * LogManager - 全局日志中心（单例）
 *
 * 职责：
 *   1. 接收所有模块的日志请求
 *   2. 生成 LogMessage 并通过信号分发
 *   3. 管理 FileSink 线程的生命周期
 *
 * 线程模型：
 *   - LogManager 自身在主线程
 *   - FileSink 通过 moveToThread 运行在独立日志线程
 *   - LogView 在主线程接收 logReceived 信号更新 UI
 */
class LogManager : public QObject
{
    Q_OBJECT

public:
    static LogManager& instance();

    void init(const QString& logDir);
    void shutdown();

    void log(LogLevel level, const QString& module, const QString& text);
    void log(LogLevel level, const QString& text);

    void debug(const QString& module, const QString& text);
    void debug(const QString& text);
    void info(const QString& module, const QString& text);
    void info(const QString& text);
    void success(const QString& module, const QString& text);
    void success(const QString& text);
    void warning(const QString& module, const QString& text);
    void warning(const QString& text);
    void error(const QString& module, const QString& text);
    void error(const QString& text);
    void fatal(const QString& module, const QString& text);
    void fatal(const QString& text);

signals:
    /** 广播日志消息，LogView 和 FileSink 都通过此信号接收 */
    void logReceived(const LogMessage& message);

    /** 请求 FileSink 初始化（跨线程，QueuedConnection） */
    void requestFileSinkInit(const QString& logDir);

    /** 请求 FileSink 关闭（跨线程，BlockingQueuedConnection） */
    void requestFileSinkClose();

private:
    explicit LogManager(QObject* parent = nullptr);
    ~LogManager();

    LogManager(const LogManager&) = delete;
    LogManager& operator=(const LogManager&) = delete;

private:
    QThread*  m_fileThread = nullptr;
    FileSink* m_fileSink   = nullptr;
    bool      m_initialized = false;
};

// ========= 全局日志宏 =========
// 其他模块只需 #include "logmanager.h" 即可使用

#define LOG_DEBUG(...)   LogManager::instance().debug(__VA_ARGS__)
#define LOG_INFO(...)    LogManager::instance().info(__VA_ARGS__)
#define LOG_SUCCESS(...) LogManager::instance().success(__VA_ARGS__)
#define LOG_WARN(...)    LogManager::instance().warning(__VA_ARGS__)
#define LOG_ERROR(...)   LogManager::instance().error(__VA_ARGS__)
#define LOG_FATAL(...)   LogManager::instance().fatal(__VA_ARGS__)
