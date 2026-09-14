#pragma once

#include <QString>

/**
 * 日志等级定义
 */
enum class LogLevel
{
    Debug,
    Info,
    Success,
    Warning,
    Error,
    Fatal
};

/**
 * 日志等级 → 字符串（文件日志用，全大写）
 */
inline QString logLevelToString(LogLevel level)
{
    switch (level)
    {
    case LogLevel::Debug:   return "DEBUG";
    case LogLevel::Info:    return "INFO";
    case LogLevel::Success: return "SUCCESS";
    case LogLevel::Warning: return "WARNING";
    case LogLevel::Error:   return "ERROR";
    case LogLevel::Fatal:   return "FATAL";
    }
    return "UNKNOWN";
}

/**
 * 日志等级 → UI 显示字符串（简短）
 */
inline QString logLevelToUiString(LogLevel level)
{
    switch (level)
    {
    case LogLevel::Debug:   return "DEBUG";
    case LogLevel::Info:    return "INFO";
    case LogLevel::Success: return "SUCCESS";
    case LogLevel::Warning: return "WARN";
    case LogLevel::Error:   return "ERROR";
    case LogLevel::Fatal:   return "FATAL";
    }
    return "UNKNOWN";
}

/**
 * 日志等级 → UI 颜色（十六进制）
 */
inline QString logLevelToColor(LogLevel level)
{
    switch (level)
    {
    case LogLevel::Debug:   return "#9CA3AF";
    case LogLevel::Info:    return "#D1D5DB";
    case LogLevel::Success: return "#22C55E";
    case LogLevel::Warning: return "#FACC15";
    case LogLevel::Error:   return "#EF4444";
    case LogLevel::Fatal:   return "#FF0000";
    }
    return "#D1D5DB";
}
