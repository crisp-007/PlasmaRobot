#pragma once

#include <QObject>
#include <QPointer>
#include "logmessage.h"

class QPlainTextEdit;

/**
 * LogView - 日志界面控制器
 *
 * 接管 MainWindow.ui 中的 QPlainTextEdit 控件（objectName: log），
 * 负责将 LogMessage 以彩色 HTML 格式追加显示。
 *
 * 约束：
 *   - 必须在 UI 线程中运行
 *   - 不继承 QWidget，由 MainWindow 持有
 *   - 限制最大显示行数（1000行）防止内存增长
 */
class LogView : public QObject
{
    Q_OBJECT

public:
    explicit LogView(QPlainTextEdit* textEdit, QObject* parent = nullptr);

public slots:
    void appendLog(const LogMessage& message);
    void clear();

private:
    QPointer<QPlainTextEdit> m_textEdit;
};
