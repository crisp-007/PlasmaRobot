#include "logview.h"
#include "logdefs.h"
#include <QPlainTextEdit>
#include <QScrollBar>
#include <QTextCursor>

LogView::LogView(QPlainTextEdit* textEdit, QObject* parent)
    : QObject(parent)
    , m_textEdit(textEdit)
{
    if (m_textEdit)
    {
        m_textEdit->setReadOnly(true);
        m_textEdit->document()->setMaximumBlockCount(1000);

        // 暗色主题样式
        m_textEdit->setStyleSheet(
            "QPlainTextEdit#log {"
            "    background-color: #1e1e1e;"
            "    color: #dcdcdc;"
            "    border: 1px solid #444444;"
            "    border-radius: 6px;"
            "    padding: 6px;"
            "    font-size: 12px;"
            "    font-family: 'Consolas', 'Source Code Pro', 'Microsoft YaHei', monospace;"
            "}"
            "QPlainTextEdit#log QScrollBar:vertical {"
            "    background: #2a2a2a;"
            "    width: 10px;"
            "    margin: 0px;"
            "}"
            "QPlainTextEdit#log QScrollBar::handle:vertical {"
            "    background: #555555;"
            "    border-radius: 5px;"
            "}"
            "QPlainTextEdit#log QScrollBar::handle:vertical:hover {"
            "    background: #777777;"
            "}"
            "QPlainTextEdit#log QScrollBar::add-line:vertical,"
            "QPlainTextEdit#log QScrollBar::sub-line:vertical {"
            "    height: 0px;"
            "}"
        );
    }
}

void LogView::appendLog(const LogMessage& message)
{
    if (!m_textEdit)
        return;

    QString time      = message.time.toString("hh:mm:ss");
    QString color     = logLevelToColor(message.level);

    // Info 和 Warning 等级不显示 level 标签
    QString levelText;
    if (message.level != LogLevel::Info && message.level != LogLevel::Warning)
    {
        levelText = logLevelToUiString(message.level);
    }

    // 逐段拼接：[time] [level?] [module?] text
    QString html = QString("<span style='color:%1;'>[%2]</span>")
                       .arg(color, time);

    if (!levelText.isEmpty())
    {
        html += QString(" <span style='color:%1;'>[%2]</span>")
                     .arg(color, levelText);
    }

    if (!message.module.isEmpty())
    {
        html += QString(" <span style='color:%1;'>[%2]</span>")
                     .arg(color, message.module.toHtmlEscaped());
    }

    html += " " + message.text.toHtmlEscaped();

    // QPlainTextEdit 使用 appendHtml 追加富文本
    m_textEdit->appendHtml(html);

    // 自动滚动到底部
    if (m_textEdit->verticalScrollBar())
    {
        QScrollBar* bar = m_textEdit->verticalScrollBar();
        bar->setValue(bar->maximum());
    }
}

void LogView::clear()
{
    if (m_textEdit)
    {
        m_textEdit->clear();
    }
}
