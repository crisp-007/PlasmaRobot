#ifndef STATE_BTN_H
#define STATE_BTN_H

#include <QPushButton>
#include <QTimer>
#include <QPixmap>
#include <QString>

/**
 * @brief 带状态指示圆点的图标按钮
 *
 * 按钮本身就是一个普通 QPushButton（支持 stylesheet hover/pressed），
 * 在其上方叠加绘制：
 *   1. setIconPath() 加载的图标（居中）
 *   2. 状态圆点（右下角）
 *
 * Ready/NotReady 语义：
 *   Ready    — 绿色圆点常亮，按钮可点击
 *   NotReady — 红色圆点闪烁，按钮变灰不可点击
 */
class StateBtn : public QPushButton
{
    Q_OBJECT

public:
    enum Status {
        Disconnected,  ///< 未连接，红色闪烁
        Connected,     ///< 已连接，绿色常亮
        Error          ///< 错误，黄色闪烁
    };
    Q_ENUM(Status)

    explicit StateBtn(QWidget *parent = nullptr);
    ~StateBtn() override = default;

    // ── 状态接口 ──────────────────────────────
    void setStatus(Status status, bool clickable = true);
    Status status() const { return status_; }

    void setBlinkInterval(int ms);
    int blinkInterval() const { return blink_interval_ms_; }

    // ── 图标接口 ────────────────────────────────
    void setIconPath(const QString &path);

    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

protected:
    void paintEvent(QPaintEvent *event) override;

private slots:
    void onBlinkTick();

private:
    // ── 状态 ────────────────────────────────────
    Status  status_            = Disconnected;
    int     blink_interval_ms_ = 500;
    bool    dot_visible_       = true;
    QTimer  blink_timer_;
    QPixmap icon_pixmap_;

    /** 根据当前状态返回圆点颜色 */
    QColor dotColor() const;

    /** 启动或停止闪烁定时器 */
    void updateBlinkTimer();
};

#endif // STATE_BTN_H
