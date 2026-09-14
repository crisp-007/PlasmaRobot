#include "state_btn.h"

#include <QPainter>
#include <QStyleOptionButton>
#include <QStyle>
#include <algorithm>

// 圆点绘制参数（由 UI 布局固定按钮尺寸，无需外部设置）
static constexpr int kDotRadius = 3;   // 状态圆点半径
static constexpr int kDotMargin = 1;   // 圆点距按钮右下角边距

// ─────────────────────────────────────────────
// 构造 / 析构
// ─────────────────────────────────────────────

StateBtn::StateBtn(QWidget *parent)
    : QPushButton(parent)
{
    setText(QString());

    connect(&blink_timer_, &QTimer::timeout, this, &StateBtn::onBlinkTick);

    updateBlinkTimer();
}

// ─────────────────────────────────────────────
// 公共接口
// ─────────────────────────────────────────────

void StateBtn::setStatus(Status status, bool clickable)
{
    if (status_ == status && isEnabled() == clickable)
        return;

    status_      = status;
    dot_visible_ = true;
    setEnabled(clickable);
    updateBlinkTimer();
    update();
}

void StateBtn::setBlinkInterval(int ms)
{
    blink_interval_ms_ = std::max(100, ms);
    if (blink_timer_.isActive())
        blink_timer_.setInterval(blink_interval_ms_);
}


void StateBtn::setIconPath(const QString &path)
{
    if (path.isEmpty()) {
        icon_pixmap_ = QPixmap();
    } else {
        QPixmap px(path);
        if (px.isNull()) {
            qWarning("StateBtn: failed to load icon from '%s', using placeholder.",
                     qPrintable(path));
            icon_pixmap_ = QPixmap();
        } else {
            icon_pixmap_ = px;
        }
    }
    update();
}

QSize StateBtn::sizeHint() const
{
    QSize ms = minimumSize();
    if (ms.width() > 0 && ms.height() > 0)
        return ms;
    return QSize(26, 26);
}

QSize StateBtn::minimumSizeHint() const
{
    QSize ms = minimumSize();
    if (ms.width() > 0 && ms.height() > 0)
        return ms;
    return QSize(26, 26);
}

// ─────────────────────────────────────────────
// 绘制入口
// ─────────────────────────────────────────────

void StateBtn::paintEvent(QPaintEvent *event)
{
    const QRect btn_rect = rect();
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // ── 1. 样式引擎绘制背景（处理 hover / pressed / checked）──
    {
        QStyleOptionButton opt;
        initStyleOption(&opt);
        opt.rect = btn_rect;
        style()->drawControl(QStyle::CE_PushButton, &opt, &p, this);
    }

    // ── 2. 叠加绘制图标（居中）──
    if (!icon_pixmap_.isNull()) {
        const QPixmap scaled = icon_pixmap_.scaled(
            btn_rect.size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
        const int x = (btn_rect.width()  - scaled.width())  / 2;
        const int y = (btn_rect.height() - scaled.height()) / 2;
        p.drawPixmap(x, y, scaled);
    }

    // ── 3. 叠加绘制状态圆点（右下角）──
    if (!dot_visible_ && status_ != Connected)
        return;

    const QPointF dot_center(
        btn_rect.width()  - kDotMargin - kDotRadius,
        btn_rect.height() - kDotMargin - kDotRadius
    );
    // 外圈光晕
    p.setPen(Qt::NoPen);
    QColor glow = dotColor();
    glow.setAlpha(60);
    p.setBrush(glow);
    p.drawEllipse(dot_center, kDotRadius + 2.0, kDotRadius + 2.0);
    // 圆点本体
    p.setBrush(dotColor());
    p.setPen(QPen(Qt::black, 0.5));
    p.drawEllipse(dot_center, static_cast<qreal>(kDotRadius), static_cast<qreal>(kDotRadius));
}

// ─────────────────────────────────────────────
// 私有方法
// ─────────────────────────────────────────────

QColor StateBtn::dotColor() const
{
    switch (status_) {
    case Connected:    return QColor(0x00, 0xe6, 0x76);   // 绿
    case Error:        return QColor(0xff, 0xc1, 0x07);   // 黄
    case Disconnected: // fall-through
    default:           return QColor(0xff, 0x44, 0x44);   // 红
    }
}

void StateBtn::updateBlinkTimer()
{
    const bool needs_blink = (status_ == Disconnected || status_ == Error);

    if (needs_blink) {
        blink_timer_.setInterval(blink_interval_ms_);
        if (!blink_timer_.isActive())
            blink_timer_.start();
    } else {
        blink_timer_.stop();
        dot_visible_ = true;
    }
}

// ─────────────────────────────────────────────
// 槽
// ─────────────────────────────────────────────

void StateBtn::onBlinkTick()
{
    dot_visible_ = !dot_visible_;
    update();
}
