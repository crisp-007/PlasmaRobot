#include "tip_widget.h"
#include <QPaintEvent>
#include <QApplication>
#include <QScreen>
#include <QFontMetrics>
#include <QDebug>
#include <QParallelAnimationGroup>

TipWidget::TipWidget(QWidget *parent)
    : QWidget(parent)
    , m_text("")
    , m_status(Succeed)
    , m_duration(3000)
    , m_maxHeight(30)
    , m_animParam(0)
    , m_animY(0)
    , m_showAnimation(nullptr)
    , m_hideAnimation(nullptr)
    , m_autoHideTimer(nullptr)
    , m_borderRadius(8)
    , m_iconSize(20)
    , m_padding(15)
{
    updateColors();
    initUI();
    initAnimation();
    
    // 安装事件过滤器来监听父窗口的移动事件
    if (parent) {
        parent->installEventFilter(this);
    }
}

TipWidget::~TipWidget()
{
    // 移除事件过滤器，避免父窗口持有引用导致无法销毁
    if (parentWidget()) {
        parentWidget()->removeEventFilter(this);
    }
    
    if (m_showAnimation) {
        m_showAnimation->stop();
        delete m_showAnimation;
    }
    if (m_hideAnimation) {
        m_hideAnimation->stop();
        delete m_hideAnimation;
    }
    if (m_autoHideTimer) {
        m_autoHideTimer->stop();
        delete m_autoHideTimer;
    }
}

void TipWidget::initUI()
{
    setWindowFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint | Qt::Tool);
    setAttribute(Qt::WA_TranslucentBackground);
    setAttribute(Qt::WA_ShowWithoutActivating);
    
    // 设置初始大小和位置
    setFixedSize(300, 0);
    
    // 相对于父窗口居中显示
    if (parentWidget()) {
        QWidget *parent = parentWidget();
        // 使用 mapToGlobal 获取父窗口在屏幕上的实际位置
        QPoint parentTopLeft = parent->mapToGlobal(QPoint(0, 0));
        QSize parentSize = parent->size();
        
        int x = parentTopLeft.x() + (parentSize.width() - width()) / 2;
        int y = parentTopLeft.y() + 50; // 距离父窗口顶部50像素
        move(x, y);
    } else {
        // 如果没有父窗口，则相对于屏幕居中
        QScreen *screen = QApplication::primaryScreen();
        QRect screenGeometry = screen->geometry();
        int x = (screenGeometry.width() - width()) / 2;
        int y = 50; // 距离顶部50像素
        move(x, y);
    }
}

void TipWidget::initAnimation()
{
    // 显示动画
    m_showAnimation = new QPropertyAnimation(this, "geometry", this);
    m_showAnimation->setDuration(120);
    m_showAnimation->setEasingCurve(QEasingCurve::InOutSine);
    connect(m_showAnimation, &QPropertyAnimation::finished, this, &TipWidget::onShowAnimFinished);
    
    // 隐藏动画 - 改为并行动画组（位置+透明度）
    m_hideAnimation = new QParallelAnimationGroup(this);
    
    // 位置动画（向上移动）
    QPropertyAnimation *posAnim = new QPropertyAnimation(this, "pos", this);
    posAnim->setDuration(400);
    posAnim->setEasingCurve(QEasingCurve::OutCubic);
    
    // 透明度动画（淡出）
    QPropertyAnimation *opacityAnim = new QPropertyAnimation(this, "opacity", this);
    opacityAnim->setDuration(400);
    opacityAnim->setEasingCurve(QEasingCurve::OutCubic);
    opacityAnim->setStartValue(1.0);
    opacityAnim->setEndValue(0.0);
    
    m_hideAnimation->addAnimation(posAnim);
    m_hideAnimation->addAnimation(opacityAnim);
    
    // 自动隐藏定时器
    m_autoHideTimer = new QTimer(this);
    m_autoHideTimer->setSingleShot(true);
    connect(m_autoHideTimer, &QTimer::timeout, this, &TipWidget::onAutoHide);
}

void TipWidget::setParams(const QString &text, TipsStatus status, int duration)
{
    m_text = text;
    m_status = status;
    m_duration = duration;
    
    // 更新颜色
    updateColors();
    
    // 根据文本长度调整宽度
    QFontMetrics fm(font());
    int textWidth = fm.horizontalAdvance(m_text);
    int minWidth = textWidth + m_iconSize + m_padding * 3;
    setFixedWidth(qMax(300, minWidth));
    
    // 重新居中
    if (parentWidget()) {
        QWidget *parent = parentWidget();
        // 使用 mapToGlobal 获取父窗口在屏幕上的实际位置
        QPoint parentTopLeft = parent->mapToGlobal(QPoint(0, 0));
        QSize parentSize = parent->size();
        
        int x = parentTopLeft.x() + (parentSize.width() - width()) / 2;
        int y = parentTopLeft.y() + 50; // 距离父窗口顶部50像素
        move(x, y);
    } else {
        // 如果没有父窗口，则相对于屏幕居中
        QScreen *screen = QApplication::primaryScreen();
        QRect screenGeometry = screen->geometry();
        int x = (screenGeometry.width() - width()) / 2;
        int y = 50; // 距离顶部50像素
        move(x, y);
    }
    
    update();
}

void TipWidget::showTip()
{
    // 直接设置高度并显示
    setFixedHeight(m_maxHeight);
    show();
    
    // 注意：不再启动自动隐藏定时器，由TipManager统一管理
    // 这样可以确保窗口按照创建顺序（FIFO）依次淡出
}

void TipWidget::hideTip()
{
    if (m_autoHideTimer->isActive()) {
        m_autoHideTimer->stop();
    }
    
    // 如果动画正在运行，先停止
    if (m_hideAnimation->state() == QAbstractAnimation::Running) {
        m_hideAnimation->stop();
    }
    
    // 断开之前的连接，避免重复连接
    disconnect(m_hideAnimation, &QAbstractAnimation::finished, this, nullptr);
    
    // 获取位置动画
    QParallelAnimationGroup *hideGroup = qobject_cast<QParallelAnimationGroup*>(m_hideAnimation);
    if (hideGroup && hideGroup->animationCount() >= 2) {
        QPropertyAnimation *posAnim = qobject_cast<QPropertyAnimation*>(hideGroup->animationAt(0));
        if (posAnim) {
            // 设置向上移动的目标位置
            QPoint currentPos = pos();
            QPoint targetPos = currentPos;
            if (parentWidget()) {
                QPoint parentTopLeft = parentWidget()->mapToGlobal(QPoint(0, 0));
                targetPos.setY(parentTopLeft.y()); // 移动到父窗口顶部
            } else {
                targetPos.setY(0); // 移动到屏幕顶部
            }
            
            posAnim->setStartValue(currentPos);
            posAnim->setEndValue(targetPos);
        }
    }
    
    // 连接动画完成信号（只连接一次）
    connect(m_hideAnimation, &QAbstractAnimation::finished, this, &TipWidget::onHideAnimFinished);
    
    m_hideAnimation->start();
}

void TipWidget::setInitialPosition(int y)
{
    if (parentWidget()) {
        QPoint parentTopLeft = parentWidget()->mapToGlobal(QPoint(0, 0));
        QSize parentSize = parentWidget()->size();
        int x = parentTopLeft.x() + (parentSize.width() - width()) / 2;
        move(x, y);
    } else {
        QScreen *screen = QApplication::primaryScreen();
        QRect screenGeometry = screen->geometry();
        int x = (screenGeometry.width() - width()) / 2;
        move(x, y);
    }
}

void TipWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 不再绘制统一背景，改为分别绘制图标和文字区域的背景
    drawTipIcon(painter);
    drawText(painter);
}

void TipWidget::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);
}

void TipWidget::onAnimParamChanged()
{
    update();
}

void TipWidget::onShowAnimFinished()
{
    // 启动自动隐藏定时器
    m_autoHideTimer->start(m_duration);
}

void TipWidget::onAutoHide()
{
    hideTip();
}

void TipWidget::onHideAnimFinished()
{
    hide();
    emit hideFinished();
}

void TipWidget::forceStop()
{
    // 移除事件过滤器，避免父窗口持有引用
    if (parentWidget()) {
        parentWidget()->removeEventFilter(this);
    }
    
    // 停止自动隐藏定时器
    if (m_autoHideTimer && m_autoHideTimer->isActive()) {
        m_autoHideTimer->stop();
    }
    
    // 停止显示动画
    if (m_showAnimation && m_showAnimation->state() == QAbstractAnimation::Running) {
        m_showAnimation->stop();
    }
    
    // 停止隐藏动画
    if (m_hideAnimation && m_hideAnimation->state() == QAbstractAnimation::Running) {
        m_hideAnimation->stop();
    }
    
    // 断开所有动画信号连接
    if (m_showAnimation) {
        m_showAnimation->disconnect();
    }
    if (m_hideAnimation) {
        m_hideAnimation->disconnect();
    }
    
    // 重置窗口透明度，确保强制停止时窗口状态正确
    setWindowOpacity(1.0);
}

void TipWidget::drawBackground(QPainter &painter)
{
    painter.save();
    
    QColor bgColor = getColor();
    bgColor.setAlpha(200);
    painter.setBrush(bgColor);
    painter.setPen(Qt::NoPen);
    
    QRect bgRect = rect().adjusted(0, m_animY, 0, 0);
    painter.drawRoundedRect(bgRect, m_borderRadius, m_borderRadius);
    
    painter.restore();
}

void TipWidget::drawTipIcon(QPainter &painter)
{
    painter.save();
    
    int iconX = 0;
    int iconY = 0 + m_animY;
    int iconWidth = m_iconSize + m_padding; // 图标区域宽度包含图标和内边距
    QRect iconRect(iconX, iconY, iconWidth, height());
    
    // 绘制图标背景矩形，左上角和左下角有圆角
    QColor bgColor;
    switch (m_status) {
    case Succeed:
        bgColor = QColor(0, 128, 0); // 绿色背景
        break;
    case Dangerous:
        bgColor = QColor(255, 0, 0); // 红色背景
        break;
    case Warning:
        bgColor = QColor(255, 255, 0); // 黄色背景
        break;
    }
    
    painter.setBrush(bgColor);
    painter.setPen(Qt::NoPen);
    
    // 使用QPainterPath绘制左侧圆角矩形
    QPainterPath path;
    path.moveTo(iconRect.left() + m_borderRadius, iconRect.top());
    path.lineTo(iconRect.right(), iconRect.top());
    path.lineTo(iconRect.right(), iconRect.bottom());
    path.lineTo(iconRect.left() + m_borderRadius, iconRect.bottom());
    path.arcTo(iconRect.left(), iconRect.bottom() - 2 * m_borderRadius, 2 * m_borderRadius, 2 * m_borderRadius, 270, -90);
    path.lineTo(iconRect.left(), iconRect.top() + m_borderRadius);
    path.arcTo(iconRect.left(), iconRect.top(), 2 * m_borderRadius, 2 * m_borderRadius, 180, -90);
    path.closeSubpath();
    
    painter.drawPath(path);
    
    // 绘制图标
    QPoint iconCenter(iconRect.left() + iconWidth / 2, iconRect.center().y());
    painter.translate(iconCenter);
    
    switch (m_status) {
    case Succeed:
        drawHook(painter);
        break;
    case Dangerous:
        drawFork(painter);
        break;
    case Warning:
        drawExclamation(painter);
        break;
    }
    
    painter.restore();
}

void TipWidget::drawText(QPainter &painter)
{
    painter.save();
    
    int iconWidth = m_iconSize + m_padding;
    int textX = iconWidth;
    int textY = 0 + m_animY;
    QRect textRect(textX, textY, width() - textX, height());
    
    // 创建渐变背景，从浅色到更浅的色调
    QLinearGradient gradient(textRect.topLeft(), textRect.bottomLeft());
    QColor baseColor;
    switch (m_status) {
    case Succeed:
        baseColor = QColor(240, 248, 240); // 浅绿色
        gradient.setColorAt(0, baseColor);
        gradient.setColorAt(1, QColor(250, 255, 250)); // 更浅的绿色
        break;
    case Dangerous:
        baseColor = QColor(255, 240, 240); // 浅红色
        gradient.setColorAt(0, baseColor);
        gradient.setColorAt(1, QColor(255, 250, 250)); // 更浅的红色
        break;
    case Warning:
        baseColor = QColor(255, 252, 230); // 浅黄色
        gradient.setColorAt(0, baseColor);
        gradient.setColorAt(1, QColor(255, 255, 245)); // 更浅的黄色
        break;
    }
    
    painter.setBrush(QBrush(gradient));
    painter.setPen(Qt::NoPen);
    
    // 使用QPainterPath绘制右侧圆角矩形
    QPainterPath path;
    path.moveTo(textRect.left(), textRect.top());
    path.lineTo(textRect.right() - m_borderRadius, textRect.top());
    path.arcTo(textRect.right() - 2 * m_borderRadius, textRect.top(), 2 * m_borderRadius, 2 * m_borderRadius, 90, -90);
    path.lineTo(textRect.right(), textRect.bottom() - m_borderRadius);
    path.arcTo(textRect.right() - 2 * m_borderRadius, textRect.bottom() - 2 * m_borderRadius, 2 * m_borderRadius, 2 * m_borderRadius, 0, -90);
    path.lineTo(textRect.left(), textRect.bottom());
    path.lineTo(textRect.left(), textRect.top());
    path.closeSubpath();
    
    painter.drawPath(path);
    
    // 绘制文字
    painter.setPen(QColor(60, 60, 60)); // 深灰色文字，比纯黑色更柔和
    painter.setFont(font());
    QRect textDrawRect = textRect.adjusted(m_padding, 0, -m_padding, 0); // 添加内边距
    painter.drawText(textDrawRect, Qt::AlignLeft | Qt::AlignVCenter, m_text);
    
    painter.restore();
}

void TipWidget::drawHook(QPainter &painter)
{
    // 绘制对勾 - 白色
    painter.save();
    painter.setPen(QPen(Qt::white, 2, Qt::SolidLine, Qt::RoundCap));
    
    int size = m_iconSize / 2;
    painter.drawLine(-size/2, 0, -size/4, size/2);
    painter.drawLine(-size/4, size/2, size/2, -size/2);
    
    painter.restore();
}

void TipWidget::drawFork(QPainter &painter)
{
    // 绘制叉号 - 白色
    painter.save();
    painter.setPen(QPen(Qt::white, 2, Qt::SolidLine, Qt::RoundCap));
    
    int size = m_iconSize / 2;
    painter.drawLine(-size/2, -size/2, size/2, size/2);
    painter.drawLine(-size/2, size/2, size/2, -size/2);
    
    painter.restore();
}

void TipWidget::drawExclamation(QPainter &painter)
{
    // 绘制感叹号 - 使用图标颜色
    painter.save();
    painter.setPen(QPen(m_iconColor, 2, Qt::SolidLine, Qt::RoundCap));
    painter.setBrush(m_iconColor);
    
    int size = m_iconSize / 2;
    // 感叹号的竖线
    painter.drawLine(0, -size/2, 0, size/4);
    // 感叹号的点
    painter.drawEllipse(-1, size/2 - 1, 2, 2);
    
    painter.restore();
}

void TipWidget::drawInvalidation(QPainter &painter)
{
    // 绘制禁止符号
    painter.save();
    painter.setPen(QPen(m_iconColor, 2));
    
    int size = m_iconSize / 2;
    painter.drawEllipse(-size/2, -size/2, size, size);
    painter.drawLine(-size/3, -size/3, size/3, size/3);
    
    painter.restore();
}

QColor TipWidget::getColor() const
{
    switch (m_status) {
    case Succeed:
        return QColor(76, 175, 80);  // 绿色
    case Dangerous:
        return QColor(244, 67, 54);  // 红色
    case Warning:
        return QColor(255, 193, 7);  // 黄色
    default:
        return QColor(76, 175, 80);
    }
}

void TipWidget::updateColors()
{
    switch (m_status) {
    case Succeed:
        m_backgroundColor = QColor(76, 175, 80);  // 绿色
        m_textColor = QColor(0, 0, 0);  // 黑色文字，适配白色背景
        m_iconColor = QColor(255, 255, 255);
        break;
    case Dangerous:
        m_backgroundColor = QColor(244, 67, 54);  // 红色
        m_textColor = QColor(0, 0, 0);  // 黑色文字，适配白色背景
        m_iconColor = QColor(255, 255, 255);
        break;
    case Warning:
        m_backgroundColor = QColor(255, 193, 7);  // 黄色
        m_textColor = QColor(0, 0, 0);  // 黑色文字，适配白色背景
        m_iconColor = QColor(255, 165, 0);  // 橙色图标
        break;
    default:
        m_backgroundColor = QColor(76, 175, 80);
        m_textColor = QColor(0, 0, 0);  // 黑色文字，适配白色背景
        m_iconColor = QColor(255, 255, 255);
        break;
    }
}

void TipWidget::setAnimParam(int value)
{
    m_animParam = value;
    m_animY = value;
    onAnimParamChanged();
}

QPair<int, int> TipWidget::getAnimRange() const
{
    return QPair<int, int>(0, m_maxHeight);
}

bool TipWidget::isHiding() const
{
    return m_hideAnimation && m_hideAnimation->state() == QAbstractAnimation::Running;
}

bool TipWidget::eventFilter(QObject *watched, QEvent *event)
{
    // 监听父窗口的移动事件
    if (watched == parentWidget() && event->type() == QEvent::Move && isVisible()) {
        // 当父窗口移动时，重新计算提示窗的位置
        if (parentWidget()) {
            QWidget *parent = parentWidget();
            QPoint parentTopLeft = parent->mapToGlobal(QPoint(0, 0));
            QSize parentSize = parent->size();
            
            int x = parentTopLeft.x() + (parentSize.width() - width()) / 2;
            int y = parentTopLeft.y() + 50; // 距离父窗口顶部50像素
            move(x, y);
        }
    }
    
    return QWidget::eventFilter(watched, event);
}