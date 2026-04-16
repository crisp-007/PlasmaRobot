#include "processing_time_widget.h"
#include <QApplication>
#include <QDebug>
#include <QMouseEvent>

ProcessingTimeWidget::ProcessingTimeWidget(QWidget *parent)
    : QWidget(parent)
    , time_label(nullptr)
    , update_timer(new QTimer(this))
    , elapsed_timer(new QElapsedTimer())
    , is_running(false)
    , total_elapsed(0)
    , pulse_radius_animation(nullptr)
    , pulse_opacity_animation(nullptr)
    , scale_radius_animation(nullptr)
    , pulse_group(nullptr)
    , scale_group(nullptr)
    , pulse_radius(0.0)
    , pulse_opacity(1.0)
    , scale_radius(0.0)
    , center(0, 0)
    , max_radius(0.0)
    , scale_update_timer(new QTimer(this))
{
    SetupUI();
    SetupAnimations();
    
    // 连接计时器
    connect(update_timer, &QTimer::timeout, this, &ProcessingTimeWidget::UpdateTime);
    connect(scale_update_timer, &QTimer::timeout, this, &ProcessingTimeWidget::UpdateScaleRings);
    
    // 设置更新频率
    update_timer->setInterval(100); // 100ms更新一次时间显示
    scale_update_timer->setInterval(50); // 50ms更新一次刻度动画
    
    // 初始化刻度环 - 添加更多层次感和范围效果
    for (int i = 0; i < 8; ++i) {
        ScaleRing ring;
        ring.radius = 25 + i * 12;
        ring.opacity = 1.0 - i * 0.12;
        ring.rotation = i * 45; // 每层旋转45度，增加层次感
        ring.scale_count = 12 + i * 6;
        scale_rings.append(ring);
    }
}

ProcessingTimeWidget::~ProcessingTimeWidget()
{
    if (elapsed_timer) {
        delete elapsed_timer;
    }
}

void ProcessingTimeWidget::SetupUI()
{
    // ����ʱ���ǩ
    time_label = new QLabel("00:00", this);
    time_label->setAlignment(Qt::AlignCenter);
    time_label->resize(300,120);
    time_label->setStyleSheet(
        "QLabel {"
            "color: #00ff88;"
            "font-family: 'Consolas', 'Monaco', monospace;"
            "font-size: 105px;"
            "font-weight: 900;"
            "background: transparent;"
            "border: none;"
        "}"
    );
    
   
}

void ProcessingTimeWidget::SetupAnimations()
{
    // 脉冲半径动画
    pulse_radius_animation = new QPropertyAnimation(this, "pulseRadius", this);
    pulse_radius_animation->setDuration(1200); // 脉冲扩散周期约1.2秒
    pulse_radius_animation->setStartValue(0.0);
    pulse_radius_animation->setEndValue(200.0); // 根据窗口大小调整显示范围
    pulse_radius_animation->setEasingCurve(QEasingCurve::OutQuad);
    
    // ����͸���ȶ���
    pulse_opacity_animation = new QPropertyAnimation(this, "pulseOpacity", this);
    pulse_opacity_animation->setDuration(1200);
    pulse_opacity_animation->setStartValue(0.8);
    pulse_opacity_animation->setEndValue(0.0);
    pulse_opacity_animation->setEasingCurve(QEasingCurve::OutQuad);
    
    // 刻度半径动画
    scale_radius_animation = new QPropertyAnimation(this, "scaleRadius", this);
    scale_radius_animation->setDuration(2000);
    scale_radius_animation->setStartValue(0.0);
    scale_radius_animation->setEndValue(80.0);
    scale_radius_animation->setEasingCurve(QEasingCurve::OutCubic);
    scale_radius_animation->setLoopCount(-1); // 无限循环
    
    // 设置脉冲动画
    pulse_group = new QSequentialAnimationGroup(this);
    
    QParallelAnimationGroup *pulseParallel = new QParallelAnimationGroup();
    pulseParallel->addAnimation(pulse_radius_animation);
    pulseParallel->addAnimation(pulse_opacity_animation);
    
    pulse_group->addAnimation(pulseParallel);
    pulse_group->addPause(300); // 脉冲扩散间隔
    pulse_group->setLoopCount(-1); // 无限循环
    
    // 创建刻度动画组
    scale_group = new QParallelAnimationGroup(this);
    scale_group->addAnimation(scale_radius_animation);
    
    // 连接动画完成信号
    connect(pulse_group, &QSequentialAnimationGroup::finished, 
            this, &ProcessingTimeWidget::OnPulseAnimationFinished);
}

void ProcessingTimeWidget::StartTiming()
{
    if (!is_running) {
        is_running = true;
        elapsed_timer->start();
        update_timer->start();
        scale_update_timer->start();
        
        // 启动动画
        pulse_group->start();
        scale_group->start();
        
        qDebug() << "Processing time started";
    }
}

void ProcessingTimeWidget::StopTiming()
{
    if (is_running) {
        is_running = false;
        total_elapsed += elapsed_timer->elapsed();
        update_timer->stop();
        scale_update_timer->stop();
        
        // 停止动画
        pulse_group->stop();
        scale_group->stop();
        
        qDebug() << "Processing time stopped, total:" << total_elapsed << "ms";
    }
}

void ProcessingTimeWidget::ResetTiming()
{
    is_running = false;
    total_elapsed = 0;
    update_timer->stop();
    scale_update_timer->stop();
    
    // 停止动画
    pulse_group->stop();
    scale_group->stop();
    
    // 重置动画属性
    pulse_radius = 0.0;
    pulse_opacity = 1.0;
    scale_radius = 0.0;
    
    // 重置显示
    time_label->setText("00:00");
    update();
    
    qDebug() << "Processing time reset";
}

bool ProcessingTimeWidget::IsRunning() const
{
    return is_running;
}

void ProcessingTimeWidget::SetPulseRadius(qreal radius)
{
    pulse_radius = radius;
    update();
}

void ProcessingTimeWidget::SetPulseOpacity(qreal opacity)
{
    pulse_opacity = opacity;
    update();
}

void ProcessingTimeWidget::SetScaleRadius(qreal radius)
{
    scale_radius = radius;
    update();
}

void ProcessingTimeWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 绘制背景
    DrawBackground(painter);
    
    // 只有运行时才绘制动画效果
    if (is_running) {
        DrawPulseRings(painter);
    }
    
    // 调用父类的paintEvent，确保子控件（如label）正常绘制
    QWidget::paintEvent(event);
}

void ProcessingTimeWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    
    // 计算中心点和最大半径 - 避免绘制到右边界
    center = QPointF(width() / 2.0, height() / 2.0);
    max_radius = width() / 2.0 - 10; // 留出空隙，让光圈扩散不到右边界
    
    // 设置时间标签位置，使其居中（这里使用已设置的尺寸）
    if (time_label) {
        int labelWidth = time_label->width();   // 使用当前宽度
        int labelHeight = time_label->height(); // 使用当前高度
        time_label->move(
            (width() - labelWidth) / 2,
            (height() - labelHeight) / 2
        );
    }
}

void ProcessingTimeWidget::DrawBackground(QPainter &painter)
{
    // 绘制圆角矩形背景
    QPainterPath path;
    path.addRoundedRect(rect(), 10, 10);
    
    // 填充背景
    QLinearGradient gradient(0, 0, width(), height());
    gradient.setColorAt(0, QColor(43, 43, 43, 255));
    gradient.setColorAt(1, QColor(35, 35, 35, 255));
    
    painter.fillPath(path, gradient);
    
    // 绘制边框
    painter.setPen(QPen(QColor(85, 85, 85), 1));
    painter.drawPath(path);
}

void ProcessingTimeWidget::DrawPulseRings(QPainter &painter)
{
    painter.save();
    
    // 绘制脉冲扩散 - 圆环效果
    if (pulse_radius > 0 && pulse_opacity > 0) {
        // 外层圆环
        QRadialGradient pulseGradient(center, pulse_radius);
        pulseGradient.setColorAt(0, QColor(0, 255, 136, int(pulse_opacity * 100)));
        pulseGradient.setColorAt(0.7, QColor(0, 255, 136, int(pulse_opacity * 60)));
        pulseGradient.setColorAt(1, QColor(0, 255, 136, 0));
        
        painter.setBrush(pulseGradient);
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(center, pulse_radius, pulse_radius);
        
        // 内部圆环
        qreal innerRadius = pulse_radius * 0.7;
        painter.setPen(QPen(QColor(0, 255, 136, int(pulse_opacity * 180)), 2));
        painter.setBrush(Qt::NoBrush);
        painter.drawEllipse(center.x() - innerRadius, center.y() - innerRadius,
                           innerRadius * 2, innerRadius * 2);
        
        // 中心圆环
        qreal outerRadius = pulse_radius * 1.3;
        painter.setPen(QPen(QColor(0, 255, 136, int(pulse_opacity * 120)), 1));
        painter.drawEllipse(center.x() - outerRadius, center.y() - outerRadius,
                           outerRadius * 2, outerRadius * 2);
    }
    
    // 绘制刻度扩散效果 - 增强范围效果
    for (int ringIndex = 0; ringIndex < scale_rings.size(); ++ringIndex) {
        const auto &ring = scale_rings[ringIndex];
        if (ring.radius <= max_radius) {
            // 调整透明度，形成范围效果
            qreal enhancedOpacity = ring.opacity * (is_running ? 1.0 : 0.3);
            DrawScalePattern(painter, center, ring.radius, enhancedOpacity);
        }
    }
    
    painter.restore();
}

void ProcessingTimeWidget::DrawScalePattern(QPainter &painter, const QPointF &center, qreal radius, qreal opacity)
{
    painter.save();
    
    // 设置画笔
    QColor scaleColor(0, 255, 136, int(opacity * 80));
    painter.setPen(QPen(scaleColor, 2));
    painter.setBrush(QBrush(QColor(0, 255, 136, int(opacity * 30))));
    
    // 绘制刻度状态片段
    int scaleCount = 16; // 每个圆环的刻度数量
    qreal angleStep = 360.0 / scaleCount;
    
    for (int i = 0; i < scaleCount; ++i) {
        qreal angle = i * angleStep + (radius * 2); // 添加旋转偏移
        qreal radians = qDegreesToRadians(angle);
        
        // 计算刻度位置
        QPointF scaleCenter(
            center.x() + radius * cos(radians),
            center.y() + radius * sin(radians)
        );
        
        // 绘制当前刻度状态的圆弧
        qreal scaleWidth = 8;
        qreal scaleHeight = 4;
        
        painter.save();
        painter.translate(scaleCenter);
        painter.rotate(qRadiansToDegrees(radians) + 90); // 旋转角度调整
        
        QPainterPath scalePath;
        scalePath.addEllipse(-scaleWidth/2, -scaleHeight/2, scaleWidth, scaleHeight);
        
        // 添加渐变效果
        QRadialGradient scaleGradient(0, 0, scaleWidth/2);
        scaleGradient.setColorAt(0, QColor(0, 255, 136, int(opacity * 120)));
        scaleGradient.setColorAt(1, QColor(0, 255, 136, int(opacity * 20)));
        
        painter.fillPath(scalePath, scaleGradient);
        painter.restore();
    }
    
    painter.restore();
}

QString ProcessingTimeWidget::FormatTime(qint64 milliseconds)
{
    qint64 seconds = milliseconds / 1000;
    qint64 minutes = seconds / 60;
    seconds = seconds % 60;
    
    return QString("%1:%2")
        .arg(minutes, 2, 10, QChar('0'))
        .arg(seconds, 2, 10, QChar('0'));
}

void ProcessingTimeWidget::UpdateTime()
{
    if (is_running) {
        qint64 currentElapsed = total_elapsed + elapsed_timer->elapsed();
        time_label->setText(FormatTime(currentElapsed));
    }
}

void ProcessingTimeWidget::UpdateScaleRings()
{
    if (!is_running) return;
    
    // 更新所有环的位置和透明度
    for (auto &ring : scale_rings) {
        ring.radius += 0.5; // 扩散速度
        ring.opacity *= 0.998; // 逐渐淡出
        ring.rotation += 1.0; // 旋转
        
        // 重置环
        if (ring.radius > max_radius || ring.opacity < 0.1) {
            ring.radius = 20;
            ring.opacity = 0.8;
            ring.rotation = 0;
        }
    }
    
    update();
}

void ProcessingTimeWidget::OnPulseAnimationFinished()
{
    // 脉冲动画完成后的处理
}

void ProcessingTimeWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        emit clicked();
    }
    QWidget::mousePressEvent(event);
}