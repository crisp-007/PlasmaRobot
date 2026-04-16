#include "flowdisplaywidget.h"
#include <QPainter>
#include <QPainterPath>
#include <QLinearGradient>
#include <QRadialGradient>
#include <QResizeEvent>
#include <QEasingCurve>
#include <cmath>

/**
 * @brief 流量显示控件构造函数
 * @param parent 父控件指针
 *
 * 初始化流量显示控件，设置默认样式、布局和属性。
 * 包含文字颜色、渐变背景、边框颜色与扫描动画的初始化。
 */
FlowDisplayWidget::FlowDisplayWidget(QWidget *parent)
    : QWidget(parent)
    , flow_label(nullptr)
    , layout(nullptr)
    , scan_animation(nullptr)
    , scan_position(0.0)
    , is_animating(false)
    , ripple_timer(nullptr)
    , current_value(0.0)
    , text_color(Qt::white)
    , text_size(14)
    , text_bold(true)
    , bg_start_color(QColor(33, 150, 243, 200))
    , bg_end_color(QColor(25, 118, 210, 200))
    , border_color(QColor(21, 101, 192))
{
    SetupUI();        // 初始化用户界面
    SetupAnimation(); // 初始化动画系统
}

/**
 * @brief 初始化用户界面
 *
 * 配置控件的基础属性，创建水平布局并添加显示标签。
 * 控件固定高度为 50 像素，支持透明背景与居中显示。
 */
void FlowDisplayWidget::SetupUI()
{
    // 设置 widget 基本属性
    setMinimumHeight(50);                              // 设置最小高度
    setMaximumHeight(50);                              // 设置最大高度，固定控件高度
    setAttribute(Qt::WA_TranslucentBackground);        // 设置透明背景支持
    
    // 创建水平布局容器
    layout = new QHBoxLayout(this);
    layout->setContentsMargins(15, 8, 15, 8);          // 设置内边距：左右15px，上下8px
    
    // 创建流量值显示标签
    flow_label = new QLabel(QString::number(current_value, 'f', 1) + " L/min", this);
    flow_label->setAlignment(Qt::AlignCenter);          // 标签居中对齐
    
    // 设置标签样式
    flow_label->setStyleSheet(
        "QLabel {"
        "    color: white;"                             // 文本颜色
        "    font-size: 14px;"                          // 字体大小14px
        "    font-weight: bold;"                        // 加粗
        "    background: transparent;"                  // 透明背景
        "}"
    );
    
    // 将标签添加到布局中
    layout->addWidget(flow_label);
}

/**
 * @brief 初始化动画系统
 *
 * 创建扫描动画与刷新定时器，控制扫描线位置与界面重绘。
 * 定时器以约 60fps 刷新，实现顺滑的动态效果。
 */
void FlowDisplayWidget::SetupAnimation()
{
    // 定义扫描位置的属性动画
    scan_animation = new QPropertyAnimation(this, "scanPosition", this);
    scan_animation->setDuration(2000);                     // 动画时长2秒，适中扫描速度
    scan_animation->setStartValue(0.0);                    // 起始位置：左边缘
    scan_animation->setEndValue(1.0);                      // 结束位置：右边缘
    scan_animation->setEasingCurve(QEasingCurve::OutQuad); // 缓动曲线：前快后慢
    
    // 连接动画完成信号到回调函数
    connect(scan_animation, &QPropertyAnimation::finished, 
            this, &FlowDisplayWidget::OnAnimationFinished);
    
    // 创建波纹刷新定时器
    ripple_timer = new QTimer(this);
    ripple_timer->setInterval(16);                         // 16ms，约60fps刷新
    
    // 连接定时器超时信号，控制界面重绘
    connect(ripple_timer, &QTimer::timeout, [this]() {
        if (is_animating) {
            update();                                      // 动画运行时定期更新界面
        } else {
            ripple_timer->stop();                          // 动画停止时停止定时器
        }
    });
}

/**
 * @brief 设置流量值
 * @param value 要显示的流量值字符串
 * 
 * 更新控件显示的流量值文本。只有当新值与当前值不同时才会更新，
 * 避免不必要的界面重绘。
 */
void FlowDisplayWidget::SetFlowValue(const float &value)
{
    if (current_value != value) {
        current_value = value;                             // 保存新的流量值
        flow_label->setText(QString::number(value, 'f', 1) + " L/min");                    // 更新标签显示文本
    }
}

/**
 * @brief 开始扫描动画
 * 
 * 启动从左到右的扫描动画效果。如果动画正在运行，会先停止当前动画
 * 然后重新开始。动画会循环播放直到手动停止。
 */
void FlowDisplayWidget::StartAnimation()
{
    // 如果动画正在运行，先停止
    if (scan_animation->state() == QPropertyAnimation::Running) {
        scan_animation->stop();
    }
    
    is_animating = true;                                   // 设置动画状态标志
    scan_position = 0.0;                                   // 重置扫描位置到起始点
    
    scan_animation->start();                               // 启动扫描动画
    ripple_timer->start();                                 // 启动波纹更新定时器
}

/**
 * @brief 停止扫描动画
 * 
 * 停止扫描动画并清除所有视觉效果。重置扫描位置并停止界面刷新，
 * 恢复到静态显示状态。
 */
void FlowDisplayWidget::StopAnimation()
{
    // 如果动画正在运行，停止它
    if (scan_animation->state() == QPropertyAnimation::Running) {
        scan_animation->stop();
    }
    
    is_animating = false;                                  // 清除动画状态标志
    ripple_timer->stop();                                  // 停止波纹更新定时器
    scan_position = 0.0;                                   // 重置扫描位置
    update();                                              // 强制界面刷新，清除视觉效果
}

/**
 * @brief 设置扫描位置，由属性动画回调函数
 * @param position 扫描位置，范围0.0-1.0（0.0=左边缘，1.0=右边缘）
 * 
 * 由QPropertyAnimation的回调函数调用，用于更新扫描线的当前位置。
 * 函数内部会定时统一刷新界面，此处不需要手动调用update()。
 */
void FlowDisplayWidget::SetScanPosition(qreal position)
{
    scan_position = position;                              // 更新扫描位置
    // 由于有定时器统一刷新界面，这里不需要手动update
}

/**
 * @brief 动画完成回调函数
 * 
 * 当扫描动画播放完毕时调用。检查动画状态是否为运行状态，
 * 如果是则重新开始动画，实现循环效果；否则清除视觉效果。
 */
void FlowDisplayWidget::OnAnimationFinished()
{
    // 如果动画仍然应该继续（没有被手动停止），重新开始动画
    if (is_animating) {
        scan_position = 0.0;                               // 重置扫描位置到起始点
        scan_animation->start();                           // 重新开始动画，实现循环效果
    } else {
        scan_position = 0.0;                               // 重置扫描位置
        update();                                          // 强制界面更新，清除扫描效果
    }
}

/**
 * @brief 绘制事件处理函数
 * @param event 绘制事件（参数未使用）
 * 
 * 负责绘制控件的所有视觉元素，包括：
 * 1. 蓝色渐变背景和右侧圆角矩形；
 * 2. 扫描动画效果：辉光、扫描线、能量爆发边缘效果
 */
void FlowDisplayWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);         // 设置抗锯齿，提高绘制质量
    
    QRect rect = this->rect();                             // 获取控件绘制区域
    
    // === 绘制背景和右侧圆角（显示流量数据的蓝色渐变） ===
    QLinearGradient bgGradient(0, 0, rect.width(), 0);     // 创建水平渐变对象
    bgGradient.setColorAt(0, bg_start_color);              // 设置渐变起始颜色
    bgGradient.setColorAt(1, bg_end_color);                // 设置渐变结束颜色
    
    painter.setBrush(bgGradient);                          // 设置渐变画刷
    painter.setPen(QPen(border_color, 1));                 // 设置边框画笔
    
    // === 创建右侧圆角矩形（左侧直角，右侧圆角） ===
    QPainterPath path;
    path.moveTo(0, 0);                                     // 从左上角开始
    path.lineTo(rect.width() - 8, 0);                      // 到右上角圆弧起始点
    path.arcTo(rect.width() - 16, 0, 16, 16, 90, -90);     // 右上角圆弧
    path.lineTo(rect.width(), rect.height() - 8);          // 到右下角圆弧起始点
    path.arcTo(rect.width() - 16, rect.height() - 16, 16, 16, 0, -90); // 右下角圆弧
    path.lineTo(0, rect.height());                         // 到左下角
    path.closeSubpath();                                   // 封闭路径
    
    painter.drawPath(path);                                // 绘制背景路径
    
    // === 绘制扫描动画效果 ===
    if (is_animating && scan_position < 0.98) {            // 优化性能：避免边缘效果过度绘制
        qreal x = scan_position * rect.width();            // 计算扫描线的当前X坐标
        
        // 计算淡出效果（从0.92开始逐渐淡出）
        qreal fadeAlpha = 1.0;
        if (scan_position > 0.92) {
            fadeAlpha = 1.0 - (scan_position - 0.92) / 0.06; // 在0.92-0.98之间逐渐淡出
        }
        
        // === 1. 绘制扫描线左侧的辉光拖尾（灯光跟随效果） ===
        qreal glowWidth = 80;                               // 辉光拖尾宽度
        QLinearGradient leftGlow(x - glowWidth, 0, x, 0);   // 从左到右的渐变
        leftGlow.setColorAt(0, QColor(255, 255, 255, 0));                    // 完全透明
        leftGlow.setColorAt(0.3, QColor(255, 255, 255, 20 * fadeAlpha));     // 微弱辉光
        leftGlow.setColorAt(0.7, QColor(255, 255, 255, 60 * fadeAlpha));     // 中等辉光
        leftGlow.setColorAt(1, QColor(255, 255, 255, 120 * fadeAlpha));      // 最强辉光
        
        painter.setBrush(leftGlow);                         // 设置辉光画刷
        painter.setPen(Qt::NoPen);                          // 无边框
        painter.drawRect(x - glowWidth, 0, glowWidth, rect.height()); // 绘制辉光区域
        
        // === 2. 绘制主扫描线 ===
        QLinearGradient scanLine(x - 2, 0, x + 2, 0);      // 扫描线渐变（4像素宽度）
        scanLine.setColorAt(0, QColor(255, 255, 255, 100 * fadeAlpha));      // 边缘半透明
        scanLine.setColorAt(0.5, QColor(255, 255, 255, 255 * fadeAlpha));    // 中心不透明
        scanLine.setColorAt(1, QColor(255, 255, 255, 100 * fadeAlpha));      // 边缘半透明
        
        painter.setBrush(scanLine);                         // 设置扫描线画刷
        painter.setPen(Qt::NoPen);                          // 无边框
        painter.drawRect(x - 2, 0, 4, rect.height());      // 绘制扫描线
        
        // === 3. 扫描线的垂直光束效果 ===
        QLinearGradient beamGlow(x - 8, 0, x + 8, 0);      // 光束渐变（16像素宽度）
        beamGlow.setColorAt(0, QColor(255, 255, 255, 0));                    // 边缘透明
        beamGlow.setColorAt(0.5, QColor(255, 255, 255, 80 * fadeAlpha));     // 中心半透明
        beamGlow.setColorAt(1, QColor(255, 255, 255, 0));                    // 边缘透明
        
        painter.setBrush(beamGlow);                         // 设置光束画刷
        painter.setPen(Qt::NoPen);                          // 无边框
        painter.drawRect(x - 8, 0, 16, rect.height());     // 绘制光束效果
        
        // === 4. 扫描线的右边缘能量效果 ===
        if (scan_position > 0.75 && scan_position < 0.95) {
            qreal edgeProgress = (scan_position - 0.75) / 0.2; // 计算边缘效果进度（0到1）
            
            // 能量爆发效果，带有强度和微弹跳
            qreal bounceIntensity = edgeProgress;
            if (edgeProgress > 0.7) {
                // 后期阶段带有弹跳效果
                qreal bouncePhase = (edgeProgress - 0.7) / 0.3;
                bounceIntensity = 0.7 + 0.3 * (1.0 + 0.3 * sin(bouncePhase * 3.14159 * 2)); // 轻微弹跳
            }
            
            // 右边缘的能量聚集效果，逐渐变成椭圆
            QRadialGradient energyBurst(rect.width() - 8, rect.height() / 2, 45 * bounceIntensity);
            energyBurst.setColorAt(0, QColor(255, 255, 255, 160 * bounceIntensity * fadeAlpha)); // 中心白色
            energyBurst.setColorAt(0.3, QColor(100, 200, 255, 120 * bounceIntensity * fadeAlpha)); // 浅蓝色
            energyBurst.setColorAt(0.6, QColor(50, 150, 255, 60 * bounceIntensity * fadeAlpha));  // 深蓝色
            energyBurst.setColorAt(1, QColor(255, 255, 255, 0));                                  // 边缘透明
            
            painter.setBrush(energyBurst);              // 设置能量爆发画刷
            painter.setPen(Qt::NoPen);                  // 无边框
            painter.drawEllipse(QPointF(rect.width() - 8, rect.height() / 2), 
                              45 * bounceIntensity, rect.height() * 0.8 * bounceIntensity); // 绘制能量爆发椭圆
            
            // 添加边缘闪烁效果
            if (edgeProgress > 0.6) {
                qreal flashIntensity = sin((edgeProgress - 0.6) * 15) * 0.4 + 0.6; // 高频闪烁
                QLinearGradient edgeFlash(rect.width() - 15, 0, rect.width(), 0);   // 右边缘渐变
                edgeFlash.setColorAt(0, QColor(255, 255, 255, 0));                              // 左侧透明
                edgeFlash.setColorAt(1, QColor(255, 255, 255, 100 * flashIntensity * fadeAlpha)); // 右侧闪烁
                
                painter.setBrush(edgeFlash);            // 设置闪烁画刷
                painter.setPen(Qt::NoPen);              // 无边框
                painter.drawRect(rect.width() - 15, 0, 15, rect.height()); // 绘制边缘闪烁区域
            }
        }
    }
}

/**
 * @brief 控件大小改变事件处理函数
 * @param event 大小改变事件对象
 * 
 * 当控件大小发生变化时调用，确保绘制能够正确重绘并适应新的尺寸。
 */
void FlowDisplayWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);                           // 调用基类处理函数
    update();                                              // 强制界面刷新以适应新尺寸
}
