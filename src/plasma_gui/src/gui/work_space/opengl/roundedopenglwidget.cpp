#include "roundedopenglwidget.h"
#include <QRegion>

RoundedOpenGLWidget::RoundedOpenGLWidget(QWidget *parent)
    : QOpenGLWidget(parent), corner_radius(5), text("No Signal"), show_text(true)
{
    setAttribute(Qt::WA_OpaquePaintEvent, false);
    setAttribute(Qt::WA_NoSystemBackground, false);
    
    // 设置字体为粗体
    text_font.setBold(true);
    text_font.setPointSize(12);
}

void RoundedOpenGLWidget::SetCornerRadius(int radius)
{
    corner_radius = radius;
    UpdateMask();
    update();
}

void RoundedOpenGLWidget::SetText(const QString &text)
{
    this->text = text;
    update();
}

void RoundedOpenGLWidget::SetShowText(bool show)
{
    show_text = show;
    update();
}

void RoundedOpenGLWidget::paintEvent(QPaintEvent *event)
{
    // 先调用基类的paintEvent进行OpenGL渲染
    QOpenGLWidget::paintEvent(event);
    
    // 然后绘制覆盖层文字和边框
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 创建圆角矩形路径
    QPainterPath roundedPath;
    roundedPath.addRoundedRect(rect(), corner_radius, corner_radius);
    
    // 创建完整矩形路径
    QPainterPath fullPath;
    fullPath.addRect(rect());
    
    // 计算需要覆盖的区域（完整矩形减去圆角矩形）
    QPainterPath maskPath = fullPath.subtracted(roundedPath);
    
    // 用背景色填充需要覆盖的区域
    painter.fillPath(maskPath, QColor(74, 74, 74)); // #4a4a4a与mainDisplayWidget背景色一致
    
    // 绘制圆角边框
    QPen pen;
    pen.setWidth(2);
    pen.setColor(QColor(74, 74, 74)); // #4a4a4a
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(roundedPath);
    
    // 绘制文本
    if (show_text && !text.isEmpty()) {
        painter.setFont(text_font);
        painter.setPen(QColor(70, 130, 180)); // 钢蓝色 #4682B4
        painter.drawText(rect(), Qt::AlignCenter, text);
    }
}

void RoundedOpenGLWidget::resizeEvent(QResizeEvent *event)
{
    QOpenGLWidget::resizeEvent(event);
    UpdateMask();
}

void RoundedOpenGLWidget::UpdateMask()
{
    QPainterPath path;
    path.addRoundedRect(rect(), corner_radius, corner_radius);
    QRegion region = QRegion(path.toFillPolygon().toPolygon());
    setMask(region);
}