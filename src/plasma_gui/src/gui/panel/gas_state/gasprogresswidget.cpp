#include "gasprogresswidget.h"
#include <QPaintEvent>

GasProgressWidget::GasProgressWidget(QWidget *parent)
    : QWidget(parent)
    , value(0)
    , minimum(0)
    , maximum(100)
    , corner_radius(10)
    , background_color(QColor(51, 51, 51))  // 背景色
    , border_color(QColor(85, 85, 85))      // 边框颜色
{
    setMinimumSize(20, 100);
    setMaximumSize(20, 100);
}

void GasProgressWidget::SetPercentage(double percentage)
{
    if (percentage < 0.0) percentage = 0.0;
    if (percentage > 100.0) percentage = 100.0;
    
    int newValue = static_cast<int>((percentage / 100.0) * (maximum - minimum)) + minimum;
    if (this->value != newValue) {
        this->value = newValue;
        update();
    }
}

double GasProgressWidget::GetPercentage() const
{
    if (maximum <= minimum) return 0.0;
    return static_cast<double>(value - minimum) / (maximum - minimum) * 100.0;
}

void GasProgressWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    QRect rect = this->rect().adjusted(1, 1, -1, -1);
    
    // 绘制背景
    QPainterPath backgroundPath;
    backgroundPath.addRoundedRect(rect, corner_radius, corner_radius);
    painter.fillPath(backgroundPath, background_color);
    
    // 绘制边框
    painter.setPen(QPen(border_color, 1));
    painter.drawPath(backgroundPath);
    
    // 计算进度高度
    if (maximum > minimum) {
        double progress = static_cast<double>(value - minimum) / (maximum - minimum);
        int progressHeight = static_cast<int>(rect.height() * progress);
        double percentage = progress * 100.0;
        
        if (progressHeight > 0) {
            // 根据当前百分比确定填充颜色
            QColor fillColor;
            if (percentage >= 50.0) {
                // 50%以上使用绿色（参考图片样式）
                fillColor = QColor(76, 175, 80); // 绿色 #4CAF50
            } else {
                // 50%以下根据电位逐渐变色
                if (percentage >= 40.0) {
                    // 40-50%: 浅绿色混合
                    fillColor = QColor(139, 195, 74); // 浅绿偏黄
                } else if (percentage >= 30.0) {
                    // 30-40%: 黄绿色
                    fillColor = QColor(205, 220, 57); // 黄绿
                } else if (percentage >= 20.0) {
                    // 20-30%: 橙黄色
                    fillColor = QColor(255, 193, 7); // 橙黄
                } else if (percentage >= 10.0) {
                    // 10-20%: 橙色
                    fillColor = QColor(255, 152, 0); // 橙色
                } else {
                    // 10%以下: 红色（警告）
                    fillColor = QColor(244, 67, 54); // 红色
                }
            }
            
            // 绘制进度条 - 直接使用矩形区域确保完全填充到底部
            QRect progressRect = QRect(rect.left(), rect.bottom() - progressHeight + 1, 
                                     rect.width(), progressHeight);
            
            // 创建斜纹填充效果
            painter.setPen(Qt::NoPen);
            
            // 创建斜纹图案
            QPixmap pattern(8, 8);
            pattern.fill(fillColor.darker(130)); // 深色底色
            
            QPainter patternPainter(&pattern);
            patternPainter.setPen(QPen(fillColor, 2));
            // 绘制斜纹
            patternPainter.drawLine(0, 8, 8, 0);
            patternPainter.drawLine(-2, 6, 6, -2);
            patternPainter.drawLine(2, 10, 10, 2);
            patternPainter.end();
            
            // 设置裁剪区域为圆角矩形，确保进度条不会超出边界
            painter.setClipPath(backgroundPath);
            
            // 直接在矩形区域内绘制斜纹图案
            painter.fillRect(progressRect, QBrush(pattern));
            
            // 添加轻微的高光效果
            QLinearGradient highlightGradient(progressRect.topLeft(), progressRect.topRight());
            highlightGradient.setColorAt(0, QColor(255, 255, 255, 15));
            highlightGradient.setColorAt(0.5, QColor(255, 255, 255, 25));
            highlightGradient.setColorAt(1, QColor(255, 255, 255, 15));
            
            painter.fillRect(progressRect, QBrush(highlightGradient));
            
            painter.setClipping(false);
        }
    }
}