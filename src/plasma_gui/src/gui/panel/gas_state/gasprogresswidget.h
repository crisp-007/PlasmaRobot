#ifndef GASPROGRESSWIDGET_H
#define GASPROGRESSWIDGET_H

#include <QWidget>
#include <QPainter>
#include <QPainterPath>
#include <QLinearGradient>

class GasProgressWidget : public QWidget
{
    Q_OBJECT
public:
    // 1. 初始化
    explicit GasProgressWidget(QWidget *parent = nullptr);
    
    // 2. 设置当前的百分比 (0.0 - 100.0)
    void SetPercentage(double percentage);
    
    // 3. 获取当前的百分比
    double GetPercentage() const;

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    // 基础属性
    int value;
    int minimum;
    int maximum;
    int corner_radius;
    QColor background_color;
    QColor border_color;
};

#endif // GASPROGRESSWIDGET_H