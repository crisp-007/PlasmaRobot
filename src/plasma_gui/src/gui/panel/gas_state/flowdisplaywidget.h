#ifndef FLOWDISPLAYWIDGET_H
#define FLOWDISPLAYWIDGET_H

#include <QWidget>
#include <QLabel>
#include <QPropertyAnimation>
#include <QPainter>
#include <QTimer>
#include <QHBoxLayout>

class FlowDisplayWidget : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(qreal scanPosition READ ScanPosition WRITE SetScanPosition)

public:
    // 1. 初始化
    explicit FlowDisplayWidget(QWidget *parent = nullptr);
    
    // 2. 设置当前流量数值
    void SetFlowValue(const float &value);
    
    // 3. 开始/停止动画
    void StartAnimation();
    void StopAnimation();

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void OnAnimationFinished();

private:
    void SetupUI();
    void SetupAnimation();
    
    qreal ScanPosition() const { return scan_position; }
    void SetScanPosition(qreal position);
    
    QLabel *flow_label;
    QHBoxLayout *layout;
    
    // 动画相关
    QPropertyAnimation *scan_animation;
    qreal scan_position;
    bool is_animating;
    QTimer *ripple_timer;
    
    // 样式相关
    float current_value;
    QColor text_color;
    int text_size;
    bool text_bold;
    QColor bg_start_color;
    QColor bg_end_color;
    QColor border_color;
};

#endif // FLOWDISPLAYWIDGET_H
