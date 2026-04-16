#ifndef TIP_MANAGER_H
#define TIP_MANAGER_H

#include <QObject>
#include <QList>
#include <QPropertyAnimation>
#include <QTimer>
#include "tip_widget.h"

class TipManager : public QObject
{
    Q_OBJECT

public:
    explicit TipManager(QWidget *parent = nullptr);
    ~TipManager();
    
    void showTip(const QString &text, TipWidget::TipsStatus status = TipWidget::Succeed);
    void removeTip(TipWidget *tip);
    
private slots:
    void onTipHideFinished();
    void onAutoHideOldest();
    
private:
    void updateTipPositions();
    void animateTipToPosition(TipWidget *tip, int targetY);
    int calculateTipY(int index);
    
    QWidget *m_parent;
    QList<TipWidget*> m_activeTips;
    QTimer *m_autoHideTimer;           // 统一管理自动隐藏的定时器
    static const int TIP_HEIGHT = 40;  // 提示窗口高度
    static const int TIP_SPACING = 10; // 提示窗口间距
    static const int TOP_MARGIN = 10;  // 距离父窗口顶部的边距
    static const int MAX_TIPS = 3;     // 最大同时显示的提示窗口数量
    static const int AUTO_HIDE_INTERVAL = 1500; // 自动隐藏间隔时间（毫秒）
    static const int KEEP_DURATION = 2000; // 提示窗口保持显示时间（毫秒）
};

#endif // TIP_MANAGER_H