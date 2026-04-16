#ifndef TIP_WIDGET_H
#define TIP_WIDGET_H

#include <QWidget>
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>
#include <QTimer>
#include <QPainter>
#include <QPainterPath>
#include <QEvent>
#include <QRect>
#include <QColor>
#include <QEasingCurve>
#include <QSize>
#include <QEvent>

class TipWidget : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(int animParam READ getAnimParam WRITE setAnimParam)
    Q_PROPERTY(QRect geometry READ geometry WRITE setGeometry)
    Q_PROPERTY(qreal opacity READ windowOpacity WRITE setWindowOpacity)

public:
    enum TipsStatus {
        Succeed,
        Dangerous,
        Warning
    };

    explicit TipWidget(QWidget *parent = nullptr);
    ~TipWidget();

    void setParams(const QString &text, TipsStatus status = Succeed, int duration = 3000);
    void showTip();
    void hideTip();
    void setInitialPosition(int y);
    void forceStop(); // 强制停止所有动画和定时器
    bool isHiding() const; // 检查是否正在进行淡出动画

signals:
    void hideFinished();

protected:
    void paintEvent(QPaintEvent *event) override;
    void showEvent(QShowEvent *event) override;
    bool eventFilter(QObject *watched, QEvent *event) override;

private slots:
    void onAnimParamChanged();
    void onShowAnimFinished();
    void onAutoHide();
    void onHideAnimFinished();

private:
    void initUI();
    void initAnimation();
    void drawBackground(QPainter &painter);
    void drawTipIcon(QPainter &painter);
    void drawText(QPainter &painter);
    void drawHook(QPainter &painter);
    void drawFork(QPainter &painter);
    void drawExclamation(QPainter &painter);
    void drawInvalidation(QPainter &painter);
    QColor getColor() const;
    void updateColors();
    
    // Animation related
    int getAnimParam() const { return m_animParam; }
    void setAnimParam(int value);
    QPair<int, int> getAnimRange() const;

private:
    QString m_text;
    TipsStatus m_status;
    int m_duration;
    int m_maxHeight;
    int m_animParam;
    int m_animY;
    
    // Animation objects
    QPropertyAnimation *m_showAnimation;
    QParallelAnimationGroup *m_hideAnimation;
    QTimer *m_autoHideTimer;
    
    // UI parameters
    int m_borderRadius;
    int m_iconSize;
    int m_padding;
    QColor m_backgroundColor;
    QColor m_textColor;
    QColor m_iconColor;
};

#endif // TIP_WIDGET_H