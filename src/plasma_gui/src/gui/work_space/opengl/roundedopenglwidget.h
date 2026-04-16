#ifndef ROUNDEDOPENGLWIDGET_H
#define ROUNDEDOPENGLWIDGET_H

#include <QOpenGLWidget>
#include <QPainter>
#include <QPainterPath>
#include <QResizeEvent>
#include <QFont>

class RoundedOpenGLWidget : public QOpenGLWidget
{
    Q_OBJECT

public:
    explicit RoundedOpenGLWidget(QWidget *parent = nullptr);
    void SetCornerRadius(int radius);
    void SetText(const QString &text);
    void SetShowText(bool show);

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    int corner_radius;
    QString text;
    bool show_text;
    QFont text_font;
    void UpdateMask();
};

#endif // ROUNDEDOPENGLWIDGET_H