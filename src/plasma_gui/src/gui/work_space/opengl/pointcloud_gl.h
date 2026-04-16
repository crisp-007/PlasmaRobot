#ifndef POINTCLOUDGL_H
#define POINTCLOUDGL_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QPainter>
#include <QFont>
#include <QTimer>
#include "opengl_dis.h"

class PointCloudgl : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit PointCloudgl(QWidget *parent = nullptr);
    ~PointCloudgl();

    void SetShowNoSignal(bool show);
    void SetCustomText(const QString &text);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;

private:
    bool show_no_signal;
    QString custom_text;
    QFont text_font;
    QPixmap no_signal_image;
    private:
    void SetupFont();
    void RenderNoSignalText(QPainter& painter);
    void RenderPointCloudContent();
    void DrawRoundedBorder(QPainter& painter);
};

#endif // POINTCLOUDGL_H