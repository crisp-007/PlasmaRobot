#ifndef ROBOTARMGL_H
#define ROBOTARMGL_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QPainter>
#include <QFont>
#include <QTimer>
#include "opengl_dis.h"

class RobotArmgl : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit RobotArmgl(QWidget *parent = nullptr);
    ~RobotArmgl();

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
    void RenderRobotArmContent();
    void DrawRoundedBorder(QPainter& painter);
};

#endif // ROBOTARMGL_H