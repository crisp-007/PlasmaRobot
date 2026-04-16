#include "pointcloud_gl.h"
#include <QOpenGLContext>
#include <QPainterPath>
#include <QDebug>
#include <QApplication>
#include <QDir>

PointCloudgl::PointCloudgl(QWidget *parent)
    : QOpenGLWidget(parent), show_no_signal(true)
{
    SetupFont();
    setMinimumSize(200, 150);
    // 加载点云图片 - 使用动态路径搜索
    QString appDir = QApplication::applicationDirPath();
    qDebug() << "Point cloud - App directory:" << appDir;
    
    // 尝试多个可能的路径
    QStringList possiblePaths = {
        QDir(appDir).absoluteFilePath("image/cam.png"),                    // 同级image文件夹
        QDir(appDir).absoluteFilePath("../../image/cam.png"),             // 项目根目录的image文件夹
        QDir(appDir).absoluteFilePath("../../../image/cam.png"),          // 上级目录的image文件夹
        "d:/WorkSpace/CodeSpace/Qt/test1/image/cam.png"                   // 绝对路径
    };
    
    bool loaded = false;
    for (const QString& path : possiblePaths) {
        QString cleanPath = QDir::cleanPath(path);
        qDebug() << "Trying to load point cloud image from:" << cleanPath;
        loaded = no_signal_image.load(cleanPath);
        if (loaded) {
            qDebug() << "Successfully loaded point cloud image from:" << cleanPath;
            qDebug() << "Point cloud image size:" << no_signal_image.size();
            break;
        }
    }
    
    if (!loaded) {
        qDebug() << "Failed to load point cloud image from all attempted paths";
    }
}

PointCloudgl::~PointCloudgl()
{
}

void PointCloudgl::SetupFont()
{
    // 字体将在paintGL中根据窗口尺寸动态调整
    text_font.setPointSize(OpenGLDisplay::NO_SIGNAL_BASE_FONT_SIZE);
}

void PointCloudgl::SetShowNoSignal(bool show)
{
    show_no_signal = show;
    update();
}

void PointCloudgl::SetCustomText(const QString &text)
{
    custom_text = text;
    update();
}

void PointCloudgl::initializeGL()
{
    initializeOpenGLFunctions();
    
    // 设置清除颜色
    glClearColor(OpenGLDisplay::CLEAR_COLOR_R, OpenGLDisplay::CLEAR_COLOR_G, 
                 OpenGLDisplay::CLEAR_COLOR_B, OpenGLDisplay::CLEAR_COLOR_A);
    
    // 启用深度测试
    glEnable(GL_DEPTH_TEST);
    
    // 启用混合
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void PointCloudgl::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // 使用QPainter绘制边框和文本
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 绘制圆角边框
    DrawRoundedBorder(painter);
    
    if (show_no_signal) {
        RenderNoSignalText(painter);
    } else {
        RenderPointCloudContent();
    }
}

void PointCloudgl::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

void PointCloudgl::RenderNoSignalText(QPainter& painter)
{
    // 根据窗口尺寸动态调整字体
    int fontSize = OpenGLDisplay::calculateFontSize(width(), height());
    int fontWeight = OpenGLDisplay::calculateFontWeight(width(), height());
    
    text_font.setPointSize(fontSize);
    text_font.setWeight(static_cast<QFont::Weight>(fontWeight));
    painter.setFont(text_font);
    
    QString displayText = custom_text.isEmpty() ? OpenGLDisplay::NO_SIGNAL_TEXT : custom_text;
    
    // 计算文字尺寸
    QFontMetrics fontMetrics(text_font);
    QRect textBounds = fontMetrics.boundingRect(displayText);
    
    // 计算图片尺寸和位置
    QPixmap scaledImage;
    int imageHeight = 0;
    int imageSpacing = 0;
    
    if (!no_signal_image.isNull()) {
        // 图片最大尺寸为窗口最小边的1/4
        int maxImageSize = qMin(width(), height()) / 4;
        scaledImage = no_signal_image.scaled(maxImageSize, maxImageSize, 
                                            Qt::KeepAspectRatio, Qt::SmoothTransformation);
        imageHeight = scaledImage.height();
        imageSpacing = OpenGLDisplay::IMAGE_TEXT_SPACING;
    }
    
    // 计算总高度（图片 + 间距 + 文字）
    int totalHeight = imageHeight + imageSpacing + textBounds.height();
    
    // 计算起始Y位置（垂直居中）
    int startY = (height() - totalHeight) / 2;
    
    // 绘制图片（水平居中）
    if (!scaledImage.isNull()) {
        int imageX = (width() - scaledImage.width()) / 2;
        painter.drawPixmap(imageX, startY, scaledImage);
    }
    
    // 绘制文字，水平居中，在图片下方
    int textY = startY + imageHeight + imageSpacing;
    QRect textRect(0, textY, width(), textBounds.height());
    
    // 设置彩虹渐变
    QLinearGradient rainbowGradient = OpenGLDisplay::createRainbowGradient(textRect);
    painter.setPen(QPen(QBrush(rainbowGradient), 1));
    
    painter.drawText(textRect, Qt::AlignCenter, displayText);
}

void PointCloudgl::RenderPointCloudContent()
{
    // 这里将来会添加点云的OpenGL渲染内容
    // 例如：3D点云数据、点云处理结果等
}

void PointCloudgl::DrawRoundedBorder(QPainter& painter)
{
    // 设置边框画笔
    QPen borderPen(OpenGLDisplay::BORDER_COLOR, OpenGLDisplay::BORDER_WIDTH);
    painter.setPen(borderPen);
    painter.setBrush(Qt::NoBrush);
    
    // 边框绘制在窗口内部，使用窗口的窗体矩形
    QRect borderRect = rect();
    
    // 绘制圆角矩形边框
    painter.drawRoundedRect(borderRect, OpenGLDisplay::CORNER_RADIUS, OpenGLDisplay::CORNER_RADIUS);
}