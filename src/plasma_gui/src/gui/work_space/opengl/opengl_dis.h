#ifndef OPENGL_DIS_H
#define OPENGL_DIS_H

#include <QString>
#include <QColor>
#include <QFont>
#include <QLinearGradient>
#include <QPainter>

// OpenGL显示相关的公共配置
namespace OpenGLDisplay {
    // "No Signal"文本配置
    const QString NO_SIGNAL_TEXT = "No Signal";
    const int NO_SIGNAL_BASE_FONT_SIZE = 12;
    
    // 图片路径配置
    const QString MAIN_IMAGE_PATH = "../../../image/brain.png";
    const QString ROBOT_ARM_IMAGE_PATH = "../../../image/robot_logo.png";
    const QString POINT_CLOUD_IMAGE_PATH = "../../../image/cam.png";
    
    // 图片显示配置
    const int IMAGE_TEXT_SPACING = 10; // 图片和文字之间的间距
    
    // 背景颜色配置
    const QColor BACKGROUND_COLOR = QColor(30, 30, 30); // 深灰色背景
    
    // 边框配置
    const QColor BORDER_COLOR = QColor(74, 74, 74); // #4a4a4a
    const int BORDER_WIDTH = 8;
    const int CORNER_RADIUS = 8;
    
    // OpenGL渲染配置
    const float CLEAR_COLOR_R = 0.12f; // 30/255
    const float CLEAR_COLOR_G = 0.12f; // 30/255
    const float CLEAR_COLOR_B = 0.12f; // 30/255
    const float CLEAR_COLOR_A = 1.0f;
    
    // 字体粗度计算函数
    inline int calculateFontWeight(int windowWidth, int windowHeight) {
        int area = windowWidth * windowHeight;
        if (area > 500000) return QFont::Black;      // 最大窗口，最粗字体
        else if (area > 200000) return QFont::ExtraBold;
        else if (area > 100000) return QFont::Bold;
        else return QFont::DemiBold;
    }
    
    // 字体大小计算函数
    inline int calculateFontSize(int windowWidth, int windowHeight) {
        int minDimension = qMin(windowWidth, windowHeight);
        return qMax(12, minDimension / 15); // 根据最小尺寸计算字体大小
    }
    
    // 创建彩虹渐变函数
    inline QLinearGradient createRainbowGradient(const QRect& rect) {
        QLinearGradient gradient(rect.topLeft(), rect.bottomRight()); // 斜向渐变
        gradient.setColorAt(0.0, QColor(255, 140, 0));   // 深橙色
        gradient.setColorAt(0.2, QColor(255, 165, 0));   // 橙色
        gradient.setColorAt(0.4, QColor(30, 144, 255));  // 道奇蓝
        gradient.setColorAt(0.6, QColor(138, 43, 226));  // 蓝紫色
        gradient.setColorAt(0.8, QColor(199, 21, 133));  // 深粉色
        gradient.setColorAt(1.0, QColor(255, 20, 147));  // 深粉红
        return gradient;
    }
}

#endif // OPENGL_DIS_H
