#ifndef CROP_OVERLAY_H
#define CROP_OVERLAY_H

#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QButtonGroup>

/**
 * @brief 裁剪工具条悬浮窗
 * 提供矩形/多边形切换，内部/外部保留切换，以及确认/取消功能
 */
class CropOverlayWidget : public QWidget {
    Q_OBJECT
public:
    explicit CropOverlayWidget(QWidget *parent = nullptr);

signals:
    // --- 信号定义 ---
    // 裁剪模式改变信号：0=矩形，1=多边形
    void modeChanged(int mode); 
    // 裁剪类型改变信号：true=保留内部，false=保留外部
    void typeChanged(bool inside); 
    // 确认裁剪点击信号
    void confirmClicked();
    // 取消裁剪点击信号
    void cancelClicked();

private:
    // --- 内部辅助函数 ---
    void setupUi();

    // --- UI 控件成员 ---
    QPushButton *btnRect;     // 矩形模式按钮
    QPushButton *btnPoly;     // 多边形模式按钮
    QPushButton *btnIn;       // 保留内部按钮
    QPushButton *btnOut;      // 保留外部按钮
    QPushButton *btnConfirm;  // 确认按钮
    QPushButton *btnCancel;   // 取消按钮
};

#endif
