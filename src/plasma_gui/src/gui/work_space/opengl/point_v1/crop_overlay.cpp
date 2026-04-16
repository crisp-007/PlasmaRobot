#include "crop_overlay.h"
#include <QStyle>
#include <QFrame>

// ==========================================
// 构造函数与析构
// ==========================================

CropOverlayWidget::CropOverlayWidget(QWidget *parent) : QWidget(parent) {
    setupUi();
}

// ==========================================
// UI 初始化与设置
// ==========================================

void CropOverlayWidget::setupUi() {
    // 设置样式：半透明黑色背景，圆角
    this->setStyleSheet(
        "QWidget { background-color: rgba(0, 0, 0, 150); border-radius: 5px; }"
        "QPushButton { border: none; color: white; padding: 5px; font-weight: bold; font-family: 'Segoe UI', Arial; font-size: 14px; }"
        "QPushButton:hover { background-color: rgba(255, 255, 255, 50); }"
        "QPushButton:checked { background-color: rgba(0, 120, 212, 150); color: #87CEFA; }"
    );

    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->setContentsMargins(10, 5, 10, 5);
    layout->setSpacing(15);

    // --- 模式选择组 (矩形/多边形) ---
    btnRect = new QPushButton("⬚ 矩形", this);
    btnRect->setCheckable(true);
    btnRect->setChecked(true);
    
    btnPoly = new QPushButton("⬠ 多边形", this);
    btnPoly->setCheckable(true);

    QButtonGroup *modeGroup = new QButtonGroup(this);
    modeGroup->addButton(btnRect, 0);
    modeGroup->addButton(btnPoly, 1);
    modeGroup->setExclusive(true);

    // --- 类型选择组 (保留内部/保留外部) ---
    btnIn = new QPushButton("保留内部", this);
    btnIn->setCheckable(true);
    btnIn->setChecked(true);
    
    btnOut = new QPushButton("保留外部", this);
    btnOut->setCheckable(true);

    QButtonGroup *typeGroup = new QButtonGroup(this);
    typeGroup->addButton(btnIn, 0);
    typeGroup->addButton(btnOut, 1);
    typeGroup->setExclusive(true);

    // --- 操作按钮 (确认/取消) ---
    btnConfirm = new QPushButton("✔", this);
    btnConfirm->setToolTip("确认裁剪");
    btnConfirm->setStyleSheet("QPushButton { color: #90EE90; font-size: 16px; } QPushButton:hover { background-color: rgba(144, 238, 144, 50); }");
    
    btnCancel = new QPushButton("✘", this);
    btnCancel->setToolTip("退出裁剪");
    btnCancel->setStyleSheet("QPushButton { color: #FF6347; font-size: 16px; } QPushButton:hover { background-color: rgba(255, 99, 71, 50); }");

    // --- 添加到布局 ---
    layout->addWidget(btnRect);
    layout->addWidget(btnPoly);
    
    // 垂直分隔线 1
    QFrame* line1 = new QFrame();
    line1->setFrameShape(QFrame::VLine);
    line1->setFrameShadow(QFrame::Sunken);
    line1->setStyleSheet("background-color: rgba(255, 255, 255, 100);");
    layout->addWidget(line1);

    layout->addWidget(btnIn);
    layout->addWidget(btnOut);

    // 垂直分隔线 2
    QFrame* line2 = new QFrame();
    line2->setFrameShape(QFrame::VLine);
    line2->setFrameShadow(QFrame::Sunken);
    line2->setStyleSheet("background-color: rgba(255, 255, 255, 100);");
    layout->addWidget(line2);

    layout->addWidget(btnConfirm);
    layout->addWidget(btnCancel);

    // --- 信号连接 ---
    connect(modeGroup, &QButtonGroup::idClicked, this, &CropOverlayWidget::modeChanged);
    connect(typeGroup, &QButtonGroup::idClicked, [this](int id){ emit typeChanged(id == 0); });
    connect(btnConfirm, &QPushButton::clicked, this, &CropOverlayWidget::confirmClicked);
    connect(btnCancel, &QPushButton::clicked, this, &CropOverlayWidget::cancelClicked);
}
