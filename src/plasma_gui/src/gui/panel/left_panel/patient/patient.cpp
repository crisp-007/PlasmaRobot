#include "patient.h"

#include <QGroupBox>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QScrollArea>

// ══════════════════════════════════════════════════════════════
//  PatientInfoDialog  实现
// ══════════════════════════════════════════════════════════════

PatientInfoDialog::PatientInfoDialog(QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle("患者信息");
    setMinimumWidth(480);
    setModal(true);
    setupUi();
}

void PatientInfoDialog::setupUi()
{
    // ── 全局深色样式 ──────────────────────────────────────────
    setStyleSheet(R"(
        QDialog {
            background-color: #2b2b2b;
            color: #e0e0e0;
        }
        QLabel {
            color: #e0e0e0;
            font-size: 13px;
            background: transparent;
        }
        QLineEdit, QComboBox, QDoubleSpinBox, QTextEdit {
            background-color: #3c3c3c;
            border: 1px solid #555555;
            border-radius: 5px;
            color: #ffffff;
            padding: 5px;
            font-size: 13px;
        }
        QLineEdit:focus, QComboBox:focus,
        QDoubleSpinBox:focus, QTextEdit:focus {
            border: 1px solid #0078d4;
        }
        QComboBox::drop-down { border: none; }
        QComboBox::down-arrow {
            border-left:  5px solid transparent;
            border-right: 5px solid transparent;
            border-top:   5px solid #cccccc;
            width: 0; height: 0;
        }
        QComboBox QAbstractItemView {
            background-color: #3c3c3c;
            border: 1px solid #555;
            selection-background-color: #0078d4;
            color: #ffffff;
            outline: 0;
            padding: 0px;
        }
        QGroupBox {
            color: #aaaaaa;
            border: 1px solid #444444;
            border-radius: 6px;
            margin-top: 10px;
            font-size: 12px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 4px;
        }
        QPushButton {
            background-color: #3c3c3c;
            border: 1px solid #555555;
            border-radius: 5px;
            color: #ffffff;
            padding: 6px 20px;
            font-size: 13px;
        }
        QPushButton:hover  { background-color: #4a4a4a; }
        QPushButton:pressed{ background-color: #2a2a2a; }
        QPushButton#btn_confirm {
            background-color: #0078d4;
            border: none;
        }
        QPushButton#btn_confirm:hover  { background-color: #1088e4; }
        QPushButton#btn_confirm:pressed{ background-color: #005a9e; }
        QScrollArea { border: none; background: transparent; }
        QScrollBar:vertical {
            background: #3c3c3c; width: 6px; border-radius: 3px;
        }
        QScrollBar::handle:vertical {
            background: #666; border-radius: 3px; min-height: 20px;
        }
    )");

    // ── 输入控件 ──────────────────────────────────────────────
    edit_id_            = new QLineEdit(this);
    edit_name_          = new QLineEdit(this);
    combo_gender_       = new QComboBox(this);
    edit_age_           = new QLineEdit(this);
    combo_tumor_type_   = new QComboBox(this);
    combo_tumor_grade_  = new QComboBox(this);
    spin_tumor_size_    = new QDoubleSpinBox(this);
    edit_tumor_location_= new QLineEdit(this);
    edit_notes_         = new QTextEdit(this);

    edit_id_->setPlaceholderText("如：P-20240001");
    edit_name_->setPlaceholderText("填写患者姓名");
    edit_age_->setPlaceholderText("如：52");
    edit_tumor_location_->setPlaceholderText("如：右侧颞叶、额叶等");

    edit_notes_->setPlaceholderText("其他备注信息...");
    edit_notes_->setFixedHeight(70);

    combo_gender_->addItems({"男", "女"});

    combo_tumor_type_->addItems({
        "脑胶质瘤（Glioma）",
        "脑膜瘤（Meningioma）",
        "听神经瘤",
        "垂体腺瘤",
        "其他"
    });

    // WHO 分级
    combo_tumor_grade_->addItems({
        "WHO I 级（良性）",
        "WHO II 级（低级别）",
        "WHO III 级（间变性）",
        "WHO IV 级（高级别/GBM）",
        "未确定"
    });

    spin_tumor_size_->setRange(0.1, 20.0);
    spin_tumor_size_->setSingleStep(0.1);
    spin_tumor_size_->setDecimals(1);
    spin_tumor_size_->setSuffix(" cm");
    spin_tumor_size_->setValue(2.0);

    // ── 按钮 ─────────────────────────────────────────────────
    btn_confirm_ = new QPushButton("确 认", this);
    btn_cancel_  = new QPushButton("取 消", this);
    btn_confirm_->setObjectName("btn_confirm");

    // ── 布局 ─────────────────────────────────────────────────
    // 基本信息组
    QGroupBox   *grp_basic  = new QGroupBox("基本信息", this);
    QFormLayout *form_basic = new QFormLayout(grp_basic);
    form_basic->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    form_basic->setSpacing(8);
    form_basic->setContentsMargins(12, 14, 12, 12);
    form_basic->addRow("患者编号", edit_id_);
    form_basic->addRow("患者姓名", edit_name_);
    form_basic->addRow("性    别", combo_gender_);
    form_basic->addRow("年    龄", edit_age_);

    // 肿瘤信息组
    QGroupBox   *grp_tumor  = new QGroupBox("肿瘤信息", this);
    QFormLayout *form_tumor = new QFormLayout(grp_tumor);
    form_tumor->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    form_tumor->setSpacing(8);
    form_tumor->setContentsMargins(12, 14, 12, 12);
    form_tumor->addRow("肿瘤类型", combo_tumor_type_);
    form_tumor->addRow("WHO 级别", combo_tumor_grade_);
    form_tumor->addRow("最大直径", spin_tumor_size_);
    form_tumor->addRow("肿瘤位置", edit_tumor_location_);

    // 备注组
    QGroupBox  *grp_note  = new QGroupBox("备注", this);
    QVBoxLayout *lay_note = new QVBoxLayout(grp_note);
    lay_note->setContentsMargins(12, 14, 12, 12);
    lay_note->addWidget(edit_notes_);

    // 滚动容器（内容多时可滚动）
    QWidget     *scroll_content = new QWidget;
    QVBoxLayout *scroll_lay     = new QVBoxLayout(scroll_content);
    scroll_lay->setSpacing(10);
    scroll_lay->setContentsMargins(15, 15, 15, 15);
    scroll_lay->addWidget(grp_basic);
    scroll_lay->addWidget(grp_tumor);
    scroll_lay->addWidget(grp_note);
    scroll_lay->addStretch();

    QScrollArea *scroll = new QScrollArea(this);
    scroll->setWidget(scroll_content);
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    // 按钮行
    QHBoxLayout *btn_lay = new QHBoxLayout;
    btn_lay->addStretch();
    btn_lay->addWidget(btn_cancel_);
    btn_lay->addWidget(btn_confirm_);
    btn_lay->setContentsMargins(15, 0, 15, 15);
    btn_lay->setSpacing(10);

    // 主布局
    QVBoxLayout *main_lay = new QVBoxLayout(this);
    main_lay->setContentsMargins(0, 0, 0, 0);
    main_lay->setSpacing(0);
    main_lay->addWidget(scroll);
    main_lay->addLayout(btn_lay);

    connect(btn_confirm_, &QPushButton::clicked, this, &PatientInfoDialog::onConfirm);
    connect(btn_cancel_,  &QPushButton::clicked, this, &PatientInfoDialog::onCancel);
}

void PatientInfoDialog::setData(const PatientData &data)
{
    edit_id_->setText(data.id);
    edit_name_->setText(data.name);
    combo_gender_->setCurrentText(data.gender);
    edit_age_->setText(data.age);
    combo_tumor_type_->setCurrentText(data.tumorType);
    combo_tumor_grade_->setCurrentText(data.tumorGrade);
    spin_tumor_size_->setValue(data.tumorSize.toDouble());
    edit_tumor_location_->setText(data.tumorLocation);
    edit_notes_->setPlainText(data.notes);
}

PatientInfoDialog::PatientData PatientInfoDialog::getData() const
{
    PatientData d;
    d.id            = edit_id_->text().trimmed();
    d.name          = edit_name_->text().trimmed();
    d.gender        = combo_gender_->currentText();
    d.age           = edit_age_->text().trimmed();
    d.tumorType     = combo_tumor_type_->currentText();
    d.tumorGrade    = combo_tumor_grade_->currentText();
    d.tumorSize     = QString::number(spin_tumor_size_->value(), 'f', 1);
    d.tumorLocation = edit_tumor_location_->text().trimmed();
    d.notes         = edit_notes_->toPlainText().trimmed();
    return d;
}

void PatientInfoDialog::onConfirm()
{
    emit dataConfirmed(getData());
    accept();
}

void PatientInfoDialog::onCancel()
{
    reject();
}

// ══════════════════════════════════════════════════════════════
//  PatientCard  实现
// ══════════════════════════════════════════════════════════════

PatientCard::PatientCard(QWidget *parent)
    : QWidget(parent)
{
    setCursor(Qt::PointingHandCursor);
    // 注意：子控件在父 UI setupUi 时创建，构造时 findChild 会返回 nullptr
    name_label_ = findChild<QLabel*>("patient_name_label");
    id_label_   = findChild<QLabel*>("patient_id_label");

    dialog_ = new PatientInfoDialog(this);
    connect(dialog_, &PatientInfoDialog::dataConfirmed,
            this,    &PatientCard::onDataConfirmed);
}

void PatientCard::setPatientData(const PatientInfoDialog::PatientData &data)
{
    data_   = data;
    masked_ = false;
    updateDisplay();
}

void PatientCard::updateDisplay()
{
    // 延迟查找：构造时子控件尚未创建，首次显示前再次尝试
    if (!name_label_) name_label_ = findChild<QLabel*>("patient_name_label");
    if (!id_label_)   id_label_   = findChild<QLabel*>("patient_id_label");

    if (!name_label_ || !id_label_) return;   // 控件仍未就绪，暂不更新
    if (masked_) {
        name_label_->setText("***");
        id_label_->setText("***");
    } else {
        const QString name = data_.name.isEmpty() ? "— 未录入 —" : data_.name;
        const QString id   = data_.id.isEmpty()   ? "-编号-" : "编号：" + data_.id;
        name_label_->setText(name);
        id_label_->setText(id);
    }
}

void PatientCard::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        // 左键：弹出填写对话框
        dialog_->setData(data_);
        dialog_->exec();
    } else if (event->button() == Qt::RightButton) {
        // 右键：切换信息遮盖
        masked_ = !masked_;
        updateDisplay();
    }
    QWidget::mousePressEvent(event);
}

void PatientCard::onDataConfirmed(const PatientInfoDialog::PatientData &data)
{
    data_   = data;
    masked_ = false;
    updateDisplay();
}
