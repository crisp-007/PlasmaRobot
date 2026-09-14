#ifndef PATIENT_H
#define PATIENT_H

#include <QWidget>
#include <QLabel>
#include <QDialog>
#include <QLineEdit>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QTextEdit>
#include <QPushButton>
#include <QMouseEvent>

// ──────────────────────────────────────────────────────────────
//  PatientInfoDialog  —  患者详细信息填写对话框
// ──────────────────────────────────────────────────────────────
class PatientInfoDialog : public QDialog
{
    Q_OBJECT
public:
    explicit PatientInfoDialog(QWidget *parent = nullptr);

    // 从外部设置/获取表单数据
    struct PatientData {
        QString id;           ///< 患者编号
        QString name;         ///< 患者姓名
        QString gender;       ///< 性别
        QString age;          ///< 年龄
        QString tumorType;    ///< 肿瘤类型
        QString tumorGrade;   ///< 肿瘤级别
        QString tumorSize;    ///< 肿瘤直径(cm)
        QString tumorLocation;///< 肿瘤位置
        QString notes;        ///< 备注
    };

    void setData(const PatientData &data);
    PatientData getData() const;

signals:
    void dataConfirmed(const PatientData &data);

private slots:
    void onConfirm();
    void onCancel();

private:
    void setupUi();

    QLineEdit       *edit_id_;
    QLineEdit       *edit_name_;
    QComboBox       *combo_gender_;
    QLineEdit       *edit_age_;
    QComboBox       *combo_tumor_type_;
    QComboBox       *combo_tumor_grade_;
    QDoubleSpinBox  *spin_tumor_size_;
    QLineEdit       *edit_tumor_location_;
    QTextEdit       *edit_notes_;
    QPushButton     *btn_confirm_;
    QPushButton     *btn_cancel_;
};

// ──────────────────────────────────────────────────────────────
//  PatientCard  —  患者信息卡控件
//  左键单击 → 弹出填写对话框
//  右键单击 → 切换信息遮盖（****）
// ──────────────────────────────────────────────────────────────
class PatientCard : public QWidget
{
    Q_OBJECT
public:
    explicit PatientCard(QWidget *parent = nullptr);

    void setPatientData(const PatientInfoDialog::PatientData &data);
    const PatientInfoDialog::PatientData &patientData() const { return data_; }

protected:
    void mousePressEvent(QMouseEvent *event) override;

private slots:
    void onDataConfirmed(const PatientInfoDialog::PatientData &data);

private:
    void updateDisplay();

    QLabel  *name_label_ = nullptr;   ///< 右侧姓名
    QLabel  *id_label_   = nullptr;   ///< 右侧编号

    PatientInfoDialog            *dialog_;
    PatientInfoDialog::PatientData data_;
    bool masked_    = false;   ///< 是否遮盖信息
};

#endif // PATIENT_H
