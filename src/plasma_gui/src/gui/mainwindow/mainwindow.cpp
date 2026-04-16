#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QPainter>
#include <QPixmap>
#include <QTransform>
#include <QGraphicsDropShadowEffect>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QTimer>
#include <QOperatingSystemVersion>
#include <QShowEvent>
#include <QEvent>
#include <QApplication>
#include <random>
#include <chrono>
#include <QFileDialog>
#include <QFileInfo>

#ifdef Q_OS_WIN
#include <windows.h>
#include <dwmapi.h>
#endif

// =========初始化与生命周期===========
/** 主窗口构造：初始化UI、子控件、信号连接、仿真与提示 */
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), 
    ui(new Ui::MainWindow), 
    flow_display_widget(nullptr), 
    he_progress_bar(nullptr), 
    ar_progress_bar(nullptr), 
    processing_time_widget(nullptr), 
    serial_window(nullptr), 
    if_plasma_running(false), 
    tip_manager(nullptr)
{
    ui->setupUi(this);

    // 暗色标题栏将在窗口显示后设置

    SetupProcessingTimeWidget();
    SetupGasStatusDisplay();
    SetupSimulation();

    // 初始化串口窗口（作为独立窗口）
    serial_window = new Serial(this);

    // 连接串口按钮的点击事件
    connect(ui->serialPortButton, &QPushButton::clicked, this, &MainWindow::OnSerialPortButtonClicked);

    // 连接串口状态改变信号
    connect(serial_window, &Serial::connectionStatusChanged, this, &MainWindow::OnSerialConnectionStatusChanged);

    // 连接点云交互按钮
    connect(ui->btnCrop, &QPushButton::toggled, this, &MainWindow::OnCropToggled);
    // Move button removed
    connect(ui->btnLabel, &QPushButton::toggled, this, &MainWindow::OnLabelToggled);
    connect(ui->btnSelectFrame, &QPushButton::toggled, this, &MainWindow::OnSelectFrameToggled);

    // 连接等离子控制相关的信号槽
    SetupPlasmaControlConnections();

    // 初始化等离子状态为离线
    UpdatePlasmaConnectionStatus(false);
    
    
    // 初始化提示框管理器
    tip_manager = new TipManager(this);
    
    //延迟显示欢迎提示框
    QTimer::singleShot(1600, this, &MainWindow::ShowWelcomeTip);

    // 启动时优先加载本地点云文件，便于离线调试与演示
    const QString defaultPcdPath = "/home/larusxu/frame_right_650.pcd";
    if (ui->main_gl && QFileInfo::exists(defaultPcdPath)) {
        ui->main_gl->SetRosPaused(true);
        ui->btnSelectFrame->setChecked(true);
        ui->main_gl->LoadPCDPoint(defaultPcdPath);
        if (tip_manager) {
            tip_manager->showTip(QString("已加载点云: %1").arg(defaultPcdPath), TipWidget::Succeed);
        }
    }
}

/** 析构：关闭并释放子窗口与UI资源 */
MainWindow::~MainWindow()
{
    // 清理资源并释放内存
    if (serial_window)
    {
        serial_window->close();
        delete serial_window;
        serial_window = nullptr;
    }

    delete ui;
}

 

// =========UI相关===========
/** 初始化加工计时控件：从UI占位转换或替换为组件实例 */
void MainWindow::SetupProcessingTimeWidget()
{
    // 直接使用 UI 文件中的 processingTimeWidget
    QWidget *ui_widget = ui->processingTimeWidget;
    if (ui_widget)
    {
        // 将 UI widget 转换为 ProcessingTimeWidget 类型
        processing_time_widget = qobject_cast<ProcessingTimeWidget *>(ui_widget);
        if (!processing_time_widget)
        {
            // 如果转换失败，说明 UI 中的 widget 不是 ProcessingTimeWidget 类型
            // 如果需要创建一个新的ProcessingTimeWidget来替换
            processing_time_widget = new ProcessingTimeWidget(ui_widget->parentWidget());

            // 为新建的 UI widget 设置属性
            processing_time_widget->setObjectName("processingTimeWidget");
            processing_time_widget->setMinimumSize(ui_widget->minimumSize());
            processing_time_widget->setMaximumSize(ui_widget->maximumSize());
            processing_time_widget->setStyleSheet(ui_widget->styleSheet());

            // 获取父布局并替换widget
            QVBoxLayout *parent_layout = qobject_cast<QVBoxLayout *>(ui_widget->parentWidget()->layout());
            if (parent_layout)
            {
                int index = parent_layout->indexOf(ui_widget);
                if (index >= 0)
                {
                    parent_layout->removeWidget(ui_widget);
                    parent_layout->insertWidget(index, processing_time_widget);
                    ui_widget->setParent(nullptr);
                    ui_widget->deleteLater();
                }
            }
        }

        qDebug() << "Processing time widget setup successfully";
    }
    else
    {
        qDebug() << "Failed to find processingTimeWidget in UI file";
    }

    // //添加点击事件以通过点击开始/停止计时
    // if (processing_time_widget)
    // {
    //     connect(processing_time_widget, &ProcessingTimeWidget::clicked, this, &MainWindow::OnProcessingTimeToggle);

    //     // 自动开始计时（可选项）
    //     QTimer::singleShot(2000, [this]()
    //                        {
    //         if (processing_time_widget) {
    //             processing_time_widget->StartTiming();
    //             qDebug() << "Processing time started automatically";
    //         } });
    // }
}

/** 切换加工计时：在运行与停止状态之间切换 */
void MainWindow::OnProcessingTimeToggle()
{
    if (!processing_time_widget)
        return;

    if (processing_time_widget->IsRunning())
    {
        processing_time_widget->StopTiming();
        qDebug() << "Processing time stopped by user";
    }
    else
    {
        processing_time_widget->StartTiming();
        qDebug() << "Processing time started by user";
    }
}

/**
 * @brief 串口按钮点击事件处理函数
 *
 * 当用户点击主界面中的串口按钮时，显示串口设置窗口。
 * 窗口采用非模态方式显示，允许用户同时操作主界面和串口窗口。
 */
// =========串口相关===========
void MainWindow::OnSerialPortButtonClicked()
{
    if (!serial_window)
    {
        // 如果串口窗口不存在，创建一个新的
        serial_window = new Serial(this);

        // 连接压力值更新信号
        connect(serial_window, &Serial::pressureValuesUpdated,
                this, &MainWindow::OnPressureValuesUpdated);

        // 连接串口连接状态变化信号
        connect(serial_window, &Serial::connectionStatusChanged,
                this, &MainWindow::OnSerialConnectionStatusChanged);

        // 连接气体当前值变化信号
        connect(serial_window, &Serial::heliumCurrentValueChanged, this, &MainWindow::OnHeliumCurrentValueChanged);
        connect(serial_window, &Serial::argonCurrentValueChanged, this, &MainWindow::OnArgonCurrentValueChanged);
    }

    // 以非模态方式显示串口窗口
    serial_window->show();
    serial_window->raise();          // 将窗口置于前台
    serial_window->activateWindow(); // 激活窗口
}

// =========UI相关===========
/** 窗口显示事件：首次显示时应用暗色标题栏 */
void MainWindow::showEvent(QShowEvent *event)
{
    QMainWindow::showEvent(event);
    // 窗口首次显示时设置暗色标题栏
    static bool first_show = true;
    if (first_show)
    {
        SetupDarkTitleBar();
        first_show = false;
    }
}

/** 在Windows 10及以上启用暗色标题栏效果 */
void MainWindow::SetupDarkTitleBar()
{
#ifdef Q_OS_WIN
    if (QOperatingSystemVersion::current() >= QOperatingSystemVersion::Windows10)
    {
        HWND hwnd = (HWND)winId();
        BOOL value = TRUE;
        // 使用较浅的暗色模式
        ::DwmSetWindowAttribute(hwnd, 19, &value, sizeof(value)); // DWMWA_USE_IMMERSIVE_DARK_MODE_BEFORE_20H1
    }
#endif
}

// =========等离子相关===========
/** 连接等离子控制相关的UI信号与初始化联动逻辑 */
void MainWindow::SetupPlasmaControlConnections()
{
    // 等离子启动按钮
    connect(ui->plasma_start_btn, &QPushButton::toggled, this, &MainWindow::OnPlasmaEnableToggled);

    // 气体模式控制
    connect(ui->gasModeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::OnGasModeChanged);
    

    // 混合气体流量复选框联动
    connect(ui->blendGasEnableCheckBox, &QCheckBox::toggled, this, &MainWindow::OnBlendGasFlowCheckBoxToggled);

    // 气体比例控制
    connect(ui->argonRatioSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::OnArgonRatioChanged);
    connect(ui->heliumRatioSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::OnHeliumRatioChanged);


    connect(ui->singleGasComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::OnSingleGasChanged);
    // 氩气
    // 氩气滑动条和数值框同步
    connect(ui->argonFlowSlider, &QSlider::valueChanged, this, [this](int value)
            {
        ui->argonFlowSpinBox->blockSignals(true);
        ui->argonFlowSpinBox->setValue(value * 0.01);
        OnArgonFlowChanged(ui->argonFlowSpinBox->value());
        ui->argonFlowSpinBox->blockSignals(false); });
    connect(ui->argonFlowSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value)
            {
        ui->argonFlowSlider->blockSignals(true);
        ui->argonFlowSlider->setValue(static_cast<int>(value*100));
        ui->argonFlowSlider->blockSignals(false); });

    // 氩气->串口更新
    connect(ui->argonEnableCheckBox, &QCheckBox::toggled, this, &MainWindow::OnArgonEnableToggled);
    connect(ui->argonFlowSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::OnArgonFlowChanged);

    // 氦气
    // 氦气滑动条和数值框同步
    connect(ui->heliumFlowSlider, &QSlider::valueChanged, this, [this](int value)
            {
        ui->heliumFlowSpinBox->blockSignals(true);
        ui->heliumFlowSpinBox->setValue(value * 0.01);
        OnHeliumFlowChanged(ui->heliumFlowSpinBox->value());
        ui->heliumFlowSpinBox->blockSignals(false); });
    connect(ui->heliumFlowSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value)
            {
        ui->heliumFlowSlider->blockSignals(true);
        ui->heliumFlowSlider->setValue(static_cast<int>(value * 100));
        ui->heliumFlowSlider->blockSignals(false); });
    // 串口更新
    connect(ui->heliumEnableCheckBox, &QCheckBox::toggled, this, &MainWindow::OnHeliumEnableToggled);
    connect(ui->heliumFlowSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::OnHeliumFlowChanged);

    // 混合气体滑动条和数值框同步
    connect(ui->blendGasFlowSlider, &QSlider::valueChanged, this, [this](int value)
            {
        ui->blendGasFlowSpinBox->blockSignals(true);
        ui->blendGasFlowSpinBox->setValue(value * 0.1);
        ui->blendGasFlowSpinBox->blockSignals(false); });
    connect(ui->blendGasFlowSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value)
            {
        ui->blendGasFlowSlider->blockSignals(true);
        qDebug() << "Blend gas flow spin box value changed to " << value;
        ui->blendGasFlowSlider->setValue(static_cast<int>(value * 10));
        ui->blendGasFlowSlider->blockSignals(false); });
    // 混合气->串口更新
    connect(ui->blendGasEnableCheckBox, &QCheckBox::toggled, this, &MainWindow::OnBlendGasEnableToggled);
    connect(ui->blendGasFlowSlider, &QSlider::valueChanged, this, &MainWindow::OnBlendGasFlowChanged);
    connect(ui->blendGasFlowSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::OnBlendGasFlowChanged);

    // 初始化气体模式
    OnGasModeChanged(ui->gasModeComboBox->currentIndex());

    // 初始化气体比例
    OnArgonRatioChanged(ui->argonRatioSpinBox->value());
    OnSingleGasChanged(ui->singleGasComboBox->currentIndex());

    // 紧急停止按钮
    connect(ui->emergencyStopButton1, &QPushButton::clicked, this, &MainWindow::OnEmergencyStopClicked);

    // 为ComboBox控件安装事件过滤器以禁用滚轮事件
    ui->gasModeComboBox->installEventFilter(this);
    ui->singleGasComboBox->installEventFilter(this);

    // 初始化滑动条和数值框的同步
    ui->argonFlowSlider->setValue(static_cast<int>(ui->argonFlowSpinBox->value() * 100));
    ui->heliumFlowSlider->setValue(static_cast<int>(ui->heliumFlowSpinBox->value() * 100));
    ui->blendGasFlowSlider->setValue(static_cast<int>(ui->blendGasFlowSpinBox->value() * 10));

    // 连接串口窗口的气体当前值变化信号
    if (serial_window) {
        connect(serial_window, &Serial::heliumCurrentValueChanged, this, &MainWindow::OnHeliumCurrentValueChanged);
        connect(serial_window, &Serial::argonCurrentValueChanged, this, &MainWindow::OnArgonCurrentValueChanged);
    }
}

/** 启停等离子：校验气体条件，联动串口与UI状态 */
void MainWindow::OnPlasmaEnableToggled(bool enabled)
{
    // 检测气体流量条件
    if (enabled)
    {
        // 检查氦气和氩气checkbox的选中状态
        bool heliumChecked = ui->heliumEnableCheckBox->isChecked();
        bool argonChecked = ui->argonEnableCheckBox->isChecked();

        // 检查对应slider的值是否大于0
        bool heliumFlowValid = ui->heliumFlowSlider->value() > 0;
        bool argonFlowValid = ui->argonFlowSlider->value() > 0;

        // 在单气体模式下，如果只选中了一个checkbox，检查其对应的slider值
        if (ui->gasModeComboBox->currentIndex() == 0) // 单气体模式
        {

            // 检查氦气和氩气checkbox的选中状态
            bool heliumChecked = ui->heliumEnableCheckBox->isChecked();
            bool argonChecked = ui->argonEnableCheckBox->isChecked();
            // 检查对应slider的值是否大于0
            bool heliumFlowValid = ui->heliumFlowSlider->value() > 0;
            bool argonFlowValid = ui->argonFlowSlider->value() > 0;

            // 根据选择的气体自动设置对应的复选框为选中状态
            if (ui->singleGasComboBox->currentIndex() == 0) // 选择了氩气
            {
                if (!argonFlowValid)
                {
                    // 阻断信号，防止触发其他槽函数
                    ui->plasma_start_btn->blockSignals(true);
                    ui->plasma_start_btn->setChecked(false); // 重置按钮状态
                    ui->plasma_start_btn->blockSignals(false); // 恢复信号
                    // MessageBoxW(NULL, L"请设置氩气流量", L"错误", MB_OK | MB_ICONERROR);
                    if (tip_manager) {
                        tip_manager->showTip("请设置氩气流量！", TipWidget::Warning);
                    }
                    return; // 阻止启动
                }
                ui->argonEnableCheckBox->setChecked(true);
            }
            else if (ui->singleGasComboBox->currentIndex() == 1) // 选择了氦气
            {
                if (!heliumFlowValid)
                {
                    // 阻断信号，防止触发其他槽函数
                    ui->plasma_start_btn->blockSignals(true);
                    ui->plasma_start_btn->setChecked(false); // 重置按钮状态
                    ui->plasma_start_btn->blockSignals(false); // 恢复信号
                    // MessageBoxW(NULL, L"请设置氦气流量", L"错误", MB_OK | MB_ICONERROR);
                    if (tip_manager) {
                        tip_manager->showTip("请设置氦气流量！", TipWidget::Warning);
                    }
                    return; // 阻止启动
                }
                ui->heliumEnableCheckBox->setChecked(true);
            }
            // 如果两个checkbox都没有选中，则无法启动等离子
            // if (!heliumChecked && !argonChecked)
            // {
            //     ui->plasma_start_btn->setChecked(false); // 重置按钮状态
            //     MessageBoxW(NULL, L"请至少选择一个激发气体", L"错误", MB_OK | MB_ICONERROR);
            //     return; // 阻止启动
            // }
        }
        else
        {
            // 混合气体模式
            // 检查混合气体流量滑动条的值是否大于0
            bool blendGasFlowValid = ui->blendGasFlowSlider->value() > 0;
            // 如果混合气体流量为0，则无法启动
            if (!blendGasFlowValid)
            {
                // 阻断信号，防止触发其他槽函数
                ui->plasma_start_btn->blockSignals(true);
                ui->plasma_start_btn->setChecked(false); // 重置按钮状态
                ui->plasma_start_btn->blockSignals(false); // 恢复信号
                // MessageBoxW(NULL, L"请设置混合气体流量", L"错误", MB_OK | MB_ICONERROR);
                if (tip_manager) {
                    tip_manager->showTip("请设置混合气体流量！", TipWidget::Warning);
                }
                return; // 阻止启动
            }

            // 如果选中的checkbox对应的slider值都小于等于0，则无法启动
            // 检查比例值
            int argonRatio = ui->argonRatioSpinBox->value();
            int heliumRatio = ui->heliumRatioSpinBox->value();
            if (argonRatio + heliumRatio != 100)
            {
                // 阻断信号，防止触发其他槽函数
                ui->plasma_start_btn->blockSignals(true);
                ui->plasma_start_btn->setChecked(false); // 重置按钮状态
                ui->plasma_start_btn->blockSignals(false); // 恢复信号
                // MessageBoxW(NULL, L"请设置正确的气体比例", L"错误", MB_OK | MB_ICONERROR);
                if (tip_manager) {
                    tip_manager->showTip("请设置正确的气体比例！", TipWidget::Warning);
                }
                return; // 阻止启动
            }

            // 检查对应slider的值是否大于0
            bool heliumFlowValid = ui->heliumFlowSlider->value() > 0;
            bool argonFlowValid = ui->argonFlowSlider->value() > 0;
            if ((heliumRatio > 0 && !heliumFlowValid) || (argonRatio > 0 && !argonFlowValid))
            {
                // 阻断信号，防止触发其他槽函数
                ui->plasma_start_btn->blockSignals(true);
                ui->plasma_start_btn->setChecked(false); // 重置按钮状态
                ui->plasma_start_btn->blockSignals(false); // 恢复信号
                // MessageBoxW(NULL, L"请设置气体流量", L"错误", MB_OK | MB_ICONERROR);
                if (tip_manager) {
                    tip_manager->showTip("请设置气体流量！", TipWidget::Warning);
                }
                return; // 阻止启动
            }

            ui->blendGasEnableCheckBox->setChecked(true);
        }
    }

    if (enabled)
    {
        ui->plasma_start_btn->setText("停止等离子");
        if_plasma_running = true;  // 设置等离子运行标志位为true
        
        // 禁用气体模式和单气体类型选择
        ui->gasModeComboBox->setEnabled(false);
        ui->singleGasComboBox->setEnabled(false);
        
        // 设置提示信息
        // ui->gasModeComboBox->setToolTip("等离子运行时无法切换气体模式");
        // ui->singleGasComboBox->setToolTip("等离子运行时无法切换气体类型");

        // 联动串口窗口：打开等离子电源继电器和调压器继电器
        if (serial_window)
        {
            emit ui->argonFlowSlider->valueChanged(ui->argonFlowSlider->value());
            emit ui->heliumFlowSlider->valueChanged(ui->heliumFlowSlider->value());

            serial_window->setPlasmaControl(true, true, false);

            processing_time_widget->ResetTiming();
            processing_time_widget->StartTiming();
            flow_display_widget->StartAnimation();
            
            
        }
        
        // 显示等离子启动成功提示
        if (tip_manager) {
            tip_manager->showTip("等离子处理已启动！", TipWidget::Succeed);
        }
    }
    else
    {
        ui->plasma_start_btn->setText("启动等离子");
        if_plasma_running = false;  // 设置等离子运行标志位为false
        
        // 恢复气体模式和单气体类型选择
        ui->gasModeComboBox->setEnabled(true);
        ui->singleGasComboBox->setEnabled(true);
        
        // 清除提示信息
        ui->gasModeComboBox->setToolTip("");
        ui->singleGasComboBox->setToolTip("");
        ui->plasmaStateValue->setText("离线");
        ui->plasmaStateValue->setStyleSheet("QLabel {\n    background-color: #4a4a4a;\n    border: 1px solid #666666;\n    border-radius: 4px;\n    padding: 5px;\n    color: #ff4444;\n    font-weight: bold;\n}");

        ui->plasmaVoltageValue->setText("0.0");
        // 停止 设置各气体流量为0
        //  ui->heliumFlowSlider->setValue(0);
        //  ui->argonFlowSlider->setValue(0);
        //  ui->blendGasFlowSlider->setValue(0);

        // 联动串口窗口：关闭等离子电源继电器和调压器继电器
        if (serial_window)
        {
            serial_window->setPlasmaControl(false, false, true);
            flow_display_widget->StopAnimation();
            processing_time_widget->StopTiming();
        }
        
        // 显示等离子停止提示
        if (tip_manager) {
            tip_manager->showTip("等离子处理已停止！", TipWidget::Succeed);
        }
    }
}

// =========等离子相关===========
void MainWindow::OnEmergencyStopClicked()
{
    // 紧急停止所有操作
    ui->plasma_start_btn->setChecked(false);
    ui->argonEnableCheckBox->setChecked(false);
    ui->heliumEnableCheckBox->setChecked(false);

    // 重置所有流量为0
    ui->argonFlowSlider->setValue(0);
    ui->heliumFlowSlider->setValue(0);

    // 通过串口发送紧急停止命令
    if (serial_window)
    {
        serial_window->emergencyStop();
    }

    qDebug() << "Emergency stop activated!";
}
// =========气体相关===========
/** 切换氩气启用状态并同步串口输出 */
void MainWindow::OnArgonEnableToggled(bool enabled)
{
    // 移除滑动条的启用/禁用控制，允许在未选中checkbox时也能拖动滑动条
    // ui->argonFlowSlider->setEnabled(enabled);
    // ui->argonFlowSpinBox->setEnabled(enabled);

    if (serial_window)
    {
        if (enabled)
        {
            // 打开氩气流量计和电磁阀，并设置当前流量值
            double flowValue = ui->argonFlowSpinBox->value();
            int outputValue = static_cast<int>(ConvertArgonFlowToOutputValue(flowValue));
            serial_window->setArgonControl(true, true, outputValue);
        }
        else
        {
            // 关闭氩气流量计和电磁阀
            serial_window->setArgonControl(false, false, 0);
        }
    }
}

/** 氩气流量变化时（启用状态）更新串口输出值 */
void MainWindow::OnArgonFlowChanged(double value)
{
    // 同步滑块和数值框的值已经通过信号槽自动处理
    if (serial_window && ui->argonEnableCheckBox->isChecked())
    {
        // 如果氩气已启用，更新串口窗口中的输出值
        int outputValue = static_cast<int>(ConvertArgonFlowToOutputValue(value));
        serial_window->setArgonControl(true, true, outputValue);
    }
}

/** 切换氦气启用状态并同步串口输出 */
void MainWindow::OnHeliumEnableToggled(bool enabled)
{
    // 移除滑动条的启用/禁用控制，允许在未选中checkbox时也能拖动滑动条
    // ui->heliumFlowSlider->setEnabled(enabled);
    // ui->heliumFlowSpinBox->setEnabled(enabled);

    if (serial_window)
    {
        if (enabled)
        {
            // 打开氦气流量计和电磁阀，并设置当前流量值
            double flowValue = ui->heliumFlowSpinBox->value();
            int outputValue = static_cast<int>(ConvertHeliumFlowToOutputValue(flowValue));
            serial_window->setHeliumControl(true, true, outputValue);
        }
        else
        {
            // 关闭氦气流量计和电磁阀
            serial_window->setHeliumControl(false, false, 0);
        }
    }
}

/** 氦气流量变化时（启用状态）更新串口输出值 */
void MainWindow::OnHeliumFlowChanged(double value)
{
    // 同步滑块和数值框的值已经通过信号槽自动处理
    if (serial_window && ui->heliumEnableCheckBox->isChecked())
    {
        // 如果氦气已启用，更新串口窗口中的输出值
        int outputValue = static_cast<int>(ConvertHeliumFlowToOutputValue(value));
        serial_window->setHeliumControl(true, true, outputValue);
    }
}

 

/** 切换气体模式（单气体/混合）并调整控件可见性与启用 */
void MainWindow::OnGasModeChanged(int index)
{
    if (index == 0)
    { // 单气体模式
        ui->singleGasComboBox->setVisible(true);
        ui->gasRatioLabel->setVisible(false);
        ui->argonRatioLabel->setVisible(false);
        ui->argonRatioSpinBox->setVisible(false);
        ui->heliumRatioLabel->setVisible(false);
        ui->heliumRatioSpinBox->setVisible(false);

        // 隐藏混合气体控件
        ui->blendGasEnableCheckBox->setVisible(false);
        ui->blendGasFlowSlider->setVisible(false);
        ui->blendGasFlowSpinBox->setVisible(false);

        // 根据选择的单气体类型设置控制状态
        OnSingleGasChanged(ui->singleGasComboBox->currentIndex());
    }
    else
    { // 混合模式
        ui->singleGasComboBox->setVisible(false);
        ui->gasRatioLabel->setVisible(true);
        ui->argonRatioLabel->setVisible(true);
        ui->argonRatioSpinBox->setVisible(true);
        ui->heliumRatioLabel->setVisible(true);
        ui->heliumRatioSpinBox->setVisible(true);

        // 显示混合气体控件
        ui->blendGasEnableCheckBox->setVisible(true);
        ui->blendGasFlowSlider->setVisible(true);
        ui->blendGasFlowSpinBox->setVisible(true);

        // 启用所有气体控制
        ui->argonEnableCheckBox->setEnabled(true);
        ui->argonFlowSlider->setEnabled(true);
        ui->argonFlowSpinBox->setEnabled(true);
        ui->heliumEnableCheckBox->setEnabled(true);
        ui->heliumFlowSlider->setEnabled(true);
        ui->heliumFlowSpinBox->setEnabled(true);

        // 恢复所有控件的正常样式
        ui->argonEnableCheckBox->setStyleSheet("");
        ui->argonFlowSpinBox->setStyleSheet("border-radius:5px;");
        ui->heliumEnableCheckBox->setStyleSheet("");
        ui->heliumFlowSpinBox->setStyleSheet("border-radius:5px;");

        // 初始化气体比例
        OnArgonRatioChanged(ui->argonRatioSpinBox->value());
        
    }
}

/** 单气体模式下切换类型（氩/氦），联动启用与比例显示 */
void MainWindow::OnSingleGasChanged(int index)
{
    if (ui->gasModeComboBox->currentIndex() == 0)
    { // 只在单气体模式下生效
        if (index == 0)
        { // 选择氩气
            // 启用氩气控制，禁用氦气控制
            ui->argonEnableCheckBox->setEnabled(true);
            ui->argonFlowSlider->setEnabled(true);
            ui->argonFlowSpinBox->setEnabled(true);

            // 恢复氩气控件的正常样式
            ui->argonEnableCheckBox->setStyleSheet("");
            ui->argonFlowSpinBox->setStyleSheet("border-radius:5px;");

            //更新比例显示
            ui->ar_ratio_lab->setText(QString("比例: %1 %").arg(100));
            ui->he_ratio_lab->setText(QString("比例: %1 %").arg(0));
            //禁用氦气
            ui->heliumEnableCheckBox->setChecked(false);
            ui->heliumEnableCheckBox->setEnabled(false);
            ui->heliumFlowSlider->setValue(0);
            ui->heliumFlowSlider->setEnabled(false);
            ui->heliumFlowSpinBox->setValue(0);
            ui->heliumFlowSpinBox->setEnabled(false);

            // 设置氦气控件为灰色样式
            ui->heliumEnableCheckBox->setStyleSheet("QCheckBox { color: #888888; }");
            ui->heliumFlowSpinBox->setStyleSheet("QDoubleSpinBox { background-color: #3a3a3a; color: #888888; border: 1px solid #555555; border-radius: 5px; }");
        }
        else
        { // 选择氦气
            // 启用氦气控制，禁用氩气控制
            ui->heliumEnableCheckBox->setEnabled(true);
            ui->heliumFlowSlider->setEnabled(true);
            ui->heliumFlowSpinBox->setEnabled(true);

            // 恢复氦气控件的正常样式
            ui->heliumEnableCheckBox->setStyleSheet("");
            ui->heliumFlowSpinBox->setStyleSheet("border-radius:5px;");

            //更新比例显示
            ui->ar_ratio_lab->setText(QString("比例: %1 %").arg(0));
            ui->he_ratio_lab->setText(QString("比例: %1 %").arg(100));
            //禁用氩气
            ui->argonEnableCheckBox->setChecked(false);
            ui->argonEnableCheckBox->setEnabled(false);
            ui->argonFlowSlider->setValue(0);
            ui->argonFlowSlider->setEnabled(false);
            ui->argonFlowSpinBox->setValue(0);
            ui->argonFlowSpinBox->setEnabled(false);

            // 设置氩气控件为灰色样式
            ui->argonEnableCheckBox->setStyleSheet("QCheckBox { color: #888888; }");
            ui->argonFlowSpinBox->setStyleSheet("QDoubleSpinBox { background-color: #3a3a3a; color: #888888; border: 1px solid #555555; border-radius: 5px; }");
        }
    }
}

/** 调整氩气比例，保持总和为100%，并在混合模式联动流量 */
void MainWindow::OnArgonRatioChanged(int value)
{
    // 自动调整氦气比例，确保总和为100%
    int heliumRatio = 100 - value;
    ui->heliumRatioSpinBox->blockSignals(true);
    ui->heliumRatioSpinBox->setValue(heliumRatio);
    ui->heliumRatioSpinBox->blockSignals(false);

    //更新状态显示
    ui->ar_ratio_lab->setText(QString("比例: %1 %").arg(value));
    ui->he_ratio_lab->setText(QString("比例: %1 %").arg(heliumRatio));
    // 标签颜色已在UI文件中设置为固定颜色

    // 如果是混合模式，根据比例更新各气体流量
    if (ui->gasModeComboBox->currentIndex() == 1)
    { // 混合模式
        double blend_flow = ui->blendGasFlowSlider->value() * 10.00;
        double argon_flow = blend_flow * value / 100.00;
        double helium_flow = blend_flow - helium_flow;

        // 只更新滑动条，数值框会通过信号槽自动更新
        int blend_flow_int = ui->blendGasFlowSlider->value() * 10;
        ui->argonFlowSlider->setValue(static_cast<int>(argon_flow));
        ui->heliumFlowSlider->setValue(blend_flow_int - static_cast<int>(argon_flow));
    }
}

/** 调整氦气比例，保持总和为100%，并在混合模式联动流量 */
void MainWindow::OnHeliumRatioChanged(int value)
{
    // 自动调整氩气比例，确保总和为100%
    int argonRatio = 100 - value;
    ui->argonRatioSpinBox->blockSignals(true);
    ui->argonRatioSpinBox->setValue(argonRatio);
    ui->argonRatioSpinBox->blockSignals(false);

    // 更新显示
    //更新状态显示
    ui->ar_ratio_lab->setText(QString("比例: %1 %").arg(argonRatio));
    ui->he_ratio_lab->setText(QString("比例: %1 %").arg(value));

    // 标签颜色已在UI文件中设置为固定颜色

    // 如果是混合模式，根据比例更新各气体流量
    if (ui->gasModeComboBox->currentIndex() == 1)
    { // 混合模式
        double blend_flow = ui->blendGasFlowSlider->value() * 10.00;
        double helium_flow = blend_flow * value / 100.00;
        double argon_flow = blend_flow - helium_flow;

        // 只更新滑动条，数值框会通过信号槽自动更新
        int blend_flow_int = ui->blendGasFlowSlider->value() * 10;
        ui->heliumFlowSlider->setValue(static_cast<int>(helium_flow));
        ui->argonFlowSlider->setValue(blend_flow_int - static_cast<int>(helium_flow));
    }
}

/** 混合气体启用状态切换（当前保留控件状态，不禁用滑条） */
void MainWindow::OnBlendGasEnableToggled(bool enabled)
{
    // 混合气体启用状态改变
    // 注释掉滑动条的启用/禁用控制，允许在未选中checkbox时也能拖动滑动条
    // ui->blendGasFlowSlider->setEnabled(enabled);
    // ui->blendGasFlowSpinBox->setEnabled(enabled);
}

/** 混合气体总流量变化时按比例分配氩/氦流量并更新滑条 */
void MainWindow::OnBlendGasFlowChanged(double value)
{
    // 如果是混合模式，根据比例更新各气体流量
    if (ui->gasModeComboBox->currentIndex() == 1)
    { // 混合模式
        int blend_flow = ui->blendGasFlowSlider->value() * 10;
        int helium_ratio = ui->heliumRatioSpinBox->value();
        int argon_ratio = ui->argonRatioSpinBox->value();

        double helium_flow = blend_flow * helium_ratio / 100.0;
        double argon_flow = blend_flow * argon_ratio / 100.0;

        // 只更新滑动条，数值框会通过信号槽自动更新
        ui->heliumFlowSlider->setValue(static_cast<int>(helium_flow));
        ui->argonFlowSlider->setValue(blend_flow - static_cast<int>(helium_flow));
        qDebug() << "Blend Flow:" << blend_flow;
        qDebug("argon slider: %d helium: %d", blend_flow - static_cast<int>(helium_flow), static_cast<int>(helium_flow));
    }
}

// =========气体相关===========
/** 初始化气体状态区域：替换占位控件并设置相关控件的初始值 */
void MainWindow::SetupGasStatusDisplay()
{
    //控件：混合模式右侧动画显示+流量文字显示
    flow_display_widget = new FlowDisplayWidget(ui->gasStatusWidget);
    flow_display_widget->SetFlowValue(0.0);

    QVBoxLayout *gasStatusLayout = qobject_cast<QVBoxLayout *>(ui->gasStatusWidget->layout());
    if (gasStatusLayout && gasStatusLayout->count() > 0)
    {
        QLayoutItem *firstItem = gasStatusLayout->itemAt(0);
        if (firstItem && firstItem->layout())
        {
            QHBoxLayout *topLayout = qobject_cast<QHBoxLayout *>(firstItem->layout());
            if (topLayout)
            {
                for (int i = 0; i < topLayout->count(); ++i)
                {
                    QLayoutItem *item = topLayout->itemAt(i);
                    if (item && item->widget() && item->widget()->objectName() == "flowDisplayPlaceholder")
                    {
                        topLayout->replaceWidget(item->widget(), flow_display_widget);
                        item->widget()->setParent(nullptr);
                        break;
                    }
                }
            }
        }
    }
    //控件：气体进度条
    he_progress_bar = new GasProgressWidget(ui->gasStatusWidget);
    he_progress_bar->SetPercentage(50.0);

    ar_progress_bar = new GasProgressWidget(ui->gasStatusWidget);
    ar_progress_bar->SetPercentage(50.0);

    QHBoxLayout *heLayout = qobject_cast<QHBoxLayout *>(ui->he_gas_widget->layout());
    if (heLayout)
    {
        for (int i = 0; i < heLayout->count(); ++i)
        {
            QLayoutItem *item = heLayout->itemAt(i);
            if (item && item->widget() && item->widget()->objectName() == "heProgressWidget")
            {
                heLayout->replaceWidget(item->widget(), he_progress_bar);
                item->widget()->setParent(nullptr);
                break;
            }
        }
    }

    QHBoxLayout *arLayout = qobject_cast<QHBoxLayout *>(ui->ar_gas_widget->layout());
    if (arLayout)
    {
        for (int i = 0; i < arLayout->count(); ++i)
        {
            QLayoutItem *item = arLayout->itemAt(i);
            if (item && item->widget() && item->widget()->objectName() == "arProgressWidget")
            {
                arLayout->replaceWidget(item->widget(), ar_progress_bar);
                item->widget()->setParent(nullptr);
                break;
            }
        }
    }

    ui->gasModeLabel->setText("混合模式");

    ui->he_pressure_lab->setText("压力: 12.5 MPa");
    ui->he_volume_lab->setText("余量: 75 %");
    ui->he_flowrate_lab->setText("流量: 8.2 L/min");
    ui->he_ratio_lab->setText("比例: 53 %");

    ui->ar_pressure_lab->setText("压力: 8.3 MPa");
    ui->ar_volume_lab->setText("余量: 45 %");
    ui->ar_flowrate_lab->setText("流量: 7.3 L/min");
    ui->ar_ratio_lab->setText("比例: 47 %");
}

/** 启动仿真定时器：周期性更新流量与气体状态 */
void MainWindow::SetupSimulation()
{
    simulation_timer = new QTimer(this);
    connect(simulation_timer, &QTimer::timeout, this, &MainWindow::SimulateFlowDataChange);
    simulation_timer->start(2000);

    gas_status_timer = new QTimer(this);
    connect(gas_status_timer, &QTimer::timeout, this, &MainWindow::SimulateGasStatusChange);
    gas_status_timer->start(3000);
}

/** 仿真流量值变化：随机间隔更新流量显示与动画 */
void MainWindow::SimulateFlowDataChange()
{
    if (!flow_display_widget)
        return;

    static int counter = 0;
    QStringList flow_values = {
        "12.3 L/min", "18.7 L/min", "15.2 L/min", "21.4 L/min",
        "14.8 L/min", "19.6 L/min", "16.9 L/min", "22.1 L/min",
        "13.5 L/min", "17.3 L/min", "20.8 L/min", "15.9 L/min"};

    QString new_flow_value = flow_values[counter % flow_values.size()];
    counter++;

    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(1500, 3500);
    int next_interval = dis(gen);
    simulation_timer->start(next_interval);
}

/** 仿真气体状态变化：随机更新压力、余量与流量并刷新进度条 */
void MainWindow::SimulateGasStatusChange()
{
    if (!he_progress_bar || !ar_progress_bar)
        return;

    static std::random_device rd;
    static std::mt19937 gen(rd());

    static std::uniform_real_distribution<> he_pressure_dis(11.0, 14.0);
    static std::uniform_int_distribution<> he_volume_dis(65, 85);
    static std::uniform_real_distribution<> he_flow_dis(7.0, 10.0);

    static std::uniform_real_distribution<> ar_pressure_dis(7.5, 9.5);
    static std::uniform_int_distribution<> ar_volume_dis(35, 55);
    static std::uniform_real_distribution<> ar_flow_dis(6.0, 8.5);

    double he_pressure = he_pressure_dis(gen);
    int he_volume = he_volume_dis(gen);
    double he_flow = he_flow_dis(gen);
    int he_ratio = 100 - (he_volume * 100 / (he_volume + ar_volume_dis(gen)));

    ui->he_pressure_lab->setText(QString("压力: %1 MPa").arg(he_pressure, 0, 'f', 1));
    ui->he_volume_lab->setText(QString("余量: %1 %").arg(he_volume));

    he_progress_bar->SetPercentage(static_cast<double>(he_volume));

    double ar_pressure = ar_pressure_dis(gen);
    int ar_volume = ar_volume_dis(gen);
    double ar_flow = ar_flow_dis(gen);
    int ar_ratio = 100 - he_ratio;

    ui->ar_pressure_lab->setText(QString("压力: %1 MPa").arg(ar_pressure, 0, 'f', 1));
    ui->ar_volume_lab->setText(QString("余量: %1 %").arg(ar_volume));

    ar_progress_bar->SetPercentage(static_cast<double>(ar_volume));

    static std::uniform_int_distribution<> interval_dis(2000, 5000);
    int next_interval = interval_dis(gen);
    gas_status_timer->start(next_interval);
}
// =========事件过滤===========
/** 过滤ComboBox滚轮与点击事件；等离子运行时禁止模式/气体切换 */
bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    // 禁用ComboBox控件的滚轮事件，但将事件传递给父控件以保持页面滚动
    if ((obj == ui->gasModeComboBox || obj == ui->singleGasComboBox) && event->type() == QEvent::Wheel)
    {
        // 将滚轮事件传递给父控件
        QWidget *parent = qobject_cast<QWidget *>(obj)->parentWidget();
        if (parent)
        {
            QApplication::sendEvent(parent, event);
        }
        return true; // 阻止ComboBox处理滚轮事件
    }
    
    // 检查等离子运行状态下的ComboBox点击事件
    if ((obj == ui->gasModeComboBox || obj == ui->singleGasComboBox) && 
        event->type() == QEvent::MouseButtonPress && if_plasma_running)
    {
        // 根据控件类型显示相应的提示信息
        if (obj == ui->gasModeComboBox)
        {
            if (tip_manager) {
                tip_manager->showTip("等离子运行时无法切换气体模式！", TipWidget::Warning);
            }
        }
        else if (obj == ui->singleGasComboBox)
        {
            if (tip_manager) {
                tip_manager->showTip("等离子运行时无法切换气体类型！", TipWidget::Warning);
            }
        }
        return true; // 阻止ComboBox处理点击事件
    }
    
    return QMainWindow::eventFilter(obj, event);
}

// =========等离子相关===========
/** 串口连接状态变化时，更新等离子在线/离线显示 */
void MainWindow::OnSerialConnectionStatusChanged(bool connected)
{
    UpdatePlasmaConnectionStatus(connected);
}

/** 根据串口连接状态更新等离子状态标签与样式 */
void MainWindow::UpdatePlasmaConnectionStatus(bool connected)
{
    if (connected)
    {
        ui->plasmaStateValue->setText("在线");
        ui->plasmaStateValue->setStyleSheet("QLabel {\n    background-color: #4a4a4a;\n    border: 1px solid #666666;\n    border-radius: 4px;\n    padding: 5px;\n    color: #00ff00;\n    font-weight: bold;\n}");
    }
    else
    {
        ui->plasmaStateValue->setText("离线");
        ui->plasmaStateValue->setStyleSheet("QLabel {\n    background-color: #4a4a4a;\n    border: 1px solid #666666;\n    border-radius: 4px;\n    padding: 5px;\n    color: #ff4444;\n    font-weight: bold;\n}");
    }
}

// =========工具转换===========
/** 将氦气流量(0-30.0)转换为DAC输出(0-4096) */
double MainWindow::ConvertHeliumFlowToOutputValue(double flowValue)
{
    // 将氦气流量值(0-30.0)转换为输出值(0-4096)
    // 氦气最大流量30.0对应最大输出值4096
    return (flowValue / 30.0) * 5.0 / 3.3 * 4096.0;
}

/** 将氩气流量(0-10)转换为DAC输出(0-4096) */
double MainWindow::ConvertArgonFlowToOutputValue(double flowValue)
{
    // 将氩气流量值(0-10)转换为输出值(0-4096)
    // 氩气最大流量6.6对应最大输出值4096
    qDebug() << "传入值:" << flowValue << "氩气返回:" << (flowValue / 10) * 5.0 / 3.3 * 4096.0;
    return (flowValue / 10) * 5.0 / 3.3 * 4096.0;
}

/**
 * @brief 压力值更新槽函数
 * @param heliumPressure 氦气压力值(0-255)
 * @param argonPressure 氩气压力值(0-255)
 */
// =========串口相关===========
void MainWindow::OnPressureValuesUpdated(quint8 heliumPressure, quint8 argonPressure)
{
    // 这里可以将压力值显示在主界面上
    // 例如更新状态栏或者专门的压力显示控件

    // 输出调试信息
    qDebug() << "主窗口接收到压力值更新:";
    qDebug() << "  氦气压力:" << heliumPressure;
    qDebug() << "  氩气压力:" << argonPressure;

    // TODO: 在这里添加压力值的UI显示逻辑
    // 例如:
}

// =========气体相关===========
/** 混合流量复选框联动：勾选时启用氦/氩，取消时同时关闭 */
void MainWindow::OnBlendGasFlowCheckBoxToggled(bool checked)
{
    if (checked)
    {
        // 当混合气体流量复选框被选中时，同时选中氦气流量和氩气流量复选框
        ui->heliumEnableCheckBox->setChecked(true);
        ui->argonEnableCheckBox->setChecked(true);
    }
    else
    {
        // 当取消选中混合气体流量复选框时，同时取消氦气和氩气的选中状态
        ui->heliumEnableCheckBox->setChecked(false);
        ui->argonEnableCheckBox->setChecked(false);
    }
}

// =========串口相关===========
/** 串口回读氦气电流值，换算为流量并更新标签与总流量显示 */
void MainWindow::OnHeliumCurrentValueChanged(quint16 value)
{
    // 更新主窗口中的氦气流量标签
    double flow_value=static_cast<double>(value)/4096.0*3.3/5.0*30.0;
    ui->he_flowrate_lab->setText(QString("流量: %1 L/min").arg(flow_value, 0, 'f', 2));

    qint16 argon_current_value=serial_window->getArgonCurrentValue();
    double new_flow_value=static_cast<double>(argon_current_value)/4096.0*3.3/5.0*10.0;

    flow_display_widget->SetFlowValue(new_flow_value+flow_value);
}

/** 串口回读氩气电流值，换算为流量并更新标签与总流量显示 */
void MainWindow::OnArgonCurrentValueChanged(quint16 value)
{
    // 更新主窗口中的氩气流量标签
    double flow_value=static_cast<double>(value)/4096.0*3.3/5.0*10.0;
    ui->ar_flowrate_lab->setText(QString("流量: %1 L/min").arg(flow_value, 0, 'f', 2));

    qint16 helium_current_value=serial_window->getHeliumCurrentValue();
    double new_flow_value=static_cast<double>(helium_current_value)/4096.0*3.3/5.0*30.0;

    flow_display_widget->SetFlowValue(new_flow_value+flow_value);
}

// =========等离子相关===========
/** 获取等离子处理是否正在运行的状态标志 */
bool MainWindow::GetPlasmaDealState() const
{
    return if_plasma_running;
}

// =========提示管理===========
/** 显示欢迎使用的提示信息 */
void MainWindow::ShowWelcomeTip()
{
    if (tip_manager) {
        tip_manager->showTip("欢迎使用等离子处理系统！", TipWidget::Succeed);
    }
}


        
 

void MainWindow::OnCropToggled(bool checked)
{
    if (checked) {
        ui->btnLabel->setChecked(false);
        // 默认启用矩形裁剪
        if (ui->main_gl) {
            ui->main_gl->EnableCrop(true, MainOpengl::Crop_Rect);
        }
    } else {
        // 关闭裁剪
        if (ui->main_gl) {
            ui->main_gl->EnableCrop(false);
        }
    }
}

void MainWindow::OnLabelToggled(bool checked)
{
    if (checked) {
        ui->btnCrop->setChecked(false);
    }
    if (ui->main_gl) {
        ui->main_gl->EnableLabel(checked);
    }
}

/**
 * @brief 选帧/暂停按钮切换槽函数
 * @param checked true:暂停实时画面并保留当前帧; false:恢复实时画面
 */
void MainWindow::OnSelectFrameToggled(bool checked)
{
    if (checked) {
        if (ui->main_gl) {
            ui->main_gl->SetRosPaused(true);
        }
        if (tip_manager) {
                    tip_manager->showTip("已暂停实时画面，保留当前帧", TipWidget::Succeed);
                }
            } else {
                if (ui->main_gl) {
                    ui->main_gl->SetRosPaused(false);
                }
                if (tip_manager) {
                    tip_manager->showTip("恢复实时画面", TipWidget::Succeed);
                }
            }
}

