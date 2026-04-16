#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QLabel>
#include <QMovie>
#include <QOpenGLWidget>

#include "gasprogresswidget.h"
#include "flowdisplaywidget.h"
//opengl窗口
#include "roundedopenglwidget.h"//圆角opengl窗口嵌套
#include "main_gl.h"
#include "robotarm_gl.h"
// #include "pointcloud_gl.h"
#include "processing_time_widget.h"//处理时间
#include "serial.h"//串口
#include "tip_manager.h"//顶层提示窗

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    
    // 获取等离子处理状态
    bool GetPlasmaDealState() const;

private slots:
    void SimulateFlowDataChange();
    void SimulateGasStatusChange();
    void OnProcessingTimeToggle();
    void OnSerialPortButtonClicked();
    
    // 等离子控制相关槽函数
    void OnPlasmaEnableToggled(bool enabled);
    void OnGasModeChanged(int index);
    void OnSingleGasChanged(int index);
    void OnArgonRatioChanged(int value);
    void OnHeliumRatioChanged(int value);
    void OnArgonEnableToggled(bool enabled);
    void OnArgonFlowChanged(double value);
    void OnHeliumEnableToggled(bool enabled);
    void OnHeliumFlowChanged(double value);
    void OnBlendGasEnableToggled(bool enabled);
    void OnBlendGasFlowChanged(double value);
    void OnEmergencyStopClicked();
    
    // 压力值更新槽函数
    void OnPressureValuesUpdated(quint8 heliumPressure, quint8 argonPressure);
    void OnSerialConnectionStatusChanged(bool connected);
    
    // 气体当前值更新槽函数
    void OnHeliumCurrentValueChanged(quint16 heValue);
    void OnArgonCurrentValueChanged(quint16 arValue);
    
    // 混合气体流量复选框联动槽函数
    void OnBlendGasFlowCheckBoxToggled(bool checked);
    
    // 点云交互槽函数
    void OnCropToggled(bool checked);
    void OnLabelToggled(bool checked);
    void OnSelectFrameToggled(bool checked);

    // 测试提示框
    void ShowWelcomeTip();

protected:
    void showEvent(QShowEvent *event) override;
    bool eventFilter(QObject *obj, QEvent *event) override;

private:
    Ui::MainWindow *ui;
    QTimer *simulation_timer;
    QTimer *gas_status_timer;
    FlowDisplayWidget *flow_display_widget;
    GasProgressWidget *he_progress_bar;
    GasProgressWidget *ar_progress_bar;
    ProcessingTimeWidget *processing_time_widget;
    Serial *serial_window;
    bool if_plasma_running;  // 等离子运行状态标志位
    TipManager *tip_manager;   // 提示框管理器
    
    void SetupGasStatusDisplay();
    void SetupSimulation();
    void UpdateGasStatus();
    void SetupDarkTitleBar();
    void SetupProcessingTimeWidget();
    void SetupPlasmaControlConnections();
    void UpdatePlasmaConnectionStatus(bool connected);
    double ConvertHeliumFlowToOutputValue(double flowValue);
    double ConvertArgonFlowToOutputValue(double flowValue);
};
#endif // MAINWINDOW_H
