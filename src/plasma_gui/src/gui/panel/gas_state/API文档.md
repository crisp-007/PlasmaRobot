# 气体状态控件 API 文档

## 概述

本文档详细说明了气体状态显示系统中两个主要控件的封装、结构和使用方法：
- `FlowDisplayWidget` - 流量显示控件
- `GasProgressWidget` - 气体进度条控件

---

## FlowDisplayWidget 流量显示控件

### 概述
`FlowDisplayWidget` 继承自 `QWidget`，专门用于显示流量数据信息，支持动态扫描动画效果。

### 头文件位置
```cpp
#include "panel/gas_state/flowdisplaywidget.h"
```

### 类声明
```cpp
class FlowDisplayWidget : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(qreal scan_position READ ScanPosition WRITE SetScanPosition)

public:
    explicit FlowDisplayWidget(QWidget *parent = nullptr);
    
    // 基础接口
    void SetFlowValue(const QString &value);
    QString FlowValue() const;
    void StartScanAnimation();
    
    // 数据接口
    void SetFlowValue(double value, const QString &unit = "L/min");
    void SetFlowValueWithFormat(double value, const QString &format = "%.1f L/min");
    void SetTextColor(const QColor &color);
    void SetTextSize(int size);
    void SetTextStyle(const QColor &color, int size, bool bold = false);
    void SetBackgroundGradient(const QColor &startColor, const QColor &endColor);
    void SetBackgroundStyle(const QColor &startColor, const QColor &endColor, const QColor &borderColor);
    void SetAnimationDuration(int milliseconds);
    void SetAnimationEnabled(bool enabled);
    
    // 预设主题
    void ApplyBlueTheme();
    void ApplyGreenTheme();
    void ApplyOrangeTheme();
    void ApplyRedTheme();

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void OnAnimationFinished();

private:
    void SetupUI();
    void SetupAnimation();
    void UpdateLabelStyle();
    qreal ScanPosition() const;
    void SetScanPosition(qreal position);
};
```

### 成员变量
```cpp
private:
    QLabel *flow_label;                    // 流量显示标签
    QPropertyAnimation *scan_animation;    // 扫描动画
    QTimer *ripple_timer;                 // 波纹效果定时器
    qreal scan_position;                  // 扫描位置
    QList<QPointF> ripple_positions;      // 波纹位置列表
    
    // 样式参数
    bool animation_enabled;               // 动画开关
    QColor text_color;                   // 文本颜色
    int text_size;                       // 文本大小
    bool text_bold;                      // 文本粗体
    QColor bg_start_color;               // 背景起始颜色
    QColor bg_end_color;                 // 背景结束颜色
    QColor border_color;                 // 边框颜色
```

### 主要功能和函数

#### 1. 数据设置函数
```cpp
// 设置流量值（字符串）
void SetFlowValue(const QString &value);
// 示例：widget->SetFlowValue("15.5 L/min");

// 设置流量值（数值+单位）
void SetFlowValue(double value, const QString &unit = "L/min");
// 示例：widget->SetFlowValue(15.5, "L/min");

// 设置流量值（自定义格式）
void SetFlowValueWithFormat(double value, const QString &format = "%.1f L/min");
// 示例：widget->SetFlowValueWithFormat(15.5, "%.2f L/min");

// 启动扫描动画
void StartScanAnimation();
```

#### 2. 样式设置函数
```cpp
// 设置文本颜色
void SetTextColor(const QColor &color);
// 示例：widget->SetTextColor(QColor(255, 255, 255));

// 设置文本大小
void SetTextSize(int size);
// 示例：widget->SetTextSize(14);

// 一次设置文本样式
void SetTextStyle(const QColor &color, int size, bool bold = false);
// 示例：widget->SetTextStyle(QColor(255, 255, 255), 14, true);

// 设置背景渐变
void SetBackgroundGradient(const QColor &startColor, const QColor &endColor);
// 示例：widget->SetBackgroundGradient(QColor(0, 100, 200), QColor(0, 150, 255));

// 一次设置背景样式
void SetBackgroundStyle(const QColor &startColor, const QColor &endColor, const QColor &borderColor);
// 示例：widget->SetBackgroundStyle(QColor(0, 100, 200), QColor(0, 150, 255), QColor(100, 100, 100));
```

#### 3. 动画控制函数
```cpp
// 设置动画持续时间
void SetAnimationDuration(int milliseconds);
// 示例：widget->SetAnimationDuration(1500);

// 启用/禁用动画
void SetAnimationEnabled(bool enabled);
// 示例：widget->SetAnimationEnabled(true);
```

#### 4. 预设主题函数
```cpp
// 蓝色主题（适用于He气体）
void ApplyBlueTheme();

// 绿色主题（适用于正常运行状态）
void ApplyGreenTheme();

// 橙色主题（适用于警告状态）
void ApplyOrangeTheme();

// 红色主题（适用于错误状态）
void ApplyRedTheme();
```

---

## GasProgressWidget 气体进度条控件

### 概述
`GasProgressWidget` 继承自 `QWidget`，专门用于显示气体容量的垂直进度条，支持自动颜色变化和状态检测。

### 头文件位置
```cpp
#include "panel/gas_state/gasprogresswidget.h"
```

### 类声明
```cpp
class GasProgressWidget : public QWidget
{
    Q_OBJECT

public:
    explicit GasProgressWidget(QWidget *parent = nullptr);
    
    // 基础接口
    void SetValue(int value);
    void SetMaximum(int maximum);
    void SetMinimum(int minimum);
    void SetProgressColor(const QColor &color);
    void SetBackgroundColor(const QColor &color);
    void SetCornerRadius(int radius);
    
    // 数据接口
    void SetValueWithRange(int value, int min, int max);
    void SetPercentage(double percentage);
    double GetPercentage() const;
    void SetBorderColor(const QColor &color);
    void SetSize(int width, int height);
    void SetFullStyle(const QColor &progressColor, const QColor &bgColor, const QColor &borderColor, int radius);
    
    // 预设样式
    void ApplyHeGasStyle();
    void ApplyArGasStyle();
    void ApplyWarningStyle();
    void ApplyNormalStyle();
    
    // 气体状态管理
    void SetGasStatus(double pressure, int volume, double flowRate, int ratio);
    void SetLowVolumeWarning(bool enabled, int threshold = 20);
    bool IsLowVolume() const;
    bool IsEmpty() const;
    QString GetStatusText() const;
    QString GetVolumeStatusText() const;

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    void UpdateColorByValue();
};
```

### 成员变量
```cpp
private:
    int value;                           // 当前值
    int minimum;                         // 最小值
    int maximum;                         // 最大值
    int corner_radius;                   // 圆角半径
    QColor progress_color;               // 进度条颜色
    QColor background_color;             // 背景颜色
    QColor border_color;                 // 边框颜色
    
    // 扩展属性
    bool low_volume_warning_enabled;     // 低容量警告开关
    int low_volume_threshold;            // 低容量阈值
    double gas_pressure;                 // 气体压力
    double gas_flow_rate;               // 气体流量
    int gas_ratio;                      // 气体比例
```

### 主要功能和函数

#### 1. 数据设置函数
```cpp
// 设置当前值
void SetValue(int value);
// 示例：widget->SetValue(75);

// 设置最大值
void SetMaximum(int maximum);
// 示例：widget->SetMaximum(100);

// 设置最小值
void SetMinimum(int minimum);
// 示例：widget->SetMinimum(0);

// 同时设置值和范围
void SetValueWithRange(int value, int min, int max);
// 示例：widget->SetValueWithRange(75, 0, 100);
```

#### 2. 百分比操作函数
```cpp
// 设置百分比值
void SetPercentage(double percentage);
// 示例：widget->SetPercentage(75.0);

// 获取当前百分比
double GetPercentage() const;
// 示例：double percent = widget->GetPercentage();
```

#### 3. 样式设置函数
```cpp
// 设置进度条颜色
void SetProgressColor(const QColor &color);
// 示例：widget->SetProgressColor(QColor(33, 150, 243));

// 设置背景颜色
void SetBackgroundColor(const QColor &color);
// 示例：widget->SetBackgroundColor(QColor(51, 51, 51));

// 设置边框颜色
void SetBorderColor(const QColor &color);
// 示例：widget->SetBorderColor(QColor(85, 85, 85));

// 设置圆角半径
void SetCornerRadius(int radius);
// 示例：widget->SetCornerRadius(10);

// 设置控件尺寸
void SetSize(int width, int height);
// 示例：widget->SetSize(20, 100);

// 一次设置完整样式
void SetFullStyle(const QColor &progressColor, const QColor &bgColor, const QColor &borderColor, int radius);
// 示例：widget->SetFullStyle(QColor(33, 150, 243), QColor(51, 51, 51), QColor(85, 85, 85), 10);
```

#### 4. 预设样式函数
```cpp
// He气样式（蓝色）
void ApplyHeGasStyle();

// Ar气样式（绿色）
void ApplyArGasStyle();

// 警告样式（橙色）
void ApplyWarningStyle();

// 正常样式（绿色）
void ApplyNormalStyle();
```

#### 5. 气体状态管理函数
```cpp
// 一次设置气体状态
void SetGasStatus(double pressure, int volume, double flowRate, int ratio);
// 示例：widget->SetGasStatus(12.5, 75, 8.2, 53);
// 参数：压力(MPa), 容量(%), 流量(L/min), 比例(%)

// 设置低容量警告
void SetLowVolumeWarning(bool enabled, int threshold = 20);
// 示例：widget->SetLowVolumeWarning(true, 20); // 启用警告，阈值20%

// 检查是否为低容量
bool IsLowVolume() const;
// 示例：if (widget->IsLowVolume()) { /* 处理低容量 */ }

// 检查是否为空
bool IsEmpty() const;
// 示例：if (widget->IsEmpty()) { /* 处理空状态 */ }

// 获取状态文本
QString GetStatusText() const;
// 返回值：「空」、「低」、「中」、「高」、「满」等

// 获取容量状态文本
QString GetVolumeStatusText() const;
// 返回值：容量: 75% (正常)
```

---

## 使用示例

### 1. FlowDisplayWidget 使用示例

#### 基础使用
```cpp
// 创建控件
FlowDisplayWidget* flowWidget = new FlowDisplayWidget(parent);

// 设置流量值
flowWidget->SetFlowValue("15.5 L/min");
flowWidget->SetFlowValue(15.5, "L/min");

// 应用主题
flowWidget->ApplyBlueTheme();

// 启动动画
flowWidget->StartScanAnimation();
```

#### 自定义样式
```cpp
FlowDisplayWidget* customWidget = new FlowDisplayWidget(parent);

// 自定义样式
customWidget->SetTextStyle(QColor(255, 255, 255), 16, true);
customWidget->SetBackgroundStyle(
    QColor(0, 100, 200),    // 起始颜色
    QColor(0, 150, 255),    // 结束颜色
    QColor(100, 100, 100)   // 边框颜色
);
customWidget->SetAnimationDuration(2000);
customWidget->SetFlowValue(18.7, "L/min");
```

### 2. GasProgressWidget 使用示例

#### 基础使用
```cpp
// 创建He气体进度条
GasProgressWidget* heWidget = new GasProgressWidget(parent);
heWidget->ApplyHeGasStyle();
heWidget->SetGasStatus(12.5, 75, 8.2, 53);

// 创建Ar气体进度条
GasProgressWidget* arWidget = new GasProgressWidget(parent);
arWidget->ApplyArGasStyle();
arWidget->SetGasStatus(8.3, 45, 7.3, 47);
```

#### 状态检查
```cpp
GasProgressWidget* gasWidget = new GasProgressWidget(parent);
gasWidget->SetLowVolumeWarning(true, 20);
gasWidget->SetValue(15);

// 检查状态
if (gasWidget->IsLowVolume()) {
    qDebug() << "警告：" << gasWidget->GetVolumeStatusText();
    gasWidget->ApplyWarningStyle();
}

if (gasWidget->IsEmpty()) {
    qDebug() << "气体容量已耗尽";
    gasWidget->ApplyRedTheme();
}
```

### 3. 在MainWindow中的集成使用

```cpp
void MainWindow::SetupGasStatusDisplay()
{
    // 流量显示设置
    ui->flowDisplayWidget->ApplyBlueTheme();
    ui->flowDisplayWidget->SetFlowValue(15.5, "L/min");
    ui->flowDisplayWidget->SetAnimationDuration(1200);
    
    // He气体进度条设置
    ui->heProgressWidget->ApplyHeGasStyle();
    ui->heProgressWidget->SetGasStatus(12.5, 75, 8.2, 53);
    ui->heProgressWidget->SetLowVolumeWarning(true, 20);
    
    // Ar气体进度条设置
    ui->arProgressWidget->ApplyArGasStyle();
    ui->arProgressWidget->SetGasStatus(8.3, 45, 7.3, 47);
    ui->arProgressWidget->SetLowVolumeWarning(true, 15);
}

void MainWindow::UpdateGasStatus()
{
    // 动态更新
    double currentFlow = getCurrentFlowRate();
    ui->flowDisplayWidget->SetFlowValue(currentFlow, "L/min");
    
    // 更新He气状态
    ui->heProgressWidget->SetGasStatus(
        getHePressure(),
        getHeVolume(),
        getHeFlowRate(),
        getHeRatio()
    );
    
    // 状态检查
    if (ui->heProgressWidget->IsLowVolume()) {
        showWarning("He气容量不足：" + ui->heProgressWidget->GetVolumeStatusText());
    }
}
```

---

## 注意事项

1. **动画性能**：FlowDisplayWidget的动画会消耗一定的CPU资源，在性能敏感的场合下可以通过`SetAnimationEnabled(false)`关闭动画。

2. **颜色自动变化**：GasProgressWidget在设置百分比数值时，进度条会自动调整颜色，手动设置的颜色可能会被覆盖。

3. **线程安全**：这些控件应在主线程中使用，如果需要在其他线程中更新数据，请使用Qt的信号槽机制。

4. **内存管理**：控件会自动管理内部的动画和定时器对象，无需手动释放。

5. **样式继承**：预设主题会覆盖之前的自定义样式设置。

---

## 版本信息

- **版本**：1.0
- **创建日期**：2024年
- **兼容版本**：Qt 6.x
- **依赖**：QtWidgets, QtCore, QtGui