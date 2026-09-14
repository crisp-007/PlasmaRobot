# MFC 模块配置与测试说明

## 1. 当前接入状态

当前 MFC 相关代码只放在两个文件中：

```text
plasma_test/Inc/mfc.h
plasma_test/Src/mfc.c
```

当前硬件通道状态：

| 通道 | 电源继电器 | DAC 设定 | ADC 反馈 | 当前状态 |
|---|---|---|---|---|
| Ar_MFC | PC2 / SWRAY1 | PA4 / DAC_OUT1 / FMC_DAC1 | PB0 / ADC12_IN8 / FMC_adc1 | 禁用，满量程 10 L/min |
| He_MFC | PC3 / SWRAY2 | PA5 / DAC_OUT2 / FMC_DAC2 | PB1 / ADC12_IN9 / FMC_adc2 | 有效，满量程 30 L/min |

Ar_MFC 当前因 PC2/SWRAY1 硬件接线问题暂时禁用。即使发送 Ar_MFC 上电命令，PC2 也不会吸合；`ar_valve` 仍可单独测试。

## 2. 最常修改的参数

配置位于 `plasma_test/Src/mfc.c` 的 `g_mfc[]` 数组。

最常改的是每个 MFC 的满量程：

```c
.flow_max_lpm = 10.0f
```

含义是该 MFC 的满量程流量，单位 `L/min`。

示例：

```text
10 L/min 的 MFC：flow_max_lpm = 10.0f
30 L/min 的 MFC：flow_max_lpm = 30.0f
```

软件不会把 10 或 30 写死在公式里，而是根据每路通道自己的 `flow_max_lpm` 自动计算 DAC 和反馈流量。

## 3. 关键模拟量参数

MFC 设定输入是 `0-5 V`，但 STM32 DAC 不是直接输出 5 V。本板 DAC 后面有 2 倍运放：

```text
MCU DAC 输出 0-2.5 V
外部运放放大 2 倍
MFC 接口得到 0-5 V
```

所以配置里：

```c
.external_gain = 2.0f
.mcu_dac_max_v = 2.5f
.interface_output_max_v = 5.0f
```

这三个不要随便改，除非硬件电路变化。

MFC 反馈输出也是 `0-5 V`，进入 MCU 前经过二分压：

```text
MFC 反馈 0-5 V
10k/10k 分压
MCU ADC 引脚 0-2.5 V
```

所以配置里：

```c
.divider_ratio = 0.5f
.mcu_adc_max_v = 2.5f
.interface_input_max_v = 5.0f
```

## 4. 流量命令单位

串口命令中的 MFC 流量字段单位是 `0.01 L/min`。

```text
100  = 1.00 L/min
500  = 5.00 L/min
1000 = 10.00 L/min
1500 = 15.00 L/min
3000 = 30.00 L/min
```

当前有效通道 He_MFC 使用：

```text
HeFLOWRelay bit 0x10  控制 He_MFC 上电
HeOutValue            设置 He_MFC 目标流量
```

Ar_MFC 当前禁用，对应字段仍保留：

```text
ArFLOWRelay bit 0x10
ArOutValue
```

## 5. 测试命令

### 5.1 He_MFC 上电，流量 0.00 L/min

```text
FF FE 00 00 00 00 10 00 00 00 00 00 0D 02 00 00
```

### 5.2 He_MFC 上电，同时打开氦气电磁阀，1-5 L/min

1.00 L/min：

```text
FF FE 00 00 00 00 11 64 00 00 00 00 72 02 00 00
```

2.00 L/min：

```text
FF FE 00 00 00 00 11 C8 00 00 00 00 D6 02 00 00
```

3.00 L/min：

```text
FF FE 00 00 00 00 11 2C 01 00 00 00 3B 02 00 00
```

4.00 L/min：

```text
FF FE 00 00 00 00 11 90 01 00 00 00 9F 02 00 00
```

5.00 L/min：

```text
FF FE 00 00 00 00 11 F4 01 00 00 00 03 03 00 00
```

### 5.3 He_MFC 只上电并设置 1.00 L/min

```text
FF FE 00 00 00 00 10 64 00 00 00 00 71 02 00 00
```

### 5.4 He_MFC 只上电并设置 5.00 L/min

```text
FF FE 00 00 00 00 10 F4 01 00 00 00 02 03 00 00
```

### 5.5 He_MFC 只上电并设置 10.00 L/min

```text
FF FE 00 00 00 00 10 E8 03 00 00 00 F8 02 00 00
```

### 5.6 He_MFC 只上电并设置 15.00 L/min

```text
FF FE 00 00 00 00 10 DC 05 00 00 00 EE 02 00 00
```

### 5.7 He_MFC 只上电并设置 30.00 L/min

```text
FF FE 00 00 00 00 10 B8 0B 00 00 00 D0 02 00 00
```

### 5.8 He_MFC 设置 15.00 L/min，同时打开氦气电磁阀

```text
FF FE 00 00 00 00 11 DC 05 00 00 00 EF 02 00 00
```

### 5.9 He_MFC 断电并清零

```text
FF FE 00 00 00 00 00 00 00 00 00 00 FD 01 00 00
```

### 5.10 尝试 Ar_MFC 设置 5.00 L/min

Ar_MFC 当前禁用，所以该命令不会吸合 PC2。

```text
FF FE 00 00 00 00 00 00 00 10 F4 01 02 03 00 00
```

### 5.11 尝试 Ar_MFC 设置 10.00 L/min

Ar_MFC 满量程是 10 L/min，但当前通道仍禁用，所以该命令也不会吸合 PC2。

```text
FF FE 00 00 00 00 00 00 00 10 E8 03 F8 02 00 00
```

### 5.12 急停测试用组合命令

除三色灯和蜂鸣器外，继电器、气阀、MFC 供电位和三路风扇全部打开，并把 He/Ar MFC 目标流量设为 1.00 L/min：

```text
FF FE 00 F1 00 00 11 64 00 11 64 00 D8 03 00 00
```

发送后按下硬件急停，应观察到气阀关闭、继电器关闭、MFC 目标流量回到 0.00 L/min；ADC、压力通讯、USART1 日志和 USART6 状态包继续运行。

## 6. 日志观察

发送命令后看串口日志中的 `Gas Path`：

```text
===Gas Path===
  He_MFC : ON  Ar_MFC : DISABLED
  He Valve: ON  Ar Valve: OFF
  He Target  : 5.00 L/min  He DAC Out : 1551
  He Feedback: 4.98 L/min  He ADC Input : 1235
  He Tracking: OK
```

重点看：

| 字段 | 含义 |
|---|---|
| `He_MFC` | 氦气 MFC 电源状态 |
| `Ar_MFC` | 氩气 MFC 电源状态；当前禁用 |
| `He Target` | 软件下发给 He_MFC 的目标流量 |
| `He Feedback` | ADC 反馈换算后的 He_MFC 实际流量 |
| `He DAC Out` | He_MFC 当前 DAC 原始码值 |
| `He ADC Input` | He_MFC 当前 ADC 原始采样值 |
| `He Tracking` | He_MFC 反馈是否跟踪目标 |

## 7. DAC 码值参考

当前 He_MFC 满量程为 `30 L/min`，接口设定为 `0-5 V`，DAC 外部增益为 2。

| 目标流量 | MFC 接口电压 | MCU DAC 电压 | DAC 码值约 |
|---:|---:|---:|---:|
| 0.00 L/min | 0.00 V | 0.00 V | 0 |
| 3.00 L/min | 0.50 V | 0.25 V | 310 |
| 15.00 L/min | 2.50 V | 1.25 V | 1551 |
| 30.00 L/min | 5.00 V | 2.50 V | 3102 |

Ar_MFC 满量程为 `10 L/min`，但当前禁用。后续启用后，设置 `5.00 L/min` 也是 50% 满量程，DAC 码值同样约为 `1551`。

## 8. 改 Ar_MFC 为可用

等 Ar_MFC 硬件修复后，在 `mfc.c` 中把 Ar_MFC 的：

```c
.channel_enable = MFC_CHANNEL_DISABLED
```

改成：

```c
.channel_enable = MFC_CHANNEL_ENABLED
```

然后确认：

```c
.flow_max_lpm
.power_active_level
```

是否和实际 MFC 型号、继电器极性一致。
