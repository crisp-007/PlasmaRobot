# STM32F407 双路 MFC 模拟量控制与反馈模块设计说明

> 当前 `plasma_test` 工程已经把旧命名 `MFC1/MFC2` 业务化为 `Ar_MFC/He_MFC`：  
> `Ar_MFC` 对应 PC2/SWRAY1、PA4、PB0，满量程 10 L/min，当前因硬件接线问题禁用；  
> `He_MFC` 对应 PC3/SWRAY2、PA5、PB1，满量程 30 L/min，当前有效。  
> 当前联调和测试命令请优先查看 `MFC模块配置与测试说明.md` 和 `当前代码引脚分配与项目背景.md`；本文主体保留了早期 MFC1/MFC2 设计资料，作为实现依据和历史记录。

## 1. 文档目的

本文档用于指导编程助手为 STM32F407VGT6 控制板实现双路 MFC（Mass Flow Controller，质量流量控制器）的底层驱动与业务接口。

系统中的两路 MFC 均采用模拟量接口：

- 流量设定输入：`0~5 V`
- 流量反馈输出：`0~5 V`
- 两路 MFC 的满量程流量可能不同
- 软件必须通过每个通道独立的 `full_scale_flow_lpm` 字段自动完成流量、电压、DAC 码值和 ADC 反馈之间的换算

示例：

- MFC1：`0~30 L/min`
- MFC2：`0~10 L/min`

软件不应将 `30 L/min` 写死，而应依据通道配置自动计算。

---

## 2. 硬件通道映射

| MFC 通道 | STM32 DAC 引脚 | 外部设定信号 | STM32 ADC 引脚 | 外部反馈信号 |
|---|---|---|---|---|
| MFC1 | PA4 / DAC_OUT1 | FMC_DAC1 | PB0 / ADC12_IN8 | FMC_adc1 |
| MFC2 | PA5 / DAC_OUT2 | FMC_DAC2 | PB1 / ADC12_IN9 | FMC_adc2 |

### 2.1 DAC 输出链路

```text
STM32 DAC
PA4 / PA5
    ↓ 0~约2.5 V
外部运放同相放大 2 倍
    ↓ 0~约5 V
FMC_DAC1 / FMC_DAC2
    ↓
MFC 模拟量设定输入
```

外部运放增益：

```text
Gain = 1 + Rf / Rg
     = 1 + 1k / 1k
     = 2
```

因此：

```text
MFC_SetVoltage = MCU_DAC_Voltage × 2
```

### 2.2 ADC 反馈链路

```text
MFC 模拟量反馈输出
    ↓ 0~5 V
FMC_adc1 / FMC_adc2
    ↓ 10k / 10k 分压
运放缓冲
    ↓ 0~2.5 V
PB0 / PB1 ADC 输入
```

分压比例：

```text
MCU_ADC_PinVoltage = MFC_FeedbackVoltage / 2
```

因此：

```text
MFC_FeedbackVoltage = MCU_ADC_PinVoltage × 2
```

---

## 3. MFC 线性换算关系

每个 MFC 都满足：

```text
0 V                    → 0 L/min
5 V                    → full_scale_flow_lpm
中间范围按线性比例换算
```

其中：

```c
full_scale_flow_lpm
```

表示该通道 MFC 的满量程流量，单位为 `L/min`。

例如：

```c
MFC1.full_scale_flow_lpm = 30.0f;
MFC2.full_scale_flow_lpm = 10.0f;
```

### 3.1 流量转 MFC 设定电压

```text
MFC_SetVoltage =
    TargetFlow / full_scale_flow_lpm × 5.0 V
```

### 3.2 MFC 反馈电压转流量

```text
FeedbackFlow =
    MFC_FeedbackVoltage / 5.0 V × full_scale_flow_lpm
```

### 3.3 示例

若 MFC1 满量程为 `30 L/min`：

| 流量 | 模拟量 |
|---:|---:|
| 0 L/min | 0 V |
| 6 L/min | 1 V |
| 15 L/min | 2.5 V |
| 24 L/min | 4 V |
| 30 L/min | 5 V |

若 MFC2 满量程为 `10 L/min`：

| 流量 | 模拟量 |
|---:|---:|
| 0 L/min | 0 V |
| 2 L/min | 1 V |
| 5 L/min | 2.5 V |
| 8 L/min | 4 V |
| 10 L/min | 5 V |

---

## 4. DAC 设定换算

### 4.1 DAC 电压关系

外部电路放大 2 倍，所以 MCU DAC 实际需要输出：

```text
MCU_DAC_Voltage = MFC_SetVoltage / 2
```

结合流量：

```text
MCU_DAC_Voltage =
    TargetFlow / full_scale_flow_lpm × 5.0 V / 2
```

即：

```text
MCU_DAC_Voltage =
    TargetFlow / full_scale_flow_lpm × 2.5 V
```

### 4.2 DAC 码值

STM32F407 内部 DAC 为 12 位：

```text
DAC 范围：0~4095
参考电压：VDDA，典型约 3.3 V
```

DAC 码值：

```text
DAC_Code =
    MCU_DAC_Voltage / VDDA × 4095
```

合并后：

```text
DAC_Code =
    TargetFlow / full_scale_flow_lpm
    × 5.0
    / DAC_ExternalGain
    / VDDA
    × 4095
```

在本板中：

```text
DAC_ExternalGain = 2.0
VDDA ≈ 3.3 V
```

满量程理论 DAC 码值约为：

```text
2.5 / 3.3 × 4095 ≈ 3102
```

因此软件不能把 MFC 满流量直接映射到 `4095`，而应映射到约 `3102`。

---

## 5. ADC 反馈换算

### 5.1 ADC 引脚电压

STM32F407 ADC 为 12 位：

```text
ADC_Raw：0~4095
```

ADC 引脚电压：

```text
MCU_ADC_PinVoltage =
    ADC_Raw / 4095 × VDDA
```

### 5.2 恢复 MFC 反馈电压

由于硬件做了二分压：

```text
MFC_FeedbackVoltage =
    MCU_ADC_PinVoltage × ADC_DividerRestoreGain
```

本板：

```text
ADC_DividerRestoreGain = 2.0
```

### 5.3 恢复实际流量

```text
FeedbackFlow =
    MFC_FeedbackVoltage / 5.0
    × full_scale_flow_lpm
```

合并：

```text
FeedbackFlow =
    ADC_Raw / 4095
    × VDDA
    × ADC_DividerRestoreGain
    / 5.0
    × full_scale_flow_lpm
```

---

## 6. 推荐数据结构

建议使用每通道独立配置结构体。

```c
typedef enum
{
    MFC_CHANNEL_1 = 0,
    MFC_CHANNEL_2,
    MFC_CHANNEL_COUNT
} mfc_channel_t;

typedef struct
{
    /* MFC 规格参数 */
    float full_scale_flow_lpm;       /* MFC 满量程流量，单位 L/min */
    float control_voltage_max_v;     /* 设定输入满量程电压，默认 5.0 V */
    float feedback_voltage_max_v;    /* 反馈输出满量程电压，默认 5.0 V */

    /* 控制板模拟链路参数 */
    float dac_external_gain;         /* DAC 外部放大倍数，本板为 2.0 */
    float adc_restore_gain;          /* ADC 分压恢复倍数，本板为 2.0 */
    float vdda_v;                     /* ADC/DAC 参考电压，初始可设 3.3 V */

    /* 标定参数 */
    float dac_gain_cal;
    float dac_offset_cal;
    float adc_gain_cal;
    float adc_offset_cal;

    /* 硬件通道 */
    uint32_t dac_channel;
    uint32_t adc_channel;
} mfc_config_t;

typedef struct
{
    float target_flow_lpm;
    float feedback_flow_lpm;

    float target_voltage_v;
    float feedback_voltage_v;

    uint16_t dac_code;
    uint16_t adc_raw;

    bool enabled;
    bool tracking_ok;
} mfc_status_t;

typedef struct
{
    mfc_config_t config;
    mfc_status_t status;
} mfc_device_t;
```

---

## 7. 推荐通道配置

以下数值仅为示例。实际项目中只需要修改 `full_scale_flow_lpm`。

```c
static mfc_device_t g_mfc[MFC_CHANNEL_COUNT] =
{
    [MFC_CHANNEL_1] =
    {
        .config =
        {
            .full_scale_flow_lpm    = 30.0f,
            .control_voltage_max_v  = 5.0f,
            .feedback_voltage_max_v = 5.0f,

            .dac_external_gain      = 2.0f,
            .adc_restore_gain       = 2.0f,
            .vdda_v                 = 3.3f,

            .dac_gain_cal           = 1.0f,
            .dac_offset_cal         = 0.0f,
            .adc_gain_cal           = 1.0f,
            .adc_offset_cal         = 0.0f,

            .dac_channel            = DAC_CHANNEL_1,
            .adc_channel            = ADC_CHANNEL_8
        }
    },

    [MFC_CHANNEL_2] =
    {
        .config =
        {
            .full_scale_flow_lpm    = 10.0f,
            .control_voltage_max_v  = 5.0f,
            .feedback_voltage_max_v = 5.0f,

            .dac_external_gain      = 2.0f,
            .adc_restore_gain       = 2.0f,
            .vdda_v                 = 3.3f,

            .dac_gain_cal           = 1.0f,
            .dac_offset_cal         = 0.0f,
            .adc_gain_cal           = 1.0f,
            .adc_offset_cal         = 0.0f,

            .dac_channel            = DAC_CHANNEL_2,
            .adc_channel            = ADC_CHANNEL_9
        }
    }
};
```

---

## 8. 核心换算函数

### 8.1 流量转 DAC 码值

```c
#include <stdbool.h>
#include <stdint.h>

#define MFC_DAC_FULL_SCALE_CODE  4095.0f

static float MFC_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static uint16_t MFC_FlowToDacCode(const mfc_config_t *config,
                                  float target_flow_lpm)
{
    if ((config == NULL) ||
        (config->full_scale_flow_lpm <= 0.0f) ||
        (config->control_voltage_max_v <= 0.0f) ||
        (config->dac_external_gain <= 0.0f) ||
        (config->vdda_v <= 0.0f))
    {
        return 0U;
    }

    target_flow_lpm = MFC_ClampFloat(
        target_flow_lpm,
        0.0f,
        config->full_scale_flow_lpm
    );

    float mfc_voltage_v =
        target_flow_lpm
        / config->full_scale_flow_lpm
        * config->control_voltage_max_v;

    float mcu_dac_voltage_v =
        mfc_voltage_v / config->dac_external_gain;

    float raw_code =
        mcu_dac_voltage_v
        / config->vdda_v
        * MFC_DAC_FULL_SCALE_CODE;

    raw_code =
        raw_code * config->dac_gain_cal
        + config->dac_offset_cal;

    raw_code = MFC_ClampFloat(
        raw_code,
        0.0f,
        MFC_DAC_FULL_SCALE_CODE
    );

    return (uint16_t)(raw_code + 0.5f);
}
```

### 8.2 ADC 原始值转反馈流量

```c
#define MFC_ADC_FULL_SCALE_CODE  4095.0f

static float MFC_AdcCodeToFlow(const mfc_config_t *config,
                               uint16_t adc_raw)
{
    if ((config == NULL) ||
        (config->full_scale_flow_lpm <= 0.0f) ||
        (config->feedback_voltage_max_v <= 0.0f) ||
        (config->adc_restore_gain <= 0.0f) ||
        (config->vdda_v <= 0.0f))
    {
        return 0.0f;
    }

    float adc_pin_voltage_v =
        (float)adc_raw
        / MFC_ADC_FULL_SCALE_CODE
        * config->vdda_v;

    float feedback_voltage_v =
        adc_pin_voltage_v
        * config->adc_restore_gain;

    feedback_voltage_v =
        feedback_voltage_v
        * config->adc_gain_cal
        + config->adc_offset_cal;

    float flow_lpm =
        feedback_voltage_v
        / config->feedback_voltage_max_v
        * config->full_scale_flow_lpm;

    return MFC_ClampFloat(
        flow_lpm,
        0.0f,
        config->full_scale_flow_lpm
    );
}
```

---

## 9. 对外接口设计

建议 `bsp_mfc.h` 提供以下接口：

```c
#ifndef BSP_MFC_H
#define BSP_MFC_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MFC_CHANNEL_1 = 0,
    MFC_CHANNEL_2,
    MFC_CHANNEL_COUNT
} mfc_channel_t;

bool MFC_Init(void);

bool MFC_SetFullScaleFlow(mfc_channel_t channel,
                          float full_scale_flow_lpm);

float MFC_GetFullScaleFlow(mfc_channel_t channel);

bool MFC_SetFlow(mfc_channel_t channel,
                 float target_flow_lpm);

float MFC_GetTargetFlow(mfc_channel_t channel);

bool MFC_UpdateFeedback(mfc_channel_t channel);

float MFC_GetFeedbackFlow(mfc_channel_t channel);

uint16_t MFC_GetAdcRaw(mfc_channel_t channel);

uint16_t MFC_GetDacCode(mfc_channel_t channel);

bool MFC_IsTrackingNormal(mfc_channel_t channel);

#endif
```

---

## 10. 设置目标流量

```c
bool MFC_SetFlow(mfc_channel_t channel,
                 float target_flow_lpm)
{
    if (channel >= MFC_CHANNEL_COUNT)
    {
        return false;
    }

    mfc_device_t *device = &g_mfc[channel];

    if (device->config.full_scale_flow_lpm <= 0.0f)
    {
        return false;
    }

    target_flow_lpm = MFC_ClampFloat(
        target_flow_lpm,
        0.0f,
        device->config.full_scale_flow_lpm
    );

    uint16_t dac_code =
        MFC_FlowToDacCode(
            &device->config,
            target_flow_lpm
        );

    if (HAL_DAC_SetValue(
            &hdac,
            device->config.dac_channel,
            DAC_ALIGN_12B_R,
            dac_code) != HAL_OK)
    {
        return false;
    }

    device->status.target_flow_lpm = target_flow_lpm;
    device->status.dac_code = dac_code;

    device->status.target_voltage_v =
        target_flow_lpm
        / device->config.full_scale_flow_lpm
        * device->config.control_voltage_max_v;

    return true;
}
```

---

## 11. 更新反馈流量

ADC 读取函数可以根据工程结构采用轮询、DMA 或中断方式。

```c
bool MFC_UpdateFeedback(mfc_channel_t channel)
{
    if (channel >= MFC_CHANNEL_COUNT)
    {
        return false;
    }

    mfc_device_t *device = &g_mfc[channel];

    uint16_t adc_raw = BSP_ADC_ReadChannel(
        device->config.adc_channel
    );

    float feedback_flow_lpm =
        MFC_AdcCodeToFlow(
            &device->config,
            adc_raw
        );

    float adc_pin_voltage_v =
        (float)adc_raw
        / 4095.0f
        * device->config.vdda_v;

    float feedback_voltage_v =
        adc_pin_voltage_v
        * device->config.adc_restore_gain;

    device->status.adc_raw = adc_raw;
    device->status.feedback_voltage_v = feedback_voltage_v;
    device->status.feedback_flow_lpm = feedback_flow_lpm;

    return true;
}
```

---

## 12. 满量程字段自动计算示例

### 12.1 MFC1：30 L/min

```c
g_mfc[MFC_CHANNEL_1].config.full_scale_flow_lpm = 30.0f;

MFC_SetFlow(MFC_CHANNEL_1, 15.0f);
```

自动计算：

```text
目标比例 = 15 / 30 = 50%
MFC 设定电压 = 5 × 50% = 2.5 V
MCU DAC 电压 = 2.5 / 2 = 1.25 V
DAC 码值约 = 1.25 / 3.3 × 4095 ≈ 1551
```

### 12.2 MFC2：10 L/min

```c
g_mfc[MFC_CHANNEL_2].config.full_scale_flow_lpm = 10.0f;

MFC_SetFlow(MFC_CHANNEL_2, 5.0f);
```

自动计算：

```text
目标比例 = 5 / 10 = 50%
MFC 设定电压 = 5 × 50% = 2.5 V
MCU DAC 电压 = 2.5 / 2 = 1.25 V
DAC 码值约 = 1551
```

可以看到，虽然两台 MFC 的满量程不同，但只要目标流量占满量程的比例相同，最终模拟设定电压相同。

---

## 13. 运行时修改满量程

软件应允许修改某个通道的满量程：

```c
bool MFC_SetFullScaleFlow(mfc_channel_t channel,
                          float full_scale_flow_lpm)
{
    if ((channel >= MFC_CHANNEL_COUNT) ||
        (full_scale_flow_lpm <= 0.0f))
    {
        return false;
    }

    g_mfc[channel].config.full_scale_flow_lpm =
        full_scale_flow_lpm;

    /*
     * 满量程修改后，建议重新下发当前目标流量，
     * 避免旧 DAC 码值仍对应之前的比例。
     */
    return MFC_SetFlow(
        channel,
        g_mfc[channel].status.target_flow_lpm
    );
}
```

如果 MFC 型号固定，也可以把满量程写在编译期配置表中，不开放运行时修改。

---

## 14. 反馈异常检测

系统同时具备设定值和反馈值，可以判断 MFC 是否正常跟踪目标流量。

建议不要仅使用固定误差，而是采用：

```text
允许误差 =
    max(固定误差, 满量程百分比误差)
```

例如：

```c
static bool MFC_CheckTracking(const mfc_device_t *device)
{
    const float absolute_tolerance_lpm = 0.5f;
    const float relative_tolerance = 0.05f;

    float relative_error_lpm =
        device->config.full_scale_flow_lpm
        * relative_tolerance;

    float allowed_error_lpm =
        (relative_error_lpm > absolute_tolerance_lpm)
        ? relative_error_lpm
        : absolute_tolerance_lpm;

    float error_lpm =
        device->status.target_flow_lpm
        - device->status.feedback_flow_lpm;

    if (error_lpm < 0.0f)
    {
        error_lpm = -error_lpm;
    }

    return error_lpm <= allowed_error_lpm;
}
```

还应加入：

- MFC 上电等待时间
- 流量建立延时
- 连续异常计数
- 零流量死区
- 反馈断线判断
- 反馈超量程判断
- 气源不足判断
- MFC 未上电判断

---

## 15. 推荐滤波

MFC 模拟反馈可能存在噪声，建议先做均值滤波或一阶低通滤波。

### 15.1 简单一阶滤波

```c
static float MFC_LowPassFilter(float previous,
                               float current,
                               float alpha)
{
    if (alpha < 0.0f)
    {
        alpha = 0.0f;
    }

    if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }

    return previous
        + alpha * (current - previous);
}
```

示例：

```c
filtered_flow =
    MFC_LowPassFilter(
        filtered_flow,
        current_flow,
        0.1f
    );
```

---

## 16. 标定建议

理论值会受到以下因素影响：

- VDDA 实际电压
- 分压电阻误差
- 运放增益误差
- 运放输出摆幅
- 运放零点偏移
- MFC 零点和满量程误差
- 地线压降
- 接口负载

建议每个通道分别进行标定。

### 16.1 DAC 标定

依次设置：

```text
0%
20%
40%
60%
80%
100%
```

例如对 30 L/min MFC：

```text
0、6、12、18、24、30 L/min
```

测量 `FMC_DACx` 实际输出电压，拟合：

```text
实际输出码值 =
    理论码值 × dac_gain_cal
    + dac_offset_cal
```

### 16.2 ADC 标定

向 `FMC_adcx` 输入：

```text
0 V、1 V、2 V、3 V、4 V、5 V
```

记录 ADC 原始值并拟合：

```text
实际反馈电压 =
    理论反馈电压 × adc_gain_cal
    + adc_offset_cal
```

---

## 17. CubeMX 配置要求

### 17.1 DAC

```text
PA4 → DAC_OUT1
PA5 → DAC_OUT2
```

配置：

```text
DAC Channel 1：Enable
DAC Channel 2：Enable
Output Buffer：按实际测试决定，通常可先 Enable
Trigger：None
```

程序启动：

```c
HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

MFC_SetFlow(MFC_CHANNEL_1, 0.0f);
MFC_SetFlow(MFC_CHANNEL_2, 0.0f);
```

### 17.2 ADC

```text
PB0 → ADC12_IN8
PB1 → ADC12_IN9
```

可以使用：

- ADC1 扫描两通道
- DMA 循环采样
- 定时器触发
- 软件定时读取

建议初始采用：

```text
ADC1
Scan Conversion Mode：Enable
Continuous Conversion：Enable
DMA Continuous Requests：Enable
通道：IN8、IN9
采样时间：较长采样周期，例如 84 或 144 cycles
```

较长采样时间有利于降低模拟前端源阻抗和滤波电容对采样稳定性的影响。

---

## 18. 安全要求

### 18.1 上电默认零流量

初始化 DAC 后立即设置：

```c
MFC_SetFlow(MFC_CHANNEL_1, 0.0f);
MFC_SetFlow(MFC_CHANNEL_2, 0.0f);
```

### 18.2 参数限制

所有目标流量必须限制在：

```text
0 ≤ target_flow_lpm ≤ full_scale_flow_lpm
```

### 18.3 配置有效性检查

以下配置必须大于零：

```text
full_scale_flow_lpm
control_voltage_max_v
feedback_voltage_max_v
dac_external_gain
adc_restore_gain
vdda_v
```

### 18.4 MFC 模拟地

必须确保：

```text
MFC Signal GND
    ↔
控制板模拟地/系统地
```

否则模拟量没有共同参考，可能导致控制误差、反馈漂移或接口异常。

---

## 19. 推荐文件结构

```text
BSP/
├── bsp_mfc.h
├── bsp_mfc.c
├── bsp_adc.h
├── bsp_adc.c
├── bsp_dac.h
└── bsp_dac.c
```

职责建议：

```text
bsp_adc：
    ADC 初始化后的通道读取
    DMA 数据管理
    原始值获取

bsp_dac：
    DAC 启动
    原始码值设置

bsp_mfc：
    MFC 规格配置
    满量程字段管理
    流量与电压换算
    DAC 设定
    ADC 反馈换算
    滤波
    状态与异常检测
```

---


## 20. 分层硬件配置方案

为了避免将 PCB 参数、MFC 型号参数和生产标定参数混在一起，配置应拆分为三层：

1. **MFC 设备规格配置**：描述外部 MFC 的流量范围和模拟接口范围。
2. **板卡模拟硬件配置**：描述 DAC 放大、ADC 分压、参考电压和安全上限。
3. **标定与诊断配置**：描述每路实际标定结果和异常判断参数。

### 20.1 MFC 设备规格配置

```c
typedef struct
{
    float flow_min_lpm;              /* 最小流量，通常为 0 L/min */
    float flow_max_lpm;              /* MFC 满量程流量 */

    float set_voltage_min_v;         /* MFC 设定输入最小电压 */
    float set_voltage_max_v;         /* MFC 设定输入最大电压 */

    float feedback_voltage_min_v;    /* MFC 反馈输出最小电压 */
    float feedback_voltage_max_v;    /* MFC 反馈输出最大电压 */
} mfc_device_spec_t;
```

当前项目典型配置：

```text
MFC1：0~30 L/min，设定 0~5 V，反馈 0~5 V
MFC2：0~10 L/min，设定 0~5 V，反馈 0~5 V
```

这样不仅支持两路满量程不同，也能扩展到：

```text
1~5 V 对应 0~30 L/min
0~10 V 对应 0~100 L/min
```

换算函数不得假定接口一定是 `0~5 V`。

### 20.2 DAC 板卡硬件配置

该结构描述 STM32 DAC 到 MFC 设定输入之间的模拟链路。

```c
typedef struct
{
    float vref_v;                    /* DAC 参考电压，通常为 VDDA */
    uint16_t resolution_max;         /* 12 位 DAC 为 4095 */

    float interface_output_min_v;    /* 板卡 FMC_DACx 接口最小输出 */
    float interface_output_max_v;    /* 板卡 FMC_DACx 接口最大输出 */

    float external_gain;             /* 外部运放增益，本板理论为 2.0 */
    float external_offset_v;         /* 外部模拟链路偏置，本板理论为 0 */

    float mcu_dac_min_v;              /* PA4/PA5 允许使用的最小目标电压 */
    float mcu_dac_max_v;              /* PA4/PA5 允许使用的最大目标电压 */

    uint32_t hal_dac_channel;
} mfc_dac_hw_config_t;
```

本板理论配置：

```c
.vref_v                = 3.3f,
.resolution_max         = 4095U,

.interface_output_min_v = 0.0f,
.interface_output_max_v = 5.0f,

.external_gain          = 2.0f,
.external_offset_v      = 0.0f,

.mcu_dac_min_v           = 0.0f,
.mcu_dac_max_v           = 2.5f,
```

需要明确区分：

```c
mcu_dac_max_v
```

表示 STM32 的 PA4/PA5 引脚目标电压上限，本板约为 `2.5 V`。

```c
interface_output_max_v
```

表示经过外部运放后，`FMC_DAC1/FMC_DAC2` 接口允许输出的上限，本板为 `5.0 V`。

### 20.3 ADC 板卡硬件配置

该结构描述 MFC 反馈输出到 STM32 ADC 之间的模拟链路。

```c
typedef struct
{
    float vref_v;                    /* ADC 参考电压 */
    uint16_t resolution_max;         /* 12 位 ADC 为 4095 */

    float interface_input_min_v;     /* FMC_adcx 接口最小输入 */
    float interface_input_max_v;     /* FMC_adcx 接口最大输入 */

    float divider_ratio;             /* 外部接口电压到 ADC 引脚的比例 */
    float analog_offset_v;           /* 模拟链路偏置 */

    float mcu_adc_min_v;              /* PB0/PB1 ADC 引脚允许最小电压 */
    float mcu_adc_max_v;              /* PB0/PB1 ADC 引脚允许最大电压 */

    uint32_t hal_adc_channel;
} mfc_adc_hw_config_t;
```

本板 `10 kΩ + 10 kΩ` 分压配置：

```c
.vref_v                = 3.3f,
.resolution_max         = 4095U,

.interface_input_min_v  = 0.0f,
.interface_input_max_v  = 5.0f,

.divider_ratio          = 0.5f,
.analog_offset_v        = 0.0f,

.mcu_adc_min_v           = 0.0f,
.mcu_adc_max_v           = 2.5f,
```

其中：

```text
ADC 引脚电压 = 外部反馈电压 × divider_ratio
外部反馈电压 = ADC 引脚电压 / divider_ratio
```

推荐使用 `divider_ratio = 0.5f` 直接描述硬件，不再将 `adc_restore_gain = 2.0f` 作为唯一核心字段，以避免方向混淆。

### 20.4 标定配置

```c
typedef struct
{
    float dac_gain;                  /* DAC 码值增益修正，默认 1 */
    float dac_offset_code;           /* DAC 码值偏置修正，默认 0 */

    float adc_gain;                  /* ADC 恢复电压增益修正，默认 1 */
    float adc_offset_v;              /* ADC 恢复电压偏置修正，默认 0 */
} mfc_calibration_t;
```

建议标定参数按通道独立保存，并可存储到内部 Flash 参数区。

### 20.5 诊断配置

```c
typedef struct
{
    float tracking_abs_tolerance_lpm; /* 绝对跟踪误差 */
    float tracking_relative_tolerance;/* 满量程百分比误差 */
    float zero_deadband_lpm;           /* 零点死区 */
    uint32_t response_timeout_ms;      /* 流量建立超时 */
} mfc_diagnostic_config_t;
```

### 20.6 完整通道配置结构

```c
typedef struct
{
    mfc_device_spec_t device;
    mfc_dac_hw_config_t dac_hw;
    mfc_adc_hw_config_t adc_hw;
    mfc_calibration_t calibration;
    mfc_diagnostic_config_t diagnostics;
} mfc_channel_config_t;

typedef struct
{
    const char *name;
    mfc_channel_config_t config;
    mfc_status_t status;
} mfc_device_t;
```

---

## 21. 推荐完整通道配置示例

```c
static mfc_device_t g_mfc[MFC_CHANNEL_COUNT] =
{
    [MFC_CHANNEL_1] =
    {
        .name = "Helium MFC",
        .config =
        {
            .device =
            {
                .flow_min_lpm           = 0.0f,
                .flow_max_lpm           = 30.0f,
                .set_voltage_min_v       = 0.0f,
                .set_voltage_max_v       = 5.0f,
                .feedback_voltage_min_v  = 0.0f,
                .feedback_voltage_max_v  = 5.0f
            },

            .dac_hw =
            {
                .vref_v                 = 3.3f,
                .resolution_max          = 4095U,
                .interface_output_min_v  = 0.0f,
                .interface_output_max_v  = 5.0f,
                .external_gain           = 2.0f,
                .external_offset_v       = 0.0f,
                .mcu_dac_min_v            = 0.0f,
                .mcu_dac_max_v            = 2.5f,
                .hal_dac_channel         = DAC_CHANNEL_1
            },

            .adc_hw =
            {
                .vref_v                 = 3.3f,
                .resolution_max          = 4095U,
                .interface_input_min_v   = 0.0f,
                .interface_input_max_v   = 5.0f,
                .divider_ratio           = 0.5f,
                .analog_offset_v         = 0.0f,
                .mcu_adc_min_v            = 0.0f,
                .mcu_adc_max_v            = 2.5f,
                .hal_adc_channel         = ADC_CHANNEL_8
            },

            .calibration =
            {
                .dac_gain               = 1.0f,
                .dac_offset_code        = 0.0f,
                .adc_gain               = 1.0f,
                .adc_offset_v           = 0.0f
            },

            .diagnostics =
            {
                .tracking_abs_tolerance_lpm = 0.5f,
                .tracking_relative_tolerance = 0.05f,
                .zero_deadband_lpm           = 0.2f,
                .response_timeout_ms         = 2000U
            }
        }
    },

    [MFC_CHANNEL_2] =
    {
        .name = "Argon MFC",
        .config =
        {
            .device =
            {
                .flow_min_lpm           = 0.0f,
                .flow_max_lpm           = 10.0f,
                .set_voltage_min_v       = 0.0f,
                .set_voltage_max_v       = 5.0f,
                .feedback_voltage_min_v  = 0.0f,
                .feedback_voltage_max_v  = 5.0f
            },

            .dac_hw =
            {
                .vref_v                 = 3.3f,
                .resolution_max          = 4095U,
                .interface_output_min_v  = 0.0f,
                .interface_output_max_v  = 5.0f,
                .external_gain           = 2.0f,
                .external_offset_v       = 0.0f,
                .mcu_dac_min_v            = 0.0f,
                .mcu_dac_max_v            = 2.5f,
                .hal_dac_channel         = DAC_CHANNEL_2
            },

            .adc_hw =
            {
                .vref_v                 = 3.3f,
                .resolution_max          = 4095U,
                .interface_input_min_v   = 0.0f,
                .interface_input_max_v   = 5.0f,
                .divider_ratio           = 0.5f,
                .analog_offset_v         = 0.0f,
                .mcu_adc_min_v            = 0.0f,
                .mcu_adc_max_v            = 2.5f,
                .hal_adc_channel         = ADC_CHANNEL_9
            },

            .calibration =
            {
                .dac_gain               = 1.0f,
                .dac_offset_code        = 0.0f,
                .adc_gain               = 1.0f,
                .adc_offset_v           = 0.0f
            },

            .diagnostics =
            {
                .tracking_abs_tolerance_lpm = 0.3f,
                .tracking_relative_tolerance = 0.05f,
                .zero_deadband_lpm           = 0.1f,
                .response_timeout_ms         = 2000U
            }
        }
    }
};
```

---

## 22. 基于完整配置的统一换算

### 22.1 目标流量转 DAC 码值

推荐按以下步骤计算：

```text
目标流量
→ MFC 流量比例
→ MFC 接口设定电压
→ 根据外部增益和偏置计算 MCU DAC 电压
→ 执行 MCU DAC 电压安全限制
→ 转换为 12 位 DAC 码值
→ 应用通道标定
```

```c
static uint16_t MFC_FlowToDacCodeV2(
    const mfc_channel_config_t *cfg,
    float target_flow_lpm)
{
    if ((cfg == NULL) ||
        (cfg->device.flow_max_lpm <= cfg->device.flow_min_lpm) ||
        (cfg->device.set_voltage_max_v <=
         cfg->device.set_voltage_min_v) ||
        (cfg->dac_hw.external_gain <= 0.0f) ||
        (cfg->dac_hw.vref_v <= 0.0f) ||
        (cfg->dac_hw.resolution_max == 0U))
    {
        return 0U;
    }

    target_flow_lpm = MFC_ClampFloat(
        target_flow_lpm,
        cfg->device.flow_min_lpm,
        cfg->device.flow_max_lpm
    );

    float flow_ratio =
        (target_flow_lpm - cfg->device.flow_min_lpm)
        / (cfg->device.flow_max_lpm -
           cfg->device.flow_min_lpm);

    float interface_voltage_v =
        cfg->device.set_voltage_min_v
        + flow_ratio
        * (cfg->device.set_voltage_max_v -
           cfg->device.set_voltage_min_v);

    interface_voltage_v = MFC_ClampFloat(
        interface_voltage_v,
        cfg->dac_hw.interface_output_min_v,
        cfg->dac_hw.interface_output_max_v
    );

    float mcu_dac_voltage_v =
        (interface_voltage_v -
         cfg->dac_hw.external_offset_v)
        / cfg->dac_hw.external_gain;

    mcu_dac_voltage_v = MFC_ClampFloat(
        mcu_dac_voltage_v,
        cfg->dac_hw.mcu_dac_min_v,
        cfg->dac_hw.mcu_dac_max_v
    );

    float dac_code =
        mcu_dac_voltage_v
        / cfg->dac_hw.vref_v
        * (float)cfg->dac_hw.resolution_max;

    dac_code =
        dac_code * cfg->calibration.dac_gain
        + cfg->calibration.dac_offset_code;

    dac_code = MFC_ClampFloat(
        dac_code,
        0.0f,
        (float)cfg->dac_hw.resolution_max
    );

    return (uint16_t)(dac_code + 0.5f);
}
```

### 22.2 ADC 原始值转反馈流量

```text
ADC 原始值
→ MCU ADC 引脚电压
→ 根据分压比例恢复接口电压
→ 应用 ADC 标定
→ 计算 MFC 电压比例
→ 根据该通道流量范围换算实际流量
```

```c
static float MFC_AdcCodeToFlowV2(
    const mfc_channel_config_t *cfg,
    uint16_t adc_raw)
{
    if ((cfg == NULL) ||
        (cfg->device.flow_max_lpm <= cfg->device.flow_min_lpm) ||
        (cfg->device.feedback_voltage_max_v <=
         cfg->device.feedback_voltage_min_v) ||
        (cfg->adc_hw.divider_ratio <= 0.0f) ||
        (cfg->adc_hw.vref_v <= 0.0f) ||
        (cfg->adc_hw.resolution_max == 0U))
    {
        return 0.0f;
    }

    if (adc_raw > cfg->adc_hw.resolution_max)
    {
        adc_raw = cfg->adc_hw.resolution_max;
    }

    float adc_pin_voltage_v =
        (float)adc_raw
        / (float)cfg->adc_hw.resolution_max
        * cfg->adc_hw.vref_v;

    float interface_voltage_v =
        (adc_pin_voltage_v -
         cfg->adc_hw.analog_offset_v)
        / cfg->adc_hw.divider_ratio;

    interface_voltage_v =
        interface_voltage_v
        * cfg->calibration.adc_gain
        + cfg->calibration.adc_offset_v;

    interface_voltage_v = MFC_ClampFloat(
        interface_voltage_v,
        cfg->adc_hw.interface_input_min_v,
        cfg->adc_hw.interface_input_max_v
    );

    float voltage_ratio =
        (interface_voltage_v -
         cfg->device.feedback_voltage_min_v)
        / (cfg->device.feedback_voltage_max_v -
           cfg->device.feedback_voltage_min_v);

    voltage_ratio = MFC_ClampFloat(
        voltage_ratio,
        0.0f,
        1.0f
    );

    return cfg->device.flow_min_lpm
        + voltage_ratio
        * (cfg->device.flow_max_lpm -
           cfg->device.flow_min_lpm);
}
```

---

## 23. 配置字段的修改权限

### 23.1 编译期固定字段

这些字段由 PCB 电路决定，应放在 `bsp_mfc_hw_config.c` 中并尽量声明为 `const`：

```c
dac_hw.external_gain
dac_hw.external_offset_v
dac_hw.interface_output_min_v
dac_hw.interface_output_max_v
dac_hw.mcu_dac_min_v
dac_hw.mcu_dac_max_v
dac_hw.hal_dac_channel

adc_hw.divider_ratio
adc_hw.analog_offset_v
adc_hw.interface_input_min_v
adc_hw.interface_input_max_v
adc_hw.mcu_adc_min_v
adc_hw.mcu_adc_max_v
adc_hw.hal_adc_channel
```

### 23.2 产品型号配置字段

这些字段取决于实际安装的 MFC 型号：

```c
device.flow_min_lpm
device.flow_max_lpm
device.set_voltage_min_v
device.set_voltage_max_v
device.feedback_voltage_min_v
device.feedback_voltage_max_v
```

可由以下方式提供：

- 编译期产品型号表
- 内部 Flash 参数区
- 上位机配置命令
- 出厂参数文件

### 23.3 标定字段

以下字段允许生产或维护阶段修改：

```c
calibration.dac_gain
calibration.dac_offset_code
calibration.adc_gain
calibration.adc_offset_v
```

建议保存时带：

```text
参数版本号
数据长度
CRC
有效标志
```

---

## 24. 配置合法性检查

初始化每一路 MFC 前必须执行配置校验。

```c
static bool MFC_ValidateConfig(
    const mfc_channel_config_t *cfg)
{
    if (cfg == NULL)
    {
        return false;
    }

    if (cfg->device.flow_max_lpm <=
        cfg->device.flow_min_lpm)
    {
        return false;
    }

    if (cfg->device.set_voltage_max_v <=
        cfg->device.set_voltage_min_v)
    {
        return false;
    }

    if (cfg->device.feedback_voltage_max_v <=
        cfg->device.feedback_voltage_min_v)
    {
        return false;
    }

    if ((cfg->dac_hw.external_gain <= 0.0f) ||
        (cfg->dac_hw.vref_v <= 0.0f) ||
        (cfg->dac_hw.resolution_max == 0U))
    {
        return false;
    }

    if ((cfg->adc_hw.divider_ratio <= 0.0f) ||
        (cfg->adc_hw.divider_ratio > 1.0f) ||
        (cfg->adc_hw.vref_v <= 0.0f) ||
        (cfg->adc_hw.resolution_max == 0U))
    {
        return false;
    }

    float required_mcu_dac_v =
        (cfg->device.set_voltage_max_v -
         cfg->dac_hw.external_offset_v)
        / cfg->dac_hw.external_gain;

    if ((required_mcu_dac_v <
         cfg->dac_hw.mcu_dac_min_v) ||
        (required_mcu_dac_v >
         cfg->dac_hw.mcu_dac_max_v))
    {
        return false;
    }

    float max_adc_pin_v =
        cfg->device.feedback_voltage_max_v
        * cfg->adc_hw.divider_ratio
        + cfg->adc_hw.analog_offset_v;

    if (max_adc_pin_v >
        cfg->adc_hw.mcu_adc_max_v)
    {
        return false;
    }

    return true;
}
```

本板理论检查：

```text
DAC 最大设定：
5.0 V / 2.0 = 2.5 V
符合 MCU DAC 目标上限 2.5 V

ADC 最大反馈：
5.0 V × 0.5 = 2.5 V
符合 MCU ADC 目标上限 2.5 V
```

> 注意：这里的 `2.5 V` 是软件设计目标上限，不代表 STM32 ADC 的绝对耐压上限。STM32 引脚保护仍应以芯片数据手册和 VDDA 实际值为准。

---

## 25. 编程助手实现要求

编程助手应满足：

1. 不得把 `30 L/min` 写死在换算函数中。
2. 每个通道必须包含独立的 `full_scale_flow_lpm`。
3. 所有 DAC 和 ADC 换算必须基于对应通道配置。
4. 支持两路 MFC 满量程不同。
5. 支持后续扩展更多 MFC 通道。
6. 初始化时默认输出零流量。
7. 所有输入参数进行范围检查。
8. 对非法通道和非法配置返回错误。
9. 保留 DAC/ADC 增益与偏置标定字段。
10. 区分目标流量、反馈流量、模拟电压、原始 ADC/DAC 数值。
11. 业务层不得直接操作 HAL DAC/ADC，应通过 MFC 模块接口调用。
12. 反馈值应支持滤波和跟踪误差判断。
13. 满量程修改后，应重新计算并下发当前目标流量。
14. 代码应兼容 STM32CubeMX 生成的 HAL 工程。
15. 用户代码应放在 CubeMX 的 `USER CODE BEGIN/END` 区域或独立文件中，避免重新生成时被覆盖。
16. 配置必须拆分为 MFC 设备规格、DAC 硬件、ADC 硬件、标定和诊断五部分。
17. 必须区分 MCU DAC 引脚最大目标电压与板卡模拟接口最大输出电压。
18. 必须使用 `divider_ratio` 描述 ADC 硬件分压比例。
19. PCB 固定参数应集中放在硬件配置文件中并尽量声明为 `const`。
20. 初始化时必须调用 `MFC_ValidateConfig()`，配置非法时禁止启动对应通道。
21. 换算函数应兼容非零起点的电压范围和流量范围，例如 `1~5 V`。
22. 标定参数应按通道独立保存，并预留 Flash 持久化接口。


---

## 26. 核心结论

本板两路 MFC 模拟接口可以抽象为：

```text
流量设定：
TargetFlow
→ 根据 full_scale_flow_lpm 计算 0~5 V
→ 根据外部 2 倍增益换算 MCU DAC 电压
→ 计算 12 位 DAC 码值
→ PA4 / PA5 输出

流量反馈：
PB0 / PB1 ADC 原始值
→ 计算 MCU ADC 引脚电压
→ 根据 2 倍恢复系数还原 0~5 V
→ 根据 full_scale_flow_lpm 计算实际流量
```

最关键的设备规格字段为：

```c
float flow_min_lpm;
float flow_max_lpm;
float set_voltage_min_v;
float set_voltage_max_v;
float feedback_voltage_min_v;
float feedback_voltage_max_v;
```

最关键的板卡硬件字段为：

```c
float mcu_dac_max_v;
float interface_output_max_v;
float external_gain;

float mcu_adc_max_v;
float interface_input_max_v;
float divider_ratio;
```

只要为每路 MFC 配置正确的设备规格和板卡硬件参数，软件即可自动完成流量、电压、ADC 和 DAC 之间的统一换算，而不需要针对不同 MFC 型号分别编写公式。


---

## 27. MFC 上下电控制与通道可用状态

每路 MFC 除模拟量设定和反馈外，还具有独立的继电器上下电控制。

根据当前原理图，控制映射如下：

| 通道 | MCU 引脚 | 继电器控制网络 | 继电器触点网络 | 当前状态 |
|---|---|---|---|---|
| MFC1 | PC2 | SWRAY1 | NO1 | 接线存在问题，暂时禁用 |
| MFC2 | PC3 | SWRAY2 | NO2 | 当前有效 |

控制链路：

```text
PC2 / PC3
→ S8050 三极管
→ TLP121 光耦
→ AN06 MOSFET
→ 5 V 继电器线圈
→ NO1 / NO2 常开触点
→ MFC 电源通断
```

按原理图初步判断，PC2/PC3 高电平使继电器吸合，即：

```text
GPIO_PIN_SET   → MFC 上电
GPIO_PIN_RESET → MFC 断电
```

有效电平必须保存在配置字段中，并在实物上确认。若实测相反，只修改配置，不修改业务逻辑。

### 27.1 CubeMX 配置

```text
PC2 → GPIO_Output，User Label：MFC1_PWR_EN
PC3 → GPIO_Output，User Label：MFC2_PWR_EN
```

推荐：

```text
Mode          = Output Push Pull
Pull          = No Pull
Speed         = Low
Initial Level = Low
```

初始化后两路继电器默认释放，MFC 默认断电。

---

## 28. MFC 电源配置与状态字段

```c
typedef enum
{
    MFC_CHANNEL_DISABLED = 0,
    MFC_CHANNEL_ENABLED
} mfc_channel_enable_t;

typedef enum
{
    MFC_POWER_OFF = 0,
    MFC_POWER_STARTING,
    MFC_POWER_ON,
    MFC_POWER_STOPPING,
    MFC_POWER_FAULT
} mfc_power_state_t;

typedef struct
{
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;

    GPIO_PinState power_active_level;

    uint32_t power_on_delay_ms;
    uint32_t power_off_delay_ms;
} mfc_power_hw_config_t;
```

完整通道配置增加：

```c
typedef struct
{
    mfc_channel_enable_t channel_enable;

    mfc_device_spec_t device;
    mfc_dac_hw_config_t dac_hw;
    mfc_adc_hw_config_t adc_hw;
    mfc_power_hw_config_t power_hw;

    mfc_calibration_t calibration;
    mfc_diagnostic_config_t diagnostics;
} mfc_channel_config_t;
```

状态结构增加：

```c
typedef struct
{
    float target_flow_lpm;
    float feedback_flow_lpm;

    float target_voltage_v;
    float feedback_voltage_v;

    uint16_t dac_code;
    uint16_t adc_raw;

    mfc_power_state_t power_state;

    bool tracking_ok;
    bool feedback_valid;
    bool fault;
} mfc_status_t;
```

---

## 29. 当前双路启用配置

MFC1 暂时禁用：

```c
[MFC_CHANNEL_1] =
{
    .name = "MFC1",
    .config =
    {
        .channel_enable = MFC_CHANNEL_DISABLED,

        .power_hw =
        {
            .gpio_port          = MFC1_PWR_EN_GPIO_Port,
            .gpio_pin           = MFC1_PWR_EN_Pin,
            .power_active_level = GPIO_PIN_SET,
            .power_on_delay_ms  = 1000U,
            .power_off_delay_ms = 100U
        }

        /* 其余 device、dac_hw、adc_hw 配置保留 */
    }
},
```

MFC2 当前有效：

```c
[MFC_CHANNEL_2] =
{
    .name = "MFC2",
    .config =
    {
        .channel_enable = MFC_CHANNEL_ENABLED,

        .power_hw =
        {
            .gpio_port          = MFC2_PWR_EN_GPIO_Port,
            .gpio_pin           = MFC2_PWR_EN_Pin,
            .power_active_level = GPIO_PIN_SET,
            .power_on_delay_ms  = 1000U,
            .power_off_delay_ms = 100U
        }

        /* device.flow_max_lpm 填写第二路实际满量程 */
    }
},
```

当通道为 `MFC_CHANNEL_DISABLED` 时，该通道所有上电、流量设定和反馈更新接口都必须返回通道禁用错误。

---

## 30. MFC 模块文件结构

MFC 功能只建立：

```text
BSP/
├── bsp_mfc.h
└── bsp_mfc.c
```

不再额外建立独立的 MFC ADC、DAC 或继电器驱动文件。

CubeMX 仍负责生成：

```text
Core/Inc/adc.h
Core/Src/adc.c
Core/Inc/dac.h
Core/Src/dac.c
Core/Inc/gpio.h
Core/Src/gpio.c
```

`bsp_mfc.c` 内部直接使用：

```c
extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac;
```

并统一负责：

```text
通道配置
MFC上下电
DAC流量设定
ADC反馈读取
换算和滤波
状态管理
异常判断
```

---

## 31. 对外接口

```c
#ifndef BSP_MFC_H
#define BSP_MFC_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MFC_CHANNEL_1 = 0,
    MFC_CHANNEL_2,
    MFC_CHANNEL_COUNT
} mfc_channel_t;

typedef enum
{
    MFC_RESULT_OK = 0,
    MFC_RESULT_INVALID_CHANNEL,
    MFC_RESULT_CHANNEL_DISABLED,
    MFC_RESULT_INVALID_CONFIG,
    MFC_RESULT_NOT_POWERED,
    MFC_RESULT_HAL_ERROR
} mfc_result_t;

mfc_result_t MFC_Init(void);

bool MFC_IsChannelEnabled(mfc_channel_t channel);

mfc_result_t MFC_PowerOn(mfc_channel_t channel);
mfc_result_t MFC_PowerOff(mfc_channel_t channel);
bool MFC_IsPowered(mfc_channel_t channel);

mfc_result_t MFC_SetFlow(mfc_channel_t channel,
                         float target_flow_lpm);

mfc_result_t MFC_UpdateFeedback(mfc_channel_t channel);

float MFC_GetTargetFlow(mfc_channel_t channel);
float MFC_GetFeedbackFlow(mfc_channel_t channel);

uint16_t MFC_GetAdcRaw(mfc_channel_t channel);
uint16_t MFC_GetDacCode(mfc_channel_t channel);

bool MFC_IsTrackingNormal(mfc_channel_t channel);

#endif
```

---

## 32. 电源控制核心逻辑

```c
static GPIO_PinState MFC_GetPowerOffLevel(
    const mfc_power_hw_config_t *power)
{
    return (power->power_active_level == GPIO_PIN_SET)
        ? GPIO_PIN_RESET
        : GPIO_PIN_SET;
}

static mfc_result_t MFC_CheckChannel(mfc_channel_t channel)
{
    if (channel >= MFC_CHANNEL_COUNT)
    {
        return MFC_RESULT_INVALID_CHANNEL;
    }

    if (g_mfc[channel].config.channel_enable !=
        MFC_CHANNEL_ENABLED)
    {
        return MFC_RESULT_CHANNEL_DISABLED;
    }

    return MFC_RESULT_OK;
}
```

### 32.1 上电顺序

```text
检查通道已启用
→ DAC设定先清零
→ 吸合继电器
→ 等待MFC稳定
→ 标记为POWER_ON
```

### 32.2 断电顺序

```text
目标流量降为0
→ 等待MFC关流
→ 释放继电器
→ 反馈标记为无效
```

---

## 33. 电源与流量控制联动要求

1. 未上电时禁止设定非零流量。
2. 上电前必须先将 DAC 设定清零。
3. 断电前必须先将目标流量降到最小值。
4. 断电状态下反馈数据标记为无效。
5. 断电状态下不进行流量跟踪故障判断。
6. 继电器有效电平由 `power_active_level` 决定。
7. MFC1 当前所有操作返回 `MFC_RESULT_CHANNEL_DISABLED`。
8. MFC2 是当前唯一有效通道。

`MFC_SetFlow()` 中应增加：

```c
if ((device->status.power_state != MFC_POWER_ON) &&
    (target_flow_lpm >
     device->config.device.flow_min_lpm))
{
    return MFC_RESULT_NOT_POWERED;
}
```

---

## 34. 初始化策略

`MFC_Init()` 必须：

1. 启动 DAC 通道。
2. 所有 DAC 默认输出最小流量码值。
3. PC2、PC3 输出继电器断电电平。
4. 所有通道状态初始化为 `MFC_POWER_OFF`。
5. 第一通道保持禁用。
6. 只校验并允许第二通道投入运行。
7. 清除反馈有效标志、跟踪状态和故障状态。

后续第一路硬件修复后，只需将：

```c
.channel_enable = MFC_CHANNEL_DISABLED
```

修改为：

```c
.channel_enable = MFC_CHANNEL_ENABLED
```

无需修改上下电、DAC、ADC 或业务控制逻辑。

---

## 35. 编程助手附加要求

1. 只创建 `bsp_mfc.h` 和 `bsp_mfc.c`。
2. PC2 命名为 `MFC1_PWR_EN`，PC3 命名为 `MFC2_PWR_EN`。
3. 第一通道必须暂时禁用。
4. 第二通道为当前有效 MFC。
5. 不允许业务层直接操作继电器 GPIO、ADC 或 DAC。
6. 上电前清零 DAC，断电前清零流量。
7. 未上电禁止非零设定。
8. 继电器极性和延时必须配置化。
9. 保留两路完整配置，方便第一路修复后直接启用。
10. 所有接口必须返回明确错误码，不得静默失败。
