# AGENTS.md - 代理编程指南

本文档为在此 STM32 嵌入式项目中工作的编程代理提供指南。

## 项目概述

STM32F103 等离子控制系统嵌入式项目,使用 Keil MDK 开发环境。
- 芯片: STM32F103VE/ZE
- 框架: STM32 标准外设库 (StdPeriph_Lib v3.5.0)
- 开发工具: Keil MDK (Project/pj3.uvprojx)
- 通信: USART1 (命令串口) + USART2 (日志串口)
- 编码: UTF-8(部分旧文件使用 GB2312)

---

## 构建和测试命令

### 构建项目
```bash
"C:\Keil_v5\UV4\UV4.exe" -b "Project\pj3.uvprojx" -j0
# 或在 Keil 中: Project -> Build Target (F7)
```

### 调试与烧录
```bash
# 调试: Debug -> Start/Stop Debug Session (Ctrl+F5)
# 烧录: Flash -> Download (F8)
```

### 测试方法
1. **串口测试**: 使用串口工具发送 test_packets.txt 中的数据包
2. **Python 测试工具**: `python packet_generator.py` / `python generate_all_packets.py`
3. **手动测试**:
   ```c
   uint8_t test_packet[] = {0xFF, 0xFE, 0x00, 0x11, 0xE8, 0x03, ...};
   for (int i = 0; i < 16; i++) {
       recv_buf[rxIndex++] = test_packet[i];
   }
   DealPortData(true);
   ```

---

## 代码风格指南

### 导入顺序
```c
#include "stm32f10x.h"      // 1. STM32 标准头文件
#include <stdio.h>          // 2. 标准库
#include <string.h>
#include <stdbool.h>
#include "adc.h"            // 3. 项目内部头文件
```

### 命名约定
- **函数**: PascalCase - `Usart1_Init()`, `OpenPlasmaVol()`, `Set_HeFlow_Value()`
- **变量**: camelCase - `PackReady`, `rxIndex`, `ifHead`, `RecvBuf[]`
- **宏定义**: 全大写 - `BUFSIZE`, `HEADER`, `ADCx`
- **结构体**: PascalCase - `DataPacket`
- **类型缩写**: `u8/uint8_t`, `u16/uint16_t`, `u32/uint32_t`

### 格式化
- 缩进: 4 个空格或 tab
- 花括号: K&R 风格(左花括号在同一行)
- 注释: 中文,单行注释使用 `//`
- 头文件守卫: `#ifndef _FILENAME_H_`

### 错误处理
```c
if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {}
if (rxIndex < RX_BUFFER_SIZE - 1) {
    recv_buf[rxIndex++] = data;
} else {
    rxIndex = 0;
}
```

### 数据包协议
- 字节序: 小端
- 数据包大小: 16 字节固定
- 校验和: 前 12 字节累加和

```c
// 发送 16 位数据(小端)
void USART_SendU16(uint16_t data) {
    USART_SendOneByte((uint8_t)(data & 0xFF));
    USART_SendOneByte((uint8_t)((data >> 8) & 0xFF));
}

// 计算校验和
uint32_t checksum = 0;
for (int i = 0; i < 12; i++) {
    checksum += packet[i];
}
```

---

## 项目特定协议

### 数据包结构(16 字节)
```
偏移  | 字段         | 类型      | 说明
------|-------------|-----------|------------------
0-1   | header      | uint16_t  | 包头: 0xFEFF
2     | EmerStop    | uint8_t   | 急停标志
3     | VolRelay    | uint8_t   | 电压继电器(高4位=等离子电源, 低4位=电压调制)
4-5   | VolOutValue | uint16_t  | 电压输出值
6     | HeFLOWRelay | uint8_t   | 氦气流量继电器(高4位=流量计, 低4位=阀门)
7-8   | HeOutValue  | uint16_t  | 氦气输出值
9     | ArFLOWRelay | uint8_t   | 氩气流量继电器(高4位=流量计, 低4位=阀门)
10-11 | ArOutValue  | uint16_t  | 氩气输出值
12-15 | footer      | uint32_t  | 校验和(前 12 字节和)
```

### 继电器位定义
```c
uint8_t plasmaRelay = (VolRelay >> 4) & 0x0F;
uint8_t volModRelay = VolRelay & 0x0F;
uint8_t flowMeter = (Relay >> 4) & 0x0F;
uint8_t valve = Relay & 0x0F;
```

---

## 重要注意事项

1. **编码问题**: 部分文件使用 GB2312 编码,新文件建议使用 UTF-8
2. **中断处理**: 中断处理函数定义在 `stm32f10x_it.c` 中
3. **时序控制**: 使用 `delay_ms(n)` 进行延时
4. **内存管理**: 嵌入式系统无动态内存分配,使用静态数组
5. **数据包校验**: 必须校验包头 (0xFEFF) 和校验和
6. **调试日志**: 使用 USART2 作为日志串口,`DealPortData(true)` 开启详细日志

---

## 模块化组织
```
User/
├── AD/           # ADC 模块
├── DA/           # DAC 模块
├── LED/          # LED 控制
├── USART1/       # 串口通信
├── cmd/          # 命令处理
├── key/          # 按键处理
├── relay/        # 继电器控制
├── step/         # 步进电机
├── systic/       # 系统时钟
├── main.c        # 主程序
├── stm32f10x_it.c/h  # 中断处理
└── stm32f10x_conf.h   # 库配置
```

---

## 代码检查清单

提交代码前确认:
- [ ] 函数命名遵循 PascalCase
- [ ] 变量命名遵循 camelCase
- [ ] 宏定义使用全大写
- [ ] 头文件包含守卫正确
- [ ] 导入顺序符合规范
- [ ] 注释使用中文
- [ ] 缩进统一(4 空格)
- [ ] 错误处理完整
- [ ] 数据包校验正确
- [ ] 在 Keil 中编译无错误、无警告

---

## 文件参考
- 项目配置: `Project/pj3.uvprojx`
- 数据包分析: `packet_analysis.md`
- 测试数据: `test_packets.txt`
- 库配置: `User/stm32f10x_conf.h`
