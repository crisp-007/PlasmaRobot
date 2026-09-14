#ifndef _RELAY_H_
#define _RELAY_H_

#include "stm32f10x.h"

//等离子相关
#define Relay_port GPIOA
#define VOLRelay_pin GPIO_Pin_6//等离子电源
#define VOLModRelay_pin GPIO_Pin_4//调压器

//氦气控制 气体电磁阀引脚PF12 流量计引脚PB2

#define HeFLOWRelay_pin GPIO_Pin_12
#define HeFLOWSwitchRelay_pin GPIO_Pin_2

//氩气控制 气体电磁阀引脚PB0 流量计引脚PC4
#define ArFLOWRelay_pin GPIO_Pin_0
#define ArFLOWSwitchRelay_pin GPIO_Pin_4



void RelayInit(void);

//1、等离子电源部分

//打开等离子电源+调压器
void OpenVolDevice(void);
//关闭等离子电源+调压器
void CloseVolDevice(void);

//打开等离子电源
void OpenPlasmaVol(void);
//关闭等离子电源
void ClosePlasmaVol(void);

//打开调压器
void OpenModVol(void);
//关闭调压器
void CloseModVol(void);

//2、氦气控制部分

//打开氦气电磁阀
void OpenHeGasValve(void);
//关闭氦气电磁阀
void CloseHeGasValve(void);

//打开氦气流量计
void OpenHeFlowMeter(void);
//关闭氦气流量计
void CloseHeFlowMeter(void);

//打开氦气模块（电磁阀+流量计）
void OpenHeGasDevice(void);
//关闭氦气模块（电磁阀+流量计）
void CloseHeGasDevice(void);

//3、氩气控制部分

//打开氩气电磁阀
void OpenArGasValve(void);
//关闭氩气电磁阀
void CloseArGasValve(void);

//打开氩气流量计
void OpenArFlowMeter(void);
//关闭氩气流量计
void CloseArFlowMeter(void);

//打开氩气模块（电磁阀+流量计）
void OpenArGasDevice(void);
//关闭氩气模块（电磁阀+流量计）
void CloseArGasDevice(void);

//关闭所有气体供应（氦气和氩气的电磁阀+流量计）
void CloseFLOWDevice(void);

//4、状态获取
u8 GetVolRelayState(void);
u8 GetVolModRelayState(void);

//氦气状态获取
u8 GetHeGasValveState(void);
u8 GetHeFlowMeterState(void);

//氩气状态获取
u8 GetArGasValveState(void);
u8 GetArFlowMeterState(void);








#endif

