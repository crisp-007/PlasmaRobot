#include "relay.h"
#include "usart1.h"
#include "dac.h"


void RelayInit(void)
{
    // 使能GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // GPIOA时钟（等离子电源）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE); // GPIOF时钟（氦气电磁阀）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // GPIOB时钟（氦气流量计、氩气电磁阀）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); // GPIOC时钟（氩气流量计）

    GPIO_InitTypeDef  RelayGpioInit;
    
    // 初始化GPIOA引脚（等离子电源PA6、调压器PA4）
    RelayGpioInit.GPIO_Pin = VOLRelay_pin | VOLModRelay_pin;
    RelayGpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
    RelayGpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &RelayGpioInit);
    
    // 初始化GPIOF引脚（氦气电磁阀PF12）
    RelayGpioInit.GPIO_Pin = HeFLOWRelay_pin;
    RelayGpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
    RelayGpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOF, &RelayGpioInit);
    
    // 初始化GPIOB引脚（氦气流量计PB2、氩气电磁阀PB0）
    RelayGpioInit.GPIO_Pin = HeFLOWSwitchRelay_pin | ArFLOWRelay_pin;
    RelayGpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
    RelayGpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &RelayGpioInit);
    
    // 初始化GPIOC引脚（氩气流量计PC4）
    RelayGpioInit.GPIO_Pin = ArFLOWSwitchRelay_pin;
    RelayGpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
    RelayGpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &RelayGpioInit);
    
    // 初始化所有继电器为关闭状态（低电平）
    GPIO_ResetBits(GPIOA, VOLRelay_pin | VOLModRelay_pin);
    GPIO_ResetBits(GPIOF, HeFLOWRelay_pin);
    GPIO_ResetBits(GPIOB, HeFLOWSwitchRelay_pin | ArFLOWRelay_pin);
    GPIO_ResetBits(GPIOC, ArFLOWSwitchRelay_pin);
}

//打开等离子电源+调压器
void OpenVolDevice(void)
{
    printf("等离子/调压器电源-开！\n");
    GPIO_SetBits(Relay_port,VOLRelay_pin);
    GPIO_SetBits(Relay_port,VOLModRelay_pin);

}

//关闭等离子电源+调压器
void CloseVolDevice(void)
{
    printf("等离子/调压器电源-关！\n");
    GPIO_ResetBits(Relay_port,VOLRelay_pin);
    GPIO_ResetBits(Relay_port,VOLModRelay_pin);
}

//获取等离子电源继电器开关状态
u8 GetVolRelayState(void)
{
    if(GPIO_ReadOutputDataBit(Relay_port,VOLRelay_pin)==Bit_SET)
    {
        return 1;
    }else{
        return 0;
    }
}

//获取调压器继电器开关状态
u8 GetVolModRelayState(void)
{
    if(GPIO_ReadOutputDataBit(Relay_port,VOLModRelay_pin)==Bit_SET)
    {
        return 1;
    }else{
        return 0;
    }
}


//打开等离子电源
void OpenPlasmaVol(void)
{
    GPIO_SetBits(Relay_port,VOLRelay_pin);
}
//关闭等离子电源
void ClosePlasmaVol(void)
{
    GPIO_ResetBits(Relay_port,VOLRelay_pin);
}

//打开调压器
void OpenModVol(void)
{
    GPIO_SetBits(Relay_port,VOLModRelay_pin);
}
//关闭调压器
void CloseModVol(void)
{
    GPIO_ResetBits(Relay_port,VOLModRelay_pin);
}


//=== 氦气控制函数 ===

//打开氦气电磁阀
void OpenHeGasValve(void)
{
    printf("氦气电磁阀-开！\n");
    GPIO_SetBits(GPIOF, HeFLOWRelay_pin);
}

//关闭氦气电磁阀
void CloseHeGasValve(void)
{
    printf("氦气电磁阀-关！\n");
    GPIO_ResetBits(GPIOF, HeFLOWRelay_pin);
}

//打开氦气流量计
void OpenHeFlowMeter(void)
{
    printf("氦气流量计-开！\n");
    GPIO_SetBits(GPIOB, HeFLOWSwitchRelay_pin);
}

//关闭氦气流量计
void CloseHeFlowMeter(void)
{
    printf("氦气流量计-关！\n");
    GPIO_ResetBits(GPIOB, HeFLOWSwitchRelay_pin);
}

//打开氦气模块（电磁阀+流量计）
void OpenHeGasDevice(void)
{
    printf("氦气模块-开！\n");
    GPIO_SetBits(GPIOF, HeFLOWRelay_pin);
    GPIO_SetBits(GPIOB, HeFLOWSwitchRelay_pin);
}

//关闭氦气模块（电磁阀+流量计）
void CloseHeGasDevice(void)
{
    printf("氦气模块-关！\n");
    GPIO_ResetBits(GPIOF, HeFLOWRelay_pin);
    GPIO_ResetBits(GPIOB, HeFLOWSwitchRelay_pin);
}

//=== 氩气控制函数 ===

//打开氩气电磁阀
void OpenArGasValve(void)
{
    printf("氩气电磁阀-开！\n");
    GPIO_SetBits(GPIOB, ArFLOWRelay_pin);
}

//关闭氩气电磁阀
void CloseArGasValve(void)
{
    printf("氩气电磁阀-关！\n");
    GPIO_ResetBits(GPIOB, ArFLOWRelay_pin);
}

//打开氩气流量计
void OpenArFlowMeter(void)
{
    printf("氩气流量计-开！\n");
    GPIO_SetBits(GPIOC, ArFLOWSwitchRelay_pin);
}

//关闭氩气流量计
void CloseArFlowMeter(void)
{
    printf("氩气流量计-关！\n");
    GPIO_ResetBits(GPIOC, ArFLOWSwitchRelay_pin);
}

//打开氩气模块（电磁阀+流量计）
void OpenArGasDevice(void)
{
    printf("氩气模块-开！\n");
    GPIO_SetBits(GPIOB, ArFLOWRelay_pin);
    GPIO_SetBits(GPIOC, ArFLOWSwitchRelay_pin);
}

//关闭氩气模块（电磁阀+流量计）
void CloseArGasDevice(void)
{
    printf("氩气模块-关！\n");
    GPIO_ResetBits(GPIOB, ArFLOWRelay_pin);
    GPIO_ResetBits(GPIOC, ArFLOWSwitchRelay_pin);
}

//关闭所有气体供应（氦气和氩气的电磁阀+流量计）
void CloseFLOWDevice(void)
{
    printf("关闭所有气体供应！\n");
    // 先关闭氦气电磁阀和流量计
    GPIO_ResetBits(GPIOF, HeFLOWRelay_pin);
    GPIO_ResetBits(GPIOB, HeFLOWSwitchRelay_pin);
    // // 关闭氩气电磁阀和流量计
    // GPIO_ResetBits(GPIOB, ArFLOWRelay_pin);
    // GPIO_ResetBits(GPIOC, ArFLOWSwitchRelay_pin);
    // 设置氦气和氩气DAC输出为0
    Set_HeFlow_Value(0);
    Set_ArFlow_Value(0);
}

//=== 状态获取函数 ===

//获取氦气电磁阀状态
u8 GetHeGasValveState(void)
{
    if(GPIO_ReadOutputDataBit(GPIOF, HeFLOWRelay_pin) == Bit_SET)
    {
        return 1;
    }else{
        return 0;
    }
}

//获取氦气流量计状态
u8 GetHeFlowMeterState(void)
{
    if(GPIO_ReadOutputDataBit(GPIOB, HeFLOWSwitchRelay_pin) == Bit_SET)
    {
        return 1;
    }else{
        return 0;
    }
}

//获取氩气电磁阀状态
u8 GetArGasValveState(void)
{
    if(GPIO_ReadOutputDataBit(GPIOB, ArFLOWRelay_pin) == Bit_SET)
    {
        return 1;
    }else{
        return 0;
    }
}

//获取氩气流量计状态
u8 GetArFlowMeterState(void)
{
    if(GPIO_ReadOutputDataBit(GPIOC, ArFLOWSwitchRelay_pin) == Bit_SET)
    {
        return 1;
    }else{
        return 0;
    }
}


