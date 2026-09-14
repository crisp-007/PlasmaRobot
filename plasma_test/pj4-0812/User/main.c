#include "stm32f10x.h"
#include "usart1.h"
#include "led.h"
#include "key1.h"
#include "adc.h"
#include "systic.h"
#include "dac.h"
#include "relay.h"
#include "step.h"
#include "cmd.h"

int main(void)
{
    // usart1初始化
    Usart1_Init();
    CmdUsart_Init();//命令串口初始化
    //日志串口初始化

    // LED初始化
    Led0_Init();
    Led1_Init();
    Rgb_Init();  // 彩色LED初始化
    
    // 按键初始化
    key_init1();

    
    //模拟数字转换器初始化
    ADC_Init1(); // ADC初始化
    DAC_Init1();// DAC初始化

    // 初始状态关闭所有LED
    Led0_OFF();
    Led1_OFF();
    RgbOff();  // 关闭彩色LED
   
    // 模拟输出模块
    RelayInit();  // 初始化继电器
    
    // 判断是否使用步进电机调节电压
    ifUseStep = NotUseStepModVol;
    if(ifUseStep == UseStepModVol)
    {
        Step_init();
    }
    
    // 命令发送延时计数器
    static u8 CmdSendDelay = 0;
    
    // 初始化欢迎信息
    char SendBuf[36] = "Hello,this is STM32-PlasmaCtl!\n";
    printf(SendBuf);  // 通过UART2发送信息
    
    // 通过UART1发送数据包信息
    for(int i = 0; i < 35; i++) {
        if(SendBuf[i] != '\0') {
            USART_SendOneByte(SendBuf[i]);
        }
    }

    
    // 主循环
    while(1)
    {
        // 处理串口数据
        DealPortData(true);
        CmdDeal(PackReady);
        
        // 定时发送数据包，约260ms周期
        if(CmdSendDelay++ >= 13) 
        {
            SendPack();
            CmdSendDelay = 0;
        }
        
        // ADC数据处理，读取模拟输入值（0-3.3V）
        ADC_ConvertedValueLocal[VOL_ADC_INDEX] = (float)ADC_ConvertedValue[VOL_ADC_INDEX] / 4096 * 3.3;        // 等离子电压
        ADC_ConvertedValueLocal[HE_FLOW_ADC_INDEX] = (float)ADC_ConvertedValue[HE_FLOW_ADC_INDEX] / 4096 * 3.3;  // 氦气流量
        ADC_ConvertedValueLocal[AR_FLOW_ADC_INDEX] = (float)ADC_ConvertedValue[AR_FLOW_ADC_INDEX] / 4096 * 3.3;  // 氩气流量
        
        // DAC数据处理，读取当前输出值（0-3.3V）
        DAC_CurValue_HeFlow = (float)DAC_GetDataOutputValue(HE_FLOW_DAC_CHANNEL) / 4096 * 3.3;
        DAC_CurValue_ArFlow = (float)DAC_GetDataOutputValue(AR_FLOW_DAC_CHANNEL) / 4096 * 3.3;
        
        // 按键事件处理
        KeyEvent();
        
        // 系统延时，20ms周期
        delay_ms(20);
    }

}
