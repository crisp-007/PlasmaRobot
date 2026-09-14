#include "dac.h"

//当前DA通道的输出电压（0-3.3v）
float DAC_CurValue_HeFlow=0;  // 氦气流量控制当前值
float DAC_CurValue_ArFlow=0;  // 氩气流量控制当前值

//按键控制的模拟量值（0-4096）
u16 heFlowKeyValue=0;  // 氦气流量按键控制值
u16 arFlowKeyValue=0;  // 氩气流量按键控制值


void DAC_GpioInit(void)
{
    // 1、开启GPIOA的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // 2、初始化引脚 PA4(氦气流量) PA5(氩气流量) 的配置
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = HE_FLOW_OUT_Pin | AR_FLOW_OUT_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    // GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DAC_Port, &GPIO_InitStruct);
    // GPIO_SetBits(GPIOA,GPIO_Pin_4);
}

void DAC_Config(void)
{
    // 1、打开DAC的时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
    // 2、初始化DAC的功能配置
    DAC_InitTypeDef DAC_InitStruct;
    DAC_InitStruct.DAC_Trigger = DAC_Trigger_None;            
    DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None; // 不使用波形发生器
    DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Disable;  // 不使用DAC输出缓冲
    DAC_Init(HE_FLOW_DAC_CHANNEL, &DAC_InitStruct);//配置DAC通道1(氦气流量控制)
    DAC_Init(AR_FLOW_DAC_CHANNEL, &DAC_InitStruct);//配置DAC通道2(氩气流量控制)

    //3、开启使能
    DAC_Cmd(HE_FLOW_DAC_CHANNEL, ENABLE);//使能通道1 由PA4输出氦气流量控制
    DAC_Cmd(AR_FLOW_DAC_CHANNEL, ENABLE);//使能通道2 由PA5输出氩气流量控制

    DAC_SetChannel1Data(DAC_Align_12b_R, 0); //12位右对齐数据格式设置DAC值
    DAC_SetChannel2Data(DAC_Align_12b_R, 0); //12位右对齐数据格式设置DAC值
}

void DAC_Init1(void)
{
    DAC_GpioInit();
    DAC_Config();
}

//设置氦气流量控制值，形参为0-4096
void Set_HeFlow_Value(u16 value)
{
    DAC_SetChannel1Data(DAC_Align_12b_R,value);//12位右对齐数据格式设置DAC值
}

//设置氩气流量控制值，形参为0-4096
void Set_ArFlow_Value(u16 value)
{
    DAC_SetChannel2Data(DAC_Align_12b_R,value);//12位右对齐数据格式设置DAC值
}

//获取氦气流量控制值
uint16_t Get_HeFlow_Value(void)
{
    return DAC_GetDataOutputValue(HE_FLOW_DAC_CHANNEL);
}

//获取氩气流量控制值
uint16_t Get_ArFlow_Value(void)
{
    return DAC_GetDataOutputValue(AR_FLOW_DAC_CHANNEL);
}


// //设置通道1输出电压
// //vol:0~3300,代表0~3.3V
// void Dac1_Set_Vol(u16 vol)
// {
//     float temp=vol;
//     temp/=1000;
//     temp=temp*4096/3.3;
//     DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12位右对齐数据格式设置DAC值
// }


// //DAC通道1输出初始化
// void Dac1_Init(void)
// {
//     GPIO_InitTypeDef GPIO_InitStructure;
//     DAC_InitTypeDef DAC_InitType;
    
//     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );     //使能PORTA通道时钟
//     RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE );     //使能DAC通道时钟
    
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;                 // 端口配置
//      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;          //模拟输入
//      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//      GPIO_Init(GPIOA, &GPIO_InitStructure);
//     GPIO_SetBits(GPIOA,GPIO_Pin_4);//PA.4 输出高
    
//     DAC_InitType.DAC_Trigger=DAC_Trigger_None;    //不使用触发功能 TEN1=0
//     DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//不使用波形发生
//     DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//屏蔽、幅值设置
//     DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;    //DAC1输出缓存关闭 BOFF1=1
//     DAC_Init(DAC_Channel_1,&DAC_InitType);     //初始化DAC通道1
    
//     DAC_Cmd(DAC_Channel_1, ENABLE); //使能DAC1
//     DAC_SetChannel1Data(DAC_Align_12b_R, 0); //12位右对齐数据格式设置DAC值
// }







