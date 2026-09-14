#include "adc.h"

// u16 ADC_ConvertedValue = 0;
// float ADC_ConvertedValueLocal = 0;
// ADC原始数据数组：存放DMA从ADC数据寄存器自动传输的16位原始数值
// 数组索引对应：[VOL_ADC_INDEX]等离子电压 [HE_FLOW_ADC_INDEX]氦气流量 [AR_FLOW_ADC_INDEX]氩气流量
// 数据范围：0-4095 (12位ADC分辨率)
// 更新方式：通过DMA循环传输模式自动更新，无需软件干预
u16 ADC_ConvertedValue[NOFCHANEL]={0,0,0};

// ADC本地浮点数据数组：用于存放经过处理后的浮点型ADC数值
// 通常用于存放转换为实际物理量的数值（如电压值、流量值等）
// 需要在应用程序中手动计算和更新
float ADC_ConvertedValueLocal[NOFCHANEL]={0,0,0};


// 配置ADC引脚-初始化 等离子电压引脚PC2、氦气流量引脚PC1、氩气流量引脚PC3
void ADC_GPIO_Init(void)
{
   GPIO_InitTypeDef ADC_GPIO_InitStruct;
   // 打开 ADC IO 端口时钟
   RCC_APB2PeriphClockCmd(ADC_CLK, ENABLE);
   // 配置 ADC IO 引脚模式
   // 必须为模拟输入
   ADC_GPIO_InitStruct.GPIO_Pin = ADC_VOL_PIN | ADC_HE_FLOW_PIN | ADC_AR_FLOW_PIN;
   ADC_GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
   ADC_GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
   // 初始化 ADC IO
   GPIO_Init(ADC_PORT, &ADC_GPIO_InitStruct);
}

// 配置ADC功能属性
void ADC_Config(void)
{
   // 1、DMA配置
   DMA_InitTypeDef DMA_InitStruct;                    // DMA功能配置结构体
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // 打开 DMA 时钟

   /* ------------------DMA 模式配置---------------- */
   // 复位 DMA 控制器
   DMA_DeInit(ADC_DMA_CHANNEL);
   // 配置 DMA 初始化结构体
   // 外设基址为：ADC 数据寄存器地址
   DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(ADCx->DR));
   // 存储器地址
   DMA_InitStruct.DMA_MemoryBaseAddr = (u32)ADC_ConvertedValue;
   // 数据源来自外设
   DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
   // 缓冲区大小，应该等于数据目的地的大小
   DMA_InitStruct.DMA_BufferSize = NOFCHANEL;
   // 外设寄存器只有一个，地址不用递增
   DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   // 存储器地址递增
   DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
   // 外设数据大小为半字，即两个字节
   DMA_InitStruct.DMA_PeripheralDataSize =DMA_PeripheralDataSize_HalfWord;
   // 内存数据大小也为半字，跟外设数据大小相同
   DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
   // 循环传输模式
   DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
   // DMA 传输通道优先级为高，当使用一个 DMA 通道时，优先级设置不影响
   DMA_InitStruct.DMA_Priority = DMA_Priority_High;
   // 禁止存储器到存储器模式，因为是从外设到存储器
   DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
   // 初始化 DMA
   DMA_Init(ADC_DMA_CHANNEL, &DMA_InitStruct);
   // 使能 DMA 通道
   DMA_Cmd(ADC_DMA_CHANNEL, ENABLE);

   // 2、初始化ADC，配置ADC的工作模式
   ADC_InitTypeDef ADC_InitStruct;
   RCC_ADCCLKConfig(RCC_PCLK2_Div6);                    // 6分频，12MHz
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // 打开时钟

   // 开始配置ADC的工作模式
   // 只使用一个 ADC，属于独立模式
   ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
   // 禁止扫描模式，多通道才要，单通道不需要
   ADC_InitStruct.ADC_ScanConvMode = ENABLE;
   // 连续转换模式
   ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
   // 不用外部触发转换，软件开启即可
   ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
   // 转换结果右对齐
   ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
   // 转换通道 1 个
   ADC_InitStruct.ADC_NbrOfChannel = NOFCHANEL;
   // 初始化 ADC
   ADC_Init(ADCx, &ADC_InitStruct);

   // 配置 ADC 通道转换顺序和采样时间
   ADC_RegularChannelConfig(ADCx, VOL_CHANNEL, 1, ADC_SampleTime_55Cycles5);      // 等离子电压
   ADC_RegularChannelConfig(ADCx, HE_FLOW_CHANNEL, 2, ADC_SampleTime_55Cycles5);  // 氦气流量
   ADC_RegularChannelConfig(ADCx, AR_FLOW_CHANNEL, 3, ADC_SampleTime_55Cycles5);  // 氩气流量
   // ADC 转换结束产生中断，在中断服务程序中读取转换值
   // ADC_ITConfig(ADCx, ADC_IT_EOC, ENABLE);//取消中断

   // 使能 ADC DMA 请求
   ADC_DMACmd(ADCx, ENABLE);
   // 开启 ADC ，并开始转换
   ADC_Cmd(ADCx, ENABLE);
   // 初始化 ADC 校准寄存器
   ADC_ResetCalibration(ADCx);
   // 等待校准寄存器初始化完成
   while (ADC_GetResetCalibrationStatus(ADCx))
      ;
   // ADC 开始校准
   ADC_StartCalibration(ADCx);
   // 等待校准完成
   while (ADC_GetCalibrationStatus(ADCx))
      ;
   // 由于没有采用外部触发，所以使用软件触发 ADC 转换
   ADC_SoftwareStartConvCmd(ADCx, ENABLE);
}

void ADC_NVICConfig(void)
{

   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
   NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}

void ADC_Init1(void)
{
   ADC_GPIO_Init();
   ADC_Config();
   // ADC_NVICConfig();//采用DMA采集的方式，则不用配置中断
}
