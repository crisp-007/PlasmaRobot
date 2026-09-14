#include "usart1.h"

int RecvBuf_Index=0;
char RecvBuf[BUFSIZE]="";
int bufReady=0;
int ifPrintf=1;

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 配置NVIC中断分组为组2 */
	/* 使用 NVIC_PriorityGroupConfig() 来设置中断优先级分组*/
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* 配置USART中断 */
  NVIC_InitStructure.NVIC_IRQChannel = USART_IRQn;
  /* 抢占优先级*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  /* 子优先级 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  /* 使能中断 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* 初始化配置NVIC */
  NVIC_Init(&NVIC_InitStructure);
}

//初始化usart2用于printf重定向
void Usart1_Init()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能usart2时钟

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能usart2对应的GPIO时钟

    //配置发送引脚
    GPIO_InitTypeDef TxGpio_Init;
    TxGpio_Init.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
    TxGpio_Init.GPIO_Pin=Usart_Tx;
    TxGpio_Init.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(Usart_Port,&TxGpio_Init);

    //配置接收引脚
    GPIO_InitTypeDef RxGpio_Init;
    RxGpio_Init.GPIO_Mode=GPIO_Mode_IN_FLOATING;//浮空输入模式
    RxGpio_Init.GPIO_Pin=Usart_Rx;
    RxGpio_Init.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(Usart_Port,&RxGpio_Init);

    //usart2配置
    USART_InitTypeDef usart_init;
    //设置波特率
    usart_init.USART_BaudRate=Usart1_Rate;//波特率设置为115200
    //硬件流控制
	usart_init.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件流控制
    //工作模式
	usart_init.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//收发模式
    //奇偶校验
	usart_init.USART_Parity=USART_Parity_No;//无奇偶校验
    //停止校验位
	usart_init.USART_StopBits=USART_StopBits_1;//停止位为1位
    //数据字长位
	usart_init.USART_WordLength=USART_WordLength_8b;//数据位为8位
	USART_Init(Usartx,&usart_init);
	
    //串口中断配置
    NVIC_InitTypeDef NVIC_InitStructure;
  
    /* 配置NVIC中断分组为组2 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    /* 配置USART2中断 */
    NVIC_InitStructure.NVIC_IRQChannel = USART_IRQn;
    /* 抢占优先级*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    /* 子优先级 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    /* 使能中断 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* 初始化配置NVIC */
    NVIC_Init(&NVIC_InitStructure);
    
    // 使能串口接收中断
	USART_ITConfig(Usartx, USART_IT_RXNE, ENABLE);

	USART_Cmd(Usartx,ENABLE);//启用USART外设
}

//初始化usart1用于数据包发送
void CmdUsart_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能usart1时钟

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能usart1对应的GPIO时钟

    //配置发送引脚
    GPIO_InitTypeDef TxGpio_Init;
    TxGpio_Init.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
    TxGpio_Init.GPIO_Pin=CmdUsart_Tx;
    TxGpio_Init.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(CmdUsart_Port,&TxGpio_Init);

    //配置接收引脚
    GPIO_InitTypeDef RxGpio_Init;
    RxGpio_Init.GPIO_Mode=GPIO_Mode_IN_FLOATING;//浮空输入模式
    RxGpio_Init.GPIO_Pin=CmdUsart_Rx;
    RxGpio_Init.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(CmdUsart_Port,&RxGpio_Init);

    //usart1配置
    USART_InitTypeDef usart_init;
    //设置波特率
    usart_init.USART_BaudRate=CmdUsart_Rate;//波特率设置为115200
    //硬件流控制
	usart_init.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件流控制
    //工作模式
	usart_init.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//收发模式
    //奇偶校验
	usart_init.USART_Parity=USART_Parity_No;//无奇偶校验
    //停止校验位
	usart_init.USART_StopBits=USART_StopBits_1;//停止位为1位
    //数据字长位
	usart_init.USART_WordLength=USART_WordLength_8b;//数据位为8位
	USART_Init(CmdUsart,&usart_init);
	
    //串口中断配置
    NVIC_InitTypeDef NVIC_InitStructure;
  
    /* 配置NVIC中断分组为组2 */
        /* 使用 NVIC_PriorityGroupConfig() 来设置中断优先级分组*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    /* 配置USART1中断 */
    NVIC_InitStructure.NVIC_IRQChannel = CmdUSART_IRQn;
    /* 抢占优先级*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    /* 子优先级 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    /* 使能中断 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* 初始化配置NVIC */
    NVIC_Init(&NVIC_InitStructure);

    // 使能串口接收中断
	USART_ITConfig(CmdUsart, USART_IT_RXNE, ENABLE);

	USART_Cmd(CmdUsart,ENABLE);//启用USART外设
}

//发送一个字节
void Usart1_SendByte(uint8_t strData)
{
    //发送一个字节数据
    USART_SendData(Usartx, strData);
    //等待发送数据寄存器为空
    while(USART_GetFlagStatus(Usartx,USART_FLAG_TXE)==RESET){}
}

//发送一个字符串
void Usart1_SendStr(char *strData)
{
    int len=0;
    USART_ClearFlag(Usartx,USART_FLAG_TC);
    do{
        Usart1_SendByte(*(strData+len));
        len++;
    }while(*(strData+len)!='\0');
    //等待发送完成标志位置位
    while(USART_GetFlagStatus(Usartx,USART_FLAG_TC)==RESET){}
}


int fputc(int ch,FILE *f)//printf重定向函数
{
	USART_SendData(Usartx,(uint8_t)ch);//发送数据
	while(USART_GetFlagStatus(Usartx,USART_FLAG_TXE)==RESET);//等待发送数据寄存器空标志位置位
	  
	return (ch);
}


int fgetc(FILE *f)//接收数据函数
{
	while(USART_GetFlagStatus(Usartx,USART_FLAG_RXNE)==RESET);//等待接收数据寄存器非空标志位置位

    return (int)USART_ReceiveData(Usartx);//返回接收到的数据
}


//发送数据函数

void USART_SendOneByte(uint16_t Data) 
{
  USART_SendData(CmdUsart, Data);
  while (USART_GetFlagStatus(CmdUsart, USART_FLAG_TC) == RESET); // 等待发送完成
}

// 发送一个 8 位数据
void USART_SendU8(uint8_t data) {
    USART_SendOneByte(data);
    while (USART_GetFlagStatus(CmdUsart, USART_FLAG_TC) == RESET); // 等待发送完成
}

// 发送一个 16 位数据（小端序）
void USART_SendU16(uint16_t data) {
    USART_SendOneByte((uint8_t)(data & 0xFF));       // 低字节
    USART_SendOneByte((uint8_t)((data >> 8) & 0xFF)); // 高字节
}

// 发送一个 32 位数据（小端序）
void USART_SendU32(uint32_t data) {
    USART_SendOneByte((uint8_t)(data & 0xFF));             // 最低字节
    USART_SendOneByte((uint8_t)((data >> 8) & 0xFF));      // 次低字节
    USART_SendOneByte((uint8_t)((data >> 16) & 0xFF));     // 次高字节
    USART_SendOneByte((uint8_t)((data >> 24) & 0xFF));     // 最高字节
}

