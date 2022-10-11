#include "sys.h"
#include "usart.h"	
#include "data_transfer.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F4探索者开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/6/10
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
////////////////////////////////////////////////////////////////////////////////// 	  
 uint8_t TxBuffer[256];
uint8_t count=0;
uint8_t BTNumberSend[15] = {0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0, 0,0};//sendTemp
uint8_t BTNumberTrue[10] = {0,0,0,0,0, 0,0,0,0,0};//vlidNum
uint16_t BTNumberTrue_16[5] = {0,0,0,0,0};
uint8_t BTNumberRece[16] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,};
uint8_t BTNumberErrorTrue[15] = {0,0,0,0, 0,0,0,0, 0,0};
Bluetooth_Queue Bluetooth_queue;
int Bluth_waitTime = 0;
uint8_t receFlag = 0;//??????????
uint8_t BTNumberReceCount = 0;
uint8_t receErrorTime = 0;
uint8_t headcount = 0;//帧头判断标志位
uint8_t head2count = 0;//帧头判断2
int Bluetooth_start = 0 ;
int16_t HIGH,Speed_set=0,D_yaw_control=50,D_pitch_control=50,D_roll_control=50;//速度频率设置
int16_t P,I,D; 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	

u8 Tx1Buffer[256];
u8 Tx1Counter=0;
u8 count1=0;
int  USART_state =0 ; 
//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif
	
}


void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		} 
		Bluetoot_Receive_isr1(Res);	
		USTB_DT_Data_Receive_Prepare(Res);		
  } 
} 
#endif	

 void USART1_DataSend(uint8_t data)
{
    while((USART1->SR & 0X40) == 0); //循环发送,直到发送完毕
    USART1->DR = data;
}

void UART_SendBytes(const uint8_t *dt, uint32_t n)
{
    while (n--)
    {
        BITBAND_REG(USART1->SR, 6) = 0;
        USART1->DR = *dt++;
        while (!BITBAND_REG(USART1->SR, 6));
    }

}
/**********************************************************
Function Name:Usart2_Send
Description:数据包发送服务函数
Inputs:  unsigned char *DataToSend ,uint8_t data_num
Outputs: None
Notes:数据数组定长   
***********************************************************/
void Usart1_Send(unsigned char *DataToSend ,uint8_t data_num)
{
 uint8_t i;
	for(i=0;i<data_num;i++)
	{
		TxBuffer[count++] = *(DataToSend+i);
	}

	if(!(USART1->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
	}
}
/**********************************************************
Function Name:Bluetoot_Receive_isr
Description:修改后的协议解析函数  
Inputs:  uint8_t temp
Outputs: None
Notes:去掉CRC校验加上另外位置数据包   
***********************************************************/

uint8_t BTreceCRCTemp = 0;//奇偶校验中间变量
uint8_t BTreceCRCFlag = 0;//奇偶校验标志
uint8_t receive;
u8 auto_control=0;
void Bluetoot_Receive_isr1(uint8_t temp)
{
    int i;
    receive = temp;
    if(receFlag == 1)//调节PID和设定高度
    {
        BTNumberRece[BTNumberReceCount] = temp;
        BTNumberReceCount ++;
        //接受一个数据包完成
        if(BTNumberReceCount >= 15)
        {
            receFlag = 0;
            BTNumberReceCount = 0;
                
            BTreceCRCTemp = 0;//奇偶校验中间变量
            BTreceCRCFlag = 0;//奇偶校验标志
                
            for(i = 2; i < 12; i++)
            {
                BTreceCRCTemp = BTreceCRCTemp + BTNumberRece[i]%2;
            }
            if(BTNumberRece[12] == BTreceCRCTemp && BTNumberRece[13] == 0xf0 && BTNumberRece[14] == 0xf0)
                BTreceCRCFlag = 1;
                
            if(BTreceCRCFlag == 1)//接受数据准确存队列
            {
                auto_control = BTNumberRece[5];
                D_yaw_control =BTNumberRece[7]+BTNumberRece[6]*256;//Kd
                Speed_set=(BTNumberRece[9]+BTNumberRece[8]*256);//设定高度	
                D_pitch_control=BTNumberRece[11]+BTNumberRece[10]*256;//实际高度	      
								USART_state = 1;	
            }

        }
    }
    else
    {
        //判断帧头
        if(temp == 0xff)
        {
            headcount ++;
        }
        else
        {
            headcount = 0;
        }
        if(headcount >= 2)
        {
            headcount = 0;
            receFlag = 1;
            BTNumberRece[0] = 0xff;
            BTNumberRece[1] = 0xff;
            BTNumberReceCount = 2;
        }
    } 
}
void Bluetoot_Receive_isr(uint8_t temp)
{
    int i;
    receive = temp;
    if(receFlag == 1)//调节PID和设定高度
    {
        BTNumberRece[BTNumberReceCount] = temp;
        BTNumberReceCount ++;
        //接受一个数据包完成
        if(BTNumberReceCount >= 4)
        {
 
          Speed_set=(BTNumberRece[2]);//设定油门
	      
        }
    }
    else
    {
        //判断帧头
        if(temp == 0xee || (headcount==1 && temp == 0xff))
        {
            headcount ++;
        }
        else
        {
            headcount = 0;
        }
        if(headcount >= 2)
        {
            headcount = 0;
            receFlag = 1;
            BTNumberRece[0] = 0xee;
            BTNumberRece[1] = 0xff;
            BTNumberReceCount = 2;
        }
    } 
}


/**********************************************************
Function Name:enQueue deQueue
Description:将收到数据压入堆栈 ，防止数据丢失
Inputs:  None
Outputs: None
Notes:   
***********************************************************/
void Bluetooth_enter_Queue(struct Bluetooth_Queue * pq, uint8_t x)
{
  pq -> key[pq -> tail] = x;

  if(pq -> tail >= pq -> length) 
  {
    pq -> tail = 0;
  } 
  else 
  {
    pq -> tail++;
  }
}

uint8_t Bluetooth_exit_Queue(struct Bluetooth_Queue * pq)
{
  uint8_t x;
  
  x = pq -> key[pq -> head];
  if(pq -> head >= pq -> length) {
    pq -> head = 0;
  } 
  else 
  {
    pq -> head++;
  }
  return x;
}

/**********************************************************
Function send_Bluetooth_Data
Description:将数据打包发出去
Inputs:  None
Outputs: None
Notes: 暂时没用  
***********************************************************/
void send_Bluetooth_Data(int16_t send_Bluetooth_Data[5] )
{
  uint8_t BTsendCRCTemp = 0;
  int16_t dataTemp = 0;
	int i;
  
  //???
  BTNumberSend[0] = 0xff;
  BTNumberSend[1] = 0xff;
 
  //data*20
  //1
  dataTemp = (int16_t)send_Bluetooth_Data[0];
  BTNumberSend[2] = dataTemp >> 8;
  BTNumberSend[3] = dataTemp;
  //2
  dataTemp = (int16_t)send_Bluetooth_Data[1];
  BTNumberSend[4] = dataTemp >> 8;
  BTNumberSend[5] = dataTemp;
  //3
  dataTemp = (int16_t)send_Bluetooth_Data[2];
  BTNumberSend[6] = dataTemp >> 8;
  BTNumberSend[7] = dataTemp;
  //4
  dataTemp = (int16_t)send_Bluetooth_Data[3];
  BTNumberSend[8] = dataTemp >> 8;
  BTNumberSend[9] = dataTemp;
  //5
  dataTemp = (int16_t)send_Bluetooth_Data[4];
  BTNumberSend[10] = dataTemp >> 8;
  BTNumberSend[11] = dataTemp;
  
  for(i = 2; i < 12; i++)
  {
    BTsendCRCTemp = BTsendCRCTemp + BTNumberSend[i]%2;
  }
  BTNumberSend[12] = BTsendCRCTemp;
  //???
  BTNumberSend[13] = 0xf0;
  BTNumberSend[14] = 0xf0;
	
	Usart1_Send(BTNumberSend,15);
} // handleSendData



