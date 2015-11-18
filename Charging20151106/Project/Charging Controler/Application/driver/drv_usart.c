/**
 * @file drv_usart.c
 *
 * @note This file contains the source code for the console
 *       and serial port support functions.
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "drv_usart.h"
#include <stdio.h>
#include "user_lib.h"
#include "global_value.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "drv_led_relay.h"
extern xSemaphoreHandle UsartTimeoutSem;

void Usart4Init( void )
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);	//使能DMA传输
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);//使能GPIOC时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能USART2时钟
 
  
  DMA_DeInit(DMA2_Channel5);   //将DMA的通道1寄存器重设为缺省值
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;//DMA外设ADC基地址
  DMA_InitStructure.DMA_MemoryBaseAddr     = (u32)g_rev_usart1.auch_send_buffer;//DMA内存基地址
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;//内存作为数据传输的目的地
  //DMA_InitStructure.DMA_BufferSize         = 10;//DMA通道的DMA缓存的大小
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;//外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;//内存地址寄存器递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//数据宽度为16位
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_PeripheralDataSize_Byte;//数据宽度为16位
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;//工作在非循环模式,发送完指定长度数据即停止
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;//DMA通道x拥有高优先级
  DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;//DMA通道x没有设置为内存到内存传输
  DMA_Init(DMA2_Channel5, &DMA_InitStructure);
  
  DMA_ITConfig(DMA2_Channel5, DMA_IT_TC, ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel4_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	//PC10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;    //PC11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
  
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4,ENABLE);//复位串口3
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4,DISABLE);//停止复位
    
  USART_DeInit(UART4);
  USART_InitStructure.USART_BaudRate = 115200;//波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据长度
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;///奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发模式
  
  USART_Init(UART4, &USART_InitStructure);//初始化串口
  
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
  USART_ClearFlag(UART4 , USART_FLAG_TC); 
  USART_Cmd(UART4, ENABLE);                    //使能串口 
}


/**
 * @brief Initialize console i/o support
 *
 * This function initializes the usart interface of the STM32 uController
 */
void ConsoleInit(void)
{

  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能GPIOA时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;		//PD7端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	//PA2
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
  
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3,ENABLE);//复位串口3
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3,DISABLE);//停止复位
  
  USART_DeInit(USART3);
  
  USART_InitStructure.USART_BaudRate = 115200;//波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据长度
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;///奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发模式
  
  USART_Init(USART3, &USART_InitStructure);  //初始化串口
  
  USART_Cmd(USART3, ENABLE);                    //使能串口 
  
  RS485_ENABLE();
  
}  
/******************************************************************
** 函数名称: USART4SendByte
** 功能描述: 发送一个字节
** 日　  期: 2013.8.6
*******************************************************************/
void USART4SendByte(uint8 Data)
{ 
  while (!(UART4->SR & USART_FLAG_TXE));
  UART4->DR = (Data & (uint16_t)0x01FF);	
}

/******************************************************************
** 函数名称: USART1SendBuffer
** 功能描述: 发送指定长度的字符串,数组
** 日　  期: 2013.8.6
*******************************************************************/
void USART4SendBuffer(u8 *ptr,uint8_t count)
{
  while(count--)
  {
    USART4SendByte(*ptr++);
  }
}


void Usart4RevBufInit( void )
{
  g_rev_usart1.uch_rev_cnt = 0;
  g_rev_usart1.uch_timeout = 0;
  g_rev_usart1.uch_parse_len = 0;
  g_rev_usart1.b_parse_busy = false;
  g_rev_usart1.p_rev = g_rev_usart1.auch_rev_buffer[0];
  g_rev_usart1.p_parse = NULL;   
}

/**
  * @brief  swapRevBuf
  * @param  None
  * @retval None
  */
int SwapRevBuf(usart_data_rev_t* rev_usart)
{
  if(true == rev_usart->b_parse_busy)
    return -1;
  
  rev_usart->b_parse_busy = true;
  
  rev_usart->p_parse = rev_usart->p_rev;
  
  rev_usart->uch_parse_len = rev_usart->uch_rev_cnt;
  
  if(rev_usart->p_rev == rev_usart->auch_rev_buffer[0])
  {
    rev_usart->p_rev = rev_usart->auch_rev_buffer[1];
  }
  else
  {
    rev_usart->p_rev = rev_usart->auch_rev_buffer[0];
  }  
  
  rev_usart->uch_rev_cnt = 0;
  
  return 0;
}



void UART4_IRQHandler(void)
{ 
  volatile u8 dummy;
  portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;
  /*接收寄存器非空，接收串口的数据*/
  if(USART_GetITStatus(UART4,USART_IT_RXNE) != RESET)
  {
    dummy = (u8)UART4->DR ;
    
    g_rev_usart1.uch_timeout = USART_REV_TIMEOUT;
    
    g_rev_usart1.p_rev[g_rev_usart1.uch_rev_cnt++] = dummy;
    
    if( g_rev_usart1.uch_rev_cnt >= MAX_REV_LEN )
    {
      if(SwapRevBuf(&g_rev_usart1) < 0)
      {
        g_rev_usart1.uch_rev_cnt = 0;
      }
      
      xSemaphoreGiveFromISR(UsartTimeoutSem,&xHigherPriorityTaskWoken);
      
      if( xHigherPriorityTaskWoken != pdFALSE )
      {
        portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
      }      
    }
  }
}

