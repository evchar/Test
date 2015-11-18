#include "stm32f10x.h"
#include "user_lib.h"
#include "drv_spi.h"
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/**
 * SPI1_Configuration:
 * SPI1_Configuration() initializes the spi interface on the microcontroller
 * Returns: none
 */
void SPI1_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
  //EXTI_InitTypeDef EXTI_InitStructure;
  //NVIC_InitTypeDef NVIC_InitStructure;
  
    /* Enable the SPI clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOB, ENABLE);  

  /* Configure SPI1 pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Configure I/O for Chip select */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//SPI CS
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  SPI_I2S_DeInit(SPI1);
  
  /* SPI1 configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//设置SPI工作模式:设置为主SPI
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//设置SPI的数据大小:SPI发送接收8位帧结构
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//;//选择了串行时钟的稳态:时钟悬空低
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//;//数据捕获于第1个时钟沿
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//SPI_BaudRatePrescaler_32;//定义波特率预分频的值
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
  SPI_InitStructure.SPI_CRCPolynomial = 7;//CRC值计算的多项式
  SPI_Init(SPI1, &SPI_InitStructure);//根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
  //SPI_SSOutputCmd(SPI1,ENABLE);//SS输出
  SPI_Cmd(SPI1, ENABLE);//使能SPI外设
  
}

/*************************************************
Function: SpiTxRxByte(uint8_t byte)
Description: Spi收发一字节数据
Input: 发送的一字节数据
Output: 接收的一字节数据
Return:
*************************************************/
uint8_t SpiTxRxByte(uint8_t byte)
{
  // Loop while DR register in not emplty
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  
  //Send byte through the SPI2 peripheral
  SPI_I2S_SendData(SPI1, byte);
  
  // Wait to receive a byte
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  
  // Return the byte read from the SPI bus
  return SPI_I2S_ReceiveData(SPI1);
}

/**
  * @brief  This function handles External line 0 interrupt request.
  *         This is an interrupt service routine for the nanochip.
  *         It calls the nanoInterrupt service routine.
  * @param  None
  * @retval None
  */

void EXTI9_5_IRQHandler(void)
{
  portBASE_TYPE  xHigherPriorityTaskWoken = pdFALSE;
  if(EXTI_GetITStatus(EXTI_Line5) != RESET)
  {
    //g_interrupt_cnt++;
    EXTI_ClearITPendingBit(EXTI_Line5);
    //xQueueSendFromISR(nanoisr, NULL,&xHigherPriorityTaskWoken);
    //xSemaphoreGiveFromISR( nanoisr, &xHigherPriorityTaskWoken );
    /* Switch tasks if necessary. */
    if( xHigherPriorityTaskWoken != pdFALSE )
    {
      portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }
  }
}


