/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "user_lib.h"
#include "i2cdev.h"
#include "i2croutines.h"

/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "drv_adc.h"
extern void xPortSysTickHandler(void); 

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  xPortSysTickHandler(); 
}


/**
  * @brief  This function handles DMA1_Channel1_IRQHandler Handler.
            for adc channel 13
  * @param  None
  * @retval None
  */
void DMA1_Channel1_IRQHandler(void)
{
  if(DMA_GetFlagStatus(DMA1_FLAG_TC1) != RESET)
  {
    DMA_ClearFlag(DMA1_FLAG_TC1);
  }
}

/**
  * @brief  This function handles DMA1_Channel1_IRQHandler Handler.
            for adc channel 13
  * @param  None
  * @retval None
  */
void DMA2_Channel4_5_IRQHandler(void)
{
  if(DMA_GetFlagStatus(DMA2_FLAG_TC5) != RESET)
  {
    DMA_ClearFlag(DMA2_FLAG_TC5);
  }
}
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void DMA1_Channel4_IRQHandler(void)
{
  i2cDmaInterruptHandlerI2c2();
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void DMA1_Channel5_IRQHandler(void)
{
  i2cDmaInterruptHandlerI2c2();
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void DMA1_Channel6_IRQHandler(void)
{
  i2cDmaInterruptHandlerI2c1();
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void DMA1_Channel7_IRQHandler(void)
{
  i2cDmaInterruptHandlerI2c1();
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void I2C1_EV_IRQHandler(void)
{
  i2cInterruptHandlerI2c1();
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void I2C1_ER_IRQHandler(void)
{
  i2cErrorInterruptHandlerI2c1();
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void I2C2_EV_IRQHandler(void)
{

}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void I2C2_ER_IRQHandler(void)
{
  I2C_ClearFlag(I2C2, 0x1000FFFF);
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
