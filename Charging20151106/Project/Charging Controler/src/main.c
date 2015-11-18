/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
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
#include "stm32f10x.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "global_value.h"
#include "semphr.h"

#include "drv_usart.h"
#include "self_buffer.h"
#include "esp8266.h"
#include "drv_adc.h"
#include "drv_led_relay.h"
#include "drv_timer.h"
#include "rn8209.h"
#include "protocol.h"

void LED0Task(void * pvParameters);
void PowerMeterTask(void * pvParameters);
void UartSendTask(void * pvParameters);

extern void WifiMoudleTask(void * pvParameters);
extern void WifiParseTask(void * pvParameters);
extern void Mpu6050Task(void *pvParameters);
extern void TestTask(void * pvParameters);
extern void KeyTask(void * pvParameters);

/* Synchronisation */
xSemaphoreHandle UsartSendSem;
extern xSemaphoreHandle UsartTimeoutSem;


void vApplicationTickHook( void )
{
  if(g_rev_usart1.uch_timeout > 0)
  {
    g_rev_usart1.uch_timeout--;
    if(g_rev_usart1.uch_timeout == 0)
    {
      SwapRevBuf(&g_rev_usart1);
      xSemaphoreGive(UsartTimeoutSem);
    }
  }
}
void vApplicationStackOverflowHook( xTaskHandle pxTask, char *pcTaskName )
{
  ( void ) pcTaskName;
  ( void ) pxTask;

  printf("%s",pcTaskName);
  /* Run time stack overflow checking is performed if
  configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
  function is called if a stack overflow is detected. */
  for( ;; );
}

char PP_BODY[512];
void vApplicationIdleHook( void )
{
//  static unsigned int cnt = 0;
//  cnt++;
//  if(cnt == 900000)
//  {
//   cnt = 0;
//   //printf("idle\r\n");
//   vTaskList((signed char *)(PP_BODY));
//   printf("%s",PP_BODY);
//  }
  /* Request Wait For Interrupt */
  __WFI();
}

void vApplicationMallocFailedHook( xTaskHandle pxTask, char *pcTaskName )
{
  ( void ) pcTaskName;
  ( void ) pxTask;

  printf("ma:%s",pcTaskName);
  /* Run time stack overflow checking is performed if
  configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
  function is called if a stack overflow is detected. */
  for( ;; );
}
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
 USART_InitTypeDef USART_InitStructure;

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
while(1);
  ConsoleInit();

  TIM2_Configuration();

  LED_RELAY_Configuration();

  ADC1_Configuration();

  g_device_id = get_device_sn();  //g_device_id = 0x271e7964;

  printf("STM32 running\r\n");

  xTaskCreate(WifiParseTask, "WifiParse",
              4*configMINIMAL_STACK_SIZE, NULL, 6, NULL);

  xTaskCreate(UartSendTask, "Send",
              4*configMINIMAL_STACK_SIZE, NULL, 5, NULL);

  //xTaskCreate(Mpu6050Task, "mpu6050",
  //            4*configMINIMAL_STACK_SIZE, NULL, 3, NULL);

  xTaskCreate(LED0Task, "LED",
             1*configMINIMAL_STACK_SIZE, NULL, 3, NULL);

  //xTaskCreate(PowerMeterTask, "POW",
  //           1*configMINIMAL_STACK_SIZE, NULL, 3, NULL);

  xTaskCreate(TestTask, "TEST",
              1*configMINIMAL_STACK_SIZE, NULL, 4, NULL);

  xTaskCreate(KeyTask, "KEY",
              1*configMINIMAL_STACK_SIZE, NULL, 5, NULL);

  /* Start scheduler */
  vTaskStartScheduler();

  /* Infinite loop */
  for(;;);
}


/**
* @brief  串口发送任务；
* @param  None
* @retval None
* @note
*/
void UartSendTask(void * pvParameters)
{
  uint8 uch_length,uch_id;
  UsartSendSem = xSemaphoreCreateCounting( 5, 0 );

  for( ;; )
  {
    if(xSemaphoreTake(UsartSendSem, portMAX_DELAY) == pdTRUE)
    {
      if(QueueDelete(g_rev_usart1.auch_send_buffer,&uch_length, &uch_id))
      {
        DMA_Cmd(DMA2_Channel5, DISABLE );  //关闭UART4 TX DMA1 所指示的通道
        DMA_SetCurrDataCounter(DMA2_Channel5, uch_length);//DMA通道的DMA缓存的大小
        DMA_Cmd(DMA2_Channel5, ENABLE);  //使能UART4 TX DMA1 所指示的通道
      }
    }
  }
}


/**
  * @brief  Toggle Led4 task
  * @param  pvParameters not used
  * @retval None
  */
void LED0Task(void * pvParameters)
{

  for( ;; )
  {
    processTempData();
    /* Start ADC1 Software Conversion */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能指定的ADC1的软件转换启动功能
    
    SendChargingstatus();
    
    vTaskDelay(20);
  }
}

/**
  * @brief  PowerMeterTask
  * @param  pvParameters not used
  * @retval None
  */
void PowerMeterTask(void * pvParameters)
{
  static uint8_t sig_pf_cnt = 0;
  RN8209_Chip_Init();
  
  for( ;; )
  {
    RN8209_GET_UFREQ();
    RN8209_GET_URMS();
    RN8209_GET_IARMS();
    RN8209_GET_ENERGYA();

    printf("pf:%d\r\n",g_pf_cnt);
 
      if((g_pf_cnt-g_pre_pf_cnt)<3)/*充电停止*/
      {
        sig_pf_cnt++;
        if(sig_pf_cnt>4)
        {
           sig_pf_cnt = 0;
           f_batteryfull = 1;
        }
      }            
    g_pre_pf_cnt =  g_pf_cnt;
    vTaskDelay(3000);
  }
}
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
  {}

  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART3, (uint8_t) ch);

  return ch;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
