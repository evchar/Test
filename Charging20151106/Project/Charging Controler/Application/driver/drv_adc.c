#include "stm32f10x.h"
#include "user_lib.h"
#include "drv_adc.h"
#include "drv_led_relay.h"
#include "drv_keys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
//qepn
#include "qpn_port.h"
#include "qepn.h"
#include "qassert.h"
#include "charging.h"
#include "global_value.h"
#include <stdio.h>
u16 __IO Adc1DMAValue[3];//ADC1 DMA采集缓冲区
extern xQueueHandle     msgQueue;
/**
  * @brief  ADC1 Channel Vbat configuration (DMA, ADC, CLK)
  * @param  None
  * @retval None
  */
void ADC1_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_ADCCLKConfig(RCC_PCLK2_Div6);//72M/6=12,ADC最大时钟不能超过14M
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1,ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
  /* Configure ADCCLK such as ADCCLK = PCLK2/6 */

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//AIN3
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;//DMA外设ADC基地址
  DMA_InitStructure.DMA_MemoryBaseAddr     = (u32)Adc1DMAValue;//DMA内存基地址
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;//内存作为数据传输的目的地
  DMA_InitStructure.DMA_BufferSize         = 3;//DMA通道的DMA缓存的大小
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;//外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;//内存地址寄存器递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//数据宽度为16位
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;//数据宽度为16位
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;//工作在循环缓存模式
  DMA_InitStructure.DMA_Priority           = DMA_Priority_High;//DMA通道x拥有高优先级
  DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;//DMA通道x没有设置为内存到内存传输
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
  DMA_Cmd(DMA1_Channel1, ENABLE);

  /* ADC1 configuration ------------------------------------------------------*/
  ADC_DeInit(ADC1);
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//ADC工作模式:ADC1工作在独立模式
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;//模数转换工作在多通道模式
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//DISABLE;//模数转换工作在单次转换模式
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//转换由软件而不是外部触发启动
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC数据右对齐
  ADC_InitStructure.ADC_NbrOfChannel = 3;//顺序进行规则转换的ADC通道的数目
  ADC_Init(ADC1, &ADC_InitStructure);

  for(uint8 i=1; i<4; i++)
    ADC_RegularChannelConfig(ADC1,ADC_Channel_13, i, ADC_SampleTime_239Cycles5);//前面5路外置温度


#if 0
  /* ADC1 regular channel16 configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_239Cycles5);//ADC1,ADC通道16,规则采样顺序值为1,采样时间为239.5周期
  /* Enable the temperature sensor and vref internal channel */
  ADC_TempSensorVrefintCmd(ENABLE);
#endif

  ADC_DMACmd(ADC1, ENABLE);
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibaration register */
  ADC_ResetCalibration(ADC1);//重置指定的ADC1的校准寄存器
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));//获取ADC1重置校准寄存器的状态,设置状态则等待
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);//开始指定ADC1的校准状态
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));//获取指定ADC1的校准程序,设置状态则等待

  /* Start ADC1 Software Conversion */
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能指定的ADC1的软件转换启动功能
}



void insertionSort(uint16_t *numbers, uint32_t array_size)
{
    uint32_t i, j;
    uint32_t index;

  for (i=1; i < array_size; i++) {
    index = numbers[i];
    j = i;
    while ((j > 0) && (numbers[j-1] > index)) {
      numbers[j] = numbers[j-1];
      j = j - 1;
    }
    numbers[j] = index;
  }
}

uint32_t interquartileMean(uint16_t *array, uint32_t numOfSamples)
{
    uint32_t sum=0;
    uint32_t  index;
    /* discard  the lowest and the highest data samples */

    for (index = 1; index < 2; index++){
            sum += array[index];
    }
    /* return the mean value of the remaining samples value*/
    return (uint32_t)( sum / 1.0 );
}




void processTempData(void)
{
  uint32_t tempAVG;
  uint8 temp = 0xff;
  //static uint8 uch_last_msg = 0;
  //static uint32_t lasttemp = 0
  static uint8_t sig_cnt4 = 0;
  static uint8_t sig_cnt5 = 0;
  static uint8_t sig_cnt6 = 0;

  static uint8_t f_check_battery = 0;
  //uint32_t result = 0xffff;
  /* sort received data in */
  insertionSort((uint16*)Adc1DMAValue, 3);

  /* Calculate the Interquartile mean */
  tempAVG = interquartileMean((uint16*)Adc1DMAValue, 3);

  //printf("%f\r\n",tempAVG*3.3/4095);

#if 0
  if(tempAVG > 459 && tempAVG < 657)
  {
    temp = Q_STANDBY_SIG;
  }
  else if(tempAVG > 831 && tempAVG < 1029)
  {
    temp = Q_CAR_DETECTED_SIG;
  }
  else if(tempAVG > 1203 && tempAVG < 1402)
  {
    temp = Q_CAR_REDAY_SIG;
  }
  else
  {
    temp = 0xff;
  }
#endif

#if 1
  if(tempAVG > 1203 && tempAVG < 1402)//1.1
  {

    sig_cnt4 ++;
    sig_cnt5 = 0;
    sig_cnt6 = 0;
    if(sig_cnt4 >= 2)
    {
      temp = Q_CAR_REDAY_SIG;
    }
  }
  else if(tempAVG > 831 && tempAVG < 1029)//0.8
  {

    sig_cnt4 = 0;
    sig_cnt5 ++;
    sig_cnt6 = 0;
    if(sig_cnt5 >= 2)
    {
      f_check_battery = 1;/*触发batteryfull的检测*/
      temp = Q_CAR_DETECTED_SIG;
    }
  }
  else if(tempAVG > 459 && tempAVG < 657)//0.5
  {

    sig_cnt4 = 0;
    sig_cnt5 = 0;
    sig_cnt6 ++;

    if(sig_cnt6 >= 2)
    {
      temp = Q_STANDBY_SIG;
    }
  }
  else
  {

    sig_cnt4 = 0;
    sig_cnt5 = 0;
    sig_cnt6 = 0;

    temp = 0xff;
  }
  if(f_batteryfull && f_check_battery)
  {
      f_batteryfull = 0;
      f_check_battery = 0;      
      temp = Q_CAR_DETECTED_SIG;

  }
  
  //if(result > 200)temp = 0xff;
  //lasttemp = tempAVG;
#endif

  if(temp != g_last_adc_msg && 0xff != temp)
  {
    g_last_adc_msg = temp;
    xQueueSend(msgQueue, &temp, 0);
    //xQueueSendFromISR(msgQueue, &temp, 0);
  }

}

