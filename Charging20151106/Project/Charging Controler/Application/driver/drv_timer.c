#include "stm32f10x.h"
#include "user_lib.h"
#include "drv_led_relay.h"
#include "drv_timer.h"

void TIM2_Configuration(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  //GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//使能定时器3时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIO外设
  
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//TIM1-CH1
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  TIM_DeInit(TIM2);
  
  TIM_TimeBaseStructure.TIM_Period = 2000;//1ms
  TIM_TimeBaseStructure.TIM_Prescaler = 35;//36分频
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//72M
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  //TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  /* PWM1 模式配置: Channel3 Channel4*/
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
  TIM_OCInitStructure.TIM_Pulse = 400;//400*g_InsProperty.uch_LCDLightGrade;//高电平宽度,此时占空比
  //TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//输出极性:TIM输出比较极性高
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);//根据TIM_OCInitStruct中指定的参数初始化外设TIMx
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);//使能TIMx在CCR3上的预装载寄存器
  TIM_ARRPreloadConfig(TIM2, ENABLE);//使能TIMx在ARR上的预装载寄存器
  
  START_PWM();
  TIM_Cmd(TIM2, ENABLE);//使能TIMx外设
  
  
}
