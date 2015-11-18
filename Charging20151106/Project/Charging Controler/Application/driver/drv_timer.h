#ifndef __DRV_TIMER_H__
#define __DRV_TIMER_H__

#define TIMER_INTERVAL  10

void TIM1_Configuration(void);

void TIM2_Configuration(void);

void TIM4_Configuration(void);

#define SetCharingLevel(a)     TIM_SetCompare3(TIM4, 100*a)

#if 0
#define START_PWM()            do{\
                                  GPIO_InitTypeDef GPIO_InitStructure;\
                                  TIM_CCxCmd(TIM2,TIM_Channel_4,TIM_CCx_Enable);\
                                  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;\
                                  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;\
                                  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;\
                                  GPIO_Init(GPIOA, &GPIO_InitStructure);\
                                  }while(0);
 
#define STOP_PWM()            do{\
                                GPIO_InitTypeDef GPIO_InitStructure;\
                                TIM_CCxCmd(TIM2,TIM_Channel_4,TIM_CCx_Disable);\
                                GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;\
                                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;\
                                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;\
                                GPIO_Init(GPIOA, &GPIO_InitStructure);\
                                GPIO_SetBits(GPIOA,GPIO_Pin_3);\
                               }while(0);
#else 

#define START_PWM()            do{\
                                  TIM_CCxCmd(TIM2,TIM_Channel_4,TIM_CCx_Enable);\
                                  GPIOA->CRL = (GPIOA->CRL & ~(0xf << (3 * 4))) + (0xA << (3 * 4));\
                                 }while(0);
 
#define STOP_PWM()            do{\
                                 TIM_CCxCmd(TIM2,TIM_Channel_4,TIM_CCx_Disable);\
                                 GPIOA->CRL = (GPIOA->CRL & ~(0xf << (3 * 4))) + (0x2 << (3 * 4));\
                                 *(u32 *)(PERIPH_BB_BASE + GPIOA_ODR_OFFSET * 32 + 3 * 4) = 1;\
                                }while(0);

#endif

#endif//__DRV_TIMER_H__