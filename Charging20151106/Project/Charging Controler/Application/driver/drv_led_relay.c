#include "stm32f10x.h"
#include "user_lib.h"
#include "drv_led_relay.h"


void LED_RELAY_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | 
                                                RCC_APB2Periph_GPIOB, ENABLE);	
               
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_0 | GPIO_Pin_1;				 //LED0-->PB.5 端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		      //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		      //IO口速度为50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5	
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_9;	    		 //LED1-->PE.5 端口配置, 推挽输出
  GPIO_Init(GPIOC, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz
  
  /*ESP8266 POWER SWITCH*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;	    		 //LED1-->PE.5 端口配置, 推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);	
  
  BAT1_ON();
  BAT2_ON();
  
  RELAY_CHARGE_OFF();
  LED_FAULT_OFF();//fault
}


