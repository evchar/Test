#include "stm32f2xx.h"
#include "drv_system.h"
#include "user_typedefine.h"
#include "drv_adc.h"
#include "drv_led.h"
#include "drv_i2c24cxx.h"
#include "global_value.h"

void IWDG_Configuration(void)
{
    //分频数为32,重载值为2500,溢出时间为2s
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);//使能对寄存器IWDG_PR和IWDG_RLR的写操作
    while (IWDG->SR & 0x01);
    IWDG_SetPrescaler(IWDG_Prescaler_32);//设置IWDG预分频值:设置IWDG预分频值为32
    while (IWDG->SR & 0x02);
    IWDG_SetReload(2500);//设置IWDG重装载值
    IWDG_ReloadCounter();//按照IWDG重装载寄存器的值重装载IWDG计数器
    IWDG_Enable();//使能IWDG
    //RCC_LSICmd(ENABLE);
    //while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET);
}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable the GPIO_LED Clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG , ENABLE);//LDO
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);//LDO
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//EEPROM
    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOG, &GPIO_InitStructure);

    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    /****************EEPROM***************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void System_Configuration(void)
{
    #ifdef  CONFIG_CONSOLE
    console_init();
    #endif  /* CONFIG_CONSOLE */
    GPIO_Configuration();
    ETH_PWR_ON();
    RF0_PWR_ON();  

    EEPROMInit();
    /* configure LED Light */ 
    LED_Configuration();
}

//配置系统时钟,使能各外设时钟
void RCC_Configuration(void)
{
    //SystemInit(); //不必再次调用，在startup已调用此函数，默认120M主频
    #ifdef  CONFIG_CONSOLE
    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//console
    /* Enable USART1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);//console
    #endif  /* CONFIG_CONSOLE */

    /* Enable the SPI clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);  //NA5TR1
     /* Enable GPIO clocks */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  //NA5TR1
    
    #ifdef CONFIG_NTRX_IRQ
    /* Enable GPIOB clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    #endif  /* CONFIG_NTRX_IRQ */
       
}

//系统中断管理
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    //设置向量表的位置和偏移
    #ifdef  VECT_TAB_RAM
    /* Set the Vector Table base location at 0x20000000 */
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);//向量表位于RAM
    #else  /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00000);//向量表位于FLASH
    #endif
    
    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    #ifdef  CONFIG_CONSOLE
    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    #endif  /* CONFIG_CONSOLE */

    /* Enable the USART3 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
   
    #ifdef CONFIG_NTRX_IRQ
    /* Enable and set EXTI Line5 Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    #endif  /* CONFIG_NTRX_IRQ */
    
    //定时器2中断
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    #ifdef CONFIG_ETH_IRQ
      /* Enable the Ethernet global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = ETH_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);    
    #endif
}

//配置所有外设
void Init_All_Periph(void)
{
    RCC_Configuration();

    NVIC_Configuration();

    System_Configuration();
}

