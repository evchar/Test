#ifndef __DRV_LED_RELAY_H__
#define __DRV_LED_RELAY_H__

/*Ïê¼ûÓÃ»§ÊÖ²á¼Ä´æÆ÷Æ«ÒÆ*/
#define GPIOA_IDR_OFFSET    ((GPIOA_BASE - PERIPH_BASE) + 8)
#define GPIOA_ODR_OFFSET    ((GPIOA_BASE - PERIPH_BASE) + 12)
#define GPIOB_IDR_OFFSET    ((GPIOB_BASE - PERIPH_BASE) + 8)
#define GPIOB_ODR_OFFSET    ((GPIOB_BASE - PERIPH_BASE) + 12)
#define GPIOC_IDR_OFFSET    ((GPIOC_BASE - PERIPH_BASE) + 8)
#define GPIOC_ODR_OFFSET    ((GPIOC_BASE - PERIPH_BASE) + 12)
#define GPIOD_IDR_OFFSET    ((GPIOD_BASE - PERIPH_BASE) + 8)
#define GPIOD_ODR_OFFSET    ((GPIOD_BASE - PERIPH_BASE) + 12)
#define GPIOE_IDR_OFFSET    ((GPIOE_BASE - PERIPH_BASE) + 8)
#define GPIOE_ODR_OFFSET    ((GPIOE_BASE - PERIPH_BASE) + 12)


#define LED_1          8
#define LED_2          9

#define LED1_Bit            *(uint32_t *)(PERIPH_BB_BASE + GPIOA_ODR_OFFSET * 32 + LED_1 * 4)
#define LED2_Bit            *(uint32_t *)(PERIPH_BB_BASE + GPIOC_ODR_OFFSET * 32 + LED_2 * 4)



#define POW_C1           0   /*PC0*/
#define POW_C            1   /*PC1*/

#define BAT1            0   /*PA0*/
#define BAT2            1   /*PA1*/

#define LD_C            2  /*PC2*/

#define ESP_POWER       1  /*PB1*/
#define ESP_GPIO0       0  /*PB0*/

#define RS485_EN         0  /*PB0*/

#define POW_C_BIT      *(uint32_t *)(PERIPH_BB_BASE + GPIOC_ODR_OFFSET * 32 + POW_C * 4)
#define POW_C1_BIT     *(uint32_t *)(PERIPH_BB_BASE + GPIOC_ODR_OFFSET * 32 + POW_C1 * 4)

#define BAT1_BIT       *(uint32_t *)(PERIPH_BB_BASE + GPIOA_ODR_OFFSET * 32 + BAT1 * 4)
#define BAT2_BIT       *(uint32_t *)(PERIPH_BB_BASE + GPIOA_ODR_OFFSET * 32 + BAT2 * 4)

#define LD_C_BIT       *(uint32_t *)(PERIPH_BB_BASE + GPIOC_ODR_OFFSET * 32 + LD_C * 4)

#define ESP_POWER_BIT  *(uint32_t *)(PERIPH_BB_BASE + GPIOB_ODR_OFFSET * 32 + ESP_POWER * 4)
#define ESP_GPIO0_BIT  *(uint32_t *)(PERIPH_BB_BASE + GPIOB_ODR_OFFSET * 32 + ESP_GPIO0 * 4)

#define RS485_EN_BIT    *(uint32_t *)(PERIPH_BB_BASE + GPIOB_ODR_OFFSET * 32 + RS485_EN * 4)


#define LED1_ON()           LED1_Bit = 1
#define LED1_OFF()          LED1_Bit = 0
#define LED1_TOGGLE()       LED1_Bit ^= 1

#define LED2_ON()           LED2_Bit = 1
#define LED2_OFF()          LED2_Bit = 0
#define LED2_TOGGLE()       LED2_Bit ^= 1


#define ESP8266_ON()           ESP_POWER_BIT = 0
#define ESP8266_OFF()          ESP_POWER_BIT = 1

//#define ESP8266_GPIO0_HIGH()         ESP_GPIO0_BIT = 1
//#define ESP8266_GPIO0_LOW()          ESP_GPIO0_BIT = 0


#define RS485_ENABLE()         RS485_EN_BIT = 1
#define RS485_DISABLE()        RS485_EN_BIT = 0


#define LED_READY_ON()         LED1_ON()
#define LED_READY_OFF()        LED1_OFF()
#define LED_READY_TOGGLE()     LED1_TOGGLE()

#define LED_CHARGE_ON()         LED2_ON()
#define LED_CHARGE_OFF()        LED2_OFF()
#define LED_CHARGE_TOGGLE()     LED2_TOGGLE()

#define LED_FAULT_ON()         POW_C_BIT = 0
#define LED_FAULT_OFF()        POW_C_BIT = 1
#define LED_FAULT_TOGGLE()     POW_C_BIT ^= 1


#define BAT1_ON()          BAT1_BIT = 1
#define BAT2_ON()          BAT2_BIT = 1

#define RELAY_CHARGE_ON()         POW_C1_BIT = 1
#define RELAY_CHARGE_OFF()        POW_C1_BIT = 0

void LED_RELAY_Configuration(void);

#endif
