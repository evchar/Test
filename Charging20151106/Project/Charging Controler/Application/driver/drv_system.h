#ifndef __DRV_SYSTEM_H__
#define __DRV_SYSTEM_H__

//Î¹¹·
#ifdef USER_DEBUG
#define FeedDog()
#define IWDG_Config()
#else
#define FeedDog()            IWDG_ReloadCounter()
#define IWDG_Config()        IWDG_Configuration()
#endif

#define GPIOA_IDR_OFFSET    ((GPIOA_BASE - PERIPH_BASE) + 16)
#define GPIOA_ODR_OFFSET    ((GPIOA_BASE - PERIPH_BASE) + 20)
#define GPIOB_IDR_OFFSET    ((GPIOB_BASE - PERIPH_BASE) + 16)
#define GPIOB_ODR_OFFSET    ((GPIOB_BASE - PERIPH_BASE) + 20)
#define GPIOC_IDR_OFFSET    ((GPIOC_BASE - PERIPH_BASE) + 16)
#define GPIOC_ODR_OFFSET    ((GPIOC_BASE - PERIPH_BASE) + 20)
#define GPIOD_IDR_OFFSET    ((GPIOD_BASE - PERIPH_BASE) + 16)
#define GPIOD_ODR_OFFSET    ((GPIOD_BASE - PERIPH_BASE) + 20)
#define GPIOE_IDR_OFFSET    ((GPIOE_BASE - PERIPH_BASE) + 16)
#define GPIOE_ODR_OFFSET    ((GPIOE_BASE - PERIPH_BASE) + 20)
#define GPIOF_IDR_OFFSET    ((GPIOF_BASE - PERIPH_BASE) + 16)
#define GPIOF_ODR_OFFSET    ((GPIOF_BASE - PERIPH_BASE) + 20)
#define GPIOG_IDR_OFFSET    ((GPIOG_BASE - PERIPH_BASE) + 16)
#define GPIOG_ODR_OFFSET    ((GPIOG_BASE - PERIPH_BASE) + 20)
void IWDG_Configuration(void);
void Init_All_Periph(void);
void System_Configuration(void);
#endif//__DRV_SYSTEM_H__