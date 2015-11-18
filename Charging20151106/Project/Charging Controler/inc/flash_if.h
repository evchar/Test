/**
  ******************************************************************************
  * @file    flash_if.h 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    07-October-2011
  * @brief   Header for flash_if.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_IF_H
#define __FLASH_IF_H


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#if defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)
 #define PAGE_SIZE                         (0x400)    /* 1 Kbyte */
 #define FLASH_SIZE                        (0x20000)  /* 128 KBytes */
#elif defined STM32F10X_CL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x40000)  /* 256 KBytes */
#elif defined STM32F10X_HD || defined (STM32F10X_HD_VL)
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x80000)  /* 512 KBytes */
#elif defined STM32F10X_XL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x100000) /* 1 MByte */
#else 
 #error "Please select first the STM32 device to be used (in stm32f10x.h)"    
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define FLASH_START_ADDRESS            (uint32_t)0x08000000
#define APPLICATION_START_ADDRESS      (uint32_t)0x08018000
#define SWAP_REGION_START_ADDRESS      (uint32_t)0x08038000
#define BAK_REGION_START_ADDRESS       (uint32_t)0x08058000
#define SYSINFO_REGION_START_ADDRESS   (uint32_t)0x08078000

#define USER_FLASH_END_ADDRESS         (uint32_t)0x08077FFF

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t FLASH_If_Write(__IO uint32_t* Address, uint32_t* Data, uint16_t DataLength);
uint8_t FLASH_If_Erase(uint32_t start_addr,uint8_t page_num);
void FLASH_If_Init(void);

#endif /* __FLASH_IF_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
