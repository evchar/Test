/**
  @page ewarm EWARM Project Template
  
  @verbatim
  ******************** (C) COPYRIGHT 2011 STMicroelectronics *******************
  * @file    readme.txt
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   This sub directory contains all the user modifiable files needed
  *          to create a new project linked with the STM32F10x Standard Peripheral 
  *          Library and working with IAR Embedded Workbench for ARM (EWARM)
  *          software toolchain (version 5.50 and later).
  ******************************************************************************
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
  * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
  * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
  * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
  * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
  @endverbatim
 
 @par Directory contents

 - project .ewd/.eww/.ewp: A pre-configured project file with the provided library 
                           structure that produces an executable image with IAR 
                           Embedded Workbench.
                
 - stm32f10x_flash.icf : This file is the IAR Linker configuration file used to 
                         place program code (readonly) in internal FLASH and data
                         (readwrite, Stack and Heap)in internal SRAM. 
                         You can customize this file to your need.                    
                      
 - stm32f10x_flash_extsram.icf: This file is the IAR Linker configuration file 
                                used to place program code (readonly) in internal 
                                FLASH and data (readwrite, Stack and Heap)in 
                                external SRAM. You can customize this file to your need.  
                                This file is used only with STM32 High-density devices.

 - stm32f10x_nor.icf:  This file is the IAR Linker configuration file used to 
                       place program code (readonly) in external NOR FLASH and data
                       (readwrite, Stack and Heap)in internal SRAM. 
                       You can customize this file to your need. 
                       This file is used only with STM32 High-density devices.                                           

 - stm32f10x_ram.icf:  This file is the IAR Linker configuration file used to 
                       place program code (readonly) and data (readwrite, Stack 
                       and Heap)in internal SRAM. 
                       You can customize this file to your need.                    
                             
 @par How to use it ?

 - Open the Project.eww workspace.
 - In the workspace toolbar select the project config:
     - STM32100B-EVAL: to configure the project for STM32 Medium-density Value 
       line devices
     @note The needed define symbols for this config are already declared in the
           preprocessor section: USE_STDPERIPH_DRIVER, STM32F10X_MD_VL, USE_STM32100B_EVAL
           
     - STM3210C-EVAL: to configure the project for STM32 Connectivity line devices
     @note The needed define symbols for this config are already declared in the
           preprocessor section: USE_STDPERIPH_DRIVER, STM32F10X_CL, USE_STM3210C_EVAL

     - STM3210B-EVAL: to configure the project for STM32 Medium-density devices
     @note The needed define symbols for this config are already declared in the
           preprocessor section: USE_STDPERIPH_DRIVER, STM32F10X_MD, USE_STM3210B_EVAL

     - STM3210E-EVAL: to configure the project for STM32 High-density devices
     @note The needed define symbols for this config are already declared in the
           preprocessor section: USE_STDPERIPH_DRIVER, STM32F10X_HD, USE_STM3210E_EVAL

     - STM3210E-EVAL_XL: to configure the project for STM32 XL-density devices
     @note The needed define symbols for this config are already declared in the
           preprocessor section: USE_STDPERIPH_DRIVER, STM32F10X_XL, USE_STM3210E_EVAL

     - STM32100E-EVAL: to configure the project for STM32 High-density Value line devices
     @note The needed define symbols for this config are already declared in the
           preprocessor section: USE_STDPERIPH_DRIVER, STM32F10X_HD_VL, USE_STM32100E_EVAL           

 - Rebuild all files: Project->Rebuild all
 - Load project image: Project->Debug
 - Run program: Debug->Go(F5)

@note
 - Low-density Value line devices are STM32F100xx microcontrollers where the 
   Flash memory density ranges between 16 and 32 Kbytes.
 - Low-density devices are STM32F101xx, STM32F102xx and STM32F103xx 
   microcontrollers where the Flash memory density ranges between 16 and 32 Kbytes.
 - Medium-density Value line devices are STM32F100xx microcontrollers where the 
   Flash memory density ranges between 32 and 128 Kbytes.
 - Medium-density devices are STM32F101xx, STM32F102xx and STM32F103xx 
   microcontrollers where the Flash memory density ranges between 32 and 128 Kbytes.
 - High-density Value line devices are STM32F100xx microcontrollers where the 
   Flash memory density ranges between 256 and 512 Kbytes. 
 - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 256 and 512 Kbytes.      
 - XL-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 512 and 1024 Kbytes.
 - Connectivity line devices are STM32F105xx and STM32F107xx microcontrollers.

 * <h3><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h3>
 */


/*add by hujiang*/

/*
20150813:移植freertos成功
20150822:esp8266 TCP多连接测试通过，esp8266自定义协议解析与恢复成功。
20150826:mpu6050 DMP输出正常，esp8266链接超过24小时会掉线。
20150831:ADC正常工作，之前无法采样错误原因：将channel3当做channel13
20150904:加入按键、PWM等
20150909:开发板工程备份

20150910:移植到充电桩主板上。调试串口通，ESP8266串口还未通
20150912:移植到充电桩主板上。调试串口通，ESP8266串口已通,按键测试通过,占空比输出正常
20150913:除漏电保护，电能计量外，蓝牙模块外，其他电路皆测试通过
20150915:初步实现充电过程转换
20150919:下沙充电测试通过
20150920:加入了电池充满状态区别于充电结束状态，还需要进一步加入进入故障状态的条件，加入了RN8209，可以获取芯片ID
20150921:状态机消息派送太多，重复的消息只需要发送一次
20150926:加入服务端控制逻辑
20150929:完善RN8209代码
20150930:车库测试RN8209，屏蔽了部分功能，可以正常读取电能，有效值及电压频率。
20151008:车库测试RN8209，屏蔽了部分功能，可以正常读取电能，有效值及电压频率,PF计数正常。
20151017:重新开启8266 AT通信功能,最后一版带AT功能的程序。

20151025:在第二版硬件上调试，esp8266电源开关接口有变化，去掉esp8266 AT功能,加入串口DMA发送数据,可以正常读取设备SN。
20151026:在第二版硬件上调试，调试OTA相关的协议。
20151028:重新审阅国标文件，在standby状态就开启PWM,只有在充满和停止状态连接+12V。
20151031:在STANDBY打开pwm,在standby时检测到car ready也允许直接跳转
20151102:加入了wifi状态的打印消息，可能有丢失信息现象，加入了获取获取工作状态的指令。
20151103:加入了心跳包定时上传设备状态(宏开关)，允许在电池充满状态或充电结束状态中不重新插拔枪头直接开启，将ADC采样从10ms改为20ms。
 */