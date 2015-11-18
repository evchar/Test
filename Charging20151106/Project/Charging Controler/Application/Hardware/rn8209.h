#ifndef __RN_8209_H__
#define __RN_8209_H__
#include "stm32f10x.h"

//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//SPI驱动 代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/9
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////

#define Dummy_Byte 0xFF
#define Time_CLK    500
#define Time_T1     1000
#define Time_T2     1000
#define Time_CS     500

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08

//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入


/***********************************************************************/
#define RN8209_REG_IA      0x22
#define RN8209_REG_IB      0x23
#define RN8209_REG_U       0x24
#define RN8209_REG_FREQ    0x25
#define RN8209_REG_PA      0x26
#define RN8209_REG_STS     0x2D
/***********************************************************************/
#define TPOWERONTSTNUM     50
#define TNORMALTSTNUM      5
/***********************************************************************/


#define REG_SYSCON    0x00
#define REG_EMUCON    0x01
#define REG_HFCONST   0x02
#define REG_PSTART    0x03
#define REG_QSTART    0x04
#define REG_GPQA      0x05
#define REG_GPQB      0x06
#define REG_PHSA      0x07
#define REG_PHSB      0x08
#define REG_QPHSCAL   0x09
#define REG_APOSA     0x0A
#define REG_APOSB     0x0B
#define REG_RPOSA     0x0C
#define REG_RPOSB     0x0D
#define REG_IARMSOS   0x0E
#define REG_IBRMSOS   0x0F
#define REG_IBGAIN    0x10

#define REG_PFCNT     0x20
#define REG_QFCNT     0x21
#define REG_IARMS     0x22
#define REG_IBRMS     0x23
#define REG_URMS      0x24
#define REG_UFREQ     0x25
#define REG_POWERPA   0x26
#define REG_POWERPB   0x27
#define REG_POWERQ    0x28
#define REG_ENERGYPA  0x29
#define REG_ENERGYPB  0x2A
#define REG_ENERGYQA  0x2B
#define REG_ENERGYQB  0x2C
#define REG_EMUSTAT   0x2D

#define REG_IE        0x40
#define REG_IF        0x41
#define REG_RIF       0x42
#define REG_SYSSTAT   0x43
#define REG_LASTRDATA 0x44
#define REG_LASTWDATA 0x45

#define REG_DEVID     0x7F

#define RN8209_EA                0xEA
#define RN8209_CMD_RESET         0xFA
#define RN9208_WRITE_EN          0xE5
#define RN8209_WRITE_PROTECT     0xDC
#define RN8209_PATH_A            0x5A
#define RN8209_PATH_B            0xA5


typedef struct{
  u8 syscon[2];
  u8 emucon[2];
  u8 hfconst[2];
  u8 pstart[2];
  u8 qstart[2];
  u8 gpqa[2];
  u8 gpqb[2];
  u8 phsa[2];
  u8 phsb[2];
  u8 qphscal[2];
  u8 aposa[2];
  u8 aposb[2];
  u8 rposa[2];
  u8 rposb[2];
  u8 iarmsos[2];
  u8 ibrmsos[2];
  u8 ibgain[2];
  u8 chksum[2];
  u8 cs;
}TCALI;

typedef struct{
  u8 ugain[2];
  u8 i1gain[2];
  u8 i2gain[2];
  u8 pgain[2];
  u8 p2gain[2];
  u8 cs;
}TGAIN;


void RN8209_SPI_Init(void);      //初始化SPI口
void RN8209_SPI_Write(u8 bdata);
u8 RN8209_SPI_Read(void);
u8 RN8209_ReadData(u8 add,u8 *data,u8 *len);
void  RN8209_Chip_Init(void);

u8 RN8209_GET_UFREQ(void);
u32 RN8209_GET_URMS(void);
u32 RN8209_GET_IARMS(void);
u32 RN8209_GET_ENERGYA(void);

#endif

