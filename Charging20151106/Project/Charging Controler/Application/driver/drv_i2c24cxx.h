/***********************************************************************************************
*                      Copyright (C)杭州凯源电子研发部
*文件名：iic24cxx.h
*文件描述：本文件包含IIC存储器芯片的大部分型号的驱动程序头文件，移植者只要修改几个关键定义的宏
*   就可以使用驱动。
*   IIC采用模拟I2C时序，如需要使用硬件IIC时序，则不适用本驱动程序，
*   本驱动程序包含两个文件，iic24cxx.h和iic24cxx.c
*驱动型号：FM24_64          2011年3月11日
*          24_04            2011年3月11日
*          24_08            2011年3月11日
*          24_256           2011年3月11日
*          24_01            2011年12月30日
*          24_02            2011年12月30日
*          24_08            2011年12月30日
*          24_16            2011年12月30日
*          24_64            2011年12月30日
*          24_128           2011年12月30日
*          24_512           2011年12月30日
*类型定义：typedef unsigned char u8
*          typedef unsigned short u16
*本驱动如果有不当之处，请联系czhf201@163.com
*作者：蔡赵烽
*创建时间：2011年3月09日
*当前版本：V1.0
*修改记录
*----------------------------------------------------------------------------------------------
*
***********************************************************************************************/
#ifndef __IIC24CXX_H
#define __IIC24CXX_H
/*在这里定义具体芯片型号*/
//#define IIC_24_01
//#define IIC_24_62
//#define IIC_24_04
//#define IIC_24_08
//#define IIC_24_64
//#define IIC_24_128
//#define IIC_24_256
//#define IIC_FM24_64
#define IIC_24_32
//typedef unsigned char u8
//typedef signed char         s8
//typedef unsigned short u16
//typedef signed short  s16

/*在这里定义硬件电路上是否配置了写保护功能*/
//#define IIC24_WP

/*在这里定义IIC芯片的SCL、SDA，WP等管脚，如果相关操作比较复杂，如需要使用局部变量保存部分参数，
则以下部分或者全部函数可以在iic24xx.c文件中以函数形式定义*/
#define Iic24SCLPin          *(uint32_t *)(PERIPH_BB_BASE + GPIOB_ODR_OFFSET * 32 + 8 * 4)//PB6
#define Iic24SDAPin          *(uint32_t *)(PERIPH_BB_BASE + GPIOB_ODR_OFFSET * 32 + 9 * 4)//PB7
#define Iic24ReadSDA()       *(uint32_t *)(PERIPH_BB_BASE + GPIOB_IDR_OFFSET * 32 + 9 * 4)//PB7读
#define Iic24SCLInput()      // GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)
#define Iic24SCLOutput()    do{GPIOB->MODER &= ~(0x3 << (8 * 2));GPIOB->MODER |=  0x01 << (8 * 2);}while(0)//SCl推挽输出
#define Iic24SDAInput()     do{GPIOB->MODER  &= ~(0x3 << (9 * 2));}while(0)//SDA浮空输入
#define Iic24SDAOutput()    do{GPIOB->MODER  &= ~(0x3 << (9 * 2));GPIOB->MODER |=  0x01 << (9 * 2);}while(0)//SDA推挽输出

#define IicSCLInput()       Iic24SCLInput()
#define IicSCLOutput()      Iic24SCLOutput()
#define IicSDAInput()       Iic24SDAInput()
#define IicSDAOutput()      Iic24SDAOutput()
#define IicResetSCLPin()    Iic24SCLPin = 0
#define IicSetSCLPin()      Iic24SCLPin = 1
#define IicResetSDAPin()    Iic24SDAPin = 0
#define IicSetSDAPin()      Iic24SDAPin = 1
#define IicReadSDA()        Iic24ReadSDA()

/***********************************************************************************************
IIC额外的初始化，本宏用于对IIC总线进行事先初始化，该宏对应指令、函数或其它在IIC总线所有初始化之前
调用。
最典型的应用是用在ARM系列CPU中先打开SCL、SDA时钟、事先定义端口属性等。
***********************************************************************************************/
#ifndef IIC_EXTRA_INIT
#define IIC_EXTRA_INIT
#endif

/*关闭全局中断，在运行多任务操作系统时需要进入临界段代码来操作底层寄存器，在单任务系统中该宏可以不定义*/
#define IicCloseIsr()       

#ifdef IIC24_WP
#define IicWPPin
#define IicWPOpen() 
#define IicWPClose() 
#define IicWPInput()
#define IicWPOutput()
#endif

/*延时周期，用户根据需要和CPU频率修改*/
#define IicNop() {u8 __coun = 2; do{__ASM("nop");__ASM("nop");}while(__coun--);}

//#define IicNop() do{volatile int i = 20; while (i)i--;}while(0)

/*api*/
#define EEPROMInit()        Iic24Init()
#define WriteEEPROM(a,b,c)  Iic24WriteBytes(3,a,b,c)
#define ReadEEPROM(a,b,c)   Iic24ReadBytes(3,a,b,c)

void WriteCharToEEPROM(uint16 uin_Addr, uint8 uch_Data);
void WriteIntToEEPROM(uint16 uin_Addr, uint16 uin_Data);
void WriteLongToEEPROM(uint16 uin_Addr, uint32 ul_Data);
void WriteFloatToEEPROM(uint16 uin_Addr, fp32 f_Data);
uint8 ReadCharFromEEPROM(uint16 uin_Addr);
uint16 ReadIntFromEEPROM(uint16 uin_Addr);
uint32 ReadLongFromEEPROM(uint16 uin_Addr);
fp32 ReadFloatFromEEPROM(uint16 uin_Addr);

#ifdef IIC_24_01
#define BLOCK_ADDR_BIT_LENGTH 0     //块地址位数，必须小于等于3
#define ADDR_BYTE_LENGTH    1       //存储器地址字节长度
#define IIC_PAGE_SIZE 8             //页容量大小

#elif defined IIC_24_02
#define BLOCK_ADDR_BIT_LENGTH 0     //块地址位数，必须小于等于3
#define ADDR_BYTE_LENGTH    1       //存储器地址字节长度
#define IIC_PAGE_SIZE 8             //页容量大小

#elif defined IIC_24_04
#define BLOCK_ADDR_BIT_LENGTH 1     //块地址位数，必须小于等于3
#define ADDR_BYTE_LENGTH    1       //存储器地址字节长度
#define IIC_PAGE_SIZE 16            //页容量大小

#elif defined IIC_24_08
#define BLOCK_ADDR_BIT_LENGTH 2     //块地址位数，必须小于等于3
#define ADDR_BYTE_LENGTH    1       //存储器地址字节长度
#define IIC_PAGE_SIZE 16            //页容量大小

#elif defined IIC_24_16
#define BLOCK_ADDR_BIT_LENGTH 3     //块地址位数，必须小于等于3
#define ADDR_BYTE_LENGTH    1       //存储器地址字节长度
#define IIC_PAGE_SIZE 16            //页容量大小

#elif defined IIC_24_32
#define BLOCK_ADDR_BIT_LENGTH 0     //块地址位数，必须小于等于3
#define ADDR_BYTE_LENGTH    2       //存储器地址字节长度
#define IIC_PAGE_SIZE 32    

#elif defined IIC_24_64
#define BLOCK_ADDR_BIT_LENGTH 0     //块地址位数，必须小于等于3
#define ADDR_BYTE_LENGTH    2       //存储器地址字节长度
#define IIC_PAGE_SIZE 32            //页容量大小

#elif defined IIC_24_128
#define BLOCK_ADDR_BIT_LENGTH 0     //块地址位数，必须小于等于3
#define ADDR_BYTE_LENGTH    2       //存储器地址字节长度
#define IIC_PAGE_SIZE 64            //页容量大小

#elif defined IIC_24_256
#define BLOCK_ADDR_BIT_LENGTH 0     //块地址位数，必须小于等于3
#define ADDR_BYTE_LENGTH    2       //存储器地址字节长度
#define IIC_PAGE_SIZE 64            //页容量大小

#elif defined IIC_24_256
#define BLOCK_ADDR_BIT_LENGTH 0     //块地址位数，必须小于等于3
#define ADDR_BYTE_LENGTH    2       //存储器地址字节长度
#define IIC_PAGE_SIZE 64            //页容量大小

#elif defined IIC_FM24_64
#define BLOCK_ADDR_BIT_LENGTH 0     //块地址位数，必须小于等于3
#define ADDR_BYTE_LENGTH    2       //存储器地址字节长度
#define IIC_PAGE_SIZE 0             //页容量大小

#endif

/*IIC总线初始化*/
extern void Iic24Init(void);
/***********************************************************************************************
*函数名:Iic24WriteBytes
*功能:向Iic24芯片写入多个字节
*入口:chip_addr,芯片地址低三位A2,A1,A0,src 数据源,num,数据个数
*出口:无
***********************************************************************************************/
extern void Iic24WriteBytes(u8 chip_addr,u16 addr,u8 *src,u16 num);
/***********************************************************************************************
*函数名:Iic24ReadBytes
*功能:读取Iic24存储芯片制定地址的若干个字节
*入口:chip_addr,芯片地址低三位A2,A1,A0,dst,读出数据地址,num,数据个数
*出口:无
***********************************************************************************************/
extern void Iic24ReadBytes(u8 chip_addr,u16 addr,u8 *dst,u16 num);

#endif
