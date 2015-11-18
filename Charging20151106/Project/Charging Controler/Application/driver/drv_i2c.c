/* IOI2C.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-04-25
测试： 本程序已在第七实验室的mini IMU上完成测试
功能：
提供I2C接口操作API 。
使用IO模拟方式
------------------------------------ */

#include "api.h"

#define deay_nop_5()    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop")


#define SCL_PIN  6
#define SDA_PIN  7

#define SCL_PIN_Bit            *(uint32_t *)(PERIPH_BB_BASE + GPIOB_ODR_OFFSET * 32 + SCL_PIN * 4)
#define SDA_PIN_Bit            *(uint32_t *)(PERIPH_BB_BASE + GPIOB_ODR_OFFSET * 32 + SDA_PIN * 4)
#define SDA_PIN_Read()         *(uint32_t *)(PERIPH_BB_BASE + GPIOB_IDR_OFFSET * 32 + SDA_PIN * 4)//GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)//

#define SCL_H          SCL_PIN_Bit = 1 //GPIO_SetBits(GPIOB, GPIO_Pin_8)///* GPIO_SetBits(GPIOB , GPIO_Pin_10)   */
#define SCL_L          SCL_PIN_Bit = 0//GPIO_ResetBits(GPIOB, GPIO_Pin_8)// /* GPIO_ResetBits(GPIOB , GPIO_Pin_10) */

#define SDA_H          SDA_PIN_Bit = 1//GPIO_SetBits(GPIOB, GPIO_Pin_9)// /* GPIO_SetBits(GPIOB , GPIO_Pin_11)   */
#define SDA_L          SDA_PIN_Bit = 0//GPIO_ResetBits(GPIOB, GPIO_Pin_9)// /* GPIO_ResetBits(GPIOB , GPIO_Pin_11) */

#define SCLOutput()  do{GPIOB->MODER &= ~(0x3 << (SCL_PIN * 2));GPIOB->MODER |=  0x01 << (SCL_PIN * 2);}while(0)//SCl推挽输出
#define SDAInput()   do{GPIOB->MODER  &= ~(0x3 << (SDA_PIN * 2));}while(0)//SDA浮空输入
#define SDAOutput()  do{GPIOB->MODER  &= ~(0x3 << (SDA_PIN * 2));GPIOB->MODER |=  0x01 << (SDA_PIN * 2);}while(0)//SDA推挽输出


static void I2C_delay(void)
{
    volatile int i = 7;
    while (i)
        i--;
    //deay_nop_5();
}

/**************************实现函数********************************************
*函数原型:      void IIC_Start(void)
*功　　能:      产生IIC起始信号
*******************************************************************************/
void IIC_Start(void)
{
    //API_DISABLE_ALL_INT();
    SDAOutput(); //sda线输出
    SDA_H;
    SCL_H;
    I2C_delay();
    SDA_L;//START:when CLK is high,DATA change form high to low
    I2C_delay();
    SCL_L;//钳住I2C总线，准备发送或接收数据
}


/**************************实现函数********************************************
*函数原型:      void IIC_Stop(void)
*功　　能:      //产生IIC停止信号
*******************************************************************************/
void IIC_Stop(void)
{
    SDAOutput();//sda线输出
    SCL_L;
    SDA_L;//STOP:when CLK is high DATA change form low to high
    I2C_delay();
    SCL_H;
    SDA_H;//发送I2C总线结束信号
    I2C_delay();
    //API_ENABLE_ALL_INT();
}


/**************************实现函数********************************************
*函数原型:      void IIC_Ack(void)
*功　　能:      产生ACK应答
*******************************************************************************/
void IIC_Ack(void)
{
    SCL_L;
    SDAOutput();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;

}

/**************************实现函数********************************************
*函数原型:      void IIC_NAck(void)
*功　　能:      产生NACK应答
*******************************************************************************/
void IIC_NAck(void)
{
    SCL_L;
    SDAOutput();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
}

/**************************实现函数********************************************
*函数原型:      u8 IIC_Wait_Ack(void)
*功　　能:      等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
*******************************************************************************/
uint8 IIC_Wait_Ack(void)
{
    uint8 ucErrTime=0;
    SDAInput();  //SDA设置为输入
    SDA_H;I2C_delay();
    SCL_H;I2C_delay();
    while(SDA_PIN_Read())
    {
        ucErrTime++;
        if(ucErrTime>50)
        {
            IIC_Stop();
            return 1;
        }
       I2C_delay();
    }
    SCL_L;  //时钟输出0
    return 0;
}

/**************************实现函数********************************************
*函数原型:      void IIC_Send_Byte(u8 txd)
*功　　能:      IIC发送一个字节
*******************************************************************************/
void IIC_Send_Byte(uint8 txd)
{
    uint8_t i = 8;
    SDAOutput();
    SCL_L;
    while (i--) {
        if (txd & 0x80)
            SDA_H;
        else
            SDA_L;
        txd <<= 1;
        I2C_delay();
        SCL_H;
        I2C_delay();
        SCL_L;
        I2C_delay();
    }
}

/**************************实现函数********************************************
*函数原型:      u8 IIC_Read_Byte(unsigned char ack)
*功　　能:      //读1个字节，ack=1时，发送ACK，ack=0，发送nACK
*******************************************************************************/
uint8 IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    SDAInput();
    for(i=0;i<8;i++ )
    {
        SCL_L;
        I2C_delay();
        SCL_H;
        receive<<=1;
        if(SDA_PIN_Read())receive++;
        I2C_delay();
    }
    if (ack)
        IIC_Ack(); //发送ACK
    else
        IIC_NAck();//发送nACK
    return receive;
}

/**************************实现函数********************************************
*函数原型:      void IIC_Init(void)
*功　　能:      初始化I2C对应的接口引脚。
*******************************************************************************/
void IIC_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    //GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**************************实现函数********************************************
*函数原型:      unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*功　　能:      读取指定设备 指定寄存器的一个值
输入    I2C_Addr  目标设备地址
        addr       寄存器地址
返回   读出来的值
*******************************************************************************/
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
    unsigned char res=0;

    IIC_Start();
    IIC_Send_Byte(I2C_Addr);//发送写命令
    res++;
    IIC_Wait_Ack();
    IIC_Send_Byte(addr); res++; //发送地址
    IIC_Wait_Ack();
    //IIC_Stop();
    IIC_Start();
    IIC_Send_Byte(I2C_Addr+1); res++; //进入接收模式
    IIC_Wait_Ack();
    res=IIC_Read_Byte(0);
    IIC_Stop();//产生一个停止条件

    return res;
}


/**************************实现函数********************************************
*函数原型:  u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*功　　能:   取指定设备 指定寄存器的 length个值
输入    dev  目标设备地址
        reg   寄存器地址
        length 要读的字节数
        *data  读出的数据将要存放的指针
返回   读出来的字节数量
********************************************************************************/
uint8 IICreadBytes(uint8 dev, uint8 reg, uint8 length, uint8 *data)
{
    uint8 count = 0;
    IIC_Start();
    IIC_Send_Byte(dev);//发送写命令
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);//发送地址
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(dev+1);//进入接收模式
    IIC_Wait_Ack();
    for(count=0;count<length;count++){
         if(count!=length-1)data[count]=IIC_Read_Byte(1); //带ACK的读数据
            else  data[count]=IIC_Read_Byte(0); //最后一个字节nACK
    }
    IIC_Stop();//产生一个停止条件
    return 0;
}

/**************************实现函数********************************************
*函数原型:  u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*功　　能:   取指定设备 指定寄存器的 length个值
输入    dev  目标设备地址
        reg   寄存器地址
        length 要读的字节数
        *data  读出的数据将要存放的指针
返回   读出来的字节数量
********************************************************************************/
uint8 SMT480TreadBytes(uint8 dev, uint8 command, uint8 reg, uint8 length, uint8 *data)
{
    uint8 count = 0;
    IIC_Start();
    IIC_Send_Byte(dev);//发送写命令
    IIC_Wait_Ack();
    IIC_Send_Byte(command);//发送地址
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);//发送地址
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(dev+1);//进入接收模式
    IIC_Wait_Ack();
    for(count=0;count<length;count++){
         if(count!=length-1)data[count]=IIC_Read_Byte(1); //带ACK的读数据
            else  data[count]=IIC_Read_Byte(0); //最后一个字节nACK
    }
    IIC_Stop();//产生一个停止条件
    return count;
}


/**************************实现函数********************************************
*函数原型:      u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*功　　能:      将多个字节写入指定设备 指定寄存器
输入    dev  目标设备地址
        reg   寄存器地址
        length 要写的字节数
        *data  将要写的数据的首地址
返回   返回是否成功
*******************************************************************************/
uint8 IICwriteBytes(uint8 dev, uint8 reg, uint8 length, uint8* data)
{
    uint8 count = 0;
    IIC_Start();
    IIC_Send_Byte(dev);  //发送写命令
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);  //发送地址
    IIC_Wait_Ack();
    for(count=0;count<length;count++){
        IIC_Send_Byte(data[count]);
        IIC_Wait_Ack();
     }
    IIC_Stop();//产生一个停止条件
    return 0;
}

/**************************实现函数********************************************
*函数原型:      u8 IICreadByte(u8 dev, u8 reg, u8 *data)
*功　　能:      读取指定设备 指定寄存器的一个值
输入    dev  目标设备地址
        reg    寄存器地址
        *data  读出的数据将要存放的地址
返回   1
*******************************************************************************/
uint8 IICreadByte(uint8 dev, uint8 reg, uint8 *data)
{
    *data=I2C_ReadOneByte(dev, reg);
    return 0;
}


/**************************实现函数********************************************
*函数原型:      unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*功　　能:      写入指定设备 指定寄存器一个字节
输入    dev  目标设备地址
        reg    寄存器地址
        data  将要写入的字节
返回   1
*******************************************************************************/
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************实现函数********************************************
*函数原型:      u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:      读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入    dev  目标设备地址
        reg    寄存器地址
        bitStart  目标字节的起始位
        length   位长度
        data    存放改变目标字节位的值
返回   成功 为1
        失败为0
*******************************************************************************/
uint8 IICwriteBits(uint8 dev,uint8 reg,uint8 bitStart,uint8 length,uint8 data)
{
    uint8 b;
    if (IICreadByte(dev, reg, &b) != 0)
    {
        uint8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    }
    else
    {
        return 0;
    }
}

/**************************实现函数********************************************
*函数原型:      u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:      读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入    dev  目标设备地址
        reg    寄存器地址
        bitNum  要修改目标字节的bitNum位
        data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1
        失败为0
*******************************************************************************/
uint8 IICwriteBit(uint8 dev, uint8 reg, uint8 bitNum, uint8 data)
{
    uint8 b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}

//------------------End of File----------------------------


