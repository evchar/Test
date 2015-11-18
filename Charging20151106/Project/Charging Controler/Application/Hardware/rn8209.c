#include "rn8209.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "global_value.h"

#define RN8209_CS     PAout(4)
#define RN8209_MISO   PAin(6)
#define RN8209_MOSI   PAout(7)
#define RN8209_CLK    PAout(5)

void PF_Init(void);

void DelayNus(u32 nus)
{
  u16 i=0;
  while(nus--)
  {
    i = 9;
    while(i--) ;
  }
}

//以下是SPI模块的初始化代码，配置成主机模式，访问RN8209
void RN8209_SPI_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );//PORTA时钟使能

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;//MISO
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4| GPIO_Pin_5 | GPIO_Pin_7;  // CKL MOSI CS
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA

  RN8209_CS = 1;        //SPI 不选中
  RN8209_CLK = 0;
  RN8209_MOSI = 1;

}

/***********************************************************************
* 函数名称: void RN8209_SPI_Write(u8 bdata)
* 函数功能: 写入1个字节数据到8209中
* 输入参数: bdata-数据
* 输出参数: 无
***********************************************************************/
void RN8209_SPI_Write(u8 bdata)
{
  u8 i;
  u8 byte;
  //portENTER_CRITICAL();
  for(i=0;i<8;i++)
  {
    byte = bdata & 0x80;
    if (byte==0)
    {
      RN8209_MOSI = 0;
    }
    else
    {
      RN8209_MOSI = 1;
    }
    RN8209_CLK = 1;
    DelayNus(Time_CLK);//时钟频率
    RN8209_CLK = 0;
    DelayNus(Time_CLK);
    bdata <<= 1;
  }
  RN8209_MOSI = 1;
  DelayNus(Time_T1);//T1
  //portEXIT_CRITICAL();
  return;
}
/***********************************************************************
*函数名称: u8 RN8209_SPI_Read(void)
*函数功能: 读取8209一字节数据
*输入参数: 无
*输出参数: 读取的1字节数据
***********************************************************************/
u8 RN8209_SPI_Read(void)
{
  u8 i,rbyte=0;
  //portENTER_CRITICAL();
  for(i=0;i<8;i++)
  {
    RN8209_CLK = 1;
    DelayNus(Time_CLK);
    rbyte <<= 1;
    RN8209_CLK = 0;         //下降沿读取数据
    if (RN8209_MISO == 1)
    {
      rbyte |= 1;
    }
    DelayNus(Time_CLK);
  }
  RN8209_CLK = 0;
  DelayNus(Time_T2);//T2
  //portEXIT_CRITICAL();
  return(rbyte);
}

/***********************************************************************
* 函数名称: u8 RN8209_ReadData(u8 add,u8 *data,u8 *len)
* 函数功能: 读取8209寄存器数据
* 输入参数: add-8209寄存器地址 *data-读取后放置的缓冲区 *len-数据长度
* 输出参数: 0-读取失败 1-读取成功
***********************************************************************/
u8 RN8209_ReadData(u8 add,u8 *data,u8 *len)
{
  u8 ret=1;

  RN8209_CS = 0;
  DelayNus(Time_CS);//CS拉低后一段时间，给CLk
  RN8209_SPI_Write(add);

  switch(add)
  {
  case 0x00:
  case 0x01:
  case 0x02:
  case 0x03:
  case 0x04:
  case 0x05:
  case 0x06:
  case 0x09:
  case 0x0A:
  case 0x0B:
  case 0x0C:
  case 0x0D:
  case 0x0E:
  case 0x0F:
  case 0x10:
  case 0x20:
  case 0x21:
  case 0x25:
  case 0x45:
    *data = RN8209_SPI_Read();
    *(data+1) = RN8209_SPI_Read();
    *len = 2;
    break;
  case 0x07:
  case 0x08:
  case 0x40:
  case 0x41:
  case 0x42:
  case 0x43:
    *data =RN8209_SPI_Read();
    *len = 1;
    break;
  case 0x22:
  case 0x23:
  case 0x24:
  case 0x29:
  case 0x2A:
  case 0x2B:
  case 0x2C:
  case 0x2D:
  case 0x7f :
    *data = RN8209_SPI_Read();
    *(data+1) = RN8209_SPI_Read();
    *(data+2) = RN8209_SPI_Read();
    *len = 3;
    break;
  case 0x26 :
  case 0x27 :
  case 0x28 :
  case 0x44 :
    *data = RN8209_SPI_Read();
    *(data+1) = RN8209_SPI_Read();
    *(data+2) = RN8209_SPI_Read();
    *(data+3) = RN8209_SPI_Read();
    *len = 4;
    break;
  default :
    ret = 0;
    break;
  }

  RN8209_CS = 1;
  return(ret);
}

/***********************************************************************
* 函数名称: void RN8209_WriteData(u8 *ptr)
* 函数功能: 写入数据到8209寄存器中
* 输入参数: *ptr-指向要写入的数据缓冲区
* 输出参数: 无
***********************************************************************/
void RN8209_WriteData(u8 *ptr)
{
  u8 temp[2];

  if ((*ptr) > 0x10)      /* 控制寄存器地址不大于0x10 */
  {
    return;
  }

  RN8209_CS = 0;
  DelayNus(Time_CS);//CS拉低后一段时间，给CLk

  temp[0] = 0xea;        /* 写使能命令*/
  temp[1] = 0xe5;
  RN8209_SPI_Write(temp[0]);
  RN8209_SPI_Write(temp[1]);

  RN8209_SPI_Write((*ptr)+0x80);   /* 写入数据 */
  if ( (*ptr != 0x07) && (*ptr != 0x08) )
  {
    RN8209_SPI_Write(*(ptr+1));
  }
  RN8209_SPI_Write(*(ptr+2));

  temp[0] = 0xea;       /* 写保护命令 */
  temp[1] = 0xdc;
  RN8209_SPI_Write(temp[0]);
  RN8209_SPI_Write(temp[1]);

  DelayNus(Time_CS);
  RN8209_CS = 1;

  return;
}


/***********************************************************************
* 函数名称: RN8209_WriteCmdReg(u8 cmd)
* 函数功能: 写入数据到8209寄存器中
* 输入参数: *ptr-指向要写入的数据缓冲区
* 输出参数: 无
***********************************************************************/
void RN8209_WriteCmdReg(u8 cmd)
{
  RN8209_SPI_Write(RN8209_EA);
  RN8209_SPI_Write(cmd);
}

/***********************************************************************
* 函数名称: void RN8209_DisablePulse(void)
* 函数功能: 关闭8029脉冲输出
* 输入参数: *ptr-指向要写入的数据缓冲区
* 输出参数: 无
***********************************************************************/
void  RN8209_DisablePulse(void)
{
   u8 temp[3];

   temp[0] = 0x01;
   temp[1] = 0x00;
   temp[2] = 0x00;
   RN8209_WriteData(&temp[0]);

   return;
}

/***********************************************************************
* 函数名称: void RN8209_EnablePulse(void)
* 函数功能: 关闭8029脉冲输出
* 输入参数: *ptr-指向要写入的数据缓冲区
* 输出参数: 无
***********************************************************************/

void  RN8209_EnablePulse(void)
{
   u8 temp[3];

   temp[0] = 0x01;
   temp[1] = 0x00;
   temp[2] = 0x01;
   RN8209_WriteData(&temp[0]);

   return;
}

/***********************************************************************
* 函数名称: void RN8209_CalJBCS(void)
* 函数功能: 计算校表参数并写入到8209寄存器和EEPROM中
* 输入参数: regadr-8209寄存器地址 *ptr-指向要写入的数据缓冲区
* 输出参数: 无
***********************************************************************/
void  RN8209_CalJBCS(u8 regadr,u8 *ptr)
{

  u8 tmpbyte,tmpadr;
  u8 tmpbuf[5];
  u16 tmpcal,tmpu16;
  u32 tmp8209P,tmprealP;
  float tmpfloat;

  switch (regadr)
  {
  case 0x02:     /*HFConst设置*/
    tmpbuf[1] = *(ptr+1);
    tmpbuf[2] = *(ptr);
    break;
  case 0x03:     /*启动功率值设置*/
    /*台子打到0.2%Ib,读出有功功率,取中间两字位传到有功启动寄存器03中*/
    RN8209_ReadData(RN8209_REG_PA,&tmpbuf[0],&tmpbyte);
    break;
  case 0x05:     /*有功功率1.0校正*/
  case 0x07:    /*有功功率0.5L校正*/
    //tmp8209P = ((u32)(BN8209_CS.P[2]) << 16) + ((u32)(BN8209_CS.P[1]) << 8) + BN8209_CS.P[0];
    //tmp8209P &= 0x7fffff;    /*//////屏蔽功率符号位/////*/
    //tmprealP = ((u32)(*(ptr+2)) << 16) + ((u32)(*(ptr+1)) << 8) + (*(ptr));
    //tmp8209P = BCDLongTOHexlong(tmp8209P); /*RN8209功率值*/
    //tmprealP = BCDLongTOHexlong(tmprealP); /*校表台功率值*/

    if(regadr == 0x05)
    {

      if(tmprealP > tmp8209P)    /*误差偏负*/
      {

        tmpfloat = ((float)(tmprealP - tmp8209P)) / ((float)(tmprealP));
        tmpfloat = tmpfloat/(1.0000-tmpfloat);
        tmpu16 = (u16)(32768*tmpfloat);

      }
      else                       /*误差偏正*/
      {

        tmpfloat = ((float)(tmp8209P - tmprealP)) / ((float)(tmprealP));
        tmpfloat = tmpfloat/(1.0000+tmpfloat);
        tmpu16 = (u16)(32768*(2.0000-tmpfloat));

      }

    }
    else if(regadr == 0x07)
    {

      if(tmprealP > tmp8209P)    /*误差偏负*/
      {

        tmpfloat = ((float)(tmprealP - tmp8209P)) / ((float)(tmprealP));
        tmpu16 = (u16)(1654*tmpfloat);

      }
      else
      {

        tmpfloat = ((float)(tmp8209P - tmprealP)) / ((float)(tmprealP));
        tmpu16 = (u16)(256-(u8)(1654*tmpfloat));
      }

    }

    tmpbuf[1] = (u8)(tmpu16 >> 8);
    tmpbuf[2] = (u8)(tmpu16);
    break;
  case 0x12:   /*电流有效值校正*/
  case 0x13:   /*零线电流有效值校正*/
  case 0x14:   /*电压有效值校正*/
    if((regadr == 0x12) || (regadr == 0x13))
    {

      if(regadr == 0x12)
      {

        tmpadr = RN8209_REG_IA;

      }
      else
      {

        tmpadr = RN8209_REG_IB;

      }
      tmprealP = ((u32)(*(ptr+2)) << 16) + ((u32)(*(ptr+1)) << 8) + (*(ptr));
      //tmprealP = BCDLongTOHexlong(tmprealP); /*RN8209功率值*/
      tmpcal = 1000;

    }
    else
    {

      tmpadr = RN8209_REG_U;
      //tmprealP = (u32)(BcdTOHexInt(((u16)(*(ptr+1)) << 8) + (*(ptr))));
      tmpcal = 10;

    }

    RN8209_ReadData(tmpadr,&tmpbuf[0],&tmpbyte);  /*读取寄存器的值*/
    tmp8209P = ((u32)(tmpbuf[0]) << 16) + ((u32)(tmpbuf[1]) << 8) + tmpbuf[2];
    tmprealP = (u32)((tmpcal*tmp8209P)/tmprealP); /*结果等于寄存器值除以实际值*/

    tmpbuf[1] = (u8)(tmprealP >> 16);
    tmpbuf[2] = (u8)(tmprealP >> 8);
    tmpbuf[3] = (u8)(tmprealP);
    break;
  case 0x0A:     /*有功功率offset校正*/
    /*台体电流打到5%Ib,1.0,若此时表误差不好,需设此值,好的话就不用设置*/
    RN8209_ReadData(RN8209_REG_PA,&tmpbuf[0],&tmpbyte); /*读取寄存器的值*/
    tmpu16 = ((u16)(tmpbuf[2]) << 8) + tmpbuf[3];
    tmpu16 = (~tmpu16)+1;
    tmpbuf[1] = (u8)(tmpu16 >> 8);
    tmpbuf[2] = (u8)(tmpu16);
    break;
  case 0x0E:     /*电流有效值offset校正*/
  case 0x0f:     /*零线电流有效值offset校正*/
    if(regadr == 0x0e)
    {

      tmpadr = RN8209_REG_IA;

    }
    else if(regadr == 0x0f)
    {

      tmpadr = RN8209_REG_IB;

    }
    RN8209_ReadData(RN8209_REG_IA,&tmpbuf[0],&tmpbyte); /*读取寄存器的值*/

    tmpu16 = ((u16)(tmpbuf[1]) << 8) + tmpbuf[2];
    tmp8209P = ((u32)(tmpu16))*tmpu16;
    tmp8209P = (~tmp8209P)+1;

    tmpbuf[1] = (u8)(tmp8209P >> 16);
    tmpbuf[2] = (u8)(tmp8209P >> 8);
    break;
  case 0xff:     /*校表初始化*/
    //RN8209_PInitDefault();
    //RN8209_LoadJBCS();

    return;
    break;
  default:break;

  }

  tmpbuf[0] = regadr;     /*写入校表参数到EEP中和RN8209中*/
  //RN8209_WriteDataToEE(&tmpbuf[0]);
  RN8209_WriteData(&tmpbuf[0]);

  if(regadr == 0x02)      /*计算功率换算系数*/
  {

    tmpu16 = ((u16)(tmpbuf[1]) << 8)+tmpbuf[2];
    //   tmprealP = ((u32)(tmpu16))*1600;
    //tmpfloat = ((float)(tmpu16))*133.3013*(IMPULSE_CONSTANT);
    tmprealP = (u32)(tmpfloat);
    //UlongTOUcharbuf(tmprealP,&tmpbuf[1]);
    tmpbuf[0] = 0x11;
    //RN8209_WriteDataToEE(&tmpbuf[0]);

  }

  //RN8209_CalChkSum();   /* 更新8209校表数据累计校验和 */

  return;

}

void  RN8209_Chip_Init(void)
{
  u32 chipID;
  u8 length;
  u8 temp[3];

  RN8209_SPI_Init();
  
  PF_Init();
  
  /*rn8209c reset*/
  RN8209_WriteCmdReg(RN8209_CMD_RESET);

  /*等待复位完成*/
  DelayNus(500000);

  /*read chip ID*/
  chipID = 0;
  RN8209_ReadData(REG_DEVID,(u8*)&chipID,&length);
  printf("chip id:0x%x\r\n",chipID);

  /*选择A通道*/
  RN8209_WriteCmdReg(RN8209_PATH_A);

  /*电压电流通道增益均为1*/
  temp[0] = REG_SYSCON;
  temp[1] = 0x00;
  temp[2] = 0x00;
  RN8209_WriteData(&temp[0]);

  /*电能寄存器为累加型，使能PF脉冲输出和有功电能寄存器累加*/
  temp[0] = REG_EMUCON;
  temp[1] = 0x00;
  temp[2] = 0x01;
  RN8209_WriteData(&temp[0]);

  /*
  HFCost = INT[(14.8528*Vu*Vi*10^11 ) / (Un*Ib*Ec)]

  HFCost = 14.8528*Vu*Vi*10^11/Un*Ib*Ec  18904

  HFCost = 14.8528*0.224*0.4*10^11 / (1600*220*20) = 18904 = 0x49D8 
  HFCost = 14.8528*0.224*0.2*10^11 / (1600*220*20) = 9452 = 0x24EC*/

  /*write HFConst*/
  temp[0] = REG_HFCONST;
  temp[1] = 0x28;
  temp[2] = 0xDC;
  RN8209_WriteData(&temp[0]);
}

/***********************************************************************
* 函数名称: void RN8209_GET_FREQ(void)
* 函数功能: 获取工作电压频率
* 输入参数: 无
* 输出参数: 频率
***********************************************************************/
u8 RN8209_GET_UFREQ(void)
{
  u8 freq,length;
  u16 freq_reg;
  u32 chipID;
    /*read chip ID*/
  chipID = 0;
  RN8209_ReadData(REG_DEVID,(u8*)&chipID,&length);
  printf("chip id:0x%x\r\n",ntohs(chipID));
  
  RN8209_ReadData(REG_UFREQ,(u8*)&freq_reg,&length);

  freq = (u8)(3579545/8/(ntohs(freq_reg)));
  printf("freq:%d\r\n",freq);

  return freq;
}

/***********************************************************************
* 函数名称: void RN8209_GET_URMS(void)
* 函数功能: 获取工作电压有效值
* 输入参数: 无
* 输出参数: 电压有效值
***********************************************************************/
u32 RN8209_GET_URMS(void)
{
  u8 length;
  int32 urms;
  u8 tem[3];
  RN8209_ReadData(REG_URMS,(u8*)tem,&length);
  
  if(tem[0]&0x80)
  {
    urms = 0;
  }
  else
  {
    urms = (tem[0]<<24) + (tem[1]<<16) + tem[2];
  }
  
  printf("urms:%d\r\n",urms);

  return urms;
}

/***********************************************************************
* 函数名称: void RN8209_GET_IARMS(void)
* 函数功能: 获取工作电流有效值
* 输入参数: 无
* 输出参数: 电压有效值
***********************************************************************/
u32 RN8209_GET_IARMS(void)
{
  u8 length;
  int32 iarms;
  u8 tem[3];
  RN8209_ReadData(REG_IARMS,(u8*)tem,&length);
  
  if(tem[0]&0x80)
  {
    iarms = 0;
  }
  else
  {
    iarms = (tem[0]<<24) + (tem[1]<<16) + tem[2];
  }
  printf("iarms:%d\r\n",iarms);

  return iarms;
}



/***********************************************************************
* 函数名称: void RN8209_GET_ENERGYA(void)
* 函数功能: 获取工作电流有效值
* 输入参数: 无
* 输出参数: 电能,1/Ec kWh
***********************************************************************/
u32 RN8209_GET_ENERGYA(void)
{
  u8 length;
  u32 energy;
  
  u8 tem[3];
  
  RN8209_ReadData(REG_ENERGYPA,(u8*)tem,&length);

  energy = (tem[0]<<24) + (tem[1]<<16) + tem[2];
  printf("energy:%d\r\n",energy);

  return energy;
}


void PF_Init(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);	 

  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //使能复用功能时钟
  
  /* Configure PB5 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Clear the EXTI line 5 pending bit */
  EXTI_ClearITPendingBit(EXTI_Line5);
  
  //GPIOE.2 中断线以及中断初始化配置   下降沿触发
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource5);
  
  EXTI_InitStructure.EXTI_Line = EXTI_Line5;    //KEY2
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);     //根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;            //使能按键WK_UP所在的外部中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;    //抢占优先级2，
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;                   //子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                             //使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);
}

void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line5) != RESET)
  {
    //g_interrupt_cnt++;
    g_pf_cnt++;
      /* Clear the EXTI line 5 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line5);
  }
}











