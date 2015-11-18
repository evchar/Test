#include "stm32f10x.h"

#include <stdio.h>
#include <string.h>
#include "global_value.h"

#include "protocol.h"
#include "user_lib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "self_buffer.h"
#include "esp8266.h"
#include "flash_if.h"

/* Synchronisation */
extern xSemaphoreHandle UsartSendSem;


uint8 GetChecksum(uint8 *in,u8 len)
{
  uint8 checksum = 0;
  for(char i=0; i<len; i++)
  {
    checksum += *(in+i);
  } 
  return checksum;
}

static int8 CmdGetDeviceSn(uint8 uch_op, void *pdata, uint8 uch_length, void *data_area, uint16 sn,uint8 *uch_back_type)
{
  if(GET_SN == uch_op)
  {
    if(uch_length == 0)//get命令数据长度为0
    {
      //printf("ok\r\n");
      *uch_back_type = 0X81;
      //device_id = htonl(g_device_id);
      //memcpy((uint8 *)data_area,(uint8*)&device_id,sizeof(device_id));
      return 0;
    }
  }
  return -1;
}

static int8 CmdSetWorkStatus(uint8 uch_op, void *pdata, uint8 uch_length, void *data_area, uint16 sn,uint8 *uch_back_type)
{
  uint16 status;
  if(SET_WORK_STATUS == uch_op)
  {
    if(uch_length == 2)//set命令数据长度为2
    {

      *uch_back_type = RESP_WORK_STATUS;

      if(0x0100 == *(uint16*)pdata)
      {
        printf("server cmd start\r\n");
        g_charging_en = true;
        g_power_switch_state = POWER_ON;
        status = 4;
        memcpy((uint8 *)data_area,(uint8*)&status,sizeof(status));
      }
      else if(0x0200 == *(uint16*)pdata)
      {
        printf("server cmd stop\r\n");
        g_charging_en = false;
        g_power_switch_state = POWER_OFF;
        status = 6;
        memcpy((uint8 *)data_area,(uint8*)&status,sizeof(status));
      }
      else
      {
        return -1;
      }
      return -1;//sizeof(status);
    }
  }
  return -1;
}

static int8 CmdSetChargingMod(uint8 uch_op, void *pdata, uint8 uch_length, void *data_area, uint16 sn,uint8 *uch_back_type)
{
    if(uch_op == SET_CHARGING_MODE)
    {
      if(uch_length == 1)
      {
          *uch_back_type = RESP_CHARGING_MODE;
          if(0x1102 == *(uint16*)pdata)/*2代表20%占空比 ，快充*/
          {
            f_changemode = 1;
            TIM2->CCR4 = 400;
            *(uint8*)data_area = HIGHPOWER;
          }
          if(0x1001 == *(uint16*)pdata)/*1代表10%占空比，慢充*/
          {
            f_changemode = 1;
            TIM2->CCR4 = 200;
            *(uint8*)data_area = LOWPOWER;
          }
          return 1;
      }
    }
    return -1;
}

static int8 CmdGetWorkStatus(uint8 uch_op, void *pdata, uint8 uch_length, void *data_area, uint16 sn,uint8 *uch_back_type)
{
  if(GET_WORK_STATUS == uch_op)
  {
    if(uch_length == 0)//set命令数据长度为0
    {
      *uch_back_type = GET_WORK_STATUS_RESP;
      *(uint8*)data_area = ((uint8)g_charging_state | (uint8)g_power_switch_state);
      return 1;
    }
  }
  return -1;
}

static uint32_t flashAddress = SWAP_REGION_START_ADDRESS;

static int8 OTA_Write(uint8 uch_op, void *pdata, uint8 uch_length, void *data_area, uint16 sn,uint8 *uch_back_type)
{
  uint16 seq;
  uint32 state;
  if(OTA_WRITE == uch_op)
  {
    if(uch_length == 102)//get命令数据长度为0
    {
      FLASH_If_Init();
      state = FLASH_If_Write(&flashAddress,(uint32*)((uint8*)pdata+2),25);
      
      if(0 == state)
      {  
        *uch_back_type = OTA_WRITE_ACK;
        
        seq = *(uint16*)pdata + 1;
        memcpy((uint8 *)data_area,(uint8*)&seq,sizeof(seq));
        return 2;
      }
    }
  }
  return -1;
}

static int8 OTA_Start(uint8 uch_op, void *pdata, uint8 uch_length, void *data_area, uint16 sn,uint8 *uch_back_type)
{
  char ret = 0;
  if(OTA_START == uch_op)
  {
    if(uch_length == 0)//get命令数据长度为0
    {
      FLASH_If_Init();
      FLASH_If_Erase(SWAP_REGION_START_ADDRESS,64);
      
      *uch_back_type = OTA_START_ACK;
      
      ret = 1;
      memcpy((uint8 *)data_area,(uint8*)&ret,sizeof(ret));
      return 1;
    }
  }
  return -1;
}

/***********************************************************************************************
指令句柄
***********************************************************************************************/
const InsHander InsTable[] =
{

  {GET_SN,                    CmdGetDeviceSn},
  {SET_CHARGING_MODE,         CmdSetChargingMod},
  {SET_WORK_STATUS,           CmdSetWorkStatus},
  {GET_WORK_STATUS,           CmdGetWorkStatus},
  {OTA_START,                 OTA_Start},
  {OTA_WRITE,                 OTA_Write}
  
  //{MGMNTIF_ANCHOR_HIGH,             CmdAnchorHigh},//anchor高度
};



uint16 MakeBackFrame(uint8 uin_type,uint8 uch_len,void* out,uint8 uch_link_id)
{
  char buffer[150];
  char i,j;
  uint8 uch_checksum = 0;
  ptl_frame_head_t* h = (ptl_frame_head_t*)out;
  
  h->uch_type = uin_type;
  h->u32_device_sn = htonl(g_device_id);
  h->uch_Lengh = uch_len;

  buffer[0] = 0x45;
  buffer[1] = 0x56;
  
  uch_checksum = GetChecksum((uint8*)(out),sizeof(ptl_frame_head_t)+uch_len);

  *((uint8*)out+sizeof(ptl_frame_head_t)+uch_len) =  uch_checksum;
  
  for(i=0,j=0; i<sizeof(ptl_frame_head_t) + uch_len + 1; i++)
  {
    if(*((uint8*)out+i) == 0x45)
    {
      buffer[2+i+j] = *((uint8*)out+i);
      buffer[2+i+j+1] = 0x45;
      j++;/*多插入转义字符的个数*/
    } 
    else
    {
      buffer[2+i+j] = *((uint8*)out+i);
    } 
  }
  /*可能需要先获取校验值再转义*/
  //buffer[2+sizeof(ptl_frame_head_t)+uch_len+j] = uch_checksum;//GetChecksum((uint8*)(&buffer[2]),sizeof(ptl_frame_head_t)+uch_len+j);
  
  buffer[2+sizeof(ptl_frame_head_t)+uch_len+j+1] = 0x45;/*帧倒数第二个数*/
  buffer[2+sizeof(ptl_frame_head_t)+uch_len+j+2] = 0x43;
  
  if(QueueInsert(buffer,sizeof(ptl_frame_head_t) + uch_len + j + 5,uch_link_id))
  {
    xSemaphoreGive(UsartSendSem);
  }
  
  return (sizeof(ptl_frame_head_t) + uch_len + 5 + j);
}


uint16 ProtocolAnalysis(void *in,uint16 len,void *out,uint8 uch_link_id)
{
  unsigned char i;
  int8 back;
  uint8 uch_back_type;

  ptl_frame_head_t *h = (ptl_frame_head_t *)in;

  if(h->u32_device_sn != ntohl(g_device_id) && h->u32_device_sn != 0xffffffff)/*本机序号或者广播地址*/
  {
    return 0;
  }
  
  if(GetChecksum(in,(len-1)) != *((uint8*)(h+1)+h->uch_Lengh))
  {
    //return 0;
  }  
   
  printf("%x\r\n",h->uch_type);
  for (i = 0; i < sizeof(InsTable) / sizeof(InsHander); i++)
  {
    if ((h->uch_type) == InsTable[i].funcCode)
    {
      if (InsTable[i].h != NULL)//调用指令句柄
      {
        back = (*InsTable[i].h)(h->uch_type, h + 1, h->uch_Lengh,(uint8 *)out + sizeof(ptl_frame_head_t), 0,&uch_back_type);
        if (back < 0)
          return 0;
        return MakeBackFrame(uch_back_type,back,out,uch_link_id);
      }
      return 0;
    }
  }
  return 0;
}

uint16 FrameExtraction(char *in,uint16 len,uint8 uch_link_id)
{
  uint8_t pos;
  uint8_t buf[160];
  uint8_t uch_index = 0;
  
  bool b_head_flag = false;
  bool b_char_flag = false;
  
  if(len < 10)return -1;/*长度不足*/
  
  for(pos=0; pos<len; pos++)
  {
    if(true == b_char_flag)
    {
      b_char_flag = false;
      
      if(0x56 == *(in+pos))/*收到帧头*/
      {
        uch_index = 0;
        b_head_flag = true;
      }
      else if(0x43 == *(in+pos))/*收到帧尾*/
      {
        b_head_flag = false;
        ProtocolAnalysis(buf,uch_index,buf,uch_link_id);
      }
      else if(0x45 == *(in+pos))
      {
        if(true == b_head_flag)
        {
          buf[uch_index++] = *(in+pos);
        }
      }
      else /*帧编码出错*/
      {
        return -1;
      }   
    }
    else if(*(in+pos) == 0x45)
    {
      b_char_flag = true;
    } 
    else
    {
      if(true == b_head_flag)
      {
        buf[uch_index++] = *(in+pos);
      } 
    }  
  }
  
  return 0;
}

void SendChargingstatus( void )
{
  char buffer[30];
  static uint8 last_state;
  
#ifdef  CONFIG_UESR_BLINK 
  static uint16 time_cnt = 0;
#endif  
  
  uint8 state;
  taskENTER_CRITICAL();
  state = ((uint8)g_charging_state | (uint8)g_power_switch_state);

  //printf("state:%x,%x\r\n",state,last_state);
#ifdef  CONFIG_UESR_BLINK   
  if(++time_cnt > 500)
  {
    time_cnt = 0;
    buffer[sizeof(ptl_frame_head_t)] = state;
    MakeBackFrame(GET_WORK_STATUS_RESP,1,buffer,0);
  } 
#endif    
  
  if(state != last_state)
  {
    last_state = state;
    
    //printf("state:%x\r\n",state);
    
    buffer[sizeof(ptl_frame_head_t)] = state;
    
    MakeBackFrame(GET_WORK_STATUS_RESP,1,buffer,0);
  }
  
  taskEXIT_CRITICAL();
     
}


