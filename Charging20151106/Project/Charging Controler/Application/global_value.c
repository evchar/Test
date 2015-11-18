#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include "global_value.h"


InsResponseHandler Esp8266InsHandler;
/*指令数据处理句柄参数*/
void *InsPdata;

//char g_usart1_rec_buffer[2][MAX_REV_LEN];
//char g_usart1_rec_cnt = 0;
//char g_usart1_rec_time = 5;

usart_data_rev_t g_rev_usart1;

uint32 g_device_id = 0x12345678;//0xffff45ff;

uint8 g_server_ip[4];

uint32 g_server_port;

bool g_esp_avilable_flag = false;

bool g_charging_en = false;

uint32 g_pf_cnt = 0;
uint32 g_pre_pf_cnt = 0;
uint8_t f_batteryfull = 0;//battery full flag

uint8_t f_changemode = 0;/*改变充电模式标志位 强 慢*/

uint16 g_timeout_cnt = 0;

charging_state_t  g_charging_state = DISCONNECTED;


power_switch_state_t  g_power_switch_state = POWER_OFF;

uint8 g_last_adc_msg = 0;

uint32 IntDeviceSerial[3];
/*读取产品ID号码*/
void Get_ChipSerialNum(void)
{
  IntDeviceSerial[0] = *(__IO uint32_t*)(0x1FFFF7E8);
  IntDeviceSerial[1] = *(__IO uint32_t*)(0x1FFFF7EC);
  IntDeviceSerial[2] = *(__IO uint32_t*)(0x1FFFF7F0);
}

/*****************************************************************************/
#define UID_LEN 12
/*****************************************************************************/
static uint32_t JSHash(uint8_t* str,uint16_t len);
/*****************************************************************************/
uint32 get_device_sn(void)
{
    uint8_t uid[UID_LEN];
    uint32_t hash;
    
    Get_ChipSerialNum();
    
    memcpy(uid, &IntDeviceSerial[0], UID_LEN);

    hash = JSHash((uint8_t*)uid, UID_LEN);               //uint8_t ?

    return hash;
}
/*****************************************************************************/
static uint32_t JSHash(uint8_t* str,uint16_t len)
{
   uint32_t hash = 1315423911;
   uint16_t i    = 0;

   for(i = 0; i < len; str++, i++)
   {
      hash ^= ((hash << 5) + (*str) + (hash >> 2));
   }

   return hash;
}
