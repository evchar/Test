#ifndef __GLOBAL_VALUE_H__
#define __GLOBAL_VALUE_H__

#include "user_lib.h"
#define MAX_REV_LEN          200
#define USART_REV_TIMEOUT    5
/*¹¦ÂÊ*/

#define HIGHPOWER  0x10
#define LOWPOWER   0x20

#define CONFIG_UESR_BLINK    0

#pragma pack(1)
typedef struct
{
  u8 auch_rev_buffer[2][MAX_REV_LEN];
  u8 auch_send_buffer[MAX_REV_LEN];
  u8 uch_rev_cnt;
  u8 uch_parse_len;
  u8 uch_timeout;
  bool b_parse_busy;
  u8 *p_rev;
  u8 *p_parse;
}usart_data_rev_t;
#pragma pack()


typedef enum
{
   DISCONNECTED = 0x00,
   CONNECTED,         
   CHARGING,          
   CHARGING_FULL     
}charging_state_t;

typedef enum
{
   POWER_OFF         = 0x00,
   POWER_ON          = 0x10,
   LOCAL_POWER_ON    = 0x20,
   LOCAL_POWER_OFF   = 0x30,
}power_switch_state_t;

typedef s8 (*InsResponseHandler)(char *src,void *pdata);

extern InsResponseHandler Esp8266InsHandler;

extern void *InsPdata;

extern usart_data_rev_t g_rev_usart1;

extern uint32 g_device_id;


extern uint8 g_server_ip[4];

extern uint32 g_server_port;

extern bool g_esp_avilable_flag;

extern bool g_charging_en;

extern uint32 g_pf_cnt;
extern uint32 g_pre_pf_cnt;
extern uint8_t f_batteryfull;

extern uint8_t f_changemode;

extern uint16 g_timeout_cnt;

extern charging_state_t  g_charging_state;


extern power_switch_state_t  g_power_switch_state;
extern uint8 g_last_adc_msg;
uint32 get_device_sn(void);
//extern char g_rec_buffer[128];
//extern char g_rec_cnt;
//extern char g_rec_time;
//typedef enum {CENTER_ANCHOR = 1,NORMAL_ANCHOR} ANCHOR_TYPE;
#endif
