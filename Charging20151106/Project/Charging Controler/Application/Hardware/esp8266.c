/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include<string.h>
#include "drv_usart.h"
#include "global_value.h"
#include "self_buffer.h"
#include "user_lib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "esp8266.h"
#include "protocol.h"

/* Synchronisation */
xSemaphoreHandle UsartTimeoutSem;

/**
* @brief  查找字符串
* @param  char *s, char *t ;在s中查找t
* @retval s_temp(t在s中的位置)成功     0 （失败 ）
*/
char *mystrstr(char *s, char *t)
{
  char    *s_temp;        /*the s_temp point to the s*/
  char    *m_temp;        /*the mv_tmp used to move in the loop*/
  char    *t_temp;        /*point to the pattern string*/

  if (NULL == s || NULL == t) return NULL;

  /*s_temp point to the s string*/
  for (s_temp = s; *s_temp != '\0'; s_temp++)
  {
    /*the move_tmp used for pattern loop*/
    m_temp = s_temp;
    /*the pattern string loop from head every time*/
    for (t_temp = t; *t_temp == *m_temp; t_temp++, m_temp++);
    /*if at the tail of the pattern string return s_tmp*/
    if (*t_temp == '\0') return s_temp;

  }
  return NULL;
}

s8 Esp8266PowerOn( void )
{
  return 0;
}

/**
* @brief  ESP8266重启
* @param  None
* @retval 1--回复是对的；0--回复是错误的
 * @note
*/
s8 Esp8266Reset( void )
{
  return 0;
}




/**
* @brief  判断接收数据是否为想要的回复
* @param  None
* @retval 1--回复是对的；0--回复是错误的
* @note   根据Sim5218WaitResponse()中第二个形参决定回复内容，如果回复内容正确，那么释放信号量Sim5218CallBack到
Sim5218WaitResponse()中，并返回1；否则返回0
*/
s8 Esp8266Init( void )
{

  /*连接远程服务器*/
  g_server_ip[0] = 192;
  g_server_ip[1] = 168;
  g_server_ip[2] = 0;
  g_server_ip[3] = 105;

  g_server_port = 6800;

  return 0;
}



/**
* @brief  解析串口接收的主动任务
* @param  串口接收字符串
* @retval None
* @note   串口接收字符串为GPS信息，则将GPS任务置1；若为短信信息，则将SMS任务置1。
可能后续还要添加CREG:0，2的提醒任务等
*/
void Esp8266PromptProtocol(char *Buffer,uint8 len)
{
   FrameExtraction(Buffer,len,0);
}


/**
* @brief  串口解析任务；
* @param  None
* @retval None
* @note   优先级最高。创建串口接收断帧信号量；回调函数信号量。首先等待串口接收完一帧(串口收到相邻两字符间隔3ms以上算一帧)所释放的信号量。
解析收到的数据，看是否是模块任务中发送给串口的指令回复：若是回复，则解析后利用回调函数转回模块任务；若为模块接收到的(如短信，gps)，那么
解析数据看是什么数据，那么释放sche信号量，并且为flag赋值。转到模块任务去执行。
*/
void WifiParseTask(void * pvParameters)
{
  char auch_buf[MAX_REV_LEN];
  
  Usart4RevBufInit();

  InitQueue();

  Usart4Init();

  vSemaphoreCreateBinary(UsartTimeoutSem);

  for( ;; )
  {
    if(xSemaphoreTake(UsartTimeoutSem, portMAX_DELAY) == pdTRUE)
    {
      memset(auch_buf, 0, MAX_REV_LEN);
      
      memcpy(auch_buf, g_rev_usart1.p_parse, g_rev_usart1.uch_parse_len);

      g_rev_usart1.b_parse_busy = false;
      
      if(mystrstr(auch_buf,"StationMac") != NULL)
      {
           printf("%s\r\n",auch_buf);
      }
      else if(mystrstr(auch_buf,"TCP close") != NULL)
      {
           printf("TCP close\r\n");
      }
      else if(mystrstr(auch_buf,"TCP success") != NULL)
      {
           printf("TCP success\r\n");
      }
      else if(mystrstr(auch_buf,"WIFI success") != NULL)
      {
           printf("WIFI success\r\n");
      }
      else if(mystrstr(auch_buf,"WIFI close") != NULL)
      {
           printf("WIFI close\r\n");
      }
      
      Esp8266PromptProtocol(auch_buf,g_rev_usart1.uch_parse_len);
    }
  }
}

