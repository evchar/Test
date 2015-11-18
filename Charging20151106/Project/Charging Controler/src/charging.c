/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

#include "stdio.h"

#include "drv_led_relay.h"
#include "drv_keys.h"
#include "drv_timer.h"

#include "user_lib.h"
#include "protocol.h"

#include "global_value.h"
//qepn
#include "qpn_port.h"
#include "qepn.h"
#include "qassert.h"

#include "charging.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
xQueueHandle     msgQueue;
xTimerHandle xTimers[2];

void sw_timers_init(void);
/**
* @brief  Test task
* @param  pvParameters not used
* @retval None
*/

/* protected: */
static QState Tag_Initial(Tag * const me);
static QState Tag_Standby(Tag * const me);
static QState Tag_CarDetected(Tag * const me);
static QState Tag_CarReady(Tag * const me);
static QState Tag_Charging(Tag * const me);
static QState Tag_ChargingOver(Tag * const me);
static QState Tag_Active(Tag * const me);
static QState Tag_Fault(Tag * const me) ;
static QState Tag_BatteryFull(Tag * const me) ;

/* global objects ----------------------------------------------------------*/
Tag Tag_;

/* Active object definition ------------------------------------------------*/
/* @(/1/10) ................................................................*/
void Tag_ctor(Tag *me) {
  QHsm_ctor(&me->super, (QStateHandler)&Tag_Initial);
}

void QHsmTagInit(void)
{
  Tag *a = &Tag_;

  Tag_ctor(a);
#ifndef QF_FSM_ACTIVE
  QHsm_init(&a->super);         /* take the initial transition in HSM */
#else
  QFsm_init(&a->super);         /* take the initial transition in FSM */
#endif
}

extern xQueueHandle    msgQueue;

void QHsmMsgProcess(void)
{
  uint8 uch_Msg = 0;
  Tag *a = &Tag_;
  if (xQueueReceive(msgQueue, &uch_Msg, portMAX_DELAY) == pdTRUE)
  {
    if(uch_Msg == KEY0_UP)
    {
      g_charging_en = true;
      g_power_switch_state = LOCAL_POWER_ON;
      uch_Msg = Q_START_SIG;
    }
    if(uch_Msg == KEY1_UP)
    {
      g_charging_en = false;
      g_power_switch_state = LOCAL_POWER_OFF;
      uch_Msg = Q_STOP_SIG;
    }
    Q_SIG(a) = uch_Msg;
    QHsm_dispatch(&a->super);
  }
}

static QState Tag_Initial(Tag * const me) {
  return Q_TRAN(&Tag_Standby);
}

static QState Tag_Active(Tag * const me) {
  QState status;
  switch (Q_SIG(me)) {

  case Q_ENTRY_SIG:
    {
      printf("active\r\n");
      status = Q_HANDLED();
      break;
    }
  case Q_FAULT_SIG:
    {
      status = Q_TRAN(&Tag_Fault);
      break;
    }
  default:
    {
      status = Q_SUPER(&QHsm_top);
      break;
    }
  }
  return status;
}

static QState Tag_Fault(Tag * const me) {
  QState status;

  switch (Q_SIG(me))
  {
  case Q_ENTRY_SIG:
    {
      printf("fault\r\n");
      status = Q_HANDLED();
      break;
    }

  default:
    {
      status = Q_SUPER(&QHsm_top);
      break;
    }
  }
  return status;
}

static QState Tag_Standby(Tag * const me) {
  QState status;
  uint8 temp;
  switch (Q_SIG(me))
  {
  case Q_ENTRY_SIG:
    {
      START_PWM();
      g_charging_state = DISCONNECTED;
      LED_CHARGE_OFF();
      LED_READY_OFF();
      printf("standby\r\n");
      status = Q_HANDLED();
      break;
    }
  case Q_CAR_DETECTED_SIG:
    {
      status = Q_TRAN(&Tag_CarDetected);
      break;
    }
  case Q_CAR_REDAY_SIG:
    {
      temp = Q_CAR_REDAY_SIG;
      xQueueSend(msgQueue, &temp, 0);
      status = Q_TRAN(&Tag_CarDetected);
      break;
    }
  default:
    {
      status = Q_SUPER(&Tag_Active);
      break;
    }
  }
  return status;
}

static QState Tag_CarDetected(Tag * const me) {
  QState status;
  switch (Q_SIG(me))
  {
  case Q_ENTRY_SIG:
    {
      //START_PWM();
      g_charging_state = CONNECTED;
      printf("car detected\r\n");
      LED_READY_OFF();
      status = Q_HANDLED();
      break;
    }

  case Q_STANDBY_SIG:
    {
      status = Q_TRAN(&Tag_Standby);
      break;
    }

  case Q_CAR_REDAY_SIG:
    {
      status = Q_TRAN(&Tag_CarReady);
      break;
    }

  default:
    {
      status = Q_SUPER(&Tag_Active);
      break;
    }
  }
  return status;
}


static QState Tag_CarReady(Tag * const me) {
  QState status;
  //uint8 tem;
  switch (Q_SIG(me))
  {
  case Q_ENTRY_SIG:
    {
      LED_READY_ON();
      xTimerStart( xTimers[0], 0 );
      printf("car ready\r\n");
      status = Q_HANDLED();
      break;
    }
  case Q_START_SIG:
    {
      status = Q_TRAN(&Tag_Charging);
      break;
    }

  case Q_CHECK_SERVER_SIG:
    {
       LED_READY_TOGGLE();
       if(g_charging_en)
       {
         status = Q_TRAN(&Tag_Charging);
       }
       else
       {
         status = Q_HANDLED();
       }
      break;
    }

  case Q_CAR_DETECTED_SIG:
    {
      status = Q_TRAN(&Tag_CarDetected);
      break;
    }
  case Q_STANDBY_SIG:
    {
      status = Q_TRAN(&Tag_Standby);
      break;
    }
  case Q_EXIT_SIG:
    {
      xTimerStop( xTimers[0], 0 );
      LED_READY_OFF();
      status = Q_HANDLED();
      break;
    }

  default:
    {
      status = Q_SUPER(&Tag_Active);
      break;
    }
  }
  return status;
}

static QState Tag_Charging(Tag * const me) {
  QState status;

  switch (Q_SIG(me))
  {
  case Q_ENTRY_SIG:
    {
      printf("charging\r\n");
      //START_PWM();
      g_timeout_cnt = 0;
      g_charging_state = CHARGING;
      LED_CHARGE_ON();
      xTimerStart( xTimers[1], 0 );
      RELAY_CHARGE_ON();
      //SendChargingstatus();
      status = Q_HANDLED();
      break;
    }

  case Q_CAR_DETECTED_SIG:
    {
      if(g_timeout_cnt >= 24)
      {
        status = Q_TRAN(&Tag_BatteryFull);
      } 
      else
      {
        status = Q_HANDLED();
      }  
      
      break;
    }

  case Q_STANDBY_SIG:
    {
      status = Q_TRAN(&Tag_Standby);
      break;
    }

  case Q_CHECK_SERVER_SIG:
    {
       LED_CHARGE_TOGGLE();
       if(g_charging_en)
       {
         status = Q_HANDLED();
       }
       else
       {
         status = Q_TRAN(&Tag_ChargingOver);
       }
      break;
    }

  case Q_STOP_SIG:
    {
      status = Q_TRAN(&Tag_ChargingOver);
      break;
    }
  case Q_EXIT_SIG:
    {
      g_charging_en = false;
      //xTimerStop( xTimers[1], 0 );
      RELAY_CHARGE_OFF();
      LED_READY_OFF();
      LED_CHARGE_OFF();
      STOP_PWM();//停止和充满状态不再给PWM波
      status = Q_HANDLED();
      break;
    }
  default:
    {  
        status = Q_SUPER(&Tag_Active);
        break;
      
    }
  }
  return status;
}

static QState Tag_ChargingOver(Tag * const me) {
  QState status;
  switch (Q_SIG(me))
  {
  case Q_ENTRY_SIG:
    {
      g_charging_state = CONNECTED;
      LED_READY_ON();
      LED_CHARGE_ON();
      printf("chargingover\r\n");
      //SendChargingOver();
      status = Q_HANDLED();
      break;
    }
  case Q_STANDBY_SIG:
    {
      status = Q_TRAN(&Tag_Standby);
      break;
    }
    
  case Q_CHECK_SERVER_SIG:
    {
       if(g_charging_en)
       {
         xTimerStop( xTimers[1], 0 );
         status = Q_TRAN(&Tag_Standby);
       }
       else
       {
         status = Q_HANDLED();
       }
      break;
    }

  case Q_EXIT_SIG:
    {
      g_last_adc_msg = 0;//防止重新开启后adc状态不发生变化
      xTimerStop( xTimers[1], 0 );
      status = Q_HANDLED();
      break;
    }
    
    /*充电停止后，如果不想重新插拔枪头，还需要继续充电的话则需要重新初始化状态机，cp即可重新通信*/
  default:
    {
      status = Q_SUPER(&Tag_Active);
      break;
    }
  }
  return status;
}

static QState Tag_BatteryFull(Tag * const me) {
  QState status;
  switch (Q_SIG(me))
  {
  case Q_ENTRY_SIG:
    {
      g_charging_state = CHARGING_FULL;
      LED_CHARGE_ON();
      printf("BatteryFull\r\n");
      status = Q_HANDLED();
      break;
    }
  case Q_STANDBY_SIG:
    {
      status = Q_TRAN(&Tag_Standby);
      break;
    }
    
  case Q_CHECK_SERVER_SIG:
    {
       if(g_charging_en)
       {
         xTimerStop( xTimers[1], 0 );
         status = Q_TRAN(&Tag_Standby);
       }
       else
       {
         status = Q_HANDLED();
       }
      break;
    }
    
  case Q_EXIT_SIG:
    {
      LED_READY_OFF();
      g_last_adc_msg = 0;//防止重新开启后adc状态不发生变化
      xTimerStop( xTimers[1], 0 );
      status = Q_HANDLED();
      break;
    }

  default:
    {
      status = Q_SUPER(&Tag_Active);
      break;
    }
  }
  return status;
}

void TestTask(void * pvParameters)
{

  msgQueue = xQueueCreate(5,1);/*消息队列长度为5，数据大小为1*/
  QHsmTagInit();
  sw_timers_init();
  for( ;; )
  {
    QHsmMsgProcess();
  }
}

void vTimerCallback( xTimerHandle pxTimer )
{

  long lArrayIndex;
  uint8 temp = 0;
  // Optionally do something if the pxTimer parameter is NULL.
  configASSERT( pxTimer );

  // Which timer expired?
  lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

  if(0 == lArrayIndex)
  {
    //LED_READY_TOGGLE();
    temp = Q_CHECK_SERVER_SIG;
    xQueueSend(msgQueue, &temp, 0);
  }
  if(1 == lArrayIndex)
  {
    //LED_CHARGE_TOGGLE();
    g_timeout_cnt++;
    if(g_timeout_cnt >= 40)g_timeout_cnt= 41;
    
    temp = Q_CHECK_SERVER_SIG;
    xQueueSend(msgQueue, &temp, 0);
  }
}

void sw_timers_init(void)
{
  //for(u8 i=0; i<2; i++)
  //{
  xTimers[0] = xTimerCreate
    (  /* Just a text name, not used by the RTOS kernel. */
     "Timer",
     /* The timer period in ticks. */
     ( 250 * 1 ),
     /* The timers will auto-reload themselves when they
     expire. */
     pdTRUE,
     /* Assign each timer a unique id equal to its array
     index. */
     ( void * ) 0,
     /* Each timer calls the same callback when it expires. */
     vTimerCallback
       );
  //}

  xTimers[1] = xTimerCreate
    (  /* Just a text name, not used by the RTOS kernel. */
     "Timer",
     /* The timer period in ticks. */
     ( 250 * 2 ),
     /* The timers will auto-reload themselves when they
     expire. */
     pdTRUE,
     /* Assign each timer a unique id equal to its array
     index. */
     ( void * ) 1,
     /* Each timer calls the same callback when it expires. */
     vTimerCallback
       );
#if 0
  if( xTimers != NULL )
  {
    if( xTimerStart( xTimers, 0 ) != pdPASS )
    {
      /* The timer could not be set into the Active state. */
    }
  }
#endif
}
