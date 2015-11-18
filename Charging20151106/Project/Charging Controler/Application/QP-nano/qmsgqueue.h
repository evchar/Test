#ifndef __MSG_QUEUE_H__
#define __MSG_QUEUE_H__

#define MSG_BUF_LEN 4
#define GUI_MSG_NUM 8

enum emMSG_TIMER
{
    MSG_TIMER0,//主界面多页面时，定时切换页面
    MSG_TIMER1,//主界面定时查询温度值
    MSG_TIMER2,
    MSG_TIMER3,
    MSG_TIMER4,
    MSG_TIMER5,//刷新主界面温度电量信息的定时器
    MSG_TIMER6,//无正常无线模块时的提示定时器
    MSG_TIMER7,//提示框消失时间定时器
    MSG_TIMER_END
};

void InitQMsgqueue(void);
void QPushmsg(uint8 uch_Msg);
uint8 QPullmsg(void);
void StartMsgTimer(uint8 uch_No, uint8 uch_Msg, uint16 uch_Timeout, uint16 uch_Interval);
void StopMsgTimer(uint8 uch_No);
void ProcessMsgTimer(void);

#endif//__MSG_QUEUE_H__

