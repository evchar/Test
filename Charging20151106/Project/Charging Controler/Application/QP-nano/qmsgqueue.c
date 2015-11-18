#include "api.h"

#define TIMER_INTERVAL  10

typedef struct
{
    uint8 uch_In;//存消息Index;
    uint8 uch_Out;//取消息Index;
    uint8 auch_MsgBuf[MSG_BUF_LEN];//消息队列长度
}stQMsgqueue;
static stQMsgqueue g_QMsgqueue;//消息队列变量

typedef struct
{
    uint8 uch_Switch;//消息处理开关
    uint8 uch_Msg;//消息
    uint16 Timeout;//最大超时
    uint16 Interval;//定时时间
}stGUIMsg;
static stGUIMsg g_GUIMsg[GUI_MSG_NUM];

void InitQMsgqueue(void)
{
    uint8 i;
    
    g_QMsgqueue.uch_In = 0;
    g_QMsgqueue.uch_Out = 0;
    for(i=0; i<MSG_BUF_LEN; i++)
    {
        g_QMsgqueue.auch_MsgBuf[i] = 0;
    }

    for(i=0; i<GUI_MSG_NUM; i++)
    {
        g_GUIMsg[i].uch_Switch = 0;
    }
}

void QPushmsg(uint8 uch_Msg)
{
    if(!uch_Msg)
    {
        return;
    }
    
    if((g_QMsgqueue.uch_In+1)%MSG_BUF_LEN == g_QMsgqueue.uch_Out)//消息队列满
    {
        return;
    }

    g_QMsgqueue.auch_MsgBuf[g_QMsgqueue.uch_In++] = uch_Msg;
    if(MSG_BUF_LEN == g_QMsgqueue.uch_In)
    {
        g_QMsgqueue.uch_In = 0;
    }
}

uint8 QPullmsg(void)
{
    uint8 uch_Msg;
    
    if(g_QMsgqueue.uch_Out == g_QMsgqueue.uch_In)//队列中无消息
    {
        uch_Msg = 0;
    }
    else
    {
        uch_Msg = g_QMsgqueue.auch_MsgBuf[g_QMsgqueue.uch_Out++];
        if(MSG_BUF_LEN == g_QMsgqueue.uch_Out)
        {
            g_QMsgqueue.uch_Out = 0;
        }
    }

    return uch_Msg;
}

//uch_No-定时器编号
//uch_Msg-消息值
//uch_Timeout-首次定时时间
//uch_Interval-之后定时时间 0表示之后无定时时间
void StartMsgTimer(uint8 uch_No, uint8 uch_Msg, uint16 uch_Timeout, uint16 uch_Interval)
{
    if(uch_No >= GUI_MSG_NUM)
    {
        return;
    }

    if(g_GUIMsg[uch_No].uch_Switch == 1)
    {
        return;
    }

    if(uch_Timeout == 0 && uch_Interval == 0)
    {
        return;
    }

    if(uch_Timeout%TIMER_INTERVAL)
        g_GUIMsg[uch_No].Timeout = (uch_Timeout/TIMER_INTERVAL+1)*TIMER_INTERVAL;
    else
        g_GUIMsg[uch_No].Timeout = uch_Timeout;

    if(uch_Interval%TIMER_INTERVAL)
        g_GUIMsg[uch_No].Interval = (uch_Interval/TIMER_INTERVAL+1)*TIMER_INTERVAL;
    else
        g_GUIMsg[uch_No].Interval = uch_Interval;

    if(uch_Timeout == 0 && uch_Interval != 0)
    {
        g_GUIMsg[uch_No].Timeout = g_GUIMsg[uch_No].Interval;    
    }
    
    g_GUIMsg[uch_No].uch_Msg = uch_Msg;
    g_GUIMsg[uch_No].uch_Switch = 1;
}

void StopMsgTimer(uint8 uch_No)
{
    if(uch_No >= GUI_MSG_NUM)
    {
        return;
    }  
    
    g_GUIMsg[uch_No].uch_Switch = 0;
}

void ProcessMsgTimer(void)
{
    for(uint8 i=0; i<GUI_MSG_NUM; i++)
    {
        if(g_GUIMsg[i].uch_Switch == 1)
        {
            if(g_GUIMsg[i].Timeout > 0)
            {
                g_GUIMsg[i].Timeout -= TIMER_INTERVAL;     
            }
            else
            {
                if(g_GUIMsg[i].Interval)
                {
                    g_GUIMsg[i].Timeout = g_GUIMsg[i].Interval;
                }
                else
                {
                    g_GUIMsg[i].uch_Switch = 0;
                }
                QPushmsg(g_GUIMsg[i].uch_Msg);
            }
        }
    }
}

