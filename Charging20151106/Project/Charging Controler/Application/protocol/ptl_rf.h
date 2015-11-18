/*! @file
********************************************************************************
<PRE>
--------------------------------------------------------------------------------
文件名       : plt_rf.h
文件实现功能 : 通讯协议
作者         : hujiang
版本         : V1.0
备注         :
--------------------------------------------------------------------------------
修改记录 :
日 期        版本     修改人              修改内容
2013/4/27  1.0      hujiang              原始版本
--------------------------------------------------------------------------------
</PRE>
*******************************************************************************/

#ifndef __PTL_RF_H__
#define __PTL_RF_H__

/***********************MACRO************************************/

//type
#define RF_DATA  0X55

#define RF_RANGING  0XAA


//opcode
#define RF_GET 0X10

#define RF_SET 0X20

#define RF_SET_ONLY 0X30

#define RF_RESP 0XC0

//ctrlcode
#if 0
#define RF_CMD_TAGSWVER 0x01

#define RF_CMD_TAGHWVER 0x02

#define RF_CMD_TAG_ANCHORLIST 0x03

#define RF_CMD_TAGTXPWR 0x04

#define RF_CMD_TAGBAT 0x05

#define RF_CMD_TAGTMP 0x06

#define RF_CMD_TAGREG 0x07

#define RF_CMD_TAG_INTERVAL 0x08

#define RF_CMD_TAGEXTRA  0x09

#define RF_BC_TAGSLOT  0x0A

#define RF_CMD_RANDACESS  0x0B

#define RF_BC_TAGINFO  0x0C

#define RF_GET_STATIC_TAGINFO 0X0D

#define RF_DYNAMIC_TAGINFO 0X0E

#define RF_SYNC_WORD 0x0F

#define RF_BC_SENSORSLOT 0x10
#endif

typedef enum
{
    RF_CMD_TAGSWVER = 0x01,

    RF_CMD_TAGHWVER,

    RF_CMD_TAG_ANCHORLIST,

    RF_CMD_TAGTXPWR,

    RF_CMD_TAGBAT,

    RF_CMD_TAGTMP,

    RF_CMD_TAGREG,

    RF_CMD_TAG_INTERVAL,

    RF_CMD_TAGEXTRA,

    RF_BC_TAGSLOT,//0x0a

    RF_CMD_RANDACESS,//0x0b

    RF_BC_TAGINFO,//0x0c

    RF_GET_STATIC_TAGINFO,//0x0d

    RF_DYNAMIC_TAGINFO,//0x0e

    RF_SYNC_WORD,

    RF_BC_SENSORSLOT,

    RF_TAG_PWR_OFF,

    RF_ANCHOR_PWR = 0x21,

    RF_ANCHOR_SWVER,

    RF_ANCHOR_HWVER,

    RF_ANCHOR_TYPE,

    RF_ANCHOR_SYNC,

    RF_ANCHOR_REBOOT,
    RF_SEND_RASLOT=0x50,//测距时隙
    RF_SEND_SESLOT,//传感器时隙
    RF_WORKINFO,
    RF_ONLYRANGESLOT,
    RF_REP=0x55,//发送报告
    RF_RST,//关闭anchor的RF功能
    RF_USR,//用户命令，用于关闭tag或者更改tag同步码
    RF_TAG_RESET=0XA0,
    RF_TAG_WORKINFO,
}FunctionCode;



typedef int8(*AirSigleInsHander)(uint8 uch_op, void *pdata, uint8 uch_length,
                                            void *data_area,uint16 uin_src, uint16 sn);

typedef struct
{
    uint8 funcCode;
    AirSigleInsHander h;
}AirInsHander;

#pragma pack(1)
typedef struct
{
    uint8 timeSlotNum;
    uint16 tag_id;
}ST_BcTagSlot;
#pragma pack()

#pragma pack(1)
typedef struct
{
    uint8 period;
    uint8 slot;
    uint16 tag_id;
}ST_BcTagSensorSlot;
#pragma pack()

#pragma pack(1)
typedef struct
{
    uint16 uin_PacketNum;
    uint8  uch_Type;
    uint8  uch_OpCode;
    uint8  uch_CtrlCode;
    uint8  uch_Lengh;
}ST_AirMessageHead;
#pragma pack()

void SendMessageToTag(uint16 uin_dest,uint8 uch_len,void* pdata);

void QueueOutputAndSend(void);

uint8 AirMessageProcess(void *in,uint16 len,void *out,uint16 src);

void AnchorCenterBroadcast(void);

void AnchorCenterBroadcastWithTagInfo(void);

void AnchorCenterSensorSlotBroadcast(void);

void RemoveOneTagInfo(int8_t tag_nr);

void OccupyOneTagInfo(int8_t tag_nr,uint32 tag_id,ST_TagStaticInfo* ptagStaticInfo);
int8 getUnuseTagInfo(void);

uint8 getAccesstagNum(uint16* tagid);

bool IsAlreadyKnowTagID(uint32 tag_id);
//发送测距时隙
void Send_RangeSlot();
//发送传感器时隙
void Send_SensorSlot();
/*
函数功能：发送用户命令，主要是关机以及那个切换网络
输入参数：buf:发送数据缓存
输出参数：无
作者：hehe
*/
void SendUsrCmd(uint8* buf);
/*
函数功能：发送工作信息
输入参数：无
输出参数：无
作者：hehe
*/
void Send_WorkInfo();
/*
函数功能：只发送传感器数据
输入参数：无
输出参数：无
作者：hehe
*/
void Send_RangeOnlySlot();
/*
函数功能：发送tag工作组号(需要的时候发送)
输入参数：无
输出参数；无
作者；hehe
*/
void Send_WorkGroup();
void Send_AllTagList();//发送所有的tag列表
typedef struct 
{
    uint16 TagID;
    uint8 RSSI[2];
    uint8 Index;//表明当前需要填充哪个buffer
}TagInfoTypeDef;
typedef struct
{
    uint8 Index;//当前可用的index
    TagInfoTypeDef TagInfo[20];
}TagListTypeDef;
/*
函数功能：发送tag区域号
输入参数：无
输出参数：无
作者：hehe
*/
void Send_TagRegion();
#endif






