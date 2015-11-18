/*! @file
********************************************************************************
<PRE>
--------------------------------------------------------------------------------
文件名       : TypeDefine.h
文件实现功能 : 数据类型定义
作者         : 蔡立挺
版本         : V1.0
备注         :
--------------------------------------------------------------------------------
修改记录 :
日 期        版本     修改人              修改内容
2013/4/15   1.0      hujiang                          long long int64
--------------------------------------------------------------------------------
</PRE>
*******************************************************************************/

#ifndef __USERTYPEDEFINE_H__
#define __USERTYPEDEFINE_H__

//#define ___WIN32___

#ifndef ___WIN32___
    typedef unsigned char  bool;
    typedef unsigned char  uint8;
    typedef signed   char  int8;
    typedef unsigned short uint16;
    typedef signed   short int16;
    typedef unsigned int   uint32;
    typedef signed   int   int32;
    typedef float          fp32;
    typedef double         fp64;
    typedef unsigned long long int64; 
    typedef signed long long uint64;
#else
    typedef unsigned char  bool;
    typedef unsigned char  uint8;
    typedef signed   char  int8;
    typedef unsigned short uint16;
    typedef signed   short int16;
    typedef unsigned int   uint32;
    typedef signed   int   int32;
    typedef float          fp32;
    typedef double         fp64;
#endif

#if 1
#ifndef NULL
#define NULL    (void *)0
#endif

#ifndef false
#define false   0
#endif

#ifndef true
#define true    1
#endif
#endif

/** @brief Bool type. */
typedef enum    {
    FALSE = 0,
    TRUE = 1
}   bool_t;/** @brief Bool type. */

typedef void (*pFUNC)(void);
#define ArrayLen(a) (sizeof(a)/sizeof(a[0]))

#define BIN2BCD(val)    ((((val)/10)<<4) + (val)%10)
#define BCD2BIN(val)    (((val)&15) + ((val)>>4)*10)

#endif//__USERTYPEDEFINE_H__

