/************************************************************
Copyright (C), 1988-1999, hzrq Tech. Co., Ltd.
FileName: setprint.c
Author: Huangtongyin
Version : 
Date:
Description: Redefine some functions in C standard about printf,and using UART2 to print.
Version: 
Function List: 
1. -------
History: 
<author> <time> <version > <desc>
***********************************************************/

#include <stdio.h>
//#include <rt_misc.h>
#include "stm32f2xx_usart.h"
extern volatile uint8_t full;
//#pragma import(__use_no_semihosting_swi)

extern volatile uint8_t serBuffer;
int SendChar(int ch) 
{
    while(!(USART1->SR & USART_FLAG_TXE))    
    {
			
    } 

    USART1->DR = (ch & 0x1FF);
		
    return(ch);

}

int GetKey(void) 
{
   // while(!(USART1->SR & USART_FLAG_RXNE))
   // {
			
    //}
   // return ((int)(USART1->DR & 0x1FF));
    while (full == 0);
	full = 0;
	return serBuffer;

}


//struct __FILE
//{
//    int handle; // Add whatever you need here
//};


FILE __stdout;


FILE __stdin;


int fputc(int ch, FILE *f)
{
    return (SendChar(ch));
}


int fgetc(FILE *f) 
{
    //return (SendChar(GetKey()));
    return (GetKey());
    //while (full == 0);
	//full = 0;
	//return serBuffer;
}


void _ttywrch(int ch) 
{
    SendChar (ch);
}


int ferror(FILE *f)
{ 
    // Your implementation of ferror
    return EOF;
}


void _sys_exit(int return_code)
{
    label: goto label; // endless loop
}


