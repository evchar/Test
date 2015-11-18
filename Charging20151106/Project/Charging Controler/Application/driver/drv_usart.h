#ifndef __DRV_USART_H__
#define __DRV_USART_H__
#include "global_value.h"
void Usart4Init( void );
void USART4SendBuffer(u8 *ptr,uint8_t count);
void Usart4RevBufInit( void );
int SwapRevBuf(usart_data_rev_t* rev_usart);
void ConsoleInit(void);
#endif