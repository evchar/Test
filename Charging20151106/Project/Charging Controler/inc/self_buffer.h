#ifndef __SLEF_BUFFER_H__
#define __SLEF_BUFFER_H__

#define DATE_BUF_LEN 128
#define QUEUE_BUF_LEN 5

void InitQueue(void);
uint8 QueueInsert(void *pInputDate,uint8 dateLength,uint8 p_id);
uint8 QueueDelete(void *pOutputDate,uint8* pLength, uint8 * p_id);


#endif