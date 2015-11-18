#include "user_lib.h"
#include "stm32f10x.h"
#include "i2cdev.h"
#include "i2croutines.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define i2c_write	IICwriteBytes
#define i2c_read	IICreadBytes
#define delay_ms	vTaskDelay    //Delay_ms
#define get_ms		xTaskGetTickCount
#define log_i		printf
#define log_e		printf
#define min(a, b) 	((a < b) ? a : b)

#define DEFAULT_SAMPLE_RATE_HZ  20

#define DEFAULT_YAW_MIX_FACTOR  0