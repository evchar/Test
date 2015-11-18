/**
 * @file hwclock.c
 * @date 2007-Dez-11
 * @author S. Rohdemann, S. Radtke
 * @c (C) 2007 Nanotron Technologies
 * @brief Timer support for AVR.
 *
 * @note BuildNumber = "BuildNumber : 7951";
 *
 * @note This file contains the source code for the utility functions
 *    of the Hardware Timer Function.
 *
 */
#include	"config.h"

#ifndef __OPTIMIZE__
#	define __OPTIMIZE__ 0
#endif
#include "FreeRTOS.h"
#include "task.h"

#include    "ntrxtypes.h"

#include 	"keys.h"
#include	"avrport.h"
#include 	"portation.h"

#include "stm32f2xx.h"
#include "core_cm3.h"
#include "misc.h"

// #define CONFIG_SPI_TRACE 1
#ifdef CONFIG_SPI_TRACE
extern int traceOn;
#endif

// #define CONFIG_HWCLOCK_USE_CRYSTAL	(1)



#ifdef CONFIG_HWCLOCK_USE_CRYSTAL
#	define TIMER_RELOAD_VALUE		(256 - 32)	/*prescaler 1*/
#else
	// #define	MILLISECONDS_PER_TICK	10 /* at 7,3728 MHz prescaler 1024 -> TCNT0 = 256 - 72 for 10ms tick */
	// #define TIMER_RELOAD_VALUE		(256 - 72)
	// #define TIMER_RELOAD_VALUE		(256 - 200)
#	define TIMER_RELOAD_VALUE		(256 - 230)
#endif /*CONFIG_HWCLOCK_USE_CRYSTAL*/


#define	MILLISECONDS_PER_TICK	1 /* at 7,3728 MHz prescaler 32 -> TCNT0 = 256 - 230 for 1ms tick */


#define	STATE_HIGH	 5
#define	STATE_LOW	-5
#define MAX_DELAY_US ( 768 / ( F_CPU / 1000000UL ))
#define MAX_DELAY_MS ( 262 / ( F_CPU / 1000000UL ))

volatile bool_t key_flags[] = { FALSE, FALSE, FALSE };
volatile	uint32_t	jiffies = 0;
//static	uint32_t	bogo_mips = 0;
static	uint8_t	hwclTimerReloadVal = TIMER_RELOAD_VALUE;


#if 0
/***************************************************************************/
/**
 * Deprecated function please dont use for
 * new projects
 */
void hwdelay( uint32_t t )
{
  unsigned long lc = t >> 8;

  if (t == 0) return;

  while(lc--) _delay_loop_2(512);
  _delay_loop_2((t & 0xFF)<<1);
}
#endif


/** 
 * @功能：Initialize the systick
 * @输入：无
 * @输出：无 
 * @返回值：无
**/
void SysTickInit(void)
{
   	/* SystemFrequency / 1000    1ms中断一次
	   * SystemFrequency / 100000	 10us中断一次
	   * SystemFrequency / 1000000 1us中断一次
	   */
	   //SysTick_CLKSourceConfig
    if (SysTick_Config(SystemCoreClock / 1000 ))   // ST3.5.0库版本
	  {
		/* Capture error */
		while(1)
		{	
		}			
    }		 
}

#if 1
/***************************************************************************/
/**
 * @brief Delay processing for n microseconds.
 * @param us this is the delay in microseconds
 *
 * This function is used for waiting before continue with
 * programm execution. Interrupts are still processed.
 * Because of the high inaccuracy of the delay function
 * this function tries to compensate the delay error
 * by adding an offset.
 */
void HWDelayus( volatile uint16_t us )
{
	volatile uint8_t i;

#ifdef CONFIG_SPI_TRACE
	if( traceOn == TRUE )
	{
		printf ("           delay: %d usec\n", us);
	}
#endif
	if( us == 0 ) 
        return;

	/*
	 * this is for the loop processing time
	 * Delays in ms are about 10% longer.
	 * Delays in us between 100% down to 10%.
	 * _delay_us seams to be not very accurate.
	 */
	while(us--)
    {
		i= 32;
		while(i--);
    }
}



/***************************************************************************/
/**
 * @brief Delay processing for n milliseconds.
 * @param ms this is the delay in milliseconds
 *
 * This function is used for waiting TIMER_RELOAD_VALUEbefore continue with
 * programm execution. Interrupts are still processed.
 */
void HWDelayms( uint16_t ms )
{
	uint16_t i;
#ifdef CONFIG_SPI_TRACE
	if( traceOn == TRUE )
	{
		printf ("           delay: %d msec\n", ms);
	}
#endif /* CONFIG_SPI_TRACE */
	if( ms == 0 ) 
        return;

	/*
	 * this is for the loop processing time
	 * Delays in ms are about 10% longer.
	 * Delays in us between 100% down to 10%.
	 * _delay_us seams to be not very accurate.
	 */
	for(i = 0; i < ms; i++)
	{
		HWDelayus(2000);	
	}
}
#endif
/***************************************************************************/
/**
 * @brief clock tick initialization
 * @param start this is the start value for jiffies
 *
 * This function initializes the Timer 0 in the AVR to
 * generate an interrupt ever 10 ms.
 */
void hwclockRestart (uint32_t start)
{
    SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
	jiffies = start;
//	bogo_mips = 0;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}



/***************************************************************************/
/**
 * @brief clock tick initialization
 *
 * This function initializes the Timer 0 in the AVR to
 * generate an interrupt ever 10 ms.
 */
void hwclock_init(void)
{
    SysTickInit();

    hwclockRestart (0);
}



/***************************************************************************/
/**
 * @brief return system clock in milliseconds
 * @returns time in milliseconds
 *
 * This function returns the elapsed time since
 * program start in milliseconds.
 */
uint32_t	hwclock(void)
{
	uint32_t	r;

	//SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
	r = xTaskGetTickCount();
	//SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    
	return (r);
}



/***************************************************************************/
/**
 * @brief Modifies the time interval between any two subsequent timer ticks.
 * @param tuningDirection +1 to speed up the hwclock, -1 to speed down,
 * 0 to reset to default.
 */
void hwclock_tune(int8_t tuningDirection)
{
#	ifndef CONFIG_HWCLOCK_USE_CRYSTAL
	uint8_t reloadVal = hwclTimerReloadVal;

	switch (tuningDirection)
	{
		case -1:
			if (reloadVal > 0) reloadVal--;
			break;

		case 0:
			reloadVal = TIMER_RELOAD_VALUE;
			break;

		case 1:
			if (reloadVal < 0xFF) reloadVal++;
			break;

		default:
			break;
	}

	ENTER_TASK;
	hwclTimerReloadVal = reloadVal;
	LEAVE_TASK;
#	endif /* !CONFIG_HWCLOCK_USE_CRYSTAL */
}




/***************************************************************************/
uint8_t GetHwclReloadVal(void)
{
	return hwclTimerReloadVal;
}


#if 0
/***************************************************************************/
/**
 * @brief Update key state
 * @param stat this is the logic level of the key port
 * @param id this is the index of the keylist
 *
 * This function updates the keystate every 10ms.
 * The main purpose is to eliminate key bouncing.
 */
void KeyUpdate (uint8_t stat, uint8_t id)
{
	static int	state[] = { STATE_HIGH, STATE_HIGH, STATE_HIGH};

	if (stat != 0)
	{
		if(state[id] > STATE_LOW)
		{
			state[id]--;
		}
	}
	else
	{
		if(state[id] < STATE_HIGH)
		{
			state[id]++;
		}
	}

	if(state[id] == STATE_HIGH)
	{
		key_flags[id] = TRUE;
	}

	if(state[id] == STATE_LOW)
	{
		key_flags[id] = FALSE;
	}
}



/***************************************************************************/
/**
 * @brief Timer 0 overflow interrupt
 *
 * Interrupt service routine for the hardware timer
 */
SIGNAL(SIG_OVERFLOW0)
{
	AVR_TCNT0 = hwclTimerReloadVal;

	KeyUpdate (SW0_IN, 0);
	KeyUpdate (SW1_IN, 1);
	KeyUpdate (SW2_IN, 2);

	jiffies += MILLISECONDS_PER_TICK;
}
#endif

