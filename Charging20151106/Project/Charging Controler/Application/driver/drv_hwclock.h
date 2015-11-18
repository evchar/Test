/**
 * @file hwclock.h
 * @date 2007-Dez-11
 * @author S. Radtke
 * @c (C) 2007 Nanotron Technologies
 * @brief Timer support for AVR.
 *
 * @note BuildNumber = "BuildNumber : 7951";
 *
 * @note This file contains the function definitions for the utility functions
 *    of the AVR hardware timer.
 */
#ifndef	_DRV_HWCLOCK_H
#define	_DRV_HWCLOCK_H

#include "config.h"
#include "ntrxtypes.h"
extern uint32_t	hwclock(void);
extern void HWDelayus( uint16_t us );
extern void HWDelayms( uint16_t ms );
#endif	/* _DRV_HWCLOCK_H */


