#ifndef __TIMER_H__
#define __TIMER_H__

#include <stdint.h>

#define 	T_1MS		(1)
#define 	T_10MS		(10)
#define 	T_100MS		(100)
#define 	T_1S		(1000)
#define 	T_1MM		(T_1S*60)

#define TRUE 1
#define FALSE 0

void sysinit(void);

void Delayms(uint32_t ms);


uint32_t TimerGetSystemTimer(void);


void Timer_init(void);
void Timer_deinit(void);


#endif
