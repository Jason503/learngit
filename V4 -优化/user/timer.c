#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "device_registers.h"
#include "interrupt_manager.h"
#include "s32k_conf.h"
#include "timer.h"



static volatile uint32_t _x_pit_timer_counter = 0;



//定义1ms中断,time单位为ms
static inline void SysTickInit(void)
{	
	uint32_t ticks = (48000000/1000)*1;
	
	_DI();

	S32_SysTick->RVR  = (uint32_t)(ticks - 1UL);                      
	S32_SysTick->CVR   = 0UL;                                 
	S32_SysTick->CSR  = S32_SysTick_CSR_CLKSOURCE_MASK |
	               		S32_SysTick_CSR_TICKINT_MASK  |
	               		S32_SysTick_CSR_ENABLE_MASK;  
	_EI();
}

static inline void SysTickDeInit(void)
{
	S32_SysTick->CSR &= ~S32_SysTick_CSR_TICKINT_MASK;
	S32_SysTick->CSR &= ~S32_SysTick_CSR_ENABLE_MASK;
}

//系统时钟的初始化
void Timer_init(void) 
{
    SysTickInit();
}

//系统时钟的复位
void Timer_deinit(void) 
{
    SysTickDeInit();
}


//利用系统时钟来延时 ms级别的延时
void Delayms(uint32_t ms) {

    OSIF_TimeDelay(ms);
}


//获取系统时钟的运行时间
uint32_t TimerGetSystemTimer(void) {

	return _x_pit_timer_counter;
}

//时钟初始化
void sysinit(void)
{
	  CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
                 g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
	/*CLOCK_MANAGER_POLICY_FORCIBLE 强有力的时钟   CLOCK_MANAGER_POLICY_AGREEMENT 平缓的时钟 */
		CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_FORCIBLE);
    
}


