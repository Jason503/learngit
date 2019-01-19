#include <stdio.h>
#include <string.h>
#include "s32k.h"
#define CarUniCom	LPUART0	

extern  volatile uint32_t s_osif_tick_cnt;

//关闭中断
void _DI(void)
{
	INT_SYS_DisableIRQGlobal();
}

//开始中断
void _EI(void)
{
	INT_SYS_EnableIRQGlobal();
}











static void UARTPutChar(LPUART_Type * base, uint8_t data)
{
    /* Wait for transmit buffer to be empty */
    while((base->STAT & LPUART_STAT_TDRE_MASK)>>LPUART_STAT_TDRE_SHIFT==0);
    base->DATA=data;
}


//实现printf函数
int fputc(int ch, FILE * f)
{
	uint8_t c;
	uint8_t i;
	char timestr[16];
	int size = 0;

	c = ch & 0xFF;
	switch(c)
	{
	case '\n':
		c = '\r';
		UARTPutChar(CarUniCom, c);
		c = '\n';
		UARTPutChar(CarUniCom, c);
		size = snprintf(timestr, 16, "[P %10u] ", s_osif_tick_cnt);/*打印内核时钟的频率  s_osif_tick_cnt*/
		for(i = 0; i < size; i++)
		UARTPutChar(CarUniCom, timestr[i]);
		break;
	case '\r':
		break;
	default:
		UARTPutChar(CarUniCom, c);
		break;
	}

	return 0;
}


void IO_Write(GPIO_PinType GpioPin, bool status) {
    GPIO_Type *base;

	if(GpioPin >= GPIO_PIN_MAX) 
	{
		return;
	}
	
    base = ( GPIO_Type *)(PTA_BASE + ((GpioPin >> 5) << 6));

	if(status)
		base->PSOR = 1U << (GpioPin & 0x1F);
	else
		base->PCOR = 1U << (GpioPin & 0x1F);
}


