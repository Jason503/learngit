#ifndef __UART_H
#define __UART_H
#include "s32k_conf.h"
#define FLY_LPUART0 0U

void pinmuxinit(void);		//初始化引脚
void DebugUartInit(void);//调试串口的初始化
void getdata(void);



#endif

