#ifndef __UART_H
#define __UART_H
#include "s32k_conf.h"
#define FLY_LPUART0 0U

void pinmuxinit(void);		//��ʼ������
void DebugUartInit(void);//���Դ��ڵĳ�ʼ��
void getdata(void);



#endif

