#ifndef __S32K_H
#define __S32K_H
#include "s32k_conf.h"
#include "lpuart_driver.h"



void _DI(void);						//关闭中断
void _EI(void);						//开启中断
void IO_Write(GPIO_PinType GpioPin, bool status) ;











#endif 


