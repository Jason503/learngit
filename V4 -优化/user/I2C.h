#ifndef __I2C_H
#define __I2C_H
#include"lpi2c1.h" 
#include "s32k_conf.h"
#include "lpi2c_driver.h"
void I2C_init(void);//IIC�ĳ�ʼ��
extern void lpi2c0_SlaveCallback0(uint8_t instance,lpi2c_slave_event_t slaveEvent,void *userData);
#endif
