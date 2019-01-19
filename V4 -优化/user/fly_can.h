#ifndef __FLY_CAN_H
#define __FLY_CAN_H
#define TX_MAILBOX  (1UL)
#define TX_MSG_ID   (9UL)
#define RX_MAILBOX  (0UL)/*ԭ����0UL*/
#define RX_MSG_ID   (9UL)/*ԭ����2UL*/

#include "canCom1.h"
#include "flexcan_driver.h"
void fly_can_init(void);//CAN�ĳ�ʼ��
void SendCANData(uint8_t instance,uint32_t mailbox, uint32_t messageId, uint8_t * data, uint32_t len);
extern flexcan_data_info_t dataInfo ;
#endif


