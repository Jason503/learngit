#include"fly_can.h"

#include "stdio.h"
 flexcan_data_info_t dataInfo =
    {
            .data_length = 8U,
            .msg_id_type = FLEXCAN_MSG_ID_STD,
            .enable_brs  = true,
            .fd_enable   = true,
            .fd_padding  = 0U
    };
		
	
		
void fly_can_init(void)
{
		printf("fly_can_init\n");
	
		/*对CAN0的初始化*/
		FLEXCAN_DRV_Init(INST_CANCOM0, &canCom0_State, &canCom0_InitConfig0);
	
	  /*对CAN1的初始化*/

		FLEXCAN_DRV_Init(INST_CANCOM1, &canCom1_State, &canCom1_InitConfig0);
	
		/*对CAN2的初始化*/
		FLEXCAN_DRV_Init(INST_CANCOM2, &canCom2_State, &canCom2_InitConfig0);
	


}

void SendCANData(uint8_t instance,uint32_t mailbox, uint32_t messageId, uint8_t * data, uint32_t len)
{
    /* Set information about the data to be sent
     *  - 1 byte in length
     *  - Standard message ID
     *  - Bit rate switch enabled to use a different bitrate for the data segment
     *  - Flexible data rate enabled
     *  - Use zeros for FD padding
     */
     flexcan_data_info_t dataInfo1 =
    {
            .data_length = len,
            .msg_id_type = FLEXCAN_MSG_ID_STD,
            .enable_brs  = true,
            .fd_enable   = false,
            .fd_padding  = 0U
    };

    /* Configure TX message buffer with index TX_MSG_ID and TX_MAILBOX*/
    FLEXCAN_DRV_ConfigTxMb(instance, mailbox, &dataInfo1, messageId);

    /* Execute send non-blocking */
    FLEXCAN_DRV_Send(instance, mailbox, &dataInfo1, messageId, data);
		
}


