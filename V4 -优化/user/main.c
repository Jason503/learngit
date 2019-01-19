#include "stdio.h"
#include "s32k_conf.h"
#include "timer.h"
#include "Androidcom.h"
#include "TBT.h"
#include "string.h"

/************�봮���й�**************************/
uint8_t USART_RX_STA = 0;
uint8_t USART_RX_STA_flag = 0;
uint8_t char_num = 0;
uint8_t flybuf[255];//�洢���ڵ�����
/***********��CAN�й�****************************/

uint8_t flag_rx =0;
uint8_t receive_flag = 0;//CAN���յ����ݵı�־

/************************************************/


int main()
{
	
	
	uint8_t res = 0;
	 /*ʹ�������ں��ж�*/
    _DI();
    _EI();
	
//	IO_Write( GPIO_PTD9, false);
	
	
	/*��ʼ�����ʱ��*/
	sysinit();
	
	
	while(1){};
	/*��ʼ���������*/
  pinmuxinit();                
	
	//��ʼ��ϵͳʱ�Ӻ���
  Timer_init();
	
	/*��ʼ�����Դ���0*/
	DebugUartInit();
	
	/*��ʼ��I2C*/
	I2C_init();
	
	/*��ʼ��CAN1*/
	fly_can_init();
	

	
	 /* Configure RX message buffer with index RX_MSG_ID and RX_MAILBOX */
	//  flexcan_msgbuff_t recvBuff;
  // FLEXCAN_DRV_ConfigRxMb(INST_CANCOM0, RX_MAILBOX, &dataInfo, RX_MSG_ID);/*�ܵ���˵����Ҫ�����˽���Ϊ�յ�״̬*/


	while(1)
	{
		
		if(USART_RX_STA)//���ݽ������
		{
			res = TBT_Handler(flybuf,char_num);
			if(!res)
			{
				printf("Data receive success\n");
			}
			else printf("Data receive error res =%d \n",res);	
			
			memset(flybuf,0,sizeof(flybuf));//flybuf
			char_num = 0;
			USART_RX_STA = 0;	
		}
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		/*
		if(sendflag)
		{
			sendflag = 0;//ÿ�����ڷ���һ��a���ʹ�������CAN1����һ�Ρ�
			printf("ready send can1 data\n");
			SendCANData(INST_CANCOM0,TX_MAILBOX, TX_MSG_ID, CANData, 8);//����CAN1����       ���͸�CAN2(TX_MAILBOX, TX_MSG_ID, CANData, 1UL)
		}
		 FLEXCAN_DRV_Receive(INST_CANCOM0, RX_MAILBOX, &recvBuff);
		if(flag_rx)
		{
			 flag_rx = 0;
			
	
				 printf("value1 =%d\n", recvBuff.data[0]);
				 printf("value2 =%d\n", recvBuff.data[1]);
				 printf("value3 =%d\n", recvBuff.data[2]);
				 printf("value4 =%d\n", recvBuff.data[3]);
				 printf("value5 =%d\n", recvBuff.data[4]);
				 printf("value6 =%d\n", recvBuff.data[5]);
				 printf("value7 =%d\n", recvBuff.data[6]);
				 printf("value8 =%d\n", recvBuff.data[7]);
				 printf("value9 =%d\n", recvBuff.data[8]);
				 printf("value10=%d\n", recvBuff.data[9]);
				 printf("value11=%d\n", recvBuff.data[10]);
				 printf("value12=%d\n", recvBuff.data[11]);
				 printf("value13=%d\n", recvBuff.data[12]);
				 printf("value14=%d\n", recvBuff.data[13]);
				 printf("value15=%d\n", recvBuff.data[14]);
				 printf("value16=%d\n", recvBuff.data[15]);
				 printf("value  =%d\n", recvBuff.msgId);
				 printf("value  =%d\n", recvBuff.dataLen);
				 printf("value  =%s\n", recvBuff.data);
				 


		}
		   
*/
				 
//		        /* Define receive buffer */
//        flexcan_msgbuff_t recvBuff;

//        /* Start receiving data in RX_MAILBOX. */
//        FLEXCAN_DRV_Receive(INST_CANCOM1, RX_MAILBOX, &recvBuff);

//        /* Wait until the previous FlexCAN receive is completed */
//        while(FLEXCAN_DRV_GetTransferStatus(INST_CANCOM1, RX_MAILBOX) == STATUS_BUSY);


//        /* Check the received message ID and payload �����CAN���յ���Ϣ�����жϣ�0 �� 1�������Һ�Ҫ���յ�ID�Ž��й���*/
//        if((recvBuff.data[0] == 0) &&
//                recvBuff.msgId == RX_MSG_ID)
//        {
//						printf("recever1:%d\n",recvBuff.data[0]);
//            /* Toggle output value LED1 */
//           // PINS_DRV_TogglePins(GPIO_PORT, (1 << LED0));/*��תled0������״̬*/
//        }
//        else if((recvBuff.data[0] == 1) &&
//                recvBuff.msgId == RX_MSG_ID)
//        {
//						printf("recever2:%d\n",recvBuff.data[0]);
//            /* Toggle output value LED0 */
//           // PINS_DRV_TogglePins(GPIO_PORT, (1 << LED1));/*��תled1������״̬*/
//        }
//			Delayms(1000);
//		
//			printf("test\n\r");
	}

}



