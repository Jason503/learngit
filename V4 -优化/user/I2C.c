#include "I2C.h"
#define IO_I2C_C		GPIO_PTD9   // I2C_C
#define IO_COMM_TX_REQ	IO_I2C_C  //������ͨ��ʱ֪ͨ������Ҫ����,�����ݷ���ʱ����,û��ʱ����

/* Definition of the data transfer size */
#define TRANSFER_SIZE (256u)

/* Declaration of the LPI2C RX buffer */
uint8_t rxBuffer[TRANSFER_SIZE];
/* Declaration of the LPI2C TX buffer */
uint8_t txBuffer[TRANSFER_SIZE];


lpi2c_slave_state_t lpi2c0SlaveState;
uint8_t my1Buff[255]={0};
uint8_t myBuff[255]={0};
uint8_t my_txBuff[255]={0};

void I2C_init(void)
{
	//IO_Write(IO_COMM_TX_REQ, true);
	LPI2C_DRV_SlaveInit(INST_LPI2C0, &lpi2c0_SlaveConfig0, &lpi2c0SlaveState);/*��ʼ��I2C*/
	//LPI2C_DRV_SlaveSetRxBuffer(0U,my1Buff,sizeof(my1Buff));/*����I2C�Ľ���buffer*/
  LPI2C_DRV_SlaveReceiveData(0U,myBuff,sizeof(myBuff));
 // LPI2C_DRV_SlaveSendData(0U,my_txBuff,sizeof(my_txBuff));
  INT_SYS_SetPriority(LPI2C0_Slave_IRQn,2);/*���ô����жϵ����ȼ�Ϊ2*/
}



/*
 *  @details This function will be called by LPI2C interrupt handler and it
 *  will assign the buffer for TX or RX events.
 *  If an error event occurs, it will abort the current transfer.
 */
void lpi2c0_SlaveCallback0(uint8_t instance,lpi2c_slave_event_t slaveEvent,void *userData)
{
    /* Casting userData to void in order to avoid warning as the parameter is not used */
    (void)userData;
	
    /* Depending on the event received, set the buffers or abort the transfer */
    switch(slaveEvent)
    {
        case LPI2C_SLAVE_EVENT_RX_REQ:
            /*
             * If the bus master requests data, then set the destination RX buffer
             * and accepted transfer size
             */
            LPI2C_DRV_SlaveSetRxBuffer(instance, rxBuffer, TRANSFER_SIZE);
            break;
        case LPI2C_SLAVE_EVENT_TX_REQ:
            /*
             * If the bus master sends data, then set the source TX buffer
             * and accepted transfer size
             */
            LPI2C_DRV_SlaveSetTxBuffer(instance, txBuffer, TRANSFER_SIZE);
            break;
        case LPI2C_SLAVE_EVENT_TX_EMPTY:
        case LPI2C_SLAVE_EVENT_RX_FULL:
            /*
             * If the TX buffer is empty or the RX buffer is full, abort the current
             * transfer.
             */
            LPI2C_DRV_SlaveAbortTransferData(instance);
            break;
        case LPI2C_SLAVE_EVENT_STOP:
            /*
             * This case is used when a stop condition is on the bus. Because
             * the example does not handle this case there is no action taken.
             */
            break;
    }
}

