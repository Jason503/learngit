#include "uart.h"
#include "lpuart_driver.h"
#include "stdio.h"
lpuart_state_t lpuart1_State;
static uint8_t buf[255];

const lpuart_user_config_t lpuart1_InitConfig = {
  .transferType = LPUART_USING_INTERRUPTS,
  .baudRate = 115200U,
  .parityMode = LPUART_PARITY_DISABLED,
  .stopBitCount = LPUART_ONE_STOP_BIT,
  .bitCountPerChar = LPUART_8_BITS_PER_CHAR,
  .rxDMAChannel = 0U,
  .txDMAChannel = 0U,
};


//GPIO的初始化
void pinmuxinit(void)
{
	
	 PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
}

/*调试串口的初始化*/
void DebugUartInit(void)
{
		LPUART_DRV_Init(FLY_LPUART0, &lpuart1_State ,&lpuart1_InitConfig);
	  LPUART_DRV_ReceiveData(FLY_LPUART0 ,buf,sizeof(buf));/*开启接收中断数据*/
	  INT_SYS_SetPriority(LPUART0_RxTx_IRQn,2);/*设置串口中断的优先级为2*/
	
}



