/*
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "uart_pal.h"
#include "device_registers.h"

#define UART_OVER_LPUART 

/* Include PD files */
#if (defined(UART_OVER_LPUART))
    #include "lpuart_driver.h"
#endif

#if (defined(UART_OVER_FLEXIO))
    #include "flexio.h"
    #include "flexio_uart_driver.h"
#endif

#if (defined(UART_OVER_LINFLEXD))
    #include "linflexd_uart_driver.h"
#endif

/* Define state structures for LPUART */
#if (defined(UART_OVER_LPUART))
    /*! @brief LPUART state structures */
    lpuart_state_t * lpuartState[NO_OF_LPUART_INSTS_FOR_UART];
    /*! @brief LPUART state-instance matching */
    static uart_instance_t lpuartStateInstanceMapping[NO_OF_LPUART_INSTS_FOR_UART];
    /*! @brief LPUART available resources table */
    static bool lpuartStateIsAllocated[NO_OF_LPUART_INSTS_FOR_UART];
#endif

/* Define state structures for UART over FLEXIO */
#if (defined(UART_OVER_FLEXIO))
    /*! @brief FLEXIO state structures for Tx*/
    flexio_uart_state_t flexioUartTxState[NO_OF_FLEXIO_INSTS_FOR_UART];
    /*! @brief FLEXIO state structures for Rx*/
    flexio_uart_state_t flexioUartRxState[NO_OF_FLEXIO_INSTS_FOR_UART];
    flexio_device_state_t flexioDeviceState;
    /*! @brief FLEXIO state-instance matching */
    static uart_instance_t flexioUartStateInstanceMapping[NO_OF_FLEXIO_INSTS_FOR_UART];
    /*! @brief FLEXIO  available resources table */
    static bool flexioUartStateIsAllocated[NO_OF_FLEXIO_INSTS_FOR_UART];
#endif

/* Define state structures for LinFlexD */
#if (defined(UART_OVER_LINFLEXD))
    /*! @brief LinFlexD state structures */
    linflexd_uart_state_t linFlexDState[NO_OF_LINFLEXD_INSTS_FOR_UART];
    /*! @brief LinFlexD state-instance matching */
    static uart_instance_t linFlexDStateInstanceMapping[NO_OF_LINFLEXD_INSTS_FOR_UART];
    /*! @brief LinFlexD  available resources table */
    static bool linFlexDStateIsAllocated[NO_OF_LINFLEXD_INSTS_FOR_UART];
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_AllocateState
 * Description   : Allocates one of the available state structures.
 *
 *END**************************************************************************/
static uint8_t UART_AllocateState(bool* isAllocated,
                                  uart_instance_t* instanceMapping,
                                  uart_instance_t instance,
                                  uint8_t numberOfinstances)
{
    uint8_t i;
    /* Allocate one of the UART state structures for this instance */
    for (i = 0; i < numberOfinstances; i++)
    {
        if (isAllocated[i] == false)
        {
            instanceMapping[i] = instance;
            isAllocated[i] = true;
            break;
        }
    }
    return i;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_FreeState
 * Description   : Deallocates one of the available state structures.
 *
 *END**************************************************************************/
static void UART_FreeState(bool* isAllocated,
                           uart_instance_t* instanceMapping,
                           uart_instance_t instance,
                           uint8_t numberOfinstances)
{
    uint8_t i;
    /* Free one of the UART state structures for this instance */
    for (i = 0;i < numberOfinstances;i++)
    {
        if (instanceMapping[i] == instance)
        {
            isAllocated[i] = false;
            break;
        }
    }
}

#if (defined(UART_OVER_FLEXIO))
/*FUNCTION**********************************************************************
 *
 * Function Name : UART_FindFlexioState
 * Description   : Search the state structure of the FlexIO instance
 *
 *END**************************************************************************/
static uint8_t UART_FindFlexioState(uart_instance_t instance)
{
    uint8_t i;
    for (i = 0; i < NO_OF_FLEXIO_INSTS_FOR_UART; i++)
    {
        if (flexioUartStateInstanceMapping[i] == instance)
        {
            break;
        }
    }
    return i;
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_Init
 * Description   : Configures the UART module
 *
 * Implements    : UART_Init_Activity
 *END**************************************************************************/
status_t UART_Init(uart_instance_t instance, uart_user_config_t *config)
{
    status_t status = STATUS_SUCCESS;
    uint8_t index = 0;

    /* Define UART PAL over LPUART */
    #if (defined (UART_OVER_LPUART))
    if (instance <= LPUART_HIGH_INDEX)
    {
        lpuart_user_config_t lpuartConfig;

        lpuartConfig.baudRate = config->baudRate;

        /* LPUART supports only 8, 9 or 10 bits per character */
        DEV_ASSERT((config->bitCount >= UART_8_BITS_PER_CHAR) &&
                   (config->bitCount <= UART_10_BITS_PER_CHAR));
        switch (config->bitCount)
        {
            case UART_8_BITS_PER_CHAR:
                lpuartConfig.bitCountPerChar = LPUART_8_BITS_PER_CHAR;
                break;
            case UART_9_BITS_PER_CHAR:
                lpuartConfig.bitCountPerChar = LPUART_9_BITS_PER_CHAR;
                break;
            case UART_10_BITS_PER_CHAR:
                lpuartConfig.bitCountPerChar = LPUART_10_BITS_PER_CHAR;
                break;
            default:
                /* Impossible type - do nothing */
                break;
        }

        lpuartConfig.parityMode = (lpuart_parity_mode_t)config->parityMode;
        lpuartConfig.stopBitCount = (lpuart_stop_bit_count_t)config->stopBitCount;
        lpuartConfig.transferType = (lpuart_transfer_type_t)config->transferType;
        lpuartConfig.rxDMAChannel = config->rxDMAChannel;
        lpuartConfig.txDMAChannel = config->txDMAChannel;

        /* Allocate one of the LPUART state structure for this instance */
        index = UART_AllocateState(lpuartStateIsAllocated,
                                   lpuartStateInstanceMapping,
                                   instance,
                                   NO_OF_LPUART_INSTS_FOR_UART);
        /* Initialize LPUART instance */
        status = LPUART_DRV_Init(instance, (lpuart_state_t*)(&lpuartState[index]), &lpuartConfig);

        /* Install Rx callback */
        if (config->rxCallback != NULL)
        {
            LPUART_DRV_InstallRxCallback(instance, config->rxCallback, config->rxCallbackParam);
        }

        /* Install Tx callback */
        if (config->txCallback != NULL)
        {
            LPUART_DRV_InstallTxCallback(instance, config->txCallback, config->txCallbackParam);
        }
    }
    #endif

    /* Define UART PAL over FLEXIO */
    #if (defined (UART_OVER_FLEXIO))
    if ((instance >= FLEXIO_UART_LOW_INDEX) && (instance <= FLEXIO_UART_HIGH_INDEX))
    {
        /* FlexIO driver can be used with parity disabled and one stop bit */
        DEV_ASSERT(config->parityMode == UART_PARITY_DISABLED);
        DEV_ASSERT(config->stopBitCount == UART_ONE_STOP_BIT);

        flexio_uart_user_config_t flexioUartTxConfig;
        flexio_uart_user_config_t flexioUartRxConfig;

        /* Set baud rate for Tx and Rx */
        flexioUartTxConfig.baudRate = config->baudRate;
        flexioUartRxConfig.baudRate = config->baudRate;

        /* Set transfer type for Tx and Rx */
        switch (config->transferType)
        {
            case UART_USING_DMA:
                flexioUartTxConfig.driverType = FLEXIO_DRIVER_TYPE_DMA;
                flexioUartRxConfig.driverType = FLEXIO_DRIVER_TYPE_DMA;
                break;
            case UART_USING_INTERRUPTS:
                flexioUartTxConfig.driverType = FLEXIO_DRIVER_TYPE_INTERRUPTS;
                flexioUartRxConfig.driverType = FLEXIO_DRIVER_TYPE_INTERRUPTS;
                break;
            default:
                /* Impossible type - do nothing */
                break;
        }

        /* Set bit count per char for Tx and Rx */
        switch (config->bitCount)
        {
            case UART_7_BITS_PER_CHAR:
                flexioUartTxConfig.bitCount = 7U;
                flexioUartRxConfig.bitCount = 7U;
                break;
            case UART_8_BITS_PER_CHAR:
                flexioUartTxConfig.bitCount = 8U;
                flexioUartRxConfig.bitCount = 8U;
                break;
            case UART_9_BITS_PER_CHAR:
                flexioUartTxConfig.bitCount = 9U;
                flexioUartRxConfig.bitCount = 9U;
                break;
            case UART_10_BITS_PER_CHAR:
                flexioUartTxConfig.bitCount = 10U;
                flexioUartRxConfig.bitCount = 10U;
                break;
            case UART_15_BITS_PER_CHAR:
                flexioUartTxConfig.bitCount = 15U;
                flexioUartRxConfig.bitCount = 15U;
                break;
            case UART_16_BITS_PER_CHAR:
                flexioUartTxConfig.bitCount = 16U;
                flexioUartRxConfig.bitCount = 16U;
                break;
            default:
                /* Impossible type - do nothing */
                break;
        }

        /* Configure Tx */
        flexioUartTxConfig.direction = FLEXIO_UART_DIRECTION_TX;
        flexioUartTxConfig.dmaChannel = config->txDMAChannel;
        flexioUartTxConfig.dataPin = ((extension_flexio_for_uart_t*)(config->extension))->dataPinTx;

        /* Configure Rx */
        flexioUartRxConfig.direction = FLEXIO_UART_DIRECTION_RX;
        flexioUartRxConfig.dmaChannel = config->rxDMAChannel;
        flexioUartRxConfig.dataPin = ((extension_flexio_for_uart_t*)(config->extension))->dataPinRx;

        /* Link Flexio Callbacks to the callbacks defined in PAL */
        flexioUartRxConfig.callback = config->rxCallback;
        flexioUartRxConfig.callbackParam = config->rxCallbackParam;
        flexioUartTxConfig.callback = config->txCallback;
        flexioUartTxConfig.callbackParam = config->txCallbackParam;

        status = FLEXIO_DRV_InitDevice(0U, &flexioDeviceState);
        if (status == STATUS_SUCCESS)
        {
            /* Allocate one of the Flexio state structure for this instance */
            index = UART_AllocateState(flexioUartStateIsAllocated,
                                       flexioUartStateInstanceMapping,
                                       instance,
                                       NO_OF_FLEXIO_INSTS_FOR_UART);
            /* Init FlexIO UART driver for Tx */
            status = FLEXIO_UART_DRV_Init(0U,
                                          &flexioUartTxConfig,
                                          (flexio_uart_state_t*)&flexioUartTxState[index]);
            /* Init FlexIO UART driver for Rx */
            status = FLEXIO_UART_DRV_Init(0U,
                                          &flexioUartRxConfig,
                                          (flexio_uart_state_t*)&flexioUartRxState[index]);
        }

    }
    #endif

    /* Define UART PAL over LinFlexD */
    #if (defined (UART_OVER_LINFLEXD))
    linflexd_uart_user_config_t linFlexDConfig;

    linFlexDConfig.baudRate = config->baudRate;

    /* LinFlexD does not support 9 or 10 bits per character */
    DEV_ASSERT((config->bitCount != UART_9_BITS_PER_CHAR) &&
               (config->bitCount != UART_10_BITS_PER_CHAR));
    switch (config->bitCount)
    {
        case UART_7_BITS_PER_CHAR:
            linFlexDConfig.wordLength = LINFLEXD_UART_7_BITS;
            break;
        case UART_8_BITS_PER_CHAR:
            linFlexDConfig.wordLength = LINFLEXD_UART_8_BITS;
            break;
        case UART_15_BITS_PER_CHAR:
            linFlexDConfig.wordLength = LINFLEXD_UART_15_BITS;
            break;
        case UART_16_BITS_PER_CHAR:
            linFlexDConfig.wordLength = LINFLEXD_UART_16_BITS;
            break;
        default:
            /* Impossible type - do nothing */
            break;
    }

    switch (config->parityMode)
    {
        case UART_PARITY_DISABLED:
            linFlexDConfig.parityCheck = false;
            break;
        case UART_PARITY_EVEN:
            linFlexDConfig.parityCheck = true;
            linFlexDConfig.parityType = LINFLEXD_UART_PARITY_EVEN;
            break;
        case UART_PARITY_ODD:
            linFlexDConfig.parityCheck = true;
            linFlexDConfig.parityType = LINFLEXD_UART_PARITY_ODD;
            break;
        default:
            /* Impossible type - do nothing */
            break;
    }

    linFlexDConfig.stopBitsCount = (linflexd_uart_stop_bits_count_t)config->stopBitCount;
    linFlexDConfig.transferType = (linflexd_uart_transfer_type_t)config->transferType;
    linFlexDConfig.txDMAChannel = config->txDMAChannel;
    linFlexDConfig.rxDMAChannel = config->rxDMAChannel;

    /* Allocate one of the LinFlexD state structures for this instance */
    index = UART_AllocateState(linFlexDStateIsAllocated,
                               linFlexDStateInstanceMapping,
                               instance,
                               NO_OF_LINFLEXD_INSTS_FOR_UART);
    /* Initialize LinFlexD instance */
    status = LINFLEXD_UART_DRV_Init(instance, &linFlexDState[index], &linFlexDConfig);

    /* Install Rx callback */
    if (config->rxCallback != NULL)
    {
        LINFLEXD_UART_DRV_InstallRxCallback(instance, config->rxCallback, config->rxCallbackParam);
    }

    /* Install Tx callback */
    if (config->txCallback != NULL)
    {
        LINFLEXD_UART_DRV_InstallTxCallback(instance, config->txCallback, config->txCallbackParam);
    }

    #endif
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_Deinit
 * Description   : De-initializes the UART module
 *
 * Implements    : UART_Deinit_Activity
 *END**************************************************************************/
status_t UART_Deinit(uart_instance_t instance)
{
    status_t status = STATUS_SUCCESS;

    /* Define UART PAL over LPUART */
    #if defined(UART_OVER_LPUART)
    if (instance <= LPUART_HIGH_INDEX)
    {
        status = LPUART_DRV_Deinit((uint32_t)instance);
        if (status == STATUS_SUCCESS)
        {
            UART_FreeState(lpuartStateIsAllocated,
                           lpuartStateInstanceMapping,
                           instance,
                           NO_OF_LPUART_INSTS_FOR_UART);
        }
    }
    #endif

    /* Define UART PAL over FLEXIO */
    #if defined(UART_OVER_FLEXIO)
    if ((instance >= FLEXIO_UART_LOW_INDEX) && (instance <= FLEXIO_UART_HIGH_INDEX))
    {
        uint8_t index = UART_FindFlexioState(instance);
        status = FLEXIO_UART_DRV_Deinit(&(flexioUartTxState[index]));
        if (status == STATUS_SUCCESS)
        {
            UART_FreeState(flexioUartStateIsAllocated,
                           flexioUartStateInstanceMapping,
                           instance,
                           NO_OF_FLEXIO_INSTS_FOR_UART);
        }
        status = FLEXIO_UART_DRV_Deinit(&(flexioUartRxState[index]));
        if (status == STATUS_SUCCESS)
        {
            UART_FreeState(flexioUartStateIsAllocated,
                           flexioUartStateInstanceMapping,
                           instance,
                           NO_OF_FLEXIO_INSTS_FOR_UART);
        }
    }
    #endif

    /* Define UART PAL over LinFlexD */
    #if (defined(UART_OVER_LINFLEXD))
    status = LINFLEXD_UART_DRV_Deinit(instance);
    if (status == STATUS_SUCCESS)
    {
        UART_FreeState(linFlexDStateIsAllocated,
                       linFlexDStateInstanceMapping,
                       instance,
                       NO_OF_LINFLEXD_INSTS_FOR_UART);
    }
    #endif
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_SetBaudRate
 * Description   : Configures the UART baud rate.
 *
 * Implements    : UART_SetBaudRate_Activity
 *END**************************************************************************/
status_t UART_SetBaudRate(uart_instance_t instance, uint32_t desiredBaudRate)
{
    status_t status = STATUS_SUCCESS;

    /* Define UART PAL over LPUART */
    #if defined(UART_OVER_LPUART)
    if (instance <= LPUART_HIGH_INDEX)
    {
        status = LPUART_DRV_SetBaudRate((uint32_t)instance, desiredBaudRate);
    }
    #endif

    /* Define UART PAL over FLEXIO */
    #if defined(UART_OVER_FLEXIO)
    if ((instance >= FLEXIO_UART_LOW_INDEX) && (instance <= FLEXIO_UART_HIGH_INDEX))
    {
        uint8_t index = UART_FindFlexioState(instance);
        uint8_t bitCount = (flexioUartTxState[index]).bitCount;
        status = FLEXIO_UART_DRV_SetConfig(&(flexioUartTxState[index]),
                                           desiredBaudRate,
                                           bitCount);
        status = FLEXIO_UART_DRV_SetConfig(&(flexioUartRxState[index]),
                                           desiredBaudRate,
                                           bitCount);
    }
    #endif

    /* Define UART PAL over LinFlexD */
    #if (defined(UART_OVER_LINFLEXD))
    status = LINFLEXD_UART_DRV_SetBaudRate(instance, desiredBaudRate);
    #endif
    return status;
}

/*!
 * @brief Returns the UART baud rate.
 *
 * This function returns the UART configured baud rate.
 *
 * @param instance  instance number.
 * @param[out] configuredBaudRate configured baud rate.
 */
status_t UART_GetBaudRate(uart_instance_t instance, uint32_t * configuredBaudRate)
{
    status_t status = STATUS_SUCCESS;

    /* Define UART PAL over LPUART */
    #if defined(UART_OVER_LPUART)
    if (instance <= LPUART_HIGH_INDEX)
    {
        LPUART_DRV_GetBaudRate((uint32_t)instance, configuredBaudRate);
    }
    #endif

    /* Define UART PAL over FLEXIO */
    #if defined(UART_OVER_FLEXIO)
    if ((instance >= FLEXIO_UART_LOW_INDEX) && (instance <= FLEXIO_UART_HIGH_INDEX))
    {
        status = FLEXIO_UART_DRV_GetBaudRate(&(flexioUartTxState[UART_FindFlexioState(instance)]),
                                             configuredBaudRate);
    }
    #endif

    /* Define UART PAL over LinFlexD */
    #if (defined(UART_OVER_LINFLEXD))
    status = LINFLEXD_UART_DRV_GetBaudRate(instance, configuredBaudRate);
    #endif

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_SendDataBlocking
 * Description   : Perform a blocking UART transmission
 *
 * Implements    : UART_SendDataBlocking_Activity
 *END**************************************************************************/
status_t UART_SendDataBlocking(
        uart_instance_t instance,
        const uint8_t * txBuff,
        uint32_t txSize,
        uint32_t timeout)
{
    status_t status = STATUS_SUCCESS;

    /* Define UART PAL over LPUART */
    #if defined(UART_OVER_LPUART)
    if (instance <= LPUART_HIGH_INDEX)
    {
        status = LPUART_DRV_SendDataBlocking((uint32_t)instance, txBuff, txSize, timeout);
    }
    #endif

    /* Define UART PAL over FLEXIO */
    #if defined(UART_OVER_FLEXIO)
    if ((instance >= FLEXIO_UART_LOW_INDEX) && (instance <= FLEXIO_UART_HIGH_INDEX))
    {
        status = FLEXIO_UART_DRV_SendDataBlocking(
                        &(flexioUartTxState[UART_FindFlexioState(instance)]),
                        txBuff,
                        txSize,
                        timeout);
    }
    #endif

    /* Define UART PAL over LinFlexD */
    #if (defined(UART_OVER_LINFLEXD))
    status = LINFLEXD_UART_DRV_SendDataBlocking(instance, txBuff, txSize, timeout);
    #endif
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_SendData
 * Description   : Perform a non-blocking UART transmission
 *
 * Implements    : UART_SendData_Activity
 *END**************************************************************************/
status_t UART_SendData(uart_instance_t instance, const uint8_t * txBuff, uint32_t txSize)
{
    status_t status = STATUS_SUCCESS;

    /* Define UART PAL over LPUART */
    #if defined(UART_OVER_LPUART)

    if (instance <= LPUART_HIGH_INDEX)
    {
        status = LPUART_DRV_SendData((uint32_t)instance, txBuff, txSize);
    }
    #endif

    /* Define UART PAL over FLEXIO */
    #if defined(UART_OVER_FLEXIO)
    if ((instance >= FLEXIO_UART_LOW_INDEX) && (instance <= FLEXIO_UART_HIGH_INDEX))
    {
        status = FLEXIO_UART_DRV_SendData(
                        &(flexioUartTxState[UART_FindFlexioState(instance)]),
                        txBuff,
                        txSize);
    }
    #endif

    /* Define UART PAL over LinFlexD */
    #if (defined(UART_OVER_LINFLEXD))
    status = LINFLEXD_UART_DRV_SendData(instance, txBuff, txSize);
    #endif
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_AbortSendingData
 * Description   : This function terminates an non-blocking transmission early.
 *
 * Implements    : UART_AbortSendingData_Activity
 *END**************************************************************************/
status_t UART_AbortSendingData(uart_instance_t instance)
{
    status_t status = STATUS_SUCCESS;

    /* Define UART PAL over LPUART */
    #if defined(UART_OVER_LPUART)
    if (instance <= LPUART_HIGH_INDEX)
    {
        status = LPUART_DRV_AbortSendingData(instance);
    }
    #endif

    /* Define UART PAL over FLEXIO */
    #if defined(UART_OVER_FLEXIO)
    if ((instance >= FLEXIO_UART_LOW_INDEX) && (instance <= FLEXIO_UART_HIGH_INDEX))
    {
        status = FLEXIO_UART_DRV_TransferAbort(&(flexioUartTxState[UART_FindFlexioState(instance)]));
    }
    #endif

    /* Define UART PAL over LinFlexD */
    #if (defined(UART_OVER_LINFLEXD))
    status = LINFLEXD_UART_DRV_AbortSendingData(instance);
    #endif
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_GetTransmitStatus
 * Description   : This function returns whether the previous transmission has
 * finished
 *
 * Implements    : UART_GetTransmitStatus_Activity
 *END**************************************************************************/
status_t UART_GetTransmitStatus(uart_instance_t instance, uint32_t * bytesRemaining)
{
    status_t status = STATUS_SUCCESS;

    /* Define UART PAL over LPUART */
    #if defined(UART_OVER_LPUART)
    if (instance <= LPUART_HIGH_INDEX)
    {
        status = LPUART_DRV_GetTransmitStatus(instance, bytesRemaining);
    }
    #endif

    /* Define UART PAL over FLEXIO */
    #if defined(UART_OVER_FLEXIO)
    if ((instance >= FLEXIO_UART_LOW_INDEX) && (instance <= FLEXIO_UART_HIGH_INDEX))
    {
        status = FLEXIO_UART_DRV_GetStatus(
                        &(flexioUartTxState[UART_FindFlexioState(instance)]),
                        bytesRemaining);
    }
    #endif

    /* Define UART PAL over LinFlexD */
    #if (defined(UART_OVER_LINFLEXD))
    status = LINFLEXD_UART_DRV_GetTransmitStatus(instance, bytesRemaining);
    #endif
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_ReceiveDataBlocking
 * Description   : Perform a blocking UART reception
 *
 * Implements    : UART_ReceiveDataBlocking_Activity
 *END**************************************************************************/
status_t UART_ReceiveDataBlocking(
        uart_instance_t instance,
        uint8_t * rxBuff,
        uint32_t rxSize,
        uint32_t timeout)
{
    status_t status = STATUS_SUCCESS;

    /* Define UART PAL over LPUART */
    #if defined(UART_OVER_LPUART)
    if (instance <= LPUART_HIGH_INDEX)
    {
        status = LPUART_DRV_ReceiveDataBlocking((uint32_t)instance, rxBuff, rxSize, timeout);
    }
    #endif

    /* Define UART PAL over FLEXIO */
    #if defined(UART_OVER_FLEXIO)
    if ((instance >= FLEXIO_UART_LOW_INDEX) && (instance <= FLEXIO_UART_HIGH_INDEX))
    {
        status = FLEXIO_UART_DRV_ReceiveDataBlocking(
                        &(flexioUartRxState[UART_FindFlexioState(instance)]),
                        rxBuff,
                        rxSize,
                        timeout);
    }
    #endif

    /* Define UART PAL over LinFlexD */
    #if (defined(UART_OVER_LINFLEXD))
    status = LINFLEXD_UART_DRV_ReceiveDataBlocking(instance, rxBuff, rxSize, timeout);
    #endif
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_ReceiveData
 * Description   : Perform a non-blocking UART reception
 *
 * Implements    : UART_ReceiveData_Activity
 *END**************************************************************************/
status_t UART_ReceiveData(uart_instance_t instance, uint8_t * rxBuff, uint32_t rxSize)
{
    status_t status = STATUS_SUCCESS;

    /* Define UART PAL over LPUART */
    #if defined(UART_OVER_LPUART)
    if (instance <= LPUART_HIGH_INDEX)
    {
        status = LPUART_DRV_ReceiveData((uint32_t)instance, rxBuff, rxSize);
    }
    #endif

    /* Define UART PAL over FLEXIO */
    #if defined(UART_OVER_FLEXIO)
    if ((instance >= FLEXIO_UART_LOW_INDEX) && (instance <= FLEXIO_UART_HIGH_INDEX))
    {
        status = FLEXIO_UART_DRV_ReceiveData(
                        &(flexioUartRxState[UART_FindFlexioState(instance)]),
                        rxBuff,
                        rxSize);
    }
    #endif

    /* Define UART PAL over LinFlexD */
    #if (defined(UART_OVER_LINFLEXD))
    status = LINFLEXD_UART_DRV_ReceiveData(instance, rxBuff, rxSize);
    #endif
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_AbortReceivingData
 * Description   : Terminates a non-blocking receive early.
 *
 * Implements    : UART_AbortReceivingData_Activity
 *END**************************************************************************/
status_t UART_AbortReceivingData(uart_instance_t instance)
{
    status_t status = STATUS_SUCCESS;

    /* Define UART PAL over LPUART */
    #if defined(UART_OVER_LPUART)
    if (instance <= LPUART_HIGH_INDEX)
    {
        status = LPUART_DRV_AbortReceivingData(instance);
    }
    #endif

    /* Define UART PAL over FLEXIO */
    #if defined(UART_OVER_FLEXIO)
    if ((instance >= FLEXIO_UART_LOW_INDEX) && (instance <= FLEXIO_UART_HIGH_INDEX))
    {
        status = FLEXIO_UART_DRV_TransferAbort(&(flexioUartRxState[UART_FindFlexioState(instance)]));
    }
    #endif

    /* Define UART PAL over LinFlexD */
    #if (defined(UART_OVER_LINFLEXD))
    status = LINFLEXD_UART_DRV_AbortReceivingData(instance);
    #endif
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_GetReceiveStatus
 * Description   : This function returns whether the previous reception has
 * finished
 *
 * Implements    : UART_GetReceiveStatus_Activity
 *END**************************************************************************/
status_t UART_GetReceiveStatus(uart_instance_t instance, uint32_t * bytesRemaining)
{
    status_t status = STATUS_SUCCESS;

    /* Define UART PAL over LPUART */
    #if defined(UART_OVER_LPUART)
    if (instance <= LPUART_HIGH_INDEX)
    {
        status = LPUART_DRV_GetReceiveStatus(instance, bytesRemaining);
    }
    #endif

    /* Define UART PAL over FLEXIO */
    #if defined(UART_OVER_FLEXIO)
    if ((instance >= FLEXIO_UART_LOW_INDEX) && (instance <= FLEXIO_UART_HIGH_INDEX))
    {
        status = FLEXIO_UART_DRV_GetStatus(&(flexioUartRxState[UART_FindFlexioState(instance)]),
                                           bytesRemaining);
    }
    #endif

    /* Define UART PAL over LinFlexD */
    #if (defined(UART_OVER_LINFLEXD))
    status = LINFLEXD_UART_DRV_GetReceiveStatus(instance, bytesRemaining);
    #endif
    return status;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
