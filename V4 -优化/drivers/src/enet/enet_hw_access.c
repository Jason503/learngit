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

#include "enet_hw_access.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief MDC frequency. */
#define FEATURE_ENET_MDC_MAX_FREQUENCY 2500000U
/*! @brief NanoSecond in one second. */
#define ENET_NANOSECOND_ONE_SECOND     1000000000U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void ENET_SetTxBufferDescriptors(enet_buffer_descriptor_t *txBdStartAlign,
                                        uint32_t txRingSize);

static void ENET_SetRxBufferDescriptors(enet_buffer_descriptor_t *rxBdStartAlign,
                                        uint32_t rxRingSize,
                                        uint8_t *rxBuffStartAlign,
                                        uint16_t buffSize);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to enet handles for each instance. */
extern enet_state_t *g_enetState[ENET_INSTANCE_COUNT];

/*******************************************************************************
 * Private functions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : ENET_SetTxBufferDescriptors
 * Description   : Configures the software transmit buffer descriptors.
 *
 *END**************************************************************************/
static void ENET_SetTxBufferDescriptors(enet_buffer_descriptor_t *txBdStartAlign,
                                        uint32_t txRingSize)
{
    uint32_t count;
    enet_buffer_descriptor_t *curBuffDescrip = txBdStartAlign;

    for (count = 0; count < txRingSize; count++)
    {
        curBuffDescrip->control = ENET_BUFFDESCR_TX_TRANSMITCRC_MASK;
        /* Sets the last buffer descriptor with the wrap flag. */
        if (count == txRingSize - 1)
        {
            curBuffDescrip->control |= ENET_BUFFDESCR_TX_WRAP_MASK;
        }

        /* Increase the index. */
        curBuffDescrip++;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ENET_SetRxBufferDescriptors
 * Description   : Configures the software receive buffer descriptors.
 *
 *END**************************************************************************/
static void ENET_SetRxBufferDescriptors(enet_buffer_descriptor_t *rxBdStartAlign,
                                        uint32_t rxRingSize,
                                        uint8_t *rxBuffStartAlign,
                                        uint16_t buffSize)
{
    enet_buffer_descriptor_t *curBuffDescrip = rxBdStartAlign;
    uint32_t count = 0;

    /* Initializes receive buffer descriptors. */
    for (count = 0; count < rxRingSize; count++)
    {
        /* Set data buffer and the length. */
        curBuffDescrip->buffer = (uint8_t *)((uint32_t)&rxBuffStartAlign[count * buffSize]);
        curBuffDescrip->length = 0;

        /* Initializes the buffer descriptors with empty bit. */
        curBuffDescrip->control = ENET_BUFFDESCR_RX_EMPTY_MASK;
        /* Sets the last buffer descriptor with the wrap flag. */
        if (count == rxRingSize - 1)
        {
            curBuffDescrip->control |= ENET_BUFFDESCR_RX_WRAP_MASK;
        }

        /* Increase the index. */
        curBuffDescrip++;
    }
}

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : ENET_ConfigBufferDescriptors
 * Description   : Configures the receive and transmit buffer descriptors.
 *
 *END**************************************************************************/
void ENET_ConfigBufferDescriptors(ENET_Type * base,
                                  const enet_buffer_config_t *bufferConfig,
                                  uint16_t buffSize)
{
    /* Initializes the ENET transmit buffer descriptors. */
    ENET_SetTxBufferDescriptors(bufferConfig->txRingAligned, bufferConfig->txRingSize);

    /* Initializes the ENET receive buffer descriptors. */
    ENET_SetRxBufferDescriptors(bufferConfig->rxRingAligned, bufferConfig->rxRingSize,
                                bufferConfig->rxBufferAligned, buffSize);

    /* Initializes transmit buffer descriptor rings start address, two start address should be aligned. */
    base->TDSR = (uint32_t)bufferConfig->txRingAligned;
    base->RDSR = (uint32_t)bufferConfig->rxRingAligned;
    /* Initializes the maximum buffer size, the buffer size should be aligned. */
    base->MRBR = buffSize;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ENET_ConfigReceiveControl
 * Description   : Configures the receive block.
 *
 *END**************************************************************************/
void ENET_ConfigReceiveControl(ENET_Type *base,
                               const enet_config_t *config)
{
    uint32_t rcr = 0;

    /* Configures MAC receive controller with user configure structure. */
    rcr = ENET_RCR_NLC(!!(config->rxConfig & ENET_RX_CONFIG_ENABLE_PAYLOAD_LEN_CHECK)) |
          ENET_RCR_CFEN(!!(config->rxConfig & ENET_RX_CONFIG_ENABLE_FLOW_CONTROL)) |
          ENET_RCR_CRCFWD(!!(config->rxConfig & ENET_RX_CONFIG_STRIP_CRC_FIELD)) |
          ENET_RCR_FCE(!!(config->rxConfig & ENET_RX_CONFIG_ENABLE_FLOW_CONTROL)) |
          ENET_RCR_PAUFWD(!!(config->rxConfig & ENET_RX_CONFIG_FORWARD_PAUSE_FRAMES)) |
          ENET_RCR_PADEN(!!(config->rxConfig & ENET_RX_CONFIG_REMOVE_PADDING)) |
          ENET_RCR_BC_REJ(!!(config->rxConfig & ENET_RX_CONFIG_REJECT_BROADCAST_FRAMES)) |
          ENET_RCR_PROM(!!(config->rxConfig & ENET_RX_CONFIG_ENABLE_PROMISCUOUS_MODE)) |
          ENET_RCR_MII_MODE(1U) |
          ENET_RCR_RMII_MODE(config->miiMode) |
          ENET_RCR_RMII_10T(!config->miiSpeed) |
          ENET_RCR_MAX_FL(config->maxFrameLen);

    /* Receive setting for half duplex. */
    if (config->miiDuplex == ENET_MII_HALF_DUPLEX)
    {
        rcr |= ENET_RCR_DRT(1);
    }
    /* Sets internal loop only for MII mode. */
    if ((config->rxConfig & ENET_RX_CONFIG_ENABLE_MII_LOOPBACK) && (config->miiMode == ENET_MII_MODE))
    {
        rcr |= ENET_RCR_LOOP(1);
        rcr &= ~ENET_RCR_DRT_MASK;
    }
    base->RCR = rcr;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ENET_ConfigTransmitControl
 * Description   : Configures the transmit block.
 *
 *END**************************************************************************/
void ENET_ConfigTransmitControl(ENET_Type *base,
                                const enet_config_t *config)
{
    uint32_t tcr = base->TCR;

    /* Configures MAC transmit controller: duplex mode, mac address insertion. */
    tcr &= ~(ENET_TCR_FDEN_MASK | ENET_TCR_CRCFWD_MASK | ENET_TCR_ADDINS_MASK);
    tcr |= ENET_TCR_FDEN(config->miiDuplex) |
           ENET_TCR_CRCFWD(!!(config->txConfig & ENET_TX_CONFIG_DISABLE_CRC_APPEND)) |
           ENET_TCR_ADDINS(!!(config->txConfig & ENET_TX_CONFIG_ENABLE_MAC_ADDR_INSERTION));

    base->TCR = tcr;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ENET_TransmitIRQHandler
 * Description   : Handler for ENET transmission interrupts.
 * This handler clears the interrupt flags and invokes the installed callback,
 * if available.
 *
 *END**************************************************************************/
void ENET_TransmitIRQHandler(uint8_t instance)
{
    ENET_Type *base = s_enetBases[instance];

    /* Check if the transmit interrupt happen. */
    if ((ENET_TX_BUFFER_INTERRUPT | ENET_TX_FRAME_INTERRUPT) & base->EIR)
    {
        /* Clear the transmit interrupt event. */
        base->EIR = ENET_TX_FRAME_INTERRUPT | ENET_TX_BUFFER_INTERRUPT;
        /* Callback function. */
        if (g_enetState[instance]->callback)
        {
            g_enetState[instance]->callback(instance, ENET_TX_EVENT);
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ENET_ReceiveIRQHandler
 * Description   : Handler for ENET reception interrupts.
 * This handler clears the interrupt flags, extracts the received frame and
 * invokes the installed callback, if available. After the callback completes,
 * the buffers will be marked as empty.
 *
 *END**************************************************************************/
void ENET_ReceiveIRQHandler(uint8_t instance)
{
    ENET_Type *base = s_enetBases[instance];

    /* Check if the receive interrupt happen. */
    if ((ENET_RX_BUFFER_INTERRUPT | ENET_RX_FRAME_INTERRUPT) & base->EIR)
    {
        /* Clear the transmit interrupt event. */
        base->EIR = ENET_RX_FRAME_INTERRUPT | ENET_RX_BUFFER_INTERRUPT;

        /* Callback function. */
        if (g_enetState[instance]->callback)
        {
            g_enetState[instance]->callback(instance, ENET_RX_EVENT);
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ENET_ErrorIRQHandler
 * Description   : Handler for ENET error interrupts.
 * This handler clears the interrupt flags and invokes the installed callback,
 * if available.
 *
 *END**************************************************************************/
void ENET_ErrorIRQHandler(uint8_t instance)
{
    ENET_Type *base = s_enetBases[instance];

    uint32_t errMask = ENET_BABR_INTERRUPT | ENET_BABT_INTERRUPT | ENET_EBERR_INTERRUPT | ENET_PAYLOAD_RX_INTERRUPT |
                       ENET_LATE_COLLISION_INTERRUPT | ENET_RETRY_LIMIT_INTERRUPT | ENET_UNDERRUN_INTERRUPT;

    if (errMask & base->EIR)
    {
        errMask &= base->EIR;
        base->EIR = errMask;
        /* Callback function. */
        if (g_enetState[instance]->callback)
        {
            g_enetState[instance]->callback(instance, ENET_ERR_EVENT);
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ENET_WakeIRQHandler
 * Description   : Handler for ENET wakeup interrupts.
 * This handler clears the interrupt flags and invokes the installed callback,
 * if available. Also, the ENET module is set to normal operation mode.
 *
 *END**************************************************************************/
void ENET_WakeIRQHandler(uint8_t instance)
{
    ENET_Type *base = s_enetBases[instance];

    if (ENET_WAKEUP_INTERRUPT & base->EIR)
    {
        /* Clear the wakeup interrupt. */
        base->EIR = ENET_WAKEUP_INTERRUPT;
        ENET_DRV_SetSleepMode(instance, false);
        /* Callback function. */
        if (g_enetState[instance]->callback)
        {
            g_enetState[instance]->callback(instance, ENET_WAKE_UP_EVENT);
        }
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
