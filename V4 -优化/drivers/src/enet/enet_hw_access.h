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

#ifndef ENET_HW_ACCESS_H
#define ENET_HW_ACCESS_H

#include "enet_driver.h"
#include "device_registers.h"
#include <stdlib.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define ENET_MMFR_START_INDICATION  (1U)
#define ENET_MMFR_TURN_AROUND       (2U)

/*! @brief Control and status region bit masks of the receive buffer descriptor. */
#define ENET_BUFFDESCR_RX_EMPTY_MASK 0x8000U
#define ENET_BUFFDESCR_RX_WRAP_MASK  0x2000U
#define ENET_BUFFDESCR_RX_LAST_MASK  0x0800U

/*! @brief Control and status bit masks of the transmit buffer descriptor. */
#define ENET_BUFFDESCR_TX_READY_MASK        0x8000U
#define ENET_BUFFDESCR_TX_WRAP_MASK         0x2000U
#define ENET_BUFFDESCR_TX_LAST_MASK         0x0800U
#define ENET_BUFFDESCR_TX_TRANSMITCRC_MASK  0x0400U

/*! @brief Masks for the bits in the CRC-32 value */
#define ENET_CRC32_BIT_1_MASK       0x80000000U
#define ENET_CRC32_BITS_2_6_MASK    0x7C000000U
#define ENET_CRC32_BITS_2_6_SHIFT   26U

/*!
 * @brief Management Frame operation type
 */
typedef enum
{
    ENET_MMFR_OP_WRITE = 1U,
    ENET_MMFR_OP_READ
} enet_mmfr_op_type_t;

/*! @brief Pointers to ENET bases for each instance. */
static ENET_Type *const s_enetBases[] = ENET_BASE_PTRS;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Configures the receive and transmit buffer descriptors.
 *
 * @param[in] base The base address of the module
 * @param[in] bufferConfig The configuration used for the receive and transmit descriptors
 */
void ENET_ConfigBufferDescriptors(ENET_Type * base,
                                  const enet_buffer_config_t *bufferConfig,
                                  uint16_t buffSize);

/*!
 * @brief Configures the receive block.
 *
 * @param[in] base The base address of the module
 * @param[in] config The user configuration structure
 */
void ENET_ConfigReceiveControl(ENET_Type *base,
                               const enet_config_t *config);

/*!
 * @brief Configures the transmit block.
 *
 * @param[in] base The base address of the module
 * @param[in] config The user configuration structure
 */
void ENET_ConfigTransmitControl(ENET_Type *base,
                                const enet_config_t *config);

/*!
 * @brief Resets the ENET module.
 *
 * @param[in] base The base address of the module
 */
static inline void ENET_Reset(ENET_Type *base)
{
    base->ECR |= ENET_ECR_RESET_MASK;
#if (defined(CORE_LITTLE_ENDIAN))
    base->ECR |= ENET_ECR_DBSWP_MASK;
#endif
}

/*!
 * @brief Indicates that the driver produced empty receive buffers with the empty bit set.
 *
 * @param[in] base The base address of the module
 */
static inline void ENET_ActivateReceive(ENET_Type *base)
{
    base->RDAR = ENET_RDAR_RDAR_MASK;
}

/*!
 * @brief Indicates that the driver produced empty transmit buffers with the ready bit set.
 *
 * @param[in] base The base address of the module
 */
static inline void ENET_ActivateTransmit(ENET_Type *base)
{
#ifdef ERRATA_E7885
    uint8_t i;
    bool tdarTrigger = false;

    for (i = 0; i < 4U; i++)
    {
        if ((base->TDAR & ENET_TDAR_TDAR_MASK) == 0)
        {
            tdarTrigger = true;
            break;
        }
    }
    if (tdarTrigger)
    {
#endif
    base->TDAR = ENET_TDAR_TDAR_MASK;
#ifdef ERRATA_E7885
    }
#endif
}

/*!
 * @brief Enables the specified interrupts.
 *
 * @param[in] base The base address of the module
 * @param[in] mask Mask representing the interrupts to be enabled
 */
static inline void ENET_EnableInterrupts(ENET_Type *base,
                                         uint32_t mask)
{
    base->EIMR |= mask;
}

/*!
 * @brief Disables the specified interrupts.
 *
 * @param[in] base The base address of the module
 * @param[in] mask Mask representing the interrupts to be disabled
 */
static inline void ENET_DisableInterrupts(ENET_Type *base,
                                          uint32_t mask)
{
    base->EIMR &= ~mask;
}

/*!
 * @brief Gets a mask of the interrupts flags which are set.
 *
 * @param[in] base The base address of the module
 * @return Mask representing the interrupt flags set
 */
static inline uint32_t ENET_GetInterruptStatus(ENET_Type *base)
{
    return base->EIR;
}

/*!
 * @brief Clears the interrupts flags according to the received mask.
 *
 * @param[in] base The base address of the module
 * @param[in] mask Mask representing the interrupt flags to be cleared
 */
static inline void ENET_ClearInterruptStatus(ENET_Type *base,
                                             uint32_t mask)
{
    base->EIR = mask;
}

/*!
 * @brief Enables the ENET module.
 *
 * @param[in] base The base address of the module
 */
static inline void ENET_Enable(ENET_Type *base)
{
    base->ECR |= ENET_ECR_ETHEREN_MASK;
}

/*!
 * @brief Disables the ENET module.
 *
 * @param[in] base The base address of the module
 */
static inline void ENET_Disable(ENET_Type *base)
{
    base->ECR &= ~ENET_ECR_ETHEREN_MASK;
}

/*!
 * @brief Configures transmit accelerator functions.
 *
 * @param[in] base The base address of the module
 * @param[in] txAccelerConfig Transmit accelerator functions to be applied
 */
static inline void ENET_ConfigTransmitAccelerator(ENET_Type * base,
                                                  uint8_t txAccelerConfig)
{
    base->TACC = txAccelerConfig;

    /* Enable store and forward when accelerator is enabled */
    if (txAccelerConfig & (ENET_TX_ACCEL_INSERT_IP_CHECKSUM | ENET_TX_ACCEL_INSERT_PROTO_CHECKSUM))
    {
        base->TFWR = ENET_TFWR_STRFWD_MASK;
    }
}

/*!
 * @brief Configures receive accelerator functions.
 *
 * @param[in] base The base address of the module
 * @param[in] rxAccelerConfig Receive accelerator functions to be applied
 */
static inline void ENET_ConfigReceiveAccelerator(ENET_Type * base,
                                                 uint8_t rxAccelerConfig)
{
    base->RACC = rxAccelerConfig;

    /* Enable store and forward when accelerator is enabled */
    if (rxAccelerConfig & (ENET_RX_ACCEL_ENABLE_IP_CHECK | ENET_RX_ACCEL_ENABLE_PROTO_CHECK))
    {
        base->RSFL = 0;
    }
}

/*!
 * @brief Sets the lower 32 bits of the MAC physical address.
 *
 * @param[in] base The base address of the module
 * @param[in] addrLower The lower 32 bits of the MAC physical address
 */
static inline void ENET_SetPhyAddrLower(ENET_Type * base,
                                        uint32_t addrLower)
{
    base->PALR = addrLower;
}

/*!
 * @brief Sets the upper 16 bits of the MAC physical address.
 *
 * @param[in] base The base address of the module
 * @param[in] addrUpper The upper 16 bits of the MAC physical address
 */
static inline void ENET_SetPhyAddrUpper(ENET_Type * base,
                                        uint32_t addrUpper)
{
    base->PAUR = addrUpper << ENET_PAUR_PADDR2_SHIFT;
}

/*!
 * @brief Gets the lower 32 bits of the MAC physical address.
 *
 * @param[in] base The base address of the module
 * @return The lower 32 bits of the MAC physical address
 */
static inline uint32_t ENET_GetPhyAddrLower(ENET_Type * base)
{
    return base->PALR;
}

/*!
 * @brief Gets the upper 16 bits of the MAC physical address.
 *
 * @param[in] base The base address of the module
 * @return The upper 16 bits of the MAC physical address
 */
static inline uint32_t ENET_GetPhyAddrUpper(ENET_Type * base)
{
    return (base->PAUR & ENET_PAUR_PADDR2_MASK) >> ENET_PAUR_PADDR2_SHIFT;
}

/*!
 * @brief Writes a MII management frame.
 *
 * @param[in] base The base address of the module
 * @param[in] phyAddr The address of the PHY
 * @param[in] phyReg The address of the register to be accessed
 * @param[in] opType The operation type (read/write)
 * @param[in] data Data to be written, ignored if the operation is a read one
 */
static inline void ENET_WriteManagementFrame(ENET_Type * base,
                                             uint8_t phyAddr,
                                             uint8_t phyReg,
                                             enet_mmfr_op_type_t opType,
                                             uint16_t data)
{
    base->MMFR = ENET_MMFR_ST(ENET_MMFR_START_INDICATION) |
                              ENET_MMFR_OP(opType) |
                              ENET_MMFR_PA(phyAddr) |
                              ENET_MMFR_RA(phyReg) |
                              ENET_MMFR_TA(ENET_MMFR_TURN_AROUND) |
                              ENET_MMFR_DATA(data);
}

/*!
 * @brief Reads the data field of a MII management frame.
 *
 * @param[in] base The base address of the module
 * @return The read data
 */
static inline uint16_t ENET_ReadManagementFrameData(ENET_Type * base)
{
    return (uint16_t)(base->MMFR & ENET_MMFR_DATA_MASK);
}

/*!
 * @brief Configures the MII management interface.
 *
 * @param[in] base The base address of the module
 * @param[in] config The configuration to be applied
 */
static inline void ENET_WriteManagementConfig(ENET_Type * base,
                                              uint32_t config)
{
    base->MSCR = config;
}


/*!
 * @brief Adds an address to the hash table used in the address recognition
 * process for receive frames with multicast destination address.
 *
 * @param[in] base The base address of the module
 * @param[in] crc The CRC-32 of the MAC address
 */
static inline void ENET_AddToGroupHashTable(ENET_Type * base,
                                            uint32_t crc)
{
    if ((crc & ENET_CRC32_BIT_1_MASK) != 0U)
    {
        base->GAUR |= (1U << ((crc & ENET_CRC32_BITS_2_6_MASK) >> ENET_CRC32_BITS_2_6_SHIFT));
    }
    else
    {
        base->GALR |= (1U << ((crc & ENET_CRC32_BITS_2_6_MASK) >> ENET_CRC32_BITS_2_6_SHIFT));
    }
}

/*!
 * @brief Removes an address from the hash table used in the address recognition
 * process for receive frames with multicast destination address.
 *
 * @param[in] base The base address of the module
 * @param[in] crc The CRC-32 of the MAC address
 */
static inline void ENET_RemoveFromGroupHashTable(ENET_Type * base,
                                                 uint32_t crc)
{
    if ((crc & ENET_CRC32_BIT_1_MASK) != 0U)
    {
        base->GAUR &= ~(1U << ((crc & ENET_CRC32_BITS_2_6_MASK) >> ENET_CRC32_BITS_2_6_SHIFT));
    }
    else
    {
        base->GALR &= ~(1U << ((crc & ENET_CRC32_BITS_2_6_MASK) >> ENET_CRC32_BITS_2_6_SHIFT));
    }
}

/*!
 * @brief Adds an address to the hash table used in the address recognition
 * process for receive frames with unicast destination address.
 *
 * @param[in] base The base address of the module
 * @param[in] crc The CRC-32 of the MAC address
 */
static inline void ENET_AddToIndividualHashTable(ENET_Type * base,
                                                 uint32_t crc)
{
    if ((crc & ENET_CRC32_BIT_1_MASK) != 0U)
    {
        base->IAUR |= (1U << ((crc & ENET_CRC32_BITS_2_6_MASK) >> ENET_CRC32_BITS_2_6_SHIFT));
    }
    else
    {
        base->IALR |= (1U << ((crc & ENET_CRC32_BITS_2_6_MASK) >> ENET_CRC32_BITS_2_6_SHIFT));
    }
}

/*!
 * @brief Removes an address from the hash table used in the address recognition
 * process for receive frames with unicast destination address.
 *
 * @param[in] base The base address of the module
 * @param[in] crc The CRC-32 of the MAC address
 */
static inline void ENET_RemoveFromIndividualHashTable(ENET_Type * base,
                                                      uint32_t crc)
{
    if ((crc & ENET_CRC32_BIT_1_MASK) != 0U)
    {
        base->IAUR &= ~(1U << ((crc & ENET_CRC32_BITS_2_6_MASK) >> ENET_CRC32_BITS_2_6_SHIFT));
    }
    else
    {
        base->IALR &= ~(1U << ((crc & ENET_CRC32_BITS_2_6_MASK) >> ENET_CRC32_BITS_2_6_SHIFT));
    }
}

/*!
 * @brief Handler for ENET transmission interrupts.
 *
 * This handler clears the interrupt flags and invokes the installed callback,
 * if available.
 *
 * @param[in] instance Instance number
 */
void ENET_TransmitIRQHandler(uint8_t instance);

/*!
 * @brief Handler for ENET reception interrupts.
 *
 * This handler clears the interrupt flags, extracts the received frame and
 * invokes the installed callback, if available. After the callback completes,
 * the buffers will be marked as empty.
 *
 * @param[in] instance Instance number
 */
void ENET_ReceiveIRQHandler(uint8_t instance);

/*!
 * @brief Handler for ENET error interrupts.
 *
 * This handler clears the interrupt flags and invokes the installed callback,
 * if available.
 *
 * @param[in] instance Instance number
 */
void ENET_ErrorIRQHandler(uint8_t instance);

/*!
 * @brief Handler for ENET wakeup interrupts.
 *
 * This handler clears the interrupt flags and invokes the installed callback,
 * if available. Also, the ENET module is set to normal operation mode.
 *
 * @param[in] instance Instance number
 */
void ENET_WakeIRQHandler(uint8_t instance);

#if defined(__cplusplus)
}
#endif

#endif  /* ENET_HW_ACCESS_H */

/*******************************************************************************
 * EOF
 ******************************************************************************/
