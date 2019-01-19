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
/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced
 * These macro are used by user.
 */

#ifndef SAI_DRIVER_H_
#define SAI_DRIVER_H_

#include <stddef.h>
#include <stdbool.h>
#include "device_registers.h"
#include "clock_manager.h"
#include "interrupt_manager.h"
#include "edma_driver.h"
#include "osif.h"
#include "sai_hw_access.h"

/*!
 * @addtogroup sai_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Bit values to use with ChannelEnable param in user config struct */
#define SAI_CHANNEL_0 0x1
#define SAI_CHANNEL_1 0x2
#define SAI_CHANNEL_2 0x4
#define SAI_CHANNEL_3 0x8

/*! @brief Transmit or receive state.
 */
typedef struct
{
/*! @cond DRIVER_INTERNAL_USE_ONLY */
    uint32_t count;
    uint8_t *data;
/*! @endcond */
} sai_xfer_state_t;

/*! @brief Report to enable.
 */
typedef enum
{
    SAI_FRAME_START = 0,        /*!< Indicate a frame start */
    SAI_RUN_ERROR,              /*!< Overrun/underrun error */
    SAI_SYNC_ERROR,             /*!< Frame sync error */
    SAI_TRANSFER_COMPLETE       /*!< Non blocking transfer completed */
} sai_report_type_t;

/*! @brief Transfer type.
 */
typedef enum
{
    SAI_INTERRUPT = 0U,     /*!< Transfer type is interrupt */
    SAI_DMA                 /*!< Transfer type is DMA */
} sai_transfer_type_t;

/*! @brief Data mux line or mux memory.
 */
typedef enum
{
    SAI_MUX_DISABLED = 0U,  /*!< Each data line is a channel, uses a seperate memory block */
    SAI_MUX_LINE = 1U,      /*!< Words on data line is alternated between channels, each channel data is a seperate memory block */
    SAI_MUX_MEM = 2U        /*!< Words in memory block is alternated between channels, each channel data is on a seperate data line.*/
} sai_mux_mode_t;

/*! @brief Sai callback function type for nonblock transfer, also called to report events (sai_report_type_t).
 */
typedef void (*sai_transfer_callback_t)(uint8_t channel, sai_report_type_t report, status_t status);

/*! @brief Structure for internal use.
 *  This structure is used by the driver for its internal logic.
 *  It must be provided by the application through the initialize functions,
 *  then it cannot be freed until the driver is de-initialized using Deinit functions.
 *  The application should make no assumptions about the content of this structure.
 */
typedef struct
{
/*! @cond DRIVER_INTERNAL_USE_ONLY */
    sai_xfer_state_t ChnState[SAI_MAX_CHANNEL_COUNT];   /* store data pointer and data count of each channel transfer*/
    status_t status;
    uint8_t DmaChannel[SAI_MAX_CHANNEL_COUNT];          /* dma channel for each sai channel */
    uint8_t ElementSize;                                /* element size used for transfer */
    bool Blocking;
    sai_transfer_type_t XferType;
    semaphore_t Sema;
    sai_mux_mode_t mux;                                 /* mux mode using */
    uint8_t ChannelCount;                               /* only used for interrupt mux line */
    uint8_t NextChn;                                    /* emulate hw combine mode, work around for asdk 4634 */
    sai_transfer_callback_t Callback;
/*! @endcond */
} sai_state_t;

/*! @brief SAI run in sync or async mode.
 */
typedef enum
{
    SAI_ASYNC        = 0U,      /*!< Independent clock */
    SAI_SYNC_WITH_OTHER = 1U    /*!< Bit clock and frame sync is taken from transmitter/receiver */
} sai_sync_mode_t;

/*! @brief Select master clock.
 */
typedef enum
{
    SAI_BUS_CLK = 0U,       /*!< Master clock is module bus clock */
    SAI_EXTERNAL_CLK = 1U,   /*!< Master clock is from external */
    SAI_SOSC_CLK = 2U       /*!< Master clock is system sosc clock */
} sai_master_clk_source_t;

/*! @brief Data line state for masked word, or if data line is disabled.
 */
typedef enum
{
    SAI_MASK_TRISTATE = 0U,     /*!< Line is in high z state */
    SAI_MASK_ZERO = 1U          /*!< Line is output zero */
} sai_mask_mode_t;

/*! @brief User config structure.
 *
 * Implements : sai_user_config_t_Class
 */
typedef struct
{
    sai_sync_mode_t SyncMode;               /*!< Sync mode.*/
    sai_master_clk_source_t MasterClkSrc;   /*!< Select master clock source.*/
    bool BitClkNegPolar;                    /*!< True if bit clock is negative polar*/
    bool BitClkInternal;                    /*!< True if bit clock is generated internally.*/
    uint16_t BitClkDiv;                     /*!< If bit clock is generated internally, it is divided from master clock by this.
                                                 User need to init this if master clock is external.*/
    uint8_t ChannelEnable;                  /*!< Turn on each bit to enable each channel. 4 bit for 4 channels.*/
    uint8_t FrameSize;                      /*!< Frame size in number of words.*/
    uint8_t SyncWidth;                      /*!< Sync width in number of bit clocks.*/
    sai_mask_mode_t MaskMode;               /*!< Data line state for mask word or when data line is disabled (apply only for transmitter).*/
    bool MsbFirst;                          /*!< True if data is MSB first, false if LSB first.*/
    bool SyncEarly;                         /*!< True if frame sync is one bit clock early.*/
    bool SyncNegPolar;                      /*!< True if frame sync is negative polar.*/
    bool SyncInternal;                      /*!< True if frame sync is generated internally*/
    uint8_t Word0Width;                     /*!< First word width in number of bit clocks.*/
    uint8_t WordNWidth;                     /*!< Other words width in number of bit clocks.*/
    uint8_t FirstBitIndex;                  /*!< Index from LSB of first bit to be transmitted/received, valid range
                                                 from 0-31.*/
    uint32_t BitClkFreq;                    /*!< Desired bit clock frequency in hertz,
                                                 only for internally generated master clock and bit clock.*/
    bool RunErrorReport;                    /*!< Underrun/overrun error report. */
    bool SyncErrorReport;                   /*!< Enable sync error report.*/
    bool FrameStartReport;                  /*!< Enable frame start report.*/
    sai_mux_mode_t MuxMode;                 /*!< Enable line mux, memory mux or mux is disabled. */
    sai_transfer_type_t TransferType;       /*!< Transfer using dma or interrupt.*/
    uint8_t DmaChannel[SAI_MAX_CHANNEL_COUNT];   /*!< DMA channels to be used.*/
    uint8_t ElementSize;                    /*!< Size in bytes of each element to transfer.*/
    uint8_t ChannelCount;                   /*!< Number of channels to enable, only used when line mux mode and interrupt mode is selected.*/
    sai_transfer_callback_t callback;       /*!< User callback function, called when transfer complete or selected events occured.*/

} sai_user_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name SAI Driver
 * @{
 */

/*!
 * @brief Initialize the transmitter of driver
 *
 * @param[in] instNum  Peripheral instance number
 * @param[in] saiUserConfig    Pointer to the user configuration structure. The function
 *                         reads configuration data from this structure and initializes the
 *                         driver accordingly. The application may free this structure after
 *                         the function returns.
 * @param[in] StateAlloc    Pointer to the state structure. The driver uses
 *                  this memory area for its internal logic. The application must make no
 *                  assumptions about the content of this structure, and must not free this
 *                  memory until the driver is de-initialized.
 */
void SAI_DRV_TxInit(uint32_t instNum,
                   const sai_user_config_t* saiUserConfig,
                   sai_state_t* StateAlloc);


/*!
 * @brief Initialize the receiver of driver
 *
 * @param[in] instNum  Peripheral instance number
 * @param[in] saiUserConfig    Pointer to the user configuration structure. The function
 *                         reads configuration data from this structure and initializes the
 *                         driver accordingly. The application may free this structure after
 *                         the function returns.
 * @param[in] StateAlloc    Pointer to the state structure. The driver uses
 *                  this memory area for its internal logic. The application must make no
 *                  assumptions about the content of this structure, and must not free this
 *                  memory until the driver is de-initialized.
 */
void SAI_DRV_RxInit(uint32_t instNum,
                    const sai_user_config_t* saiUserConfig,
                    sai_state_t* StateAlloc);


/*!
 * @brief De-initialize transmitter
 *
 * This function de-initializes driver. The driver can't be used
 * again until reinitialized. The context structure is no longer needed by the driver and
 * can be freed after calling this function.
 *
 * @param[in] instNum    Peripheral instance number
 */
void SAI_DRV_TxDeinit(uint32_t instNum);


/*!
 * @brief De-initialize receiver
 *
 * This function de-initializes driver. The driver can't be used
 * again until reinitialized. The context structure is no longer needed by the driver and
 * can be freed after calling this function.
 *
 * @param[in] instNum    Peripheral instance number
 */
void SAI_DRV_RxDeinit(uint32_t instNum);


/*!
 * @brief Return true bit clock frequency of transmitter
 *
 * Only used when master clock and bit clock is internal
 *
 * @param[in] instNum Peripheral instance number
 */
uint32_t SAI_DRV_TxGetBitClockFreq(uint32_t instNum);


/*!
 * @brief Return true bit clock frequency of receiver
 *
 * Only used when master clock and bit clock is internal
 *
 * @param[in] instNum Peripheral instance number
 * @return Frequency in hertz
 */
uint32_t SAI_DRV_RxGetBitClockFreq(uint32_t instNum);


/*!
 * @brief Return true bit clock divisor of transmitter
 *
 * Only used when bit clock is internal and master clock is external
 * @param[in] instNum Peripheral instance number
 * @return Frequency in hertz
 */
uint32_t SAI_DRV_TxGetBitClockDiv(uint32_t instNum);


/*!
 * @brief Return true bit clock divisor of receiver
 *
 * Only used when bit clock is internal and master clock is external
 * @param[in] instNum Peripheral instance number
 * @return Divisor factor
 */
uint32_t SAI_DRV_RxGetBitClockDiv(uint32_t instNum);


/*!
 * @brief Set masked word index of subsequent frames for transmitter
 *
 * Each bit is a masked word.
 * Should be called in frame start event callback or
 * in four bit clock cycles after Tx init.
 *
 * @param[in] instNum Peripheral instance number
 * @param[in] WordIndex Word index to mask
 * @return Divisor factor
 */
void SAI_DRV_TxSetNextMaskWords(uint32_t instNum, uint16_t Words);


/*!
 * @brief Set masked word index of subsequent frames for receiver
 *
 * Set masked words of subsequent frames. Each bit is a masked word.
 * Should be called in frame start event callback
 * or in four bit clock cycles after Rx init.
 *
 * @param[in] instNum Peripheral instance number
 * @param[in] WordIndex Word index to mask
 */
void SAI_DRV_RxSetNextMaskWords(uint32_t instNum, uint16_t Words);


/*!
 * @brief Send a block of data, return when transfer complete.
 *
 * Should be called immidiately after a transfer complete to avoid data underrun error.
 *
 * @param[in] instNum Peripheral instance number
 * @param[in] data Array of pointer to each data block to transfer, each data block corresponds to an enabled channels
 * If mux memory is selected, only first data block is used
 * @param[in] count Number of elements to transfer for each channel
 * @param[in] timeout Timeout to return when transfer take too long.
 * @return Success, error or timeout status.
 */
status_t SAI_DRV_SendBlocking(uint32_t instNum,
                              const uint8_t* data[],
                              uint32_t count,
                              uint32_t timeout);


/*!
 * @brief Send a block of data, return immidiately.
 *
 * When transfer completed, the callback function will be executed.
 * User should use this callback function to immidiately start an other transfer to avoid data underrun error.
 *
 * @param[in] instNum Peripheral instance number
 * @param[in] data Array of pointer to each data block to transfer, each data block corresponds to an enabled channels
 * If mux memory is selected, only first data block is used
 * @param[in] count Number of elements to transfer for each channel
 */
void SAI_DRV_Send(uint32_t instNum,
                  const uint8_t* data[],
                  uint32_t count);


/*!
 * @brief Get status of a non-blocking transfer.
 *
 * @param[in] instNum Peripheral instance number
 * @param[out] countRemain Number of elements remain for each channel.
 * This parameter can be NULL
 * @return Status of the transfer, can be success, aborted or busy. Note that aborted status can imply
 * a timed out blocking transfer, not only user abort.
 */
status_t SAI_DRV_GetSendingStatus(uint32_t instNum,
                                  uint32_t *countRemain);


/*!
 * @brief Abort an ongoing transfer
 *
 * @param[in] instNum Peripheral instance number
 */
void SAI_DRV_AbortSending(uint32_t instNum);


/*!
 * @brief Receive a block of data, return when transfer complete.
 *
 * Should be called immidiately after a transfer complete to avoid data overrun error.
 *
 * @param[in] instNum Peripheral instance number
 * @param[out] data Array of pointer to each data block to transfer, each data block corresponds to an enabled channels
 * If mux memory is selected, only first data block is used
 * @param[in] count Number of elements to transfer for each channel
 * @param[in] timeout Timeout to return when transfer take too long.
 * @return Success, error or timeout status.
 */
status_t SAI_DRV_ReceiveBlocking(uint32_t instNum,
                                 uint8_t* data[],
                                 uint32_t count,
                                 uint32_t timeout);


/*!
 * @brief Receive a block of data, return immidiately.
 *
 * When transfer completed, the callback function will be executed.
 * User should use this callback function to immidiately start another transfer to avoid data overrun error.
 *
 * @param[in] instNum Peripheral instance number
 * @param[out] data Array of pointer to each data block to transfer, each data block corresponds to an enabled channels
 * If mux memory is selected, only first data block is used
 * @param[in] count Number of elements to transfer for each channel
 */
void SAI_DRV_Receive(uint32_t instNum,
                     uint8_t* data[],
                     uint32_t count);


/*!
 * @brief Get status of a non-blocking transfer.
 *
 * @param[in] instNum Peripheral instance number
 * @param[out] countRemain Number of elements remain for each channel.
 * This parameter can be NULL
 * @return Status of the transfer, can be success, aborted or busy. Note that aborted status can imply
 * a timed out blocking transfer, not only user abort.
 */
status_t SAI_DRV_GetReceivingStatus(uint32_t instNum,
                                    uint32_t *countRemain);


/*!
 * @brief Abort an ongoing transfer
 *
 * @param[in] instNum Peripheral instance number
 */
void SAI_DRV_AbortReceiving(uint32_t instNum);


/*!
 * @brief Get default config structure for I2S standard.
 * Init config structure for I2S interface: Interrupt mode, internal generated bit clock 1.4112 MHz,
 * 16 bit word, 2 channel 1 data line (data line 0),
 * @param[out] uc Pointer to config structure to fill in
 */
void SAI_DRV_GetDefaultConfig(sai_user_config_t* uc);

/*@}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/
#endif /* SAI_DRIVER_H_ */
/******************************************************************************/
/* EOF */
/******************************************************************************/
