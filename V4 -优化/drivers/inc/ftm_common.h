/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
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
/*!
 * @file ftm_common.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from
 * 'essentially Boolean' type to 'essentially unsigned'.This is required by the
 * conversion of a bit-field of a bool type into a bit-field of a register type.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.7, Composite expression with smaller
 * essential type than other operand.
 * The expression is safe as the baud rate calculation algorithm cannot overflow
 * the result.
 */

#ifndef FTM_COMMON_H
#define FTM_COMMON_H

#include "ftm_hw_access.h"

/*!
 * @addtogroup ftm_common FTM Common Driver
 * @ingroup ftm
 * @brief FlexTimer Peripheral Common Driver.
 * @addtogroup ftm_common
 * @{
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of base addresses for FTM instances. */
extern FTM_Type * const g_ftmBase[FTM_INSTANCE_COUNT];

/*! @brief Interrupt vectors for the FTM peripheral. */
extern const IRQn_Type g_ftmIrqId[FTM_INSTANCE_COUNT][FEATURE_FTM_CHANNEL_COUNT];
extern const IRQn_Type g_ftmFaultIrqId[FTM_INSTANCE_COUNT];
extern const IRQn_Type g_ftmOverflowIrqId[FTM_INSTANCE_COUNT];
extern const IRQn_Type g_ftmReloadIrqId[FTM_INSTANCE_COUNT];

#ifdef ERRATA_E10856
extern bool faultDetection;
#endif
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief FlexTimer operation mode
 *
 * Implements : ftm_config_mode_t_Class
 */
typedef enum
{
    FTM_MODE_NOT_INITIALIZED    = 0x00U,    /*!< The driver is not initialized */
    FTM_MODE_INPUT_CAPTURE      = 0x01U,    /*!< Input capture */
    FTM_MODE_OUTPUT_COMPARE     = 0x02U,    /*!< Output compare */
    FTM_MODE_EDGE_ALIGNED_PWM   = 0x03U,    /*!< Edge aligned PWM */
    FTM_MODE_CEN_ALIGNED_PWM    = 0x04U,    /*!< Center aligned PWM */
    FTM_MODE_QUADRATURE_DECODER = 0x05U,    /*!< Quadrature decoder */
    FTM_MODE_UP_TIMER           = 0x06U,    /*!< Timer with up counter */
    FTM_MODE_UP_DOWN_TIMER      = 0x07U     /*!< timer with up-down counter */
} ftm_config_mode_t;

/*!
 * @brief FlexTimer quadrature decode modes, phase encode or count and direction mode
 *
 * Implements : ftm_quad_decode_mode_t_Class
 */
typedef enum
{
    FTM_QUAD_PHASE_ENCODE   = 0x00U,    /*!< Phase encoding mode                 */
    FTM_QUAD_COUNT_AND_DIR  = 0x01U     /*!< Counter and direction encoding mode */
} ftm_quad_decode_mode_t;

/*!
 * @brief FlexTimer quadrature phase polarities, normal or inverted polarity
 *
 * Implements : ftm_quad_phase_polarity_t_Class
 */
typedef enum
{
    FTM_QUAD_PHASE_NORMAL = 0x00U,  /*!< Phase input signal is not inverted before identifying
                                     *   the rising and falling edges of this signal */
    FTM_QUAD_PHASE_INVERT = 0x01U   /*!< Phase input signal is inverted before identifying
                                     *   the rising and falling edges of this signal */
} ftm_quad_phase_polarity_t;

/*!
 * @brief Channel event callback function
 *
 * Callback functions are called by the FTM driver in Input Capture mode when an event
 * is detected(change in logical state of a pin or measurement complete)
 */
typedef void (* ftm_channel_event_callback_t)(void * userData);

/*!
 * @brief FlexTimer state structure of the driver
 *
 * Implements : ftm_state_t_Class
 */
typedef struct
{
    ftm_clock_source_t ftmClockSource;                                          /*!< Clock source used by FTM counter */
    ftm_config_mode_t ftmMode;                                                  /*!< Mode of operation for FTM */
    uint16_t ftmPeriod;                                                         /*!< This field is used only in PWM mode to store signal period */
    uint32_t ftmSourceClockFrequency;                                           /*!< The clock frequency is used for counting */
    uint16_t measurementResults[FEATURE_FTM_CHANNEL_COUNT];                     /*!< This field is used only in input capture mode to store edges time stamps */
    void * channelsCallbacksParams[FEATURE_FTM_CHANNEL_COUNT];                  /*!< Vector of callbacks  parameters for channels events */
    ftm_channel_event_callback_t channelsCallbacks[FEATURE_FTM_CHANNEL_COUNT];  /*!< Vector of callbacks for channels events */
} ftm_state_t;

/*!
 * @brief FlexTimer Registers sync parameters
 *        Please don't use software and hardware trigger simultaneously
 * Implements : ftm_pwm_sync_t_Class
 */
typedef struct
{
    bool softwareSync;                          /*!< True - enable software sync,
                                                 *   False - disable software sync */
    bool hardwareSync0;                         /*!< True - enable hardware 0 sync,
                                                 *   False - disable hardware 0 sync */
    bool hardwareSync1;                         /*!< True - enable hardware 1 sync,
                                                 *   False - disable hardware 1 sync */
    bool hardwareSync2;                         /*!< True - enable hardware 2 sync,
                                                 *   False - disable hardware 2 sync */
    bool maxLoadingPoint;                       /*!< True - enable maximum loading point,
                                                 *   False - disable maximum loading point */
    bool minLoadingPoint;                       /*!< True - enable minimum loading point,
                                                 *   False - disable minimum loading point */
    ftm_reg_update_t inverterSync;              /*!< Configures INVCTRL sync */
    ftm_reg_update_t outRegSync;                /*!< Configures SWOCTRL sync */
    ftm_reg_update_t maskRegSync;               /*!< Configures OUTMASK sync */
    ftm_reg_update_t initCounterSync;           /*!< Configures CNTIN sync */
    bool autoClearTrigger;                      /*!< Available only for hardware trigger */
    ftm_pwm_sync_mode_t syncPoint;              /*!< Configure synchronization method
                                                 *   (waiting next loading point or immediate) */
} ftm_pwm_sync_t;

/*!
 * @brief Configuration structure that the user needs to set
 *
 * Implements : ftm_user_config_t_Class
 */
typedef struct
{
    ftm_pwm_sync_t syncMethod;              /*!< Register sync options available in the
                                             *   ftm_sync_method_t enumeration  */
    ftm_config_mode_t ftmMode;              /*!< Mode of operation for FTM */
    ftm_clock_ps_t ftmPrescaler;            /*!< Register pre-scaler options available in the
                                             *   ftm_clock_ps_t enumeration  */
    ftm_clock_source_t ftmClockSource;      /*!< Select clock source for FTM */
    ftm_bdm_mode_t BDMMode;                 /*!< Select FTM behavior in BDM mode */
    bool isTofIsrEnabled;                   /*!< true: enable interrupt,
                                             *   false: write interrupt is disabled */
    bool enableInitializationTrigger;       /*!< true: enable the generation of initialization trigger
                                             *   false: disable the generation of initialization trigger */
} ftm_user_config_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointer to runtime state structure. */
extern ftm_state_t * ftmStatePtr[FTM_INSTANCE_COUNT];

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Sets the filter Pre-scaler divider.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] filterPrescale The FTM peripheral clock pre-scale divider
 *
 * Implements : FTM_DRV_SetClockFilterPs_Activity
 */
static inline void FTM_DRV_SetClockFilterPs(FTM_Type * const ftmBase,
                                            uint8_t filterPrescale)
{
    FTM_RMW_SC(ftmBase, FTM_SC_FLTPS_MASK, FTM_SC_FLTPS(filterPrescale));
}

/*!
 * @brief Reads the FTM filter clock divider.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The FTM filter clock pre-scale divider
 *
 * Implements : FTM_DRV_GetClockFilterPs_Activity
 */
static inline uint8_t FTM_DRV_GetClockFilterPs(const FTM_Type * ftmBase)
{
    return (uint8_t)((((ftmBase)->SC) & FTM_SC_FLTPS_MASK) >> FTM_SC_FLTPS_SHIFT);
}

/*!
 * @brief Reads the FTM clock source.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The FTM clock source selection
 *          - 00: No clock
 *          - 01: system clock
 *          - 10: fixed clock
 *          - 11: External clock
 *
 * Implements : FTM_DRV_GetClockSource_Activity
 */
static inline uint8_t FTM_DRV_GetClockSource(const FTM_Type * ftmBase)
{
    return (uint8_t)((((ftmBase)->SC) & FTM_SC_CLKS_MASK) >> FTM_SC_CLKS_SHIFT);
}

/*!
 * @brief Reads the FTM clock divider.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The FTM clock pre-scale divider
 *
 * Implements : FTM_DRV_GetClockPs_Activity
 */
static inline uint8_t FTM_DRV_GetClockPs(const FTM_Type * ftmBase)
{
    return (uint8_t)((((ftmBase)->SC) & FTM_SC_PS_MASK) >> FTM_SC_PS_SHIFT);
}

/*!
 * @brief Reads the bit that controls enabling the FTM timer overflow interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return State of Timer Overflow Interrupt
 *         - true : Overflow interrupt is enabled
 *         - false: Overflow interrupt is disabled
 *
 * Implements : FTM_DRV_IsOverflowIntEnabled_Activity
 */
static inline bool FTM_DRV_IsOverflowIntEnabled(const FTM_Type * ftmBase)
{
    return ((ftmBase->SC & FTM_SC_TOIE_MASK) >> FTM_SC_TOIE_SHIFT) != 0U;
}

/*!
 * @brief Returns the FTM peripheral timer overflow interrupt flag.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return State of Timer Overflow Flag
 *         - true : FTM counter has overflowed
 *         - false: FTM counter has not overflowed
 *
 * Implements : FTM_DRV_HasTimerOverflowed_Activity
 */
static inline bool FTM_DRV_HasTimerOverflowed(const FTM_Type * ftmBase)
{
    return ((ftmBase->SC & FTM_SC_TOF_MASK) >> FTM_SC_TOF_SHIFT) != 0U;
}

/*!
 * @brief Gets the FTM count direction bit.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The Center-Aligned PWM selection
 *         - 1U: Up counting mode
 *         - 0U: Up down counting mode
 *
 * Implements : FTM_DRV_GetCpwms_Activity
 */
static inline bool FTM_DRV_GetCpwms(const FTM_Type * ftmBase)
{
    return ((ftmBase->SC & FTM_SC_CPWMS_MASK) >> FTM_SC_CPWMS_SHIFT) != 0U;
}

/*!
 * @brief Set the FTM reload interrupt enable.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable - true : Reload interrupt is  enabled
 *                   - false: Reload interrupt is disabled
 *
 * Implements : FTM_DRV_SetReIntEnabledCmd_Activity
 */
static inline void FTM_DRV_SetReIntEnabledCmd(FTM_Type * const ftmBase,
                                              bool enable)
{
    FTM_RMW_SC(ftmBase, FTM_SC_RIE_MASK, FTM_SC_RIE(enable));
}

/*!
 * @brief Get the state whether the FTM counter reached a reload point.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return State of reload point
 *         - true : FTM counter reached a reload point
 *         - false: FTM counter did not reach a reload point
 *
 * Implements : FTM_DRV_GetReloadFlag_Activity
 */
static inline bool FTM_DRV_GetReloadFlag(const FTM_Type * ftmBase)
{
    return ((ftmBase->SC & FTM_SC_RF_MASK) >> FTM_SC_RF_SHIFT) != 0U;
}

/*!
 * @brief Clears the reload flag bit.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * Implements : FTM_DRV_ClearReloadFlag_Activity
 */
static inline void FTM_DRV_ClearReloadFlag(FTM_Type * const ftmBase)
{
    FTM_RMW_SC(ftmBase, FTM_SC_RF_MASK, FTM_SC_RF(0U));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->SC;
#endif
}

/*!
 * @brief Returns the FTM peripheral current counter value.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The current FTM timer counter value
 *
 * Implements : FTM_DRV_GetCounter_Activity
 */
static inline uint16_t FTM_DRV_GetCounter(const FTM_Type * ftmBase)
{
    return (uint16_t)((((ftmBase)->CNT) & FTM_CNT_COUNT_MASK) >> FTM_CNT_COUNT_SHIFT);
}

/*!
 * @brief Returns the FTM peripheral counter modulo value.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return FTM timer modulo value
 *
 * Implements : FTM_DRV_GetMod_Activity
 */
static inline uint16_t FTM_DRV_GetMod(const FTM_Type * ftmBase)
{
    return (uint16_t)((((ftmBase)->MOD) & FTM_MOD_MOD_MASK) >> FTM_MOD_MOD_SHIFT);
}

/*!
 * @brief Returns the FTM peripheral counter initial value.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return FTM timer counter initial value
 *
 * Implements : FTM_DRV_GetCounterInitVal_Activity
 */
static inline uint16_t FTM_DRV_GetCounterInitVal(const FTM_Type * ftmBase)
{
    return (uint16_t)((((ftmBase)->CNTIN) & FTM_CNTIN_INIT_MASK) >> FTM_CNTIN_INIT_SHIFT);
}

/*!
 * @brief Clears the content of Channel (n) Status And Control.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel  The FTM peripheral channel number
 *
 * Implements : FTM_DRV_ClearChSC_Activity
 */
static inline void FTM_DRV_ClearChSC(FTM_Type * const ftmBase,
                                     uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    ((ftmBase)->CONTROLS[channel].CnSC) = 0U;
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->CONTROLS[channel].CnSC;
#endif
}

/*!
 * @brief Gets the FTM peripheral timer channel mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * @return The MSnB:MSnA mode value, will be 00, 01, 10, 11
 *
 * Implements : FTM_DRV_GetChnMode_Activity
 */
static inline uint8_t FTM_DRV_GetChnMode(const FTM_Type * ftmBase,
                                         uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);
    uint8_t retValue;

    retValue = (uint8_t)((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_MSA_MASK) >> FTM_CnSC_MSA_SHIFT);

    retValue |= (uint8_t)(((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_MSB_MASK) >> FTM_CnSC_MSB_SHIFT) << 1U);

    return retValue;
}

/*!
 * @brief Gets the FTM peripheral timer channel edge level.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * @return The ELSnB:ELSnA mode value, will be 00, 01, 10, 11
 *
 * Implements : FTM_DRV_GetChnEdgeLevel_Activity
 */
static inline uint8_t FTM_DRV_GetChnEdgeLevel(const FTM_Type * ftmBase,
                                              uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);
    uint8_t retValue;

    retValue = (uint8_t)((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_ELSA_MASK) >> FTM_CnSC_ELSA_SHIFT);

    retValue |= (uint8_t)(((((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_ELSB_MASK) >> FTM_CnSC_ELSB_SHIFT) << 1U);

    return retValue;
}

/*!
 * @brief Configure the feature of FTM counter reset by the selected input capture event.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] enable Enable the FTM counter reset
 *                   - true : FTM counter is reset
 *                   - false: FTM counter is not reset
 *
 * Implements : FTM_DRV_SetChnIcrstCmd_Activity
 */
static inline void FTM_DRV_SetChnIcrstCmd(FTM_Type * const ftmBase,
                                          uint8_t channel,
                                          bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    /* Write ICRST bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_ICRST_MASK, FTM_CnSC_ICRST(enable));
}

/*!
 * @brief Returns whether the FTM FTM counter is reset.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @return State of the FTM peripheral timer channel ICRST
 *         - true : Enabled the FTM counter reset
 *         - false: Disabled the FTM counter reset
 *
 * Implements : FTM_DRV_IsChnIcrst_Activity
 */
static inline bool FTM_DRV_IsChnIcrst(const FTM_Type * ftmBase,
                                      uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_ICRST_MASK) != 0U;
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel DMA.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] enable Enable DMA transfers for the channel
 *                   - true : Enabled DMA transfers
 *                   - false: Disabled DMA transfers
 *
 * Implements : FTM_DRV_SetChnDmaCmd_Activity
 */
static inline void FTM_DRV_SetChnDmaCmd(FTM_Type * const ftmBase,
                                        uint8_t channel,
                                        bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    /* Write DMA bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_DMA_MASK, FTM_CnSC_DMA(enable));
}

/*!
 * @brief Returns whether the FTM peripheral timer channel DMA is enabled.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @return State of the FTM peripheral timer channel DMA
 *         - true : Enabled DMA transfers
 *         - false: Disabled DMA transfers
 *
 * Implements : FTM_DRV_IsChnDma_Activity
 */
static inline bool FTM_DRV_IsChnDma(const FTM_Type * ftmBase,
                                    uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_DMA_MASK) != 0U;
}

/*!
 * @brief Get FTM channel(n) interrupt enabled or not.
 * @param[in] ftmBase FTM module base address
 * @param[in] channel The FTM peripheral channel number
 *
 * Implements : FTM_DRV_IsChnIntEnabled_Activity
 */
static inline bool FTM_DRV_IsChnIntEnabled(const FTM_Type * ftmBase,
                                           uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_CHIE_MASK) != 0U;
}

/*!
 * @brief Returns whether any event for the FTM peripheral timer channel has occurred.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * @return State of channel flag
 *         - true : Event occurred
 *         - false: No event occurred.
 *
 * Implements : FTM_DRV_HasChnEventOccurred_Activity
 */
static inline bool FTM_DRV_HasChnEventOccurred(const FTM_Type * ftmBase,
                                               uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_CHF_MASK) != 0U;
}

/*!
 * @brief Enables or disables the trigger generation on FTM channel outputs.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] enable Trigger mode control
 *                   - false : Enable PWM output without generating a pulse
 *                   - true  : Disable a trigger generation on channel output
 *
 * Implements : FTM_DRV_SetTrigModeControlCmd_Activity
 */
static inline void FTM_DRV_SetTrigModeControlCmd(FTM_Type * const ftmBase,
                                                 uint8_t channel,
                                                 bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    /* Write TRIGMODE bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_TRIGMODE_MASK, FTM_CnSC_TRIGMODE((enable)));
}

/*!
 * @brief Returns whether the trigger mode is enabled.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @return State of the channel outputs
 *         - true : Enabled a trigger generation on channel output
 *         - false: PWM outputs without generating a pulse
 *
 * Implements : FTM_DRV_GetTriggerControled_Activity
 */
static inline bool FTM_DRV_GetTriggerControled(const FTM_Type * ftmBase,
                                               uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_TRIGMODE_MASK) != 0U;
}

/*!
 * @brief Get the state of channel input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @return State of the channel inputs
 *         - true : The channel input is one
 *         - false: The channel input is zero
 *
 * Implements : FTM_DRV_GetChInputState_Activity
 */
static inline bool FTM_DRV_GetChInputState(const FTM_Type * ftmBase,
                                           uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_CHIS_MASK) != 0U;
}

/*!
 * @brief Get the value of channel output.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @return Value of the channel outputs
 *         - true : The channel output is one
 *         - false: The channel output is zero
 *
 * Implements : FTM_DRV_GetChOutputValue_Activity
 */
static inline bool FTM_DRV_GetChOutputValue(const FTM_Type * ftmBase,
                                            uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->CONTROLS[channel].CnSC) & FTM_CnSC_CHOV_MASK) != 0U;
}

/*!
 * @brief Gets the FTM peripheral timer channel counter value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * @return Channel counter value
 *
 * Implements : FTM_DRV_GetChnCountVal_Activity
 */
static inline uint16_t FTM_DRV_GetChnCountVal(const FTM_Type * ftmBase,
                                              uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (uint16_t)((ftmBase)->CONTROLS[channel].CnV);
}

/*!
 * @brief Gets the FTM peripheral timer  channel event status.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * @return Channel event status
 *         - true  : A channel event has occurred
 *         - false : No channel event has occurred
 *
 * Implements : FTM_DRV_GetChnEventStatus_Activity
 */
static inline bool FTM_DRV_GetChnEventStatus(const FTM_Type * ftmBase,
                                             uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    return (((ftmBase)->STATUS) & (1UL << channel)) != 0U;
}

/*!
 * @brief Gets the FTM peripheral timer status info for all channels.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return Channel event status value
 *
 * Implements : FTM_DRV_GetEventStatus_Activity
 */
static inline uint32_t FTM_DRV_GetEventStatus(const FTM_Type * ftmBase)
{
    return ((ftmBase)->STATUS) & (0xFFU);
}

/*!
 * @brief Clears the FTM peripheral timer all channel event status.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 *
 * Implements : FTM_DRV_ClearChnEventStatus_Activity
 */
static inline void FTM_DRV_ClearChnEventStatus(FTM_Type * const ftmBase,
                                               uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    ((ftmBase)->STATUS) &= (~(1UL << channel));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->STATUS;
#endif
}

/*!
 * @brief Sets the FTM peripheral timer channel output mask.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] mask Value to set Output Mask
 *                 - true : Channel output is masked
 *                 - false: Channel output is not masked
 *
 * Implements : FTM_DRV_SetChnOutputMask_Activity
 */
static inline void FTM_DRV_SetChnOutputMask(FTM_Type * const ftmBase,
                                            uint8_t channel,
                                            bool mask)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if (mask)
    {
        ((ftmBase)->OUTMASK) |= (1UL << channel);
    }
    else
    {
        ((ftmBase)->OUTMASK) &= ~(1UL << channel);
    }
}

/*!
 * @brief Sets the FTM peripheral timer channel output initial state 0 or 1.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] state Initial state for channels output
 *                  - true : The initialization value is 1
 *                  - false: The initialization value is 0
 *
 * Implements : FTM_DRV_SetChnOutputInitStateCmd_Activity
 */
static inline void FTM_DRV_SetChnOutputInitStateCmd(FTM_Type * const ftmBase,
                                                    uint8_t channel,
                                                    bool state)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if (state)
    {
        ((ftmBase)->OUTINIT) |= (1UL << channel);
    }
    else
    {
        ((ftmBase)->OUTINIT) &= ~(1UL << channel);
    }
}

/*!
 * @brief Disables the FTM peripheral timer fault interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * Implements : FTM_DRV_DisableFaultInt_Activity
 */
static inline void FTM_DRV_DisableFaultInt(FTM_Type * const ftmBase)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_FAULTIE_MASK, FTM_MODE_FAULTIE(0U));
}

/*!
 * @brief Return true/false whether the Fault interrupt was enabled or not
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * Implements : FTM_DRV_IsFaultIntEnabled_Activity
 */
static inline bool FTM_DRV_IsFaultIntEnabled(const FTM_Type * ftmBase)
{
    return ((ftmBase->MODE & FTM_MODE_FAULTIE_MASK) >> FTM_MODE_FAULTIE_SHIFT) != 0U;
}

/*!
 * @brief Clears all fault interrupt flags that are active.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * Implements : FTM_DRV_ClearFaultsIsr_Activity
 */
static inline void FTM_DRV_ClearFaultsIsr(FTM_Type * const ftmBase)
{
    FTM_RMW_FMS(ftmBase, FTM_FMS_FAULTF0_MASK | FTM_FMS_FAULTF_MASK, FTM_FMS_FAULTF0(0U) | FTM_FMS_FAULTF(0U));
    FTM_RMW_FMS(ftmBase, FTM_FMS_FAULTF1_MASK | FTM_FMS_FAULTF_MASK, FTM_FMS_FAULTF1(0U) | FTM_FMS_FAULTF(0U));
    FTM_RMW_FMS(ftmBase, FTM_FMS_FAULTF2_MASK | FTM_FMS_FAULTF_MASK, FTM_FMS_FAULTF2(0U) | FTM_FMS_FAULTF(0U));
    FTM_RMW_FMS(ftmBase, FTM_FMS_FAULTF3_MASK | FTM_FMS_FAULTF_MASK, FTM_FMS_FAULTF3(0U) | FTM_FMS_FAULTF(0U));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->FMS;
#endif
}

/*!
 * @brief Enables or disables the FTM peripheral timer capture test mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable Capture Test Mode Enable
 *            - true : Capture test mode is enabled
 *            - false: Capture test mode is disabled
 *
 * Implements : FTM_DRV_SetCaptureTestCmd_Activity
 */
static inline void FTM_DRV_SetCaptureTestCmd(FTM_Type * const ftmBase,
                                             bool enable)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_CAPTEST_MASK, FTM_MODE_CAPTEST(enable));
}

/*!
 * @brief Get status of the FTMEN bit in the FTM_MODE register.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @return the FTM Enable status
 *         - true : TPM compatibility. Free running counter and synchronization compatible with TPM
 *         - false: Free running counter and synchronization are different from TPM behaviour
 *
 * Implements : FTM_DRV_IsFtmEnable_Activity
 */
static inline bool FTM_DRV_IsFtmEnable(const FTM_Type * ftmBase)
{
    return ((ftmBase->MODE & FTM_MODE_FTMEN_MASK) >> FTM_MODE_FTMEN_SHIFT) != 0U;
}

/*!
 * @brief Initializes the channels output.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable Initialize the channels output
 *                   - true : The channels output is initialized according to the state of OUTINIT reg
 *                   - false: No effect
 *
 * Implements : FTM_DRV_SetInitChnOutputCmd_Activity
 */
static inline void FTM_DRV_SetInitChnOutputCmd(FTM_Type * const ftmBase,
                                               bool enable)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_INIT_MASK, FTM_MODE_INIT(enable));
}

/*!
 * @brief Determines if the FTM counter is re-initialized when the selected trigger for
 * synchronization is detected.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable FTM counter re-initialization selection
 *                   - true : To update FTM counter when triggered
 *                   - false: To count normally
 *
 * Implements : FTM_DRV_SetCountReinitSyncCmd_Activity
 */
static inline void FTM_DRV_SetCountReinitSyncCmd(FTM_Type * const ftmBase,
                                                 bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_REINIT_MASK, FTM_SYNC_REINIT(enable));
}

/*!
 * @brief Enables the FTM peripheral timer dual edge capture mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 *
 * @return Dual edge capture mode status
 *         - true : To enable dual edge capture
 *         - false: To disable
 *
 * Implements : FTM_DRV_GetDualEdgeCaptureBit_Activity
 */
static inline bool FTM_DRV_GetDualEdgeCaptureBit(const FTM_Type * ftmBase,
                                                 uint8_t chnlPairNum)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    return (((ftmBase)->COMBINE) & ((uint32_t)FTM_COMBINE_DECAPEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH))) != 0U;
}

/*!
 * @brief Verify if an channels pair is used in combine mode or not.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 *
 * @return Channel pair output combine mode status
 *         - true : Channels pair are combined
 *         - false: Channels pair are independent
 *
 * Implements : FTM_DRV_GetDualChnCombineCmd_Activity
 */
static inline bool FTM_DRV_GetDualChnCombineCmd(const FTM_Type * ftmBase,
                                                uint8_t chnlPairNum)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    return (((ftmBase)->COMBINE) & (FTM_COMBINE_COMBINE0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH))) != 0U;
}

/*!
 * @brief Checks whether any channel trigger event has occurred.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @return Channel trigger status
 *         - true : If there is a channel trigger event
 *         - false: If not.
 *
 * Implements : FTM_DRV_IsChnTriggerGenerated_Activity
 */
static inline bool FTM_DRV_IsChnTriggerGenerated(const FTM_Type * ftmBase)
{
    return (ftmBase->EXTTRIG & FTM_EXTTRIG_TRIGF_MASK) != 0U;
}

/*!
 * @brief Clear the channel trigger flag.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * Implements : FTM_DRV_ClearChnTriggerFlag_Activity
 */
static inline void FTM_DRV_ClearChnTriggerFlag(FTM_Type * const ftmBase)
{
    FTM_RMW_EXTTRIG_REG(ftmBase, FTM_EXTTRIG_TRIGF_MASK, FTM_EXTTRIG_TRIGF(0UL));
}

/*Fault mode status*/
/*!
 * @brief Gets the FTM detected fault input.
 *
 * This function reads the status for all fault inputs
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The fault byte
 *         - 0 : No fault condition was detected.
 *         - 1 : A fault condition was detected.
 *
 * Implements : FTM_DRV_GetDetectedFaultInput_Activity
 */
static inline bool FTM_DRV_GetDetectedFaultInput(const FTM_Type * ftmBase)
{
    return (ftmBase->FMS & FTM_FMS_FAULTF_MASK) != 0U;
}

/*!
 * @brief Checks whether the write protection is enabled.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return Write-protection status
 *         - true : If enabled
 *         - false: If not
 *
 * Implements : FTM_DRV_IsWriteProtectionEnabled_Activity
 */
static inline bool FTM_DRV_IsWriteProtectionEnabled(const FTM_Type * ftmBase)
{
    return (ftmBase->FMS & FTM_FMS_WPEN_MASK) != 0U;
}

/*!
 * @brief Checks whether the logic OR of the fault inputs is enabled.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return the enabled fault inputs status
 *         - true : The logic OR of the enabled fault inputs is 1
 *         - false: The logic OR of the enabled fault inputs is 0
 *
 * Implements : FTM_DRV_IsFaultInputEnabled_Activity
 */
static inline bool FTM_DRV_IsFaultInputEnabled(const FTM_Type * ftmBase)
{
    return (ftmBase->FMS & FTM_FMS_FAULTIN_MASK) != 0U;
}

/*!
 * @brief Checks whether a fault condition is detected at the fault input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel
 *
 * @return the fault condition status
 *         - true : A fault condition was detected at the fault input
 *         - false: No fault condition was detected at the fault input
 *
 * Implements : FTM_DRV_IsFaultFlagDetected_Activity
 */
static inline bool FTM_DRV_IsFaultFlagDetected(const FTM_Type * ftmBase,
                                               uint8_t channel)
{
    DEV_ASSERT(channel < CHAN4_IDX);

    return (ftmBase->FMS & (FTM_FMS_FAULTF0_MASK << channel)) != 0U;
}

/*!
 * @brief Clear a fault condition is detected at the fault input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel
 *
 * Implements : FTM_DRV_ClearFaultFlagDetected_Activity
 */
static inline void FTM_DRV_ClearFaultFlagDetected(FTM_Type * const ftmBase,
                                                  uint8_t channel)
{
    DEV_ASSERT(channel < CHAN4_IDX);

    ((ftmBase)->FMS) &= (~(1UL << channel));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->FMS;
#endif
}

/*!
 * @brief Enables or disables the phase B input filter.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of phase B input filter
 *                   - true : Enables the phase input filter
 *                   - false: Disables the filter
 *
 * Implements : FTM_DRV_SetQuadPhaseBFilterCmd_Activity
 */
static inline void FTM_DRV_SetQuadPhaseBFilterCmd(FTM_Type * const ftmBase,
                                                  bool enable)
{
    if (enable)
    {
        ((ftmBase)->QDCTRL) |= (1UL << FTM_QDCTRL_PHBFLTREN_SHIFT);
    }
    else
    {
        ((ftmBase)->QDCTRL) &= ~(1UL << FTM_QDCTRL_PHBFLTREN_SHIFT);
    }
}

/*!
 * @brief Selects polarity for the quadrature decode phase A input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode Phase A input polarity selection
 *                - FTM_QUAD_PHASE_NORMAL: Normal polarity
 *                - FTM_QUAD_PHASE_INVERT: Inverted polarity
 *
 * Implements : FTM_DRV_SetQuadPhaseAPolarity_Activity
 */
static inline void FTM_DRV_SetQuadPhaseAPolarity(FTM_Type * const ftmBase,
                                                 ftm_quad_phase_polarity_t mode)
{
    FTM_RMW_QDCTRL(ftmBase, FTM_QDCTRL_PHAPOL_MASK, FTM_QDCTRL_PHAPOL(mode));
}

/*!
 * @brief Selects polarity for the quadrature decode phase B input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode Phase B input polarity selection
 *                - FTM_QUAD_PHASE_NORMAL: Normal polarity
 *                - FTM_QUAD_PHASE_INVERT: Inverted polarity
 *
 * Implements : FTM_DRV_SetQuadPhaseBPolarity_Activity
 */
static inline void FTM_DRV_SetQuadPhaseBPolarity(FTM_Type * const ftmBase,
                                                 ftm_quad_phase_polarity_t mode)
{
    FTM_RMW_QDCTRL(ftmBase, FTM_QDCTRL_PHBPOL_MASK, FTM_QDCTRL_PHBPOL(mode));
}

/*!
 * @brief Sets the encoding mode used in quadrature decoding mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] quadMode Quadrature decoder mode selection
 *                     - FTM_QUAD_PHASE_ENCODE: Phase A and Phase B encoding mode
 *                     - FTM_QUAD_COUNT_AND_DIR: Count and direction encoding mode
 *
 * Implements : FTM_DRV_SetQuadMode_Activity
 */
static inline void FTM_DRV_SetQuadMode(FTM_Type * const ftmBase,
                                       ftm_quad_decode_mode_t quadMode)
{
    FTM_RMW_QDCTRL(ftmBase, FTM_QDCTRL_QUADMODE_MASK, FTM_QDCTRL_QUADMODE(quadMode));
}

/*!
 * @brief Gets the FTM counter direction in quadrature mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The counting direction
 *         - 1U: if counting direction is increasing
 *         - 0U: if counting direction is decreasing
 *
 * Implements : FTM_DRV_GetQuadDir_Activity
 */
static inline bool FTM_DRV_GetQuadDir(const FTM_Type * ftmBase)
{
    return (ftmBase->QDCTRL & FTM_QDCTRL_QUADIR_MASK) != 0U;
}

/*!
 * @brief Gets the Timer overflow direction in quadrature mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return The timer overflow direction
 *         - 1U: if TOF bit was set on the top of counting
 *         - 0U: if TOF bit was set on the bottom of counting
 *
 * Implements : FTM_DRV_GetQuadTimerOverflowDir_Activity
 */
static inline bool FTM_DRV_GetQuadTimerOverflowDir(const FTM_Type * ftmBase)
{
    return (ftmBase->QDCTRL & FTM_QDCTRL_TOFDIR_MASK) != 0U;
}

/*!
 * @brief Enables or disables the channel invert for a channel pair.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel invert for a channel pair
 *                   - true : To enable channel inverting
 *                   - false: To disable channel inversion
 *
 * Implements : FTM_DRV_SetDualChnInvertCmd_Activity
 */
static inline void FTM_DRV_SetDualChnInvertCmd(FTM_Type * const ftmBase,
                                               uint8_t chnlPairNum,
                                               bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->INVCTRL) |=  (1UL << chnlPairNum);
    }
    else
    {
        ((ftmBase)->INVCTRL) &=  ~(1UL << chnlPairNum);
    }
}

/*FTM software output control*/
/*!
 * @brief Enables or disables the channel software output control.
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel Channel to be enabled or disabled
 * @param[in] enable State of channel software output control
 *                   - true : To enable, channel output will be affected by software output control
 *                   - false: To disable, channel output is unaffected
 *
 * Implements : FTM_DRV_SetChnSoftwareCtrlCmd_Activity
 */
static inline void FTM_DRV_SetChnSoftwareCtrlCmd(FTM_Type * const ftmBase,
                                                 uint8_t channel,
                                                 bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if (enable)
    {
        ((ftmBase)->SWOCTRL) |=  (1UL << channel);
    }
    else
    {
        ((ftmBase)->SWOCTRL) &=  ~(1UL << channel);
    }
}

/*!
 * @brief Sets the channel software output control value.
 *
 * @param[in] ftmBase The FTM base address pointer.
 * @param[in] channel Channel to be configured
 * @param[in] enable State of software output control value
 *                   - true : to force 1 to the channel output
 *                   - false: to force 0 to the channel output
 *
 * Implements : FTM_DRV_SetChnSoftwareCtrlVal_Activity
 */
static inline void FTM_DRV_SetChnSoftwareCtrlVal(FTM_Type * const ftmBase,
                                                 uint8_t channel,
                                                 bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if (enable)
    {
        ((ftmBase)->SWOCTRL) |=  (1UL << (channel + FTM_SWOCTRL_CH0OCV_SHIFT));
    }
    else
    {
        ((ftmBase)->SWOCTRL) &=  ~(1UL << (channel + FTM_SWOCTRL_CH0OCV_SHIFT));
    }
}

/*FTM PWM load control*/
/*!
 * @brief Set the global load mechanism.
 *
 * @param[in] ftmBase The FTM base address pointer
 *                   - true : LDOK bit is set
 *                   - false: No action
 *
 * Implements : FTM_DRV_SetGlobalLoadCmd_Activity
 */
static inline void FTM_DRV_SetGlobalLoadCmd(FTM_Type * const ftmBase)
{
    ((ftmBase)->PWMLOAD) |=  (1UL << FTM_PWMLOAD_GLDOK_SHIFT);
}

/*!
 * @brief Enable the global load.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of the global load mechanism
 *                   - true : Global Load OK enabled
 *                   - false: Global Load OK disabled
 *
 * Implements : FTM_DRV_SetLoadCmd_Activity
 */
static inline void FTM_DRV_SetLoadCmd(FTM_Type * const ftmBase,
                                      bool enable)
{
    if (enable)
    {
        ((ftmBase)->PWMLOAD) |=  (1UL << FTM_PWMLOAD_GLEN_SHIFT);
    }
    else
    {
        ((ftmBase)->PWMLOAD) &=  ~(1UL << FTM_PWMLOAD_GLEN_SHIFT);
    }
}

/*!
 * @brief Enable the half cycle reload.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of the half cycle match as a reload opportunity
 *                   - true : Half cycle reload is enabled
 *                   - false: Half cycle reload is disabled
 *
 * Implements : FTM_DRV_SetHalfCycleCmd_Activity
 */
static inline void FTM_DRV_SetHalfCycleCmd(FTM_Type * const ftmBase,
                                           bool enable)
{
    if (enable)
    {
        ((ftmBase)->PWMLOAD) |=  (1UL << FTM_PWMLOAD_HCSEL_SHIFT);
    }
    else
    {
        ((ftmBase)->PWMLOAD) &=  ~(1UL << FTM_PWMLOAD_HCSEL_SHIFT);
    }
}

/*!
 * @brief Enables or disables the loading of MOD, CNTIN and CV with values of their write buffer.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of loading updated values
 *                   - true : To enable
 *                   - false: To disable
 *
 * Implements : FTM_DRV_SetPwmLoadCmd_Activity
 */
static inline void FTM_DRV_SetPwmLoadCmd(FTM_Type * const ftmBase,
                                         bool enable)
{
    if (enable)
    {
        ((ftmBase)->PWMLOAD) |=  (1UL << FTM_PWMLOAD_LDOK_SHIFT);
    }
    else
    {
        ((ftmBase)->PWMLOAD) &=  ~(1UL << FTM_PWMLOAD_LDOK_SHIFT);
    }
}

/*!
 * @brief Includes or excludes the channel in the matching process.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel Channel to be configured
 * @param[in] enable State of channel
 *                - true : means include the channel in the matching process
 *                - false: means do not include channel in the matching process
 *
 * Implements : FTM_DRV_SetPwmLoadChnSelCmd_Activity
 */
static inline void FTM_DRV_SetPwmLoadChnSelCmd(FTM_Type * const ftmBase,
                                               uint8_t channel,
                                               bool enable)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if (enable)
    {
        ((ftmBase)->PWMLOAD) |=  (1UL << channel);
    }
    else
    {
        ((ftmBase)->PWMLOAD) &=  ~(1UL << channel);
    }
}

/*FTM configuration*/
/*!
 * @brief Enables or disables the FTM initialization trigger on Reload Point.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable bit controls whether an initialization trigger is generated
 *                   - true : Trigger is generated when a reload point is reached
 *                   - false: Trigger is generated on counter wrap events
 *
 * Implements : FTM_DRV_SetInitTrigOnReloadCmd_Activity
 */
static inline void FTM_DRV_SetInitTrigOnReloadCmd(FTM_Type * const ftmBase,
                                                  bool enable)
{
    ftmBase->CONF = (ftmBase->CONF & ~FTM_CONF_ITRIGR_MASK) | FTM_CONF_ITRIGR(enable);
}

/*!
 * @brief Enables or disables the FTM global time base signal generation to other FTM's.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of global time base signal
 *                   - true : To enable
 *                   - false: To disable
 *
 * Implements : FTM_DRV_SetGlobalTimeBaseOutputCmd_Activity
 */
static inline void FTM_DRV_SetGlobalTimeBaseOutputCmd(FTM_Type * const ftmBase,
                                                      bool enable)
{
    ftmBase->CONF = (ftmBase->CONF & ~FTM_CONF_GTBEOUT_MASK) | FTM_CONF_GTBEOUT(enable);
}

/*!
 * @brief Enables or disables the FTM timer global time base.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of global time base
 *                   - true : To enable
 *                   - false: To disable
 *
 * Implements : FTM_DRV_SetGlobalTimeBaseCmd_Activity
 */
static inline void FTM_DRV_SetGlobalTimeBaseCmd(FTM_Type * const ftmBase,
                                                bool enable)
{
    ftmBase->CONF = (ftmBase->CONF & ~FTM_CONF_GTBEEN_MASK) | FTM_CONF_GTBEEN(enable);
}

/*!
 * @brief Sets the FTM timer TOF Frequency
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] val Value of the TOF bit set frequency
 *
 * Implements : FTM_DRV_SetLoadFreq_Activity
 */
static inline void FTM_DRV_SetLoadFreq(FTM_Type * const ftmBase,
                                       uint8_t val)
{
    FTM_RMW_CONF(ftmBase, FTM_CONF_LDFQ_MASK, FTM_CONF_LDFQ(val));
}

/*!
 * @brief Sets the FTM extended dead-time value for the channel pair.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channelPair The FTM peripheral channel pair (n)
 * @param[in] value The FTM peripheral extend pre-scale divider using the concatenation with the dead-time value
 *
 * Implements : FTM_DRV_SetExtPairDeadtimeValue_Activity
 */
static inline void FTM_DRV_SetExtPairDeadtimeValue(FTM_Type * const ftmBase,
                                                   uint8_t channelPair,
                                                   uint8_t value)
{
    DEV_ASSERT(value < 16U);
    DEV_ASSERT(channelPair < CHAN4_IDX);

    switch (channelPair)
    {
        case CHAN0_IDX:
            FTM_RMW_PAIR0DEADTIME(ftmBase, FTM_PAIR0DEADTIME_DTVALEX_MASK, FTM_PAIR0DEADTIME_DTVALEX(value));
            break;
        case CHAN1_IDX:
            FTM_RMW_PAIR1DEADTIME(ftmBase, FTM_PAIR1DEADTIME_DTVALEX_MASK, FTM_PAIR1DEADTIME_DTVALEX(value));
            break;
        case CHAN2_IDX:
            FTM_RMW_PAIR2DEADTIME(ftmBase, FTM_PAIR2DEADTIME_DTVALEX_MASK, FTM_PAIR2DEADTIME_DTVALEX(value));
            break;
        case CHAN3_IDX:
            FTM_RMW_PAIR3DEADTIME(ftmBase, FTM_PAIR3DEADTIME_DTVALEX_MASK, FTM_PAIR3DEADTIME_DTVALEX(value));
            break;
        default:
            /* Nothing to do */
            break;
    }
}

/*!
 * @brief Sets the FTM dead time divider for the channel pair.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channelPair The FTM peripheral channel pair (n)
 * @param[in] divider The FTM peripheral pre-scaler divider
 *                    - FTM_DEADTIME_DIVID_BY_1 : Divide by 1
 *                    - FTM_DEADTIME_DIVID_BY_4 : Divide by 4
 *                    - FTM_DEADTIME_DIVID_BY_16: Divide by 16
 *
 * Implements : FTM_DRV_SetPairDeadtimePrescale_Activity
 */
static inline void FTM_DRV_SetPairDeadtimePrescale(FTM_Type * const ftmBase,
                                                   uint8_t channelPair,
                                                   ftm_deadtime_ps_t divider)
{
    DEV_ASSERT(channelPair < CHAN4_IDX);

    switch (channelPair)
    {
        case CHAN0_IDX:
            FTM_RMW_PAIR0DEADTIME(ftmBase, FTM_PAIR0DEADTIME_DTPS_MASK, FTM_PAIR0DEADTIME_DTPS((uint8_t)divider));
            break;
        case CHAN1_IDX:
            FTM_RMW_PAIR1DEADTIME(ftmBase, FTM_PAIR1DEADTIME_DTPS_MASK, FTM_PAIR1DEADTIME_DTPS((uint8_t)divider));
            break;
        case CHAN2_IDX:
            FTM_RMW_PAIR2DEADTIME(ftmBase, FTM_PAIR2DEADTIME_DTPS_MASK, FTM_PAIR2DEADTIME_DTPS((uint8_t)divider));
            break;
        case CHAN3_IDX:
            FTM_RMW_PAIR3DEADTIME(ftmBase, FTM_PAIR3DEADTIME_DTPS_MASK, FTM_PAIR3DEADTIME_DTPS((uint8_t)divider));
            break;
        default:
            /* Nothing to do */
            break;
    }
}

/*!
 * @brief Sets the FTM dead-time value for the channel pair.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channelPair The FTM peripheral channel pair (n)
 * @param[in] count The FTM peripheral selects the dead-time value
 *                  - 0U : no counts inserted
 *                  - 1U : 1 count is inserted
 *                  - 2U : 2 count is inserted
 *                  - ... up to a possible 63 counts
 *
 * Implements : FTM_DRV_SetPairDeadtimeCount_Activity
 */
static inline void FTM_DRV_SetPairDeadtimeCount(FTM_Type * const ftmBase,
                                                uint8_t channelPair,
                                                uint8_t count)
{
    DEV_ASSERT(channelPair < CHAN4_IDX);
    DEV_ASSERT(count < 64U);

    switch (channelPair)
    {
        case CHAN0_IDX:
            FTM_RMW_PAIR0DEADTIME(ftmBase, FTM_PAIR0DEADTIME_DTVAL_MASK, FTM_PAIR0DEADTIME_DTVAL(count));
            break;
        case CHAN1_IDX:
            FTM_RMW_PAIR1DEADTIME(ftmBase, FTM_PAIR1DEADTIME_DTVAL_MASK, FTM_PAIR1DEADTIME_DTVAL(count));
            break;
        case CHAN2_IDX:
            FTM_RMW_PAIR2DEADTIME(ftmBase, FTM_PAIR2DEADTIME_DTVAL_MASK, FTM_PAIR2DEADTIME_DTVAL(count));
            break;
        case CHAN3_IDX:
            FTM_RMW_PAIR3DEADTIME(ftmBase, FTM_PAIR3DEADTIME_DTVAL_MASK, FTM_PAIR3DEADTIME_DTVAL(count));
            break;
        default:
            /* Nothing to do */
            break;
    }
}

#if FEATURE_FTM_HAS_SUPPORTED_DITHERING
/*!
 * @brief Sets the mirror of the modulo integer value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value The value to be set to the timer modulo
 *
 * Implements : FTM_DRV_SetMirrorMod_Activity
 */
static inline void FTM_DRV_SetMirrorMod(FTM_Type * const ftmBase,
                                        uint16_t value)
{
    FTM_RMW_MOD_MIRROR(ftmBase, FTM_MOD_MIRROR_MOD_MASK, FTM_MOD_MIRROR_MOD(value));
}

/*!
 * @brief Returns the mirror of the FTM peripheral counter modulo value.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return the mirror of the FTM timer modulo value
 *
 * Implements : FTM_DRV_GetMirrorMod_Activity
 */
static inline uint16_t FTM_DRV_GetMirrorMod(const FTM_Type * ftmBase)
{
    return (uint16_t)((((ftmBase)->MOD_MIRROR) & FTM_MOD_MIRROR_MOD_MASK) >> FTM_MOD_MIRROR_MOD_SHIFT);
}

/*!
 * @brief Returns The modulo fractional value is used in the PWM period dithering.
 *
 * @param[in] ftmBase The FTM base address pointer
 *
 * @return the modulo fractional value
 *
 * Implements : FTM_DRV_GetModFracVal_Activity
 */
static inline uint8_t FTM_DRV_GetModFracVal(const FTM_Type * ftmBase)
{
    return (uint8_t)((((ftmBase)->MOD_MIRROR) & FTM_MOD_MIRROR_FRACMOD_MASK) >> FTM_MOD_MIRROR_FRACMOD_SHIFT);
}

/*!
 * @brief Sets the mirror of the channel (n) match integer value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel (n)
 * @param[in] value The value to be set to the mirror of the channel (n) match integer value
 *
 * Implements : FTM_DRV_SetMirrorChnMatchVal_Activity
 */
static inline void FTM_DRV_SetMirrorChnMatchVal(FTM_Type * const ftmBase,
                                                uint8_t channel,
                                                uint16_t value)
{
    FTM_RMW_CnV_MIRROR(ftmBase, channel, FTM_CV_MIRROR_VAL_MASK, FTM_CV_MIRROR_VAL(value));
}

/*!
 * @brief Returns the mirror of the channel (n) match integer value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel (n)
 *
 * @return the mirror of the channel (n) match value
 *
 * Implements : FTM_DRV_GetMirrorChnMatchVal_Activity
 */
static inline uint16_t FTM_DRV_GetMirrorChnMatchVal(const FTM_Type * ftmBase,
                                                    uint8_t channel)
{
    return (uint16_t)((((ftmBase)->CV_MIRROR[channel]) & FTM_CV_MIRROR_VAL_MASK) >> FTM_CV_MIRROR_VAL_SHIFT);
}

/*!
 * @brief Returns the channel (n) match fractional value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel (n)
 *
 * @return The channel (n) match fractional value is used in the PWM edge dithering
 *
 * Implements : FTM_DRV_GetChnMatchFracVal_Activity
 */
static inline uint8_t FTM_DRV_GetChnMatchFracVal(const FTM_Type * ftmBase,
                                                 uint8_t channel)
{
    return (uint8_t)((((ftmBase)->CV_MIRROR[channel]) & FTM_CV_MIRROR_FRACVAL_MASK) >> FTM_CV_MIRROR_FRACVAL_SHIFT);
}
#endif

/*!
 * @brief Initializes the FTM driver.
 *
 * @param[in] instance The FTM peripheral instance number.
 * @param[in] info The FTM user configuration structure, see #ftm_user_config_t.
 * @param[out] state The FTM state structure of the driver.
 * @return operation status
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_Init(uint32_t instance,
                      const ftm_user_config_t * info,
                      ftm_state_t * state);

/*!
 * @brief Shuts down the FTM driver.
 *
 * @param[in] instance The FTM peripheral instance number.
 * @return operation status
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_Deinit(uint32_t instance);

/*!
 * @brief This function will mask the output of the channels and at match events will be ignored
 * by the masked channels.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channelsMask The mask which will select which channels will ignore match events.
 * @param [in] softwareTrigger If true a software trigger is generate to update PWM parameters.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_MaskOutputChannels(uint32_t instance,
                                    uint32_t channelsMask,
                                    bool softwareTrigger);

/*!
 * @brief This function configure the initial counter value. The counter will get this
 * value after an overflow event.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] counterValue Initial counter value.
 * @param [in] softwareTrigger If true a software trigger is generate to update parameters.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */

status_t FTM_DRV_SetInitialCounterValue(uint32_t instance,
                                        uint16_t counterValue,
                                        bool softwareTrigger);

/*!
 * @brief This function configure the value of the counter which will generates an reload point.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] reloadPoint Counter value which generates the reload point.
 * @param [in] softwareTrigger If true a software trigger is generate to update parameters.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_SetHalfCycleReloadPoint(uint32_t instance,
                                         uint16_t reloadPoint,
                                         bool softwareTrigger);

/*!
 * @brief This function will force the output value of a channel to a specific value.
 * Before using this function it's mandatory to mask the match events using
 * FTM_DRV_MaskOutputChannels and to enable software output control using
 * FTM_DRV_SetSoftwareOutputChannelControl.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channelsValues The values which will be software configured for channels.
 * @param [in] softwareTrigger If true a software trigger is generate to update registers
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_SetSoftOutChnValue(uint32_t instance,
                                    uint8_t channelsValues,
                                    bool softwareTrigger);

/*!
 * @brief This function will configure which output channel can be software controlled.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channelsMask The mask which will configure the channels which can be software controlled.
 * @param [in] softwareTrigger If true a software trigger is generate to update registers
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_SetSoftwareOutputChannelControl(uint32_t instance,
                                                 uint8_t channelsMask,
                                                 bool softwareTrigger);

/*!
 * @brief This function will configure if the second channel of a pair will be inverted or not.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channelsPairMask The mask which will configure which channel pair will invert the second channel.
 * @param [in] softwareTrigger If true a software trigger is generate to update registers.
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_SetInvertingControl(uint32_t instance,
                                     uint8_t channelsPairMask,
                                     bool softwareTrigger);

/*!
 * @brief This function configure the maximum counter value.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] counterValue Maximum counter value
 * @param [in] softwareTrigger If true a software trigger is generate to update parameters
 * @return success
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_SetModuloCounterValue(uint32_t instance,
                                       uint16_t counterValue,
                                       bool softwareTrigger);

/*!
 * @brief This function configures sync mechanism for some FTM registers (MOD, CNINT, HCR,
 *          CnV, OUTMASK, INVCTRL, SWOCTRL).
 *
 * @param[in] instance The FTM peripheral instance number.
 * @param[in] param The sync configuration structure.
 * @return operation status
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t FTM_DRV_SetSync(uint32_t instance,
                         const ftm_pwm_sync_t * param);

/*!
 * @brief Retrieves the frequency of the clock source feeding the FTM counter.
 *
 * Function will return a 0 if no clock source is selected and the FTM counter is disabled
 *
 * @param [in] instance The FTM peripheral instance number.
 * @return The frequency of the clock source running the FTM counter (0 if counter is disabled)
 */
uint32_t FTM_DRV_GetFrequency(uint32_t instance);

/*!
 * @brief This function is used to covert the given frequency to period in ticks
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] freqencyHz Frequency value in Hz.
 *
 * @return The value in ticks of the frequency
 */
uint16_t FTM_DRV_ConvertFreqToPeriodTicks(uint32_t instance,
                                          uint32_t freqencyHz);

#if defined(__cplusplus)
}
#endif

/*! @}*/

/*! @}*/ /* End of addtogroup ftm_common */

#endif /* FTM_COMMON_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
