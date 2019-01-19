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
 * @file ftm_hw_access.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The macros defined are used to define features for each driver, so this might be reported
 * when the analysis is made only on one driver.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Directive 4.9, Function-like macro defined.
 * This macro is needed in creating a common name for any IP.
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

#ifndef FTM_HW_ACCESS_H
#define FTM_HW_ACCESS_H

#include <stdbool.h>
#include <stddef.h>
#include "status.h"
#include "device_registers.h"
#include "interrupt_manager.h"
#include "clock_manager.h"

/*!
 * @defgroup ftm_hw_access FTM HW ACCESS
 * @brief FlexTimer Module Hardware Abstraction Level.
 * FTM HW ACCESS provides low level APIs for reading and writing to all hardware features
 * of the FlexTimer module.
 * @{
 */

/*
 * S32K14x FTM
 *
 * FlexTimer Module
 *
 * Registers defined in this header file:
 * - FTM_SC - Status And Control
 * - FTM_CNT - Counter
 * - FTM_MOD - Modulo
 * - FTM_C0SC - Channel (n) Status And Control
 * - FTM_C0V - Channel (n) Value
 * - FTM_C1SC - Channel (n) Status And Control
 * - FTM_C1V - Channel (n) Value
 * - FTM_C2SC - Channel (n) Status And Control
 * - FTM_C2V - Channel (n) Value
 * - FTM_C3SC - Channel (n) Status And Control
 * - FTM_C3V - Channel (n) Value
 * - FTM_C4SC - Channel (n) Status And Control
 * - FTM_C4V - Channel (n) Value
 * - FTM_C5SC - Channel (n) Status And Control
 * - FTM_C5V - Channel (n) Value
 * - FTM_C6SC - Channel (n) Status And Control
 * - FTM_C6V - Channel (n) Value
 * - FTM_C7SC - Channel (n) Status And Control
 * - FTM_C7V - Channel (n) Value
 * - FTM_CNTIN - Counter Initial Value
 * - FTM_STATUS - Capture And Compare Status
 * - FTM_MODE - Features Mode Selection
 * - FTM_SYNC - Synchronization
 * - FTM_OUTINIT - Initial State For Channels Output
 * - FTM_OUTMASK - Output Mask
 * - FTM_COMBINE - Function For Linked Channels
 * - FTM_DEADTIME - Dead-time Insertion Control
 * - FTM_EXTTRIG - FTM External Trigger
 * - FTM_POL - Channels Polarity
 * - FTM_FMS - Fault Mode Status
 * - FTM_FILTER - Input Capture Filter Control
 * - FTM_FLTCTRL - Fault Control
 * - FTM_QDCTRL - Quadrature Decoder Control And Status
 * - FTM_CONF - Configuration
 * - FTM_FLTPOL - FTM Fault Input Polarity
 * - FTM_SYNCONF - Synchronization Configuration
 * - FTM_INVCTRL - FTM Inverting Control
 * - FTM_SWOCTRL - FTM Software Output Control
 * - FTM_PWMLOAD - FTM PWM Load
 * - FTM_HCR - Half Cycle Register
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @name FTM instance number
 * @{
 */
/*!< @brief Instance number for FTM0. */
#define FTM0_IDX (0U)
/*!< @brief Instance number for FTM1. */
#define FTM1_IDX (1U)
/*!< @brief Instance number for FTM2. */
#define FTM2_IDX (2U)
/*!< @brief Instance number for FTM3. */
#define FTM3_IDX (3U)
/*!< @brief Instance number for FTM4. */
#define FTM4_IDX (4U)
/*!< @brief Instance number for FTM5. */
#define FTM5_IDX (5U)
/*!< @brief Instance number for FTM6. */
#define FTM6_IDX (6U)
/*!< @brief Instance number for FTM7. */
#define FTM7_IDX (7U)
/*@}*/

/*!
 * @brief FTM_SC - Read and modify and write to Status And Control (RW)
 */
#define FTM_RMW_SC(base, mask, value) (((base)->SC) = ((((base)->SC) & ~(mask)) | (value)))

/*!
 * @brief FTM_CNT - Read and modify and write to Counter (RW)
 */
#define FTM_RMW_CNT(base, mask, value) (((base)->CNT) = ((((base)->CNT) & ~(mask)) | (value)))

/*!
 * @brief FTM_MOD - Read and modify and write Modulo (RW)
 */
#define FTM_RMW_MOD(base, mask, value) (((base)->MOD) = ((((base)->MOD) & ~(mask)) | (value)))

/*!
 * @brief FTM_CNTIN - Read and modify and write Counter Initial Value (RW)
 */
#define FTM_RMW_CNTIN(base, mask, value) (((base)->CNTIN) = ((((base)->CNTIN) & ~(mask)) | (value)))

/*!
 * @brief FTM_STATUS - Read and modify and write Capture And Compare Status (RW)
 */
#define FTM_RMW_STATUS(base, mask, value) (((base)->STATUS) = ((((base)->STATUS) & ~(mask)) | (value)))

/*!
 * @brief FTM_MODE -  Read and modify and write Counter Features Mode Selection (RW)
 */
#define FTM_RMW_MODE(base, mask, value) (((base)->MODE) = ((((base)->MODE) & ~(mask)) | (value)))

/*!
 * @brief FTM_CnSCV -  Read and modify and write Channel (n) Status And Control (RW)
 */
#define FTM_RMW_CnSCV_REG(base, channel, mask, value) (((base)->CONTROLS[channel].CnSC) = ((((base)->CONTROLS[channel].CnSC) & ~(mask)) | (value)))

/*!
 * @brief FTM_DEADTIME - Read and modify and write Dead-time Insertion Control (RW)
 */
#define FTM_RMW_DEADTIME(base, mask, value) (((base)->DEADTIME) = ((((base)->DEADTIME) & ~(mask)) | (value)))
/*!
 * @brief FTM_EXTTRIG - Read and modify and write External Trigger Control (RW)
 */
#define FTM_RMW_EXTTRIG_REG(base, mask, value) (((base)->EXTTRIG) = ((((base)->EXTTRIG) & ~(mask)) | (value)))

/*!
 * @brief FTM_FLTCTRL -  Read and modify and write Fault Control (RW)
 */
#define FTM_RMW_FLTCTRL(base, mask, value) (((base)->FLTCTRL) = ((((base)->FLTCTRL) & ~(mask)) | (value)))

/*!
 * @brief FTM_FMS -  Read and modify and write Fault Mode Status (RW)
 */
#define FTM_RMW_FMS(base, mask, value) (((base)->FMS) = ((((base)->FMS) & ~(mask)) | (value)))

/*!
 * @brief FTM_CONF -  Read and modify and write Configuration (RW)
 */
#define FTM_RMW_CONF(base, mask, value) (((base)->CONF) = ((((base)->CONF) & ~(mask)) | (value)))

/*!
 * @brief POL -  Read and modify and write Polarity (RW)
 */
#define FTM_RMW_POL(base, mask, value) (((base)->POL) = ((((base)->POL) & ~(mask)) | (value)))

/*!
 * @brief FILTER -  Read and modify and write Filter (RW)
 */
#define FTM_RMW_FILTER(base, mask, value) (((base)->FILTER) = ((((base)->FILTER) & ~(mask)) | (value)))

/*!
 * @brief SYNC -  Read and modify and write Synchronization (RW)
 */
#define FTM_RMW_SYNC(base, mask, value) (((base)->SYNC) = ((((base)->SYNC) & ~(mask)) | (value)))

/*!
 * @brief QDCTRL -  Read and modify and write Quadrature Decoder Control And Status (RW)
 */
#define FTM_RMW_QDCTRL(base, mask, value) (((base)->QDCTRL) = ((((base)->QDCTRL) & ~(mask)) | (value)))

/*!
 * @brief FTM_PAIR0DEADTIME - Read and modify and write Dead-time Insertion Control for the pair 0 (RW)
 */
#define FTM_RMW_PAIR0DEADTIME(base, mask, value) (((base)->PAIR0DEADTIME) = ((((base)->PAIR0DEADTIME) & ~(mask)) | (value)))

/*!
 * @brief FTM_PAIR1DEADTIME - Read and modify and write Dead-time Insertion Control for the pair 1 (RW)
 */
#define FTM_RMW_PAIR1DEADTIME(base, mask, value) (((base)->PAIR1DEADTIME) = ((((base)->PAIR1DEADTIME) & ~(mask)) | (value)))

/*!
 * @brief FTM_PAIR2DEADTIME - Read and modify and write Dead-time Insertion Control for the pair 2 (RW)
 */
#define FTM_RMW_PAIR2DEADTIME(base, mask, value) (((base)->PAIR2DEADTIME) = ((((base)->PAIR2DEADTIME) & ~(mask)) | (value)))

/*!
 * @brief FTM_PAIR3DEADTIME - Read and modify and write Dead-time Insertion Control for the pair 3 (RW)
 */
#define FTM_RMW_PAIR3DEADTIME(base, mask, value) (((base)->PAIR3DEADTIME) = ((((base)->PAIR3DEADTIME) & ~(mask)) | (value)))

#if FEATURE_FTM_HAS_SUPPORTED_DITHERING
/*!
 * @brief FTM_MOD_MIRROR - Read and modify and write mirror of modulo value for the FTM counter (RW)
 */
#define FTM_RMW_MOD_MIRROR(base, mask, value) (((base)->MOD_MIRROR) = ((((base)->MOD_MIRROR) & ~(mask)) | (value)))

/*!
 * @brief FTM_CnV_MIRROR -  Read and modify and write mirror of channel (n) match value (RW)
 */
#define FTM_RMW_CnV_MIRROR(base, channel, mask, value) (((base)->CV_MIRROR[channel]) = ((((base)->CV_MIRROR[channel]) & ~(mask)) | (value)))
#endif

/*!< @brief Channel number for CHAN0.*/
#define CHAN0_IDX (0U)
/*!< @brief Channel number for CHAN1.*/
#define CHAN1_IDX (1U)
/*!< @brief Channel number for CHAN2.*/
#define CHAN2_IDX (2U)
/*!< @brief Channel number for CHAN3.*/
#define CHAN3_IDX (3U)
/*!< @brief Channel number for CHAN4.*/
#define CHAN4_IDX (4U)
/*!< @brief Channel number for CHAN5.*/
#define CHAN5_IDX (5U)
/*!< @brief Channel number for CHAN6.*/
#define CHAN6_IDX (6U)
/*!< @brief Channel number for CHAN7.*/
#define CHAN7_IDX (7U)

/*******************************************************************************
 * Enumerations
 ******************************************************************************/

/*!
 * @brief FlexTimer clock source selection
 */
typedef enum
{
    FTM_CLOCK_SOURCE_NONE           = 0x00U,    /*!< None use clock for FTM  */
    FTM_CLOCK_SOURCE_SYSTEMCLK      = 0x01U,    /*!< System clock            */
    FTM_CLOCK_SOURCE_FIXEDCLK       = 0x02U,    /*!< Fixed clock             */
    FTM_CLOCK_SOURCE_EXTERNALCLK    = 0x03U     /*!< External clock          */
} ftm_clock_source_t;

/*!
 * @brief FlexTimer pre-scaler factor selection for the clock source.
 * In quadrature decoder mode set FTM_CLOCK_DIVID_BY_1
 */
typedef enum
{
    FTM_CLOCK_DIVID_BY_1    = 0x00U,    /*!< Divide by 1   */
    FTM_CLOCK_DIVID_BY_2    = 0x01U,    /*!< Divide by 2   */
    FTM_CLOCK_DIVID_BY_4    = 0x02U,    /*!< Divide by 4   */
    FTM_CLOCK_DIVID_BY_8    = 0x03U,    /*!< Divide by 8   */
    FTM_CLOCK_DIVID_BY_16   = 0x04U,    /*!< Divide by 16  */
    FTM_CLOCK_DIVID_BY_32   = 0x05U,    /*!< Divide by 32  */
    FTM_CLOCK_DIVID_BY_64   = 0x06U,    /*!< Divide by 64  */
    FTM_CLOCK_DIVID_BY_128  = 0x07U     /*!< Divide by 128 */
} ftm_clock_ps_t;

/*!
 * @brief FlexTimer pre-scaler factor for the dead-time insertion
 */
typedef enum
{
    FTM_DEADTIME_DIVID_BY_1  = 0x01U, /*!< Divide by 1   */
    FTM_DEADTIME_DIVID_BY_4  = 0x02U, /*!< Divide by 4   */
    FTM_DEADTIME_DIVID_BY_16 = 0x03U  /*!< Divide by 16  */
} ftm_deadtime_ps_t;

/*!
 * @brief FlexTimer PWM output pulse mode, high-true or low-true on match up
 */
typedef enum
{
    FTM_POLARITY_LOW  = 0x00U,  /*!< When counter > CnV output signal is LOW */
    FTM_POLARITY_HIGH = 0x01U   /*!< When counter > CnV output signal is HIGH */
} ftm_polarity_t;

/*!
 * @brief FlexTimer PWM channel (n+1) polarity for combine mode
 */
typedef enum
{
    FTM_MAIN_INVERTED   = 0x01U,  /*!< The channel (n+1) output is the inverse of the
                                   *   channel (n) output  */
    FTM_MAIN_DUPLICATED = 0x00U   /*!< The channel (n+1) output is the same as the
                                   *   channel (n) output */
} ftm_second_channel_polarity_t;

/*!
 * @brief Options for the FlexTimer behavior in BDM Mode
 */
typedef enum
{
    FTM_BDM_MODE_00 = 0x00U,    /*!< FTM counter stopped, CH(n)F bit can be set, FTM channels
                                 *   in functional mode, writes to MOD,CNTIN and C(n)V registers bypass
                                 *   the register buffers */
    FTM_BDM_MODE_01 = 0x01U,    /*!< FTM counter stopped, CH(n)F bit is not set, FTM channels
                                 *   outputs are forced to their safe value , writes to MOD,CNTIN and
                                 *   C(n)V registers bypass the register buffers */
    FTM_BDM_MODE_10 = 0x02U,    /*!< FTM counter stopped, CH(n)F bit is not set, FTM channels
                                *    outputs are frozen when chip enters in BDM mode, writes to MOD,
                                *    CNTIN and C(n)V registers bypass the register buffers */
    FTM_BDM_MODE_11 = 0x03U     /*!< FTM counter in functional mode, CH(n)F bit can be set,
                                 *   FTM channels in functional mode, writes to MOD,CNTIN and C(n)V
                                 *   registers is in fully functional mode */
} ftm_bdm_mode_t;

/*!
 * @brief FlexTimer fault control
 */
typedef enum
{
    FTM_FAULT_CONTROL_DISABLED  = 0x00U,    /*!< Fault control is disabled for all channels */
    FTM_FAULT_CONTROL_MAN_EVEN  = 0x01U,    /*!< Fault control is enabled for even channels
                                             *   only (channels 0, 2, 4, and 6), and the selected
                                             *   mode is the manual fault clearing */
    FTM_FAULT_CONTROL_MAN_ALL   = 0x02U,    /*!< Fault control is enabled for all channels,
                                             *   and the selected mode is the manual fault clearing */
    FTM_FAULT_CONTROL_AUTO_ALL  = 0x03U     /*!< Fault control is enabled for all channels, and
                                             *   the selected mode is the automatic fault clearing */
} ftm_fault_mode_t;

/*!
 * @brief FTM sync source
 */
typedef enum
{
    FTM_SYSTEM_CLOCK    = 0U,       /*!< Register is updated with its buffer value at all rising
                                     *   edges of system clock */
    FTM_PWM_SYNC        = 1U        /*!< Register is updated with its buffer value at the
                                     *   FTM synchronization */
} ftm_reg_update_t;

/*!
 * @brief FTM update register
 */
typedef enum
{
    FTM_WAIT_LOADING_POINTS = 0U,   /*!< FTM register is updated at first loading point */
    FTM_UPDATE_NOW          = 1U    /*!< FTM register is updated immediately */
} ftm_pwm_sync_mode_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Sets the value for the half cycle reload register.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value The 16 bit counter value
 */
static inline void FTM_DRV_SetHalfCycleValue(FTM_Type * const ftmBase,
                                             uint16_t value)
{
    ((ftmBase)->HCR) = value;
}

/*!
 * @brief Sets the FTM clock source.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] clock The FTM peripheral clock selection
 *            - 00: No clock
 *            - 01: system clock
 *            - 10: fixed clock
 *            - 11: External clock
 */
static inline void FTM_DRV_SetClockSource(FTM_Type * const ftmBase,
                                          ftm_clock_source_t clock)
{
    FTM_RMW_SC(ftmBase, FTM_SC_CLKS_MASK, FTM_SC_CLKS(clock));
}

/*!
 * @brief Sets the FTM clock divider.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] ps The FTM peripheral clock pre-scale divider
 */
static inline void FTM_DRV_SetClockPs(FTM_Type * const ftmBase,
                                      ftm_clock_ps_t ps)
{
    FTM_RMW_SC(ftmBase, FTM_SC_PS_MASK, FTM_SC_PS(ps));
}

/*!
 * @brief  Enables the FTM peripheral timer overflow interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] state - true : Overflow interrupt enabled
 *                  - false: Overflow interrupt disabled
 */
static inline void FTM_DRV_SetTimerOverflowInt(FTM_Type * const ftmBase,
                                               bool state)
{
    FTM_RMW_SC(ftmBase, FTM_SC_TOIE_MASK, FTM_SC_TOIE(state));
}

/*!
 * @brief Enable PWM channel Outputs.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM channel

 */
static inline void FTM_DRV_EnablePwmChannelOutputs(FTM_Type * const ftmBase,
                                                   uint8_t channel)
{
    FTM_RMW_SC(ftmBase, (1UL << (channel + FTM_FEATURE_OUTPUT_CHANNEL_OFFSET)), (1UL << (channel + FTM_FEATURE_OUTPUT_CHANNEL_OFFSET)));
}

/*!
 * @brief Disable PWM channel Outputs.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM channel
 */
static inline void FTM_DRV_DisablePwmChannelOutputs(FTM_Type * const ftmBase,
                                                    uint8_t channel)
{
    uint32_t regValue = ((ftmBase)->SC);
    regValue = regValue & (~(1UL << (channel + FTM_FEATURE_OUTPUT_CHANNEL_OFFSET)));
    ((ftmBase)->SC) = (regValue);
}

/*!
 * @brief Clears the timer overflow interrupt flag.
 *
 * @param[in] ftmBase The FTM base address pointer
 */
static inline void FTM_DRV_ClearTimerOverflow(FTM_Type * const ftmBase)
{
    FTM_RMW_SC(ftmBase, FTM_SC_TOF_MASK, FTM_SC_TOF(0U));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->SC;
#endif
}

/*!
 * @brief Sets the FTM count direction bit.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode The Center-Aligned PWM selection
 *                 - 1U: Up counting mode
 *                 - 0U: Up down counting mode
 */
static inline void FTM_DRV_SetCpwms(FTM_Type * const ftmBase,
                                    bool mode)
{
    FTM_RMW_SC(ftmBase, FTM_SC_CPWMS_MASK, FTM_SC_CPWMS(mode));
}

/*!
 * @brief Sets the FTM peripheral current counter value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value The FTM timer counter value to be set
 */
static inline void FTM_DRV_SetCounter(FTM_Type * const ftmBase,
                                      uint16_t value)
{
    FTM_RMW_CNT(ftmBase, FTM_CNT_COUNT_MASK, FTM_CNT_COUNT(value));
}

/*!
 * @brief Sets the FTM peripheral timer modulo value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value The value to be set to the timer modulo
 */
static inline void FTM_DRV_SetMod(FTM_Type * const ftmBase,
                                  uint16_t value)
{
    FTM_RMW_MOD(ftmBase, FTM_MOD_MOD_MASK, FTM_MOD_MOD(value));
}

/*!
 * @brief Sets the FTM peripheral timer counter initial value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value initial value to be set
 */
static inline void FTM_DRV_SetCounterInitVal(FTM_Type * const ftmBase,
                                             uint16_t value)
{
    FTM_RMW_CNTIN(ftmBase, FTM_CNTIN_INIT_MASK, FTM_CNTIN_INIT(value));
}

/*!
 * @brief Sets the FTM peripheral timer channel mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] selection The mode to be set valid value MSnB:MSnA :00, 01, 10, 11
 */
static inline void FTM_DRV_SetChnMSnBAMode(FTM_Type * const ftmBase,
                                           uint8_t channel,
                                           uint8_t selection)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    /* write MSA bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_MSA_MASK, FTM_CnSC_MSA((uint32_t)selection & 0x01U));

    /* write MSB bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_MSB_MASK, FTM_CnSC_MSB(((uint32_t)selection & 0x02U) >> 1U));
}

/*!
 * @brief Sets the FTM peripheral timer channel edge level.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] level ELSnB:ELSnA :00, 01, 10, 11
 */
static inline void FTM_DRV_SetChnEdgeLevel(FTM_Type * const ftmBase,
                                           uint8_t channel,
                                           uint8_t level)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    /* write MSA bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_ELSA_MASK, FTM_CnSC_ELSA((uint32_t)level & 0x01U));

    /* write MSB bit */
    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_ELSB_MASK, FTM_CnSC_ELSB(((uint32_t)level & 0x02U) >> 1U));
}

/*!
 * @brief Enables the FTM peripheral timer channel(n) interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 */
static inline void FTM_DRV_EnableChnInt(FTM_Type * const ftmBase,
                                        uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_CHIE_MASK, FTM_CnSC_CHIE(1U));
}

/*!
 * @brief Disables the FTM peripheral timer channel(n) interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 */
static inline void FTM_DRV_DisableChnInt(FTM_Type * const ftmBase,
                                         uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_CHIE_MASK, FTM_CnSC_CHIE(0U));
}

/*!
 * @brief Clear the channel flag by writing a 0 to the CHF bit.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 */
static inline void FTM_DRV_ClearChnEventFlag(FTM_Type * const ftmBase,
                                             uint8_t channel)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    FTM_RMW_CnSCV_REG(ftmBase, channel, FTM_CnSC_CHF_MASK, FTM_CnSC_CHF(0U));
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    ftmBase->CONTROLS[channel].CnSC;
#endif
}

/*FTM channel control*/
/*!
 * @brief Sets the FTM peripheral timer channel counter value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] value Counter value to be set
 */
static inline void FTM_DRV_SetChnCountVal(FTM_Type * const ftmBase,
                                          uint8_t channel,
                                          uint16_t value)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    ((ftmBase)->CONTROLS[channel].CnV) = value;
}

/*!
 * @brief Writes the provided value to the OUTMASK register.
 *
 * This function will mask/unmask multiple channels.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] regVal Value to be written to the register
 */
static inline void FTM_DRV_SetOutmaskReg(FTM_Type * const ftmBase,
                                         uint32_t regVal)
{
    ((ftmBase)->OUTMASK) = regVal;
}

/*!
 * @brief Sets the FTM peripheral timer channel output polarity.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] polarity The polarity to be set
 *            - FTM_POLARITY_HIGH : The channel polarity is active low
 *            - FTM_POLARITY_LOW  : The channel polarity is active high
 */
static inline void FTM_DRV_SetChnOutputPolarityCmd(FTM_Type * const ftmBase,
                                                   uint8_t channel,
                                                   ftm_polarity_t polarity)
{
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);

    if (FTM_POLARITY_HIGH == polarity)
    {
        ((ftmBase)->POL) &= ~(1UL << channel);
    }
    else
    {
        ((ftmBase)->POL) |= (1UL << channel);
    }
}

/*!
 * @brief Sets the FTM peripheral timer channel fault input polarity.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] fltChannel The FTM peripheral channel number
 * @param[in] polarity The polarity to be set
 *                - FTM_POLARITY_LOW : The fault input polarity is active high
 *                - FTM_POLARITY_HIGH: The fault input polarity is active low
 */
static inline void FTM_DRV_SetChnFaultInputPolarityCmd(FTM_Type * const ftmBase,
                                                       uint8_t fltChannel,
                                                       ftm_polarity_t polarity)
{
    DEV_ASSERT(fltChannel < FTM_FEATURE_FAULT_CHANNELS);

    if (FTM_POLARITY_HIGH == polarity)
    {
        ((ftmBase)->FLTPOL) &= ~(1UL << fltChannel);
    }
    else
    {
        ((ftmBase)->FLTPOL) |= (1UL << fltChannel);
    }
}

/*!
 * @brief Enables/disables the FTM peripheral timer fault interrupt.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] state Timer fault interrupt state
 *            - true : Fault control interrupt is enable
 *            - false: Fault control interrupt is disabled
 */
static inline void FTM_DRV_SetFaultInt(FTM_Type * const ftmBase,
                                       bool state)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_FAULTIE_MASK, FTM_MODE_FAULTIE(state));
}

/*!
 * @brief Defines the FTM fault control mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode Fault control mode value
 * - FTM_FAULT_CONTROL_DISABLED: Fault control disabled
 * - FTM_FAULT_CONTROL_MAN_EVEN: Fault control enabled for even channel (0, 2, 4, 6) and manual fault clearing.
 * - FTM_FAULT_CONTROL_MAN_ALL : Fault control enabled for all channels and manual fault clearing is enabled.
 * - FTM_FAULT_CONTROL_AUTO_ALL: Fault control enabled for all channels and automatic fault clearing is enabled.
 */
static inline void FTM_DRV_SetFaultControlMode(FTM_Type * const ftmBase,
                                               ftm_fault_mode_t mode)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_FAULTM_MASK, FTM_MODE_FAULTM((uint32_t)mode));
}

/*!
 * @brief Enables or disables the FTM write protection.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable The FTM write protection selection
 *                   - true : Write-protection is enabled
 *                   - false: Write-protection is disabled
 */
static inline void FTM_DRV_SetWriteProtectionCmd(FTM_Type * const ftmBase,
                                                 bool enable)
{
    if (enable)
    {
        ftmBase->FMS = (ftmBase->FMS & ~FTM_FMS_WPEN_MASK) | FTM_FMS_WPEN(1U);
    }
    else
    {
        ftmBase->MODE = (ftmBase->MODE & ~FTM_MODE_WPDIS_MASK) | FTM_MODE_WPDIS(1U);
    }
}

/*!
 * @brief Enables the FTM peripheral timer group.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable FTM mode selection
 *                   - true : All registers including FTM-specific registers are available
 *                   - false: Only the TPM-compatible registers are available
 */
static inline void FTM_DRV_Enable(FTM_Type * const ftmBase,
                                  bool enable)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_FTMEN_MASK, FTM_MODE_FTMEN(enable));
}

/*!
 * @brief Sets the FTM peripheral timer sync mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable PWM synchronization mode
 *                   - false: No restriction both software and hardware triggers can be used
 *                   - true : Software trigger can only be used for MOD and CnV synchronization,
 *                            hardware trigger only for OUTMASK and FTM counter synchronization.
 */
static inline void FTM_DRV_SetPwmSyncMode(FTM_Type * const ftmBase,
                                          bool enable)
{
    FTM_RMW_MODE(ftmBase, FTM_MODE_PWMSYNC_MASK, FTM_MODE_PWMSYNC(enable));
}

/*!
 * @brief Enables or disables the FTM peripheral timer software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer.
 * @param[in] enable Software trigger selection
 *                   - true : Software trigger is selected
 *                   - false: Software trigger is not selected
 */
static inline void FTM_DRV_SetSoftwareTriggerCmd(FTM_Type * const ftmBase,
                                                 bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_SWSYNC_MASK, FTM_SYNC_SWSYNC(enable));
}

/*!
 * @brief Sets the FTM hardware synchronization trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] trigger_num Number of trigger
 *                        - 0U: trigger 0
 *                        - 1U: trigger 1
 *                        - 2U: trigger 2
 * @param[in] enable State of trigger
 *                   - true : Enable hardware trigger from field trigger_num for PWM synchronization
 *                   - false: Disable hardware trigger from field trigger_num for PWM synchronization
 */
static inline void FTM_DRV_SetHardwareSyncTriggerSrc(FTM_Type * const ftmBase,
                                                     uint8_t trigger_num,
                                                     bool enable)
{
    DEV_ASSERT(trigger_num < 3U);

    if (enable)
    {
        ((ftmBase)->SYNC) |= ((uint32_t)(FTM_SYNC_TRIG0_MASK) << trigger_num);
    }
    else
    {
        ((ftmBase)->SYNC) &= ~((uint32_t)(FTM_SYNC_TRIG0_MASK) << trigger_num);
    }
}

/*!
 * @brief Determines when the OUTMASK register is updated with the value of its buffer.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable Output Mask synchronization selection
 *                   - true : OUTMASK register is updated only by PWM synchronization
 *                   - false: OUTMASK register is updated in all rising edges of the system clock
 */
static inline void FTM_DRV_SetOutmaskPwmSyncModeCmd(FTM_Type * const ftmBase,
                                                    bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_SYNCHOM_MASK, FTM_SYNC_SYNCHOM(enable));
}

/*!
 * @brief Enables or disables the FTM peripheral timer maximum loading points.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable Maximum loading point selection
 *                   - true : To enable maximum loading point
 *                   - false: To disable
 */
static inline void FTM_DRV_SetMaxLoadingCmd(FTM_Type * const ftmBase,
                                            bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_CNTMAX_MASK, FTM_SYNC_CNTMAX(enable));
}

/*!
 * @brief Enables or disables the FTM peripheral timer minimum loading points.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable Minimum loading point selection
 *                   - true : To enable minimum loading point
 *                   - false: To disable
 */
static inline void FTM_DRV_SetMinLoadingCmd(FTM_Type * const ftmBase,
                                            bool enable)
{
    FTM_RMW_SYNC(ftmBase, FTM_SYNC_CNTMIN_MASK, FTM_SYNC_CNTMIN(enable));
}

/*!
 * @brief Enables the FTM peripheral timer channel modified combine mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel pair outputs modified combine
 *                   - true : To enable modified combine
 *                   - false: To disable modified combine
 */
static inline void FTM_DRV_SetDualChnMofCombineCmd(FTM_Type * const ftmBase,
                                                   uint8_t chnlPairNum,
                                                   bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_MCOMBINE0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_MCOMBINE0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables the FTM peripheral timer channel pair fault control.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel pair fault control
 *                   - true : To enable fault control
 *                   - false: To disable
 */
static inline void FTM_DRV_SetDualChnFaultCmd(FTM_Type * const ftmBase,
                                              uint8_t chnlPairNum,
                                              bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_FAULTEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_FAULTEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel pair counter PWM sync.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel pair counter PWM sync
 *                   - true : To enable PWM synchronization
 *                   - false: To disable
 */
static inline void FTM_DRV_SetDualChnPwmSyncCmd(FTM_Type * const ftmBase,
                                                uint8_t chnlPairNum,
                                                bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_SYNCEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_SYNCEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables or disabled the FTM peripheral timer channel pair deadtime insertion.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel pair deadtime insertion
 *                   - true : To enable deadtime insertion
 *                   - false: To disable
 */
static inline void FTM_DRV_SetDualChnDeadtimeCmd(FTM_Type * const ftmBase,
                                                 uint8_t chnlPairNum,
                                                 bool enable)
{
    DEV_ASSERT(chnlPairNum < FEATURE_FTM_CHANNEL_COUNT);

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_DTEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_DTEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel dual edge capture.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel dual edge capture
 *                   - true : To enable dual edge capture mode
 *                   - false: To disable
 */
static inline void FTM_DRV_SetDualChnDecapCmd(FTM_Type * const ftmBase,
                                              uint8_t chnlPairNum,
                                              bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_DECAP0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_DECAP0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables the FTM peripheral timer dual edge capture mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of dual edge capture mode
 *                   - true : To enable dual edge capture
 *                   - false: To disable
 */
static inline void FTM_DRV_SetDualEdgeCaptureCmd(FTM_Type * const ftmBase,
                                                 uint8_t chnlPairNum,
                                                 bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= ((uint32_t)FTM_COMBINE_DECAPEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~((uint32_t)FTM_COMBINE_DECAPEN0_MASK << (chnlPairNum * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel pair output complement mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] polarity State of channel pair output complement mode
 *            - FTM_MAIN_INVERTED : The channel (n+1) output is the complement of the channel (n) output
 *            - FTM_MAIN_DUPLICATED: The channel (n+1) output is the same as the channel (n) output
 */
static inline void FTM_DRV_SetDualChnCompCmd(FTM_Type * const ftmBase,
                                             uint8_t chnlPairNum,
                                             ftm_second_channel_polarity_t polarity)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (polarity == FTM_MAIN_INVERTED)
    {
        ((ftmBase)->COMBINE) |= (FTM_COMBINE_COMP0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~(FTM_COMBINE_COMP0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Enables or disables the FTM peripheral timer channel pair output combine mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 * @param[in] enable State of channel pair output combine mode
 *                   - true : Channels pair are combined
 *                   - false: Channels pair are independent
 */
static inline void FTM_DRV_SetDualChnCombineCmd(FTM_Type * const ftmBase,
                                                uint8_t chnlPairNum,
                                                bool enable)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    if (enable)
    {
        ((ftmBase)->COMBINE) |= (FTM_COMBINE_COMBINE0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
    else
    {
        ((ftmBase)->COMBINE) &= ~(FTM_COMBINE_COMBINE0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH));
    }
}

/*!
 * @brief Verify if an channels pair is used in the modified combine mode or not.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] chnlPairNum The FTM peripheral channel pair number
 *
 * @return Channel pair output combine mode status
 *         - true : Channels pair are combined
 *         - false: Channels pair are independent
 */
static inline bool FTM_DRV_GetDualChnMofCombineCmd(const FTM_Type * ftmBase,
                                                   uint8_t chnlPairNum)
{
    DEV_ASSERT(chnlPairNum < (FEATURE_FTM_CHANNEL_COUNT >> 1U));

    return (((ftmBase)->COMBINE) & (FTM_COMBINE_MCOMBINE0_MASK << ((uint32_t)(chnlPairNum) * FTM_FEATURE_COMBINE_CHAN_CTRL_WIDTH))) != 0U;
}

/*!
 * @brief Sets the FTM extended dead-time value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value The FTM peripheral extend pre-scale divider
 */
static inline void FTM_DRV_SetExtDeadtimeValue(FTM_Type * const ftmBase,
                                               uint8_t value)
{
    DEV_ASSERT(value < 16U);

    FTM_RMW_DEADTIME(ftmBase, FTM_DEADTIME_DTVALEX_MASK, FTM_DEADTIME_DTVALEX(value));
}

/*!
 * @brief Sets the FTM dead time divider.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] divider The FTM peripheral pre-scaler divider
 *                    - FTM_DEADTIME_DIVID_BY_1 : Divide by 1
 *                    - FTM_DEADTIME_DIVID_BY_4 : Divide by 4
 *                    - FTM_DEADTIME_DIVID_BY_16: Divide by 16
 */
static inline void FTM_DRV_SetDeadtimePrescale(FTM_Type * const ftmBase,
                                               ftm_deadtime_ps_t divider)
{
    FTM_RMW_DEADTIME(ftmBase, FTM_DEADTIME_DTPS_MASK, FTM_DEADTIME_DTPS((uint8_t)divider));
}

/*!
 * @brief Sets the FTM deadtime value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] count The FTM peripheral pre-scaler divider
 *                  - 0U : no counts inserted
 *                  - 1U : 1 count is inserted
 *                  - 2U : 2 count is inserted
 *                  - ... up to a possible 63 counts
 */
static inline void FTM_DRV_SetDeadtimeCount(FTM_Type * const ftmBase,
                                            uint8_t count)
{
    DEV_ASSERT(count < 64U);

    FTM_RMW_DEADTIME(ftmBase, FTM_DEADTIME_DTVAL_MASK, FTM_DEADTIME_DTVAL(count));
}

/*FTM external trigger */
/*!
 * @brief Enables or disables the generation of the trigger when the FTM counter is equal
 * to the CNTIN register.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of initialization trigger
 *                   - true : To enable
 *                   - false: To disable
 */
static inline void FTM_DRV_SetInitTriggerCmd(FTM_Type * const ftmBase,
                                             bool enable)
{
    ftmBase->EXTTRIG = (ftmBase->EXTTRIG & ~FTM_EXTTRIG_INITTRIGEN_MASK) | FTM_EXTTRIG_INITTRIGEN(enable);
}

/* Quadrature decoder control */
/*!
 * @brief Enables the channel quadrature decoder.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of channel quadrature decoder
 *                  - true : To enable
 *                  - false: To disable
 */
static inline void FTM_DRV_SetQuadDecoderCmd(FTM_Type * const ftmBase,
                                             bool enable)
{
    if (enable)
    {
        ((ftmBase)->QDCTRL) |= (1UL << FTM_QDCTRL_QUADEN_SHIFT);
    }
    else
    {
        ((ftmBase)->QDCTRL) &= ~(1UL << FTM_QDCTRL_QUADEN_SHIFT);
    }
}

/*!
 * @brief Enables or disables the phase A input filter.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of phase A input filter
 *                   - true : Enables the phase input filter
 *                   - false: Disables the filter
 */
static inline void FTM_DRV_SetQuadPhaseAFilterCmd(FTM_Type * const ftmBase,
                                                  bool enable)
{
    if (enable)
    {
        ((ftmBase)->QDCTRL) |= (1UL << FTM_QDCTRL_PHAFLTREN_SHIFT);
    }
    else
    {
        ((ftmBase)->QDCTRL) &= ~(1UL << FTM_QDCTRL_PHAFLTREN_SHIFT);
    }
}

/*!
 * @brief Sets the fault input filter value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value Fault input filter value
 */
static inline void FTM_DRV_SetFaultInputFilterVal(FTM_Type * const ftmBase,
                                                  uint32_t value)
{
    FTM_RMW_FLTCTRL(ftmBase, FTM_FLTCTRL_FFVAL_MASK, FTM_FLTCTRL_FFVAL(value));
}

/*!
 * @brief Enables or disables the fault input filter.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] inputNum Fault input to be configured, valid value 0, 1, 2, 3
 * @param[in] enable State of fault input filter
 *                   - true : To enable fault input filter
 *                   - false: To disable fault input filter
 */
static inline void FTM_DRV_SetFaultInputFilterCmd(FTM_Type * const ftmBase,
                                                  uint8_t inputNum,
                                                  bool enable)
{
    DEV_ASSERT(inputNum < CHAN4_IDX);

    if (enable)
    {
        ((ftmBase)->FLTCTRL) |=  (1UL << (inputNum + FTM_FLTCTRL_FFLTR0EN_SHIFT));
    }
    else
    {
        ((ftmBase)->FLTCTRL) &=  ~(1UL << (inputNum + FTM_FLTCTRL_FFLTR0EN_SHIFT));
    }
}

/*!
 * @brief Clears the entire content value of the Fault control register.
 *
 * @param[in] ftmBase The FTM base address pointer
 */
static inline void FTM_DRV_ClearFaultControl(FTM_Type * const ftmBase)
{
    ((ftmBase)->FLTCTRL) =  0U;
}

/*!
 * @brief Enables or disables the fault input.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] inputNum fault input to be configured, valid value 0, 1, 2, 3
 * @param[in] enable State of fault input
 *                   - true : To enable fault input
 *                   - false: To disable fault input
 */
static inline void FTM_DRV_SetFaultInputCmd(FTM_Type * const ftmBase,
                                            uint8_t inputNum,
                                            bool enable)
{
    DEV_ASSERT(inputNum < CHAN4_IDX);

    if (enable)
    {
        ((ftmBase)->FLTCTRL) |=  (1UL << inputNum);
    }
    else
    {
        ((ftmBase)->FLTCTRL) &=  ~(1UL << inputNum);
    }
}

/*!
 * @brief Configures the behavior of the PWM outputs when a fault is detected
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of fault output
 *                   - true : Output pins are set tri-state,
 *                   - false: Pins are set to a safe state determined by POL bits
 */
static inline void FTM_DRV_SetPwmFaultBehavior(FTM_Type * const ftmBase,
                                               bool enable)
{
    if (enable)
    {
        ((ftmBase)->FLTCTRL) |=  (1UL << FTM_FLTCTRL_FSTATE_SHIFT);
    }
    else
    {
        ((ftmBase)->FLTCTRL) &=  ~(1UL << FTM_FLTCTRL_FSTATE_SHIFT);
    }
}

/*!
 * @brief Writes the provided value to the Inverting control register.
 *
 * This function is enable/disable inverting control on multiple channel pairs.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] regVal Value to be written to the register
 */
static inline void FTM_DRV_SetInvctrlReg(FTM_Type * const ftmBase,
                                         uint32_t regVal)
{
    ((ftmBase)->INVCTRL) = regVal;
}

/*FTM software output control*/
/*!
 * @brief Enables or disables the channel software output control.The
 * main difference between this function and FTM_HAL_SetChnSoftwareCtrlCmd
 * is that this can configure all the channels in the same time.
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channelsMask Channels to be enabled or disabled
 */
static inline void FTM_DRV_SetAllChnSoftwareCtrlCmd(FTM_Type * const ftmBase,
                                                    uint8_t channelsMask)
{
    uint32_t mask = FTM_SWOCTRL_CH0OC_MASK | FTM_SWOCTRL_CH1OC_MASK | FTM_SWOCTRL_CH2OC_MASK |
                    FTM_SWOCTRL_CH3OC_MASK | FTM_SWOCTRL_CH4OC_MASK | FTM_SWOCTRL_CH5OC_MASK |
                    FTM_SWOCTRL_CH6OC_MASK | FTM_SWOCTRL_CH7OC_MASK;
    ((ftmBase)->SWOCTRL) = (((ftmBase)->SWOCTRL) & (~(mask))) | channelsMask;
}

/*!
 * @brief Sets the channel software output control value.
 *
 * @param[in] ftmBase The FTM base address pointer.
 * @param[in] channelsValues The values which will overwrite the output channels
 */
static inline void FTM_DRV_SetAllChnSoftwareCtrlVal(FTM_Type * const ftmBase,
                                                    uint8_t channelsValues)
{
    uint32_t mask = FTM_SWOCTRL_CH0OCV_MASK | FTM_SWOCTRL_CH1OCV_MASK | FTM_SWOCTRL_CH2OCV_MASK |
                        FTM_SWOCTRL_CH3OCV_MASK | FTM_SWOCTRL_CH4OCV_MASK | FTM_SWOCTRL_CH5OCV_MASK |
                        FTM_SWOCTRL_CH6OCV_MASK | FTM_SWOCTRL_CH7OCV_MASK;
   ((ftmBase)->SWOCTRL) = (((ftmBase)->SWOCTRL) & (~(mask))) | ((uint32_t)channelsValues << FTM_SWOCTRL_CH0OCV_SHIFT);
}

/*!
 * @brief Sets the BDM mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] val The FTM behavior in BDM mode
 *                - FTM_BDM_MODE_00: FTM counter stopped, CH(n)F bit can be set, FTM channels
 *                                   in functional mode, writes to MOD,CNTIN and C(n)V registers bypass
 *                                   the register buffers
 *                - FTM_BDM_MODE_01: FTM counter stopped, CH(n)F bit is not set, FTM channels
 *                                   outputs are forced to their safe value , writes to MOD,CNTIN and
 *                                   C(n)V registers bypass the register buffers
 *                - FTM_BDM_MODE_10: FTM counter stopped, CH(n)F bit is not set, FTM channels
 *                                   outputs are frozen when chip enters in BDM mode, writes to MOD,
 *                                   CNTIN and C(n)V registers bypass the register buffers
 *                - FTM_BDM_MODE_11: FTM counter in functional mode, CH(n)F bit can be set,
 *                                   FTM channels in functional mode, writes to MOD,CNTIN and C(n)V
 *                                   registers is in fully functional mode
 */
static inline void FTM_DRV_SetBdmMode(FTM_Type * const ftmBase,
                                      ftm_bdm_mode_t val)
{
    FTM_RMW_CONF(ftmBase, FTM_CONF_BDMMODE_MASK, FTM_CONF_BDMMODE(val));
}

/*FTM Synchronization configuration*/
/*!
 * @brief Sets the sync mode for the FTM SWOCTRL register when using a hardware trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of software output control synchronization
 *                   - true : The hardware trigger activates SWOCTRL register sync
 *                   - false: The hardware trigger does not activate SWOCTRL register sync
 */
static inline void FTM_DRV_SetSwoctrlHardwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWSOC_MASK) | FTM_SYNCONF_HWSOC(enable);
}

/*!
 * @brief Sets sync mode for FTM INVCTRL register when using a hardware trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of inverting control synchronization
 *                   - true : The hardware trigger activates INVCTRL register sync
 *                   - false: The hardware trigger does not activate INVCTRL register sync
 */
static inline void FTM_DRV_SetInvctrlHardwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWINVC_MASK) | FTM_SYNCONF_HWINVC(enable);
}

/*!
 * @brief Sets sync mode for FTM OUTMASK register when using a hardware trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of output mask synchronization
 *                   - true : The hardware trigger activates OUTMASK register sync
 *                   - false: The hardware trigger does not activate OUTMASK register sync
 */
static inline void FTM_DRV_SetOutmaskHardwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWOM_MASK) | FTM_SYNCONF_HWOM(enable);
}

/*!
 * @brief Sets sync mode for FTM MOD, CNTIN and CV registers when using a hardware trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of registers synchronization
 *                   - true : The hardware trigger activates  MOD, HCR, CNTIN, and CV registers sync
 *                   - false: The hardware trigger does not activate MOD, HCR, CNTIN, and CV registers sync
 */
static inline void FTM_DRV_SetModCntinCvHardwareSyncModeCmd(FTM_Type * const ftmBase,
                                                            bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWWRBUF_MASK) | FTM_SYNCONF_HWWRBUF(enable);
}

/*!
 * @brief Sets sync mode for FTM counter register when using a hardware trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of FTM counter synchronization
 *                   - true : The hardware trigger activates FTM counter sync
 *                   - false: The hardware trigger does not activate FTM counter sync
 */
static inline void FTM_DRV_SetCounterHardwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWRSTCNT_MASK) | FTM_SYNCONF_HWRSTCNT(enable);
}

/*!
 * @brief Sets sync mode for FTM SWOCTRL register when using a software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of software output control synchronization
 *                   - true : The software trigger activates SWOCTRL register sync
 *                   - false: The software trigger does not activate SWOCTRL register sync
 */
static inline void FTM_DRV_SetSwoctrlSoftwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWSOC_MASK) | FTM_SYNCONF_SWSOC(enable);
}

/*!
 * @brief Sets sync mode for FTM INVCTRL register when using a software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of State of inverting control synchronization
 *                   - true : The software trigger activates INVCTRL register sync
 *                   - false: The software trigger does not activate INVCTRL register sync
 */
static inline void FTM_DRV_SetInvctrlSoftwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWINVC_MASK) | FTM_SYNCONF_SWINVC(enable);
}

/*!
 * @brief Sets sync mode for FTM OUTMASK register when using a software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of output mask synchronization
 *                   - true : The software trigger activates OUTMASK register sync
 *                   - false: The software trigger does not activate OUTMASK register sync
 */
static inline void FTM_DRV_SetOutmaskSoftwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWOM_MASK) | FTM_SYNCONF_SWOM(enable);
}

/*!
 * @brief Sets sync mode for FTM MOD, CNTIN and CV registers when using a software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of registers synchronization
 *                   - true : The software trigger activates FTM MOD, CNTIN and CV registers sync
 *                   - false: The software trigger does not activate FTM MOD, CNTIN and CV registers sync
 */
static inline void FTM_DRV_SetModCntinCvSoftwareSyncModeCmd(FTM_Type * const ftmBase,
                                                            bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWWRBUF_MASK) | FTM_SYNCONF_SWWRBUF(enable);
}

/*!
 * @brief Sets hardware trigger mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] enable State of hardware trigger mode
 *                   - true : FTM does not clear the TRIGx bit when the hardware trigger j is detected
 *                   - false: FTM clears the TRIGx bit when the hardware trigger j is detected
 */
static inline void FTM_DRV_SetHwTriggerSyncModeCmd(FTM_Type * const ftmBase,
                                                   bool enable)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_HWTRIGMODE_MASK) | FTM_SYNCONF_HWTRIGMODE(enable);
}

/*!
 * @brief Sets sync mode for FTM counter register when using a software trigger.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] update_mode State of FTM counter synchronization
 *                   - true : The software trigger activates FTM counter sync
 *                   - false: The software trigger does not activate FTM counter sync
 */
static inline void FTM_DRV_SetCounterSoftwareSyncModeCmd(FTM_Type * const ftmBase,
                                                         ftm_pwm_sync_mode_t update_mode)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWRSTCNT_MASK) | FTM_SYNCONF_SWRSTCNT(update_mode);
}

/*!
 * @brief Sets the PWM synchronization mode to enhanced or legacy.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode State of synchronization mode
 *                   - true : Enhanced PWM synchronization is selected
 *                   - false: Legacy PWM synchronization is selected
 */
static inline void FTM_DRV_SetPwmSyncModeCmd(FTM_Type * const ftmBase,
                                             bool mode)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SYNCMODE_MASK) | FTM_SYNCONF_SYNCMODE(mode);
}

/*!
 * @brief Sets the SWOCTRL register PWM synchronization mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode State of register synchronization
 *                   - true : SWOCTRL register is updated by PWM sync
 *                   - false: SWOCTRL register is updated at all rising edges of system clock
 */
static inline void FTM_DRV_SetSwoctrlPwmSyncModeCmd(FTM_Type * const ftmBase,
                                                    ftm_reg_update_t mode)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_SWOC_MASK) | FTM_SYNCONF_SWOC(mode);
}

/*!
 * @brief Sets the INVCTRL register PWM synchronization mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode State of register synchronization
 *                   - true : INVCTRL register is updated by PWM sync
 *                   - false: INVCTRL register is updated at all rising edges of system clock
 */
static inline void FTM_DRV_SetInvctrlPwmSyncModeCmd(FTM_Type * const ftmBase,
                                                    ftm_reg_update_t mode)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_INVC_MASK) | FTM_SYNCONF_INVC(mode);
}

/*!
 * @brief Sets the CNTIN register PWM synchronization mode.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] mode State of register synchronization
 *                   - true : CNTIN register is updated by PWM sync
 *                   - false: CNTIN register is updated at all rising edges of system clock
 */
static inline void FTM_DRV_SetCntinPwmSyncModeCmd(FTM_Type * const ftmBase,
                                                  ftm_reg_update_t mode)
{
    ftmBase->SYNCONF = (ftmBase->SYNCONF & ~FTM_SYNCONF_CNTINC_MASK) | FTM_SYNCONF_CNTINC(mode);
}

#if FEATURE_FTM_HAS_SUPPORTED_DITHERING
/*!
 * @brief Sets the modulo fractional value in the PWM dithering.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] value The value to be set to the fractional value for the modulo
 */
static inline void FTM_DRV_SetModFracVal(FTM_Type * const ftmBase,
                                         uint8_t value)
{
    FTM_RMW_MOD_MIRROR(ftmBase, FTM_MOD_MIRROR_FRACMOD_MASK, FTM_MOD_MIRROR_FRACMOD(value));
}

/*!
 * @brief Sets the channel (n) match fractional value.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel (n)
 * @param[in] value The value to be set to the channel (n) match fractional value
 */
static inline void FTM_DRV_SetChnMatchFracVal(FTM_Type * const ftmBase,
                                              uint8_t channel,
                                              uint8_t value)
{
    FTM_RMW_CnV_MIRROR(ftmBase, channel, FTM_CV_MIRROR_FRACVAL_MASK, FTM_CV_MIRROR_FRACVAL(value));
}
#endif

/* DRV functionality */
/*!
 * @brief Resets the FTM registers. All the register use in the driver should be
 * reset to default value of each register.
 *
 * @param[in] ftmBase The FTM base address pointer
 */
void FTM_DRV_Reset(FTM_Type * const ftmBase);

/*!
 * @brief Initializes the FTM. This function will enable the flexTimer module
 * and selects one pre-scale factor for the FTM clock source.
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] ftmClockPrescaler The FTM peripheral clock pre-scale divider
 */
void FTM_DRV_InitModule(FTM_Type * const ftmBase,
                        ftm_clock_ps_t ftmClockPrescaler);

/*!
 * @brief Enables or disables the generation of the FTM peripheral timer channel trigger when the
 * FTM counter is equal to its initial value
 *
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number
 * @param[in] enable Enables the generation of the channel trigger
 *                   - true : The generation of the channel trigger is enabled
 *                   - false: The generation of the channel trigger is disabled
 */
void FTM_DRV_SetChnTriggerCmd(FTM_Type * const ftmBase,
                              uint8_t channel,
                              bool enable);

/*!
 * @brief Sets the FTM peripheral timer channel input capture filter value.
 * @param[in] ftmBase The FTM base address pointer
 * @param[in] channel The FTM peripheral channel number, only 0,1,2,3, channel 4, 5,6, 7 don't have
 * @param[in] value Filter value to be set
 */
void FTM_DRV_SetChnInputCaptureFilter(FTM_Type * const ftmBase,
                                      uint8_t channel,
                                      uint8_t value);

#if defined(__cplusplus)
}
#endif

/*! @}*/


#endif /* FTM_HW_ACCESS_H */

/*******************************************************************************
 * EOF
 ******************************************************************************/
