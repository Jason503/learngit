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
 * @file ftm_common.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable. A source of error in
 * writing dynamic code is that the stack segment may be different from the data
 * segment.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, external symbol defined without a prior
 * declaration.
 * The symbols are declared in the driver common file as external; they are needed
 * at driver initialization to install the correct interrupt handler, but are not
 * a part of the public API.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * The function is defined for use by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, could define variable at block scope
 * The variables are defined in the source file to make transition to other
 * platforms easier.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from
 * 'essentially enum<i>' to 'essentially Boolean'. This is required by
 * the conversion of a enum type into a bool type.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 */

#include "ftm_common.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Table of base addresses for FTM instances. */
FTM_Type * const g_ftmBase[FTM_INSTANCE_COUNT] = FTM_BASE_PTRS;

/*! @brief Interrupt vectors for the FTM peripheral. */
const IRQn_Type g_ftmIrqId[FTM_INSTANCE_COUNT][FEATURE_FTM_CHANNEL_COUNT] = FTM_IRQS;
const IRQn_Type g_ftmFaultIrqId[FTM_INSTANCE_COUNT] = FTM_Fault_IRQS;
const IRQn_Type g_ftmOverflowIrqId[FTM_INSTANCE_COUNT] = FTM_Overflow_IRQS;
const IRQn_Type g_ftmReloadIrqId[FTM_INSTANCE_COUNT] = FTM_Reload_IRQS;

#ifdef ERRATA_E10856
bool faultDetection = false;
#endif

/*! @brief Pointer to runtime state structure. */
ftm_state_t * ftmStatePtr[FTM_INSTANCE_COUNT] = {NULL};

/*! @brief  Select external clock pin or clock source for peripheral */
static const clock_names_t g_ftmExtClockSel[FTM_INSTANCE_COUNT][2] = {{SIM_FTM0_CLOCKSEL, FTM0_CLK},
                                                                      {SIM_FTM1_CLOCKSEL, FTM1_CLK},
                                                                      {SIM_FTM2_CLOCKSEL, FTM2_CLK},
                                                                      {SIM_FTM3_CLOCKSEL, FTM3_CLK},
#if (FTM_INSTANCE_COUNT > 4U)
                                                                      {SIM_FTM4_CLOCKSEL, FTM4_CLK},
#endif
#if (FTM_INSTANCE_COUNT > 5U)
                                                                      {SIM_FTM5_CLOCKSEL, FTM5_CLK},
#endif
#if (FTM_INSTANCE_COUNT > 6U)
                                                                      {SIM_FTM6_CLOCKSEL, FTM6_CLK},
#endif
#if (FTM_INSTANCE_COUNT > 7U)
                                                                      {SIM_FTM7_CLOCKSEL, FTM7_CLK},
#endif
                                                                      };

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_Init
 * Description   : Initializes the FTM driver and get the clock frequency value
 * which select one of three possible clock sources for the FTM counter.
 *
 * Implements    : FTM_DRV_Init_Activity
 *END**************************************************************************/
status_t FTM_DRV_Init(uint32_t instance,
                      const ftm_user_config_t * info,
                      ftm_state_t * state)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(info != NULL);
    DEV_ASSERT(state != NULL);
    FTM_Type * ftmBase = g_ftmBase[instance];
    status_t status = STATUS_SUCCESS;

    /* Check if this instance is already initialized */
    if (ftmStatePtr[instance] != NULL)
    {
        status = STATUS_ERROR;
    }
    else
    {
        /* Configure state structure. */
        state->ftmClockSource = info->ftmClockSource;
        state->ftmMode = FTM_MODE_NOT_INITIALIZED;
        state->ftmPeriod = 0U;
        ftmStatePtr[instance] = state;
        /* The reset operation doesn't care about write protection. FTM_DRV_Reset will
         * disable this protection.*/
        FTM_DRV_Reset(ftmBase);
        FTM_DRV_InitModule(ftmBase, info->ftmPrescaler);
        /* Get clock name used to configure the FlexTimer module */
        state->ftmSourceClockFrequency = FTM_DRV_GetFrequency(instance);
        /* Check the functional clock is selected for FTM */
        DEV_ASSERT(state->ftmSourceClockFrequency > 0U);
    }

    if (STATUS_SUCCESS == status)
    {
        /* Check if the mode operation in PWM mode */
        if ((FTM_MODE_EDGE_ALIGNED_PWM == info->ftmMode) || (FTM_MODE_CEN_ALIGNED_PWM == info->ftmMode) || (FTM_MODE_OUTPUT_COMPARE == info->ftmMode))
        {
            /* Configure sync for between registers and buffers */
            status = FTM_DRV_SetSync(instance, &(info->syncMethod));
        }

        /* Enable the generation of initialization trigger on chip module */
        FTM_DRV_SetInitTriggerCmd(ftmBase, info->enableInitializationTrigger);
        FTM_DRV_SetBdmMode(ftmBase, info->BDMMode);

        /* Check if enable interrupt in counter mode */
        if (info->isTofIsrEnabled)
        {
            FTM_DRV_SetTimerOverflowInt(ftmBase, true);
            INT_SYS_EnableIRQ(g_ftmOverflowIrqId[instance]);
        }
        else
        {
            FTM_DRV_SetTimerOverflowInt(ftmBase, false);
            INT_SYS_DisableIRQ(g_ftmOverflowIrqId[instance]);
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_Deinit
 * Description   : Shuts down the FTM driver.
 * First, FTM_DRV_Init must be called. Then this function will disables the FTM module.
 *
 * Implements    : FTM_DRV_Deinit_Activity
 *END**************************************************************************/
status_t FTM_DRV_Deinit(uint32_t instance)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    /* Reset all FTM register */
    FTM_DRV_Reset(ftmBase);
    ftmStatePtr[instance] = NULL;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_MaskOutputChannels
 * Description   : This function will mask the output of the channels and at match
 * events will be ignored by the masked channels.
 *
 * Implements : FTM_DRV_MaskOutputChannels_Activity
 *END**************************************************************************/
status_t FTM_DRV_MaskOutputChannels(uint32_t instance,
                                    uint32_t channelsMask,
                                    bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    FTM_DRV_SetOutmaskReg(ftmBase, channelsMask);
    if (softwareTrigger)
    {
        FTM_DRV_SetSoftwareTriggerCmd(ftmBase, true);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetInitialCounterValue
 * Description   : This function configure the initial counter value. The counter
 * will get this value after an overflow event.
 *
 * Implements : FTM_DRV_SetInitialCounterValue_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetInitialCounterValue(uint32_t instance,
                                        uint16_t counterValue,
                                        bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    FTM_DRV_SetCounterInitVal(ftmBase, counterValue);
    if (softwareTrigger)
    {
        FTM_DRV_SetSoftwareTriggerCmd(ftmBase, true);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetHalfCycleReloadPoint
 * Description   : This function configure the value of the counter which will
 * generates an reload point.
 *
 * Implements : FTM_DRV_SetHalfCycleReloadPoint_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetHalfCycleReloadPoint(uint32_t instance,
                                         uint16_t reloadPoint,
                                         bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    FTM_DRV_SetHalfCycleValue(ftmBase, reloadPoint);
    if (softwareTrigger)
    {
        FTM_DRV_SetSoftwareTriggerCmd(ftmBase, true);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetSoftwareOutputChannelValue
 * Description   : This function will force the output value of a channel to a specific value.
 * Before using this function it's mandatory to mask the match events using
 * FTM_DRV_MaskOutputChannels and to enable software output control using
 * FTM_DRV_SetSoftwareOutputChannelControl.
 *
 * Implements : FTM_DRV_SetSoftOutChnValue_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetSoftOutChnValue(uint32_t instance,
                                    uint8_t channelsValues,
                                    bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];
    FTM_DRV_SetAllChnSoftwareCtrlVal(ftmBase, channelsValues);
    if (softwareTrigger)
    {
        FTM_DRV_SetSoftwareTriggerCmd(ftmBase, true);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetSoftwareOutputChannelControl
 * Description   : This function will configure which output channel can be
 * software controlled.
 *
 * Implements : FTM_DRV_SetSoftwareOutputChannelControl_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetSoftwareOutputChannelControl(uint32_t instance,
                                                 uint8_t channelsMask,
                                                 bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];
    FTM_DRV_SetAllChnSoftwareCtrlCmd(ftmBase, channelsMask);
    if (softwareTrigger)
    {
        FTM_DRV_SetSoftwareTriggerCmd(ftmBase, true);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetInvertingControl
 * Description   : This function will configure if the second channel of a pair
 * will be inverted or not.
 *
 * Implements : FTM_DRV_SetInvertingControl_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetInvertingControl(uint32_t instance,
                                     uint8_t channelsPairMask,
                                     bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    FTM_DRV_SetInvctrlReg(ftmBase, channelsPairMask);
    if (softwareTrigger)
    {
        FTM_DRV_SetSoftwareTriggerCmd(ftmBase, true);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetModuloCounterValue
 * Description   : This function configure the maximum counter value.
 *
 * Implements : FTM_DRV_SetModuloCounterValue_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetModuloCounterValue(uint32_t instance,
                                       uint16_t counterValue,
                                       bool softwareTrigger)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type * ftmBase = g_ftmBase[instance];

    FTM_DRV_SetMod(ftmBase, counterValue);
    if (softwareTrigger)
    {
        FTM_DRV_SetSoftwareTriggerCmd(ftmBase, true);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_SetSync
 * Description   : This function configure the synchronization for PWM register
 * (CnV, MOD, CINT, HCR, OUTMASK).If this function is used whit wrong parameters
 * it's possible to generate wrong waveform. Registers synchronization need to
 * be configured for PWM and output compare mode.
 *
 * Implements : FTM_DRV_SetSync_Activity
 *END**************************************************************************/
status_t FTM_DRV_SetSync(uint32_t instance,
                         const ftm_pwm_sync_t * param)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(param != NULL);
    FTM_Type * ftmBase = g_ftmBase[instance];
    status_t retStatus = STATUS_SUCCESS;
    bool hardwareSync = param->hardwareSync0 || param->hardwareSync1 || param->hardwareSync2;

    /* Software and hardware triggers are not allowed in the same time */
    if ((param->softwareSync && hardwareSync) || (true != (param->softwareSync || hardwareSync)))
    {
        retStatus = STATUS_ERROR;
    }
    else if (param->softwareSync)
    {
        /* Configure sync for OUTMASK register */
        FTM_DRV_SetOutmaskSoftwareSyncModeCmd(ftmBase, true);
        /* Configure sync for INVCTRL register */
        FTM_DRV_SetInvctrlSoftwareSyncModeCmd(ftmBase, true);
        /* Configure sync for SWOCTRL register */
        FTM_DRV_SetSwoctrlSoftwareSyncModeCmd(ftmBase, true);
        /* Configure sync for MOD, HCR, CNTIN, and CnV registers */
        FTM_DRV_SetModCntinCvSoftwareSyncModeCmd(ftmBase, true);
        /* Configure synchronization method (waiting next loading point or now) */
        FTM_DRV_SetCounterSoftwareSyncModeCmd(ftmBase, param->syncPoint);
    }
    else
    {
        /* Configure sync for OUTMASK register */
        FTM_DRV_SetOutmaskHardwareSyncModeCmd(ftmBase, true);
        /* Configure sync for INVCTRL register */
        FTM_DRV_SetInvctrlHardwareSyncModeCmd(ftmBase, true);
        /* Configure sync for SWOCTRL register */
        FTM_DRV_SetSwoctrlHardwareSyncModeCmd(ftmBase, true);
        /* Configure sync for MOD, HCR, CNTIN, and CnV registers */
        FTM_DRV_SetModCntinCvHardwareSyncModeCmd(ftmBase, true);
        /* Configure synchronization method (waiting next loading point or now) */
        FTM_DRV_SetCounterHardwareSyncModeCmd(ftmBase, (bool)param->syncPoint);
    }

    if (STATUS_SUCCESS == retStatus)
    {
        /* Enhanced PWM sync is used */
        FTM_DRV_SetPwmSyncModeCmd(ftmBase, true);
        /* Configure trigger source for sync */
        FTM_DRV_SetHardwareSyncTriggerSrc(ftmBase, 0U, param->hardwareSync0);
        FTM_DRV_SetHardwareSyncTriggerSrc(ftmBase, 1U, param->hardwareSync1);
        FTM_DRV_SetHardwareSyncTriggerSrc(ftmBase, 2U, param->hardwareSync2);
        /* Configure loading points */
        FTM_DRV_SetMaxLoadingCmd(ftmBase, param->maxLoadingPoint);
        FTM_DRV_SetMinLoadingCmd(ftmBase, param->minLoadingPoint);
        /* Configure sync for OUTMASK register */
        FTM_DRV_SetOutmaskPwmSyncModeCmd(ftmBase, (bool)param->maskRegSync);
        /* Configure sync for INVCTRL register */
        FTM_DRV_SetInvctrlPwmSyncModeCmd(ftmBase, param->inverterSync);
        /* Configure sync for SWOCTRL register */
        FTM_DRV_SetSwoctrlPwmSyncModeCmd(ftmBase, param->outRegSync);
        /* Configure sync for MOD, HCR, CNTIN, and CnV registers */
        FTM_DRV_SetCntinPwmSyncModeCmd(ftmBase, param->initCounterSync);
        /* Configure if FTM clears TRIGj (j=0,1,2) when the hardware trigger j is detected. */
        FTM_DRV_SetHwTriggerSyncModeCmd(ftmBase, param->autoClearTrigger);
    }

    return retStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_GetFrequency
 * Description   : Retrieves the frequency of the clock source feeding the FTM counter.
 * Function will return a 0 if no clock source is selected and the FTM counter is disabled.
 * The returned value is clock sources for the FTM counter.
 *
 * Implements    : FTM_DRV_GetFrequency_Activity
 *END**************************************************************************/
uint32_t FTM_DRV_GetFrequency(uint32_t instance)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    FTM_Type const * ftmBase = g_ftmBase[instance];
    status_t returnCode = STATUS_SUCCESS;
    clock_names_t ftmClkName;
    uint8_t clkPs;
    uint32_t frequency = 0U;
    const ftm_state_t * state = ftmStatePtr[instance];
    clkPs = (uint8_t)(1U << FTM_DRV_GetClockPs(ftmBase));

    switch (state->ftmClockSource)
    {
        case FTM_CLOCK_SOURCE_EXTERNALCLK:
            returnCode = CLOCK_SYS_GetFreq(g_ftmExtClockSel[instance][1], &frequency);
            if (0U == frequency)
            {
                ftmClkName = g_ftmExtClockSel[instance][0];
            }
            else
            {
                ftmClkName = g_ftmExtClockSel[instance][1];
            }

            /* Get the clock frequency value */
            returnCode = CLOCK_SYS_GetFreq(ftmClkName, &frequency);
            break;
        case FTM_CLOCK_SOURCE_FIXEDCLK:
            /* Get the clock frequency value */
            returnCode = CLOCK_SYS_GetFreq(SIM_RTCCLK_CLK, &frequency);
            break;
        case FTM_CLOCK_SOURCE_SYSTEMCLK:
            /* Get the clock frequency value */
            returnCode = CLOCK_SYS_GetFreq(CORE_CLK, &frequency);
            break;
        default:
            /* Nothing to do */
            break;
    }

    /* Checks the functional clock of FTM module */
    (void)returnCode;
    DEV_ASSERT(returnCode == STATUS_SUCCESS);

    return (uint32_t)(frequency / clkPs);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_ConvertFreqToPeriodTicks
 * Description   : This function converts the input parameters representing
 * frequency in Hz to a period value in ticks needed by the hardware timer.
 *
 * Implements    : FTM_DRV_ConvertFreqToPeriodTicks_Activity
 *END**************************************************************************/
uint16_t FTM_DRV_ConvertFreqToPeriodTicks(uint32_t instance,
                                          uint32_t freqencyHz)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(freqencyHz != 0U);
    uint32_t uFTMhz;
    const ftm_state_t * state = ftmStatePtr[instance];
    uFTMhz = state->ftmSourceClockFrequency;

    return (uint16_t)(uFTMhz / freqencyHz);
}

/*******************************************************************************
* EOF
******************************************************************************/
