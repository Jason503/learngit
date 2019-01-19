/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED  WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER
 * RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF
 * CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
/*!
 * @file ftm_ic_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
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
 */

#include "ftm_ic_driver.h"


/*******************************************************************************
 * Code
 ******************************************************************************/
static void FTM_DRV_InputCaptureHandler(uint32_t instance,
                                        uint8_t channelPair);

static void FTM_DRV_IrqHandler(uint32_t instance,
                               uint8_t channelPair);

void FTM0_Ch0_Ch1_IRQHandler(void);

void FTM0_Ch2_Ch3_IRQHandler(void);

void FTM0_Ch4_Ch5_IRQHandler(void);

void FTM0_Ch6_Ch7_IRQHandler(void);

void FTM1_Ch0_Ch1_IRQHandler(void);

void FTM1_Ch2_Ch3_IRQHandler(void);

void FTM1_Ch4_Ch5_IRQHandler(void);

void FTM1_Ch6_Ch7_IRQHandler(void);

void FTM2_Ch0_Ch1_IRQHandler(void);

void FTM2_Ch2_Ch3_IRQHandler(void);

void FTM2_Ch4_Ch5_IRQHandler(void);

void FTM2_Ch6_Ch7_IRQHandler(void);

void FTM3_Ch0_Ch1_IRQHandler(void);

void FTM3_Ch2_Ch3_IRQHandler(void);

void FTM3_Ch4_Ch5_IRQHandler(void);

void FTM3_Ch6_Ch7_IRQHandler(void);

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_InitInputCapture
 * Description   : Configures Channel Input Capture for either getting time-stamps on edge detection
 * or on signal measurement . When the edge specified in the captureMode
 * argument occurs on the channel the FTM counter is captured into the CnV register.
 * The user will have to read the CnV register separately to get this value. The filter
 * function is disabled if the filterVal argument passed in is 0. The filter function
 * is available only on channels 0,1,2,3.
 *
 * Implements    : FTM_DRV_InitInputCapture_Activity
 *END**************************************************************************/
status_t FTM_DRV_InitInputCapture(uint32_t instance,
                                  const ftm_input_param_t * param)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(param != NULL);
    FTM_Type * ftmBase = g_ftmBase[instance];
    uint8_t chnlPairNum = 0U;
    uint8_t index = 0U;
    uint8_t hwChannel = 0U;
    ftm_signal_measurement_mode_t measurementType;
    ftm_state_t * state = ftmStatePtr[instance];
    status_t retStatus = STATUS_SUCCESS;

    if ((NULL != state) && (FTM_MODE_NOT_INITIALIZED == state->ftmMode))
    {
        FTM_DRV_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
        FTM_DRV_SetCounterInitVal(ftmBase, 0U);
        FTM_DRV_SetMod(ftmBase, param->nMaxCountValue);
        FTM_DRV_SetCpwms(ftmBase, false);
        /* Disable the combine mode */
        FTM_DRV_SetDualChnMofCombineCmd(ftmBase, chnlPairNum, false);
        FTM_DRV_SetDualChnCombineCmd(ftmBase, chnlPairNum, false);

        for (index = 0U; index < param->nNumChannels; index++)
        {
            hwChannel = param->inputChConfig[index].hwChannelId;
            chnlPairNum =  (uint8_t)(hwChannel >> 1U);
            /* Save in state structure user define handlers */
            state->channelsCallbacksParams[hwChannel] =  param->inputChConfig[index].channelsCallbacksParams;
            state->channelsCallbacks[hwChannel] = param->inputChConfig[index].channelsCallbacks;
            /* Enable filtering for input channels */
            if (hwChannel < CHAN4_IDX)
            {
                if (true == param->inputChConfig[index].filterEn)
                {
                    FTM_DRV_SetChnInputCaptureFilter(ftmBase, hwChannel, (uint8_t)param->inputChConfig[index].filterValue);
                }
                else
                {
                    FTM_DRV_SetChnInputCaptureFilter(ftmBase, hwChannel, 0U);
                }
            }

            if (FTM_EDGE_DETECT == param->inputChConfig[index].inputMode)
            {
                /* Disable the dual edge mode */
                FTM_DRV_SetDualEdgeCaptureCmd(ftmBase, chnlPairNum, false);
                /* Set input capture mode */
                FTM_DRV_SetChnMSnBAMode(ftmBase, hwChannel, 0U);
                /* Check if no edge is selected */
                DEV_ASSERT (FTM_NO_PIN_CONTROL != param->inputChConfig[index].edgeAlignement);
                /* Set the event which will generate the interrupt */
                FTM_DRV_SetChnEdgeLevel(ftmBase, hwChannel, (uint8_t)param->inputChConfig[index].edgeAlignement);
                /* Enable interrupt request for the current channel */
                FTM_DRV_EnableChnInt(ftmBase, hwChannel);
                INT_SYS_EnableIRQ(g_ftmIrqId[instance][hwChannel]);
            }
            else if (FTM_SIGNAL_MEASUREMENT == param->inputChConfig[index].inputMode)
            {
                /* Enable the dual edge mode */
                FTM_DRV_SetDualEdgeCaptureCmd(ftmBase, chnlPairNum, true);
                /* Enable dual edge input capture */
                FTM_DRV_SetDualChnDecapCmd(ftmBase, chnlPairNum, true);
                /* If continuous mode is set*/
                if (true == param->inputChConfig[index].continuousModeEn)
                {
                    /* Set MSnA and MSnB bit*/
                    FTM_DRV_SetChnMSnBAMode(ftmBase, hwChannel, 3U);
                }
                else
                {
                    /* Clear MSnA and Set MSnB bit*/
                    FTM_DRV_SetChnMSnBAMode(ftmBase, hwChannel, 2U);
                }

                measurementType = param->inputChConfig[index].measurementType;
                /* Check If want to measure a pulse width or period of the signal */
                if ((FTM_PERIOD_ON_MEASUREMENT == measurementType) || (FTM_RISING_EDGE_PERIOD_MEASUREMENT== measurementType))
                {
                    FTM_DRV_SetChnEdgeLevel(ftmBase, hwChannel, 1U);
                    if (FTM_PERIOD_ON_MEASUREMENT == measurementType)
                    {
                        /* Measure time between rising and falling edge - positive duty */
                        FTM_DRV_SetChnEdgeLevel(ftmBase, (uint8_t)(hwChannel + 1U), 2U);
                    }
                    else
                    {
                        /* If channel (n) is configured to capture falling edges (ELS(n)B:ELS(n)A = 0:1)
                         * then channel (n+1) also captures falling edges (ELS(n+1)B:ELS(n+1)A = 0:1) */
                        FTM_DRV_SetChnEdgeLevel(ftmBase, (uint8_t)(hwChannel + 1U), 1U);
                    }
                }
                else if ((FTM_PERIOD_OFF_MEASUREMENT == measurementType) || (FTM_FALLING_EDGE_PERIOD_MEASUREMENT == measurementType))
                {
                    FTM_DRV_SetChnEdgeLevel(ftmBase, hwChannel, 2U);
                    if (FTM_PERIOD_OFF_MEASUREMENT == measurementType)
                    {
                        /* Measure time between falling and rising edge - negative duty */
                        FTM_DRV_SetChnEdgeLevel(ftmBase, (uint8_t)(hwChannel + 1U), 1U);
                    }
                    else
                    {
                        /* If channel (n) is configured to capture rising edges (ELS(n)B:ELS(n)A = 1:0) than
                         * channel (n+1) is setup to capture also raising edges (ELS(n+1)B:ELS(n+1)A = 1:0) */
                        FTM_DRV_SetChnEdgeLevel(ftmBase, (uint8_t)(hwChannel + 1U), 2U);
                    }
                }
                else
                {
                    retStatus = STATUS_ERROR;
                    break;
                }

                /* Enable the interrupt request for the channel which will indicate that the measurement is done. */
                FTM_DRV_EnableChnInt(ftmBase, (uint8_t)(hwChannel + 1U));
                INT_SYS_EnableIRQ(g_ftmIrqId[instance][hwChannel]);
            }
            else
            {
                /* Do nothing */
            }
        }

        if (STATUS_SUCCESS == retStatus)
        {
            state->ftmMode = FTM_MODE_INPUT_CAPTURE;
            /* Set clock source to start the counter */
            FTM_DRV_SetClockSource(ftmBase, state->ftmClockSource);
        }
    }
    else
    {
        retStatus = STATUS_ERROR;
    }

    return retStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_DeinitInputCapture
 * Description   : Disables Channel Input Capture
 *
 * Implements    : FTM_DRV_DeinitInputCapture_Activity
 *END**************************************************************************/
status_t FTM_DRV_DeinitInputCapture(uint32_t instance,
                                    const ftm_input_param_t * param)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(param != NULL);
    FTM_Type * ftmBase = g_ftmBase[instance];
    uint8_t chnlPairNum = 0U;
    uint8_t index = 0U;
    uint8_t hwChannel = 0U;
    ftm_state_t * state = ftmStatePtr[instance];

    /* FTM counter is disabled */
    FTM_DRV_SetClockSource(ftmBase, FTM_CLOCK_SOURCE_NONE);
    FTM_DRV_SetCounterInitVal(ftmBase, 0U);
    FTM_DRV_SetMod(ftmBase, 0xFFFFU);
    FTM_DRV_SetCpwms(ftmBase, false);
    for (index = 0U; index < param->nNumChannels; index++)
    {
        hwChannel = param->inputChConfig[index].hwChannelId;
        chnlPairNum =  (uint8_t)(hwChannel >> 1U);
        /* Disable filtering for input channels */
        if (hwChannel < CHAN4_IDX)
        {
            FTM_DRV_SetChnInputCaptureFilter(ftmBase, hwChannel, 0U);
        }

        FTM_DRV_SetDualChnCombineCmd(ftmBase, chnlPairNum, false);
        FTM_DRV_SetDualEdgeCaptureCmd(ftmBase, chnlPairNum, false);
        FTM_DRV_SetChnEdgeLevel(ftmBase, hwChannel, (uint8_t)0U);
        FTM_DRV_DisableChnInt(ftmBase, hwChannel);
    }

    /* Clear Callbacks function from the state structure */
    for (index = 0U; index < FEATURE_FTM_CHANNEL_COUNT; index++)
    {
        state->channelsCallbacksParams[index] =  NULL;
        state->channelsCallbacks[index] = NULL;
    }

    state->ftmMode = FTM_MODE_NOT_INITIALIZED;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_GetInputCaptureMeasurement
 * Description   : This function is used to calculate the measurement and/or time stamps values
 * which are read from the C(n, n+1)V registers and stored to the static buffers.
 *
 * Implements    : FTM_DRV_GetInputCaptureMeasurement_Activity
 *END**************************************************************************/
uint16_t FTM_DRV_GetInputCaptureMeasurement(uint32_t instance,
                                            uint8_t channel)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);
    const ftm_state_t * state = ftmStatePtr[instance];

    return state->measurementResults[channel];
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FTM_DRV_StartNewSignalMeasurement
 * Description   : This function starts new Signal Measurements on a dual input compare channel
 * that is configured as single-shot measurement.
 *
 * Implements    : FTM_DRV_StartNewSignalMeasurement_Activity
 *END**************************************************************************/
status_t FTM_DRV_StartNewSignalMeasurement(uint32_t instance,
                                           uint8_t channel)
{
    DEV_ASSERT(instance < FTM_INSTANCE_COUNT);
    DEV_ASSERT(channel < FEATURE_FTM_CHANNEL_COUNT);
    /* Clear CH(n)F and CH(n+1)F  flags and Set DECAP bit */
    FTM_Type * ftmBase = g_ftmBase[instance];
    uint8_t chnlPairNum = (uint8_t)(channel >> 1U);

    /* Get channel mode */
    if (FTM_FEATURE_INPUT_CAPTURE_SINGLE_SHOT == FTM_DRV_GetChnMode(ftmBase, channel))
    {
        if (FTM_DRV_GetDualChnCombineCmd(ftmBase, chnlPairNum))
        {
            /* Clear event flags for channel n and n + 1 */
            FTM_DRV_ClearChnEventFlag(ftmBase, (uint8_t)(channel + 1U));
            FTM_DRV_ClearChnEventFlag(ftmBase, channel);
            /* Set DECAP bit to start measurement */
            FTM_DRV_SetDualChnDecapCmd(ftmBase, chnlPairNum, true);
        }
    }
    else
    {
        /* Nothing to do */
    }

    return STATUS_SUCCESS;
}

/* Implementation of FTM0_Ch0_Ch1_IRQHandler master handler named in startup code. */
void FTM0_Ch0_Ch1_IRQHandler(void)
{
    FTM_DRV_IrqHandler(0U, 0U);
}

/* Implementation of FTM0_Ch2_Ch3_IRQHandler master handler named in startup code. */
void FTM0_Ch2_Ch3_IRQHandler(void)
{
    FTM_DRV_IrqHandler(0U, 1U);
}

/* Implementation of FTM0_Ch4_Ch5_IRQHandler master handler named in startup code. */
void FTM0_Ch4_Ch5_IRQHandler(void)
{
    FTM_DRV_IrqHandler(0U, 2U);
}

/* Implementation of FTM0_Ch6_Ch7_IRQHandler master handler named in startup code. */
void FTM0_Ch6_Ch7_IRQHandler(void)
{
    FTM_DRV_IrqHandler(0U, 3U);
}

/* Implementation of FTM1_Ch0_Ch1_IRQHandler master handler named in startup code. */
void FTM1_Ch0_Ch1_IRQHandler(void)
{
    FTM_DRV_IrqHandler(1U, 0U);
}

/* Implementation of FTM1_Ch2_Ch3_IRQHandler master handler named in startup code. */
void FTM1_Ch2_Ch3_IRQHandler(void)
{
    FTM_DRV_IrqHandler(1U, 1U);
}

/* Implementation of FTM1_Ch4_Ch5_IRQHandler master handler named in startup code. */
void FTM1_Ch4_Ch5_IRQHandler(void)
{
    FTM_DRV_IrqHandler(1U, 2U);
}

/* Implementation of FTM1_Ch6_Ch7_IRQHandler master handler named in startup code. */
void FTM1_Ch6_Ch7_IRQHandler(void)
{
    FTM_DRV_IrqHandler(1U, 3U);
}

/* Implementation of FTM2_Ch0_Ch1_IRQHandler master handler named in startup code. */
void FTM2_Ch0_Ch1_IRQHandler(void)
{
    FTM_DRV_IrqHandler(2U, 0U);
}

/* Implementation of FTM2_Ch2_Ch3_IRQHandler master handler named in startup code. */
void FTM2_Ch2_Ch3_IRQHandler(void)
{
    FTM_DRV_IrqHandler(2U, 1U);
}

/* Implementation of FTM2_Ch4_Ch5_IRQHandler master handler named in startup code. */
void FTM2_Ch4_Ch5_IRQHandler(void)
{
    FTM_DRV_IrqHandler(2U, 2U);
}

/* Implementation of FTM2_Ch6_Ch7_IRQHandler master handler named in startup code. */
void FTM2_Ch6_Ch7_IRQHandler(void)
{
    FTM_DRV_IrqHandler(2U, 3U);
}

/* Implementation of FTM3_Ch0_Ch1_IRQHandler master handler named in startup code. */
void FTM3_Ch0_Ch1_IRQHandler(void)
{
    FTM_DRV_IrqHandler(3U, 0U);
}

/* Implementation of FTM3_Ch2_Ch3_IRQHandler master handler named in startup code. */
void FTM3_Ch2_Ch3_IRQHandler(void)
{
    FTM_DRV_IrqHandler(3U, 1U);
}

/* Implementation of FTM3_Ch4_Ch5_IRQHandler master handler named in startup code. */
void FTM3_Ch4_Ch5_IRQHandler(void)
{
    FTM_DRV_IrqHandler(3U, 2U);
}

/* Implementation of FTM3_Ch6_Ch7_IRQHandler master handler named in startup code. */
void FTM3_Ch6_Ch7_IRQHandler(void)
{
    FTM_DRV_IrqHandler(3U, 3U);
}

static void FTM_DRV_IrqHandler(uint32_t instance,
                               uint8_t channelPair)
{
    const ftm_state_t * state = ftmStatePtr[instance];
    switch (state->ftmMode)
    {
        case FTM_MODE_INPUT_CAPTURE:
            FTM_DRV_InputCaptureHandler(instance, channelPair);
            break;
        default:
            /* Nothing to do */
            break;
    }
}

static void FTM_DRV_InputCaptureHandler(uint32_t instance,
                                        uint8_t channelPair)
{
    ftm_state_t * state = ftmStatePtr[instance];
    FTM_Type * ftmBase = g_ftmBase[instance];

    /* Verify the mode for current pair of channels */
    if (FTM_DRV_GetDualEdgeCaptureBit(ftmBase, channelPair))
    {
        /* Dual edge input capture case */
        uint16_t first_event_time = FTM_DRV_GetChnCountVal(ftmBase, (uint8_t)(channelPair << 1U));
        uint16_t second_event_time = FTM_DRV_GetChnCountVal(ftmBase, (uint8_t)((channelPair << 1U) + 1U));
        if (second_event_time < first_event_time)
        {
            /* Measurement when overflow occurred */
            state->measurementResults[channelPair << 1U] = (uint16_t)(second_event_time + (FTM_DRV_GetMod(ftmBase) - first_event_time));
        }
        else
        {
            /* Measurement when overflow doesn't occurred */
            state->measurementResults[channelPair << 1U] = (uint16_t)(second_event_time - first_event_time);
        }

        /* Clear flags for channels n and n+1 */
        FTM_DRV_ClearChnEventFlag(ftmBase, (uint8_t)(channelPair << 1U));
        FTM_DRV_ClearChnEventFlag(ftmBase, (uint8_t)((channelPair << 1U) + 1U));
        /* If the callback is define use it */
        if ((state->channelsCallbacks[(channelPair << 1U)]) != NULL)
        {
            state->channelsCallbacks[(channelPair << 1U)](state->channelsCallbacksParams[channelPair << 1U]);
        }
    }
    else
    {
        /* To get the channel interrupt source the both channels flag must be checked */
        if (FTM_DRV_HasChnEventOccurred(ftmBase, (uint8_t)(channelPair << 1U)))
        {
            /* Get the time stamp of the event */
            state->measurementResults[channelPair << 1U] = FTM_DRV_GetChnCountVal(ftmBase, (uint8_t)(channelPair << 1U));
            /* Clear the flag for C(n) channel */
            FTM_DRV_ClearChnEventFlag(ftmBase, (uint8_t)(channelPair << 1U));
            /* If the callback is define use it */
            if ((state->channelsCallbacks[channelPair << 1U]) != NULL)
            {
                state->channelsCallbacks[channelPair << 1U](state->channelsCallbacksParams[channelPair << 1U]);
            }
        }
        else
        {
            /* Get the time stamp of the event */
            state->measurementResults[(channelPair << 1U) + 1U] = FTM_DRV_GetChnCountVal(ftmBase, (uint8_t)((channelPair << 1U) + 1U));
            /* Clear the flag for C(n+1) channel */
            FTM_DRV_ClearChnEventFlag(ftmBase, (uint8_t)((channelPair << 1U) + 1U));
            /* If the callback is define use it */
            if ((state->channelsCallbacks[(channelPair << 1U) + 1U]) != NULL)
            {
                state->channelsCallbacks[(channelPair << 1U) + 1U](state->channelsCallbacksParams[(channelPair << 1U) + 1U]);
            }
        }
    }
}

/*******************************************************************************
* EOF
******************************************************************************/
