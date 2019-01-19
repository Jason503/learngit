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
#ifndef SAI_HW_ACCESS_H_
#define SAI_HW_ACCESS_H_

#include "device_registers.h"

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_RxSetWatermark
 * Description   : Set fifo watermark
 *
 *END**************************************************************************/
static inline void SAI_DRV_RxSetWatermark(SAI_Type* inst, uint8_t level)
{
    inst->RCR1 = SAI_RCR1_RFW((uint32_t) level);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_IsTxChannelEnabled
 * Description   : check if channel is enabled
 *
 *END**************************************************************************/
static inline bool SAI_DRV_IsTxChannelEnabled(const SAI_Type* inst, uint8_t channel)
{
    bool ret;

    if ((inst->TCR3 & SAI_TCR3_TCE(1UL << (uint32_t)channel)) != 0UL)
    {
        ret = true;
    }
    else
    {
        ret = false;
    }
    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_IsRxChannelEnabled
 * Description   : check if channel is enabled
 *
 *END**************************************************************************/
static inline bool SAI_DRV_IsRxChannelEnabled(const SAI_Type* inst, uint8_t channel)
{
    bool ret;

    if ((inst->RCR3 & SAI_RCR3_RCE((uint32_t)1UL << (uint32_t)channel)) != 0UL)
    {
        ret = true;
    }
    else
    {
        ret =false;
    }
    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_TxResetFifo
 * Description   : Reset fifo
 *
 *END**************************************************************************/
static inline void SAI_DRV_TxResetFifo(SAI_Type* inst)
{
    uint32_t val = inst->TCSR;
    /* careful not to clear flags */
    val &= ~((1UL << SAI_TCSR_WSF_SHIFT) | (1UL << SAI_TCSR_SEF_SHIFT) | (1UL << SAI_TCSR_FEF_SHIFT));
    /* enable fifo request int */
    val |= (1UL << SAI_TCSR_FR_SHIFT);
    inst->TCSR = val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_RxResetFifo
 * Description   : Reset fifo
 *
 *END**************************************************************************/
static inline void SAI_DRV_RxResetFifo(SAI_Type* inst)
{
    uint32_t val = inst->RCSR;
    /* careful not to clear flags */
    val &= ~((1UL << SAI_RCSR_WSF_SHIFT) | (1UL << SAI_RCSR_SEF_SHIFT) | (1UL << SAI_RCSR_FEF_SHIFT));
    /* enable fifo request int */
    val |= (1UL << SAI_RCSR_FR_SHIFT);
    inst->RCSR = val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_TxDisableFifoReqInt
 * Description   : Disable fifo request interrupt
 *
 *END**************************************************************************/
static inline void SAI_DRV_TxDisableFifoReqInt(SAI_Type* inst)
{
    uint32_t val = inst->TCSR;
    /* careful not to clear flags */
    val &= ~((1UL << SAI_TCSR_WSF_SHIFT) | (1UL << SAI_TCSR_SEF_SHIFT) | (1UL << SAI_TCSR_FEF_SHIFT));
    /* enable fifo request int */
    val &= ~(1UL << SAI_TCSR_FRIE_SHIFT);
    inst->TCSR = val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_TxClearFlag
 * Description   : Clear one of folowing flags: word start, fifo erro,
 *                 sync error
 *
 *END**************************************************************************/
static inline void SAI_DRV_TxClearFlag (SAI_Type* inst, uint32_t shift)
{
    uint32_t val = inst->TCSR;
    /* careful not to clear flags */
    val &= ~((1UL << SAI_TCSR_WSF_SHIFT) | (1UL << SAI_TCSR_SEF_SHIFT) | (1UL << SAI_TCSR_FEF_SHIFT));
    /* clear fifo error flag */
    val |= 1UL << shift;
    inst->TCSR = val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_RxClearFlag
 * Description   : Clear one of folowing flags: word start, fifo erro,
 *                 sync error
 *
 *END**************************************************************************/
static inline void SAI_DRV_RxClearFlag (SAI_Type* inst, uint32_t shift)
{
    uint32_t val = inst->RCSR;
    /* careful not to clear flags */
    val &= ~((1UL << SAI_RCSR_WSF_SHIFT) | (1UL << SAI_RCSR_SEF_SHIFT) | (1UL << SAI_RCSR_FEF_SHIFT));
    /* clear fifo error flag */
    val |= 1UL << shift;
    inst->RCSR = val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_RxEnableFifoReqInt
 * Description   : Enable fifo request interrupt
 *
 *END**************************************************************************/
static inline void SAI_DRV_RxEnableFifoReqInt(SAI_Type* inst)
{
    uint32_t val = inst->RCSR;

    /* careful not to clear flags */
    val &= ~((1UL << SAI_RCSR_WSF_SHIFT) | (1UL << SAI_RCSR_SEF_SHIFT) | (1UL << SAI_RCSR_FEF_SHIFT));
    /* enable fifo request int */
    val |= 1UL << SAI_RCSR_FRIE_SHIFT;
    inst->RCSR = val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_RxDisableFifoReqInt
 * Description   : Disable fifo request interrupt
 *
 *END**************************************************************************/
static inline void SAI_DRV_RxDisableFifoReqInt(SAI_Type* inst)
{
    uint32_t val = inst->RCSR;

    /* careful not to clear flags */
    val &= ~((1UL << SAI_RCSR_WSF_SHIFT) | (1UL << SAI_RCSR_SEF_SHIFT) | (1UL << SAI_RCSR_FEF_SHIFT));
    /* enable fifo request int */
    val &= ~(1UL << SAI_RCSR_FRIE_SHIFT);
    inst->RCSR = val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_RxEnableFifoReqDma
 * Description   : Enable fifo request dma
 *
 *END**************************************************************************/
static inline void SAI_DRV_RxEnableFifoReqDma(SAI_Type* inst)
{
    uint32_t val = inst->RCSR;

    /* careful not to clear flags */
    val &= ~((1UL << SAI_RCSR_WSF_SHIFT) | (1UL << SAI_RCSR_SEF_SHIFT) | (1UL << SAI_RCSR_FEF_SHIFT));
    /* enable fifo request int */
    val |= 1UL << SAI_RCSR_FRDE_SHIFT;
    inst->RCSR = val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_RxDisableFifoReqDma
 * Description   : Disable fifo request dma
 *
 *END**************************************************************************/
static inline void SAI_DRV_RxDisableFifoReqDma(SAI_Type* inst)
{
    uint32_t val = inst->RCSR;

    /* careful not to clear flags */
    val &= ~((1UL << SAI_RCSR_WSF_SHIFT) | (1UL << SAI_RCSR_SEF_SHIFT) | (1UL << SAI_RCSR_FEF_SHIFT));
    /* enable fifo request int */
    val &= ~(1UL << SAI_RCSR_FRDE_SHIFT);
    inst->RCSR = val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_TxEnableFifoReqInt
 * Description   : Enable fifo request interrupt
 *
 *END**************************************************************************/
static inline void SAI_DRV_TxEnableFifoReqInt(SAI_Type* inst)
{
    uint32_t val = inst->TCSR;

    /* careful not to clear flags */
    val &= ~((1UL << SAI_TCSR_WSF_SHIFT) | (1UL << SAI_TCSR_SEF_SHIFT) | (1UL << SAI_TCSR_FEF_SHIFT));
    /* enable fifo request int */
    val |= 1UL << SAI_TCSR_FRIE_SHIFT;
    inst->TCSR = val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_TxEnableFifoReqDma
 * Description   : Enable fifo request dma
 *
 *END**************************************************************************/
static inline void SAI_DRV_TxEnableFifoReqDma(SAI_Type* inst)
{
    uint32_t val = inst->TCSR;

    /* careful not to clear flags */
    val &= ~((1UL << SAI_TCSR_WSF_SHIFT) | (1UL << SAI_TCSR_SEF_SHIFT) | (1UL << SAI_TCSR_FEF_SHIFT));
    /* enable fifo request int */
    val |= 1UL << SAI_TCSR_FRDE_SHIFT;
    inst->TCSR = val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_TxDisableFifoReqDma
 * Description   : Disable fifo request dma
 *
 *END**************************************************************************/
static inline void SAI_DRV_TxDisableFifoReqDma(SAI_Type* inst)
{
    uint32_t val = inst->TCSR;

    /* careful not to clear flags */
    val &= ~((1UL << SAI_TCSR_WSF_SHIFT) | (1UL << SAI_TCSR_SEF_SHIFT) | (1UL << SAI_TCSR_FEF_SHIFT));
    /* enable fifo request int */
    val &= ~(1UL << SAI_TCSR_FRDE_SHIFT);
    inst->TCSR = val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_TxDisableBitClock
 * Description   : Disable tx clock
 *
 *END**************************************************************************/
static inline void SAI_DRV_TxDisableBitClock(SAI_Type* inst)
{
    uint32_t val;

    val = inst->TCSR;
    /* careful not to clear flags */
    val &= ~((1UL << SAI_TCSR_WSF_SHIFT) | (1UL << SAI_TCSR_SEF_SHIFT) | (1UL << SAI_TCSR_FEF_SHIFT) | (1UL << SAI_TCSR_BCE_SHIFT));
    inst->TCSR = val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_TxDisableBitClock
 * Description   : Enable tx clock
 *
 *END**************************************************************************/
static inline void SAI_DRV_TxEnableBitClock(SAI_Type* inst)
{
    uint32_t val;

    val = inst->TCSR;
    /* careful not to clear flags */
    val &= ~((1UL << SAI_TCSR_WSF_SHIFT) | (1UL << SAI_TCSR_SEF_SHIFT) | (1UL << SAI_TCSR_FEF_SHIFT));
    val |= 1UL << SAI_TCSR_BCE_SHIFT;
    inst->TCSR = val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_IsTxFifoFull
 * Description   : Check if Tx fifo is full
 *
 *END**************************************************************************/
static inline bool SAI_DRV_IsTxFifoFull(const SAI_Type* inst,
                          uint8_t channel)
{
    uint8_t wfp;
    uint8_t rfp;
    bool ret;

    wfp = (uint8_t)((inst->TFR[channel] & SAI_TFR_WFP_MASK) >> SAI_TFR_WFP_SHIFT);
    rfp = (uint8_t)((inst->TFR[channel] & SAI_TFR_RFP_MASK) >> SAI_TFR_RFP_SHIFT);

    /* if WFP and RFP is identical except msb then fifo is full */
    if (((uint32_t)wfp ^ (uint32_t)rfp) == (1UL << (SAI_TFR_WFP_WIDTH - 1UL)))
    {
        ret = true;
    }
    else
    {
        ret = false;
    }
    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_IsTxFifoEmpty
 * Description   : Check if Tx fifo is empty
 *
 *END**************************************************************************/
static inline bool SAI_DRV_IsTxFifoEmpty(const SAI_Type* inst,
                          uint8_t channel)
{
    uint8_t wfp;
    uint8_t rfp;
    bool ret;

    wfp = (uint8_t)((inst->TFR[channel] & SAI_TFR_WFP_MASK) >> SAI_TFR_WFP_SHIFT);
    rfp = (uint8_t)((inst->TFR[channel] & SAI_TFR_RFP_MASK) >> SAI_TFR_RFP_SHIFT);

    /* if WFP and RFP is identical then fifo is empty */
    if (wfp == rfp)
    {
        ret = true;
    }
    else
    {
        ret = false;
    }
    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_TxWrite
 * Description   : Write data register once
 *
 *END**************************************************************************/
static inline void SAI_DRV_TxWrite(SAI_Type* inst,
                     uint8_t channel,
                     uint32_t data)
{
    inst->TDR[channel] = data;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_RxDisableBitClock
 * Description   : Disable rx clock
 *
 *END**************************************************************************/
static inline void SAI_DRV_RxDisableBitClock(SAI_Type* inst)
{
    uint32_t val;

    val = inst->RCSR;
    /* careful not to clear flags */
    val &= ~((1UL << SAI_RCSR_WSF_SHIFT) | (1UL << SAI_RCSR_SEF_SHIFT) | (1UL << SAI_RCSR_FEF_SHIFT) | (1UL << SAI_RCSR_BCE_SHIFT));
    inst->RCSR = val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_RxDisableBitClock
 * Description   : Enable rx clock
 *
 *END**************************************************************************/
static inline void SAI_DRV_RxEnableBitClock(SAI_Type* inst)
{
    uint32_t val;

    val = inst->RCSR;
    /* careful not to clear flags */
    val &= ~((1UL << SAI_RCSR_WSF_SHIFT) | (1UL << SAI_RCSR_SEF_SHIFT) | (1UL << SAI_RCSR_FEF_SHIFT));
    val |= 1UL << SAI_RCSR_BCE_SHIFT;
    inst->RCSR = val;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_IsRxFifoFull
 * Description   : Check if Rx fifo is full
 *
 *END**************************************************************************/
static inline bool SAI_DRV_IsRxFifoFull(const SAI_Type* inst,
                          uint8_t channel)
{
    uint8_t wfp;
    uint8_t rfp;
    bool ret;

    wfp = (uint8_t)((inst->RFR[channel] & SAI_RFR_WFP_MASK) >> SAI_RFR_WFP_SHIFT);
    rfp = (uint8_t)((inst->RFR[channel] & SAI_RFR_RFP_MASK) >> SAI_RFR_RFP_SHIFT);

    /* if WFP and RFP is identical except msb then fifo is full */
    if (((uint32_t)wfp ^ (uint32_t)rfp) == (1UL << ((uint32_t)SAI_RFR_WFP_WIDTH - 1UL)))
    {
        ret = true;
    }
    else
    {
        ret = false;
    }
    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_IsRxFifoEmpty
 * Description   : Check if Rx fifo is empty
 *
 *END**************************************************************************/
static inline bool SAI_DRV_IsRxFifoEmpty(const SAI_Type* inst,
                          uint8_t channel)
{
    uint8_t wfp;
    uint8_t rfp;
    bool ret;

    wfp = (uint8_t)((inst->RFR[channel] & SAI_RFR_WFP_MASK) >> SAI_RFR_WFP_SHIFT);
    rfp = (uint8_t)((inst->RFR[channel] & SAI_RFR_RFP_MASK) >> SAI_RFR_RFP_SHIFT);

    /* if WFP and RFP is identical then fifo is empty */
    if (wfp == rfp)
    {
        ret = true;
    }
    else
    {
        ret = false;
    }
    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_RxRead
 * Description   : Read data register once
 *
 *END**************************************************************************/
static inline uint32_t SAI_DRV_RxRead(const SAI_Type* inst,
                     uint8_t channel)
{
    return inst->RDR[channel];
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_TxGetFifoErrorFlag
 * Description   : Return tx fifo error flag
 *
 *END**************************************************************************/
static inline bool SAI_DRV_TxGetFifoErrorFlag(const SAI_Type* inst)
{
    uint32_t temp = inst->TCSR;
    return (((temp & SAI_TCSR_FEF_MASK) != 0UL) && ((temp & SAI_TCSR_FEIE_MASK) != 0UL));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_TxGetFifoErrorFlag
 * Description   : Return tx sync error flag
 *
 *END**************************************************************************/
static inline bool SAI_DRV_TxGetSyncErrorFlag(const SAI_Type* inst)
{
    uint32_t temp = inst->TCSR;
    return (((temp & SAI_TCSR_SEF_MASK) != 0UL) && ((temp & SAI_TCSR_SEIE_MASK) != 0UL));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_TxGetWordStartFlag
 * Description   : Return tx word start flag
 *
 *END**************************************************************************/
static inline bool SAI_DRV_TxGetWordStartFlag(const SAI_Type* inst)
{
    uint32_t temp = inst->TCSR;
    return (((temp & SAI_TCSR_WSF_MASK) != 0UL) && ((temp & SAI_TCSR_WSIE_MASK) != 0UL));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_TxGetFifoReqFlag
 * Description   : Return fifo request flag
 *
 *END**************************************************************************/
static inline bool SAI_DRV_TxGetFifoReqFlag(const SAI_Type* inst)
{
    return ((inst->TCSR & SAI_TCSR_FRF_MASK) != 0UL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_RxGetFifoErrorFlag
 * Description   : Return rx fifo error flag
 *
 *END**************************************************************************/

static inline bool SAI_DRV_RxGetFifoErrorFlag(const SAI_Type* inst)
{
    uint32_t temp = inst->RCSR;
    return (((temp & SAI_RCSR_FEF_MASK) != 0UL) && ((temp & SAI_RCSR_FEIE_MASK) != 0UL));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_RxGetFifoErrorFlag
 * Description   : Return rx sync error flag
 *
 *END**************************************************************************/
static inline bool SAI_DRV_RxGetSyncErrorFlag(const SAI_Type* inst)
{
    uint32_t temp = inst->RCSR;
    return (((temp & SAI_RCSR_SEF_MASK) != 0UL) && ((temp & SAI_RCSR_SEIE_MASK) != 0UL));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_RxGetFifoReqFlag
 * Description   : Return fifo request flag
 *
 *END**************************************************************************/
static inline bool SAI_DRV_RxGetFifoReqFlag(const SAI_Type* inst)
{
    return ((inst->RCSR & SAI_RCSR_FRF_MASK) != 0UL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_RxGetWordStartFlag
 * Description   : Return rx word start flag
 *
 *END**************************************************************************/
static inline bool SAI_DRV_RxGetWordStartFlag(const SAI_Type* inst)
{
    uint32_t temp = inst->RCSR;
    return (((temp & SAI_RCSR_WSF_MASK) != 0UL) && ((temp & SAI_RCSR_WSIE_MASK) != 0UL));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_GetParamFrameSize
 * Description   : Return frame size in param register
 *
 *END**************************************************************************/
static inline uint8_t SAI_DRV_GetParamFrameSize(const SAI_Type* inst)
{
    return (uint8_t)((inst->PARAM & SAI_PARAM_FRAME_MASK) >> SAI_PARAM_FRAME_SHIFT);
}


#endif /* SAI_HW_ACCESS_H_ */
