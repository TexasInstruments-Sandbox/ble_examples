/*
 * Copyright (c) 2023-2024, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>

/* Kernel services */
#include <ti/drivers/dpl/DebugP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>

/* TI Drivers */
#include <ti/drivers/ADC.h>
#include <ti/drivers/adc/ADCLPF3.h>
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/adcbuf/ADCBufLPF3.h>
#include <ti/drivers/dma/UDMALPF3.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Power.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_ints.h)
#include DeviceFamily_constructPath(inc/hw_types.h)
#include DeviceFamily_constructPath(inc/hw_adc.h)
#include DeviceFamily_constructPath(driverlib/adc.h)
#include DeviceFamily_constructPath(driverlib/interrupt.h)
#include DeviceFamily_constructPath(driverlib/hapi.h)
#include DeviceFamily_constructPath(driverlib/ull.h)

/*
 * =============================================================================
 * Definitions
 * =============================================================================
 */
#define ADC_WAKE_UP_TIME_US 5

/*
 * =============================================================================
 * Public Function Declarations
 * =============================================================================
 */
void ADCBufLPF3_init(ADCBuf_Handle handle);
ADCBuf_Handle ADCBufLPF3_open(ADCBuf_Handle handle, const ADCBuf_Params *params);
int_fast16_t ADCBufLPF3_convert(ADCBuf_Handle handle, ADCBuf_Conversion conversions[], uint_fast8_t channelCount);
int_fast16_t ADCBufLPF3_convertCancel(ADCBuf_Handle handle);
void ADCBufLPF3_close(ADCBuf_Handle handle);
uint_fast8_t ADCBufLPF3_getResolution(ADCBuf_Handle handle);
int_fast16_t ADCBufLPF3_adjustRawValues(ADCBuf_Handle handle,
                                        void *sampleBuffer,
                                        uint_fast16_t sampleCount,
                                        uint32_t adcChannel);
int_fast16_t ADCBufLPF3_convertAdjustedToMicroVolts(ADCBuf_Handle handle,
                                                    uint32_t adcChannel,
                                                    void *adjustedSampleBuffer,
                                                    uint32_t outputMicroVoltBuffer[],
                                                    uint_fast16_t sampleCount);
int_fast16_t ADCBufLPF3_control(ADCBuf_Handle handle, uint_fast16_t cmd, void *arg);

/*
 * =============================================================================
 * Private Function Declarations
 * =============================================================================
 */
static void ADCBufLPF3_configDma(ADCBuf_Handle handle);
static void ADCBufLPF3_hwiFxn(uintptr_t arg);
static void ADCBufLPF3_conversionCallback(ADCBuf_Handle handle,
                                          ADCBuf_Conversion *conversion,
                                          void *completedADCBuffer,
                                          uint32_t completedChannel,
                                          int_fast16_t status);
static void ADCBufLPF3_cleanAdc(ADCBuf_Handle handle);
static void ADCBufLPF3_loadDmaControlTableEntry(ADCBuf_Handle handle, ADCBuf_Conversion *conversion, bool primaryEntry);
static void ADCBufLPF3_setupAdcForAutoConversion(ADCBuf_Handle handle, uint32_t adcChannel);

/*
 * =============================================================================
 * Constants
 * =============================================================================
 */

const ADCBuf_FxnTable ADCBufLPF3_fxnTable = {
    /*! Function to close the specified peripheral */
    .closeFxn                       = ADCBufLPF3_close,
    /*! Function to driver implementation specific control function */
    .controlFxn                     = ADCBufLPF3_control,
    /*! Function to initialize the given data object */
    .initFxn                        = ADCBufLPF3_init,
    /*! Function to open the specified peripheral */
    .openFxn                        = ADCBufLPF3_open,
    /*! Function to start an ADC conversion with the specified peripheral */
    .convertFxn                     = ADCBufLPF3_convert,
    /*! Function to abort a conversion being carried out by the specified
     *  peripheral
     */
    .convertCancelFxn               = ADCBufLPF3_convertCancel,
    /*! Function to get the resolution in bits of the ADC */
    .getResolutionFxn               = ADCBufLPF3_getResolution,
    /*! Function to adjust raw ADC output values to values comparable between
     *  devices of the same type
     */
    .adjustRawValuesFxn             = ADCBufLPF3_adjustRawValues,
    /*! Function to convert adjusted ADC values to microvolts */
    .convertAdjustedToMicroVoltsFxn = ADCBufLPF3_convertAdjustedToMicroVolts,
};

/* Variable used by secondary DMA channel to read from and write to byte 1 of
 * ADC_O_CTL2 to re-enable DMA triggers
 */
const uint8_t enableDmaTriggerByte = (ADC_CTL2_DMAEN_EN >> 8) | (ADC_CTL2_FIFOEN_DIS >> 8);

/* This is the value that needs to be written to the control table entry for
 * the auxiliary DMA channel used to re-enable DMA triggers for the data DMA
 * channel.
 */
static const uDMAControlTableEntry reenableDmaTriggerControlTableEntryValue = {
    .pSrcEndAddr = (void *)&enableDmaTriggerByte,
    .pDstEndAddr = (void *)(ADC_BASE + ADC_O_CTL2 + 1),
    .control     = UDMA_MODE_AUTO | UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_1 |
               UDMALPF3_SET_TRANSFER_SIZE(1),
};

/*
 * =============================================================================
 * External Global Variables
 * =============================================================================
 */
/* Keep track of the number of ADC (and ADCBuf) instances using the external
 * ADC Reference. This is owned by the ADC driver, but is used by the ADCBuf
 * driver to be able to use the external reference.
 */
extern uint8_t ADCLPF3_adcExternalReferenceUsageCount;

/* Keep track of the adc handle instance to create and delete
 * ADCLPF3_adcSemaphore. This is owned by the ADC driver, but is used by the
 * ADCBuf driver to be able to access the ADC peripheral.
 */
extern uint8_t ADCLPF3_adcInstanceCount;

/* Semaphore to arbitrate access to the single ADC peripheral between multiple
 * handles. This is owned by the ADC driver, but is used by the ADCBuf driver
 * to be able to access the ADC peripheral.
 */
extern SemaphoreP_Struct ADCLPF3_adcSemaphore;

/*
 *  ======== ADCBufLPF3_enableDma ========
 *  Enable the generation of DMA trigger from the ADC and enable dma channel
 */
static inline void ADCBufLPF3_enableDma(ADCBuf_Handle handle)
{
    uintptr_t key;
    ADCBufLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;
    ADCBufLPF3_Object const *object   = handle->object;

    key = HwiP_disable();

    /* Enable DMA trigger generation from the ADC */
    ADCEnableDmaTrigger();

    /* Enable the uDMA channel for use */
    UDMALPF3_channelEnable(hwAttrs->dataDmaChannelMask);

    /* Enable secondary DMA channel to re-enable DMA triggers if in continuous
     * mode.
     */
    if (object->recurrenceMode == ADCBuf_RECURRENCE_MODE_CONTINUOUS)
    {
        UDMALPF3_channelEnable(hwAttrs->auxDmaChannelMask);
    }

    HwiP_restore(key);
}

/*!
 *  @brief  Function to configure the DMA to allow automatically transferring
 *          ADC output data into a provided array.
 *
 *  @pre    ADCBufLPF3_open() has to be called first.
 *
 *  @pre    There must not currently be a conversion in progress
 *
 *  @param  handle An ADCBufLPF3 handle returned from ADCBufLPF3_open()
 *
 *  @param  conversion A pointer to an ADCBuf_Conversion
 *
 */
static void ADCBufLPF3_configDma(ADCBuf_Handle handle)
{

    ADCBufLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;
    ADCBufLPF3_Object const *object   = handle->object;

    /* Disable the generation of DMA triggers from the ADC */
    ADCDisableDmaTrigger();

    /* Disable the DMA channels and clear all interrupts */
    UDMALPF3_channelDisable(hwAttrs->dataDmaChannelMask);
    UDMALPF3_clearInterrupt(hwAttrs->dataDmaChannelMask);
    UDMALPF3_channelDisable(hwAttrs->auxDmaChannelMask);
    UDMALPF3_clearInterrupt(hwAttrs->auxDmaChannelMask);

    /* Load primary control table entry */
    ADCBufLPF3_loadDmaControlTableEntry(handle, object->currentConversion, true);

    /* If we are operating in continuous mode, load the alternate DMA control
     * table data structure.
     */
    if (object->recurrenceMode == ADCBuf_RECURRENCE_MODE_CONTINUOUS)
    {
        ADCBufLPF3_loadDmaControlTableEntry(handle, object->currentConversion, false);

        /* Trigger the auxiliary DMA channel on the ADC_COMB event which is
         * triggered by the DMADONE signal (of the data DMA channel)
         */
        HWREG(hwAttrs->auxDmaEvtReg) = hwAttrs->auxDmaEvtMux;

        /* Configure control table entry for auxiliary DMA channel */
        *(hwAttrs->auxDmaTableEntryPri) = reenableDmaTriggerControlTableEntryValue;

        uDMAEnableSwEventInt(hwAttrs->auxDmaChannelMask);
    }
    else
    {
        /* Disable the alternate DMA structure */
        UDMALPF3_disableAttribute(hwAttrs->dataDmaChannelMask, UDMA_ATTR_ALTSELECT);
    }
}

/*
 * ======== ADCBufLPF3_init ========
 */
void ADCBufLPF3_init(ADCBuf_Handle handle)
{
    /* Note: We rely on the object being auto-initialized to zero since it is in
     * the bss section. This means that handle->object->isOpen is already false, so we
     * do not need to set it to false here
     */
    return;
}

/*
 * ======== ADCBufLPF3_open ========
 */
ADCBuf_Handle ADCBufLPF3_open(ADCBuf_Handle handle, const ADCBuf_Params *params)
{
    /* ADCBuf object and HWattrs */
    ADCBufLPF3_Object *object;
    ADCBufLPF3_HWAttrs const *hwAttrs;

    /* Get the pointer to the adcbuf object and hwAttrs */
    object  = handle->object;
    hwAttrs = handle->hwAttrs;

    uintptr_t key;
    key = HwiP_disable();

    if (object->isOpen == true)
    {
        HwiP_restore(key);
        return NULL;
    }

    object->isOpen = true;

    HwiP_restore(key);

    /* Store Params for later retrieval */
    object->conversionStatus         = ADCBufLPF3_CONVERSION_NONE;
    object->semaphoreTimeout         = params->blockingTimeout;
    object->samplingFrequency        = params->samplingFrequency;
    object->returnMode               = params->returnMode;
    object->recurrenceMode           = params->recurrenceMode;
    object->userAcquiredAdcSemaphore = false;

    /* Determine if an ADC or ADCBuf instance is already open */
    key = HwiP_disable();

    /* If this is the first handle requested, set up the semaphore */
    if (ADCLPF3_adcInstanceCount == 0)
    {
        /* Setup semaphore */
        SemaphoreP_constructBinary(&ADCLPF3_adcSemaphore, 1);
    }
    ADCLPF3_adcInstanceCount++;

    /* Register power dependency - i.e. power up and enable clock for ADC */
    Power_setDependency(PowerLPF3_PERIPH_ADC0);

    HwiP_restore(key);

    /* Un-mask the MEMRESIFG0 interrupt. This interrupt must be unmasked to
     * allow the ADC to trigger the DMA to make a readout when new data is
     * available in the MEMRES0 register.
     *
     * NOTE: MEMRES0IFG is used because the interrupt is generated by the
     * MEMRES0 register, which is used because MEMCTL0 register is used to
     * configure the ADC. Should a different register be used, then the MEMRESx
     * register corresponds to the MEMCTLx register.
     *
     * Use IMASK2 so that the event is triggered but the CPU does not vector to
     * the ISR because a single result is ready.
     */
    HWREG(ADC_BASE + ADC_O_IMASK2) |= ADC_IMASK2_MEMRESIFG0_SET;

    /* Enable DMADONE interrupt to allow CPU to vector to hwiFxn once the
     * DMA transaction is completed
     */
    ADCEnableInterrupt(ADC_INT_DMADONE);

    /* Configure DMA event fabric to listen for the ADC event */
    HWREG(hwAttrs->dataDmaEvtReg) = hwAttrs->dataDmaEvtMux;

    if (params->returnMode == ADCBuf_RETURN_MODE_BLOCKING)
    {
        /* Continuous trigger mode and blocking return mode is an illegal
         * combination
         */
        DebugP_assert(!(params->recurrenceMode == ADCBuf_RECURRENCE_MODE_CONTINUOUS));

        /* Create a semaphore to block task execution for the duration of the
         * ADC conversions
         */
        SemaphoreP_constructBinary(&(object->conversionSemaphore), 0);

        /* Store internal callback function */
        object->callbackFxn = ADCBufLPF3_conversionCallback;
    }
    else
    {
        /* Callback mode without a callback function defined */
        DebugP_assert(params->callbackFxn);

        /* Save the callback function pointer */
        object->callbackFxn = params->callbackFxn;
    }

    /* Clear the interrupt flags to prevent an immediate interrupt from a
     * previous configuration from either ADC or DMA
     */
    HwiP_clearInterrupt(INT_ADC_COMB);
    UDMALPF3_clearInterrupt(hwAttrs->dataDmaChannelMask);

    /* Create the Hwi for this ADCBuf instance, to trigger the ADCBufLPF3_hwiFxn
     * when the DMADONE signal is asserted.
     */
    HwiP_Params hwiParams;
    HwiP_Params_init(&hwiParams);
    hwiParams.arg      = (uintptr_t)handle;
    hwiParams.priority = hwAttrs->intPriority;
    HwiP_construct(&(object->hwi), INT_ADC_COMB, ADCBufLPF3_hwiFxn, &hwiParams);

    /* Initialize and Enable the DMA */
    UDMALPF3_init();

    /* Enable clocks on the DMA while this driver needs it */
    Power_setDependency(PowerLPF3_PERIPH_DMA);

    /* Return the handle after finishing initialisation of the driver */
    return handle;
}

/*!
 *  @brief  HWI ISR of the ADC triggered when the DMA transaction is complete
 *
 *  @param  arg         An ADCBufLPF3_Handle
 *
 */
static void ADCBufLPF3_hwiFxn(uintptr_t arg)
{
    ADCBuf_Handle handle              = ((ADCBuf_Handle)arg);
    ADCBufLPF3_Object *object         = handle->object;
    ADCBufLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;
    ADCBuf_Conversion *conversion     = object->currentConversion;
    int_fast16_t status               = ADCBuf_STATUS_SUCCESS;
    uintptr_t key;

    /* Get the status of the ADC interrupt flags */
    uint32_t adcRawInterruptStatus = ADCRawInterruptStatus();
    uint32_t adcInterruptStatus    = ADCMaskedInterruptStatus();

    /* Clear the ADC interrupt flags */
    ADCClearInterrupt(adcInterruptStatus | ((ADC_INT_OVIFG | ADC_INT_UVIFG) & adcRawInterruptStatus));

    /* Clear DMA interrupts */
    UDMALPF3_clearInterrupt(hwAttrs->dataDmaChannelMask);

    /* Set activeSampleBuffer to primary as default */
    object->activeSampleBuffer = conversion->sampleBuffer;

    if (object->recurrenceMode == ADCBuf_RECURRENCE_MODE_ONE_SHOT)
    {
        /* Disable interrupts while we check and update the conversion status */
        key = HwiP_disable();

        /* Only clean up ADC if conversion has not already been cancelled by
         * ADCBuf_convertCancel()
         */
        if (object->isOpen && (object->conversionStatus == ADCBufLPF3_CONVERSION_IN_PROGRESS))
        {
            /* Clean up ADC if we are only doing one conversion. This will also
             * update object->conversionStatus to ADCBufLPF3_CONVERSION_NONE.
             */
            ADCBufLPF3_cleanAdc(((ADCBuf_Handle)arg));
        }

        /* Restore interrupts */
        HwiP_restore(key);

        /* Release ADC semaphore, unless the user acquired it. If the user
         * acquired the semaphore, it is their responsibility to release it
         * again.
         */
        if (!object->userAcquiredAdcSemaphore)
        {
            SemaphoreP_post(&ADCLPF3_adcSemaphore);
        }

        /* Indicate we are done with this transfer */
        object->currentConversion = NULL;
    }
    else if (object->recurrenceMode == ADCBuf_RECURRENCE_MODE_CONTINUOUS)
    {

        /* Reload the finished DMA control table entry.
         *
         * uDMAGetChannelMode() returns UDMA_MODE_STOP if the channel control
         * structure is inactive and safe to modify. Used to check whether the
         * primary or the alternate control structure is being used for the
         * assigned DMA channel.
         */
        if (uDMAGetChannelMode(hwAttrs->dataDmaTableEntryPri) == UDMA_MODE_STOP)
        {
            /* We are currently using the alternate entry -> we just finished
             * the primary entry -> reload primary entry
             */
            ADCBufLPF3_loadDmaControlTableEntry((ADCBuf_Handle)arg, conversion, true);
        }
        else if (uDMAGetChannelMode(hwAttrs->dataDmaTableEntryAlt) == UDMA_MODE_STOP)
        {
            /* We are currently using the primary entry -> we just finished the
             * alternate entry -> reload the alternate entry
             */
            ADCBufLPF3_loadDmaControlTableEntry((ADCBuf_Handle)arg, conversion, false);
            object->activeSampleBuffer = conversion->sampleBufferTwo;
        }

        /* Wait until auxiliary DMA channel has re-enabled DMA trigger
         * generation.
         */
        while (uDMAGetChannelMode(hwAttrs->auxDmaTableEntryPri) != UDMA_MODE_STOP) {}

        /* Re-configure auxiliary DMA channel */
        *(hwAttrs->auxDmaTableEntryPri) = reenableDmaTriggerControlTableEntryValue;

        /* Re-enable auxiliary DMA channel */
        UDMALPF3_channelEnable(hwAttrs->auxDmaChannelMask);

        /* Check for overflow and/or underflow */
        switch (adcRawInterruptStatus & (ADC_INT_OVIFG | ADC_INT_UVIFG))
        {
            case ADC_INT_OVIFG | ADC_INT_UVIFG:
                status = ADCBufLPF3_STATUS_UNDERFLOW_AND_OVERFLOW;
                break;
            case ADC_INT_OVIFG:
                status = ADCBufLPF3_STATUS_OVERFLOW;
                break;
            case ADC_INT_UVIFG:
                status = ADCBufLPF3_STATUS_UNDERFLOW;
                break;
            default:
                status = ADCBuf_STATUS_SUCCESS;
                break;
        }
    }

    /* Perform callback */
    object->callbackFxn(handle, conversion, object->activeSampleBuffer, 0, status);
}

/*!
 *  @brief  LPF3 internal callback function that posts the semaphore in blocking mode
 *
 *  @param  handle      An ADCBufLPF3_Handle
 *
 *  @param  conversion  A pointer to the current ADCBuf_Conversion
 *
 */
static void ADCBufLPF3_conversionCallback(ADCBuf_Handle handle,
                                          ADCBuf_Conversion *conversion,
                                          void *completedADCBuffer,
                                          uint32_t completedChannel,
                                          int_fast16_t status)
{
    ADCBufLPF3_Object *object;

    /* Get the pointer to the object */
    object = handle->object;

    /* Post the semaphore */
    SemaphoreP_post(&(object->conversionSemaphore));
}

/*
 * ======== ADCBufLPF3_convert ========
 */
int_fast16_t ADCBufLPF3_convert(ADCBuf_Handle handle, ADCBuf_Conversion conversions[], uint_fast8_t channelCount)
{
    uint32_t key;
    ADCBufLPF3_Object *object;
    uint32_t adcSemaphorePendTimeout = HwiP_inISR() ? SemaphoreP_NO_WAIT : SemaphoreP_WAIT_FOREVER;

    DebugP_assert(handle);

    /* Get the pointer to the object */
    object = handle->object;

    DebugP_assert(channelCount == 1);
    DebugP_assert((conversions->samplesRequestedCount <= UDMA_XFER_SIZE_MAX));
    DebugP_assert(conversions->sampleBuffer);
    DebugP_assert((object->recurrenceMode != ADCBuf_RECURRENCE_MODE_CONTINUOUS) ||
                  (conversions->sampleBufferTwo != NULL));
    DebugP_assert(!HwiP_inISR() || object->returnMode != ADCBuf_RETURN_MODE_BLOCKING);

    /* Disable interrupts */
    key = HwiP_disable();

    /* Check if ADC is open and that no other transfer is in progress */
    if (!(object->isOpen) || object->conversionStatus != ADCBufLPF3_CONVERSION_NONE)
    {
        /* Restore interrupts */
        HwiP_restore(key);
        return ADCBuf_STATUS_ERROR;
    }

    /* Mark conversion to be starting, meaning another call to ADCBuf_convert(),
     * for example while we pend on the semaphore below, will fail because the
     * status is not ADCBufLPF3_CONVERSION_NONE. A call to
     * ADCBuf_convertCancel() will also fail while the conversion is being
     * started, since the status is not ADCBufLPF3_CONVERSION_IN_PROGRESS.
     * The conversion can only be cancelled once it is fully started and the
     * status has been updated to ADCBufLPF3_CONVERSION_IN_PROGRESS further
     * below.
     */
    object->conversionStatus = ADCBufLPF3_CONVERSION_STARTING;

    /* Restore interrupts */
    HwiP_restore(key);

    /* Try to acquire the ADC semaphore, if user did not already acquire it. */
    if (!object->userAcquiredAdcSemaphore)
    {
        if (SemaphoreP_pend(&ADCLPF3_adcSemaphore, adcSemaphorePendTimeout) != SemaphoreP_OK)
        {
            object->conversionStatus = ADCBufLPF3_CONVERSION_NONE;
            return ADCBuf_STATUS_ERROR;
        }
    }

    /* Store location of the current conversion */
    object->currentConversion = conversions;

    /* Setup ADC for repeated single channel conversions */
    ADCBufLPF3_setupAdcForAutoConversion(handle, conversions->adcChannel);

    /* Configure and arm the DMA and AUX DMA control */
    ADCBufLPF3_configDma(handle);
    ADCBufLPF3_enableDma(handle);

    /* Clear any overflow/underflow/DMADONE interrupts */
    ADCClearInterrupt(ADC_INT_DMADONE | ADC_INT_OVIFG | ADC_INT_UVIFG);

    /* Set constraints to guarantee operation */
    Power_setConstraint(PowerLPF3_DISALLOW_STANDBY);

    /* Disable interrupts while we trigger the ADC and update the
     * conversionStatus. Otherwise the ADCBufLPF3_hwiFxn() function might update
     * the conversionStatus before it is updated to
     * ADCBufLPF3_CONVERSION_IN_PROGRESS below.
     */
    key = HwiP_disable();

    /* Generate SW trigger */
    ADCStartConversion();

    /* Mark conversion as in progress after it has been started */
    object->conversionStatus = ADCBufLPF3_CONVERSION_IN_PROGRESS;

    /* Restore interrupts */
    HwiP_restore(key);

    if (object->returnMode == ADCBuf_RETURN_MODE_BLOCKING)
    {
        if (SemaphoreP_OK != SemaphoreP_pend(&(object->conversionSemaphore), object->semaphoreTimeout))
        {
            /* Cancel the transfer if we experience a timeout */
            ADCBufLPF3_convertCancel(handle);

            /* ADCBufLPF3_convertCancel performs a callback which posts a
             * conversionSemaphore semaphore. This call consumes this extra
             * post.
             */
            SemaphoreP_pend(&(object->conversionSemaphore), SemaphoreP_NO_WAIT);
            return ADCBuf_STATUS_ERROR;
        }
    }

    return ADCBuf_STATUS_SUCCESS;
}

/*
 * ======== ADCBufLPF3_convertCancel ========
 */
int_fast16_t ADCBufLPF3_convertCancel(ADCBuf_Handle handle)
{
    ADCBufLPF3_Object *object;
    ADCBuf_Conversion *conversion;

    DebugP_assert(handle);

    /* Get the pointer to the object and hwAttrs*/
    object = handle->object;

    uintptr_t key = HwiP_disable();

    /* Check if ADC is open and that a transfer is in progress */
    if (!(object->isOpen) || (object->conversionStatus != ADCBufLPF3_CONVERSION_IN_PROGRESS))
    {
        return ADCBuf_STATUS_ERROR;
    }

    /* Set hardware and software configuration to default and turn off driver */
    ADCBufLPF3_cleanAdc(handle);

    /* Use a temporary transaction pointer in case the callback function
     * attempts to perform another ADCBuf_convert call
     */
    conversion = object->currentConversion;

    HwiP_restore(key);

    /* Release ADC semaphore, unless the user acquired it. If the user acquired
     * the semaphore, it is their responsibility to release it again.
     */
    if (!object->userAcquiredAdcSemaphore)
    {
        SemaphoreP_post(&ADCLPF3_adcSemaphore);
    }

    /* Perform callback if we are in one-shot mode. In continuous mode,
     * ADCBuf_convertCancel will probably be called from the callback function
     * itself. No need to call it again.
     */
    if ((conversion != NULL) && object->recurrenceMode == ADCBuf_RECURRENCE_MODE_ONE_SHOT)
    {
        object->callbackFxn(handle,
                            conversion,
                            conversion->sampleBuffer,
                            0, /* This implementation only supports one channel */
                            ADCBuf_STATUS_ERROR);
    }

    return ADCBuf_STATUS_SUCCESS;
}

/*
 * ======== ADCBufLPF3_close ========
 */
void ADCBufLPF3_close(ADCBuf_Handle handle)
{
    /* ADCBuf object*/
    ADCBufLPF3_Object *object;

    DebugP_assert(handle);

    /* Get the pointer to the object */
    object = handle->object;

    /* Check if the ADC is running and abort conversion if necessary.
     * Note: The case were conversionStatus is ADCBufLPF3_CONVERSION_STARTING,
     * is not handled. It cannot be handled since the ADCBufLPF3_convert()
     * function is most likely pending on the ADC semaphore, which will prevent
     * us from closing the object.
     * For this reason, the documentation of ADCBuf_close() states that it must
     * not be called before all calls to ADCBuf_convert() have returned.
     */
    if (object->conversionStatus == ADCBufLPF3_CONVERSION_IN_PROGRESS)
    {
        ADCBuf_convertCancel(handle);
    }

    /* Disable the MEMRESIFG0 event for DMA trigger generation */
    HWREG(ADC_BASE + ADC_O_IMASK2) &= ~ADC_IMASK2_MEMRESIFG0_SET;

    /* Disable DMADONE interrupt */
    ADCDisableInterrupt(ADC_INT_DMADONE);

    /* Get the pointer to the object */
    object = handle->object;

    /* Destroy the Hwi */
    HwiP_destruct(&(object->hwi));

    Power_releaseDependency(PowerLPF3_PERIPH_DMA);

    if (object->returnMode == ADCBuf_RETURN_MODE_BLOCKING)
    {
        SemaphoreP_destruct(&(object->conversionSemaphore));
    }

    /* Release ADC semaphore if it is in our possession.
     * The ADCBufLPF3_releaseAdcSemaphore() function itself performs the
     * possession check.
     */
    ADCBufLPF3_releaseAdcSemaphore(handle);

    /* Mark the module as available */
    object->isOpen = false;
}

/*
 * ======== ADCBufLPF3_getResolution ========
 */
uint_fast8_t ADCBufLPF3_getResolution(ADCBuf_Handle handle)
{
    ADCBufLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;
    switch (hwAttrs->resolutionBits)
    {
        case ADCLPF3_RESOLUTION_8_BIT:
            return 8;
        case ADCLPF3_RESOLUTION_10_BIT:
            return 10;
        case ADCLPF3_RESOLUTION_12_BIT:
        default:
            return 12;
    }
}

/*
 * ======== ADCBufLPF3_adjustRawValues ========
 */
int_fast16_t ADCBufLPF3_adjustRawValues(ADCBuf_Handle handle,
                                        void *sampleBuffer,
                                        uint_fast16_t sampleCount,
                                        uint32_t adcChannel)
{
    ADCBufLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;
    uint32_t gain                     = ADCGetAdjustmentGain(hwAttrs->adcChannelLut[adcChannel].refSource);

    for (uint32_t i = 0; i < sampleCount; i++)
    {
        uint32_t tmpRawADCVal         = ((uint16_t *)sampleBuffer)[i];
        ((uint16_t *)sampleBuffer)[i] = ADCAdjustValueForGain(tmpRawADCVal, hwAttrs->resolutionBits, gain);
    }

    return ADCBuf_STATUS_SUCCESS;
}

/*
 * ======== ADCBufLPF3_convertAdjustedToMicroVolts ========
 */
int_fast16_t ADCBufLPF3_convertAdjustedToMicroVolts(ADCBuf_Handle handle,
                                                    uint32_t adcChannel,
                                                    void *adjustedSampleBuffer,
                                                    uint32_t outputMicroVoltBuffer[],
                                                    uint_fast16_t sampleCount)
{
    ADCBufLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    for (uint32_t i = 0; i < sampleCount; i++)
    {
        outputMicroVoltBuffer[i] = ADCValueToMicrovolts(((uint16_t *)adjustedSampleBuffer)[i],
                                                        hwAttrs->resolutionBits,
                                                        hwAttrs->adcChannelLut[adcChannel].refVoltage);
    }

    return ADCBuf_STATUS_SUCCESS;
}

/*!
 *  @brief  Function to configure the adc DMA control table entry for basic or
 *          ping pong mode
 *
 *  @pre    ADCBufLPF3_open() has to be called first.
 *
 *  @pre    There must not currently be a conversion in progress
 *
 *  @param  handle An ADCBufLPF3 handle returned from ADCBufLPF3_open()
 *
 *  @param  conversion A pointer to an ADCBuf_Conversion
 *
 *  @param  primaryEntry Is this supposed to modify the primary or the alternate
 *          control table entry
 *
 */
static void ADCBufLPF3_loadDmaControlTableEntry(ADCBuf_Handle handle, ADCBuf_Conversion *conversion, bool primaryEntry)
{
    ADCBufLPF3_Object const *object;
    ADCBufLPF3_HWAttrs const *hwAttrs;
    volatile uDMAControlTableEntry *dmaControlTableEntry;
    uint32_t numberOfBytes;

    /* Get the pointer to the object*/
    object  = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Calculate the number of bytes for the transfer */
    numberOfBytes = (uint16_t)(conversion->samplesRequestedCount) * ADCBufLPF3_BYTES_PER_SAMPLE;

    /* Set configure control table entry */
    dmaControlTableEntry = primaryEntry ? hwAttrs->dataDmaTableEntryPri : hwAttrs->dataDmaTableEntryAlt;

    dmaControlTableEntry->control = ((object->recurrenceMode == ADCBuf_RECURRENCE_MODE_ONE_SHOT) ? UDMA_MODE_BASIC
                                                                                                 : UDMA_MODE_PINGPONG) |
                                    UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1 |
                                    UDMALPF3_SET_TRANSFER_SIZE((uint16_t)conversion->samplesRequestedCount);

    dmaControlTableEntry->pDstEndAddr = (void *)((uint32_t)(primaryEntry ? conversion->sampleBuffer
                                                                         : conversion->sampleBufferTwo) +
                                                 numberOfBytes - 1);

    /* Conversion results are stored in MEMRES0, since MEMCTL0 register is
     * used as control register. The source address must be changed if a
     * different MEMCTLx register is used by the ADC. In repeat conversion
     * mode, MEMRESx corresponds to MEMCTLx
     */
    dmaControlTableEntry->pSrcEndAddr = (void *)(ADC_BASE + ADC_O_MEMRES0);
}

/*!
 *  @brief  Function to undo all configurations done by the ADCBuf driver,
 *          except releasing the ADC semaphore.
 *
 *  @warning Must NOT be called while handle->object->conversionStatus is
 *           ADCBufLPF3_CONVERSION_STARTING
 *  @warning Must NOT be called while handle->object->currentConversion is NULL
 *
 *  @pre    ADCBuf_open() has to be called first.
 *
 *  @pre    ADCBuf_convert() has to be called first.
 *
 *  @post   The ADC semaphore must be released, unless it is managed by the user.
 *
 *  @param  handle An ADCBufLPF3 handle returned from ADCBufLPF3_open()
 *
 */
static void ADCBufLPF3_cleanAdc(ADCBuf_Handle handle)
{
    uintptr_t key;
    ADCBufLPF3_Object *object         = handle->object;
    ADCBufLPF3_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Disable conversion. This will also allow making changes to the
     * configuration of the ADC.
     */
    ADCDisableConversion();

    /* Set power down policy back to auto, meaning the ADC will be powered down
     * since it is not in use.
     */
    ADCSetPowerDownPolicy(ADC_POWER_DOWN_POLICY_AUTO);

    /* Set constraints to guarantee operation */
    Power_releaseConstraint(PowerLPF3_DISALLOW_STANDBY);

    /* Disable the UDMA channels */
    UDMALPF3_channelDisable(hwAttrs->dataDmaChannelMask);
    UDMALPF3_channelDisable(hwAttrs->auxDmaChannelMask);

    /* Disable UDMA mode for ADC */
    ADCDisableDmaTrigger();

    /* Clear the interrupt flags */
    ADCClearInterrupt(ADC_INT_DMADONE | ADC_INT_OVIFG | ADC_INT_UVIFG);
    ULLSync();
    HwiP_clearInterrupt(INT_ADC_COMB);
    UDMALPF3_clearInterrupt(hwAttrs->dataDmaChannelMask);
    UDMALPF3_clearInterrupt(hwAttrs->auxDmaChannelMask);

    if (object->currentConversion != NULL &&
        hwAttrs->adcChannelLut[object->currentConversion->adcChannel].refSource == ADCLPF3_EXTERNAL_REFERENCE)
    {
        key = HwiP_disable();

        ADCLPF3_adcExternalReferenceUsageCount--;

        /* Reset external reference pins, if no other open instance is using
         * external reference.
         */
        if (ADCLPF3_adcExternalReferenceUsageCount == 0)
        {
            GPIO_resetConfig(hwAttrs->adcRefPosDIO);
            GPIO_resetConfig(hwAttrs->adcRefNegDIO);
        }

        HwiP_restore(key);
    }

    /* Note that the driver is no longer converting */
    object->conversionStatus = ADCBufLPF3_CONVERSION_NONE;
}

/*
 *  ======== ADCBufLPF3_setupAdcForAutoConversion ========
 */
/* To be used by ADCBuf only to configure the ADC for use.
 * The ADC will be set up to use software trigger and to only require a trigger
 * for the first sample. The sample duration will be set based on the value of
 * frequency to achieve the desired sample rate. The sample duration and adc
 * clock divider from the HW attrs will be ignored.
 * frequency:       The desired sample rate.
 */
static void ADCBufLPF3_setupAdcForAutoConversion(ADCBuf_Handle handle, uint32_t adcChannel)
{
    ADCBufLPF3_Object *object;
    ADCBufLPF3_HWAttrs const *hwAttrs;
    uintptr_t key;

    DebugP_assert(handle);

    /* Get the pointer to the object */
    object = handle->object;

    /* Get handle */
    hwAttrs = handle->hwAttrs;

    uint32_t samplingDuration;
    ADCLPF3_Clock_Divider adcClkDivider;

    /* Get system clock frequency to be used to calculate sample timer prescaler
     * and target counter value.
     */
    ClockP_FreqHz freq;
    ClockP_getCpuFreq(&freq);

#if (DeviceFamily_PARENT == DeviceFamily_PARENT_CC27XX)
    freq.lo /= 2;
#endif

    /* Determine number of system clock cycles per sample
     * systemClocksPerSample = round(freq.lo/object->samplingFrequency)
     */
    uint32_t systemClocksPerSample = (freq.lo + (object->samplingFrequency / 2)) / object->samplingFrequency;

    /* Determine conversion clock divider based on reference source */
    uint32_t conversionClockDivider;
    switch (hwAttrs->adcChannelLut[adcChannel].refSource)
    {
        case ADCLPF3_FIXED_REFERENCE_1V4: /* Intentional fall through */
        case ADCLPF3_FIXED_REFERENCE_2V5:
            conversionClockDivider = 12;
            break;
        case ADCLPF3_VDDS_REFERENCE:     /* Intentional fall through */
        case ADCLPF3_EXTERNAL_REFERENCE: /* Intentional fall through */
        default:
            conversionClockDivider = 2;
            break;
    }

    /* Subtract the number of cycles used for conversion based on resolution */
    switch (hwAttrs->resolutionBits)
    {
        case ADCLPF3_RESOLUTION_8_BIT:
            systemClocksPerSample -= 9 * conversionClockDivider;
            break;
        case ADCLPF3_RESOLUTION_10_BIT:
            systemClocksPerSample -= 12 * conversionClockDivider;
            break;
        case ADCLPF3_RESOLUTION_12_BIT:
        default:
            systemClocksPerSample -= 14 * conversionClockDivider;
            break;
    }

    /* Determine smallest possible ADC Clock divider.
     * adcClkDividerVal = ceil(systemClocksPerSample/(maxCounterValue + 1))
     */
    uint32_t adcClkDividerVal = (systemClocksPerSample + 1022) / (1023);
    adcClkDivider             = ADCLPF3_CLKDIV_1;
    if (adcClkDividerVal > 32)
    {
        adcClkDivider    = ADCLPF3_CLKDIV_48;
        adcClkDividerVal = 48;
    }
    else if (adcClkDividerVal > 24)
    {
        adcClkDivider    = ADCLPF3_CLKDIV_32;
        adcClkDividerVal = 32;
    }
    else if (adcClkDividerVal > 16)
    {
        adcClkDivider    = ADCLPF3_CLKDIV_24;
        adcClkDividerVal = 24;
    }
    else if (adcClkDividerVal > 8)
    {
        adcClkDivider    = ADCLPF3_CLKDIV_16;
        adcClkDividerVal = 16;
    }
    else if (adcClkDividerVal > 4)
    {
        adcClkDivider    = ADCLPF3_CLKDIV_8;
        adcClkDividerVal = 8;
    }
    else if (adcClkDividerVal > 2)
    {
        adcClkDivider    = ADCLPF3_CLKDIV_4;
        adcClkDividerVal = 4;
    }
    else if (adcClkDividerVal > 1)
    {
        adcClkDivider    = ADCLPF3_CLKDIV_2;
        adcClkDividerVal = 2;
    }
    else
    {
        adcClkDivider    = ADCLPF3_CLKDIV_1;
        adcClkDividerVal = 1;
    }

    /* Calculate counter frequency.
     * samplingDuration = round(systemClocksPerSample/adcClkDividerVal)
     */
    samplingDuration = (systemClocksPerSample + (adcClkDividerVal / 2)) / adcClkDividerVal;

    if (hwAttrs->adcChannelLut[adcChannel].refSource == ADCLPF3_EXTERNAL_REFERENCE)
    {
        key = HwiP_disable();

        /* Configure external reference pins, if no other open instance has
         * already done it.
         */
        if (ADCLPF3_adcExternalReferenceUsageCount == 0)
        {
            GPIO_setConfigAndMux(hwAttrs->adcRefPosDIO, GPIO_CFG_INPUT, GPIO_MUX_PORTCFG_PFUNC6);
            GPIO_setConfigAndMux(hwAttrs->adcRefNegDIO, GPIO_CFG_INPUT, GPIO_MUX_PORTCFG_PFUNC6);
        }
        ADCLPF3_adcExternalReferenceUsageCount++;

        HwiP_restore(key);
    }

    /* Set input pin to analog function. If pin is unused, value is set to GPIO_INVALID_INDEX */
    GPIO_setConfigAndMux(hwAttrs->adcChannelLut[adcChannel].adcInputDIO, GPIO_CFG_INPUT, GPIO_MUX_PORTCFG_PFUNC6);

    /* Make sure conversion is disabled to allow configuration changes */
    ADCDisableConversion();

    /* Use manual power down policy, meaning the ADC remains powered on as long
     * as the power down policy is set to manual.
     */
    ADCSetPowerDownPolicy(ADC_POWER_DOWN_POLICY_MANUAL);

    /* Specify range of ctrl registers for conversion. Use ctrl register 0 */
    ADCSetMemctlRange(0, 0);

    /* Set clock-divider and sampling duration */
    ADCSetSampleDuration(adcClkDivider, samplingDuration);

    /* Set sampling mode to auto, meaning the sample duration is determined by
     * sample duration configured above.
     */
    ADCSetSamplingMode(ADC_SAMPLE_MODE_AUTO);

    /* Set resolution */
    ADCSetResolution(hwAttrs->resolutionBits);

    /* Set reference source */
    ADCSetInput(hwAttrs->adcChannelLut[adcChannel].refSource, hwAttrs->adcChannelLut[adcChannel].internalChannel, 0);

    /* Pass correct offset-value to ADC peripheral, depending on reference source */
    ADCSetAdjustmentOffset(hwAttrs->adcChannelLut[adcChannel].refSource);

    /* Configure ADC to repeatedly convert a single channel */
    ADCSetSequence(ADC_SEQUENCE_REPEATSINGLE);

    /* Automatically start next conversion after another is done */
    ADCSetTriggerPolicy(ADC_TRIGGER_POLICY_AUTO_NEXT, 0);

    /* Use software trigger source. The software trigger will trigger the first
     * conversion, but all other conversion will be automatically started due to
     * the trigger policy selected above.
     */
    ADCSetTriggerSource(ADC_TRIGGER_SOURCE_SOFTWARE);

    /* Enable conversion. ADC will wait for trigger. */
    ADCEnableConversion();

    /* Wait for ADC to be powered on */
    HapiWaitUs(ADC_WAKE_UP_TIME_US);
}

/*
 *  ======== ADCBufLPF3_acquireAdcSemaphore ========
 */
bool ADCBufLPF3_acquireAdcSemaphore(ADCBuf_Handle handle, uint32_t timeout)
{
    ADCBufLPF3_Object *object;
    bool result = false;

    /* Get the pointer to the object */
    object = handle->object;

    if (object->isOpen && (object->conversionStatus == ADCBufLPF3_CONVERSION_NONE))
    {
        if (SemaphoreP_pend(&ADCLPF3_adcSemaphore, timeout) == SemaphoreP_OK)
        {
            result                           = true;
            object->userAcquiredAdcSemaphore = true;
        }
    }

    return result;
}

/*
 *  ======== ADCBufLPF3_releaseAdcSemaphore ========
 */
bool ADCBufLPF3_releaseAdcSemaphore(ADCBuf_Handle handle)
{
    ADCBufLPF3_Object *object;
    bool result = false;

    /* Get the pointer to the object */
    object = handle->object;

    /* Only release semaphore, if object is open and if no conversion is in
     * progress.
     */
    if (object->isOpen && (object->conversionStatus == ADCBufLPF3_CONVERSION_NONE))
    {
        object->userAcquiredAdcSemaphore = false;
        result                           = true;
        SemaphoreP_post(&ADCLPF3_adcSemaphore);
    }

    return result;
}

/*
 * ======== ADCBufLPF3_control ========
 */
int_fast16_t ADCBufLPF3_control(ADCBuf_Handle handle, uint_fast16_t cmd, void *arg)
{
    return ADCBuf_STATUS_UNDEFINEDCMD;
}
