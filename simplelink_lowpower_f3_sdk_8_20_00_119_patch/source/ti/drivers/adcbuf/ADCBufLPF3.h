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
/*!****************************************************************************
 *  @file       ADCBufLPF3.h
 *
 *  @brief      ADCBuf driver implementation for an LPF3 analog-to-digital
 *              converter
 *
 * # Driver include #
 *  The ADCBuf header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/ADCBuf.h>
 *  #include <ti/drivers/adc/ADCBufLPF3.h>
 *  @endcode
 *
 * # Overview #
 * This is an LPF3 specific implementation of the generic ADCBuf driver.
 * The generic ADCBuf API specified in @ref ti/drivers/ADCBuf.h should be called
 * by the application, not the device specific implementation in
 * ti/drivers/adcbuf/ADCBufLPF3.c
 *
 * # General Behavior #
 * The internal ADC sample timer is used to generate samples at a specified
 * frequency and the DMA is used to fill a buffer with a specified size in the
 * background. The application may execute other tasks while the hardware
 * handles the conversions. In contrast to the standard ti/drivers/ADC driver,
 * this driver allows for precise sampling of waveforms.
 *
 * | Driver         | Number of samples needed in one call    |
 * |----------------|-----------------------------------------|
 * | ADC.h          | 1                                       |
 * | ADCBuf.h       | > 1                                     |
 *
 * The ADC drivers supports making between 1 and 1024 measurements once or
 * continuous measuring with returned buffer sizes between 1 and 1024
 * measurements.
 *
 * The ADCBuf driver is opened by calling #ADCBuf_open() which will
 * set up interrupts and configure the internal components of the driver.
 * However, the ADCBuf hardware or analog pins are not configured by calling
 * #ADCBuf_open() since other software components might be using the ADC.
 *
 * In order to perform an ADC conversion, the application must call
 * #ADCBuf_convert(). This call will request the ADC resource, configure the
 * ADC, set up the DMA, and perform the requested ADC conversions on the
 * selected DIO or internal signal.
 *
 * @warning If the ADCBUF driver is setup in ADCBuf_RECURRENCE_MODE_CONTINUOUS
 *          mode, the user must assure that the provided callback function is
 *          completed before the next conversion completes. If the next
 *          conversion completes before the callback function finishes, the DMA
 *          will clobber the previous buffer with new data.
 *
 * If the ADC is used by another component when the driver requests it at the
 * start of the #ADCBuf_convert() call, the calling task will be put in blocked
 * state until the ADC is available. If #ADCBuf_convert() is called from ISR
 * context, the call will fail and return #ADCBuf_STATUS_ERROR.
 * The ADC resource may be pre-acquired by calling the
 * #ADCBufLPF3_acquireAdcSemaphore() function. The semaphore must be manually
 * released again when done with the ADC by calling
 * #ADCBufLPF3_releaseAdcSemaphore()
 *
 * # Error handling #
 * The following errors may occur when opening the ADC without assertions enabled:
 * - The ADC handle is already open.
 *
 * The following errors may occur when requesting an ADC conversion:
 * - The ADC is currently already doing a conversion.
 * - The ADC is not available.
 *
 * # Power Management #
 * The TI Power management framework will try to put the device into the
 * most power efficient mode whenever possible. Please see the technical
 * reference manual for further details on each power mode.
 *
 * While converting, the ADCBufLPF3 driver sets a power constraint to keep
 * the device out of standby. When the conversion has finished, the power
 * constraint is released. The driver also sets a dependency on the DMA to
 * enable background transfers from the ADC FIFO to memory.
 * The following statements are valid:
 *      - After #ADCBuf_convert(): the device cannot enter standby.
 *      - After #ADCBuf_convertCancel(): the device can enter standby.
 *      - After a conversion finishes: the device can enter standby.
 *
 *
 * # Supported Functions #
 * | API function                          | Description                                                           |
 * |---------------------------------------|-----------------------------------------------------------------------|
 * | #ADCBuf_init()                        | Initialize ADC driver                                                 |
 * | #ADCBuf_open()                        | Open the ADC driver and configure driver                              |
 * | #ADCBuf_convert()                     | Perform ADC conversion                                                |
 * | #ADCBuf_convertCancel()               | Cancel ongoing ADC conversion                                         |
 * | #ADCBuf_close()                       | Close ADC driver                                                      |
 * | #ADCBuf_Params_init()                 | Initialise ADCBuf_Params structure to default values                  |
 * | #ADCBuf_getResolution()               | Get the resolution of the ADC of the current device                   |
 * | #ADCBuf_adjustRawValues()             | Adjust the values in a returned buffer for manufacturing tolerances   |
 * | #ADCBuf_convertAdjustedToMicroVolts() | Convert a buffer of adjusted values to microvolts                     |
 * | #ADCBufLPF3_acquireAdcSemaphore()     | Manually pre-acquire ADC semaphore                                    |
 * | #ADCBufLPF3_releaseAdcSemaphore()     | Release ADC semaphore previously manually pre-acquired using          |
 *
 * # Supported Sampling Frequencies
 * | Reference        | Resolution | Min Frequency   | Max Frequency |
 * |------------------|------------|-----------------|---------------|
 * | External or VDDS | 8-bit      | 978 sps         | 1600 ksps     |
 * | External or VDDS | 10-bit     | 978 sps         | 1330 ksps     |
 * | External or VDDS | 12-bit     | 977 sps         | 1200 ksps     |
 * | Internal         | 8-bit      | 976 sps         |  400 ksps     |
 * | Internal         | 10-bit     | 975 sps         |  308 ksps     |
 * | Internal         | 12-bit     | 975 sps         |  267 ksps     |
 *
 * # Not Supported Functionality #
 * - Performing conversions on multiple channels simultaneously is not
 *   supported. In other words, the parameter @c channelCount must always be
 *   set to 1 when calling #ADCBuf_convert(). For the same reason, the value
 *   of the @c completedChannel argument in the #ADCBuf_Callback will always
 *   be 0.
 * - It is not supported to have multiple ADCBuf instances open at the same
 *   time. You must close the currently open instance (if any) before trying
 *   to open another instance.
 *   It is suggested to only have one ADCBuf instances.
 *
 * # Used Resources #
 * - The ADC Peripheral
 * - 2 DMA channels
 *   - One "data" channel is used to copy the samples from the ADC to memory
 *     - The channel is selected using the relevant data DMA fields in
 *       #ADCBufLPF3_HWAttrs:
 *       - #ADCBufLPF3_HWAttrs.dataDmaTableEntryPri
 *       - #ADCBufLPF3_HWAttrs.dataDmaTableEntryAlt
 *       - #ADCBufLPF3_HWAttrs.dataDmaChannelMask
 *       - #ADCBufLPF3_HWAttrs.dataDmaEvtReg
 *       - #ADCBufLPF3_HWAttrs.dataDmaEvtMux
 *   - Another "auxiliary" channel is used to re-configure the ADC to generate
 *     DMA triggers for the data DMA channel after each data DMA transfer. This
 *     is needed because the ADC disables generation of DMA triggers when a DMA
 *     transfer completes. So in continuos mode, the generation of DMA triggers
 *     must be re-enabled, and it must be re-enabled within one sample period to
 *     not cause any overflow. To be able to re-enable triggers quickly enough,
 *     the auxiliary DMA channel is used.
 *     - The channel is selected using the relevant auxiliary DMA fields in
 *       #ADCBufLPF3_HWAttrs:
 *       - #ADCBufLPF3_HWAttrs.auxDmaTableEntryPri
 *       - #ADCBufLPF3_HWAttrs.auxDmaChannelMask
 *       - #ADCBufLPF3_HWAttrs.auxDmaEvtReg
 *       - #ADCBufLPF3_HWAttrs.auxDmaEvtMux
 *
 ******************************************************************************
 */

#ifndef ti_drivers_adcbuf_adcbufLPF3__include
#define ti_drivers_adcbuf_adcbufLPF3__include

#include <stdint.h>
#include <stdbool.h>

/* Driver includes */
#include <ti/drivers/ADC.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/adc/ADCLPF3.h>
#include <ti/drivers/dma/UDMALPF3.h>

/* Kernel includes */
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SwiP.h>
#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/dpl/SemaphoreP.h>

/* Driverlib includes */
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_adc.h)
#include DeviceFamily_constructPath(inc/hw_types.h)
#include DeviceFamily_constructPath(driverlib/adc.h)
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_evtsvt.h)

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief  Underflow occurred during conversion.
 *
 * Underflow means that the DMA tried to read data from the ADC before it was
 * ready.
 *
 * This error code is passed to the #ADCBuf_Callback function as the @c status
 * parameter if an underflow has occurred.
 *
 * Only applicable when the recurrence mode is
 * #ADCBuf_RECURRENCE_MODE_CONTINUOUS
 */
#define ADCBufLPF3_STATUS_UNDERFLOW (ADCBuf_STATUS_RESERVED - 0)

/*!
 * @brief  Overflow occurred during conversion.
 *
 * Overflow means that the DMA tried to read data from the ADC before it was
 * ready.
 *
 * This error code is passed to the #ADCBuf_Callback function as the @c status
 * parameter if an underflow has occurred.
 *
 * Only applicable when the recurrence mode is
 * #ADCBuf_RECURRENCE_MODE_CONTINUOUS
 */
#define ADCBufLPF3_STATUS_OVERFLOW (ADCBuf_STATUS_RESERVED - 1)

/*!
 * @brief  Both overflow and underflow occurred during conversion.
 *
 * Overflow means that the DMA tried to read data from the ADC before it was
 * ready.
 *
 * Underflow means that the DMA tried to read data from the ADC before it was
 * ready.
 *
 * This error code is passed to the #ADCBuf_Callback function as the @c status
 * parameter if both an overflow and underflow has occurred.
 *
 * Only applicable when the recurrence mode is
 * #ADCBuf_RECURRENCE_MODE_CONTINUOUS
 */
#define ADCBufLPF3_STATUS_UNDERFLOW_AND_OVERFLOW (ADCBuf_STATUS_RESERVED - 2)

#define ADCBufLPF3_BYTES_PER_SAMPLE 2

/*
 * =============================================================================
 * Constants
 * =============================================================================
 */

/* ADCBuf function table */
extern const ADCBuf_FxnTable ADCBufLPF3_fxnTable;

/*
 * =============================================================================
 * Enumerations
 * =============================================================================
 */

/*!
 *  @brief  Conversion status used internally by the ADCBufLPF3 driver.
 */
typedef enum
{
    ADCBufLPF3_CONVERSION_NONE = 0,
    ADCBufLPF3_CONVERSION_STARTING,
    ADCBufLPF3_CONVERSION_IN_PROGRESS,
} ADCBufLPF3_ConversionStatus;

/*
 * =============================================================================
 * Structs
 * =============================================================================
 */

/*!
 *  @brief  Table entry that maps a virtual adc channel to the corresponding
 *          hardware configuration.
 */
typedef struct
{
    /*! Reference voltage of the selected reference source in microvolts */
    uint_fast32_t refVoltage;
    /*! DIO that this virtual channel is routed to. Should be set to
     * GPIO_INVALID_INDEX if internal signal is used.
     */
    uint8_t adcInputDIO;
    /*! Internal signal routed to ADC */
    uint8_t internalChannel;
    /*! Reference source for the ADC to use */
    ADCLPF3_Reference_Source refSource;
} ADCBufLPF3_AdcChannelLutEntry;

/*!
 *  @brief      ADCBufLPF3 Hardware Attributes
 *
 *  A sample structure is shown below:
 *  @code
 *
 * const ADCBufLPF3_AdcChannelLutEntry adcChannelLut[1] =
 * {
 *     {
 *         .refVoltage      = 3300000,
 *         .adcInputDIO     = CONFIG_GPIO_ADCBUF_0_CHANNEL_0,
 *         .internalChannel = 8,
 *         .refSource       = ADCLPF3_VDDS_REFERENCE,
 *     },
 * }
 *
 *  ALLOCATE_CONTROL_TABLE_ENTRY(dmaChannel3ControlTableEntry, 3 + UDMA_PRI_SELECT);
 *  ALLOCATE_CONTROL_TABLE_ENTRY(dmaChannel3AltControlTableEntry, 3 + UDMA_ALT_SELECT);
 * ALLOCATE_CONTROL_TABLE_ENTRY(dmaChannel7ControlTableEntry, 7 + UDMA_PRI_SELECT);
 *
 *  const ADCBufLPF3_HWAttrs ADCBufLPF3HWAttrs[] = {
 *      {
 *          .dataDmaTableEntryPri = &dmaChannel3ControlTableEntry,
 *          .dataDmaTableEntryAlt = &dmaChannel3AltControlTableEntry,
 *          .auxDmaTableEntryPri  = &dmaChannel7ControlTableEntry,
 *          .adcChannelLut    = adcChannelLut,
 *          .resolutionBits   = ADCLPF3_RESOLUTION_12_BIT,
 *          .dataDmaChannelMask   = UDMA_CHANNEL_3_M,
 *          .auxDmaChannelMask    = UDMA_CHANNEL_7_M,
 *          .dataDmaEvtReg        = EVTSVT_BASE + EVTSVT_O_DMACH3SEL,
 *          .auxDmaEvtReg         = EVTSVT_BASE + EVTSVT_O_DMACH7SEL,
 *          .dataDmaEvtMux        = EVTSVT_DMACH3SEL_IPID_ADC0TRG,
 *          .auxDmaEvtMux         = EVTSVT_DMACH7SEL_PUBID_ADC_COMB,
 *          .intPriority      = ~0,
 *          .adcRefPosDIO     = GPIO_INVALID_INDEX,
 *          .adcRefNegDIO     = GPIO_INVALID_INDEX,
 *      }
 *  };
 *  @endcode
 */
typedef struct
{
    /*! uDMA primary control table entry for the data DMA channel */
    volatile uDMAControlTableEntry *dataDmaTableEntryPri;

    /*! uDMA secondary control table entry for the data DMA channel */
    volatile uDMAControlTableEntry *dataDmaTableEntryAlt;

    /*! uDMA primary control table entry for auxiliary DMA channel used to
     *  re-enable DMA triggers for the data DMA channel.
     */
    volatile uDMAControlTableEntry *auxDmaTableEntryPri;

    /*! Pointer to a table of #ADCBufLPF3_AdcChannelLutEntry's.
     * When starting a conversion using #ADCBuf_convert(),
     * #ADCBuf_Conversion.adcChannel is used as an index into this table
     */
    ADCBufLPF3_AdcChannelLutEntry const *adcChannelLut;

    /*! Resolution of ADC-conversion. */
    ADCLPF3_Resolution_Bits resolutionBits;

    /*! Mask for the data DMA channel (1 << channel number) */
    uint32_t dataDmaChannelMask;

    /*! Mask for auxiliary DMA channel (1 << channel number) */
    uint32_t auxDmaChannelMask;

    /*! Address of event fabric register EVTSVT.DMACHnSEL for the data DMA
     * channel to be used.
     */
    uint32_t dataDmaEvtReg;

    /*! Address of event fabric register EVTSVT.DMACHnSEL for the auxiliary DMA
     * channel to be used.
     */
    uint32_t auxDmaEvtReg;

    /*! DMA Mux ID to be written to the register selected by #dataDmaEvtReg.
     *  The value must select the ADC0TRG event.
     */
    uint8_t dataDmaEvtMux;

    /*! DMA Mux ID to be written to the register selected by #auxDmaEvtReg.
     *  The value must select the ADC_COMB event.
     */
    uint8_t auxDmaEvtMux;

    /*! ADC peripheral's interrupt priority. This value is passed unmodified to
     *  #HwiP_construct().
     */
    uint8_t intPriority;

    /*! DIO that the ADC positive reference is routed to. If no channel in the
     * #ADCBufLPF3_HWAttrs.adcChannelLut is using the external ADC reference,
     * then this should be set to GPIO_INVALID_INDEX.
     */
    uint8_t adcRefPosDIO;

    /*! DIO that the ADC negative reference is routed to. If no channel in the
     * #ADCBufLPF3_HWAttrs.adcChannelLut is using the external ADC reference,
     * then this should be set to GPIO_INVALID_INDEX.
     */
    uint8_t adcRefNegDIO;
} ADCBufLPF3_HWAttrs;

/*!
 *  @brief      ADCBufLPF3 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct
{
    /*! Conversion semaphore. Used in blocking mode only. */
    SemaphoreP_Struct conversionSemaphore;
    /*! Pointer to the current conversion struct */
    ADCBuf_Conversion *currentConversion;
    /*! Hwi object */
    HwiP_Struct hwi;
    /*! Pointer to callback function */
    ADCBuf_Callback callbackFxn;
    /*! The last complete sample buffer used by the DMA */
    uint32_t *activeSampleBuffer;
    /*! Timeout for read semaphore in #ADCBuf_RETURN_MODE_BLOCKING */
    uint32_t semaphoreTimeout;
    /*! Frequency in Hz at which the ADC is triggered */
    uint32_t samplingFrequency;
    /*! Is the ADC currently doing conversions */
    ADCBufLPF3_ConversionStatus conversionStatus;
    /*! Mode for all conversions */
    ADCBuf_Return_Mode returnMode;
    /*! Should we convert continuously or one-shot */
    ADCBuf_Recurrence_Mode recurrenceMode;
    /*! Has the object been opened */
    bool isOpen;
    /*! Did the user manually acquire the ADC semaphore? */
    bool userAcquiredAdcSemaphore;
} ADCBufLPF3_Object, *ADCBufLPF3_Handle;

/*
 * =============================================================================
 * Functions
 * =============================================================================
 */

/*!
 *  @brief  Function to acquire the semaphore that arbitrates access to the ADC.
 *
 *  This function tries to pre-acquire the ADC semaphore before
 *  #ADCBuf_convert() is called by the application. Normally, the ADCBuf driver
 *  acquires the ADC semaphore when calling #ADCBuf_convert(). The driver may
 *  need to wait for the ADC to be available. Consequently, the time at which
 *  the conversion is actually made is normally non-deterministic. Pre-acquiring
 *  the semaphore makes the #ADCBuf_convert() call deterministic.
 *
 *  If the ADC semaphore is manually acquired using this function, it must also
 *  be manually released using #ADCBufLPF3_releaseAdcSemaphore(). It is not
 *  allowed to close the ADCBuf driver using #ADCBuf_close() before the ADC
 *  semaphore has been released.
 *
 *  @pre    #ADCBuf_open() has to be called first.
 *
 *  @pre    There must not currently be a conversion in progress
 *
 *  @param  handle An ADCBufLPF3 handle returned from #ADCBufLPF3_open()
 *
 *  @param  timeout Timeout (in ClockP ticks) to wait for the semaphore to
 *                  be posted (signalled). Passed directly to #SemaphoreP_pend()
 *
 *  @return true if the ADC semaphore was successfully acquired, otherwise false.
 *
 */
bool ADCBufLPF3_acquireAdcSemaphore(ADCBuf_Handle handle, uint32_t timeout);

/*!
 *  @brief  This function releases the ADC semaphore if it was pre it is in the possession
 *          of the ADCBuf instance.
 *
 *  @pre    ADCBufLPF3_open() has to be called first.
 *
 *  @pre    The ADC semaphore has to have been manually acquired using
 *          #ADCBufLPF3_acquireAdcSemaphore()
 *
 *  @pre    There must not currently be a conversion in progress.
 *
 *  @param  handle An ADCBuf handle returned from ADCBufLPF3_open()
 *
 */
bool ADCBufLPF3_releaseAdcSemaphore(ADCBuf_Handle handle);

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_adcbuf_ADCBufLPF3__include */
