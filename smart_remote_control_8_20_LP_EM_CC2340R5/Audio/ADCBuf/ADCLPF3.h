/*
 * Copyright (c) 2022-2024, Texas Instruments Incorporated
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
 *  @file       ADCLPF3.h
 *  @brief      ADC driver implementation for the ADC peripheral on Low Power F3
 *              devices
 *
 *  This ADC driver implementation is designed to operate on a ADC peripheral
 *  for Low Power F3 devices.
 *
 *  Refer to @ref ADC.h for a complete description of APIs & example of use.
 *
 * # Limitations #
 *
 * ## #ADC_Params Attributes Limitations ##
 *
 * Some attributes in the #ADC_Params structure have a limited set of supported
 * values. These limitations are described below:
 *  - #ADC_Params.isProtected
 *    - Must be true. If this is false, #ADC_open() will fail and return NULL.
 *
 ******************************************************************************
 */
#ifndef ti_drivers_adc_ADCLPF3__include
#define ti_drivers_adc_ADCLPF3__include

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/ADC.h>
#include <ti/drivers/Power.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/adc.h)

#ifdef __cplusplus
extern "C" {
#endif

/*!
 *  @brief Resolution of ADC-conversion
 *
 *  This sets the resolution of the ADC conversion, and will affect how long
 *  a conversion takes.
 */
typedef enum
{
    ADCLPF3_RESOLUTION_12_BIT = ADC_RESOLUTION_12_BIT,
    ADCLPF3_RESOLUTION_10_BIT = ADC_RESOLUTION_10_BIT,
    ADCLPF3_RESOLUTION_8_BIT  = ADC_RESOLUTION_8_BIT,
} ADCLPF3_Resolution_Bits;

/*!
 *  @brief  Specifies the source of the ADC reference voltage
 *
 *  - The ADC reference voltage will determine the upper limit of the input voltage.
 *
 *
 *  @note   The actual reference values are slightly different for each device
 *          and are higher than the values specified above. This gain is saved
 *          in the FCFG. The function ::ADC_convertToMicroVolts() must be used
 *          to derive actual voltage values. Do not attempt to compare raw
 *          values between devices or derive a voltage from them yourself. The
 *          results of doing so will only be approximately correct.
 *
 *  @warning    Refer to the datasheet to make sure the input voltage to the ADC
 *              never exceeds the specifies limits.
 */
typedef enum
{
    ADCLPF3_FIXED_REFERENCE_1V4 = ADC_FIXED_REFERENCE_1V4,
    ADCLPF3_FIXED_REFERENCE_2V5 = ADC_FIXED_REFERENCE_2V5,
    ADCLPF3_VDDS_REFERENCE      = ADC_VDDS_REFERENCE,
    ADCLPF3_EXTERNAL_REFERENCE  = ADC_EXTERNAL_REFERENCE
} ADCLPF3_Reference_Source;

/*!
 *  @brief ADC clock-divider
 *
 *  This sets the clock divider value for the ADC clock, derived from the SYSCLK
 */
typedef enum
{
    ADCLPF3_CLKDIV_1  = ADC_CLOCK_DIVIDER_1,
    ADCLPF3_CLKDIV_2  = ADC_CLOCK_DIVIDER_2,
    ADCLPF3_CLKDIV_4  = ADC_CLOCK_DIVIDER_4,
    ADCLPF3_CLKDIV_8  = ADC_CLOCK_DIVIDER_8,
    ADCLPF3_CLKDIV_16 = ADC_CLOCK_DIVIDER_16,
    ADCLPF3_CLKDIV_24 = ADC_CLOCK_DIVIDER_24,
    ADCLPF3_CLKDIV_32 = ADC_CLOCK_DIVIDER_32,
    ADCLPF3_CLKDIV_48 = ADC_CLOCK_DIVIDER_48
} ADCLPF3_Clock_Divider;

/* ADC function table pointer */
extern const ADC_FxnTable ADCLPF3_fxnTable;

/*!
 *  @brief  ADCLPF3 Hardware attributes
 *  These fields are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions.
 *
 */
typedef struct
{
    /*! DIO that the ADC input is routed to */
    uint8_t adcInputDIO;
    /*! DIO that the ADC positive reference is routed to */
    uint8_t adcRefPosDIO;
    /*! DIO that the ADC negative reference is routed to */
    uint8_t adcRefNegDIO;
    /*! Internal signal routed to ADC */
    uint8_t internalChannel;
    /*! Should the raw output be trimmed before returning it */
    bool returnAdjustedVal;
    /*! Reference voltage in microvolts*/
    uint_fast32_t refVoltage;
    /*! Reference source for the ADC to use */
    ADCLPF3_Reference_Source refSource;
    /*! Time the ADC spends sampling, in CLK-cycles.*/
    uint16_t samplingDuration;
    /*! Resolution of ADC-conversion. */
    ADCLPF3_Resolution_Bits resolutionBits;
    /*! ADC clock divider value */
    ADCLPF3_Clock_Divider adcClkkDivider;

} ADCLPF3_HWAttrs;

/*!
 *  @brief  ADCLPF3 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct
{
    /*! Flag if the instance is in use */
    bool isOpen;
} ADCLPF3_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_adc_ADCLPF3__include */
