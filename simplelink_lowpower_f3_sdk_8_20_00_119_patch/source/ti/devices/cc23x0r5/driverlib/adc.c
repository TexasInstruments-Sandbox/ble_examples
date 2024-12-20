/******************************************************************************
 *  Filename:       adc.c
 *
 *  Description:    Driver for the ADC.
 *
 *  Copyright (c) 2022-2024 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1) Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2) Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3) Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#include "adc.h"
#include "debug.h"

//*****************************************************************************
//
// Sets the clock-divider value, and sample duration
//
//*****************************************************************************
void ADCSetSampleDuration(uint32_t clkDiv, uint16_t clkCycles)
{
    uint32_t tempCtl;

    ASSERT((ADC_CTL0_SCLKDIV_M & clkDiv) == clkDiv);
    ASSERT((ADC_SCOMP0_VAL_M & clkCycles) == clkCycles);

    // Read current control register
    tempCtl = HWREG(ADC_BASE + ADC_O_CTL0);

    /* Clear clk-div bits */
    tempCtl &= ~(ADC_CTL0_SCLKDIV_M);

    /* Set clk-div bits from argument */
    tempCtl |= clkDiv;

    /* Write back control register */
    HWREG(ADC_BASE + ADC_O_CTL0) = tempCtl;

    /* write sample-duration */
    HWREG(ADC_BASE + ADC_O_SCOMP0) = clkCycles;
}

//*****************************************************************************
//
// Sets the ADC bit resolution
//
//*****************************************************************************
void ADCSetResolution(uint32_t resolution)
{
    uint32_t tempCtl;

    ASSERT((ADC_CTL2_RES_M & resolution) == resolution);

    // Read current control register
    tempCtl = HWREG(ADC_BASE + ADC_O_CTL2);

    /* Clear resolution bits */
    tempCtl &= ~(ADC_CTL2_RES_M);

    /* Set resolution bits from argument */
    tempCtl |= resolution;

    /* Write back control register */
    HWREG(ADC_BASE + ADC_O_CTL2) = tempCtl;
}

//*****************************************************************************
//
// Sets the ADC reference source and input channel
//
//*****************************************************************************
void ADCSetInput(uint32_t reference, uint8_t channel, uint32_t index)
{
    uint32_t tempCtl;
    /* Set internal reference to disabled by default */
    uint32_t refCfg = ADC_REFCFG_REFEN_DIS;

    ASSERT(reference == ADC_FIXED_REFERENCE_1V4 || reference == ADC_FIXED_REFERENCE_2V5 ||
           reference == ADC_EXTERNAL_REFERENCE || reference == ADC_VDDS_REFERENCE);
    ASSERT(channel < (ADC_MEMCTL0_CHANSEL_M >> ADC_MEMCTL0_CHANSEL_S));

    /* Read current control register */
    tempCtl = HWREG(ADC_BASE + ADC_O_MEMCTL0 + (4 * index));

    /* Clear reference and channel bits */
    tempCtl &= ~(ADC_MEMCTL0_VRSEL_M | ADC_MEMCTL0_CHANSEL_M);

    /* Set channel */
    tempCtl |= channel << ADC_MEMCTL0_CHANSEL_S;

    /* Set internal reference, if selected */
    if ((reference == ADC_FIXED_REFERENCE_1V4) || (reference == ADC_FIXED_REFERENCE_2V5))
    {
        /* Enable internal reference, set bias current */
        refCfg = ADC_REFCFG_IBPROG_VAL0 | ADC_REFCFG_REFEN_EN;

        /* Set mem-ctrl register to use internal reference */
        tempCtl |= ADC_MEMCTL0_VRSEL_INTREF;

        /* Set internal reference voltage level */
        if (reference == ADC_FIXED_REFERENCE_1V4)
        {
            refCfg |= ADC_REFCFG_REFVSEL_V1P4;
        }
        else
        {
            refCfg |= ADC_REFCFG_REFVSEL_V2P5;
        }
    }
    else if (reference == ADC_EXTERNAL_REFERENCE)
    {
        /* Set external reference */
        tempCtl |= ADC_MEMCTL0_VRSEL_EXTREF;
    }
    else
    {
        /* Neither internal, nor external reference pin selected. Set VDDS as reference */
        tempCtl |= ADC_MEMCTL0_VRSEL_VDDS;
    }

    /* Write back control registers */
    HWREG(ADC_BASE + ADC_O_MEMCTL0 + (4 * index)) = tempCtl;
    HWREG(ADC_BASE + ADC_O_REFCFG)                = refCfg;
}

//*****************************************************************************
//
// Set the ADC sampling mode
//
//*****************************************************************************
void ADCSetSamplingMode(uint32_t samplingMode)
{
    uint32_t tempCtl;

    ASSERT((ADC_CTL1_SAMPMODE_M & samplingMode) == samplingMode);

    /* Read current control register */
    tempCtl = HWREG(ADC_BASE + ADC_O_CTL1);

    /* Clear sampling mode related fields */
    tempCtl &= ~(ADC_CTL1_SAMPMODE_M);

    /* Set sampling mode */
    tempCtl |= samplingMode;

    /* Write back control register */
    HWREG(ADC_BASE + ADC_O_CTL1) = tempCtl;
}

//*****************************************************************************
//
// Set the ADC trigger source
//
//*****************************************************************************
void ADCSetTriggerSource(uint32_t triggerSource)
{
    uint32_t tempCtl;

    ASSERT((ADC_CTL1_TRIGSRC_M & triggerSource) == triggerSource);

    /* Read current control register */
    tempCtl = HWREG(ADC_BASE + ADC_O_CTL1);

    /* Clear trigger-related fields */
    tempCtl &= ~(ADC_CTL1_SC_M | ADC_CTL1_TRIGSRC_M);

    /* Set trigger source */
    tempCtl |= triggerSource;

    /* Write back control register */
    HWREG(ADC_BASE + ADC_O_CTL1) = tempCtl;
}

//*****************************************************************************
//
// Set the ADC trigger policy
//
//*****************************************************************************
void ADCSetTriggerPolicy(uint32_t triggerPolicy, uint32_t index)
{
    uint32_t tempCtl;

    ASSERT((ADC_MEMCTL0_TRG_M & triggerPolicy) == triggerPolicy);

    /* Read current memory control register */
    tempCtl = HWREG(ADC_BASE + ADC_O_MEMCTL0 + (4 * index));

    /* Clear trigger policy related fields */
    tempCtl &= ~ADC_MEMCTL0_TRG_M;

    /* Set trigger policy */
    tempCtl |= triggerPolicy;

    /* Write back memory control register */
    HWREG(ADC_BASE + ADC_O_MEMCTL0 + (4 * index)) = tempCtl;
}

//*****************************************************************************
//
// Set start and stop control registers
//
//*****************************************************************************
void ADCSetMemctlRange(uint32_t start, uint32_t stop)
{
    uint32_t tempCtl;

    /* Read current control register */
    tempCtl = HWREG(ADC_BASE + ADC_O_CTL2);

    /* Clear start and stop address bits */
    tempCtl &= ~(ADC_CTL2_ENDADD_M | ADC_CTL2_STARTADD_M);

    /* Set start and stop address bits */
    tempCtl |= (start << ADC_CTL2_STARTADD_S) & ADC_CTL2_STARTADD_M;
    tempCtl |= (stop << ADC_CTL2_ENDADD_S) & ADC_CTL2_ENDADD_M;

    /* Write back control register */
    HWREG(ADC_BASE + ADC_O_CTL2) = tempCtl;
}

//*****************************************************************************
//
// Set power down policy
//
//*****************************************************************************
void ADCSetPowerDownPolicy(uint32_t powerDownPolicy)
{
    uint32_t tempCtl;

    ASSERT((ADC_CTL0_PWRDN_M & powerDownPolicy) == powerDownPolicy);

    /* Read current control register */
    tempCtl = HWREG(ADC_BASE + ADC_O_CTL0);

    /* Clear power down policy bits */
    tempCtl &= ~(ADC_CTL0_PWRDN_M);

    /* Set power down policy bits */
    tempCtl |= powerDownPolicy;

    /* Write back control register */
    HWREG(ADC_BASE + ADC_O_CTL0) = tempCtl;
}

//*****************************************************************************
//
// Set conversion sequence
//
//*****************************************************************************
void ADCSetSequence(uint32_t sequence)
{
    uint32_t tempCtl;

    ASSERT((ADC_CTL1_CONSEQ_M & sequence) == sequence);

    /* Read current control register */
    tempCtl = HWREG(ADC_BASE + ADC_O_CTL1);

    /* Clear sequence bits */
    tempCtl &= ~(ADC_CTL1_CONSEQ_M);

    /* Set sequence bits */
    tempCtl |= sequence;

    /* Write back control register */
    HWREG(ADC_BASE + ADC_O_CTL1) = tempCtl;
}

//*****************************************************************************
//
// Performs ADC value gain adjustment.
//
//*****************************************************************************
uint32_t ADCAdjustValueForGain(uint32_t adcValue, uint32_t bitResolution, uint16_t gain)
{
    uint32_t adcMaxCode;
    uint32_t adjustedValue;

    /* Adjust value for gain and offset. Actual gain ratio is (gain/0x8000) */
    adjustedValue = ((adcValue * gain) + 0x4000) / 0x8000;

    switch (bitResolution)
    {
        case ADC_RESOLUTION_8_BIT:
            adcMaxCode = 0xFF;
            break;
        case ADC_RESOLUTION_10_BIT:
            adcMaxCode = 0x3FF;
            break;
        case ADC_RESOLUTION_12_BIT: /* Default to 12-bit */
        default:
            adcMaxCode = 0xFFF;
            break;
    }

    /* Make sure no overflow occurs */
    if (adjustedValue > adcMaxCode)
    {
        adjustedValue = adcMaxCode;
    }

    return (adjustedValue);
}

//*****************************************************************************
//
// Convert ADC code to microvolts
//
//*****************************************************************************
uint32_t ADCValueToMicrovolts(uint32_t adcCode, uint32_t bitResolution, uint32_t referenceVoltageMicroVolt)
{
    uint32_t adcMaxCode;
    uint32_t shift;
    uint32_t microVolts;

    switch (bitResolution)
    {
        case ADC_RESOLUTION_8_BIT:
            adcMaxCode = 0xFF;
            shift      = 0;
            break;
        case ADC_RESOLUTION_10_BIT:
            adcMaxCode = 0x3FF;
            shift      = 2;
            break;
        case ADC_RESOLUTION_12_BIT: /* Default to 12-bit */
        default:
            adcMaxCode = 0xFFF;
            shift      = 4;
            break;
    }

    /* shift down voltage to avoid 32bit overflow */
    referenceVoltageMicroVolt >>= shift;

    /* Convert from code to microvolts */
    microVolts = (((adcCode * referenceVoltageMicroVolt) + (adcMaxCode >> 1)) / adcMaxCode);

    /* Shift result back up */
    microVolts <<= shift;

    return microVolts;
}

//*****************************************************************************
//
// Get gain value for given reference source
//
//*****************************************************************************
uint16_t ADCGetAdjustmentGain(uint32_t reference)
{
    uint16_t gain;

    ASSERT(reference == ADC_FIXED_REFERENCE_1V4 || reference == ADC_FIXED_REFERENCE_2V5 ||
           reference == ADC_EXTERNAL_REFERENCE || reference == ADC_VDDS_REFERENCE);

    switch (reference)
    {
        /* 1.4V reference */
        case ADC_FIXED_REFERENCE_1V4:

            gain = fcfg->appTrims.cc23x0r5.adcGainWord1.adcGainIntref1P4V;
            break;

        /* 2.5V reference */
        case ADC_FIXED_REFERENCE_2V5:

            gain = fcfg->appTrims.cc23x0r5.adcGainWord1.adcGainIntref2P5V;
            break;

        /* External reference */
        case ADC_EXTERNAL_REFERENCE:

            gain = fcfg->appTrims.cc23x0r5.adcGainWord0.adcGainExtref;
            break;

        /* VDDS reference */
        case ADC_VDDS_REFERENCE:

            gain = fcfg->appTrims.cc23x0r5.adcGainWord0.adcGainVdds;
            break;

        default:
            gain = 0x8000;
            break;
    }

    /*
     * On an untrimmed device, the gain fields will read 0xFFFF. In this case, set the gain to unity (0x8000)
     * The chance that an actual trimmed gain will be 0xFFFF is near zero.
     */
    if (gain == 0xFFFF)
    {
        gain = 0x8000;
    }

    return gain;
}

//*****************************************************************************
//
// Set offset value in ADC peripheral
//
//*****************************************************************************
void ADCSetAdjustmentOffset(uint32_t reference)
{
    int8_t offset;
    uint32_t tmute2_temp;

    ASSERT(reference == ADC_FIXED_REFERENCE_1V4 || reference == ADC_FIXED_REFERENCE_2V5 ||
           reference == ADC_EXTERNAL_REFERENCE || reference == ADC_VDDS_REFERENCE);

    switch (reference)
    {
        /* 1.4V reference */
        case ADC_FIXED_REFERENCE_1V4:

            offset = fcfg->appTrims.cc23x0r5.adcOffset.adcOffsetIntref1P4V;
            break;

        /* 2.5V reference */
        case ADC_FIXED_REFERENCE_2V5:

            offset = fcfg->appTrims.cc23x0r5.adcOffset.adcOffsetIntref2P5V;
            break;

        /* External reference */
        case ADC_EXTERNAL_REFERENCE:

            offset = fcfg->appTrims.cc23x0r5.adcOffset.adcOffsetExtref;
            break;

        /* VDDS reference */
        case ADC_VDDS_REFERENCE:

            offset = fcfg->appTrims.cc23x0r5.adcOffset.adcOffsetVdds;
            break;

        default:
            offset = 0;
            break;
    }

    /* Read out current TMUTE2 register */
    tmute2_temp = HWREG(SYS0_BASE + SYS0_O_TMUTE2);

    /* Clear offset value in TMUTE2 */
    tmute2_temp &= ~(SYS0_TMUTE2_OFFSET_M);

    /* Sign-extend the offset value from 8-bit signed to 16-bit signed. Shift and mask, and place into tmute2_temp */
    tmute2_temp |= (((int16_t)offset) << SYS0_TMUTE2_OFFSET_S) & SYS0_TMUTE2_OFFSET_M;

    /*
     * Unlock mutable registers to allow writing back tmute2. Register-write must follow within 32 clk-cycles,
     * after which the mutable registers will be automatically locked. Key taken from hw_sys0.h
     */
    HWREG(SYS0_BASE + SYS0_O_MUNLOCK) = 0xC5AF6927;

    /* Write back tmute2 register */
    HWREG(SYS0_BASE + SYS0_O_TMUTE2) = tmute2_temp;

    /* Lock the mutable registers by writing something other than the key */
    HWREG(SYS0_BASE + SYS0_O_MUNLOCK) = 0;
}