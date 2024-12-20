/*
 * Copyright (c) 2022-2024, Texas Instruments Incorporated - http://www.ti.com
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
 *
 */

/*
 *  ======== ADCLPF3.syscfg.js ========
 */

"use strict";

/* Get Common /ti/drivers utility functions */
let Common = system.getScript("/ti/drivers/Common.js");

/*
 *  ======== devSpecific ========
 *  Device-specific extensions to be added to base ADC configuration
 */
let devSpecific = {
    config: [
        {
            name: "resolution",
            displayName: "Resolution",
            description: "Specifies the ADC's resolution",
            default: "ADCLPF3_RESOLUTION_12_BIT",
            hidden: false,
            options: [
                { displayName: "8 Bits", name: "ADCLPF3_RESOLUTION_8_BIT", legacyNames: ["8 Bits"] },
                { displayName: "10 Bits", name: "ADCLPF3_RESOLUTION_10_BIT", legacyNames: ["10 Bits"]},
                { displayName: "12 Bits", name: "ADCLPF3_RESOLUTION_12_BIT", legacyNames: ["12 Bits"]}
            ]
        },
        {
            name: "adcClockDivider",
            displayName: "ADC Clock Divider",
            description: "Specifies the divider value from system clock to ADC clock. This option, along with the " +
                "ADC sample duration (in adc clock-cycles) must ensure a minimum sampling time of 250 ns.",
            default: "ADCLPF3_CLKDIV_8",
            options: [
                { displayName: "1", name: "ADCLPF3_CLKDIV_1", legacyNames: ["1"] },
                { displayName: "2", name: "ADCLPF3_CLKDIV_2", legacyNames: ["2"] },
                { displayName: "4", name: "ADCLPF3_CLKDIV_4", legacyNames: ["4"] },
                { displayName: "8", name: "ADCLPF3_CLKDIV_8", legacyNames: ["8"] },
                { displayName: "16", name: "ADCLPF3_CLKDIV_16", legacyNames: ["16"] },
                { displayName: "24", name: "ADCLPF3_CLKDIV_24", legacyNames: ["24"] },
                { displayName: "32", name: "ADCLPF3_CLKDIV_32", legacyNames: ["32"] },
                { displayName: "48", name: "ADCLPF3_CLKDIV_48", legacyNames: ["48"] }
            ]
        },
        {
            name: "adcSampleTime",
            displayName: "ADC Sample Duration (ADC Clk Cycles)",
            description: "Specifies the ADC sample duration in ADC clock cycles. A value of 0 will count as 1 cycle. " +
                "This option, along with the ADC clock divider must ensure a minimum sampling time of 250 ns.",
            default: 16,
            range: [0, 1023]
        },
        {
            name: "adjustSampleValue",
            displayName: "Adjust Sample Value",
            longDescription: "Specifies if the returned raw sample value is adjusted or not. If not, the returned value " +
                "might differ noticeably between different devices. Any sample that is converted to micro-volts " +
                " through the driver API is automatically adjusted, regardless of this setting.",
            default: false
        },
        /* Deprecated configs */
        {
            deprecated: true,
            name: "referenceSource",
            displayName: "Reference Source",
            default: "ADCLPF3_VDDS_REFERENCE",
            description: "Specifies the ADC's reference source.",
            options: [
                { displayName: "VDDS", name: "ADCLPF3_VDDS_REFERENCE", legacyNames: ["VDDS"] },
                { displayName: "Internal 1.4V", name: "ADCLPF3_FIXED_REFERENCE_1V4", legacyNames: ["Internal 1.4V"] },
                { displayName: "Internal 2.5V", name: "ADCLPF3_FIXED_REFERENCE_2V5", legacyNames: ["Internal 2.5V"] },
                { displayName: "External reference", name: "ADCLPF3_EXTERNAL_REFERENCE", legacyNames: ["External reference"] }
            ]
        },
        {
            deprecated: true,
            name: "referenceVoltage",
            displayName: "Reference Voltage",
            description: "Specifies the ADC's reference voltage in microvolts.",
            default: 3300000,
            readOnly: false
        },
        {
            deprecated: true,
            name: "internalSignal",
            displayName: "Internal Signal",
            description: "Specifies internal signal(s) to use as a"
                + " sample source.",
            default: "None",
            options: [
                { name: "None" },
                {
                    name: "Battery Channel",
                    description: "Adds a channel to sample the internal"
                        + " battery voltage, VDDS."
                }
            ]
        }
    ],

    migrateLegacyConfiguration: migrateLegacyConfiguration,

    modules: Common.autoForceModules(["Board", "Power"]),

    moduleInstances: moduleInstances,

    templates: {
        boardc: "/ti/drivers/adc/ADCLPF3.Board.c.xdt",
        boardh: "/ti/drivers/adc/ADC.Board.h.xdt"
    }
};

/*
 * ======== migrateLegacyConfiguration ========
 */
function migrateLegacyConfiguration(inst)
{
    inst.adcChannel.internalSignal = inst.internalSignal;
    inst.adcChannel.referenceSource = inst.referenceSource;
    if(inst.adcChannel.referenceSource === "ADCLPF3_VDDS_REFERENCE")
    {
        inst.adcChannel.vddsVoltage = inst.referenceVoltage;
    }
    else if (inst.adcChannel.referenceSource === "ADCLPF3_EXTERNAL_REFERENCE")
    {
        inst.adcChannel.externalReference.referenceVoltage = inst.referenceVoltage;
    }
}

/*
 *  ======== moduleInstances ========
 */
function moduleInstances(inst)
{
    /* This avoids constructions like CONFIG_GPIO_CONFIG_ADC_0_AIN */
    let shortName = inst.$name.replace("CONFIG_", "");
    let instances = new Array();

    instances.push(
        {
            name: "adcChannel",
            displayName: "ADC Channel",
            moduleName: "/ti/drivers/adc/ADCLPF3Channel",
            collapsed: false,
            args: {
                $name: "CONFIG_" + shortName + "_CHANNEL"
            }
        }
    );

    return instances;
}

/*
 *  ======== extend ========
 *  Extends a base exports object to include any device specifics
 *
 *  This function is invoked by the generic ADC module to
 *  allow us to augment and override as needed.
 */
function extend(base)
{
    /* display which driver implementation can be used */
    base = Common.addImplementationConfig(base, "ADC", null,
        [{ name: "ADCLPF3" }], null);

    /* merge and overwrite base module attributes */
    let result = Object.assign({}, base, devSpecific);

    /* concatenate device-specific configs */
    result.config = base.config.concat(devSpecific.config);

    return result;
}

/*
 *  ======== exports ========
 *  Export device-specific extensions to base exports
 */
exports = {
    /* required function, called by base ADC module */
    extend: extend
};
