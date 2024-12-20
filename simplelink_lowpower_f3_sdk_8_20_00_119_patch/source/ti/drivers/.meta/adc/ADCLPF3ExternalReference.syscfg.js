/*
 * Copyright (c) 2024, Texas Instruments Incorporated - http://www.ti.com
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

/*
 *  ======== pinmuxRequirements ========
 *  Returns peripheral pin requirements of the specified instance
 */
function pinmuxRequirements(inst)
{

    let adc = {
        name: "adc",
        displayName: "ADC Peripheral",
        interfaceName: "ADC",
        canShareWith: "ADC",
        resources: [
            {
                name: "refPinPos",
                canShareWith: "ADC",
                hidden: false,
                displayName: "ADC Reference Pin, Positive",
                interfaceNames: [
                    "REF_P"
                ]
            },
            {
                name: "refPinNeg",
                canShareWith: "ADC",
                hidden: false,
                displayName: "ADC Reference Pin, Negative",
                interfaceNames: [
                    "REF_N"
                ]
            }
        ],
        signalTypes: {
            refPinPos: ["AIN"],
            refPinNeg: ["AIN"]
        }
    };

    return [adc];
}

/*
 *  ======== moduleInstances ========
 *  returns GPIO instances
 */
function moduleInstances(inst)
{
    /* This avoids constructions like CONFIG_GPIO_CONFIG_ADC_0_AIN */
    let shortName = inst.$name.replace("CONFIG_", "");
    let gpioInstances = new Array();

    gpioInstances.push(
        {
            name: "adcRefPInstance",
            displayName: "ADC positive reference pin configuration while not in use",
            moduleName: "/ti/drivers/GPIO",
            collapsed: true,
            hidden: true,
            requiredArgs: {
                parentInterfaceName: "adc",
                parentSignalName: "refPinPos",
                parentSignalDisplayName: "ADC Reference Pin, Positive"
            },
            args: {
                $name: "CONFIG_GPIO_" + shortName + "_REF_P",
                mode: "Input"
            }
        }
    );
    gpioInstances.push(
        {
            name: "adcRefNInstance",
            displayName: "ADC negative reference pin configuration while not in use",
            moduleName: "/ti/drivers/GPIO",
            collapsed: true,
            hidden: true,
            requiredArgs: {
                parentInterfaceName: "adc",
                parentSignalName: "refPinNeg",
                parentSignalDisplayName: "ADC Reference Pin, Negative"
            },
            args: {
                $name: "CONFIG_GPIO_" + shortName + "_REF_N",
                mode: "Input"
            }
        }
    );

    return gpioInstances;
}

/*
 *  ======== exports ========
 *  Export device-specific extensions to base exports
 */
exports = {
    displayName: "ADC External Reference",
    description: "This module is used to let multiple ADC and ADCBuf instances share the same external ADC reference",
    maxInstances: 1,
    config: [
        {
            name: "referenceVoltage",
            displayName: "Reference Voltage",
            description: "Specifies the external reference voltage in microvolts used to convert measurements to voltage.",
            default: 3300000,
            readOnly: false
        }
    ],

    /* override generic requirements with device-specific reqs (if any) */
    pinmuxRequirements: pinmuxRequirements,

    /* GPIO instances */
    moduleInstances: moduleInstances
};
