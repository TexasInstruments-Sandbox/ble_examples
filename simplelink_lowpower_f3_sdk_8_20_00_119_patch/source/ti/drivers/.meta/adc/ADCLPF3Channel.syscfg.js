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
 *  ======== ADCLPF3Channel.syscfg.js ========
 */

"use strict";

/* Get Common /ti/drivers utility functions */
let Common = system.getScript("/ti/drivers/Common.js");

let config = [
    {
        name: "$name",
        defaultPrefix: "CONFIG_ADC_CHANNEL_",
        description: "C identifier used to identify this ADCBuf channel.",
        hidden: false
    },
    {
        name: "referenceSource",
        displayName: "Reference Source",
        default: "ADCLPF3_VDDS_REFERENCE",
        description: "Specifies the ADC's reference source.",
        options: [
            { displayName: "VDDS", name: "ADCLPF3_VDDS_REFERENCE" },
            { displayName: "Internal 1.4V", name: "ADCLPF3_FIXED_REFERENCE_1V4" },
            { displayName: "Internal 2.5V", name: "ADCLPF3_FIXED_REFERENCE_2V5" },
            { displayName: "External reference", name: "ADCLPF3_EXTERNAL_REFERENCE" }
        ],
        onChange: onReferenceSourceChange
    },
    {
        name: "vddsVoltage",
        displayName: "VDDS Voltage",
        description: "Specifies the VDDS voltage in microvolts used to convert measurements to voltage.",
        default: 3300000
    },
    {
        name: "internalSignal",
        displayName: "Internal Signal",
        description: "Specifies internal signal(s) to use as a sample source.",
        default: "None",
        options: [
            { name: "None" },
            {
                name: "Battery Channel",
                description: "Adds a channel to sample the internal battery voltage divided by 3, VDDS/3."
            }
        ]
    }
];

/*
 * ======== onReferenceSourceChange ========
 */
function onReferenceSourceChange(inst, ui)
{
    ui.vddsVoltage.hidden = true;

    if (inst.referenceSource == "ADCLPF3_VDDS_REFERENCE") {
        ui.vddsVoltage.hidden = false;
    }
}

/*
 *  ======== pinmuxRequirements ========
 *  Returns peripheral pin requirements of the specified instance
 *
 *  @param inst    - a fully configured ADCLPF3Channel instance
 *
 *  @returns req[] - an array of pin requirements needed by inst
 */
function pinmuxRequirements(inst)
{

    if (inst.internalSignal != "None") {
        return [];
    }

    let adc = {
        name: "adc",
        hidden: true,
        displayName: "ADC Peripheral",
        interfaceName: "ADC",
        canShareWith: "ADC",
        resources: [

        ],
        signalTypes: {
            adcPin: ["AIN"]
        }
    };

    let adcPinResources = [
        {
            name: "adcPin",
            canShareWith: "ADC",
            hidden: false,
            displayName: "ADC Pin",
            interfaceNames: [
                "PIN0", "PIN1", "PIN2", "PIN3",
                "PIN4", "PIN5", "PIN6", "PIN7",
                "PIN8", "PIN9", "PIN10", "PIN11"
            ]
        }
    ];

    adc.resources = adc.resources.concat(adcPinResources);

    return [adc];
}

/*
 *  ======== moduleInstances ========
 */
function moduleInstances(inst)
{
    let modInstances = new Array();
    let shortName = inst.$name.replace("CONFIG_", "");

    if (inst.internalSignal === "None") {
        modInstances.push(
            {
                name: "adcPinInstance",
                displayName: "Input configuration when not in use",
                moduleName: "/ti/drivers/GPIO",
                collapsed: true,
                requiredArgs: {
                    parentInterfaceName: "adc",
                    parentSignalName: "adcPin",
                    parentSignalDisplayName: shortName
                },
                args: {
                    $name: "CONFIG_GPIO_" + shortName,
                    mode: "Input"
                }
            }
        );
    }

    return modInstances;
}

/*
 *  ======== sharedModuleInstances ========
 */
function sharedModuleInstances(inst)
{
    let instances = new Array();

    let externalReferenceModule = 
    {
        name: "externalReference",
        displayName: "External Reference",
        moduleName: "ti/drivers/adc/ADCLPF3ExternalReference",
        collapsed: false,
        shareWithByDefault: "ADC"
    };

    if (inst.referenceSource === "ADCLPF3_EXTERNAL_REFERENCE") {
        instances.push(externalReferenceModule);
    }

    return instances;
}

/*
 *  ======== _getPinResources ========
 */
function _getPinResources(inst)
{
    let pin;

    if (inst.internalSignal === "None") {
        let devicePinName = inst.adc.adcPin.$solution.devicePinName;
        pin = "DIO" + devicePinName.substring(4);
    }
    else {
        pin = inst.internalSignal;
    }

    return pin;
}

function _getChannelInfo(inst)
{
    let result = 
    {
        inputPin: "GPIO_INVALID_INDEX",
        adcChannel: 15, /* Battery channel */
        refPosPin: "GPIO_INVALID_INDEX",
        refNegPin: "GPIO_INVALID_INDEX",
        refVoltage: 3300000,
        refSource: inst.referenceSource
    };

    if(inst.referenceSource === "ADCLPF3_FIXED_REFERENCE_1V4"){
        result.refVoltage = 1400000;
    } else if(inst.referenceSource === "ADCLPF3_FIXED_REFERENCE_2V5"){
        result.refVoltage = 2500000;
    } else if(inst.referenceSource === "ADCLPF3_VDDS_REFERENCE"){
        result.refVoltage = inst.vddsVoltage;
    } else if(inst.referenceSource === "ADCLPF3_EXTERNAL_REFERENCE"){
        result.refVoltage = inst.externalReference.referenceVoltage;
        result.refPosPin = inst.externalReference.adcRefPInstance.$name;
        result.refNegPin = inst.externalReference.adcRefNInstance.$name;
    }

    if (inst.internalSignal === "None") {
        result.inputPin = inst.adcPinInstance.$name;

        /* Extract pin number from peripheralPinName on the form "ADC2" */
        let peripheralPinName = inst.adc.adcPin.$solution.peripheralPinName.substring(3);
        result.adcChannel = parseInt(peripheralPinName, 10);
    }

    return result;
}

/*
 *  ======== exports ========
 */
exports = {
    config: config,

    displayName: "ADC Channel",

    /* override generic requirements with  device-specific reqs (if any) */
    pinmuxRequirements: pinmuxRequirements,

    moduleInstances: moduleInstances,
    sharedModuleInstances: sharedModuleInstances,
    modules: Common.autoForceModules(["Board"]),

    _getPinResources: _getPinResources,

    _getChannelInfo: _getChannelInfo
};
