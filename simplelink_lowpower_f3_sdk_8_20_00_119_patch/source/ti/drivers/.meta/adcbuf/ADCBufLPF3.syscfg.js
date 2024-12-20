/*
 * Copyright (c) 2023-2024 Texas Instruments Incorporated - http://www.ti.com
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
 *  ======== ADCBufLPF3.syscfg.js ========
 */

"use strict";

/* Get Common /ti/drivers utility functions */
let Common = system.getScript("/ti/drivers/Common.js");

let intPriority = Common.newIntPri()[0];
intPriority.name = "interruptPriority";
intPriority.displayName = "Hardware Interrupt Priority";

const moduleDesc = `
The ADCBuf driver allows for using the ADC and DMA to sample and store ADC
results in a user-defined buffer`;

/*
 *  ======== devSpecific ========
 *  Device-specific extensions to be added to base SD configuration
 */
let devSpecific = {
    longDescription: moduleDesc,
    templates: {
        boardc: "/ti/drivers/adcbuf/ADCBufLPF3.Board.c.xdt",
        boardh: "/ti/drivers/adcbuf/ADCBufLPF3.Board.h.xdt"
    },
    config : [
        {
            name: "resolution",
            displayName: "Resolution",
            description: "Specifies the ADC's resolution",
            default: "ADCLPF3_RESOLUTION_12_BIT",
            hidden: false,
            options: [
                { displayName: "8 Bits", name: "ADCLPF3_RESOLUTION_8_BIT" },
                { displayName: "10 Bits", name: "ADCLPF3_RESOLUTION_10_BIT"},
                { displayName: "12 Bits", name: "ADCLPF3_RESOLUTION_12_BIT"}
            ]
        }
    ],
    pinmuxRequirements: pinmuxRequirements,
    moduleInstances: moduleInstances,
    modules: Common.autoForceModules(["Board", "DMA"])

};

/*
 *  ======== pinmuxRequirements ========
 *  Returns peripheral pin requirements of the specified instance
 */
function pinmuxRequirements(inst)
{
    let dataDmaChannel = {
        name : "dataDmaChannel",
        displayName : "DMA ADC Channel",
        description : "DMA channel used for ADC",
        canShareWith: "ADC",
        interfaceNames : ["ADCTRG"]
    };

    let resources = [];
    resources.push(dataDmaChannel);

    let adcBuf = {
        name: "adc",
        displayName: "ADC Peripheral",
        interfaceName: "ADC",
        canShareWith: "ADC",
        resources: resources
    };

    return [adcBuf];
}

function moduleInstances(inst)
{
    let modInstances = new Array();

    modInstances.push(
        {
            name: "adcChannels",
            displayName: "ADC Channels",
            moduleName: "ti/drivers/adc/ADCLPF3Channel",
            defaultInstanceCount: 1,
            minInstanceCount: 1,
            useArray: true
        }
    );
    return(modInstances);
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
    base = Common.addImplementationConfig(base, "ADCBuf", null,
        [{ name: "ADCBufLPF3" }], null);

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