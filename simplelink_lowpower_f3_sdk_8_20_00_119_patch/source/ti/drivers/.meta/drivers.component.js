/*
 * Copyright (c) 2018-2024, Texas Instruments Incorporated - https://www.ti.com
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
 *  ======== drivers.component.js ========
 */

"use strict";

let topModules;
let displayName = "TI Drivers";
let description = "TI Drivers System Configuration";
let deviceId = system.deviceData.deviceId;

if (deviceId.match(/CC13.4|CC26.4|CC2653/)) {
    /* CC13X4 & CC26X4 */
    topModules = [
        {
            displayName: displayName,
            description: description,
            modules: [
                "/ti/display/Display",
                "/ti/drivers/ADC",
                "/ti/drivers/ADCBuf",
                "/ti/drivers/AESCBC",
                "/ti/drivers/AESCCM",
                "/ti/drivers/AESCMAC",
                "/ti/drivers/AESCTR",
                "/ti/drivers/AESCTRDRBG",
                "/ti/drivers/AESECB",
                "/ti/drivers/AESGCM",
                "/ti/drivers/ANSIX936KDF",
                "/ti/drivers/Board",
                //"/ti/drivers/CryptoKey", // unused - no configuration currently required for CC26X4 family
                "/ti/drivers/DAC",
                "/ti/drivers/DMA",
                "/ti/drivers/ECDH",
                "/ti/drivers/ECDSA",
                "/ti/drivers/ECJPAKE",
                "/ti/drivers/EDDSA",
                "/ti/drivers/GPIO",
                "/ti/drivers/I2C",
                "/ti/drivers/I2S",
                "/ti/drivers/ITM",
                "/ti/drivers/NVS",
                "/ti/drivers/Power",
                "/ti/drivers/PWM",
                "/ti/drivers/RNG",
                "/ti/drivers/SD",
                "/ti/drivers/SHA2",
                "/ti/drivers/SPI",
                "/ti/drivers/Temperature",
                "/ti/drivers/Timer",
                "/ti/drivers/timer/GPTimerCC26XX",
                "/ti/drivers/TRNG",
                "/ti/drivers/UART2",
                "/ti/drivers/Watchdog"
            ],
            "categories": [
                {
                    "displayName": "TI Driver Apps",
                    "description": "TI Drivers Apps Configuration",
                    "modules": [
                        "/ti/drivers/apps/Button",
                        "/ti/drivers/apps/LED"
                    ]
                }
            ]
        }
    ];
} else if (deviceId.match(/CC13.2|CC26.2/)) {
    /* CC13X2 & CC26X2 */
    topModules = [
        {
            displayName: displayName,
            description: description,
            modules: [
                "/ti/display/Display",
                "/ti/drivers/ADC",
                "/ti/drivers/ADCBuf",
                "/ti/drivers/AESCBC",
                "/ti/drivers/AESCCM",
                "/ti/drivers/AESCMAC",
                "/ti/drivers/AESCTR",
                "/ti/drivers/AESCTRDRBG",
                "/ti/drivers/AESECB",
                "/ti/drivers/AESGCM",
                "/ti/drivers/ANSIX936KDF",
                "/ti/drivers/Board",
                "/ti/drivers/CAN",
                "/ti/drivers/DAC",
                "/ti/drivers/DMA",
                "/ti/drivers/ECDH",
                "/ti/drivers/ECDSA",
                "/ti/drivers/ECJPAKE",
                "/ti/drivers/EDDSA",
                "/ti/drivers/GPIO",
                "/ti/drivers/I2C",
                "/ti/drivers/I2S",
                "/ti/drivers/ITM",
                "/ti/drivers/NVS",
                "/ti/drivers/Power",
                "/ti/drivers/PWM",
                "/ti/drivers/RNG",
                "/ti/drivers/SD",
                "/ti/drivers/SHA2",
                "/ti/drivers/SPI",
                "/ti/drivers/Temperature",
                "/ti/drivers/Timer",
                "/ti/drivers/timer/GPTimerCC26XX",
                "/ti/drivers/TRNG",
                "/ti/drivers/UART2",
                "/ti/drivers/Watchdog"
            ],
            "categories": [
                {
                    "displayName": "TI Driver Apps",
                    "description": "TI Drivers Apps Configuration",
                    "modules": [
                        "/ti/drivers/apps/Button",
                        "/ti/drivers/apps/LED"
                    ]
                }
            ]
        }
    ];
} else if (deviceId.match(/CC13.1|CC26.1/)) {
    /* CC13X1 & CC26X1 */
    topModules = [
        {
            displayName: displayName,
            description: description,
            modules: [
                "/ti/display/Display",
                "/ti/drivers/ADC",
                "/ti/drivers/ADCBuf",
                "/ti/drivers/AESCBC",
                "/ti/drivers/AESCCM",
                "/ti/drivers/AESCMAC",
                "/ti/drivers/AESCTR",
                "/ti/drivers/AESCTRDRBG",
                "/ti/drivers/AESECB",
                "/ti/drivers/ANSIX936KDF",
                "/ti/drivers/Board",
                "/ti/drivers/CAN",
                "/ti/drivers/DAC",
                "/ti/drivers/DMA",
                "/ti/drivers/ECDH",
                "/ti/drivers/ECDSA",
                "/ti/drivers/GPIO",
                "/ti/drivers/I2C",
                "/ti/drivers/I2S",
                "/ti/drivers/NVS",
                "/ti/drivers/Power",
                "/ti/drivers/PWM",
                "/ti/drivers/RNG",
                "/ti/drivers/SD",
                "/ti/drivers/SHA2",
                "/ti/drivers/SPI",
                "/ti/drivers/Temperature",
                "/ti/drivers/Timer",
                "/ti/drivers/timer/GPTimerCC26XX",
                "/ti/drivers/TRNG",
                "/ti/drivers/UART2",
                "/ti/drivers/Watchdog"
            ],
            "categories": [
                {
                    "displayName": "TI Driver Apps",
                    "description": "TI Drivers Apps Configuration",
                    "modules": [
                        "/ti/drivers/apps/Button",
                        "/ti/drivers/apps/LED"
                    ]
                }
            ]
        }
    ];
} else if (deviceId.match(/CC23.0/)) {
    /* CC23X0 */
    topModules = [
        {
            displayName: displayName,
            description: description,
            modules: [
                "/ti/display/Display",
                "/ti/drivers/ADC",
                "/ti/drivers/ADCBuf",
                "/ti/drivers/AESCBC",
                "/ti/drivers/AESCCM",
                "/ti/drivers/AESCMAC",
                "/ti/drivers/AESCTR",
                "/ti/drivers/AESCTRDRBG",
                "/ti/drivers/AESECB",
                "/ti/drivers/ANSIX936KDF",
                "/ti/drivers/BatteryMonitor",
                "/ti/drivers/Board",
                "/ti/drivers/CAN",
                "/ti/drivers/Comparator",
                "/ti/drivers/DMA",
                "/ti/drivers/ECDH",
                "/ti/drivers/ECDSA",
                "/ti/drivers/ECIES",
                "/ti/drivers/GPIO",
                "/ti/drivers/I2C",
                "/ti/drivers/I2CTarget",
                "/ti/drivers/LGPTimer",
                "/ti/drivers/NVS",
                "/ti/drivers/Power",
                "/ti/drivers/PWM",
                "/ti/drivers/RNG",
                "/ti/drivers/SD",
                "/ti/drivers/SHA2",
                "/ti/drivers/SPI",
                "/ti/drivers/Temperature",
                "/ti/drivers/UART2",
                "/ti/drivers/Watchdog"
            ],
            "categories": [
                {
                    "displayName": "TI Driver Apps",
                    "description": "TI Drivers Apps Configuration",
                    "modules": [
                        "/ti/drivers/apps/Button"
                        // "/ti/drivers/apps/LED"
                    ]
                }
            ]
        }
    ];
} else if (deviceId.match(/CC27/)) {
    /* CC27XX */
    topModules = [
        {
            displayName: displayName,
            description: description,
            modules: [
                "/ti/display/Display",
                "/ti/drivers/ADC",
                // "/ti/drivers/ADCBuf",
                "/ti/drivers/AESCBC",
                "/ti/drivers/AESCCM",
                "/ti/drivers/AESCMAC",
                "/ti/drivers/AESCTR",
                // "/ti/drivers/AESCTRDRBG",
                "/ti/drivers/AESECB",
                "/ti/drivers/AESGCM",
                "/ti/drivers/BatteryMonitor",
                "/ti/drivers/Board",
                "/ti/drivers/CAN",
                "/ti/drivers/Comparator",
                "/ti/drivers/DMA",
                "/ti/drivers/ECDH",
                "/ti/drivers/ECDSA",
                "/ti/drivers/GPIO",
                "/ti/drivers/I2C",
                "/ti/drivers/I2CTarget",
                "/ti/drivers/I2S",
                "/ti/drivers/ITM",
                "/ti/drivers/LGPTimer",
                "/ti/drivers/NVS",
                "/ti/drivers/Power",
                "/ti/drivers/PWM",
                "/ti/drivers/RNG",
                "/ti/drivers/SD",
                "/ti/drivers/SHA2",
                "/ti/drivers/SPI",
                "/ti/drivers/Temperature",
                "/ti/drivers/UART2",
                "/ti/drivers/VCE",
                "/ti/drivers/Watchdog"
            ],
            "categories": [
                {
                    "displayName" : "TI Driver Apps",
                    "description" : "TI Drivers Apps Configuration",
                    "modules" : [
                        "/ti/drivers/apps/Button"
                        // "/ti/drivers/apps/LED"
                    ]
                }
            ]
        }
    ];
} else if (deviceId.match(/CC35.*/)) {
    /* CC35XX */
    topModules = [
        {
            displayName: displayName,
            description: description,
            modules: [
                // "/ti/display/Display",
                // "/ti/drivers/ADC",
                "/ti/drivers/Board",
                // "/ti/drivers/Capture",
                // "/ti/drivers/Crypto",
                "/ti/drivers/DMA",
                "/ti/drivers/GPIO",
                "/ti/drivers/GPTimer",
                "/ti/drivers/I2C",
                // "/ti/drivers/I2S",
                // "/ti/drivers/ITM",
                // "/ti/drivers/NVS",
                "/ti/drivers/Power",
                "/ti/drivers/PWM",
                // "/ti/drivers/SD",
                // "/ti/drivers/SPI",
                "/ti/drivers/UART2"
                // "/ti/drivers/Watchdog"
            ],
            "categories": [
                {
                    "displayName": "TI Driver Apps",
                    "description": "TI Drivers Apps Configuration",
                    "modules": [
                        "/ti/drivers/apps/Button"
                        // "/ti/drivers/apps/LED" /* Disable because there is no PWM module yet */
                    ]
                }
            ]
        }
    ];
}

let templates = [
    {
        "name": "/ti/drivers/templates/Board.c.xdt",
        "outputPath": "ti_drivers_config.c",
        "alwaysRun": false
    },
    {
        "name": "/ti/drivers/templates/Board.h.xdt",
        "outputPath": "ti_drivers_config.h",
        "alwaysRun": false
    }
];

exports = {
    displayName: displayName,
    topModules: topModules,
    templates: templates
};
