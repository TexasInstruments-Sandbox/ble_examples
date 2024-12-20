/******************************************************************************

@file  power_shutdown.c

@brief This file contains the application main functionality

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2024, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

******************************************************************************
*****************************************************************************/

#include <string.h>
#include "ti_ble_config.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Power.h>
#include <Power_shutdown/power_shutdown.h>
#include <HID/hiddev.h>
#include <KeyScan/Key_scan.h>

#define RC_IDLE_TIMEOUT         900000  // 15 minutes
#define CLOCK_UNITS_MS          1000  // convert to ms

ClockP_Handle rc_timeout_clkHandle_conn;
ClockP_Handle rc_timeout_clkHandle_adv;
extern uint16_t connHandle;

void RestartShutdownClock(void)
{
    ClockP_stop(rc_timeout_clkHandle_conn);
    int32_t clockTicks = RC_IDLE_TIMEOUT * (CLOCK_UNITS_MS);
    ClockP_setTimeout(rc_timeout_clkHandle_conn, clockTicks);
    ClockP_start(rc_timeout_clkHandle_conn);
}

void TriggerShutdown(void)
{
   // Config GPIO to wakeup on interrupt
   GPIO_clearInt(CONFIG_GPIO_BUTTON_0_INPUT);
   GPIO_setConfig(CONFIG_GPIO_BUTTON_0_INPUT, GPIO_CFG_IN_NOPULL | GPIO_CFG_OUT_HIGH | GPIO_CFG_SHUTDOWN_WAKE_HIGH);

   GPIO_clearInt(CONFIG_GPIO_BUTTON_1_INPUT);
   GPIO_setConfig(CONFIG_GPIO_BUTTON_1_INPUT, GPIO_CFG_IN_NOPULL | GPIO_CFG_OUT_HIGH | GPIO_CFG_SHUTDOWN_WAKE_HIGH);

   GPIO_write(CONFIG_KEY_COL_1, 1);
   GPIO_write(CONFIG_KEY_COL_2, 1);
   GPIO_write(CONFIG_KEY_COL_3, 1);
   GPIO_write(CONFIG_KEY_COL_4, 1);
   GPIO_write(CONFIG_KEY_COL_5, 1);

   GPIO_clearInt(CONFIG_GPIO_KEY_0_INPUT);
   GPIO_setConfig(CONFIG_GPIO_KEY_0_INPUT, GPIO_CFG_IN_PD | GPIO_CFG_SHUTDOWN_WAKE_HIGH);

   GPIO_clearInt(CONFIG_GPIO_KEY_1_INPUT);
   GPIO_setConfig(CONFIG_GPIO_KEY_1_INPUT, GPIO_CFG_IN_PD | GPIO_CFG_SHUTDOWN_WAKE_HIGH);

   GPIO_clearInt(CONFIG_GPIO_KEY_2_INPUT);
   GPIO_setConfig(CONFIG_GPIO_KEY_2_INPUT, GPIO_CFG_IN_PD | GPIO_CFG_SHUTDOWN_WAKE_HIGH);

   GPIO_clearInt(CONFIG_GPIO_KEY_5_INPUT);
   GPIO_setConfig(CONFIG_GPIO_KEY_5_INPUT, GPIO_CFG_IN_PD | GPIO_CFG_SHUTDOWN_WAKE_HIGH);

   int_fast16_t  Status = PowerCC23X0_notify(PowerLPF3_ENTERING_SHUTDOWN);

   if(Status == Power_SOK)
   {
       Power_shutdown(0,0);
   }
}

void triggerDisconnect(void){

    GAP_TerminateLinkReq(connHandle, HCI_DISCONNECT_REMOTE_USER_TERM);
}

static void rc_timeout_clkHandleCB_conn(void)
{
    BLEAppUtil_invokeFunctionNoData(triggerDisconnect);
}

void ConfigPowerShutdownConn_start(void) {

    ClockP_Params clockpParams;
    static ClockP_Struct rc_timeout_clkStruct;
    ClockP_Params_init(&clockpParams);
    uint32_t clockTicks = RC_IDLE_TIMEOUT * (CLOCK_UNITS_MS);     //set timeout
    clockpParams.period = clockTicks;   //config for one-shot timer
    clockpParams.startFlag = true;     // start now
    clockpParams.arg = (uintptr_t)rc_timeout_clkHandleCB_conn;
    rc_timeout_clkHandle_conn = ClockP_construct(&rc_timeout_clkStruct, (void *)BLEAppUtil_invokeFunctionNoData, clockTicks, &clockpParams); // Initialize clock instance.

}

static void rc_timeout_clkHandleCB_adv(void)
{
    TriggerShutdown();
}

void ConfigPowerShutdownAdv_start(void) {

    ClockP_Params clockpParams;
    static ClockP_Struct rc_timeout_clkStruct;
    ClockP_Params_init(&clockpParams);
    uint32_t clockTicks = RC_IDLE_TIMEOUT * (CLOCK_UNITS_MS);     //set timeout
    clockpParams.period = clockTicks;   //config for one-shot timer
    clockpParams.startFlag = true;     // start now
    clockpParams.arg = (uintptr_t)rc_timeout_clkHandleCB_adv;
    rc_timeout_clkHandle_adv = ClockP_construct(&rc_timeout_clkStruct, (void *)BLEAppUtil_invokeFunctionNoData, clockTicks, &clockpParams); // Initialize clock instance.
}
