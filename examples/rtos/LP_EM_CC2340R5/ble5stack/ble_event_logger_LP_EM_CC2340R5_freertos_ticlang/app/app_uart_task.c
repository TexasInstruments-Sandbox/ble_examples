/******************************************************************************

@file  app_uart_task.c

@brief This file implements the uart functionality

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

/* For usleep() */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/Watchdog.h>
  /* POSIX Header files */

/* RTOS header files */
#include <FreeRTOS.h>
#include <global.h>
#include <task.h>

/* Stack size in bytes */


/* Driver configuration */
#include "ti_drivers_config.h"
#include "bcomdef.h"

/* Driver configuration */
#include <ti/drivers/dpl/ClockP.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <string.h>
#include <ti/drivers/dpl/ClockP.h>
#include "global.h"


//*****************************************************************************
//! Prototypes
//*****************************************************************************
void Uart_ReadCallback(UART2_Handle handle, void *rxBuf, size_t size,void *userArg, int_fast16_t status);
void splitStringToIntegers( char *str, int *numbers, int *count);
void processPacket(size_t size);
void handleCommand(char command, char *payload, int length);
void UART_TimeOut(void);

//*****************************************************************************
//! Defines
//*****************************************************************************
#define MAX_PACKET_SIZE      256
#define HEADER_SIZE          4       // SOP (1) + Command (1) + Length (2)
#define FOOTER_SIZE          1       // EOP (1)
#define SOP                 'S'
#define EOP                 'E'

//ClockP
static ClockP_Struct gattLimit_clkStruct;
ClockP_Handle gattLimit_clkHandle;
ClockP_Params clockpParams;

//UART
UART2_Handle uart;
UART2_Params uartParams;
size_t bytesRead;
size_t bytesWritten = 0;
uint32_t status     = UART2_STATUS_SUCCESS;
char rxBuf[MAX_PACKET_SIZE];
char txBuf[MAX_PACKET_SIZE];
int firstCommand=1;

//*****************************************************************************
//! Functions
//*****************************************************************************

/* fn Uart_ReadCallback
 * breif UART_EXPECTED PACKET  SOP(1)+ CMD(1) + LENGTH OF PAYLOAD(2) + PAYLOAD + EOP(1)
*        CMD For Setting BASE_TIME is '1'
*        PACKET FOR TIME BASE: S1192024-11-11-11-11-11E : S 1 19 2024-11-11-11-11-11 E
*        HEADER_SIZE : 4, SOP+CMD+PAYLOAD LENGTH
*        FOOTER_SIZE : 1, EOP
*  @param handle:UART Handle
*  @param rxBuf: Rx Buffer pointer
*  @param size: size of packet received
**/
void Uart_ReadCallback(UART2_Handle handle, void *rxBuf, size_t size,void *userArg, int_fast16_t status)
{

    static size_t expectedSize = 0;
    if (expectedSize == 0) {

        if (size == HEADER_SIZE) {
            char *data = (char *)rxBuf;
            int length = (data[2]- 48)*10+ (data[3]-48);

            expectedSize = HEADER_SIZE + length + FOOTER_SIZE;

            if (expectedSize <= MAX_PACKET_SIZE) {

                UART2_read(handle, rxBuf + HEADER_SIZE,expectedSize - HEADER_SIZE, &bytesRead);

            } else {

                expectedSize = 0;
                UART2_read(handle, rxBuf, HEADER_SIZE,&bytesRead);

            }
        }
        else
        {
            expectedSize = 0;
            UART2_read(handle, rxBuf, HEADER_SIZE,&bytesRead);
        }
    } else {
        // Full packet received, process it
        processPacket(expectedSize);
        expectedSize = 0;
        // Prepare to receive the next packet header
        UART2_read(handle, rxBuf, HEADER_SIZE,&bytesRead);
    }
}


/* ProcessPacket:
 * Validate the incoming Packet
 * @param   size: expectedSize of the packet=HEADER_SIZE + length + FOOTER_SIZE;
**/
void processPacket(size_t size) {
    if (rxBuf[0] == SOP && rxBuf[size - 1] == EOP) {

        char command = rxBuf[1];
        int length = (rxBuf[2]- 48)*10 + (rxBuf[3]-48); //int conversion form char to get length

        // Validate length
        if (length == size - HEADER_SIZE - FOOTER_SIZE) {
            char *payload = (char*)&rxBuf[4];
            handleCommand(command, payload, length);
        }
    }
}

/* handleCommand:
 * Handles different Commands given through UART
 * @param   command: char command '1' for TimeBase Config
 * @param   paylaoad: Payload for the command
 * @param   length: length of the payload
**/
void handleCommand(char command, char *payload, int length) {
    int params[6];
    int count=0;
    switch (command) {

        case '1': //CMD FOR SETTING TIME BASE
             splitStringToIntegers(payload, params, &count);
             if(count==6)
             {

                BaseTimeStamp.year=params[0];
                BaseTimeStamp.month=params[1];
                BaseTimeStamp.day=params[2];
                BaseTimeStamp.hour=params[3];
                BaseTimeStamp.minute=params[4];
                BaseTimeStamp.second=params[5];

                HWREG(RTC_BASE + RTC_O_CTL)|=00000001U;  //Reset RTC Clock

             }
            break;

       //TODO: Implement Other Commands IF needed
        default:

            break;
    }


    UART_TimeOut();
    if(firstCommand==1)
      {
              firstCommand=0;
              BLEAppUtil_invokeFunctionNoData((InvokeFromBLEAppUtilContext_t)ADV_start);
              BLEAppUtil_invokeFunctionNoData((InvokeFromBLEAppUtilContext_t)StartCalendarClockP);
      }

}

// Splits Payload into the calendar data
void splitStringToIntegers( char *str, int *numbers, int *count) {
    char *token;
    char *tempStr;
    int index = 0;

    tempStr = strdup(str);


    token = strtok(tempStr, "-");


    while (token != NULL) {
        numbers[index] = atoi(token);
        index++;
        token = strtok(NULL, "-");
    }

    *count = index;

    free(tempStr);
}




// Used to close UART after a given Interval
void UART_TimeOut(void)
{

    GPIO_toggle(CONFIG_GPIO_LED_RED);

    UART2_close(uart);
    GPIO_setCallback(CONFIG_PIN_UART_RX, UartTask);
    GPIO_enableInt(CONFIG_PIN_UART_RX);

    ClockP_stop (gattLimit_clkHandle);
    ClockP_delete (gattLimit_clkHandle);
}

//ClockP Setup for Calling the UART_TimeOut after UART_TIMEOUT_MS time elasped
void UART_ClockP()
{
   ClockP_Params_init(&clockpParams);
   uint32_t clockTicks = UART_TIMEOUT_MS * (CLOCK_MS);
   clockpParams.period = clockTicks; //one-shot timer
   clockpParams.startFlag = true;
   clockpParams.arg = (uintptr_t)UART_TimeOut;
   // Initialize clock instance.
   gattLimit_clkHandle = ClockP_construct(&gattLimit_clkStruct, (void *)BLEAppUtil_invokeFunctionNoData, clockTicks, &clockpParams);
}


/*
 *  ======== mainThread ========
 */
void UartTask(uint_least8_t index)
{
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;
    uartParams.readMode = UART2_Mode_CALLBACK;
    uartParams.readCallback = Uart_ReadCallback;
    uartParams.readReturnMode = UART2_ReadReturnMode_PARTIAL;

    uart = UART2_open(CONFIG_DISPLAY_UART, &uartParams);
       if (uart == NULL)
       {
           /* UART2_open() failed */
           while (1) {}
       }

    UART_ClockP(); // ClockP instance to close UART after timeout
    UART2_read(uart, rxBuf, HEADER_SIZE,&bytesRead);
}

