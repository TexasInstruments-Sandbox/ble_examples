/******************************************************************************

@file  global.h

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

#ifndef GLOBAL_H_
#define GLOBAL_H_

typedef struct {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
} Timestamp;


#define CLOCK_MS 1000 //MS UNIT
#define CALENDAR_UPDATE_MS  1000
#define UART_TIMEOUT_MS  10000
#define ADV_INTERVAL_MS 1000
#define EventTimeThreshold 10 // Event Time Threshold in seconds to validate Event


#define RTC_O_CTL            0x00000004U
#define RTC_O_TIME8U         0x00000018U
#define RTC_O_TIME524M       0x0000001CU


void ADV_start();
void Event_ADV_Data_Update();
void StartCalendarClockP();

// return new_timestamp = old_timestamp + offset seconds, in the timestamp format
void apply_offset(Timestamp *ts,long int offset_seconds );

// updates the current timestamp using the RTC and Base Timestamp
void modify_timestamp();

/*********************************************************************
* @fn      EventButtonPressed
*
* @brief   Send to notify the value only if the left button has been pressed
*
*/
void EventButtonPressed(uint_least8_t index);

void UartTask(uint_least8_t index);




extern Timestamp BaseTimeStamp;
extern Timestamp CurrentTimeStamp;
extern Timestamp LastEventTimeStamp;
#endif /* GLOBAL_H_ */
