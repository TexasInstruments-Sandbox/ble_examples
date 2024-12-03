/******************************************************************************

@file  app_calendar_task.c

@brief This file implements the calendar functionality

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

#include <stdio.h>
#include "ti_drivers_config.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <app_main.h>
#include <global.h>
#include "bcomdef.h"

//local function
int is_leap_year(int year);




uint64_t RTC_cnt_524M;
uint64_t RTC_cnt_8U;

Timestamp BaseTimeStamp = {2021, 1, 1, 0, 0, 0};
Timestamp CurrentTimeStamp= {0, 0, 0, 0, 0, 0};
Timestamp LastEventTimeStamp= {0, 0, 0, 0, 0, 0};


// check if a given year is a lear year or not
int is_leap_year(int year)
{
    return (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
}


// return the number of days in a given month
int days_in_month(int year, int month) {
    if (month == 2) {
        return is_leap_year(year) ? 29 : 28;
    }
    const int days_in_month[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    return days_in_month[month - 1];
}

// return new_timestamp = old_timestamp + offset seconds, in the timestamp format
void apply_offset(Timestamp *ts,long int offset_seconds )
{
       ts->second += offset_seconds;
       ts->minute += ts->second / 60;
       ts->second = ts->second % 60;

       ts->hour += ts->minute / 60;
       ts->minute = ts->minute % 60;

       ts->day += ts->hour / 24;
       ts->hour = ts->hour % 24;

       while (ts->day > days_in_month(ts->year, ts->month)) {
           ts->day -= days_in_month(ts->year, ts->month);
           ts->month++;
           if (ts->month > 12) {
               ts->month = 1;
               ts->year++;
           }
       }

}

// updates the current timestamp using the RTC and Base Timestamp
void modify_timestamp()
{

    long int offset_seconds;
    RTC_cnt_8U = HWREG(RTC_BASE + RTC_O_TIME8U) & 0xFFFFFFFF;
    RTC_cnt_524M = HWREG(RTC_BASE + RTC_O_TIME524M) & 0xFFFFFFFF;
    while ((RTC_cnt_524M & 0xFFFF) != ((RTC_cnt_8U >> 16) & 0xFFFF))
    {
       RTC_cnt_8U = HWREG(RTC_BASE + RTC_O_TIME8U);
       RTC_cnt_524M = HWREG(RTC_BASE + RTC_O_TIME524M);
    }

    offset_seconds = (RTC_cnt_524M * 524) + (RTC_cnt_8U & 0xFFFF)*8/1000;
    offset_seconds = (uint32_t)offset_seconds/1000;

    Timestamp ts;
    ts = BaseTimeStamp;
    apply_offset(&ts,offset_seconds);

    CurrentTimeStamp=ts;

    // TODO: if needed after certain time change the base time stamp to current time and reset RTC

}


