/******************************************************************************

@file  app_event_task.c

@brief This file implements the GPIO event detection and logging 

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

#include <common/Profiles/logger_gatt/logger_gatt_profile.h>
#include <ti/drivers/GPIO.h>
#include "ti_drivers_config.h"
#include <pthread.h>
#include <semaphore.h>
#include "bcomdef.h"
#include <global.h>


//*****************************************************************************
//! Defines
//*****************************************************************************
#define THREADSTACKSIZE 1024
#define EventSize 5

extern void BLEAppUtil_stackRegister();

/* Local Function */
int ValidateEvent();

/*Local Variable*/
static sem_t sem;
int EventCount=0;
int EventFlag;
extern int firstCommand;


  /*********************************************************************
  * @fn      EventButtonPressed
  *
  * @brief   Send to notify the value only if the left button has been pressed
  *
  * @param   index
  *
  * @return  None.
  */
  void EventButtonPressed(uint_least8_t index)
  {
  if(!firstCommand)
  sem_post(&sem);
  }

  // validate event by checking if the new_event_timestamp > last_event_time + threshold seconds
  int ValidateEvent()
  {
      Timestamp  ts_temp = LastEventTimeStamp;
      Timestamp ts2 = CurrentTimeStamp;

      // get the timestamp==> last_event_time + Threshold
      apply_offset(&ts_temp,EventTimeThreshold);

      if (ts_temp.year < ts2.year) return 1;
      if (ts_temp.year > ts2.year) return 0;

      if (ts_temp.month < ts2.month) return 1;
      if (ts_temp.month > ts2.month) return 0;

      if (ts_temp.day < ts2.day) return 1;
      if (ts_temp.day > ts2.day) return 0;

      if (ts_temp.hour < ts2.hour) return 1;
      if (ts_temp.hour > ts2.hour) return 0;

      if (ts_temp.minute < ts2.minute) return 1;
      if (ts_temp.minute > ts2.minute) return 0;
      if (ts_temp.second <= ts2.second) return 1;
      return 0;

  }

/*
 *  ======== mainThread ========
 */
  void *EventThread(void *arg0)
  {
    BLEAppUtil_stackRegister();
    int32_t semStatus;
    /* Create semaphore */

    semStatus = sem_init(&sem, 0, 0);
    if (semStatus != 0)
    {
      /* Error creating semaphore */
      while (1) {}
    }

    while (1)
    {
      /* Do not write until read callback executes */
      sem_wait(&sem);
      modify_timestamp();  //Update Current Timestamp
      //Validate EventEvent
      if(!ValidateEvent()) continue;
      // update the calendar
      LastEventTimeStamp=CurrentTimeStamp;

      // uint8 format to update the gatt profile
      uint8 EventTimeStamp[GATTPROFILE_TIMESTAMP_LEN]={
                       LastEventTimeStamp.year/100,
                       LastEventTimeStamp.year%100,
                       LastEventTimeStamp.month,
                       LastEventTimeStamp.day,
                       LastEventTimeStamp.hour,
                       LastEventTimeStamp.minute,
                       LastEventTimeStamp.second,

      };
      SimpleGattProfile_setParameter( EventCount%EventSize, GATTPROFILE_TIMESTAMP_LEN,
                                      EventTimeStamp );  // Update the GATT PROFILE
      EventCount++;
      GPIO_toggle(CONFIG_GPIO_LED_RED);

      Event_ADV_Data_Update(); // update the advertisement data


    }
  }

void EventMain(void)
{
  pthread_t thread;
  pthread_attr_t attrs;
  struct sched_param priParam;
  int retc;

  /* Initialize the attributes structure with deEvent values */
  pthread_attr_init(&attrs);

  /* Set priority, detach state, and stack size attributes */
  priParam.sched_priority = 5; // Lower the priority of this task
  retc = pthread_attr_setschedparam(&attrs, &priParam);
  retc |= pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
  retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
  if (retc != 0)
  {
    /* failed to set attributes */
    while (1) {}
  }
  retc = pthread_create(&thread, &attrs, EventThread, NULL);

  GPIO_setCallback(CONFIG_GPIO_Event_BTN, EventButtonPressed);
  GPIO_enableInt(CONFIG_GPIO_Event_BTN);

  if (retc != 0)
  {

    /* pthread_create() failed */
    while (1) {}
  }
}
