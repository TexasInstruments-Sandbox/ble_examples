/******************************************************************************

@file  util.h

@brief This example file demonstrates how to activate the peripheral role with
the help of BLEAppUtil APIs.

Two structures are used for event handling, one for connection events and one
for advertising events.
In each, eventMask is used to specify the events that will be received
and handled.
In addition, fill the BLEAppUtil_AdvInit_t structure with variables generated
by the Sysconfig.

In the events handler functions, write what actions are done after each event.
In this example, after a connection is made, activation is performed for
re-advertising up to the maximum connections.

In the Peripheral_start() function at the bottom of the file, registration,
initialization and activation are done using the BLEAppUtil API functions,
using the structures defined in the file.

More details on the functions and structures can be seen next to the usage.

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

#ifndef UTIL_H
#define UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES
 */
#include <stdbool.h>
#ifdef FREERTOS
#include <mqueue.h>
#include <pthread.h>
    #ifndef Event_Handle
    #define Event_Handle void*
    #endif
#else
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#endif

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/**
 * @brief   Util Queue Event ID
 *
 * In order to wake an application when a message is inserted into its
 * queue, an event must be posted.  Util reserved Event Id 30 for a generic
 * queue event.
 */
#ifdef FREERTOS
#define UTIL_EVENT_ID_NONE       (0)   // Event_Id_NONE
#define UTIL_EVENT_ID_00         (0x1) // Event_Id_00
#define UTIL_EVENT_ID_01         (0x2) // Event_Id_01
#define UTIL_EVENT_ID_02         (0x4) // Event_Id_02
#define UTIL_EVENT_ID_03         (0x8) // Event_Id_03
#define UTIL_EVENT_ID_04         (0x10)// Event_Id_04
#define UTIL_EVENT_ID_05         (0x20)// Event_Id_05
#define UTIL_EVENT_ID_06         (0x40)// Event_Id_06
#define UTIL_QUEUE_EVENT_ID      (0x00100000)//Event_Id_30
#define UTIL_TL_CB_EVENT         UTIL_EVENT_ID_00 // Event_Id_00
#else
#define UTIL_QUEUE_EVENT_ID Event_Id_30
#endif
/*********************************************************************
 * TYPEDEFS
 */

#ifdef FREERTOS
typedef struct Clock_Struct
{
    timer_t clock;
    void *cback;
    uint32_t arg;
    sigevent evnt;
    pthread_attr_t timerThrdAttr;
    struct itimerspec timeVal;
    uint8_t isActive;
}Clock_Struct;
#endif

typedef struct
{
  uint16_t event; // Event type.
  uint8_t state; // Event state;
}appEvtHdr_t;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * API FUNCTIONS
 */

/**
 * @brief   Initialize a TIRTOS Clock instance.
 *
 * @param   pClock        - pointer to clock instance structure.
 * @param   clockCB       - callback function upon clock expiration.
 * @param   clockDuration - longevity of clock timer in milliseconds
 * @param   clockPeriod   - duration of a periodic clock, used continuously
 *                          after clockDuration expires.
 * @param   startFlag     - TRUE to start immediately, FALSE to wait.
 * @param   arg           - argument passed to callback function.
 *
 * @return  Clock_Handle  - a handle to the clock instance.
 */
#ifdef FREERTOS
void* Util_constructClock(Clock_Struct *entry, void *clockCB,
                          uint32_t clockDuration, uint32_t clockPeriod,
                          uint8_t startFlag, uint32_t arg);

void Clock_destruct(Clock_Struct *structP);
#else
extern Clock_Handle Util_constructClock(Clock_Struct *pClock,
                                        Clock_FuncPtr clockCB,
                                        uint32_t clockDuration,
                                        uint32_t clockPeriod,
                                        uint8_t startFlag,
                                        UArg arg);
#endif

/**
 * @brief   Start a clock.
 *
 * @param   pClock - pointer to clock struct
 *
 * @return  none
 */
extern void Util_startClock(Clock_Struct *pClock);

/**
 * @brief   Restart a clock by changing the timeout.
 *
 * @param   pClock - pointer to clock struct
 * @param   clockTimeout - longevity of clock timer in milliseconds
 *
 * @return  none
 */
extern void Util_restartClock(Clock_Struct *pClock, uint32_t clockTimeout);

/**
 * @brief   Determine if a clock is currently active.
 *
 * @param   pClock - pointer to clock struct
 *
 * @return  TRUE or FALSE
 */
extern bool Util_isActive(Clock_Struct *pClock);

/**
 * @brief   Stop a clock.
 *
 * @param   pClock - pointer to clock struct
 *
 * @return  none
 */
extern void Util_stopClock(Clock_Struct *pClock);

/**
 * @brief   Reschedule a clock by changing the timeout and period values.
 *
 * @param   pClock - pointer to clock struct
 * @param   clockPeriod - longevity of clock timer in milliseconds
 * @return  none
 */
extern void Util_rescheduleClock(Clock_Struct *pClock, uint32_t clockPeriod);

/**
 * @brief   Initialize an RTOS queue to hold messages from profile to be
 *          processed.
 *
 * @param   pQueue - pointer to queue instance structure.
 *
 * @return  A queue handle.
 */
#ifdef FREERTOS
extern void Util_constructQueue(mqd_t *pQueue);
#else
extern Queue_Handle Util_constructQueue(Queue_Struct *pQueue);
#endif

/**
 * @brief   Creates a queue node and puts the node in RTOS queue.
 *
 * @param   msgQueue - queue handle.
 *
 * @param   event - the thread's event processing event that this queue is
 *                  associated with.
 *
 * @param   pMsg - pointer to message to be queued
 *
 * @return  TRUE if message was queued, FALSE otherwise.
 */
#ifdef FREERTOS
uint8_t Util_enqueueMsg(mqd_t msgQueue,
                        Event_Handle event,
                        uint8_t *pMsg);
#else

extern uint8_t Util_enqueueMsg(Queue_Handle msgQueue,
                               Event_Handle event,
                               uint8_t *pMsg);
#endif

/**
 * @brief   Dequeues the message from the RTOS queue.
 *
 * @param   msgQueue - queue handle.
 *
 * @return  pointer to dequeued message, NULL otherwise.
 */
#ifdef FREERTOS
uint8_t *Util_dequeueMsg(mqd_t msgQueue);
#else
extern uint8_t *Util_dequeueMsg(Queue_Handle msgQueue);
#endif

/**
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @param   pAddr - BD address
 *
 * @return  BD address as a string
 */
extern char *Util_convertBdAddr2Str(uint8_t *pAddr);

/**
 * @brief   Check if contents of buffer matches byte pattern.
 *
 * @param   pBuf    - buffer to check
 * @param   pattern - pattern to match
 * @param   len     - len of buffer (in bytes) to iterate over
 *
 * @return  TRUE if buffer matches the pattern, FALSE otherwise.
 */
extern uint8_t Util_isBufSet(uint8_t *pBuf, uint8_t pattern, uint16_t len);

#ifdef  FREERTOS
/*********************************************************************
 * @fn      Event_post
 *
 * @brief   DPL that is used to signal events. If a task is waiting
 *          for the event and the event conditions are met, post()
 *          unblocks the task. If no tasks are waiting, post() simply
 *          registers the event with the event object and returns.
 *
 * @param   eventQueue - handle of a event queue
 * @param   msg - mask of eventIds to post (must be non-zero)
 */
extern void Event_post(mqd_t eventQueue, uint32_t msg);
#endif

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* UTIL_H */

/** @} End Util */
