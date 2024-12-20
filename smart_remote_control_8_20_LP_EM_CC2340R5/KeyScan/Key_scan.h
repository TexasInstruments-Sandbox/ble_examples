/******************************************************************************

@file  Key_scan.h

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

#ifndef ti_drivers_Key__include
#define ti_drivers_Key__include

#include <stdint.h>
#include <stdbool.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/dpl/ClockP.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_KEY_0                     0
#define CONFIG_KEY_1                     1
#define CONFIG_KEY_2                     2
#define CONFIG_KEY_5                     3

#define CONFIG_TI_DRIVERS_KEY_COUNT      4
#define CONFIG_KEY_COUNT                 4

#define CONFIG_GPIO_KEY_0_INPUT         0
#define CONFIG_GPIO_KEY_1_INPUT         24
#define CONFIG_GPIO_KEY_2_INPUT         7
#define CONFIG_GPIO_KEY_5_INPUT         8

/* Number of user defined Key configurations */
extern const uint_least8_t Key_count;

/*!
 *  @brief    Key configuration
 *  Each #Key_Config represents a single physical key. It contains
 *  pointers to the key's #Key_HWAttrs and #Key_Object. The user must
 *  statically allocate all of these structures.
 */
typedef struct Key_Config
{
    /*! Pointer to a #Key_Object struct */
    void *object;

    /*! Pointer to a #Key_HWAttrs structure */
    void const *hwAttrs;
} Key_Config;

/*!
 *  @brief    A handle that is returned from a Key_open() call.
 *
 *  User will use this handle to interact with a given key instance.
 */
typedef struct Key_Config *Key_Handle;

/*!
 *  @brief    Key State
 *  @private
 *
 * This enumeration describes whether the key is pressed or released etc.
 * This is for internal state machine handling.
 */
typedef enum Key_State
{
    /*! Edge detected, debouncing */
    Key_PRESSING             = 1,
    /*! Press verified, not detecting longpress */
    Key_PRESSED              = 2,
    /*! Press verified, waiting for longpress timeout. */
    Key_LONGPRESSING         = 3,
    /*! Longpress verified, waiting for neg-edge */
    Key_LONGPRESSED          = 4,
    /*! Neg-edge received, debouncing */
    Key_RELEASING            = 5,
    /*! Neg-edge received after long-press, debouncing. */
    Key_RELEASING_LONG       = 6,
    /*! Key release verified. */
    Key_RELEASED             = 7,
    /*! EDGE detected doublepress */
    Key_DBLPRESS_DETECTION   = 8,
    /*! EDGE detected doublepress */
    Key_DBLPRESSING          = 9,
    /*! DOUBLE PRESS verified, waiting for neg edge */
    Key_DBLPRESSED           = 10,
    /*! DOUBLE PRESS verified, waiting for neg edge k*/
    Key_RELEASING_DBLPRESSED = 11,

    Key_PRESSING_REPEAT  = 12,
    Key_LONGPRESSED_REPEAT   = 13,
} Key_State;

/*!
 *  @brief Key event flags
 *
 *  The event flags can be used by the user to subscribe to specific kinds of
 *  key actions and by the driver to signal which event caused a callback.
 */
typedef enum Key_Events
{
    /*! Key pressed down, may or may not subsequently have been released */
    Key_EV_PRESSED       = 0x01,
    /*! Key held down for more than tLongpress (ms) */
    Key_EV_LONGPRESSED   = 0x02,
    /*! Key released after press or longpress */
    Key_EV_RELEASED      = 0x04,
    /*! Key was pressed and released, but was not a long press */
    Key_EV_CLICKED       = 0x08,
    /*!
     * Key was pressed and released, and held for longer than
     * longPressDuration (ms)
     */
    Key_EV_LONGCLICKED   = 0x10,
    /*! Key was pressed when double click detection was active */
    Key_EV_DOUBLECLICKED = 0x20,

    Key_EV_PRESSED_REPEAT = 0x40,

} Key_Events;

/*! @brief Event subscription and notification mask type */
typedef uint8_t Key_EventMask;

/*!
 *  @brief    A handler to receive key callbacks.
 */
typedef void (*Key_Callback)(Key_Handle keyHandle, Key_EventMask keyEvents);

/*!
 *  @brief    Key Pull settings
 *
 * This enumeration defines whether the key is active low (PULL_UP) or
 * active high (PULL_DOWN) and is used to control internal logic.
 */
typedef enum Key_Pull
{
    /* NOTE: DO NOT change the values of DOWN/UP from (0,1) */
    Key_PULL_DOWN = 0, /*!< Key is PULLED DOWN. */
    Key_PULL_UP   = 1, /*!< Key is PULLED UP. */
} Key_Pull;

/*!
 *  @brief    Hardware specific settings for a key
 *
 *  This structure should be defined and provided by the application.
 */
typedef struct Key_HWAttrs
{
    /*! GPIO configuration index. */
    uint_least8_t gpioIndex;

    /*! Whether the key is active high or active low. */
    Key_Pull pullMode;

    /*! True/False whether the pullup on the GPIO pin should be enabled */
    uint32_t internalPullEnabled;
} Key_HWAttrs;

/*!
 *  @brief  Key State Variables
 *  @private
 *
 *  Each key instance needs set of variables to monitor its state.
 *  We group these variables under the structure Key_State.
 *
 *  @sa     Key_Params_init()
 */
typedef struct Key_StateVariables
{
    /*! Key state */
    Key_State state;
    /*! Key pressed start time in milliseconds(ms) */
    uint32_t pressedStartTime;
    /*! Key pressed duration (ms) */
    uint32_t lastPressedDuration;
} Key_StateVariables;

/*!
 *  @brief    Internal to Key module. Members should not be accessed
 *            by the application.
 */
typedef struct Key_Object
{
    /*! Handle to clock used for timing */
    ClockP_Handle clockHandle;

    /*! State variables for handling the debounce state machine */
    Key_StateVariables keyStateVariables;

    /*! Event subscription mask for the key */
    Key_EventMask keyEventMask;

    /*! Callback function for the key */
    Key_Callback keyCallback;

    /*! Debounce duration for the key in milliseconds(ms) */
    uint32_t debounceDuration;

    /*! Long press duration is milliseconds(ms) */
    uint32_t longPressDuration;

    /*! Repat interval is milliseconds(ms) */
    uint32_t repeatInterval;

    /*! Double press detection timeout is milliseconds(ms) */
    uint32_t doublePressDetectiontimeout;
} Key_Object;

/*!
 *  @brief  Key Parameters
 *
 *  Key parameters are used with the Key_open() call. Default values for
 *  these parameters are set using Key_Params_init().
 *
 *  @sa     Key_Params_init()
 */
typedef struct Key_Params
{
    /*! Debounce duration for the key in milliseconds(ms) */
    uint32_t debounceDuration;

    /*! Long press duration is milliseconds(ms) */
    uint32_t longPressDuration;

    /*! Double press detection timeout is milliseconds(ms) */
    uint32_t doublePressDetectiontimeout;

    /*! Key repeat detection timeout is milliseconds(ms) */
    uint32_t repeatIntervaltimeout;

    /*! Event subscription mask for the key */
    Key_EventMask keyEventMask;

    /*! A #Key_Callback that is called when a masked event occurs. */
    Key_Callback keyCallback;
} Key_Params;

/*!
 *  @brief  Function to close a Key specified by the #Key_Handle
 *
 *  @pre        Key_open() had to be called first.
 *
 *  @param[in]  handle    A #Key_Handle returned from Key_open() call
 *
 *  @return     True on success or false upon failure.
 */
extern void Key_close(Key_Handle handle);

/*!
 *  @brief  Function to initialize Key driver.
 */
extern void Key_init(void);

/*!
 *  @brief  Function to open a given Key
 *
 *  Function to open a key instance corresponding to a #Key_Config in the
 *  Key_config array. The GPIO configurations must exist prior to calling
 *  this function. The #Key_Params may be used to specify runtime parameters.
 *
 *  @pre       Key_init() has to be called first
 *
 *  @param[in] keyIndex    Logical key number indexed into
 *                            the Key_config table
 *
 *  @param[in] *params        A pointer to #Key_Params structure. If NULL,
 *                            it will use default values.
 *
 *  @return  A #Key_Handle on success, or a NULL on failure.
 *
 *  @sa      Key_init()
 *  @sa      Key_Params_init()
 *  @sa      Key_close()
 */
extern Key_Handle Key_open(uint_least8_t keyIndex, Key_Params *params);

/*!
 *  @brief  Function to initialize a #Key_Params struct to its defaults
 *
 *  @param[in] params   A pointer to a #Key_Params structure that will be
 *                      initialized.
 *
 *  Default values
 *  ------------------------------------------------------------------
 *  parameter        | value        | description              | unit
 *  -----------------|--------------|--------------------------|------------
 *  debounceDuration | 10           | debounce duration        | ms
 *  longPressDuration| 2000         | long press duration      | ms
 *  keyEventMask     | 0xFF         | subscribed to all events | NA
 */
extern void Key_Params_init(Key_Params *params);

/*!
 *  @brief  Function to return the lastPressedDuration (valid only for short
 *          press, long press)
 *
 *  The API returns last pressed duration and it is valid only for shortpress,
 *  longpress. If this API is called after receiving an event click or long
 *  click then the API returns the press duration which is time delta between
 *  the press and release of the key.
 *  @note This API call is only valid after a click or long click and not after
 *  a double click.
 *
 *  @param[in] handle   Pointer to the #Key_Handle of the desired key.
 *
 *  @return    time duration in milliseconds.
 *
 */
extern uint32_t Key_getLastPressedDuration(Key_Handle handle);

/*!
 *  @brief     Function to set callback function for the key instance
 *
 *  @param[in] handle           A #Key_Handle returned from Key_open()
 *
 *  @param[in] keyCallback   key callback function
 *
 */
extern void Key_setCallback(Key_Handle handle, Key_Callback keyCallback);

/*!
 *  @brief  This is the GPIO interrupt callback function which is called on a
 *          key press or release. This is internally used by key module.
 *
 *  This function is internally used by key module for receiving the GPIO
 *  interrupt callbacks. This is exposed to the application for wake up cases.
 *  In some of the MCUs, when in LPDS(Low power deep sleep) the GPIO interrupt
 *  is consumed for wake up, and in order to make the key module work the
 *  the application has to call this API with the index of the GPIO pin which
 *  actually was the reason for the wake up.
 *
 *  @param[in]  index   Index of the GPIO for which the key press has to be
 *                      detected. This is an index in #GPIO_PinConfig array.
 */
void Key_gpioCallbackFxn(uint_least8_t index);

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_Key__include */
