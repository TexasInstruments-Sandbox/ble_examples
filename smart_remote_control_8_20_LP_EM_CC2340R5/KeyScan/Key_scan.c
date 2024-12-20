/******************************************************************************

@file  Key_scan.c

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

#include <stdint.h>
#include <stdbool.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/dpl/ClockP.h>
#include "Key_scan.h"

/*
 *  ======== Key ========
 */
typedef struct Key_state
{
    uint8_t gkeyState;
    uint8_t gkeyPressedState;
    uint8_t gkeyreleaseState;
} Key_state;

uint8_t gkeycallbackCnt=0;

GPIO_PinConfig clearBit(GPIO_PinConfig x, GPIO_PinConfig n);

Key_Object KeyObjects[CONFIG_KEY_COUNT];

static const Key_HWAttrs KeyHWAttrs[CONFIG_KEY_COUNT] = {
    /* CONFIG_KEY_0 */
    {
        .gpioIndex = CONFIG_GPIO_KEY_0_INPUT,
        .pullMode = Key_PULL_UP,
        .internalPullEnabled = 1,
    },

    /* CONFIG_KEY_1 */
    {
        .gpioIndex = CONFIG_GPIO_KEY_1_INPUT,
        .pullMode = Key_PULL_UP,
        .internalPullEnabled = 1,
    },
    /* CONFIG_KEY_2 */
    {
        .gpioIndex = CONFIG_GPIO_KEY_2_INPUT,
        .pullMode = Key_PULL_UP,
        .internalPullEnabled = 1,
    },
    /* CONFIG_KEY_5 */
    {
        .gpioIndex = CONFIG_GPIO_KEY_5_INPUT,
        .pullMode = Key_PULL_UP,
        .internalPullEnabled = 1,
    },
};

const Key_Config Key_config[CONFIG_KEY_COUNT] = {
    /* CONFIG_KEY_0 */
    {
        .object = &KeyObjects[CONFIG_KEY_0],
        .hwAttrs = &KeyHWAttrs[CONFIG_KEY_0]
    },
    /* CONFIG_KEY_1 */
    {
        .object = &KeyObjects[CONFIG_KEY_1],
        .hwAttrs = &KeyHWAttrs[CONFIG_KEY_1]
    },

    /* CONFIG_KEY_2 */
    {
        .object = &KeyObjects[CONFIG_KEY_2],
        .hwAttrs = &KeyHWAttrs[CONFIG_KEY_2]
    },

    /* CONFIG_KEY_5 */
    {
        .object = &KeyObjects[CONFIG_KEY_5],
        .hwAttrs = &KeyHWAttrs[CONFIG_KEY_5]
    },
};


/*

const uint_least8_t CONFIG_KEY_0_CONST;
const uint_least8_t CONFIG_KEY_1_CONST;
*/


const uint_least8_t CONFIG_KEY_0_CONST = CONFIG_KEY_0;
const uint_least8_t CONFIG_KEY_1_CONST = CONFIG_KEY_1;
const uint_least8_t Key_count = CONFIG_KEY_COUNT;

/* Owned by CONFIG_KEY_0 as  */
const uint_least8_t CONFIG_GPIO_KEY_0_INPUT_CONST = CONFIG_GPIO_KEY_0_INPUT;
/* Owned by CONFIG_KEY_1 as  */
const uint_least8_t CONFIG_GPIO_KEY_1_INPUT_CONST = CONFIG_GPIO_KEY_1_INPUT;


/* Default Key_Params parameters structure */
const Key_Params Key_defaultParams = {
    5,    /* 5 ms is the debounce timer value */
    2000, /* 2 seconds long press duration */
    200,  /* 200 ms double key press detection timeout */
    20,   /* 20 ms key repeat interval*/
    0xFF  /* key subscribed for all callbacks */
};

/* Local functions */

/*
 *  ======== ticksToMs ========
 * Convert system ticks to milliseconds(ms). If the value cannot be represented
 * with 32 bits, ~0 is returned.
 */
static uint32_t ticksToMs(uint32_t ticks)
{
    uint64_t ms = (ticks * ClockP_getSystemTickPeriod()) / 1000;

    if ((uint64_t)ms > (uint32_t)ms)
    {
        return ((uint32_t)~0);
    }
    else
    {
        return ((uint32_t)ms);
    }
}

/*
 *  ======== timediff ========
 *  Calculates time difference between input system tick and current system
 *  tick value. If more than 32 bits of ticks have passed, this function is
 *  unreliable.
 */
static uint32_t timediff(uint32_t startTick)
{
    uint32_t currentTick = ClockP_getSystemTicks();

    if (currentTick > startTick)
    {
        return (currentTick - startTick);
    }
    else
    {
        /* System tick value overflowed */
        return (currentTick + ((uint32_t)(~0) - startTick));
    }
}

/*
 *  ======== msToTicks ========
 * Convert milliseconds to system ticks
 */
static uint32_t msToTicks(uint32_t ms)
{
    if (ms == 0)
    {
        return 0;
    }

    uint32_t ticks = (ms * 1000) / ClockP_getSystemTickPeriod();

    /* If ticks is 0, return 1 (the smallest representable value) */
    return ticks ? ticks : 1;
}

/*
 *  ======== Key_close ========
 *  Closes an instance of a Key
 */
void Key_close(Key_Handle handle)
{
    Key_Object *obj = (Key_Object *)(handle->object);
    Key_HWAttrs *hw = (Key_HWAttrs *)handle->hwAttrs;

    GPIO_resetConfig(hw->gpioIndex);

    ClockP_stop(obj->clockHandle);
    ClockP_delete(obj->clockHandle);
    obj->clockHandle = NULL;
}

/*
 *  ======== Key_gpioCallbackFxn ========
 * Callback function for the key interrupts
 */
void Key_gpioCallbackFxn(uint_least8_t index)
{
    /* Handle GPIO interrupt */
    uint_least8_t i;
    static uint8_t j=0;
    gkeycallbackCnt++;
    for (i = 0; i < Key_count; i++)
    {
        Key_Object *obj = (Key_Object *)Key_config[i].object;
        Key_HWAttrs *hw = (Key_HWAttrs *)Key_config[i].hwAttrs;
        if (hw->gpioIndex == index)
        {
            ClockP_setTimeout(obj->clockHandle, obj->debounceDuration);
            ClockP_start(obj->clockHandle);
          //  gkeyState[j++] = obj->keyStateVariables.state;
#if KEY_STATE_DEBUG
            gKey_state[gKeyIndex++].gkeyState = obj->keyStateVariables.state;
#endif
            switch (obj->keyStateVariables.state)
            {
                case Key_PRESSING:
                case Key_PRESSING_REPEAT:
                    obj->keyStateVariables.state = Key_RELEASING;
                    break;
                case Key_PRESSED:
                case Key_LONGPRESSING:
                    obj->keyStateVariables.state = Key_RELEASING;
                    break;
                case Key_LONGPRESSED:
                    obj->keyStateVariables.state = Key_RELEASING_LONG;
                    break;
                case Key_RELEASED:
                    obj->keyStateVariables.state = Key_PRESSING;
                    break;
                case Key_DBLPRESS_DETECTION:
                    obj->keyStateVariables.state = Key_DBLPRESSING;
                    break;
                case Key_DBLPRESSED:
                    obj->keyStateVariables.state = Key_RELEASING_DBLPRESSED;
                    break;
                /*
                 * Any other case, mark the key as pressed.
                 * Typically we should not come here.
                 */
                default:
                    obj->keyStateVariables.state = Key_PRESSING;
                    break;
            }
           // gkeyState[j++] = obj->keyStateVariables.state;
#if KEY_STATE_DEBUG
            gKey_state[gKeyIndex++].gkeyState = obj->keyStateVariables.state;
#endif
            /* Disable the interrupt until the debounce timer expires */
            GPIO_disableInt(index);
            break;
        }
    }
}

/*
 *  ======== Key_init ========
 */
void Key_init(void)
{
    /* Initialize GPIO driver if it wasn't already */
    GPIO_init();
}

/*
 *  ======== Key_clockTimeoutHandler ========
 * Timeout handler for the clock timeouts
 */
static void Key_clockTimeoutHandler(uintptr_t arg)
{
    Key_Object *obj;
    Key_HWAttrs *hw;
    Key_Handle keyHandle;
    Key_EventMask keyEvents = 0;
#if 1
    GPIO_PinConfig pinConfig =0;
#endif
    keyHandle = (Key_Handle)arg;
    obj          = (Key_Object *)keyHandle->object;
    hw           = (Key_HWAttrs *)keyHandle->hwAttrs;

#if 1
    pinConfig |= hw->pullMode == Key_PULL_DOWN ? GPIO_CFG_IN_INT_RISING : GPIO_CFG_IN_INT_FALLING;

    /* Configure input mode with or without pull */
    if (hw->internalPullEnabled)
    {
        pinConfig |= hw->pullMode == Key_PULL_DOWN ?  GPIO_CFG_IN_PD : GPIO_CFG_IN_PU;
    }
    else
    {
        pinConfig |= GPIO_CFG_IN_NOPULL;
    }
    GPIO_setConfig(hw->gpioIndex, pinConfig);

#endif
    if (GPIO_read(hw->gpioIndex) == hw->pullMode)
    {
        /*
         * Getting a debounce duration timeout callback. The key is
         * currently in unpressed (pull) state.
         */
        switch (obj->keyStateVariables.state)
        {
            case Key_RELEASING:
                if (obj->keyEventMask & Key_EV_DOUBLECLICKED)
                {
                    /* Set clock to detect a double press */
                    obj->keyStateVariables.state = Key_DBLPRESS_DETECTION;
                    ClockP_setTimeout(obj->clockHandle, obj->doublePressDetectiontimeout - obj->debounceDuration);
                    ClockP_start(obj->clockHandle);
                }
                else
                {
                    obj->keyStateVariables.lastPressedDuration = timediff(
                        obj->keyStateVariables.pressedStartTime);
                    obj->keyStateVariables.state = Key_RELEASED;
                    if (obj->keyEventMask & Key_EV_RELEASED)
                    {
                        keyEvents |= Key_EV_RELEASED;
                    }
                    if (obj->keyEventMask & Key_EV_CLICKED)
                    {
                        keyEvents |= Key_EV_CLICKED;
                    }

                }
                break;

            case Key_DBLPRESS_DETECTION:
                obj->keyStateVariables.lastPressedDuration = timediff(obj->keyStateVariables.pressedStartTime);
                if (obj->keyEventMask & Key_EV_RELEASED)
                {
                    keyEvents |= Key_EV_RELEASED;
                }
                if (obj->keyEventMask & Key_EV_CLICKED)
                {
                    keyEvents |= Key_EV_CLICKED;
                }
                obj->keyStateVariables.state = Key_RELEASED;
                break;

            case Key_RELEASING_LONG:
                obj->keyStateVariables.lastPressedDuration = timediff(obj->keyStateVariables.pressedStartTime);
                if (obj->keyEventMask & Key_EV_LONGCLICKED)
                {
                    keyEvents |= Key_EV_LONGCLICKED;
                }
                if (obj->keyEventMask & Key_EV_RELEASED)
                {
                    keyEvents |= Key_EV_RELEASED;
                }
                obj->keyStateVariables.state = Key_RELEASED;
                break;

            case Key_RELEASING_DBLPRESSED:
                obj->keyStateVariables.state = Key_RELEASED;
                if (obj->keyEventMask & Key_EV_RELEASED)
                {
                    keyEvents |= Key_EV_RELEASED;
                }
                break;


            case Key_PRESSING:
            case Key_PRESSING_REPEAT:
            case Key_DBLPRESSING:
            case Key_LONGPRESSING:
                /*
                 * Key was pressed and released within debounce time
                 * Does not count.
                 */
                obj->keyStateVariables.state = Key_RELEASED;
#if 1
                keyEvents |= Key_EV_RELEASED;
#endif
                break;

            /*
             * Any other case, mark the key as pressed.
             * Typically we should not come here.
             */
            default:
                obj->keyStateVariables.state = Key_RELEASED;
                break;
        }
#if 0
        if (hw->pullMode == Key_PULL_DOWN)
        {
            GPIO_setInterruptConfig(hw->gpioIndex, GPIO_CFG_IN_INT_RISING | GPIO_CFG_INT_ENABLE);
        }
        else if (hw->pullMode == Key_PULL_UP)
        {
            GPIO_setInterruptConfig(hw->gpioIndex, GPIO_CFG_IN_INT_FALLING | GPIO_CFG_INT_ENABLE);
        }
#else

        pinConfig |= hw->pullMode == Key_PULL_DOWN ? GPIO_CFG_IN_INT_RISING : GPIO_CFG_IN_INT_FALLING;

        /* Configure input mode with or without pull */
        if (hw->internalPullEnabled)
        {
            pinConfig |= hw->pullMode == Key_PULL_DOWN ? GPIO_CFG_IN_PD : GPIO_CFG_IN_PU;
        }
        else
        {
            pinConfig |= GPIO_CFG_IN_NOPULL;
        }
#if KEY_STATE_DEBUG
        gKey_state[gKeyIndex++].gkeyreleaseState = obj->keyStateVariables.state;
#endif
        GPIO_setConfig(hw->gpioIndex, pinConfig);
        GPIO_enableInt(hw->gpioIndex);
#endif
    }
    /*
     * Getting a debounce duration timeout callback. The key is currently
     * pressed.
     */
    else
    {
        switch (obj->keyStateVariables.state)
        {
            case Key_PRESSING:
                /* This is a debounced press */
                obj->keyStateVariables.pressedStartTime = ClockP_getSystemTicks();
                if (obj->keyEventMask & Key_EV_PRESSED)
                {
                    keyEvents |= Key_EV_PRESSED;
                }
                else if(obj->keyEventMask & Key_EV_PRESSED_REPEAT)
                {
                    obj->keyStateVariables.state = Key_PRESSING_REPEAT;

                    ClockP_setTimeout(obj->clockHandle, obj->repeatInterval - obj->debounceDuration);
                    ClockP_start(obj->clockHandle);
                    break;
                }

                /* Start countdown if interest in long-press */
                if (obj->keyEventMask & (Key_EV_LONGPRESSED | Key_EV_LONGCLICKED))
                {
                    obj->keyStateVariables.state = Key_LONGPRESSING;
                    ClockP_setTimeout(obj->clockHandle, obj->longPressDuration - obj->debounceDuration);
                    ClockP_start(obj->clockHandle);
                }
                else
                {
                    obj->keyStateVariables.state = Key_PRESSED;
                }


                break;

            case Key_DBLPRESSING:
                /* This is a debounced press (this is considered as double click) */
                if (obj->keyEventMask & Key_EV_DOUBLECLICKED)
                {
                    keyEvents |= Key_EV_DOUBLECLICKED;
                }
                obj->keyStateVariables.state = Key_DBLPRESSED;
                break;

            case Key_LONGPRESSING:
                obj->keyStateVariables.state = Key_LONGPRESSED;
                if (obj->keyEventMask & Key_EV_LONGPRESSED)
                {
                    keyEvents |= Key_EV_LONGPRESSED;
                }
                break;

            case Key_PRESSING_REPEAT:
               // obj->keyStateVariables.state = Key_LONGPRESSED_REPEAT;
                if (obj->keyEventMask & Key_EV_PRESSED_REPEAT)
                {
                    keyEvents |= Key_PRESSING_REPEAT;
                    ClockP_setTimeout(obj->clockHandle, obj->repeatInterval);
                    ClockP_start(obj->clockHandle);
                }
                break;

            case Key_RELEASING:
            case Key_RELEASING_LONG:
            case Key_RELEASING_DBLPRESSED:
                /*
                 * We're releasing, but isn't released after debounce.
                 * Start count down again if interest in long-press
                 */
                if (obj->keyEventMask & (Key_EV_LONGPRESSED | Key_EV_LONGCLICKED))
                {
                    obj->keyStateVariables.state = Key_LONGPRESSING;
                    ClockP_setTimeout(obj->clockHandle, obj->longPressDuration - obj->debounceDuration);
                    ClockP_start(obj->clockHandle);
                    obj->keyStateVariables.state = Key_LONGPRESSING;
                }
                else if (obj->keyEventMask & Key_EV_PRESSED_REPEAT)
                {
                    obj->keyStateVariables.state = Key_PRESSING_REPEAT;
                }
                else
                {
                    obj->keyStateVariables.state = Key_PRESSED;
                }

            /*
             * Any other case, mark the key as pressed.
             * Typically we should not come here
             */
            default:
                obj->keyStateVariables.state = Key_PRESSED;
                break;
        }

#if 0
        if (hw->pullMode == Key_PULL_DOWN)
        {
            GPIO_setInterruptConfig(hw->gpioIndex, GPIO_CFG_IN_INT_FALLING | GPIO_CFG_INT_ENABLE);
        }
        else if (hw->pullMode == Key_PULL_UP)
        {
            GPIO_setInterruptConfig(hw->gpioIndex, GPIO_CFG_IN_INT_RISING | GPIO_CFG_INT_ENABLE);
        }
#else
        pinConfig |= hw->pullMode == Key_PULL_DOWN ? GPIO_CFG_IN_INT_RISING : GPIO_CFG_IN_INT_FALLING;

        /* Configure input mode with or without pull */
        if (hw->internalPullEnabled)
        {
            pinConfig |= hw->pullMode == Key_PULL_DOWN ? GPIO_CFG_IN_PU : GPIO_CFG_IN_PD;  //GPIO_CFG_IN_PD : GPIO_CFG_IN_PU;
        }
        else
        {
            pinConfig |= GPIO_CFG_IN_NOPULL;
        }
#if KEY_STATE_DEBUG
        gKey_state[gKeyIndex++].gkeyPressedState = obj->keyStateVariables.state;
#endif
        GPIO_setConfig(hw->gpioIndex, pinConfig);
        GPIO_enableInt(hw->gpioIndex);
#endif
    }
    if ((keyEvents != 0) && (obj->keyCallback != NULL))
    {
        obj->keyCallback(keyHandle, keyEvents);
    }
}

/*
 *  ======== Key_open ========
 *  Open a Key instance
 */
Key_Handle Key_open(uint_least8_t keyIndex, Key_Params *params)
{
    Key_Params localParams;
    Key_Handle handle;
    Key_Object *obj;
    Key_HWAttrs *hw;
    GPIO_PinConfig pinConfig;
    ClockP_Params clockParams;
    GPIO_PinConfig pinConfig_1;

    /*
     * This sets the init state of the key
     * keyIndex cannot be greater than total KeyCount
     */
    if (keyIndex >= Key_count)
    {
        return NULL;
    }

    /* If params is null then use the default params */
    if (params == NULL)
    {
        /*
         * Make a local copy of default params to pass, to avoid casting away
         * const on Key_defaultParams
         */
        localParams = Key_defaultParams;
        params      = &localParams;
    }

    /* Get instance state structure */
    handle = (Key_Handle)&Key_config[keyIndex];
    obj    = (Key_Object *)(Key_config[keyIndex].object);
    hw     = (Key_HWAttrs *)(Key_config[keyIndex].hwAttrs);

    /* Set internal variables */
    obj->debounceDuration            = msToTicks(params->debounceDuration);
    obj->longPressDuration           = msToTicks(params->longPressDuration);
    obj->doublePressDetectiontimeout = msToTicks(params->doublePressDetectiontimeout);
    obj->repeatInterval              = msToTicks(params->repeatIntervaltimeout);
    obj->keyCallback              = params->keyCallback;
    obj->keyEventMask             = params->keyEventMask;

    /* If instance already has a clock then it is already open */
    if (obj->clockHandle != NULL)
    {
        return NULL;
    }

    /* Create one shot clock for handling debounce */
    ClockP_Params_init(&clockParams);
    clockParams.period    = 0; /* Indicates a one shot clock */
    clockParams.startFlag = false;
    clockParams.arg       = (uintptr_t)handle;
    obj->clockHandle      = ClockP_create(Key_clockTimeoutHandler, 0, &clockParams);
    if (NULL == obj->clockHandle)
    {
        return NULL;
    }

    /* Enable interrupt by default */
    pinConfig = GPIO_CFG_INT_ENABLE;
    pinConfig |= hw->pullMode == Key_PULL_DOWN ? GPIO_CFG_IN_INT_RISING : GPIO_CFG_IN_INT_FALLING;

    /* Configure input mode with or without pull */
    if (hw->internalPullEnabled)
    {
        pinConfig |= hw->pullMode == Key_PULL_DOWN ? GPIO_CFG_IN_PD : GPIO_CFG_IN_PU;
    }
    else
    {
        pinConfig |= GPIO_CFG_IN_NOPULL;
    }
    /* Set callback first in case we have a pending interrupt */
    GPIO_setCallback(hw->gpioIndex, &Key_gpioCallbackFxn);
    GPIO_setConfig(hw->gpioIndex, pinConfig);

    return handle;
}

/*
 *  ======== Key_Params_init ========
 * Initialize a Key_Params struct to default settings.
 */
void Key_Params_init(Key_Params *params)
{
    *params = Key_defaultParams;
}

/*
 *  ======== Key_setCallback ========
 * Set the callback for the keys.
 */
void Key_setCallback(Key_Handle handle, Key_Callback keyCallback)
{
    Key_Object *obj = (Key_Object *)handle->object;

    obj->keyCallback = keyCallback;
}

/*
 *  ======== Key_getLastPressedDuration ========
 * Return the get last pressed duration
 */
extern uint32_t Key_getLastPressedDuration(Key_Handle handle)
{
    Key_Object *obj = (Key_Object *)handle->object;
    uint32_t ticks     = obj->keyStateVariables.lastPressedDuration;
    return ticksToMs(ticks);
}

// Clears the nth bit of x
GPIO_PinConfig clearBit(GPIO_PinConfig x, GPIO_PinConfig n)
{
    return (x & ~(1 << n));
}
