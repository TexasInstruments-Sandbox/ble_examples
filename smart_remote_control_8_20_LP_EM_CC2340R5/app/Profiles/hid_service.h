/******************************************************************************

 @file  hidkbdccservice.h

 @brief This file contains the Device Information service definitions and
        prototypes.

 Group: WCS, BTS
 $Target Device: DEVICES $

 ******************************************************************************
 $License: BSD3 2012 $
 ******************************************************************************
 $Release Name: PACKAGE NAME $
 $Release Date: PACKAGE RELEASE DATE $
 *****************************************************************************/

#ifndef HIDKBDCCSERVICE_H
#define HIDKBDCCSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Number of HID reports defined in the service

#define HID_NUM_REPORTS          8
#endif

// HID Report IDs for the service
#define HID_RPT_ID_KEY_IN        1  // Keyboard input report ID
#define HID_RPT_ID_CC_IN         2  // Consumer Control input report ID
#define HID_RPT_ID_LED_OUT       0  // LED output report ID

// HID feature flags
#define HID_KBD_FLAGS            HID_FLAGS_REMOTE_WAKE

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */


/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      HidKbdCC_addService
 *
 * @brief   Initializes the HID service for keyboard by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t HidKbdCC_AddService( void );



/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

