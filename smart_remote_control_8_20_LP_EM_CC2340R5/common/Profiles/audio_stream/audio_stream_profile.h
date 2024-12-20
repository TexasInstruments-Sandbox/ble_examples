/******************************************************************************

 @file  data_stream_profile.h

 @brief This file contains the Audio Stream profile definitions and prototypes.

 Group: WCS, BTS
 $Target Device: DEVICES $

 ******************************************************************************
 $License: BSD3 2010 $
 ******************************************************************************
 $Release Name: PACKAGE NAME $
 $Release Date: PACKAGE RELEASE DATE $
 *****************************************************************************/

#ifndef AUDIOSTREAMPROFILE_H
#define AUDIOSTREAMPROFILE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>

/*********************************************************************
 * Profile Callback
 */
// Callback to indicate client characteristic configuration has been updated
typedef void (*ASP_onCccUpdate_t)( uint16 connHandle, uint16 pValue);

// Callback when receiving data
typedef void (*ASP_incomingData_t)( uint16 connHandle, char *pValue, uint16 len );

typedef struct
{
  ASP_onCccUpdate_t   pfnOnCccUpdateCb;  // Called when client characteristic configuration has been updated
  ASP_incomingData_t  pfnIncomingDataCB;     // Called when receiving data
} ASP_cb_t;

/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      ASP_start
 *
 * @brief   This function adds the Data Stream Server service,
 *          registers the application callback function and initializes
 *          buffer for incoming data.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t ASP_start( ASP_cb_t *appCallbacks);

/*
 * @fn      ASP_sendData
 *
 * @brief   Send data
 *
 * @param   pValue - pointer to data to write
 * @param   len - length of data to write
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t ASP_sendData( uint8 *pValue, uint16 len );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* AUDIOSTREAMPROFILE_H */
