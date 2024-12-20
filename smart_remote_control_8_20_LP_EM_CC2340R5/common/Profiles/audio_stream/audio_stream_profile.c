/******************************************************************************

 @file  simple_gatt_profile.c

 @brief This file contains the Audio Stream profile sample GATT profile
        for use with the BLE sample application.

 Group: WCS, BTS
 $Target Device: DEVICES $

 ******************************************************************************
 $License: BSD3 2010 $
 ******************************************************************************
 $Release Name: PACKAGE NAME $
 $Release Date: PACKAGE RELEASE DATE $
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <icall.h>
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "audio_stream_profile.h"
#include "audio_stream_server.h"
#include <ti/drivers/GPIO.h>
#include <ti/drivers/dpl/ClockP.h>

#define RC_IDLE_TIMEOUT         5000 // 5 seconds
#define CLOCK_UNITS_MS          1000  // convert to ms
static ClockP_Struct rc_audio_parmupdate_clkStruct;
ClockP_Handle rc_audio_parmupdate_clkHandle;
ClockP_Params clockpParams;
uint8_t buf_ctl_response[9] = {0x0B, 0x01, 0x00, 0x02, 0x03, 0x00, 0xFF, 0x01, 0x00};

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static ASP_cb_t *asp_appCB = NULL;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void ASP_onCccUpdateCB( char *pValue );
static void ASP_incomingDataCB( char *pValue );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Audio Stream profile callback function for incoming data
static AS_cb_t asp_serverCB =
{
  ASP_onCccUpdateCB,
  ASP_incomingDataCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      ASP_start
 *
 * @brief   This function adds the Audio Stream Server service,
 *          registers the service and profile callback function and allocates
 *          buffer for incoming data.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t ASP_start( ASP_cb_t *appCallbacks )
{
  uint8 status = SUCCESS;

  // Add audio stream service
  status = AS_addService();
  if ( status != SUCCESS )
  {
    // Return status value
    return ( status );
  }

  // Register to service callback function
  status = AS_registerProfileCBs( &asp_serverCB );
  if ( status != SUCCESS )
  {
    // Return status value
    return ( status );
  }

  // Registers the application callback function
  if ( appCallbacks )
  {
    asp_appCB = appCallbacks;
  }
  else
  {
    return ( INVALIDPARAMETER );
  }

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      ASP_SendData
 *
 * @brief   Send data over the GATT notification
 *
 * @param   pValue - pointer to data to write
 * @param   len - length of data to write
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t ASP_sendData( uint8 *pValue, uint16 len )
{
  uint8 status = SUCCESS;

  status = AS_setParameter( AUDIOPROFILE_AUDIO, pValue, len );

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      ASP_onCccUpdateCB
 *
 * @brief   Callback from Audio_Stream_Server indicating CCC has been update
 *
 * @param   connHandle - connection message was received on
 * @param   pValue - pointer to data
 *
 * @return  none
 */
static void ASP_onCccUpdateCB( char *pValue  )
{
  AS_cccUpdate_t *cccUpdate = ( AS_cccUpdate_t *)pValue;

  if ( asp_appCB && asp_appCB->pfnOnCccUpdateCb )
  {
    asp_appCB->pfnOnCccUpdateCb( cccUpdate->connHandle, cccUpdate->value );
  }
}

/*********************************************************************
 * @fn      ASP_IncomingDataCB
 *
 * @brief   Callback from Audio_Stream_Server indicating incoming data
 *
 * @param   pValue - pointer to the incoming data
 *
 * @return  none
 */

// Kick off timeout for requesting parameter update
static void rc_audio_parmupdate_clkHandleCB(void)
{
    gapUpdateLinkParamReq_t pParamUpdateReq =
    {
     .connectionHandle = 0,
     .intervalMin = 6,
     .intervalMax = 6,
     .connLatency = 200,
     .connTimeout = 600
    };

    GAP_UpdateLinkParamReq(&pParamUpdateReq);
}

void config_audio_parmupdate(void)
{
    ClockP_Params_init(&clockpParams);
    uint32_t clockTicks = RC_IDLE_TIMEOUT * (CLOCK_UNITS_MS);
    clockpParams.period = 0;
    clockpParams.startFlag = true;
    clockpParams.arg = (uintptr_t)rc_audio_parmupdate_clkHandleCB;
    rc_audio_parmupdate_clkHandle = ClockP_construct(&rc_audio_parmupdate_clkStruct, (void *)BLEAppUtil_invokeFunctionNoData, clockTicks, &clockpParams); // Initialize clock instance.
}

static void ASP_incomingDataCB( char *pValue )
{
  AS_dataIn_t *dataIn = ( AS_dataIn_t * )pValue;

  if ( asp_appCB && asp_appCB->pfnIncomingDataCB )
  {
    asp_appCB->pfnIncomingDataCB( dataIn->connHandle, dataIn->pValue, dataIn->len );
  }

  if (dataIn->pValue[0] == 0x0A){

      config_audio_parmupdate();
      Audio_sendBufferOverBLE(buf_ctl_response, sizeof(buf_ctl_response), 2);
  }

}

/*********************************************************************
*********************************************************************/
