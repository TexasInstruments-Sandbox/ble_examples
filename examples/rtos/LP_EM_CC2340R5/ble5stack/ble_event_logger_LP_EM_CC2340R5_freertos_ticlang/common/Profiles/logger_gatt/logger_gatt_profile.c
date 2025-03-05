/******************************************************************************

 @file  simple_gatt_profile.c

 @brief This file contains the Simple GATT profile sample GATT service profile
        for use with the BLE sample application.

 Group: WCS, BTS
 Target Device: cc23xx

 ******************************************************************************
 
 Copyright (c) 2010-2024, Texas Instruments Incorporated
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

/*********************************************************************
 * INCLUDES
 */
#include <common/Profiles/logger_gatt/logger_gatt_profile.h>
#include <global.h>
#include <string.h>
#include <icall.h>
#include "icall_ble_api.h"

#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/drivers/dpl/ClockP.h>
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */
void SimpleGattProfile_callback( uint8 paramID  );
void SimpleGattProfile_invokeFromFWContext( char *pData );

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Simple GATT Profile Service UUID: 0xFFF0
GATT_BT_UUID(simpleGattProfile_ServUUID, SIMPLEGATTPROFILE_SERV_UUID);

// Characteristic 1 UUID: 0xFFF1
GATT_BT_UUID(simpleGattProfile_char1UUID, SIMPLEGATTPROFILE_CHAR1_UUID);

// Characteristic 2 UUID: 0xFFF2
GATT_BT_UUID(simpleGattProfile_char2UUID, SIMPLEGATTPROFILE_CHAR2_UUID);

// Characteristic 3 UUID: 0xFFF3
GATT_BT_UUID(simpleGattProfile_char3UUID, SIMPLEGATTPROFILE_CHAR3_UUID);

// Characteristic 4 UUID: 0xFFF4
GATT_BT_UUID(simpleGattProfile_char4UUID, SIMPLEGATTPROFILE_CHAR4_UUID);

// Characteristic 5 UUID: 0xFFF5
GATT_BT_UUID(simpleGattProfile_char5UUID, SIMPLEGATTPROFILE_CHAR5_UUID);

/*********************************************************************
 * EXTERNAL VARIABLES
 */


/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static SimpleGattProfile_CBs_t *simpleGattProfile_appCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Simple GATT Profile Service attribute
static const gattAttrType_t simpleGattProfile_Service = { ATT_BT_UUID_SIZE, simpleGattProfile_ServUUID };


// Simple GATT Profile Characteristic 1 Properties
static uint8 simpleGattProfile_Char1Props = GATT_PROP_READ;

// Characteristic 1 Value
static uint8 simpleGattProfile_Char1[GATTPROFILE_TIMESTAMP_LEN] = {0,0,0,0,0,0,0};

// Simple GATT Profile Characteristic 1 User Description
static uint8 simpleGattProfile_Char1UserDesp[17] = "Characteristic 1";


// Simple GATT Profile Characteristic 2 Properties
static uint8 simpleGattProfile_Char2Props = GATT_PROP_READ;

// Characteristic 2 Value
static uint8 simpleGattProfile_Char2[GATTPROFILE_TIMESTAMP_LEN] = {0,0,0,0,0,0,0};

// Simple Profile Characteristic 2 User Description
static uint8 simpleGattProfile_Char2UserDesp[17] = "Characteristic 2";


// Simple GATT Profile Characteristic 3 Properties
static uint8 simpleGattProfile_Char3Props = GATT_PROP_READ;

// Characteristic 3 Value
static uint8 simpleGattProfile_Char3[GATTPROFILE_TIMESTAMP_LEN] = {0,0,0,0,0,0,0};

// Simple GATT Profile Characteristic 3 User Description
static uint8 simpleGattProfile_Char3UserDesp[17] = "Characteristic 3";


// Simple GATT Profile Characteristic 4 Properties
static uint8 simpleGattProfile_Char4Props = GATT_PROP_READ;

// Characteristic 4 Value
static uint8 simpleGattProfile_Char4[GATTPROFILE_TIMESTAMP_LEN] = {0,0,0,0,0,0,0};


// Simple GATT Profile Characteristic 4 User Description
static uint8 simpleGattProfile_Char4UserDesp[17] = "Characteristic 4";


// Simple GATT Profile Characteristic 5 Properties
static uint8 simpleGattProfile_Char5Props = GATT_PROP_READ;

// Characteristic 5 Value
static uint8 simpleGattProfile_Char5[GATTPROFILE_TIMESTAMP_LEN] = {0,0,0,0,0,0,0};

// Simple GATT Profile Characteristic 5 User Description
static uint8 simpleGattProfile_Char5UserDesp[17] = "Characteristic 5";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t simpleGattProfile_attrTbl[] =
{
 /*------------------type-----------------*/ /*-----------permissions-----------*/ /*-----------------pValue----------------*/
   // Simple Profile Service
   GATT_BT_ATT( primaryServiceUUID,           GATT_PERMIT_READ,                      (uint8 *) &simpleGattProfile_Service ),

   // Characteristic 1 Declaration
   GATT_BT_ATT( characterUUID,                GATT_PERMIT_READ,                      &simpleGattProfile_Char1Props ),
   // Characteristic Value 1
   GATT_BT_ATT( simpleGattProfile_char1UUID,  GATT_PERMIT_AUTHEN_READ ,  simpleGattProfile_Char1 ),
   // Characteristic 1 User Description
   GATT_BT_ATT( charUserDescUUID,             GATT_PERMIT_READ,                      simpleGattProfile_Char1UserDesp ),

   // Characteristic 2 Declaration
   GATT_BT_ATT( characterUUID,                GATT_PERMIT_READ,                      &simpleGattProfile_Char2Props ),
   // Characteristic Value 2
   GATT_BT_ATT( simpleGattProfile_char2UUID,  GATT_PERMIT_ENCRYPT_READ ,                      simpleGattProfile_Char2 ),
   // Characteristic 2 User Description
   GATT_BT_ATT( charUserDescUUID,             GATT_PERMIT_READ,                      simpleGattProfile_Char2UserDesp ),

   // Characteristic 3 Declaration
   GATT_BT_ATT( characterUUID,                GATT_PERMIT_READ,                      &simpleGattProfile_Char3Props ),
   // Characteristic Value 3
   GATT_BT_ATT( simpleGattProfile_char3UUID,  GATT_PERMIT_AUTHEN_READ,                     simpleGattProfile_Char3 ),
   // Characteristic 3 User Description
   GATT_BT_ATT( charUserDescUUID,             GATT_PERMIT_READ,                      simpleGattProfile_Char3UserDesp ),

   // Characteristic 4 Declaration
   GATT_BT_ATT( characterUUID,                GATT_PERMIT_READ,                      &simpleGattProfile_Char4Props ),
   // Characteristic Value 4
   GATT_BT_ATT( simpleGattProfile_char4UUID,  GATT_PERMIT_AUTHEN_READ,                       simpleGattProfile_Char4 ),
   // Characteristic 4 configuration
   //GATT_BT_ATT( clientCharCfgUUID,            GATT_PERMIT_READ ,  (     uint8 *) &simpleGattProfile_Char4Config ),
   // Characteristic 4 User Description
   GATT_BT_ATT( charUserDescUUID,             GATT_PERMIT_READ,                      simpleGattProfile_Char4UserDesp ),

   // Characteristic 5 Declaration
   GATT_BT_ATT( characterUUID,                GATT_PERMIT_READ,                      &simpleGattProfile_Char5Props ),
   // Characteristic Value 5
   GATT_BT_ATT( simpleGattProfile_char5UUID,  GATT_PERMIT_AUTHEN_READ,                        simpleGattProfile_Char5 ),
   // Characteristic 5 User Description
   GATT_BT_ATT( charUserDescUUID,             GATT_PERMIT_READ,                      simpleGattProfile_Char5UserDesp ),
};
/*********************************************************************
 * LOCAL FUNCTIONS
 */
bStatus_t SimpleGattProfile_readAttrCB( uint16_t connHandle,
                                        gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t *pLen,
                                        uint16_t offset, uint16_t maxLen,
                                        uint8_t method );
bStatus_t SimpleGattProfile_writeAttrCB( uint16_t connHandle,
                                         gattAttribute_t *pAttr,
                                         uint8_t *pValue, uint16_t len,
                                         uint16_t offset, uint8_t method );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Simple GATT Profile Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
const gattServiceCBs_t simpleGattProfile_CBs =
{
  SimpleGattProfile_readAttrCB,  // Read callback function pointer
  SimpleGattProfile_writeAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleGattProfile_addService
 *
 * @brief   This function initializes the Simple GATT Server service
 *          by registering GATT attributes with the GATT server.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t SimpleGattProfile_addService( void )
{
  uint8 status = SUCCESS;

//  // Allocate Client Characteristic Configuration table
//  simpleGattProfile_Char4Config = (gattCharCfg_t *)ICall_malloc( sizeof( gattCharCfg_t ) *
//                                                                 MAX_NUM_BLE_CONNS );
//  if ( simpleGattProfile_Char4Config == NULL )
//  {
//    return ( bleMemAllocError );
//  }
//
//  // Initialize Client Characteristic Configuration attributes
//  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, simpleGattProfile_Char4Config );


  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( simpleGattProfile_attrTbl,
                                        GATT_NUM_ATTRS( simpleGattProfile_attrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &simpleGattProfile_CBs );

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      SimpleGattProfile_registerAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   appCallbacks - pointer to application callback.
 *
 * @return  SUCCESS or INVALIDPARAMETER
 */
bStatus_t SimpleGattProfile_registerAppCBs( SimpleGattProfile_CBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    simpleGattProfile_appCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      SimpleGattProfile_setParameter
 *
 * @brief   Set a Simple GATT Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *                  the parameter ID and WILL be cast to the appropriate
 *                  data type (example: data type of uint16 will be cast to
 *                  uint16 pointer).
 *
 * @return  SUCCESS or INVALIDPARAMETER
 */
bStatus_t SimpleGattProfile_setParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t status = SUCCESS;

  switch ( param )
  {
    case SIMPLEGATTPROFILE_CHAR1:
        if ( len == GATTPROFILE_TIMESTAMP_LEN )
             {
               VOID memcpy( simpleGattProfile_Char1, value, GATTPROFILE_TIMESTAMP_LEN );
             }
             else
             {
               status = bleInvalidRange;
             }
             break;
    case SIMPLEGATTPROFILE_CHAR2:
        if ( len == GATTPROFILE_TIMESTAMP_LEN )
             {
               VOID memcpy( simpleGattProfile_Char2, value, GATTPROFILE_TIMESTAMP_LEN );
             }
             else
             {
               status = bleInvalidRange;
             }
             break;
    case SIMPLEGATTPROFILE_CHAR3:
          if ( len == GATTPROFILE_TIMESTAMP_LEN )
          {
            VOID memcpy( simpleGattProfile_Char3, value, GATTPROFILE_TIMESTAMP_LEN );
          }
          else
          {
            status = bleInvalidRange;
          }
          break;

    case SIMPLEGATTPROFILE_CHAR4:
        if ( len == GATTPROFILE_TIMESTAMP_LEN )
          {
            VOID memcpy( simpleGattProfile_Char4, value, GATTPROFILE_TIMESTAMP_LEN );
          }
          else
          {
            status = bleInvalidRange;
          }
          break;

    case SIMPLEGATTPROFILE_CHAR5:
        if ( len == GATTPROFILE_TIMESTAMP_LEN )
          {
            VOID memcpy( simpleGattProfile_Char5, value, GATTPROFILE_TIMESTAMP_LEN );
          }
          else
          {
            status = bleInvalidRange;
          }
          break;

    default:
      status = INVALIDPARAMETER;
      break;
  }

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      SimpleGattProfile_getParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SimpleGattProfile_getParameter( uint8 param, void *value )
{
  bStatus_t status = SUCCESS;
  switch ( param )
  {
    case SIMPLEGATTPROFILE_CHAR1:
        VOID memcpy( value, simpleGattProfile_Char1, GATTPROFILE_TIMESTAMP_LEN );
             break;

    case SIMPLEGATTPROFILE_CHAR2:
        VOID memcpy( value, simpleGattProfile_Char2, GATTPROFILE_TIMESTAMP_LEN );
      break;

    case SIMPLEGATTPROFILE_CHAR3:
        VOID memcpy( value, simpleGattProfile_Char3, GATTPROFILE_TIMESTAMP_LEN );
             break;

    case SIMPLEGATTPROFILE_CHAR4:
        VOID memcpy( value, simpleGattProfile_Char4, GATTPROFILE_TIMESTAMP_LEN );
             break;


    case SIMPLEGATTPROFILE_CHAR5:
      VOID memcpy( value, simpleGattProfile_Char5, GATTPROFILE_TIMESTAMP_LEN );
      break;

    default:
      status = INVALIDPARAMETER;
      break;
  }

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn          SimpleGattProfile_readAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
bStatus_t SimpleGattProfile_readAttrCB(uint16_t connHandle,
                                       gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen,
                                       uint16_t offset, uint16_t maxLen,
                                       uint8_t method)
{


  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      // characteristics 1 and 2 have read permissions
      // characteritisc 3 does not have read permissions; therefore it is not
      //   included here
      // characteristic 4 does not have read permissions, but because it
      //   can be sent as a notification, it is included here

      case SIMPLEGATTPROFILE_CHAR1_UUID:
      case SIMPLEGATTPROFILE_CHAR4_UUID:
      case SIMPLEGATTPROFILE_CHAR2_UUID:
      case SIMPLEGATTPROFILE_CHAR5_UUID:
      case SIMPLEGATTPROFILE_CHAR3_UUID:
         *pLen = GATTPROFILE_TIMESTAMP_LEN;
         VOID memcpy( pValue, pAttr->pValue, GATTPROFILE_TIMESTAMP_LEN );
         break;

      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      SimpleGattProfile_writeAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
bStatus_t SimpleGattProfile_writeAttrCB( uint16_t connHandle,
                                     gattAttribute_t *pAttr,
                                     uint8_t *pValue, uint16_t len,
                                     uint16_t offset, uint8_t method )
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {

      default:
        // Should never get here! (characteristics 1,2,3,4 and 5 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a characteristic value changed then callback function to notify application of change
  if ((notifyApp != 0xFF ) && simpleGattProfile_appCBs && simpleGattProfile_appCBs->pfnSimpleGattProfile_Change)
  {
      SimpleGattProfile_callback( notifyApp );
  }

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      SimpleGattProfile_callback
 *
 * @brief   This function will be called from the BLE App Util module
 *          context.
 *          Calling the application callback
 *
 * @param   pData - data
 *
 * @return  None
 */
void SimpleGattProfile_callback( uint8 paramID )
{
  char *pData = ICall_malloc(sizeof(char));

  if(pData == NULL)
  {
    return;
  }

  pData[0] = paramID;

  BLEAppUtil_invokeFunction(SimpleGattProfile_invokeFromFWContext, pData);
}

/*********************************************************************
 * @fn      SimpleGattProfile_invokeFromFWContext
 *
 * @brief   This function will be called from the BLE App Util module
 *          context.
 *          Calling the application callback
 *
 * @param   pData - data
 *
 * @return  None
 */
void SimpleGattProfile_invokeFromFWContext( char *pData )
{
  simpleGattProfile_appCBs->pfnSimpleGattProfile_Change(pData[0]);
}

/*********************************************************************
*********************************************************************/
