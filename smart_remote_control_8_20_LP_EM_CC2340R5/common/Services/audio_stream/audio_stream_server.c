/******************************************************************************

 @file  audio_stream_server.c

 @brief This file contains the Data Stream service sample GATT service
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

#include "audio_stream_server.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include "ble_stack_api.h"
#include <ti/drivers/GPIO.h>

/*********************************************************************
 * CONSTANTS
 */
// The size of the notification header is opcode + handle
#define AS_NOTI_HDR_SIZE   (ATT_OPCODE_SIZE + 2)

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/// @brief TI Base 128-bit UUID: F000XXXX-0451-4000-B000-000000000000
#define ANDROID_BASE_UUID_128( uuid )  0x64, 0xB6, 0x17, 0xF6, 0x01, 0xAF, 0x7D, 0xBC, \
                                  0x05, 0x4F, 0x21, 0x5A, LO_UINT16( uuid ), HI_UINT16( uuid ), 0x5E, 0xAB

static uint16_t heapHeadroom = 0;

// Data queue to hold the outgoing data
static List_List streamOutQueue;

// Audio GATT Profile Service UUID: 0x0001
static const uint8 audioProfileServUUID[ATT_UUID_SIZE] =
{
 ANDROID_BASE_UUID_128(AUDIO_GOOGLE_SERVICE_UUID)
};

// Write Characteristic UUID: 0x0002
static const uint8 audioProfileWriteUUID[ATT_UUID_SIZE] =
{
 ANDROID_BASE_UUID_128(AUDIO_GOOGL_TX_CHAR_UUID)
};

// Read Characteristic UUID: 0x0003
static const uint8 audioProfileReadUUID[ATT_UUID_SIZE] =
{
 ANDROID_BASE_UUID_128(AUDIO_GOOGL_RX_CHAR_UUID)
};

// Control Characteristic UUID: 0x0004
static const uint8 audioProfileControlUUID[ATT_UUID_SIZE] =
{
 ANDROID_BASE_UUID_128(AUDIO_GOOGL_CTL_CHAR_UUID)
};

//----------------------------------------------------------------------//

typedef struct {
    uint16_t version;
    uint16_t legacy_0x0003;
    uint8_t  supported_assist;
} rc_req_capabilities;

typedef struct {
    rc_req_capabilities get_caps;
    uint8_t mic_open;
    uint8_t mic_close;
    uint8_t mic_extend;
} atvv_char_tx;

typedef struct {
    uint8_t  reason;
    uint8_t  codec_used;
    uint8_t  stream_id;
} audio_start_struct;

typedef struct {
    uint8_t  codec_used;
    uint16_t frame_no;
    uint16_t pred_value;
    uint8_t  step_index;
} audio_sync_struct;

typedef struct {
    uint8_t command;
    uint16_t version;
    uint8_t codec_supported;
    uint8_t assistant_interaction;
    uint16_t audio_fram_size;
    uint8_t extra_config;
    uint8_t reserved;
} rc_rep_capabilities;

typedef struct {
    uint8_t audio_stop;
    audio_start_struct audio_start;
    uint8_t start_search;
    audio_sync_struct audio_sync;
    rc_rep_capabilities caps_resp;
    uint16_t mic_open_error;
} atvv_char_ctl;

#define ATVV_CHAR_TX_LEN 8
#define ATVV_CHAR_AUDIO_LEN 255
#define ATVV_CHAR_CTL_LEN 21

//AUDIO_GOOGLE_SERVICE_UUID
// Audio Profile Service
static const gattAttrType_t audioProfileService = {ATT_UUID_SIZE, audioProfileServUUID};

//AUDIO_GOOGL_TX_CHAR_UUID
//atv_char_tx_data
// WRITE Characteristic Properties
static uint8 audioProfileWriteProps = GATT_PROP_WRITE_NO_RSP;
// WRITE Characteristic Value
static uint8_t audioProfileWrite[ATVV_CHAR_TX_LEN];
// WRITE Characteristic Configuration Descriptor Value
static gattCharCfg_t *audioProfileWriteConfig;

// READ Characteristic Properties
static uint8 audioProfileReadProps = GATT_PROP_NOTIFY;
// READ Characteristic Value
static uint8_t audioProfileRead[ATVV_CHAR_AUDIO_LEN];
// READ Characteristic Configuration Descriptor Value
static gattCharCfg_t *audioProfileReadConfig;

// CONTROL Characteristic Properties
static uint8 audioProfileControlProps = GATT_PROP_NOTIFY;
// CONTROL Characteristic Value
static uint8_t audioProfileControl[ATVV_CHAR_CTL_LEN];
// CONTROL Characteristic Configuration Descriptor Value
static gattCharCfg_t *audioProfileControlConfig;

static AS_cb_t *as_profileCBs = NULL;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t audioProfileAttrTbl[] =
{
 /*--------------------type-------------------*/ /*------------permissions-------------*/ /*------------------pValue--------------------*/
   // GOOGLE Audio Stream Service
   GATT_BT_ATT( primaryServiceUUID,                 GATT_PERMIT_READ,                        (uint8 *)&audioProfileService ),

   // WRITE Characteristic Declaration
   GATT_BT_ATT( characterUUID,                      GATT_PERMIT_READ,                        &audioProfileWriteProps ),
   // WRITE Characteristic Value
   GATT_ATT( audioProfileWriteUUID,                 GATT_PERMIT_WRITE,                       audioProfileWrite ),
   // WRITE Characteristic configuration
   GATT_BT_ATT( clientCharCfgUUID,                  GATT_PERMIT_READ | GATT_PERMIT_WRITE,    (uint8 *)&audioProfileWriteConfig ),

   // READ Characteristic Declaration
   GATT_BT_ATT( characterUUID,                      GATT_PERMIT_READ,                        &audioProfileReadProps ),
   // READ Characteristic Value
   GATT_ATT( audioProfileReadUUID,                  GATT_PERMIT_READ,                        audioProfileRead ),
   // READ Characteristic configuration
   GATT_BT_ATT( clientCharCfgUUID,                  GATT_PERMIT_READ | GATT_PERMIT_WRITE,    (uint8 *)&audioProfileReadConfig ),

   // Control Characteristic Declaration
   GATT_BT_ATT( characterUUID,                      GATT_PERMIT_READ,                        &audioProfileControlProps ),
   // Control Characteristic Value
   GATT_ATT( audioProfileControlUUID,               GATT_PERMIT_READ,                        audioProfileControl ),
   // Control Characteristic configuration
   GATT_BT_ATT( clientCharCfgUUID,                  GATT_PERMIT_READ | GATT_PERMIT_WRITE,    (uint8 *)&audioProfileControlConfig ),
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t AS_writeAttrCB( uint16 connHandle, gattAttribute_t *pAttr, uint8 *pValue, uint16 len, uint16 offset, uint8 method );
static bStatus_t AS_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr, uint8_t *pValue, uint16_t *pLen, uint16_t offset, uint16_t maxLen, uint8_t method );
static bStatus_t AS_transmitNode( AudioStreamNode_t *node );
static bStatus_t AS_queueData( AudioStreamNode_t *node );
static void AS_clearQueue();

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Data Stream Server Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
const gattServiceCBs_t as_servCBs =
{
  AS_ReadAttrCB,                 // Read callback function pointer
  AS_writeAttrCB,                // Write callback function pointer
  NULL                           // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      AS_addService
 *
 * @brief   This function initializes the Data Stream Server service
 *          by registering GATT attributes with the GATT server.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t AS_addService( void )
{
  bStatus_t status = SUCCESS;

  // Allocate Client Characteristic Configuration table
  audioProfileWriteConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * MAX_NUM_BLE_CONNS );
  if ( audioProfileWriteConfig == NULL )
  {
    return ( bleMemAllocError );
  }
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, audioProfileWriteConfig );

  // Allocate Audio Stream Client Characteristic Configuration table
  audioProfileReadConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) * MAX_NUM_BLE_CONNS);
  if ( audioProfileReadConfig == NULL )
  {
    return bleMemAllocError;
  }
  // Initialize Audio Stream Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, audioProfileReadConfig );

  // Allocate Audio Stream Client Characteristic Configuration table
  audioProfileControlConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) * MAX_NUM_BLE_CONNS);
  if ( audioProfileControlConfig == NULL )
  {
    return bleMemAllocError;
  }
  // Initialize Audio Stream Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, audioProfileControlConfig );


  // Register GATT attribute list and CBs with GATT Server
  status = GATTServApp_RegisterService( audioProfileAttrTbl,
                                        GATT_NUM_ATTRS( audioProfileAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &as_servCBs );

  List_clearList(&streamOutQueue);

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      AS_registerProfileCBs
 *
 * @brief   Registers the profile callback function. Only call
 *          this function once.
 *
 * @param   profileCallback - pointer to profile callback functions.
 *
 * @return  SUCCESS or INVALIDPARAMETER
 */
bStatus_t AS_registerProfileCBs( AS_cb_t *profileCallback )
{
  if ( profileCallback )
  {
    as_profileCBs = profileCallback;

    return ( SUCCESS );
  }

  return ( INVALIDPARAMETER );
}

/*********************************************************************
 * @fn      AS_setParameter
 *
 * @brief   Set a Data Stream Service parameter.
 *
 * @param   param - Characteristic UUID
 * @param   pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 * @param   len - length of data to write
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t AS_setParameter(uint8 param, void *pValue, uint16 len)
{
  bStatus_t status = SUCCESS;

  // Verify input parameters
  if ( pValue == NULL )
  {
    return ( INVALIDPARAMETER );
  }

/*
  switch ( param )
  {
    case AUDIOPROFILE_AUDIO:
      status = AS_sendNotification( (uint8 *)pValue, len );
      break;

    default:
      status = INVALIDPARAMETER;
      break;
  }
*/

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      AS_writeAttrCB
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
 * @return  SUCCESS or stack call status
 */

static bStatus_t AS_writeAttrCB( uint16 connHandle,
                                        gattAttribute_t *pAttr,
                                        uint8 *pValue, uint16 len,
                                        uint16 offset, uint8 method )
{

  bStatus_t status = SUCCESS;
  //__asm__("BKPT");
  // Verify input parameters
  if ( pAttr == NULL || pValue == NULL )
  {
    return ( INVALIDPARAMETER );
  }

  /******************************************************/
  /****** Client Characteristic Configuration ***********/
  /******************************************************/

  if ( ! memcmp( pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len ) )
  {
     AS_cccUpdate_t *cccUpdate;
    // Allow only notifications
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY );

    // Notify profile
    if ( status == SUCCESS )
    {
      // This allocation will be free by bleapp_util
      cccUpdate = (AS_cccUpdate_t *)ICall_malloc( sizeof( AS_cccUpdate_t ) );
      if ( cccUpdate == NULL )
      {
        // Return error status
        return ( bleMemAllocError );
      }

      // Copy the data and send it to the profile
      cccUpdate->connHandle = connHandle;
      cccUpdate->value = BUILD_UINT16( pValue[0], pValue[1] );

      // Callback function to notify profile of change on the client characteristic configuration
      BLEAppUtil_invokeFunction( as_profileCBs->pfnOnCccUpdateCB, (char *)cccUpdate );
    }
  }

  /******************************************************/
  /*********** Data In Characteristic  ******************/
  /******************************************************/
  else if ( ! memcmp( pAttr->type.uuid, audioProfileWriteUUID, pAttr->type.len ) )
  {

    // Only notify profile if there is any data in the payload
    if ( len > 0  && as_profileCBs && as_profileCBs->pfnIncomingDataCB)
    {
      AS_dataIn_t *dataIn;

      // This allocation will be free by bleapp_util
      dataIn = (AS_dataIn_t *)ICall_malloc( sizeof( AS_dataIn_t ) + len);
      if ( dataIn == NULL )
      {
        // Return error status
        return ( bleMemAllocError );
      }

      // If allocation was successful,
      // Copy the data and send it to the profile
      if ( len > 0 )
      {
        memcpy( dataIn->pValue, pValue, len );
      }
      dataIn->connHandle = connHandle;
      dataIn->len = len;

      // Callback function to notify profile of change on the client characteristic configuration
      status = BLEAppUtil_invokeFunction( (void *)as_profileCBs->pfnIncomingDataCB, (char *)dataIn );
    }
  }

  // If we get here, that means you've forgotten to add an if clause for a
  // characteristic value attribute in the attribute table that has WRITE permissions.
  else
  {
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Return status value
  return ( status );
}

/*********************************************************************
 * @fn      AS_sendNotification_Control
 *
 * @brief   Transmits data over BLE notifications.
 *
 * @param   pValue - pointer to data to be written
 * @param   len - length of data to be written
 *
 * @return  SUCCESS, or stack call status
 */
int totalNotificationsAttempted = 0;
int notificationsFailed = 0;
int notificationsSent = 0;
int lengthOfNotification = 0;
int notificationAllocationsAttempted = 0;
int notificationAllocationsFailed = 0;
int notificationAllocationsSuccess = 0;
uint8_t *payloadNotification;
volatile uint16_t notiHandle = 0;

/*********************************************************************
 * @fn      AS_queueData
 *
 * @brief   Adds a new AudioStreamNode_t node to the data queue
 *
 * @param   node  - data node to add to the queue
 *
 * @return  SUCCESS, FAILURE or INVALIDPARAMETER
 */
static bStatus_t AS_queueData( AudioStreamNode_t *node )
{
    bStatus_t ret = SUCCESS;
    gattCharCfg_t *pItem = NULL;

    if (node != NULL)
    {
        // Find the correct CCCD
        int i;
        for ( i = 0; i < linkDBNumConns; i++ )
        {
            if (audioProfileControlConfig[i].connHandle == node->connHandle)
            {
                pItem = &(audioProfileControlConfig[i]);
                break;
            }
        }

        // Only store the data if the connection is valid an notifications is allowed
        if ( ( pItem != NULL) &&
             ( pItem->connHandle != LINKDB_CONNHANDLE_INVALID ) &&
             ( pItem->value != GATT_CFG_NO_OPERATION ) &&
             ( pItem->value & GATT_CLIENT_CFG_NOTIFY ))
        {
            List_put(&streamOutQueue, (List_Elem *) node);
        }
        else
        {
            ret = FAILURE;
        }
    }
    else
    {
        ret = INVALIDPARAMETER;
    }

    return ret;
}

/*********************************************************************
 * @fn          AS_transmitNode
 *
 * @brief       Transmits as much as possible of a AudioStreamNode_t node
 *              over BLE notifications.
 *
 * @param       node - The node to send
 *
 * @return      SUCCESS, FAILURE, INVALIDPARAMETER, MSG_BUFFER_NOT_AVAIL,
 *              bleNotCOnnected, bleMemAllocError, blePending, bleInvaludMtuSize or
 *              bleTimeout
 */

static bStatus_t AS_transmitNode( AudioStreamNode_t* node)
{
    bStatus_t ret = SUCCESS;
    attHandleValueNoti_t noti;
    linkDBInfo_t connInfo;

    // Find out what the maximum MTU size is
    ret = linkDB_GetInfo(node->connHandle, &connInfo);

    // Queue up as many notification slots as possible
    if ( (ret == SUCCESS) && (node != NULL) ) {

        // Determine allocation size
        uint16_t allocLen = (node->len - node->offset);
        if ( allocLen > (connInfo.MTU - AS_NOTI_HDR_SIZE) )
        {
            allocLen = connInfo.MTU - AS_NOTI_HDR_SIZE;
        }

        noti.len = 0;
        noti.pValue = (uint8 *)GATT_bm_alloc( node->connHandle, ATT_HANDLE_VALUE_NOTI,
                                              allocLen, &noti.len );
        notificationAllocationsAttempted++;

        // If allocation was successful, copy out data out of the buffer and send it
        if (noti.pValue) {
            notificationAllocationsSuccess++;

            // Normal use
            memcpy(noti.pValue, (void *) ((uint8_t *) node->payload + node->offset), noti.len);

            // Known data test
//            memcpy(noti.pValue, (void *) ((uint8_t *) node->payload + node->offset), noti.len);


            lengthOfNotification = noti.len;
            payloadNotification = noti.pValue;

            // The outgoing data attribute offset is 4
            noti.handle = audioProfileAttrTbl[4].handle;

            ret = GATT_Notification( node->connHandle, &noti, FALSE );
            totalNotificationsAttempted++;

            // If unable to send the message, free allocated buffers and return
            if ( ret != SUCCESS )
            {
                GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
                notificationsFailed++;
            }
            else
            {
                // Increment node data offset
                node->offset += noti.len;
                notificationsSent++;
            }
        }
        else
        {
            // Unable to allocate space for a notification, return failure
            ret = bleMemAllocError;
            notificationAllocationsFailed++;
        }
    }

    return ret;
}

/*********************************************************************
 * @fn      AS_clearQueue
 *
 * @brief   Clears and free the allocated outgoing stream queue
 *
 * @param   None
 *
 * @return  None
 */
void AS_clearQueue()
{
    // Pop and free the whole queue
    while(!List_empty(&streamOutQueue))
    {
        AudioStreamNode_t *node = (AudioStreamNode_t *) List_get(&streamOutQueue);
        ICall_free(node);
    }
}

/*********************************************************************
 * @fn      AS_processStream
 *
 * @brief   Sends out as much as possible from the outgoing stream
 *          queue using BLE notifications
 *
 * @param   connHandle  - connection message was received on
 * @param   *pValue     - pointer to data buffer
 * @param   len         - size of the data buffer
 *
 * @return  SUCCESS, FAILURE, INVALIDPARAMETER, MSG_BUFFER_NOT_AVAIL,
 *          bleNotCOnnected, bleMemAllocError, blePending, bleInvaludMtuSize or
 *          bleTimeout
 */
bStatus_t AS_processStream()
{
    bStatus_t ret = SUCCESS;

    // Send data starting from the list head
    AudioStreamNode_t *node = (AudioStreamNode_t *) List_get(&streamOutQueue);

    while ((ret == SUCCESS) && (node != NULL))
    {
        ret = AS_transmitNode(node);

        // Check that we really did send all data before freeing the node
        if ((node->len - node->offset) == 0)
        {
            ICall_free(node);
            // Move to next queue entry
            node = (AudioStreamNode_t *) List_get(&streamOutQueue);
        }

        if (ret != SUCCESS)
        {
            // We could not send all the data contained in the node, add it back to the queue
            List_putHead(&streamOutQueue, (List_Elem *) node);
        }
    }

    return ret;
}

/*********************************************************************
 * @fn      AS_disconnectStream
 *
 * @brief   Disconnect the steam.
 *          Clear and free up the existing outgoing stream queue.
 *
 * @param   none
 *
 * @return  none
 */
void AS_disconnectStream()
{
    // Clear the outgoing stream queue
    AS_clearQueue();
}

/*********************************************************************
 * @fn      AS_setHeadroomLimit
 *
 * @brief   Sets the limit on how much heap that needs to be available
 *          following a memory allocation.
 *
 * @param   minHeapHeadRoom - Smallest amount of free heap following
 *          an memory allocation.
 *
 * @return  none
 */
void AS_setHeadroomLimit(uint16_t minHeapHeadroom)
{
    // Store the minimal heap headroom limit
    heapHeadroom = minHeapHeadroom;
}

/*********************************************************************
 * @fn      AS_allocateWithHeadroom
 *
 * @brief   Checks if there will be enough free heap left following
 *          a memory allocation. If there is enough heap, it will allocate
 *          the memory.
 *
 * @param   allocSize - number of bytes to be allocated
 *
 * @return  none
 */
int32_t totalFreeSizeTest = 0;
uint16_t allocSizeTest= 0;
void* AS_allocateWithHeadroom(uint16_t allocSize)
{
    void *allocatedBuffer = NULL;
    ICall_heapStats_t  stats;
    ICall_CSState key;
    allocSizeTest = allocSize;

    // Perform this inside a critical section
    key = ICall_enterCriticalSection();

    // Get the current free heap
    ICall_getHeapStats(&stats);
    totalFreeSizeTest = stats.totalFreeSize;
//    if (((uint16_t) allocSize) < ((int32_t)(stats.totalFreeSize - heapHeadroom)))
//    {
        allocatedBuffer = ICall_malloc(allocSize);
//    }

    // Leave the critical section
    ICall_leaveCriticalSection(key);

    return allocatedBuffer;
}

/*********************************************************************
 * @fn      AS_sendData
 *
 * @brief   Put the data into the outgoing stream queue and sends as
 *          much as possible using BLE notifications.
 *
 * @param   connHandle  - connection message was received on
 * @param   *pValue     - pointer to data buffer
 * @param   len         - size of the data buffer
 *
 * @return  SUCCESS, FAILURE, INVALIDPARAMETER, MSG_BUFFER_NOT_AVAIL,
 *          bleNotCOnnected, bleMemAllocError, blePending, bleInvaludMtuSize or
 *          bleTimeout
 */

uint8_t nodeLength = 0;
uint8_t* nodePayloadQueued;
int queueSuccess = 0;
int queueFailure = 0;

bStatus_t AS_sendData(uint16_t connHandle, void *data, uint16_t len , uint8_t char_type)
{

    bStatus_t ret = SUCCESS;
    attHandleValueNoti_t noti;
    linkDBInfo_t connInfo;

        uint16_t allocLen = len;

        noti.len = 0;
        noti.pValue = (uint8 *)GATT_bm_alloc( connHandle, ATT_HANDLE_VALUE_NOTI,
                                              allocLen, &noti.len );
        notificationAllocationsAttempted++;

        // If allocation was successful, copy out data out of the buffer and send it
        if (noti.pValue)
        {
            notificationAllocationsSuccess++;

            memcpy(noti.pValue, (void *) ((uint8_t *) data), noti.len);

            lengthOfNotification = noti.len;
            payloadNotification = noti.pValue;

            if (char_type == 1) {
                noti.handle = (GATTServApp_FindAttr(audioProfileAttrTbl, GATT_NUM_ATTRS(audioProfileAttrTbl), audioProfileRead))->handle;
            }
            else {
                noti.handle = (GATTServApp_FindAttr(audioProfileAttrTbl, GATT_NUM_ATTRS(audioProfileAttrTbl), audioProfileControl))->handle;
            }

            ret = GATT_Notification( connHandle, &noti, FALSE );
            totalNotificationsAttempted++;

            // If unable to send the message, free allocated buffers and return
            if ( ret != SUCCESS )
            {
                GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
                notificationsFailed++;
            }
            else
            {
                notificationsSent++;
            }
        }
        else
        {
            // Unable to allocate space for a notification, return failure
            ret = bleMemAllocError;
            notificationAllocationsFailed++;
        }

    return ret;

}

static bStatus_t AS_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{
  bStatus_t status = SUCCESS;

  // If we get here, that means you've forgotten to add an if clause for a
  // characteristic value attribute in the attribute table that has READ permissions.
  *pLen = 0;
  status = ATT_ERR_ATTR_NOT_FOUND;

  return status;
}

/*********************************************************************
*********************************************************************/
