/******************************************************************************

 @file  data_stream_server.h

 @brief This file contains the Audio Audio service definitions and prototypes.

 Group: WCS, BTS
 $Target Device: DEVICES $

 ******************************************************************************
 $License: BSD3 2010 $
 ******************************************************************************
 $Release Name: PACKAGE NAME $
 $Release Date: PACKAGE RELEASE DATE $
 *****************************************************************************/

#ifndef DATASTREAMSERVER_H
#define DATASTREAMSERVER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <ti/drivers/utils/List.h>
#include "icall_ble_api.h"

/*********************************************************************
 * CONSTANTS
 */
// Service UUID

#define AUDIO_GOOGLE_SERVICE_UUID         0x0001
#define AUDIO_GOOGL_TX_CHAR_UUID          0x0002
#define AUDIO_GOOGL_RX_CHAR_UUID          0x0003
#define AUDIO_GOOGL_CTL_CHAR_UUID         0x0004

#define AS_DATAIN_LEN  255

// Characteristic defines
#define AUDIOPROFILE_AUDIO   1
#define AS_DATAOUT_UUID 0xB002
#define AS_DATAOUT_LEN  255

// Profile UUIDs
extern const uint8_t ASUUID[ATT_UUID_SIZE];
extern const uint8_t AS_DataInUUID[ATT_UUID_SIZE];
extern const uint8_t AS_DataOutUUID[ATT_UUID_SIZE];




/*********************************************************************
 * TYPEDEFS
 */
// Data structure used to store incoming data
typedef struct
{
  uint16 connHandle;
  uint16 len;
  char pValue[];
} AS_dataIn_t;

// Data structure used to store ccc update
typedef struct
{
  uint16 connHandle;
  uint16 value;
} AS_cccUpdate_t;

// Data structure used to store outgoing data
typedef struct
{
    List_Elem elem;
    uint16_t connHandle;
    uint16_t offset;
    uint16_t len;
    uint8_t payload[];
} AudioStreamNode_t;

/*********************************************************************
 * Profile Callbacks
 */
// Callback to indicate client characteristic configuration has been updated
typedef void (*AS_onCccUpdate_t)( char *pValue );

// Callback when data is received
typedef void (*AS_incomingData_t)(uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue);

typedef struct
{
  AS_onCccUpdate_t         pfnOnCccUpdateCB;     // Called when client characteristic configuration has been updated
  AS_incomingData_t        pfnIncomingDataCB;    // Called when receiving data
} AS_cb_t;

/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      AS_addService
 *
 * @brief   This function initializes the Audio Audio Server service
 *          by registering GATT attributes with the GATT server.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t AS_addService( void );

/*
 * @fn      AS_registerProfileCBs
 *
 * @brief   Registers the profile callback function. Only call
 *          this function once.
 *
 * @param   profileCallback - pointer to profile callback.
 *
 * @return  SUCCESS or INVALIDPARAMETER
 */
bStatus_t AS_registerProfileCBs( AS_cb_t *profileCallback );

/*
 * @fn      AS_setParameter
 *
 * @brief   Set a Audio Audio Service parameter.
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
bStatus_t AS_setParameter( uint8 param, void *pValue, uint16 len);

extern bStatus_t AS_sendData(uint16_t connHandle, void *data, uint16_t len, uint8_t char_type);

//extern bStatus_t AS_sendNotification_Control(uint16_t connHandle, void *data, uint16_t len);

//extern bStatus_t AS_sendNotification_StartStop( uint8 *pValue, uint16 len );
/*
 * AS_processStream - Sends out as much as possible from the outgoing stream
 *                                    queue using BLE notifications.
 */
extern bStatus_t AS_processStream();

/*
 * AS_disconnectStream - Disconnect the steam.
 *                                       Clear and free up the existing outgoing stream queue.
 */
extern void      AS_disconnectStream();

/*
 * AS_setHeadroomLimit - Sets the limit on how much heap that needs to be available
 *                                       following a memory allocation.
 */
extern void     AS_setHeadroomLimit(uint16_t minHeapHeadroom);

/*
 * AS_allocateWithHeadroom - Checks if there will be enough free heap left
 *                                           following a memory allocation. If there is
 *                                           enough heap, it will allocate the memory.
 */
extern void*    AS_allocateWithHeadroom(uint16_t allocSize);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* DATASTREAMSERVER_H */
