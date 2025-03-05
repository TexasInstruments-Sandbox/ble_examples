/******************************************************************************

@file  app_simple_gatt.c

@brief This file contains the Simple GATT application functionality

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

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <app_main.h>
#include <common/Profiles/logger_gatt/logger_gatt_profile.h>

//*****************************************************************************
//! Defines
//*****************************************************************************

//*****************************************************************************
//! Globals
//*****************************************************************************



//*****************************************************************************
//! Functions
//*****************************************************************************



/*********************************************************************
 * @fn      SimpleGatt_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Simple GATT profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t SimpleGatt_start( void )
{
  bStatus_t status = SUCCESS;

  // Add Simple GATT service
  status = SimpleGattProfile_addService();
  if(status != SUCCESS)
  {
    // Return status value
    return(status);
  }

  // Setup the Simple GATT Characteristic Values
  // For more information, see the GATT and GATTServApp sections in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  {
    uint8_t charValue1[GATTPROFILE_TIMESTAMP_LEN] = { 0, 0, 0, 0, 0,0,0 };
    uint8_t charValue2[GATTPROFILE_TIMESTAMP_LEN] = { 0, 0, 0, 0, 0,0,0 };
    uint8_t charValue3[GATTPROFILE_TIMESTAMP_LEN] = { 0, 0, 0, 0, 0,0,0 };
    uint8_t charValue4[GATTPROFILE_TIMESTAMP_LEN] = { 0, 0, 0, 0, 0,0,0 };
    uint8_t charValue5[GATTPROFILE_TIMESTAMP_LEN] = {0, 0, 0, 0, 0, 0,0 };

    SimpleGattProfile_setParameter( SIMPLEGATTPROFILE_CHAR1, GATTPROFILE_TIMESTAMP_LEN,
                                    charValue1 );
    SimpleGattProfile_setParameter( SIMPLEGATTPROFILE_CHAR2, GATTPROFILE_TIMESTAMP_LEN,
                                    charValue2 );
    SimpleGattProfile_setParameter( SIMPLEGATTPROFILE_CHAR3, GATTPROFILE_TIMESTAMP_LEN,
                                    charValue3 );
    SimpleGattProfile_setParameter( SIMPLEGATTPROFILE_CHAR4, GATTPROFILE_TIMESTAMP_LEN,
                                    charValue4 );
    SimpleGattProfile_setParameter( SIMPLEGATTPROFILE_CHAR5, GATTPROFILE_TIMESTAMP_LEN,
                                    charValue5 );
  }

  // Return status value
  return(status);
}

