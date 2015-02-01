/*-----------------------------------------------------------------------------
|   File:      hal_wait.c
|   Target:    cc1110, cc2510
|   Author:    TFL
|   Revised:   2007-09-05
|   Revision:  1.0
+------------------------------------------------------------------------------
|  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.
|
|  IMPORTANT: Your use of this Software is limited to those specific rights
|  granted under the terms of a software license agreement between the user who
|  downloaded the software, his/her employer (which must be your employer) and
|  Texas Instruments Incorporated (the "License"). You may not use this Software
|  unless you agree to abide by the terms of the License. The License limits
|  your use, and you acknowledge, that the Software may not be modified, copied
|  or distributed unless embedded on a Texas Instruments microcontroller or used
|  solely and exclusively in conjunction with a Texas Instruments radio
|  frequency transceiver, which is integrated into your product. Other than for
|  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
|  works of, modify, distribute, perform, display or sell this Software and/or
|  its documentation for any purpose.
|
|  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
|  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
|  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
|  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
|  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
|  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
|  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING
|  BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
|  CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
|  SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
|  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
|
|  Should you have any questions regarding your right to use this Software,
|  contact Texas Instruments Incorporated at www.TI.com.
|
+------------------------------------------------------------------------------
| Purpose:    Waiting function
+------------------------------------------------------------------------------
| Decription: This file contains the functionality to pause the execution.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/

/*==== INCLUDES ==============================================================*/

#include "hal_main.h"

/*==== MACROS ================================================================*/
/*==== CONSTS ================================================================*/
/*==== TYPES =================================================================*/
/*==== LOCALS ================================================================*/
/*==== PRIVATE FUNCTIONS =====================================================*/
/*==== PUBLIC FUNCTIONS ======================================================*/

/******************************************************************************
* @fn  halWait
*
* @brief
*      This function waits approximately a given number of m-seconds
*      regardless of main clock speed.
*
* Parameters:
*
* @param  BYTE	 wait ( hence max value = 255 )
*         The number of m-seconds to wait.
*
* @return void
*
******************************************************************************/
void halWait(BYTE wait){

   UINT32 largeWait;

   if(wait == 0)
   {return;}

   largeWait = ((UINT16) (wait << 7));
   largeWait += 59*wait;

   largeWait = (largeWait >> CLKSPD);
   while(largeWait--);

   return;
}

/*==== END OF FILE ==========================================================*/
