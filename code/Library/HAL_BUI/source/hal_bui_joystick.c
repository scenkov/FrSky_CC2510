/*-----------------------------------------------------------------------------
|   File:      hal_bui_joystick.c
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
| Purpose:    Button control for use with SmartRF04EB
+------------------------------------------------------------------------------
| Decription: Function implementations for button
|               control when using with SmartRF04EB
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
* @fn  halBuiJoystickPushed
*
* @brief
*      This function detects if the joystick is being pushed. The function
*      implements software debounce. Return true only if previously called
*      with joystick not pushed. Return true only once each time the joystick
*      is pressed.
*
* Parameters:
*
* @param  void
*
* @return BOOL
*          TRUE: Button is being pushed
*          FALSE: Button is not being pushed
*
******************************************************************************/
BOOL halBuiJoystickPushed( void ) {
   UINT8 i;
   BOOL value;
   static BOOL prevValue;

   if (value = JOYSTICK_PRESSED()){
      for(i = 0;i < BUTTON_ACTIVE_TIMEOUT; i++){
         if(!JOYSTICK_PRESSED()){
            value = FALSE;
            break;
         }
      }
   }

   if(value){
      if (!prevValue){
         value = prevValue = TRUE;
      }
      else{
         value = FALSE;
      }
   }
   else{
      prevValue = FALSE;
   }

   return value;
}

/******************************************************************************
* @fn  halBuiGetJoystickDirection
*
* @brief
*      This function utilizes the ADC to give an indication of the current
*      position of the joystick. Current support is for 90 degrees
*      positioning only.
*
*      The joystick control is encoded as an analog voltage.  Keep on reading
*      the ADC until two consecutive key decisions are the same.
*
*
*      Meassured values from the ADC for CC1110 / CC2510:
*      ------------------------------------
*      |Direction | REV 'x'   | REV 'y'   |
*      ------------------------------------
*      |DOWN      | 0xFF-0x00 | 0x00      |
*      |LEFT      | 0x2F-0x30 | 0x35-0x36 |
*      |RIGHT     | 0x4C-0x4E | 0x55-0x57 |
*      |UP        | 0x5A-0x5C | 0x65-0x67 |
*      |CENTER    | 0x69-0x6B | 0x76-0x78 |
*      ------------------------------------
*      NOTE: 'x' is old chip revisions, 'y' is the new (CC1110 (3), CC2510 (4))
*
* Parameters:
*
* @param  BYTE chipType
*          The part number identifying the chip
*
* @return JOYSTICK_DIRECTION_ENUM
*          DOWN:    Joystick direction is down (270 degrees)
*          LEFT:    Joystick direction is left (180 degrees)
*	   RIGHT:   Joystick direction is right (0 degrees)
*	   UP:      Joystick direction is up (90 degrees)
*	   CENTERED: Joystick direction is centred (passive position)
*
******************************************************************************/

JOYSTICK_DIRECTION_ENUM halBuiGetJoystickDirection(BYTE chipType) {
    INT8 adcValue;
    JOYSTICK_DIRECTION_ENUM direction[2];

    do{
        direction[1] = direction[0];
        adcValue = halAdcSampleSingle(ADC_REF_AVDD, ADC_7_BIT, ADC_INPUT_JOYSTICK);
        adcValue <<= 1; // Shift from 7 to 8 bit value, to match table values

        if ((chipType == 0x81 /*CC2510*/ && CHIPREVISION < REV_E) || (chipType == 0x01/*1110*/ && CHIPREVISION < REV_D)) {
            // Use these limits for "old" chip revisions
            if (adcValue < 0x10) {
                direction[0] = DOWN;
            } else if (adcValue < 0x40) {
                direction[0] = LEFT;
            } else if (adcValue < 0x50) {
                direction[0] = RIGHT;
            } else if (adcValue < 0x60) {
                direction[0] = UP;
            } else {
                direction[0] = CENTERED;
            }
        }
        else
        {
            // Use these limits for "new" chip revisions
            if (adcValue < 0x10) {
                direction[0] = DOWN;
            } else if (adcValue < 0x40) {
                direction[0] = LEFT;
            } else if (adcValue < 0x60) {
                direction[0] = RIGHT;
            } else if (adcValue < 0x70) {
                direction[0] = UP;
            } else {
                direction[0] = CENTERED;
            }
        }
    } while(direction[0] != direction[1]);

  return direction[0];
}


/*==== END OF FILE ==========================================================*/
