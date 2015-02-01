/*-----------------------------------------------------------------------------
|   File:      hal_bui_button.h
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
| Purpose:    Button control.
+------------------------------------------------------------------------------
| Decription: Functions/Macros for using/enabling the Button.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/

#ifndef HAL_BUI_BUTTON_H
#define HAL_BUI_BUTTON_H

/*==== INCLUDES ==============================================================*/

#include "hal_main.h"

/*==== CONSTS ================================================================*/

/******************************************************************************
* Button S1
******************************************************************************/
#define BUTTON_PUSH         P1_2
#define BUTTON_PRESSED()    (!BUTTON_PUSH)
//#define INIT_BUTTON()       (P1SEL &= ~0x04) (P1DIR &= ~0x04) (P1INP |= 0x04)
#define INIT_BUTTON()       do {(P1SEL &= ~0x04); (P1DIR &= ~0x04); (P1INP |= 0x00);} while (0) // P1_2 PULL_UP

#define BUTTON_ACTIVE_TIMEOUT   10   // Used to implement debounce

/*==== TYPES =================================================================*/
/*==== EXPORTS ===============================================================*/
/*==== MACROS=================================================================*/
/*==== FUNCTIONS =============================================================*/

/******************************************************************************
* @fn  halBuiButtonPushed
*
* @brief
*      This function detects if the button is being pushed. The function
*      implements software debounce. Return true only if previuosly called
*      with button not pushed. Return true only once each time the button
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
BOOL halBuiButtonPushed( void );

#endif /* HAL_BUI_BUTTON_H */

/*==== END OF FILE ==========================================================*/
