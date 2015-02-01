/*-----------------------------------------------------------------------------
|   File:      hal_bui_SmartRF04EB.h
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
| Purpose:    Handling of RF04EB specific functionality
+------------------------------------------------------------------------------
| Decription: Commonly used macros and function prototypes for use with RF04EB.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/

#ifndef HAL_BUI_SMARTRF04EB_H
#define HAL_BUI_SMARTRF04EB_H

/*==== INCLUDES ==============================================================*/

#include "hal_bui_lcd.h"
#include "hal_bui_button.h"
#include "hal_bui_joystick_SmartRF04EB.h"

/*==== CONSTS ================================================================*/

/******************************************************************************
* LED - These settings are RF04EB specific
*
* LED1 = GLED (green)
* LED2 = RLED (red)     <- not connected for SoC
* LED3 = YLED (yellow)
* LED4 = BLED (blue)    <- not connected for SoC
*
******************************************************************************/
#define LED_OFF 0
#define LED_ON  1

#define LED1          P1_0
#define LED3          P1_1

#define GLED          LED1
#define YLED          LED3

/*==== TYPES =================================================================*/
/*==== EXPORTS ===============================================================*/

/*==== MACROS=================================================================*/

#define INIT_LED1()      do { LED1 = LED_OFF; IO_DIR_PORT_PIN(1, 0, IO_OUT); P1SEL &= ~0x01;} while (0)
#define INIT_LED3()      do { LED3 = LED_OFF; IO_DIR_PORT_PIN(1, 1, IO_OUT); P1SEL &= ~0x02;} while (0)

#define INIT_GLED()      INIT_LED1()
#define INIT_YLED()      INIT_LED3()

#define SET_LED1()  (LED1 = LED_ON)
#define SET_LED3()  (LED3 = LED_ON)


#define TOGGLE_LED1()  (LED1 ^= 1)
#define TOGGLE_LED3()  (LED3 ^= 1)

#define SET_GLED()  (GLED = LED_ON)
#define SET_YLED()  (YLED = LED_ON)

#define CLR_LED1()  (LED1 = LED_OFF)
#define CLR_LED3()  (LED3 = LED_OFF)

#define CLR_GLED()  (GLED = LED_OFF)
#define CLR_YLED()  (YLED = LED_OFF)

#define SET_LED_MASK( n )                            \
    do {                                             \
        if ((n) & 0x01) SET_LED1(); else CLR_LED1(); \
        if ((n) & 0x02) SET_LED3(); else CLR_LED3(); \
    } while (0)

/*==== FUNCTIONS =============================================================*/

#endif /* HAL_BUI_SMARTRF04EB_H */

/*==== END OF FILE ==========================================================*/
