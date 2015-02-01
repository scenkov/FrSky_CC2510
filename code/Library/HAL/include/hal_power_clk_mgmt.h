/*-----------------------------------------------------------------------------
|   File:      hal_power_clk_mgmt.h
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
| Purpose:    Power and clock management
+------------------------------------------------------------------------------
| Decription: In this file the handlers for power and clock management are placed
|               (constants, macros and functions)
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/

#ifndef HAL_POWER_CLK_MGMT_H
#define HAL_POWER_CLK_MGMT_H

/*==== INCLUDES ==============================================================*/

#include "hal_main.h"

/*==== CONSTS ================================================================*/

/* SEE DATA SHEET FOR DETAILS ABOUT THE FOLLOWING BIT MASKS */

/* Pre defined values for source, as used in halPowerClkMgmtSetMainClkSrc() */
#define CRYSTAL           0x00  /*  High speed Crystal Oscillator Control */
#define RC                0x01  /*  Low power RC Oscillator */

// Bit masks to check CLKCON register
#define OSC_BIT           0x40  // bit mask used to select/check the system clock oscillator
#define TICKSPD_BITS      0x38  // bit mask used to check the timer ticks output setting
#define CLKSPD_BIT        0x03  // bit maks used to check the clock speed
#define MAIN_OSC_BITS     0x7F  // bit mask used to control the system clock oscillator
                                // e.g. ~MAIN_OSC_BITS can be used to start Crystal OSC

// Bit masks to check SLEEP register
#define XOSC_STABLE_BIT   0x40  // bit mask used to check the stability of XOSC
#define HFRC_STB_BIT      0x20  // bit maks used to check the stability of the High-frequency RC oscillator
#define OSC_PD_BIT        0x04  // bit maks used to power down system clock oscillators

/*==== TYPES =================================================================*/
/*==== EXPORTS ===============================================================*/

/*==== MACROS=================================================================*/

// Macro for checking status of the high frequency RC oscillator.
#define HIGH_FREQUENCY_RC_OSC_STABLE (SLEEP & HFRC_STB_BIT)

// Macro for getting the clock division factor
#define CLKSPD  (CLKCON & CLKSPD_BIT)


// Macro for getting the timer tick division factor.
#define TICKSPD ((CLKCON & TICKSPD_BITS) >> 3)

// Macro for checking status of the crystal oscillator
#define XOSC_STABLE (SLEEP & XOSC_STABLE_BIT)

/*==== FUNCTIONS =============================================================*/

/******************************************************************************
* @fn  halPowerClkMgmtSetMainClkSrc
*
* @brief
* Function for setting the main clock oscillator source, turns off the
* clock source not used changing to XOSC will take approx 150 us
*  Settings TICKSPD equal CLKSPD
*
* Parameters:
*
* @param  UINT8 source
*
* @return void
*
******************************************************************************/
void halPowerClkMgmtSetMainClkSrc(UINT8 source);


#endif /* HAL_POWER_CLK_MGMT_H */

/*==== END OF FILE ==========================================================*/
