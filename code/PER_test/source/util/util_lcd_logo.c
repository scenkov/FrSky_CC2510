/*-----------------------------------------------------------------------------
|   File:      util_lcd_logo.c
|   Target:    SmartRF04 EB, cc1110, cc2510
|   Author:    KJA, ESY
|   Revised:   2007-09-06
|   Revision:  1.0
|   Project:   PER_test
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
| Purpose:    Displays logo and chip name and revision number on LCD
+------------------------------------------------------------------------------
| Decription: This function shows the Chipcon logo and chip name and chip
|             revision number in the LCD display on the SmartRF04 Evaluation
|             Board.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ===================================================*/
/*==== INCLUDES ==============================================================*/

#include <hal_main.h>
#include <string.h>
#include <stdio.h>


/*==== PUBLIC FUNCTIONS ======================================================*/
// Prototype
void showLogo(void);


// Char1  Char2  Char3
// -----  -####  -----	
// ----#  #####  #----
// ---##  #----  ###--
// --###  -----  --##-
// -###-  -----  ---##
// -###-  -####  #---#
// ###--  #----  -##--
// ###-#  -----  -----	
//		
// ###-#  -----  -----
// ###--  #----  -##--
// -###-  -####  #---#
// -###-  -----  ----#
// --###  -----  --##-
// ---##  #----  ###--
// ----#  #####  #----
// -----  -####  -----
// Char4  Char5  Char6

/*
void showLogo(void)
{
    char chipconLogo[6][8] ={
        {0x00, 0x01, 0x03, 0x07, 0x0E, 0x0E, 0x1C, 0x1D}, //char1
        {0x0F, 0x1F, 0x10, 0x00, 0x00, 0x0F, 0x10, 0x00}, //char2
        {0x00, 0x10, 0x1C, 0x06, 0x03, 0x11, 0x0C, 0x00}, //char3
        {0x1D, 0x1C, 0x0E, 0x0E, 0x07, 0x03, 0x01, 0x00}, //char4
        {0x00, 0x10, 0x0F, 0x00, 0x00, 0x10, 0x1F, 0x0F}, //char5
        {0x00, 0x0C, 0x11, 0x01, 0x06, 0x1C, 0x10, 0x00}  //char6
    };


    UINT8 i;
    char logo[2][16] = {0};
    char chipName[14] = {0};

    //init new symbols
    for(i = 0; i < 6; i++){
      halBuiLcdInitNewSymbol(&chipconLogo[i][0], CHAR1_ADDRESS + (i*0x08));
    }

    logo[0][0] = 0x01;
    logo[0][1] = 0x02;
    logo[0][2] = 0x03;
    strcpy(&logo[0][3], (char*)"  Chipcon    ");

    logo[1][0] = 0x04;
    logo[1][1] = 0x05;
    logo[1][2] = 0x06;
    logo[1][3] = ' ';

    if (PARTNUM == 0x81) {           // Part number for CC2510
        sprintf(chipName, "  CC2510 (%u)", CHIPREVISION);
    }
    else if (PARTNUM == 0x01) {      // Part number for CC1110
        sprintf(chipName, "  CC1110 (%u)", CHIPREVISION);
    }
    strcpy(&logo[1][3], chipName);

    // Write logo and chipname to LCD
    halBuiLcdUpdate(logo[0], logo[1]);
}
*/
