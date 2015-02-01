/*-----------------------------------------------------------------------------
|   File:      per_test_menu.c
|   Target:    cc1110, cc2510
|   Author:    ESY
|   Revised:   2007-09-06
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
| Purpose:    Functions for menu and updates on LCD for PER test
+------------------------------------------------------------------------------
| Decription: All functions related to setup menu and other display messages
|             on the LCD for use by the packet error rate test application.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ===================================================*/
/*==== INCLUDES ==============================================================*/

#include "hal_main.h"
#include <stdio.h>
#include "per_test_main.h"

/*==== PUBLIC FUNCTIONS ======================================================*/


/******************************************************************************
* @fn  selectFromMenu
*
* @brief
*      Show a vertically scrolled text menu on the LCD. Text lines given in
*      an array is shown in lower line of the LCD, these lines are browsable.
*      The menu is navigated by using the joystick up and down, and press
*      S1 button to select an option. The function then returns the element number
*      in the given menu list that was chosen by the user.
*
* Parameters:
*
* @param  const char *ppMenuList[]
*           A pointer to an array of text strings, each array element points to
*           a char array, each constituting a menu option.
* @param  UINT8 numOptions
*           The number of menu elements (menu options) in the menu list.
* @param  UINT8 selectedOption
*           The menu element number (option) the menu should start at
*
* @return UINT8
*           The menu element number (option) selected by user
*
******************************************************************************/
/*
UINT8 selectFromMenu(const char *ppMenuList[], UINT8 numOptions,
                            UINT8 selectedOption)
{
    // Flag set when the LCD needs an update
    BOOL updLcd = TRUE;

    // Conserve previous and current joystick direction to detect movements
    JOYSTICK_DIRECTION_ENUM jd_latest = CENTERED;
    JOYSTICK_DIRECTION_ENUM jd_current;

    // Values for the up and down arrow char symbols
    char display_up = ARROW_UP;
    char display_down = ARROW_DOWN;

    // Fail-safe validation to avoid inconsistent input
    if (selectedOption >= numOptions) {
        selectedOption = numOptions - 1;
    }

    // Stay in the menu until an option has been selected
    while (TRUE) {

//        if (halBuiButtonPushed()) {
          if ( !(P1 & 0x04) ) {
            return selectedOption;
        }

        // Update the current joystick direction
        jd_current = halBuiGetJoystickDirection(PARTNUM);

        // Check for joystick movements
        if(jd_current != jd_latest) {
            jd_latest = jd_current;

            switch (jd_latest) {
            case UP:
                // Go to previous menu entry in list if possible
                if (selectedOption > 0) {
                    selectedOption--;
                    updLcd = TRUE;
                }
                break;

            case DOWN:
                // Go to next menu entry in list if possible
                if (selectedOption < numOptions - 1) {
                    selectedOption++;
                    updLcd = TRUE;
                }
                break;

            case LEFT:
            case RIGHT:
            case CENTERED:
            default:
                // No functionality for these directions
                break;
            }
        }

        // Update the LCD only if necessary
        if (updLcd) {

            updLcd = FALSE;
            halBuiLcdUpdateLine(LINE2, (char __xdata*) ppMenuList[selectedOption]);

            // For the menu option at the top or bottom of the list,
            // the corresponding arrow symbol must be hidden
            if (selectedOption == 0) {
                display_up = ' ';
            }
            else {
                display_up = ARROW_UP;
            }

            if (selectedOption == (numOptions - 1)) {
                display_down = ' ';
            } else {
                display_down = ARROW_DOWN;
            }

            // Write arrow symbols to end of line on the LCD
            halBuiLcdUpdateChar(LINE1, LINE_SIZE - 1 , display_up);
            halBuiLcdUpdateChar(LINE2, LINE_SIZE - 1 , display_down);
        }
    }
}

*/
/******************************************************************************
* @fn  selectMode
*
* @brief
*      Controlls mode choice
*
* Parameters:
*
* @param void
*
* @return BYTE
*           RADIO_MODE_TX or RADIO_MODE_RX
*
*
******************************************************************************/
/*
BYTE selectMode(void) {

    halBuiLcdUpdateLine(LINE1, (char __xdata*)pModeMenuTop);

    return modeValues[selectFromMenu((const char **) modeMenu,
                                     NUMBER_OF_MODES, (NUMBER_OF_MODES - 1))];
}

*/
/******************************************************************************
* @fn  selectBurstSize
*
* @brief
*      Shows a menu for input of number of packets the transmitter should
*      send in a packet burst for the PER test.
*
* Parameters:
*
* @param void
*
* @return UINT32
*           Number of packet the transmitter should transmit
*
*
******************************************************************************/
/*
UINT32 selectBurstSize(void) {

    halBuiLcdUpdateLine(LINE1, (char __xdata*) pBurstSizeMenuTop);

    return burstSizeValues[selectFromMenu((const char **) burstSizeMenu,
                                          NUMBER_OF_BURST_SIZES,
                                          (NUMBER_OF_BURST_SIZES - 1))];
}
*/

/******************************************************************************
* @fn  selectRadioFrequency
*
* @brief
*      Controls choosing the frequency
*
* Parameters:
*
* @param  void
*
* @return UINT32
*           The selected frequency, in kHz
*
******************************************************************************/
/*
UINT32 selectRadioFrequency(void) {

    UINT32 chosenFreq = 0;

    halBuiLcdUpdateLine(LINE1, (char __xdata*) pFreqMenuTop);

    if (PARTNUM == PARTNUM_CC1110) {
        chosenFreq = freqValuesCC1110[selectFromMenu((const char **) freqMenuCC1110,
                                                     NUMBER_OF_FREQUENCIES_CC1110,
                                                     (NUMBER_OF_FREQUENCIES_CC1110 - 1))];
    }
    else if (PARTNUM == PARTNUM_CC2510) {
        chosenFreq = freqValuesCC2510[selectFromMenu((const char **) freqMenuCC2510,
                                                     NUMBER_OF_FREQUENCIES_CC2510,
                                                     (NUMBER_OF_FREQUENCIES_CC2510 - 1))];
    }
    return chosenFreq;
}

*/
/******************************************************************************
* @fn  selectDataRate
*
* @brief
*      Shows a menu on LCD to let user select a data rate
*
* Parameters:
*
* @param void
*
* @return UINT32
*         The selected data rate, in bits per second
*
******************************************************************************/
/*
UINT32 selectDataRate(void) {

    UINT32 chosenDataRate = 0;

    halBuiLcdUpdateLine(LINE1, (char __xdata*) pDataRateMenuTop);

    if (PARTNUM == PARTNUM_CC1110) {
        chosenDataRate = dataRateValuesCC1110[selectFromMenu((const char **) dataRateMenuCC1110,
                                                             NUMBER_OF_DATA_RATES_CC1110,
                                                             (NUMBER_OF_DATA_RATES_CC1110 - 1))];
    }
    else if (PARTNUM == PARTNUM_CC2510) {
        chosenDataRate = dataRateValuesCC2510[selectFromMenu((const char **) dataRateMenuCC2510,
                                                             NUMBER_OF_DATA_RATES_CC2510,
                                                             (NUMBER_OF_DATA_RATES_CC2510 - 1))];
    }

    return chosenDataRate;
}

*/
/******************************************************************************
* @fn  checkChipVersion
*
* @brief
*      Checks the chip's part number and version register to determine if
*      it is supported by this software. If it is too old/new a menu with an
*      error message is displayed and the program is halted. This is
*      necessary because PER testing with (possibly) mismatched radio settings
*      (which are hard coded into this software) makes little sense.
*
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/
/*
void checkChipVersion(void)
{
    // Check if chip version is either too old or too new (i.e. unsupported),
    // and if so write an error message to LCD and trap the program
    if (((PARTNUM == PARTNUM_CC1110) && VERSION < CC1110_MIN_SUPPORTED_VERSION) ||
        ((PARTNUM == PARTNUM_CC2510) && (VERSION < CC2510_MIN_SUPPORTED_VERSION))) {
        // The chip is too old, browse error message forever
        halBuiLcdUpdateLine(LINE1, (char __xdata*) pVersionOldTop);
        while (1) {
            selectFromMenu((const char **) versionOldMenu,
                           VERSION_OLD_MENU_LINES, 0);
        }

    }
    else if (((PARTNUM == PARTNUM_CC1110) && (VERSION > CC1110_MAX_SUPPORTED_VERSION)) ||
             ((PARTNUM == PARTNUM_CC2510) && (VERSION > CC2510_MAX_SUPPORTED_VERSION))) {
        // The chip is too new for this software, browse error message forever
        halBuiLcdUpdateLine(LINE1, (char __xdata*) pVersionNewTop);
        while (1) {
            selectFromMenu((const char **) versionNewMenu,
                           VERSION_NEW_MENU_LINES, 0);
        }
    }
    else if ((PARTNUM != PARTNUM_CC1110) && (PARTNUM != PARTNUM_CC2510)) {
        // Another chip, which is not supported
        halBuiLcdUpdateLine(LINE1, (char *) "UNSUPPORTED CHIP");
        halBuiLcdUpdateLine(LINE2, (char *) " ");
        while (1); // Halt here
    }
    return;
}

*/

/******************************************************************************
* @fn  lcdWritePer
*
* @brief
*      This function updates the instantaneous PER on the LCD
*
*
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/
/*
void lcdWritePer(void)
{
    char perText[5];      // Array to hold the PER in chars

    // The PER in units per 1000
    UINT32 per = (((perExpectedSeqNum - 1) - perRcvdPkts + perBadPkts)*1000)/
        (perExpectedSeqNum - 1);

    // Convert the PER to percent and store in char array
    sprintf(perText, "%4.1f", ((float)per)/10);

    halBuiLcdUpdateChar(LINE1, 6, perText[0]);
    halBuiLcdUpdateChar(LINE1, 7, perText[1]);
    // We skip the decimal point
    halBuiLcdUpdateChar(LINE1, 9, perText[3]);
    return;
}
*/

/******************************************************************************
* @fn  lcdWriteRssi
*
* @brief
*      This function updates the averaged RSSI on the LCD
*
*
* Parameters:
*
* @param  WORD rssiSum
*         The sum used in calculation of the mean as INT16
*
* @return void
*
******************************************************************************/
/*
void lcdWriteRssi(INT16 rssiSum)
{
    char rssiText[5];       // Array to hold the RSSI mean in chars

    // Calculate the average RSSI and store in char array
    sprintf(rssiText, "%4d", (rssiSum/RSSI_AVG_WINDOW_SIZE));

    halBuiLcdUpdateChar(LINE2, 6, rssiText[0]);
    halBuiLcdUpdateChar(LINE2, 7, rssiText[1]);
    halBuiLcdUpdateChar(LINE2, 8, rssiText[2]);
    halBuiLcdUpdateChar(LINE2, 9, rssiText[3]);
    return;
}

*/
/******************************************************************************
* @fn  lcdWriteSeqNum
*
* @brief
*      This function updates LINE2 of LCD with the transmitted sequence number
*
*
* Parameters:
*
* @param  UNIT32 seqNum
*         The sequence number to print on LCD
*
* @return void
*
******************************************************************************/
/*
void lcdWriteSeqNum(UINT32 seqNum)
{
    char printSeqNumber[LINE_SIZE]; // Array to hold text string to display

    sprintf(printSeqNumber, "%15lu", seqNum);
    halBuiLcdUpdateLine(LINE2, printSeqNumber);
    return;
}

*/
/*==== END OF FILE ==========================================================*/
