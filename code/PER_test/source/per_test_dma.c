/*-----------------------------------------------------------------------------
|   File:      per_test_dma.c
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
| Purpose:    Functions for setup of DMA used with radio.
+------------------------------------------------------------------------------
| Decription: Functions to configure the DMA channel 0 for transport of data
|             either to or from the radio. For use by packet error rate test
|             application.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ===================================================*/
/*==== INCLUDES ==============================================================*/

#include "hal_main.h"
#include "per_test_dma.h"
#include "per_test_main.h"


/*==== GLOBAL VARS============================================================*/

BYTE radioPktBuffer[PACKET_LENGTH + 3];


/*==== PUBLIC FUNCTIONS ======================================================*/

/******************************************************************************
* @fn  dmaRadioSetup
*
* @brief
*      This funtion configures the static dmaConfig struct with the correct
*      values for receiving or transmitting data using the DMA.
*
* Parameters:
*
* @param  BYTE mode
*           Either RADIO_MODE_TX or RADIO_MODE_RX to select transmitter or
*           receiver settings
*
* @return void
*
******************************************************************************/
void dmaRadioSetup(BYTE mode)
{
    // Some configuration that are common for both TX and RX:

    // CPU has priority over DMA
    // Use 8 bits for transfer count
    // No DMA interrupt when done
    // DMA triggers on radio
    // Single transfer per trigger.
    // One byte is transferred each time.
    dmaConfig.PRIORITY       = DMA_PRI_LOW;
    dmaConfig.M8             = DMA_M8_USE_8_BITS;
    dmaConfig.IRQMASK        = DMA_IRQMASK_DISABLE;
    dmaConfig.TRIG           = DMA_TRIG_RADIO;
    dmaConfig.TMODE          = DMA_TMODE_SINGLE;
    dmaConfig.WORDSIZE       = DMA_WORDSIZE_BYTE;

    if (mode == RADIO_MODE_TX) {
        // Transmitter specific DMA settings

        // Source: radioPktBuffer
        // Destination: RFD register
        // Use the first byte read + 1
        // Sets the maximum transfer count allowed (length byte + data)
        // Data source address is incremented by 1 byte
        // Destination address is constant
        SET_WORD(dmaConfig.SRCADDRH, dmaConfig.SRCADDRL, radioPktBuffer);
        SET_WORD(dmaConfig.DESTADDRH, dmaConfig.DESTADDRL, &X_RFD);
        dmaConfig.VLEN           = DMA_VLEN_FIRST_BYTE_P_1;
        SET_WORD(dmaConfig.LENH, dmaConfig.LENL, (PACKET_LENGTH + 1));
        dmaConfig.SRCINC         = DMA_SRCINC_1;
        dmaConfig.DESTINC        = DMA_DESTINC_0;

    }
    else if (mode == RADIO_MODE_RX) {
        // Receiver specific DMA settings:

        // Source: RFD register
        // Destination: radioPktBuffer
        // Use the first byte read + 3 (incl. 2 status bytes)
        // Sets maximum transfer count allowed (length byte + data + 2 status bytes)
        // Data source address is constant
        // Destination address is incremented by 1 byte for each write
        SET_WORD(dmaConfig.SRCADDRH, dmaConfig.SRCADDRL, &X_RFD);
        SET_WORD(dmaConfig.DESTADDRH, dmaConfig.DESTADDRL, radioPktBuffer);
        dmaConfig.VLEN           = DMA_VLEN_FIRST_BYTE_P_3;
        SET_WORD(dmaConfig.LENH, dmaConfig.LENL, (PACKET_LENGTH + 3));
        dmaConfig.SRCINC         = DMA_SRCINC_0;
        dmaConfig.DESTINC        = DMA_DESTINC_1;
    }

    // Save pointer to the DMA configuration struct into DMA-channel 0
    // configuration registers
    SET_WORD(DMA0CFGH, DMA0CFGL, &dmaConfig);

    return;
}


/*==== END OF FILE ==========================================================*/
