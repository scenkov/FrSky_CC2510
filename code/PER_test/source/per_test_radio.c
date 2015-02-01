/*-----------------------------------------------------------------------------
|   File:      per_test_radio.c
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
| Purpose:    Functions for radio and packet handling for PER test
+------------------------------------------------------------------------------
| Decription: All functions related to radio configuration and packet
|             handling for the packet error rate test application.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ===================================================*/
/*==== INCLUDES ==============================================================*/

#include "hal_main.h"
#include "per_test_main.h"


/*==== GLOBAL VARS ===========================================================*/

UINT8 chann_hoping;
UINT8 chann_counter;
UINT8 channel_map[50];
UINT8 calData1[50];
UINT8 calData3[50];
UINT8 tx_id[2]; // Network ID key that identifies transmitter/receiver pair

extern volatile UINT8 __xdata rxPacketStatus;

extern UINT8 bind_tx_id[2];
extern UINT8 listLength;		// Length of channel hop-list

/* random number generator */
#define RCTRL_CLOCK_LFSR    BV(2)

/*==== PUBLIC FUNCTIONS ======================================================*/


// Skip to next channel based on skip input
void nextChannel(UINT8 skip)
{
      chann_hoping += skip;
      
      if (chann_hoping >= listLength) 
         chann_hoping -= listLength;

    CHANNR = channel_map[chann_hoping]; //chann_hoping; //channel_map[i];

    FSCAL3 = calData3[chann_hoping];  
    FSCAL1 = calData1[chann_hoping];
    
    FSCAL3   = 0x89;
}


// 2404 MHz - FrSky 
//adjust here yor frequency offset in case  you cannot binding with your CC2500 ,module   
//static uint8_t fine = 0xd7;//* 215 *//give values from 0 to 255 for freq offset
                           //values from 0-127 offset increase frequency ,
                           //values from 255 to 127 decrease base frequency
                            //this is useful for tunning TX base frequency to frsky RX freq.
void frsky2way_init(int bind) {
    
        perRssiOffset = 72;// Set proper RSSI offset for receiver

        MCSM1 = 0x0c;
        MCSM0 = 0x18;
//        PKTLEN = 0x19;
        PKTCTRL1 = 0x04;
        PKTCTRL0 = 0x05;
        PA_TABLE0 = 0xff;
        FSCTRL1 = 0x08;
        FSCTRL0 = 0x00; //fine;
        FREQ2 = 0x5c;
        FREQ1 = 0x76;
        FREQ0 = 0x27;
        MDMCFG4 = 0xaa;
        MDMCFG3 = 0x39;
        MDMCFG2 = 0x11;
        MDMCFG1 = 0x23;
        MDMCFG0 = 0x7a;
        DEVIATN = 0x42;
        FOCCFG = 0x16;
        BSCFG = 0x6c;
        AGCCTRL2 == bind ? 0x43 : 0x03;      
        AGCCTRL1 = 0x40;
        AGCCTRL0 = 0x91;
        FREND1 = 0x56;
        FREND0 = 0x10;
        FSCAL3 = 0xa9;
        FSCAL2 = 0x0a;
        FSCAL1 = 0x00;
        FSCAL0 = 0x11;
        FSTEST = 0x59;
        TEST2 = 0x88;
        TEST1 = 0x31;
        TEST0 = 0x0b;
 //       FIFOTHR = 0x07;
        ADDR = 0x00;

        // Common radio settings for CCxx10, any frequency and data rate
        CHANNR   = 0x00;            // Channel number.
 //       MCSM1    = 0x30;            // Main Radio Control State Machine configuration.
        IOCFG2   = 0x1C;            // P1_7 GDO2 LNA_PD. Can be used to control an external LNA or RX/TX switch. 
                                    // Signal is asserted when the radio enters RX state
        IOCFG1   = 15;            // P1_6 GDO1 output pin configuration.
        IOCFG0   = 0x01;            // P1_5 GDO0 output pin configuration. Sync word.

        PKTLEN   = PACKET_LENGTH;   // Packet length.
    
     for(int i = 0; i < listLength; i++) {
           CHANNR = channel_map[i];
           RFST   = RFST_SIDLE;    
           RFST   = RFST_SCAL; // Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode 
           while(MARCSTATE != 0x01);

           calData1[i] = FSCAL1;
           calData3[i] = FSCAL3;
    }

    FSCAL3   = 0x89;// Disable charge-pump calibration for faster switching now that it has been calibrated
    CHANNR   = 0x00;
    
    RFST   = RFST_SCAL; // Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode 
    while(MARCSTATE != 0x01);
            
    return;
}


/******************************************************************************
* @fn  pktCheckValidity
*
* @brief
*      Checks the received packet length and network ID to decide if it is a
*      valid PER test packet, hence affecting PER statistics.
*      The packet's CRC and sequence number will be analyzed in the
*      process of updating the appropriate variables needed to keep PER
*      statistics
*
* Parameters:
*
* @param  void
*
* @return BOOL
*         TRUE: Packet was a PER test packet
*         FALSE: Packet was not recognized as a PER test packet
*
******************************************************************************/
BOOL pktCheckValidity(void)
{
    // Check if the packet length is correct (byte 0 in packet)
    if (radioPktBuffer[0] == PACKET_LENGTH) {

        // Check if the network identifier bytes matches our network ID (byte 1 + 2)
        if (pktCheckId()) {
            // We have a match, this packet is probably meant for us

            // Check if received packet has correct CRC
            if (pktCheckCrc()) {
                rxPacketStatus = PKT_OK;
                return TRUE;
            }else {
                rxPacketStatus = CRC_ERROR;
                return FALSE;
            }
            }else {
                rxPacketStatus = PKT_ID_ERROR;
                return FALSE;
            }
    }
 rxPacketStatus = PKT_ERROR;
 return FALSE;

}

/******************************************************************************
* @fn  convertRssiByte
*
* @brief
*      Converts the RSSI (received signal strength indicator) value,
*      given as a 2's complement number, to dBm value. This requires that a
*      proper RSSI offset value is specified in global variable perRssiOffset
*      before use.
*
* Parameters:
*
* @param  BYTE rssiComp
*                   The RSSI value received from the radio as a 2's complement
*                   number
*
* @return INT16
*           The RSSI value in dBm
*
******************************************************************************/
INT16 convertRssiByte(BYTE rssiComp)
{
    // Convert RSSI value from 2's complement to decimal value.
    INT16 rssiDec = (INT16) rssiComp;

    // Convert to absolute value (RSSI value from radio has resolution of
    // 0.5 dBm) and subtract the radio's appropriate RSSI offset.
    if(rssiDec < 128){
        return (rssiDec/2) - perRssiOffset;
    }
    else{
        return ((rssiDec - 256)/2) - perRssiOffset;
    }
}




/*==== PRIVATE FUNCTIONS =====================================================*/

/******************************************************************************
* @fn  pktCheckId
*
* @brief
*      Check the NETWORK_ID_KEY of the received packet to ensure we are the
*      intended recipient.
*
* Parameters:
*
* @param  void
*
* @return BOOL
*         TRUE: The NETWORK_ID_KEY was correct
*         FALSE: The NETWORK_ID_KEY was wrong
*
******************************************************************************/
static BOOL pktCheckId(void)
{
    // The NETWORK_ID_KEY is sent as the second and third byte in the packet
    if ((radioPktBuffer[1]== tx_id[0]) &&
         (radioPktBuffer[2]== tx_id[1])) {
          // Reset the NETWORK_ID_KEY from packet buffer to ensure that the next packet will
          // have to update the buffer with it again (to rule out false positives).
          radioPktBuffer[1] = 0x00;
          radioPktBuffer[2] = 0x00;
          return TRUE;
    }
    return FALSE;
}

/******************************************************************************
* @fn  pktCheckCrc
*
* @brief
*      Check the if the CRC is correct for the received packet
*
* Parameters:
*
* @param  void
*
* @return BOOL
*          TRUE: The CRC was correct
*          FALSE: The CRC was wrong
*
******************************************************************************/
static BOOL pktCheckCrc(void)
{
    // Check if CRC_OK bit (bit 7) in the second status byte received is set
    if(radioPktBuffer[PACKET_LENGTH + 2] & 0x80){
       radioPktBuffer[PACKET_LENGTH + 2] = 0x00;   // Clear status byte in buffer
       return TRUE;
    }
    else {
        radioPktBuffer[PACKET_LENGTH + 2] = 0x00;   // Clear status byte in buffer
        return FALSE;
    }
}


/*==== END OF FILE ==========================================================*/
