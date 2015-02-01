/*-----------------------------------------------------------------------------
|   File:      per_test_main.h
|   Target:    cc1110, cc2510
|   Author:    ESY
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
| Purpose:    PER test main header file
+------------------------------------------------------------------------------
| Decription: This file contains settings and defines for the packet error rate
|             test application.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/

#ifndef PER_TEST_MAIN_H
#define PER_TEST_MAIN_H


/*==== INCLUDES ==============================================================*/
#include "per_test_dma.h"


/*==== CONSTS ================================================================*/

#ifndef false
   #define false 0
#endif

#ifndef true
   #define true 1
#endif

// Macros for turning timers on or off
#define TIMER3_RUN(value)      (T3CTL = (value) ? T3CTL|0x10 : T3CTL&~0x10)
#define TIMER4_RUN(value)      (T4CTL = (value) ? T4CTL|0x10 : T4CTL&~0x10)
#define T3OVFIF                      0x01

// RX packet status
#define PKT_OK                          0x01
#define CRC_ERROR                       0x02
#define	TIMEOUT_ERROR                   0x04
#define PKT_ERROR                       0x08
#define DMA_ERROR                       0x10
#define PKT_ID_ERROR                    0x20
#define PKT_COUNT_ERROR                 0x40
#define PKT_STATUS_UNKNOWN              0xFE


#define SEEK_CHANSKIP	13		
#define MAX_MISSING_PKT	20


#define PARTNUM_CC1110               0x01   // Part number for the CC1110
#define PARTNUM_CC2510               0x81   // Part number for the CC2510

// Define oldest and newest chip revision supported by radio configuration
#define CC1110_MIN_SUPPORTED_VERSION 0x03   // Oldest CC1110 revision supported
#define CC1110_MAX_SUPPORTED_VERSION 0x03   // Newest CC1110 revision supported
#define CC2510_MIN_SUPPORTED_VERSION 0x04   // Oldest CC2510 revision supported
#define CC2510_MAX_SUPPORTED_VERSION 0x04   // Newest CC2510 revision supported

#define STROBE_TX                    0x03   // Strobe commands for the RFST
#define STROBE_RX                    0x02   // register

#define IRQ_DONE                     0x10   // The IRQ_DONE bit in the RFIF-
                                            // and RFIM-register
#define DMAARM_CHANNEL0              0x01   // The value to arm the DMA
                                            // channel 0 in the DMAARM register

#define NUMBER_OF_MODES              2      // Operational mode constants
#define RADIO_MODE_TX                0x10
#define RADIO_MODE_RX                0x20

#define VERSION_OLD_MENU_LINES       10     // Number of text menu lines used
#define VERSION_NEW_MENU_LINES       9      // for the warnings shown on LCD for
                                            // unsupported chip versions

/* Some adjustable settings */
#define PACKET_LENGTH                17     // Payload length. Does not include
                                            // 1 length byte (prefixing payload,
                                            // containing this value) and 2
                                            // appended bytes CRC. Does include
                                            // 2 bytes network identifier and 4
                                            // bytes sequence number, hence
                                            // minimum value is 6.
#define RSSI_AVG_WINDOW_SIZE         32     // Size of ring buffer for RSSI
                                            // values to average over (sliding
                                            // window). Max 256

#define NETWORK_ID_KEY               0x5AA5 // Network ID key that identifies
                                            // transmitter/receiver pair


/* Some NOT SO adjustable settings ** See also LOCALS section if manipulated */

// Preset frequency alternatives
#define NUMBER_OF_FREQUENCIES_CC1110  4
#define FREQUENCY_1_CC1110       915000     // kHz. NOTE: If you want to alter
#define FREQUENCY_2_CC1110       903000     // these values you will also have
#define FREQUENCY_3_CC1110       868000     // to modify the register settings
#define FREQUENCY_4_CC1110       433500     // radioConfigure() in
                                            // per_test_radio.c

#define NUMBER_OF_FREQUENCIES_CC2510  4
#define FREQUENCY_1_CC2510      2480000     // kHz. NOTE: If you want to alter
#define FREQUENCY_2_CC2510      2460000     // these values you will also have
#define FREQUENCY_3_CC2510      2440000     // to modify the register settings in
#define FREQUENCY_4_CC2510      2420000     // radioConfigure() in
                                            // per_test_radio.c
// Preset data rate alternatives
#define NUMBER_OF_DATA_RATES_CC1110   3
#define DATA_RATE_1_CC1110       250000     // bps. NOTE: If you alter these
#define DATA_RATE_2_CC1110        38400     // values you will also have to
#define DATA_RATE_3_CC1110         1200     // modify register settings in
                                            // radioConfigure() in
                                            // per_test_radio.c
#define NUMBER_OF_DATA_RATES_CC2510   3
#define DATA_RATE_1_CC2510       500000     // bps. NOTE: If you alter these
#define DATA_RATE_2_CC2510       250000     // values you will also have to
#define DATA_RATE_3_CC2510        10000     // modify register settings in
                                            // radioConfigure() in
                                            // per_test_radio.c

// Preset burst size alternatives, i.e. number of packets to transmit
#define NUMBER_OF_BURST_SIZES         4     // Please update LCD text below
#define BURST_SIZE_1            1000000     // accordingly.
#define BURST_SIZE_2             100000
#define BURST_SIZE_3              10000
#define BURST_SIZE_4               1000

/*==== MACROS ================================================================*/
/*==== TYPES =================================================================*/
/*==== GLOBALS ================================================================*/
/*====  GLOBAL VARS ==========================================================*/

// This is the text displayed on the LCD for each menu option and their mapping
// to the defined values

// FREQUENCY MENUS
static const char __code pFreqMenuTop[] = "Frequency?";
static const UINT32 freqValuesCC1110[NUMBER_OF_FREQUENCIES_CC1110] =
    { FREQUENCY_1_CC1110,
      FREQUENCY_2_CC1110,
      FREQUENCY_3_CC1110,
      FREQUENCY_4_CC1110 };
static const char *freqMenuCC1110[NUMBER_OF_FREQUENCIES_CC1110] =
    { "915 MHz",  // LCD
      "903 MHz",  // menu
      "868 MHz",  // text
      "433 MHz" };
static const UINT32 freqValuesCC2510[NUMBER_OF_FREQUENCIES_CC2510] =
    { FREQUENCY_1_CC2510,
      FREQUENCY_2_CC2510,
      FREQUENCY_3_CC2510,
      FREQUENCY_4_CC2510 };
static const char *freqMenuCC2510[NUMBER_OF_FREQUENCIES_CC2510] =
    { "2480 MHz", // LCD
      "2460 MHz", // menu
      "2440 MHz", // text
      "2420 MHz"};

// DATA RATE MENUS
static const char __code pDataRateMenuTop[] = "Data rate?";
static const UINT32 dataRateValuesCC1110[NUMBER_OF_DATA_RATES_CC1110] =
    { DATA_RATE_1_CC1110,
      DATA_RATE_2_CC1110,
      DATA_RATE_3_CC1110 };
static const char *dataRateMenuCC1110[NUMBER_OF_DATA_RATES_CC1110] =
    { "250.0 kbps", // LCD
      " 38.4 kbps", // menu
      "  1.2 kbps"};// text
static const UINT32 dataRateValuesCC2510[NUMBER_OF_DATA_RATES_CC2510] =
    { DATA_RATE_1_CC2510,
      DATA_RATE_2_CC2510,
      DATA_RATE_3_CC2510};
static const char *dataRateMenuCC2510[NUMBER_OF_DATA_RATES_CC2510] =
    { "500.0 kbps", // LCD
      "250.0 kbps", // menu
      " 10.0 kbps"};// text

// BURST SIZE MENU
static const char __code pBurstSizeMenuTop[] = "Burst size?";
static UINT32 burstSizeValues[NUMBER_OF_BURST_SIZES] =
    { BURST_SIZE_1,
      BURST_SIZE_2,
      BURST_SIZE_3,
      BURST_SIZE_4 };
static const char *burstSizeMenu[NUMBER_OF_BURST_SIZES] =
    { "1000000 pkts", // LCD
      " 100000 pkts", // menu
      "  10000 pkts", // text
      "   1000 pkts"};

// MODE MENU
static const char __code pModeMenuTop[] = "Operating mode?";
static const char *modeMenu[NUMBER_OF_MODES] = {"Transmitter", "Receiver"};
static BYTE modeValues[NUMBER_OF_MODES] = { RADIO_MODE_TX, RADIO_MODE_RX };

// Text warnings in case of unsupported (too old or too new) chip version
static const char __code pVersionOldTop[] = "OLD CHIP VERS'N";
static char *versionOldMenu[] = {  "Chip version is", "too old to be  ",
                                   "supported by   ", "this software. ",
                                   "Please contact ", "your local     ",
                                   "TI sales office", "to get newer   ",
                                   "chips, or go to", "ti.com/support"};
static const char __code pVersionNewTop[] = "OLD SW VERSION";
static char *versionNewMenu[] = {  "Chip version is", "too new and not",
                                   "supported by   ", "this software. ",
                                   "Please download", "a newer version",
                                   "of this        ", "software from  ",
                                   "ti.com/lpw     "};


enum {
    FRSKY_BIND        = 0,
    FRSKY_BIND_DONE  = 1000,
    FRSKY_DATA1,
    FRSKY_DATA2,
    FRSKY_DATA3,
    FRSKY_DATA4,
    FRSKY_DATA5,
};

// Other global variables
extern BYTE radioPktBuffer[PACKET_LENGTH + 3];  // Buffer for packets to send or receive,
                                                // sized to match the receiver's needs


static const char __code blinkCursor[2] = {'-', '*'};  // Blinking cursor symbols,
                                                       // to indicate link at receiver
static UINT8 blinkCursorIdx = 0;            // Blink counter. Bit5 toggles symbol on LCD.
static BOOL updateLcd = FALSE;              // Controls when to update the LCD

static BOOL pktSentFlag = FALSE;            // Flag set whenever a packet is sent
static BOOL pktRcvdFlag = FALSE;            // Flag set whenever a packet is received
static BYTE mode;                           // Radio operating mode, either RX or TX

static DMA_DESC dmaConfig;                  // Struct for the DMA configuration

// Receiver variables for PER and RSSI statistics
extern UINT32 perRcvdSeqNum;            // The sequence number of the last
                                        // received packet
extern UINT32 perExpectedSeqNum;        // The expected sequence number of the
                                        // next packet
extern UINT32 perBadPkts;               // The total number of packets received
                                        // with correct ID, but wrong CRC
extern UINT32 perRcvdPkts;              // The total number of received packets
                                        // with correct ID, regardless of CRC
static INT16 perRssiOffset;             // RSSI offset for receiver, depends on
                                        // chip model and data rate
static INT16 perRssiBuf[RSSI_AVG_WINDOW_SIZE] = {0};    // Ring buffer for RSSI
                                        // values used for (sliding window)
                                        // averaging
static UINT8 perRssiBufCounter;         // Counter to keep track of the oldest/
                                        // newest byte in RSSI ring buffer
static INT16 perRssiSum;                // Sum of all RSSI values in buffer,
                                        // as absolute RSSI value

/*==== FUNCTIONS =============================================================*/

/* See per_test_dma.c for description */
extern void dmaRadioSetup(UINT8 mode);

/* See per_test_radio.c for description */
extern void radioConfigure(UINT32 dataRate, UINT32 frequency);
extern void pktSetSeqNum(UINT32 seqNum);
extern UINT32 pktGetSeqNum(void);
extern BOOL pktCheckId(void);
extern BOOL pktCheckCrc(void);
extern BOOL pktCheckValidity(void);
extern INT16 convertRssiByte(BYTE RSSI_value);

/* See per_test_menu.c for description */
extern UINT8 selectFromMenu(const char **ppMenuList, UINT8 numOptions,
                            UINT8 selectedOption);
extern UINT32 selectRadioFrequency(void);
extern UINT32 selectDataRate(void);
extern BYTE selectMode(void);
extern UINT32 selectBurstSize(void);
extern void lcdWritePer(void);
extern void lcdWriteRssi(INT16 RSSIsum);
extern void lcdWriteSeqNum(UINT32 seqNum);
extern void checkChipVersion(void);


extern void set_servo_rssi(void);

#endif  /* PER_TEST_MAIN_H */

/*==== END OF FILE ==========================================================*/
