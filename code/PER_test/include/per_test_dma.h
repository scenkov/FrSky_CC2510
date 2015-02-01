/*-----------------------------------------------------------------------------
|   File:      per_test_dma.h
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
| Purpose:    DMA configuration and constants
+------------------------------------------------------------------------------
| Decription: This file contains settings and constants used to configure
|             the DMA.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/

#ifndef PER_TEST_DMA_H
#define PER_TEST_DMA_H


/*==== INCLUDES =============================================================*/
/*==== CONSTS ===============================================================*/

#define DMA_VLEN_USE_LEN                       0x00      // Use LEN for transfer count
#define DMA_VLEN_FIRST_BYTE_P_1                0x01      // Transfer the number of bytes specified by the first byte +1
#define DMA_VLEN_FIRST_BYTE                    0x02      // Transfer the number of bytes indicated by the first byte (itself included)
#define DMA_VLEN_FIRST_BYTE_P_2                0x03      // Transfer the number of bytes specified by the first byte +2
#define DMA_VLEN_FIRST_BYTE_P_3                0x04      // Transfer the number of bytes specified by the first byte +3
#define DMA_LEN_MAX                            0xFF      // The maximum length is always decided by the first byte
#define DMA_WORDSIZE_BYTE                      0x00      // Transfer a byte at a time
#define DMA_WORDSIZE_WORD                      0x01      // Transfer a 16-bit word at a time
#define DMA_TMODE_SINGLE                       0x00      // Transfer a single byte/word after each DMA trigger
#define DMA_TMODE_BLOCK                        0x01      // Transfer block of data (length len) after each DMA trigger
#define DMA_TMODE_SINGLE_REPEATED              0x02      // Transfer single byte/word (after len transfers, rearm DMA)
#define DMA_TMODE_BLOCK_REPEATED               0x03      // Transfer block of data (after len transfers, rearm DMA)

#define DMA_TRIG_NONE           0   // No trigger, setting DMAREQ.DMAREQx bit starts transfer
#define DMA_TRIG_PREV           1   // DMA channel is triggered by completion of previous channel
#define DMA_TRIG_T1_CH0         2   // Timer 1, compare, channel 0
#define DMA_TRIG_T1_CH1         3   // Timer 1, compare, channel 1
#define DMA_TRIG_T1_CH2         4   // Timer 1, compare, channel 2
#define DMA_TRIG_T2_COMP        5   // Timer 2, compare
#define DMA_TRIG_T2_OVFL        6   // Timer 2, overflow
#define DMA_TRIG_T3_CH0         7   // Timer 3, compare, channel 0
#define DMA_TRIG_T3_CH1         8   // Timer 3, compare, channel 1
#define DMA_TRIG_T4_CH0         9   // Timer 4, compare, channel 0
#define DMA_TRIG_T4_CH1        10   // Timer 4, compare, channel 1
#define DMA_TRIG_ST            11   // Sleep Timer compare
#define DMA_TRIG_IOC_0         12   // Port 0 I/O pin input transition
#define DMA_TRIG_IOC_1         13   // Port 1 I/O pin input transition
#define DMA_TRIG_URX0          14   // USART0 RX complete
#define DMA_TRIG_UTX0          15   // USART0 TX complete
#define DMA_TRIG_URX1          16   // USART1 RX complete
#define DMA_TRIG_UTX1          17   // USART1 TX complete
#define DMA_TRIG_FLASH         18   // Flash data write complete
#define DMA_TRIG_RADIO         19   // RF packet byte received/transmit
#define DMA_TRIG_ADC_CHALL     20   // ADC end of a conversion in a sequence, sample ready
#define DMA_TRIG_ADC_CH0       21   // ADC end of conversion channel 0 in sequence, sample ready
#define DMA_TRIG_ADC_CH1       22   // ADC end of conversion channel 1 in sequence, sample ready
#define DMA_TRIG_ADC_CH2       23   // ADC end of conversion channel 2 in sequence, sample ready
#define DMA_TRIG_ADC_CH3       24   // ADC end of conversion channel 3 in sequence, sample ready
#define DMA_TRIG_ADC_CH4       25   // ADC end of conversion channel 4 in sequence, sample ready
#define DMA_TRIG_ADC_CH5       26   // ADC end of conversion channel 5 in sequence, sample ready
#define DMA_TRIG_ADC_CH6       27   // ADC end of conversion channel 6 in sequence, sample ready
#define DMA_TRIG_ADC_CH7       28   // ADC end of conversion channel 7 in sequence, sample ready
#define DMA_TRIG_ENC_DW        29   // AES encryption processor requests download input data
#define DMA_TRIG_ENC_UP        30   // AES encryption processor requests upload output data

#define DMA_SRCINC_0                           0x00      // Increment source pointer by 0 bytes/words after each transfer
#define DMA_SRCINC_1                           0x01      // Increment source pointer by 1 bytes/words after each transfer
#define DMA_SRCINC_2                           0x02      // Increment source pointer by 2 bytes/words after each transfer
#define DMA_SRCINC_M1                          0x03      // Decrement source pointer by 1 bytes/words after each transfer

#define DMA_DESTINC_0                          0x00      // Increment destination pointer by 0 bytes/words after each transfer
#define DMA_DESTINC_1                          0x01      // Increment destination pointer by 1 bytes/words after each transfer
#define DMA_DESTINC_2                          0x02      // Increment destination pointer by 2 bytes/words after each transfer
#define DMA_DESTINC_M1                         0x03      // Decrement destination pointer by 1 bytes/words after each transfer

#define DMA_IRQMASK_DISABLE                    0x00      // Disable interrupt generation
#define DMA_IRQMASK_ENABLE                     0x01      // Enable interrupt generation upon DMA channel done

#define DMA_M8_USE_8_BITS                      0x00      // Use all 8 bits for transfer count
#define DMA_M8_USE_7_BITS                      0x01      // Use 7 LSB for transfer count

#define DMA_PRI_LOW                            0x00      // Low, CPU has priority
#define DMA_PRI_GUARANTEED                     0x01      // Guaranteed, DMA at least every second try
#define DMA_PRI_HIGH                           0x02      // High, DMA has priority
#define DMA_PRI_ABSOLUTE                       0x03      // Highest, DMA has priority. Reserved for DMA port access.


/*==== TYPES =================================================================*/

// This makes sure the bitfields members go from the most to least significant bit
#pragma bitfields=reversed

// DMA configuration data structure with the correct bit lengths for each value
typedef struct {
   BYTE SRCADDRH;
   BYTE SRCADDRL;
   BYTE DESTADDRH;
   BYTE DESTADDRL;
   BYTE VLEN      : 3;
   BYTE LENH      : 5;
   BYTE LENL      : 8;
   BYTE WORDSIZE  : 1;
   BYTE TMODE     : 2;
   BYTE TRIG      : 5;
   BYTE SRCINC    : 2;
   BYTE DESTINC   : 2;
   BYTE IRQMASK   : 1;
   BYTE M8        : 1;
   BYTE PRIORITY  : 2;
} DMA_DESC;

// There is no longer need for the bitfields to be reversed, so they are set back to default here.
#pragma bitfields=default

/*==== EXPORTS ===============================================================*/

#endif  /* PER_TEST_DMA_H */

/*==== END OF FILE ===========================================================*/
