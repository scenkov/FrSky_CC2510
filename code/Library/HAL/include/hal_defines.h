/*-----------------------------------------------------------------------------
|   File:      hal_defines.h
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
| Purpose:    Header file for type defines used by HAL
+------------------------------------------------------------------------------
| Decription: This file carries all the HAL related type defines.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/

#ifndef HAL_DEFINES_H
#define HAL_DEFINES_H

/*==== INCLUDES ==============================================================*/
/*==== CONSTS ================================================================*/

/* Common values */
/* ============= */
#ifndef FALSE
   #define FALSE 0
#endif

#ifndef TRUE
   #define TRUE 1
#endif

#ifndef NULL
   #define NULL 0
#endif

#ifndef HIGH
   #define HIGH 1
#endif

#ifndef LOW
   #define LOW 0
#endif


// Pin
#define PIN7     0x80
#define PIN6     0x40
#define PIN5     0x20
#define PIN4     0x10
#define PIN3     0x08
#define PIN2     0x04
#define PIN1     0x02
#define PIN0     0x01


/******************************************************************************
*******************              Chip revisions             *******************
******************************************************************************/
#define REV_A   0x00
#define REV_B   0x01
#define REV_C   0x02
#define REV_D   0x03
#define REV_E   0x04

/******************************************************************************
*******************              Specific values            *******************
******************************************************************************/
#define NBR_OF_INTERRUPTS 18

/*==== TYPES =================================================================*/

/* Common types */
/* ============= */

/* Boolean */
typedef unsigned char       BOOL;

/* Data */
typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef unsigned long       DWORD;

/* Unsigned numbers */
typedef unsigned char       UINT8;
typedef unsigned short      UINT16;
typedef unsigned long       UINT32;

/* Signed numbers */
typedef signed char         INT8;
typedef signed short        INT16;
typedef signed long         INT32;


/*==== EXPORTS ===============================================================*/


/*==== MACROS ===============================================================*/

/******************************************************************************
*******************        Bit, byte and word macros        *******************
******************************************************************************/

// Bit mask
#define BM( b )       ( 0x01 << ( b ))

#define HIBYTE(a)     (BYTE) ((WORD)(a) >> 8 )
#define LOBYTE(a)     (BYTE)  (WORD)(a)

#define SET_WORD(regH, regL, word) \
   do{                             \
      (regH) = HIBYTE( word );     \
      (regL) = LOBYTE( word );     \
   }while(0)

// Macro to read a word out
// Must not be used for all registers as e.g. Timer1 and Timers2 require that regL is read first
#define GET_WORD(regH, regL, word) \
   do{                             \
      word = (WORD)regH << 8;      \
      word |= regL;                \
   }while(0)


#endif /* HAL_DEFINES_H */

/*==== END OF FILE ==========================================================*/










