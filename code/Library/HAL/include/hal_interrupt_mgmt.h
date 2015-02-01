/*-----------------------------------------------------------------------------
|   File:      hal_interrupt_mgmt.h
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
| Purpose:    Interrupt management
+------------------------------------------------------------------------------
| Decription:   Macros which simplify access to interrupt enables, interrupt
|               flags and interrupt priorities. Increases code legibility.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/

#ifndef HAL_INTERRUPT_MGMT_H
#define HAL_INTERRUPT_MGMT_H

/*==== INCLUDES ==============================================================*/

#include "hal_main.h"

/*==== CONSTS ================================================================*/

#define INT_ON   1
#define INT_OFF  0
#define INT_SET  1
#define INT_CLR  0

// Global interrupt enables (without taking care of the current EA value
#define INT_GLOBAL_ENABLE(on) EA = (!!on)

// Disabling the individual interrupts
#define DISABLE_ALL_INTERRUPTS() (IEN0 = IEN1 = IEN2 = 0x00)

// Pausing the global interrupt (save current state)
#define INT_ENTER_CRITICAL_SECTION(current_EA) \
   do {                                        \
      current_EA = EA;                         \
      EA = INT_OFF;                            \
   } while (0)

// Bringing global interrupt back to previous
// state (using value saved from INT_ENTER_CRITICAL_SECTION)
#define INT_LEAVE_CRITICAL_SECTION(previous_EA) EA = (!!previous_EA)


#if(chip == 2430 || chip == 2431)
#define INUM_RFERR 0
#endif
#if(chip == 1110 || chip == 2510 || chip == 2511)
#define INUM_RFTXRX 0
#endif
#define INUM_ADC   1
#define INUM_URX0  2
#define INUM_URX1  3
#define INUM_ENC   4
#define INUM_ST    5
#define INUM_P2INT 6
#define INUM_UTX0  7
#define INUM_DMA   8
#define INUM_T1    9
#define INUM_T2    10
#define INUM_T3    11
#define INUM_T4    12
#define INUM_P0INT 13
#define INUM_UTX1  14
#define INUM_P1INT 15
#define INUM_RF    16
#define INUM_WDT   17


/*==== TYPES =================================================================*/

/*==== EXPORTS ===============================================================*/

/*==== MACROS=================================================================*/

// Macro used together with the INUM_* constants
// to enable or disable certain interrupts.
// Example usage:
//   HAL_INT_ENABLE(INUM_RFERR, INT_ON);
//   HAL_INT_ENABLE(INUM_URX0, INT_OFF);
//   HAL_INT_ENABLE(INUM_T1, INT_ON);
//   HAL_INT_ENABLE(INUM_T2, INT_OFF);
#if(chip == 2430 || chip == 2431)
#define HAL_INT_ENABLE(inum, on)                                                \
   do {                                                                         \
      if      (inum==INUM_RFERR) { RFERRIE = on; }                              \
      else if (inum==INUM_ADC)   { ADCIE   = on; }                              \
      else if (inum==INUM_URX0)  { URX0IE  = on; }                              \
      else if (inum==INUM_URX1)  { URX1IE  = on; }                              \
      else if (inum==INUM_ENC)   { ENCIE   = on; }                              \
      else if (inum==INUM_ST)    { STIE    = on; }                              \
      else if (inum==INUM_P2INT) { (on) ? (IEN2 |= 0x02) : (IEN2 &= ~0x02); }   \
      else if (inum==INUM_UTX0)  { (on) ? (IEN2 |= 0x04) : (IEN2 &= ~0x04); }   \
      else if (inum==INUM_DMA)   { DMAIE   = on; }                              \
      else if (inum==INUM_T1)    { T1IE    = on; }                              \
      else if (inum==INUM_T2)    { T2IE    = on; }                              \
      else if (inum==INUM_T3)    { T3IE    = on; }                              \
      else if (inum==INUM_T4)    { T4IE    = on; }                              \
      else if (inum==INUM_P0INT) { P0IE    = on; }                              \
      else if (inum==INUM_UTX1)  { (on) ? (IEN2 |= 0x08) : (IEN2 &= ~0x08); }   \
      else if (inum==INUM_P1INT) { (on) ? (IEN2 |= 0x10) : (IEN2 &= ~0x10); }   \
      else if (inum==INUM_RF)    { (on) ? (IEN2 |= 0x01) : (IEN2 &= ~0x01); }   \
      else if (inum==INUM_WDT)   { (on) ? (IEN2 |= 0x20) : (IEN2 &= ~0x20); }   \
   } while (0)
#endif

#if(chip == 1110 || chip == 2510 || chip == 2511)
#define HAL_INT_ENABLE(inum, on)                                                \
   do {                                                                         \
      if      (inum==INUM_RFTXRX) { RFTXRXIE = on; }                            \
      else if (inum==INUM_ADC)    { ADCIE   = on;  }                            \
      else if (inum==INUM_URX0)   { URX0IE  = on;  }                            \
      else if (inum==INUM_URX1)   { URX1IE  = on;  }                            \
      else if (inum==INUM_ENC)    { ENCIE   = on;  }                            \
      else if (inum==INUM_ST)     { STIE    = on;  }                            \
      else if (inum==INUM_P2INT)  { (on) ? (IEN2 |= 0x02) : (IEN2 &= ~0x02); }  \
      else if (inum==INUM_UTX0)   { (on) ? (IEN2 |= 0x04) : (IEN2 &= ~0x04); }  \
      else if (inum==INUM_DMA)    { DMAIE   = on;  }                            \
      else if (inum==INUM_T1)     { T1IE    = on;  }                            \
      else if (inum==INUM_T2)     { T2IE    = on;  }                            \
      else if (inum==INUM_T3)     { T3IE    = on;  }                            \
      else if (inum==INUM_T4)     { T4IE    = on;  }                            \
      else if (inum==INUM_P0INT)  { P0IE    = on;  }                            \
      else if (inum==INUM_UTX1)   { (on) ? (IEN2 |= 0x08) : (IEN2 &= ~0x08); }  \
      else if (inum==INUM_P1INT)  { (on) ? (IEN2 |= 0x10) : (IEN2 &= ~0x10); }  \
      else if (inum==INUM_RF)     { (on) ? (IEN2 |= 0x01) : (IEN2 &= ~0x01); }  \
      else if (inum==INUM_WDT)    { (on) ? (IEN2 |= 0x20) : (IEN2 &= ~0x20); }  \
   } while (0)
#endif

/*==== FUNCTIONS =============================================================*/

#endif /* HAL_INTERRUPT_MGMT_H */

/*==== END OF FILE ==========================================================*/
