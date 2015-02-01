/*-----------------------------------------------------------------------------
|   File:      hal_io_mgmt.h
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
| Purpose:    I/O management
+------------------------------------------------------------------------------
| Decription: Macros for simplifying access to I/O pin setup and usage
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/

#ifndef HAL_IO_MGMT_H
#define HAL_IO_MGMT_H

/*==== INCLUDES ==============================================================*/

/*==== CONSTS ================================================================*/

/*==== TYPES =================================================================*/
/*==== EXPORTS ===============================================================*/

/*==== MACROS=================================================================*/

/******************************************************************************
  Macros for simplifying access to I/O pin setup and usage.
******************************************************************************/

// Macros for configuring IO direction:
// Example usage:
//   IO_DIR_PORT_PIN(0, 3, IO_IN);    // Set P0_3 to input
//   IO_DIR_PORT_PIN(2, 1, IO_OUT);   // Set P2_1 to output

#define IO_DIR_PORT_PIN(port, pin, dir)  \
   do {                                  \
      if (dir == IO_OUT)                 \
         P##port##DIR |= BM( pin );      \
      else                               \
         P##port##DIR &= ~BM( pin );     \
   }while(0)

// Where port={0,1,2}, pin={0,..,7} and dir is one of:
#define IO_IN   0
#define IO_OUT  1

// Macros for configuring IO input mode:
// Example usage:
//   IO_IMODE_PORT_PIN(0, 0, IO_IMODE_PUD);
//   IO_IMODE_PORT_PIN(2, 0, IO_IMODE_TRI);
//   IO_IMODE_PORT_PIN(1, 3, IO_IMODE_PUD);

#define IO_IMODE_PORT_PIN(port, pin, imode) \
   do {                                     \
      if (imode == IO_IMODE_TRI)            \
         P##port##INP |= BM( pin );         \
      else                                  \
         P##port##INP &= ~BM( pin );        \
   } while (0)

// where imode is one of:
#define IO_IMODE_PUD  0 // Pull-up/pull-down
#define IO_IMODE_TRI  1 // Tristate

// Macro for configuring IO drive mode:
// Example usage:
//   IIO_PUD_PORT(0, IO_PULLUP);
//   IIO_PUD_PORT(1, IO_PULLDOWN);
//   IIO_PUD_PORT(2, IO_PULLUP);

#define IO_PUD_PORT(port, pud)        \
   do {                               \
      if (pud == IO_PULLDOWN)         \
         P2INP |= BM( port + 5 );     \
      else                            \
         P2INP &= ~BM( port + 5 );    \
   } while (0)

#define IO_PULLUP          0
#define IO_PULLDOWN        1

// Macros for function select (General purpose I/O / Peripheral function):
// Example usage:
//   IO_FUNC_PORT0_PIN0(0, 0, IO_FUNC_PERIPH);
//   IO_FUNC_PORT0_PIN1(0, 1, IO_FUNC_GIO);
//   IO_FUNC_PORT2_PIN3(2, 3, IO_FUNC_PERIPH);

#define IO_FUNC_PORT_PIN(port, pin, func)  \
   do {                                    \
      if((port == 2) && (pin == 3)){       \
         if (func) {                       \
            P2SEL |= 0x02;                 \
         } else {                          \
            P2SEL &= ~0x02;                \
         }                                 \
      }                                    \
      else if((port == 2) && (pin == 4)){  \
         if (func) {                       \
            P2SEL |= 0x04;                 \
         } else {                          \
            P2SEL &= ~0x04;                \
         }                                 \
      }                                    \
      else{                                \
         if (func) {                       \
            P##port##SEL |= BM( pin );     \
         } else {                          \
            P##port##SEL &= ~BM( pin );    \
        }                                  \
      }                                    \
   } while (0)

// where func is one of:
#define IO_FUNC_GIO     0 // General purpose I/O
#define IO_FUNC_PERIPH  1 // Peripheral function

// Macros for configuring the ADC input:
// Example usage:
//   IO_ADC_PORT0_PIN(0, IO_ADC_EN);
//   IO_ADC_PORT0_PIN(4, IO_ADC_DIS);
//   IO_ADC_PORT0_PIN(6, IO_ADC_EN);

#define IO_ADC_PORT0_PIN(pin, adcEn)  \
  do {                                \
    if (adcEn)                        \
      ADCCFG |= BM( pin );            \
    else                              \
      ADCCFG &= ~BM( pin );           \
  }while (0)

// where adcEn is one of:
#define IO_ADC_EN           1 // ADC input enabled
#define IO_ADC_DIS          0 // ADC input disab

/*==== FUNCTIONS =============================================================*/

#endif /* HAL_IO_MGMT_H */

/*==== END OF FILE ==========================================================*/
