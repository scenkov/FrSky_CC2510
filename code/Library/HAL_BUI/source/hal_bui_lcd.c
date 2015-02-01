/*-----------------------------------------------------------------------------
|   File:      hal_bui_lcd.c
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
| Purpose:    LCD control for use with the SmartRF04EB.
+------------------------------------------------------------------------------
| Decription: Function implementations for common LCD functions
|               for use with the SmartRF04EB.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/

/*==== INCLUDES ==============================================================*/

#include "hal_main.h"

/*==== MACROS ================================================================*/

#define DATA_HIGH()    do{IO_DIR_PORT_PIN(1, 2, IO_IN); } while(0)
#define DATA_LOW()     do{IO_DIR_PORT_PIN(1, 2, IO_OUT); LCD_SDA = 0; }while(0)

/*==== CONSTS ================================================================*/

/* SM-Bus address of the LCD controller */
#define LCD_ADDR        0x76

/*  RS selects the register to be accessed for read and write
    RS = 0 => selects instruction register for write and busy flag
    RS = 1 => selects the data register for both read and write */
#define RS_0            0x00
#define RS_1            0x40

/*==== TYPES =================================================================*/
/*==== LOCALS ================================================================*/
/*==== PRIVATE FUNCTIONS =====================================================*/

/* Prototypes - for details of the private functions see implementation below */
static void smbSend(BYTE *buffer, const UINT8 n);
static void smbStart( void );
static void smbStop( void );
static void smbClock( BOOL value );
static void smbWrite( BOOL value );
static BOOL smbSendByte( BYTE b );
static BYTE lcdConvertChar(BYTE c);
static void waitLCD( void );

/*==== PUBLIC FUNCTIONS ======================================================*/

/******************************************************************************
* See hal_bui_lcd.h for a description of this function.
******************************************************************************/
void halBuiInitLcd(void)
{
   BYTE sendBuffer[8];

   // Setting the ports as inputs.
   IO_DIR_PORT_PIN(1, 2, IO_IN);
   IO_DIR_PORT_PIN(2, 0, IO_IN);

   // Setting P2_3 and P2_4 for general IO operation.
   IO_FUNC_PORT_PIN(1, 2, IO_FUNC_GIO);
   IO_FUNC_PORT_PIN(2, 0, IO_FUNC_GIO);

   // Setting ports for pull-up.
   IO_IMODE_PORT_PIN(1, 2, IO_IMODE_PUD);
   IO_IMODE_PORT_PIN(2, 0, IO_IMODE_PUD);
   IO_PUD_PORT(1, IO_PULLUP);
   IO_PUD_PORT(2, IO_PULLUP);

   // Setting up the lcd initialisation data.
   sendBuffer[0] = LCD_ADDR;
   sendBuffer[1] = RS_0;      // Instruction Register
   sendBuffer[2] = 0x0C;      // Display control         D =  1:      Display On
   //                                                    C =  0:      Cursor Off
   //                                                    B =  0:      Cursor character blink off
   sendBuffer[3] = 0x21;      // Function set            H =  1:      Use extended instruction set
   sendBuffer[4] = 0xA0;      // Set DDRAM address       ADD = 0x20
   sendBuffer[5] = 0x07;      // Display configuration   P =  1:      Column data right to left
   //                                                    Q =  1:      Row data, bottom to top
   sendBuffer[6] = 0x34;      // Function set            DL=  0:      4 bits
   //                                                    M =  1:      2-line by 16 display
   //                                                    SL=  0:      MUX1:18
   //                                                    H =  0:      Use basic instruction set
   sendBuffer[7] = 0x01;      // Clearing display
   smbSend(sendBuffer, 8);

   halBuiLcdUpdate((char*)"", (char*)"");  //clear display
}

/******************************************************************************
* See hal_bui_lcd.h for a description of this function.
******************************************************************************/
void halBuiLcdUpdate(char *pLine1, char *pLine2)
{
   halBuiLcdUpdateLine(LINE1, pLine1);
   halBuiLcdUpdateLine(LINE2, pLine2);
   return;
}

/******************************************************************************
* See hal_bui_lcd.h for a description of this function.
******************************************************************************/
void halBuiLcdUpdateLine(UINT8 line, char *pLine)
{
   BYTE sendBuffer[50];
   UINT8 i, j;
   char c;

   i = 0;
   sendBuffer[i++] = LCD_ADDR;
   sendBuffer[i++] = RS_0;
   sendBuffer[i++] = ((line == LINE1) ?  LINE1_ADDR : LINE2_ADDR);
   smbSend(sendBuffer, i);

   i = j = 0;
   sendBuffer[i++] = LCD_ADDR;
   sendBuffer[i++] = RS_1;
   while( ( (c = pLine[j]) != '\0' ) && j < LINE_SIZE ){
      sendBuffer[i++] = lcdConvertChar(c);
      j++;
   }
   for ( ;j < LINE_SIZE; j++){
      sendBuffer[i++] = lcdConvertChar(' ');
   }
   smbSend(sendBuffer, i);
}

/******************************************************************************
* See hal_bui_lcd.h for a description of this function.
******************************************************************************/
void halBuiLcdUpdateChar(UINT8 line, UINT8 position, char c){
   halBuiLcdUpdateSymbol(line, position, lcdConvertChar(c));
}

/******************************************************************************
* See hal_bui_lcd.h for a description of this function.
******************************************************************************/
void halBuiLcdUpdateSymbol(UINT8 line, UINT8 position, char c){
   BYTE sendBuffer[3];
   UINT8 i;

   if(position > LINE_SIZE){
      position = LINE_SIZE - 1;
   }

   i = 0;
   sendBuffer[i++] = LCD_ADDR;
   sendBuffer[i++] = RS_0;
   sendBuffer[i++] = ((line == LINE1) ?  LINE1_ADDR : LINE2_ADDR) + position;
   smbSend(sendBuffer, i);

   i = 0;
   sendBuffer[i++] = LCD_ADDR;
   sendBuffer[i++] = RS_1;
   sendBuffer[i++] = c;
   smbSend(sendBuffer, i);
}

/******************************************************************************
* See hal_bui_lcd.h for a description of this function.
******************************************************************************/
void halBuiLcdInitNewSymbol(char* symbol, BYTE address){
   BYTE sendBuffer[10];

   //send control data
   sendBuffer[0] = LCD_ADDR;
   sendBuffer[1] = RS_0;

   if (address < 0x40)
      sendBuffer[2] = 0x80;
   else
      sendBuffer[2] = 0xC0;

   sendBuffer[3] = 0x40 | (address & 0x3F);
   smbSend(sendBuffer, 4);

   //send data
   sendBuffer[0] = LCD_ADDR;
   sendBuffer[1] = RS_1;

   sendBuffer[2] = symbol[0];
   sendBuffer[3] = symbol[1];
   sendBuffer[4] = symbol[2];
   sendBuffer[5] = symbol[3];
   sendBuffer[6] = symbol[4];
   sendBuffer[7] = symbol[5];
   sendBuffer[8] = symbol[6];
   sendBuffer[9] = symbol[7];

   smbSend(sendBuffer, 10);
}

/*==== PRIVATE FUNCTIONS =====================================================*/

/*****************************************************************************
* Internal function for hal_bui_lcd.c
******************************************************************************/
static void smbSend(BYTE *buffer, const UINT8 n)
{
   UINT8 i = 0;

   smbStart();
   for(i = 0; i < n; i++){
      while(!smbSendByte(buffer[i])); //send until ACK received
   }
   smbStop();
}

/******************************************************************************
* Internal function for hal_bui_lcd.c
*
* This function initiates SMBus communication. It makes sure that both the
* data and the clock of the SMBus are high. Then the data pin is set low
* while the clock is kept high. This initializes SMBus transfer.
******************************************************************************/
static void smbStart()
{
   while (! (LCD_SDA && LCD_SCL) ); //wait for Data and clk high
   DATA_LOW();
   waitLCD();
   smbClock(0);
}

/******************************************************************************
* Internal function for hal_bui_lcd.c
*
* This function terminates SMBus communication. It makes sure that the data
* and clock of the SMBus are low and high, respectively. Then the data pin is
* set high while the clock is kept high. This terminates SMBus transfer.
******************************************************************************/
static void smbStop()
{
   while (! (!LCD_SDA && LCD_SCL));
   smbClock(0);
   DATA_HIGH();
   waitLCD();
   smbClock(1);
}

/******************************************************************************
* Internal function for hal_bui_lcd.c
*
* This function is used to clock the SMBus connected to the LCD. If a negative
* edge is to follow, the pin is set as an output and driven low. If a positive
* edge is to follow, the pin is set as an input and the pull-up resistor is
* to drive the node high. This way the slave device can hold the node low if
* a longer setup time is desired.
*
******************************************************************************/
static void smbClock(BOOL value)
{
   if(!value)
   {
      IO_DIR_PORT_PIN(2, 0, IO_OUT);
      LCD_SCL = 0;
   }
   else {
      IO_DIR_PORT_PIN(2, 0, IO_IN);
   }
   waitLCD();
}

/******************************************************************************
* Internal function for hal_bui_lcd.c
*
* Function for writing bit to the data line. Setting the port as input
* make the SMBus go high because of the pull-up resistors.
******************************************************************************/
static void smbWrite( BOOL value )
{
   smbClock(0);
   waitLCD();
   if(value){
      DATA_HIGH();
   }
   else{
      DATA_LOW();
   }
   smbClock(1);
   waitLCD();
}

/******************************************************************************
* Internal function for hal_bui_lcd.c
******************************************************************************/
static BOOL smbSendByte(BYTE b)
{
   UINT8 i;
   BOOL ack;
   for (i = 0; i < 8; i++){
      smbWrite(b & 0x80);
      b = (b <<  1);
   }
   smbClock(0);
   DATA_HIGH();
   smbClock(1);
   ack = !LCD_SDA;
   return ack; //high = ACK received, else ACK not received
}

/******************************************************************************
* Internal function for hal_bui_lcd.c
*
*  BYTE LcdConvertChar(BYTE c)
*
*  DESCRIPTION:
*      Converts ASCII characters to the character set the LCD display uses
*      (works for all digits and letters and some other common characters)
******************************************************************************/
static BYTE lcdConvertChar(BYTE c)
{
   //character set R in data sheet
   if ((c >= 'a') && (c <= 'z')) //lower case
      c += ('a' - 0xE1);//0x80;
   else if ((c >= 'A') && (c <= 'Z')) //upper case
      c+= ('A' - 0xC1);
   else if ((c >= ' ') && (c <= '#') || (c >= '%') && (c <= '?'))
      c += (' ' - 0xA0);
   else{
      switch (c){
      case '$':
         c = 0x82;
         break;
      case '§':
         c = 0xDF;
         break;
      case '£':
         c = 0x81;
         break;
      case '@':
         c = 0x80;
         break;
      case '[':
         c = 0x8A;
         break;
      case ']':
         c = 0x54;
         break;
      case '_':
         c = 0x5A;
         break;
      case 'æ':
         c = 0x9D;
         break;
      case 'ø':
         c = 0x8C;
         break;
      case 'å':
         c = 0x8F;
         break;
      case 'Æ':
         c = 0x9C;
         break;
      case 'Ø':
         c = 0x8B;
         break;
      case 'Å':
         c = 0x8E;
         break;
      case ARROW_LEFT:
         c = 0x10;
         break;
      case ARROW_RIGHT:
         c = 0x20;
         break;
      case ARROW_UP:
         c = 0x12;
         break;
      case ARROW_DOWN:
         c = 0x30;
         break;
      }
   }
   return c;
}

/******************************************************************************
* Internal function for hal_bui_lcd.c
*
* This function does the timing of clk
******************************************************************************/
static void waitLCD()
{
   UINT8 i = 0x01;
   while(i--);
}
/*==== END OF FILE ==========================================================*/
