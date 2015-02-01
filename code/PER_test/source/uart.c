
#include <stdio.h>
#include "uart.h"
#include "hal_main.h"
#include "per_test_main.h"


/***************************************************************************
   * Setup I/O ports
   *
   * Port and pins used by USART0 operating in UART-mode are
   * RX     : P0_2
   * TX     : P0_3
   * CT/CTS : P0_4
   * RT/RTS : P0_5
   *
   * These pins can be set to function as peripheral I/O to be be used by UART0.
   * The TX pin on the transmitter must be connected to the RX pin on the receiver.
   * If enabling hardware flow control (U0UCR.FLOW = 1) the CT/CTS (Clear-To-Send)
   * on the transmitter must be connected to the RS/RTS (Ready-To-Send) pin on the
   * receiver.
   */

void Init_UART(){
   // Configure USART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0)
  // To avoid potential I/O conflict with USART1:
  // configure USART1 for Alternative 2 => Port P1 (PERCFG.U1CFG = 1) 
    PERCFG = (PERCFG & ~PERCFG_U0CFG) | PERCFG_U1CFG;
 //   PERCFG |= PERCFG_U1CFG;//UART1 Alt2 loc. RX = P1_7  TX = P1_6

  // Configure relevant Port P0 pins for peripheral function:
  // P0SEL.SELP0_2/3/4/5 = 1 => RX = P0_2, TX = P0_3, CT = P0_4, RT = P0_5
 // P0SEL |= BIT5 | BIT4 | BIT3 | BIT2;
  P0SEL |= (1<<3) | (1<<2);
  
  // Initialise bitrate = 57.6 kbps (U0BAUD.BAUD_M = 34, U0GCR.BAUD_E = 11)
  U0BAUD = UART_BAUD_M;
  U0GCR = (U0GCR&~U0GCR_BAUD_E) | UART_BAUD_E;



  // Initialise UART protocol (start/stop bit, data bits, parity, etc.):

  // USART mode = UART (U0CSR.MODE = 1)
  U0CSR |= U0CSR_MODE;

  // Start bit level = low => Idle level = high  (U0UCR.START = 0)
  U0UCR &= ~U0UCR_START;

  // Stop bit level = high (U0UCR.STOP = 1)
  U0UCR |= U0UCR_STOP;

  // Number of stop bits = 1 (U0UCR.SPB = 0)
  U0UCR &= ~U0UCR_SPB;

  // Parity = disabled (U0UCR.PARITY = 0)
  U0UCR &= ~U0UCR_PARITY;

  // 9-bit data enable = 8 bits transfer (U0UCR.BIT9 = 0)
  U0UCR &= ~U0UCR_BIT9;

  // Level of bit 9 = 0 (U0UCR.D9 = 0), used when U0UCR.BIT9 = 1
  // Level of bit 9 = 1 (U0UCR.D9 = 1), used when U0UCR.BIT9 = 1
  // Parity = Even (U0UCR.D9 = 0), used when U0UCR.PARITY = 1
  // Parity = Odd (U0UCR.D9 = 1), used when U0UCR.PARITY = 1
  U0UCR &= ~U0UCR_D9;

  // Flow control = disabled (U0UCR.FLOW = 0)
  U0UCR &= ~U0UCR_FLOW;

  // Bit order = LSB first (U0GCR.ORDER = 0)
  U0GCR &= ~U0GCR_ORDER;

}


void cons_putc(UINT8 ch)
{
    // Clear any pending TX interrupt request (set U0CSR.TX_BYTE = 0)
  U0CSR &= ~U0CSR_TX_BYTE;
  
    U0DBUF = ch;
    while(!(U0CSR & U0CSR_TX_BYTE)); // wait for byte to be transmitted
    U0CSR &= ~U0CSR_TX_BYTE;         // Clear transmit byte status
}

void cons_puts(const char *s)
{
	while(0 != *s)
		cons_putc((UINT8)(*s++));
}


char nibble_to_char(UINT8 nibble)
{
	if (nibble < 0xA)
		return nibble + '0';
	return nibble - 0xA + 'A';
}

void cons_puthex8(UINT8 h)
{
	cons_putc(nibble_to_char((h & 0xF0)>>4));
	cons_putc(nibble_to_char(h & 0x0F));
}

void cons_puthex16(UINT16 h)
{
	cons_putc(nibble_to_char((h & 0xF000)>>12));
	cons_putc(nibble_to_char((h & 0x0F00)>>8));
	cons_putc(nibble_to_char((h & 0x00F0)>>4));
	cons_putc(nibble_to_char(h & 0x000F));
}

void cons_puthex32(UINT32 h)
{
        cons_putc(nibble_to_char((h & 0xF000)>>28));
        cons_putc(nibble_to_char((h & 0xF000)>>24));
        cons_putc(nibble_to_char((h & 0xF000)>>20));
        cons_putc(nibble_to_char((h & 0xF000)>>16));
	cons_putc(nibble_to_char((h & 0xF000)>>12));
	cons_putc(nibble_to_char((h & 0x0F00)>>8));
	cons_putc(nibble_to_char((h & 0x00F0)>>4));
	cons_putc(nibble_to_char(h & 0x000F));
}

//void uart0_isr(void) 
#pragma vector=UTX0_VECTOR
__interrupt void URX0_IRQ(void)
{
    URX0IF = 0;

// HACK we know the buffer is big enough, as client is waiting for our ACK
//    if(rxfifo_in != (( rxfifo_out - 1 + RXFIFO_SIZE) % RXFIFO_SIZE)) // not full
    {
        rxfifo[rxfifo_in] = U0DBUF;
        if (rxfifo_in + 1 == RXFIFO_SIZE)
            rxfifo_in = 0;
        else
            rxfifo_in++;
    }
}

void UartWriteRssi(UINT16 rssiSum)
{
    char rssiText[5];       // Array to hold the RSSI mean in chars

    // Calculate the average RSSI and store in char array
    sprintf(rssiText, "%4d", (UINT16)(rssiSum/RSSI_AVG_WINDOW_SIZE));
/*
    halBuiLcdUpdateChar(LINE2, 6, rssiText[0]);
    halBuiLcdUpdateChar(LINE2, 7, rssiText[1]);
    halBuiLcdUpdateChar(LINE2, 8, rssiText[2]);
    halBuiLcdUpdateChar(LINE2, 9, rssiText[3]);
 */   
//    cons_puthex16(rssiSum/RSSI_AVG_WINDOW_SIZE); cons_putc(0x20);
    cons_puts(rssiText);// cons_putc(0x20);
   //   uart0Send((uint16*)rssiText, 4);
    return;
}

void UartWritePer(void)
{
    char perText[5];      // Array to hold the PER in chars

    // The PER in units per 1000
    UINT16 per = (((perExpectedSeqNum - 1) - perRcvdPkts + perBadPkts)*100)/
        (perExpectedSeqNum - 1);

    // Convert the PER to percent and store in char array
    sprintf(perText, "%4.1f", ((float)per)/1);
/*
    halBuiLcdUpdateChar(LINE1, 6, perText[0]);
    halBuiLcdUpdateChar(LINE1, 7, perText[1]);
    // We skip the decimal point
    halBuiLcdUpdateChar(LINE1, 9, perText[3]);
 */   
    cons_puts(perText);
    
    return;
}