#include "hal_main.h"
#include "per_test_main.h"

/***********************************************************************************
* CONSTANTS
*/

// Size of allocated UART RX/TX buffer (just an example)
#define SIZE_OF_UART_RX_BUFFER   200
#define SIZE_OF_UART_TX_BUFFER   SIZE_OF_UART_RX_BUFFER

// UART test characters
#define UART_TST_CHAR_1  0xA5
#define UART_TST_CHAR_2  0x26

// Test definitions
//#define UART_TST_MODE_RX
#define UART_TST_MODE_TX

// Baudrate = 57.6 kbps (U0BAUD.BAUD_M = 34, U0GCR.BAUD_E = 11)
#define UART_BAUD_M  34
//#define UART_BAUD_E  11 // 57600
#define UART_BAUD_E  12 //115200
//#define UART_BAUD_E  13 //230400
/***********************************************************************************
* LOCAL VARIABLES
*/

// Buffer+index for UART RX/TX
static UINT8 __xdata uartRxBuffer[SIZE_OF_UART_RX_BUFFER];
static UINT8 __xdata uartTxBuffer[SIZE_OF_UART_TX_BUFFER];
static UINT16 __xdata uartTxIndex;
static UINT16 __xdata uartRxIndex;

// Variable for buffer indexing
static UINT16 __xdata i;

// Variable for UART packet monitoring
static uint8 __xdata uartPktReceived = 1;


#define RXFIFO_ELEMENTS 50
#define RXFIFO_SIZE (RXFIFO_ELEMENTS - 1)
static __xdata UINT8 rxfifo[RXFIFO_SIZE];
static UINT8 rxfifo_in;
static UINT8 rxfifo_out;

void Init_UART();
void cons_putc(UINT8 ch);
void uart0Receive(UINT16* uartRxBuf, UINT16 uartRxBufLength);
void uart0Send(UINT16* uartTxBuf, UINT16 uartTxBufLength);
void cons_puts(const char *s);
char nibble_to_char(UINT8 nibble);
void cons_puthex8(UINT8 h);
void cons_puthex16(UINT16 h);
void cons_puthex32(UINT32 h);
void UartWriteRssi(UINT16 rssiSum);
void UartWritePer(void);