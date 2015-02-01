/*-----------------------------------------------------------------------------
|   File:      per_test_main.c
|   Target:    cc2510
|   Author:    ESY
|   Revised:   2014-04-18
|   Revision:  1.0
|   Project:   FrSky 8Ch Receiver 
+------------------------------------------------------------------------------
|  Copyright 2014 
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ===================================================*/
/*==== INCLUDES ==============================================================*/

#include "hal_main.h"
#include "per_test_main.h"
#include "uart.h"
#include "hal_i2c.h"
#include <stdio.h>

/* See per_test_dma.c for description */
extern void dmaRadioSetup(uint8_t mode);

/* See per_test_radio.c for description */
extern void frsky2way_init(int bind);
extern void radioConfigure(UINT32 dataRate, UINT32 frequency);
extern void pktSetSeqNum(UINT32 seqNum);
extern UINT32 pktGetSeqNum(void);
extern BOOL pktCheckId(void);
extern BOOL pktCheckCrc(void);
extern BOOL pktCheckValidity(void);
extern INT16 convertRssiByte(BYTE RSSI_value);

#define RFIF_IM_DONE	(1 << 4)

/* One whole page of flash memory is to be reserved, i.e., 1 KiB. */
#define PAGE_SIZE 1024
__no_init const char __code flashDataAddr[PAGE_SIZE] @ 0x7c00; // Page 31

#define DATA_AMOUNT 100
#define RAM_BUF_SIZE 50
/* Where in RAM the function to be run will reside. Must be big enough! */
static uint16_t __xdata ramFuncAddr[RAM_BUF_SIZE];
static uint8_t data[DATA_AMOUNT] = "Flash Controller";

#define BIT0              0x01
#define BIT1              0x02
#define BIT2              0x04
#define BIT3              0x08
#define BIT4              0x10
#define BIT5              0x20
#define BIT6              0x40
#define BIT7              0x80

#define RFIF_IM_DONE	(1 << 4)

#define ENABLE_INTERRUPTS { EA = 1; }
#define DISABLE_INTERRUPTS { EA = 0; }

#define PWM_TIMER_CORRECTION 1.230769f // =1/(26/32) (us)
#define PWM_20_MS (uint16_t)(20000 / PWM_TIMER_CORRECTION)
#define PWM__CORRECTION 0.8125f // =26/32 (us)

/*
  P1_1 --> CPPM OUTPUT
*/

#define Servo1 P1_0
#define INIT_Servo1()    do { Servo1 = 0; IO_DIR_PORT_PIN(1, 0, IO_OUT); P1SEL &= ~BIT0;} while (0)
#define Servo1_OUT_HIGH() (Servo1 = 1)
#define Servo1_OUT_LOW()  (Servo1 = 0)

#define Servo2 P1_1
#define INIT_Servo2()    do { Servo2 = 0; IO_DIR_PORT_PIN(1, 1, IO_OUT); P0SEL &= ~BIT1;} while (0)
#define Servo2_OUT_HIGH() (Servo2 = 1)
#define Servo2_OUT_LOW()  (Servo2 = 0)

#define Servo3 P1_2
#define INIT_Servo3()    do { Servo3 = 0; IO_DIR_PORT_PIN(1, 2, IO_OUT); P1SEL &= ~BIT2;} while (0)
#define Servo3_OUT_HIGH() (Servo3 = 1)
#define Servo3_OUT_LOW()  (Servo3 = 0)

#define Servo4 P1_3
#define INIT_Servo4()    do { Servo4 = 0; IO_DIR_PORT_PIN(1, 3, IO_OUT); P1SEL &= ~BIT3;} while (0)
#define INIT_Servo4_IN()    do { Servo4 = 0; IO_DIR_PORT_PIN(1, 3, IO_IN); P1SEL &= ~BIT3;} while (0)
#define Servo4_OUT_HIGH() (Servo4 = 1)
#define Servo4_OUT_LOW()  (Servo4 = 0)

#define Servo5 P1_4
#define INIT_Servo5()    do { Servo5 = 0; IO_DIR_PORT_PIN(1, 4, IO_OUT); P1SEL &= ~BIT4;} while (0)
#define Servo5_OUT_HIGH() (Servo5 = 1)
#define Servo5_OUT_LOW()  (Servo5 = 0)

#define Servo6 P1_5
#define INIT_Servo6()    do { Servo6 = 0; IO_DIR_PORT_PIN(1, 5, IO_OUT); P0SEL &= ~BIT5;} while (0)
#define Servo6_OUT_HIGH() (Servo6 = 1)
#define Servo6_OUT_LOW()  (Servo6 = 0)

#define Servo7 P1_6
#define INIT_Servo7()    do { Servo7 = 0; IO_DIR_PORT_PIN(1, 6, IO_OUT); P0SEL &= ~BIT6;} while (0)
#define Servo7_OUT_HIGH() (Servo7 = 1)
#define Servo7_OUT_LOW()  (Servo7 = 0)

#define Servo8 P1_7
#define INIT_Servo8()    do { Servo8 = 0; IO_DIR_PORT_PIN(1, 7, IO_OUT); P0SEL &= ~BIT7;} while (0)
#define Servo8_OUT_HIGH() (Servo8 = 1)
#define Servo8_OUT_LOW()  (Servo8 = 0)

//static uint16_t T1_COUNTER;
static uint16_t total_ppm_time;
uint16_t Servo_Position[8] = {1500,1500,1000,1500,1000,1000,1000,1000};
uint16_t pwm_reseived[8] = {1500,1500,1000,1500,1000,1000,1000,1000};
uint8_t Servo_Number = 9;
uint8_t tx_present = 0;


volatile uint8_t sync = FALSE;
volatile uint8_t __xdata rxPacketStatus;
uint8_t  missingPackets = 0;
uint8_t  failsafe_count = 0;
uint8_t  failsafe = 0;

uint16_t ms_count = 0;

uint16_t RSSI_dbm;
uint8_t attenuator = 0;
uint8_t rssi_atennuation_correction = 1;

void ListenforMaster(uint8_t timeout, uint8_t t3_multiplier);
void rfReceivePacket(uint8_t timeout, uint8_t t3_multiplier);
void Binding_RX(void);
void RX_TEST(void);
extern uint8_t chann_hoping;
extern uint8_t chann_counter;
extern uint8_t channel_map[50];
extern uint8_t calData3[50];
extern uint8_t calData1[50];
extern uint8_t tx_id[2];

uint8_t bind_tx_id[2];
uint8_t listLength;	   // Length of channel hop-list
uint8_t counter;

uint8_t receiver_mode = 0; // Parallel PPM

uint8_t listLength;	   // Length of channel hop-list

BOOL eol = false; 

uint8_t counter;

void servo_low(){
    Servo1_OUT_LOW();
    if (receiver_mode==0) // 0-PWM OUT / 1-RSSI OUT
      Servo2_OUT_LOW();
    Servo3_OUT_LOW();
    Servo4_OUT_LOW();
    Servo5_OUT_LOW();
    Servo6_OUT_LOW();
    Servo7_OUT_LOW();
    Servo8_OUT_LOW();
}

void Servo_Init(){
    INIT_Servo1();
    INIT_Servo2();
    INIT_Servo3();
    INIT_Servo4();
    INIT_Servo5();
    INIT_Servo6();
    INIT_Servo7();
    INIT_Servo8();
}

// check Receiver Mode (jumper servo3/servo4)
// 0-PWM OUT / 1-CPPM OUT
void check_ppm_pwm(void){
  uint8_t receiver_mode_count = 0;
  
  INIT_Servo4_IN();
  
  Servo3_OUT_HIGH();
  if ( Servo4 == 1){
    receiver_mode_count++;
    Servo3_OUT_LOW();  
  }
  if ( Servo4 == 0){
    receiver_mode_count++;
    Servo3_OUT_HIGH();  
  }
  if ( Servo4 == 1){
    receiver_mode_count++;
    Servo3_OUT_LOW();  
  }
  
  if (receiver_mode_count >= 3)
    receiver_mode = 1; // CPPM - servo2 / RSSI - servo1
  else
    receiver_mode = 0; // PWM - servo1..servo8
  
  INIT_Servo4();
  Servo4_OUT_LOW();
}

/***********************************************************************************
* @fn          timer1_ISR
*/

#pragma vector = T1_VECTOR
__interrupt void timer1_ISR(void)
{
  unsigned int us;
    // Channel 1 interrupt
    if (T1CTL & T1CTL_CH2IF)
    {
        T1CTL = (~T1CTL_CH2IF & 0xF0) | (T1CTL & 0x0F);
    }

    // Overflow interrupt
    if (T1CTL & T1CTL_OVFIF)
    {
      T1CTL = (~T1CTL_OVFIF & 0xF0) | (T1CTL & 0x0F);
      servo_low();

      Servo_Number++; // jump to next servo
        if (Servo_Number > 8) // 8 back to the first servo 
          {
          total_ppm_time = 0; // clear the total servo ppm time
          Servo_Number=0;
          T1CNTL = 0; // reset the timer1 value for next
          }
        
          if (Servo_Number == 8)  // Check the servo number. 
            {
              //Servos accepting 50hz ppm signal, this is why we are waiting for 20ms before second signal brust. 
              us = PWM_20_MS - total_ppm_time; //wait for total 20ms loop.  waiting time = 20.000us - total servo times
            }
          else{
              us = Servo_Position[Servo_Number];
          }
        
          total_ppm_time += us; // calculate total servo signal times.
     
            switch (Servo_Number) {
              case 0:
                if (receiver_mode==0) // CPPM output
                Servo1_OUT_HIGH();
                break;
              case 1:
                if (receiver_mode==0) // RSSI output
                Servo2_OUT_HIGH();
                break;
              case 2:
                if (receiver_mode==0) // Parallel PPM Jumper CH3-CH4
                  Servo3_OUT_HIGH();
                break;
              case 3:
                if (receiver_mode==0) // Parallel PPM Jumper CH3-CH4
                  Servo4_OUT_HIGH();
                break;
              case 4:
                Servo5_OUT_HIGH();
                break;
              case 5:
                Servo6_OUT_HIGH();
                break;
              case 6:
                Servo7_OUT_HIGH();
                break;
              case 7:
                Servo8_OUT_HIGH();
                break;
 //             case 8:
 //               Servo9_OUT_HIGH();
 //               break;  
                }     

          T1CC0L = us & 0xff;
          T1CC0H = us >> 8;

    }
}

uint8_t RSSI_PWM_count = 0;
uint8_t RSSI_PWM_value = 10;
uint16_t time_count = 0;

#pragma vector = T4_VECTOR
__interrupt void t4_isr(void)
{
//  time_count++;
    /* Clears the module interrupt flag. */
    
    if (++RSSI_PWM_count >=90){
       RSSI_PWM_count = 0;
       Servo2_OUT_HIGH();
    }
    
    if (RSSI_PWM_count > RSSI_PWM_value){
       Servo2_OUT_LOW();
    }

    /* Clears the CPU interrupt flag. */
    T4OVFIF = 0;
    T4IF = 0;
}

/*******************************************************************************/

#define PWM_MAX 2100 // =2100ms * 0.309
#define PWM_MIN 950 // =950ms * 0.309

unsigned short random_rand(void);
void random_init(unsigned short seed);
extern void nextChannel(uint8_t skip);
extern void get_chan_num(uint8_t skip);


/*==== PUBLIC FUNCTIONS ======================================================*/

/*******************************************************************************
* LOCAL FUNCTIONS
*/
void halFlashStartErase(void); // Implemented in assembly
void halFlashStartWrite(void); // Implemented in assembly

/*******************************************************************************
* @fn          runFunctionFromRam
*
* @brief       Copies another function from flash to RAM and executes the
*              function. Does check whether the space provided in RAM is enough,
*              this must be done prior to the call.
*
* @param       void (*func)(void) - address of function to be run from RAM.
*              uint8_t __xdata *ramAddr - adress of function location in RAM.
*              uint16_t funcLength - size of buffer to place function in [bytes].
*
* @return      void
*******************************************************************************/
void runFunctionFromRam(void (*func)(void), uint16_t __xdata *ramAddr,
						uint16_t funcLength)
{
    /* flashFuncAddr is a pointer to where in flash the function is. */
    uint16_t __code *flashFuncAddr = (uint16_t __code *)(uint16_t)func;

    /* f is a function pointer to the address in RAM where the function will be
     * placed.
     */
    VFPTR f = (VFPTR)(uint16_t)ramAddr;

    /* Copy the function from flash to RAM. */
    uint16_t i;
    for (i = 0; i < funcLength; i++)
    {
        ramAddr[i] = flashFuncAddr[i];
    }

    /* Run function from RAM. */
    (*f)();

    return;
}

/*******************************************************************************
* @fn          flashWriter
*
* @brief       Writes contents of the array "data" to flash. Must be run from
*              RAM. Relies on eksternal variables. Setup of FCTL, FADDRH and
*              FADDRL msut be done prior to running this function.
*
* @param       void
*
* @return      void
*******************************************************************************/
void flashWriter(void)
{
    /* Disable interrupts. */
    EA = 0;

    /* Waiting for the flash controller to be ready. */
    while (FCTL & FCTL_BUSY);

    /* Setup of FCTL, FADDRH and FADDRL done in main().
     * Enabling flash write.
     */
    FCTL |= FCTL_WRITE;

    /* Write all the data to flash. */
    uint8_t i = 0;
    while (i < DATA_AMOUNT)
    {
        /* Write two bytes to FWDATA. Flash write starts after second byte is
         * written to the register. The first byte written to FWDATA is the LSB
         * of the 16-bit word.
         */
        FWDATA = data[i++];
        FWDATA = data[i++];

        /* Wait for the flash write to complete. */
        while (FCTL & FCTL_SWBSY);
    }

    /* If desired, interrupts may again be enabled. */

    return;
}
/*
void Tx_enable(void){
  P0_7 = 0; // Tx ENABLE = 0
  P0_6 = 1; // Rx ENABLE = 1
}

void Rx_enable(void){
  P0_7 = 1; // Tx ENABLE = 1
  P0_6 = 0; // Rx ENABLE = 0
}        
*/
void delay_5ms(void){
    T2CT = 180;        // Reset the Frame Timer
    while (T2CT > 0){  // Wait until it's time to listen for the Master
    }
}

void set_failsafe_servo(void){
//  cons_puts("\r\nF/S Load values : ");
  /*
  .....
  */
}

void setup_Tim1_ch2(){
   if (receiver_mode == 1){ // CPPM
        // Clear Timer 1 channel 1 and overflow interrupt flag
        // CPU interrupt flag (IRCON) for Timer 1 is cleared automatically by hardware.
        T1CTL &= ~T1CTL_CH2IF;
        T1CTL &= ~T1CTL_OVFIF;
    
        // Set individual interrupt enable bit in the peripherals SFR
        
        OVFIM = 1;                  // Enable overflow interrupt
        T1CCTL0 &= ~T1CCTL0_IM;     // Disable interrupt on channel 0
        T1CCTL2 &= ~T1CCTL1_IM;     // Disable interrupt on channel 1
        T1CCTL2 &= ~T1CCTL2_IM;     // Disable interrupt on channel 2
    
        PERCFG = PERCFG_T1CFG; // Tim1 Alternative 2 location CH1 to P1_1 (replace CH1 to P0_3) CH2 to P1_0
    
        /***************************************************************************
         * Setup peripheral I/O for Timer
         *
         * We can also choose the Alternative 2 location for Timer 1, or for the peripherals
         * that use the same pins as Timer 1. This can be done by setting PERCFG-register
         */

        // Select P1_0 for peripheral function PPM Output
        P1SEL |= BIT0;
    
        // Set P2DIR to prioritize Timer 1 channel 1's control over port 0 pins over USART 0
        // See p. 89 in datasheet for more details
        P2DIR = (P2DIR & ~P2DIR_PRIP0) | P2DIR_PRIP0_2;
    
        // Set channel 2 to compare mode and to toggle on compare.
        T1CCTL2 = (T1CCTL2 & ~T1CCTL2_CMP) | T1C2_SET_C2_CLR_C0 | T1CCTL2_MODE; // PPM 1_0
        
//        T1CCTL1 = (T1CCTL1 & ~T1CCTL1_CMP) | T1C1_SET_C1_CLR_C0 | T1CCTL1_MODE; // PPM 1_1
        // Set compare registers
        T1CC2L = 0x65;  // 357 440us
        T1CC2H = 0x01;
        
        // Set Timer 4 - RSSI PWM
        T4CC0 = 180;   // PWM signal period 5ms
        T4CTL = T4CTL_DIV_8 | T4CTL_START | T4CTL_CLR | T4CTL_OVFIM | T4CTL_MODE_MODULO;
        T4IE = 1;
   }     
    T1CC0L = 0x7a;
    T1CC0H = 0x3f;
    
    // Set prescalar divider value to 128 to get a tickspeed of 101.56 kHz and
    // set Timer 1 to modulo mode
    T1CTL = (T1CTL & ~(T1CTL_MODE | T1CTL_DIV)) | T1CTL_MODE_MODULO | T1CTL_DIV_32;
    // Timer 1 will now start counting...
    
    T1CNTL = 0; // reset the timer1 value for next
    T1IE = 1; 
}

uint8_t temp_count0 = 0;
uint16_t temp_count1 = 1;
uint16_t pause = 0;

uint8_t t3_count;
uint8_t multiplier;

uint8_t Binding = FALSE;

uint16_t state = FRSKY_DATA1;

/*==== PUBLIC FUNCTIONS ======================================================*/

/******************************************************************************
* @fn  main
*
* @brief
*      Main function. Triggers setup menus and main loops for both receiver
*      and transmitter. This function supports both CC1110 and CC2510.
*
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/
void main(void)
 {
  halPowerClkMgmtSetMainClkSrc(CRYSTAL);   
    
    // Initialize LED1
    INIT_LED1();
    INIT_LED3();
    
    
    LED1 = LED_ON;
  //  LED3 = LED_ON;

    P0SEL &= ~ BIT0;  // TEST PIN
    P0_0 = 1;
    P0DIR |= BIT0;
    
    P0SEL &= ~ BIT1;  // TEST PIN
    P0_1 = 0;
    P0DIR |=  BIT1;
 
    P2SEL &= ~ 0x01;  // TEST PIN
    P2_0 = 1;
    P2DIR |= 0x01;
    
    INIT_BUTTON();
    
     // Choose the crystal oscillator as the system clock
//    halPowerClkMgmtSetMainClkSrc(CRYSTAL);

//    Servo_Init();
    
//    check_ppm_pwm();
//    receiver_mode = 1; // CPPM
    // #define UART_BAUD_E  13 //230400   #define UART_BAUD_E  12 //115200  
    Init_UART();    

// Initilize Timer 2 (Frame Timer)
// Notes: Timer 2 consists of an 8 bit 'down' counter with a 18 bit prescaler
// The Timer tic period is T = T2PR * Val(T2CTL.TIP) clock cycles, where
// Val = 64 if T2CTL bits 1:0 = 0
// Val = 128 if T2CTL bits 1:0 = 1
// Val = 256 if T2CTL bits 1:0 = 2
// Val = 1024 if T2CTL bits 1:0 = 3

    T2CT = 0;         // Stop the Timer
    T2PR = 6;         // Timer Prescale Multipler
    T2CTL = 0x01;     // Set Tick period to 128*T2PR clock cycles (128*6/26.000 MHz = 29.538 usec)
                      // Interrupts Disabled
    
//   Initialize timer3 for use as RX timeout timer.
//   Note: Timer 3 is clocked by the 26.000 MHz XTAL divided by the 'Prescale Divider', defined
//   in bits 7:5 of T3CTL as 128. This sets the 'tic' period as 128/26 = 4.923076923 usec    
    T3CCTL0 = 0;
    T3CCTL1 = 0;
    T3CTL = T3CTL_DIV_128 | T3CTL_START | T3CTL_CLR | T3CTL_MODE_MODULO; //T3CTL_MODE_DOWN;
    // Enable global interrupt by setting the [IEN0.EA=1]
    EA = 1;    
   
    cons_puts("\r\nFrSky RX-8ch :");

    delay_5ms();  
    
    // Read from flash - TX_id , number of hoping chanels and hoping channels
    for (uint8_t i = 0; i <= 50; i++) {
      data[i] = flashDataAddr[i];
    }
    tx_id[0] = data[0]; // TX-id
    tx_id[1] = data[1];
    listLength = data[2];// number of hoping chanels
    
    if (listLength == 0xff || listLength > 0x50)
    {  
      cons_puts("\r\nHopping channels length error, Bind recuired : ");
       while (!halBuiButtonPushed()) { 
           TOGGLE_LED1();
           TOGGLE_LED3();
           for (uint8_t i = 0; i <= 20; i++) {
             delay_5ms();
           }
       }
       delay_5ms();
       Binding = FALSE;
    }else{
      Binding = TRUE;
      cons_puts("\r\nchannel_map : "); 
      for (uint8_t j = 0x00; j <= listLength; j++) {
        channel_map[j] = data[j+3];
        cons_puthex8(channel_map[j]);
        cons_putc(0x20); 
      }
    }
    
    cons_puts("\r\nTx_ID : "); cons_puthex8(tx_id[1]);cons_puthex8(tx_id[0]);
    cons_puts("\r\nNum.hopping channels "); cons_puthex8(data[2]);  
    cons_puts("\r\n"); cons_puts("\r\n"); 
    
  frsky2way_init(1);
  
    mode = RADIO_MODE_RX; //selectMode();

    if (mode == RADIO_MODE_TX) {
      // Tx code...
    }
    else if (mode == RADIO_MODE_RX) {
      
      rxfifo_in = rxfifo_out = 0;     

        // Set up the DMA to move packet data from radio to buffer
        dmaRadioSetup(RADIO_MODE_RX);

        // Configure interrupt for every received packet
        HAL_INT_ENABLE(INUM_RF, INT_ON);    // Enable RF general interrupt
        RFIM = IRQ_DONE;                    // Mask IRQ_DONE flag only
        INT_GLOBAL_ENABLE(INT_ON);          // Enable interrupts globally
        
        CHANNR = channel_map[0];
        RFST   = RFST_SIDLE;
        while(MARCSTATE != 0x01);
                 
        // Start receiving
        DMAARM = DMAARM_CHANNEL0;           // Arm DMA channel 0
        RFST   = STROBE_RX;                 // Switch radio to RX

        /*  Check button for Binding the receiver  */
        if (halBuiButtonPushed() || Binding == FALSE) // Binding button     
          {
            Binding_RX();
          }
        
        while (TRUE) {

//#if defined  BINDING_TEST 
//        RX_TEST();  
//#else     // Main Radio Loop
          
          rxPacketStatus = PKT_STATUS_UNKNOWN;
          
          if (state == FRSKY_DATA4){ // send telemetry
            /*
          P2_0 = 1;  
             halWait(10);
          P2_0 = 0;   
             
              RFST   = RFST_SIDLE;
              dmaRadioSetup(RADIO_MODE_TX);
              radioPktBuffer[0] = PACKET_LENGTH; // Length byte
              radioPktBuffer[1] = tx_id[0];      // Network identifier
              radioPktBuffer[2] = tx_id[1];          

//Get voltage A1 (52mv/count)
//Telemetry.p.frsky.volt[0] = (u32)pkt[3] * 52 / 100; //In 1/10 of Volts
//TELEMETRY_SetUpdated(TELEM_FRSKY_VOLT1);
//Get voltage A2 (~13.2mv/count) (Docs say 1/4 of A1)
//Telemetry.p.frsky.volt[1] = (u32)pkt[4] * 132 / 1000; //In 1/10 of Volts
//TELEMETRY_SetUpdated(TELEM_FRSKY_VOLT2);

              radioPktBuffer[3] = 12.4 * 1.923076923; // = volt * 100/52
              radioPktBuffer[4] = 3.86 * 7.575757576; // = volt * 1001/132
              radioPktBuffer[5] = 57;
              
              // Send the packet
              DMAARM |= DMAARM_CHANNEL0;  // Arm DMA channel 0
              RFST = STROBE_TX;           // Switch radio to TX
              while(!pktSentFlag); //{
              pktSentFlag = FALSE;
                
              state = FRSKY_DATA1;
              
              dmaRadioSetup(RADIO_MODE_RX);
              
              // Configure interrupt for every received packet
              HAL_INT_ENABLE(INUM_RF, INT_ON);    // Enable RF general interrupt
              RFIM = IRQ_DONE;                    // Mask IRQ_DONE flag only
              INT_GLOBAL_ENABLE(INT_ON);          // Enable interrupts globally
              RFST   = RFST_SIDLE;    
              nextChannel(1);
              RFST = STROBE_RX;
              */
            state = FRSKY_DATA1;
          }else{
                      multiplier = 162; // ~160

                        TIMIF &= ~T3OVFIF;            // Clear RX_TIMEOUT_TIMER_FLAG
                        T3CC0 = 10; //timeout;
                        T3CTL |= T3CTL_CLR;
                        
                        for (t3_count = 0; t3_count <= multiplier; t3_count++) {
                            while(!(TIMIF & T3OVFIF)){
                                if (pktRcvdFlag == TRUE) {  // If SYNC has been detected
                                  pktRcvdFlag = FALSE;
                                 // Packet received
                                  pktCheckValidity();
                                  
                                    if (rxPacketStatus == PKT_OK){
                                      temp_count0 ++;                                    
                                       counter = radioPktBuffer[3];
                                       cons_putc(0x20); cons_puthex8(chann_hoping);        
                                       failsafe = 0;
                                       missingPackets = 0;
                                       RFST   = RFST_SIDLE; 
                                       nextChannel(1);        
                                       set_servo_rssi();
                                       break;
                                    }                  
                                  }
                              }  // Wait for timeout
                              TIMIF &= ~T3OVFIF;     // Clear RX_TIMEOUT_TIMER_FLAG
                            if (rxPacketStatus == PKT_OK)
                               break;
                            }
                          if (t3_count > multiplier ){
                               missingPackets++;
                               RFST   = RFST_SIDLE;
                               if (missingPackets > MAX_MISSING_PKT){
                                    nextChannel(13); 
                                    cons_puts("F ");
                                  }else	
                                  {
                                    nextChannel(1);
                                 // failsave_count++;
                                  }                          
                        }
                            rxPacketStatus = TIMEOUT_ERROR;

                    if (counter % 4 == 2){       
                      // Send Telemetry 
                     }
                     RFST = STROBE_RX;               
          }
        } 
    }
}


/*==== PRIVATE FUNCTIONS =====================================================*/

void set_servo_rssi(void)
{
  uint16_t servo_PWM;
 // uint8_t check_hopping;
  
  if (tx_present == 0){
    tx_present = 1;
 //   set_tim1();
    setup_Tim1_ch2();
  }
    // Format Channel Packed 
    // Packet offset 0x06 - |1L|2L|3L|4L|2H,1H|4H,3H|5L|6L|7L|6H,5H|8L|8H,7H|  
  
    pwm_reseived[0] = (uint16_t)(((radioPktBuffer[10] & 0x0F)<<8 | radioPktBuffer[6]));
    pwm_reseived[1] = (uint16_t)(((radioPktBuffer[10] & 0xF0)<<4 | radioPktBuffer[7]));
    pwm_reseived[2] = (uint16_t)(((radioPktBuffer[11] & 0x0F)<<8 | radioPktBuffer[8]));
    pwm_reseived[3] = (uint16_t)(((radioPktBuffer[11] & 0xF0)<<4 | radioPktBuffer[9]));
    pwm_reseived[4] = (uint16_t)(((radioPktBuffer[16] & 0x0F)<<8 | radioPktBuffer[12]));
    pwm_reseived[5] = (uint16_t)(((radioPktBuffer[16] & 0xF0)<<4 | radioPktBuffer[13]));
    pwm_reseived[6] = (uint16_t)(((radioPktBuffer[17] & 0x0F)<<8 | radioPktBuffer[14]));
    pwm_reseived[7] = (uint16_t)(((radioPktBuffer[17] & 0xF0)<<4 | radioPktBuffer[15]));                

    for (i = 0; i <= 7; i++) {
        servo_PWM = (uint16_t)(pwm_reseived[i] / 1.846); // 1.5/PWM__CORRECTION = 1.21875
        Servo_Position[i] = servo_PWM;
    }
    
      
    /*
        // Check for F/S BUTTON
      if (halBuiButtonPushed()){    
      // store F/S values
          while (BUTTON_PRESSED());
      }
    */ 

      // Subtract old RSSI value from sum
      perRssiSum -= perRssiBuf[perRssiBufCounter];
      // Store new RSSI value in ring buffer, will add it to sum later
      perRssiBuf[perRssiBufCounter] = convertRssiByte(radioPktBuffer[PACKET_LENGTH+1]);
      
      // Add the new RSSI value to sum. Calculate and print
      // average RSSI to LCD
      perRssiSum += perRssiBuf[perRssiBufCounter];
      
      RSSI_dbm = perRssiBuf[perRssiBufCounter]; //convertRssiByte(radioPktBuffer[PACKET_LENGTH+1]); //perRssiSum/RSSI_AVG_WINDOW_SIZE;
                  
      if (++perRssiBufCounter == RSSI_AVG_WINDOW_SIZE) {
          perRssiBufCounter = 0;      // Wrap ring buffer counter
      }
}

/*==== INTERRUPT SERVICE ROUTINES ============================================*/

/******************************************************************************
* @fn  rf_IRQ
*
* @brief
*      The only interrupt flag which throws this interrupt is the IRQ_DONE interrupt.
*      So this is the code which runs after a packet has been received or
*      transmitted.
*
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/
#pragma vector=RF_VECTOR
__interrupt void rf_IRQ(void) {
    RFIF &= ~IRQ_DONE;        // Tx/Rx completed, clear interrupt flag
    S1CON &= ~0x03;           // Clear the general RFIF interrupt registers

    if (mode == RADIO_MODE_RX) {
        pktRcvdFlag = TRUE;
        DMAARM = DMAARM_CHANNEL0;
    }
    else {
        pktSentFlag = TRUE;
    }
}

/*  Check button for Binding the receiver  */
void Binding_RX(void){
      // Binding MODE
      frsky2way_init(0);
      // Start receiving
      CHANNR = 0;
      FSCAL1 = calData1[0];
      FSCAL3 = calData3[0];
       
      DMAARM = DMAARM_CHANNEL0;           // Arm DMA channel 0
      RFST   = STROBE_RX;                 // Switch radio to RX
      
      LED1 = LED_ON;
      LED3 = LED_ON;

         cons_puts("\r\n Wait for Binding values : \r\n");
           tx_id[0] = 0x03;
           tx_id[1] = 0x01; 
         {
           while (1){
             if (pktRcvdFlag == TRUE) {        // If SYNC has been detected
               pktRcvdFlag = FALSE;
               DMAARM = DMAARM_CHANNEL0;
               RFST = STROBE_RX;
                if (pktCheckValidity()) {
                    if (radioPktBuffer[5] == 0x00){
                        bind_tx_id[0] = radioPktBuffer[3];
                        bind_tx_id[1] = radioPktBuffer[4];   
                        
                        for (uint8_t n = 0x00; n <= 5; n++) {
                            channel_map[radioPktBuffer[5]+n] = radioPktBuffer[6+n];
                        }
                        break;
                      }
                  }
               }
             TOGGLE_LED3();
             for (uint8_t i = 0; i <= 10; i++) {
               delay_5ms();
             }
           }
       
           for (uint8_t bindIdx = 5; bindIdx <= 120; bindIdx += 5) {
             while (1) {
                if (pktRcvdFlag == TRUE) {        // If SYNC has been detected
                   pktRcvdFlag = FALSE;
                   DMAARM = DMAARM_CHANNEL0;
                   RFST = STROBE_RX;
                      if (pktCheckValidity()) {
                         if((radioPktBuffer[3] == bind_tx_id[0]) && (radioPktBuffer[4] == bind_tx_id[1])) {	// Correct TXID
                              if(radioPktBuffer[5] == bindIdx) {	// Correct bind packet number based on bindIdx
                                  for (uint8_t nn = 0; nn < 5 ; nn++) {
                                    if (radioPktBuffer[6 + nn] == 0 || radioPktBuffer[6 + nn] == radioPktBuffer[17]){ // Check for EOL
                                        eol = true;
                                        listLength = radioPktBuffer[5] + nn;
                                        break;
                                      }
                                      channel_map[radioPktBuffer[5] + nn] = radioPktBuffer[6 + nn];
                                  }
                                  break;	// Go to next packet index
                              }
                         }
                      }
                }
             }
             cons_puts("\r\nPacket "); cons_puthex8(bindIdx/5); 
             if (eol) break;	// End of list found, stop!
           }
         }

         data[0] = bind_tx_id[0]; // TX-id
         data[1] = bind_tx_id[1];
         data[2] = listLength;    // number of hoping chanels
         
         cons_puts("\r\nchannel_map values : ");
         for (uint8_t j = 0x00; j <= listLength; j++) {
            data[j+3] = channel_map[j];
            cons_puthex8(channel_map[j]);
            cons_putc(0x20);
          }
         cons_puts("\r\n listLength : "); cons_puthex8(listLength);

         cons_puts("\r\n Write to flash ");
             // Write to flash controller to be ready. //
            while (FCTL & FCTL_BUSY);
        
            // Setup FADDRH, FADDRL. See the datasheet for flash address details. //
            FADDRH = (uint16_t)flashDataAddr >> 9;
            FADDRL = ((uint16_t)flashDataAddr >> 1) & ~0xFF00;
            
            // Erase the page that later will be written to. (2-byte alignment.) //
            halFlashStartErase();  
            
            // Wait for the erase operation to complete. //
            while (FCTL & FCTL_BUSY);
            
            // Copy the flash write function to RAM and execute it. //
            runFunctionFromRam(flashWriter, ramFuncAddr, sizeof ramFuncAddr);

         // Binding successful - Flashing RED LED and wait for receiver reboot
         while (TRUE) { 
             TOGGLE_LED3();
             for (uint8_t i = 0; i <= 30; i++) {
               delay_5ms();
             }
         }
}                  
/* end Check button for Binding the receiver */  

/*==== END OF FILE ==========================================================*/
