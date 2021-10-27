// main.c Version 1.0 16 January, 2021
// Copyright 2021 Advanced Design and Manufacturing, LLC
// For SonaLume S2C Engine 0838.00000015.006+ (HWV0.6))
// Initial internal release...

/* MIT License

Copyright (c) 2021 Advanced Design and Manufacturing, LLC

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */


// *************************************************
// Remove this line from released source code
#include "flavor.h"                                 // Pull in the configuration "flavor" for production use
// *************************************************

// Configuration bits:
// FBS
#pragma config BWRP = OFF                           // Boot Segment Write Protect Disabled
#pragma config BSS = OFF                            // No boot program flash segment
// FGS
#pragma config GWRP = OFF                           // General segment may be written
#pragma config GSS0 = OFF                           // No Protection
// FOSCSEL
#pragma config FNOSC = FRC                          // Oscillator Select->FRC
#pragma config SOSCSRC = ANA                        // SOSC Source Type Analog Mode for use with crystal
#pragma config LPRCSEL = HP                         // LPRC Osc Power and Accuracy High Power, High Accuracy Mode
#pragma config IESO = ON                            // Int Ext Switchover mode enabled (Two-speed Start-up enabled)
// FOSC
#pragma config POSCMOD = NONE                       // Primary oscillator disabled
#pragma config OSCIOFNC = OFF                       // CLKO output signal is not active on the OSCO pin
#pragma config POSCFREQ = HS                        // Primary osc/external clock input frequency greater than 8MHz
#pragma config SOSCSEL = SOSCHP                     // Secondary Oscillator configured for high-power operation
#pragma config FCKSM = CSECMD                       // Clock Switching is enabled, Fail-safe Clock Monitor is disabled
// FWDT
#pragma config WDTPS = PS32768                      // Watchdog Timer Postscale Select bits 1:32768
#pragma config FWPSA = PR128                        // WDT Prescaler bit WDT prescaler ratio of 1:128
#pragma config FWDTEN = OFF                         // WDT disabled in hardware SWDTEN bit disabled
#pragma config WINDIS = OFF                         // Standard WDT selected(windowed WDT disabled)
// FPOR
#pragma config BOREN = BOR3                         // Brown-out Reset enabled in hardware, SBOREN bit disabled
#pragma config LVRCFG = OFF                         // Low Voltage regulator is not available
#pragma config PWRTEN = ON                          // PWRT enabled
#pragma config I2C1SEL = PRI                        // Use Default SCL1/SDA1 Pins For I2C1
#pragma config BORV = V30                           // Brown-out Reset set to highest voltage (3.0V)
#pragma config MCLRE = ON                           // RA5 input pin disabled, MCLR pin enabled
// FICD
#pragma config ICS = PGx1                           // ICD Pin Placement Select EMUC/EMUD share PGC1/PGD1
// FDS
#pragma config DSWDTPS = DSWDTPSF                   // Deep Sleep Watchdog Timer Postscale 1:2,147,483,648 (25.7 Days)
#pragma config DSWDTOSC = LPRC                      // DSWDT uses Low Power RC Oscillator (LPRC)
#pragma config DSBOREN = ON                         // Deep Sleep BOR enabled in Deep Sleep
#pragma config DSWDTEN = ON                         // DSWDT enabled


// Unused Definitions:
//#define BTSUPPORT                                   // Support for Bluetooth control of parameters
//#define BTDEBUG                                     // Adds debug messages to UART output
//#define PIN9RAW                                     // Supports boards newer than V0.6 that use pin 9 for the raw audio input

// *************************************************
// Remove this line from released source code, replace with "CFG0" lines from configurations.h
#include "configurations.h"                         // Pull in the list of configurations chosen by the flavor definition
// *************************************************

#define ADC12BITS                                   // Use 12-Bit ADC mode instead of 10-Bit mode
#define SYS_FREQ        32000000L
#define FCY             (SYS_FREQ/2)
#define MAXSAMPLES      25                          // Fixed maximum number of samples used in AGC - memory limited
#define WS2812_L        0xC0                        // Crazy way of defining the Low signal for the WS2812B using SPI
#define WS2812_H        0xF0                        // Crazy way of defining the High signal for the WS2812B using SPI
#define UP              1                           // For digital pot comms
#define DOWN            0
#define BLACK           0x0                         // Preset colors
#define WHITE           0xFFFFFF
#define RED             0x00FF00
#define GREEN           0x800000
#define BLUE            0x0000FF
#define ORANGE          0x25FF00
#define CYAN            0xFF00FF
#define VIOLET          0x30D3D3
#define ORANGERED       0x11FF00
#define PALEGREEN       0xF24646
#define BLUEVIOLET      0x06C4F5
#define HOTPINK         0x1EFF69
#define SLATEBLUE       0x161F91
#define INDIGO          0x000E30
#define YELLOW          0xFFFF00

#define SAMPLES         10                          // Number of rolling average samples to use
#define HITHRESH        500                         // Threshold where gain will be reduced (in ADC counts)
#define LOTHRESH        75                          // Threshold where gain will be increased (in ADC counts)
#define AGCDELAY        2                           // Used to allow AGC averages to change a bit before altering gain
#define DOWNSTEPS       4                           // Number of digital pot steps to make at a time, DOWN attenuates
#define UPSTEPS         4                           // UP increases sensitivity
// Fine tune ADC sample rate here
#define SMPIVALUE       3                           // Interrupt after x + 1 samples 0 - 31
#define SAMCVALUE       1                           // Auto-Sample Time Select (Tad) 0 - 31                       

#ifndef PIN9RAW
#define RAWAUDIO ADC1BUF4                           // ADC data for raw channel 4 (Raw) hardware < V0.7
#endif
#ifdef PIN9RAW
#define RAWAUDIO ADC1BUF15                          // ADC data for raw channel 4 (Raw) hardware >= V0.7
#endif
#define LOWAUDIO ADC1BUF12                          // ADC data for low channel 12 (Red)
#define MIDAUDIO ADC1BUF13                          // ADC data for mid channel 13 (Green)
#define HIGHAUDIO ADC1BUF10                         // ADC data for high channel 10 (Blue)

#ifdef BTSUPPORT                                    // Experimental control of functions via UART
    #define UART1_CONFIG_TX_BYTEQ_LENGTH (8+1)      // Length of UART1 TX buffer
    #define UART1_CONFIG_RX_BYTEQ_LENGTH (8+1)      // Length of UART1 RX buffer
#endif

// Included Files:
#include "xc.h"
#include "stdint.h"
#include <libpic30.h>
#include <stdlib.h>
#include <string.h>
#ifdef BTSUPPORT
#include <stdio.h>
#endif

// Globals:
#ifdef BTSUPPORT
static uint8_t txQueue[UART1_CONFIG_TX_BYTEQ_LENGTH]; // Define UART1 TX buffer
static uint8_t rxQueue[UART1_CONFIG_RX_BYTEQ_LENGTH]; // Define UART1 RX buffer
static uint8_t * volatile rxTail;
static uint8_t *rxHead;
static uint8_t *txTail;
static uint8_t * volatile txHead;
static uint8_t volatile rxOverflowed;
uint8_t rdata[3];                                   // Array for received data
#endif
// ******** SET NUMBER OF LEDS HERE ********
uint16_t numLEDs = NLEDS;                           // Set number of WS2812 LEDs in string here
// *****************************************
uint16_t samples;
uint16_t hiThresh;
uint16_t loThresh;
uint16_t agcDelay;
uint16_t downSteps;
uint16_t upSteps;
uint16_t avgLvl;                                    // Average value of incoming ADC data
uint8_t mode;                                       // Number of selected display mode

uint32_t ledData[MAXNUMLEDS];                       // Array for holding separate 24-bit LED data in GRB format
uint16_t averageTable[MAXSAMPLES];                  // Array of 10 or 12 bit samples to use in calculating average

#ifdef ADC12BITS
    #define SCALINGFACTOR   (4096/numLEDs)          // Scaling factor to match ADC resolution
#else
    #define SCALINGFACTOR   (1024/numLEDs)          // Scaling factor to match ADC resolution
#endif

const uint8_t cie[256] = {                          // CIE table to help mix random colors
	0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 
	1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 
	2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 
	3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 
	5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 
	7, 8, 8, 8, 8, 9, 9, 9, 10, 10, 
	10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 
	13, 14, 14, 15, 15, 15, 16, 16, 17, 17, 
	17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 
	22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 
	28, 28, 29, 29, 30, 31, 31, 32, 32, 33, 
	34, 34, 35, 36, 37, 37, 38, 39, 39, 40, 
	41, 42, 43, 43, 44, 45, 46, 47, 47, 48, 
	49, 50, 51, 52, 53, 54, 54, 55, 56, 57, 
	58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 
	68, 70, 71, 72, 73, 74, 75, 76, 77, 79, 
	80, 81, 82, 83, 85, 86, 87, 88, 90, 91, 
	92, 94, 95, 96, 98, 99, 100, 102, 103, 105, 
	106, 108, 109, 110, 112, 113, 115, 116, 118, 120, 
	121, 123, 124, 126, 128, 129, 131, 132, 134, 136, 
	138, 139, 141, 143, 145, 146, 148, 150, 152, 154, 
	155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 
	175, 177, 179, 181, 183, 185, 187, 189, 191, 193, 
	196, 198, 200, 202, 204, 207, 209, 211, 214, 216, 
	218, 220, 223, 225, 228, 230, 232, 235, 237, 240, 
	242, 245, 247, 250, 252, 255, 
};


// Functions:
void CLOCK_Initialize(void)                         // Set up the system clock
{
    CLKDIV = 0x3000;                                // RCDIV FRC/1; DOZE 1:8; DOZEN disabled; ROI disabled;
    OSCTUN = 0x00;                                  // TUN Center frequency; 
    PMD1 = 0x00;                                    // ADC1MD enabled; T3MD enabled; T4MD enabled; T1MD enabled; 
                                                    // U2MD enabled; T2MD enabled; U1MD enabled; SPI2MD enabled; 
                                                    // SPI1MD enabled; T5MD enabled; I2C1MD enabled; 
    PMD2 = 0x00;                                    // IC3MD enabled; OC1MD enabled; IC2MD enabled; OC2MD enabled; 
                                                    // IC1MD enabled; OC3MD enabled;
    PMD3 = 0x00;                                    // RTCCMD enabled; CMPMD enabled; CRCPMD enabled; I2C2MD enabled;
    PMD4 = 0x00;                                    // EEMD enabled; CTMUMD enabled; REFOMD enabled; ULPWUMD enabled; 
                                                    // HLVDMD enabled; 
    __builtin_write_OSCCONH((uint8_t) ((0x01 << _OSCCON_NOSC_POSITION ) >> 0x08 )); // CF no clock failure; NOSC FRCPLL; 
    __builtin_write_OSCCONL((uint8_t) ((0x100 | _OSCCON_OSWEN_MASK) & 0xFF)); // SOSCEN disabled; CLKLOCK unlocked; 
                                                    // OSWEN Switch is Complete;
    while (OSCCONbits.OSWEN != 0);                  // Wait for Clock switch to occur
    while (OSCCONbits.LOCK != 1);
}


void PIN_MANAGER_Initialize (void)
{
    LATA = 0x0000;                                  // Set the Output Latch SFRs
    LATB = 0x0000;
    TRISA = 0x5;                                    // Configure pins RA0 (VRef) and RA2 (AN13 Green) as digital inputs
#ifdef PIN9RAW
    TRISB = 0xD000;                                 // RB4 (AN15 Raw), RB12 (AN12 Red) and RB14 (AN10 Blue) as digital inputs
#endif
    TRISB = 0x5004;                                 // RB2 (AN4 Raw), RB12 (AN12 Red) and RB14 (AN10 Blue) as digital inputs
    CNPD1 = 0x0000;                                 // Set the Weak Pull Up and Weak Pull Down SFRs
    CNPD2 = 0x0000;
    CNPD3 = 0x0000;
    CNPU1 = 0x0000;
    CNPU2 = 0x0000;
    CNPU3 = 0x0000;
    ODCA = 0x0000;                                  // Set the Open Drain SFRs
    ODCB = 0x0000;
    ANSA = 0x0004;                                  // Configure Port A pins as all digital except RA2 (AN13 Green) as analog input
#ifdef PIN9RAW
    ANSB = 0x9400;                                  // RB4 (AN15 Raw), RB12 (AN12 Red) and RB14 (AN10 Blue) as analog inputs
#endif
    ANSB = 0x1404;                                  // RB2 (AN4 Raw), RB12 (AN12 Red) and RB14 (AN10 Blue) as analog inputs
}


void INTERRUPT_Initialize (void)
{
    IPC4bits.MI2C1IP = 1;                           // MICI: MI2C1 - I2C1 Master Events Priority: 1   
    IPC3bits.AD1IP = 1;                             // ADI: ADC1 - A/D Converter 1 Priority: 1   
    IPC0bits.T1IP = 1;                              // TI: T1 - Timer1 Priority: 1
}


void TMR1_Initialize (void)                         // Timer for adjusting AGC gain
{ 
    TMR1 = 0x00;
    //PR1 = 0xFFFF;                                 // PERIOD = 1.04856 s; FREQUENCY = 16000000 Hz; PR1 65535; // Alt. timing  
    //PR1 = 0x7FFF;                                 // PERIOD = 0.52428 s; FREQUENCY = 16000000 Hz; PR1 32767; // Alt. timing
    PR1 = 0X3FFF;                                   // PERIOD = 0.26214 S; FREQUENCY = 16000000 HZ; PR1 16383;
    T1CON = 0x8030;                                 // TCKPS 1:256; TON enabled; TSIDL disabled; TCS FOSC/2; 
                                                    // TECS SOSC; TSYNC disabled; TGATE disabled; 
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
}


void TMR2_Initialize (void)                         // Timer for calculating AGC averages
{
    TMR2 = 0x00;
    //PR2 = 0xFFFF;                                 // PERIOD = 1.04856 s; FREQUENCY = 16000000 Hz; PR2 65535; // Alt. timing 
    //PR2 = 0x7FFF;                                 // PERIOD = 0.52428 s; FREQUENCY = 16000000 Hz; PR2 32767; // Alt. timing
    PR2 = 0X3FFF;                                   // PERIOD = 0.26214 S; FREQUENCY = 16000000 HZ; PR2 16383;
    T2CON = 0x8030;                                 // TCKPS 1:256; TON enabled; TSIDL disabled; TCS FOSC/2; 
                                                    // TECS SOSC; TSYNC disabled; TGATE disabled; 
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
}


void ADC1_Initialize (void)
{
    AD1CON1 = 0x8474;                               // ASAM enabled; ADSIDL disabled; DONE disabled; FORM Absolute 
                                                    // decimal result, unsigned, right-justified; SAMP disabled; 
                                                    // SSRC Internal counter ends sampling and starts conversion; 
                                                    // MODE12 12-bit; ADON enabled;
    
    AD1CON2 = 0x8C0C;                               // CSCNA enabled; NVCFG AVSS; PVCFG 2 * Internal VBG; 
                                                    // ALTS disabled; BUFM disabled; SMPI Generates interrupt 
                                                    // after completion of every 4th sample/conversion operation; 
                                                    // OFFCAL disabled; BUFREGEN enabled; 

    AD1CON3 = 0x9F3F;                               // SAMC 31; EXTSAM disabled; ADRC RC clock; ADCS 64;

    AD1CHS = 0x00;                                  // CH0SA AN0; CH0SB AN0; CH0NB AVSS; CH0NA AVSS;

    AD1CSSH = 0x00;                                 // CSS26 disabled; CSS30 disabled; CSS29 disabled; CSS17 disabled; 
                                                    // CSS28 disabled; CSS16 disabled; CSS27 disabled; 
                
    AD1CSSL = 0x3410;                               // CSS9 disabled; CSS8 disabled; CSS7 disabled; CSS6 disabled; 
                                                    // CSS5 disabled; CSS4 enabled; CSS3 disabled; CSS2 disabled; 
                                                    // CSS15 disabled; CSS1 disabled; CSS14 disabled; CSS0 disabled; 
                                                    // CSS13 enabled; CSS12 enabled; CSS11 disabled; CSS10 enabled; 

    AD1CON5 = 0x1000;                               // ASEN disabled; WM Legacy operation; ASINT No interrupt; 
                                                    // CM Less Than mode; BGREQ disabled; CTMUREQ disabled; 
                                                    // LPEN disabled; 

    AD1CHITH = 0x00;                                // CHH17 disabled; CHH16 disabled;

    AD1CHITL = 0x00;                                // CHH9 disabled; CHH8 disabled; CHH7 disabled; CHH6 disabled; 
                                                    // CHH5 disabled; CHH4 disabled; CHH3 disabled; CHH2 disabled; 
                                                    // CHH1 disabled; CHH0 disabled; CHH11 disabled; CHH10 disabled; 
                                                    // CHH13 disabled; CHH12 disabled; CHH15 disabled; CHH14 disabled; 

    AD1CTMUENH = 0x00;                              // CTMEN30 disabled; CTMEN29 disabled; CTMEN16 disabled; 
                                                    // CTMEN17 disabled;
    
    AD1CTMUENL = 0x00;                              // CTMEN5 disabled; CTMEN6 disabled; CTMEN7 disabled; 
                                                    // CTMEN8 disabled; CTMEN9 disabled; CTMEN12 disabled; 
                                                    // CTMEN13 disabled; CTMEN10 disabled; CTMEN0 disabled; 
                                                    // CTMEN11 disabled; CTMEN1 disabled; CTMEN2 disabled; 
                                                    // CTMEN3 disabled; CTMEN4 disabled; CTMEN14 disabled; 
                                                    // CTMEN15 disabled; 
#ifdef ADC12BITS
    AD1CON1bits.MODE12 = 1;                         // Use 12-bits ADC mode
#else
    AD1CON1bits.MODE12 = 0;                         // Use 10-bits ADC mode
#endif                                              // Tad = Tcy = 1.67uS, Tad min. = 600nS
    AD1CON2bits.SMPI = SMPIVALUE;                   // Set interrupt after x + 1 samples        
    AD1CON3bits.SAMC = SAMCVALUE;                   // Number of Tad clocks in sample
    IEC0bits.AD1IE = 1;                             // Enable the ADC1 interrupt.           
}


void SPI1_Initialize (void)
{ 
    SPI1CON1 = 0x1277;                              // MSTEN Master; DISSDO disabled; PPRE 1:1; SPRE 3:1; MODE16 disabled; SMP End; DISSCK enabled; CKP Idle:High, Active:Low; CKE Idle to Active; 
    SPI1CON2 = 0x01;                                // SPIBEN enabled; SPIFPOL disabled; SPIFE disabled;
    SPI1STAT = 0x800C;                              // SPITBF disabled; SISEL SPI_INT_SPIRBF; SPIRBF disabled; SPIROV disabled; SPIEN enabled; SRXMPT disabled; SRMPT disabled; SPISIDL disabled; SPIBEC disabled; 
}

#ifdef BTSUPPORT
void UART1_Initialize(void)
{
    IEC0bits.U1TXIE = 0;
    IEC0bits.U1RXIE = 0;
    U1MODE = (0x8008 & ~(1<<15));                   // Disable UART ON bit
                                                    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; 
                                                    // USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; RXINV disabled; UEN TX_RX; 
                                                    // Data Bits = 8; Parity = None; Stop Bits = 1;
    U1STA = 0x00;                                   // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; 
                                                    // UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
    U1BRG = 0x1A0;                                  // BaudRate = 9600; Frequency = 16000000 Hz; BRG 416; 
    txHead = txQueue;
    txTail = txQueue;
    rxHead = rxQueue;
    rxTail = rxQueue;
    rxOverflowed = 0;
    IEC0bits.U1RXIE = 1;
    // Make sure to set LAT bit corresponding to TxPin as high before UART initialization
    U1MODEbits.UARTEN = 1;                          // Enable UART ON bit
    U1STAbits.UTXEN = 1;
}


void UART1_Transmit_ISR(void)
{ 
    if(txHead == txTail)
    {
        IEC0bits.U1TXIE = 0;
        return;
    }
    IFS0bits.U1TXIF = 0;
    while(!(U1STAbits.UTXBF == 1))
    {
        U1TXREG = *txHead++;
        if(txHead == (txQueue + UART1_CONFIG_TX_BYTEQ_LENGTH))
        {
            txHead = txQueue;
        }
        if(txHead == txTail)                        // Is buffer empty?
        {
            break;
        }
    }
}


void UART1_Receive_ISR(void)
{
    while((U1STAbits.URXDA == 1))
    {
        *rxTail = U1RXREG;
        if ( ( rxTail    != (rxQueue + UART1_CONFIG_RX_BYTEQ_LENGTH-1)) && // Check for collisions
             ((rxTail+1) != rxHead) )
        {
            rxTail++;
        } 
        else if ( (rxTail == (rxQueue + UART1_CONFIG_RX_BYTEQ_LENGTH-1)) &&
                  (rxHead !=  rxQueue) )
        {
            rxTail = rxQueue;                       // No collision
        } 
        else
        {
            rxOverflowed = 1;                       // Collision occurred
        }
    }
    IFS0bits.U1RXIF = 0;
}

uint8_t UART1_Read(void)
{
    uint8_t data = 0;

    while (rxHead == rxTail)
    {
    }
    data = *rxHead;
    rxHead++;
    if (rxHead == (rxQueue + UART1_CONFIG_RX_BYTEQ_LENGTH))
    {
        rxHead = rxQueue;
    }
    return data;
}


uint8_t UART1_IsTxReady(void)
{
    uint16_t size;
    uint8_t *snapshot_txHead = (uint8_t*)txHead;
    
    if (txTail < snapshot_txHead)
    {
        size = (snapshot_txHead - txTail - 1);
    }
    else
    {
        size = ( UART1_CONFIG_TX_BYTEQ_LENGTH - (txTail - snapshot_txHead) - 1 );
    }
    return (size != 0);
}

void UART1_Write(uint8_t byte)
{
    while(UART1_IsTxReady() == 0)
    {
    }
    *txTail = byte;
    txTail++;
    if (txTail == (txQueue + UART1_CONFIG_TX_BYTEQ_LENGTH))
    {
        txTail = txQueue;
    }
    IEC0bits.U1TXIE = 1;
}


uint8_t UART1_IsRxReady(void)
{    
    return !(rxHead == rxTail);
}


uint8_t UART1_IsTxDone(void)
{
    if(txTail == txHead)
    {
        return (uint8_t)U1STAbits.TRMT;
    }
    return 0;
}


void getSerial(void)
{
    uint8_t i = 0;
    
    while(UART1_IsRxReady())                        // Get received data and place in buffer
    {
        rdata[i] = UART1_Read();
#ifdef BTDEBUG
        printf("Received 0x%X at index %d\r\n", rdata[i], i); // Debug
#endif
        i++;
        if(i > 3)                                   // Valid incoming data has a command byte and two data bytes
        {
#ifdef BTDEBUG
            printf("ERROR - Too many bytes received"); // Debug
#endif
            memset(rdata, 0, sizeof(rdata));        // Clear the array
            return;
        }
    }
    if(rdata[0] == 1)                               // Get the number of LEDs variable
    {
 #ifdef BTDEBUG       
        printf("Got a Get numLEDs command\r\n");    // Debug
        printf("0x%X\r\n", numLEDs);                // Debug
#endif
        UART1_Write(numLEDs);                       // Send raw data
        memset(rdata, 0, sizeof(rdata));            // Clear the array
        return;
    }
    if(rdata[0] == 2)                               // Set the number of LEDs variable
    {
 #ifdef BTDEBUG
        printf("Got a Set numLEDs command - 0x%X\r\n", ((rdata[1] << 8) | rdata[2])); // Debug
#endif
        numLEDs = (rdata[1] << 8) | rdata[2];
        memset(rdata, 0, sizeof(rdata));            // Clear the array
        return;
    }
    if(rdata[0] == 3)                               // Get the number of rolling average samples to use in AGC
    {
 #ifdef BTDEBUG
        printf("Got a Get samples command\r\n");    // Debug
        printf("0x%X\r\n", samples);                // Debug
#endif
        UART1_Write(samples);                       // Send raw data
        memset(rdata, 0, sizeof(rdata));            // Clear the array
        return;
    }
    if(rdata[0] == 4)                               // Set the number of rolling average samples to use in AGC
    {
 #ifdef BTDEBUG
        printf("Got a Set samples command - 0x%X\r\n", ((rdata[1] << 8) | rdata[2])); // Debug
#endif
        samples = (rdata[1] << 8) | rdata[2];
        memset(rdata, 0, sizeof(rdata));            // Clear the array
        return;
    }
    if(rdata[0] == 5)                               // Get threshold where gain will be reduced (in ADC counts)
    {
#ifdef BTDEBUG
        printf("Got a Get hiThresh command\r\n");   // Debug
        printf("0x%X\r\n", hiThresh);               // Debug
#endif
        UART1_Write(hiThresh);                      // Send raw data
        memset(rdata, 0, sizeof(rdata));            // Clear the array
        return;
    }
    if(rdata[0] == 6)                               // Set threshold where gain will be reduced (in ADC counts)
    {
 #ifdef BTDEBUG
        printf("Got a Set hiThresh command - 0x%X\r\n", ((rdata[1] << 8) | rdata[2])); // Debug
#endif
        hiThresh = (rdata[1] << 8) | rdata[2];
        memset(rdata, 0, sizeof(rdata));            // Clear the array
        return;
    }
    if(rdata[0] == 7)                               // Get threshold where gain will be increased (in ADC counts)
    {
#ifdef BTDEBUG
        printf("Got a Get loThresh command\r\n");   // Debug
        printf("0x%X\r\n", loThresh);               // Debug
#endif
        UART1_Write(loThresh);                      // Send raw data
        memset(rdata, 0, sizeof(rdata));            // Clear the array
        return;
    }
    if(rdata[0] == 8)                               // Set threshold where gain will be increased (in ADC counts)
    {
#ifdef BTDEBUG
        printf("Got a Set loThresh command - 0x%X\r\n", ((rdata[1] << 8) | rdata[2])); // Debug
#endif
        loThresh = (rdata[1] << 8) | rdata[2];
        memset(rdata, 0, sizeof(rdata));            // Clear the array
        return;
    }
    if(rdata[0] == 9)                               // Get to allow AGC averages to change a bit before altering gain
    {
#ifdef BTDEBUG
        printf("Got a Get agcDelay command\r\n");   // Debug
        printf("0x%X\r\n", agcDelay);               // Debug
#endif
        UART1_Write(agcDelay);                      // Send raw data
        memset(rdata, 0, sizeof(rdata));            // Clear the array
        return;
    }
    if(rdata[0] == 10)                              // Set to allow AGC averages to change a bit before altering gain
    {
#ifdef BTDEBUG
        printf("Got a Set agcDelay command - 0x%X\r\n", ((rdata[1] << 8) | rdata[2])); // Debug
#endif
        agcDelay = (rdata[1] << 8) | rdata[2];
        memset(rdata, 0, sizeof(rdata));            // Clear the array
        return;
    }
    if(rdata[0] == 11)                              // Get number of digital pot steps to make at a time, DOWN attenuates 
    {
#ifdef BTDEBUG
        printf("Got a Get downSteps command\r\n");  // Debug
        printf("0x%X\r\n", downSteps);              // Debug
#endif
        UART1_Write(downSteps);                     // Send raw data
        memset(rdata, 0, sizeof(rdata));            // Clear the array
        return;
    }
    if(rdata[0] == 12)                              // Set number of digital pot steps to make at a time, DOWN attenuates 
    {
#ifdef BTDEBUG
        printf("Got a Set downSteps command - 0x%X\r\n", ((rdata[1] << 8) | rdata[2])); // Debug
#endif
        downSteps = (rdata[1] << 8) | rdata[2];
        memset(rdata, 0, sizeof(rdata));            // Clear the array
        return;
    }
    if(rdata[0] == 13)                              // Get number of digital pot steps to make at a time, UP increases 
    {
#ifdef BTDEBUG
        printf("Got a Get upSteps command\r\n");    // Debug
        printf("0x%X\r\n", upSteps);                // Debug
#endif
        UART1_Write(upSteps);                       // Send raw data
        memset(rdata, 0, sizeof(rdata));            // Clear the array
        return;
    }
    if(rdata[0] == 14)                              // Set number of digital pot steps to make at a time, UP increases 
    {
#ifdef BTDEBUG
        printf("Got a Set upSteps command - 0x%X\r\n", ((rdata[1] << 8) | rdata[2])); // Debug
#endif
        upSteps = (rdata[1] << 8) | rdata[2]; 
        memset(rdata, 0, sizeof(rdata));            // Clear the array
        return;
    }
    if(rdata[0] == 15)
    {
#ifdef BTDEBUG
        printf("Got a Get avgLvl command\r\n");     // Debug
        printf("0x%X\r\n", avgLvl);                 // Debug
#endif
        UART1_Write(avgLvl);                        // Send raw data
        memset(rdata, 0, sizeof(rdata));            // Clear the array
        return;
    }
    
    if(rdata[0] == 16)                              // Set mode to ?? 
    {
#ifdef BTDEBUG
        printf("Got a Set mode command - 0x%X\r\n", ((rdata[1] << 8) | rdata[2])); // Debug
#endif
        mode = (rdata[1] << 8) | rdata[2]; 
        memset(rdata, 0, sizeof(rdata));            // Clear the array
        return;
    }
    if(rdata[0] == 17)                              // Get current mode
    {
#ifdef BTDEBUG
        printf("Got a Get mode command\r\n");       // Debug
        printf("0x%X\r\n", mode);                   // Debug
#endif
        UART1_Write(mode);                          // Send raw data
        memset(rdata, 0, sizeof(rdata));            // Clear the array
        return;
    }
    return;
}
#endif


void setDigPot(uint8_t dir, uint8_t count)          // U/D Pin is connected to RB8
{                                                   // CS Pin is connected to RB9
                                                    // Pot used is MCP4011, see data sheet for details
    uint8_t x;
    
    if(dir == UP)                                   // Increment to a lower resistance...
    {
        _LATB9 = 1;                                 // Chip select should already be high...
        _LATB8 = 1;                                 // Set U/D pin to start an increment condition
        __delay_us(1);                              // Minimum delay of 500 nS needed to meet tLUC
        _LATB9 = 0;                                 // Drop chip select to enable incrementing of pot
        __delay_us(1);                              // Minimum delay of 500 nS needed to meet tLCUF
        for(x = 0; x < count; x++)
        {
            _LATB8 = 0;                             // Clear U/D pin to signal increment
            __delay_us(1);                          // Minimum delay of 500 nS needed to meet tLO
            _LATB8 = 1;                             // Set U/D pin again to end step
        }
        __delay_us(2);                              // Minimum delay of 5 uS needed to meet tCSLO
        _LATB9 = 1;                                 // Raise chip select to deselect pot
        return;
    }
    if(dir == DOWN)                                 // Decrement to a higher resistance
    {
        _LATB9 = 1;                                 // Chip select should already be high...
        _LATB8 = 0;                                 // Clear U/D pin to start a decrement condition
        __delay_us(1);                              // Minimum delay of 500 nS needed to meet tLUC
        _LATB9 = 0;                                 // Drop chip select to enable incrementing of pot
        __delay_us(3);                              // Minimum delay of 3 uS needed to meet tLCUR
        for(x = 0; x < count; x++)
        {
            _LATB8 = 1;                             // Set U/D pin to signal decrement
            __delay_us(1);                          // Minimum delay of 500 nS needed to meet tHI
            _LATB8 = 0;                             // Clear U/D pin again to end step
        }
        _LATB9 = 1;                                 // tCSLO should be met now, raise chip select to deselect pot
        return;
    }
}


uint16_t rollingAverage(uint16_t newSample)         // Compute a rolling average of the audio levels
{                                                   // Only used for AGC function
    uint32_t sum = 0;
    uint16_t indx = 0, divisor = 0;

    for(indx = (SAMPLES - 1); indx > 0; indx--)     // Copy data from index SAMPLES - 1 to index SAMPLES.
    {                                               // Overwrite the oldest data first, then index backwards
        averageTable[indx] = averageTable[indx - 1];// through the array shifting the values to the right
    }                                               // and leaving index 0 ready for new data.
    averageTable[0] = newSample;                    // Add the new value to index 0.

    for(indx = 0; indx < SAMPLES; indx++)           // Sum the data in the table
     {
         if(averageTable[indx] > 0)                 // If cell has a zero, don't use it in the averaging calculation
         {
             sum += averageTable[indx];             // otherwise add it to the sum
             divisor++;                             // Increment the divisor for each value added
         }
     }

    if(sum == 0)                                    // Eliminate divide by zero???
    {
        avgLvl = 0;
        return(0);
    }
    avgLvl = (uint16_t)(sum / divisor);             // Update global, casting result of 32-bit and 16-bit types
    return(sum / divisor);
}


void setSens(uint16_t average)                      // Simple input level control to keep display interesting
{
    if(average > HITHRESH)
    {
        setDigPot(DOWN, DOWNSTEPS);                 // DOWN to increase the pot's resistance
        return;
    }
    
    if(average < LOTHRESH)
    {
        setDigPot(UP, UPSTEPS);                     // UP to lower the pot's resistance
        return;
    }
    return;
}


void updateDisplay(void)                            // Send the color data to each LED in sequence, MSB first
{
    uint16_t i;
    uint8_t q, t;
    uint32_t pixel;
    IEC0bits.T2IE = 0;
    for(i = 0; i < numLEDs; i++)                    // This handles the number of LEDs in a string
    {   
#ifdef SWAPRG                                       // Swap red and green data for "wrong" LED strings
        pixel = ledData[i];
        pixel = ((pixel << 8) & 0xFF0000) | ((pixel >> 8) & 0x00FF00) | (pixel & 0x0000FF);
#else
        pixel = ledData[i];
#endif
        for(q = 0; q < 24; q++)                     // Break the three bytes of color data into bits, MSB first
        {
            if((pixel & 0x800000) == 0x800000)      // Use a bit mask to look at the MSB and see if it is one or zero
            {
                  SPI1BUF = WS2812_H;               // Use the SPI module to send the data out to the WS2812
                  while(SPI1STATbits.SPITBF);       // Wait for data to be sent
            }                                       // This method exceeds the low time for the WS2812, but
            else                                    // the part does not seem to mind. Low time should
            {                                       // be .6 - .8 uS, measured time is 2.9 - 3.1uS.
                  SPI1BUF = WS2812_L;
                  while(SPI1STATbits.SPITBF);       // Wait for data to be sent
            }
            pixel <<= 1;                            // Shift the remaining bits to the left and save the result in pixel
        }
    }
    for(t = 0; t < 110; t++)                        // Creates a low signal to latch the data in the WS2812 string
    {
        SPI1BUF = 0x0;                              // Send all zeroes as a latch command
        while(SPI1STATbits.SPITBF);                 // When the SPI TX buffer is clear, send more data
    }
    IEC0bits.T2IE = 1;
    return;
}


void bigMemSet(void *dest, uint32_t val, size_t len) // Version of memset for 32-bit values
{
    uint32_t *ptr = dest;
    while (len-- > 0)
        *ptr++ = val;
    return;
}

void atrophy(void)                                  // Sound activated pseudo-random seed generator
{
    srand(RAWAUDIO);                                // Use whatever is in the ADC as the seed
}
// **** Different display mode functions go here ****
#ifdef COLORTEST
void colorTest()                                    // For testing and creating new colors, also possible startup display
{                                                   // used during development to see if uC gets recycled during operation
    uint8_t divider = numLEDs / 13;                     
    
    bigMemSet(ledData, HOTPINK, divider);           // Turn on columns of LEDs in chosen color
    bigMemSet(ledData + divider, RED, divider);                    
    bigMemSet(ledData + (divider * 2), ORANGERED, divider);             
    bigMemSet(ledData + (divider * 3), ORANGE, divider);                
    bigMemSet(ledData + (divider * 4), YELLOW, divider);
    bigMemSet(ledData + (divider * 5), GREEN, divider);
    bigMemSet(ledData + (divider * 6), PALEGREEN, divider);
    bigMemSet(ledData + (divider * 7), CYAN, divider);
    bigMemSet(ledData + (divider * 8), BLUE, divider);
    bigMemSet(ledData + (divider * 9), SLATEBLUE, divider);
    bigMemSet(ledData + (divider * 10), INDIGO, divider);
    bigMemSet(ledData + (divider * 11), BLUEVIOLET, divider);
    bigMemSet(ledData + (divider * 12), VIOLET, divider);
    updateDisplay();
    return;
}
#endif

#ifdef MODE0
    void mode0(void)                                // Original S2C function, a proportional number of random elements light 
    {                                               // depending upon sound level in three ADC channels (Red = Low, Green = Mid, Blue = High)
        uint16_t midVal, lowVal, highVal, pixel, i;
        uint16_t adcValue = 0;

        samples = 10;
        hiThresh = 500;
        loThresh = 75;
        agcDelay = 2;
        downSteps = 4;
        upSteps = 4;  

        memset(ledData, 0, sizeof(ledData));        // Set all LED data to black
        adcValue = MIDAUDIO;                        // Get ADC data for mid channel 13 (Green)
        midVal = adcValue / SCALINGFACTOR;          // Scale ADC output to fit number of elements in display

        adcValue = LOWAUDIO;                        // Get ADC data for low channel 12 (Red)
        lowVal = adcValue / SCALINGFACTOR;          // Scale ADC output to fit number of elements in display

        adcValue = HIGHAUDIO;                       // Get ADC data for high channel 10 (Blue)
        highVal = adcValue / SCALINGFACTOR;         // Scale ADC output to fit number of elements in display

        for(i = 0; i < midVal; i++)                 // Set a number of LEDs in the array per audio level in each band
        {
            pixel = rand() % numLEDs;               // Random pixel number from 0 - number of LEDs
            ledData[pixel] = GREEN;                 // Set the green elements of the display randomly
        }
        for(i = 0; i < lowVal; i++)
        {
            pixel = rand() % numLEDs;               // Random pixel number from 0 - number of LEDs
            ledData[pixel] = RED;                   // Set the red elements of the display randomly
        }
        for(i = 0; i < highVal; i++)
        {
            pixel = rand() % numLEDs;               // Random pixel number from 0 - number of LEDs
            ledData[pixel] = BLUE;                  // Set the blue elements of the display randomly
        }
        updateDisplay();
        return;
    }
#endif    
#ifdef MODE1    
    void mode1(void)                                // Initial display with alternate pixel colors, proportional number of random elements light 
    {                                               // depending upon sound level in three channels
        uint16_t midVal, lowVal, highVal, pixel, i;
        uint16_t adcValue = 0;

        samples = 10;
        hiThresh = 500;
        loThresh = 75;
        agcDelay = 2;
        downSteps = 4;
        upSteps = 4;     

        memset(ledData, 0, sizeof(ledData));        // Set all LED data to black
        adcValue = MIDAUDIO;                        // Get ADC data for mid channel 13 (Green)
        midVal = adcValue / SCALINGFACTOR;          // Scale ADC output to fit number of elements in display

        adcValue = LOWAUDIO;                        // Get ADC data for low channel 12 (Red)
        lowVal = adcValue / SCALINGFACTOR;          // Scale ADC output to fit number of elements in display

        adcValue = HIGHAUDIO;                       // Get ADC data for high channel 10 (Blue)
        highVal = adcValue / SCALINGFACTOR;         // Scale ADC output to fit number of elements in display

        for(i = 0; i < midVal; i++)                 // Set a number of LEDs in the array per audio level in each band
        {
            pixel = rand() % numLEDs;               // Random pixel number from 0 - number of LEDs
            ledData[pixel] = YELLOW;                // Set the elements of the display randomly
        }
        for(i = 0; i < lowVal; i++)
        {
            pixel = rand() % numLEDs;               // Random pixel number from 0 - number of LEDs
            ledData[pixel] = HOTPINK;               // Set the elements of the display randomly
        }
        for(i = 0; i < highVal; i++)
        {
            pixel = rand() % numLEDs;               // Random pixel number from 0 - number of LEDs
            ledData[pixel] = CYAN;                  // Set the elements of the display randomly
        }
        updateDisplay();
        return;
    }
#endif    
#ifdef MODE2
    void mode2(void)                                // 5 channel display, proportional number of random elements light 
    {                                               // depending upon sound level in three channels
        uint16_t midVal, lowVal, highVal, pixel, i;
        uint16_t lowMidVal, highMidVal, adcValue = 0;

        samples = 10;
        hiThresh = 500;
        loThresh = 75;
        agcDelay = 2;
        downSteps = 4;
        upSteps = 4;  

        memset(ledData,0,sizeof(ledData));          // Set all LED data to black
        adcValue = MIDAUDIO;                        // Get ADC data for mid channel 13 (Green)
        midVal = adcValue / SCALINGFACTOR;          // Scale ADC output to fit number of elements in display

        adcValue = LOWAUDIO;                        // Get ADC data for low channel 12 (Red)
        lowVal = adcValue / SCALINGFACTOR;          // Scale ADC output to fit number of elements in display

        adcValue = HIGHAUDIO;                       // Get ADC data for high channel 10 (Blue)
        highVal = adcValue / SCALINGFACTOR;         // Scale ADC output to fit number of elements in display

        lowMidVal = (LOWAUDIO + MIDAUDIO) / 2;      // "Synthesize" a low-mid audio channel
        lowMidVal = lowMidVal / SCALINGFACTOR;      // Scale ADC output to fit number of elements in display

        highMidVal = (HIGHAUDIO + MIDAUDIO) / 2;    // "Synthesize" a high-mid audio channel
        highMidVal = highMidVal / SCALINGFACTOR;    // Scale ADC output to fit number of elements in display

        for(i = 0; i < midVal; i++)                 // Set a number of LEDs in the array per audio level in each band
        {
            pixel = rand() % numLEDs;               // Random pixel number from 0 - number of LEDs
            ledData[pixel] = GREEN;                 // Set the green elements of the display randomly
        }
        for(i = 0; i < lowVal; i++)
        {
            pixel = rand() % numLEDs;               // Random pixel number from 0 - number of LEDs
            ledData[pixel] = RED;                   // Set the red elements of the display randomly
        }
        for(i = 0; i < highVal; i++)
        {
            pixel = rand() % numLEDs;               // Random pixel number from 0 - number of LEDs
            ledData[pixel] = BLUE;                  // Set the blue elements of the display randomly
        }
        for(i = 0; i < lowMidVal; i++)
        {
            pixel = rand() % numLEDs;               // Random pixel number from 0 - number of LEDs
            ledData[pixel] = ORANGE;                // Set the blue elements of the display randomly
        }
        for(i = 0; i < highMidVal; i++)
        {
            pixel = rand() % numLEDs;               // Random pixel number from 0 - number of LEDs
            ledData[pixel] = CYAN;                  // Set the blue elements of the display randomly
        }
        updateDisplay();
        return;
    }
#endif    
#ifdef MODE3    
    void mode3(void)                                // Strobe effect, needs improvement
    {
        uint32_t color;

        samples = 2;
        hiThresh = 2500;
        loThresh = 200;
        agcDelay = 1;
        downSteps = 4;
        upSteps = 1;   

        if (LOWAUDIO > 500)                         // Bass frequencies
        {
            color = cie[rand() % 256] << 8 | cie[rand() % 256];
            color = color << 8 | cie[rand() % 256];


            bigMemSet(ledData, color, numLEDs);     // Turn on all LEDs
            updateDisplay();

            bigMemSet(ledData, BLACK, numLEDs);     // Turn off LEDs
            updateDisplay();
            __delay_ms(200);
       }

        return;
    }
#endif    
#ifdef MODE4    
    void mode4(void)                                // A proportional number of random elements light 
    {                                               // depending upon sound level in loudest channels (Red = Low, Green = Mid, Blue = High)
        uint16_t midVal, lowVal, highVal, pixel, i;
        uint16_t adcValue = 0;

        samples = 25;
        hiThresh = 700;
        loThresh = 75;
        agcDelay = 3;
        downSteps = 2;
        upSteps = 2;        

        bigMemSet(ledData, BLACK, numLEDs);         // Set all LED data to black
        adcValue = MIDAUDIO;                        // Get ADC data for mid channel 13 (Green)
        midVal = adcValue / SCALINGFACTOR;          // Scale ADC output to fit number of elements in display

        adcValue = LOWAUDIO;                        // Get ADC data for low channel 12 (Red)
        lowVal = adcValue / SCALINGFACTOR;          // Scale ADC output to fit number of elements in display

        adcValue = HIGHAUDIO;                       // Get ADC data for high channel 10 (Blue)
        highVal = adcValue / SCALINGFACTOR;         // Scale ADC output to fit number of elements in display
        if (midVal >= lowVal && midVal >= highVal)
        {
            for(i = 0; i < midVal; i++)             // Set a number of LEDs in the array per audio level in each band
            {
                pixel = rand() % numLEDs;           // Random pixel number from 0 - number of LEDs
                ledData[pixel] = GREEN;             // Set the green elements of the display randomly
            }
        }
        else if (lowVal >= midVal && lowVal >= highVal)
        {
            for(i = 0; i < lowVal; i++)
            {
                pixel = rand() % numLEDs;           // Random pixel number from 0 - number of LEDs
                ledData[pixel] = RED;               // Set the red elements of the display randomly
            }
        }
        else
        {    
            for(i = 0; i < highVal; i++)
            {
                pixel = rand() % numLEDs;           // Random pixel number from 0 - number of LEDs
                ledData[pixel] = BLUE;              // Set the blue elements of the display randomly
            }
        }
        updateDisplay();
        return;
    }
#endif    
#ifdef MODE5    
    void mode5(void)                                // A proportional number of random elements light but with random colors defined
    {                                               // depending upon sound level in loudest channels (Red = Low, Green = Mid, Blue = High)
        uint16_t midVal, lowVal, highVal, pixel, i;
        uint16_t adcValue = 0;
        uint32_t color;

        samples = 25;
        hiThresh = 700;
        loThresh = 75;
        agcDelay = 3;
        downSteps = 2;
        upSteps = 2;   

        memset(ledData, 0, sizeof(ledData));        // Set all LED data to black
        adcValue = MIDAUDIO;                        // Get ADC data for mid channel 13
        midVal = adcValue / SCALINGFACTOR;          // Scale ADC output to fit number of elements in display

        adcValue = LOWAUDIO;                        // Get ADC data for low channel 12
        lowVal = adcValue / SCALINGFACTOR;          // Scale ADC output to fit number of elements in display

        adcValue = HIGHAUDIO;                       // Get ADC data for high channel 10
        highVal = adcValue / SCALINGFACTOR;         // Scale ADC output to fit number of elements in display
        if (midVal >= lowVal && midVal >= highVal)
        {
            for(i = 0; i < midVal; i++)             // Set a number of LEDs in the array per audio level in each band
            {
                pixel = rand() % numLEDs;           // Random pixel number from 0 - number of LEDs  
                color = cie[rand() % 256] << 8 | cie[rand() % 256];
                color = color << 8 | cie[rand() % 256]; // Choose a random color for this pixel
                ledData[pixel] = color;                 
            }
        }
        else if (lowVal >= midVal && lowVal >= highVal)
        {
            for(i = 0; i < lowVal; i++)
            {
                pixel = rand() % numLEDs;           // Random pixel number from 0 - number of LEDs  
                color = cie[rand() % 256] << 8 | cie[rand() % 256];
                color = color << 8 | cie[rand() % 256]; // Choose a random color for this pixel
                ledData[pixel] = color;                 
            }
        }
        else
        {    
            for(i = 0; i < highVal; i++)
            {
                pixel = rand() % numLEDs;           // Random pixel number from 0 - number of LEDs  
                color = cie[rand() % 256] << 8 | cie[rand() % 256];
                color = color << 8 | cie[rand() % 256]; // Choose a random color for this pixel
                ledData[pixel] = color;                 
            }
        }
        updateDisplay();
        return;
    }   
#endif    
#ifdef MODE6    
void mode6(void)                                    // Divides display into three parts and shows solid blocks of red, green and blue 
{                                                   // (Red = Low, Green = Mid, Blue = High)
    uint32_t pixel;
    uint16_t divider = numLEDs / 3;

    samples = 12;
    hiThresh = 2000;
    loThresh = 1000;
    agcDelay = 2;
    downSteps = 4;
    upSteps = 4;  

    memset(ledData, 0, sizeof(ledData));            // Set all LED data to black

    pixel = 0x0 << 8 | (LOWAUDIO >> 4);             // Red
    pixel = pixel << 8 | 0x0;
    bigMemSet(ledData + (divider * 2), pixel, divider + 1);   

    pixel = (MIDAUDIO >> 4) << 8 | 0x0;             // Green
    pixel = pixel << 8 | 0x0;
    bigMemSet(ledData + divider, pixel, divider);

    pixel = 0x0 << 8 | 0x0;                         // Blue
    pixel = pixel << 8 | (HIGHAUDIO >> 4);
    bigMemSet(ledData, pixel, divider);

    updateDisplay();
    return;
}
#endif    
    
void runMode(uint8_t mode)                          // Called by interrupt, set early in main()...
{
    switch(mode)                                    // Using switch for future selections of running modes
    {                                               // like pushbuttons, etc...
#ifdef MODE0       
        case 0: mode0(); break;                     // Original random number of elements/band volume display
#endif
#ifdef MODE1
        case 1: mode1(); break;                     // Mode0 with alternate colors
#endif
#ifdef MODE2        
        case 2: mode2(); break;                     // 5-channel display with extra channels synthesized
#endif        
#ifdef MODE3
        case 3: mode3(); break;                     // Strobe display (Must be compiled in as it's buggy and ugly)
#endif
#ifdef MODE4        
        case 4: mode4(); break;                     // Audio band with highest level dominates display
#endif
#ifdef MODE5        
        case 5: mode5(); break;                     // Same as MODE4 but with random colors defined
#endif
#ifdef MODE6        
        case 6: mode6(); break;                     // Three sections, red, green and blue solid bands of color
#endif        
#ifdef COLORTEST
        default: colorTest();
#endif
    }
    return;
}
// **** Different display mode functions ends ****

void SYSTEM_Initialize(void)                        // Housekeeping and initialization
{
    CLOCK_Initialize();
    PIN_MANAGER_Initialize();
    SPI1_Initialize();
    INTERRUPT_Initialize();
    ADC1_Initialize();
    TMR1_Initialize();
    TMR2_Initialize();
    atrophy();                                      // Seed rand with ADC input value (Experimental))
}


int main(void)                                      // Main function:
{
    SYSTEM_Initialize();                            // Initialize the device
    setDigPot(UP, 64);                              // Set pot at lowest attenuation
// ******** SET FIXED MODE HERE ********
//    mode = 0;                                     // Set initial mode, can be overridden by UART commands if available
#ifdef START0
    mode = 0;
#endif
#ifdef START1
    mode = 1;
#endif
#ifdef START2
    mode = 2;
#endif
#ifdef START3
    mode = 3;
#endif
#ifdef START4
    mode = 4;
#endif
#ifdef START5
    mode = 5;
#endif
#ifdef START6
    mode = 6;
#endif
// *************************************
    memset(ledData, BLACK, sizeof(ledData));        // Turn off LEDs at run time
    updateDisplay();
    while(1)
    {
                                                    // Nothing currently happening here, code is interrupt driven
    }
    return(1);
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _T1Interrupt (  )
{                                                   // Maintains AGC gain settings, triggered once every 0.26214 second...
    static uint8_t count = 0;
    
    if (count >= AGCDELAY)                          // Delay setting AGC to give input average time to change
    {    
        setSens(avgLvl);                            // Set the new sensitivity level
        count = 0;                                  // Reset the counter
        IFS0bits.T1IF = 0;                          // Clear the interrupt flag
        return;
    }
    else
    {
        count++;                                    // Increment the counter
        IFS0bits.T1IF = 0;
        return;
    }
    
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _T2Interrupt (  ) 
{                                                   // For calculating AGC levels, triggered every 0.26214 seconds...
    //rollingAverage(HIGHAUDIO);                    // AGC based on average high audio level
    //rollingAverage(MIDAUDIO);                     // AGC based on average mid audio level
    rollingAverage(LOWAUDIO);                       // AGC based on average low audio level
    //rollingAverage(RAWAUDIO);                     // AGC based on average raw audio level
    IFS0bits.T2IF = 0;
    return;
}


void __attribute__ ( ( __interrupt__ , auto_psv ) ) _ADC1Interrupt ( void )
{
	if(IFS0bits.AD1IF)
	{
        runMode(mode);                              // Run the desired function
		IFS0bits.AD1IF = 0;                         // Clear the ADC interrupt flag
	}
    return;
}

#ifdef BTSUPPORT
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1TXInterrupt ( void )
{
    UART1_Transmit_ISR();
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1RXInterrupt( void )
{
    UART1_Receive_ISR();
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1ErrInterrupt( void )
{
    if ((U1STAbits.OERR == 1))
    {
        U1STAbits.OERR = 0;
    }
    IFS4bits.U1ERIF = 0;
}


int __attribute__((__section__(".libc.write"))) write(int handle, void *buffer, unsigned int len) 
{
    unsigned int i;
    uint8_t *data = buffer;

    for(i=0; i<len; i++)
    {
        while(UART1_IsTxReady() == 0)
        {
        }
        UART1_Write(*data++);
    }
    return(len);
}
#endif