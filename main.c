/* 
 * File:   main.c
 * Author: ShaneW
 *
 * Getting the PIC16F1783 I2C to work with the Evolve Controller
 * I2C Slave at 100kHz
 * Created on 23 April 2015, 1:41 PM
 */
// PIC16F1783 Configuration Bit Settings
#include <xc.h>
#ifndef _XTAL_FREQ
    #define _XTAL_FREQ 32000000
#endif
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1
#pragma config FOSC =       INTOSC      // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE =       OFF         // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE =      OFF         // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE =      ON          // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP =         OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD =        OFF         // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN =      OFF         // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN =   OFF         // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO =       OFF         // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN =      ON          // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT =        OFF         // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN =     OFF         // Voltage Regulator Capacitor Enable bit (Vcap functionality is disabled on RA6.)
#pragma config PLLEN =      OFF          // PLL Enable (4x PLL disabled)
#pragma config STVREN =     ON          // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV =       LO          // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR =      OFF         // Low Power Brown-Out Reset Enable Bit (Low power brown-out is disabled)
#pragma config LVP =        ON          // Low-Voltage Programming Enable (Low-voltage programming enabled)
/*
 * 
 */
#define GREEN_LED   LATCbits.LATC7
#define RED_LED     LATCbits.LATC6
#include <stdio.h>
#include <stdlib.h>
#include "NB_Time.h"

unsigned long tick_us1 =   0;

int main() {
    NB_TIMER_ sysLED    =   Tick24BitTimer(64000);
    //Timer0 and Interrupt setup
    OPTION_REGbits.TMR0CS = 0;              // Timer increments on instruction clock
    OPTION_REGbits.TMR0SE = 0;              //Timer increments on low-to-high transition of T0CKI pin
    OPTION_REGbits.INTEDG = 1;              //Interrupt on rising edge
    INTCONbits.TMR0IE = 1;                  // Enable interrupt on TMR0 overflow
    OPTION_REGbits.PSA  =1;                 //Disable Prescaler

    TRISAbits.TRISA =   0;      //Configure all pins as outputs
    TRISBbits.TRISB =   0;
    TRISCbits.TRISC =   0b00011000; //Configure RC3 and RC4 as inputs
    LATAbits.LATA   =   0;      //Configure all Outputs as low
    LATBbits.LATB   =   0;
    LATCbits.LATC   =   0;

    APFCONbits.SCKSEL   =   0;      //SCL/SCK is on pin RC3
    APFCONbits.SDISEL   =   0;      //SDA/SDI is on pin RC4

    SetClockSpeed();

    //I2C setup
    SSPCON1bits.WCOL    =   0;      //no collision detected
    SSPCON1bits.SSPOV   =   0;      //no overflow detected
    SSPCON1bits.SSPEN   =   1;      //enable SDA and SCL for serial
    SSPCON1bits.CKP     =   0;      //enable clock stretching
//    SSPCON1bits.SSPM    =   6;      //I2C Slave mode 7-bit address
    SSPCON1bits.SSPM    =   12;     //I2C Slave mode, 7 bits address, Start/Stop bit interrupt enabled

    SSPCON2bits.GCEN    =   0;      //General call address disabled
    SSPCON2bits.ACKSTAT =   1;      //Ack was not received
    SSPCON2bits.SEN     =   1;      //Clock stretch enabled (Slave)

    SSPCON3bits.PCIE    =   0;      //Stop Detection inturrupt disabled
    SSPCON3bits.SCIE    =   1;      //Start Detection inturrupt enabled
    SSPCON3bits.BOEN    =   1;      //Ignore SSPOV when BF is 0
    SSPCON3bits.SDAHT   =   0;      //0 = Minimum of 100ns hold time on SDA after falling dege of SCL. 1 = Minimum of 300ns
    SSPCON3bits.SBCDE   =   0;      //Slave bus collision interrupt disabled
    SSPCON3bits.AHEN    =   0;      //Address holding is disabled
    SSPCON3bits.DHEN    =   0;      //Data holding is disabled

//    SSPMSK              =   255;    //Compare address bit n received with SSPADD<n>
    SSPMSK              =   0;    //Compare address bit n received with SSPADD<n>

    SSPADDbits.SSPADD   =   0x60;    //Slave address 0x60

    __delay_ms(1000);
    INTCONbits.INTE = 1;                // enable the external interrupt
    INTCONbits. GIE = 1;                // Global interrupt enable

    GREEN_LED   =  1;
    while(1){
        if(CheckTick24Timer(sysLED)){
            sysLED  =   Tick24BitTimer(16000);
            RED_LED ^=  1;
        }
    }

    return 0;
}
void interrupt   tc_int  (void)        // interrupt function

{
    if(INTCONbits.T0IF && INTCONbits.T0IE){// if timer flag & enabled is set
                TMR0 -= 250;               // reload the timer - 31.25us per interrupt
                INTCONbits.T0IF = 0;                  // clear the interrupt flag
                tick_us1++;
                if ((tick_us1&0x7FFFFF)>=TIMER1_MAX_TICK){
                    tick_us1    -=  TIMER1_MAX_TICK;
                    tick_us1    ^=  0x00800000; //toggle ms bit
                }
    }
    if(PIR1bits.SSP1IF){
        PIR1bits.SSP1IF =   0;
        GREEN_LED   =   0;
    }
}
