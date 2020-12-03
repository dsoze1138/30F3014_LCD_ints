/*
 * File:     main.c
 * Target:   dsPIC30F3014
 * Author:   dan1138
 * Compiler: XC16 v1.60
 * IDE:      MPLABX v5.45
 *
 *                               dsPIC30F3014
 *                    +--------------:_:--------------+
 *          VPP ->  1 : MCLRn                    AVDD : 40 <- PWR
 *          LED <>  2 : RB0/AN0                  AVSS : 39 <- GND
 * RESET_BUTTON <>  3 : RB1/AN1               AN9/RB9 : 38 <> LCD_R/W
 *              <>  4 : RB2/AN2             AN10/RB10 : 37 <> LCD_EN
 *              <>  5 : RB3/AN3             AN11/RB11 : 36 <> LCD_RS
 *              ->  6 : RB4/AN4             AN12/RB12 : 35 <> 
 *              ->  7 : RB5/AN5              PGC2/RD0 : 34 <> LCD_D4
 *          PGC <>  8 : RB6/PGC/AN6          PGD2/RD1 : 33 <> LCD_D5
 *          PGD <>  9 : RB7/PGD/AN7               VDD : 32 <- PWR
 *              <> 10 : RB8/AN8                   VSS : 31 <- GND
 *          PWR -> 11 : VDD                       RF0 : 30 -> 
 *          GND -> 12 : VSS                       RF1 : 29 -> 
 *              -> 13 : OSC1                  RX2/RF4 : 28 -> 
 *              <- 14 : RC15/OSC2             TX2/RF5 : 27 -> 
 *              <> 15 : RC13/PGD1/SOSCO       RX1/RF2 : 26 <- RXD
 *              <> 16 : RC14/PGC1/SOSCI  TX1/PGD3/RF3 : 25 -> TXD
 *      Sensor0 <> 17 : RA11/INT0            PGC3/RF6 : 24 <> 
 *      Sensor2 <> 18 : RD9/INT2             INT1/RD8 : 23 <> Sensor1
 *       LCD_D7 <> 19 : RD3                       RD2 : 22 <> LCD_D6
 *          GND -> 20 : VSS                       VDD : 21 <- PWR
 *                    +-------------------------------:
 *                                 DIP-40
 *
 *   LCD 4-bit connections:
 *   RD0  <> LCD_D4    Special note that the LCD module I am using is a 
 *   RD1  <> LCD_D5    NOVATEK 7605. In 4-bit mode the NOVATEK 7605 is 
 *   RD2  <> LCD_D6    not 100% compatible with the Hitachi HD44780. 
 *   RD3  <> LCD_D7    The issue is that in 4-bit mode a status read 
 *   RB10 -> LCD_EN    returns the 4-bits in an order that is different 
 *   RB11 -> LCD_RS    from the HD44780.
 *   RB9  -> LCD_R/W
 *
 * Created on November 25, 2020, 2:18 PM
 * 
 */
// FOSC
#pragma config FOSFPR = FRC_PLL16       // Oscillator (FRC w/PLL 16x)
#pragma config FCKSMEN = CSW_ON_FSCM_OFF// Clock Switching and Monitor (Sw Enabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_OFF         // POR Timer Value (Timer Disabled)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_OFF         // PBOR Enable (Disabled)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)
/*
 * Target specific include files
 */
#include <xc.h>
/*
 * Standard C library include files
 */
#include <stdio.h>
#include <stdbool.h>
/*
 * Application specific include files
 */
#include "init.h"
#include "lcd.h"
/*
 * Hook to send printf output to LCD
 */
int __attribute__((__section__(".libc.write"))) write(int handle, void *buffer, unsigned int len) 
{
    unsigned int i;

    for (i = len; i; --i)
    {
        LCD_WriteData(*(unsigned char*)buffer++);
    }
    return(len);
}
/*
 * Global memory
 */
bool INT0_Event;
bool INT1_Event;
bool INT2_Event;
/*
 * Initialize this PIC
 */
void PIC_Init(void)
{
    __builtin_disi(0x3FFF); /* disable interrupts for 16383 cycles */
    
    IEC0 = 0;
    IEC1 = 0;
    IEC2 = 0;
    
    INTCON1bits.NSTDIS = 1; /* disable interrupt nesting */
    
    __builtin_disi(0x0000); /* enable interrupts */
    
    ADPCFG = 0x1FFF; /* Set for digital I/O */
}
/*
 * Guard sensor interrupt
 */
void INT0_Init(void)
{
    IEC0bits.INT0IE = 0;    //Disable interrupts
    TRISAbits.TRISA11 = 1;  //Make INT0 an input
    INTCON2bits.INT0EP = 0; //Interrupt on Positive Edge (When metal is not detected)
    IPC0bits.INT0IP = 5;    //Set interrupt priority
    IFS0bits.INT0IF = 0;    //External interrupt 0 flag status bit
    INT0_Event = 0;
    IEC0bits.INT0IE = 1;    //Enable interrupts
}
/*
 * INT0 handler
 */
void __attribute__((interrupt, auto_psv)) _INT0Interrupt(void)
{
    IFS0bits.INT0IF = 0; //Clear Raised Interrupt Flag
    INT0_Event = 1;     /* Set by handler cleared in process loop */
    /*
     * Perform emergency hardware dependent actions here.
     * DO NOT SPIN WAIT WITHIN AN INTERRUPT HANDLER,
     */
}
/*
 * Wrench sensor interrupt
 */
void INT1_Init(void)
{
    IEC1bits.INT1IE = 0;    //Disable interrupts
    TRISDbits.TRISD9 = 1;   //Make INT2 an input
    INTCON2bits.INT1EP = 0; //Interrupt on Positive Edge (When metal is detected)
    IPC4bits.INT1IP = 5;    //Set interrupt priority
    IFS1bits.INT1IF = 0;    //External interrupt 1 flag status bit
    INT1_Event = 0;
    IEC1bits.INT1IE = 1;    //Enable interrupts
}
/*
 * INT1 handler
 */
void __attribute__((interrupt, auto_psv)) _INT1Interrupt(void)
{
    IFS1bits.INT1IF = 0; //Clear Raised Interrupt Flag
    INT1_Event = 1;     /* Set by handler cleared in process loop */
    /*
     * Perform emergency hardware dependent actions here.
     * DO NOT SPIN WAIT WITHIN AN INTERRUPT HANDLER,
     */
}
/*
 * sensor2 interrupt
 */
void INT2_Init(void)
{
    IEC1bits.INT2IE = 0;    //Disable interrupts
    TRISDbits.TRISD8 = 1;   //Make INT1 an input
    INTCON2bits.INT2EP = 0; //Interrupt on Positive Edge (When metal is detected)
    IPC5bits.INT2IP = 5;    //Set interrupt priority
    IFS1bits.INT2IF = 0;    //External interrupt 1 flag status bit
    INT2_Event = 0;
    IEC1bits.INT2IE = 1;    //Enable interrupts
}
/*
 * INT2 handler
 */
void __attribute__((interrupt, auto_psv)) _INT2Interrupt(void)
{
    IFS1bits.INT2IF = 0; //Clear Raised Interrupt Flag
    INT2_Event = 1;     /* Set by handler cleared in process loop */
    /*
     * Perform emergency hardware dependent actions here.
     * DO NOT SPIN WAIT WITHIN AN INTERRUPT HANDLER,
     */
}
/*
 * Main application
 */
int main(void)
{
    /*
     * Application initialization
     */
    PIC_Init();
    LCD_Init();
    /*
     * User interface initialization
     */
    TRISBbits.TRISB0 = 0; // Make LED and output
    LATBbits.LATB0   = 0; // Set LED off
    TRISBbits.TRISB1 = 1; // Make RESET_BITTON an input
    /*
     * Show we are started
     */
    LCD_SetDDRamAddr(LINE_ONE);
    printf("dsPIC30F3014 Start");
    /*
     * Enable interrupts
     */
    INT0_Init();
    INT1_Init();
    INT2_Init();
    /*
     * Application process loop
     */
    for(;;)
    {
        /*
         * Implement state machines to show status to operator 
         * and handle interactions with the operator.
         */
        if(INT0_Event)
        {
            Nop();
            INT0_Event = 0;     /* Set by handler cleared in process loop */
        }
        if(INT1_Event)
        {
            Nop();
            INT1_Event = 0;     /* Set by handler cleared in process loop */
        }
        if(INT2_Event)
        {
            Nop();
            INT2_Event = 0;     /* Set by handler cleared in process loop */
        }
    }
    return 0;
}