/*
 * File:   dspic33ep256gp506_explorer_16.c
 *
 * Created on 18 September 2014
 *
 * Initializes the device configuration fuses, clock frequency, UART2 pins,
 * LED I/O, Button I/O, and 25LC256 SPI2 pins for the dsPIC33EP256GP506 PIM on
 * the Explorer 16 development board.
 */

// DOM-IGNORE-BEGIN
/*******************************************************************************
  Copyright (C) 2015 Microchip Technology Inc.

  MICROCHIP SOFTWARE NOTICE AND DISCLAIMER:  You may use this software, and any
  derivatives created by any person or entity by or on your behalf, exclusively
  with Microchip's products.  Microchip and its licensors retain all ownership
  and intellectual property rights in the accompanying software and in all
  derivatives here to.

  This software and any accompanying information is for suggestion only.  It
  does not modify Microchip's standard warranty for its products.  You agree
  that you are solely responsible for testing the software and determining its
  suitability.  Microchip has no obligation to modify, test, certify, or
  support the software.

  THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER
  EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
  WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR
  PURPOSE APPLY TO THIS SOFTWARE, ITS INTERACTION WITH MICROCHIP'S PRODUCTS,
  COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

  IN NO EVENT, WILL MICROCHIP BE LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT
  (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), STRICT LIABILITY,
  INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE,
  EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF
  ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWSOEVER CAUSED, EVEN IF
  MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
  TO THE FULLEST EXTENT ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
  CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF
  FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

  MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
  TERMS.
*******************************************************************************/
// DOM-IGNORE-END

#if defined(__dsPIC33EP512GP506__) || defined(__dsPIC33EP256GP506__) || defined(__dsPIC33EP128GP506__) || defined(__dsPIC33EP64GP506__) || defined(__dsPIC33EP32GP506__) || defined(__dsPIC33EP512MC506__) || defined(__dsPIC33EP256MC506__) || defined(__dsPIC33EP128MC506__) || defined(__dsPIC33EP64MC506__) || defined(__dsPIC33EP32MC506__)

#define FCY         70000000ul

#include <xc.h>

// Device Configuration Fuses
// These are commented out because they are defined in the bootloader project
// instead. If you want to use these, be sure to remove the applicable
// definition(s) from the bootloader project. Different config words can be
// split (ex: define _FOSC in the bootloader while _FWDT is defined here in the
// application).
//_FICD(ICS_PGD1 & JTAGEN_OFF)                                                // Debug using PGEC1 and PGED1, turn off JTAG to recover I/O pins
//_FPOR(ALTI2C1_OFF & ALTI2C2_OFF & WDTWIN_WIN75)                             // Use primary I2C1 and I2C2 pin mappings, set watchdog window to 75% (not important in non-window mode, but included for completness)
//_FWDT(WDTPOST_PS2048 & WDTPRE_PR32 & PLLKEN_OFF & WINDIS_OFF & FWDTEN_OFF)  // ~2048ms timeout when watchdog is turned on in software (not forced on in hardware), continue executing from original clock before PLL locks on clock switch, use ordinary (non-windowed) Watchdog Timer mode
//_FOSC(POSCMD_NONE & OSCIOFNC_ON & IOL1WAY_OFF & FCKSM_CSECMD)               // No primary oscillator, recover OSC2 pin as GPIO, allow multiple PPS remappings, enable clock switching but disable fail safe clock monitor
//_FOSCSEL(FNOSC_FRCPLL & IESO_ON)                                            // Start with FRC, then auto-switch to FRC+PLL when PLL locks
//_FGS(GWRP_OFF & GCP_OFF)                                                    // Code Protect = OFF, Write Protect = OFF (Don't enable Write Protect either if you plan to use a bootloader!)

extern int __C30_UART;


// @return: FCY clock speed we just configured the processor for
unsigned long InitializeBoard(void)
{
    // Switch to FRC clock (no PLL), in case if the PLL is currently in use.
    // We are not allowed to change the PLL prescalar, postscalar or feedback
    // divider while it is running.
    __builtin_write_OSCCONH(0x00);
    __builtin_write_OSCCONL(0x01);

    // Wait for clock switch to complete
    while(OSCCONbits.OSWEN);

    // Configure PLL for Fosc = 140MHz/Fcy = 70MIPS using 7.37 MHz internal FRC oscillator
    CLKDIV = 0xB000; // ROI = 1, DOZE = 8:1, FRCDIV = 1:1, PLLPOST = 2:1, PLLPRE = 2:1
    PLLFBD = (FCY * 2u * 2u * 2u + 7370000u / 2u) / 7370000u - 2u; // 74 @ 70 MIPS (7.37 MHz input clock from FRC)
    __builtin_write_OSCCONH(0x01);      // Initiate Clock Switch to use the FRC Oscillator + PLL (NOSC = 0b001)
    __builtin_write_OSCCONL(0x01);

//    // Configure PLL for Fosc = 140MHz/Fcy = 70MIPS using 8.000 MHz Primary Oscilator (make sure to set Configuration Fuse to enable the OSC1/OSC2 oscillator)
//    CLKDIV = 0xB000; // ROI = 1, DOZE = 8:1, FRCDIV = 1:1, PLLPOST = 2:1, PLLPRE = 2:1
//    PLLFBD = (FCY * 2u * 2u * 2u + 8000000u / 2u) / 8000000u - 2u; // 68 @ 70 MIPS (8.000 MHz input clock)
//    __builtin_write_OSCCONH(0x03);      // Initiate Clock Switch to use the Primary Oscillator with PLL (NOSC = 0b011)
//    __builtin_write_OSCCONL(0x01);


    // Set push buttons as GPIO inputs
    //
    // Function     Explorer 16 PIM Header      dsPIC33EP256GP506 Device Pins
    // Button       PIM#, PICtail#, name        PIC#, name
    // S4 (LSb)     80, 106, RD13               53, RD5
    // S5           92,  74, RA7                52, RP56/RC8             <- Pin function is muxed with LED D10; S5 button shouldn't be used because LED clamps weak 10k pull up voltage too low
    // S6           84, 100, RD7                51, RP55/RC7
    // S3 (MSb)     83,  99, RD6                54, RD6
    TRISDbits.TRISD5 = 1;
    TRISCbits.TRISC8 = 1;
    TRISCbits.TRISC7 = 1;
    TRISDbits.TRISD6 = 1;


    // Set LED pins as GPIO outputs
    //
    // Function     Explorer 16 PIM Header      dsPIC33EP256GP506 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             21, AN6/OA3OUT/C4IN1+/OCFB/RC0
    // D4           38, 70, RA1/TCK             22, AN7/C3IN1-/C4IN1-/RC1
    // D5           58, 38, RA2/SCL2            32, SCL2/RP36/RB4
    // D6           59, 40, RA3/SDA2            31, SDA2/RPI24/RA8
    // D7           60, 71, RA4/TDI             27, AN12/C2IN2-/U2RTS/BCLK2/RE12
    // D8           61, 72, RA5/TDO             28, AN13/C3IN2-/U2CTS/RE13
    // D9           91, 73, RA6                 29, AN14/RPI94/RE14
    // D10 (MSb)    92, 74, RA7                 52, RP56/RC8             <- Pin function is muxed with button S5; we will use it as an LED output only
    ANSELCbits.ANSC0 = 0;
    ANSELCbits.ANSC1 = 0;
    ANSELEbits.ANSE12 = 0;
    ANSELEbits.ANSE13 = 0;
    ANSELEbits.ANSE14 = 0;
    LATCbits.LATC0 = 0;
    LATCbits.LATC1 = 0;
    LATBbits.LATB4 = 0;
    LATAbits.LATA8 = 0;
    LATEbits.LATE12 = 0;
    LATEbits.LATE13 = 0;
    LATEbits.LATE14 = 0;
    LATCbits.LATC8 = 0;
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;
    TRISBbits.TRISB4 = 0;
    TRISAbits.TRISA8 = 0;
    TRISEbits.TRISE12 = 0;
    TRISEbits.TRISE13 = 0;
    TRISEbits.TRISE14 = 0;
    TRISCbits.TRISC8 = 0;


    // Set 16x2 character LCD pins as GPIO outputs
    //
    // Function     Explorer 16 PIM Header      dsPIC33EP256GP506 Device Pins
    // 16x2 LCD     PIM#, PICtail#, name        PIC#, name
    // Data 0        93, 109, RE0/PMPD0          48, TCK/CVREF1O/ASCL1/RP40/T4CK/RB8
    // Data 1        94, 110, RE1/PMPD1          49, TMS/ASDA1/RP41/RB9
    // Data 2        98, 111, RE2/PMPD2          60, RP42/RB10
    // Data 3        99, 112, RE3/PMPD3          61, RP43/RB11
    // Data 4       100, 113, RE4/PMPD4          62, RPI44/RB12
    // Data 5         3, 114, RE5/PMPD5          63, RPI45/CTPLS/RB13
    // Data 6         4, 115, RE6/PMPD6           2, RPI46/T3CK/RB14
    // Data 7         5, 116, RE7/PMPD7           3, RPI47/T5CK/RB15
    // E (Enable)    81,  97, RD4/PMPWR          50, RP54/RC6
    // R/!W          82,  98, RD5/PMPRD          47, RC13
    // RS (Reg Sel)  44,  84, RB15/PMPA0         64, TDO/RA10
    LATB   &= ~(1<<8 | 1<<9 | 1<<10 | 1<<11 | 1<<12 | 1<<13 | 1<<14 | 1<<15);
    TRISB  &= ~(1<<8 | 1<<9 | 1<<10 | 1<<11 | 1<<12 | 1<<13 | 1<<14 | 1<<15);
    LATCbits.LATC6 = 0;
    LATCbits.LATC13 = 0;
    LATAbits.LATA10 = 0;
    TRISCbits.TRISC6 = 0;
    TRISCbits.TRISC13 = 0;
    TRISAbits.TRISA10 = 0;


    // Configure UART2 pins as UART.
    // - Pin names are with respect to the PIC.
    // - Outputs bits in TRIS registers are all set as inputs because the PPS or
    //   UART2 hardware overrides it.
    //
    // Function     Explorer 16 PIM Header      dsPIC33EP256GP506 Device Pins
    // UART2        PIM#, PICtail#, name        PIC#, name
    // U2TX  (out)  50, 36, RF5/PMPA8/U2TX      43, PGED2/ASDA2/RP37/RB5
    // U2RTS (out)  39, 52, RF13/U2RTS          23, AN8/C3IN1+/U1RTS/BCLK1/RC2
    // U2RX  (in)   49, 34, RF4/PMPA9/U2RX      44, PGEC2/ASCL2/RP38/RB6
    // U2CTS (in)   40, 51, RF12/U2CTS          24, AN11/C1IN2-/U1CTS/RC11
    TRISBbits.TRISB5 = 1;
    TRISCbits.TRISC2 = 1;
    TRISBbits.TRISB6 = 1;
    TRISCbits.TRISC11 = 1;
    ANSELCbits.ANSC2 = 0;
    ANSELCbits.ANSC11 = 0;
    CNPDCbits.CNPDC2 = 1;   // Turn on pull-down on U2RTS output in case if we don't enable this output in the actual UART module
    RPINR19bits.U2RXR = 38; // U2RX on RP38
    RPOR1bits.RP37R = 0x03; // U2TX on RP37
    __C30_UART = 2;         // printf() on UART2 since it has the DB9 connector and MAX3232 transceiver


    // Configure pins for 25LC256 (32Kbyte SPI EEPROM). 
    // - Pin names are with respect to the PIC, which is the SPI Master.
    // - Outputs bits in TRIS registers are all set as inputs because the PPS or
    //   SPI hardware overrides it.
    //
    // Function     Explorer 16 PIM Header      dsPIC33EP256GP506 Device Pins
    // SPI2         PIM#, PICtail#, name        PIC#, name
    // !CS   (out)  79, 105, RD12                8, RPI121/RG9
    // SCK2  (out)  10,  35, RG6/PMPA5/SCK2      4, RP118/RG6
    // SDI2  (in)   11,  37, RG7/PMPA4/SDI2      5, RPI119/RG7
    // SDO2  (out)  12,  39, RG8/PMPA3/SDO2      6, RP120/RG8
    LATGbits.LATG9 = 1;
    TRISGbits.TRISG9 = 0;
    TRISGbits.TRISG6 = 1;
    TRISGbits.TRISG7 = 1;
    TRISGbits.TRISG8 = 1;
    CNPDGbits.CNPDG7 = 1;       // Turn on pull down on SDI2 so it doesn't float when SPI module tri-states it
    RPINR22bits.SDI2R = 119;    // SDI2 on RPI119
    RPOR8bits.RP118R = 0x09;    // SCK2 on RP118
    RPOR9bits.RP120R = 0x08;    // SDO2 on RP120


    // Configure Pins for SPI1 on Explorer 16 (nothing attached on the Explorer
    // 16 directly, but useful for various PICtails).
    // - Pin names are with respect to the PIC, which is the SPI Master.
    // - Outputs bits in TRIS registers are all set as inputs because the PPS or
    //   SPI hardware overrides it.
    //
    // Function     Explorer 16 PIM Header      dsPIC33EP256GP506 Device Pins
    // SPI1         PIM#, PICtail#, name        PIC#, name
    // !CS   (out)  23,   1, RB2/SS1/AN2        59, RP97/RF1
    // SCK1  (out)  55,   3, RF6/SCK1           35, SCK1/RPI51/RC3
    // SDI1  (in)   54,   5, RF7/SDI1           34, SDI1/RPI25/RA9
    // SDO1  (out)  53,   7, RF8/SDO1           33, CVREF2O/SDO1/RP20/T1CK/RA4
    LATFbits.LATF1 = 1;
    TRISFbits.TRISF1 = 0;
    TRISCbits.TRISC3 = 1;
    TRISAbits.TRISA9 = 1;
    TRISAbits.TRISA4 = 1;
    CNPDAbits.CNPDA9 = 1;   // Turn on SDI1 pull down so it doesn't float when slave chip is inactive


    // Configure Analog Inputs for U4 TC1047A Temperature Sensor and R6 10K Potentiometer
    //
    // Function     Explorer 16 PIM Header      dsPIC33EP256GP506 Device Pins
    // Analog Input PIM#, PICtail#, name        PIC#, name
    // TC1047A Temp 21, 14, RB4/AN4             14, AN1/C2IN1+/RA1
    // 10K Pot      20, 77, RB5/AN5             13, AN0/OA2OUT/RA0
    ANSELAbits.ANSA1 = 1;
    ANSELAbits.ANSA0 = 1;


    // Report 70 MIPS on dsPIC33E
    return FCY;
}


/**
 *  @function LEDSet Turns on or off all LEDs according to the supplied
 *                  bitfield.
 *  @param ledBitField A bitmask representing an array of LEDs that should be
 *                     turned on and off. Each '1' bit in the bit field turns
 *                     the corresponding LED on while each '0' turns the LED
 *                      off.
 *  @return Bitmask representing the LEDs that were previously on before setting
 *          the new value. Bits for LEDs that do not exist are returned as '0'
 *          values in the bitmask.
 */
unsigned int LEDSet(unsigned int ledBitField)
{
    unsigned int ret;

    // Function     Explorer 16 PIM Header      dsPIC33EP256GP506 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             21, AN6/OA3OUT/C4IN1+/OCFB/RC0
    // D4           38, 70, RA1/TCK             22, AN7/C3IN1-/C4IN1-/RC1
    // D5           58, 38, RA2/SCL2            32, SCL2/RP36/RB4
    // D6           59, 40, RA3/SDA2            31, SDA2/RPI24/RA8
    // D7           60, 71, RA4/TDI             27, AN12/C2IN2-/U2RTS/BCLK2/RE12
    // D8           61, 72, RA5/TDO             28, AN13/C3IN2-/U2CTS/RE13
    // D9           91, 73, RA6                 29, AN14/RPI94/RE14
    // D10 (MSb)    92, 74, RA7                 52, RP56/RC8             <- Pin function is muxed with button S5; we will use it as an LED output only

    __asm__ ("    clr   w2          ; Start with all bits clear \n"
             "    btsc  LATC, #0    ;   Read a bit to see if the LED is on\n"
             "    bset  w2, #0      ;   If bit was set, set this other output bit\n"
             "    btsc  LATC, #1    \n"
             "    bset  w2, #1      \n"
             "    btsc  LATB, #4    \n"
             "    bset  w2, #2      \n"
             "    btsc  LATA, #8    \n"
             "    bset  w2, #3      \n"
             "    btsc  LATE, #12   \n"
             "    bset  w2, #4      \n"
             "    btsc  LATE, #13   \n"
             "    bset  w2, #5      \n"
             "    btsc  LATE, #14   \n"
             "    bset  w2, #6      \n"
             "    btsc  LATC, #8    \n"
             "    bset  w2, #7      \n"
             "    mov   w2, %0      ; Store the full result\n"
             "                      \n"
             "    ; Write output latches\n"
             "    mov   %0, w3      ; Get a copy of ledBitField into a local wreg\n"
             "    xor   w2, w3, w2  ;   XOR the current values with bit field so we can figure out which bits need toggling\n"
             "    btsc  w2, #0      ; Read a bit to see if the LED should toggle\n"
             "    btg   LATC, #0    ; If bit was set, toggle the latch bit\n"
             "    btsc  w2, #1      \n"
             "    btg   LATC, #1    \n"
             "    btsc  w2, #2      \n"
             "    btg   LATB, #4    \n"
             "    btsc  w2, #3      \n"
             "    btg   LATA, #8    \n"
             "    btsc  w2, #4      \n"
             "    btg   LATE, #12   \n"
             "    btsc  w2, #5      \n"
             "    btg   LATE, #13   \n"
             "    btsc  w2, #6      \n"
             "    btg   LATE, #14   \n"
             "    btsc  w2, #7      \n"
             "    btg   LATC, #8    \n"
            : "=g"(ret) : "g"(ledBitField) : "w2", "w3", "cc");  // "ret" output, ledBitField input, clobbers w2, w3 registers + condition codes

    return ret;
}


/**
 *  @function LEDOn Turns on an array of LEDs using a logical OR of the
 *                  ledBitField provided. I.e. only set bits turn on LEDs, clear
 *                  bits have no effect.
 *  @param ledBitField A bitmask representing an array of LEDs that should be 
 *                     turned on. Each '1' bit in the bit field turns the
 *                     corresponding LED on.
 *  @return Bitmask representing the LEDs that were previously on before ORing
 *          in the ledBitField value. Bits for LEDs that do not exist are
 *          returned as '0' values in the bitmask.
 */
unsigned int LEDOn(unsigned int ledBitField)
{
    unsigned int ret;

    // Function     Explorer 16 PIM Header      dsPIC33EP256GP506 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             21, AN6/OA3OUT/C4IN1+/OCFB/RC0
    // D4           38, 70, RA1/TCK             22, AN7/C3IN1-/C4IN1-/RC1
    // D5           58, 38, RA2/SCL2            32, SCL2/RP36/RB4
    // D6           59, 40, RA3/SDA2            31, SDA2/RPI24/RA8
    // D7           60, 71, RA4/TDI             27, AN12/C2IN2-/U2RTS/BCLK2/RE12
    // D8           61, 72, RA5/TDO             28, AN13/C3IN2-/U2CTS/RE13
    // D9           91, 73, RA6                 29, AN14/RPI94/RE14
    // D10 (MSb)    92, 74, RA7                 52, RP56/RC8             <- Pin function is muxed with button S5; we will use it as an LED output only

    __asm__ ("    clr   w2          ; Start with all bits clear \n"
             "    btsc  LATC, #0    ;   Read a bit to see if the LED is on\n"
             "    bset  w2, #0      ;   If bit was set, set this other output bit\n"
             "    btsc  LATC, #1    \n"
             "    bset  w2, #1      \n"
             "    btsc  LATB, #4    \n"
             "    bset  w2, #2      \n"
             "    btsc  LATA, #8    \n"
             "    bset  w2, #3      \n"
             "    btsc  LATE, #12   \n"
             "    bset  w2, #4      \n"
             "    btsc  LATE, #13   \n"
             "    bset  w2, #5      \n"
             "    btsc  LATE, #14   \n"
             "    bset  w2, #6      \n"
             "    btsc  LATC, #8    \n"
             "    bset  w2, #7      \n"
             "    mov   w2, %0      ; Store the full result\n"
            : "=g"(ret) : : "w2");  // Write output to "ret" variable, no inputs, clobbers w2 register

    __asm__ ("    mov   %0, w2      ; Get a copy of ledBitField into a local wreg\n"
             "    btsc  w2, #0      ; Read a bit to see if the LED should turn on\n"
             "    bset  LATC, #0    ; If bit was set, set the latch bit\n"
             "    btsc  w2, #1      \n"
             "    bset  LATC, #1    \n"
             "    btsc  w2, #2      \n"
             "    bset  LATB, #4    \n"
             "    btsc  w2, #3      \n"
             "    bset  LATA, #8    \n"
             "    btsc  w2, #4      \n"
             "    bset  LATE, #12   \n"
             "    btsc  w2, #5      \n"
             "    bset  LATE, #13   \n"
             "    btsc  w2, #6      \n"
             "    bset  LATE, #14   \n"
             "    btsc  w2, #7      \n"
             "    bset  LATC, #8    \n"
            : : "g"(ledBitField) : "w2");  // No output, ledBitField as input, clobbers w2 register
    
    return ret;
}


/**
 *  @function LEDOff Turns off an array of LEDs using a logical NOT-AND of the
 *                   ledBitField provided. I.e. only set bits turn LEDs off,
 *                   clear bits have no effect.
 *  @param ledBitField A bitmask representing an array of LEDs that should be
 *                     turned off. Each '1' bit in the bit field turns the
 *                     corresponding LED off.
 *  @return Bitmask representing the LEDs that were previously off before
 *          NOT-ANDing in the ledBitField value. Bits for LEDs that do not exist
 *          are returned as '1' values in the bitmask.
 */
unsigned int LEDOff(unsigned int ledBitField)
{
    unsigned int ret;

    // Function     Explorer 16 PIM Header      dsPIC33EP256GP506 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             21, AN6/OA3OUT/C4IN1+/OCFB/RC0
    // D4           38, 70, RA1/TCK             22, AN7/C3IN1-/C4IN1-/RC1
    // D5           58, 38, RA2/SCL2            32, SCL2/RP36/RB4
    // D6           59, 40, RA3/SDA2            31, SDA2/RPI24/RA8
    // D7           60, 71, RA4/TDI             27, AN12/C2IN2-/U2RTS/BCLK2/RE12
    // D8           61, 72, RA5/TDO             28, AN13/C3IN2-/U2CTS/RE13
    // D9           91, 73, RA6                 29, AN14/RPI94/RE14
    // D10 (MSb)    92, 74, RA7                 52, RP56/RC8             <- Pin function is muxed with button S5; we will use it as an LED output only

    __asm__ ("    clr   w2          ; Start with all bits clear \n"
             "    btss  LATC, #0    ;   Read a bit to see if the LED is off\n"
             "    bset  w2, #0      ;   If bit was clear, set this other output bit\n"
             "    btss  LATC, #1    \n"
             "    bset  w2, #1      \n"
             "    btss  LATB, #4    \n"
             "    bset  w2, #2      \n"
             "    btss  LATA, #8    \n"
             "    bset  w2, #3      \n"
             "    btss  LATE, #12   \n"
             "    bset  w2, #4      \n"
             "    btss  LATE, #13   \n"
             "    bset  w2, #5      \n"
             "    btss  LATE, #14   \n"
             "    bset  w2, #6      \n"
             "    btss  LATC, #8    \n"
             "    bset  w2, #7      \n"
             "    mov   w2, %0      ; Store the full result\n"
            : "=g"(ret) : : "w2");  // Write output to "ret" variable, no inputs, clobbers w2 register

    __asm__ ("    mov   %0, w2      ; Get a copy of ledBitField into a local wreg\n"
             "    btsc  w2, #0      ; Read a bit to see if the LED should be turned off\n"
             "    bclr  LATC, #0    ; If bit was set, clear the latch bit\n"
             "    btsc  w2, #1      \n"
             "    bclr  LATC, #1    \n"
             "    btsc  w2, #2      \n"
             "    bclr  LATB, #4    \n"
             "    btsc  w2, #3      \n"
             "    bclr  LATA, #8    \n"
             "    btsc  w2, #4      \n"
             "    bclr  LATE, #12   \n"
             "    btsc  w2, #5      \n"
             "    bclr  LATE, #13   \n"
             "    btsc  w2, #6      \n"
             "    bclr  LATE, #14   \n"
             "    btsc  w2, #7      \n"
             "    bclr  LATC, #8    \n"
            : : "g"(ledBitField) : "w2");  // No output, ledBitField as input, clobbers w2 register

    return ret;
}


/**
 *  @function LEDToggle Toggles the state of an array of LEDs using a logical
 *                  XOR of the ledBitField provided. I.e. only set bits toggle
 *                  an LED state. Clear bits have no effect.
 *  @param ledBitField A bitmask representing an array of LEDs that should be
 *                     toggled state. Each '1' bit in the bit field toggles the
 *                     corresponding LED on or off.
 *  @return Bitmask representing the LEDs that were previously on before
 *          XORing in the ledBitField value. Bits for LEDs that do not exist are
 *          returned as '0' values in the bitmask.
 */
unsigned int LEDToggle(unsigned int ledBitField)
{
    unsigned int ret;

    // Function     Explorer 16 PIM Header      dsPIC33EP256GP506 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             21, AN6/OA3OUT/C4IN1+/OCFB/RC0
    // D4           38, 70, RA1/TCK             22, AN7/C3IN1-/C4IN1-/RC1
    // D5           58, 38, RA2/SCL2            32, SCL2/RP36/RB4
    // D6           59, 40, RA3/SDA2            31, SDA2/RPI24/RA8
    // D7           60, 71, RA4/TDI             27, AN12/C2IN2-/U2RTS/BCLK2/RE12
    // D8           61, 72, RA5/TDO             28, AN13/C3IN2-/U2CTS/RE13
    // D9           91, 73, RA6                 29, AN14/RPI94/RE14
    // D10 (MSb)    92, 74, RA7                 52, RP56/RC8             <- Pin function is muxed with button S5; we will use it as an LED output only

    __asm__ ("    clr   w2          ; Start with all bits clear \n"
             "    btsc  LATC, #0    ;   Read a bit to see if the LED is on\n"
             "    bset  w2, #0      ;   If bit was set, set this other output bit\n"
             "    btsc  LATC, #1    \n"
             "    bset  w2, #1      \n"
             "    btsc  LATB, #4    \n"
             "    bset  w2, #2      \n"
             "    btsc  LATA, #8    \n"
             "    bset  w2, #3      \n"
             "    btsc  LATE, #12   \n"
             "    bset  w2, #4      \n"
             "    btsc  LATE, #13   \n"
             "    bset  w2, #5      \n"
             "    btsc  LATE, #14   \n"
             "    bset  w2, #6      \n"
             "    btsc  LATC, #8    \n"
             "    bset  w2, #7      \n"
             "    mov   w2, %0      ; Store the full result\n"
            : "=g"(ret) : : "w2");  // Write output to "ret" variable, no inputs, clobbers w2 register

    __asm__ ("    mov   %0, w2      ; Get a copy of ledBitField into a local wreg\n"
             "    btsc  w2, #0      ; Read a bit to see if the LED should toggle\n"
             "    btg   LATC, #0    ; If bit was set, toggle the latch bit\n"
             "    btsc  w2, #1      \n"
             "    btg   LATC, #1    \n"
             "    btsc  w2, #2      \n"
             "    btg   LATB, #4    \n"
             "    btsc  w2, #3      \n"
             "    btg   LATA, #8    \n"
             "    btsc  w2, #4      \n"
             "    btg   LATE, #12   \n"
             "    btsc  w2, #5      \n"
             "    btg   LATE, #13   \n"
             "    btsc  w2, #6      \n"
             "    btg   LATE, #14   \n"
             "    btsc  w2, #7      \n"
             "    btg   LATC, #8    \n"
            : : "g"(ledBitField) : "w2");  // No output, ledBitField as input, clobbers w2 register

    return ret;
}



/**
 *  @function ButtonPeek Reads the state of all buttons without updating any
 *                       internal state. ButtonsLastState, ButtonsPushed,
 *                       ButtonsReleased, and ButtonsChanged are unaffected.
 *  @return Bitmask representing the buttons that are currently pushed ('1' =
 *          pushed, '0' = released). Button positions that do not exist are
 *          returned as '0' or released in the bitmask.
 */
unsigned int ButtonPeek(void)
{
    unsigned int ret;

    // Function     Explorer 16 PIM Header      dsPIC33EP256GP506 Device Pins
    // Button       PIM#, PICtail#, name        PIC#, name
    // S4 (LSb)     80, 106, RD13               53, RD5
    // S5           92,  74, RA7                52, RP56/RC8             <- Pin function is muxed with LED D10; S5 button shouldn't be used because LED clamps weak 10k pull up voltage too low
    // S6           84, 100, RD7                51, RP55/RC7
    // S3 (MSb)     83,  99, RD6                54, RD6

    //      Port  &    Pin      Pin- BUTTONx
    ret  = (PORTD & (1<<5))  >> (5 - 0);
    ret |= (PORTC & (1<<7))  >> (7 - 2);
    ret |= (PORTD & (1<<6))  >> (6 - 3);

    ret  = ~ret;            // Invert logic since depressed is measured as '0' while released has a pull-up on it and returns '1'.
    ret &=  0xD;            // Mask off buttons that don't exist

    return ret;
}


/**
 *  @function ButtonRead
 *      Reads the state of all buttons and updates the values of the
 *      ButtonsLastState ButtonsToggled, ButtonsPushed, and ButtonsReleased
 *      global bitmask variables (also ButtonsLastState). Each '1' bit in the
 *      ButtonsToggled/Pushed/Released variables represents an event. For
 *      example if ButtonRead() indicates a button that was previously released
 *      is now pushed ButtonsToggled and ButtonsPushed will have a '1' stored in
 *      the corresponding button bitmask position. '0' will be returned in all
 *      variables if ButtonRead() is subsequently called and the user is still
 *      holding down the button. Similarly, upon button release, ButtonsToggled
 *      and ButtonsReleased will have a '1' stored in the corresponding button
 *      bitmask position. These '1's will revert to '0's when ButtonRead() is
 *      called again
 *  @return Bitmask representing the buttons that are currently pushed ('1' =
 *          pushed, '0' = released). Button positions that do not exist are
 *          returned as '0' in the bitmask. If you only want to read the current
 *          button state without altering the ButtonsLastState tracking and
 *          ButtonsToggled, ButtonsPushed, and ButtonsReleased values, then do
 *          not call this function. Instead, call the ButtonPeek() function.
 */
unsigned int ButtonsLastState = 0;
unsigned int ButtonsToggled = 0;
unsigned int ButtonsPushed = 0;
unsigned int ButtonsReleased = 0;
unsigned int ButtonRead(void)
{
    unsigned int ret;

    // Get immediate current button state
    ret = ButtonPeek();

    // Compute toggled, released, and pushed values
    ButtonsToggled = (ButtonsLastState ^ ret);
    ButtonsPushed = ButtonsToggled & ~ret;
    ButtonsReleased = ButtonsToggled & ret;
    
    // Save immediate button state to last state for future Toggle/Push/Release
    // event tracking
    ButtonsLastState = ret; // Save value for other API event checking return values

    return ret;
}

#endif //#if defined(__dsPIC33EP512GP506__) || defined(__dsPIC33EP256GP506__) || defined(__dsPIC33EP128GP506__) || defined(__dsPIC33EP64GP506__) || defined(__dsPIC33EP32GP506__) || defined(__dsPIC33EP512MC506__) || defined(__dsPIC33EP256MC506__) || defined(__dsPIC33EP128MC506__) || defined(__dsPIC33EP64MC506__) || defined(__dsPIC33EP32MC506__)

