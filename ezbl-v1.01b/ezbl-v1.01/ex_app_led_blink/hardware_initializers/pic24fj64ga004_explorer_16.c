/*
 * File:   pic24fj64ga004_explorer_16.c
 *
 * Created on August 28, 2014
 *
 * Initializes the device configuration fuses, clock frequency, UART2 pins,
 * LED I/O, Button I/O, LCD and 25LC256 SPI2 pins for the PIC24FJ64GA004 PIM on
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

#if defined(__PIC24FJ64GA004__) || defined(__PIC24FJ48GA004__) || defined(__PIC24FJ32GA004__) || defined(__PIC24FJ16GA004__)

#define FCY         16000000ul

#include <xc.h>

// Device Configuration Fuses
// These are commented out because they are defined in the bootloader project
// instead. If you want to use these, be sure to remove the applicable
// definition(s) from the bootloader project. Different config words can be
// split (ex: define _CONFIG1 in the bootloader while _CONFIG2 is defined here
// in the application).
//_CONFIG2(POSCMOD_NONE & I2C1SEL_PRI & IOL1WAY_OFF & OSCIOFNC_ON & FCKSM_CSECMD & FNOSC_FRCPLL & SOSCSEL_SOSC & WUTSEL_FST & IESO_OFF)
//_CONFIG1(WDTPS_PS2048 & FWPSA_PR32 & WINDIS_ON & FWDTEN_OFF & ICS_PGx1 & COE_OFF & GWRP_OFF & GCP_OFF & JTAGEN_OFF) // Code Protect = OFF, Write Protect = OFF

extern int __C30_UART;


// @return: FCY clock speed we just configured the processor for
unsigned long InitializeBoard(void)
{
    // Set the CPU clock to maximum 16 MIPS speed
    CLKDIV = 0x0000;    // Set 1:1 8MHz FRC postscalar

    // Set push buttons as GPIO inputs
    //
    // Function     Explorer 16 PIM Header      PIC24FJ64GA004 Device Pins
    // Button       PIM#, PICtail#, name        PIC#, name
    // S4 (LSb)     80, 106, RD13               13, PMPA7/RA7/TCK               // Muxed with LED D4  (LED bit 1) -> Configured for button use
    // S5           92,  74, RA7                 2, RP22/CN18/PMPA1/RC6         // Muxed with LED D10 (LED bit 7) -> Configured for button use
    // S6           84, 100, RD7                35, PMPA9/RA9/TDI               // Muxed with LED D7  (LED bit 4) -> Configured for button use
    // S3 (MSb)     83,  99, RD6                12, PMPA10/RA10/TMS             // Muxed with LED D3  (LED bit 0) -> Configured for button use
    TRISA |= (1<<7 | 1<<1 | 1<<9 | 1<<10);


    // Set LED pins as GPIO outputs
    //
    // Function     Explorer 16 PIM Header      PIC24FJ64GA004 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             12, PMPA10/RA10/TMS             // Muxed with Button S3 (Button bit 3) ->  Configured for Button Use
    // D4           38, 70, RA1/TCK             13, PMPA7/RA7/TCK               // Muxed with Button S4 (Button bit 0) ->  Configured for Button Use
    // D5           58, 38, RA2/SCL2            44, RP8/SCL1/CN22/PMPD4/RB8     // Muxed with LCD Data 4
    // D6           59, 40, RA3/SDA2             1, RP9/SDA1/CN21/PMPD3/RB9     // Muxed with LCD Data 3
    // D7           60, 71, RA4/TDI             35, PMPA9/RA9/TDI               // Muxed with Button S6 (Button bit 2) ->  Configured for Button Use
    // D8           61, 72, RA5/TDO             32, PMPA8/RA8/TDO               // Muxed with SPI2 25LC256 Chip Select ->  Both outputs so can coexist, but do not use if you care about EEPROM contents
    // D9           91, 73, RA6                 10, RP12/CN14/PMPD0/RB12        // Muxed with LCD Data 0
    // D10 (MSb)    92, 74, RA7                  2, RP22/CN18/PMPA1/RC6         // Muxed with Button S5 (Button bit 1) ->  Configured for Button Use
    //LATA   &= ~(1<<10 | 1<<7 | 1<<9 | 1<<8);  // Commented out since we want to use muxed Button functions instead
    //TRISA  &= ~(1<<10 | 1<<7 | 1<<9 | 1<<8);  // Commented out since we want to use muxed Button functions instead
    LATAbits.LATA8 = 1;         // Set to 1 because of SPI2 25LC256 !CS muxing
    TRISAbits.TRISA8 = 0;
    LATBbits.LATB8 = 0;
    LATBbits.LATB9 = 0;
    LATBbits.LATB12 = 0;
    TRISBbits.TRISB8 = 0;
    TRISBbits.TRISB9 = 0;
    TRISBbits.TRISB12 = 0;
    //LATCbits.LATC6 = 0;       // Commented out because we want to use I/O pin for Button S5 (Button bit 1) instead
    //TRISCbits.TRISC6 = 0;     // Commented out because we want to use I/O pin for Button S5 (Button bit 1) instead
    

    // Set 16x2 character LCD pins as GPIO outputs
    //
    // Function     Explorer 16 PIM Header      PIC24FJ64GA004 Device Pins
    // 16x2 LCD     PIM#, PICtail#, name        PIC#, name
    // Data 0        93, 109, RE0/PMPD0          10, RP12/CN14/PMPD0/RB12       // Muxed with LED D9 (LED bit 6), but compatible as both are outputs
    // Data 1        94, 110, RE1/PMPD1           9, RP11/CN15/PMPD1/RB11
    // Data 2        98, 111, RE2/PMPD2           8, RP10/CN16/PMPD2/RB10
    // Data 3        99, 112, RE3/PMPD3           1, RP9/SDA1/CN21/PMPD3/RB9    // Muxed with LED D6 (LED bit 3), but compatible as both are outputs
    // Data 4       100, 113, RE4/PMPD4          44, RP8/SCL1/CN22/PMPD4/RB8    // Muxed with LED D5 (LED bit 2), but compatible as both are outputs
    // Data 5         3, 114, RE5/PMPD5          43, RP7/INT0/CN23/PMPD5/RB7
    // Data 6         4, 115, RE6/PMPD6          42, RP6/ASCL1/CN24/PMPD6/RB6
    // Data 7         5, 116, RE7/PMPD7          41, RP5/ASDA1/CN27/PMPD7/RB5
    // E (Enable)    81,  97, RD4/PMPWR          14, AN10/CVREF/RTCC/RP14/CN12/PMPWR/RB14
    // R/!W          82,  98, RD5/PMPRD          11, RP13/CN13/PMPRD/RB13
    // RS (Reg Sel)  44,  84, RB15/PMPA0          3, RP23/CN17/PMPA0/RC4
    LATB   &= ~(1<<12 | 1<<11 | 1<<10 | 1<<9 | 1<<8 | 1<<7 | 1<<6 | 1<<5 | 1<<14 | 1<<13);
    TRISB  &= ~(1<<12 | 1<<11 | 1<<10 | 1<<9 | 1<<8 | 1<<7 | 1<<6 | 1<<5 | 1<<14 | 1<<13);
    LATCbits.LATC4 = 0;
    TRISCbits.TRISC4 = 0;
    AD1PCFGbits.PCFG10 = 0;    // Disable analog input function on AN10/CVREF/RTCC/RP14/CN12/PMPWR/RB14


    // Configure UART2 pins as UART.
    // - Pin names are with respect to the PIC.
    // - Outputs bits in TRIS registers are all set as inputs because the PPS or
    //   UART2 hardware overrides it.
    //
    // Function     Explorer 16 PIM Header      PIC24FJ64GA004 Device Pins
    // UART2        PIM#, PICtail#, name        PIC#, name
    // U2TX  (out)  50, 36, RF5/PMPA8/U2TX       5, RP25/CN19/PMPA6/RC9
    // U2RTS (out)  39, 52, RF13/U2RTS          38, RP21/CN26/PMPA3/RC5         // Muxed with SPI2 SDO output to 25LC256 -> Let's configure for SPI2 SDO function
    // U2RX  (in)   49, 34, RF4/PMPA9/U2RX      36, RP19/CN28/PMPBE/RC3
    // U2CTS (in)   40, 51, RF12/U2CTS          37, RP20/CN25/PMPA4/RC4         // Muxed with SPI2 SDI input from 25LC256 -> Let's configure for SPI2 SDI function
    TRISC  |=  (1<<9 | 1<<5 | 1<<3 | 1<<4);
    LATCbits.LATC5 = 1;         // Initialize to drive high (if we don't enable this output in the actual UART2 or SPI2 modules, and elsewhere someone clears the TRISC5 bit)
    RPINR19bits.U2RXR  = 19;    // U2RX on RP19
    RPOR12bits.RP25R   = 5;     // U2TX on RP25
    //RPOR10bits.RP21R   = 6;     // U2RTS on RP21, but let's comment this out so the SPI2 module can use this I/O pin instead
    //RPINR19bits.U2CTSR = 20;    // U2CTS on RP20, but let's comment this out so the SPI2 module can use this I/O pin instead
    __C30_UART = 2;             // printf() on UART2 since it has the DB9 connector and MAX3232 transceiver


    // Configure pins for 25LC256 (32Kbyte SPI EEPROM). 
    // - Pin names are with respect to the PIC, which is the SPI Master.
    // - Outputs bits in TRIS registers are all set as inputs because the PPS or
    //   SPI hardware overrides it.
    //
    // Function     Explorer 16 PIM Header      PIC24FJ64GA004 Device Pins
    // SPI2         PIM#, PICtail#, name        PIC#, name
    // !CS   (out)  79, 105, RD12               32, PMPA8/RA8/TDO               // Muxed with LED D8 (LED bit 5), compatible as both are outputs
    // SCK2  (out)  10,  35, RG6/PMPA5/SCK2      4, RP24/CN20/PMPA5/RC8
    // SDI2  (in)   11,  37, RG7/PMPA4/SDI2     37, RP20/CN25/PMPA4/RC4         // Muxed with UART2 CTS input
    // SDO2  (out)  12,  39, RG8/PMPA3/SDO2     38, RP21/CN26/PMPA3/RC5         // Muxed with UART2 RTS output
    LATAbits.LATA8    = 1;      // 1 is inactive
    TRISAbits.TRISA8  = 0;      // !CS on RA8
    TRISC            |= (1<<8 | 1<<4 | 1<<5);
    CNPU2bits.CN25PUE = 1;      // Turn on pull up on SDI2 so it doesn't float when SPI module tri-states it
    RPINR22bits.SDI2R = 20;     // SDI2 on RP20
    RPOR12bits.RP24R  = 11;     // SCK2 on RP24
    RPOR10bits.RP21R  = 10;     // SDO2 on RP21


    // Configure Analog Inputs for U4 TC1047A Temperature Sensor and R6 10K Potentiometer
    //
    // Function     Explorer 16 PIM Header      PIC24FJ64GA004 Device Pins
    // Analog Input PIM#, PICtail#, name        PIC#, name
    // TC1047A Temp 21, 14, RB4/AN4             25, AN6/RP16/CN8/RC0
    // 10K Pot      20, 77, RB5/AN5             26, AN7/RP17/CN9/RC1
    AD1PCFGbits.PCFG6 = 1;
    AD1PCFGbits.PCFG7 = 1;


    // Report 16 MIPS on PIC24F
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
    unsigned int ret = 0;

    // Function     Explorer 16 PIM Header      PIC24FJ64GA004 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             12, PMPA10/RA10/TMS             // Muxed with Button S3 (Button bit 3) ->  Configured for Button Use
    // D4           38, 70, RA1/TCK             13, PMPA7/RA7/TCK               // Muxed with Button S4 (Button bit 0) ->  Configured for Button Use
    // D5           58, 38, RA2/SCL2            44, RP8/SCL1/CN22/PMPD4/RB8     // Muxed with LCD Data 4
    // D6           59, 40, RA3/SDA2             1, RP9/SDA1/CN21/PMPD3/RB9     // Muxed with LCD Data 3
    // D7           60, 71, RA4/TDI             35, PMPA9/RA9/TDI               // Muxed with Button S6 (Button bit 2) ->  Configured for Button Use
    // D8           61, 72, RA5/TDO             32, PMPA8/RA8/TDO               // Muxed with SPI2 25LC256 Chip Select ->  Both outputs so can coexist, but do not use if you care about EEPROM contents
    // D9           91, 73, RA6                 10, RP12/CN14/PMPD0/RB12        // Muxed with LCD Data 0
    // D10 (MSb)    92, 74, RA7                  2, RP22/CN18/PMPA1/RC6         // Muxed with Button S5 (Button bit 1) ->  Configured for Button Use
    if(LATBbits.LATB8)
        ret |= 0x1;
    if(LATBbits.LATB9)
        ret |= 0x2;
    if(LATAbits.LATA8)
        ret |= 0x4;
    if(LATBbits.LATB12)
        ret |= 0x8;

    LATBbits.LATB8  = (ledBitField & 0x1) ? 1 : 0;
    LATBbits.LATB9  = (ledBitField & 0x2) ? 1 : 0;
    LATAbits.LATA8  = (ledBitField & 0x4) ? 1 : 0;
    LATBbits.LATB12 = (ledBitField & 0x8) ? 1 : 0;

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
    unsigned int ret = 0;

    // Function     Explorer 16 PIM Header      PIC24FJ64GA004 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             12, PMPA10/RA10/TMS             // Muxed with Button S3 (Button bit 3) ->  Configured for Button Use
    // D4           38, 70, RA1/TCK             13, PMPA7/RA7/TCK               // Muxed with Button S4 (Button bit 0) ->  Configured for Button Use
    // D5           58, 38, RA2/SCL2            44, RP8/SCL1/CN22/PMPD4/RB8     // Muxed with LCD Data 4
    // D6           59, 40, RA3/SDA2             1, RP9/SDA1/CN21/PMPD3/RB9     // Muxed with LCD Data 3
    // D7           60, 71, RA4/TDI             35, PMPA9/RA9/TDI               // Muxed with Button S6 (Button bit 2) ->  Configured for Button Use
    // D8           61, 72, RA5/TDO             32, PMPA8/RA8/TDO               // Muxed with SPI2 25LC256 Chip Select ->  Both outputs so can coexist, but do not use if you care about EEPROM contents
    // D9           91, 73, RA6                 10, RP12/CN14/PMPD0/RB12        // Muxed with LCD Data 0
    // D10 (MSb)    92, 74, RA7                  2, RP22/CN18/PMPA1/RC6         // Muxed with Button S5 (Button bit 1) ->  Configured for Button Use
    if(LATBbits.LATB8)
        ret |= 0x1;
    if(LATBbits.LATB9)
        ret |= 0x2;
    if(LATAbits.LATA8)
        ret |= 0x4;
    if(LATBbits.LATB12)
        ret |= 0x8;

    if(ledBitField & 0x1)
        LATBbits.LATB8  = 1;
    if(ledBitField & 0x2)
        LATBbits.LATB9  = 1;
    if(ledBitField & 0x4)
        LATAbits.LATA8  = 1;
    if(ledBitField & 0x8)
        LATBbits.LATB12 = 1;
    
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
    unsigned int ret = 0;

    // Function     Explorer 16 PIM Header      PIC24FJ64GA004 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             12, PMPA10/RA10/TMS             // Muxed with Button S3 (Button bit 3) ->  Configured for Button Use
    // D4           38, 70, RA1/TCK             13, PMPA7/RA7/TCK               // Muxed with Button S4 (Button bit 0) ->  Configured for Button Use
    // D5           58, 38, RA2/SCL2            44, RP8/SCL1/CN22/PMPD4/RB8     // Muxed with LCD Data 4
    // D6           59, 40, RA3/SDA2             1, RP9/SDA1/CN21/PMPD3/RB9     // Muxed with LCD Data 3
    // D7           60, 71, RA4/TDI             35, PMPA9/RA9/TDI               // Muxed with Button S6 (Button bit 2) ->  Configured for Button Use
    // D8           61, 72, RA5/TDO             32, PMPA8/RA8/TDO               // Muxed with SPI2 25LC256 Chip Select ->  Both outputs so can coexist, but do not use if you care about EEPROM contents
    // D9           91, 73, RA6                 10, RP12/CN14/PMPD0/RB12        // Muxed with LCD Data 0
    // D10 (MSb)    92, 74, RA7                  2, RP22/CN18/PMPA1/RC6         // Muxed with Button S5 (Button bit 1) ->  Configured for Button Use
    if(!LATBbits.LATB8)
        ret |= 0x1;
    if(!LATBbits.LATB9)
        ret |= 0x2;
    if(!LATAbits.LATA8)
        ret |= 0x4;
    if(!LATBbits.LATB12)
        ret |= 0x8;

    if(ledBitField & 0x1)
        LATBbits.LATB8  = 0;
    if(ledBitField & 0x2)
        LATBbits.LATB9  = 0;
    if(ledBitField & 0x4)
        LATAbits.LATA8  = 0;
    if(ledBitField & 0x8)
        LATBbits.LATB12 = 0;

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
    unsigned int ret = 0;

    // Function     Explorer 16 PIM Header      PIC24FJ64GA004 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             12, PMPA10/RA10/TMS             // Muxed with Button S3 (Button bit 3) ->  Configured for Button Use
    // D4           38, 70, RA1/TCK             13, PMPA7/RA7/TCK               // Muxed with Button S4 (Button bit 0) ->  Configured for Button Use
    // D5           58, 38, RA2/SCL2            44, RP8/SCL1/CN22/PMPD4/RB8     // Muxed with LCD Data 4
    // D6           59, 40, RA3/SDA2             1, RP9/SDA1/CN21/PMPD3/RB9     // Muxed with LCD Data 3
    // D7           60, 71, RA4/TDI             35, PMPA9/RA9/TDI               // Muxed with Button S6 (Button bit 2) ->  Configured for Button Use
    // D8           61, 72, RA5/TDO             32, PMPA8/RA8/TDO               // Muxed with SPI2 25LC256 Chip Select ->  Both outputs so can coexist, but do not use if you care about EEPROM contents
    // D9           91, 73, RA6                 10, RP12/CN14/PMPD0/RB12        // Muxed with LCD Data 0
    // D10 (MSb)    92, 74, RA7                  2, RP22/CN18/PMPA1/RC6         // Muxed with Button S5 (Button bit 1) ->  Configured for Button Use
    if(LATBbits.LATB8)
        ret |= 0x1;
    if(LATBbits.LATB9)
        ret |= 0x2;
    if(LATAbits.LATA8)
        ret |= 0x4;
    if(LATBbits.LATB12)
        ret |= 0x8;

    if(ledBitField & 0x1)
        LATBbits.LATB8  ^= 1;
    if(ledBitField & 0x2)
        LATBbits.LATB9  ^= 1;
    if(ledBitField & 0x4)
        LATAbits.LATA8  ^= 1;
    if(ledBitField & 0x8)
        LATBbits.LATB12 ^= 1;

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

    // Function     Explorer 16 PIM Header      PIC24FJ64GA004 Device Pins
    // Button       PIM#, PICtail#, name        PIC#, name
    // S4 (LSb)     80, 106, RD13               13, PMPA7/RA7/TCK               // Muxed with LED D4  (LED bit 1) -> Configured for button use
    // S5           92,  74, RA7                 2, RP22/CN18/PMPA1/RC6         // Muxed with LED D10 (LED bit 7) -> Configured for button use
    // S6           84, 100, RD7                35, PMPA9/RA9/TDI               // Muxed with LED D7  (LED bit 4) -> Configured for button use
    // S3 (MSb)     83,  99, RD6                12, PMPA10/RA10/TMS             // Muxed with LED D3  (LED bit 0) -> Configured for button use

    //      Port  &    Pin      Pin - BUTTONx
    ret  = (PORTA & (1<<7))  >> (7  - 0);
    ret |= (PORTC & (1<<6))  >> (6  - 1);
    ret |= (PORTA & (1<<9))  >> (9  - 2);
    ret |= (PORTA & (1<<10)) >> (10 - 3);

    ret  = ~ret;            // Invert logic since depressed is measured as '0' while released has a pull-up on it and returns '1'.
    ret &=  0xF;            // Mask off buttons that don't exist

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
#endif //#if defined(__PIC24FJ64GA004__) || defined(__PIC24FJ48GA004__) || defined(__PIC24FJ32GA004__) || defined(__PIC24FJ16GA004__)
