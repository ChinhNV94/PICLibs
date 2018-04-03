/*
 * File:   dspic33fj256gp710a_explorer_16.c
 *
 * Created on October, 9th 2014
 *
 * Initializes the device configuration fuses, clock frequency, UART2 pins,
 * LED I/O, Button I/O, 25LC256 SPI2 pins, and clock frequency for the
 * dsPIC33FJ256GP710A PIM on the Explorer 16 development board.
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

#if defined(__dsPIC33FJ256GP710A__) || defined(__dsPIC33FJ128GP710A__) || defined(__dsPIC33FJ64GP710A__) || defined(__dsPIC33FJ256GP710__) || defined(__dsPIC33FJ128GP710__) || defined(__dsPIC33FJ64GP710__)

#define FCY         40000000ul

#include <xc.h>

// Device Configuration Fuses
_FBS(BWRP_WRPROTECT_OFF & BSS_NO_FLASH & RBS_NO_RAM)                        // No Boot Segment
_FSS(SWRP_WRPROTECT_OFF & SSS_NO_FLASH & RSS_NO_RAM)                        // No Secure Segment
_FGS(GWRP_OFF & GSS_OFF)                                                    // No Code Protect and especially, no Write Protect
_FOSCSEL(FNOSC_FRCPLL & IESO_ON)                                            // Start with FRC, then switch to FRC+PLL on boot up
_FOSC(POSCMD_NONE & OSCIOFNC_ON & FCKSM_CSECMD)                             // No crystal, enable run-time clock switching, disable clock monitor
_FWDT(WDTPOST_PS2048 & WDTPRE_PR32 & PLLKEN_ON & WINDIS_OFF & FWDTEN_OFF)   // ~2.048 second watchdog timeout, software enable/disableable
_FPOR(FPWRT_PWR16)                                                          // 16ms
_FICD(ICS_PGD1 & JTAGEN_OFF)                                                // Use PGD1/PGC1 for debug, disable JTAG


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

    // Configure PLL for Fosc = 80MHz (Fcy = 40MIPS) using 7.37 MHz internal FRC oscillator
    // FOSC = FIN*PLLFBD/PLLPRE/PLLPOST
    // PLLFBD = FOSC*PLLPOST*PLLPRE/FIN; FOSC = FCY * 2
    CLKDIV = 0xB000; // ROI = 1, DOZE = 8:1 (not enabled), FRCDIV = 1:1, PLLPOST = 2:1, PLLPRE = 2:1
    PLLFBD = (FCY * 2u * 2u * 2u + 7370000u / 2u) / 7370000u; // 43 @ 40 MIPS (7.37 MHz input clock from FRC)
    __builtin_write_OSCCONH(0x01);      // Initiate Clock Switch to use the FRC Oscillator + PLL (NOSC = 0b001)
    __builtin_write_OSCCONL(0x01);

//    // Configure PLL for Fosc = 80MHz (Fcy = 40MIPS) using 8.000 MHz Primary Oscilator (make sure to set Configuration Fuse to enable the OSC1/OSC2 oscillator)
//    // Configure PLL for Fosc = 80MHz (Fcy = 40MIPS) using 7.37 MHz internal FRC oscillator
//    // FOSC = FIN*PLLFBD/PLLPRE/PLLPOST
//    // PLLFBD = FOSC*PLLPOST*PLLPRE/FIN; FOSC = FCY * 2
//    CLKDIV = 0xB000; // ROI = 1, DOZE = 8:1 (not enabled), FRCDIV = 1:1, PLLPOST = 2:1, PLLPRE = 2:1
//    PLLFBD = (FCY * 2u * 2u * 2u + 8000000u / 2u) / 8000000u; // 40 @ 40 MIPS (8.000 MHz input clock)
//    __builtin_write_OSCCONH(0x03);      // Initiate Clock Switch to use the Primary Oscillator with PLL (NOSC = 0b011)
//    __builtin_write_OSCCONL(0x01);

    // Set push buttons as GPIO inputs
    //
    // Function     Explorer 16 PIM Header      dsPIC33FJ256GP710A Device Pins
    // Button       PIM#, PICtail#, name        PIC#, name
    // S4 (LSb)     80, 106, RD13               80, IC6/CN19/RD13
    // S5           92,  74, RA7                92, AN23/CN23/RA7                 <- Pin function is muxed with LED D10; S5 button can't be used because LED clamps weak 10k pull up voltage too low
    // S6           84, 100, RD7                84, OC8/CN16/RD7
    // S3 (MSb)     83,  99, RD6                83, OC7/CN15/RD6
    TRISD |= (1<<13 | 1<<7 | 1<<6);
    TRISAbits.TRISA7 = 1;
    AD1PCFGHbits.PCFG23 = 0;


    // Set LED pins as GPIO outputs
    //
    // Function     Explorer 16 PIM Header      dsPIC33FJ256GP710A Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             17, TMS/RA0
    // D4           38, 70, RA1/TCK             38, TCK/RA1
    // D5           58, 38, RA2/SCL2            58, SCL2/RA2
    // D6           59, 40, RA3/SDA2            59, SDA2/RA3
    // D7           60, 71, RA4/TDI             60, TDI/RA4
    // D8           61, 72, RA5/TDO             61, TDO/RA5
    // D9           91, 73, RA6                 91, AN22/CN22/RA6
    // D10 (MSb)    92, 74, RA7                 92, AN23/CN23/RA7         <- Pin function is muxed with button S5; we will use it as an LED output only
    TRISA  &= ~(1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7);
    LATA   &= ~(1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7);
    AD1PCFGHbits.PCFG22 = 0;
    AD1PCFGHbits.PCFG23 = 0;


    // Set 16x2 character LCD pins as GPIO outputs
    //
    // Function     Explorer 16 PIM Header      dsPIC33FJ256GP710A Device Pins
    // 16x2 LCD     PIM#, PICtail#, name        PIC#, name
    // Data 0        93, 109, RE0/PMPD0          93, AN24/RE0
    // Data 1        94, 110, RE1/PMPD1          94, AN25/RE1
    // Data 2        98, 111, RE2/PMPD2          98, AN26/RE2
    // Data 3        99, 112, RE3/PMPD3          99, AN27/RE3
    // Data 4       100, 113, RE4/PMPD4         100, AN28/RE4
    // Data 5         3, 114, RE5/PMPD5           3, AN29/RE5
    // Data 6         4, 115, RE6/PMPD6           4, AN30/RE6
    // Data 7         5, 116, RE7/PMPD7           5, AN31/RE7
    // E (Enable)    81,  97, RD4/PMPWR          81, OC5/CN13/RD4
    // R/!W          82,  98, RD5/PMPRD          82, OC6/CN14/RD5
    // RS (Reg Sel)  44,  84, RB15/PMPA0         44, AN15/OCFB/CN12/RB15
    TRISE    &= ~(1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7);
    LATE     &= ~(1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7);
    AD1PCFGH &= ~(1<<8 | 1<<9 | 1<<10 | 1<<11 | 1<<12 | 1<<13 | 1<<14 | 1<<15);
    TRISD    &= ~(1<<4 | 1<<5);
    LATD     &= ~(1<<4 | 1<<5);
    TRISBbits.TRISB15 = 0;
    LATBbits.LATB15 = 0;
    AD2PCFGLbits.PCFG15 = 0;


    // Configure UART2 pins as UART.
    // - Pin names are with respect to the PIC.
    // - Outputs bits in TRIS registers are all set as inputs because the PPS or
    //   UART2 hardware overrides it.
    //
    // Function     Explorer 16 PIM Header      dsPIC33FJ256GP710A Device Pins
    // UART2        PIM#, PICtail#, name        PIC#, name
    // U2TX  (out)  50, 36, RF5/PMPA8/U2TX      50, U2TX/CN18/RF5
    // U2RTS (out)  39, 52, RF13/U2RTS          39, U2RTS/RF13
    // U2RX  (in)   49, 34, RF4/PMPA9/U2RX      49, U2RX/CN17/RF4
    // U2CTS (in)   40, 51, RF12/U2CTS          40, U2CTS/RF12
    TRISF  |=  (1<<5 | 1<<13 | 1<<4 | 1<<12);
    __C30_UART = 2;             // printf() on UART2 since it has the DB9 connector and MAX3232 transceiver


    // Configure pins for 25LC256 (32Kbyte SPI EEPROM). 
    // - Pin names are with respect to the PIC, which is the SPI Master.
    // - Outputs bits in TRIS registers are all set as inputs because the PPS or
    //   SPI hardware overrides it.
    //
    // Function     Explorer 16 PIM Header      dsPIC33FJ256GP710A Device Pins
    // SPI2         PIM#, PICtail#, name        PIC#, name
    // !CS   (out)  79, 105, RD12               79, IC5/RD12
    // SCK2  (out)  10,  35, RG6/PMPA5/SCK2     10, SCK2/CN8/RG6
    // SDI2  (in)   11,  37, RG7/PMPA4/SDI2     11, SDI2/CN9/RG7
    // SDO2  (out)  12,  39, RG8/PMPA3/SDO2     12, SDO2/CN10/RG8
    TRISDbits.TRISD12 = 0;      // !CS on RD12
    LATDbits.LATD12   = 1;      // 1 is inactive
    TRISG            |= (1<<6 | 1<<7 | 1<<8);
    CNPU1bits.CN9PUE  = 1;      // Turn on pull up on SDI2 so it doesn't float when SPI module tri-states it


    // Configure Analog Inputs for U4 TC1047A Temperature Sensor and R6 10K Potentiometer
    //
    // Function     Explorer 16 PIM Header      dsPIC33FJ256GP710A Device Pins
    // Analog Input PIM#, PICtail#, name        PIC#, name
    // TC1047A Temp 21, 14, RB4/AN4             21, AN4/CN6/RB4
    // 10K Pot      20, 77, RB5/AN5             20, AN5/CN7/RB5
    TRISB    |=  (1<<4 | 1<<5);
    AD1PCFGLbits.PCFG4 = 1;
    AD1PCFGLbits.PCFG5 = 1;


    // Report 40 MIPS on dsPIC33F
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

    // Function     Explorer 16 PIM Header      dsPIC33FJ256GP710A Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             17, TMS/RA0
    // D4           38, 70, RA1/TCK             38, TCK/RA1
    // D5           58, 38, RA2/SCL2            58, SCL2/RA2
    // D6           59, 40, RA3/SDA2            59, SDA2/RA3
    // D7           60, 71, RA4/TDI             60, TDI/RA4
    // D8           61, 72, RA5/TDO             61, TDO/RA5
    // D9           91, 73, RA6                 91, AN22/CN22/RA6
    // D10 (MSb)    92, 74, RA7                 92, AN23/CN23/RA7         <- Pin function is muxed with button S5; we will use it as an LED output only
    ret                        =  LATA & 0x00FF;
    ((unsigned char*)&LATA)[0] = (char)ledBitField;

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

    // Function     Explorer 16 PIM Header      dsPIC33FJ256GP710A Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             17, TMS/RA0
    // D4           38, 70, RA1/TCK             38, TCK/RA1
    // D5           58, 38, RA2/SCL2            58, SCL2/RA2
    // D6           59, 40, RA3/SDA2            59, SDA2/RA3
    // D7           60, 71, RA4/TDI             60, TDI/RA4
    // D8           61, 72, RA5/TDO             61, TDO/RA5
    // D9           91, 73, RA6                 91, AN22/CN22/RA6
    // D10 (MSb)    92, 74, RA7                 92, AN23/CN23/RA7         <- Pin function is muxed with button S5; we will use it as an LED output only
    ret    = LATA & 0x00FF;
    LATA  |= ledBitField & 0x00FF;
    
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

    // Function     Explorer 16 PIM Header      dsPIC33FJ256GP710A Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             17, TMS/RA0
    // D4           38, 70, RA1/TCK             38, TCK/RA1
    // D5           58, 38, RA2/SCL2            58, SCL2/RA2
    // D6           59, 40, RA3/SDA2            59, SDA2/RA3
    // D7           60, 71, RA4/TDI             60, TDI/RA4
    // D8           61, 72, RA5/TDO             61, TDO/RA5
    // D9           91, 73, RA6                 91, AN22/CN22/RA6
    // D10 (MSb)    92, 74, RA7                 92, AN23/CN23/RA7         <- Pin function is muxed with button S5; we will use it as an LED output only
    ret    = ~(LATA & 0x00FF);
    LATA  &= ~(ledBitField & 0x00FF);

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

    // Function     Explorer 16 PIM Header      dsPIC33FJ256GP710A Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             17, TMS/RA0
    // D4           38, 70, RA1/TCK             38, TCK/RA1
    // D5           58, 38, RA2/SCL2            58, SCL2/RA2
    // D6           59, 40, RA3/SDA2            59, SDA2/RA3
    // D7           60, 71, RA4/TDI             60, TDI/RA4
    // D8           61, 72, RA5/TDO             61, TDO/RA5
    // D9           91, 73, RA6                 91, AN22/CN22/RA6
    // D10 (MSb)    92, 74, RA7                 92, AN23/CN23/RA7         <- Pin function is muxed with button S5; we will use it as an LED output only
    ret                         = LATA & 0x00FF;
    ((unsigned char*)&LATA)[0] ^= ledBitField;

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

    // Function     Explorer 16 PIM Header      dsPIC33FJ256GP710A Device Pins
    // Button       PIM#, PICtail#, name        PIC#, name
    // S4 (LSb)     80, 106, RD13               80, IC6/CN19/RD13
    // S5           92,  74, RA7                92, AN23/CN23/RA7                 <- Pin function is muxed with LED D10; S5 button can't be used because LED clamps weak 10k pull up voltage too low
    // S6           84, 100, RD7                84, OC8/CN16/RD7
    // S3 (MSb)     83,  99, RD6                83, OC7/CN15/RD6

    //      Port  &    Pin      Pin- BUTTONx
    ret  = (PORTD & (1<<13)) >> (13- 0);
    ret |= (PORTD & (1<<7))  >> (7 - 2);
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
#endif //#if defined(__dsPIC33FJ256GP710A__) || defined(__dsPIC33FJ128GP710A__) || defined(__dsPIC33FJ64GP710A__) || defined(__dsPIC33FJ256GP710__) || defined(__dsPIC33FJ128GP710__) || defined(__dsPIC33FJ64GP710__)

