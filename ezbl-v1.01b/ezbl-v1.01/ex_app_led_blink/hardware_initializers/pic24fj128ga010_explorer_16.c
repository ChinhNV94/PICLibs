/*
 * File:   pic24fj128ga010_explorer_16.c
 *
 * Created on August 04, 2014
 *
 * Initializes the device configuration fuses, clock frequency, UART2 pins,
 * LED I/O, Button I/O, 25LC256 SPI2 pins, and clock frequency for the
 * PIC24FJ128GA010 PIM on the Explorer 16 development board.
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

#if defined(__PIC24FJ128GA010__) || defined(__PIC24FJ96GA010__) || defined(__PIC24FJ64GA010__)

#define FCY         16000000ul

#include <xc.h>

// Device Configuration Fuses
// These are commented out because they are defined in the bootloader project
// instead. If you want to use these, be sure to remove the applicable
// definition(s) from the bootloader project. Different config words can be
// split (ex: define _CONFIG1 in the bootloader while _CONFIG2 is defined here
// in the application).
//_CONFIG2(POSCMOD_NONE & OSCIOFNC_ON & FCKSM_CSECMD & FNOSC_FRCPLL & IESO_OFF)
//_CONFIG1(WDTPS_PS2048 & FWPSA_PR32 & WINDIS_ON & FWDTEN_OFF & ICS_PGx2 & COE_OFF & GWRP_OFF & GCP_OFF & JTAGEN_OFF)


extern int __C30_UART;


// @return: FCY clock speed we just configured the processor for
unsigned long InitializeBoard(void)
{
    // Set the CPU clock to maximum 16 MIPS speed
    CLKDIV = 0x0000;    // Set 1:1 8MHz FRC postscalar so 4x PLL gives 16 MIPS (32MHz)

    // Set push buttons as GPIO inputs
    //
    // Function     Explorer 16 PIM Header      PIC24FJ128GA010 Device Pins
    // Button       PIM#, PICtail#, name        PIC#, name
    // S4 (LSb)     80, 106, RD13               80, CN19/RD13
    // S5           92,  74, RA7                92, RA7                 <- Pin function is muxed with LED D10; S5 button can't be used because LED clamps weak 10k pull up voltage too low
    // S6           84, 100, RD7                84, CN16/RD7
    // S3 (MSb)     83,  99, RD6                83, CN15/RD6
    TRISD |= (1<<13 | 1<<7 | 1<<6);
    TRISAbits.TRISA7 = 1;


    // Set LED pins as GPIO outputs
    //
    // Function     Explorer 16 PIM Header      PIC24FJ128GA010 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             17, TMS/RA0
    // D4           38, 70, RA1/TCK             38, TCK/RA1
    // D5           58, 38, RA2/SCL2            58, SCL2/RA2
    // D6           59, 40, RA3/SDA2            59, SDA2/RA3
    // D7           60, 71, RA4/TDI             60, TDI/RA4
    // D8           61, 72, RA5/TDO             61, TDO/RA5
    // D9           91, 73, RA6                 91, RA6
    // D10 (MSb)    92, 74, RA7                 92, RA7         <- Pin function is muxed with button S5; we will use it as an LED output only
    TRISA  &= ~(1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7);
    LATA   &= ~(1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7);


    // Set 16x2 character LCD pins as GPIO outputs
    //
    // Function     Explorer 16 PIM Header      PIC24FJ128GA010 Device Pins
    // 16x2 LCD     PIM#, PICtail#, name        PIC#, name
    // Data 0        93, 109, RE0/PMPD0          93, PMD0/RE0
    // Data 1        94, 110, RE1/PMPD1          94, PMD1/RE1
    // Data 2        98, 111, RE2/PMPD2          98, PMD2/RE2
    // Data 3        99, 112, RE3/PMPD3          99, PMD3/RE3
    // Data 4       100, 113, RE4/PMPD4         100, PMD4/RE4
    // Data 5         3, 114, RE5/PMPD5           3, PMD5/RE5
    // Data 6         4, 115, RE6/PMPD6           4, PMD6/RE6
    // Data 7         5, 116, RE7/PMPD7           5, PMD7/RE7
    // E (Enable)    81,  97, RD4/PMPWR          81, PMWR/OC5/CN13/RD4
    // R/!W          82,  98, RD5/PMPRD          82, PMRD/CN14/RD5
    // RS (Reg Sel)  44,  84, RB15/PMPA0         44, PMA0/AN15/OCFB/CN12/RB15
    TRISE  &= ~(1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7);
    LATE   &= ~(1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7);
    TRISD  &= ~(1<<4 | 1<<5);
    LATD   &= ~(1<<4 | 1<<5);
    TRISBbits.TRISB15 = 0;
    LATBbits.LATB15 = 0;
    AD1PCFGbits.PCFG15 = 0;


    // Configure UART2 pins as UART.
    // - Pin names are with respect to the PIC.
    // - Outputs bits in TRIS registers are all set as inputs because the PPS or
    //   UART2 hardware overrides it.
    //
    // Function     Explorer 16 PIM Header      PIC24FJ128GA010 Device Pins
    // UART2        PIM#, PICtail#, name        PIC#, name
    // U2TX  (out)  50, 36, RF5/PMPA8/U2TX      50, PMA8/U2TX/CN18/RF5
    // U2RTS (out)  39, 52, RF13/U2RTS          39, U2RTS/BCLK2/RF13
    // U2RX  (in)   49, 34, RF4/PMPA9/U2RX      49, PMA9/U2RX/CN17/RF4
    // U2CTS (in)   40, 51, RF12/U2CTS          40, U2CTS/RF12
    TRISF  |=  (1<<5 | 1<<13 | 1<<4 | 1<<12);
    __C30_UART = 2;             // printf() on UART2 since it has the DB9 connector and MAX3232 transceiver


    // Configure pins for 25LC256 (32Kbyte SPI EEPROM). 
    // - Pin names are with respect to the PIC, which is the SPI Master.
    // - Outputs bits in TRIS registers are all set as inputs because the PPS or
    //   SPI hardware overrides it.
    //
    // Function     Explorer 16 PIM Header      PIC24FJ128GA010 Device Pins
    // SPI2         PIM#, PICtail#, name        PIC#, name
    // !CS   (out)  79, 105, RD12               79, IC5/RD12
    // SCK2  (out)  10,  35, RG6/PMPA5/SCK2     10, PMA5/SCK2/CN8/RG6
    // SDI2  (in)   11,  37, RG7/PMPA4/SDI2     11, PMA4/SDI2/CN9/RG7
    // SDO2  (out)  12,  39, RG8/PMPA3/SDO2     12, PMA3/SDO2/CN10/RG8
    TRISDbits.TRISD12 = 0;      // !CS on RD12
    LATDbits.LATD12   = 1;      // 1 is inactive
    TRISG            |= (1<<6 | 1<<7 | 1<<8);
    CNPU1bits.CN9PUE  = 1;      // Turn on pull up on SDI2 so it doesn't float when SPI module tri-states it


    // Configure Analog Inputs for U4 TC1047A Temperature Sensor and R6 10K Potentiometer
    //
    // Function     Explorer 16 PIM Header      PIC24FJ128GA010 Device Pins
    // Analog Input PIM#, PICtail#, name        PIC#, name
    // TC1047A Temp 21, 14, RB4/AN4             21, C1IN-/AN4/CN6/RB4
    // 10K Pot      20, 77, RB5/AN5             20, C1IN+/AN5/CN7/RB5
    TRISB   |=  (1<<4 | 1<<5);
    AD1PCFG &= ~(1<<4 | 1<<5);


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
    unsigned int ret;

    // Function     Explorer 16 PIM Header      PIC24FJ128GA010 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             17, TMS/RA0
    // D4           38, 70, RA1/TCK             38, TCK/RA1
    // D5           58, 38, RA2/SCL2            58, SCL2/RA2
    // D6           59, 40, RA3/SDA2            59, SDA2/RA3
    // D7           60, 71, RA4/TDI             60, TDI/RA4
    // D8           61, 72, RA5/TDO             61, TDO/RA5
    // D9           91, 73, RA6                 91, RA6
    // D10 (MSb)    92, 74, RA7                 92, RA7         <- Pin function is muxed with button S5; we will use it as an LED output only
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

    // Function     Explorer 16 PIM Header      PIC24FJ128GA010 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             17, TMS/RA0
    // D4           38, 70, RA1/TCK             38, TCK/RA1
    // D5           58, 38, RA2/SCL2            58, SCL2/RA2
    // D6           59, 40, RA3/SDA2            59, SDA2/RA3
    // D7           60, 71, RA4/TDI             60, TDI/RA4
    // D8           61, 72, RA5/TDO             61, TDO/RA5
    // D9           91, 73, RA6                 91, RA6
    // D10 (MSb)    92, 74, RA7                 92, RA7         <- Pin function is muxed with button S5; we will use it as an LED output only
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

    // Function     Explorer 16 PIM Header      PIC24FJ128GA010 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             17, TMS/RA0
    // D4           38, 70, RA1/TCK             38, TCK/RA1
    // D5           58, 38, RA2/SCL2            58, SCL2/RA2
    // D6           59, 40, RA3/SDA2            59, SDA2/RA3
    // D7           60, 71, RA4/TDI             60, TDI/RA4
    // D8           61, 72, RA5/TDO             61, TDO/RA5
    // D9           91, 73, RA6                 91, RA6
    // D10 (MSb)    92, 74, RA7                 92, RA7         <- Pin function is muxed with button S5; we will use it as an LED output only
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

    // Function     Explorer 16 PIM Header      PIC24FJ128GA010 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             17, TMS/RA0
    // D4           38, 70, RA1/TCK             38, TCK/RA1
    // D5           58, 38, RA2/SCL2            58, SCL2/RA2
    // D6           59, 40, RA3/SDA2            59, SDA2/RA3
    // D7           60, 71, RA4/TDI             60, TDI/RA4
    // D8           61, 72, RA5/TDO             61, TDO/RA5
    // D9           91, 73, RA6                 91, RA6
    // D10 (MSb)    92, 74, RA7                 92, RA7         <- Pin function is muxed with button S5; we will use it as an LED output only
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

    // Function     Explorer 16 PIM Header      PIC24FJ128GA010 Device Pins
    // Button       PIM#, PICtail#, name        PIC#, name
    // S4 (LSb)     80, 106, RD13               80, CN19/RD13
    // S5           92,  74, RA7                92, RA7                 <- Pin function is muxed with LED D10; S5 button can't be used because LED clamps weak 10k pull up voltage too low
    // S6           84, 100, RD7                84, CN16/RD7
    // S3 (MSb)     83,  99, RD6                83, CN15/RD6

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
#endif //#if defined(__PIC24FJ128GA010__) || defined(__PIC24FJ96GA010__) || defined(__PIC24FJ64GA010__)

