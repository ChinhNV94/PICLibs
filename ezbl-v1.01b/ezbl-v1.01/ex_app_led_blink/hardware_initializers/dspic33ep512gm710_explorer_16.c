/*
 * File:   dspic33ep512gm710_explorer_16.c
 *
 * Created on July 2014ish
 *
 * Initializes the device configuration fuses, clock frequency, UART2 pins,
 * LED I/O, Button I/O, 25LC256 SPI2 pins, and clock frequency for the
 * dsPIC33EP512MU810 PIM on the Explorer 16 development board.
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

#if defined(__dsPIC33EP512GM710__) || defined(__dsPIC33EP256GM710__) || defined(__dsPIC33EP128GM710__) || defined(__dsPIC33EP512GM310__) || defined(__dsPIC33EP256GM310__) || defined(__dsPIC33EP128GM310__)

#define FCY         70000000ul

#include <xc.h>

// Device Configuration Fuses
// These are commented out because they are defined in the bootloader project
// instead. If you want to use these, be sure to remove the applicable
// definition(s) from the bootloader project. Different config words can be
// split (ex: define _FOSC in the bootloader while _FWDT is defined here in the
// application).
//_FICD(ICS_PGD1 & JTAGEN_OFF)                                                // Debug using PGEC1 and PGED1, turn off JTAG to recover I/O pins
//_FPOR(BOREN_ON & ALTI2C1_OFF & ALTI2C2_OFF & WDTWIN_WIN75)                  // Enable Brown-out Reset (BOR), use primary I2C1 and I2C2 pin mappings, set watchdog window to 75% (not important in non-window mode, but included for completness)
//_FWDT(WDTPOST_PS2048 & WDTPRE_PR32 & PLLKEN_OFF & WINDIS_OFF & FWDTEN_OFF)  // ~2048ms timeout when watchdog is turned on in software (not forced on in hardware), continue executing from original clock before PLL locks on clock switch, use ordinary (non-windowed) Watchdog Timer mode
//_FOSC(POSCMD_NONE & OSCIOFNC_ON & IOL1WAY_OFF & FCKSM_CSECMD)               // No primary oscillator, recover OSC2 pin as GPIO, allow multiple PPS remappings, enable clock switching but disable fail safe clock monitor
//_FOSCSEL(FNOSC_FRCPLL & PWMLOCK_OFF & IESO_ON)                              // Start with FRC, then auto-switch to FRC+PLL when PLL locks, allow
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
    // Function     Explorer 16 PIM Header      dsPIC33EP512GM710 Device Pins
    // Button       PIM#, PICtail#, name        PIC#, name
    // S4 (LSb)     80, 106, RD13               80, RPI77/RD13
    // S5           92,  74, RA7                92, RF7             <- Pin function is muxed with LED D10; S5 button can't be used because LED clamps weak 10k pull up voltage too low
    // S6           84, 100, RD7                84, RP57/RC9
    // S3 (MSb)     83,  99, RD6                83, RP70/RD6
    _TRISD13 = 1;
    _TRISC9  = 1;
    _TRISD6  = 1;


    // Set LED pins as GPIO outputs
    //
    // Function     Explorer 16 PIM Header      dsPIC33EP512GM710 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             17, AN22/RG10
    // D4           38, 70, RA1/TCK             38, AN35/RG11
    // D5           58, 38, RA2/SCL2            58, AN42/RG2
    // D6           59, 40, RA3/SDA2            59, AN43/RG3
    // D7           60, 71, RA4/TDI             60, AN44/RF4
    // D8           61, 72, RA5/TDO             61, AN45/RF5
    // D9           91, 73, RA6                 91, RF6
    // D10 (MSb)    92, 74, RA7                 92, RF7             <- Pin function is muxed with button S5; we will use it as an LED output only
    ANSELG &= ~(1<<10 | 1<<11 | 1<<2 | 1<<3);
    LATG   &= ~(1<<10 | 1<<11 | 1<<2 | 1<<3);
    TRISG  &= ~(1<<10 | 1<<11 | 1<<2 | 1<<3);
    ANSELF &= ~(1<<4 | 1<<5 | 1<<6 | 1<<7);
    LATF   &= ~(1<<4 | 1<<5 | 1<<6 | 1<<7);
    TRISF  &= ~(1<<4 | 1<<5 | 1<<6 | 1<<7);


    // Set 16x2 character LCD pins as GPIO outputs
    //
    // Function     Explorer 16 PIM Header      dsPIC33EP512GM710 Device Pins
    // 16x2 LCD     PIM#, PICtail#, name        PIC#, name
    // Data 0        93, 109, RE0/PMPD0          93, RP42/PWM3H/PMD0/RB10
    // Data 1        94, 110, RE1/PMPD1          94, RP43/PWM3L/PMD1/RB11
    // Data 2        98, 111, RE2/PMPD2          98, RPI44/PWM2H/PMD2/RB12
    // Data 3        99, 112, RE3/PMPD3          99, RPI45/PWM2L/CTPLS/PMD3/RB13
    // Data 4       100, 113, RE4/PMPD4         100, TDO/PWM4H/PMD4/RA10
    // Data 5         3, 114, RE5/PMPD5           3, TDI/PWM4L/PMD5/RA7
    // Data 6         4, 115, RE6/PMPD6           4, RPI46/PWM1H/T3CK/T7CK/PMD6/RB14
    // Data 7         5, 116, RE7/PMPD7           5, RPI47/PWM1L/T5CK/T6CK/PMD7/RB15
    // E (Enable)    81,  97, RD4/PMPWR          81, RP56/PMWR/RC8
    // R/!W          82,  98, RD5/PMPRD          82, RP69/PMRD/RD5
    // RS (Reg Sel)  44,  84, RB15/PMPA0         44, AN15/RPI95/FLT8/PMA0/RE15
    LATB    &= ~(1<<10 | 1<<11 | 1<<12 | 1<<13 | 1<<14 | 1<<15);
    TRISB   &= ~(1<<10 | 1<<11 | 1<<12 | 1<<13 | 1<<14 | 1<<15);
    _LATA10  = 0;
    _TRISA10 = 0;
    _LATA7   = 0;
    _TRISA7  = 0;
    _LATC8   = 0;
    _TRISC8  = 0;
    _LATD5   = 0;
    _TRISD5  = 0;
    _ANSE15  = 0;
    _LATE15  = 0;
    _TRISE15 = 0;


    // Configure UART2 pins as UART.
    // - Pin names are with respect to the PIC.
    // - Outputs bits in TRIS registers are all set as inputs because the PPS or
    //   UART2 hardware overrides it.
    //
    // Function     Explorer 16 PIM Header      dsPIC33EP512GM710 Device Pins
    // UART2        PIM#, PICtail#, name        PIC#, name
    // U2TX  (out)  50, 36, RF5/PMPA8/U2TX      50, FLT32/SCL2/RP36/PMA8/RB4
    // U2RTS (out)  39, 52, RF13/U2RTS          41, AN12/C2IN2-/C5IN2-/!U2RTS/BCLK2/FLT5/PMA11/RE12
    // U2RX  (in)   49, 34, RF4/PMPA9/U2RX      49, SDA2/RPI24/PMA9/RA8
    // U2CTS (in)   40, 51, RF12/U2CTS          42, AN13/C3IN2-/!U2CTS/FLT6/PMA10/RE13
    _TRISB4  = 1;   // Actually an output, but PPS will override this
    _ANSE12  = 0;
    _CNPDE12 = 1;   // Turn on pull-down on U2RTS output in case if we don't enable this output in the actual UART module
    _TRISE12 = 1;
    _TRISA8  = 1;
    _ANSE13  = 0;
    _TRISE13 = 1;
    _U2RXR   = 24;  // U2RX on RPI24
    _RP36R   = 0x03;// U2TX on RP36
    __C30_UART = 2; // printf() on UART2 since it has the DB9 connector and MAX3232 transceiver


    // Configure pins for 25LC256 (32Kbyte SPI EEPROM). 
    // - Pin names are with respect to the PIC, which is the SPI Master.
    // - Outputs bits in TRIS registers are all set as inputs because the PPS or
    //   SPI hardware overrides it.
    //
    // Function     Explorer 16 PIM Header      dsPIC33EP512GM710 Device Pins
    // SPI2         PIM#, PICtail#, name        PIC#, name
    // !CS   (out)  79, 105, RD12               79, RPI76/RD12
    // SCK2  (out)  10,  35, RG6/PMPA5/SCK2     10, AN19/RP118/PMA5/RG6
    // SDI2  (in)   11,  37, RG7/PMPA4/SDI2     11, AN18/ASCL1/RPI119/PMA4/RG7
    // SDO2  (out)  12,  39, RG8/PMPA3/SDO2     12, AN17/ASDA1/RP120/PMA3/RG8
    _LATD12  = 1;
    _TRISD12 = 0;
    ANSELG  &=  (1<<6 | 1<<7 | 1<<8);
    TRISG   |=  (1<<6 | 1<<7 | 1<<8);
    _CNPDG7  = 1;    // Turn on pull down on SDI2 so it doesn't float when SPI module tri-states it
    _SDI2R   = 119;  // SDI2 on RPI119
    _RP118R  = 0x09; // SCK2 on RP118
    _RP120R  = 0x08; // SDO2 on RP120


    // Configure Pins for SPI1 on Explorer 16 (nothing attached on the Explorer
    // 16 directly, but useful for various PICtails).
    // - Pin names are with respect to the PIC, which is the SPI Master.
    // - Outputs bits in TRIS registers are all set as inputs because the PPS or
    //   SPI hardware overrides it.
    //
    // Function     Explorer 16 PIM Header      dsPIC33EP512GM710 Device Pins
    // SPI1         PIM#, PICtail#, name        PIC#, name
    // !CS   (out)  23,   1, RB2/SS1/AN2        23, OA2IN+/AN1/C2IN1+/RPI17/RA1
    // SCK1  (out)  55,   3, RF6/SCK1           55, AN29/SCK1/RPI51/RC3
    // SDI1  (in)   54,   5, RF7/SDI1           54, AN28/SDI1/RPI25/RA9
    // SDO1  (out)  53,   7, RF8/SDO1           51, OA5IN+/AN24/C5IN3-/C5IN1+/SDO1/RP20/T1CK/RA4
    _LATA1   = 1;
    _TRISA1  = 0;
    _TRISC3  = 1;
    _TRISA4  = 1;
    _TRISA9  = 1;
    ANSELA  &= ~(1<<1 | 1<<9 | 1<<4);
    _ANSC3   = 0;
    _CNPDA9  = 1;    // Turn on SDI1 pull down so it doesn't float when slave chip is inactive


    // Configure Analog Inputs for U4 TC1047A Temperature Sensor and R6 10K Potentiometer
    //
    // Function     Explorer 16 PIM Header      dsPIC33EP512GM710 Device Pins
    // Analog Input PIM#, PICtail#, name        PIC#, name
    // TC1047A Temp 21, 14, RB4/AN4             21, AN9/RPI27/RA11
    // 10K Pot      20, 77, RB5/AN5             19, AN20/RE9
    _ANSA11 = 1;
    _ANSE9  = 1;


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
    unsigned int mask;
    unsigned int ret;

    // Function     Explorer 16 PIM Header      dsPIC33EP512GM710 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             17, AN22/RG10
    // D4           38, 70, RA1/TCK             38, AN35/RG11
    // D5           58, 38, RA2/SCL2            58, AN42/RG2
    // D6           59, 40, RA3/SDA2            59, AN43/RG3
    // D7           60, 71, RA4/TDI             60, AN44/RF4
    // D8           61, 72, RA5/TDO             61, AN45/RF5
    // D9           91, 73, RA6                 91, RF6
    // D10 (MSb)    92, 74, RA7                 92, RF7             <- Pin function is muxed with button S5

    ret    = LATF & 0xF0;
    ret   |= LATG & 0x0C;
    ret   |= (LATG & (1<<10 | 1<<11)) >> 10;
    mask   = ledBitField & 0x03;
    mask <<= 10;
    mask  |= ledBitField & 0x0C;
    LATG  &= mask | ~(1<<10 | 1<<11 | 1<<2 | 1<<3);  // Turn off zeros
    LATG  |= mask;                                   // Turn on ones
    LATF  &= (ledBitField & 0xF0) | ~(0x00F0);       // Turn off zeros
    LATF  |= ledBitField & 0xF0;                     // Turn on ones

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
    unsigned int mask;
    unsigned int ret;

    // Function     Explorer 16 PIM Header      dsPIC33EP512GM710 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             17, AN22/RG10
    // D4           38, 70, RA1/TCK             38, AN35/RG11
    // D5           58, 38, RA2/SCL2            58, AN42/RG2
    // D6           59, 40, RA3/SDA2            59, AN43/RG3
    // D7           60, 71, RA4/TDI             60, AN44/RF4
    // D8           61, 72, RA5/TDO             61, AN45/RF5
    // D9           91, 73, RA6                 91, RF6
    // D10 (MSb)    92, 74, RA7                 92, RF7             <- Pin function is muxed with button S5

    ret    = LATF & 0xF0;
    ret   |= LATG & 0x0C;
    ret   |= (LATG & (1<<10 | 1<<11)) >> 10;
    mask   = ledBitField & 0x03;
    mask <<= 10;
    mask  |= ledBitField & 0x0C;
    LATG  |= mask;
    LATF  |= ledBitField & 0xF0;
    
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
    unsigned int mask;
    unsigned int ret;

    // Function     Explorer 16 PIM Header      dsPIC33EP512GM710 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             17, AN22/RG10
    // D4           38, 70, RA1/TCK             38, AN35/RG11
    // D5           58, 38, RA2/SCL2            58, AN42/RG2
    // D6           59, 40, RA3/SDA2            59, AN43/RG3
    // D7           60, 71, RA4/TDI             60, AN44/RF4
    // D8           61, 72, RA5/TDO             61, AN45/RF5
    // D9           91, 73, RA6                 91, RF6
    // D10 (MSb)    92, 74, RA7                 92, RF7             <- Pin function is muxed with button S5

    ret    =  LATF & 0xF0;
    ret   |=  LATG & 0x0C;
    ret   |=  (LATG & (1<<10 | 1<<11)) >> 10;
    mask   =  ledBitField & 0x03;
    mask <<=  10;
    mask  |=  ledBitField & 0x0C;
    LATG  &= ~mask;
    LATF  &= ~(ledBitField & 0xF0);

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
    unsigned int mask;
    unsigned int ret;

    // Function     Explorer 16 PIM Header      dsPIC33EP512GM710 Device Pins
    // LED          PIM#, PICtail#, name        PIC#, name
    // D3 (LSb)     17, 69, RA0/TMS             17, AN22/RG10
    // D4           38, 70, RA1/TCK             38, AN35/RG11
    // D5           58, 38, RA2/SCL2            58, AN42/RG2
    // D6           59, 40, RA3/SDA2            59, AN43/RG3
    // D7           60, 71, RA4/TDI             60, AN44/RF4
    // D8           61, 72, RA5/TDO             61, AN45/RF5
    // D9           91, 73, RA6                 91, RF6
    // D10 (MSb)    92, 74, RA7                 92, RF7             <- Pin function is muxed with button S5

    ret = LATF & 0xF0;
    ret |= LATG & 0x0C;
    ret |= (LATG & (1<<10 | 1<<11)) >> 10;
    mask = ledBitField & 0x03;
    mask <<= 10;
    mask |= ledBitField & 0x0C;
    LATG ^= mask;
    LATF ^= ledBitField & 0xF0;

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

    // Function     Explorer 16 PIM Header      dsPIC33EP512GM710 Device Pins
    // Button       PIM#, PICtail#, name        PIC#, name
    // S4 (LSb)     80, 106, RD13               80, RPI77/RD13
    // S5           92,  74, RA7                92, RF7             <- Pin function is muxed with LED D10; S5 button can't be used because LED clamps weak 10k pull up voltage too low
    // S6           84, 100, RD7                84, RP57/RC9
    // S3 (MSb)     83,  99, RD6                83, RP70/RD6

    //      Port  &    Pin      Pin- BUTTONx
    ret  = (PORTD & (1<<13)) >> (13- 0);
    ret |= (PORTC & (1<<9))  >> (9 - 2);
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
    ButtonsToggled  = ButtonsLastState ^ ret;
    ButtonsPushed   = ButtonsToggled & ~ret;
    ButtonsReleased = ButtonsToggled & ret;
    
    // Save immediate button state to last state for future Toggle/Push/Release
    // event tracking
    ButtonsLastState = ret; // Save value for other API event checking return values

    return ret;
}

#endif //#if defined(__dsPIC33EP512GM710__) || defined(__dsPIC33EP256GM710__) || defined(__dsPIC33EP128GM710__) || defined(__dsPIC33EP512GM310__) || defined(__dsPIC33EP256GM310__) || defined(__dsPIC33EP128GM310__)

