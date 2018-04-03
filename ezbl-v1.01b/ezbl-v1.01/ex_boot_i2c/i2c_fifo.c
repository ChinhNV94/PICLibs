/*
 * File:   i2c_fifo.c
 * Author: Howard Schlunder
 *
 * Created on September 22, 2014
 *
 * An easily ported interrupt based I2C TX and RX buffering implementation.
 * Implements independent software FIFOs for TX and RX directions.
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

#define I2C_FIFO_C      // Unique identifier in case if a #include .h header needs to know which file it is included in

#include <xc.h>         // Needed for I2C SFR definitions
#include "fifo.h"       // For FIFO_FEATURES flag definitions

// Configuration parameters
#define I2C_NUMBER              1       // Hardware I2C module # to apply this 
                                        // software FIFO to. 1 means I2C1, 2 
                                        // means I2C2, etc.
#define I2C_ADDRESS             0x60    // I2C 7-bit address of this node
#define I2C_TX_FIFO_SIZE        8u      // Bytes for software TX FIFO buffering
#define I2C_RX_FIFO_SIZE        128u    // Bytes for software RX FIFO buffering



// Static macros for concatenating tokens together without making them strings
// first. This is useful for prefixing letters or words onto functions,
// variables, and other symbol names.
// Ex: you can write 1 to the T2CONbits.ON register bit using:
// #define TIMER_SELECT  2
//     CAT3(T,TIMER_SELECT,CONbits.ON) = 1;
// The preprocessor will resolve this into:
//     T2CONbits.ON = 1;
#if !defined(CAT2)
#define CAT2_IN(arg0, arg1)                         arg0##arg1                  // Not recommended to use this macro directly; macro expansion won't occur unless another macro wraps it.
#define CAT3_IN(arg0, arg1, arg2)                   arg0##arg1##arg2            // Not recommended to use this macro directly; macro expansion won't occur unless another macro wraps it.
#define CAT2(arg0, arg1)                            CAT2_IN(arg0, arg1)         // Use this; allows macro expansion
#define CAT3(arg0, arg1, arg2)                      CAT3_IN(arg0, arg1, arg2)   // Use this; allows macro expansion
#endif


// Static token concatenation macros that make the code easier to read.
// Don't directly change these unless porting to different hardware with 
// different register names. Instead change the I2C_NUMBER macro above to change 
// which physical I2C hardware module this FIFOing library applies to.
#define I2CxCON         CAT3(I2C,I2C_NUMBER,CON)
#define I2CxCONbits     CAT3(I2C,I2C_NUMBER,CONbits)
#define I2CxSTAT        CAT3(I2C,I2C_NUMBER,STAT)
#define I2CxSTATbits    CAT3(I2C,I2C_NUMBER,STATbits)
#define I2CxMSK         CAT3(I2C,I2C_NUMBER,MSK)
#define I2CxRCV         CAT3(I2C,I2C_NUMBER,RCV)
#define I2CxADD         CAT3(I2C,I2C_NUMBER,ADD)
#define I2CxTRN         CAT3(I2C,I2C_NUMBER,TRN)
#define I2CxBRG         CAT3(I2C,I2C_NUMBER,BRG)
#define _MI2CxIF        CAT3(_MI2C,I2C_NUMBER,IF)
#define _SI2CxIF        CAT3(_SI2C,I2C_NUMBER,IF)
#define _MI2CxIE        CAT3(_MI2C,I2C_NUMBER,IE)
#define _SI2CxIE        CAT3(_SI2C,I2C_NUMBER,IE)
#define _MI2CxIP        CAT3(_MI2C,I2C_NUMBER,IP)
#define _SI2CxIP        CAT3(_SI2C,I2C_NUMBER,IP)
#define _MI2CxInterrupt CAT3(_MI2C,I2C_NUMBER,Interrupt)
#define _SI2CxInterrupt CAT3(_SI2C,I2C_NUMBER,Interrupt)


// Instantiate I2C TX FIFO
#define FIFO_NAME               I2C_TX_FIFO_
#define FIFO_SIZE               I2C_TX_FIFO_SIZE
#define FIFO_FEATURES           FIFO_FEATURE_WRITABLE
#include "fifo.c.h"


// Instantiate I2C RX FIFO
#define FIFO_NAME               I2C_RX_FIFO_
#define FIFO_SIZE               I2C_RX_FIFO_SIZE
#define FIFO_FEATURES           FIFO_FEATURE_READABLE | FIFO_FEATURE_PEEKABLE
#define FIFO_EXTRA_VARS         volatile unsigned int errors;   // Add an extra variable for capturing RX framing and hardware + software FIFO overrun errors
#include "fifo.c.h"


static unsigned int bytesToFrameHeader;  // Number of bytes we can send out the I2C before we need to send another txCount update byte for framing


/** 
 * Resets the hardware I2C and software FIFOs. All FIFO data is lost. The I2C
 * is configured and enabled afterwards and ready for communications/interrupts.
 *
 * @param peripheralClockSpeed Frequency, in hertz, that the I2C peripheral is
 *                             operating with. On PIC24 and dsPIC products, this 
 *                             is the same as the instructions executed/second 
 *                             while not dozing. Ex: set to 70000000 if running 
 *                             at 70 MIPS.
 *
 *                             If operating in slave mode, this parameter is
 *                             ignored.
 *
 * @param baudRate Desired baud rate for the I2C communications when operating 
 *                 in master mode. Ex: set to 400000 for 400kHz (bits/second).
 *                 The actual baud rate programmed will be as close as possible
 *                 to the requested value, but could still result in appreciable
 *                 error if specifying very fast baud rates or when operating at
 *                 a slow peripheral clock speed. See the I2C chapter in the
 *                 device data sheet or Family Reference Manual documentation to
 *                 understand the underlying error limits.
 *
 *                 For slave mode, this parameter is ignored.
 */
void I2C_Reset(unsigned long peripheralClockSpeed, unsigned long baudRate)
{
    I2CxCONbits.I2CEN = 0;          // Disable the I2C module if already enabled. This will suppress interrupts and allow us to reconfigure the module.
    I2C_RX_FIFO_vars.errors = 0;    // Clear overflow or other logged RX errors

    bytesToFrameHeader = 0;
    I2C_TX_FIFO_Reset();
    I2C_RX_FIFO_Reset();

    _MI2CxIP = 1;           // Set Master interrupt to Priority 1 (0 is main context, 7 is time-critical highest priority)
    _SI2CxIP = 1;           // Set Slave interrupt to Priority 1 (0 is main context, 7 is time-critical highest priority)
    I2CxCON = 0x1040;       // Set SCLREL, STREN = 1, all other bits = 0. Keeps I2C module disabled, but configures to enable clock stretching. Uses normal 7-bit address mode.
    I2CxSTAT = 0x0000;      // Clear Write Collision and Receive Overflow sticky status bits
    I2CxMSK = 0x0000;       // Set RX Address mask to use all 7 lower-order bits
    I2CxADD = I2C_ADDRESS;  // Set I2C slave address of this node for RX filtering
    I2CxBRG = (peripheralClockSpeed + (baudRate<<1))/(baudRate<<2) - 1u;    // Program Master mode baud rate generator to what was set outside this file
    _MI2CxIF = 0;           // Clear Master Interrupt Flag
    _SI2CxIF = 0;           // Clear Slave Interrupt Flag
    _MI2CxIE = 1;           // Enable Master Interrupts and normal operation
    _SI2CxIE = 1;           // Enable Slave Interrupts and normal operation

    I2CxCONbits.I2CEN = 1;  // Enable the I2C module
}


/**
 * Blocks execution until everything pending is finished being physically
 * transmitted. The software TX FIFO and the hardware I2C TX FIFO are drained
 * to 0 bytes before returning.
 */
void I2C_TX_FIFO_WaitUntilFlushed(void)
{
    while(I2C_TX_FIFO_vars.dataCount);  // Wait for software TX FIFO to empty
    while(I2CxSTATbits.TBF);            // Wait for TX hardware shift register to empty
}


/**
 * Enables interrupts used by the I2C FIFO TX and RX routines.
 */
void I2C_FIFO_EnableInterrupts(void)
{
    _MI2CxIE = 1;
    _SI2CxIE = 1;
}


/**
 * Disables all interrupts used by the I2C FIFO routines.
 */
void I2C_FIFO_DisableInterrupts(void)
{
    _MI2CxIE = 0;
    _SI2CxIE = 0;
}



///*********************************************************************
// * Function:        void _ISR _MI2CxInterrupt(void)
// *
// * PreCondition:    None
// *
// * Input:           None
// *
// * Output:          None
// *
// * Side Effects:    None
// *
// * Overview:        Skeleton function that nothing but clear the 
// *                  master I2C interrupt flag. Not used.
// *
// * Note:            None
// ********************************************************************/
//void __attribute__((interrupt, no_auto_psv)) _MI2CxInterrupt(void)
//{
//    // Clear the interrupt flag so we don't keep entering this ISR
//    _MI2CxIF = 0;
//
//    // Do whatever you want here. This example does not operate as an I2C
//    // master, so will never get here.
//}


/*********************************************************************
 * Function:        void _ISR _SI2CxInterrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Records hardware receive data into the software RX FIFO for
 *                  later processing elsewhere, such as the main() context.
 *
 *                  Copies any pending data from the TX FIFO to the I2C shift
 *                  transmit register whenever possible and needed. 
 *                  This TX data is framed using a 1 byte frame header
 *                  representing the number of pending TX bytes in 
 *                  the frame so that the master knows how many I2C 
 *                  clocks it should give us.
 *
 * Side Effects:    None
 *
 * Overview:        Receives a physical RX byte from the I2C hardware 
 *                  and places it in a local RAM FIFO for software to
 *                  read it at its leisure. Saves any errors, and 
 *                  places TX FIFO data into the transmit shift 
 *                  hardware.
 *
 * Note:            None
 ********************************************************************/
void __attribute__((interrupt, no_auto_psv)) _SI2CxInterrupt(void)
{
    unsigned int i;

    // Clear the interrupt flag so we don't keep entering this ISR
    _SI2CxIF = 0;

    // If an RX byte is available and there is space in the RX FIFO, read it
    // and place it in the RX FIFO
    if(I2CxSTATbits.R_W == 0u)  // Master write request -> we can RX a byte
    {
        // To ensure we are always in synch with the master, reset framing byte
        // count for our TX direction anytime the master sends us something
        bytesToFrameHeader = 0;

        // Check if we received an address byte. If so, read it to avoid overflow
        // and then throw it away.
        if(!I2CxSTATbits.D_A)
        {
            *((volatile unsigned int*)&i) = I2CxRCV;    // Cast i with volatile qualifier so compiler isn't allowed to optimize away I2CxRCV read operation since the i return result is unused.
        }
        else if(I2C_RX_FIFO_vars.dataCount < sizeof(I2C_RX_FIFO_vars.fifoRAM)) // Check for RX FIFO Free Space
        {
            // Get the byte
            i = I2CxRCV;

            // Collect any RX overflow status into a sticky container
            I2C_RX_FIFO_vars.errors |= I2CxSTAT & (_I2C1STAT_IWCOL_MASK | _I2C1STAT_I2COV_MASK);

            // If this is a data byte (not an address byte), copy the byte into
            // the local RX FIFO. The address byte should be thrown away.
            // NOTE: The FIFO internal data structures are being accessed directly
            // here rather than calling the I2C_RX_FIFO_Write*() functions because
            // any function call in an ISR will trigger a whole lot of compiler
            // context saving overhead. The compiler has no way of knowing what
            // registers any given function will clobber, so it has to save them
            // all. For efficiency, the needed write-one-byte code is duplicated
            // here.
            *I2C_RX_FIFO_vars.headPtr++ = i;
            if(I2C_RX_FIFO_vars.headPtr >= I2C_RX_FIFO_vars.fifoRAM + sizeof(I2C_RX_FIFO_vars.fifoRAM))
            {
                I2C_RX_FIFO_vars.headPtr = I2C_RX_FIFO_vars.fifoRAM;
            }
            ATOMIC_ADD(I2C_RX_FIFO_vars.dataCount, 1);
        }

        // Clear overflow flag if it it has become set. This shouldn't normally
        // be possible since we have clock stretching enabled.
        I2CxSTAT &= ~_I2C1STAT_I2COV_MASK;      // Done using a Read-Modify-Write on whole word instead of a bit in case if a silicon I2C errata is applicable on this device
        //I2CxSTATbits.I2COV = 0;
    }
    else    // Master Read Request -> might need to TX a byte
    {
        // Check if we received an address byte. If so, read it to avoid overflow
        // and then throw it away.
        if(!I2CxSTATbits.D_A)
        {
            *((volatile unsigned int*)&i) = I2CxRCV;    // Cast i with volatile qualifier so compiler isn't allowed to optimize away I2CxRCV read operation since the i return result is unused.
        }

        // Check the SCL clock stretch state to see if this interrupt was caused
        // by a module reset (unneeded/unused interrupt cause), or by a master 
        // read request, in which case we need to send something for this 
        // interrupt.
        if(!I2CxCONbits.SCLREL)
        {
            if(bytesToFrameHeader == 0u)
            {
                // Send a frame header indicating how many bytes we have pending
                // transmission, not including this frame header byte. This
                // value can be 0 to 255, and will saturate at 255 in case if
                // the TX FIFO is is holding more than 255 bytes in it. The next
                // frame header will always follow exactly this number of
                // advertised bytes + 1 in the future. In the event the TX FIFO
                // has no waiting data, a frame header indicating 0 bytes will
                // be sent so the master can poll this node and differentiate a
                // 0x00 frame header from a genuine 0x00 data byte.
                bytesToFrameHeader = I2C_TX_FIFO_vars.dataCount;
                if(bytesToFrameHeader > 255u)
                {
                    bytesToFrameHeader = 255;
                }
                I2CxTRN = (unsigned int)((unsigned char)bytesToFrameHeader);
            }
            else
            {
                // Get a byte from the TX FIFO buffer and place it in I2CxTRN
                bytesToFrameHeader--;
                I2CxTRN = (unsigned int)(*I2C_TX_FIFO_vars.tailPtr++);
                if(I2C_TX_FIFO_vars.tailPtr >= I2C_TX_FIFO_vars.fifoRAM + sizeof(I2C_TX_FIFO_vars.fifoRAM))
                    I2C_TX_FIFO_vars.tailPtr = I2C_TX_FIFO_vars.fifoRAM;
                ATOMIC_SUB(I2C_TX_FIFO_vars.dataCount, 1);
            }
        }
    }

    // Done processing TX or RX byte, reenable the master control of SCL.
    I2CxCON |= _I2C1CON_SCLREL_MASK;
}
