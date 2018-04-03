/*
 * File:   i2c_fifo.h
 * Author: Howard Schlunder
 *
 * Created on September 19, 2014
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

#ifndef I2C_FIFO_H
#define	I2C_FIFO_H

// All prototypes listed below are functions declared in i2c_fifo.c. The actual
// I2C_TX_FIFO*() and I2C_RX_FIFO*() functions are automatically generated
// via fifo.h being included twice in i2c_fifo.c, so you can't easily find
// them. If you're searching for them though, see fifo.h!
void I2C_Reset(unsigned long peripheralClockSpeed, unsigned long baudRate); // Resets the hardware I2C module, clears all pending data in the RX and TX FIFOs, and then initializes the I2C for automatic ISR based TX/RX operation.
void I2C_TX_FIFO_WaitUntilFlushed(void);                                    // Blocks until TX FIFO is empty (in both software and hardware)
void I2C_FIFO_EnableInterrupts(void);                                       // Sets this module's I2C TX and RX interrupt enable flags
void I2C_FIFO_DisableInterrupts(void);                                      // Clears this module's I2C TX and RX interrupt enable flags


// I2C receive functions
unsigned int        I2C_RX_FIFO_WritableLength(void);
unsigned int        I2C_RX_FIFO_ReadableLength(void);
void                I2C_RX_FIFO_ReadSuspendUntil(unsigned int bytesAvailable);      // Put the CPU into idle until at least bytesAvailable becomes present in the RX FIFO. If there already are enough bytes in the FIFO to meet the requested number, this function returns immediately.
unsigned int        I2C_RX_FIFO_Read(void *destPtr, unsigned int readLength);       // Retruns data from interrupt reception via the I2C
unsigned char       I2C_RX_FIFO_Read8(void);                                        // Retruns data from interrupt reception via the I2C
unsigned short      I2C_RX_FIFO_Read16(void);                                       // Retruns data from interrupt reception via the I2C
unsigned long       I2C_RX_FIFO_Read24(void);                                       // Retruns data from interrupt reception via the I2C
unsigned long       I2C_RX_FIFO_Read32(void);                                       // Retruns data from interrupt reception via the I2C
unsigned long long  I2C_RX_FIFO_Read64(void);                                       // Retruns data from interrupt reception via the I2C
unsigned int        I2C_RX_FIFO_Peek(void *destPtr, unsigned int readLength);       // Retruns data from interrupt reception via the I2C
unsigned char       I2C_RX_FIFO_Peek8(void);                                        // Retruns data from interrupt reception via the I2C
unsigned short      I2C_RX_FIFO_Peek16(void);                                       // Retruns data from interrupt reception via the I2C
unsigned long       I2C_RX_FIFO_Peek24(void);                                       // Retruns data from interrupt reception via the I2C
unsigned long       I2C_RX_FIFO_Peek32(void);                                       // Retruns data from interrupt reception via the I2C
unsigned long long  I2C_RX_FIFO_Peek64(void);                                       // Retruns data from interrupt reception via the I2C
void                I2C_RX_FIFO_WriteSuspendUntil(unsigned int bytesFree);          // Put the CPU into idle until at least bytesFree become available in the RX FIFO. You generally would only want to call this if you are using the Write() functions to inject software data into the RX path via an ISR or alternate RTOS thread.
//unsigned int        I2C_RX_FIFO_Write(void *sourcePtr, unsigned int writeLength);   // You generally won't need to write to the RX FIFO in your code because it is already attached to a hardware ISR in i2c_fifo.c. However, this function is still prototyped because you may want to inject something via software in the RX path.
//void                I2C_RX_FIFO_Write8(unsigned char writeData);                    // You generally won't need to write to the RX FIFO in your code because it is already attached to a hardware ISR in i2c_fifo.c. However, this function is still prototyped because you may want to inject something via software in the RX path.
//void                I2C_RX_FIFO_Write16(unsigned short writeData);                  // You generally won't need to write to the RX FIFO in your code because it is already attached to a hardware ISR in i2c_fifo.c. However, this function is still prototyped because you may want to inject something via software in the RX path.
//void                I2C_RX_FIFO_Write24(unsigned long writeData);                   // You generally won't need to write to the RX FIFO in your code because it is already attached to a hardware ISR in i2c_fifo.c. However, this function is still prototyped because you may want to inject something via software in the RX path.
//void                I2C_RX_FIFO_Write32(unsigned long writeData);                   // You generally won't need to write to the RX FIFO in your code because it is already attached to a hardware ISR in i2c_fifo.c. However, this function is still prototyped because you may want to inject something via software in the RX path.
//void                I2C_RX_FIFO_Write64(unsigned long long writeData);              // You generally won't need to write to the RX FIFO in your code because it is already attached to a hardware ISR in i2c_fifo.c. However, this function is still prototyped because you may want to inject something via software in the RX path.

// I2C transmit functions
unsigned int        I2C_TX_FIFO_WritableLength(void);                               // Returns the number of bytes of free space in the TX FIFO which can be written immediately without blocking or idling.
unsigned int        I2C_TX_FIFO_ReadableLength(void);                               // Returns the number of bytes still queued for I2C transmission which haven't entered the hardware's TX FIFO yet (which generally is very small, like 4 bytes)
void                I2C_TX_FIFO_ReadSuspendUntil(unsigned int bytesAvailable);      // Put the CPU into idle until at least bytesAvailable becomes present in the TX FIFO. You generally would only want to call this if you have the hardware I2C TX function disabled and you want to use the Read() functions below to collect the data queued for transmission in an alternative ISR or RTOS thread.
void                I2C_TX_FIFO_WriteSuspendUntil(unsigned int bytesFree);          // Put the CPU into idle until bytesFree becomes available in the TX FIFO. If there already enough freespace to put the specified number of bytes into the TX FIFO, this function returns immediately.
//unsigned int        I2C_TX_FIFO_Read(void *destPtr, unsigned int readLength);       // You generally won't need to read from the TX FIFO in your code because it is already attached to a hardware ISR in i2c_fifo.c. However, this function is still prototyped because you may want to pluck something out of the TX path that some other portion of software decided to send. Particularly useful if you disable the hardware I2C TX ISR function.
//unsigned char       I2C_TX_FIFO_Read8(void);                                        // You generally won't need to read from the TX FIFO in your code because it is already attached to a hardware ISR in i2c_fifo.c. However, this function is still prototyped because you may want to pluck something out of the TX path that some other portion of software decided to send. Particularly useful if you disable the hardware I2C TX ISR function.
//unsigned short      I2C_TX_FIFO_Read16(void);                                       // You generally won't need to read from the TX FIFO in your code because it is already attached to a hardware ISR in i2c_fifo.c. However, this function is still prototyped because you may want to pluck something out of the TX path that some other portion of software decided to send. Particularly useful if you disable the hardware I2C TX ISR function.
//unsigned long       I2C_TX_FIFO_Read24(void);                                       // You generally won't need to read from the TX FIFO in your code because it is already attached to a hardware ISR in i2c_fifo.c. However, this function is still prototyped because you may want to pluck something out of the TX path that some other portion of software decided to send. Particularly useful if you disable the hardware I2C TX ISR function.
//unsigned long       I2C_TX_FIFO_Read32(void);                                       // You generally won't need to read from the TX FIFO in your code because it is already attached to a hardware ISR in i2c_fifo.c. However, this function is still prototyped because you may want to pluck something out of the TX path that some other portion of software decided to send. Particularly useful if you disable the hardware I2C TX ISR function.
//unsigned long long  I2C_TX_FIFO_Read64(void);                                       // You generally won't need to read from the TX FIFO in your code because it is already attached to a hardware ISR in i2c_fifo.c. However, this function is still prototyped because you may want to pluck something out of the TX path that some other portion of software decided to send. Particularly useful if you disable the hardware I2C TX ISR function.
unsigned int        I2C_TX_FIFO_Write(void *sourcePtr, unsigned int writeLength);   // Queues data for interrupt transmission out the I2C peripheral.
void                I2C_TX_FIFO_Write8(unsigned char writeData);                    // Queues data for interrupt transmission out the I2C peripheral.
void                I2C_TX_FIFO_Write16(unsigned short writeData);                  // Queues data for interrupt transmission out the I2C peripheral.
void                I2C_TX_FIFO_Write24(unsigned long writeData);                   // Queues data for interrupt transmission out the I2C peripheral.
void                I2C_TX_FIFO_Write32(unsigned long writeData);                   // Queues data for interrupt transmission out the I2C peripheral.
void                I2C_TX_FIFO_Write64(unsigned long long writeData);              // Queues data for interrupt transmission out the I2C peripheral.


#if !defined(I2C_FIFO_C) // Only include this structure in .c files outside i2c_fifo.c which instantiates this variable in the first place. This is required because the types conflict without a known size for fifoRAM in extern use.
extern struct
{
    volatile unsigned int dataCount; // Use ATOMIC_ADD() and ATOMIC_SUBTRACT() to change this value
    unsigned char *headPtr;
    unsigned char *tailPtr;
    volatile unsigned int errors;
    unsigned char fifoRAM[0];   // As a quasi variable length field, this should be at the bottom of the structure
} I2C_RX_FIFO_vars;
#endif

#endif	/* I2C_FIFO_H */

