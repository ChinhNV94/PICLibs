/**
 * File:   main.c
 * Author: Howard Schlunder
 *
 * Created on September 22, 2014
 *
 * Bootloader "main" file for initializing communications hardware, checking 
 * app-installed flags, receiving new application firmware, and dispatching 
 * execution to the application's main routine (following its own, normal CRT 
 * start up).
 *
 * This file is executed after the CRT executes for the bootloader, but before 
 * any bootloaded code is executed. Items in this file/project can optionally be 
 * called by a bootloaded application.
 *
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

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <libpic30.h>
#include <string.h>
#include "ezbl_integration/ezbl.h"
#include "ezbl_integration/i2c_fifo.h"
#include "now.h"
#include "hardware_initializers/hardware_initializer.h"



// String transmitted when a '?' character is received from the coms. This
// can be shrunk to an empty null string "", or deleted along with the '?'
// command handler to save Flash space. It isn't required for correct bootloader
// operation.
#define DEVICE_QUERY_STRING         "EZBL v" STRINGIFY(EZBL_VERSION) " I2C"

// Wait value on reset, staying in the bootloader. During this window, the
// bootloader will be in complete control of the processor, accepting bootload
// commands and controlling needed communications hardware. This can normally be
// quite short for applications like ex_app_led_blink where the bootloader can
// still be active while the application is running. After this value elapses
// execution is passed to the installed application (if a valid one is found).
// The timer for this timeout resets anytime there is com RX traffic so with
// quite short timeout values, it is still possible to bootload the device
// simply by having the remote master continuously transmit something (could be
// NOP data) and issuing a reset or power cycling this device. This value cannot
// be zero, although NOW_millisecond*128 might work in some cases (as would
// NOW_second/8).
#define BOOTLOADER_TIMEOUT          (NOW_second * 1ul)


// Variable representing number of instructions executed per second, FCY. Ex:
// 16000000 for many PIC24F, 70000000 for many dsPIC33E/PIC24E
unsigned long runtimeFCY;

// Declare a variable so we can control which UART module printf statements
// direct to. Ex: set __C30_UART = 1 for UART1, __C30_UART = 2 for UART2, etc.
int __C30_UART;

// Flag for tracking if the current bootloader function was called from within
// the bootloaded application, or from within the bootloader itself. Value is 0 
// when the Bootloader starts up and is changed to 1 just before dispatching 
// execution to the Application.
unsigned int EZBL_appIsRunning;

// Encryption not supported at this time, so no need to spend RAM on this
//EZBL_CRYPT_STATE EZBL_cryptState;


#if defined(__DEBUG)
void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void)
{
    // If this interrupt is unexpected, you should check all of the INTCON1 bits
    // to see if a trap is responsible.
    __builtin_software_breakpoint();

    // Clear trap(s), if any, so we can return to see where the PC was when this
    // interrupt was triggered
    INTCON1 &= 0x8700;
}
#endif //defined(__DEBUG)



// main() function
int main(void)
{
    unsigned long ledTimer = 0;
    unsigned long now;
    unsigned long timeoutStart; // NOW count at the start of an operation for timeout determination
    unsigned int  response;

    // Initialize bootloader flags so that all interrupts stay within this 
    // bootloader for now. Later, just before launching the application (or even 
    // later from within the application) these flags can be switched to '1's so 
    // the application gets the communications and timer interrupts (assuming 
    // the application even wants to take over handling of these interrupts; it 
    // can use the bootloader's NOW and communications FIFOing routines as is if 
    // it wants instead.)
    EZBL_ForwardBootloaderISR = 0x00000000;
    
    // Set processor clock, configure I/O pin states, configure certain PPS
    // functions, etc. Returns clock speed (instructions-per-second).
    runtimeFCY = InitializeBoard();

    // Initialize NOW timing module (uses Timer 1)
    NOW_Init(runtimeFCY);

    // Reset and intialize I2C communications module (and it's two FIFO submodules)
    I2C_Reset(runtimeFCY, 400000);  // Initialize I2C hardware for 400kHz (speed not actually important since we are only using I2C slave mode)

    // Start the countdown to exit the bootloader
    timeoutStart = NOW_32();

    // Initialize flag that we are currently in bootloader mode
    EZBL_appIsRunning = 0;

    
    // Main bootload loop
    while(1)
    {
        // Every 32ms toggle an LED
        now = NOW_32();
        if(now - ledTimer > (NOW_millisecond<<5))
        {
            ledTimer = now;
            EZBL_CallLongPointer1((unsigned int)&LEDToggle, 0x01);   // LEDToggle(0x01); function call, but LEDToggle() is defined as 'weak' so this will devolve to a few NOPs if LEDToggle() doesn't exist.
        }

        // Reset the watchdog timer every loop iteration. We might not be in
        // control of the watchdog if it is force enabled in configuration words
        // defined in an application instead of this project. This would be
        // particularly bad if the timeout value is shorter than 1 second such
        // that we wouldn't even jump into the application before it timed out
        // and reset the device. This is also handy in the ICSP Programming
        // projects since certain ICSP sequences are blocking and could stall
        // the PIC indefinitely if, for example, the ICSP cable were unplugged
        // in the middle of a blocking operation that lacks any means of
        // otherwise detecting such an interruption.
        ClrWdt();


        // Check if it is time to jump into the application (default is 1 second
        // of nothing being received). NOTE: "1 second" and NOW_second type
        // definitions are only correct if the config words enable the correct
        // clock frequency reported by InitializeBoard().
        if(now - timeoutStart > BOOTLOADER_TIMEOUT)
        {
            // Check if an app is present
            if(EZBL_appBootloadState == EZBL_APP_INSTALLED)
            {
                // Turn off any LEDs we might have been blinking to indicate we 
                // were in bootloader mode.
                EZBL_CallLongPointer1((unsigned int)&LEDOff, 0xFF);   // LEDOff(0xFF); function call, but LEDOff() is defined as 'weak' so this will devolve to a few NOPs if LEDOff() doesn't exist.
                
                // Flag so we know if future calls to the bootloader came from
                // the App instead of ourself
                EZBL_appIsRunning = 1;

                // App exists and timeout has occurred, jump to installed application
                __asm__ volatile("    disi  #1000                               \n" // Suppress bootloader interrupts for a while so as not to interfere with application stack initialization
                                 "    call _EZBL_APP_RESET_BASE                 \n" // Start running application
                                 "    reset                                     \n" // Reset if they return (which they shouldn't)
                                 : : : "memory");
            }

            // No app present, just continue to run the bootloader
            // communications loop until we get one uploaded to us
        }


        // Look for RX commands in the EZBL_BootloaderComTask() app callable
        // function
        response = EZBL_BootloaderComTask(1);
        if(((unsigned char*)&response)[1] == 0x00u)
        {   // Unsupported command, send NACK back
            I2C_TX_FIFO_Write8('-');
        }

        // Anything other than 0x0100 means at least one character was received
        if(response != 0x0100u)
        {
            // Just handled com RX data, reset the bootloader exit timeout counter
            // so we don't timeout in the middle of bootloading or try to start the
            // just bootloaded application while the remote host is still sending
            // verification and other commands to us.
            timeoutStart = NOW_32();
        }
    }
}

/**
 * unsigned int EZBL_BootloaderComTask(unsigned int allowSwitchToAutoBaudMode);
 *
 * Bootloader function for processing incoming commands via the communications
 * interface. This automatically checks for any pending RX data, and if none,
 * returns immediately with the value 0x0100, doing nothing. This function
 * should be called from the running application if it wants to use the
 * bootloader without issuing a device reset.
 *
 * If a command is pending, the command is removed from the FIFO and executed.
 * This will normally automatically generate queued TX data in response. The TX
 * data is written into the TX COM FIFO for asynchronous transmission via the
 * communications ISR, so this function will not normally block. Blocking will
 * only occur when more data is generated than can currently fit in the TX FIFO.
 * In such cases, this function will block only until enough data is transmitted
 * via the ISR to allow all remaining generated data to fit in the TX FIFO.
 *
 * Some remote initiated commands, like the 'MCHPb' bootload request will cause
 * the application (if present) to be erased. If this function is called from
 * your application and this happens, then this EZBL_BootloaderComTask()
 * will gracefully not return and instead handle the full bootload operation
 * until complete. The device will then reset so the new application can run (if
 * successful), or so that the rest of the bootloader can take over for future
 * retries (if unsuccessful).
 *
 * This function should be called periodically to keep the bootloader alive and
 * always ready to receive new firmware. Max time between successive calls 
 * should be less than 1 second (if auto-baud is being used) as long delays will 
 * place the state machine back into an auto-baud state and interfere with 
 * bootloader discovery by an external host wishing to update us.
 *
 * If the bootloader's communications or Timer 1 interrupt are forwarded into
 * the application via the EZBL_ForwardBootloaderISR flags variable, this
 * function should no longer be called. This will put the bootloader into a
 * dormant state. Writing 0x00000000 to EZBL_ForwardBootloaderISR and
 * resuming periodic calls to this function will return the bootloader to a
 * ready state. If using this mechanism, be sure that the Timer 1 and
 * communications peripheral settings are still valid and that the associated
 * Interrupt Enables are still set.
 *
 * This function is not reentrant safe, so don't call it from an ISR and the
 * main() context of your application simultaneously. However, it should work
 * correctly if called from an ISR that is set to a lower priority than the
 * Timer 1 and communications channel interrupts.
 *
 * NOTE: This function is not in the ezbl_lib16.a or ezbl_lib16ep.a archive 
 * libraries. You must declare and implement this function in your bootloader
 * project in order to be able to call this function.
 * 
 * @param allowSwitchToAutoBaudMode Flag indicating if the bootloader is allowed 
 *                                  to use the auto-baud feature. When using 
 *                                  communications mediums that do not need or 
 *                                  support auto-baud, then this parameter is 
 *                                  ignored.
 *                                  0 = do not change auto-baud feature. If
 *                                      already enabled, it will remain enabled
 *                                      until completed by hardware. If not
 *                                      enabled, the baud rate will stay fixed
 *                                      at the same baud rate setting.
 *                                  1 = bootloader is allowed to turn on
 *                                      auto-baud. This will very likely
 *                                      interfere with application code if it
 *                                      needs to share the communications
 *                                      channel.
 *
 * @return 16-bit unsigned integer who's upper and lower bytes define the
 *         meaning:
 *         0x0100 = No commands waiting; nothing took place, except auto-baud
 *                  control, if allowed.
 *         0x80xx = Successfully read and executed a bootloader command
 *                  character. The lower 8-bits specify which command character
 *                  was processed.
 *         0x00xx = Unknown command code read. The lower 8-bits specify which
 *                  command character was received unexpectedly.
 *         Other values are undefined and will not be returned in this
 *                  bootloader version, but should be treated as reserved as
 *                  future versions could define them.
 *         No return = Generally means that a bootload took place and the device
 *                  will reset instead. The appearance of no return is also
 *                  possible if the the communications medium is synchronous
 *                  (ex: I2C slave) and the TX FIFO is full. As an I2C slave,
 *                  TX data can only be sent when the I2C master issues a read
 *                  request. Therefore, this case would actually indicate an
 *                  indefinite wait for the I2C master, which, when it occurs,
 *                  will then cause a return to occur normally.
 */
unsigned int EZBL_BootloaderComTask(unsigned int allowSwitchToAutoBaudMode)
{
    static char bootUnlockState = 0x00; // Unlock state for stepping through {MCHPb} unlock sequence
    char nextBootUnlockState;
    unsigned long currentTime;
    unsigned long addressL;             // Program memory address/start of program memory address range
    //unsigned long addressH;             // End program memory address - only used for commented out data dump functionality
    char c;                             // COM RX character for processing commands
    unsigned int i;                     // Loop counters
    signed int ret;                     // Variable for storing return codes
    unsigned int tableEntries;          // Loop counter for transmitting table values
    unsigned long isrForwardMaskBit;

    // Return immediately if the bootloader's communications RX interrupt is 
    // being forwarded to the application right now.
    EZBL_GetWeakSymbol(isrForwardMaskBit, EZBL_FORWARD_MASK_SI2C1);
    if(EZBL_ForwardBootloaderISR & isrForwardMaskBit)
        return 0x0000;
    
    // Do nothing if nothing received
    if(I2C_RX_FIFO_ReadableLength() == 0u)
    {
        return 0x0100;   // Upper char = 0x01 means nothing to do, lower char = 0xxx as no data
    }
    
    // Get a command character
    c = (char)I2C_RX_FIFO_Read8();

    // All values received other than {MCHPb} cause the bootUnlockState to
    // reset, so choose the reset state as the default value and we can override
    // it later if the state is following the correct unlock path.
    nextBootUnlockState = 0x00;

    // Decode and process the incoming command character
    switch(c)
    {
        case 0x55:   // AUTO-BAUD/NOP character ('U' = 0x55): just echo it back. These are for signaling that auto-baud has been successfully completed. This is effectively a NOP on communications mediums that don't need auto-baud or when auto-baud is already complete.
            I2C_TX_FIFO_Write8(c);
            break;

        case 'E':   // Echo string request
            // This is handy to check if the communications medium is working 
            // reliably. To maximize performance, it is desirable to auto-baud 
            // to a maximal baud rate. However, when attempting to do so, it 
            // is possible that we will successfully send and receive the 0x55 
            // 'U' character, but certain other bytes may fail or be received 
            // (by either node) as the wrong value. By receiving and 
            // retransmitting several bytes, you can quickly test if various bit 
            // patterns make it through the chain or not.
            // Input:   16 byte string to echo back.
            // Output:  16 byte string we saw
            currentTime = NOW_32(); // Get a starting timestamp
            for(i = 0; i < 16; i++)
            {
                // Wait for 1 byte waiting for read and 1 byte free in TX buffer for write, but with a timeout
                while(((I2C_RX_FIFO_ReadableLength() == 0u) || (I2C_TX_FIFO_WritableLength() == 0u)) && (NOW_32() - currentTime < (NOW_second>>2)));

                // Abort if we spend more than 125ms total doing this echo. All
                // data should arrive in a burst so we can quickly move on and 
                // test a different baud rate if this one fails.
                if(NOW_32() - currentTime >= (NOW_second>>2))
                    break;
                I2C_TX_FIFO_Write8(I2C_RX_FIFO_Read8());
            }
            break;

        case '?':   // Device query string: Return null terminated string
            // Input  0 bytes
            // Output variable length character string (includes null terminator)
            I2C_TX_FIFO_Write(DEVICE_QUERY_STRING, sizeof(DEVICE_QUERY_STRING));
            break;

        case '#':   // Query DEVID, DEVREV, and RX buffer size
            // Remote node requesting device ID, revision ID, and COM transfer
            // buffer size
            // Input  0 bytes
            // Output 13 bytes
            //  byte[0]: Command Acknowledgment character, '+'
            //  byte[4:1]: 32-bits DEVID read-back value (byte[4] is 0x00)
            //  byte[8:5]: 32-bits DEVREV read-back value (byte[8] is 0x00)
            //  byte[12:9]: RX buffer size, in bytes
            I2C_TX_FIFO_Write8('+');
            I2C_TX_FIFO_Write32(EZBL_ReadDEVID());  // Obtain DEVID
            I2C_TX_FIFO_Write32(EZBL_ReadDEVREV()); // Obtain DEVREV
            I2C_TX_FIFO_Write32(I2C_RX_FIFO_WritableLength() + I2C_RX_FIFO_ReadableLength());
            break;

        case 'g':
            // Remote node requesting table defining the non-volatile device
            // memory geometries, bootloader reserved address regions, and app
            // space regions (subtraction of the two, so rather redundant, but
            // still useful for verification)
            // Input  0 bytes
            // Output 7+6*n bytes, where n is the number of non-contiguous
            // memory regions present in all three region tables.
            //  byte[0]: Command Acknowledgment character, '+'
            //  byte[2:1]: 16-bit memory region count that follows. NOTE: this
            //             is shrunken to 16-bits, not the full 24-bits that
            //             will exist when read from Flash. We will not be
            //             supporting more than 65535 fragments.
            //  byte[3+(6*n)-1:3+6*(n-1)]: 24-bit starting address and 24-bit
            //                           ending address pairs where the starting
            //                           address is inclusive and the ending
            //                           address is exclusive. n is the index
            //                           into the table, starting at 1 for the
            //                           first element.
            // byte[m+2:m]: 16-bit memory region count that follows for the
            //              bootloader reserved space table
            // byte[...]: Next table address pairs, followed by final app space
            // table region count, and it's address pairs.
            EZBL_GetSymbol(addressL, EZBL_NV_GEOMETRY_TABLE_BASE);
            I2C_TX_FIFO_Write8('+');
            for(i = 0; i < 3; i++)  // NV_GEOMETRY_TABLE, ROM_USE_TABLE, and APP_SPACE_TABLE, in this order
            {
                tableEntries = (unsigned int)EZBL_ReadFlashInstruction(addressL);
                addressL += 2;
                I2C_TX_FIFO_Write16(tableEntries);
                tableEntries <<= 1; // Multiply by 2 since each table entry consists of a start address and end address that will be read/transmitted individually
                while(tableEntries--)
                {
                    I2C_TX_FIFO_Write24(EZBL_ReadFlashInstruction(addressL));
                    addressL += 2;
                }
            }
            break;

// This is commented out because it would be a blatant security hole if you were
// using Code Protect. It is useful for debugging though. If you want to do a
// read for the purpose of verification, you can use one or more of the CRC
// commands instead. These will avoid the slowness of having to transmit all of
// the Flash contents and leaking too much code information, but can still be
// used to externally check what application or bootloader version is programmed
// on the device (assuming you keep track of the generated CRCs for each
// firmware update.
//        case 'F':   // Dump all Main Flash
//            addressL = 0;
//            EZBL_GetSymbol(addressH, EZBL_MAIN_FLASH_END_ADDRESS);
//            while(addressL < addressH)
//            {
//                I2C_TX_FIFO_Write24(EZBL_ReadFlashInstruction(addressL));
//                addressL += 2;
//            }
//            break;

// This command is commented out too because it would not be very secure to 
// allow outsiders to jump to arbitrary program addresses. It also isn't 
// normally needed for a bootloader. On the other hand, uncommenting this 
// reference implementation could be handy if you wanted to extend bootloader 
// functionality by uploading short lived code into the application space and 
// then jump to it for execution without resetting the device, bootloader or 
// communications channel. You wouldn't have to wait for the 1 second reset 
// application launch delay to expire and can return and pick up bootloading 
// right where you left off. A PC, for example, wouldn't have to reenumerate you
// through the process if you were using USB for communications.
//        case 'j':   // Reserved for PC jump (function call) operation triggered from remote.
//            // Input: 4 bytes (jump target address reached via a long call)
//            // Output: 2 bytes: '+' if the jump is being taken, another '+' when 
//            //                 the function call returns, or '--' if a timeout 
//            //                 or other error occurred. Note that if the function 
//            //                 call does not return, the second '+' will not be 
//            //                 transmitted. The timeout does not check time 
//            //                 spent at the called address, only correct 
//            //                 reception of the 4 address bytes. If 4 bytes 
//            //                 don't arrive, any bytes that did arrive will be 
//            //                 processed as new commands, not consumed here.
//            for(currentTime = NOW_32(); (UART_RX_FIFO_ReadableLength() < 4u) && (NOW_32() - currentTime < NOW_second); );
//            if(I2C_RX_FIFO_ReadableLength() >= 4u)
//            {
//                I2C_TX_FIFO_Write8('+');
//                EZBL_CallLongPointer0(I2C_RX_FIFO_Read32());   // Opcode expects 4 additional jump target address bytes. 
//                I2C_TX_FIFO_Write8('+');
//            }
//            else
//            {
//                I2C_TX_FIFO_Write16('-' | (((unsigned int)'-')<<8));
//            }
//            break;

            
        case 'c':   // CRC32 of all non-volatile memory regions
        case 'd':   // CRC32 of bootloader reserved non-volatile memory regions
        case 'e':   // CRC32 of application space (non-bootloader) non-volatile memory regions; includes CRC over unprogrammed regions
            // Input: 0 bytes
            // Output: 5 bytes
            //      byte[0] = command acknowledgment character, '+'
            //      byte[4:1] = computed CRC32
            I2C_TX_FIFO_Write8('+');
            // Compute and send the CRC32 to the remote host
            I2C_TX_FIFO_Write32(EZBL_CRC32NVRegions(c - 'c'));
            break;

        case 'M':
        case 'C':
        case 'H':
        case 'P':
            if((bootUnlockState == 0x00) && (c == 'M'))
                nextBootUnlockState = c;
            else if((bootUnlockState == 'M') && (c == 'C'))
                nextBootUnlockState = c;
            else if((bootUnlockState == 'C') && (c == 'H'))
                nextBootUnlockState = c;
            else if((bootUnlockState == 'H') && (c == 'P'))
                nextBootUnlockState = c;
            break;

        case 'b':   // Binary bootloader protocol. Interfaces to EZBL Tools Communicator. Requires the 'MCHP' unlock characters to be sent first or nothing will happen.
            if(bootUnlockState == 'P')
            {
                // Send acknowledgment of command (remote node should not send data
                // yet)
                I2C_TX_FIFO_Write8('+');

                // If called from the running application, disable all but our own
                // interrupts since we are about to erase the previously running app
                if(EZBL_appIsRunning)
                {
                    // Clear all interrupt enables
                    EZBL_RAMSet((void*)&IEC0, 0x00, &IPC0 - &IEC0);

                    // Ensure all bootloader interrupts stay within the bootloader
                    EZBL_ForwardBootloaderISR = 0;

                    // Turn our interrupts back on
                    NOW_Init(runtimeFCY);               // Pauses the timer, but doesn't reset the actual count; restores interrupt enable
                    I2C_FIFO_EnableInterrupts();        // Need communications interrupts to get the new firmware.
                }

                // Erase the device and send back a period for each erased page
                // (possibly off by 1 since first call may or may not erase anything)
                addressL = 0;
                while(1)
                {
                    EZBL_NVMKey = 0xFC21;    // Required unlock value for EZBL_EraseAllPiecewise()
                    addressL = EZBL_EraseAllPiecewise(addressL);

                    if(addressL == 0xFFFFFFFFu)
                        break;

                    I2C_TX_FIFO_Write8('.');
                }

                // Send second acknowledgment to request the actual data starts
                // being sent to us. This is needed so the remote node doesn't send
                // us data while we are busy erasing. Doing so could be subject
                // to COM RX overflow on certain asynchronous communications
                // mediums (ex: UART) since the erase time will be greater
                // than the COM RX hardware FIFO size can handle (4 bytes on
                // normal UART modules). Ex: page erase takes 20.1ms while
                // 4 bytes @ 115200 baud is only 347us. If the page erase blocks
                // the CPU, overflow would likely occur.
                I2C_TX_FIFO_Write8('+');

                // Program the blob and decode the return status
                ret = EZBL_InstallBlobFromCOM(1000);
                
                // Send a zero byte request (first 16-bits), indicating
                // termination (either success or failure), followed by a 16-bit
                // return status code, which if negative means failure, or
                // positive indicating success.
                I2C_TX_FIFO_Write32((((unsigned long)ret)<<16) | 0x0000u);

                // If we were called while the application was previously running
                // (which we erased and hopefully overwrote with something newer),
                // we must not return to a mystery address. Instead, let's reset to
                // let the new Config Words take effect (if applicable) and reenter
                // the bootloader proper (eventually dispatching to the installed
                // application).
                if(EZBL_appIsRunning)
                {
                    I2C_TX_FIFO_WaitUntilFlushed();    // Wait for any pending characters to transmit
                    __asm__ volatile("reset" : : : "memory");
                }
                break;
            }

            break;

        default:    // Unrecognized command: pass back to caller with upper char clear so they can process it if desired
            bootUnlockState = 0x00;
            return (unsigned int)c;
    }

    // Set the new bootload command unlock state (always goes to 0x00 unless exactly following the unlock sequence)
    bootUnlockState = nextBootUnlockState;

    // Return last command processed with bit 15 set to indicate successful
    // processing.
    return 0x8000 | (unsigned int)c;
}

/**
 * signed int EZBL_InstallBlobFromCOM(unsigned int millisecondTimeout);
 *
 * Reads a .blob file from the communications medium and decodes it, decrypts it
 * (if applicable), and then programs the device Flash with the decoded .blob
 * contents. This function must be called when the very next character that will
 * come out of the RX communications channel is the first byte of a .blob file.
 *
 * This function will block until either the whole .blob file is received and
 * successfully programmed or anytime earlier if something goes wrong.
 *
 * Before calling this function, the device (application space) Flash should be
 * in an erased state. Erasing can be handled externally via the
 * EZBL_EraseAll() API or EZBL_EraseAllPiecewise() if granular chunked
 * erases are desired (for status reporting, com timeout management, ISR
 * performance, etc.).
 *
 * If the Flash is not erased, but the .blob file only has data records for
 * Flash addresses that are all erased, then it is still legal to call this
 * function. Doing so will allow more data to be programmed independently of the
 * original application. Although no demo exists to mange this or help create
 * suitable .blob's for this purpose, this could be used as a basis to create a
 * multi-image project. For example: keep presently installed application
 * installed and unmodified while bootloading new firmware to unused Flash so
 * there is never a possibility of bootload failure causing a temporary brick.
 * (Bootload failure is very unlikely to damage the bootloader, however, so
 * application recovery will generally always be feasible without performing
 * multi-image operations).
 *
 * Because reception, decryption and programming of a whole .blob file could
 * take an extended period, this function internally calls the ClrWdt()
 * function to avoid possible unintended device reset. As communications
 * operations do not have deterministic timing, you must NOT enable the watchdog
 * timer in a Windowed mode. Only traditional watchdog timing will work. Also,
 * this function will not work if the watchdog timeout is too short. The
 * watchdog is only cleared when a chunk of data is received and processed,
 * which will be up to sizeof(programBuffer) bytes long (384 byte by default).
 * Additional time may be needed for the programming operations.
 * 
 * NOTE: This function is not in the ezbl_lib16.a or ezbl_lib16ep.a archive 
 * libraries. You must declare and implement this function in your bootloader
 * project in order to be able to call this function.
 * 
 * @param millisecondTimeout A timer to ensure communications interruptions
 *                           won't stall indefinitely so that a retry can be
 *                           attempted. This timeout is with respect to
 *                           contiguous com silence, not for the time it takes
 *                           to perform the full .blob reception and
 *                           programming. I.e. any time the internal state
 *                           advances, the timer is reset. Set this value from
 *                           1 to 65535 milliseconds such that the timeout is
 *                           longer than the worst case communications latency.
 *
 * @return Signed integer error code. Generally zero or greater is good, while
 *         negative values indicate a failure. In all failing cases, the
 *         application will not be flagged as runnable, so subsequent device
 *         reset will keep execution within the bootloader, available for retry.
 *         Anything programmed prior to a failure is not erased or otherwise
 *         clean up.
 *
 *         Specific return code meanings are:
 *          1 = Programming of the full .blob completed successfully. The .blob
 *              file and communications integrity was confirmed with a valid
 *              CRC32, and all data programmed passed read-back verification.
 *         -1 = Communications timeout waiting for the .blob header
 *         -2 = Communications timeout waiting for a record header
 *         -3 = .blob or record header contains illegal size information
 *              (.blob corrupt or incorrect RX data)
 *         -4 = Record header contained illegal address information
 *              (.blob corrupt or incorrect RX data)
 *         -5 = Communications timeout waiting for record data
 *         -6 = Communications timeout waiting for final 4 byte CRC
 *         -7 = CRC mismatch (.blob corrupt or incorrect RX data)
 *         -8 = Read back verify failure (.blob record targeted an address that
 *              wasn't erased, isn't implemented on this device, or wasn't
 *              otherwise correctly programmed). If the blob is known to be good
 *              and built for this bootloader, check device voltage, proper
 *              operating frequency, and that no ISR code bugs could potentially
 *              have corrupted RAM.
 *         No return = Unhandled exception or unknown. (.blob may have
 *              records targeting unimplemented memory, causing an Address
 *              Error Trap upon verification, or electrical operating parameters
 *              are incorrect.) Ensure the .blob file was compiled specifically
 *              for this device and bootloader. For unhandled exceptions, the
 *              device will automatically reset back into the bootloader.
 */
signed int EZBL_InstallBlobFromCOM(unsigned int millisecondTimeout)
{
    EZBL_BLOB_HEADER blobHeader;
    struct
    {
        unsigned long length;
        unsigned long address;
    } recordHeader;
    unsigned char programBuffer[384];   // Must be a multiple of 3 bytes (for single instruction programming, if possible), 6 bytes (for 48-bit Flash word programming), or 384 bytes (for 128-instruction row programming)
    unsigned char readBackBuffer[sizeof(programBuffer)];    // Must match size of programBuffer
    unsigned int chunkLen;              // Chunk size for communications handling
    unsigned long nextAddress;
    unsigned long incomingDataCRC = 0;
    unsigned long appBootloadStateAddress;
    unsigned long reservedBitAddress;
    unsigned long reservedBitMask;
    unsigned long maskAddress;
    unsigned int arrayIndex;
    unsigned int i;
    signed int ret = EZBL_ERROR_SUCCESS;

    
    // Fetch the address of the EZBL_appBootloadState in Flash since we will 
    // need to mask this address off during read-back verification.
    EZBL_GetSymbol(appBootloadStateAddress, EZBL_appBootloadState);
    
    // Fetch a value needed for checking the special Reserved bit in
    // Flash-based Configuration Words. The reserved bit doesn't belong
    // to the bootloader (unless the bootloader programs something else
    // in the same config word). However, it will normally/correctly
    // read-back as zero even when the .hex/.blob file doesn't program
    // it to '0'.
    EZBL_GetWeakSymbol(reservedBitAddress, EZBL_RESERVED_BIT_ADDRESS);
    
    // Encryption is not supported right now, so this needs to be commented out
    //EZBL_cryptState.offset = 0;      // Starting a new blob, so set decryption state to begin as well

    // Get blob Header
    if(EZBL_COMReadWithTimeout(&blobHeader, sizeof(blobHeader), &incomingDataCRC, millisecondTimeout) != sizeof(blobHeader))
    {
        return EZBL_ERROR_TIMEOUT_IN_BLOB_HEADER;  // Error (-1): Communications timeout attempting to read the first 4 bytes of the .blob file (where the file's length is contained)
    }

    // Subtract blob header length from total blob length
    blobHeader.length -= sizeof(blobHeader);


    // Collect the remainder of the blob and write it to Flash
    recordHeader.length = 0;    // Initialize so we start by reading a record header
    while(blobHeader.length > 4u) // Don't process last 4 byte CRC32
    {
        if(recordHeader.length == 0u)
        {
            // Get a Record Header
            if(EZBL_COMReadWithTimeout(&recordHeader, sizeof(recordHeader), &incomingDataCRC, millisecondTimeout) != sizeof(recordHeader))
            {
                return EZBL_ERROR_TIMEOUT_IN_RECORD_HEADER;  // Error (-2): Communications timeout attempting to read a record header from the .blob file
            }
            blobHeader.length -= sizeof(recordHeader);
            recordHeader.length -= sizeof(recordHeader);
        }

        // Sanity check values and ensure we never underflow the blob header
        if((recordHeader.length | blobHeader.length) & 0xFF800000)
        {
            return EZBL_ERROR_ILLEGAL_LENGTH;           // Error (-3): Communications corruption occurred or the .blob file contains an illegally long length field in a data record or the overall .blob header. The PIC24/dsPIC architecture only has 24-program memory address bits, and no record should be anywhere near that big in practice due to unimplemented memory regions or unprogrammed locations causing data to split into two separate records.
        }
        if(recordHeader.address >= 0x01000000ul)
        {
            return EZBL_ERROR_ILLEGAL_RECORD_ADDRESS;   // Error (-4): Communications corruption occurred or the .blob file contains an illegally high record address. The PIC24/dsPIC architecture only has 24-program memory address bits.
        }

        // Ensure we don't try to read/write a chunk that is larger
        // than the remaining expected bytes
        chunkLen = sizeof(programBuffer);
        if(chunkLen > recordHeader.length)
        {
            chunkLen = recordHeader.length;
        }

        // Wait for a full chunk to arrive
        if(EZBL_COMReadWithTimeout(programBuffer, chunkLen, &incomingDataCRC, millisecondTimeout) != chunkLen)
        {
            return EZBL_ERROR_TIMEOUT_IN_RECORD_DATA;  // Error (-5): Communications timeout trying to read .blob record data.
        }

        // Get a copy of the original data we are being asked to write to Flash
        // before actually performing the write. This is needed for read-back
        // verification since the EZBL_WriteROMChecked() function will mask
        // off bootloader Flash regions to all 0xFFFFFF's (programBuffer being
        // passed in) and then skip them during programming, so the data will
        // no longer match what we expect to read from Flash after the write is
        // complete.
        EZBL_RAMCopy(readBackBuffer, programBuffer, chunkLen);

        // Program this chunk to Flash
        ClrWdt();
        EZBL_NVMKey += 0x03DF;
        nextAddress = EZBL_WriteROMChecked(recordHeader.address, programBuffer, chunkLen);   // Program data to Flash. This function will skip addresses within bootloader regions.
        EZBL_RAMCopy(programBuffer, readBackBuffer, chunkLen);                               // Restore programBuffer contents since programming may have masked off bootloader regions

        // Read what now exists in Flash at the given programming location
        EZBL_ReadPackedFlash(readBackBuffer, recordHeader.address, 0, chunkLen);

        // Verify that all data matches what we were asked to program to in the
        // .blob file
        if(EZBL_RAMCompare(programBuffer, readBackBuffer, chunkLen) != 0u)
        {
            // Read data does not match write data. This could be due to masking
            // off bootloader regions and having the bootloader region contain
            // the wrong bootloader reference code in the .blob (i.e. you
            // inadvertently compiled against modified bootloader source code 
            // when you've already deployed your product and therefore aren't
            // allowed to make bootloader changes), or possibly because we hit a
            // reserved address having a 0 already in it, so we must check if
            // this is an actual Flash programming failure or just an expected
            // "we were asked to do something bad and we prevented it (saving
            // you from bricking yourself)" type failure.

            // Check if the data mismatch is due to Non-volatile Config fuse
            // byte data. These fuses can have unimplemented bits in them, so
            // while the compiler may generate '1' bits in unused locations,
            // they will correctly read back as '0' bits.
            if((recordHeader.address & 0xFFFF00u) == 0xF80000u)
            {
                // Yes, we are working with Config fuse bytes. These are hard to
                // verify completely because the mismatch is both expected and
                // because the presence and locations of unimplemented fuse bits
                // varies between devices. To handle this problem, we'll switch
                // strategies and only verify that all bits we were asked to
                // write a '0' to do indeed have a '0' stored there. '1's won't
                // be verified since we have no way of knowing if a '1' saved in
                // the .hex file by the compiler is a don't care/unimplemented
                // bit, or a value that really must be programmed to '1'.
                for(i = 0; i < chunkLen; i++)
                {
                    programBuffer[i] &= readBackBuffer[i];
                }
            }
            else if(reservedBitAddress)  // Devices with Volatile Flash-based Configuration Words have a reserved bit that needs correcting because it reads as '0' like an unimplemented Config Fuse bit.
            {
                maskAddress = recordHeader.address + EZBL_Div3Mul2(chunkLen, 0);
                if((recordHeader.address <= reservedBitAddress) && (maskAddress > reservedBitAddress))
                {
                    arrayIndex = EZBL_Mul3Div2(reservedBitAddress - recordHeader.address);
                    EZBL_GetWeakSymbol(reservedBitMask, EZBL_RESERVED_BIT_MASK);
                    for(i = 0; i < 2; i++)  // Seek to the correct byte in the program word
                    {
                        if((reservedBitMask & 0xFF) == 0x00u)
                        {
                            arrayIndex += 1;
                            reservedBitMask >>= 8;
                        }
                    }
                    programBuffer[arrayIndex] &= ~((unsigned char)reservedBitMask);     // Mask off the reserved bit, forcing it to '0' in both the read-back and expected data
                    readBackBuffer[arrayIndex]  &= ~((unsigned char)reservedBitMask);   // Mask off the reserved bit, forcing it to '0' in both the read-back and expected data
                }
            }

            // Check if this programming range includes EZBL_appBootloadState, 
            // and if so, mask the program data to all '1's to match what should 
            // exist during verification
            maskAddress = recordHeader.address + EZBL_Div3Mul2(chunkLen, 0);
            if((recordHeader.address <= appBootloadStateAddress) && (maskAddress > appBootloadStateAddress))
            {
                arrayIndex = EZBL_Mul3Div2(appBootloadStateAddress - recordHeader.address);
                programBuffer[arrayIndex++] = 0xFF;
                programBuffer[arrayIndex] = 0xFF;
            }

            // Now recompare to see if all application data has been programmed
            // correctly
            if(EZBL_RAMCompare(programBuffer, readBackBuffer, chunkLen) != 0u)
            {
                // Still data mismatch: could be due to bootloader contents not 
                // matching with .blob contents for bootloader regions. Let's try 
                // masking off all bootloader data and see if it matches then.
                // In both the program data buffer and the read back buffers, mask
                // off the bootloader regions so their locations will contain all
                // '1's.
                EZBL_MaskBootloaderRegions(recordHeader.address, programBuffer, chunkLen);
                EZBL_MaskBootloaderRegions(recordHeader.address, readBackBuffer, chunkLen);
                if(EZBL_RAMCompare(programBuffer, readBackBuffer, chunkLen) != 0u)
                {
                    // Again data mismatch; this looks like a write failure, but
                    // unless your Flash is very worn out, or you are programming at
                    // an illegally low voltage, it is much more likely that
                    // somebody already programmed something into the needed address
                    // range so it isn't erased. Make sure your .hex file (and
                    // therefore .blob file) doesn't have any data records in
                    // it that overlaps with another data record in the same file.
                    // Also make sure that the needed Erase operations actually took
                    // place.
                    return EZBL_ERROR_READ_BACK_VERIFICATION;  // Error (-8): Read-back verification mismatch. Probable configuration error or write protected memory.
                }
                
                // Masking off the bootloader addresses fixed the mismatch, but 
                // lets record this error and continue as if no error happened 
                // yet since the application might still be close enough to 
                // work. We also don't want to fail with an error immediately 
                // since it would allow an attacker to continually attempt 
                // sending test data and very slowly recover the unencrypted 
                // contents of the bootloader (which would be very bad since the 
                // bootloader would likely have a decryption key in it). By 
                // completing all other operations and only then signaling the 
                // mismatch, we can reduce leaking information as to what 
                // address(es) within the bootloader/.blob contents caused the 
                // data mismatch, thus making such an attack far less feasible.
                ret = EZBL_ERROR_BOOTLOADER_MISMATCH;          // Error (-9): Read-back verification mismatch. All programming completed, but data in the existing bootloader does not match the bootloader copy in the uploaded image. Make sure you transmitted a correct .hex/.blob file that exactly matches and was built for the installed bootloader. The Application must be compiled with _merge.s and _merge.gld files generated when the bootloader was originally built and deployed.
            }
        }

        recordHeader.address = nextAddress;
        blobHeader.length -= chunkLen;
        recordHeader.length -= chunkLen;
    }

    // Read the last 4 bytes of the blob, which is the CRC32 over all prior
    // bytes. nextAddress is just a conveniently aligned 4 byte value of the
    // correct data type to use as the destination.
    if(EZBL_COMReadWithTimeout(&nextAddress, 4, 0, millisecondTimeout) != 4)
    {
        return EZBL_ERROR_TIMEOUT_IN_CRC;  // Error (-6): Communications timeout reading last 4 byte CRC field.
    }

    // Validate that what the remote host sent is what we received (to ensure
    // there was no communications channel corruption)
    if(nextAddress != incomingDataCRC)
    {
        return EZBL_ERROR_BLOB_CRC;  // Error (-7): CRC of received .blob contents mismatch with CRC contained in .blob. Probable communications corruption.
    }


    // Whole .blob received and programmed, write a flag indicating successful
    // bootload
    i = EZBL_APP_INSTALLED;                      // Flag value indicating success
    EZBL_NVMKey += 0x03DF;
    EZBL_WriteROMOnlyOnce(appBootloadStateAddress, &i, 2);   // Uses EZBL_WriteROMOnlyOnce() instead of EZBL_WriteROMChecked() since this is considered a bootloader region and therefore cannot be programmed with EZBL_WriteROMChecked();

    // Send a final ACK to signify programming complete
    return ret;   // EZBL_ERROR_SUCCESS: Success (1): Operation completed successfully. Signal successful bootload back to host/upper layer
                  // or
                  // EZBL_ERROR_BOOTLOADER_MISMATCH: Error (-9): Read-back verification mismatch. All programming completed, but data in the existing bootloader does not match the bootloader copy in the uploaded image. Make sure you transmitted a correct .hex/.blob file that exactly matches and was built for the installed bootloader. The Application must be compiled with _merge.s and _merge.gld files generated when the bootloader was originally built and deployed.
}


/**
 * Copies the specified number of bytes out of the EZBL COM RX software FIFO and
 * into the specified destination memory buffer. Software protocol data 
 * byte requests, if applicable, are transmitted to implement flow control
 * and not overrun our hardware ability to receive.
 *
 * In the event no new characters arrive for a period longer than the specified
 * timeout (in milliseconds), the function aborts the read and returns all bytes
 * that were read until the timeout occurred.
 *
 * This function can optionally compute or update a CRC32 for each of the bytes
 * read.
 *
 * NOTE: This function is not in the ezbl_lib16.a or ezbl_lib16ep.a archive 
 * libraries. You must declare and implement this function in your bootloader
 * project in order to be able to call this function.
 * 
 * @param *dest pointer to RAM memory to write the incoming RX data to.
 *
 * @param bytesToRead Number of bytes that should be read from the RX channel
 *                    before returning. At least this many bytes of RAM must be
 *                    allocated at the *dest memory location.
 * @param *crc32      Pointer to a CRC32 value to which the data read should be
 *                    added. If starting a new CRC, the CRC's initial value
 *                    should generally be set to 0x00000000. 
 * 
 *                    This parameter is optional and can be set to null if no
 *                    CRC computation is needed.
 *
 *                    If the RX data stream is encrypted, this CRC is applied to
 *                    the actual RX data received, before performing decryption.
 * @millisecondTimeout Number of milliseconds of elapsed silence to wait for
 *                     more RX characters before giving up and returning
 *                     everything that was received.
 *
 *                     This timing parameter is with respect to the most
 *                     recently received character, not the time from when the
 *                     function was called. Therefore, on slow communications
 *                     mediums or when reading large blocks of data, this
 *                     function may block for a long time. The timeout need only
 *                     be adjusted for the medium's worst case latency, not for
 *                     the ratio of the baud rate/bandwidth to the bytesToRead.
 *
 * @return Number of characters written to *dest. For successful operation, the
 *         return value will match the bytesToRead parameter passed in. If
 *         timeout occurs, the return value will be smaller than the bytesToRead
 *         passed in.
 */
unsigned int EZBL_COMReadWithTimeout(void *dest, unsigned int bytesToRead, unsigned long *crc32, unsigned int millisecondTimeout)
{
    unsigned int chunkSize;
    unsigned long timeout;
    unsigned long startTime;
    unsigned int bytesRead;
    unsigned int bytesRequested;

    // Initialize timeout, return parameters, and things that are constant
    startTime = NOW_32();
    timeout = NOW_millisecond * millisecondTimeout;
    bytesRead = 0;
    bytesRequested = 0;

    // Loop until all bytes requested are received (or we timeout internally)
    while(bytesRead != bytesToRead)
    {
        // Wait doing nothing but checking timeout until some data is waiting
        // for us
        while(1)
        {
            // Check for available data
            chunkSize = I2C_RX_FIFO_ReadableLength();
            if(chunkSize)
            {   // Indeed, data is waiting for us, break out so we can use it
                break;
            }

            // Check for timeout
            if(NOW_32() - startTime >= timeout)
            {   // Indeed nothing received for too long, terminate
                return bytesRead;
            }
        }

        // Limit the amount of bytes we try and read to no more than what is
        // currently waiting. This is done so we can timeout instead of having a
        // difficult to recover from blocking loop above.
        if(chunkSize > bytesToRead - bytesRead)
        {
            chunkSize = bytesToRead - bytesRead;
        }
        bytesRequested -= chunkSize;

        // Copy the determined chunk of data out of the COMs FIFO
        I2C_RX_FIFO_Read(dest, chunkSize);

        // Add the just read data to the CRC32, if non-null pointer provided
        if(crc32)
        {
            *crc32 = EZBL_CRC32(*crc32, dest, chunkSize);
        }

        // Decrypt this data chunk, if applicable (encryption/decryption not supported right now, so commented out)
//        EZBL_CryptBlob(&EZBL_cryptState, dest, dest, chunkSize);

        // Advance pointers and counters
        dest += chunkSize;
        bytesRead += chunkSize;

        // Reset the timeout since data successfully arrived
        startTime = NOW_32();
    }

    // Successfully read all requested data
    return bytesRead;
}


/**
 * unsigned long __attribute__((weak)) InitializeBoard(void);
 * 
 * A dummy weak function for allowing us to compile this project on devices 
 * which we don't have code written for actually initializing anything. Since 
 * this is weak, any ordinary InitializeBoard() function which is implemented 
 * will take precedence and this code will get discarded, allowing correct 
 * operation.
 * 
 * @return NOW_systemFrequency  (whatever value was given when NOW_Init() was 
 *                              last called)
 */
unsigned long __attribute__((weak)) InitializeBoard(void)
{
    return NOW_systemFrequency;
}

