/*******************************************************************************
 *
 * Time Counting and Measurement Functions of NOW library
 * 
 * Includes routines for measuring absolute time, polled action triggering and 
 * measuring and tuning oscillators, such as your Fast internal RC oscllator 
 * (FRC), Digitally Controlled Oscillator (DCO), or other clock source.
 * 
 *******************************************************************************
 * FileName:        now.h
 * Dependencies:    Timer 1
 * Processor:       PIC24, dsPIC33, PIC32
 * Compiler:        Microchip XC16 v1.21 or higher
 *                  Microchip XC32 v1.40 or higher
 * Company:         Microchip Technology, Inc.
 * Author:          Howard Schlunder
*******************************************************************************/

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

#ifndef NOW_H
#define	NOW_H


// Static macros for concatenating tokens together without making them strings
// first. This is useful for prefixing letters or words onto functions,
// variables, and other symbol names.
// Ex: you can write 1 to the T2CONbits.ON register bit using:
// #define TIMER_SELECT  2
//     CAT3(T,TIMER_SELECT,CONbits.ON) = 1;
// The preprocessor will resolve this into:
//     T2CONbits.ON = 1;
#if !defined(CAT2)
#define CAT2_IN(a0,a1)              a0##a1                      // Not recommended to use this macro directly; macro expansion won't occur unless another macro wraps it.
#define CAT3_IN(a0,a1,a2)           a0##a1##a2                  // Not recommended to use this macro directly; macro expansion won't occur unless another macro wraps it.
#define CAT4_IN(a0,a1,a2,a3)        a0##a1##a2##a3              // Not recommended to use this macro directly; macro expansion won't occur unless another macro wraps it.
#define CAT5_IN(a0,a1,a2,a3,a4)     a0##a1##a2##a3##a4          // Not recommended to use this macro directly; macro expansion won't occur unless another macro wraps it.
#define CAT6_IN(a0,a1,a2,a3,a4,a5)  a0##a1##a2##a3##a4##a5      // Not recommended to use this macro directly; macro expansion won't occur unless another macro wraps it.
#define CAT2(a0,a1)                 CAT2_IN(a0,a1)              // Use this; allows macro expansion
#define CAT3(a0,a1,a2)              CAT3_IN(a0,a1,a2)           // Use this; allows macro expansion
#define CAT4(a0,a1,a2,a3)           CAT4_IN(a0,a1,a2,a3)        // Use this; allows macro expansion
#define CAT5(a0,a1,a2,a3,a4)        CAT5_IN(a0,a1,a2,a3,a4)     // Use this; allows macro expansion
#define CAT6(a0,a1,a2,a3,a4,a5)     CAT6_IN(a0,a1,a2,a3,a4,a5)  // Use this; allows macro expansion
#endif


// Macros for converting a macro value into a string representation of the
// macro value. This is needed for concatenating macro contents to other strings.
// Using STRINGIFY_INNER() directly does not allow macro expansion.
// Using STRINGIFY() does perform macro expansion of value before passing
// to the inner version.
// As an example, consider this code:
//  #define APP_VERSION_MAJOR    3
//  #define APP_VERSION_MINOR    14
//      UARTPrintROMString("Firmware version: " STRINGIFY(APP_VERSION_MAJOR) "." STRINGIFY(APP_VERSION_MINOR) "\r\n");
// Here, the version macros are integer compile-time constants. With
// stringification, the UARTPrintROMString() function doesn't need any logic to
// do binary to ASCII number conversion like the heavyweight printf() function.
#if !defined(STRINGIFY)
#define STRINGIFY_INNER(x)              #x
#define STRINGIFY(value)                STRINGIFY_INNER(value)
#endif


/**
 * <code><b>
 * int NOW_PreSleep(unsigned long expectedSleepDuration);
 * </b></code>
 * 
 * User callback function that is called anytime NOW_Sleep() is called 
 * elsewhere in the system. Implement this function in order to do any shutdown 
 * or peripheral reconfiguration before the processor goes to sleep or 
 * optionally, reject the sleep request altogether. 
 * 
 * For example, you can implement this callback to ensure all UART/SPI/I2C 
 * communications are in an idle state so data doesn't get truncated nor do you 
 * stall any other devices that you were in the middle of transmitting to. You 
 * can also perform other application specific operations to reduce power, like 
 * turn off a power supply to other board level components.
 * 
 * This function is linked as an undefined weak symbol, so is optional. If you 
 * do not define this callback function, NOW_Sleep() will put the processor to 
 * sleep without calling anybody first.
 * 
 * This function executes in the same CPU IPL context as NOW_Sleep() was called 
 * in. This could be either main(), or any ISR level, depending on your code. 
 * Caution is advised as it could even be issued from both places at the same 
 * time in a unintentionally recursive or reentrant loop. 
 * Ex: - You call NOW_Sleep() from main()
 *     - NOW_Sleep() calls NOW_PreSleep() callback in the main() context
 *     - An Interrupt is triggered before you return from NOW_PreSleep()
 *     - ISR also calls NOW_Sleep()
 *     - NOW_Sleep() again calls NOW_PreSleep(), now in an Interrupt context
 * 
 * @param expectedSleepDuration
 *      Number of NOW counts the sleep request was for. Use NOW_second, 
 *      NOW_millisecond, or NOW_microsecond to decode this meaning.
 * 
 * @return 
 *      <p>Return >= 0 to approve the sleep operation.
 *      <p>Return < 0 to reject the sleep operation.
 */
extern int NOW_PreSleep(unsigned long expectedSleepDuration);

/**
 * <code><b>
 * extern void NOW_PostSleep(unsigned long timeAsleep);
 * </b></code>
 * 
 * User callback function that is called anytime NOW_Sleep() is called 
 * and we have awoken and are about to return. Implement this function in 
 * order to do any peripheral restart or reconfiguration in a central location.
 * 
 * This function is linked as an undefined weak symbol, so is optional. If you 
 * do not define this callback function, NOW_Sleep() will return without calling 
 * anybody first.
 * 
 * This function executes in the same CPU IPL context as NOW_Sleep() was called 
 * in, and is complementary to the NOW_PreSleep() callback function.
 * 
 * If NOW_PreSleep() returns a cancellation value for the sleep request, this 
 * callback is NOT called.
 * 
 * @param timeAsleep
 *      Number of NOW counts that we actually spent sleeping. Because interrupts 
 *      could have woken the CPU, executed asynchronously, and then NOW_Sleep() 
 *      put the CPU back to sleep for the requested duration, the timeAsleep 
 *      value will normally be shorter than the original expectedSleepDuration 
 *      given in the NOW_PreSleep() callback, or requested when NOW_Sleep() was 
 *      originally called. The minimum value is 0, which will occur if the 
 *      amount of cumulative time spent in the NOW_PreSleep() and NOW_Sleep() 
 *      functions executing overhead operations ends up being equal or longer 
 *      than the requested NOW_Sleep() duration.
 * 
 *      Use NOW_second, NOW_millisecond, or NOW_microsecond to decode 
 *      parameter's meaning.
 */
extern void NOW_PostSleep(unsigned long timeAsleep);

/**
 * NOW_second
 *
 * Number of NOW counts per second of physical time (assuming the NOW_Init()
 * function is called with the running device clock frequency).
 *
 * If run time clock switching occurs externally and you are using the system or 
 * peripheral clock for the timing reference to NOW, you must call the 
 * NOW_SetTimeIntervals() function to recompute this variable. Any NOW APIs 
 * which include clock switching internally do not carry this requirement since 
 * they will automatically update NOW_microsecond, NOW_millisecond, and 
 * NOW_second for you.
 *
 * This variable is the same value as the device clock frequency, so results in
 * no rounding or truncation error.
 *
 * Note that this is an unsigned long value. You must up-cast this to an
 * (unsigned long long) if you are running at a high clock frequency and want to
 * compute an interval that would overflow a 32-bit integer. For example, if the
 * device clock is 70.0MHz, NOW_second would be 70,000,000. If you want to
 * compute something that is two minutes (120 seconds), the correct number of
 * NOW counts would be 8,400,000,000, which can't fit in a 32-bit variable. For
 * this clock speed, any calculation (including intermediate ones) requiring more
 * than 61.356 seconds will be invalid without an initial cast to 64-bits.
 */
extern unsigned long NOW_second;

/**
 * NOW_millisecond
 *
 * Number of NOW counts per millisecond of physical time (0.001 seconds,
 * assuming the NOW_Init() function is called with the running device clock
 * frequency).
 *
 * If run time clock switching occurs externally and you are using the system or 
 * peripheral clock for the timing reference to NOW, you must call the 
 * NOW_SetTimeIntervals() function to recompute this variable. Any NOW APIs 
 * which include clock switching internally do not carry this requirement since 
 * they will automatically update NOW_microsecond, NOW_millisecond, and 
 * NOW_second for you.
 *
 * Note that this variable is an integral number, rounded to result in the least
 * amount of error, either positive or negative. Ex: if the clock
 * frequency is 32.768kHz, one NOW count would represent 1/32768Hz or
 * 0.030517578125 milliseconds. Therefore, NOW_millisecond, the number of NOW
 * counts/millisecond, would be 32.768 if precision were infinite. Since
 * rounding instead of truncation occurs, this will result in NOW_millisecond
 * returning 33 instead of 32 (-0.708% error instead of +2.344% error).
 */
extern unsigned long NOW_millisecond;

/**
 * <code><b>
 * unsigned short NOW_microsecond;
 * </b></code>
 *
 * Number of NOW counts per microsecond of physical time (0.000001 seconds,
 * assuming the NOW_Init() function is called with a suitably fast timer clock
 * such has microsecond resolution). 
 * 
 * 31/32kHz LPRC and 32.768kHz SOSC clocks have a period of over 30us, so DO NOT 
 * USE NOW_microsecond with such clocks. The value would always be zero! Use a 
 * less accurate NOW_millisecond or accurate NOW_second reference instead.
 *
 * If run time clock switching occurs externally and you are using the system or 
 * peripheral clock for the timing reference to NOW, you must call the 
 * NOW_SetTimeIntervals() function to recompute this variable. Any NOW APIs 
 * which include clock switching internally do not carry this requirement since 
 * they will automatically update NOW_microsecond, NOW_millisecond, and 
 * NOW_second for you.
 *
 * Note that this variable is an integral number, rounded to result in the least
 * amount of error, either positive or negative. Ex: if the clock
 * frequency is 1.500001MHz, one NOW count would represent 1/1500001Hz or 0.6667
 * microseconds. Therefore, NOW_microsecond, the number of NOW
 * counts/microsecond, would be 1.500001 if precision were infinite. Since
 * rounding instead of truncation occurs, this will result in NOW_microsecond
 * returning 2 instead of 1 (-33.33324% error instead of +33.33338%).
 *
 * Be very cautious using this value if your clock frequency is under 500kHz. In
 * this case, 0 will be returned, which could cause a divide by zero exception 
 * when divided or 100% error condition when multiplied.
 */
extern unsigned short NOW_microsecond;

/**
 * <code><b>
 * unsigned long NOW_systemFrequency;
 * </b></code>
 *
 * System instruction clock frequency, in Hz. Normally this will match 
 * NOW_second, but this value could differ if the NOW Timer clock source is 
 * different from the system execution clock. Value is set when NOW_Init() is 
 * called.
 */
extern unsigned long NOW_systemFrequency;



/**
 * <code><b>
 * void NOW_Init(unsigned long timerInputFrequency);
 * </b></code>
 *
 * Initializes a Timer for time keeping with the NOW_* API. Timing is 
 * implemented relative to the timer's selected input clock frequency. This 
 * function uses this value to sets the values stored in NOW_second, 
 * NOW_millisecond and NOW_microsecond for direct use in calling code to 
 * translate run-time adjustable NOW counts to physical seconds and other human 
 * meaningful times.
 *
 * This function defaults to enabling the Timer/CCP Interrupt at priority 
 * level 4. You can modify this value. The interrupt will fire every 65536 
 * instructions and need around ~15 or 30 cycles per interrupt event on 16-bit 
 * processors. i.e. less than 0.05% CPU use.
 *
 * Most of the NOW APIs should only be called at IPL levels below this ISR
 * priority if accurate and monotonic NOW counts are needed. If you wish to use
 * these APIs in higher priority ISRs, increase the timer ISR priority level
 * to one higher than the highest caller.
 *
 * @param timerInputFrequency 
 *          Number of input timer clock cycles per second of real time. For 
 *          example, use 70000000 if your device is operating at 70 MIPS and you 
 *          are using the system instruction clock to clock the timer. LPRC's 
 *          will need ~31000 for 31kHz, and of course 32.768kHz SOSC crystals 
 *          are also normally used with 32768. 
 * 
 *          However, if you know your crystal or clock input is biased above or 
 *          below the optimal crystal spec, you can deliberately provide a value 
 *          slightly faster or slower to automatically correct for this 
 *          oscillation frequency error in software computations. At room 
 *          temperature, crystals will oscillate slower than intended when you 
 *          have oversized crystal loading capacitors (ex: more parasitic pin 
 *          capacitance than you calculated for), and similarly, run fast when 
 *          there is below optimal capacitance.
 */
void NOW_Init(unsigned long timerInputFrequency);


/**
 * <code><b>
 * void NOW_SetTimeIntervals(unsigned long timerInputFrequency);
 * </b></code>
 *
 * Updates the NOW_second, NOW_millisecond and NOW_microsecond variables to
 * correspond to the specified timer input clock frequency.
 *
 * @param timerInputFrequency 
 *          Number of timer input clock cycles per second. For example, use 
 *          70000000 if your device is operating at 70 MIPS and using the system 
 *          clock to clock the timer.
 */
void NOW_SetTimeIntervals(unsigned long timerInputFrequency);


/**
 * <code><b>
 * unsigned long NOW_MeasureAbsoluteClock(unsigned int targetClockType, void *targetClock, unsigned int refClockType, void *refClock, unsigned long refClockFrequency, unsigned int desiredPPMPrecision, unsigned int milliTimeout);
 * </b></code>
 * 
 * Measures an unknown-frequency clock source against another known-frequency 
 * reference clock source and computes the unknown frequency.
 * 
 * @param targetClockType
 *        <p>0x0 = 16-bit Timer/CCP/Counter in an SFR or RAM 
 *                 location. *refClock will be dereferenced as a 
 *                 volatile unsigned short to get "tick" 
 *                 values. In the case of a RAM location, an 
 *                 external ISR of higher priority than the present 
 *                 CPU IPL must write the new values into the given 
 *                 address as this function does not return until 
 *                 the DCO measurements are complete.
 *        <p>0x1 = 32-bit Timer/CCP/Counter in an SFR or RAM. 
 *                 *refClock will be dereferenced as a 
 *                 volatile unsigned long to get "tick" values.
 *        <p>0x2 = 16-bit "tick" via function pointer return.
 *                 *refClock must point to an executable function 
 *                 that takes no parameters and will return an 
 *                 incrementing 16-bit integer type (unsigned int, 
 *                 signed int, unsigned short, unsigned short int, 
 *                 signed short, etc.). Generally, it is possible 
 *                 to use this value even for 32-bit return valued 
 *                 functions, but in such a case, the upper 16-bits 
 *                 of information will always be discarded.
 *        <p>0x3 = 32-bit "tick" via function pointer return.
 *                 *refClock must point to an executable function 
 *                 that takes no parameters and will return an 
 *                 incrementing 16-bit integer type (unsigned int, 
 *                 signed int, unsigned short, unsigned short int, 
 *                 signed short, etc.).
 * 
 * @param targetClock       Pointer to the specified clock resource. See 
 *                          targetClockType parameter definition for exact 
 *                          meaning of this pointer.
 * 
 * @param refClockType      Same as targetClockType, but for the known-frequency 
 *                          reference clock.
 * 
 * @param refClock          Same as targetClock, but for the known-frequency 
 *                          reference clock.
 * 
 * @param refClockFrequency Frequency of the known-frequency refClock, in Hz.
 * 
 * @param desiredPPMPrecision   
 *                          Desired precision in measuring the target clock
 *                          frequency, in units of PPM (Parts Per Million). 
 *                          1000 PPM is equivalent to +/-0.1% clock error 
 *                          (1000/1000000 * 100%). 
 * 
 *                          Higher precision takes longer to measure, 
 *                          potentially with a Gaussian tail, meaning a LOT 
 *                          longer. 
 *                          
 *                          NOTE: if either the reference, and to a lesser 
 *                          extent, target clock frequency is unstable and 
 *                          drifts faster than the desired precision can be 
 *                          achieved, this function could block forever. To 
 *                          avoid this, specify a timeout.
 * 
 * @param milliTimeout      Maximum number of milliseconds allowed to try and 
 *                          achieve the desiredPPMPrecision criteria. To disable 
 *                          the timeout, specify 0.
 * 
 * @return  Present frequency of the targetClock, in cycles-per-second (Hz). If an 
 *          unrecognized parameter is given, 0xFFFFFFFF is returned.
 */
unsigned long NOW_MeasureAbsoluteClock(unsigned int targetClockType, void *targetClock, unsigned int refClockType, void *refClock, unsigned long refClockFrequency, unsigned int desiredPPMPrecision, unsigned int milliTimeout);


/**
 * <code><b>
 * unsigned short NOW_16(void);
 * </b></code>
 *
 * Atomically returns the least significant 16 bits of the current NOW counter
 * value. This function is safe to be called from all ISRs (and main context), 
 * but only when the ISR priority is at least one higher than the Timer's ISR 
 * priority. This function is reentrant capable.
 *
 * Execution time is 5 or 10 cycles on 16-bit PIC24 or dsPIC processors, 
 * depending on device family. This includes the call and return branches.
 *
 * @return 16-bit NOW counter value, where 1 NOW count is equal to clock cycle 
 *         of the reference timer input. Use the NOW_second, NOW_millisecond, 
 *         NOW_microsecond variable contents, or the NOW_Diff_to_ms() and 
 *         NOW_Diff_to_us() functions in order to make physical sense of how 
 *         long a NOW count is.
 *
 *         Because this return value is only 16-bits wide and could increment as 
 *         at up to the system device frequency, some clock sources will 
 *         overflow 16-bits very easily. Ex: at 70MIPS and the CPU clock used 
 *         for the NOW timer, the longest interval you could correctly measure 
 *         with this function is 936us. Use the NOW_32() or NOW_64() functions 
 *         if longer durations are being measured.
 */
unsigned short NOW_16(void);


/**
 * <code><b>
 * unsigned long NOW_32(void);
 * </b></code>
 *
 * Atomically returns the least significant 32 bits of the current NOW counter
 * value. This function is safe to be called from the main contexts and ISRs
 * of lower priority than the TMR1 ISR priority (IPL=1,2 or 3 by default).
 * Calling from a higher or equal ISR priority context will succeed and cause
 * no harm, but the value returned may be off by up to 0x10000 counts if the
 * lower 16 bit count rolls over.
 *
 * Execution time is 12 or 20 cycles best (and normal) case, depending on device
 * family, which includes the call and return branches. If you are very unlucky
 * and the lower 16-bits rolls over during the read (less than 0.013% chance if
 * no interrupt occurs), the final return value will still be atomic, but
 * execution time will approximately double.
 *
 * @return 32-bit NOW counter value, where 1 NOW count is equal to 1 instruction
 *         cycle. Use the NOW_second, NOW_millisecond, NOW_microsecond
 *         variable contents, or the NOW_Diff_to_ms() and NOW_Diff_to_us()
 *         functions in order to make physical sense of how long a NOW count is.
 *
 *         Because this return value is only 32-bits wide and increments at the
 *         system device frequency, it is subject to overflow when working with
 *         longer intervals . Ex: at 70MIPS, the longest interval you could
 *         correctly measure with this function is 61.356 seconds. Use the
 *         NOW_64() function if longer durations are being measured.
 */
unsigned long NOW_32(void);

/**
 * <code><b>
 * unsigned long long NOW_64(void);
 * </b></code>
 *
 * Atomically returns the entire 64 bits of the current NOW counter value
 * value. This function is safe to be called from the main context and ISRs
 * of lower priority than the TMRx ISR priority (IPL=1,2 or 3 by default).
 * Calling from a higher or equal ISR priority context will succeed and cause
 * no harm, but the value returned may be off by up to 0x100000000 counts if the
 * lower 32 bit count rolls over.
 *
 * Execution time is 20 or 32 cycles best (and normal) case, depending on device
 * family, which includes the call and return branches. If you are very unlucky
 * and the lower 16-bits rolls over during the read (less than 0.013% chance if 
 * no interrupt occurs), the final return value will still be atomic, but
 * execution time will approximately double.
 *
 * @return 64-bit NOW counter value, where 1 NOW count is equal to 1 instruction
 *         cycle. Use the NOW_second, NOW_millisecond, NOW_microsecond
 *         variable contents, or the NOW_Diff_to_ms() and NOW_Diff_to_us()
 *         functions in order to make physical sense of how long a NOW count is.
 *
 *         Because this return value is a whopping 64-bits, the returned value
 *         will be virtually immune to overflow. Ex: at 1GHz (I want that PIC!),
 *         overflow won't occur for over 584 years. As a result, this NOW count
 *         can also be used for absolute time measurement. Do do so, just
 *         initialize NOW_internalCount to the present time (relative to
 *         whatever date/time you consider Time T=0) before calling NOW_init().
 *         If you do not initialize NOW_internalCount, the default will be
 *         0x0000000000000000, so NOW_64() will still be usable as an absolute
 *         timestamp, but values returned will be relative to the last device
 *         reset.
 */
unsigned long long NOW_64(void);

/**
 * <code><b>
 * unsigned long NOW_Diff_to_ms(unsigned long nowCounts);
 * </b></code>
 *
 * Converts the difference between two NOW counts to absolute time in
 * milliseconds. This function is primarily intended for instances where you
 * wish to display a time interval to a human in decimal forms. It is, however,
 * not subject to the integral approximations that NOW_millisecond represent, so
 * can be used in other cases when absolute accuracy is critical and the device
 * operating frequency is very low (ex: 32.768kHz).
 *
 * nowCounts must be measured against the same clock frequency in use during
 * invocation of this function. In other words, a wrong result will be returned
 * if your collect two NOW counts at one clock frequency, do a run-time clock 
 * switch, call NOW_SetTimeInterval(), and then pass the difference of the
 * original counts in for the nowCounts parameter.
 *
 * This function can be called from any CPU IPL and is reentrant capable.
 *
 * This function requires two 32x32 unsigned integer divide operations and
 * therefore requires a fair amount of execution time relative to most other
 * NOW APIs. If you just need to compare two timestamps where one value
 * represents the present and another value represents a constant timeout, it is
 * likely more efficient to use the NOW_second and NOW_millisecond variables.
 *
 * @param nowCounts The number of NOW counts to convert. Acceptable range is 0
 *                  to 536,870,911 counts (or up to 2,684 ms @ 200MHz; longer
 *                  for slower clock frequencies). Specifying
 *                  nowCounts > 536,870,911 will return an undefined value. The
 *                  function will safely succeed, however.
 *
 * @return 32-bit unsigned long representing complete milliseconds elapsed.
 *         Rounding is not performed; partial milliseconds are truncated off.
 */
unsigned long NOW_Diff_to_ms(unsigned long nowCounts);

/**
 * <code><b>
 * unsigned long NOW_Diff_to_us(unsigned long nowCounts);
 * </b></code>
 *
 * Converts the difference between two NOW counts to absolute time in
 * microseconds. This function is primarily intended for instances where you
 * wish to display a time interval to a human in decimal forms. It is, however,
 * not subject to the integral approximations that NOW_millisecond and
 * NOW_microsecond represent, so can be used in other cases when absolute
 * accuracy is critical.
 *
 * nowCounts must be measured against the same clock frequency in use during 
 * invocation of this function. In other words, a wrong result will be returned 
 * if your collect two NOW counts at one clock frequency, do a run-time clock 
 * switch, call NOW_SetTimeInterval(), and then pass the difference of the 
 * original counts in for the nowCounts parameter.
 *
 * This function can be called from any CPU IPL and is reentrant capable.
 *
 * This function requires one 32x16 unsigned integer divide plus one 32x32 
 * unsigned integer divide operations and therefore requires a fair amount of 
 * execution time relative to most other NOW APIs. If you just need to compare 
 * two timestamps where one value represents the present and another value 
 * represents a constant timeout, it may be more efficient to use the 
 * NOW_millisecond and NOW_microsecond variables. Using these variables can,
 * however, result in greater error for certain clock frequencies, so this
 * function may still be needed when absolute accuracy has to be minimized.
 *
 * @param nowCounts The number of NOW counts to convert. Acceptable range is 0 
 *                  to 67,108,863 counts (or up to 335,544 microseconds @ 
 *                  200MHz; longer for slower clock frequencies). Specifying
 *                  nowCounts > 67,108,863 will return an undefined value. The
 *                  function will safely succeed, however.
 *
 * @return 32-bit unsigned long representing complete microseconds elapsed.
 *         Rounding is not performed; partial microseconds are truncated off.
 */
unsigned long NOW_Diff_to_us(unsigned long nowCounts);

/**
 * <code><b>
 * unsigned long NOW_Sleep(unsigned long blockTime);
 * </b></code>
 *
 * Configures the NOW timer to operate asynchronously in Sleep mode and then 
 * puts the CPU to Sleep. An optional blocking time can be given to have the CPU 
 * automatically wake up and return after the given interval elapses.
 * 
 * If an enabled Interrupt wakes the CPU before the specified blockTime elapses, 
 * this function will allow the wake-up ISR to execute. When the ISR is 
 * complete, if there is still blocking time required to meet the original 
 * count, the CPU is placed back into sleep mode. The function always ensures 
 * that the at least the given blocking interval is met before returning 
 * (independent of how long the CPU cumulatively actually spent asleep).
 *
 * @param blockTime 
 *          Number of NOW counts this function must block before returning. The 
 *          blocking duration will be spent optimally sleeping, but allowing 
 *          ISRs to still wake the CPU and execute as needed. If one of the 
 *          asynchronous ISRs calls NOW_WakeUp() before the given blockTime is 
 *          met the sleep call is aborted and returns early.
 * 
 *          To sleep indefinitely unless/until signaled by NOW_WakeUp(), specify 
 *          0xFFFFFFFF for blockTime. This special value is treated as infinite 
 *          rather than an actual NOW count.
 *
 * @return 
 *          Number of NOW counts that the CPU actually spent sleeping. This 
 *          value will always be less than or equal to the given blockTime. 
 *          Unless commanded to stop sleeping early from NOW_WakeUp(), dividing 
 *          this return value and the provided blockTime and subtracting from 1 
 *          therefore represents a CPU utilization ratio, directly usable for 
 *          doing power consumption estimates.
 * 
 *          To be able to identify when the blocking interval is ended early 
 *          from NOW_WakeUp(), you should get the current NOW counter 
 *          immediately before calling NOW_Sleep(), immediately recapture the 
 *          NOW counter after this function returns, and then subtract the two. 
 *          For example:
 *          <code>
 *          unsigned long   sleepStart, sleepCount, trueBlockingCount;
 *          double          cpuUtilizationPercent;
 * 
 *          sleepStart = NOW_32();                      // Get starting timestamp
 *          sleepCount = NOW_Sleep(NOW_Second/4);       // Sleep for 250ms
 *          trueBlockingCount = NOW_32() - sleepStart;  // Compute time difference
 *          cputUtilizationPercent = ((double)(sleepCount*100u))/trueBlockingCount;
 *          </code>
 */
unsigned long NOW_Sleep(unsigned long blockTime);



#endif	/* NOW_H */

