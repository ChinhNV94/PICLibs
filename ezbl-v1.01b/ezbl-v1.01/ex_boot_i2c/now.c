/*******************************************************************************
 *
 * Time Counting and Measurement Functions of NOW library
 * 
 * Includes routines for measuring absolute time, polled action triggering and 
 * measuring and tuning oscillators, such as your Fast internal RC oscllator 
 * (FRC), Digitally Controlled Oscillator (DCO), or other clock source.
 * 
 *******************************************************************************
 * FileName:        now.c
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

#define __NOW_C

#include <xc.h>
#include <math.h>
#include "now.h"

#define NOW_TIMER_TYPE      T       // Use 'T' for Timer 1/2/3/etc. or 'CCP' for MCCP or SCCP type timers
#define NOW_TIMER_NUM       1       // Peripheral Timer/CCP # for this NOW module to use. NOTE: CURRENTLY ONLY SUPPORTS TIMER 1. OTHERS WILL NOT WORK!

// General macros for abstracting the Timer module use. Generally, you shouldn't 
// change these. Change NOW_TIMER_TYPE and NOW_TIMER_RESOURCE instead to 
// change which Timer module is used.
#if 1 //NOW_TIMER_TYPE == T
// Timer case
#define EATIF_T(ccp_token_to_eat)     
#define EATIF_CCP(t_token_to_eat)  t_token_to_eat
#else               
// MCCP/SCCP case
#define EATIF_T(ccp_token_to_eat)   ccp_token_to_eat
#define EATIF_CCP(t_token_to_eat)
#endif

#define Tx                      CAT2(NOW_TIMER_TYPE,NOW_TIMER_NUM)
#define TxCON                   CAT3(Tx,CON,EATIF_T(1))
#define TxCONbits               CAT4(Tx,CON,EATIF_T(1),bits)
#define TxCONCLR                CAT4(Tx,CON,EATIF_T(1),CLR)
#define TxCONSET                CAT4(Tx,CON,EATIF_T(1),SET)
#define TxCONINV                CAT4(Tx,CON,EATIF_T(1),INV)
#define PRx                     CAT4(EATIF_T(NOW_TIMER_TYPE),EATIF_CCP(PR),NOW_TIMER_NUM,EATIF_T(PR))
#define TMRx                    CAT4(EATIF_T(NOW_TIMER_TYPE),EATIF_CCP(TMR),NOW_TIMER_NUM,EATIF_T(TMR))
#define TxInterrupt             CAT2(Tx,Interrupt)
#define TxIP                    CAT2(Tx,IP)
#define TxIF                    CAT2(Tx,IF)
#define TxIE                    CAT2(Tx,IE)
#define _TIMER_x_VECTOR         CAT5(_,NOW_TIMER_TYPE,EATIF_CCP(IMER_),NOW_TIMER_NUM,_VECTOR)


// Compiler specific macro for ensuring PIC24/dsPIC assembly instructions can 
// quickly address the NOW_millisecond, NOW_microsecond, and NOW_internalCount
// variables. XC32 doesn't need this, so this macro deletes the 'near' 
// attribute.
#if defined(__XC32__)
#define _ASM_ACCESSIBLE
#elif defined(__XC16__)
#define _ASM_ACCESSIBLE     __attribute__((near))
#define SIDL                TSIDL                   // Needed for bit name compatibility between PIC32 and PIC24/dsPIC devices. These are the same bit.
#endif


/**
 * <code><b>
 * unsigned long NOW_second;
 * </b></code>
 *
 * Number of NOW counts per second of physical time (assuming the NOW_Init()
 * function is called with the running device clock frequency).
 *
 * If run time clock switching occurs, you should call the
 * NOW_SetTimeIntervals() function to automatically set this variable.
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
unsigned long _ASM_ACCESSIBLE NOW_second;

/**
 * <code><b>
 * unsigned long NOW_millisecond;
 * </b></code>
 *
 * Number of NOW counts per millisecond of physical time (0.001 seconds,
 * assuming the NOW_Init() function is called with the running device clock
 * frequency).
 *
 * If run time clock switching occurs, you should call the
 * NOW_SetTimeIntervals() function to automatically recompute this variable.
 *
 * Note that this variable is an integral number, rounded to result in the least
 * amount of error, either positive or negative. Ex: if the clock
 * frequency is 32.768kHz, one NOW count would represent 1/32768Hz or
 * 0.030517578125 milliseconds. Therefore, NOW_millisecond, the number of NOW
 * counts/millisecond, would be 32.768 if precision were infinite. Since
 * rounding instead of truncation occurs, this will result in NOW_millisecond
 * returning 33 instead of 32 (-0.708% error instead of +2.344% error).
 */
unsigned long _ASM_ACCESSIBLE NOW_millisecond;

/**
 * <code><b>
 * unsigned short NOW_microsecond;
 * </b></code>
 *
 * Number of NOW counts per microsecond of physical time (0.000001 seconds,
 * assuming the NOW_Init() function is called with a suitably fast timer clock
 * that has sub-microsecond resolution).
 * 
 * 31/32kHz LPRC and 32.768kHz SOSC clocks have a period of over 30us, so DO NOT 
 * USE NOW_microsecond with such clocks. The value would always be zero! Use a 
 * less accurate NOW_millisecond or accurate NOW_second reference instead. 
 * Alternatively, see the NOW_Diff_to_us() API, which can compute a time 
 * difference without losing microsecond resolution due to integer rounding.
 *
 * If run time clock switching occurs externally, you must call the 
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
 * Be very cautious using this value if your timer clock frequency is under 
 * 500kHz. In this case, 0 will be returned, which could cause a divide by zero 
 * exception when divided or 100% error condition when multiplied.
 */
unsigned short _ASM_ACCESSIBLE NOW_microsecond;


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
unsigned long NOW_systemFrequency;


/**
 * NOW_internalCount - Internal 64/80-bit NOW timer variable. Use the NOW_16(),
 * NOW_32(), NOW_64() APIs to read this instead of referencing this directly.
 * While this variable is 64-bits, it does not include the lower 16-bits which
 * reside in the Timer 1 hardware instead. It also won't be read atomically if
 * this is referenced directly.
 */
volatile unsigned long long _ASM_ACCESSIBLE NOW_internalCount;


// Re-declare the NOW_PreSleep() and NOW_PostSleep() callback functions as weak 
// so we don't get a linking error if the user doesn't implement or need such a 
// function.
extern int __attribute__((weak)) NOW_PreSleep(unsigned long expectedSleepDuration);
extern void __attribute__((weak)) NOW_PostSleep(unsigned long timeAsleep);


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
void NOW_Init(unsigned long timerInputFrequency)
{
	// Use Timer with 1:1 prescale
    TxCONbits.SIDL = 0;         // 0 = Keep running the Timer + input clock in Idle mode; 1 = Stop timer in Idle
#if defined(__XC32__)           // Only PIC32 version of Timer 1 supports TECS for other clock sources. PIC24/dsPIC Timer 1 module has TCS only for 
    TxCONbits.TECS = 0x2;       // External Clock source: 0x0 = SOSC; 0x1 = TxCK pin; 0x2 = LPRC
#endif
    TxCONbits.TCS = 0;          // 0 = Use internal peripheral bus clock (PBCLK/UPBCLK); 1 = Use other clock source, as specified by TECS on PIC32 or T1CK on PIC24/dsPIC module versions)
    TxCONbits.TCKPS = 0x0;      // 0x0 = 1:1 prescalar; 0x1 = 1:8 prescalar; 0x2 = 1:64 prescalar; 0x3 = 1:256 prescalar
    TxCONbits.TSYNC = 1;        // 0 = let timer increment asynchronously (needed if counter and clock source should continue in Idle/Sleeping; best when period is 0xFFFF); 1 = synchronize clock edges into local clock domain for the lowest latency accesses (must use an external clock that is slower than half the peripheral clock bus speed (PBCLK/UPBCLK), unless the clock actually is PBCLK/UPBCLK. Also, this mode will not keep the clock and timer active when in Sleep mode and a regular Timer module is used (versus MCCP/SCCP timer module, which can probably auto-switch to asynchronous and back when Sleeping))
    
    // Set period (actual period is 1 greater due to 0x0000 count)
	PRx = 0xFFFF;

    // Set NOW_systemFrequency to match the given input frequency
    NOW_systemFrequency = timerInputFrequency;  // Note: This may change to something more advanced in a future version. Presently, this code expects the timer to always be clocked from the system clock, making NOW_systemFrequency a redundant value that exists in NOW_second as well.
    
	// Enable timer interrupt
#if defined(__PIC32__) || defined(__XC32__)
    IPC2bits.TxIP = 4;          // Interrupt priority 4 (medium; 0 is main() context, 7 is maximum)
    IEC0SET = CAT3(_IEC0_,Tx,IE_MASK);  // Enable Interrupt
#else   // defined(__PIC24__) || defined(__dsPIC__) || defined(__XC16__))
    CAT2(_,TxIP) = 4;          // Set Interrupt priority = 4 (medium; 0 is main() context, 7 is maximum); Evaluates to something like: _T1IP = 4; 
    CAT2(_,TxIE) = 1;          // Enable Interrupt; Evaluates to something like: _T1IE = 1; 
#endif

    // Set NOW_second, NOW_millisecond, and NOW_microsecond values
    NOW_SetTimeIntervals(timerInputFrequency);

	// Start timer
	TxCONbits.TON = 1;

}


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
void NOW_SetTimeIntervals(unsigned long timerInputFrequency)
{
    NOW_second = timerInputFrequency;
    NOW_millisecond = (timerInputFrequency+500u)/1000u;
    NOW_microsecond = (timerInputFrequency+500000u)/1000000u;
}


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
unsigned long NOW_MeasureAbsoluteClock(unsigned int targetClockType, void *targetClock, unsigned int refClockType, void *refClock, unsigned long refClockFrequency, unsigned int desiredPPMPrecision, unsigned int milliTimeout)
{
    unsigned int lookingGood;
    unsigned long refTime, startRefTime, lastRefTime, deltaRef;
    unsigned long targetTime, startTargetTime, lastTargetTime, deltaTarget;
    unsigned long long targetAccumulator;
    unsigned long long refAccumulator;
    unsigned long long refAccumulatorTimeout;
    double targetHz, lastTargetHz;
    double elapsedTotalTime;
    double precisionLimit;
    //unsigned long minSampleTime;
    
    // Function pointers
    unsigned int (*refClockFunc16)(void) = refClock;
    unsigned long (*refClockFunc32)(void) = refClock;
    unsigned int (*targetClockFunc16)(void) = targetClock;
    unsigned long (*targetClockFunc32)(void) = targetClock;
    
    // Return impossibly high 4GHz value if an invalid null pointer is supplied
    if((refClock == 0u) || (targetClock == 0u))
        return 0xFFFFFFFF;
    
    
    // Initialize variables before entering timing sensitive loop
    refAccumulator = 0;
    targetAccumulator = 0;
    lookingGood = 0;
    startRefTime = 0;
    lastRefTime = 0;
    startTargetTime = 0;
    lastTargetTime = 0;
    lastTargetHz = 0;
    refAccumulatorTimeout = (((unsigned long long)milliTimeout)*refClockFrequency + 500u)/1000u;
    precisionLimit = desiredPPMPrecision/1.0e6;
    //minSampleTime = (((unsigned long long)(1000000u - desiredPPMPrecision)) * refClockFrequency)/1000000u;
    
    // Enter timing compare loop
    while(1)
    {
        // Get the reference clock
        switch(refClockType)
        {
            case 0x1:   // 32-bit SFR/RAM/data pointer
                refTime = *((volatile unsigned long*)(refClock));
                break;
            case 0x2:   // 16-bit function pointer
                refTime = refClockFunc16();
                break;
            case 0x3:   // 32-bit function pointer
                refTime = refClockFunc32();
                break;
            default:    // case 0x0 or anything else -> Assume 16-bit  SFR/RAM/data pointer
                refTime = *((volatile unsigned short*)(refClock));
        }

        // Get the target clock to measure
        switch(targetClockType) 
        {
            case 0x1:   // 32-bit SFR/RAM/data pointer
                targetTime = *((volatile unsigned long*)(targetClock));
                break;
            case 0x2:   // 16-bit function pointer
                targetTime = targetClockFunc16();
                break;
            case 0x3:   // 32-bit function pointer
                targetTime = targetClockFunc32();
                break;
            default:    // case 0x0 or anything else -> Assume 16-bit SFR/RAM/data pointer
                targetTime = *((volatile unsigned short*)(targetClock));
        }

        // Initialize history if this is the very start of the test
        if(startRefTime == 0u)
        {
            startRefTime = refTime;
            lastRefTime = refTime;
            startTargetTime = targetTime;
            lastTargetTime = targetTime;
        }
        
        // Don't process anything if the reference timer value hasn't incremented 
        // to give us new knowledge.
        if(refTime <= lastRefTime)
            continue;
        if(refTime <= startRefTime)
            continue;
        
        // Compute commonly used differences, update accumulators
        deltaRef = refTime - lastRefTime;
        deltaTarget = targetTime - lastTargetTime;
        lastRefTime = refTime;
        lastTargetTime = targetTime;
        refAccumulator += deltaRef;
        targetAccumulator += deltaTarget;
        
        // Compute target frequency using everything we know so far
        elapsedTotalTime = ((double)(refAccumulator))/refClockFrequency;
        targetHz =((double)targetAccumulator)/elapsedTotalTime;
        
        // Check for timeout
        if(milliTimeout)
        {
            if(refAccumulator >= refAccumulatorTimeout)
            {
                return targetHz;
            }
        }
        
        // Check if target delta meets the desired precision
        //if(refAccumulator > minSampleTime)          // Must wait long enough to reach desired PPM according to the reference clock
        {
            //if(targetAccumulator > minSampleTime)   // Must also wait long enough to reach the desired PPM according to the approximate target clock
            {
                if((fabs(targetHz - lastTargetHz)/targetHz) < precisionLimit)   // Lastly, filter a few samples to ensure the delta for several iterations are below the PPM limit
                {
                    // We are looking good, but let's collect several such readings 
                    // matching the desired precision to avoid any flukes and 
                    // instead be sure our target delta really has leveled off.
                    if(lookingGood++ >= 16u)
                    {
                        return (unsigned long)targetHz;
                    }
                }
                else if(lookingGood)
                {
                    lookingGood = 0;
                }
            }
        }

        lastTargetHz = targetHz;
    }
}


#if defined(__PIC32__) || defined(__XC32__)   // PIC32 specific code followed by PIC24/dsPIC optimized code
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
unsigned long NOW_Diff_to_ms(unsigned long nowCounts)
{
    return (((unsigned long long)nowCounts * 1000u) + (NOW_second/2u))/NOW_second;
}


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
unsigned long NOW_Diff_to_us(unsigned long nowCounts)
{
    return (((unsigned long long)nowCounts * 1000000u) + (NOW_second/2u))/NOW_second;
}


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
 * TODO: THIS IS NOT DONE!!! IT SHOULD SLEEP LONG ENOUGH, BUT LACKS VARIOUS 
 *       CLAIMED FEATURES.
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
unsigned long NOW_Sleep(unsigned long blockTime)
{
    unsigned long startTime, endTime, callbackTime;
    unsigned long synchronousStartTime, synchronousEndTime;
    unsigned int c0CauseSave;
    unsigned int TCONSave;
    
    // Switch NOW's Timer to asynchronous mode, if needed
    TCONSave = TxCON;
    TxCONCLR = _T1CON_TSYNC_MASK;           // Sleep require an asynchronous timer to keep running the timer for wake up
    c0CauseSave = _mfc0(_CP0_CAUSE, _CP0_CAUSE_SELECT);
    _bcc0(_CP0_CAUSE, _CP0_CAUSE_SELECT,  _CP0_CAUSE_DC_MASK);  // Turn on Coprocessor 0 Core Timer (Counter @ SYSCLK/2) if it is turned off

    // Check if the NOW_PreSleep() user call-back exists, and if so, call it
    if(&NOW_PreSleep)
    {
        callbackTime = NOW_32();                // Time how long the NOW_PreSleep() function takes
        if(NOW_PreSleep(blockTime) < 0)
        {
            // User callback canceled the request, restore timer states and return immediately
            TxCONSET = TCONSave & _T1CON_TSYNC_MASK;   // Woke-up, switch back to synchronous mode if it was set to synchronous beforehand
            _bsc0(_CP0_CAUSE, _CP0_CAUSE_SELECT, _CP0_CAUSE_DC_MASK & c0CauseSave);  // Turn off Coprocessor 0 Core Timer (Counter @ SYSCLK/2) if it was originally off
            return 0;
        }
        // Adjust sleep duration on account of how long the NOW_PreSleep() code took 
        // to return
        callbackTime = NOW_32() - callbackTime;
        blockTime = callbackTime < blockTime ? blockTime - callbackTime : 0;
    }
    
    // Sleep the requested duration
    synchronousStartTime = _mfc0(_CP0_COUNT, _CP0_COUNT_SELECT);
    startTime = NOW_32();                   // Get our asynchronous start sleep time
    while(NOW_32() - startTime < blockTime) // Sleep for the needed duration
    {
        _wait();    // Go to sleep
    }
    endTime = NOW_32();
    synchronousEndTime = _mfc0(_CP0_COUNT, _CP0_COUNT_SELECT);
    
    // Restore timer states
    TxCONSET = TCONSave & _T1CON_TSYNC_MASK;   // Woke-up, switch back to synchronous mode if it was set to synchronous beforehand
    _bsc0(_CP0_CAUSE, _CP0_CAUSE_SELECT, _CP0_CAUSE_DC_MASK & c0CauseSave);  // Turn off Coprocessor 0 Core Timer (Counter @ SYSCLK/2) if it was originally off

    // Remove starting offsets from counters
    endTime -= startTime;
    synchronousEndTime -= synchronousStartTime;

    // Convert synchronous timer counts into NOW counts
    // System Core Timer only increments at SYSCLK/2, so need to adjust NOW count to match
    synchronousEndTime = (((unsigned long long)synchronousEndTime) * NOW_second)/(NOW_systemFrequency>>1);
    
    // Compute actual NOW counts spent sleeping by subtracting off the time in 
    // ISRs. This needs to be done with zero saturation to ensure we don't 
    // underflow if our arithmetic has rounding error or we had no measurable 
    // sleep time.
    startTime = 0;
    if(synchronousEndTime < endTime)
    {
        startTime = endTime - synchronousEndTime;
    }
    
    
    // Call NOW_PostSleep() user call-back, if it is implemented. )
    if(&NOW_PostSleep)
    {
        NOW_PostSleep(startTime);
    }
        
    // Return total time in the sleep loop. Since ISRs could have been executing 
    // while the timer was still counting asynchronously, this count is adjusted 
    // from the synchronous core timer.
    return startTime;   
}

/**
 * <code><b>
 * void __attribute__((vector(_TIMER_x_VECTOR), interrupt(IPL4SOFT))) TxInterrupt(void);
 * </b></code>
 *
 * Increments internal NOW_internalCount to emulate a 64-bit timer. This
 * interrupt will fire every 65536 instructions and will need around ~15 or 30
 * cycles per interrupt event (exact count depends on processor family).
 */
void __attribute__((vector(_TIMER_x_VECTOR), interrupt(IPL4AUTO), keep)) TxInterrupt(void)
{
    IFS0CLR = CAT3(_IFS0_,Tx,IF_MASK);  // Clear interrupt flag
    NOW_internalCount++;                // Increment internal high order timer bits
    
    // Ensure the timer MSbit is actually clear indicating that it is 
    // appropriate for us to be doing the MSbit carry out to NOW_internalCount 
    // right now. Timer reads in asynchronous mode can return a stale value 
    // relative to the ISR trigger due to clock domain synchronization 
    // uncertainty.
    if(!TxCONbits.TSYNC)
    {
        // Block until we read a 0 in all the bits at or above the MSbit of the 
        // period (PRx). This will ensure that the NOW_internalCount increment 
        // we did is valid with respect to reads done elsewhere.
        while(TMRx & 0x8000); 
    }
}

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
unsigned short NOW_16(void)
{
    return TMRx;
}


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
unsigned long NOW_32(void)
{
    unsigned int returnValueL;
    unsigned int returnValueH;
    
    do
    {
        do
        {
            returnValueL = TMRx;                            // Get lowest order bits
            returnValueH = ((volatile unsigned int*)&NOW_internalCount)[0]; // Get [31:16] bits from internal counter variable (more on PIC32, but don't care upper bits)
        } while((signed int)(TMRx - returnValueL) < 0);     // Re-get lowest order bits and check for rollover
    } while(returnValueH != ((volatile unsigned int*)&NOW_internalCount)[0]);   // Re-get everything if NOW_internalCount incremented after reading it. The ISR could be delayed in synchronized mode when a non-SYSCLK/PBCLK/UPBCLK is used as the timer clock source.
    
    return ((unsigned long)returnValueL) | (((unsigned long)returnValueH)<<16);
}

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
unsigned long long NOW_64(void)
{
    unsigned int returnValueL;
    unsigned long long returnValueH;

    do
    {
        do
        {
            returnValueL = TMRx;                            // Get lowest order bits
            returnValueH = NOW_internalCount;               // Get [63:16] bits from internal counter variable
        } while((signed int)(TMRx - returnValueL) < 0);     // Re-get lowest order bits and check for rollover
    } while(returnValueH != NOW_internalCount);             // Re-get everything if NOW_internalCount incremented after reading it. The ISR could be delayed in synchronized mode when a non-SYSCLK/PBCLK/UPBCLK is used as the timer clock source.
    
    return ((unsigned long long)returnValueL) | (returnValueH<<16);
}

#else   // PIC24/dsPIC case follows
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
 * TODO: THIS IS NOT DONE!!! IT SHOULD SLEEP LONG ENOUGH, BUT LACKS VARIOUS 
 *       CLAIMED FEATURES.
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
unsigned long NOW_Sleep(unsigned long blockTime)
{
    unsigned long startTime;

    startTime = NOW_32();    

    // Switch Timer to asynchronous mode so it keeps running in sleep
    TxCONbits.TON = 0;
    TxCONbits.TSYNC = 0;
    TxCONbits.TON = 1;

    while(NOW_32() - startTime < blockTime)
    {
        // Go to sleep
        Sleep();        
    }
    
    // Switch Timer to synchronous mode so we can ensure proper reads
    TxCONbits.TON = 0;
    TxCONbits.TSYNC = 1;
    TxCONbits.TON = 1;
    
    // Return total time in the sleep loop. Since ISRs could have been executing 
    // while the timer was still counting asynchronously, this count does not 
    // match the API description right now of returning the actual time spent in 
    // power saving mode. TODO: Use a second synchronous timer so we don't have 
    // to start and stop an asynchronous one, giving us an ability to be 
    // accurate while measuring this sleep duration.
    return NOW_32() - startTime;
}


/**
 * <code><b>
 * void _ISR _TxInterrupt(void);
 * </b></code>
 *
 * Increments internal NOW_internalCount to emulate a 64-bit timer. This
 * interrupt will fire every 65536 instructions and will need around ~15 or 30
 * cycles per interrupt event (exact count depends on processor family).
 */
__asm__("   .pushsection .text._" STRINGIFY(TxInterrupt) ", code, keep \n"
        "   .global __" STRINGIFY(TxInterrupt) "\n"
        "   .weak   __" STRINGIFY(TxInterrupt) "\n"
        "__" STRINGIFY(TxInterrupt) ":          \n"
        "   push    w0                          \n"
        "   bclr    IFS0, #3                    \n" // IFS0, #3 = IFS0bits.T1IF // TODO: Make compatible with NOW_TIMER_RESOURCE macro. This presently will only work for Timer 1.
        "   inc     _NOW_internalCount+0        \n"
        "   clr     w0                          \n"
        "   addc    _NOW_internalCount+2        \n"
        "   addc    _NOW_internalCount+4        \n"
        "   addc    _NOW_internalCount+6        \n"
        "   pop     w0                          \n"
        "   retfie                              \n"
        "   .popsection                         \n"
        );

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
__asm__("   .pushsection .text.NOW_16, code \n"
        "   .global _NOW_16                 \n"
        "_NOW_16:                           \n"
        "   mov     " STRINGIFY(TMRx) ", w0 \n"
        "   return                          \n"
        "   .popsection                     \n"
        );


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
__asm__("   .pushsection .text.NOW_32, code \n"
        "   .global _NOW_32                 \n"
        "_NOW_32:                           \n"
        "   mov     " STRINGIFY(TMRx) ", w0 \n" // Get [15:0] bits
        "   mov     _NOW_internalCount, w1  \n" // Get [31:16] bits from internal counter variable
        "   mov     " STRINGIFY(TMRx) ", w2 \n" // Re-get [15:0] bits to check for rollover
        "   mov     _NOW_internalCount, w3  \n" // Re-get [31:16] bits from internal counter variable
        "   sub     w2, w0, w2              \n" // Check that [15:0] bits did not rollover
        "   bra     N, _NOW_32              \n" // On rollover, start all over again
        "   xor     w1, w3, w3              \n" // Check that [31:16] bits did not increment during the read sequence
        "   bra     NZ, _NOW_32             \n" // On increment, start all over again
        "   return                          \n"
        "   .popsection                     \n"
        );


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
__asm__("   .pushsection .text.NOW_64, code \n"
        "   .global _NOW_64                 \n"
        "_NOW_64:                           \n"
        "   mov     " STRINGIFY(TMRx) ", w0 \n" // Get [15:0] bits
        "   mov     _NOW_internalCount, w1  \n"
        "   mov     _NOW_internalCount+2, w2\n"
        "   mov     _NOW_internalCount+4, w3\n"
        "   mov     " STRINGIFY(TMRx) ", w4 \n" // Reget [15:0] bits for verification
        "   mov     _NOW_internalCount, w5  \n"
        "   mov     _NOW_internalCount+2, w6\n"
        "   mov     _NOW_internalCount+4, w7\n"
        "   sub     w4, w0, w4              \n"
        "   bra     N, _NOW_64              \n"
        "   xor     w1, w5, w5              \n"
        "   bra     NZ, _NOW_64             \n"
        "   xor     w2, w6, w6              \n"
        "   bra     NZ, _NOW_64             \n"
        "   xor     w3, w7, w7              \n"
        "   bra     NZ, _NOW_64             \n"
        "   return                          \n"
        "   .popsection                     \n"
        );

/**
 * <code><b>
 * unsigned long NOW_Diff_to_ms(unsigned long nowCounts);
 * </b></code>
 *
 * Converts the difference between two NOW counts to absolute time in
 * milliseconds. This function is primiarly intended for instances where you
 * wish to display a time interval to a human in decimal forms. It is, however,
 * not subject to the integral approximations that NOW_millisecond represent, so
 * can be used in other cases when absolute accuracy is critical and the device
 * operating frequency is very low (ex: 32.768kHz).
 *
 * nowCounts must be measured against the same clock frequency in use during
 * invokation of this function. In other words, a wrong result will be returned
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
__asm__("   .pushsection .text.NOW_Diff_to_ms, code \n"
        "   .global _NOW_Diff_to_ms     \n" // unsigned int NOW_Diff_to_ms(unsigned long cycleDifference);
        "_NOW_Diff_to_ms:               \n" // Supports up to 536,870,911 cycles (a range of 2,684 ms @ 200MHz)
        "   mul.uu  w0, #8, w2          \n" // Multiply time by 8 (and will divide clock rate by 125) to scale for milliseconds
        "   mul.uu  w1, #8, w0          \n"
        "   add     w1, w3, w3          \n"
        "   push.d  w2                  \n"
        "   mov     _NOW_second+0, w0   \n" // Get 32-bit clock rate
        "   mov     _NOW_second+2, w1   \n"
        "   mov     #125, w2            \n" // Divide 32-bit clock rate by 125
        "   clr     w3                  \n"
        "   call    ___udivsi3          \n" // Compute  (clock_freq/125) (must do 32/32 divide because 32/16 divide instruction is limited to 16-bits out)
        "   mov.d   w0, w2              \n"
        "   pop.d   w0                  \n"
        "   call    ___udivsi3          \n" // Compute  (8*time)/(clock_freq/125)
        "   return                      \n"
        "   .popsection                 \n"
        );


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
__asm__("   .pushsection .text.NOW_Diff_to_us, code \n"
        "   .global _NOW_Diff_to_us     \n" // unsigned long NOW_Diff_to_us(unsigned long cycleDifference);
        "_NOW_Diff_to_us:               \n" // Supports up to 67,108,863 cycles (a range of 335,544 microseconds @ 200MHz)
        "   mov     #64, w4             \n" // Multiply time by 64 (and will divide clock rate by 15,625) to scale for microseconds
        "   mul.uu  w1, w4, w2          \n"
        "   mul.uu  w0, w4, w0          \n"
        "   add     w2, w1, w1          \n"
        "   mov.d   w0, w2              \n"
        "   mov     _NOW_second+0, w0   \n" // Get 32-bit clock rate
        "   mov     _NOW_second+2, w1   \n"
        "   mov     #15625, w4          \n" // Divide 32-bit clock rate by 15625
        "   repeat  #17                 \n"
        "   div.ud  w0, w4              \n"
        "   exch    w0, w2              \n"
        "   exch    w1, w3              \n"
        "   clr     w3                  \n"
        "   call    ___udivsi3          \n" // Compute  (64*time)/(clock_freq/15,625)
        "   return                      \n"
        "   .popsection                 \n"
        );
#endif  // #if defined(__PIC32__) || defined(__XC32__)
