/**
 * IHM.c
 *
 *  Created on: 23 juil. 2024
 *      Author: mazet
 */
#define IHM_C
#include "device.h"
#include "IHM.h"

LEDepwmInformation epwmLEDInfo;

__interrupt void epwm6ISR(void);

void updateLED(LEDepwmInformation *epwmInfo)
{
    uint16_t compAValue;

    compAValue = EPWM_getCounterCompareValue(epwmInfo->epwmModule,
                                             EPWM_COUNTER_COMPARE_A);

    //
    //  Change the CMPA values every 10th interrupt.
    //
    if(epwmInfo->epwmTimerIntCount == 10U)
    {
        epwmInfo->epwmTimerIntCount = 0U;

        //
        // If we were increasing CMPA, check to see if we reached the max
        // value. If not, increase CMPA else, change directions and decrease
        // CMPA
        //
        if(epwmInfo->epwmCompADirection == EPWM_CMP_UP)
        {
            if(compAValue < (epwmInfo->epwmMaxCompA))
            {
                EPWM_setCounterCompareValue(epwmInfo->epwmModule,
                                            EPWM_COUNTER_COMPARE_A,
                                            ++compAValue);
            }
            else
            {
                epwmInfo->epwmCompADirection = EPWM_CMP_DOWN;
                EPWM_setCounterCompareValue(epwmInfo->epwmModule,
                                            EPWM_COUNTER_COMPARE_A,
                                            --compAValue);
            }
        }
        //
        // If we were decreasing CMPA, check to see if we reached the min
        // value. If not, decrease CMPA else, change directions and increase
        // CMPA
        //
        else
        {
            if( compAValue == (epwmInfo->epwmMinCompA))
            {
                epwmInfo->epwmCompADirection = EPWM_CMP_UP;
                EPWM_setCounterCompareValue(epwmInfo->epwmModule,
                                            EPWM_COUNTER_COMPARE_A,
                                            ++compAValue);
            }
            else
            {
                EPWM_setCounterCompareValue(epwmInfo->epwmModule,
                                            EPWM_COUNTER_COMPARE_A,
                                            --compAValue);
            }
        }

    }
    else
    {
        epwmInfo->epwmTimerIntCount++;
    }
}





//
// epwm6ISR - ePWM 6 ISR
//
__interrupt void epwm6ISR(void)
{
    //
    // Update the CMPA and CMPB values
    //
    updateLED(&epwmLEDInfo);

    //
    // Clear INT flag for this timer
    //
    EPWM_clearEventTriggerInterruptFlag(EPWM6_BASE);

    //
    // Acknowledge interrupt group
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}



//
// TestLEDS - cycle led pins to test functionality
//
void TestLEDPulse(){
    Interrupt_register(INT_EPWM6, &epwm6ISR);
    initCaseLEDPWM();
    Interrupt_enable(INT_EPWM6);

}//TestLEDPulse
//
// TestLEDS - cycle led pins to test functionality
//
void TestLEDS(){

    GPIO_togglePin(10);
    GPIO_togglePin(31);
    GPIO_togglePin(34);
    GPIO_togglePin(41);
    GPIO_togglePin(52);
    GPIO_togglePin(65);

    DEVICE_DELAY_US(200000);
}//TestLEDS

//
// initCaseLEDPWM
//
void initCaseLEDPWM(void)
{

    Interrupt_register(INT_EPWM6, &epwm6ISR);

    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);    // Enable pullup on GPIO10
    GPIO_setPinConfig(GPIO_10_EPWM6A);              // GPIO10 = EPWM6A
    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM6_BASE, EPWM6_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM6_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM6_BASE, 0U);

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM6_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                1000U);


    //
    // Set up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM6_BASE, EPWM_COUNTER_MODE_UP);
    EPWM_disablePhaseShiftLoad(EPWM6_BASE);
    EPWM_setClockPrescaler(EPWM6_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM6_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM6_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM6_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);


    //
    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    EPWM_setInterruptSource(EPWM6_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM6_BASE);
    EPWM_setInterruptEventCount(EPWM6_BASE, 6U);

    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //
    epwmLEDInfo.epwmCompADirection = EPWM_CMP_UP;
    epwmLEDInfo.epwmTimerIntCount = 0U;
    epwmLEDInfo.epwmModule = EPWM6_BASE;
    epwmLEDInfo.epwmMaxCompA = EPWM6_MAX_CMPA;
    epwmLEDInfo.epwmMinCompA = EPWM6_MIN_CMPA;
}

