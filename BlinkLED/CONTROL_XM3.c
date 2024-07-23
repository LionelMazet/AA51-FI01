/**
 * CONTROL_XM3.c
 *
 *  Created on: 23 juil. 2024
 *  AUTHOR: mazet (ERNEO), mfeurtado, Matthew Feurtado (Wolfspeed)
 *  @brief Initialisation des GPIO LED, GPIO Alim 15V, GPIO eQEP, LED
 *
 */

#define CONTROL_XM3_C
#include "driverlib.h"
#include "device.h"



//
// initGPIO - this initializes the GPIO for controller not in external files.
//
void initGPIO(void)
{
    //
    // LEDS
    //
    // Enable GPIO outputs on GPIO10,31,34,41,52,65, set it LOW
    /*
    10  CASE-LED
    31  LP-LED-BLUE
    34  LP-LED-RED
    41  LED-G
    52  LED-Y
    65  LED-R
    */
    //CASELED1 is PWM to do fade
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO10
    GPIO_setPinConfig(GPIO_10_EPWM6A);              // GPIO10 = EPWM6A


    GPIO_setPadConfig(31, GPIO_PIN_TYPE_STD);    // Enable pullup on GPIO10
    GPIO_writePin(31, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_31_GPIO31);              // GPIO10 = GPIO10
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_OUT);   // GPIO10 = output

    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);    // Enable pullup on GPIO10
    GPIO_writePin(34, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_34_GPIO34);              // GPIO10 = GPIO10
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);   // GPIO10 = output

    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);    // Enable pullup on GPIO10
    GPIO_writePin(41, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_41_GPIO41);              // GPIO10 = GPIO10
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_OUT);   // GPIO10 = output

    GPIO_setPadConfig(52, GPIO_PIN_TYPE_STD);    // Enable pullup on GPIO10
    GPIO_writePin(52, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_52_GPIO52);              // GPIO10 = GPIO10
    GPIO_setDirectionMode(52, GPIO_DIR_MODE_OUT);   // GPIO10 = output

    GPIO_setPadConfig(65, GPIO_PIN_TYPE_STD);    // Enable pullup on GPIO10
    GPIO_writePin(65, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_65_GPIO65);              // GPIO10 = GPIO10
    GPIO_setDirectionMode(65, GPIO_DIR_MODE_OUT);   // GPIO10 = output

    // PS-CONTROL +15V and -15V
    //
    // Enable GPIO outputs on GPIO124,125, set it LOW
    /*
    124 SHUTDOWN+15V
    125 SHUTDOWN-15V
    */
    GPIO_setPadConfig(124, GPIO_PIN_TYPE_STD);    // Enable pullup on GPIO10
    GPIO_writePin(124, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_124_GPIO124);              // GPIO10 = GPIO10
    GPIO_setDirectionMode(124, GPIO_DIR_MODE_OUT);   // GPIO10 = output

    GPIO_setPadConfig(125, GPIO_PIN_TYPE_STD);    // Enable pullup on GPIO10
    GPIO_writePin(125, 0);                           // Load output latch
    GPIO_setPinConfig(GPIO_125_GPIO125);              // GPIO10 = GPIO10
    GPIO_setDirectionMode(125, GPIO_DIR_MODE_OUT);   // GPIO10 = output

    //
    // Enable EQEP1 on GPIO's 20,21,99
    /*
    20  QEA_A
    21  QEA_B
    99  QEA_I
    */
    GPIO_setPadConfig(20, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO20
    GPIO_setPadConfig(21, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO21
    GPIO_setPadConfig(99, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO23
    GPIO_setQualificationMode(20, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setQualificationMode(21, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setQualificationMode(99, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setPinConfig(GPIO_20_EQEP1A);              // GPIO20 = EQEP1A
    GPIO_setPinConfig(GPIO_21_EQEP1B);              // GPIO21 = EQEP1B
    GPIO_setPinConfig(GPIO_99_EQEP1I);              // GPIO99 = EQEP1I

    //
    // Enable EQEP2 on GPIO's 54,55,57
    /*
    54  QEB_A
    55  QEB_B
    57  QEB_I
    */
    GPIO_setPadConfig(54, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO54
    GPIO_setPadConfig(55, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO55
    GPIO_setPadConfig(57, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO57
    GPIO_setQualificationMode(54, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setQualificationMode(55, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setQualificationMode(57, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setPinConfig(GPIO_54_EQEP2A);              // GPIO54 = EQEP1A
    GPIO_setPinConfig(GPIO_55_EQEP2B);              // GPIO55 = EQEP1B
    GPIO_setPinConfig(GPIO_57_EQEP2I);              // GPIO57 = EQEP1I
}


/*
124 SHUTDOWN+15V
125 SHUTDOWN-15V
*/
void enableNeg15V()
{
    GPIO_writePin(125,0);
}
void enablePos15V()
{
    GPIO_writePin(124,0);
}
void disableNeg15V()
{
    GPIO_writePin(125,1);
}
void disablePos15V()
{
    GPIO_writePin(124,1);
}
