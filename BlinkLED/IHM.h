/**
 * @brief IHM LED 
 */

#ifndef IHM_H
#define IHM_H

// CASELED1 PWM for Effect 
#define EPWM_CMP_UP           1U
#define EPWM_CMP_DOWN         0U
#define EPWM6_TIMER_TBPRD  2000U
#define EPWM6_MAX_CMPA     1950U
#define EPWM6_MIN_CMPA       50U

typedef struct
{
    uint32_t epwmModule;
    uint16_t epwmCompADirection;
    uint16_t epwmTimerIntCount;
    uint16_t epwmMaxCompA;
    uint16_t epwmMinCompA;
}LEDepwmInformation;

#ifndef IHM_C
extern __interrupt void epwm6ISR(void);
extern LEDepwmInformation epwmLEDInfo;
extern void TestLEDS(void);
extern void TestLEDPulse(void);
extern void initCaseLEDPWM(void);
extern void updateLED(LEDepwmInformation *);
extern void initIHM(void);

#else // IHM_C
void TestLEDS(void);
void TestLEDPulse(void);
void initCaseLEDPWM(void);

#endif // IHM_C

#endif // IHM_H
