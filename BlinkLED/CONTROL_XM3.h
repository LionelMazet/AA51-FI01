/**
 * @brief Hardware Abstraction Layer.
 * This module configure GPIO and provide functional abstraction.
 */


#if !defined CONTROL_XM3_H
#define CONTROL_XM3_H



#ifndef CONTROL_XM3_C
// initialisation des GPIO LED, alim 15V des capteurs de courant et eQEP1 et eQEP2
extern void initGPIO(void);
// Current Sensor
extern void enableNeg15V(void);
extern void enablePos15V(void);
extern void disableNeg15V(void);
extern void disablePos15V(void);

#else // CONTROL_XM3_C
void initGPIO(void);
// Current Sensor
void enableNeg15V(void);
void enablePos15V(void);
void disableNeg15V(void);
void disablePos15V(void);
#endif

#endif // CONTROL_XM3_H
