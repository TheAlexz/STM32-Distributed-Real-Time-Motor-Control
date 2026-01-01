#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/

/* Enable both half-bridges to drive the motor */
void Peripheral_GPIO_EnableMotor(void);

/* Drive the motor in both directions: +/- 1,000,000,000 <=> +/- 100% duty cycle */
void Peripheral_PWM_ActuateMotor(int32_t vel);

/* Read the counter register to get the encoder state */
int16_t Peripheral_Timer_ReadEncoder(void);

#ifdef __cplusplus
}
#endif

#endif   // _PERIPHERALS_H_
