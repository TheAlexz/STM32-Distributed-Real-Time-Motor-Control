#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_
#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/

/* Calculate the current velocity in rpm, based on encoder value and time */
int32_t Controller_CalculateVelocity(int16_t encoder, uint32_t millisec);

/* Apply a PI-control law to calcuate the control signal for the motor*/
int32_t Controller_PIController(int32_t ref, int32_t current, uint32_t millisec);

/* Reset internal state variables, such as the integrator */
void Controller_Reset(void);

#ifdef __cplusplus
 }
#endif

#endif   // _CONTROLLER_H_
