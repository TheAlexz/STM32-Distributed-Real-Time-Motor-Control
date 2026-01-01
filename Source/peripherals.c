#include "stm32l4xx.h"
#include "peripherals.h"

/* Enable both half-bridges to drive the motor */
void Peripheral_GPIO_EnableMotor(void)
{
	// INH_1 <=> D12 <=> PA6
	// Set PA6 pin to logic 1. Enables Half-Bridge 1.
	GPIOA->BSRR = GPIO_BSRR_BS6;
	
	// INH_2 <=> D13 <=> PA5
	// Set PA5 pin to logic 1. Enables Half-Bridge 2.
	GPIOA->BSRR = GPIO_BSRR_BS5;
	
	// IN_1 <=> D5 <=>  PB4
	// Set CH1 PWM's DC to 0.
	TIM3->CCR1 = 0;
	
	// IN_2 <=> D11 <=> PA7
	// Set CH2 PWM's DC to 0.
	TIM3->CCR2 = 0;
	
	return;
}

// Spinning motor direction (0 = CW, 1 = CCW)
unsigned char direction = 0;

// The motor reaches max speed at 2^11 - 1 = 2047 on CCR
uint16_t dc = 0;

// Value of vel's MSB (sign bit)
uint16_t velSign = 0;

/* Drive the motor in both directions: +/- 1,000,000,000 <=> +/- 100% duty cycle */
void Peripheral_PWM_ActuateMotor(int32_t vel)
{
	velSign = (vel >> 31) & 0x1;
	
	// Positive number
	if(velSign == 0){
		direction = 0;
		
		// Disable CH1
		TIM3->CCR1 = 0;
		// Scale "vel" (32 bits signed integer) to get CH2 PWM's DC (11 bits unsigned integer)
		dc = vel >> 19;
		TIM3->CCR2 = dc;
	}
	// Negative number
	else{
		direction = 1;
		
		// Disable CH2
		TIM3->CCR2 = 0;
		// Apply two's complement and scale "vel" (32 bits signed integer) to get CH1 PWM's DC (11 bits unsigned integer)
		dc = ((~vel) + 1) >> 19;
		TIM3->CCR1 = dc;
	}
	dc = dc*100/2047;
	return;
}

// Timer 1 counter
uint16_t counter = 0;
// Previous timer 1 counter value
uint16_t prevCounter = 0;
// Timer 1 counter value difference between two consecutive samples
int16_t delta = 0;

/* Read the counter register to get the encoder state */
int16_t Peripheral_Timer_ReadEncoder(void)
{
	// Read TIMER1 counter
	counter = TIM1->CNT;
	
	// Calculate counter value change.
	if(direction == 0){
		if(counter >= prevCounter){
			delta = (int16_t)(counter - prevCounter);
		}
		else{
			delta = (int16_t)(counter + (UINT16_MAX - prevCounter + 1));
		}
	}
	else{
		if(prevCounter >= counter){
			delta = -((int16_t)(prevCounter - counter));
		}
		else{
			delta = -((int16_t)(prevCounter + (UINT16_MAX - counter + 1)));
		}
	}
	
	// Save counter's value
	prevCounter = counter;
	
	return delta;
}
