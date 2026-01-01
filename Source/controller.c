#include "controller.h"

int32_t speed = 0;
uint32_t prevMs1 = 0;
uint32_t prevMs2 = 0;
uint32_t timeChange = 0;
uint8_t firstCallFlag1 = 0;
uint8_t firstCallFlag2 = 0;

int32_t clamp_30bit(int32_t a, int32_t b);

/* Calculate the current velocity in rpm, based on encoder value and time */
int32_t Controller_CalculateVelocity(int16_t encoder, uint32_t millisec)
{
	/*
		Motor speed is calculated as follows:
		speed = change in counter / change in time		[pulses per ms]
		speed = speed*1000 		[pulses per s]
		speed = speed*60	 		[pulses per min]
		speed = speed/512*4		[rev per min = rpm]
	*/
	
	if(firstCallFlag1 == 1){
		if(millisec >= prevMs1){
			timeChange = millisec - prevMs1;
		}
		else{
			timeChange = millisec + (UINT32_MAX - prevMs1 + 1);
		}
		prevMs1 = millisec;
		
		// Top speed ~ 10800 rpm
		speed = (1875*encoder)/(64*(int16_t)timeChange);
		return speed;
	}
	// If the function is invoked for the first time, only store prevMs1:
	else{
		firstCallFlag1 = 1;
		prevMs1 = millisec;
		return 0;
	}
}

int32_t error = 0;
int32_t output = 0;
int32_t integrator = 0;

// Kp
int32_t Kp_num = 20;
int32_t Kp_den = 1;
// Ti
int32_t Ti_num = 1;
int32_t Ti_den = 40;

uint8_t resetFlag = 0;
int32_t controlSignal = 0;
/* Apply a PI-control law to calcuate the control signal for the motor*/
int32_t Controller_PIController(int32_t ref, int32_t current, uint32_t millisec)
{
	if(resetFlag == 1){
		resetFlag = 0;
		return 0;
	}
	
	if(firstCallFlag2 == 1){
		error = ref - current;
		if(error < -21600){
			error = -21600;
		}
		else if(error > 21600){
			error = 21600;
		}
		
		if(millisec >= prevMs2){
			timeChange = millisec - prevMs2;
		}
		else{
			timeChange = millisec + (UINT32_MAX - prevMs2 + 1);
		}
		prevMs2 = millisec;
		
		integrator = clamp_30bit(integrator, (error*Kp_num*Ti_den/(Kp_den*Ti_num))*timeChange);
		controlSignal = clamp_30bit(error*Kp_num/Kp_den, integrator);
		
		// Apply controller
		return controlSignal;
	}
	// If the function is invoked for the first time, only store prevMs2:
	else{
		firstCallFlag2 = 1;
		prevMs2 = millisec;
		return 0;
	}
}

/* Reset internal state variables, such as the integrator */
void Controller_Reset(void)
{
	resetFlag = 1;
	integrator = 0;
	output = 0;
	firstCallFlag1 = 0;
	firstCallFlag2 = 0;
	return;
}

// Clamping helper function
int32_t clamp_30bit(int32_t a, int32_t b){
	uint8_t aSign, bSign;
	int32_t sum;
	aSign = (a >> 31) & 0x1;
	bSign = (b >> 31) & 0x1;
	int32_t upperLimit = (1<<30) - 1;
	int32_t lowerLimit = (1<<30)*-1;
	
	sum = a + b;
	
	if(aSign == bSign){
		if(aSign == 0 && (sum < a || sum < b || sum > upperLimit)) return upperLimit;
		else if(aSign == 1 && (sum > a || sum > b || sum < lowerLimit)) return lowerLimit;
	}
	
	return sum;
}