#include "main.h"

#include "application.h"
#include "controller.h"
#include "peripherals.h"
#include "cmsis_os.h"

#define PERIOD_CTRL 10
#define PERIOD_REF 4000

/* Threads -------------------------------------------------------------------*/
//Create the thread handles
osThreadId thrd_ctrlID, thrd_refID, main_id;

//Prototypes of the threads
void controller_thread (void const *argument);
void toggle_reference_thread (void const *argument);

//Define the parameters of each thread
//Including priority of the thread, number of instances and its stack size (zero indicates
//it will have the default stack size)
osThreadDef(controller_thread, osPriorityNormal, 1, 0);
osThreadDef(toggle_reference_thread, osPriorityAboveNormal, 1, 0);

/* Virtual Timers ------------------------------------------------------------*/
void thread_handler(void const *param);
osTimerDef(controller_thread_timer_handle, thread_handler);
osTimerDef(toggle_reference_thread_timer_handle, thread_handler); 

/* Global variables ----------------------------------------------------------*/
int16_t encoder;
int32_t reference, velocity, control;
uint32_t millisec;

/* Functions -----------------------------------------------------------------*/

/* Run setup needed for all periodic tasks */
int Application_Setup()
{
	//Stop the CMSIS-RTOS scheduler (which was already initialized)
	//since by the default it initializes once when main() is entered
	osKernelInitialize();
	
	// Reset global variables
	reference = 1000;
	velocity = 0;
	control = 0;
	millisec = 0;

	// Initialise hardware
	Peripheral_GPIO_EnableMotor();
	
	//Ensure the main thread has a handle and a priority	
	main_id = osThreadGetId();
	osThreadSetPriority(main_id, osPriorityNormal);
	
	//Create the threads
	thrd_ctrlID = osThreadCreate(osThread(controller_thread), NULL);
	thrd_refID = osThreadCreate(osThread(toggle_reference_thread), NULL);
		
	//Start the RTOS scheduler again
	osKernelStart();

	return 0;
}

/* Define what to do in the infinite loop */
void Application_Loop()
{
		//Do nothing
		osSignalWait(0x01, osWaitForever);		
}

void controller_thread (void const *argument){
	
		//Instantiate the virtual timer
		osTimerId controller_thread_timer = osTimerCreate(osTimer(controller_thread_timer_handle),osTimerPeriodic, (void *) 0); 
	
		while(1){
			// Every 10 millisec ...
			
			// First implementation
			//osDelay(PERIOD_CTRL);  
			
			// Clear flag signal for controller timer.
			osSignalClear(thrd_ctrlID, 1);
			// Start virtual timer for 10 milliseconds.
			osTimerStart(controller_thread_timer, PERIOD_CTRL);
			// Wait indefinitely until flag 1 for this thread is set.
			osSignalWait(1, osWaitForever);	
			
			// Get time
			millisec = Main_GetTickMillisec();
		
			// Get current velocity
			encoder = Peripheral_Timer_ReadEncoder();
			velocity = Controller_CalculateVelocity(encoder, millisec);

			// Calculate control signal
			control = Controller_PIController(reference, velocity, millisec);
			
			// Apply control signal to motor
			Peripheral_PWM_ActuateMotor(control);		
		}		
}
void toggle_reference_thread (void const *argument){
	
		//Instantiate the virtual timer
		osTimerId toggle_reference_thread_timer = osTimerCreate(osTimer(toggle_reference_thread_timer_handle),osTimerPeriodic, (void *) 1);
	
		while(1){
			// Every 4 sec ...
			
			// First implementation
			//osDelay(PERIOD_REF); 
			
			// Clear flag signal for toggle reference timer.
			osSignalClear(thrd_refID, 1);
			// Start virtual timer for 4 seconds.
			osTimerStart(toggle_reference_thread_timer, PERIOD_REF);
			// Wait indefinitely until flag 1 for this thread is set.
			osSignalWait(1, osWaitForever);	
				
			// Flip the direction of the reference
			reference = -reference;		
		}
}

// Callback function for virtual timers
void thread_handler(void const *param){
	switch((uint32_t) param){
		// Set flag signal for controller timer.
		case 0:
			osSignalSet(thrd_ctrlID, 1);
		break;
		// Set flag signal for toggle reference timer.
		case 1:
			osSignalSet(thrd_refID, 1);
		break;
	}
}
