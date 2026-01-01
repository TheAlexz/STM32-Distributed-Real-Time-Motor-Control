#include "main.h"
#include <stdio.h>
#include "application.h"
#include "controller.h"
#include "peripherals.h"
#include "cmsis_os.h"
#include "socket.h"
#include "wizchip_conf.h"
#include "w5500.h"

#define PERIOD_CTRL 10
#define PERIOD_REF 4000

#define APP_SOCK 0
#define SERVER_PORT 2103
uint8_t server_addr[4] = {192, 168, 0, 10};
uint8_t sock_status;

/* Threads -------------------------------------------------------------------*/
//Create the thread handles
osThreadId main_id, thrd_samplingID, thrd_refID, thrd_txrxID, thrd_actuateID;

//Prototypes of the threads
void sampling_thread (void const *argument);
void toggle_reference_thread (void const *argument);
void txrx_thread (void const *argument);
void actuate_thread (void const *argument);

//Define the parameters of each thread
//Including priority of the thread, number of instances and its stack size (zero indicates
//it will have the default stack size)
osThreadDef(actuate_thread, osPriorityNormal, 1, 0);
osThreadDef(txrx_thread, osPriorityBelowNormal, 1, 0);
osThreadDef(toggle_reference_thread, osPriorityLow, 1, 0);
osThreadDef(sampling_thread, osPriorityIdle, 1, 0);

/* Virtual Timers ------------------------------------------------------------*/
void virtualTimer_threadHandler(void const *param);
osTimerDef(sampling_thread_timer_handle, virtualTimer_threadHandler);
osTimerDef(toggle_reference_thread_timer_handle, virtualTimer_threadHandler);
uint8_t samplingTimer_started = 0;
uint8_t toggleReferenceTimer_started = 0;

/* Global variables ----------------------------------------------------------*/
int16_t encoder;
int32_t reference, velocity, control;
uint32_t millisec;

/* Functions -----------------------------------------------------------------*/
uint8_t receiveByte(uint8_t *byte);
uint8_t transmitByte(uint8_t byte);
void resetSystem(void);

/* Run setup needed for all periodic tasks */
int Application_Setup(){
	//Stop the CMSIS-RTOS scheduler
	//since by the default it initializes once when main() is entered
	osKernelInitialize();
		
	// Reset global variables
	velocity = 0;
	control = 0;
	millisec = 0;

	// Initialize hardware
	Peripheral_GPIO_EnableMotor();
	
	// Ensure the main thread has a handle and a priority
	main_id = osThreadGetId();
	osThreadSetPriority(main_id, osPriorityAboveNormal);
	
	//Create the threads
	thrd_samplingID = osThreadCreate(osThread(sampling_thread), NULL);
	thrd_refID = osThreadCreate(osThread(toggle_reference_thread), NULL);
	thrd_txrxID = osThreadCreate(osThread(txrx_thread), NULL);
	thrd_actuateID = osThreadCreate(osThread(actuate_thread), NULL);
	
	//Start the RTOS scheduler again
	osKernelStart();

	return 0;
}

/* Define what to do in the infinite loop */
uint8_t connectionStatus = 0;
uint8_t cableConnection = 0;
void Application_Loop(){
	// Verify virtual timers status.
	if(samplingTimer_started == 1){
		samplingTimer_started = 0;
		osSignalSet(thrd_samplingID, 1);
	}
	if(toggleReferenceTimer_started == 1){
		toggleReferenceTimer_started = 0;
		osSignalSet(thrd_refID, 1);
	}
	
	// Verify ethernet cable connection
	//printf("C > Verify ethernet cable connection... \r\n");
	if(cableConnection = (getPHYCFGR() & PHYCFGR_LNK_ON)){
		// Open socket
		printf("C > Opening socket... \r\n");
		if(socket(APP_SOCK, SOCK_STREAM, SERVER_PORT, SF_TCP_NODELAY) == APP_SOCK){
			printf("C > Socket created! \r\n");	
			// Connect to server
			printf("C > Trying to connect to server... \r\n");
			connectionStatus = connect(APP_SOCK, server_addr, SERVER_PORT);
			if(connectionStatus == SOCK_OK){
				printf("C > Server connected! \r\n");		
				// Clear flag signal.
				osSignalClear(main_id, 1);
				// Wait indefinitely until flag 1 for this thread is set.
				osSignalWait(1, osWaitForever);
			}
			// Else, failed to connect to server
			else{
				printf("C > Failed to connect to server \r\n");
				
				if(connectionStatus == SOCK_OK) close(APP_SOCK);

			}
		}
		// Else, failed to open socket
		//else printf("C > Failed to open socket :( \r\n");
	}
	//else printf("C > Ethernet cable not connected :( \r\n");
}

void sampling_thread(void const *argument){
	// Create virtual timer
	osTimerId sampling_thread_timer = osTimerCreate(osTimer(sampling_thread_timer_handle),osTimerPeriodic, (void *) 0);
	
	// Every 10 millisec ...
	while(1){
		// Clear flag signal for controller timer.
		osSignalClear(thrd_samplingID, 1);
		// Start virtual timer for 10 milliseconds.
		osTimerStart(sampling_thread_timer, PERIOD_CTRL);
		samplingTimer_started = 1;
		// Wait indefinitely until flag 1 for this thread is set.
		osSignalWait(1, osWaitForever);
		
		if(samplingTimer_started == 1){
			// Get time
			millisec = Main_GetTickMillisec();
		
			// Get current velocity
			encoder = Peripheral_Timer_ReadEncoder();
			velocity = Controller_CalculateVelocity(encoder, millisec);
			
			// Run txrx_thread
			osSignalSet(thrd_txrxID, 1);
		}
		samplingTimer_started = 0;
	}		
}

void toggle_reference_thread(void const *argument){	
	// Create virtual timer
	osTimerId toggle_reference_thread_timer = osTimerCreate(osTimer(toggle_reference_thread_timer_handle),osTimerPeriodic, (void *) 1);	
	
	// Every 4 sec ...
	while(1){
		// Clear flag signal for toggle reference timer.
		osSignalClear(thrd_refID, 1);
		// Start virtual timer for 4 seconds.
		osTimerStart(toggle_reference_thread_timer, PERIOD_REF);
		toggleReferenceTimer_started = 1;
		// Wait indefinitely until flag 1 for this thread is set.
		osSignalWait(1, osWaitForever);
		
		if(toggleReferenceTimer_started == 1){
			// Run txrx_thread
			osSignalSet(thrd_txrxID, 2);
		}
		toggleReferenceTimer_started = 0;
	}
}

// Global variables
osEvent evt;
uint8_t byte;
uint8_t shift;
uint32_t aux;
uint8_t loopFlag;


void txrx_thread(void const *argument){
	while(1){
		// Clear flag signals.
		osSignalClear(thrd_txrxID, 1);
		osSignalClear(thrd_txrxID, 2);
		
		// Wait indefinitely until flag 1 or 2 for this thread is set.
		evt = osSignalWait(0, osWaitForever);
		
		// Signal set
		if(evt.status == osEventSignal){
			//----- Send speed and request controller output -----
			if(evt.value.signals == 1){
				// Request to send speed and receive back controller output
				//printf("C > Requesting to send speed and receive back controller output... \r\n");
				if(transmitByte(0) == 1){
					// Await for server acknowledgment.
					if(receiveByte(&byte) == 1){
						// Send speed, one byte at a time
						//printf("C > Sending speed... Speed value: %i \r\n", velocity);
						shift = 0;
						loopFlag = 1;
						for(int i=0; i<4; i++){
							byte = velocity >> shift;
							shift += 8;													
							if(transmitByte(byte) == 1){
								// Await for client acknowledgment.
								if(receiveByte(&byte) == 0){
									loopFlag = 0;
									break;
								}
							}
							else{
								loopFlag = 0;
								break;
							}
						}
						if(loopFlag == 1){
							//printf("C > Speed sent! \r\n");
							
							// Receive controller output from the server, one byte at a time ----------
							//printf("C > Receiving controller output... \r\n");
							control = 0;
							shift = 0;
							for(int i=0; i<4; i++){
								if(receiveByte(&byte) == 1){
									aux = byte << shift;
									control |= aux;
									shift += 8;
									// Acknowledge received data
									if(transmitByte(1) == 0){
										loopFlag = 0;
										break;
									}
								}
								else{
									loopFlag = 0;
									break;
								}
							}
							if(loopFlag == 1){
								//printf("C > Controller output received! Control value: %i \r\n", control);
								
								// Restart actuate_thread
								osSignalSet(thrd_actuateID, 1);
							}
						}
					}
				}
			}
			//----- Request reference toggle -----
			else if(evt.value.signals == 2){
				//printf("C > Request reference toggle \r\n");
				transmitByte(1);
			}
		}
	}
}

void actuate_thread(void const *argument){
	while(1){
		// Clear flag signal.
		osSignalClear(thrd_actuateID, 1);
		// Wait indefinitely until flag 1 for this thread is set.
		osSignalWait(1, osWaitForever);
		
		// Apply control signal to motor
		Peripheral_PWM_ActuateMotor(control);
	}		
}

/*
	sock_status:
		0x14 => SOCK_LISTEN
		0x17 => SOCK_ESTABLISHED
*/
uint8_t recvBuf_status;
uint8_t receiveByte(uint8_t *byte){
	// Verify connection status
	getsockopt(APP_SOCK, SO_STATUS, &sock_status);
	getsockopt(APP_SOCK, SO_RECVBUF, &recvBuf_status);
	while((getPHYCFGR() & PHYCFGR_LNK_ON) && (sock_status == SOCK_LISTEN || sock_status == SOCK_ESTABLISHED)){
		if(sock_status == SOCK_ESTABLISHED && recvBuf_status > 0){
			// Receive byte
			recv(APP_SOCK, (uint8_t*)&(*byte), sizeof(*byte));
			return 1;
		}
		else{
			getsockopt(APP_SOCK, SO_STATUS, &sock_status);
			getsockopt(APP_SOCK, SO_RECVBUF, &recvBuf_status);
		}
	}
	// Connection lost
	resetSystem();
	return 0;
}

uint8_t transmitByte(uint8_t byte){
	// Verify connection status
	getsockopt(APP_SOCK, SO_STATUS, &sock_status);
	while((getPHYCFGR() & PHYCFGR_LNK_ON) && (sock_status == SOCK_LISTEN || sock_status == SOCK_ESTABLISHED)){
		if(sock_status == SOCK_ESTABLISHED){
			// Send byte
			send(APP_SOCK, (uint8_t*)&byte, sizeof(byte));
			return 1;
		}
		else getsockopt(APP_SOCK, SO_STATUS, &sock_status);
	}
	// Connection lost
	resetSystem();
	return 0;
}

// Callback function for virtual timers
void virtualTimer_threadHandler(void const *param){
	switch((uint32_t) param){
		// Set flag signal for controller timer.
		case 0:
			osSignalSet(thrd_samplingID, 1);
		break;
		// Set flag signal for toggle reference timer.
		case 1:
			osSignalSet(thrd_refID, 1);
		break;
		case 2:
			resetSystem();
		break;
	}
}

// Reset system
void resetSystem(){
	// Turn off motor
	Peripheral_PWM_ActuateMotor(0);
	
	printf("C > Connection lost! System reset... \r\n");
	
	Controller_Reset();
	
	if(connectionStatus == SOCK_OK) disconnect(APP_SOCK);
	
	// Reset global variables
	velocity = 0;
	control = 0;
	millisec = 0;
	
	// Restart main thread to reestablish connection.
	osSignalSet(main_id, 1);
}
