#include "main.h"
#include <stdio.h>
#include "application.h"
#include "controller.h"
#include "peripherals.h"
#include "cmsis_os.h"
#include "socket.h"
#include "wizchip_conf.h"
#include "w5500.h"

#define APP_SOCK 0
#define SERVER_PORT 2103
uint8_t sock_status;

/* Threads -------------------------------------------------------------------*/
//Create the thread handles
osThreadId main_id, thrd_refID, thrd_ctrlID, thrd_txrxID;

//Prototypes of the threads
void toggle_reference_thread (void const *argument);
void controller_thread (void const *argument);
void txrx_thread (void const *argument);

// Define the parameters of each thread
// Including priority of the thread, number of instances and its stack size (zero indicates
// it will have the default stack size)
osThreadDef(toggle_reference_thread, osPriorityNormal, 1, 0);
osThreadDef(controller_thread, osPriorityBelowNormal, 1, 0);
osThreadDef(txrx_thread, osPriorityLow, 1, 0);

/* Global variables ----------------------------------------------------------*/
int32_t reference, velocity, control;
int32_t debugVel;
uint32_t millisec;

/* Functions -----------------------------------------------------------------*/
uint8_t receiveByte(uint8_t *byte);
uint8_t transmitByte(uint8_t byte);
void resetSystem(void);

/* Run setup needed for all periodic tasks */
int Application_Setup(){
	// Stops the CMSIS-RTOS scheduler.
	osKernelInitialize();
	
	// Reset global variables.
	reference = 1000;
	velocity = 0;
	control = 0;
	millisec = 0;

	// Initialize hardware.
	Peripheral_GPIO_EnableMotor();
	
	// Ensure the main thread has a handle and a priority	.
	main_id = osThreadGetId();
	osThreadSetPriority(main_id, osPriorityAboveNormal);
	
	// Create the threads.
	thrd_refID = osThreadCreate(osThread(toggle_reference_thread), NULL);
	thrd_ctrlID = osThreadCreate(osThread(controller_thread), NULL);
	thrd_txrxID = osThreadCreate(osThread(txrx_thread), NULL);	
	
	// Restart the RTOS scheduler.
	osKernelStart();

	return 0;
}

/* Define what to do in the infinite loop */
uint8_t connectionStatus = 0;
uint8_t cableConnection = 0;
void Application_Loop(){
	// Verify ethernet cable connection
	//printf("C > Verify ethernet cable connection... \r\n");
	if(cableConnection = (getPHYCFGR() & PHYCFGR_LNK_ON)){
		printf("S > Opening socket... \r\n");	
		if(socket(APP_SOCK, SOCK_STREAM, SERVER_PORT, SF_TCP_NODELAY) == APP_SOCK){
			printf("S > Socket initialized! \r\n");			
			// Put socket in listening mode.
			printf("S > Listening for a connection request... \r\n");
			if((connectionStatus = listen(APP_SOCK)) == SOCK_OK){
				printf("S > Connection established! \r\n");
				// Clear flag signal.
				osSignalClear(main_id, 1);
				// Wait indefinitely until flag 1 for this thread is set.
				osSignalWait(1, osWaitForever);
			}
			// Else, failed to listen
			else{
				printf("S > Failed to listen :( \r\n");
				if(connectionStatus == SOCK_OK) close(APP_SOCK);
			} 
		}
		// Else, failed to open socket		
		else printf("S > Failed to open socket :( \r\n");
	}
	//else printf("C > Ethernet cable not connected :( \r\n");
}

void controller_thread(void const *argument){	
	while(1){
			// Clear flag signal.
			osSignalClear(thrd_ctrlID, 1);
			// Wait indefinitely until flag 1 for this thread is set.
			osSignalWait(1, osWaitForever);
			
			// Get time
			millisec = Main_GetTickMillisec();
			
			// Calculate control signal
			control = Controller_PIController(reference, velocity, millisec);
		}
}

void toggle_reference_thread (void const *argument){
	while(1){
		// Clear flag signal.
		osSignalClear(thrd_refID, 1);
		// Wait indefinitely until flag 1 for this thread is set.
		osSignalWait(1, osWaitForever);
			
		// Flip the direction of the reference
		reference = -reference;		
	}
}

uint8_t byte;
uint8_t shift;
uint32_t aux;
uint8_t loopFlag;
void txrx_thread(void const *argument){
	while(1){
		//printf("S > Listening for requests... \r\n");
		if(receiveByte(&byte) == 1){
			//printf("S > Request received! Request code: %i \r\n", byte);
			//----- Client sends speed and requests controller output -----
			if(byte == 0){
				// Receive speed, one byte at a time ----------
				//printf("S > Receiving speed... \r\n");
				velocity = 0;
				shift = 0;
				loopFlag = 1;
				// Acknowledge request.
				if(transmitByte(0) == 1){
					for(int i=0; i<4; i++){
						if(receiveByte(&byte) == 1){
							aux = byte << shift;
							velocity |= aux;
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
						//printf("S > Speed received! Value: %i \r\n", velocity);
						debugVel = velocity;
						// Toggle controller_thread
						osSignalSet(thrd_ctrlID, 1);
						
						// Send controller output back to the client, one byte at a time ----------
						//printf("S > Sending controller output... Control value: %i \r\n", control);
						shift = 0;
						for(int i=0; i<4; i++){
							byte = control >> shift;
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
						//if(loopFlag == 1) printf("S > Controller output sent! \r\n");
					}
				}
			}
			//----- Client requests reference toggle -----
			else if(byte == 1){
				// Run toggle_reference_thread
				//printf("S > Toggle reference \r\n");
				osSignalSet(thrd_refID, 1);
			}
		}
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

// Reset system
void resetSystem(){
	printf("S > Connection lost! System reset... \r\n");
	
	Controller_Reset();
	
	if(connectionStatus == SOCK_OK) disconnect(APP_SOCK);
	
	// Reset global variables
	reference = 1000;
	velocity = 0;
	control = 0;
	millisec = 0;
	
	// Restart main thread to reestablish connection.
	osSignalSet(main_id, 1);
}
