/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "queue.h"
/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"
#include "string.h"

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

typedef struct 
{
    char MessageID;
    char Data[ 20 ];
}AMessage;
QueueHandle_t Monitor_QueueHandler;

BaseType_t  xReturned;
TaskHandle_t Button_1_MonitorHandler 			= NULL;
TaskHandle_t Button_2_MonitorHandler 			= NULL;
TaskHandle_t Periodic_TransmitterHandler  = NULL;
TaskHandle_t Uart_ReceiverHandler				  = NULL;
TaskHandle_t Load_1_SimulationHandler 		= NULL;
TaskHandle_t Load_2_SimulationHandler 		= NULL;
/*Global vriables*/
int task_1_in_time = 0,task_1_out_time = 0,task_1_total_time = 0;
int task_2_in_time = 0,task_2_out_time = 0,task_2_total_time = 0;
int task_3_in_time = 0,task_3_out_time = 0,task_3_total_time = 0;
int task_4_in_time = 0,task_4_out_time = 0,task_4_total_time = 0;
int task_5_in_time = 0,task_5_out_time = 0,task_5_total_time = 0;
int task_6_in_time = 0,task_6_out_time = 0,task_6_total_time = 0;
int systemTime=0;
int CPU_Load=0;
char runTimeStatsBuff[200];
/*Defines*/
#define BUTTON1_PORT 										PORT_0
#define BUTTON2_PORT 										PORT_0
#define BUTTON1_PIN 									  PIN8
#define BUTTON2_PIN  										PIN9

#define TASK1_PERIOD										50
#define TASK2_PERIOD										50
#define TASK3_PERIOD										100
#define TASK4_PERIOD										20
#define TASK5_PERIOD										10
#define TASK6_PERIOD										100

#define RUNTIME_STATUS_EN               0
/* Task to be created. */


uint8_t B1_Current_state,B1_Prev_state;
uint8_t B2_Current_state,B2_Prev_state;
void Button_1_Monitor( void * pvParameters )
{
		 TickType_t xLastWakeTime = xTaskGetTickCount();
//		 static uint8_t B1_Current_state,B1_Prev_state;
//		 static uint8_t B1_Prev_state;
		 AMessage Button_1_Message;	
		 Button_1_Message.MessageID=1;
		 
		 vTaskSetApplicationTaskTag( NULL, ( void * ) 1 );
		 for(;;)
		 {
			 B1_Current_state = GPIO_read(BUTTON1_PORT, BUTTON1_PIN);
			 B1_Current_state ?(strcpy(Button_1_Message.Data,"BUTTON_1 is High   \n")):(strcpy(Button_1_Message.Data,"BUTTON_1 is LOW    \n"));
			 if(B1_Current_state != B1_Prev_state)
			 {
				 B1_Prev_state=B1_Current_state;
				 xQueueSend( Monitor_QueueHandler, ( void * ) &Button_1_Message, ( TickType_t ) 0 );	
			 }
			 vTaskDelayUntil(&xLastWakeTime,TASK1_PERIOD);
		 }	 		 
}

void Button_2_Monitor( void * pvParameters )
{
		 TickType_t xLastWakeTime = xTaskGetTickCount();
//		 static uint8_t B2_Current_state,B2_Prev_state;
		 AMessage Button_2_Message;	
		 Button_2_Message.MessageID=2;		 
		 vTaskSetApplicationTaskTag( NULL, ( void * ) 2 );
		 for(;;)
		 {
			 B2_Current_state = GPIO_read(BUTTON2_PORT, BUTTON2_PIN);
			 B2_Current_state ?(strcpy(Button_2_Message.Data,"BUTTON_2 is High   \n")):(strcpy(Button_2_Message.Data,"BUTTON_2 is LOW    \n"));
			 if(B2_Current_state != B2_Prev_state)
			 {
				 B2_Prev_state=B2_Current_state;
				 xQueueSend( Monitor_QueueHandler, ( void * ) &Button_2_Message, ( TickType_t ) 0 );	
			 }			 
			 vTaskDelayUntil(&xLastWakeTime,TASK2_PERIOD);
		 }
}

void Periodic_Transmitter( void * pvParameters )
{
		 TickType_t xLastWakeTime = xTaskGetTickCount();
		 AMessage Transmit_Message;	
		 Transmit_Message.MessageID=3;
		 vTaskSetApplicationTaskTag( NULL, ( void * ) 3 );
		 for(;;)
		 {
			 strcpy(Transmit_Message.Data,"Transmitter_Task   \n");
			 xQueueSend( Monitor_QueueHandler, ( void * ) &Transmit_Message, ( TickType_t ) 0 );		 
			 vTaskDelayUntil(&xLastWakeTime,TASK3_PERIOD);
		 }
}

void Uart_Receiver( void * pvParameters )
{
	AMessage Recv_Message;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 4 );	
	for(;;)
	{
		if(xQueueReceive( Monitor_QueueHandler,&Recv_Message,( TickType_t ) 0 ) == pdTRUE)
		{
			vSerialPutString((const signed char *)&(Recv_Message.Data),20);	
		}
		
		vTaskDelayUntil(&xLastWakeTime,TASK4_PERIOD);
	}
	
}



void vApplicationTickHook( void )
{
	GPIO_write(PORT_0, PIN0,PIN_IS_HIGH);
	GPIO_write(PORT_0, PIN0,PIN_IS_LOW);
}
void vApplicationIdleHook( void )
{
//	GPIO_write(PORT_0, PIN3,PIN_IS_HIGH);
//	GPIO_write(PORT_0, PIN3,PIN_IS_LOW);
}


void Load_1_Simulation( void * pvParameters )
{
	TickType_t xLastWakeTime;
  uint32_t i = 0;
	xLastWakeTime = xTaskGetTickCount();
  vTaskSetApplicationTaskTag( NULL, ( void * ) 5 );
	for( ;; )
		{
			for(i=0;i<37500;i++) {}  /*produce Excution Time of around 5 ms */
				
			vTaskDelayUntil( &xLastWakeTime, TASK5_PERIOD );
		}
}
void Load_2_Simulation( void * pvParameters )
{
	TickType_t xLastWakeTime;
  uint32_t i = 0;
	xLastWakeTime = xTaskGetTickCount();
  vTaskSetApplicationTaskTag( NULL, ( void * ) 6 );
	for( ;; )
		{
			for(i=0;i<89500;i++) {}  /*produce Excution Time of around 5 ms */
				
			vTaskDelayUntil( &xLastWakeTime, TASK6_PERIOD );
		}
	
}

 


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	/* Create a queue capable of containing 10 AMessages. */
  Monitor_QueueHandler = xQueueCreate( 10, sizeof(AMessage) );
	
		xReturned = xTaskPeriodicCreate(
									Button_1_Monitor,                /* Function that implements the task. */
									"Button_1_Monitor Task",              /* Text name for the task. */
									100,      									/* Stack size in words, not bytes. */
									( void * ) 0,    						/* Parameter passed into the task. */
									1,													/* Priority at which the task is created. */
									&Button_1_MonitorHandler,
									TASK1_PERIOD);       /* Used to pass out the created task's handle. */
									
		xReturned = xTaskPeriodicCreate(
									Button_2_Monitor,                /* Function that implements the task. */
									"Button_2_Monitor Task",              /* Text name for the task. */
									100,      									/* Stack size in words, not bytes. */
									( void * ) 0,    						/* Parameter passed into the task. */
									1,													/* Priority at which the task is created. */
									&Button_2_MonitorHandler,
									TASK2_PERIOD);       /* Used to pass out the created task's handle. */
	
	xReturned = xTaskPeriodicCreate(
									Periodic_Transmitter,                /* Function that implements the task. */
									"Periodic_Transmitter Task",              /* Text name for the task. */
									100,      									/* Stack size in words, not bytes. */
									( void * ) 0,    						/* Parameter passed into the task. */
									1,													/* Priority at which the task is created. */
									&Periodic_TransmitterHandler,
									TASK3_PERIOD);       /* Used to pass out the created task's handle. */

	xReturned = xTaskPeriodicCreate(
									Uart_Receiver,                /* Function that implements the task. */
									"Uart_Receiver Task",              /* Text name for the task. */
									100,      									/* Stack size in words, not bytes. */
									( void * ) 0,    						/* Parameter passed into the task. */
									1,													/* Priority at which the task is created. */
									&Uart_ReceiverHandler,
									TASK4_PERIOD);       /* Used to pass out the created task's handle. */
										
	xReturned = xTaskPeriodicCreate(
									Load_1_Simulation,                /* Function that implements the task. */
									"Load_1_Simulation Task",              /* Text name for the task. */
									100,      									/* Stack size in words, not bytes. */
									( void * ) 0,    						/* Parameter passed into the task. */
									1,													/* Priority at which the task is created. */
									&Load_1_SimulationHandler,
									TASK5_PERIOD);       /* Used to pass out the created task's handle. */

	xReturned = xTaskPeriodicCreate(
									Load_2_Simulation,                /* Function that implements the task. */
									"Load_2_Simulation Task",              /* Text name for the task. */
									100,      									/* Stack size in words, not bytes. */
									( void * ) 0,    						/* Parameter passed into the task. */
									1,													/* Priority at which the task is created. */
									&Load_2_SimulationHandler,
									TASK6_PERIOD);       /* Used to pass out the created task's handle. */									
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/

