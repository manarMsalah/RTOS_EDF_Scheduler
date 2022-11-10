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
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "queue.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )
	
/* The given period for every task. */	
#define BUTTON_1_MONITOR_TASK_PERIOD         50
#define BUTTON_2_MONITOR_TASK_PERIOD         50
#define PERIODIC_TRANSMITTER_TASK_PERIOD     100
#define UART_RECEIVER_TASK_PERIOD            20
#define LOAD_1_SIMULATION_TASK_PERIOD        10
#define LOAD_2_SIMULATION_TASK_PERIOD        100

/* The created queue's handle. */
QueueHandle_t Button1MonitorQueueHandle = NULL;
QueueHandle_t Button2MonitorQueueHandle = NULL;
QueueHandle_t PeriodicTransmitterQueueHandle = NULL;

/* The created task's handle. */
TaskHandle_t Button1MonitorTaskHandle = NULL;
TaskHandle_t Button2MonitorTaskHandle = NULL;
TaskHandle_t PeriodicTransmitterTaskHandle = NULL;
TaskHandle_t UartReceiverTaskHandle = NULL;
TaskHandle_t Load1SimulationTaskHandle = NULL;
TaskHandle_t Load2SimulationTaskHandle = NULL;

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );

void Button_1_Monitor( void * pvParameters );
void Button_2_Monitor (void *pvParameters);
void Periodic_Transmitter (void *pvParameters);
void Uart_Receiver (void *pvParameters);
void Load_1_Simulation (void *pvParameters);
void Load_2_Simulation (void *pvParameters);
/*-----------------------------------------------------------*/


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	
	/* Queues creation. */
	Button1MonitorQueueHandle = xQueueCreate( 1,sizeof(char*) );
	Button2MonitorQueueHandle = xQueueCreate( 1,sizeof(char*) );
	PeriodicTransmitterQueueHandle = xQueueCreate( 50,sizeof(char*) );
	
	
  /* Create Tasks here */
	
	xTaskPeriodicCreate(
                    Button_1_Monitor,       /* Function that implements the task. */
                    "Button 1 Monitor",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Button1MonitorTaskHandle,/* Used to pass out the created task's handle. */
										BUTTON_1_MONITOR_TASK_PERIOD );/* Task's period. */
										
										
	xTaskPeriodicCreate(
                    Button_2_Monitor,       /* Function that implements the task. */
                    "Button_2 Monitor",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Button2MonitorTaskHandle,/* Used to pass out the created task's handle. */
										BUTTON_2_MONITOR_TASK_PERIOD );/* Task's period. */
										
										
	xTaskPeriodicCreate(
                    Periodic_Transmitter,       /* Function that implements the task. */
                    "Periodic Transmitter",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &PeriodicTransmitterTaskHandle,/* Used to pass out the created task's handle. */
										PERIODIC_TRANSMITTER_TASK_PERIOD );/* Task's period. */		
										
										
	xTaskPeriodicCreate(
                    Uart_Receiver,       /* Function that implements the task. */
                    "Uart Receiver",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &UartReceiverTaskHandle,/* Used to pass out the created task's handle. */
										UART_RECEIVER_TASK_PERIOD );/* Task's period. */										
										
										
									
	xTaskPeriodicCreate(
                    Load_1_Simulation,       /* Function that implements the task. */
                    "Load 1 Simulation",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Load1SimulationTaskHandle,/* Used to pass out the created task's handle. */
										LOAD_1_SIMULATION_TASK_PERIOD );/* Task's period. */

									

	xTaskPeriodicCreate(
                    Load_2_Simulation,       /* Function that implements the task. */
                    "Load 2 Simulation",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Load2SimulationTaskHandle,/* Used to pass out the created task's handle. */
										LOAD_2_SIMULATION_TASK_PERIOD );/* Task's period. */							
										

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
/*********************************************************T.A.S.K.S***********************************************************************/

void Button_1_Monitor( void * pvParameters )
{
	 TickType_t xLastWakeTimeButton1Monitor;
   const TickType_t xFrequency = BUTTON_1_MONITOR_TASK_PERIOD; 
	 pinState_t button1PreviousState = PIN_IS_HIGH;
	 pinState_t button1CurrentState;
	 char txMonitor1Message;
	 char risingCounter = 0, fallingCounter = 0, levelCounter = 0;

   /* Initialise the xLastWakeTime variable with the current time. */
   xLastWakeTimeButton1Monitor = xTaskGetTickCount(); 

    for( ;; )
    {
        /* Task code goes here. */
			
			button1CurrentState = GPIO_read(PORT_0, PIN1);
			
			if( button1CurrentState == PIN_IS_HIGH && button1PreviousState == PIN_IS_LOW )
			{
				risingCounter++;
				if(risingCounter == 2)/* Because of debouncing. */
				{
		      txMonitor1Message= 'R';
				  xQueueOverwrite( Button1MonitorQueueHandle , &txMonitor1Message );
					risingCounter = 0;
					button1PreviousState = button1CurrentState;
				}
			
			}
			else if( button1CurrentState == PIN_IS_LOW && button1PreviousState == PIN_IS_HIGH )
			{
				fallingCounter++;
				if(fallingCounter == 2)
				{
				  txMonitor1Message= 'F';
				  xQueueOverwrite( Button1MonitorQueueHandle , &txMonitor1Message );
					fallingCounter = 0;
					button1PreviousState = button1CurrentState;
				}
			}
			else /* To make the WCET = BCET. */
			{
				levelCounter++;
				if(levelCounter == 1)
				{
				  txMonitor1Message= 'L';
				  xQueueOverwrite( Button1MonitorQueueHandle , &txMonitor1Message );
					levelCounter = 0;
					button1PreviousState = button1CurrentState;
				}
			}
				
			GPIO_write(PORT_0, PIN3, PIN_IS_LOW);
			/* Wait for the next cycle. */
      vTaskDelayUntil( &xLastWakeTimeButton1Monitor, xFrequency );
			GPIO_write(PORT_0, PIN3, PIN_IS_HIGH);
		
    }	
}

/*********************************************************************************************************************************/

void Button_2_Monitor( void * pvParameters )
{
	 TickType_t xLastWakeTimeButton2Monitor;
   const TickType_t xFrequency = BUTTON_2_MONITOR_TASK_PERIOD; 
	 pinState_t button2PreviousState = PIN_IS_HIGH;
	 pinState_t button2CurrentState;
	 char txMonitor2Message;
   char risingCounter = 0, fallingCounter = 0, levelCounter = 0;

   /* Initialise the xLastWakeTime variable with the current time. */
   xLastWakeTimeButton2Monitor = xTaskGetTickCount(); 

    for( ;; )
    {
        /* Task code goes here. */
			
			button2CurrentState = GPIO_read(PORT_0, PIN2);
			
			if( button2CurrentState == PIN_IS_HIGH && button2PreviousState == PIN_IS_LOW )
			{
				risingCounter++;
				if(risingCounter == 2)/* Because of debouncing. */
				{
		      txMonitor2Message= 'R';
				  xQueueOverwrite( Button2MonitorQueueHandle , &txMonitor2Message );
					risingCounter = 0;
					button2PreviousState = button2CurrentState;
				}
			
			}
			else if( button2CurrentState == PIN_IS_LOW && button2PreviousState == PIN_IS_HIGH )
			{
				fallingCounter++;
				if(fallingCounter == 2)
				{
				  txMonitor2Message= 'F';
				  xQueueOverwrite( Button2MonitorQueueHandle , &txMonitor2Message );
					fallingCounter = 0;
					button2PreviousState = button2CurrentState;
				}
			}
			else /* To make the WCET = BCET. */
			{
				levelCounter++;
				if(levelCounter == 1)
				{
				  txMonitor2Message= 'L';
				  xQueueOverwrite( Button2MonitorQueueHandle , &txMonitor2Message );
					levelCounter = 0;
					button2PreviousState = button2CurrentState;
				}
			}	
						
			GPIO_write(PORT_0, PIN4, PIN_IS_LOW);
			/* Wait for the next cycle. */
      vTaskDelayUntil( &xLastWakeTimeButton2Monitor, xFrequency );
			GPIO_write(PORT_0, PIN4, PIN_IS_HIGH);
			
    }
}
/********************************************************************************************************************************/
void Periodic_Transmitter( void * pvParameters )
{
  TickType_t xLastWakeTimePeriodicTransmitter;
  const TickType_t xFrequency = PERIODIC_TRANSMITTER_TASK_PERIOD; 
	char txString[]= "\nImplemented EDF Scheduler successfully";
	char i;
	char strLength= strlen(txString);
	
  /* Initialise the xLastWakeTime variable with the current time. */
  xLastWakeTimePeriodicTransmitter = xTaskGetTickCount();

    for( ;; )
    {
        /* Task code goes here. */
			
			for(i=0; i< strLength; i++)
			{
				xQueueSend( PeriodicTransmitterQueueHandle, (&(txString[0]))+i, ( TickType_t )0 );
			}
			
		  GPIO_write(PORT_0, PIN5, PIN_IS_LOW);
			/* Wait for the next cycle. */
		  vTaskDelayUntil( &xLastWakeTimePeriodicTransmitter, xFrequency );
			GPIO_write(PORT_0, PIN5, PIN_IS_HIGH);
		
    }
}

/*******************************************************************************************************************************/
void Uart_Receiver( void * pvParameters )
{
  TickType_t xLastWakeTimeUartReceiver;	
  const TickType_t xFrequency = UART_RECEIVER_TASK_PERIOD; 
	char RxMonitor1Message, RxMonitor2Message;
	char* edgeMessage; 
	char RxString[50];
	int i, numOfCycles = 2400;/* 2400= 12000*0.2 and 0.2 is the excution time of receiving the string from periodic transmitter task.  */
	
   /* Initialise the xLastWakeTime variable with the current time. */
  xLastWakeTimeUartReceiver = xTaskGetTickCount();

    for( ;; )
    {
        /* Task code goes here. */
			
			/* Receiving from Periodic Transmitter Task. */
			if(xQueueReceive( PeriodicTransmitterQueueHandle, &( RxString[0] ), ( TickType_t ) 0 ))
			{	
				i=1;
			  while(RxString[i-1]!= '\0')
				{			
					xQueueReceive( PeriodicTransmitterQueueHandle, ( RxString+i ), ( TickType_t ) 0 );
					i++;
				}
				vSerialPutString( (signed char *)RxString, strlen(RxString) );
			}
			else
			{
				/* Just for making the BCET nearest thing to WCET. */
				for(i=0; i < numOfCycles; i++);		
			}
			
			/* Receiving from Button 1 Monitor Task. */
			if(xQueueReceive( Button1MonitorQueueHandle, &( RxMonitor1Message ), ( TickType_t ) 0 ))
			{
			  if(RxMonitor1Message == 'R')
				{
				  edgeMessage= "\n\n                                 Rising edge from button 1\n";
					vSerialPutString( (signed char *)edgeMessage, strlen(edgeMessage) );			
				}
				else if(RxMonitor1Message == 'F')
				{
				  edgeMessage= "\n\n                                Falling edge from button 1\n";
					vSerialPutString( (signed char *)edgeMessage, strlen(edgeMessage) );			
				}
        else if(RxMonitor1Message == 'L')
				{
				  edgeMessage= "                                                                ";
					vSerialPutString( (signed char *)edgeMessage, strlen(edgeMessage) );
				}			
			}

      /* Receiving from Button 2 Monitor Task. */			
			else if(xQueueReceive( Button2MonitorQueueHandle, &( RxMonitor2Message ), ( TickType_t ) 0 ))
			{
				if(RxMonitor2Message == 'R')
				{
				  edgeMessage= "\n\n                                 Rising edge from button 2\n";
					vSerialPutString( (signed char *)edgeMessage, strlen(edgeMessage) );	
				}
				else if(RxMonitor2Message == 'F')
				{
				  edgeMessage= "\n\n                                Falling edge from button 2\n";
					vSerialPutString( (signed char *)edgeMessage, strlen(edgeMessage) );	
				}
				else if(RxMonitor2Message == 'L')
				{
				  edgeMessage= "                                                                 ";
					vSerialPutString( (signed char *)edgeMessage, strlen(edgeMessage) );	
				}	
				
			}
			else /* Just for making the BCET nearest thing to WCET. */
			{
				edgeMessage= "                                                                 ";
				vSerialPutString( (signed char *)edgeMessage, strlen(edgeMessage) );
			}

			
			
			GPIO_write(PORT_0, PIN6, PIN_IS_LOW);
      /* Wait for the next cycle. */
      vTaskDelayUntil( &xLastWakeTimeUartReceiver, xFrequency );
			GPIO_write(PORT_0, PIN6, PIN_IS_HIGH);
			
    }
}
/**********************************************************************************************************************************/
void Load_1_Simulation( void * pvParameters )
{
	
  TickType_t xLastWakeTimeLoad1Simulation;
  const TickType_t xFrequency = LOAD_1_SIMULATION_TASK_PERIOD; 
	int i;
	int numOfCycles = 12000*5; /* 12000 = 12.0MHz * 1000 */

  /* Initialise the xLastWakeTime variable with the current time. */
  xLastWakeTimeLoad1Simulation = xTaskGetTickCount();

    for( ;; )
    {
        /* Task code goes here. */
			
			for(i=0; i< numOfCycles; i++); /* Excution time = 5 ms. */
			
			GPIO_write(PORT_0, PIN7, PIN_IS_LOW);
      /* Wait for the next cycle. */
		  vTaskDelayUntil( &xLastWakeTimeLoad1Simulation, xFrequency );
			GPIO_write(PORT_0, PIN7, PIN_IS_HIGH);

    }
}
/*********************************************************************************************************************************/
void Load_2_Simulation( void * pvParameters )
{
   TickType_t xLastWakeTimeLoad2Simulation;
   const TickType_t xFrequency = LOAD_2_SIMULATION_TASK_PERIOD; 
   
	 int i;
	 int numOfCycles = 12000*12;

   /* Initialise the xLastWakeTime variable with the current time. */
  xLastWakeTimeLoad2Simulation = xTaskGetTickCount();

    for( ;; )
    {
        /* Task code goes here. */
			
			for(i=0; i< numOfCycles; i++); /* Excution time = 12 ms. */
					
			GPIO_write(PORT_0, PIN8, PIN_IS_LOW);
			/* Wait for the next cycle. */
		  vTaskDelayUntil( &xLastWakeTimeLoad2Simulation, xFrequency );
			GPIO_write(PORT_0, PIN8, PIN_IS_HIGH);

    }
		
}
/**********************************************************************************************************************************/
void vApplicationTickHook(void)
{
	GPIO_write(PORT_0, PIN0, PIN_IS_HIGH);
	GPIO_write(PORT_0, PIN0, PIN_IS_LOW);
}
/**********************************************************************************************************************************/