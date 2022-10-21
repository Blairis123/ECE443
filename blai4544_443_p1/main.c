/** @file main.c
 * 
 * @brief Main program file for Reference Design 1 using FreeRTOS V2021 
 *
 * @details       
 * Demonstrates the use of FreeRTOS, Doxygen, Git, and Tracealyzer.
 * Built on FreeRTOS V202104.00, design had two sending tasks that read
 * buttons a and b. Then another task that uses that data and toggles
 * LED a and b only when the button is released.
 *
 * @author
 * Owen Blair
 * @date
 * 9/6/2022
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"

/* Hardware specific includes. */
#include "CerebotMX7cK.h"

/* Standard includes. */
#include <plib.h>
#include <stdio.h>

/* User created code includes + function prototypes + structure */
#include "main.h"

/* TraceAlyzer config  */
#if ( configUSE_TRACE_FACILITY == 1 )
    traceString trace_handler;
    traceString trace_ms;
	traceString trace_ledc_on;
#endif

// Make Queue Handles
xQueueHandle xQueA; // Btn 1 Queue handle
xQueueHandle xQueB; // Btn 2 Queue handle

// Inputs to remember previous btn inputs
static int prevA, prevB;

/* main Function Description ***************************************
 * SYNTAX:		int main( void );
 * KEYWORDS:		Initialize, create, tasks, scheduler
 * DESCRIPTION:         This is a typical RTOS set up function. Hardware is
 * 			initialized, tasks are created, and the scheduler is
 * 			started.
 * PARAMETERS:		None
 * RETURN VALUE:	Exit code - used for error handling
 * NOTES:		All three buttons are polled using the same code
 *                      for reading the buttons.
 * END DESCRIPTION *****************************************************/
int main( void )
{
    prvSetupHardware();		/*  Configure hardware */
    
/* -----  NO FreeRTOS API calls BEFORE this line!!! ------------*/
    
    // Variable to save task return value
    //BaseType_t xReturned;
    
    /* Create the tasks defined within this file. */
    xTaskCreate( StartTsk, "Starting Task", 200, NULL, 3, NULL );
    
    /*
    // Check if task created, for Debug!
    if( xReturned != pdPASS){
        for( ;; );
        // Empty for a breakpoint for debug
    }
    */
    
    vTaskStartScheduler();	/*  Finally start the scheduler. */

/* Will only reach here if there is insufficient heap available to start
 *  the scheduler. */
    return 0;
}  /* End of main */


/* prvTestTask1 Function Description *****************************************
 * SYNTAX:          static void prvTestTask1( void *pvParameters );
 * KEYWORDS:        RTOS, Task
 * DESCRIPTION:     If LEDA is not lit, all LEDs are turned off and LEDA is
 *                  turned on. Increments a counter each time the task is
 *                  resumed.
 * PARAMETER 1:     void pointer - data of unspecified data type sent from
 *                  RTOS scheduler
 * RETURN VALUE:    None (There is no returning from this function)
 * NOTES:           LEDA is switched on and LEDB switched off if LEDA was
 *                  detected as off.
 * END DESCRIPTION ************************************************************/
static void StartTsk( void *pvParameters ) {       

    // Start making Queues!
    xQueA = xQueueCreate (5, sizeof(unsigned int));
    xQueB = xQueueCreate (5, sizeof(unsigned int));
    
    // Only start making tasks if Queues are OK
    if( xQueA != NULL && xQueB != NULL){
        
        // Create create sender tasks
        xTaskCreate(sendBtn, (signed char *) "BtnA", 240, ( void * ) 'A', 1, NULL);
        xTaskCreate(sendBtn, (signed char *) "BtnB", 240, ( void * ) 'B', 1, NULL);

        // Create toggleLED Task
        xTaskCreate(toggleLED, (signed char *) "ToggleLED", 240, NULL, 2, NULL);
    }
    

    for( ;; ){// Run task in forever loop. Nothing could stop this task! NO FORSHADOWING
        
    // Commit Seppuku
    vTaskDelete( NULL );
    
    }
}  /* End of prvTestTask1 */


// Task for figuring out what to do with LED A & B
void toggleLED( void *btnParameters){
    
    // Variables used in Task
    unsigned int btnA, btnB; // Current btn inputs
    
    portBASE_TYPE xStatus;   // Store status of Queues
    
    // Variable for ticks to wait for de-bounce
    const portTickType xTicksToWait = 20 / portTICK_RATE_MS; // How many ms in ticks?
    
    
    for( ;; ){
        
        // Wait for de-bounce
        vTaskDelay( xTicksToWait );
        
        // Get the button status from queue A
        xStatus = xQueueReceive( xQueA, &btnA, xTicksToWait );
        
        // Get the button status from queue B
        xStatus = xQueueReceive( xQueB, &btnB, xTicksToWait );
        
        // Toggle the LEDs if needed!
            //  LED A
        if(prevA == 0 && btnA == 1){
            LATBINV = LEDA;
        }
            // LED B
        if(prevB == 0 && btnB == 1){
            LATBINV = LEDB;
        }
        
        // Move current values into previous variables
        prevA = btnA;
        prevB = btnB;
        
        // Let another task go
        taskYIELD();
    }
    
    // Commit Seppuku
    vTaskDelete( NULL );
}

  
// Task for sending button information to the queue---------------------------
void sendBtn( void *btnIn ){
    
    // Variables used in Task
    const portTickType xTicksToWait = 20 / CORE_MS_TICK_RATE;
    portBASE_TYPE xStatus;
    unsigned int btnValue;
    unsigned int btnOut;
    
    // Recasting passed in parameter!
    char btnChar = ( char ) btnIn;
    
    // Start task forever loop!
    for( ;; ){
        
        // Use for debuging
        //LATBINV = LEDA;//  Toggle Led A
        
        // Read Buttons!
        unsigned int btnValue = read_buttons();
        if(btnChar == 'A'){
            switch(btnValue){
                case 0:
                    btnOut = 0;
                    break;
                case 1:
                    btnOut = 1;
                    break;
                case 2:
                    btnOut = 0;
                    break;
                case 3:
                    btnOut = 1;
                    break;
            }
            
            // Send data to queue
            xStatus = xQueueSendToBack(xQueA, &btnOut, 0);
        }
        else if(btnChar == 'B'){
            switch(btnValue){
                case 0:
                    btnOut = 0;
                    break;
                case 1:
                    btnOut = 0;
                    break;
                case 2:
                    btnOut = 1;
                    break;
                case 3:
                    btnOut = 1;
                    break;
            }
            
            // Send data to queue
            xStatus = xQueueSendToBack(xQueB, &btnOut, 0);
        }
        
		// Let another task go
        taskYIELD();
    }
}


// Hardware Setup--------------------------------------------------------------
void prvSetupHardware( void )
{
    Cerebot_mx7cK_setup();
    
    /* Set up PmodSTEM LEDs */
    PORTSetPinsDigitalOut(IOPORT_B, SM_LEDS);
    LATBCLR = SM_LEDS;                      /* Clear all SM LED bits */
    
/* Enable multi-vector interrupts */
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);  /* Do only once */
    INTEnableInterrupts();   /*Do as needed for global interrupt control */
    portDISABLE_INTERRUPTS();
    

}

/*-----------------------------------------------------------*/

/* read_buttons Function Description *****************************************
 * SYNTAX:          int read_buttons(void);
 * KEYWORDS:        button, read, inputs
 * DESCRIPTION:     Reads the status of the input buttons.  Button status is
 *                  reported for button bit positions only. All other bits in
 *                  the returned value are set to zero as shown below:
 *
 *      Port G Bit position [15|14|13|12|11|10| 9| 8| 7| 6| 5| 4| 3| 2| 1| 0]
 *      Port G Bit value    [ 0| 0| 0| 0| 0| 0| 0| 0|B2|B1| 0| 0| 0| 0| 0| 0]
 *
 *      B1 will be 1 if BTN1 is pressed otherwise B1 will be zero
 *      B2 will be 1 if BTN2 is pressed otherwise B2 will be zero
 * END DESCRIPTION ************************************************************/
unsigned int read_buttons(void)
{
    unsigned int x = 0;
    x = PORTReadBits(IOPORT_G, BTN1 | BTN2);
    x = x >> 6; //Shift bits over so the return value will be 0,1,2, or 3
    return x;
}

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time task stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook	function is 
	called if a task stack overflow is detected.  Note the system/interrupt
	stack is not checked. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}
/*-----------------------------------------------------------*/

void _general_exception_handler( unsigned long ulCause, unsigned long ulStatus )
{
	/* This overrides the definition provided by the kernel.  Other exceptions 
	should be handled here. */
	for( ;; );
}
/*-----------------------------------------------------------*/


void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
volatile unsigned long ul = 0;

	( void ) pcFile;
	( void ) ulLine;

	__asm volatile( "di" );
	{
		/* Set ul to a non-zero value using the debugger to step out of this
		function. */
		while( ul == 0 )
		{
			portNOP();
		}
	}
	__asm volatile( "ei" );
}