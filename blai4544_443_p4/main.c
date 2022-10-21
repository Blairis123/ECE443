/** @file main.c
 * 
 * @brief Fourth Project for ECE443 
 *
 * @details       
 * Demonstrates the use of FreeRTOS in accordance to the directions given for
 * the fourth ECE443 project with Dr.J. This project uses I2C to read data from
 * an IR sensor and determines temperature from IR radiation
 *
 * @author
 * Owen Blair
 * @date
 * 09/30/22
 */

// Kernel includes.
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "message_buffer.h"

// Hardware specific includes.
#include "CerebotMX7cK.h"
#include "comm.h"
#include "I2C_EEPROM_lib.h"
#include "LCDlib.h"

// Standard demo includes.
#include <plib.h>

// I want to have a nicer time with strings
#include "string.h"

// User defined function includes in this file!
#include "main.h"

/*-----------------------------------------------------------*/

// Stuff for ISR wrappers
void __attribute__( (interrupt(ipl2), vector(_CHANGE_NOTICE_VECTOR))) CN_ISR_Wrapper(void);

// Make Queue Handle(s)
xQueueHandle OutgoingQueue; // Messages to display on LCD Queue

// Make message buffer handle
static MessageBufferHandle_t messageBuffer;

// Make task handle to use for notification
static TaskHandle_t IR_Handler_Task_Handle;

// Global variables because I'm lazy
static char charBuffer[80] = {'\0'}; // Message buffer

// Init Tracealizer stuff  
#if ( configUSE_TRACE_FACILITY == 1 )
    traceString CN_Task_trace;
    traceString CN_ISR_trace;
    traceString LCD_Handler_trace;
    traceString IR_Handler_trace;
    traceString blink_trace;
#endif

/* MAIN FUNCTION
 * @brief Main function that creates all needed stuff and starts scheduler
 * 
 * @param None
 * 
 * @return 0, But this shouldn't happen
*/
int main( void )
{
    prvSetupHardware();		/*  Configure hardware */
    
    // Tracealizer Def
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTraceEnable(TRC_START); // Initialize and start recording
        CN_Task_trace = xTraceRegisterString("CN task");
        CN_ISR_trace = xTraceRegisterString("CN ISR");
        LCD_Handler_trace = xTraceRegisterString("LCD Handler task");
        IR_Handler_trace = xTraceRegisterString("Infrared sensor Handler task");
        blink_trace = xTraceRegisterString("3ms heartbeat task");
    #endif

/* -----  NO FreeRTOS API calls BEFORE this line!!! ------------*/
    
/* Create the tasks then start the scheduler. */
    
    // Variable to test return of tasks. Forever loop for debug
    BaseType_t xReturned;
        
    // Make message buffer
    messageBuffer = xMessageBufferCreate( 32 );
    if(messageBuffer == NULL){
        // Queue failed to create
        for( ;; );
    }
        
    // Create heartbeat task (LED A)
	xReturned = xTaskCreate(msHeartbeatTask, "3ms Heartbeat LED C ", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	if(xReturned == NULL){
        // Task failed to create
        for( ;; );
    }
    
    
    // Create IR_Handler_task (LED C for debug)
	xReturned = xTaskCreate(IR_Handler_Task, "Reading from IR sensor", configMINIMAL_STACK_SIZE, NULL, 2, &IR_Handler_Task_Handle);
	if(xReturned == NULL){
        // Task failed to create
        for( ;; );
    }
    
    
    // Create change notice task to set CN flag (LED B for debug)
	xReturned = xTaskCreate(CN_Task, "Set CN ISR flag", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	if(xReturned == NULL){
        // Task failed to create
        for( ;; );
    }
    
    // LCD temperature display task (LED D for debug)
	xReturned = xTaskCreate(LCD_Handler_Task, "LCD temperature display", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	if(xReturned == NULL){
        // Task failed to create
        for( ;; );
    }
    
    vTaskStartScheduler();	/*  Finally start the scheduler. */

/* Will only reach here if there is insufficient heap available to start
 *  the scheduler. */
    return 0;
}  /* End of main */


/* CN_Task
 * 
 * @brief Task that triggers a CN interrupt every 6ms
 * 
 * @param None
 * 
 * @return None
 */
static void CN_Task(){
    while(1){
        // Delay for 6 ms
        vTaskDelay(6/portTICK_RATE_MS);

        // Toggle LED B for debug
        LATBINV = LEDB;

        // Set CN flag
        INTSetFlag(INT_CN);
        
        // Stuff for Tracealizer
        #if(configUSE_TRACE_FACILITY)
            vTracePrint(CN_Task_trace, "CN flag set");
        #endif
    }
}


/* LCD_Handler task!
 * @brief Read from message buffer and display message on LCD (priority 1)
 * 
 * @param None
 * 
 * @return None
 */
static void LCD_Handler_Task(){
    char* displayStringOutPointer[13]; // 11 chars +\n + null
    size_t xBytesRx;
    
    while(1){
        
        // Toggle LED D for debug
        LATBINV = LEDD;
        
        // String to show on LCD
        char* strOut[13]; // 11 chars +\n + null
        
        // Read from message buffer
        xBytesRx = xMessageBufferReceive( messageBuffer, ( void * ) strOut, sizeof(strOut), 0 );
        if(xBytesRx){
            // Display string on LCD using 341 LCD lib if Rx is working
            LCD_puts(strOut);
        }
        else{
            //End up in forever loop if Rx didn't work
            //for( ;; );
        }
        
        // Stuff for Tracealizer
        #if(configUSE_TRACE_FACILITY)
            vTracePrint(LCD_Handler_trace, "Message written to LCD");
        #endif
        
        // Yeld for other tasks
        taskYIELD();
    }
}


/* IR_Handler Task
 * 
 * @brief Task that is inblocked by CN, reads from IR sensor and formats
 *          a string that is saved to a local buffer then into a message buffer
 * 
 * @param None
 * 
 * @return None
 */
static void IR_Handler_Task(){
    const unsigned int slaveAddr = 0x5A;
    const unsigned int mem_addr = 0x07;
    char* msgFromIR[3];
    
    // return from message buffer send call
    size_t xBytesSent;
    
    char* strOut[13]; // 11 chars +\n + null
    while(1){
        // Rx task notification
        ulTaskNotifyTake(TRUE, portMAX_DELAY);

        // Toggle LED C for debug
        LATBINV = LEDC;

        // Read from IR sensor
            // LSB read
        
        /*
         * This is currently not working!!!!!! I will pass a predetermined temp
         * into the buffer so I can finish the rest of the project
         * 
         * I will re-introduce this if everything else is worked out
         */
        //I2C_IR_Read( slaveAddr, mem_addr, *msgFromIR, 3);

        // Convert to F
        //LCD_puts(msgFromIR);
        float tempTemprature = 172.5956; // Float used for temporary testing
        
        // Format string & save to local buffer & put a pointer to local buffer
        //      into the message buffer
        sprintf(strOut, "Temp=%3.1f\n", tempTemprature);
        
        // Make a pointer to local buffer that can be given to API
        char * ptr;
        ptr = &strOut[0];
        
            // Debug to see what string looks like
        //LCD_puts(tempratureOut);
        
        
        xBytesSent = xMessageBufferSend( messageBuffer, ( void * ) ptr, sizeof(ptr), 0 );
        
        // Did message buffer send something?
        if(xBytesSent == NULL){
            // End up in forever loop if sending didn't happen
            //for( ;; );
        }
        // What does the string look like?
        //LCD_puts(strOut);
        
        // Stuff for Tracealizer
        #if(configUSE_TRACE_FACILITY)
            vTracePrint(IR_Handler_trace, "IR sensor read & message to buffer");
        #endif

        // Yeld for other tasks
        taskYIELD();
    }
}


/*!     HEARTBEAT TASK
 * @brief 3 millisecond heartbeat toggle of LED A
 *
 * @param[in] No input
 * 
 * @return None
 */
static void msHeartbeatTask(){
	while(1) {
        
        // Stuff for Tracealizer
        #if(configUSE_TRACE_FACILITY)
            vTracePrint(blink_trace, "3ms heartbeat");
        #endif

        // Invert LED A
		LATBINV = LEDA;
        
        // Delay for 3ms heartbeat
		vTaskDelay(3/portTICK_RATE_MS);
	}
}


/*!     READ BUTONS function!
 * @brief Function to read buttons 1 and 2
 *
 * @param[in] No input
 *
 * @return Unsigned int indicating button 1 and 2's state
 *      Return --> [Btn 2 bit, Btn 1 bit]
 *      btn 1 on, btn 2 off = 01 (1)
 *      btn 1 off, btn 2 on = 10 (2)
 *      btn 1 on, btn 2 on  = 11 (3)
 */
unsigned int read_buttons(void){
    unsigned int x = 0;
    x = PORTReadBits(IOPORT_G, BTN1);
    x = x >> 6; //Shift bits over so the return value will be 0,1,2, or 3
    return x;
}


/*!     HARDWARE ASSISTED DELAY function
 * @brief Hardware assisted delay function from ECE341 w/ Dr.J
 *
 * @param[in] mS,   Unsigned int of how many milliseconds to wait
 *
 * @return None
 */
void hw_msDelay(unsigned int mS)
{
    unsigned int tWait, tStart;
    tStart = ReadCoreTimer();   //Read core timer count -- SW breakpoint
    tWait = (CORE_MS_TICK_RATE * mS);    //Time to wait
    while((ReadCoreTimer() - tStart) < tWait);  //Empty loop, whit for time
    LATBINV = LEDA;
}


/*!     CHANGE NOTICE ISR
 * @brief Change notice ISR that gives direct task notification
 *
 * @param[in] No input
 *
 * @return None
 */
void CN_ISR_Handler( void ){ // Mostly copied from project 2!
    
    // For Tracealizer
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(CN_ISR_trace, "CN ISR Active");
    #endif
    
    portBASE_TYPE xHigherPriotityTaskWoken = pdFALSE;
    
    // Give the task notification!
    vTaskNotifyGiveFromISR(IR_Handler_Task_Handle, &xHigherPriotityTaskWoken);
    
    //xTaskNotifyGive(IR_Handler_Task_Handle);
    
    // Clear CN ISR flag
    mCNClearIntFlag();
    
	// Equivalent task yeld thing
    portEND_SWITCHING_ISR(xHigherPriotityTaskWoken);
    //portYIELD_FROM_ISR(xHigherPriotityTaskWoken);
}


/*!
 * @brief Function to set up Cerabot hardware
 *
 * @param[in] No input
 *
 * @return None
 */
static void prvSetupHardware( void )
{
    Cerebot_mx7cK_setup();
    
    /* Set up PmodSTEM LEDs */
    PORTSetPinsDigitalOut(IOPORT_B, SM_LEDS);
    LATBCLR = SM_LEDS;                      /* Clear all SM LED bits */
    //LATBSET = LEDA;                         /* Turn on LEDA */
    
    //LCD w/ PMP init
    LCDInit();
    
    // I2C init
    init_EEPROM();
        
	// Btn1 interrupt setup (CN ISR)
	mCNOpen(CN_ON, CN8_ENABLE, 0);
	mCNSetIntPriority(1);
	mCNSetIntSubPriority(0);
	unsigned int dummy = PORTReadBits(IOPORT_G, BTN1);
	mCNClearIntFlag();
    
    // Enable interrupts
	mCNIntEnable(1);
            
	INTEnableSystemMultiVectoredInt();
    
    // Notify user that stuff is set up
    LCD_puts("Setup Complete\r");
}

/*-----------------------------------------------------------*/

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

/*** end of file ***/