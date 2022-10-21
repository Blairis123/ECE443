/** @file main.c
 * 
 * @brief Main program file for ECE443 Project 2 using FreeRTOS
 *
 * @details       
 * This is for the assigned project 2 for ECE443 with Dr.J. Design a C.N.
 *  interrupt that gives a semaphore to unblock a task that is a pushbutton for
 *  LED C. Input is from button 1, 1 ms 'heartbeat' on LED B, an Idle hook
 *  that shows btn 1 state on LED A, and LED D is on when the C part of the
 *  ISR is active
 *
 * @author
 * Owen Blair
 * @date
 * 12 September 2022
 */


// Kernel includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Standard Lib includes
#include <plib.h>

// Hardware specific includes
#include "CerebotMX7cK.h" 

//  Stuff for Tracealizer
#if ( configUSE_TRACE_FACILITY == 1 )
    traceString trace_handler;
    traceString trace_ms;
	traceString trace_IdleHook;
    traceString trace_ledc;
#endif


/* User Defined Function Prototypes  */
static void prvSetupHardware( void );


/* User Defined Task Prototypes  */
static void msHeartbeatTask();
static void LEDC_Handler();

/* Semaphore handle creation */
xSemaphoreHandle led_semaphore;
 
// Global variables because I'm lazy
static unsigned int prevA;
unsigned int LEDC_COUNT = 0;

// Stuff for ISR wrapper
void __attribute__( (interrupt(ipl1), vector(_CHANGE_NOTICE_VECTOR))) ISR_Wrapper(void);

int main( void )
{
    prvSetupHardware();		/*  Configure hardware */
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTraceEnable(TRC_START); // Initialize and start recording
        trace_handler = xTraceRegisterString("LED C handler task");
        trace_ms = xTraceRegisterString("ms heartbeat task");
		trace_IdleHook = xTraceRegisterString("Idle Hook");
        trace_ledc = xTraceRegisterString("LED C 2x Trace");
    #endif
    
    BaseType_t xReturned; //Empty task return var.
        
	xReturned = xTaskCreate(msHeartbeatTask, "1ms Heartbeat LED B ", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
	if(xReturned == NULL){
        // Task failed to create
        for( ;; );
    }


    
	xReturned = xTaskCreate(LEDC_Handler, "Push btn LED C Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

	if(xReturned == NULL){ // Did it go OK?
        // Task failed to create
        for( ;; );
    }
    

	// Make a semaphore!
	led_semaphore = xSemaphoreCreateBinary();
    
	if (led_semaphore == NULL){ // Did it go OK?
        // Semaphore failed to create
        for( ;; );
    }

	/* Create the tasks then start the scheduler. */

    vTaskStartScheduler();	/*  Finally start the scheduler. */

	// Fingers crossed, the program will never reach here!
    return 0;
}   /* End of main */

static void prvSetupHardware( void )
{
    Cerebot_mx7cK_setup();
    
    // Digital I/O setup
    PORTSetPinsDigitalOut(IOPORT_B, SM_LEDS);
    LATBSET = SM_LEDS;
    LATBCLR = SM_LEDS;                      /* Clear all SM LED bits */

	PORTSetPinsDigitalIn(IOPORT_G, BTN1);
    
	/* Enable the Interrupt for BTN1 */
    
	mCNOpen(CN_ON, CN8_ENABLE, 0);
	mCNSetIntPriority(1);
	mCNSetIntSubPriority(0);
	unsigned int dummy = PORTReadBits(IOPORT_G, BTN1);
	mCNClearIntFlag();
	mCNIntEnable(1);
    
            
	INTEnableSystemMultiVectoredInt();
}
/*-----------------------------------------------------------*/

void CN_ISR_Handler( void ){
    
    // Turn LED D on
	LATBSET = LEDD;
    portBASE_TYPE xHigherPriotityTaskWoken = pdFALSE;
    
    // Give the LED C semaphore! The null is optional, change if issues happen
	xSemaphoreGiveFromISR(led_semaphore, NULL);
    
    //Disable the CN ISR to avoid bouncing
    mCNIntEnable(0);
	
	// 
    portEND_SWITCHING_ISR(xHigherPriotityTaskWoken);
    
    // Turn LED D off
    LATBCLR = LEDD;
}

static void msHeartbeatTask() {
	while(1) {
        #if(configUSE_TRACE_FACILITY)
            vTracePrint(trace_IdleHook, "LEA = Btn1");
        #endif

        // Invert LED B
		LATBINV = LEDB;
        
        // Stuff to count how many times LEDC has run
        if((PORTB & LEDC) == LEDC) {
			LEDC_COUNT ++;
            
            // Has LED C toggled 2x?
			if(LEDC_COUNT >= 2) {
                
                // Make a trace!
				#if(configUSE_TRACE_FACILITY)
            		vTracePrint(trace_ledc, "LEDC on twice");
        		#endif

                // Reset LED C count to start again
				LEDC_COUNT = 0;
			}
		}
        
        // Delay for 1ms heartbeat
		vTaskDelay(1/portTICK_RATE_MS);
	}
}

static void LEDC_Handler() {
	while(1) {
        
        //Stuff for tracing! (Tracealizer)
		#if (configUSE_TRACE_FACILITY)
			vTracePrint(trace_handler, "LED C Handler task");
		#endif
        
        // Wait for semaphore
		xSemaphoreTake(led_semaphore, portMAX_DELAY);
        
        // Read the button and toggle if needed
        if( PORTG & BTN1){
            LATBINV = LEDC;
        }
        
        // Debounce for 20 ms
		vTaskDelay(20/portTICK_PERIOD_MS);
        
        // Clear CN ISR
        mCNClearIntFlag();
        
        // Enable the CN ISR again
        mCNIntEnable(1);
        
	}
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
    while(1){
        // Tracealyzer tracking
        #if(configUSE_TRACE_FACILITY)
            vTracePrint(trace_ms, "Toggle LEDB");
        #endif


        // Look at port G
        int status = PORTG;

        // Is button 1 on?
        if((status & BTN1) == BTN1) {
            LATBSET = LEDA;
        }
        else {
            LATBCLR = LEDA;
        }
    }
    
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