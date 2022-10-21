/** @file main.c
 * 
 * @brief Third Project for ECE443 
 *
 * @details       
 * Demonstrates the use of FreeRTOS in accordance to the directions given for
 * the third ECE443 project with Dr.J
 *
 * @author
 * Owen Blair
 * @date
 * 09/19/22
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Hardware specific includes. */
#include "CerebotMX7cK.h"
#include "comm.h"
#include "I2C_EEPROM_lib.h"
#include "LCDlib.h"

/* Standard demo includes. */
#include <plib.h>

// I want to have a nicer time with strings
#include "string.h"

/*-----------------------------------------------------------*/

/*
 * Set up the hardware ready to run this demo.
 */
static void prvSetupHardware( void );

// User defined functions
unsigned int read_buttons();
void hw_msDelay(unsigned int);
void formatDisplay(char *);

// Task Prototypes!
static void msHeartbeatTask();
static void EEPROM_Hndlr_tsk();
static void LCD_Hndlr_tsk();

// Stuff for ISR wrappers
void __attribute__( (interrupt(ipl3), vector(_CHANGE_NOTICE_VECTOR))) CN_ISR_Wrapper(void);
void __attribute__( (interrupt(ipl1), vector(_UART1_VECTOR))) UART_ISR_Wrapper(void);

// Semaphore handles!
xSemaphoreHandle CN_semaphore;
xSemaphoreHandle UART_semaphore;

// Make Queue Handle(s)
xQueueHandle OutgoingQueue; // Messages to display on LCD Queue

// Global variables because I'm lazy
unsigned int msgCount = 0; // Count of how many messages there are
unsigned int btnCount = 0; // Count of how many times btn1 is pressed
static char msgBuffer[80] = {'\0'}; // Message buffer

// Init Tracealizer stuff  
#if ( configUSE_TRACE_FACILITY == 1 )
    traceString trace_ms;
    traceString CN_ISR_trace;
    traceString LCD_Handler_trace;
    traceString EEPROM_Handler_trace;
    traceString UART_ISR_trace;
#endif

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
    
    // Tracealizer Def
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTraceEnable(TRC_START); // Initialize and start recording
        trace_ms = xTraceRegisterString("ms heartbeat task");
        CN_ISR_trace = xTraceRegisterString("CN ISR");
        UART_ISR_trace = xTraceRegisterString("UART ISR");
        LCD_Handler_trace = xTraceRegisterString("LCD Handler task");
        EEPROM_Handler_trace = xTraceRegisterString("EEPROM Handler task");
    #endif

/* -----  NO FreeRTOS API calls BEFORE this line!!! ------------*/
    
/* Create the tasks then start the scheduler. */
    
    // Variable to test return of tasks. Forever loop for debug
    BaseType_t xReturned;
    
    // Make the semaphores!
    CN_semaphore = xSemaphoreCreateCounting(5,0);
    UART_semaphore = xSemaphoreCreateBinary();
    
    // Make Queue(s)
    OutgoingQueue = xQueueCreate(5, sizeof(msgBuffer[msgCount])); // Size of 5 80 char str
    if(OutgoingQueue == NULL){
        // Queue failed to create
        for( ;; );
    }
        
    // Create heartbeat task
	xReturned = xTaskCreate(msHeartbeatTask, "1ms Heartbeat LED C ", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
	if(xReturned == NULL){
        // Task failed to create
        for( ;; );
    }
    
    // Create EEPROM_Handler_tsk task
	xReturned = xTaskCreate(EEPROM_Hndlr_tsk, "EEPROM Handler task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
	if(xReturned == NULL){
        // Task failed to create
        for( ;; );
    }
    
    
    // Create LCD_Handler_tsk task
	xReturned = xTaskCreate(LCD_Hndlr_tsk, "LCD Handler task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
	if(xReturned == NULL){
        // Task failed to create
        for( ;; );
    }
    
    
    vTaskStartScheduler();	/*  Finally start the scheduler. */

/* Will only reach here if there is insufficient heap available to start
 *  the scheduler. */
    return 0;
}  /* End of main */

// 1 ms Heartbeat task ---------------------------------------------------------
static void msHeartbeatTask(){
	while(1) {
        
        // Stuff for Tracealizer
        #if(configUSE_TRACE_FACILITY)
            vTracePrint(trace_ms, "ms heartbeat");
        #endif

        // Invert LED C
		LATBINV = LEDC;
        
        // Delay for 1ms heartbeat
		vTaskDelay(1/portTICK_RATE_MS);
	}
}


// task that handles storage of EEPROM -----------------------------------------
static void EEPROM_Hndlr_tsk(){
    
    // List of where 80 char message memory slots in EEPROM
    int msgMemorySlot[5] = {0,80,160,240,320};
    
    // Slave Address of EEPROM
    static unsigned int slaveAddress = 0x50;
    
        
    for( ;; ){
        
        int EEPROMreturn;
        
        //Take UART semaphore
        xSemaphoreTake(UART_semaphore, portMAX_DELAY);
        
        //Disable ISRs
        mU1RXIntEnable(0);
        mCNIntEnable(0);
        
        // Does LED A need to be on or off?
        if(msgCount >= 5){
            LATBCLR = LEDA; // Message buffer full!
        }
        else{
            LATBSET = LEDA; // Room for messages!
        }
        
        // Check if there are messages to retrieve!
        if(msgCount <= 0){
            LATBSET = LEDB;
            
            // Write to the EEPROM
            EEPROMreturn = writeEEPROM(slaveAddress, msgMemorySlot[msgCount], msgBuffer, 80);
        }
        
        // Do tracealizer things
        #if ( configUSE_TRACE_FACILITY == 1 )
            vTracePrint(EEPROM_Handler_trace, "String written to EEPROM");
        #endif

        
        
        
        // Check for EEPROM write error
        if(EEPROMreturn == 0){ // Write OK!
            
            // Write is complete, turn LED A on
            //LATBSET = LEDA;
            
            // Copy message to OutgoingQueue, don't wait for space!
            //      Message count check is in ISR
            xQueueSendToBack(OutgoingQueue, &msgMemorySlot[msgCount], 0);
            
            //Notify used that message was saved
            putsU1("\nYour message of:");
            putsU1(msgBuffer);
            putsU1("Was saved in EEPROM!\n");
            
            // Increment message count
            msgCount++;
            
            // Clean message buffer for another string
            int i;
            for(i=0; i<80; i++){
                msgBuffer[i] = '\0';
            }
        }
        else{
            // Notify user of EEPROM write error
            putsU1("Write EEPROM error\n");
            LCD_puts("\rEEPROM wrt error\r"); // Cut short due to 16 char limit
        }
        
        // Enable interrupts
        mU1RXIntEnable(1);
        mCNIntEnable(1);
    }
}


// task that handles LCD -------------------------------------------------------
static void LCD_Hndlr_tsk(){
    
    // List of slots where 80 char message memory slots in EEPROM are located
    unsigned int msgMemorySlot = 0;
    
    // Message char array
    char lcdMsgBuffer[80] = {'\0'};
    
    // Slave Address of EEPROM
    static unsigned int slaveAddress = 0x50;
    
    for( ;; ){
        
        // Wait for semaphore
		xSemaphoreTake(CN_semaphore, portMAX_DELAY);
        
        //Stuff for tracing! (Tracealizer)
		#if (configUSE_TRACE_FACILITY)
			vTracePrint(LCD_Handler_trace, "LCD Handler task");
		#endif
        
        // Debounce for 20 ms
		vTaskDelay(20/portTICK_PERIOD_MS);
        
        // Check to see if there are messages to show
        if(msgCount > 0){
            
            // Is this a btn1 push?
            if(read_buttons() == 1){
                xQueueReceive(OutgoingQueue, &msgMemorySlot, 0);
                
                // Read from EEPROM using memory slot from queue
                int returnVal = readEEPROM(slaveAddress, msgMemorySlot, lcdMsgBuffer, 80);
                
                // Check to see if read went OK
                if(returnVal == 0){ // OK!
                    
                    // Decrement the message counter
                    msgCount--; // Seems to decrement forever?
                    
                    // Write the message to the LCD
                    formatDisplay(lcdMsgBuffer);
                    //hw_msDelay(1000);
                    
                    // Clean message buffer for another string
                    int i;
                    for(i=0; i<80; i++){
                        msgBuffer[i] = '\0';
                    }
                }
                else{   // READ IS NOT OK
                    putsU1("Read EEPROM error\n");
                    LCD_puts("\rEEPROM Rd error\r"); // Cut, 16 char limit
                }
                /*
                if(msgCount > 0){
                    msgCount--;
                }
                 * */
            }
        }
        
        // Clear CN ISR flag (FOR DEBUG)
        mCNClearIntFlag();

        //Enable the CN ISR
        mCNIntEnable(1);
    }
}


// FORMAT LCD OUTPUT FUNCTION---------------------------------------------------
// I know that 80/16 = 5, so the max number of lines is 5!
void formatDisplay(char *message){
    
    // Counting variable!
    unsigned int i = 0;
    
    unsigned int iPrevLine = 0;
    
    // How high can counting variable go?
    unsigned int iMax = strlen(message); 

    // Start at bottom left!
    //LCD_puts("                ");
    
    // Ensure that the LCD is clear and ready to use
    LCD_clear();
    
    while(i<=iMax){
        
        LCD_putc('\n');
        // Move EOLine to end of current line
        int EOLine = 16+i;
        
        // Count until usable EOL is reached
        for (EOLine; EOLine>i ;EOLine--){
            if(message[EOLine]==' '||EOLine==(iMax-1) ){
                // This loop will decrement EOLine until a space or return
                break;
            }
        }
        
        // save prev starting point
        iPrevLine = i + 1;
        
        // Iterate through and place message string character by character
        for (i; i<=EOLine; i++){ 
            LCD_putc(message[i]);
        }
        
        // Delay 1 second
        LCDDelay(1000);
        LCD_clear();
        
        // write top line
        writeLCD(0,0x80); // Return to top left
        
        for (iPrevLine; iPrevLine<i; iPrevLine++){
            LCD_putc(message[iPrevLine]);
        }
        
    }
    
    // Be blank for 1 second!
    LCDDelay(1000);
    LCD_clear();
    
    
}
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
    x = PORTReadBits(IOPORT_G, BTN1);
    x = x >> 6; //Shift bits over so the return value will be 0,1,2, or 3
    return x;
}


/*hw_msDelay Function Description ******************************************
* SYNTAX:         void hw_msDelay(unsigned int mS);
* DESCRIPTION: This is a millisecond delay function uses the core time
*   to set the base millisecond delay period. Delay periods
*   of zero are permitted. LEDA is toggled each millisecond.
* KEYWORDS:  delay, ms, milliseconds, software delay, core timer
* PARAMETER1: mS - the total number of milliseconds to delay
* RETURN VALUE: None:
* END DESCRIPTION *********************************************************/
void hw_msDelay(unsigned int mS)
{
    unsigned int tWait, tStart;
    tStart = ReadCoreTimer();   //Read core timer count -- SW breakpoint
    tWait = (CORE_MS_TICK_RATE * mS);    //Time to wait
    while((ReadCoreTimer() - tStart) < tWait);  //Empty loop, whit for time
    LATBINV = LEDA;
}


// Change notice ISR handler ---------------------------------------------------
void CN_ISR_Handler( void ){ // Mostly copied from project 2!
    
    // For Tracealizer
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(CN_ISR_trace, "CN ISR Active");
    #endif

    // For Debug
    //LATBINV = LEDB;
    
    portBASE_TYPE xHigherPriotityTaskWoken = pdFALSE;
    
    // Give the CN semaphore! The null is optional, change if issues happen
	xSemaphoreGiveFromISR(CN_semaphore, NULL);
    
    // Clear CN ISR flag (FOR DEBUG)
    //mCNClearIntFlag();
        
    //Disable CN ISR to avoid bouncing
    mCNIntEnable(0);
    
	// Equivalent task yeld thing
    portEND_SWITCHING_ISR(xHigherPriotityTaskWoken);
}


// UART ISR handler ------------------------------------------------------------
void UART_ISR_Handler( void ){
    
    // For Tracealizer
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTracePrint(UART_ISR_trace, "UART ISR Active");
    #endif

    // For Debug
    //LATBINV = LEDA;
    if(getstrU1(msgBuffer, sizeof(msgBuffer))) {
        
        // Turn off LED A until EEPROM has saved its message
        LATBCLR = LEDA;
        
        // New line for putty
        putcU1('\n');
        
        // Are message slots full?
        if(msgCount >= 5){ // Message slots are full, don't give UART semaphore
            putsU1("Message slots full. Press Btn1 to empty out a slot\n");
        }
        else{ // Message slots are not full, give UART semaphore
            xSemaphoreGiveFromISR(UART_semaphore, NULL);
        }
    }
    
    portBASE_TYPE xHigherPriotityTaskWoken = pdFALSE;
    
    // Clear UART ISR flag
    mU1RXClearIntFlag();
    
	// Equivalent task yeld thing
    portEND_SWITCHING_ISR(xHigherPriotityTaskWoken);
}


// Setup hardware function -----------------------------------------------------
static void prvSetupHardware( void )
{
    Cerebot_mx7cK_setup();
    
    /* Set up PmodSTEM LEDs */
    PORTSetPinsDigitalOut(IOPORT_B, SM_LEDS);
    LATBCLR = SM_LEDS;                      /* Clear all SM LED bits */
    //LATBSET = LEDA;                         /* Turn on LEDA */
    
    //LCD w/ PMP init
    LCDInit();
    
    // UART init
    initialize_uart1(19200, ODD_PARITY);
    
    //EEPROM w/ I2C init
    init_EEPROM();
    
	// Btn1 interrupt setup (CN ISR)
	mCNOpen(CN_ON, CN8_ENABLE, 0);
	mCNSetIntPriority(1);
	mCNSetIntSubPriority(0);
	unsigned int dummy = PORTReadBits(IOPORT_G, BTN1);
	mCNClearIntFlag();
    
    // UART interrupt setup (UART ISR)
    mU1SetIntPriority(1);
    mU1SetIntSubPriority(0);
    
    // Enable both interrupts
    mU1RXIntEnable(1);
	mCNIntEnable(1);
            
	INTEnableSystemMultiVectoredInt();
    
    // Notify user that stuff is set up
    LCD_puts("Setup Complete\r");
    putsU1("Setup Complete\n");
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
