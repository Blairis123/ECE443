/** @file main.c
 * 
 * @brief Main program file for Reference Design 1 using FreeRTOS V2021 
 *
 * @details       
 * Demonstrates the use of FreeRTOS, Doxygen, Git, and Tracealyzer.
 * Built on FreeRTOS V202104.00, uses CAN1 and CAN2 modules.
 * Every second, the CAN1 TX function will toggle LEDA and send the
 * LED "status" to CAN2 in an EID message. When CAN2 receives a message,
 * it will set LEDD to the value received in the message, toggle LEDB,
 * and then send the status of LEDB to CAN1. CAN1 will then set LEDC
 * to the received value.
 * 
 * @author
 * Owen Blair
 * @date
 * 10 October 2022
 */


/******************************************************************************
 * This project provides a simple CAN  project.
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Hardware specific includes. */
#include "CerebotMX7cK.h" // JFF

/* Standard demo includes. */
#include <plib.h>

// I want to have a nicer time with strings
#include "string.h"

#include "GenericTypeDefs.h"
#include "CANFunctions.h"

// Extern variable for the timeDiff value for the IO CAN2 module
//extern unsigned int timeDiff = 0;

// Previous 341 libs
#include "pwmlib.h"
#include "input_captureLib.h"
#include "LCDlib.h"

// IR sensor stuff
#include "I2C_IRSensor_lib.h"

/*-----------------------------------------------------------*/

/*
 * Set up the hardware ready to run this demo.
 */
static void prvSetupHardware( void );

/* Tasks that make project run!  */
static void millisecondWaitTask( void );
static void mainIOTask( void );
static void mainCtrlTask( void );
static void RTRWaitTask( void );
//static void inputCaptureHandlerTask( void );

// Helper functions
float calculateMotorSpeed(unsigned int);
float calculateTemperature(unsigned short int);

// Create semaphore handle
xSemaphoreHandle read_semaphore;
xSemaphoreHandle RTR_semaphore;
xSemaphoreHandle inputCapture_semaphore;

// Create queue handle
xQueueHandle inputCaptureQueueHandle;

#if ( configUSE_TRACE_FACILITY == 1 )
    traceString str;
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
    
    #if ( configUSE_TRACE_FACILITY == 1 )
        vTraceEnable(TRC_START); // Initialize and start recording
        str = xTraceRegisterString("Channel");
    #endif
    
/* Create the tasks then start the scheduler. */

    // Variable to check if things created ok
    BaseType_t xReturned;
    
    // Create semaphores!
    read_semaphore = xSemaphoreCreateBinary();
    if(read_semaphore == NULL){
        // Semaphore failed to create
        for( ;; );
    }
    
    RTR_semaphore = xSemaphoreCreateBinary();
    if(RTR_semaphore == NULL){
        // Semaphore failed to create
        for( ;; );
    }
    
    inputCapture_semaphore = xSemaphoreCreateBinary();
    if(inputCapture_semaphore == NULL){
        // Semaphore failed to create
        for( ;; );
    }
    
    // Create queue!
    inputCaptureQueueHandle = xQueueCreate( 1, sizeof(unsigned int));
    if(inputCaptureQueueHandle == NULL){
        // Queue failed to create
        for( ;; );
    }
        
    /* Create the tasks defined within this file. */
    xReturned = xTaskCreate( millisecondWaitTask, "500ms Wait Task", configMINIMAL_STACK_SIZE,
                                    NULL, tskIDLE_PRIORITY, NULL );
    if(xReturned == NULL){
        // Task failed to create
        for( ;; );
    }
    
    xReturned = xTaskCreate( RTRWaitTask, "2 Second RTR Wait Task", configMINIMAL_STACK_SIZE,
                                    NULL, tskIDLE_PRIORITY, NULL );
    if(xReturned == NULL){
        // Task failed to create
        for( ;; );
    }
    
    xReturned = xTaskCreate( mainIOTask, "IO Module", configMINIMAL_STACK_SIZE,
                                    NULL, tskIDLE_PRIORITY, NULL );
    if(xReturned == NULL){
        // Task failed to create
        for( ;; );
    }
    
    xReturned = xTaskCreate( mainCtrlTask, "Control Module", configMINIMAL_STACK_SIZE,
                                    NULL, tskIDLE_PRIORITY, NULL );
    if(xReturned == NULL){
        // Task failed to create
        for( ;; );
    }
    
    /*
    xReturned = xTaskCreate( inputCaptureHandlerTask, "Input Capture Handler", configMINIMAL_STACK_SIZE,
                                    NULL, tskIDLE_PRIORITY, NULL );
    if(xReturned == NULL){
        // Task failed to create
        for( ;; );
    }
    */
    
    vTaskStartScheduler();	/*  Finally start the scheduler. */

/* Will only reach here if there is insufficient heap available to start
 *  the scheduler. */
    return 0;
}  /* End of main */

// Wait 500 milliseconds to give semaphore!
static void millisecondWaitTask( void ){
    for( ;; ){
        // Delay for 500ms
        vTaskDelay(500/portTICK_RATE_MS);
        
        // Give the OK to read temp and motor speed
        xSemaphoreGive(read_semaphore);
    }
}

// Wait 2 seconds to give the OK to send the RTR message semaphore!
static void RTRWaitTask( void ){
    for( ;; ){
        // Delay for 2 seconds
        vTaskDelay(2000/portTICK_RATE_MS);
        
        // Give the OK to send CAN2 RTR for data
        xSemaphoreGive(RTR_semaphore);
    }
}



static void mainCtrlTask ( void ){
    // Variables used to remember things and whatnot
    unsigned int RTRSemaphoreValue = 0;// Used to check for semaphore
    unsigned int sensorReading = 0; // Data from sensor
    unsigned int time_diff = 0; // Speed data
    unsigned int lowTempSet = 0; // Low temperature setpoint
    unsigned int hiTempSet = 0; // Hi temperature setpoint
    unsigned short int controlMode = 0; // Start in Config mode
    LATGSET = LED1; // Start in config mode
    
    unsigned char TxPWMSetting = 0; // Sent PWM setting
    unsigned char RxPWMSetting = 0; // Received PWM setting
    
    float lowTemp = 0.0;
    float highTemp = 0.0;
    float curentTemp = 0.0; // Calculated temperature from IR sensor
    float curentSpeed = 0.0; // Calculated speed of motor
    
    // Strings to print to LCD
    char lineOne[17];
    char lineTwo[18];
    for( ;; ){
        
        // Non-blocking semaphore take
        RTRSemaphoreValue = xSemaphoreTake(RTR_semaphore, 0);
        
        // Was the semaphore taken?
        if(RTRSemaphoreValue != NULL){
            CAN1TxSendMsg(TxPWMSetting, 1);
            LATBINV = LEDA;
        }
        
        // Is there a CAN message?
        unsigned int msgByteData = CAN1RxMsgProcess( &sensorReading, &time_diff, &RxPWMSetting);
        if(msgByteData != 0){ // There was a message!
            LATBINV = LEDB;
            curentTemp = calculateTemperature(sensorReading);
            curentSpeed = calculateMotorSpeed(time_diff);
        }
        
        if(controlMode == 0){ // Config mode
            
            if( PORTG & BTN1){ // If button 1 pressed
                controlMode = 1;
                LATGCLR = LED1;
            }
            
            if( PORTG & BTN2){ // If button 2 pressed
                highTemp = curentTemp;
            }
            
            if( PORTA & BTN3){ // If button 3 pressed
                lowTemp = curentTemp;
            }
            
            if( highTemp <= lowTemp){ // temperature reset condition
                highTemp = 0.0;
                lowTemp = 0.0;
            }
            
            // Create lines to print on the LCD
            LCD_clear();
            sprintf(lineOne, "      %2.1f      ", curentTemp);
            sprintf(lineTwo, "%2.1f        %2.1f\n", lowTemp, highTemp);
            LCD_puts(lineOne);
            LCD_puts(lineTwo);
            // Debounce!
            vTaskDelay(20/portTICK_RATE_MS);
            
            
        }
        else{ // Operational mode
            if( PORTG & BTN1){ // If button 1 pressed
                controlMode = 0;
                LATGSET = LED1;
            }
            // Debounce!
            vTaskDelay(20/portTICK_RATE_MS);
            
            
            // Format the display!
            LCD_clear();
            LCD_puts("                ");
            LCD_puts("                 ");
            sprintf(lineOne, "%2u       %3.2f\n", RxPWMSetting, curentSpeed);
            sprintf(lineTwo, "%2.1f  %2.1f  %2.1f\n", lowTemp, curentTemp, highTemp);
            LCD_puts(lineOne);
            LCD_puts(lineTwo);
            
            // Calculate and send a new PWM signal based on current temperature
            if(curentTemp > highTemp){ // above high temp
                TxPWMSetting = 95;
                CAN1TxSendMsg(TxPWMSetting, 0); // 20% duty cycle, not RTR msg
            }
            else if(curentTemp < lowTemp){ // below low temp
                TxPWMSetting = 20;
                CAN1TxSendMsg(TxPWMSetting, 0); // 95% duty cycle, not RTR msg
            }
            else{ // Between high and low temp
                
                // Calculate PWM using y=mx+b format
                float slope = 0; // the m
                float yIntercept = 0; // the b
                
                slope = (85.0 - 30.0)/(highTemp - lowTemp);
                yIntercept = (slope * (-1.0*lowTemp)) - 30.0;
                //y = m * x + b
                TxPWMSetting = (unsigned char) ( (int) ((slope * curentTemp) + yIntercept));
                TxPWMSetting = TxPWMSetting - 200;
                // Send the updated PWM
                CAN1TxSendMsg(TxPWMSetting, 0); // 30-85% duty cycle, not RTR msg
            }
            LATBINV = LEDC;
        }
        // Debounce!
        vTaskDelay(20/portTICK_RATE_MS);
        
        //taskYIELD();
    }
}

// Main task for IO!
static void mainIOTask ( void ){
    unsigned int returnSemaphoreValue = 0; // Used to check semaphore return
    unsigned short int sensorTemp = 0; // Where to save sensor reading from
    unsigned short int length = 3; // Len in bytes of the reading from sensor
    unsigned int speedData = 0; // Variable to store data from input capture
    
    unsigned char currentPWM = 0; // Remember current PWM setting
    
    unsigned int con_buff[4]; // Buffer for input capture module
    
    char outStr[17]; // General string for debugging (16 chars + a null)
    
    // Task forever loop!
    for ( ;; ){
        
        // Debug set of motor output
        //pwm_set(70);
        
        // Non-blocking semaphore taking
        returnSemaphoreValue = xSemaphoreTake( read_semaphore, 0);
        
        // Check to see if there was a semaphore to take
        if(returnSemaphoreValue == 1){ // There was something to take
            //LATBINV = LEDA;
            
            // Read data from input capture queue
            xQueueReceive( inputCaptureQueueHandle, &speedData, 0);
            
            // Read data from IR sensor
            int i2cError = I2C1_IR_Read(&sensorTemp, length);
            
            // For debugging, check values
            /*
            calculateTemperature(speedData);
            sprintf(outStr, " %u ", speedData);
            LCD_puts(outStr);
            */
            
            // Check to see if there was an IR read error
            if(i2cError != 0){
                for( ;; ); // Get stuck here if there is an error
            }
        }
        
        // Check to see if there is a message
        unsigned char msgByteData = CAN2RxMsgProcess();
        
        // Check to see if there is a message
        if(msgByteData != NULL){
            // Check to see if message was an RTR
            if(msgByteData == 100){ // If true then message is an RTR
                
                // DO THE CAN 2 SEND FUNCTION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                CAN2TxSendMsg(sensorTemp, speedData, currentPWM);
            }
            else{ // Message is not an RTR!
                
                // Update current PWM setting
                currentPWM = msgByteData;

                //Write received PWM settings to motor
                pwm_set(msgByteData);
                LATBINV = LEDD;
            }
        }
        
        // IF DATA FRAME RECEIVE SET MOTOR PWM HERE-----------------------------
        
        
        // The end has been reached. It is safe to switch to a different task
        taskYIELD();
    }
}


/* Calculate Motor Speed Function
 * @brief Function that calculated motor speed from two time measurements
 * 
 * @param1 The time difference between the previous input capture timestamp and
 *          the second newest timestamp
 * 
 * @return Returns the motor speed as a float in Revolutions per second
*/
float calculateMotorSpeed(unsigned int time_diff){
    // Used in motor speed calculations
    float motorSpeed = 0.0;

    // Speed in RPS --> Pclk / (time diff * pre-scale value)
    motorSpeed = 10000000.0 / (time_diff * 256.0);
    
    return motorSpeed;
}


/* Calculate Temperature Function
 * @brief Function that calculates temperature from IR sensor. Only works for
 *          the sensor used in ECE443.
 * 
 * @param1 Raw sensor data
 * 
 * @return Float: The temperature reading of object 1 in degrees (F)
*/
float calculateTemperature(unsigned short int sensorData){
    
    float realTemp = sensorData / 50.0; // Convert data to degrees K
    realTemp = realTemp - 273.15; // Convert to degrees C
    realTemp = realTemp * (9.0/5.0); // Convert to degrees F
    realTemp = realTemp  + 32.0;
    
    return realTemp;
}

/* Setup Hardware Function
 * @brief Function that sets up hardware for the PIC32MX on the Cerabot board
 *          by Digilent
 * 
 * @return None
*/
static void prvSetupHardware( void )
{
    Cerebot_mx7cK_setup();
    
    /* Set up PmodSTEM LEDs */
    PORTSetPinsDigitalOut(IOPORT_B, SM_LEDS);
    LATBCLR = SM_LEDS;                      /* Clear all SM LED bits */
    
    /* Functions are defined in CANFunctions.c */
    CAN1Init();
    CAN2Init(); 
    
    // Setup LCD
    LCDInit();
    
    // Setup PWM for driving the motor. Input is the starting PWM setting
    pwm_init(0); // Uses timer 2
    
    // Setup for input capture module to read motor speed
    inputCapture_init(); // Uses timer 3
    
    // Setup for IR sensor
    init_I2C1_IRSensor();
    
    // Start CAN modules 1 and 2
    CAN1Init();
    CAN2Init();
    
/* Enable multi-vector interrupts */
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);  /* Do only once */
    INTEnableInterrupts();   /*Do as needed for global interrupt control */
    portDISABLE_INTERRUPTS();
    

}


// INPUT CAPTURE MODULE ISR IS HERE---------------------------------------------
/*
 * I put the input capture module here because when it was in the lib it wasn't
 * liking that I had a FreeRTOS call in it. When I moved it here it seemed to
 * work. This way I didn't need an extern to be able to read data from the
 * input capture module.
 */
void __ISR(_INPUT_CAPTURE_5_VECTOR, ipl3) Capture5(void){
    static unsigned int con_buf[4];//   Input capture buffer
    static unsigned int timeDiff;
    
    unsigned char strOut[17]; // Debugging LCD string
    
    // Read from input capture module and make your calculations!
    // Then send time_diff to a queue that is overwritten
    ReadCapture5(con_buf);
    
    // Compute time difference
    timeDiff = con_buf[0]-con_buf[1];
    
    // Debug LCD output
    sprintf(strOut, " %u ", timeDiff);
    
    // Send to a queue that overwrites!
    xQueueOverwriteFromISR(inputCaptureQueueHandle, &timeDiff, NULL );

    
    
    // Clear ISR flag
    mIC5ClearIntFlag();
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
