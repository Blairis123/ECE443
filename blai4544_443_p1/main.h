/* ************************************************************************** */
/** Descriptive File Name

  @Company
 University of Idaho

  @File Name
 main.h

  @Summary
 Header file for the main.c file in project 1

  @Description
    See summary
 */
/* ************************************************************************** */

/*-----PROTOTYPES------------------------------------------------------------*/

// Set up the hardware!
void prvSetupHardware( void );
unsigned int read_buttons(void);

// Tasks to be created. 1 start task, 2 sendBtn tasks, 1 toggleLED task
static void StartTsk( void *pvParameters );
void sendBtn( void *pvParameters );
static void toggleLED( void *pvParameters );
