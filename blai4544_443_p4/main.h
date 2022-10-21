/** @file module.h
 * 
 * @brief Header file for prototypes and other stuff! 
 *
 * @author
 * Owen Blair
 */ 

#ifndef MODULE_H
#define MODULE_H


static void prvSetupHardware( void );

// User defined functions
unsigned int read_buttons();
void hw_msDelay(unsigned int);

// Task Prototypes!
static void msHeartbeatTask();
static void EEPROM_Hndlr_tsk();
static void LCD_Handler_Task();
static void IR_Handler_Task();
static void CN_Task();

#endif /* MODULE_H */

/*** end of file ***/