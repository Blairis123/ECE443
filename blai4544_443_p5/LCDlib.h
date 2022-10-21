/** @file LCDlib.h
 * 
 * @brief Lib for using the LCD attached to the Cerabot board
 *
 * @author
 * Owen Blair
 */ 

#ifndef LCDLIB_H
#define LCDLIB_H

#include <plib.h>
#include "CerebotMX7cK.h"

void LCDInit(void);
//int readLCD(int);
//void busyLCD(void);
void LCDWriteChar(int, char);
//void LCDDisplayChar(char);
//void LCDWriteStr(char *);
void LCDDelay(unsigned int);
void LCD_clear(void);


#endif /* MODULE_H */

/*** end of file ***/

