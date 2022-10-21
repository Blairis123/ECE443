

//#define Fsck 400000 // From ECE341 for EEPROM on PIC32MX7 Cerabot board
#define Fsck 50000 // Set for IR sensor in Project 4

#define BRG_VAL ((FPB/2/Fsck)-2)

// Function prototypes
void init_I2C1_IRSensor();
int I2C1_IR_Read(unsigned short int* , unsigned short int);
//int writeEEPROM(int, int, char *, int);
void wait_I2C_Xfer(int);