

//#define Fsck 400000 // From ECE341 for EEPROM on PIC32MX7 Cerabot board
#define Fsck 50000 // Set for IR sensor in Project 4

#define BRG_VAL ((FPB/2/Fsck)-2)

// Function prototypes
void init_EEPROM();
int I2C_IR_Read(int, int, char *, int);
int writeEEPROM(int, int, char *, int);
void wait_I2C_Xfer(int);