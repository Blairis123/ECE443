
// Inputs from Hall sensors on DC motor
#define MTR_SA BIT_3
#define MTR_SB BIT_12

// For the running average buffer
#define bufSize 16

// Extern Variable for reporting speed to LCD
extern unsigned int timeDiff;

// Function prototypes
void inputCapture_init(void);
void cn_interrupt_initialize(void);
void timer3_init(void);
