#include <plib.h>
#include "CerebotMX7cK.h"
#include "input_captureLib.h"

//-----------------------------INPUT CAPTURE INIT AND ISR---------------------
void inputCapture_init()
{
    // Set inputs for hall sensors
    PORTSetPinsDigitalIn(IOPORT_D, (MTR_SA | MTR_SB));
    
    // Setup timer 3 for timing of incoming inputs!
    timer3_init();
    
    // Clear pending input interrupts
    mIC5ClearIntFlag();
    
    // Setup IC5 with the following options
    int c1 = IC_ON;             // IC5 settings
    int c2 = IC_CAP_16BIT;
    int c3 = IC_IDLE_STOP;
    int c4 = IC_FEDGE_FALL;
    int c5 = IC_TIMER3_SRC;
    int c6 = IC_INT_1CAPTURE;
    int c7 = IC_EVERY_FALL_EDGE;
    OpenCapture5(c1 | c2 | c3 | c4 | c5 | c6 | c7);// The actual setup!
    
    // Configure ISR for input capture 5
    int ic1 = IC_INT_ON;            // Enable input capture interrupt
    int ic2 = IC_INT_PRIOR_3;       // Priority level 3
    int ic3 = IC_INT_SUB_PRIOR_0;   // Sub priority level 0
    ConfigIntCapture5(ic1 | ic2 | ic3);// The actual setup!
}





//-----------------------------TIMER 3 INIT AND ISR---------------------------
void timer3_init()
{
    // Set timer 3 with a pre-scale of 256 and a max interval
    OpenTimer3(T3_ON | T3_PS_1_256 | T3_SOURCE_INT, 0xFFFF);
    
    // Set timer priority and enable
    mT3SetIntPriority(2);
    mT3SetIntSubPriority(2);
    mT3IntEnable(1);
}

void __ISR(_TIMER_3_VECTOR, ipl2) T3Interrupt(void)
{
    // Toggle LED C
    //LATBINV = LEDC;
    
    // Clear interrupt flag
    mT3ClearIntFlag();
}