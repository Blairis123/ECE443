#include <plib.h>
#include "CerebotMX7cK.h"
#include "pwmlib.h"

void pwm_init(int d_cyc){
    int OC3RS = (d_cyc * T2_INTR_RATE) / 100;
    //timer2_interrupt_initialize();
    // Timer 2 and T2_ISR init
    //configure Timer 2 with internal clock, 1:1 prescale, PR1 for 1 ms period
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, T2_INTR_RATE-1);
    
    // set up the timer interrupt with a priority of 2, sub priority 0
    mT2SetIntPriority(2); // Group priority range: 1 to 7
    mT2SetIntSubPriority(0); // Subgroup priority range: 0 to 3
    mT2IntEnable(1); // Enable T1 interrupts
    // Global interrupts must enabled to complete the initialization.
    
    // Output compare init stuff
    OpenOC3(OC_ON|OC_TIMER_MODE16|OC_TIMER2_SRC|OC_PWM_FAULT_PIN_DISABLE, OC3RS, OC3RS);
    
    pwm_set(d_cyc);// Set starting speed
}

void __ISR(_TIMER_2_VECTOR, IPL2) Timer2Handler(void){
    LATBINV = LEDA;
    mT2ClearIntFlag();
}

void pwm_set(int dutyCycle){
    int nOC3RS = ((dutyCycle * T2_INTR_RATE) / 100);
    SetDCOC3PWM(nOC3RS);
}