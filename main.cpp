/*
 * main.cpp
 *
 *  Created on: 2015Äê9ÔÂ4ÈÕ
 *      Author: jfanl
 */
#include "driverlib.h"
#include "msp430_clock.h"
#include "msp430_interrupt.h"
static inline void msp430PlatformInit(void)
{
    //Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);
    //Set VCore = 1 for 12MHz clock
    PMM_setVCore(PMM_CORE_LEVEL_1);
    msp430_clock_init(12000000);
    /* Enable interrupts. */
	__bis_SR_register(GIE);
}

uint32_t time;
bool blinkstate = false;
void blink()
{
	blinkstate = !blinkstate;
}

void main()
{
	msp430PlatformInit();
	msp430_delay_ms(11);
	msp430_get_clock_ms(&time);
	msp430_delay_ms(21);
	msp430_get_clock_ms(&time);
	GPIO_setAsOutputPin(GPIO_PORT_P1,
			            GPIO_PIN0
			            );
	msp430_reg_int_cb(blink, INT_PIN_P17, INT_EXIT_NONE, 1);
	while(1)
	{
		if(blinkstate){
			msp430_delay_ms(100);
			GPIO_setOutputHighOnPin(GPIO_PORT_P1,
	                                GPIO_PIN0
	                                );
			msp430_delay_ms(100);
			GPIO_setOutputLowOnPin(GPIO_PORT_P1,
	                               GPIO_PIN0
	                               );
		}
	}
}
