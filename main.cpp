/*
 * main.cpp
 *
 *  Created on: 2015Äê9ÔÂ4ÈÕ
 *      Author: jfanl
 */
#include "driverlib.h"
#include "msp430_clock.h"
#include "msp430_interrupt.h"
#include "msp430_uart.h"
#include "msp430_i2c.h"
#include "msp430_sendMPUxxxxData.h"
#include "mpuxxxx.h"

static inline void msp430PlatformInit(void)
{
    //Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);
    //Set VCore = 1 for 12MHz clock
    PMM_setVCore(PMM_CORE_LEVEL_1);
    msp430_clock_init(12000000);
    msp430_uart_init(115200);
    msp430_i2c_init();
    /* Enable interrupts. */
	__bis_SR_register(GIE);
	mpu9250.init();
}

void main()
{
	msp430PlatformInit();
    P8DIR |= BIT1;
    P8OUT |= BIT1;
	while(1)
	{
		mpu9250.update();
		__bis_SR_register(LPM0_bits + GIE);
	}
}
