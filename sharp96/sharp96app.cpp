/*
 * sharp96app.cpp
 *
 *  Created on: 2015Äê9ÔÂ18ÈÕ
 *      Author: jfanl
 */
#include "sharp96.h"
#include "msp430_clock.h"

#include "sharp96app.h"
#include "lzu_logo.c"

lcdSharp96 sharp96;

void sharp96_init(){
    sharp96.init();
    sharp96.drawImage(0,0,95,95,lzu_logo_96x96);
    sharp96.flush();
    msp430_delay_ms(1000);
    sharp96.drawRectangle(0,0,95,95,true);
    sharp96.flush();
    msp430_delay_ms(1000);
    sharp96.clearBuffer();
    sharp96.setFont(1);
    sharp96.displayString(1,5,"UAIS LAB<",8);
    sharp96.drawHorizontalLine(0,95,20);
    sharp96.drawHorizontalLine(0,95,22);
    sharp96.setFont(0);
    sharp96.displayString(24,25,"16-Leads",8);
    sharp96.displayString(18,35,"EEG system",10);
    sharp96.drawHorizontalLine(10,85,45);
    sharp96.drawVerticalLine(5,28,40);
    sharp96.drawVerticalLine(90,28,40);
    sharp96.drawRectangle(5,45,50,90,FILL);
    sharp96.drawRectangle(50,90,50,65,UNFILL);
    sharp96.drawRectangle(10,40,55,85,SHARP_WHITE,FILL);
    sharp96.drawRectangle(49,91,74,91,SHARP_WHITE,UNFILL);
    sharp96.flush();
}

void sharp96_powerDown(){
	sharp96.powerDown();
}

void sharp96_powerUp(){  //display same information that was displayed before power down.
	sharp96.powerUp();
	msp430_delay_us(20);
    sharp96.flush();
}

void sharp96_displayBatteryCapacity(unsigned char capacity, unsigned char chargeStatus){
	sharp96.setFont(0);
	char cap[4];
	if(chargeStatus){
		sharp96.displayString(52,54,"C",1);
	}else{
		sharp96.displayString(52,54,"D",1);
	}
	if(capacity == 100){
		sharp96.displayString(65,54,"100%",4);
	}else{
		cap[0] = ' ';
		cap[1] = capacity / 10 + '0';
		cap[2] = capacity % 10 + '0';
		cap[3] = '%';
		sharp96.displayString(65,54,cap,4);
	}
    sharp96.flush();
}
