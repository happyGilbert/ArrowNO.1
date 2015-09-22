/*
 * bluetoothapp.c
 *
 *  Created on: 2015Äê9ÔÂ18ÈÕ
 *      Author: jfanl
 */

#include "gpio.h"
#include "bluetoothapp.h"

#define BLUETOOTH_PW_PORT GPIO_PORT_P4
#define BLUETOOTH_PW_PIN  GPIO_PIN6

void bluetooth_init(){
	GPIO_setAsOutputPin(BLUETOOTH_PW_PORT, BLUETOOTH_PW_PIN);
	GPIO_setOutputLowOnPin(BLUETOOTH_PW_PORT, BLUETOOTH_PW_PIN);
}

void bluetooth_powerDown(){
	GPIO_setOutputHighOnPin(BLUETOOTH_PW_PORT, BLUETOOTH_PW_PIN);
}

void bluetooth_powerUp(){
	GPIO_setOutputLowOnPin(BLUETOOTH_PW_PORT, BLUETOOTH_PW_PIN);
}
