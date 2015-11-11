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
#include "mpu9250app.h"
#include "ads1299app.h"
#include "sharp96app.h"
#include "ds2786app.h"
#include "bluetoothapp.h"

const uint8_t header[] = "uais";
uint8_t rx_header[4] = {0};
bool ads1299_is_running = false;
bool mpu9250_is_running = false;
bool ads1299_manually_send = false;
bool *mpu9250_is_sending;

static void cmdAction(uint8_t cmd);
void serialEvent();

static inline void msp430PlatformInit(void)
{
    //Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);
    //Set VCore = 1 for 12MHz clock
    PMM_setVCore(PMM_CORE_LEVEL_3);
    msp430_clock_init(24000000);
    /* Enable interrupts. */
	__bis_SR_register(GIE);

	mpu9250_is_sending = &ads1299_manually_send;
    bluetooth_init();
    sharp96_init();
    msp430_uart_init(230400);
    msp430_i2c_init();


    ads1299_init(ADS_LEADS_16,&ads1299_manually_send);
    mpu9250_init(mpu9250_is_sending);
	mpu9250_is_running = true;
	ads1299_startRunning();
	ads1299_is_running = true;
}

void main()
{
	unsigned char capacity, chargeStatus,indicate=0;
	msp430PlatformInit();
#ifdef DS27866_WERITE_EEPROM_PARAMETER
	ds2786_setParameter();
#endif

	while(1)
	{
		__bis_SR_register(LPM0_bits + GIE);
		mpu9250_update();
		ads1299_sendData();
		serialEvent();
		if(indicate == 0){
			capacity = ds2786_getRelativeCapacity();
		}else if(indicate == 1){
			chargeStatus = ds2786_getChargeStatus();
		}else if(indicate == 2){
			sharp96_displayBatteryCapacity(capacity, chargeStatus);
		}
		indicate++;
		if(indicate == 10) indicate=0;

	}
}


static inline void clearRxHeader(){
	rx_header[0] = 0;
	rx_header[1] = 0;
	rx_header[2] = 0;
	rx_header[3] = 0;
}

static void cmdAction(uint8_t cmd){
	switch(cmd){
	case 'a':      //start eeg acquisition
		ads1299_startRunning();
		ads1299_is_running = true;
		break;
	case 'b':      //stop eeg acquisition
		ads1299_stopRunning();
		ads1299_is_running = false;
		break;
	case 'c':      //self test signal mode: +3.75mv ~ -3.75mv, 1.953hz
		ads1299_activateAllChannelsToTestCondition_fast_2X();
		ads1299_startRunning();
		ads1299_is_running = true;
		break;
	case 'd':      //turn off self test signal mode, enter normal sampling eeg mode.
		ads1299_activeAllChannelToNormalOperation();
		ads1299_is_running = false;
		break;
	case 'e':      //turn on mpu9250
		if(ads1299_is_running)
			ads1299_stopRunning();
		mpu9250_powerUp(mpu9250_is_sending);
		mpu9250_is_running = true;
		if(ads1299_is_running)
			ads1299_startRunning();
		break;
	case 'f':      //turn off mpu9250
		mpu9250_powerDown();
		mpu9250_is_running = false;
		break;
	case 'g':      //calibrate mpu9250
		if(mpu9250_is_running)
			mpu9250_runSelfTest();
		break;
	case 'h':      //enable print quat
		if(mpu9250_is_running)
			mpu9250_setPrintSensor(PRINT_QUAT, ENABLEOUTPUT);
		break;
	case 'i':      //disable print quat
		if(mpu9250_is_running)
			mpu9250_setPrintSensor(PRINT_QUAT, DISABLEOUTPUT);
		break;
	case 'j':      //enable print euler
		if(mpu9250_is_running)
			mpu9250_setPrintSensor(PRINT_EULER, ENABLEOUTPUT);
		break;
	case 'k':      //disable print euler
		if(mpu9250_is_running)
			mpu9250_setPrintSensor(PRINT_EULER, DISABLEOUTPUT);
		break;
	case 'l':      //enable print accel
		if(mpu9250_is_running)
			mpu9250_setPrintSensor(PRINT_ACCEL, ENABLEOUTPUT);
		break;
	case 'm':      //disable print accel
		if(mpu9250_is_running)
			mpu9250_setPrintSensor(PRINT_ACCEL, DISABLEOUTPUT);
		break;
	case 'n':      //enable print gyro
		if(mpu9250_is_running)
			mpu9250_setPrintSensor(PRINT_GYRO, ENABLEOUTPUT);
		break;
	case 'o':      //disable print gyro
		if(mpu9250_is_running)
			mpu9250_setPrintSensor(PRINT_GYRO, DISABLEOUTPUT);
		break;
	case 'p':      //enable print compass
		if(mpu9250_is_running)
			mpu9250_setPrintSensor(PRINT_COMPASS, ENABLEOUTPUT);
		break;
	case 'q':      //disable print compass
		if(mpu9250_is_running)
			mpu9250_setPrintSensor(PRINT_COMPASS, DISABLEOUTPUT);
		break;
	case 'r':      //save data to SD card
		//TODO:      add save data to sd card function!
		break;
	case 's':
		//TODO:
		break;
	case 't':      //turn on sharp96 lcd
		sharp96_powerUp();
		break;
	case 'u':      //turn off sharp96 lcd
		sharp96_powerDown();
		break;
	default:
		break;
	}
}

void serialEvent()
{
	uint8_t character;
	if(msp430_uart_available() > 0){
		msp430_uart_read(&character);
		if(rx_header[0] == header[0]){
			if(rx_header[1] == header[1]){
				if(rx_header[2] == header[2]){
					if(rx_header[3] == header[3]){
						clearRxHeader();
						cmdAction(character);
					}else if(character == header[3]){
						rx_header[3] = character;
					}else{
						clearRxHeader();
					}
				}else if(character == header[2]){
					rx_header[2] = character;
				}else{
					clearRxHeader();
				}
			}else if(character == header[1]){
				rx_header[1] = character;
			}else{
				clearRxHeader();
			}
		}else if(character == header[0]){
			rx_header[0] = character;
		}
	}
}

