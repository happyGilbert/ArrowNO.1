/*
 * ds2786app.c
 *
 *  Created on: 2015Äê10ÔÂ28ÈÕ
 *      Author: jfanl
 */


#include "msp430_i2c.h"
#include "ds2786app.h"
#include "msp430_clock.h"


/*Register Map*/
#define DS2786SC       0x01 //  Status/Config
#define DS2786RC       0x02 //  Relative Capacity
#define DS2786AI0      0x08 //  Auxiliary Input 0
#define DS2786AI1      0x0a //  Auxiliary Input
#define DS2786TEMP     0x0a //  Temperature
#define DS2786VOLTAGE  0x0c //  Voltage
#define DS2786CURRENT  0x0e //  Current
#define DS2786IV       0x14 //  Initial Voltage
#define DS2786LOCVRC   0x16 //  Last OCV Relative Capacity
#define DS2786LCSCF    0x17 //  Learned Capacity Scaling Factor
#define DS2786COMMAND  0xfe //  Command


/*EEPORM Parameter Registers*/
#define DS2786COBR     0x60 //  Current Offset Bias Register
#define DS2786CAP1     0x61 //  Capacity 1
#define DS2786CAP2     0x62 //  Capacity 2
#define DS2786CAP3     0x63 //  Capacity 3
#define DS2786CAP4     0x64 //  Capacity 4
#define DS2786CAP5     0x65 //  Capacity 5
#define DS2786CAP6     0x66 //  Capacity 6
#define DS2786CAP7     0x67 //  Capacity 7
#define DS2786VB0      0x68 //  Volatge Break Point 0
#define DS2786VB1      0x6a //  Volatge Break Point 1
#define DS2786VB2      0x6c //  Volatge Break Point 2
#define DS2786VB3      0x6e //  Volatge Break Point 3
#define DS2786VB4      0x70 //  Volatge Break Point 4
#define DS2786VB5      0x72 //  Volatge Break Point 5
#define DS2786VB6      0x74 //  Volatge Break Point 6
#define DS2786VB7      0x76 //  Volatge Break Point 7
#define DS2786VB8      0x78 //  Volatge Break Point 8
#define DS2786ICSF     0x7a //  Initial Capacity Scaling Threshold
#define DS2786OCVCT    0x7b //  Blanking/OCV Current Threshold
#define DS2786OCVDVDT  0x7c //  OCV dV/dt Threshold
#define DS2786I2CADDR  0x7d //  I2C Address Configuration
#define DS2786LT       0x7e //  Learn Threshold
#define DS2786UE       0x7f //  User EEPROM


#define DS2786ADDR  0x36
#define BATCAPACITY 580  // Battery capacity in mAh.
#define RSENSE      20 // Current sense register in Ohm.

#ifdef DS27866_WERITE_EEPROM_PARAMETER
//*****************************************************************************
//
// when a battery with different capacity connects to the system, modify
// BATCAPACITY and run function "ds2786_setParameter()" once, ONLY ONCE!!
// WARNING: EEPROM has limited write times. ds_setParameter() will write EEPROM
// once to save battery information. so, do not run ds_setParameter() when
// capacity has not changed!
//
//*****************************************************************************
void ds2786_setParameter(){
	unsigned char ICSF, status, cmd, dvdt;
	float data;
	data = 100.0/BATCAPACITY*1000/RSENSE*1000/78.125;
	ICSF = (unsigned char)data;
	msp430_i2c_read(DS2786ADDR,DS2786OCVDVDT,1,&dvdt);
	dvdt = (dvdt & 0x0f) + 0x30;
	msp430_i2c_write(DS2786ADDR,DS2786ICSF,1,&ICSF);
	msp430_i2c_write(DS2786ADDR,DS2786OCVDVDT,1,&dvdt);
	cmd = 0x01;
	msp430_i2c_write(DS2786ADDR,DS2786COMMAND,1,&cmd);
	msp430_delay_ms(3);
	cmd = 0x00;
	msp430_i2c_write(DS2786ADDR,DS2786COMMAND,1,&cmd);
	msp430_delay_ms(20);
	cmd = 0x80;
	msp430_i2c_write(DS2786ADDR,DS2786COMMAND,1,&cmd);
	msp430_delay_ms(100);
	status = 0;
	ICSF = 0;
	msp430_i2c_read(DS2786ADDR,DS2786ICSF,1,&ICSF);
	msp430_i2c_read(DS2786ADDR,DS2786SC,1,&status);
	msp430_delay_ms(100);
}
#endif

//*****************************************************************************
//
//! \brief get battery remaining capacity in percentage.
//!
//! \return battery remaining capacity in percentage.
//
//*****************************************************************************
unsigned char ds2786_getRelativeCapacity(){
	unsigned char relativeCapacity;
//	unsigned char dat;
//	msp430_i2c_read(DS2786ADDR,DS2786ICSF,1,dat);
	msp430_i2c_read(DS2786ADDR,DS2786RC,1,&relativeCapacity);
	relativeCapacity = relativeCapacity >> 1;
	return relativeCapacity;
}
//*****************************************************************************
//
//! \brief get battery voltage.
//!
//! \return vlotage in mV, resolution: 1.22mV.
//
//*****************************************************************************
float ds2786_getVoltage(){
	unsigned char data[2];
	float voltage;
	msp430_i2c_read(DS2786ADDR,DS2786VOLTAGE,2,data);
	voltage = ((((unsigned int)(data[0] & 0x7F)) << 5) + (data[1] >> 3)) * 1.22;
	return voltage;
}
//*****************************************************************************
//
//! \brief get current flow into or flow out of the battery.
//!
//! \return current in mA, resolution: 1.25mA.
//
//*****************************************************************************
float ds2786_getCurrent(){
	unsigned char data[2];
	float current;
	msp430_i2c_read(DS2786ADDR,DS2786CURRENT,2,data);
	current = ((((unsigned int)(data[0] & 0x7F)) << 4) + (data[1] >> 4)) * 25 / 20;
	return current;
}
//*****************************************************************************
//
//! \brief get charge status.
//!
//! \return 0: discharge, 1:charge.
//
//*****************************************************************************
unsigned char ds2786_getChargeStatus(){
	unsigned char data[2];
	msp430_i2c_read(DS2786ADDR,DS2786CURRENT,2,data);
	if( (data[0] & 0x80) ){
		return 0;
	}else{
		return 1;
	}
}
//*****************************************************************************
//
//! \brief get battery temperature.
//!
//! \return temperature in Celsius degree, resolution: 0.125 Celsius degree.
//
//*****************************************************************************
float ds2786_getTemperature(){
	unsigned char data[2];
	float temperature;
	msp430_i2c_read(DS2786ADDR,DS2786TEMP,2,data);
	temperature = ((((unsigned int)(data[0] & 0x7F)) << 3) + (data[1] >> 5)) * 0.125;
	return temperature;
}
