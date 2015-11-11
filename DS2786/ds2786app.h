/*
 * ds2786app.h
 *
 *  Created on: 2015Äê10ÔÂ28ÈÕ
 *      Author: jfanl
 */

#ifndef _DS2786APP_H_
#define _DS2786APP_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"{
#endif

//*****************************************************************************
//
// when a battery with different capacity connects to the system, uncomment
// below define to enable function "ds2786_setParameter()" ,modify BATCAPACITY
// (in the ds2786app.c)and run program once, then comment below define again!
// WARNING: EEPROM has limited write times. ds_setParameter() will write EEPROM
// once to save battery information. so, do not run ds_setParameter() when
// capacity has not changed!
//
//
//*****************************************************************************
//#define DS27866_WERITE_EEPROM_PARAMETER
#ifdef DS27866_WERITE_EEPROM_PARAMETER
void ds2786_setParameter();
#endif
//*****************************************************************************
//
//! \brief get battery remaining capacity in percentage.
//!
//! \return battery remaining capacity in percentage.
//
//*****************************************************************************
unsigned char ds2786_getRelativeCapacity();
//*****************************************************************************
//
//! \brief get charge status.
//!
//! \return 0: discharge, 1:charge.
//
//*****************************************************************************
unsigned char ds2786_getChargeStatus();
//*****************************************************************************
//
//! \brief get battery voltage.
//!
//! \return vlotage in mV, resolution: 1.22mV.
//
//*****************************************************************************
float ds2786_getVoltage();
//*****************************************************************************
//
//! \brief get current flow into or flow out of the battery.
//!
//! \return current in mA, resolution: 1.25mA.
//
//*****************************************************************************
float ds2786_getCurrent();
//*****************************************************************************
//
//! \brief get battery temperature.
//!
//! \return temperature in Celsius degree, resolution: 0.125 Celsius degree.
//
//*****************************************************************************
float ds2786_getTemperature();

#ifdef __cplusplus
}
#endif

#endif /* _DS2786APP_H_ */
