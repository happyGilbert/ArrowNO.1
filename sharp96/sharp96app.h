/*
 * sharp96app.h
 *
 *  Created on: 2015��9��18��
 *      Author: jfanl
 */

#ifndef SHARP96APP_H_
#define SHARP96APP_H_

void sharp96_init();
void sharp96_powerDown();
void sharp96_powerUp();
void sharp96_displayBatteryCapacity(unsigned char capacity, unsigned char chargeStatus);

#endif /* SHARP96APP_H_ */
