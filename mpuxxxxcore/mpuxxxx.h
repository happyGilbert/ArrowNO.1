/*
 * mpuxxxx.h
 *
 *  Created on: 2015Äê8ÔÂ29ÈÕ
 *      Author: jfanl
 */

#ifndef _MPU9250_H_
#define _MPU9250_H_

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)
/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (20)

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char rx_new;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};

class MPUCLASS{
public:
	MPUCLASS();
	//*****************************************************************************
	//
	//! \brief Initializes MPUxxxx device with 9-axes fution and output quaternion.
	//
	//*****************************************************************************
	void init(void);
	//*****************************************************************************
	//
	//! \brief Use to get new data when new data available.
	//!
	//! If use this function when data is not ready, the function will do nothing.
	//
	//*****************************************************************************
	void update();
	//*****************************************************************************
	//
	//! \brief Set which data need output.
	//!
	//! This function will turn on the necessary sensor according to output data
	//!      type requirements.
	//!
	//! \param printMask is the specified data to print.
	//!  Mask value is the logical OR of any of the following:
	//!  PRINT_NONE
	//!  PRINT_ACCEL
	//!  PRINT_GYRO
	//!  PRINT_QUAT
	//!  PRINT_COMPASS
	//!  PRINT_EULER
	//!  PRINT_ROT_MAT
	//!  PRINT_HEADING
	//!  PRINT_PEDO
	//!  PRINT_LINEAR_ACCEL
	//!  PRINT_GRAVITY_VECTOR
	//
	//*****************************************************************************
	void printSensor(uint16_t printMask);
	//*****************************************************************************
	//
	//! \brief Set sampling rate.
	//
	//*****************************************************************************
	void setSampleRate(unsigned short accel_gyro_rate_hz, unsigned short compass_rate_hz);
	//*****************************************************************************
	//
	//! \brief Enable DMP function.
	//
	//*****************************************************************************
	void dmpEnable(uint8_t enable);
	//*****************************************************************************
	//
	//! \brief Set DMP interrupt mode
	//!
	//! \param mode: DMP_INT_GESTURE or DMP_INT_CONTINUOUS.*/
	//
	//*****************************************************************************
	void dmpInterruptMode(unsigned char mode);
	//*****************************************************************************
	//
	//! \brief Enter low power accelerometer mode.
	//!
	//! only use accelerometer to get accelerate data, other sensors will be power
	//!   down.
	//
	//*****************************************************************************
	void lpAccelMode();
	//*****************************************************************************
	//
	//! \brief Enter low power motion mode.
	//!
	//! only use accelerometer to get accelerate data, and determine whether the
	//!   specified motion is happened. Other sensors will be power down.
	//! Motion is specified by thresh, duration and sampling rate.
	//
	//*****************************************************************************
	void motionInterruptEnable(uint8_t enable);
	//*****************************************************************************
	//
	//! \brief Run MPUxxxx device self test and get bias data, storage to flash.
	//!
	//! Run self test. if device is passed to test, then get bias data of
	//!   accelerometer and gyroscope and storage them to flash.
	//! Bias data is used to calibrated initial value.
	//
	//*****************************************************************************
	void runSelfTest(void);
private:
	void read_from_mpl(void);  //get mpl 9-axes fution data and send to uart.
	void setup_sensor(void);   //Handle sensor on/off combinations.
	int8_t loadMplState();     //load bias data from flash.
	int8_t saveMplState();     //save bias data to flash.
	struct hal_s hal;
};

extern MPUCLASS mpu9250;

#endif /* _MPUXXXX_H_ */
