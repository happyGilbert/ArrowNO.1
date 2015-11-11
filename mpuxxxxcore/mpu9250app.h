/*
 * mpu9250app.h
 *
 *  Created on: 2015Äê8ÔÂ29ÈÕ
 *      Author: jfanl
 */

#ifndef _MPU9250_H_
#define _MPU9250_H_

#define ENABLEOUTPUT    true
#define DISABLEOUTPUT   false

#define PRINT_NONE      (0x00)
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

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

//*****************************************************************************
//
//! \brief Initializes MPUxxxx device with 9-axes fution and output quaternion.
//!
//! \param mpu_isSending used to indicate whether or not mpu9250 is sending
//!                      data to bluetooth uart port!.
//
//*****************************************************************************
void mpu9250_init(bool *mpu_isSending);
//*****************************************************************************
//
//! \brief Use to get new data when new data available.
//!
//! If use this function when data is not ready, the function will do nothing.
//
//*****************************************************************************
void mpu9250_update();
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
void mpu9250_setPrintSensor(uint8_t printMask, uint8_t enable);
//*****************************************************************************
//
//! \brief Set sampling rate.
//
//*****************************************************************************
void mpu9250_setSampleRate(unsigned short accel_gyro_rate_hz, unsigned short compass_rate_hz);
//*****************************************************************************
//
//! \brief Enable DMP function.
//
//*****************************************************************************
void mpu9250_dmpEnable(uint8_t enable);
//*****************************************************************************
//
//! \brief Set DMP interrupt mode
//!
//! \param mode: DMP_INT_GESTURE or DMP_INT_CONTINUOUS.*/
//
//*****************************************************************************
void mpu9250_dmpInterruptMode(unsigned char mode);
//*****************************************************************************
//
//! \brief Enter low power accelerometer mode.
//!
//! only use accelerometer to get accelerate data, other sensors will be power
//!   down.
//
//*****************************************************************************
void mpu9250_lpAccelMode();
//*****************************************************************************
//
//! \brief Enter low power motion mode.
//!
//! only use accelerometer to get accelerate data, and determine whether the
//!   specified motion is happened. Other sensors will be power down.
//! Motion is specified by thresh, duration and sampling rate.
//
//*****************************************************************************
void mpu9250_motionInterruptEnable(uint8_t enable);
//*****************************************************************************
//
//! \brief Run MPUxxxx device self test and get bias data, storage to flash.
//!
//! Run self test. if device is passed to test, then get bias data of
//!   accelerometer and gyroscope and storage them to flash.
//! Bias data is used to calibrated initial value.
//
//*****************************************************************************
void mpu9250_runSelfTest(void);
//*****************************************************************************
//
//! \brief power up mpu9250 and call mpu9250_init() to Initializes mpu9250.
//
//*****************************************************************************
void mpu9250_powerUp(bool *mpu_isSending);
//*****************************************************************************
//
//! \brief power down mpu9250.
//
//*****************************************************************************
void mpu9250_powerDown();

#endif /* _MPUXXXX_H_ */
