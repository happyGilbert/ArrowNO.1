/*
 * msp430_interrupt.h
 *
 *  Created on: 2015��9��4��
 *      Author: jfanl
 */

#ifndef MSP430_INTERRUPT_H_
#define MSP430_INTERRUPT_H_

#ifdef __cplusplus
extern "C" {
#endif


#define INT_EXIT_NONE   (0)
#define INT_EXIT_LPM0   (1)
#define INT_EXIT_LPM1   (2)
#define INT_EXIT_LPM2   (3)
#define INT_EXIT_LPM3   (4)
#define INT_EXIT_LPM4   (5)

#define INT_PORT_P1     (0x100)
#define INT_PIN_P10     (0x102)
#define INT_PIN_P11     (0x104)
#define INT_PIN_P12     (0x106)
#define INT_PIN_P13     (0x108)
#define INT_PIN_P14     (0x10A)
#define INT_PIN_P15     (0x10C)
#define INT_PIN_P16     (0x10E)
#define INT_PIN_P17     (0x110)

#define INT_PORT_P2     (0x200)
#define INT_PIN_P20     (0x202)
#define INT_PIN_P21     (0x204)
#define INT_PIN_P22     (0x206)
#define INT_PIN_P23     (0x208)
#define INT_PIN_P24     (0x20A)
#define INT_PIN_P25     (0x20C)
#define INT_PIN_P26     (0x20E)
#define INT_PIN_P27     (0x210)

/**
 *  @brief Initialize and Enable interrupts.
 */
void msp430_int_init(void);

/**
 *  @brief  Disable interrupts.
 */
void msp430_int_disable(void);

/**
 *  @brief      Register callback for a particular interrupt pin.
 *  This function will override any function already registered.
 *  \n If @e cb is set to NULL, this interrupt will be disabled.
 *  @param[in]  cb          Function executed for this interrupt.
 *  @param[in]  pin         Port/pin number (INT_PIN_Pxx).
 *  @param[in]  lp_exit     Low-power mode exited after this interrupt
 *                          (INT_EXIT_LPMx).
 *  @param[in]  active_low  1 if this interrupt is active low.
 *
 *  @return 0 if successful
 */
int8_t msp430_reg_int_cb(void (*cb)(void), unsigned short pin,
    unsigned char lp_exit, unsigned char active_low);


#ifdef __cplusplus
}
#endif

#endif /* MSP430_INTERRUPT_H_ */
