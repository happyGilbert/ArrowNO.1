/*
 * msp430_spi_b1.h
 *
 *  Created on: 2015Äê9ÔÂ16ÈÕ
 *      Author: jfanl
 */

#ifndef MSP430_SPI_B1_H_
#define MSP430_SPI_B1_H_
//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"{
#endif
#include "stdint.h"
//*****************************************************************************
//
//! \brief Initializes USCI_B1 with SPI mode.
//!
//! CLK: 1MHZ.
//! ClockPhase: USCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT.
//! ClockPolarity: USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW
//! MSB_first
//!
//! The Seeting is specified to Sharp96 communication.
//
//*****************************************************************************
void msp430_spi_b1_init();
//*****************************************************************************
//
//! \brief transmit and receive one byte via SPI.
//!
//! \param transmitData is the data to be transmited.
//
//*****************************************************************************
void msp430_spi_b1_transmitData(uint8_t transmitData);
//*****************************************************************************
//
// Clears CS line
//
// This macro allows to clear the Chip Select (CS) line
//
//
//*****************************************************************************
void msp430_spi_b1_clearCS(void);
//*****************************************************************************
//
// Set CS line
//
// This macro allows to set the Chip Select (CS) line
//
//*****************************************************************************
void msp430_spi_b1_setCS(void);
//*****************************************************************************
//
// Waits until the SPI communication with the LCD is finished a command to
// the LCD Driver
//
// \param None
//
// \return None
//*****************************************************************************
void msp430_spi_b1_waitUntilLcdWriteFinish(void);


#ifdef __cplusplus
}
#endif


#endif /* MSP430_SPI_B1_H_ */
