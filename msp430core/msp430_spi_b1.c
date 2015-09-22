/*
 * msp430_spi_b1.c
 *
 *  Created on: 2015Äê9ÔÂ16ÈÕ
 *      Author: jfanl
 */
#include "msp430_spi_b1.h"
#include "usci_b_spi.h"
#include "gpio.h"
#include "ucs.h"

#define LCD_SPI_PORT       GPIO_PORT_P4
#define LCD_SPI_CS_PORT	   GPIO_PORT_P4
#define LCD_SPI_SI_PIN     GPIO_PIN1
#define LCD_SPI_SO_PIN     GPIO_PIN2
#define LCD_SPI_CLK_PIN    GPIO_PIN3
#define LCD_SPI_CS_PIN     GPIO_PIN0
// Definition of USCI base address to be used for SPI communication
#define LCD_USCI_BASE	   USCI_B1_BASE
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
void msp430_spi_b1_init(){
    // Configure SPI PORT
    GPIO_setAsPeripheralModuleFunctionOutputPin(LCD_SPI_PORT,
    		LCD_SPI_SI_PIN + LCD_SPI_SO_PIN + LCD_SPI_CLK_PIN);
	// Configure LCD_SPI_CS_PIN as output pin
    GPIO_setAsOutputPin(LCD_SPI_CS_PORT,
    		LCD_SPI_CS_PIN);
    msp430_spi_b1_clearCS();
    USCI_B_SPI_initMasterParam spiMasterParams=
	{
		USCI_B_SPI_CLOCKSOURCE_SMCLK,
		UCS_getSMCLK(),
		1000000,
		USCI_B_SPI_MSB_FIRST,
		USCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,
		USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW
	};

	USCI_B_SPI_initMaster(LCD_USCI_BASE,
                    &spiMasterParams);

    USCI_B_SPI_enable(LCD_USCI_BASE);
}


//*****************************************************************************
//
//! \brief transmit and receive one byte via SPI.
//!
//! \param transmitData is the data to be transmited.
//
//*****************************************************************************
void msp430_spi_b1_transmitData(uint8_t transmitData){
	while (!USCI_B_SPI_getInterruptStatus(LCD_USCI_BASE,
	                    USCI_B_SPI_TRANSMIT_INTERRUPT));
	USCI_B_SPI_transmitData(LCD_USCI_BASE,transmitData);
}
//*****************************************************************************
//
// Clears CS line
//
// This macro allows to clear the Chip Select (CS) line
//
//
//*****************************************************************************
void msp430_spi_b1_clearCS(void){
	GPIO_setOutputLowOnPin(LCD_SPI_CS_PORT, LCD_SPI_CS_PIN);
}

//*****************************************************************************
//
// Set CS line
//
// This macro allows to set the Chip Select (CS) line
//
//*****************************************************************************
void msp430_spi_b1_setCS(void){
	GPIO_setOutputHighOnPin(LCD_SPI_CS_PORT, LCD_SPI_CS_PIN);
}
//*****************************************************************************
//
// Waits until the SPI communication with the LCD is finished a command to
// the LCD Driver
//
// \param None
//
// \return None
//*****************************************************************************
void msp430_spi_b1_waitUntilLcdWriteFinish(void)
{
	while  (USCI_B_SPI_isBusy(LCD_USCI_BASE));
}
