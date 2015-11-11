/*
 * sharp96.h
 *
 *  Created on: 2015Äê9ÔÂ16ÈÕ
 *      Author: jfanl
 */
//*****************************************************************************
//
//sharp96.h - Prototypes for the Sharp96x96
//                  LCD display driver. There is no output from Sharp96x96 LCD
//
//             MSP430F5529(USCI B1)               LCD Sharp96x96
//           -----------------------             -----------------
//          |           P4.3/UCB1CLK|---------> |SPI_CLK  EXT_MODE|--GND
//          |                       |           |            	  |
//          |          P4.1/UCB1SIMO|---------> |SPI_SI   EXTCOMIN|--GND
//          |                       |           |              	  |
//          | msp430_spi_b1.c-> P4.0|---------> |SPI_CS(          |
//          |                  	    |		    |         	      |
//          |                   P3.6|---------> |DISP             |
//          |                       |		    |                 |
//          |                   P3.7|-----*---> |VDD              |
//          |                       |      `--> |VDDA             |
//           -----------------------             -----------------
//
//*****************************************************************************
#ifndef SHARP96_H__
#define SHARP96_H__

#include "stdint.h"
#include "Terminal6.h"
#include "Terminal12.h"

#define FILL   true
#define UNFILL false
#define SHARP_BLACK							0x00
#define SHARP_WHITE							0xFF

// SYSTEM_CLOCK_SPEED (in Hz) allows to properly closeout SPI communication
#define SYSTEM_CLOCK_SPEED      12000000
#define LCD_VERTICAL_MAX    96
#define LCD_HORIZONTAL_MAX  96
#define NUM_OF_FONTS 2

typedef enum
{
    LCDWrapNone,                         // do not wrap
    LCDWrapLine,                         // wrap, to beginning of line
    LCDWrapNextLine                      // wrap, to next line
}
tLCDWrapType;
typedef uint8_t tNumOfFontsType;



class lcdSharp96{
public:
	//*****************************************************************************
	//
	//! Initializes the display driver.
	//!
	//! This function initializes the Sharp96x96 display. This function
	//! configures the GPIO pins used to control the LCD display when the basic
	//! GPIO interface is in use. On exit, the LCD has been reset and is ready to
	//! receive command and data writes.
	//!
	//! \return None.
	//
	//*****************************************************************************
    void init();
    //*****************************************************************************
    //
    //! Power Down the Sharp96.
    //!
    //! This function used to disable the power which supply to shar96 lcd.
    //! Because MCU has hold 96X96 RAM space for display buffer, shap96 can dispay
    //! same information which displayed before sharp96 power down when sharp96
    //! power up again and use flush function to force to display information!
    //!
    //! \return None.
    //
    //*****************************************************************************
    void powerDown();
    //*****************************************************************************
    //
    //! Power up the Sharp96.
    //!
    //! This function used to enable the power which supply to shar96 lcd.
    //! Because MCU has hold 96X96 RAM space for display buffer, shap96 can dispay
    //! same information which displayed before sharp96 power down when sharp96
    //! power up again and use flush function to force to display information!
    //!
    //! \return None.
    //
    //*****************************************************************************
    void powerUp();
    //*****************************************************************************
    //
    //! Clear screen and display buffer.
    //!
    //! This function used to clear screen and display buffer!
    //!
    //! \return None.
    //
    //*****************************************************************************
    void ClearScreen();
    //*****************************************************************************
    //
    //! Clear display buffer.
    //!
    //! This function used to display buffer!
    //!
    //! \return None.
    //
    //*****************************************************************************
    void clearBuffer();
    //*****************************************************************************
    //
    //! set font which will be used next.
    //!
    //! This function used to set font which will be used next.
    //! \param: font: 0: 5x8, 1: 11x16
    //!
    //! \return None.
    //
    //*****************************************************************************
    void setFont(tNumOfFontsType font=0);
    //*****************************************************************************
    //
    //! set line space between adjacent lines.
    //!
    //! This function used to set line space between adjacent lines.
    //! \param: line space in pixel.
    //!
    //! \return None.
    //
    //*****************************************************************************
    void setLineSpacing(uint8_t pixel);
    //*****************************************************************************
    //
    //! set a point in display buffer.
    //! \param: x: x of point, 0 ~ 95
    //! \param: y: y of point, 0 ~ 95
    //! \param: ulValue: 0: clear the point, 1: set the point.
    //!
    //! /return none.
    //
    //*****************************************************************************
    void setXY(uint8_t x, uint8_t y,  uint8_t ulValue);
    //*****************************************************************************
    //
    //! Draw rectangle.
    //!
    //! This function used to draw a rectangle.
    //! \param: x1: x of start point, 0 ~ 95
    //! \param: y1: y of start point, 0 ~ 95
    //! \param: x2: y of end point, 0 ~ 95
    //! \param: y2: y of end point, 0 ~ 95
    //! \param: fill : false: unfill, true: fill.
    //!
    //! \return None.
    //
    //*****************************************************************************
    void drawRectangle(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2, bool fill);
    //*****************************************************************************
    //
    //! Draw rectangle.
    //!
    //! This function used to draw a rectangle.
    //! \param: x1: x of start point, 0 ~ 95
    //! \param: y1: y of start point, 0 ~ 95
    //! \param: x2: y of end point, 0 ~ 95
    //! \param: y2: y of end point, 0 ~ 95
    //! \param: fill : false: unfill, true: fill.
    //! \param: color :SHARP_BLACK(lit), SHARP_WHITE(unlit)
    //!
    //! \return None.
    //
    //*****************************************************************************
    void drawRectangle(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2, uint8_t color, bool fill);
    //*****************************************************************************
    //
    //! Draw horizontal line.
    //!
    //! This function used to draw a horizontal line.
    //! \param: x1: x of start point, 0 ~ 95
    //! \param: x2: x of end point, 0 ~ 95
    //! \param: y: y of horizontal line, 0 ~ 95
    //!
    //! \return None.
    //
    //*****************************************************************************
    void drawHorizontalLine(uint8_t x1, uint8_t x2, uint8_t y);
    //*****************************************************************************
    //
    //! Draw horizontal line.
    //!
    //! This function used to draw a horizontal line.
    //! \param: x1: x of start point, 0 ~ 95
    //! \param: x2: x of end point, 0 ~ 95
    //! \param: y: y of horizontal line, 0 ~ 95
    //! \param: color :SHARP_BLACK(lit), SHARP_WHITE(unlit)
    //!
    //! \return None.
    //
    //*****************************************************************************
    void drawHorizontalLine(uint8_t x1, uint8_t x2, uint8_t y, uint8_t color);
    //*****************************************************************************
    //
    //! Draw vertical line.
    //!
    //! This function used to draw a vertical line.
    //! \param: x: x of vertical line, 0 ~ 95
    //! \param: y1: y of start point, 0 ~ 95
    //! \param: y2: y of end point, 0 ~ 95
    //!
    //! \return None.
    //
    //*****************************************************************************
    void drawVerticalLine(uint8_t x, uint8_t y1, uint8_t y2);
    //*****************************************************************************
    //
    //! Draw vertical line.
    //!
    //! This function used to draw a vertical line.
    //! \param: x: x of vertical line, 0 ~ 95
    //! \param: y1: y of start point, 0 ~ 95
    //! \param: y2: y of end point, 0 ~ 95
    //! \param: color :SHARP_BLACK(lit), SHARP_WHITE(unlit)
    //!
    //! \return None.
    //
    //*****************************************************************************
    void drawVerticalLine(uint8_t x, uint8_t y1, uint8_t y2, uint8_t color);
    //*****************************************************************************
    //
    //! Display string .
    //!
    //! This function used to display a string.
    //! \param: x: x of start point, 0 ~ 95
    //! \param: y: y of start point, 0 ~ 95
    //! \param: data: data of string.
    //! A pixel for space between characters.
    //!
    //! \return None.
    //
    //*****************************************************************************
    void displayString(uint8_t x, uint8_t y, const char *data);
    //*****************************************************************************
    //
    //! Display string .
    //!
    //! This function used to display a string.
    //! \param: x: x of start point, 0 ~ 95
    //! \param: y: y of start point, 0 ~ 95
    //! \param: data: data of string.
    //! \param: length: the number of character of string.
    //! A pixel for space between characters.
    //!
    //! \return None.
    //
    //*****************************************************************************
    void displayString(uint8_t x, uint8_t y, const char *data, uint8_t length);
    //*****************************************************************************
    //
    //! Display string .
    //!
    //! This function used to display a string.
    //! \param: x: x of start point, 0 ~ 95
    //! \param: y: y of start point, 0 ~ 95
    //! \param: data: data of string.
    //! \param: length: the number of character of string.
    //! \param: wrap LCDWrapNone: do not wrap,
    //!              LCDWrapLine: wrap, to beginning of line,
    //!              LCDWrapNextLine: wrap, to next line.
    //! A pixel for space between characters.
    //!
    //! \return None.
    //
    //*****************************************************************************
    void displayString(uint8_t x, uint8_t y, const char *data, uint8_t length, tLCDWrapType wrap);
    //*****************************************************************************
    //
    //! Reverse display information!.
    //!
    //! Reverse a rectangle area of display buffer, need use flush to force to screen.
    //! \param: x1: x of start point, 0 ~ 95
    //! \param: y1: y of start point, 0 ~ 95
    //! \param: x2: x of end point, 0 ~ 95
    //! \param: y2: y of end point, 0 ~ 95
    //
    //*****************************************************************************
    void reversalDisplay(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2);
    //*****************************************************************************
    //
    //! Draw image .
    //!
    //! This function used to display a image.
    //! \param: x: x of start point, 0 ~ 95
    //! \param: y: y of start point, 0 ~ 95
    //! \param: width: width of image in pixel.
    //! \param: high:  high of image in pixel.
    //! \param: data: data of image.
    //! Image is horizontal scan!
    //!
    //! \return None.
    //
    //*****************************************************************************
    void drawImage(uint8_t x, uint8_t y, uint8_t width, uint8_t high, const uint8_t *data);
    //*****************************************************************************
    //
    //! Force display buffer information to display in screen.
    //!
    //! /return none.
    //
    //*****************************************************************************
    void flush();


private:
    tNumOfFontsType _font;
    void setCharXY(uint8_t x, uint8_t y);
    uint8_t reverse(uint8_t x);
};

#endif
