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
    void init();
    void powerDown();
    void powerUp();
    void ClearScreen();
    void clearBuffer();
    void setFont(tNumOfFontsType font=0);
    void setLineSpacing(uint8_t pixel);
    void setXY(uint8_t x, uint8_t y,  uint8_t ulValue);
    void drawRectangle(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2, bool fill);
    void drawRectangle(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2, uint8_t color, bool fill);
    void drawHorizontalLine(uint8_t x1, uint8_t x2, uint8_t y);
    void drawHorizontalLine(uint8_t x1, uint8_t x2, uint8_t y, uint8_t color);
    void drawVerticalLine(uint8_t x, uint8_t y1, uint8_t y2);
    void drawVerticalLine(uint8_t x, uint8_t y1, uint8_t y2, uint8_t color);
    void displayString(uint8_t x, uint8_t y, const char *data);
    void displayString(uint8_t x, uint8_t y, const char *data, uint8_t length);
    void displayString(uint8_t x, uint8_t y, const char *data, uint8_t length, tLCDWrapType wrap);
    void reversalDisplay(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2);
    void drawImage(uint8_t x, uint8_t y, uint8_t width, uint8_t high, const uint8_t *data);
    void flush();


private:
    tNumOfFontsType _font;
    void setCharXY(uint8_t x, uint8_t y);
    uint8_t reverse(uint8_t x);
};

#endif
