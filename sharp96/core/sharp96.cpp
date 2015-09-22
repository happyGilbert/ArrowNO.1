/*
 * sharp96.c
 *
 *  Created on: 2015Äê9ÔÂ16ÈÕ
 *      Author: jfanl
 */
//*****************************************************************************
#include "sharp96.h"
#include "gpio.h"
#include "msp430_spi_b1.h"
#include "msp430_clock.h"

//*****************************************************************************
//
// User Configuration for the LCD Driver
//
//*****************************************************************************
// Ports from MSP430 connected to LCD
#define LCD_DISP_PORT                       GPIO_PORT_P5
#define LCD_POWER_PORT                      GPIO_PORT_P5

// Pins from MSP430 connected to LCD
#define LCD_DISP_PIN                        GPIO_PIN6
#define LCD_POWER_PIN                       GPIO_PIN7


//*****************************************************************************
//
// Macros for the Display Driver
//
//*****************************************************************************
#define SHARP_SEND_COMMAND_RUNNING          0x01
#define SHARP_REQUEST_TOGGLE_VCOM           0x02
#define SHARP_LCD_TRAILER_BYTE				0x00
#define SHARP_VCOM_TOGGLE_BIT 		   		0x40
#define SHARP_LCD_CMD_CHANGE_VCOM			0x00
#define SHARP_LCD_CMD_CLEAR_SCREEN			0x20
#define SHARP_LCD_CMD_WRITE_LINE			0x80


unsigned char DisplayBuffer[LCD_VERTICAL_MAX][LCD_HORIZONTAL_MAX/8];
unsigned char VCOMbit = 0x40;
unsigned char flagSendToggleVCOMCommand = 0;

uint8_t textx = 0; 
uint8_t texty = 0;
uint8_t textstartx =0; 
uint8_t textstarty =0;
uint8_t lineSpacing[NUM_OF_FONTS] = {9,16};

static void SendToggleVCOMCommand(void);

static uint8_t sizeofstr(const char *str){
	uint8_t size = 0;
	while(*str++ != 0) size++;
	return size;
}

void lcdSharp96::setXY(uint8_t x, uint8_t y, uint8_t  ulValue) {
    if( ulValue != 0)
        DisplayBuffer[y][x>>3] &= ~(0x80 >> (x & 0x7));
    else
        DisplayBuffer[y][x>>3] |= (0x80 >> (x & 0x7));
}

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
void lcdSharp96::init() {
	// Init SPI
	msp430_spi_b1_init();
    // Provide power to LCD
	GPIO_setAsOutputPin(LCD_POWER_PORT,
	        LCD_POWER_PIN);

	GPIO_setOutputHighOnPin(LCD_POWER_PORT,
            LCD_POWER_PIN);

	// Turn on DISP
	GPIO_setAsOutputPin(LCD_DISP_PORT,
	        LCD_DISP_PIN);

	GPIO_setOutputHighOnPin(LCD_DISP_PORT,
            LCD_DISP_PIN);
	msp430_register_task_cb(SendToggleVCOMCommand,1000);
	ClearScreen();
    _font = 0;
}

void lcdSharp96::powerDown(){
	GPIO_setOutputLowOnPin(LCD_DISP_PORT,
            LCD_DISP_PIN);
	GPIO_setOutputLowOnPin(LCD_POWER_PORT,
            LCD_POWER_PIN);
}
void lcdSharp96::powerUp(){
	GPIO_setOutputHighOnPin(LCD_POWER_PORT,
            LCD_POWER_PIN);
	GPIO_setOutputHighOnPin(LCD_DISP_PORT,
            LCD_DISP_PIN);
}

void lcdSharp96::ClearScreen() {
  
  unsigned char command = SHARP_LCD_CMD_CLEAR_SCREEN;
  
  // set flag to indicate command transmit is running
  flagSendToggleVCOMCommand |= SHARP_SEND_COMMAND_RUNNING;
  
  command |= VCOMbit;                    //COM inversion bit

  msp430_spi_b1_setCS();

  msp430_spi_b1_transmitData(command);
  msp430_spi_b1_transmitData(SHARP_LCD_TRAILER_BYTE);

  // Wait for last byte to be sent, then drop SCS
  msp430_spi_b1_waitUntilLcdWriteFinish();

  // Ensure a 2us min delay to meet the LCD's thSCS
  __delay_cycles(SYSTEM_CLOCK_SPEED * 0.000002);

  msp430_spi_b1_clearCS();
  
  // clear flag to indicate command transmit is free
  flagSendToggleVCOMCommand &= ~SHARP_SEND_COMMAND_RUNNING;  
  SendToggleVCOMCommand(); // send toggle if required

  clearBuffer();
  
}

void lcdSharp96::clearBuffer() {
    unsigned int i=0,j=0;
    for(i =0; i< LCD_VERTICAL_MAX; i++)
    for(j =0; j< (LCD_HORIZONTAL_MAX>>3); j++)
       DisplayBuffer[i][j] = SHARP_WHITE;
}

void lcdSharp96::setFont(tNumOfFontsType font) {
    _font = font;
}

void lcdSharp96::setLineSpacing(uint8_t pixel) {
    lineSpacing[_font] = pixel;
}

void lcdSharp96::drawHorizontalLine(uint8_t x1, uint8_t x2, uint8_t y){
	drawHorizontalLine(x1, x2, y, SHARP_BLACK);
}

void lcdSharp96::drawHorizontalLine(uint8_t x1, uint8_t x2, uint8_t y, uint8_t color){
    uint8_t xi = 0;
    uint8_t x1_index = x1 >> 3,  x2_index = x2 >> 3;
    uint8_t *buf_ptr, first_x_byte, last_x_byte;

    //calculate first byte
    //mod by 8 and shift this # bits
    first_x_byte = (0xFF >> (x1 & 0x07));
    //calculate last byte
    //mod by 8 and shift this # bits
    last_x_byte = (0xFF << (7 - (x2 & 0x07)));

    //set buffer to correct location
    buf_ptr = &DisplayBuffer[y][x1_index];

    //check if more than one data byte
    if(x1_index != x2_index){
    	//black pixels (clear bits)
    	if(color == SHARP_BLACK){
    		//write first byte
    		*buf_ptr++ &= ~first_x_byte;
    		//write middle bytes
    		for(xi = x1_index; xi < x2_index - 1; xi++){
    			*buf_ptr++ = SHARP_BLACK;
    		}
    		//write last byte
    		*buf_ptr &= ~last_x_byte;
    	}else{
    		//white pixels (set bits)
    		*buf_ptr++ |= first_x_byte;
    		//write middle bytes
    		for(xi = x1_index; xi < x2_index - 1; xi++){
    			*buf_ptr++ = SHARP_WHITE;
    		}
    		//write last byte
    		*buf_ptr |= last_x_byte;
    	}
    }else{//only one data byte
    	//calculate value of single byte
    	first_x_byte &= last_x_byte;
    	if(color == SHARP_BLACK){
    		//draw black pixels (clear bits)
    		*buf_ptr &= ~first_x_byte;
    	}else{
    		//white pixels (set bits)
    		*buf_ptr |= first_x_byte;
    	}
    }
}

void lcdSharp96::drawVerticalLine(uint8_t x, uint8_t y1, uint8_t y2){
	drawVerticalLine(x, y1, y2, SHARP_BLACK);
}

void lcdSharp96::drawVerticalLine(uint8_t x, uint8_t y1, uint8_t y2, uint8_t color){
    uint16_t yi = 0;
    uint16_t x_index = x >> 3;
    uint8_t data_byte;

    //calculate data byte
    //mod by 8 and shift this # bits
    data_byte = (0x80 >> (x & 0x7));

    //write data to the display buffer
    for(yi = y1; yi <= y2; yi++)
    {
        //black pixels (clear bits)
        if(color == SHARP_BLACK)
        {
            DisplayBuffer[yi][x_index] &= ~data_byte;
        }
        //white pixels (set bits)
        else
        {
            DisplayBuffer[yi][x_index] |= data_byte;
        }
    }
}

void lcdSharp96::drawRectangle(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2, bool fill){
	drawRectangle(x1, x2, y1, y2, SHARP_BLACK, fill);
}

void lcdSharp96::drawRectangle(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2, uint8_t color, bool fill){
	uint8_t xi = 0, yi = 0;
    uint8_t x1_index = x1 >> 3, x2_index = x2 >> 3;
    uint8_t *buf_ptr, first_x_byte, last_x_byte, fill_color;

    drawHorizontalLine(x1, x2, y1, color);
    drawHorizontalLine(x1, x2, y2, color);


    if(fill){
    	//calculate first byte
        //mod by 8 and shift this # bits
        first_x_byte = (0xFF >> (x1 & 0x7));

        //calculate last byte
        //mod by 8 and shift this # bits
        last_x_byte = (0xFF << (7 - (x2 & 0x7)));
        //calculate value of single byte
        if(x1_index == x2_index){
        	first_x_byte &= last_x_byte;
        }
        fill_color = color;
    }else{
    	//calculate first byte
        //mod by 8 and shift this # bits
        first_x_byte = (0x80 >> (x1 & 0x7));

        //calculate last byte
        //mod by 8 and shift this # bits
        last_x_byte = (0x80 >> (x2 & 0x7));
        //calculate value of single byte
        if(x1_index == x2_index){
        	first_x_byte |= last_x_byte;
        }
        fill_color = ~color;
    }
    //write bytes
    for(yi = y1+1; yi < y2; yi++){
        //set buffer to correct location
        buf_ptr = &DisplayBuffer[yi][x1_index];

        //black pixels (clear bits)
        if(color == SHARP_BLACK){
            //write first byte
            *buf_ptr++ &= ~first_x_byte;

            //check if more than one data byte
            if(x1_index != x2_index){
                //write middle bytes
                for(xi = x1_index; xi < x2_index - 1; xi++)
                {
                    *buf_ptr++ = fill_color;
                }

                //write last byte
                *buf_ptr &= ~last_x_byte;
            }
        }//white pixels (set bits)
        else{
        	if(!fill){
        		*buf_ptr &= (0xFF << (8 - (x1 & 0x07)));
        	}
            //write first byte
            *buf_ptr++ |= first_x_byte;

            //check if more than one data byte
            if(x1_index != x2_index){
                //write middle bytes
                for(xi = x1_index; xi < x2_index - 1; xi++)
                {
                    *buf_ptr++ = fill_color;
                }

            	if(!fill){
            		*buf_ptr &= (0xFF >> (1+ (x2 & 0x07)));
            	}
                //write last byte
                *buf_ptr |= last_x_byte;
            }
        }

    }
}

void lcdSharp96::displayString(uint8_t x, uint8_t y, const char *data) {
	displayString(x, y, data, sizeofstr(data), LCDWrapNextLine);
}

void lcdSharp96::displayString(uint8_t x, uint8_t y, const char *data, uint8_t length) {
	displayString(x, y, data, length, LCDWrapNextLine);
}

void lcdSharp96::displayString(uint8_t x, uint8_t y, const char *data, uint8_t length, tLCDWrapType wrap) {
    uint8_t i;
    uint8_t j;
    int8_t k;
    int8_t deltax = 5;
    int8_t deltay = lineSpacing[_font];
	char c;
	
	setCharXY(x, y);
    if (_font==1) {
		deltax = 11;
	}
    
	for (j=0; j<length; j++) {
		c = data[j];
		if  ((wrap == LCDWrapLine)     && (textx+deltax > LCD_HORIZONTAL_MAX)) {textx = textstartx;}
		if  ((wrap == LCDWrapNextLine) && (textx+deltax > LCD_HORIZONTAL_MAX)) {textx = textstartx; texty += deltay;}
		if  (c == '\n') {textx = textstartx; texty += deltay;}
		else if  (c >= ' ') {
			if (_font==0) {
				for (i=0; i<deltax; i++){ 
					for (k=7; k>=0; k--) setXY(textx, texty+k, Terminal6x8[c-' '][i]&(1<<k));
					textx += 1;
				}
			}
			if (_font==1) {
				for (i=0; i<deltax; i++){
					for (k=7; k>=0; k--){ 
						setXY(textx, texty+k,   Terminal11x16[c-' '][2*i]&(1<<k));
						setXY(textx, texty+k+8, Terminal11x16[c-' '][2*i+1]&(1<<k));
					}
					textx += 1;
				}
			}
			textx += 1;  // spacing
		}
	}
}

void lcdSharp96::drawImage(uint8_t x, uint8_t y, uint8_t width, uint8_t high, const uint8_t *data){
	uint8_t deltax = 0, deltay = 0;
	uint16_t index = 0;
    if(((y + high) > LCD_VERTICAL_MAX) || ((x + width) > LCD_HORIZONTAL_MAX)) return;

    for(deltay = 0; deltay < high; deltay++){
    	for(deltax = 0; deltax < width;){
    		setXY(x + deltax, y + deltay, data[index] & (0x80 >> (deltax & 0x07)));
    		deltax++;
    		if((!(deltax & 0x07)) || (deltax == width)) index++;
    	}
    }
}

void lcdSharp96::reversalDisplay(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2){
	uint8_t xi = 0, yi = 0;
    uint8_t x1_index = x1 >> 3, x2_index = x2 >> 3;
    uint8_t *buf_ptr, first_x_byte, last_x_byte;

	//calculate first byte
    //mod by 8 and shift this # bits
    first_x_byte = (0xFF >> (x1 & 0x7));

    //calculate last byte
    //mod by 8 and shift this # bits
    last_x_byte = (0xFF << (7 - (x2 & 0x7)));
    if(x1_index == x2_index){
    	first_x_byte &= last_x_byte;
    }
    //write bytes
    for(yi = y1; yi <= y2; yi++){
        //set buffer to correct location
        buf_ptr = &DisplayBuffer[yi][x1_index];
        //write first byte
        *buf_ptr++ ^= first_x_byte;

        //check if more than one data byte
        if(x1_index != x2_index){
        	//write middle bytes
        	for(xi = x1_index; xi < x2_index - 1; xi++){
        		*buf_ptr = ~(*buf_ptr);
        		buf_ptr++;
        	}
        	//write last byte
        	*buf_ptr ^= last_x_byte;
        }
    }
}

void lcdSharp96::setCharXY(uint8_t x, uint8_t y) {
	textx =x; 
	texty =y;
	textstartx =x; 
	textstarty =y;
}

const uint8_t referse_data[] = {0x0, 0x8, 0x4, 0xC, 0x2, 0xA, 0x6, 0xE, 0x1, 0x9, 0x5, 0xD, 0x3, 0xB, 0x7, 0xF};
uint8_t lcdSharp96::reverse(uint8_t x)
{
  uint8_t b = 0;
  
  b  = referse_data[x & 0xF]<<4;
  b |= referse_data[(x & 0xF0)>>4];
  return b;
}


void lcdSharp96::flush (void)
{
    unsigned char *buf_ptr = &DisplayBuffer[0][0];
    long xi =0;
    long xj = 0;
    //image update mode(1X000000b)
    unsigned char command = SHARP_LCD_CMD_WRITE_LINE;

    // set flag to indicate command transmit is running
    flagSendToggleVCOMCommand |= SHARP_SEND_COMMAND_RUNNING;
    //COM inversion bit
    command |= VCOMbit;
    msp430_spi_b1_setCS();

    msp430_spi_b1_transmitData((char)command);
    for(xj=0; xj<LCD_VERTICAL_MAX; xj++)
    {
        msp430_spi_b1_transmitData((char)reverse(xj + 1));

        for(xi=0; xi<(LCD_HORIZONTAL_MAX>>3); xi++)
        {
            msp430_spi_b1_transmitData((char)*(buf_ptr++));
        }
        msp430_spi_b1_transmitData(SHARP_LCD_TRAILER_BYTE);
    }

    msp430_spi_b1_transmitData((char)SHARP_LCD_TRAILER_BYTE);

    // Wait for last byte to be sent, then drop SCS
    msp430_spi_b1_waitUntilLcdWriteFinish();

    // Ensure a 2us min delay to meet the LCD's thSCS
    __delay_cycles(SYSTEM_CLOCK_SPEED * 0.000002);

    msp430_spi_b1_clearCS();
    // clear flag to indicate command transmit is free
    flagSendToggleVCOMCommand &= ~SHARP_SEND_COMMAND_RUNNING;  
    SendToggleVCOMCommand(); // send toggle if required
}

static void SendToggleVCOMCommand(void)
{
    if(!(flagSendToggleVCOMCommand & SHARP_REQUEST_TOGGLE_VCOM)){ // no request pending ?
        VCOMbit ^= SHARP_VCOM_TOGGLE_BIT;                 // Toggle VCOM Bit
    }

    if(flagSendToggleVCOMCommand & SHARP_SEND_COMMAND_RUNNING){
        // set request flag
        flagSendToggleVCOMCommand |= SHARP_REQUEST_TOGGLE_VCOM;
    }else{  // if no communication to LCD -> send toggle sequence now
        unsigned char command = SHARP_LCD_CMD_CHANGE_VCOM;
        command |= VCOMbit;                    //COM inversion bit

        // Set P2.4 High for CS
        msp430_spi_b1_setCS();

        msp430_spi_b1_transmitData((char)command);
        msp430_spi_b1_transmitData((char)SHARP_LCD_TRAILER_BYTE);

        // Wait for last byte to be sent, then drop SCS
        msp430_spi_b1_waitUntilLcdWriteFinish();

        // Ensure a 2us min delay to meet the LCD's thSCS
        __delay_cycles(SYSTEM_CLOCK_SPEED * 0.000002);

        msp430_spi_b1_clearCS();

        // clear request flag
        flagSendToggleVCOMCommand &= ~SHARP_REQUEST_TOGGLE_VCOM;
    }
}



