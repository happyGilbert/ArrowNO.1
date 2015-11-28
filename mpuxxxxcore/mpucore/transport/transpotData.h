/*
 * transpotData.h
 *
 *  Created on: 2015Äê8ÔÂ27ÈÕ
 *      Author: jfanl
 */

#ifndef MSP430_SENDMPUXXXXDATA__
#define MSP430_SENDMPUXXXXDATA__

#ifdef __cplusplus
extern "C" {
#endif

//int _MLPrintLog (int priority, const char* tag, const char* fmt, ...);
//void eMPL_send_quat(long *quat);
void eMPL_send_data(unsigned char type, long *data, unsigned long timestamp);

#ifdef __cplusplus
}
#endif

#endif /* MSP430_SENDMPUXXXXDATA__ */
