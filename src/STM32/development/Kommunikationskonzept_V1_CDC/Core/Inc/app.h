/* ***************************************************************************************
 *  Project:    USB oscilloscope
 *  File:       app.h
 *
 *  Language:   C
 ************************************************************************************** */

#ifndef APP_H_
#define APP_H_

/* #INCLUDES */
#include "stm32f7xx.h"
#include "core_cm7.h"

/* #DEFINES */

/* FUNCTION PROTOTYPES */
void app_Init();
void setOnboardStatusLEDs(uint8_t status);
void collectData();
void sendData();

void usbRxCallback(uint8_t* Buf, uint32_t *Len);
void sendCommand(const char* command);

#endif /* APP_H_ */
