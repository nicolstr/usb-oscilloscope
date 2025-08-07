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
#define NUMBER_OF_ROWS 1
#define NUMBER_OF_COLUMNS 2
#define NUM_OF_TRANSFERS_PER_TRANSACTION 50000

/* TYPEDEFS */

/* trigger modes */
typedef enum{
	TRIGGER_RISING_EDGE,
	TRIGGER_FALLING_EDGE
	/* add trigger modes here */
} TRIGGERMODE_T;

/* FUNCTION PROTOTYPES */
void app_Init();
void reInit_DMA();
void setOnboardStatusLEDs(uint8_t status);

void configureSamplingFrequency();
void configureTrigger(TRIGGERMODE_T triggerMode);
void enableTrigger();
void disableTrigger();
void manualTrigger();

void sendData();
void usbRxCallback(uint8_t* Buf, uint32_t *Len);
void sendCommand(const char* command);

#endif /* APP_H_ */
