/* ***************************************************************************************
 *  Project:    USB oscilloscope
 *  File:       queue.h
 *
 *  Language:   C
 ************************************************************************************** */

#ifndef QUEUE_H_
#define QUEUE_H_

/* #INCLUDES */
#include "stm32f7xx.h"
#include "core_cm7.h"

/* #DEFINES */

/* TYPEDEFS */

/* Queue structure
 *
 * implements a circular buffer */
typedef struct{
    uint8_t fifo[256];
    uint8_t head;	// read Idx
    uint8_t tail;	// write Idx
}queue_t;

/* FUNCTION PROTOTYPES */
void enqueue(volatile queue_t *queue, uint8_t in);
void dequeue(volatile queue_t *queue, uint8_t *out);

#endif /* QUEUE_H_ */
