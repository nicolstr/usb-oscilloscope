/* ***************************************************************************************
 *  Project:    USB oscilloscope
 *  File:       queue.c
 *
 *  Language:   C
 ************************************************************************************** */

/* #INCLUDES */
#include "queue.h"

/* #DEFINES */

/* PRIVATE FUNCTION PROTOTYPES */
void STANDBY();
void ENTER_CRITICAL();
void LEAVE_CRITICAL();

/* FUNCTION DEFINITIONS */

/* void enqueue(volatile queue_t *queue, uint8_t in)
 *
 * inserts an element 'in' into the queue
 *
 * NO HANDLING OF A FULL QUEUE!		 					*/
void enqueue(volatile queue_t *queue, uint8_t in){
    ENTER_CRITICAL();
    queue->fifo[queue->tail++] = in;
    LEAVE_CRITICAL();
}

/* void dequeue(volatile queue_t *queue, uint8_t *out)
 *
 * extracts an element from the queue and inserts it into variable
 * referenced by 'out'
 *
 * HANDLING OF EMTPY QUEUE -> STANDBY		 			*/
void dequeue(volatile queue_t *queue, uint8_t *out){
	//LEAVE_CRITICAL();
    while(queue->head == queue->tail){
        //STANDBY();
    }

    ENTER_CRITICAL();
    *out = queue->fifo[queue->head++];
    LEAVE_CRITICAL();
}

/* void STANDBY()
 *
 * standby function										*/
void STANDBY(){
    asm("ISB"); /* Flush pipeline      */
    asm("WFI"); /* Wait for interrupts */
}

/* void ENTER_CRITICAL()
 *
 * execute before entering critical section				*/
void ENTER_CRITICAL(){
    asm("CPSID I"); /* Disable interrupts */
}

/* void LEAVE_CRITICAL()
 *
 * execute after leaving critical section				*/
void LEAVE_CRITICAL(){
    asm("CPSIE I"); /* Enable interrupts */
    asm("ISB");     /* Flush pipeline    */
}
