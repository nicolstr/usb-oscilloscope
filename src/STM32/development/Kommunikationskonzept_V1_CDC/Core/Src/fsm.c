/* ***************************************************************************************
 *  Project:    USB oscilloscope
 *  File:       fsm.c
 *
 *  Language:   C
 ************************************************************************************** */

/* #INCLUDES */
#include "fsm.h"
#include "queue.h"
#include "app.h"

/* #DEFINES */

/* Private events - only available inside of fsm */
#define EV_ENTRY 1U
#define EV_EXIT  2U

/* TYPEDEFS */

/* FSM structure
 *
 * fsm has one active state at a time (deterministic fsm)
 *
 * state is represented by a function pointer to the active state function
 * 														*/
typedef struct fsm_t fsm_t;
typedef void(*state_fp)(fsm_t *fsm, uint8_t event);
struct fsm_t{
    state_fp state;
};

/* EXTERN GLOBAL VARIABLES */
extern volatile uint8_t vcpConnected;

/* STATIC GLOBAL VARIABLES */
static volatile queue_t event_queue = {.head = 0, .tail = 0};

/* PRIVATE FUNCTION PROTOTYPES */

/* fsm BASIC FUNCTIONS */
static void fsm_init(fsm_t *fsm, state_fp init_state);
static void fsm_dispatch(fsm_t *fsm, uint8_t event);
static void fsm_transition(fsm_t *fsm, state_fp new_state);

/* fsm STATE FUNCTIONS */
static void fsm_state_init(fsm_t *fsm, uint8_t event);
static void fsm_state_idle(fsm_t *fsm, uint8_t event);
static void fsm_state_data_acquisition(fsm_t *fsm, uint8_t event);
static void fsm_state_data_transmission(fsm_t *fsm, uint8_t event);
static void fsm_state_wait_for_ACK(fsm_t *fsm, uint8_t event);


/* FUNCTION DEFINITIONS */

/* void fsm_run()
 *
 * run the main fsm application
 *
 * GLOBAL VISIBILITY		 							*/
void fsm_run(){
    fsm_t   fsm_i; /* FSM instance.                     */
    uint8_t event; /* Buffer for storing incoming event */

    // wait for connection
    while(!vcpConnected) {}

    fsm_init(&fsm_i, fsm_state_init);

    while(1){
    	// only execute fsm, when VCP is connected
    	while(vcpConnected)
    	{
            dequeue(&event_queue, &event);
            /*
             * Some events can be handled independently of state,
             * while others are dispatched to the current state
             * in the default condition.
             */
            switch(event){
            //case EV_FOO:
            //    foo();
            //    break;
            case EV_RESET:
                fsm_init(&fsm_i, fsm_state_init);
                break;
            default:
                fsm_dispatch(&fsm_i, event);
                break;
            }
    	}
    }
}

/* void fsm_add_event(uint8_t in)
 *
 * used to enqueue events (e.g. in ISRs)
 *
 * GLOBAL VISIBILITY		 							*/
void fsm_add_event(uint8_t in){
    enqueue(&event_queue, in);
}

/* static void fsm_init(fsm_t *fsm, state_fp init_state)
 *
 * initializes the FSM and brings it into Init State
 * 		 												*/
static void fsm_init(fsm_t *fsm, state_fp init_state){
    fsm->state = init_state;
    fsm_dispatch(fsm, EV_ENTRY);
}

/* static void fsm_dispatch(fsm_t *fsm, uint8_t event)
 *
 * calls the specific function of a state
 * 		 												*/
static void fsm_dispatch(fsm_t *fsm, uint8_t event){
    (*(fsm)->state)(fsm, event);
}

/* static void fsm_transition(fsm_t *fsm, state_fp new_state)
 *
 * 1. executes the exit statements of current state
 * 2. changes the state (-> transition)
 * 3. executes the entry statements of new state
 * 		 												*/
static void fsm_transition(fsm_t *fsm, state_fp new_state){
    fsm_dispatch(fsm, EV_EXIT);
    fsm->state = new_state;
    fsm_dispatch(fsm, EV_ENTRY);
}


/* STATE FUNCTIONS
 * each state has entry and exit statements, that are executed, when function is called
 * with the corresponding event EV_ENTRY and EV_EXIT 	*/

/* initialization state
 *
 * ONLY RUN ONCE - initialize peripherals										*/
static void fsm_state_init(fsm_t *fsm, uint8_t event){
    switch(event){
    case EV_ENTRY:
        /* State entry actions */
        app_Init();
        setOnboardStatusLEDs(1);
        break;
    case EV_START_APP:
        fsm_transition(fsm, fsm_state_idle);
        break;
    case EV_EXIT:
        /* State exit actions */
        break;
    default:
        break;
    }
}

/* idle state
 *
 * WAIT FOR RUN_START FROM HOST (PC) */
static void fsm_state_idle(fsm_t *fsm, uint8_t event){
    switch(event){
    case EV_ENTRY:
        /* State entry actions */
    	setOnboardStatusLEDs(2);
    	sendCommand("STATE:IDLE");
        break;
    case EV_SAMPL_RUN_START:
        fsm_transition(fsm, fsm_state_data_acquisition);
        break;
    case EV_EXIT:
        /* State exit actions */
        break;
    default:
        break;
    }
}

static void fsm_state_data_acquisition(fsm_t *fsm, uint8_t event){
    switch(event){
    case EV_ENTRY:
        /* State entry actions */
    	setOnboardStatusLEDs(3);
    	sendCommand("STATE:ACQUIRE");
        break;
    case EV_B1_PRESSED:
    	collectData();
        fsm_add_event(EV_DATA_READY);	// later in DMA ISR
        break;
    case EV_DATA_READY:
    	sendCommand("ACQUISITON:DONE");
        break;
    case EV_DATA_TRANSMIT:
        fsm_transition(fsm, fsm_state_data_transmission);
        break;
    case EV_EXIT:
        /* State exit actions */
        break;
    default:
        break;
    }
}

static void fsm_state_data_transmission(fsm_t *fsm, uint8_t event){
    switch(event){
    case EV_ENTRY:
        /* State entry actions */
    	setOnboardStatusLEDs(4);
    	sendCommand("STATE:TRANSMISSION");
    	// send data
    	sendData();
        break;
    case EV_SAMPL_RECEIVED:
        fsm_transition(fsm, fsm_state_wait_for_ACK);
        break;
    case EV_EXIT:
        /* State exit actions */
        break;
    default:
        break;
    }
}

static void fsm_state_wait_for_ACK(fsm_t *fsm, uint8_t event){
    switch(event){
    case EV_ENTRY:
        /* State entry actions */
    	setOnboardStatusLEDs(5);
    	sendCommand("STATE:WAIT_FOR_ACK");
        break;
    case EV_SAMPL_ACK_CONTINUE:
        fsm_transition(fsm, fsm_state_data_acquisition);
        break;
    case EV_SAMPL_ACK_STOP:
        fsm_transition(fsm, fsm_state_idle);
        break;
    case EV_EXIT:
        /* State exit actions */
        break;
    default:
        break;
    }
}

