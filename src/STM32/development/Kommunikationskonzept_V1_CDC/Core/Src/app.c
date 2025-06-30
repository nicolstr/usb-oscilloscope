/* ***************************************************************************************
 *  Project:    USB oscilloscope
 *  File:       app.c
 *
 *  Language:   C
 ************************************************************************************** */

/* #INCLUDES */
#include "app.h"
#include "main.h"
#include "fsm.h"
#include "usbd_cdc_if.h"
#include <string.h>


/* #DEFINES */
#define USB_BUFLEN 64
#define CMD_RX_BUFFER_SIZE 128
#define CMD_TX_BUFFER_SIZE 128

/* PRIVATE FUNCTION PROTOTYPES */
static void CDC_Send(uint8_t* data, uint16_t len);
static void receiveCommand();
static void processCommand(const char* cmd);
static void GPIO_Init();
static void EXTI_Init();
static void TIM3_Init();

/* STATIC GLOBAL VARIABLES */
static char cmdTxBuffer[CMD_TX_BUFFER_SIZE];

static uint8_t usbRxBuf[USB_BUFLEN];
static uint16_t usbRxBufLen;
static volatile uint8_t usbRxFlag = 0;

static uint8_t cmdRxBuffer[CMD_RX_BUFFER_SIZE];
static uint16_t cmdRxIndex = 0;

static uint16_t sampleBuffer[1000] = {0};

/* FUNCTION DEFINITIONS */

/* app_Init()
 *
 * run all application initializations
 *	 													*/
void app_Init()
{
	__disable_irq();

		GPIO_Init();
		EXTI_Init();
		TIM3_Init();

	__enable_irq();
}

/* void setOnboardStatusLEDs(uint8_t status)
 *
 * uses onboard LEDs LD1...3 of NUCLEO-F767 for status display
 *
 * possible inputs: 0...7
 *
 * 0 is not recommended -> LEDs are off
 * (not distinguishable from error or invalid input)
 *	 													*/
void setOnboardStatusLEDs(uint8_t status)
{
	int ledMask = 0;
	// turn LD1..3 off
	GPIOB->ODR &= ~(LD1_Pin|LD2_Pin|LD3_Pin);
	switch(status)
	{
	case 0:
		break;
	case 1:
		ledMask = LD1_Pin;
		break;
	case 2:
		ledMask = LD2_Pin;
		break;
	case 3:
		ledMask = LD2_Pin|LD1_Pin;
		break;
	case 4:
		ledMask = LD3_Pin;
		break;
	case 5:
		ledMask = LD3_Pin|LD1_Pin;
		break;
	case 6:
		ledMask = LD3_Pin|LD2_Pin;
		break;
	case 7:
		ledMask = LD3_Pin|LD2_Pin|LD1_Pin;
		break;
	default:
		break;
	}
	GPIOB->ODR |= ledMask;
}

/* void usbRxCallback(uint8_t* Buf, uint32_t *Len)
 *
 * callback function called by CDC_Receive_FS(...) in "usbd_cdc_if.h"
 *
 * receives a newline-terminated command
 *	 													*/
void usbRxCallback(uint8_t* Buf, uint32_t *Len)
{
	memcpy(usbRxBuf, Buf, *Len);
	usbRxBufLen = *Len;
	usbRxFlag = 1;
	receiveCommand();
}

/* void sendCommand(const char* command)
 *
 * takes the command string as an argument
 *
 * sends a newline-terminated command
 *	 													*/
void sendCommand(const char* command)
{
    // format command into newline-terminated string
    int len = snprintf(cmdTxBuffer, CMD_TX_BUFFER_SIZE, "%s\n", command);

    // only send if length is valid (safety)
    if (len > 0 && len < CMD_TX_BUFFER_SIZE) {
        CDC_Send((uint8_t*)cmdTxBuffer, len);
    }
}

void collectData()
{
	for(int i = 0; i < sizeof(sampleBuffer)/sizeof(sampleBuffer[0]); i++)
	{
		sampleBuffer[i] = i;
	}
}

void sendData()
{
	int size = sizeof(sampleBuffer);
	for(int i = 0; i < 10; i++)
	{
		CDC_Send((uint8_t*)sampleBuffer, size);
	}
}

/* void CDC_Send(uint8_t* data, uint16_t len)
 *
 * safely uses CDC_Transmit_FS(...)
 *	 													*/
static void CDC_Send(uint8_t* data, uint16_t len)
{
	while (CDC_Transmit_FS(data, len) == USBD_BUSY)
	{
		// wait until USB stack is ready
	}
}

static void receiveCommand()
{
	// wait for command reception
	while(usbRxFlag == 0) {}

	for (uint32_t i = 0; i < usbRxBufLen; i++)
	{
		uint8_t ch = usbRxBuf[i];

		// check for termination character
		if (ch == '\n' || ch == '\r') {
			if (cmdRxIndex > 0) {
			cmdRxBuffer[cmdRxIndex] = '\0'; // NULL-terminate

			// process the received command:
			processCommand((char*)cmdRxBuffer);

			cmdRxIndex = 0; // reset buffer
			}
		}
		else
		{
			if (cmdRxIndex < CMD_RX_BUFFER_SIZE - 1)
			{
				// insert character into buffer
				cmdRxBuffer[cmdRxIndex++] = ch;
			}
			else
			{
				// buffer overflow -> reset
				cmdRxIndex = 0;
			}
		}
	}

	usbRxFlag = 0;
	usbRxBufLen = 0;
}

/* static void processCommand(const char* cmd)
 *
 * receives NULL-terminated (\0) string with the
 * corresponding command sent to the µC from host (PC)
 *	 													*/
static void processCommand(const char* cmd)
{
    if (strcmp(cmd, "RESET") == 0) {
    	fsm_add_event(EV_RESET);
    }
	else if (strcmp(cmd, "APP") == 0) {
    	fsm_add_event(EV_START_APP);
    }
    else if (strcmp(cmd, "RUN_START") == 0) {
    	fsm_add_event(EV_SAMPL_RUN_START);
    }
    else if (strcmp(cmd, "READY_FOR_DATA") == 0) {
        	fsm_add_event(EV_DATA_TRANSMIT); // change to EV_SAMPLE_TRANSMIT
    }
    else if (strcmp(cmd, "RECEIVED") == 0) {
    	fsm_add_event(EV_SAMPL_RECEIVED);
    }
    else if (strcmp(cmd, "CONTINUE") == 0) {
    	fsm_add_event(EV_SAMPL_ACK_CONTINUE);
    }
    else if (strcmp(cmd, "STOP") == 0) {
    	fsm_add_event(EV_SAMPL_ACK_STOP);
    }
    else {
        const char* error = "ERR:UNKNOWN_CMD\n";
        CDC_Transmit_FS((uint8_t*)error, strlen(error));
    }
}

/* --- SYSTEM INITIALIZATION --- */

/* static void GPIO_Init()
 *
 * inititalization of the used GPIOs
 *	 													*/
static void GPIO_Init()
{
	/* GPIO for button to manually start sampling
	 * PC13: USR_Btn B1 (NUCLEO-F767)*/
	/* enable clock for GPIOC */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	// USR_Btn->input (MODE: 0b00)
	GPIOC->MODER &= ~GPIO_MODER_MODER13_Msk;


	/* GPIO for onboard status LEDs (LD1..3 of NUCLEO-F767)
	 * PB0:  LD1 (green)
	 * PB7:  LD2 (blue)
	 * PB14: LD3 (red) */
	/* enable clock for GPIOB */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	// turn LD1..3 off
	GPIOB->ODR &= ~(LD1_Pin|LD2_Pin|LD3_Pin);
	// LD1..3->output (MODE: 0b01)
	GPIOB->MODER &= ~(GPIO_MODER_MODER0_Msk|GPIO_MODER_MODER7_Msk|GPIO_MODER_MODER14_Msk);
	GPIOB->MODER |= (GPIO_MODER_MODER0_0|GPIO_MODER_MODER7_0|GPIO_MODER_MODER14_0);
}

/* static void EXTI_Init()
 *
 * initialization and configuration of EXTI-modules
 *	 													*/
static void EXTI_Init()
{
	// enable SYSCFG-clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* EXTI13 Configuration for USR_Btn B1 (PC13) */
	// SysCfg configuration EXTI13->PC13
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
	// unmask EXTI13
	EXTI->IMR |= EXTI_IMR_MR13;
	// enable rising edge detection for EXTI13 (USR_Btn is high-active)
	EXTI->RTSR |= EXTI_RTSR_TR13;
	// NVIC IRQ config for EXTI 10-15
	NVIC_SetPriority(EXTI15_10_IRQn, 15);
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* static void TIM3_Init()
 *
 * initalization of TIM3
 * TIM3 for debouncing USR-Btn
 *	 													*/
static void TIM3_Init()
{
	// enable TIM3-clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// set clock with prescaler to clock_cnt=clock_in/(PSC+1)
	uint16_t prescaleValueTIM3 = 49999;
	TIM3->PSC |= prescaleValueTIM3;
	// APB1-TimerClk is 100MHz -> 100MHz/(49999+1)=2kHz-> Timer-Clock is 2kHz

	// set auto-reload value
	TIM3->ARR &= ~(0xFFFFFFFF);
	uint16_t TIM3_ticks = 10;
	TIM3->ARR |= TIM3_ticks;      // count time: 5ms

	// enable update interrupt-flag
	TIM3->DIER |= (TIM_DIER_UIE);

	// activate upward counter (default) and one-pulse mode
	TIM3->CR1 |= (TIM_CR1_OPM);

	// NVIC IRQ config
	NVIC_SetPriority(TIM3_IRQn, 1);
	NVIC_ClearPendingIRQ(TIM3_IRQn);
	NVIC_EnableIRQ(TIM3_IRQn);
}
