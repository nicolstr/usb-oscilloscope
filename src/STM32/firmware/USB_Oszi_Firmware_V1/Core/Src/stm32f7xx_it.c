/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fsm.h"
#include "app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
/* USER CODE BEGIN EV */
extern volatile uint16_t sampleBuffer[NUMBER_OF_ROWS*NUMBER_OF_COLUMNS*NUM_OF_TRANSFERS_PER_TRANSACTION];	// from file "app.c"
extern volatile uint16_t overflowBuffer[1000];													// from file "app.c"
extern volatile uint32_t* addressArray[NUMBER_OF_ROWS][NUMBER_OF_COLUMNS];		// from file "app.c"
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* void TIM3_IRQHandler()
 *
 * exception handler for TIM3
 *
 * used to debounce USR_Btn B1 (PC13)
 *	 													*/
void TIM3_IRQHandler()
{
	TIM3->SR &= ~(1 << 0);  	// clear TIM3 pending flag (UIF)

	if(GPIOC->IDR & USER_Btn_Pin)
	{
		// enqueue event EV_B1_PRESSED
		fsm_add_event(EV_B1_PRESSED);

	}
	// note:
	// clearing UIF at end causes problem, because of pipelining architecture
	// in combination with tailchaining mechanism (ISR is incorrectly called again)
}

/* void EXTI15_10_IRQHandler()
 *
 * exception handler for EXTI15_10
 *
 * executed, when USR_Btn B1 (PC13) is pushed
 *	 													*/
void EXTI15_10_IRQHandler()
{
	EXTI->PR |= EXTI_PR_PR13;		// clear EXTI13 pending flag

	TIM3->CR1 |= TIM_CR1_CEN;       // enable Counter TIM4
}

/* void EXTI2_IRQHandler()
 *
 * exception handler for EXTI15_10
 *
 * executed, when Trigger condition is met
 *	 													*/
void EXTI2_IRQHandler()
{
	EXTI->PR |= EXTI_PR_PR2;		// clear EXTI2 pending flag

	if( ((EXTI->RTSR & EXTI_RTSR_TR2) && (GPIOF->IDR & GPIO_IDR_ID2)) ||	// rising edge was detected
		((EXTI->FTSR & EXTI_FTSR_TR2) && !(GPIOF->IDR & GPIO_IDR_ID2)) )	// falling edge was detected
	{
		disableTrigger();
		/* enable DMA transfer from ADC parallel interface */
		// enable DMA2Stream5
		//if(!(DMA2_Stream5->CR & DMA_SxCR_EN))	// only start, if DMA is currently disabled
				DMA2_Stream5->CR |= DMA_SxCR_EN;
	}
}

/* DMA2_Stream5_IRQHandler()
 *
 * exception handler for DMA2 Stream5
 *
 * executed, when NDTR (number of transfers) is reached
 *	 													*/
void DMA2_Stream5_IRQHandler()
{
	static uint32_t M0AR_counter = 0;
	static uint32_t M1AR_counter = 0;

	if(DMA2->HISR & DMA_HISR_TEIF5)			// transfer error interrupt
	{
		DMA2->HIFCR |= DMA_HIFCR_CTEIF5;	// clear transfer error Interrupt Flag
	}
	else if(DMA2->HISR & DMA_HISR_TCIF5)	// transfer complete interrupt
	{
		DMA2->HIFCR |= DMA_HIFCR_CTCIF5;	// clear transfer complete Interrupt Flag
		// memory pointers (M0AR and M1AR) were swapped
		// check current target (CT)
		if(DMA2_Stream5->CR & DMA_SxCR_CT)	// CT=1 -> M1AR
		{
			M0AR_counter++;	// PAR->M0AR was just finished
			if(M0AR_counter < NUMBER_OF_ROWS)
				DMA2_Stream5->M0AR = addressArray[M0AR_counter][0];
			else
				DMA2_Stream5->M0AR = (uint32_t)&overflowBuffer;
				// Buffer is filled, until DMA2 stream 5 is eventually disabled
		}
		else								// CT=0 ->M0AR
		{
			M1AR_counter++;	// PAR->M1AR was just finished
			DMA2_Stream5->M1AR = addressArray[M1AR_counter][1];
		}
	}
	if(M1AR_counter >= 1)	// last transaction of PAR->M1AR just happened
	{
		DMA2_Stream5->CR &= ~(DMA_SxCR_EN);		// disable DMA2 stream 5
		while(DMA2_Stream5->CR & DMA_SxCR_EN) {}

		// sampling is done --> enqueue event EV_SAMPL_DONE
		fsm_add_event(EV_SAMPL_DONE);

		// reset counters and address pointers
		M0AR_counter = 0;
		M1AR_counter = 0;
		DMA2_Stream5->M0AR = addressArray[M0AR_counter][0];
		DMA2_Stream5->M1AR = addressArray[M1AR_counter][1];
	}
}

/* USER CODE END 1 */
