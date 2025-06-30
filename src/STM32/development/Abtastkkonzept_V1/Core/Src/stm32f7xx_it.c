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
extern volatile uint16_t samplingDataBuffer[100000];
extern volatile uint32_t bufferSize;
extern volatile uint16_t overflowBuffer[100];
extern volatile uint32_t* addressArray[5][2];
extern volatile uint32_t numOfRows;
extern volatile uint8_t dataReady;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

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

/* USER CODE BEGIN 1 */

/* exception handler for TIM3 */
void TIM3_IRQHandler()
{
	TIM3->SR &= ~(1 << 0);  	// clear TIM3 pending flag (UIF)

	if(GPIOC->IDR & USER_Btn_Pin)
	{
		// Test Call by toggling LD2
		GPIOB->ODR ^= LD1_Pin|LD2_Pin|LD3_Pin;

		// enable DMA2Stream5
		DMA2_Stream5->CR |= DMA_SxCR_EN;
	}
	// note:
	// clearing UIF at end causes problem, because of pipelining architecture
	// in combination with tailchaining mechanism (ISR is incorrectly called again)
}

/* exception handler for EXTI15_10 */
void EXTI15_10_IRQHandler()
{
	EXTI->PR |= EXTI_PR_PR13;		// clear EXTI13 pending flag

	TIM3->CR1 |= TIM_CR1_CEN;       // enable Counter TIM4
}

/* exception handler for DMA2 Stream5 */
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
			if(M0AR_counter < numOfRows)
				DMA2_Stream5->M0AR = addressArray[M0AR_counter][0];
			else
				DMA2_Stream5->M0AR = &overflowBuffer;
				// Buffer is filled, until DMA2 stream 5 is eventually disabled
		}
		else								// CT=0 ->M0AR
		{
			M1AR_counter++;	// PAR->M1AR was just finished
			if(M1AR_counter < numOfRows)
				DMA2_Stream5->M1AR = addressArray[M1AR_counter][1];
			else
				DMA2_Stream5->M1AR = &overflowBuffer;	// not necessary
				// just for symmetric reasons

		}
	}
	if(M1AR_counter >= numOfRows)	// last transaction of PAR->M1AR just happened
	{
		DMA2_Stream5->CR &= ~(DMA_SxCR_EN);		// disable DMA2 stream 5
		while(DMA2_Stream5->CR & DMA_SxCR_EN) {}

		// set flag, that data is ready
		dataReady = 1;

		// reset counters
		M0AR_counter = 0;
		M1AR_counter = 0;

		// reinitialize DMA2 stream 5
		DMA_Init(bufferSize/(numOfRows*2));
	}
}
/* USER CODE END 1 */
