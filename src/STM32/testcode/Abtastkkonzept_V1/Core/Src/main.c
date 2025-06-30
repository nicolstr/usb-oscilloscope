/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
volatile uint16_t samplingDataBuffer[100000] = {0};
volatile uint32_t bufferSize;
volatile uint16_t overflowBuffer[100] = {0};
volatile uint32_t* addressArray[5][2] = {0};
volatile uint32_t numOfRows;
volatile uint8_t dataReady = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */
void ClockInit();
void GPIO_Init();
void EXTI_Init();
void ADC_Init();
void TIM3_Init();
void TIM1_Init();
void DMA_Init(uint32_t numOfTransfers);
int determineBufferAdresses(uint16_t buffer[], uint32_t bufferSize, uint32_t numOfRows, uint32_t* addressArray[][2]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  // disable Interrupts globally
  __disable_irq();

  ClockInit();
  GPIO_Init();
  EXTI_Init();
  TIM3_Init();
  TIM1_Init();

  // set the buffersize
  bufferSize = sizeof(samplingDataBuffer)/sizeof(samplingDataBuffer[0]);
  // calculate the number of rows of the addressArray
  numOfRows = sizeof(addressArray)/sizeof(addressArray[0]);
  // fill the buffer (and overflow buffer) with ones
  for(int i = 0; i < bufferSize; i++)
	  samplingDataBuffer[i] = 0xFFFF;
  for(int i = 0; i < 100; i++)
	  overflowBuffer[i] = 0xFFFF;
  // determine all values for the addressArray
  determineBufferAdresses(samplingDataBuffer, bufferSize, numOfRows, addressArray);

  // Initialize the DMA
  DMA_Init(bufferSize/(numOfRows*2));

  // enable Interrupts globally
  __enable_irq();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void ClockInit()
{
	uint32_t SysClockFreq = HAL_RCC_GetSysClockFreq();
	uint32_t APB1Freq = HAL_RCC_GetPCLK1Freq();
	uint32_t APB2Freq = HAL_RCC_GetPCLK2Freq();
	uint32_t AHBFreq = HAL_RCC_GetHCLKFreq();
}

/* function: inititalization of the used GPIOs */
void GPIO_Init()
{
	/* GPIO for TIMer output compare event
	 * PE9: TIM1_CH1 (AF1)*/
	/* enable clock for GPIOE */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	// PE9->alternate function (MODE: 0b10)
	GPIOE->MODER &= ~GPIO_MODER_MODER9_Msk;
	GPIOE->MODER |= GPIO_MODER_MODER9_1;
	// output speed (very high speed: 0b11)
	GPIOE->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR9_1|GPIO_OSPEEDR_OSPEEDR9_0);
	// select alternate function AF1 of PE9 (TIM1_CH1)
	GPIOE->AFR[1] &= ~(GPIO_AFRH_AFRH1);
	GPIOE->AFR[1] |= GPIO_AFRH_AFRH1_0;


	/* GPIO for parallel interface (only input data!)
	 * Port G (PG0..15)*/
	/* enable clock for GPIOG */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
	// all GPIOG-Pins except PG6 and PG7 -> input (MODE: 0b00)
	// PG6 and PG7 are used for the USB OTG FS interface
	GPIOG->MODER &= ~(0xFFFF0FFF);
	// assign pull-downs for defined value of pins
	GPIOG->PUPDR &= ~(0xFFFF0FFF);
	GPIOG->PUPDR |= GPIO_PUPDR_PUPDR15_1|GPIO_PUPDR_PUPDR14_1|GPIO_PUPDR_PUPDR13_1|
					GPIO_PUPDR_PUPDR12_1|GPIO_PUPDR_PUPDR11_1|GPIO_PUPDR_PUPDR10_1|
					GPIO_PUPDR_PUPDR9_1|GPIO_PUPDR_PUPDR8_1|GPIO_PUPDR_PUPDR5_1|
					GPIO_PUPDR_PUPDR4_1|GPIO_PUPDR_PUPDR3_1|GPIO_PUPDR_PUPDR2_1|
					GPIO_PUPDR_PUPDR1_1|GPIO_PUPDR_PUPDR0_1;


	/* GPIO for button to manually starting sampling
	 * PC13: USR_Btn (NUCLEO-F767)*/
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

// initialization and configuration of EXTI-modules
void EXTI_Init()
{
	// enable SYSCFG-clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* EXTI13 Configuration for USR_Btn (PC13) */
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

/* function: inititalization of the used ADC */
void ADC_Init()
{
	/* ADC to sample analog signals with rate configured in TIM_Init() */
	/* ADC characteristics: DS p.164 */

	// internal ADC currently not used !!!
}

/* function: initalization of TIM3
 * TIM3 for debouncing USR-Btn */
void TIM3_Init()
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

/* function: initalization of TIM1
 * TIMer to control the sampling rate / DMA transfer of the ADC	 */
void TIM1_Init()
{
	// determine APB2-Clk
	//uint32_t Sysfreq = HAL_RCC_GetSysClockFreq();
	//uint32_t APB1freq = HAL_RCC_GetPCLK1Freq();
	//uint32_t APB2freq = HAL_RCC_GetPCLK2Freq();

	/* create ADC-Clock (10MHz) at OC-Pin PE9 (TIM1_CH1)
	 * used timer: TIM1 (APB2)
	 * clock: APB2 Timer Clock: 100MHz */
	// enable TIM1-clock
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	// set clock with prescaler to clock_cnt=clock_in/(PSC+1)
	uint16_t prescaleValueTIM1 = 0;
	TIM1->PSC = prescaleValueTIM1;
	// APB1-TimerClk is 100MHz -> 100MHz/(0+1)=100MHz-> Timer-Clock is 100MHz

	// set auto-reload value
	TIM1->ARR &= ~(0xFFFF);
	uint16_t TIM1_ticks = 9;
	TIM1->ARR |= TIM1_ticks; 	// period: (9+1)ticks = 100ns
	// set Capture/Compare register value
	TIM1->CCR1 &= ~(0xFFFF);
	TIM1->CCR1 |= 5; 			// HIGH for 50ns (duty-cycle=50%)
	// capture mode enabled, active high and configure channel as output
	TIM1->CCER |= TIM_CCER_CC1E;
	// PWM mode 1 (active: CNT<CCR1, else inactive) (mode: 0b0110)
	TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M);
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1M_1;
	// CC1 channel configured as output (0b00)
	TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S);

	// Main output enable (enables OC1)
	TIM1->BDTR |= TIM_BDTR_MOE;

	// enable update dma request (used for transferring GPIOG_IDR->SRAM)
	TIM1->DIER |= (TIM_DIER_UDE);

	// enable TIM1
	TIM1->CR1 |= TIM_CR1_CEN;
}

// determines all adresses needed in the DMA transfers for the complete buffer
// returns the adresses in a <numOfElements>x2-Array (adress is zero, if invalid)
// used for the memory pointers M0AR (first column) and M1AR (second column) of DMA2
int determineBufferAdresses(uint16_t buffer[], uint32_t bufferSize, uint32_t numOfRows, uint32_t* addressArray[][2])
{
	uint32_t index = 0;
	uint32_t addressArraySize = numOfRows*2;
	uint32_t partSize = bufferSize/(addressArraySize);
	// if the buffer can't be separated into (numOfRows*2) equal parts -> error
	if(bufferSize%(addressArraySize) != 0)
		return 1;
	for(int i = 0; i < numOfRows; i++)
	{
		for(int j = 0; j < 2; j++)
		{
			addressArray[i][j] = (uint32_t*)&buffer[index];
			index += partSize;
		}
	}
	return 0;
}

/* function: inititalization of the used DMA */
void DMA_Init(uint32_t numOfTransfers)
{
	/* DMA for transferring data between GPIOG_IDR (input data register) and SRAM
	 * also triggered by TIMer
	 * used DAM-Controller: DMA2
	 * channel: 6
	 * stream: 5 (TIM1_UP)*/

	// enable DMA2 clock
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	/* following procedure in RM p.267 */

	// 1. disable stream2 of DMA2 & clear all stream-dedicated status-bits
	DMA2_Stream5->CR &= ~(DMA_SxCR_EN);
	while(DMA2_Stream5->CR & DMA_SxCR_EN) {}
	DMA2->HIFCR |= (DMA_HIFCR_CTCIF5|DMA_HIFCR_CHTIF5|DMA_HIFCR_CTEIF5|DMA_HIFCR_CDMEIF5|DMA_HIFCR_CFEIF5);

	// 2. set peripheral port (periph2mem: source) address
	// source address is GPIOG_IDR
	DMA2_Stream5->PAR = (uint32_t)&GPIOG->IDR;

	// 3. set memory (periph2mem: destination) addresses
	// initialized with entries of the first row of addressArray
	uint32_t initialM0AR = addressArray[0][0];
	uint32_t initialM1AR = addressArray[0][1];
	DMA2_Stream5->M0AR = initialM0AR;
	DMA2_Stream5->M1AR = initialM1AR;

	// 4. configure total number of transferred items
	DMA2_Stream5->NDTR &= ~(0xFFFF);
	DMA2_Stream5->NDTR |= numOfTransfers;

	// 5. select DMA channel 6
	DMA2_Stream5->CR &= ~(DMA_SxCR_CHSEL);
	DMA2_Stream5->CR |= DMA_SxCR_CHSEL_2|DMA_SxCR_CHSEL_1;

	// 6. (only if peripheral=flow-controller)

	// 7. configure stream priority (0b11:highest - 0b00:lowest)
	DMA2_Stream5->CR &= ~(DMA_SxCR_PL);
	DMA2_Stream5->CR |= (DMA_SxCR_PL_1|DMA_SxCR_PL_0);	// highest priority

	// 8. configure FIFO usage (default: direct mode (DMDIS=0))
	DMA2_Stream5->FCR &= ~(DMA_SxFCR_DMDIS);

	// 9.
	// data transfer direction (0b00: periph2mem)
	DMA2_Stream5->CR &= ~(DMA_SxCR_DIR);
	//DMA2_Stream5->CR |= 0;

	// peripheral (source) increment mode (PINC=0: no increment - fixed address)
	DMA2_Stream5->CR &= ~(DMA_SxCR_PINC);

	// memory (destination) increment mode (MINC=1: according to MSIZE)
	DMA2_Stream5->CR |= DMA_SxCR_MINC;

	// XSIZE[1:0]: (0b00: byte, 0b01: half-word, 0b10: word)
	// peripheral (source) data size
	DMA2_Stream5->CR &= ~(DMA_SxCR_PSIZE);
	DMA2_Stream5->CR |= (DMA_SxCR_PSIZE_0);	// half-word

	// memory (destination) data size
	DMA2_Stream5->CR &= ~(DMA_SxCR_MSIZE);
	DMA2_Stream5->CR |= (DMA_SxCR_MSIZE_0);	// half-word

	/* enable double-buffer mode (circular mode is automatically enabled)
	* after one transaction M0AR and M1AR are switched as the destination address
	* while M0AR is active, M1AR can be changed (and vice versa)
	* CR_CT=0 -> M0AR used;	CR_CT=1 -_>M1AR used	(CT: current target) */
	DMA2_Stream5->CR |= (DMA_SxCR_DBM);

	// enable transfer error interrupt
	DMA2_Stream5->CR |= DMA_SxCR_TCIE;
	// enable transfer complete interrupt
	DMA2_Stream5->CR |= DMA_SxCR_TEIE;

	// configure NVIC for DMA2 stream5 interrupts
	NVIC_SetPriority(DMA2_Stream5_IRQn, 0);
	NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);
	NVIC_EnableIRQ(DMA2_Stream5_IRQn);


	// DMA2 stream 5 is enabled, when trigger condition is met
	// currently in another file in the TIM3 Exc. Handler
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
