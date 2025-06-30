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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */
void ClockOutputInit();
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  ClockOutputInit();
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
  RCC_OscInitStruct.PLL.PLLN = 96;
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
void ClockOutputInit()
{
	uint32_t SysClockFreq = HAL_RCC_GetSysClockFreq();
	uint32_t APB1Freq = HAL_RCC_GetPCLK1Freq();
	uint32_t APB2Freq = HAL_RCC_GetPCLK2Freq();
	uint32_t AHBFreq = HAL_RCC_GetHCLKFreq();

	/* explanations of some register values in RCC-module (reset and clock control)
	 * note: these are the default values used by creating a project in STM32CubeIDE 1.16.0
	 */

	// RCC clock control register - enables PLLs, and clock sources (HSE, HSI, ...):
	// RM p.164
	int32_t RCC_clock_control_register = RCC->CR;
	// PLLRDY=1, PLLON=1
	// 	-main PLL enabled and/or locked
	// HSERDY=1, HSEON=1, HSEBYP=0
	// 	-HSE (external quartz) oscillator ready and/or ON (can be used for SysClk or as input for PLL)
	//	-HSE oscillator can be bypassed with external clock (HSEBYP=1)
	// HSIRDY=1, HSION=1
	// 	-HSI (internal RC-) oscillator ready and/or ON (can be used for SysClk or as input for PLL)
	// 	-HSI takes over in case of leaving Stop or Standby mode or in cae of HSE-failure
	//	-value can be trimmed to adjust to variations in voltage in temperature

	// PLL (phase-locked loop) and PLL-prescaler configurations:
	// RM p.166
	uint32_t RCC_PLL_configuration_register = RCC->PLLCFGR;
	// PLLSRC=1: PLL clock source is HSE
	// PLLP=0b00 (equals PLLP=2)
	// PLLN=0b001100000=96
	// PLLM=0b000100=4
	// f_PLLout = f_Vco   / PLLP			= 96MHz		(=SysClk)
	// f_Vco    = f_PLLin * (PLLN / PLLM)	= 192MHz

	// APB, AHB prescaler and SysClk-selection:
	// RM p.169
	uint32_t RCC_clock_configuration_register = RCC->CFGR;
	// SWS=0b10, SW=0b10
	// 	-PLL (96MHz) used as SysClock
	// PPRE1=0b100
	// 	-APB1 prescaler: div by 2		->PLCK1 = 48MHz
	// PPRE2=0b000
	// 	-APB2 prescaler: no division	->PLCK2 = 96MHz
	// HPRE=0b0000
	// 	-AHB prescaler: no division		->HCLK	= 96MHz


	/* output clock configurations */

	/* 1. It is possible to directly output certain clocks
	 * via the alternate function of a GPIO-pin.
	 * these pins are MCO1 and MCO2 (microcontroller clock output)
	 * MCO1: AF0 of PA8 (DS p.89)
	 * MCO2: AF0 of PC9 (DS p.92)
	 */
	// enable clocks of GPIOA, GPIOC
	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOCEN);
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// select AF0 of PA8: MCO1
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH0);
	// select AF= of PC9: MCO2
	GPIOC->AFR[1] &= ~(GPIO_AFRH_AFRH1);

	// activate AF-mode (0b10)
	GPIOA->MODER &= ~(GPIO_MODER_MODER8);
	GPIOA->MODER |= (GPIO_MODER_MODER8_1);
	GPIOC->MODER &= ~(GPIO_MODER_MODER9);
	GPIOC->MODER |= (GPIO_MODER_MODER9_1);

	// select clock source and prescale value of MOC1,MOC2
	RCC->CFGR &= ~(RCC_CFGR_MCO1|
				   RCC_CFGR_MCO1PRE|
				   RCC_CFGR_MCO2|
				   RCC_CFGR_MCO2PRE);
	RCC->CFGR |= (RCC_CFGR_MCO1_1|
			   	  RCC_CFGR_MCO1PRE_2|
				  RCC_CFGR_MCO1PRE_1|
				  RCC_CFGR_MCO2PRE_2|
				  RCC_CFGR_MCO2PRE_1|
				  RCC_CFGR_MCO2PRE_0);
	// MCO1: HSE (division by 4) -> 2MHz, MCO2: SysClk (division by 5) -> 19.2MHz

	/* 2. use timers to derive a frequency out of APBX clock */
	// TIM10: 16-bit auto-reload upcounter RM p.1026
	/* using output-compare and auto-reload to deliver output signal on OC1 of timer
	 * which can be redirected to a GPIO-pin, when the alternate function is activated
	 * TIM10_CH1 (connected to OC1 of the timer): AF3 of PB8 (DS p.91)*/

	// enable TIM10 and GPIOB clock
	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	// select AF3 of PB8: TIM10_CH1
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFRH0);
	GPIOB->AFR[1] |= (GPIO_AFRH_AFRH0_1|GPIO_AFRH_AFRH0_0);
	// activate AF-mode (0b10)
	GPIOB->MODER &= ~(GPIO_MODER_MODER8);
	GPIOB->MODER |= (GPIO_MODER_MODER8_1);
	// activate very fast speed for GPIO-pin (OSPEED=0b11)
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR8);
	GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR8_1|GPIO_OSPEEDR_OSPEEDR8_0);

	// configure TIM10 prescaler f_TIM = f_APB2/(PSC+1) = 96MHz/(5+1)=16MHz
	TIM10->PSC |= 5;
	// configure output frequency count to/toggle at CCR1=1 -> f_OC1 = f_TIM / ((CCR1+1)*2) = 4MHz;
	TIM10->ARR &= ~(0xFFFF);
	TIM10->ARR |= 1;
	TIM10->CCR1 &= ~(0xFFFF);
	TIM10->CCR1 |= 1;

	// configure TIM10 in output compare mode (CC1S=0b00)
	TIM10->CCMR1 &= ~(TIM_CCMR1_CC1S);
	// OC1 is toggled, when TIM10_CNT=TIM10_CCR1 (OC1M=0b0011)
	TIM10->CCMR1 &= ~(TIM_CCMR1_OC1M);
	TIM10->CCMR1 |= (TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_0);
	// enable Compare 1 output
	TIM10->CCER |= TIM_CCER_CC1E;

	// enable  TIM10
	TIM10->CR1 |= TIM_CR1_CEN;

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
