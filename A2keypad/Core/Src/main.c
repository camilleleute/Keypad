/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

static void Keypad_TurnOnCols(void) {
	GPIOC->ODR |= GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12;
}

static void Keypad_TurnOffCols(void) {
	GPIOC->ODR &= ~(GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12);
}

void LED_Init(void) {
	// Turn on clock for GPIOC
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);
	// Configure LEDs
	GPIOC->MODER &= ~(
			GPIO_MODER_MODE0
			| GPIO_MODER_MODE1
			| GPIO_MODER_MODE2
			| GPIO_MODER_MODE3
		);
	GPIOC->MODER |=
			(1 << GPIO_MODER_MODE0_Pos)
			| (1 << GPIO_MODER_MODE1_Pos)
			| (1 << GPIO_MODER_MODE2_Pos)
			| (1 << GPIO_MODER_MODE3_Pos);
	GPIOC->OTYPER &= ~(
			GPIO_OTYPER_OT0
			| GPIO_OTYPER_OT1
			| GPIO_OTYPER_OT2
			| GPIO_OTYPER_OT3
		);
	GPIOC->OSPEEDR |=
			GPIO_OSPEEDR_OSPEED0
			| GPIO_OSPEEDR_OSPEED1
			| GPIO_OSPEEDR_OSPEED2
			| GPIO_OSPEEDR_OSPEED3;
	GPIOC->PUPDR &= ~(
			GPIO_PUPDR_PUPD0
			| GPIO_PUPDR_PUPD1
			| GPIO_PUPDR_PUPD2
			| GPIO_PUPDR_PUPD3
		);
}

// Cols: PC10-12, Rows: PB13, 14, 15, 1
void Keypad_Init(void) {
	  // Turn on clock for GPIOB and GPIOC
	  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);

	  // Configure cols (PC10-12) for output
	  GPIOC->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11 | GPIO_MODER_MODE12);
	  GPIOC->MODER |=
			  (1 << GPIO_MODER_MODE10_Pos)
			  | (1 << GPIO_MODER_MODE11_Pos)
			  | (1 << GPIO_MODER_MODE12_Pos);
	  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11 | GPIO_OTYPER_OT12);
	  GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED11 | GPIO_OSPEEDR_OSPEED12);
	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD10 | GPIO_PUPDR_PUPD11 | GPIO_PUPDR_PUPD12);

	  // Configure rows (PB13-15,1) for input
	  GPIOB->MODER &= ~(
			  GPIO_MODER_MODE13
			  | GPIO_MODER_MODE14
			  | GPIO_MODER_MODE15
			  | GPIO_MODER_MODE1
		  );

	  // Use pull down resistors on rows.
	  GPIOB->PUPDR &= ~(
			  GPIO_PUPDR_PUPD13
			  | GPIO_PUPDR_PUPD14
			  | GPIO_PUPDR_PUPD15
			  | GPIO_PUPDR_PUPD1
		  );
	  GPIOB->PUPDR |=
			  GPIO_PUPDR_PUPD13_1
			  | GPIO_PUPDR_PUPD14_1
			  | GPIO_PUPDR_PUPD15_1
			  | GPIO_PUPDR_PUPD1_1;

	  Keypad_TurnOnCols();
}

int8_t Keypad_CalculateButton(int8_t row, int8_t col) {
	if (row <= 2) {
		return row * 3 + col + 1;
	}
	// Special cases for bottom row
	if (col == 0) return 10;
	if (col == 1) return 0;
	if (col == 2) return 11;
	return -1;
}

int8_t Keypad_GetButton(void) {
	// Return early if no buttons are pressed
	if (!(GPIOB->IDR & (GPIO_IDR_ID13 | GPIO_IDR_ID14 | GPIO_IDR_ID15 | GPIO_IDR_ID1))) {
		return -1;
	}
	for (int8_t col = 0; col <= 2; col++) {
		// Turn on only the specific column being tested
		Keypad_TurnOffCols();
		GPIOC->ODR |= 1 << (GPIO_ODR_OD10_Pos + col);
		// Check if any buttons are pressed in the column
		int8_t row = -1;
		uint32_t row_pins = GPIOB->IDR;
		if (row_pins & GPIO_IDR_ID13) row = 0;
		if (row_pins & GPIO_IDR_ID14) row = 1;
		if (row_pins & GPIO_IDR_ID15) row = 2;
		if (row_pins & GPIO_IDR_ID1) row = 3;
		// If none are pressed, keep going
		if (row == -1) {
			continue;
		}
		// Turn on all columns before returning
		Keypad_TurnOnCols();
		return Keypad_CalculateButton(row, col);
	}
	// Turn on all columns before returning
	Keypad_TurnOnCols();
	return -1;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  Keypad_Init();
  LED_Init();

  while (1)
  {
	  int8_t button = Keypad_GetButton();
	  if (button == -1) continue;
	  // Show button being pressed on LEDs
	  GPIOC->ODR &= ~(GPIO_ODR_OD0 | GPIO_ODR_OD1 | GPIO_ODR_OD2 | GPIO_ODR_OD3);
	  GPIOC->ODR |= button;
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
