/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */


	RCC->AHBENR |= (1 << 19); // Enable peripheral clock for port C (19th bit is set)
	//RCC->AHBENR |= 0x00020000; // Enable peripheral clock for port A (17th bit is set)

	// Set LED pins PC9 to PC6 to 01 (General purpose output mode)
	GPIOC->MODER = 0;
	GPIOC->MODER |= (1 << 18) | (1 << 16) | (1 << 14) | (1 << 12); // Set MODER9-6 to 0101
	
	// Set pins PC9 to PC6 to push-pull output type in the OTYPER reg
	GPIOC->OTYPER = 0;
	
	// Set pins to low speed in OSPEEDR
	GPIOC->OSPEEDR = 0; // Set to clear all bits such that pins are set to 00, low speed
	
	// Set pins to no pull-up/down resistors in the PUPDR reg
	GPIOC->PUPDR = 0; // Set clear all bits such that pins are set to 00, no pull-up, pull-down
	
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Start PC8 high
	GPIOC->ODR = (1 << 9); // Set ODR9 to be high
	
	// USER BUTTON config
	GPIOA->MODER &= ~((1 << 0) | (1 << 1)); // Set MODER0 on port A to Input Mode
	GPIOA->OSPEEDR &= ~((1 << 0)); // Set speed to Low OSPEEDR0
	GPIOA->PUPDR = (1 << 1); // Set to pull-down resistor enabled
	
	
	
	// CONFIGURE EXTI0, ROUTE TO PA0, ENABLE AND SET PRIORITY OF EXTI INTERUPT
	// Setup EXTI0 (enables interupts from PA0)
	EXTI->IMR |= 1; // Unmasked/Enabled
	EXTI->RTSR |= 1; // Rising-trigger enabled
	
	// Setup SYSCFG
	RCC->APB2ENR |= 1; // Substep: enable RCC to SYSCFG
	SYSCFG->EXTICR[0] &= ~((1 << 2) | (1 << 1) | (1 << 0)); // Route EXTI0 to PA0
	
	
	// Enable and Set Priority of EXTI Interupt
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	NVIC_SetPriority(EXTI0_1_IRQn, 1);
	
	
	// WRITING THE EXTI INTERUPT HANDLER
	// (see function just below main)
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
		HAL_Delay(400);
		
		GPIOC->ODR ^= (1 << 6); // Invert PC6 (Red LED)
  }
  /* USER CODE END 3 */
}


// EXTI0 and 1 interupt handler
void EXTI0_1_IRQHandler()
{
	// Toggle Green and Orange LEDs
	GPIOC->ODR ^= (1 << 8) | (1 << 9); // Invert ODR8 and ODR9, toggles PC8 PC9
	
	// Ensure this interupt is no longer pending
	EXTI->PR |= 1; // write 1 to bit 0, which clears the event on line 0
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
