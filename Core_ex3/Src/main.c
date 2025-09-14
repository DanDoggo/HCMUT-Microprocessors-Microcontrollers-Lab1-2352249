/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
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
/*
 * Define human-readable names for the traffic light pins.
 * The original pin names are mapped to a logical 4-way intersection.
 *
 * North Traffic Light
 */
#define NORTH_RED_Pin 		LED_REDA3_Pin
#define NORTH_YELLOW_Pin 	LED_YELLOWA4_Pin
#define NORTH_GREEN_Pin 	LED_GREENA5_Pin

/*
 * South Traffic Light
 */
#define SOUTH_RED_Pin 		LED_REDA6_Pin
#define SOUTH_YELLOW_Pin 	LED_YELLOWA7_Pin
#define SOUTH_GREEN_Pin 	LED_GREENA8_Pin

/*
 * East Traffic Light
 */
#define EAST_RED_Pin 		LED_REDA9_Pin
#define EAST_YELLOW_Pin 	LED_YELLOWA10_Pin
#define EAST_GREEN_Pin 	    LED_GREENA11_Pin

/*
 * West Traffic Light
 */
#define WEST_RED_Pin 		LED_RED_Pin
#define WEST_YELLOW_Pin 	LED_YELLOW_Pin
#define WEST_GREEN_Pin 	    LED_GREEN_Pin
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*
     * PHASE 1: North/South are green, East/West are red.
     * Duration: 3 seconds.
     * All green LEDs are controlled by their negative pin (low is on).
     */
    // North/South: GREEN on
    HAL_GPIO_WritePin(GPIOA, NORTH_GREEN_Pin | SOUTH_GREEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, NORTH_RED_Pin | NORTH_YELLOW_Pin | SOUTH_RED_Pin | SOUTH_YELLOW_Pin, GPIO_PIN_SET);

    // East/West: RED on
    HAL_GPIO_WritePin(GPIOA, EAST_RED_Pin | WEST_RED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, EAST_GREEN_Pin | EAST_YELLOW_Pin | WEST_GREEN_Pin | WEST_YELLOW_Pin, GPIO_PIN_SET);

    HAL_Delay(3000); // Wait for 3 seconds (Green light duration)

    /*
     * PHASE 2: North/South transition to yellow, East/West remain red.
     * Duration: 2 seconds.
     */
    // North/South: GREEN off, YELLOW on
    HAL_GPIO_WritePin(GPIOA, NORTH_GREEN_Pin | SOUTH_GREEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, NORTH_YELLOW_Pin | SOUTH_YELLOW_Pin, GPIO_PIN_RESET);

    // East/West: RED remains on
    HAL_GPIO_WritePin(GPIOA, EAST_RED_Pin | WEST_RED_Pin, GPIO_PIN_RESET);
    HAL_Delay(2000); // Wait for 2 seconds (Yellow light duration)

    /*
     * PHASE 3: North/South are red, East/West are green.
     * Duration: 3 seconds.
     */
    // North/South: YELLOW off, RED on
    HAL_GPIO_WritePin(GPIOA, NORTH_YELLOW_Pin | SOUTH_YELLOW_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, NORTH_RED_Pin | SOUTH_RED_Pin, GPIO_PIN_RESET);

    // East/West: RED off, GREEN on
    HAL_GPIO_WritePin(GPIOA, EAST_RED_Pin | WEST_RED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, EAST_GREEN_Pin | WEST_GREEN_Pin, GPIO_PIN_RESET);

    HAL_Delay(3000); // Wait for 3 seconds (Green light duration)

    /*
     * PHASE 4: East/West transition to yellow, North/South remain red.
     * Duration: 2 seconds.
     */
    // East/West: GREEN off, YELLOW on
    HAL_GPIO_WritePin(GPIOA, EAST_GREEN_Pin | WEST_GREEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, EAST_YELLOW_Pin | WEST_YELLOW_Pin, GPIO_PIN_RESET);

    // North/South: RED remains on
    HAL_GPIO_WritePin(GPIOA, NORTH_RED_Pin | SOUTH_RED_Pin, GPIO_PIN_RESET);
    HAL_Delay(2000); // Wait for 2 seconds (Yellow light duration)

    // Reset all lights to their initial state before looping again
    HAL_GPIO_WritePin(GPIOA, NORTH_YELLOW_Pin | SOUTH_YELLOW_Pin | EAST_YELLOW_Pin | WEST_YELLOW_Pin, GPIO_PIN_SET);
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RED_Pin|LED_YELLOW_Pin|LED_GREEN_Pin|LED_REDA3_Pin
                          |LED_YELLOWA4_Pin|LED_GREENA5_Pin|LED_REDA6_Pin|LED_YELLOWA7_Pin
                          |LED_GREENA8_Pin|LED_REDA9_Pin|LED_YELLOWA10_Pin|LED_GREENA11_Pin, GPIO_PIN_RESET);

  /*
   * Configure GPIO pins for all 12 LEDs.
   * The pins are configured as Push-Pull output with no pull-up/pull-down.
   * This is a standard configuration for driving LEDs.
   */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_YELLOW_Pin|LED_GREEN_Pin|LED_REDA3_Pin
                          |LED_YELLOWA4_Pin|LED_GREENA5_Pin|LED_REDA6_Pin|LED_YELLOWA7_Pin
                          |LED_GREENA8_Pin|LED_REDA9_Pin|LED_YELLOWA10_Pin|LED_GREENA11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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
  * where the assert_param error has occurred.
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
