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
// Array for active-low 7-segment patterns (gfedcba) for numbers 0-9
const uint8_t seven_seg_patterns[10] = {
    0x40, // 0
    0x79, // 1
    0x24, // 2
    0x30, // 3
    0x19, // 4
    0x12, // 5
    0x02, // 6
    0x78, // 7
    0x00, // 8
    0x10  // 9
};

// Countdown sequences for North and East displays
const int north_sequence[10] = {3, 2, 1, 2, 1, 5, 4, 3, 2, 1};
const int east_sequence[10] =  {5, 4, 3, 2, 1, 3, 2, 1, 2, 1};

// Global counter for the 10-second traffic cycle
int cycle_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void displayNorth(int number);
void displayEast(int number);
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
    /* USER CODE END WHILE */

    // Display the current countdown values on the 7-segment LEDs
    displayNorth(north_sequence[cycle_counter]);
    displayEast(east_sequence[cycle_counter]);

    // Update traffic light LEDs based on the current second of the cycle
    if (cycle_counter < 3) { // 0, 1, 2
      // PHASE 1: N/S Green (3s), E/W Red
      HAL_GPIO_WritePin(GPIOA, NORTH_GREEN_Pin | SOUTH_GREEN_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, NORTH_RED_Pin | NORTH_YELLOW_Pin | SOUTH_RED_Pin | SOUTH_YELLOW_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, EAST_RED_Pin | WEST_RED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, EAST_GREEN_Pin | EAST_YELLOW_Pin | WEST_GREEN_Pin | WEST_YELLOW_Pin, GPIO_PIN_SET);
    }
    else if (cycle_counter < 5) { // 3, 4
      // PHASE 2: N/S Yellow (2s), E/W Red
      HAL_GPIO_WritePin(GPIOA, NORTH_GREEN_Pin | SOUTH_GREEN_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, NORTH_YELLOW_Pin | SOUTH_YELLOW_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, EAST_RED_Pin | WEST_RED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, EAST_GREEN_Pin | EAST_YELLOW_Pin | WEST_GREEN_Pin | WEST_YELLOW_Pin, GPIO_PIN_SET);
    }
    else if (cycle_counter < 8) { // 5, 6, 7
      // PHASE 3: E/W Green (3s), N/S Red
      HAL_GPIO_WritePin(GPIOA, NORTH_YELLOW_Pin | SOUTH_YELLOW_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, NORTH_RED_Pin | SOUTH_RED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, EAST_RED_Pin | WEST_RED_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, EAST_GREEN_Pin | WEST_GREEN_Pin, GPIO_PIN_RESET);
    }
    else { // 8, 9
      // PHASE 4: E/W Yellow (2s), N/S Red
      HAL_GPIO_WritePin(GPIOA, EAST_GREEN_Pin | WEST_GREEN_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, EAST_YELLOW_Pin | WEST_YELLOW_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, NORTH_RED_Pin | SOUTH_RED_Pin, GPIO_PIN_RESET);
    }

    // Wait for 1 second before the next update
    HAL_Delay(1000);

    // Increment and loop the counter
    cycle_counter++;
    if (cycle_counter >= 10) {
      cycle_counter = 0;
    }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  // Set all LED pins on GPIOA high to turn them OFF initially
  HAL_GPIO_WritePin(GPIOA, LED_RED_Pin|LED_YELLOW_Pin|LED_GREEN_Pin|LED_REDA3_Pin
                          |LED_YELLOWA4_Pin|LED_GREENA5_Pin|LED_REDA6_Pin|LED_YELLOWA7_Pin
                          |LED_GREENA8_Pin|LED_REDA9_Pin|LED_YELLOWA10_Pin|LED_GREENA11_Pin
                          |f_Pin|g_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  // Set all 7-segment pins on GPIOB high to turn segments OFF (active low)
  HAL_GPIO_WritePin(GPIOB, 0x3FFF, GPIO_PIN_SET);

  /*Configure GPIO pins : Traffic Light LEDs */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_YELLOW_Pin|LED_GREEN_Pin|LED_REDA3_Pin
                          |LED_YELLOWA4_Pin|LED_GREENA5_Pin|LED_REDA6_Pin|LED_YELLOWA7_Pin
                          |LED_GREENA8_Pin|LED_REDA9_Pin|LED_YELLOWA10_Pin|LED_GREENA11_Pin
                          |f_Pin|g_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : 7-Segment LEDs */
  // PB0-PB6 for North Display, PB7-PB13 for East Display
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Displays a number on the North 7-segment LED (PB0-PB6).
  * @param  number: The integer to display (0-9).
  * @retval None
  */
void displayNorth(int number) {
    if (number < 0 || number > 9) return;
    uint32_t portb_data = GPIOB->ODR;
    // Clear the lower 7 bits (PB0-PB6) and set the new pattern
    portb_data = (portb_data & 0xFF80) | seven_seg_patterns[number];
    GPIOB->ODR = portb_data;
}

/**
  * @brief  Displays a number on the East 7-segment LED (PB7-PB13).
  * @param  number: The integer to display (0-9).
  * @retval None
  */
void displayEast(int number) {
    if (number < 0 || number > 9) return;
    uint32_t portb_data = GPIOB->ODR;
    uint16_t pattern = (uint16_t)(seven_seg_patterns[number] << 7);
    // Clear bits PB7-PB13 and set the new shifted pattern
    portb_data = (portb_data & 0xC07F) | pattern;
    GPIOB->ODR = portb_data;
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
