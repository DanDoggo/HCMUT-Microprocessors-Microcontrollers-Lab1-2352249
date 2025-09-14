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
void map_time_to_leds(int *hour_led, int *minute_led, int *seconds_led, int hours, int minutes, int seconds);
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
  int hours = 10;
  int minutes = 33;
  int seconds = 0;  // Start at 0 for immediate first update

  int prev_hour_led = -1;
  int prev_minute_led = -1;
  int prev_seconds_led = -1;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  // It's good practice to clear all LEDs initially.
  // This is handled in MX_GPIO_Init by setting the initial pin state to HIGH (off for active-low).
  /* USER CODE END 2 */

  /* USER CODE BEGIN 3 */
  // Initial LED update at startup (before the loop)
  {
    int current_hour_led;
    int current_minute_led;
    int current_seconds_led;
    map_time_to_leds(&current_hour_led, &current_minute_led, &current_seconds_led, hours, minutes, seconds);

    // Clear previous LEDs (none on first run, but safe)
    if (prev_hour_led != -1) {
      clearNumberOnClock(prev_hour_led);
    }
    if (prev_minute_led != -1) {
      clearNumberOnClock(prev_minute_led);
    }
    if (prev_seconds_led != -1) {
      clearNumberOnClock(prev_seconds_led);
    }

    // Set initial LEDs
    setNumberOnClock(current_hour_led);
    setNumberOnClock(current_minute_led);
    setNumberOnClock(current_seconds_led);

    // Store the current LED positions for the next cycle
    prev_hour_led = current_hour_led;
    prev_minute_led = current_minute_led;
    prev_seconds_led = current_seconds_led;
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Wait for 1 second
    HAL_Delay(1000);

    // Increment the seconds
    seconds++;

    // Handle time rollovers
    if (seconds >= 60) {
      seconds = 0;
      minutes++;
      if (minutes >= 60) {
        minutes = 0;
        hours++;
        if (hours >= 12) {
          hours = 0;  // Reset to 12 (LED 0); use hours = 12 if you want to display 12 explicitly
        }
      }
    }

    // Update LEDs every second for real-time seconds and minute hand movement
    {
      int current_hour_led;
      int current_minute_led;
      int current_seconds_led;
      map_time_to_leds(&current_hour_led, &current_minute_led, &current_seconds_led, hours, minutes, seconds);

      // Clear previous LEDs only if they changed
      if (current_hour_led != prev_hour_led && prev_hour_led != -1) {
        clearNumberOnClock(prev_hour_led);
      }
      if (current_minute_led != prev_minute_led && prev_minute_led != -1) {
        clearNumberOnClock(prev_minute_led);
      }
      if (current_seconds_led != prev_seconds_led && prev_seconds_led != -1) {
        // Only clear seconds LED if it’s not the same as current hour or minute LED
        if (prev_seconds_led != current_hour_led && prev_seconds_led != current_minute_led) {
          clearNumberOnClock(prev_seconds_led);
        }
      }

      // Set new LEDs only if they changed (or first run)
      if (current_hour_led != prev_hour_led) {
        setNumberOnClock(current_hour_led);
      }
      if (current_minute_led != prev_minute_led) {
        setNumberOnClock(current_minute_led);
      }
      // Always set seconds LED to ensure it’s on, even if overlapping with hour/minute
      setNumberOnClock(current_seconds_led);

      // Store the current LED positions for the next cycle
      prev_hour_led = current_hour_led;
      prev_minute_led = current_minute_led;
      prev_seconds_led = current_seconds_led;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 4 */
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

  // For active-low LEDs, write a high signal (GPIO_PIN_SET) to turn them off initially.
  HAL_GPIO_WritePin(GPIOA, twelve_Pin|one_Pin|two_Pin|three_Pin
                              |four_Pin|five_Pin|six_Pin|seven_Pin
                              |eight_Pin|nine_Pin|ten_Pin|eleven_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : twelve_Pin one_Pin two_Pin three_Pin
                           four_Pin five_Pin six_Pin seven_Pin
                           eight_Pin nine_Pin ten_Pin eleven_Pin */
  GPIO_InitStruct.Pin = twelve_Pin|one_Pin|two_Pin|three_Pin
                              |four_Pin|five_Pin|six_Pin|seven_Pin
                              |eight_Pin|nine_Pin|ten_Pin|eleven_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Sets the LED on the clock face corresponding to the input number.
 * @param  num: The number on the clock (0-11). 0 corresponds to 12 o'clock.
 * @retval None
 */
void setNumberOnClock(int num) {
    // Check for valid input range
    if (num < 0 || num > 11) {
        // Optional: Handle error or return
        return;
    }

    switch (num) {
        case 0: // 12 o'clock position
            HAL_GPIO_WritePin(GPIOA, twelve_Pin, GPIO_PIN_RESET);
            break;
        case 1:
            HAL_GPIO_WritePin(GPIOA, one_Pin, GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(GPIOA, two_Pin, GPIO_PIN_RESET);
            break;
        case 3:
            HAL_GPIO_WritePin(GPIOA, three_Pin, GPIO_PIN_RESET);
            break;
        case 4:
            HAL_GPIO_WritePin(GPIOA, four_Pin, GPIO_PIN_RESET);
            break;
        case 5:
            HAL_GPIO_WritePin(GPIOA, five_Pin, GPIO_PIN_RESET);
            break;
        case 6:
            HAL_GPIO_WritePin(GPIOA, six_Pin, GPIO_PIN_RESET);
            break;
        case 7:
            HAL_GPIO_WritePin(GPIOA, seven_Pin, GPIO_PIN_RESET);
            break;
        case 8:
            HAL_GPIO_WritePin(GPIOA, eight_Pin, GPIO_PIN_RESET);
            break;
        case 9:
            HAL_GPIO_WritePin(GPIOA, nine_Pin, GPIO_PIN_RESET);
            break;
        case 10:
            HAL_GPIO_WritePin(GPIOA, ten_Pin, GPIO_PIN_RESET);
            break;
        case 11:
            HAL_GPIO_WritePin(GPIOA, eleven_Pin, GPIO_PIN_RESET);
            break;
    }
}

/**
 * @brief  Clears the LED on the clock face corresponding to the input number.
 * @param  num: The number on the clock (0-11). 0 corresponds to 12 o'clock.
 * @retval None
 */
void clearNumberOnClock(int num) {
    // Check for valid input range
    if (num < 0 || num > 11) {
        // Optional: Handle error or return
        return;
    }

    switch (num) {
        case 0: // 12 o'clock position
            HAL_GPIO_WritePin(GPIOA, twelve_Pin, GPIO_PIN_SET);
            break;
        case 1:
            HAL_GPIO_WritePin(GPIOA, one_Pin, GPIO_PIN_SET);
            break;
        case 2:
            HAL_GPIO_WritePin(GPIOA, two_Pin, GPIO_PIN_SET);
            break;
        case 3:
            HAL_GPIO_WritePin(GPIOA, three_Pin, GPIO_PIN_SET);
            break;
        case 4:
            HAL_GPIO_WritePin(GPIOA, four_Pin, GPIO_PIN_SET);
            break;
        case 5:
            HAL_GPIO_WritePin(GPIOA, five_Pin, GPIO_PIN_SET);
            break;
        case 6:
            HAL_GPIO_WritePin(GPIOA, six_Pin, GPIO_PIN_SET);
            break;
        case 7:
            HAL_GPIO_WritePin(GPIOA, seven_Pin, GPIO_PIN_SET);
            break;
        case 8:
            HAL_GPIO_WritePin(GPIOA, eight_Pin, GPIO_PIN_SET);
            break;
        case 9:
            HAL_GPIO_WritePin(GPIOA, nine_Pin, GPIO_PIN_SET);
            break;
        case 10:
            HAL_GPIO_WritePin(GPIOA, ten_Pin, GPIO_PIN_SET);
            break;
        case 11:
            HAL_GPIO_WritePin(GPIOA, eleven_Pin, GPIO_PIN_SET);
            break;
    }
}

/**
 * @brief  Maps the current time to the corresponding LED positions on the clock.
 * @param  hour_led: A pointer to store the calculated hour LED number.
 * @param  minute_led: A pointer to store the calculated minute LED number.
 * @param  seconds_led: A pointer to store the calculated seconds LED number.
 * @param  hours: The current hour (0-11).
 * @param  minutes: The current minute (0-59).
 * @param  seconds: The current second (0-59).
 * @retval None
 */
void map_time_to_leds(int *hour_led, int *minute_led, int *seconds_led, int hours, int minutes, int seconds) {
    // Hour hand calculation: Discrete, based on full hours (updates on minute rollover).
    *hour_led = hours % 12;

    // Minute hand calculation: Continuous across 12 positions (each ~5 minutes).
    double fractional_minutes = minutes + (double)seconds / 60.0;
    *minute_led = (int)(fractional_minutes / 5.0);
    if (*minute_led >= 12) {
        *minute_led = 0;  // Wrap around (handled by time rollover above)
    }

    // Seconds hand calculation: 12 positions, each ~5 seconds (60 / 12 = 5).
    *seconds_led = seconds / 5;
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
