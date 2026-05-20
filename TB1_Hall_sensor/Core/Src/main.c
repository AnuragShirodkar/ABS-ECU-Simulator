/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAGNETS        4
#define RADIUS_M       0.025f
#define CIRCUMFERENCE  (2.0f * 3.14159f * RADIUS_M)
#define STOPPED_MS     500     // no pulse for 500ms = stopped
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// ── Capture (set inside interrupt, read in main loop) ──
//volatile uint32_t capture1      = 0;
//volatile uint32_t capture2      = 0;
//volatile uint32_t pulse_period  = 0;
volatile uint8_t  capture_done  = 0;
//volatile uint8_t  capture_index = 0;

float    wheel_rpm  = 0.0f;
float    speed_ms   = 0.0f;
float    speed_kmh  = 0.0f;

uint32_t last_tick  = 0;
uint8_t  is_stopped = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

// Full revolution — waits for all 4 magnets then calculates
volatile uint32_t pulse_times[4] = {0};
volatile uint8_t  pulse_count    = 0;
volatile uint32_t rev_period     = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        pulse_times[pulse_count] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        pulse_count++;

        if (pulse_count >= 4)
        {
            uint32_t t_start = pulse_times[0];
            uint32_t t_end   = pulse_times[3];

            if (t_end > t_start)
                rev_period = t_end - t_start;
            else
                rev_period = (1000000 - t_start) + t_end;

            capture_done = 1;
            pulse_count  = 0;
            last_tick    = HAL_GetTick();
        }
    }
}
//int __io_putchar(int ch)
//{
//    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
//    return ch;
//}
//
//// Fires every time a magnet passes the sensor
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim->Instance == TIM2)
//    {
//        if (capture_index == 0)
//        {
//            // First pulse — save timestamp
//            capture1      = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//            capture_index = 1;
//        }
//        else
//        {
//            // Second pulse — calculate period
//            capture2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//
//            if (capture2 > capture1)
//                pulse_period = capture2 - capture1;
//            else
//                pulse_period = (65535 - capture1) + capture2;
//
//            capture_done  = 1;
//            capture_index = 0;
//            last_tick     = HAL_GetTick();
//        }
//    }
//}
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

  printf("\r\n=============================\r\n");
  printf("  ABS WHEEL SPEED MONITOR\r\n");
  printf("  Magnets : 4\r\n");
  printf("  Radius  : 2.5 cm\r\n");
  printf("  Circ    : %.4f m\r\n", CIRCUMFERENCE);
  printf("=============================\r\n");
  printf("  Spin the wheel...\r\n\r\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (last_tick != 0 &&
	     (HAL_GetTick() - last_tick) > STOPPED_MS)
	  {
	      if (is_stopped == 0)
	      {
	          is_stopped = 1;
	          wheel_rpm  = 0;
	          speed_kmh  = 0;
	          printf("-----------------------------\r\n");
	          printf("  WHEEL STOPPED\r\n");
	          printf("  RPM   : 0\r\n");
	          printf("  Speed : 0.00 km/h\r\n");
	          printf("-----------------------------\r\n\r\n");
	      }
	  }

	  // New reading available
	  if (capture_done == 1)
	  {
	      capture_done = 0;

	      // rev_period = time for full revolution in us
	      // valid range: 10 RPM to 8000 RPM
	      if (rev_period >= 7500 && rev_period <= 100000)
	      {
	          wheel_rpm = 60000000.0f / (float)rev_period;
	          speed_ms  = (wheel_rpm / 60.0f) * CIRCUMFERENCE;
	          speed_kmh = speed_ms * 3.6f;
	          is_stopped = 0;

	          printf("  Rev Period : %lu us\r\n",   rev_period);
	          printf("  RPM        : %.1f\r\n",     wheel_rpm);
	          printf("  Speed      : %.2f km/h\r\n",speed_kmh);
	          printf("-----------------------------\r\n");
	      }
	  }

	  HAL_Delay(10);

//	  if (last_tick != 0 &&
//	     (HAL_GetTick() - last_tick) > STOPPED_MS)
//	  {
//	      if (is_stopped == 0)
//	      {
//	          is_stopped = 1;
//	          wheel_rpm  = 0;
//	          speed_ms   = 0;
//	          speed_kmh  = 0;
//
//	          printf("-----------------------------\r\n");
//	          printf("  WHEEL STOPPED\r\n");
//	          printf("  RPM   : 0\r\n");
//	          printf("  Speed : 0.00 km/h\r\n");
//	          printf("-----------------------------\r\n\r\n");
//	      }
//	  }
//
//	  // ── New data from interrupt ──────────────
//	  if (capture_done == 1)
//	  {
//	      capture_done = 0;
//
//	      // Valid range:
//	      // Max RPM 8000 → period = 60M/(8000×4) = 1875 us
//	      // Min RPM 10   → period = 60M/(10×4)   = 1,500,000 us
//	      if (pulse_period >= 1875 && pulse_period <= 1500000)
//	      {
//	          wheel_rpm  = 60000000.0f / ((float)pulse_period * (float)MAGNETS);
//	          speed_ms   = (wheel_rpm / 60.0f) * CIRCUMFERENCE;
//	          speed_kmh  = speed_ms * 3.6f;
//	          is_stopped = 0;
//
//	          printf("  Period : %lu us\r\n",   pulse_period);
//	          printf("  RPM    : %.1f\r\n",     wheel_rpm);
//	          printf("  Speed  : %.3f m/s\r\n", speed_ms);
//	          printf("  Speed  : %.2f km/h\r\n",speed_kmh);
//	          printf("-----------------------------\r\n");
//	      }
//	  }
//
//	  HAL_Delay(10);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 49;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
