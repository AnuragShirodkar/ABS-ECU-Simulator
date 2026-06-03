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
#include "string.h"
#include "math.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define MAGNETS              4U
//#define WHEEL_RADIUS_M       0.025f
//#define WHEEL_CIRCUMFERENCE_M (2.0f * 3.1415926f * WHEEL_RADIUS_M)
//#define SPEED_SCALE_FACTOR   12.0f
#define HALL_TO_MOTOR_RATIO   7.0f
#define MAX_MOTOR_RPM         5000.0f
#define MAX_VEHICLE_SPEED     250.0f

#define STOPPED_TIMEOUT_MS   500U
#define ABS_SLIP_THRESHOLD   20.0f
#define ABS_PHASE_MS         80U

#define CAN_ID_BMS_BRAKE     0x101U
#define CAN_ID_STATUS        0x102U
#define CAN_ID_TELEMATICS    0x107U
#define CAN_ID_TELEM_TX      0x117U  // ABS ECU → Telematics

//#define CAN_ID_TELEMATICS_SPEED  0x104U
//ADC_HandleTypeDef hadc1;
//CAN_HandleTypeDef hcan1;

volatile uint32_t capture1 = 0;
volatile uint32_t capture2 = 0;
volatile uint32_t pulse_period_us = 0;
volatile uint8_t capture_done = 0;
volatile uint8_t capture_index = 0;

uint8_t temp_warning_active = 0U;
uint32_t last_capture_tick = 0;
uint32_t last_speed_tx_tick = 0;
//uint32_t last_temp_tx_tick = 0;
uint32_t last_status_tx_tick = 0;
uint32_t last_imm_rx_tick = 0xFFFFFFFFU;
uint32_t last_mob_rx_tick = 0xFFFFFFFFU;

float wheel_rpm = 0.0f;
float wheel_speed_kmh = 0.0f;
float brake_temp_c = 30.0f;
float v_ref_kmh = 0.0f;

uint8_t brake_input_pct = 0;
uint8_t brake_cmd_pct = 0;
uint8_t abs_active = 0;
uint8_t handbrake = 0;
uint8_t immobilized = 0;
uint8_t prev_brake_input_pct = 0;
uint8_t abs_phase = 0;
uint32_t abs_phase_tick = 0;
CAN_FilterTypeDef filter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t ADC_ToBrakePct(uint32_t adc);
static HAL_StatusTypeDef CAN_SendStd(uint16_t std_id, const uint8_t *data, uint8_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

static uint8_t ADC_ToBrakePct(uint32_t adc)
{
    // DEAD ZONE around center position

    if (adc < 2100)
        return 0;

    if (adc > 4000)
        adc = 4000;

    return (uint8_t)(((adc - 2100) * 100U) / (4000 - 2100));
}

static HAL_StatusTypeDef CAN_SendStd(uint16_t std_id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef txHeader = {0};
    uint32_t txMailbox = 0;

    txHeader.StdId             = std_id;
    txHeader.ExtId             = 0;
    txHeader.IDE               = CAN_ID_STD;
    txHeader.RTR               = CAN_RTR_DATA;
    txHeader.DLC               = len;
    txHeader.TransmitGlobalTime = DISABLE;

    // ===== CAN BUS HEALTH CHECK =====
    uint32_t can_err = HAL_CAN_GetError(&hcan1);
    if (can_err != HAL_CAN_ERROR_NONE)
    {
        printf("\033[0;31m[CAN BUS ERROR] 0x%lX — Bus may be offline!\r\n\033[0m", can_err);
    }

    uint32_t timeout = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        if ((HAL_GetTick() - timeout) > 5U)
        {
            printf("\033[0;31m[CAN TX TIMEOUT] ID:0x%03X — No free mailbox!\r\n\033[0m", std_id);
            return HAL_TIMEOUT;
        }
    }

    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &txMailbox);

    if (status == HAL_OK)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

        // ===== DESTINATION PRINT =====
        const char *dest = "Unknown";
        if      (std_id == CAN_ID_BMS_BRAKE) dest = "BMS";
        else if (std_id == CAN_ID_STATUS)    dest = "Infotainment";
        else if (std_id == CAN_ID_TELEM_TX)  dest = "Telematics";

        printf("\033[0;32m[CAN TX OK] 0x%03X → %-18s | DLC:%u | Data:", std_id, dest, len);
        for (uint8_t i = 0; i < len; i++)
            printf(" 0x%02X", data[i]);
        printf("\r\n\033[0m");
    }
    else
    {
        printf("\033[0;31m[CAN TX FAIL] 0x%03X ERR:0x%lX\r\n\033[0m",
               std_id, HAL_CAN_GetError(&hcan1));
    }

    return status;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;

    if (capture_index == 0)
    {
        capture1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        capture_index = 1;
    }
    else
    {
        capture2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        if (capture2 >= capture1)
            pulse_period_us = capture2 - capture1;
        else
        	pulse_period_us = (0xFFFFFFFFU - capture1) + 1U + capture2;

        capture_index = 0;
        capture_done = 1;
        last_capture_tick = HAL_GetTick();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;

    CAN_RxHeaderTypeDef rxHeader = {0};
    uint8_t rxData[8] = {0};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        return;

    if (rxHeader.StdId == 0x107U)
    {
        static uint8_t last_cmd = 0x00;
        uint8_t current_cmd = rxData[0];

        // Ignore repeated same command
        if (current_cmd != last_cmd)
        {
            last_cmd = current_cmd;

            if (current_cmd == 0x99U)
            {
                immobilized = 1U;
                last_imm_rx_tick = HAL_GetTick();

                printf("\033[0;31m[CAN RX] IMMOBILIZE received!\r\n\033[0m");
            }
            else if (current_cmd == 0x11U)
            {
                immobilized = 0U;
                last_mob_rx_tick = HAL_GetTick();

                printf("\033[0;32m[CAN RX] MOBILIZE received!\r\n\033[0m");
            }
        }
    }
}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

  last_imm_rx_tick = HAL_GetTick() - 1000U;
  last_mob_rx_tick = HAL_GetTick() - 1000U;

  filter.FilterActivation     = CAN_FILTER_ENABLE;
  filter.FilterBank           = 0;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterIdHigh         = 0x0000;
  filter.FilterIdLow          = 0x0000;
  filter.FilterMaskIdHigh     = 0x0000;
  filter.FilterMaskIdLow      = 0x0000;
  filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  filter.SlaveStartFilterBank = 14;        // ← add this

  HAL_CAN_ConfigFilter(&hcan1, &filter);   // only once

  if (HAL_CAN_Start(&hcan1) == HAL_OK)
      printf("CAN START SUCCESS\r\n");
  else
      printf("CAN START FAIL\r\n");

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  printf("ABS ECU Ready\r\n");
  printf("Hall PA0 | Brake PA1 | Handbrake PB0\r\n");
  printf("CAN 500kbps | UART 115200\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  /* USER CODE BEGIN 3 */
	  uint8_t info_data[4] = {0};  // declare here so printf can access it
	  	  uint32_t now = HAL_GetTick();
	  	  uint32_t adc_raw = 0;

	  	  // ========== SPEED FROM HALL SENSOR ==========
	  	  if (capture_done)
	  	  {
	  		printf("RPM = %.1f\r\n", wheel_rpm);
	  	      capture_done = 0;
	  	      if (pulse_period_us > 0U)
	  	      {
	  	    	float freq_hz = 1000000.0f / (float)pulse_period_us;

	  	    	wheel_rpm = (freq_hz * 60.0f) / (float)MAGNETS;

	  	    	float motor_rpm = wheel_rpm * HALL_TO_MOTOR_RATIO;

	  	    	wheel_speed_kmh =
	  	    	    (motor_rpm * MAX_VEHICLE_SPEED) / MAX_MOTOR_RPM;

	  	    	if (wheel_speed_kmh > MAX_VEHICLE_SPEED)
	  	    	{
	  	    	    wheel_speed_kmh = MAX_VEHICLE_SPEED;
	  	    	}
	  	      }
	  	  }

	  	  if ((now - last_capture_tick) > STOPPED_TIMEOUT_MS)
	  	  {
	  	      wheel_rpm       = 0.0f;
	  	      wheel_speed_kmh = 0.0f;
	  	  }

	  	  // ========== ADC BRAKE INPUT ==========
	  	  if (HAL_ADC_Start(&hadc1) == HAL_OK)
	  	  {
	  	      if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
	  	          adc_raw = HAL_ADC_GetValue(&hadc1);
	  	      HAL_ADC_Stop(&hadc1);
	  	  }

	  	  brake_input_pct = ADC_ToBrakePct(adc_raw);
	  	  handbrake = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) ? 1U : 0U;

	  	  // ========== V_REF CAPTURE ==========
	  	  if (brake_input_pct > 0U && prev_brake_input_pct == 0U)
	  	      v_ref_kmh = (wheel_speed_kmh > 1.0f) ? wheel_speed_kmh : 1.0f;

	  	  if (brake_input_pct == 0U && handbrake == 0U && immobilized == 0U)
	  	  {
	  	      v_ref_kmh  = 0.0f;
	  	      abs_active = 0U;
	  	      abs_phase  = 0U;
	  	  }

	  	  // ========== BRAKE + ABS LOGIC ==========
	  	  if (immobilized || handbrake)
	  	  {
	  	      brake_cmd_pct = 100U;
	  	      abs_active    = 0U;
	  	  }
	  	  else
	  	  {
	  	      float slip = 0.0f;
	  	      if (v_ref_kmh > 1.0f && wheel_speed_kmh < v_ref_kmh)
	  	          slip = ((v_ref_kmh - wheel_speed_kmh) / v_ref_kmh) * 100.0f;

	  	      if (brake_input_pct == 0U)
	  	      {
	  	          brake_cmd_pct = 0U;
	  	          abs_active    = 0U;
	  	      }
	  	      else if (slip > ABS_SLIP_THRESHOLD)
	  	      {
	  	          abs_active = 1U;
	  	          if ((now - abs_phase_tick) >= ABS_PHASE_MS)
	  	          {
	  	              abs_phase_tick = now;
	  	              abs_phase = (abs_phase + 1U) % 3U;
	  	          }
	  	          if (abs_phase == 0U)
	  	              brake_cmd_pct = brake_input_pct;
	  	          else if (abs_phase == 1U)
	  	              brake_cmd_pct = (brake_input_pct > 20U) ? (brake_input_pct - 20U) : 0U;
	  	          else
	  	          {
	  	              uint16_t temp_cmd = brake_input_pct + 10U;
	  	              brake_cmd_pct = (temp_cmd > 100U) ? 100U : (uint8_t)temp_cmd;
	  	          }
	  	      }
	  	      else
	  	      {
	  	          abs_active    = 0U;
	  	          abs_phase     = 0U;
	  	          brake_cmd_pct = brake_input_pct;
	  	      }
	  	  }

	  	  // ========== TEMPERATURE SIMULATION ==========
	  	  if (brake_cmd_pct > 0U)
	  	  {
	  	      float heat_rate = ((float)brake_cmd_pct / 100.0f) * 0.28f;
	  	      brake_temp_c += heat_rate;
	  	  }
	  	  else
	  	  {
	  	      float cool_rate = (brake_temp_c - 30.0f) * 0.005f;
	  	      brake_temp_c -= cool_rate;
	  	  }
	  	  if (brake_temp_c < 30.0f)  brake_temp_c = 30.0f;
	  	  if (brake_temp_c > 250.0f) brake_temp_c = 250.0f;

	  	  // ========== ABS STATUS BASED ON TEMPERATURE ==========
	  	  uint8_t abs_status_byte = 0U;
	  	  if (brake_temp_c >= 200.0f)
	  	  {
	  	      temp_warning_active = 1U;
	  	      abs_status_byte     = 0U;   // FAULT
	  	  }
	  	  else if (temp_warning_active && brake_temp_c > 80.0f)
	  	  {
	  	      abs_status_byte = 0U;       // hold fault until below 80C
	  	  }
	  	  else
	  	  {
	  	      temp_warning_active = 0U;
	  	      abs_status_byte     = abs_active ? 1U : 0U;  // 1=working 0=off
	  	  }

	  	  // ========== CAN TX EVERY 50ms ==========
	  	  if ((now - last_speed_tx_tick) >= 500U)
	  	  {
	  	      // --- 0x101 → BMS (1 byte: brake% only) ---
	  		// --- 0x101 → BMS (2 bytes: joystick brake% + handbrake state) ---
	  		uint8_t bms_data[4];

	  		if (immobilized)
	  		{
	  		    bms_data[0] = 100U;
	  		    bms_data[1] = 1U;
	  		}
	  		else
	  		{
	  		    bms_data[0] = brake_cmd_pct;
	  		    bms_data[1] = handbrake;
	  		}

	  		// speed packed in byte 2 and 3
	  		bms_data[2] = (uint8_t)wheel_speed_kmh;
	  		bms_data[3] = (uint8_t)((uint16_t)(wheel_speed_kmh * 10.0f) % 10U);

	  		CAN_SendStd(CAN_ID_BMS_BRAKE, bms_data, 4);

	  	      // --- 0x102 → Infotainment (3 bytes: status + speed) ---
	  	     // uint8_t info_data[4];
	  	      if (temp_warning_active)
	  	          info_data[0] = 0x02;              // brake failure / ABS fault
	  	      else if (abs_active)
	  	          info_data[0] = 0x01;              // ABS active
	  	      else
	  	          info_data[0] = 0x00;              // normal
	  	    info_data[0] = temp_warning_active ? 0x02 : abs_active ? 0x01 : 0x00;
	  	    info_data[1] = (uint8_t)wheel_speed_kmh;
	  	    info_data[2] = (uint8_t)((uint16_t)(wheel_speed_kmh * 10.0f) % 10U);
	  	    info_data[3] = handbrake;   // ← add handbrake here
	  	    CAN_SendStd(CAN_ID_STATUS, info_data, 4);  // DLC 3→4



	  	    // --- 0x117 → Telematics ABS status (1 byte) ---
	  	    uint8_t telem_data[1];
	  	    if (temp_warning_active)
	  	        telem_data[0] = 0x02;        // ABS FAULT
	  	    else if (abs_active)
	  	        telem_data[0] = 0x01;        // ABS ACTIVE
	  	    else
	  	        telem_data[0] = 0x00;        // ABS NORMAL
	  	    CAN_SendStd(CAN_ID_TELEM_TX, telem_data, 1);

	  	      // --- Fault print if temp warning active ---
	  	      if (temp_warning_active)
	  	      {
	  	          printf("\033[1;31m");
	  	          printf("╔══════════════════════════════════════════╗\r\n");
	  	          printf("║        ⚠  ABS FAULT DETECTED  ⚠       	 ║\r\n");
	  	          printf("║  Temp     : %.1f C  [CRITICAL]           ║\r\n", brake_temp_c);
	  	          printf("║  ABS      : DISABLED (0)                 ║\r\n");
	  	          printf("║  Fault TX : 0x102 → Infotainment         ║\r\n");
	  	          printf("║  Fault TX : 0x117 → Telematics           ║\r\n");
	  	          printf("╚══════════════════════════════════════════╝\r\n");
	  	          printf("\033[0m");
	  	      }

	  	      last_speed_tx_tick = now;
	  	  }

	  	  prev_brake_input_pct = brake_input_pct;

	  	  // ========== LED LOGIC ==========
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);  // LD3 ORANGE = READY

	  	  if (brake_cmd_pct > 0)                                // LD4 GREEN = BRAKING
	  	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	  	  else
	  	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

	  	  if ((now - last_speed_tx_tick) < 25U)                 // LD5 RED = TX BLINK
	  	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	  	  else
	  	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

	  	  if ((HAL_GetTick() - last_imm_rx_tick) < 200U ||      // LD6 BLUE = RX BLINK
	  	      (HAL_GetTick() - last_mob_rx_tick) < 200U)
	  	  {
	  	      if ((HAL_GetTick() % 100) < 50)
	  	          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	  	      else
	  	          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	  	  }
	  	  else if (immobilized)
	  	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	  	  else
	  	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

	  	  // ========== UART STATUS PRINT 500ms ==========
	  	  if ((now - last_status_tx_tick) >= 500U)
	  	  {
	  	      printf("\033[2J\033[H");
	  	      printf("\033[1;36m╔══════════════════════════════════════════╗\r\n");
	  	      printf("║          ABS ECU STATUS MONITOR           ║\r\n");
	  	      printf("╚══════════════════════════════════════════╝\033[0m\r\n");

	  	      printf("\033[1;33mSPEED    :\033[0m %.1f km/h\r\n", wheel_speed_kmh);
	  	      printf("\033[1;33mBRAKE    :\033[0m %u%%\r\n", brake_cmd_pct);
	  	      printf("\033[1;33mABS      :\033[0m %s\r\n",
	  	             abs_status_byte ? "\033[0;32mWORKING (1)\033[0m"
	  	                             : "\033[0;31mFAULT / OFF (0)\033[0m");
	  	      printf("\033[1;33mTEMP     :\033[0m %.1f C  %s\r\n", brake_temp_c,
	  	             brake_temp_c >= 200.0f ? "\033[0;31m[CRITICAL]\033[0m" :
	  	             brake_temp_c >= 100.0f ? "\033[0;33m[WARNING]\033[0m"  :
	  	                                      "\033[0;32m[NORMAL]\033[0m");
	  	      printf("\033[1;33mHANDBRK  :\033[0m %s\r\n",
	  	             handbrake ? "\033[0;31mON\033[0m" : "\033[0;32mOFF\033[0m");
	  	      printf("\033[1;33mIMMOBILE :\033[0m %s\r\n",
	  	             immobilized ? "\033[0;31mACTIVE\033[0m" : "\033[0;32mCLEAR\033[0m");

	  	      printf("\033[1;36m────────── CAN TX ──────────────────────\033[0m\r\n");
	  	      printf("\033[1;33m0x101 BMS  :\033[0m \033[0;32mbrake_pct=%u%%\033[0m\r\n",
	  	             brake_cmd_pct);
	  	      printf("\033[1;33m0x102 INFO :\033[0m \033[0;32mstatus=0x%02X  spd=%.1f km/h  abs=%u  hbrk=%u\033[0m\r\n",
	  	           info_data[0], wheel_speed_kmh, abs_status_byte, handbrake);
	  	      printf("\033[1;33m0x104 TCU  :\033[0m \033[0;32mspd=%.1f km/h\033[0m\r\n",
	  	             wheel_speed_kmh);
	  	      printf("\033[1;33m0x117 TELE :\033[0m \033[0;32mabs_status=%u\033[0m\r\n",
	  	             abs_status_byte);

	  	    // ===== REAL CAN BUS HEALTH CHECK =====
	  	    uint32_t can_err = HAL_CAN_GetError(&hcan1);

	  	    // Read transmit and receive error counters directly from hardware
	  	    uint8_t tec = (uint8_t)((CAN1->ESR >> 16) & 0xFF);  // TX error count
	  	    uint8_t rec = (uint8_t)((CAN1->ESR >> 24) & 0xFF);  // RX error count

	  	    printf("\033[1;36m────────── CAN BUS HEALTH ──────────────\033[0m\r\n");

	  	    if (can_err & HAL_CAN_ERROR_BOF)
	  	        printf("\033[1;33mBUS STATE:\033[0m \033[0;31mBUS-OFF — Disconnected!\033[0m\r\n");
	  	    else if (can_err & HAL_CAN_ERROR_EPV)
	  	        printf("\033[1;33mBUS STATE:\033[0m \033[0;31mERROR PASSIVE — Bad connection!\033[0m\r\n");
	  	    else if (tec > 96 || rec > 96)
	  	        printf("\033[1;33mBUS STATE:\033[0m \033[0;33mWARNING — High error count!\033[0m\r\n");
	  	    else
	  	        printf("\033[1;33mBUS STATE:\033[0m \033[0;32mONLINE ✓\033[0m\r\n");

	  	    printf("\033[1;33mTX ERR   :\033[0m %u\r\n", tec);
	  	    printf("\033[1;33mRX ERR   :\033[0m %u\r\n", rec);
	  	    printf("\033[1;33mERR CODE :\033[0m 0x%08lX\r\n", can_err);

	  	      printf("\033[1;36m────────── CAN RX ──────────────────────\033[0m\r\n");
	  	    printf("\033[1;33mADC RAW  :\033[0m %lu\r\n", (uint32_t)adc_raw);
	  	      printf("\033[1;33m0x107 TELE :\033[0m %s\r\n",
	  	             immobilized ? "\033[0;31mIMMOBILIZED\033[0m" : "\033[0;32mMOBILE\033[0m");

	  	      printf("\033[1;36m═══════════════════════════════════════════\033[0m\r\n");

	  	      last_status_tx_tick = now;
	  	  }
	    /* USER CODE END 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = ENABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  // ================= USART2 =================
  // PA2 -> TX
  // PA3 -> RX

  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // ================= TIM2 CH1 =================
  // PA0 -> Hall Sensor

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // ================= CAN1 =================
  // PD0 -> CAN RX
  // PD1 -> CAN TX

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  // ================= HANDBRAKE =================

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // ================= LEDs =================

  HAL_GPIO_WritePin(GPIOD,
                    GPIO_PIN_12 |
                    GPIO_PIN_13 |
                    GPIO_PIN_14 |
                    GPIO_PIN_15,
                    GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_12 |
                        GPIO_PIN_13 |
                        GPIO_PIN_14 |
                        GPIO_PIN_15;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
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
