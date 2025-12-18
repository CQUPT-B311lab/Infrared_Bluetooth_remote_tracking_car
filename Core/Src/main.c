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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "COM.h"
#include "MPU6500.h"
#include "Motor.h"
// #include "OLED.h"
#include "PID.h"
#include "math.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_uart.h"
#include "trace_sensor.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/_intsup.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUF_LEN 64
// #define LOOP_TEST
// #define READ_TEMPLE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint8_t RX_Buf_A[UART_RX_BUF_LEN] = {0};
uint8_t RX_Buf_B[UART_RX_BUF_LEN] = {0};
uint8_t *RX_Buf_Active = RX_Buf_A; // DMA正在写入的
uint8_t *RX_Buf_Ready = NULL;      // 准备好供主循环处理的
volatile uint8_t dataFlag = 0;

volatile int16_t M_speed_L = 0;
volatile int16_t M_speed_R = 0;
volatile uint8_t stop_flag = 1;

char float_buf1[20];
char float_buf2[20];
char float_buf3[20];

volatile uint8_t print_flag = 0;

PID_t pid_L;
PID_t pid_R;
volatile float target_L = 0;
volatile float target_R = 0;
PID_t pid_yaw;
volatile float target_Y = 0;
int16_t yaw_P = 10; // yaw调节比例系数
volatile int16_t target_d = 0;
uint8_t distance_mode = 0;
volatile int16_t measure_d = 0;

volatile uint8_t TIM4_RCC = 0;
volatile uint8_t TIM1_RCC = 0;

MPU6500_Data mpu_data;
uint8_t MPU6500_flag = 0;
uint8_t joystick_mode = 0; // 摇杆模式
uint8_t turn_l_flag = 0;
uint8_t turn_r_flag = 0;

extern volatile uint16_t adc_dma_buffer[TRACE_CHANNELS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static inline float clampf(float x, float minv, float maxv);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float normalize_angle(float angle) {
  while (angle > 180.0f)
    angle -= 360.0f;
  while (angle < -180.0f)
    angle += 360.0f;
  return angle;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    TIM1_RCC++;
    if (TIM1_RCC == 10) {
      TIM1_RCC = 0;
      if (MPU6500_flag)
        MPU6500_Get_All_Data(&mpu_data);
      TraceSensor_Update();
    }
  }
  if (htim->Instance == TIM4) {
    TIM4_RCC++;
    if (TIM4_RCC == 10) {
      TIM4_RCC = 0;
      M_speed_L = GetSpeed(&htim2); // 脉冲/10ms
      M_speed_R = GetSpeed(&htim3); // 脉冲/10ms
      float yaw_now = mpu_data.yaw;

      if (stop_flag) {
        // 清状态，防止再启动猛冲
        pid_L.I = 0;
        pid_L.Err = 0;
        pid_L.Err1 = 0;
        pid_L.Out = 0;
        pid_R.I = 0;
        pid_R.Err = 0;
        pid_R.Err1 = 0;
        pid_R.Out = 0;
        pid_yaw.I = 0;
        pid_yaw.Err = 0;
        pid_yaw.Err1 = 0;
        pid_yaw.Out = 0;

        SetSpeed(&htim4, 0);
        SetSpeed(&htim1, 0);
      } else {

        float diff_yaw = normalize_angle(target_Y) - normalize_angle(yaw_now);
        // 处理跨越±180°的情况
        if (diff_yaw > 180.0f) {
          diff_yaw -= 360.0f;
        } else if (diff_yaw < -180.0f) {
          diff_yaw += 360.0f;
        }

        pid_yaw.Exp = 0;
        pid_yaw.Mea = diff_yaw;
        float yaw_correction = PID_Update(&pid_yaw);

        // 目标限幅：避免不可能达到的设定导致长时间饱和
        float tL, tR;
        if (fabs(diff_yaw) < 5.0) {
          tL = clampf(target_L + yaw_correction, -TARGET_LIMIT, TARGET_LIMIT);
          tR = clampf(target_R - yaw_correction, -TARGET_LIMIT, TARGET_LIMIT);
        } else {
          tL = clampf(yaw_correction, -TARGET_LIMIT, TARGET_LIMIT);
          tR = clampf(-yaw_correction, -TARGET_LIMIT, TARGET_LIMIT);
        }

        pid_L.Exp = tL;
        pid_L.Mea = (float)M_speed_L;

        pid_R.Exp = tR;
        pid_R.Mea = (float)M_speed_R;

        int16_t outL = (int16_t)PID_Update(&pid_L);
        int16_t outR = (int16_t)PID_Update(&pid_R);

        if (distance_mode) {
          measure_d += (M_speed_L + M_speed_R) / 2;
          float remain = measure_d - target_d;
          if (measure_d >= target_d) {
            SetSpeed(&htim1, 0);
            SetSpeed(&htim4, 0);
            distance_mode = 0;
            target_d = 0;
            measure_d = 0;
            stop_flag = 1;
            outL = 0;
            outR = 0;
            return;
          } else if (remain / target_d <= 0.3f) {
            outL = (float)outL * 0.5;
            outR = (float)outR * 0.5;
          } else if (remain / target_d <= 0.1f) {
            outL = (float)outL * 0.2;
            outR = (float)outR * 0.2;
          }
        }

        if (outL > PWM_MAX)
          outL = PWM_MAX;
        if (outL < -PWM_MAX)
          outL = -PWM_MAX;
        if (outR > PWM_MAX)
          outR = PWM_MAX;
        if (outR < -PWM_MAX)
          outR = -PWM_MAX;

        SetSpeed(&htim4, outL); // 左轮
        SetSpeed(&htim1, outR); // 右轮
      }
      print_flag = 1;
    }
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RX_Buf_Active, UART_RX_BUF_LEN);
  TraceSensor_Init();
  TraceSensor_Start();
  // Init_BT(1500); // 这个只需要执行一次去初始化
  // OLED_Init();
  MPU6500_Init_With_Calibration();

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  // if (MPU6050_Init() == 0) {
  //   Error_Handler();
  // }

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim4);

  PID_Init(&pid_L, 10.0f, 10.0f, 3.0f, 999.0f, -999.0f, 400.0f);
  PID_Init(&pid_R, 10.0f, 10.0f, 3.0f, 999.0f, -999.0f, 400.0f);
  PID_Init(&pid_yaw, 1.0f, 0.0f, 10.0f, 10.0f, -10.0f, 400.0f);

  target_L = 0;
  target_R = 0;

  Motor_Init();
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  // MPU6050_Calculate_Angle();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // target_L = 3;
  // target_R = 3;
  // stop_flag = 0;
  MPU6500_flag = 1;
  while (1) {

    // OLED_ShowString(0, 0, "Motor Monitor", OLED_6X8);
    // OLED_Printf(16, 0, OLED_6X8, "Roll:%.2f, Pitch:%.2f, Yaw:%.2f
    // temp:%.2f\n",
    //             angle.pitch, angle.yaw, angle.temp);
    // OLED_Update();
    if (dataFlag) {
      dataFlag = 0;
      if (RX_Buf_Ready != NULL && RX_Buf_Ready[0] == '#') {
        if (!cmd_parser((char *)RX_Buf_Ready)) {
        }
      }
    }
    if (print_flag) {
      if (turn_l_flag) {
        target_Y += 0.5;
      } else if (turn_r_flag) {
        target_Y -= 0.5;
      }
      print_flag = 0;
      // float x = pid_yaw.Out;
      float y = mpu_data.yaw;

      msgf(MSG_LOG, "%s,%d",
           float_to_string(TICK_TO_V((M_speed_L + M_speed_R) / 2.0), float_buf1,
                           2),
           (int)y);
      // msgf(MSG_LOG, "%d, %d", (int)(target_L + pid_yaw.Out),
      //      (int)(target_R - pid_yaw.Out));
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BIN2_Pin | STBY_Pin | AIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SDA_Pin | SCL_Pin | AIN1_Pin | BIN1_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : BIN2_Pin STBY_Pin LED_Pin AIN2_Pin */
  GPIO_InitStruct.Pin = BIN2_Pin | STBY_Pin | LED_Pin | AIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SDA_Pin SCL_Pin BUZZ_Pin AIN1_Pin
                           BIN1_Pin */
  GPIO_InitStruct.Pin = SDA_Pin | SCL_Pin | BUZZ_Pin | AIN1_Pin | BIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// 自定义浮点数转字符串函数
char *float_to_string(float f, char *buffer, int decimal_places) {
  char *ptr = buffer;

  if (f < 0) {
    f = -f;
    *ptr++ = '-';
  }

  int integer_part = (int)f;
  float decimal = f - integer_part;

  // 整数部分
  sprintf(ptr, "%d", integer_part);
  ptr += strlen(ptr);

  // 小数部分 - 更精确的方法
  if (decimal_places > 0) {
    *ptr++ = '.';
    int multiplier = 1;
    for (int i = 0; i < decimal_places; i++) {
      multiplier *= 10;
    }
    int decimal_int = (int)(decimal * multiplier + 0.5f); // 四舍五入
    sprintf(ptr, "%0*d", decimal_places, decimal_int);
  }

  return buffer;
}

// static uint8_t BT_SendCmd(const char *cmd, uint32_t timeout) {
//   HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
//   uint32_t tick_start = HAL_GetTick();
//   while (!dataFlag) {
//     if (HAL_GetTick() - tick_start > timeout) {
//       return 0; // 超时
//     }
//   }
//   dataFlag = 0;
//   memset(RX_Buf_A, 0, sizeof(RX_Buf_A));
//   return 1; // 成功
// }

// void Init_BT(uint32_t time_out) {
//   if (!BT_SendCmd("AT\r\n", time_out))
//     Error_Handler();
//   if (!BT_SendCmd("AT+ORGL\r\n", time_out))
//     Error_Handler();
//   if (!BT_SendCmd("AT+ROLE=0\r\n", time_out))
//     Error_Handler();
//   // 这个版本的HC-05密码是6位数字
//   if (!BT_SendCmd("AT+PSWD=123456\r\n", time_out))
//     Error_Handler();
//   if (!BT_SendCmd("AT+NAME=STM32-OTA\r\n", time_out))
//     Error_Handler();
//   if (!BT_SendCmd("AT+UART=38400,0,0\r\n", time_out))
//     Error_Handler();
//   if (!BT_SendCmd("AT+CMODE=1\r\n", time_out))
//     Error_Handler();
//   if (!BT_SendCmd("AT+ADDR?\r\n", time_out)) // 20:4D:E9:1C:EE:99
//     Error_Handler();
//   if (!BT_SendCmd("AT+RESET\r\n", time_out))
//     Error_Handler();

//   dataFlag = 0;
//   memset(RX_Buf_A, 0, sizeof(RX_Buf_A));

//   // LED闪烁指示
//   for (uint8_t i = 0; i < 5; i++) {
//     HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//     HAL_Delay(100);
//   }
// }

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART2) {
    if (Size < UART_RX_BUF_LEN) {
      RX_Buf_Active[Size] = '\0'; // 确保字符串结尾
    } else {
      RX_Buf_Active[UART_RX_BUF_LEN - 1] = '\0';
      msg(MSG_ERROR, "RX overflow");
    }
    dataFlag = 1;

    RX_Buf_Ready = RX_Buf_Active;
    RX_Buf_Active = (RX_Buf_Active == RX_Buf_A) ? RX_Buf_B : RX_Buf_A;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RX_Buf_Active, UART_RX_BUF_LEN);
  }
}

static inline float clampf(float x, float minv, float maxv) {
  if (x > maxv)
    return maxv;
  if (x < minv)
    return minv;
  return x;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
  }
}

/**
 * @note https://zhuanlan.zhihu.com/p/114973609
 */
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    for (volatile uint32_t i = 0; i < 5000000; i++)
      ;
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
