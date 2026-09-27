/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Seamless MPPT Buck-Boost)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "ina219.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_MIN 91   // 5% Duty Cycle
#define PWM_MAX 1646 // 90% Duty Cycle
#define VREF 3.3f
#define ADC_MAX 4095.0f

// Hệ số cầu chia áp
#define DIVIDER_VIN 11.0f  // (100k + 10k) / 10k
#define DIVIDER_VOUT 11.0f // (100k + 10k) / 10k
#define DIVIDER_ACS 2.0f   // (10k + 10k) / 10k

// Thông số ACS712
#define ACS712_SENSITIVITY 0.100f
#define ACS712_OFFSET 2.5f
#define EMA_ALPHA 0.1f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */
INA219_HandleTypeDef hina219;
const float VREFINT_VOLTAGE = 1.20f;
float dynamic_ADC_REF = 3.3f;

// Biến lưu trữ cho thuật toán MPPT P&O
float p_in_prev = 0.0f;
float v_in_prev = 0.0f;
float mppt_step = 0.5f;
float power_threshold = 0.1f;

float v_in = 0.0f;
float v_out = 0.0f;
float i_in = 0.0f;
float i_out = 0.0f;

float i_out_filtered = 0.0f;
float v_in_filtered = 0.0f;
float v_out_filtered = 0.0f;

// --- BIẾN ĐIỀU KHIỂN BUCK-BOOST ---
float current_duty_buck = 90.0f;
float current_duty_boost = 90.0f;

// Biến cho bộ đếm thời gian phân luồng tác vụ
uint32_t last_lcd_time = 0;
uint32_t last_measure_time = 0;
uint32_t last_mppt_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void Control_Loop(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Set_PWM_Safe(TIM_HandleTypeDef *htim, uint32_t Channel, float duty_percent)
{
    uint32_t ccr_val = (uint32_t)((duty_percent / 100.0) * 1828.0);
    if (ccr_val < PWM_MIN) ccr_val = PWM_MIN;
    if (ccr_val > PWM_MAX) ccr_val = PWM_MAX;
    __HAL_TIM_SET_COMPARE(htim, Channel, ccr_val);
}

uint32_t Read_ADC_Channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return adc_value;
}
/* Hàm tự động tính toán lại điện áp nguồn thực tế của STM32 */
// THÊM LẠI HÀM LỌC EMA BỊ THIẾU
float EMA(float input, float previous) {
    return EMA_ALPHA * input + (1.0f - EMA_ALPHA) * previous;
}

/* Hàm tự động tính toán lại điện áp nguồn thực tế của STM32 */
void Calibrate_VDDA(void) {
    // ĐÃ SỬA LẠI TÊN HÀM CHO ĐÚNG: Read_ADC_Channel
    uint32_t vrefint_adc = Read_ADC_Channel(ADC_CHANNEL_VREFINT);

    if (vrefint_adc > 0) { // Tránh lỗi chia cho 0
        // Tính ra VDDA thực tế đang là bao nhiêu
        float current_vdda = (VREFINT_VOLTAGE * 4095.0f) / (float)vrefint_adc;

        // Đưa qua bộ lọc EMA
        dynamic_ADC_REF = EMA(current_vdda, dynamic_ADC_REF);
    }
}


void Control_Loop(void)
{
    float p_in = v_in * i_in;
    float delta_p = p_in - p_in_prev;
    float delta_v = v_in - v_in_prev;

    int mppt_dir = 0;

    // =========================================================
    // 1. CƠ CHẾ MỒI DÒNG (KICKSTART) - Phá vỡ trạng thái đóng băng
    // =========================================================
    if (i_in < 0.1f) { // Dòng điện < 50mA (Gần như không có dòng chảy qua)
        // Bỏ qua P&O, ép mạch phải "rút thêm dòng" để mồi điện áp lên cao
        mppt_dir = 1;
    }
    // =========================================================
    // 2. THUẬT TOÁN P&O (Chạy bình thường khi đã có dòng điện)
    // =========================================================
    else if (delta_p > power_threshold || delta_p < -power_threshold) {
        if (delta_p > 0) {
            if (delta_v > 0) mppt_dir = -1;
            else             mppt_dir = 1;
        } else {
            if (delta_v > 0) mppt_dir = 1;
            else             mppt_dir = -1;
        }
    }

    // =========================================================
    // 3. THỰC THI MPPT TRƯỢT MƯỢT MÀ (50% - 90%)
    // =========================================================
    if (mppt_dir == 1) {
        // CẦN RÚT THÊM DÒNG (Mồi áp)
        if (current_duty_buck < 90.0f) {
            current_duty_buck += mppt_step;  // Mở dần Buck trước
        } else {
            current_duty_boost -= mppt_step; // Ép Boost mạnh lên (hạ PWM về 50%) để đẩy áp qua diode
        }
    }
    else if (mppt_dir == -1) {
        // CẦN GIẢM DÒNG RÚT
        if (current_duty_boost < 90.0f) {
            current_duty_boost += mppt_step; // Xả bớt Boost
        } else {
            current_duty_buck -= mppt_step;  // Khép dần Buck
        }
    }

    // KHÓA BIÊN ĐỘ AN TOÀN QUY ƯỚC
    if (current_duty_buck < 5.0f)  current_duty_buck = 5.0f;
    if (current_duty_buck > 90.0f)  current_duty_buck = 90.0f;

    if (current_duty_boost < 70.0f) current_duty_boost = 70.0f;
    if (current_duty_boost > 90.0f) current_duty_boost = 90.0f;

    Set_PWM_Safe(&htim3, TIM_CHANNEL_3, current_duty_buck);
    Set_PWM_Safe(&htim1, TIM_CHANNEL_1, current_duty_boost);

    p_in_prev = p_in;
    v_in_prev = v_in;
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  lcd_init();
  lcd_clear_display();
  lcd_goto_XY(1, 0);
  lcd_send_string("MPPT Buck-Boost");
  HAL_Delay(1000);
  lcd_clear_display();

  INA219_Init(&hina219, &hi2c2, 0x40);
  HAL_ADCEx_Calibration_Start(&hadc1);

  char lcd_buffer[21];

  // Khởi tạo thời gian cho các luồng tác vụ
  uint32_t tick_start = HAL_GetTick();
  last_lcd_time = tick_start;
  last_measure_time = tick_start;
  last_mppt_time = tick_start;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      uint32_t current_time = HAL_GetTick();

      // ==========================================================
      // TASK 1: ĐO LƯỜNG & LỌC NHIỄU (Chạy nhanh mỗi 10ms)
      // ==========================================================
      if (current_time - last_measure_time >= 10)
      {
          last_measure_time = current_time;
          Calibrate_VDDA();

          i_in = INA219_GetCurrent_A(&hina219) * 2;

          // Lọc Vin (Kênh 7)
          uint32_t adc_vin_sum = 0;
          for(int i = 0; i < 10; i++) adc_vin_sum += Read_ADC_Channel(ADC_CHANNEL_7);
          float adc_vin_avg = (float)adc_vin_sum / 10.0f;
          float v_in_raw = (adc_vin_avg / ADC_MAX) * VREF * DIVIDER_VIN;
          v_in_filtered = (EMA_ALPHA * v_in_raw) + ((1.0f - EMA_ALPHA) * v_in_filtered);
          v_in = v_in_filtered;

          // Lọc Vout (Kênh 0)
          uint32_t adc_vout_sum = 0;
          for(int i = 0; i < 10; i++) adc_vout_sum += Read_ADC_Channel(ADC_CHANNEL_0);
          float adc_vout_avg = (float)adc_vout_sum / 10.0f;
          float v_out_raw = (adc_vout_avg / ADC_MAX) * VREF * DIVIDER_VOUT;
          v_out_filtered = (EMA_ALPHA * v_out_raw) + ((1.0f - EMA_ALPHA) * v_out_filtered);
          v_out = v_out_filtered;

          // Lọc Iout (Kênh 1)
          uint32_t adc_acs_sum = 0;
          for(int i = 0; i < 10; i++) adc_acs_sum += Read_ADC_Channel(ADC_CHANNEL_1);
          float adc_acs_avg = (float)adc_acs_sum / 10.0f;
          float v_acs_mcu = (adc_acs_avg / ADC_MAX) * VREF;
          float v_acs_actual = v_acs_mcu * DIVIDER_ACS;
          float i_out_raw = (v_acs_actual - ACS712_OFFSET) / ACS712_SENSITIVITY;
          i_out_filtered = (EMA_ALPHA * i_out_raw) + ((1.0f - EMA_ALPHA) * i_out_filtered);

          if (i_out_filtered < 0.05f && i_out_filtered > -0.05f) {
              i_out = 0.0f;
          } else {
              i_out = i_out_filtered;
          }
          i_out = i_out - (v_out-13.4f)/4.0f;

      }

      // ==========================================================
      // TASK 2: ĐIỀU KHIỂN MPPT (Chạy chậm mỗi 100ms tránh EMI)
      // ==========================================================
      if (current_time - last_mppt_time >= 10)
      {
          last_mppt_time = current_time;

          if (v_in >= 1.0f) {
              Control_Loop();
          } else {
              Set_PWM_Safe(&htim3, TIM_CHANNEL_3, 90.0f);
              Set_PWM_Safe(&htim1, TIM_CHANNEL_1, 90.0f);
          }
      }

      // ==========================================================
      // TASK 3: HIỂN THỊ LCD (Cập nhật mỗi 500ms chống treo I2C)
      // ==========================================================
      if (current_time - last_lcd_time >= 500)
      {
          last_lcd_time = current_time;

          lcd_goto_XY(1, 0);
          sprintf(lcd_buffer, "Vi:%.1fV,Vo:%.1fV  ", v_in, v_out);
          lcd_send_string(lcd_buffer);

          lcd_goto_XY(2, 0);
          sprintf(lcd_buffer, "Ii:%.1fA,Io:%.1fA  ", i_in, i_out);
          lcd_send_string(lcd_buffer);

		lcd_goto_XY(3, 0);
		sprintf(lcd_buffer, "Bck:%.0f%% Bst:%.0f%%  ", current_duty_buck, current_duty_boost);
		lcd_send_string(lcd_buffer);
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
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
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1828;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1828;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
