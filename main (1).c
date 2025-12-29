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
#include "ssd1306.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_IDLE,      // Màn hình chờ/Menu
    STATE_SETTING,   // Màn hình cài đặt
    STATE_AUTO_RUN,  // Chạy PID
    STATE_MANUAL_RUN,// Chạy tay
    STATE_ERROR      // Lỗi hệ thống
} SystemState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SHT31_ADDR (0x44<<1)

#define FAN1_MIN_DUTY 2500
#define FAN2_MIN_DUTY 5200
#define FAN_MAX_DUTY 8399

#define LEVEL_1 3000
#define LEVEL_2 5500
#define LEVEL_3 8399

#define DEBOUNCE_DELAY 200
#define DEADBAND_LOW  -2.0f   // Dưới mức này là Lạnh
#define DEADBAND_HIGH  2.0f   // Trên mức này là Nóng

#define TEMP_MIN_LIMIT -5.0f
#define TEMP_MAX_LIMIT 85.0f

#define DISPLAY_REFRESH_RATE 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
//--------------BIEN TRANG THAI---------------
volatile SystemState current_state = STATE_IDLE;

//--------------DU LIEU CAM BIEN--------------
uint8_t cmd[2];
uint8_t data[6];
volatile float temp=0, hum=0;
int temp_nguyen= 0, temp_thap_phan = 0;
int hum_nguyen = 0, hum_thap_phan = 0;

float Kp = 5.0f, Ki = 1.0f, Kd = 0.01f;
float setpoint_temp = 25.0f;
volatile float integral = 0.0f, prev_error = 0.0f;

volatile float fan_power = 0.0f;
volatile float heater_power = 0.0f;

volatile int mode_auto = 1;
volatile int fan_level = 0;
volatile int heater_level = 0;

volatile float pid_output = 0.0f;

// --- BUTTON FLAGS
volatile uint8_t flag_btn_menu = 0;   // PB12
volatile uint8_t flag_btn_up = 0;     // PB13
volatile uint8_t flag_btn_down = 0;   //PB14

// --- TIMING ---
uint32_t last_sensor_read = 0;
uint32_t last_display_update = 0;

uint32_t last_update_time = 0;

//-----------------BIEN DIEU KHIEN FSM---------------


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void SHT31_ReadTempHumi(void);
uint32_t Calculate_PWM(float power_percent, uint32_t min_duty);
void TempControl(float measured_temp);
void Screen_Menu(void);
void Screen_Monitor(void);
void Check_Safety(void);
void Screen_Setting(void);

void State_Idle(void);
void State_Setting(void);
void State_Auto_Run(void);
void State_Manual_Run(void);
void State_Error(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// --- HAM DOC CAM BIEN SHT31 ---
void SHT31_ReadTempHumi(void)
{
	cmd[0] = 0x24;
	cmd[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c2, SHT31_ADDR, cmd, 2, 100);


	HAL_Delay(20);


	HAL_I2C_Master_Receive(&hi2c2, SHT31_ADDR, data, 6, 100);


    uint16_t rawT = (data[0] << 8) | data[1];
    uint16_t rawH = (data[3] << 8) | data[4];

    temp = -45 + 175 * ((float)rawT / 65535.0);
    hum = 100 * ((float)rawH / 65535.0);

}

uint32_t Calculate_PWM(float power_percent, uint32_t min_duty) {
    if (power_percent < 1.0f) return 0;
    if (power_percent > 100.0f) power_percent = 100.0f;

    uint32_t pwm = min_duty + (uint32_t)((power_percent / 100.0f) * (FAN_MAX_DUTY - min_duty));
    if (pwm > FAN_MAX_DUTY) pwm = FAN_MAX_DUTY;
    return pwm;
}

void TempControl(float measured_temp) {
	if (current_state != STATE_AUTO_RUN) return;
    float error = setpoint_temp - measured_temp;
    integral += error * 0.1f;

    // Anti-windup
    if (integral > 100.0f) integral = 100.0f;
    if (integral < -100.0f) integral = -100.0f;

    float derivative = (error - prev_error) / 1.0f;
    float output = Kp * error + Ki * integral + Kd * derivative;

    fan_power = 0.0f;
    heater_power = 0.0f;

    if (output < DEADBAND_LOW) { //LAM LANH
            float pid_percent = fabs(output);
            if (pid_percent > 100.0f) pid_percent = 100.0f;

            uint32_t pwm1 = Calculate_PWM(pid_percent, FAN1_MIN_DUTY);
            uint32_t pwm2 = Calculate_PWM(pid_percent, FAN2_MIN_DUTY);

            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm1);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm2);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

            fan_power = pid_percent;

        } else if (output > DEADBAND_HIGH) { //LAM NONG
            float pid_percent = output;
            if (pid_percent > 100.0f) pid_percent = 100.0f;
            uint32_t duty_heat = (uint32_t)(pid_percent / 100.0f * 8399.0f);
            if (duty_heat > 8399) duty_heat = 8399;

            heater_power = pid_percent;

            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_heat);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

        } else //KHONG HOAT DONG
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        }

        prev_error = error;
}


void State_Idle(void) {
    // Tắt toàn bộ tải
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    // Điều hướng
    if (flag_btn_menu) {
        current_state = STATE_SETTING;
        flag_btn_menu = 0;
    }
    if (flag_btn_up) {
        current_state = STATE_AUTO_RUN;
        integral = 0; // Reset PID
        flag_btn_up = 0;
    }
    if (flag_btn_down) {
        current_state = STATE_MANUAL_RUN;
        fan_level = 0; heater_level = 0;
        flag_btn_down = 0;
    }
}

// 2. Xử lý trạng thái SETTING
void State_Setting(void) {
    // Đảm bảo tải tắt khi đang chỉnh
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    // Logic chỉnh Setpoint
    if (flag_btn_up) {
        setpoint_temp += 1.0f;
        if (setpoint_temp > 50.0f) setpoint_temp = 50.0f;
        flag_btn_up = 0;
    }
    if (flag_btn_down) {
        setpoint_temp -= 1.0f;
        if (setpoint_temp < 10.0f) setpoint_temp = 10.0f;
        flag_btn_down = 0;
    }
    if (flag_btn_menu) {
        current_state = STATE_IDLE;
        flag_btn_menu = 0;
    }
}


void State_Auto_Run(void) {

    if (flag_btn_menu) {
        current_state = STATE_IDLE;
        flag_btn_menu = 0;
    }

    flag_btn_up = 0;
    flag_btn_down = 0;
}

// 4. Xử lý trạng thái MANUAL RUN
void State_Manual_Run(void) {
    if (flag_btn_up) { // Tăng Fan
        fan_level++; if(fan_level > 3) fan_level=0;
        heater_level = 0;

        float target = (fan_level==1)?33:(fan_level==2?66:(fan_level==3?100:0));
        uint32_t pwm1 = Calculate_PWM(target, FAN1_MIN_DUTY);
        uint32_t pwm2 = Calculate_PWM(target, FAN2_MIN_DUTY);

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

        fan_power = target; heater_power = 0;
        flag_btn_up = 0;
    }

    if (flag_btn_down) { // Tăng Heat
        heater_level++; if(heater_level > 3) heater_level=0;
        fan_level = 0;

        uint32_t pwm = 0;
        if(heater_level==1) pwm = LEVEL_1;
        else if(heater_level==2) pwm = LEVEL_2;
        else if(heater_level==3) pwm = LEVEL_3;

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

        heater_power = (float)pwm/8399.0f*100; fan_power = 0;
        flag_btn_down = 0;
    }

    if (flag_btn_menu) {
        current_state = STATE_IDLE;
        flag_btn_menu = 0;
    }
}


void State_Error(void) {
    // Ngắt toàn bộ tải khẩn cấp
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    // Nhấn Menu để Reset về Idle
    if (flag_btn_menu) {
        current_state = STATE_IDLE;
        flag_btn_menu = 0;
    }
}

// --- CÁC HÀM PHỤ TRỢ KHÁC ---
void Check_Safety(void) {
    if (temp < TEMP_MIN_LIMIT || temp > TEMP_MAX_LIMIT) {
        current_state = STATE_ERROR;
    }
}

uint32_t GetPWM_From_Level(int level) {
    switch(level) {
        case 1: return LEVEL_1;
        case 2: return LEVEL_2;
        case 3: return LEVEL_3;
        default: return 0;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        TempControl(temp);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint32_t last_irq = 0;
	    uint32_t now = HAL_GetTick();
	    if (now - last_irq < 200) return;
	    last_irq = now;

	    if (GPIO_Pin == GPIO_PIN_12) flag_btn_menu = 1;
	    if (GPIO_Pin == GPIO_PIN_13) flag_btn_up = 1;
	    if (GPIO_Pin == GPIO_PIN_14) flag_btn_down = 1;
}

void Screen_Menu(void){
	SSD1306_GotoXY(25, 0); SSD1306_Puts("MAIN MENU", &Font_7x10, 1);
	SSD1306_GotoXY(0, 15); SSD1306_Puts("BUTTON 1: SETTING", &Font_7x10, 1);
	SSD1306_GotoXY(0, 30); SSD1306_Puts("BUTTON 2: AUTO RUN", &Font_7x10, 1);
	SSD1306_GotoXY(0, 45); SSD1306_Puts("BUTTON 3: MANUAL", &Font_7x10, 1);
}
void Screen_Setting(void){
    SSD1306_GotoXY(30, 0); SSD1306_Puts("SET TEMP", &Font_7x10, 1);
    int s_i = (int)setpoint_temp; int s_d = (int)((setpoint_temp-s_i)*10);
    SSD1306_Printf(40, 15, &Font_7x10, 1, "%d.%dC", s_i, s_d);
    SSD1306_GotoXY(15, 33); SSD1306_Puts("[OK]  [+]  [-]", &Font_7x10, 1);
}
void Screen_Monitor(void){
	int t_i = (int)temp; int t_d = (int)((fabs(temp)-abs(t_i))*10);
	int h_i = (int)hum;
	if (current_state == STATE_AUTO_RUN) {
	    SSD1306_Printf(0, 0, &Font_7x10, 1, "AUTO|SETPOINT:%dC", (int)setpoint_temp);
	    SSD1306_Printf(0, 15, &Font_7x10, 1, "TEMP:%d.%dC", t_i, t_d);
	    SSD1306_Printf(0, 30, &Font_7x10, 1, "HUMI:%d%%", h_i);
        SSD1306_Printf(0, 45, &Font_7x10, 1, "FAN:%d%% HEAT:%d%%", (int)fan_power, (int)heater_power);
	} else if (current_state == STATE_MANUAL_RUN) {
		SSD1306_GotoXY(0, 0); SSD1306_Puts("MANUAL", &Font_7x10, 1);
		SSD1306_Printf(0, 15, &Font_7x10, 1, "TEMP:%d.%dC", t_i, t_d);
	    SSD1306_Printf(0, 30, &Font_7x10, 1, "HUMI:%d%%", h_i);
	    SSD1306_Printf(0, 45, &Font_7x10, 1, "FAN:L%d HEAT:L%d", fan_level, heater_level);
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  if (SSD1306_Init() == 0) {
        Error_Handler();
    }

  	SSD1306_Fill(SSD1306_COLOR_BLACK);
  	SSD1306_GotoXY(25, 10);
  	SSD1306_Puts("HE THONG DO", &Font_7x10, SSD1306_COLOR_WHITE);
  	SSD1306_GotoXY(18, 25);
  	SSD1306_Puts("VA DIEU CHINH", &Font_7x10, SSD1306_COLOR_WHITE);
  	SSD1306_GotoXY(11, 40);
  	SSD1306_Puts("NHIET DO, DO AM", &Font_7x10, SSD1306_COLOR_WHITE);
  	SSD1306_UpdateScreen();
  	HAL_Delay(4000);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
      {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

    	uint32_t now = HAL_GetTick();
    	      if (current_state != STATE_SETTING && (now - last_sensor_read >= 200)) {
    	          SHT31_ReadTempHumi();
    	          Check_Safety();
    	          last_sensor_read = now;
    	      }

            // --- 2. FSM LOGIC ---
            switch (current_state) {
                case STATE_IDLE:       State_Idle();       break;
                case STATE_SETTING:    State_Setting();    break;
                case STATE_AUTO_RUN:   State_Auto_Run();   break;
                case STATE_MANUAL_RUN: State_Manual_Run(); break;
                case STATE_ERROR:      State_Error();      break;
                default:               State_Idle();       break;
            }

            // --- 3. HIỂN THỊ MÀN HÌNH ---
            if (now - last_display_update >= DISPLAY_REFRESH_RATE) {
                SSD1306_Fill(SSD1306_COLOR_BLACK);

                if (current_state == STATE_IDLE) Screen_Menu();
                else if (current_state == STATE_SETTING) Screen_Setting();
                else if (current_state == STATE_ERROR) {
                    SSD1306_GotoXY(20, 10); SSD1306_Puts("ERROR!", &Font_11x18, 1);
                }
                else Screen_Monitor();

                SSD1306_UpdateScreen();
                last_display_update = now;
            }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hi2c1.Init.ClockSpeed = 400000;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
