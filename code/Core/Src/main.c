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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "pt1000_app.h"
#include "temperature_control.h"
#include "pwm_duty_test.h"
#include "board_io.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/** @brief PWM 输出运行模式。 */
typedef enum
{
  PWM_RUN_MODE_WORK = 0, /**< 正常工作：由四路温度反馈和 PID 调整占空比。 */
  PWM_RUN_MODE_TEST      /**< 测试模式：循环输出五档固定占空比。 */
} PWM_RunMode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** 只需修改此宏即可在正常 PID 工作模式和 PWM 阶梯测试模式之间切换。 */
#define PWM_RUN_MODE PWM_RUN_MODE_WORK

/*
 * 第二轮整定加入小积分项。Kp 的单位为“占空比百分点/℃”：
 * 例如误差为 10 ℃、Kp=3 时，未限幅的 P 输出为 30%。
 * Ki 的单位为“占空比百分点/(℃·s)”，用于缓慢消除纯 P 稳态误差。
 */
#define TEMPERATURE_SETPOINT_C 42.0f
#define TEMPERATURE_PID_KP      3.0f
#define TEMPERATURE_PID_KI      0.025f
#define TEMPERATURE_PID_KD      0.0f
/** 工作模式下允许输出的最大高有效 PWM 占空比。 */
#define TEMPERATURE_PWM_MAX_DUTY_PERCENT 30.0f
/** 积分项单独限幅，避免积分独自给出过大的持续加热功率。 */
#define TEMPERATURE_PID_INTEGRAL_MAX_PERCENT 12.0f
/** 12 V 实测维持功率约 7.4%，预置 6% 可减轻接近目标时的温度回落。 */
#define TEMPERATURE_PID_INTEGRAL_INITIAL_PERCENT 5.5f
/** 任一有效通道到达此温度时锁定关闭加热，直到重新上电。 */
#define TEMPERATURE_OVERTEMPERATURE_C 48.0f
/** 非 0 时每轮输出一行纯数字 FireWater 温控数据，当前采集频率为 5 Hz。 */
#define TEMPERATURE_PID_TELEMETRY_ENABLE 1U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static PT1000_App pt1000_app;
static PT1000_AppConfig pt1000_config;
static PWM_DutyTest pwm_duty_test;
static TemperatureControl temperature_control;
static TemperatureControl_Config temperature_control_config;

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
  MX_GPIO_Init();
  /* USART2/H1 用于输出纯数字 FireWater 温控数据。 */
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  BoardIO_Init();

  PT1000_AppGetDefaultConfig(&pt1000_config);
  PT1000_AppInit(&pt1000_app, &hspi1, &huart2,
                 ADS124S08_CS_GPIO_Port, ADS124S08_CS_Pin,
                 GPIOA, GPIO_PIN_6, &pt1000_config);

  if (PWM_RUN_MODE == PWM_RUN_MODE_TEST)
  {
    /* 测试模式绕过 PID，避免温度有效性检查把测试占空比重新置为 0%。 */
    if (PWM_DutyTest_Init(&pwm_duty_test, &htim17,
                          TIM_CHANNEL_1) != HAL_OK)
    {
      __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0U);
    }
  }
  else
  {
    TemperatureControl_GetDefaultConfig(&temperature_control_config);
    /* 根据首轮纯 P 曲线加入小积分项，用于消除约 2.37 ℃ 稳态误差。 */
    temperature_control_config.setpoint_c = TEMPERATURE_SETPOINT_C;
    temperature_control_config.pid.kp = TEMPERATURE_PID_KP;
    temperature_control_config.pid.ki = TEMPERATURE_PID_KI;
    temperature_control_config.pid.kd = TEMPERATURE_PID_KD;
    temperature_control_config.pid.output_max =
        TEMPERATURE_PWM_MAX_DUTY_PERCENT;
    temperature_control_config.pid.integral_max =
        TEMPERATURE_PID_INTEGRAL_MAX_PERCENT;
    temperature_control_config.pid.integral_initial =
        TEMPERATURE_PID_INTEGRAL_INITIAL_PERCENT;
    temperature_control_config.feedback_mode =
        TEMPERATURE_FEEDBACK_MAXIMUM;
    temperature_control_config.overtemperature_c =
        TEMPERATURE_OVERTEMPERATURE_C;
    temperature_control_config.telemetry_enabled =
        TEMPERATURE_PID_TELEMETRY_ENABLE;
    if (TemperatureControl_Init(&temperature_control, &htim17, TIM_CHANNEL_1,
                                &pt1000_app,
                                &temperature_control_config) != HAL_OK)
    {
      /* PWM/PID 初始化失败时保持 PD1/EN 为安全的非加热状态。 */
      __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0U);
    }
  }
  /* 定时器输出启动后再把 PD1 切为复用功能，避免 EN 出现启动毛刺。 */
  MX_TIM17_PWM_GPIO_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    BoardButton_Update();
    PT1000_AppTask(&pt1000_app);
    if (PWM_RUN_MODE == PWM_RUN_MODE_TEST)
    {
      PWM_DutyTest_Task(&pwm_duty_test);
    }
    else
    {
      TemperatureControl_Task(&temperature_control);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  /*
   * 发生不可恢复错误时关闭加热并停机。系统时钟等早期初始化也可能进入
   * 本函数，因此只有 TIM17 已绑定实例后才能访问其比较寄存器。
   */
  if (htim17.Instance == TIM17)
  {
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0U);
  }
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
