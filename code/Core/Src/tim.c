/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   TIM17 CH1 generates a 1-kHz PWM on PD1/EN (AF1).
  ******************************************************************************
  */
/* USER CODE END Header */
#include "tim.h"

#define TIM17_CH1_PD1_AF 1U

TIM_HandleTypeDef htim17;

void MX_TIM17_PWM_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  /* STM32G031 数据手册 DS12992 第 41 页：PD1 的 AF1 为 TIM17_CH1。 */
  gpio_init.Pin = PWM_OUT_Pin;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Alternate = TIM17_CH1_PD1_AF;
  HAL_GPIO_Init(PWM_OUT_GPIO_Port, &gpio_init);
}

void MX_TIM17_Init(void)
{
  TIM_OC_InitTypeDef pwm_config = {0};

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 63;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }

  pwm_config.OCMode = TIM_OCMODE_PWM1;
  pwm_config.Pulse = 0;
  pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;
  pwm_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  pwm_config.OCFastMode = TIM_OCFAST_DISABLE;
  pwm_config.OCIdleState = TIM_OCIDLESTATE_RESET;
  pwm_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &pwm_config, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_base_handle)
{
  if (tim_base_handle->Instance == TIM17)
  {
    __HAL_RCC_TIM17_CLK_ENABLE();
  }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *tim_pwm_handle)
{
  if (tim_pwm_handle->Instance == TIM17)
  {
    __HAL_RCC_TIM17_CLK_ENABLE();
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_base_handle)
{
  if (tim_base_handle->Instance == TIM17)
  {
    __HAL_RCC_TIM17_CLK_DISABLE();
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *tim_pwm_handle)
{
  if (tim_pwm_handle->Instance == TIM17)
  {
    __HAL_RCC_TIM17_CLK_DISABLE();
    HAL_GPIO_DeInit(PWM_OUT_GPIO_Port, PWM_OUT_Pin);
  }
}
