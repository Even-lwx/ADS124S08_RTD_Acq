/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   TIM17 PWM configuration for PD1/EN.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32g0xx_hal_tim.h"

extern TIM_HandleTypeDef htim17;

void MX_TIM17_Init(void);
void MX_TIM17_PWM_GPIO_Init(void);

#ifdef __cplusplus
}
#endif

#endif
