/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   独立看门狗配置。
  ******************************************************************************
  */
#include "iwdg.h"

/** 独立看门狗 HAL 句柄。 */
IWDG_HandleTypeDef hiwdg;

void MX_IWDG_Init(void)
{
  /*
   * IWDG 由独立 LSI 驱动，即使主时钟异常也可复位 MCU。
   * 典型超时：(499 + 1) * 256 / 32000 Hz = 4 s。
   * 禁用窗口限制后，主循环可在任意安全时刻刷新计数器。
   */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
  hiwdg.Init.Reload = 499U;

  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}
