/**
  ******************************************************************************
  * @file    iwdg.h
  * @brief   独立看门狗初始化接口。
  ******************************************************************************
  */
#ifndef IWDG_H
#define IWDG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/** 全局独立看门狗句柄，供主循环定期喂狗。 */
extern IWDG_HandleTypeDef hiwdg;

/**
 * @brief 初始化并启动独立看门狗。
 * @note LSI 按典型 32 kHz 计算，256 分频、重装值 499 对应约 4 秒超时。
 */
void MX_IWDG_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* IWDG_H */
