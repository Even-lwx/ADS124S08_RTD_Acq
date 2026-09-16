/**
 * @file temperature_control.h
 * @brief 四路温度反馈与仅加热 PWM 控制框架。
 */
#ifndef TEMPERATURE_CONTROL_H
#define TEMPERATURE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pid.h"
#include "pt1000_app.h"
#include "stm32g0xx_hal_tim.h"

typedef enum
{
  /** 可选用四路平均值、极值或单通道作为 PID 反馈。 */
  TEMPERATURE_FEEDBACK_AVERAGE = 0,
  TEMPERATURE_FEEDBACK_MAXIMUM,
  TEMPERATURE_FEEDBACK_MINIMUM,
  TEMPERATURE_FEEDBACK_CHANNEL_1,
  TEMPERATURE_FEEDBACK_CHANNEL_2,
  TEMPERATURE_FEEDBACK_CHANNEL_3,
  TEMPERATURE_FEEDBACK_CHANNEL_4
} TemperatureControl_FeedbackMode;

typedef struct
{
  /** 温度设定值、PID 参数、反馈策略以及总使能。 */
  float setpoint_c;
  PID_Config pid;
  TemperatureControl_FeedbackMode feedback_mode;
  uint8_t enabled;
} TemperatureControl_Config;

typedef struct
{
  /** 控制实例及运行状态；duty_percent 始终表示高电平有效占空比。 */
  TIM_HandleTypeDef *pwm_timer;
  uint32_t pwm_channel;
  PT1000_App *temperature_app;
  TemperatureControl_Config config;
  PID_Controller pid;
  uint32_t last_sequence;
  uint32_t last_update_tick;
  float feedback_c;
  float duty_percent;
} TemperatureControl;

/** @brief 载入安全默认值；PID 系数默认为 0，需在实机整定。 */
void TemperatureControl_GetDefaultConfig(TemperatureControl_Config *config);
/** @brief 绑定 PD1/TIM17 PWM 与温度应用，强制高有效并从 0% 启动。 */
HAL_StatusTypeDef TemperatureControl_Init(
    TemperatureControl *control,
    TIM_HandleTypeDef *pwm_timer,
    uint32_t pwm_channel,
    PT1000_App *temperature_app,
    const TemperatureControl_Config *config);
/** @brief 每当出现一轮新温度时更新一次仅加热 PID。 */
void TemperatureControl_Task(TemperatureControl *control);
/** @brief 立即归零 PWM 并清除 PID 历史状态。 */
void TemperatureControl_Stop(TemperatureControl *control);

#ifdef __cplusplus
}
#endif

#endif
