/**
 * @file temperature_control.c
 * @brief 四路反馈选择、仅加热 PID 与高有效 PWM 输出实现。
 *
 * 安全策略：任一路温度无效、控制禁用或采样时间异常时，PWM 归零并清积分。
 */
#include "temperature_control.h"

#include <stddef.h>
#include <string.h>

static void TemperatureControl_SetDuty(TemperatureControl *control,
                                       float duty_percent)
{
  uint32_t period_counts;
  uint32_t compare;

  if (duty_percent < 0.0f)
  {
    duty_percent = 0.0f;
  }
  else if (duty_percent > 100.0f)
  {
    duty_percent = 100.0f;
  }

  /* CCR/周期计数换算为 0～100%；100% 时 CCR=ARR+1，输出持续为高。 */
  period_counts = __HAL_TIM_GET_AUTORELOAD(control->pwm_timer) + 1U;
  compare = (uint32_t)(duty_percent * (float)period_counts / 100.0f + 0.5f);
  __HAL_TIM_SET_COMPARE(control->pwm_timer, control->pwm_channel, compare);
  control->duty_percent = duty_percent;
}

static uint8_t TemperatureControl_GetFeedback(
    const TemperatureControl *control,
    const PT1000_AppSnapshot *snapshot,
    float *feedback_c)
{
  uint8_t channel;

  for (channel = 0U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
  {
    if (snapshot->valid[channel] == 0U)
    {
      return 0U;
    }
  }

  switch (control->config.feedback_mode)
  {
    case TEMPERATURE_FEEDBACK_MAXIMUM:
      *feedback_c = snapshot->temperature_c[0];
      for (channel = 1U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
      {
        if (snapshot->temperature_c[channel] > *feedback_c)
        {
          *feedback_c = snapshot->temperature_c[channel];
        }
      }
      break;

    case TEMPERATURE_FEEDBACK_MINIMUM:
      *feedback_c = snapshot->temperature_c[0];
      for (channel = 1U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
      {
        if (snapshot->temperature_c[channel] < *feedback_c)
        {
          *feedback_c = snapshot->temperature_c[channel];
        }
      }
      break;

    case TEMPERATURE_FEEDBACK_CHANNEL_1:
    case TEMPERATURE_FEEDBACK_CHANNEL_2:
    case TEMPERATURE_FEEDBACK_CHANNEL_3:
    case TEMPERATURE_FEEDBACK_CHANNEL_4:
      channel = (uint8_t)(control->config.feedback_mode -
                          TEMPERATURE_FEEDBACK_CHANNEL_1);
      *feedback_c = snapshot->temperature_c[channel];
      break;

    case TEMPERATURE_FEEDBACK_AVERAGE:
    default:
      *feedback_c = 0.0f;
      for (channel = 0U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
      {
        *feedback_c += snapshot->temperature_c[channel];
      }
      *feedback_c /= (float)PT1000_APP_CHANNEL_COUNT;
      break;
  }
  return 1U;
}

void TemperatureControl_GetDefaultConfig(TemperatureControl_Config *config)
{
  if (config == NULL)
  {
    return;
  }
  memset(config, 0, sizeof(*config));
  config->setpoint_c = 25.0f;
  config->pid.kp = 0.0f;
  config->pid.ki = 0.0f;
  config->pid.kd = 0.0f;
  config->pid.output_min = 0.0f;
  config->pid.output_max = 100.0f;
  config->pid.integral_min = 0.0f;
  config->pid.integral_max = 100.0f;
  config->feedback_mode = TEMPERATURE_FEEDBACK_AVERAGE;
  config->enabled = 1U;
}

HAL_StatusTypeDef TemperatureControl_Init(
    TemperatureControl *control,
    TIM_HandleTypeDef *pwm_timer,
    uint32_t pwm_channel,
    PT1000_App *temperature_app,
    const TemperatureControl_Config *config)
{
  HAL_StatusTypeDef status;

  if ((control == NULL) || (pwm_timer == NULL) ||
      (temperature_app == NULL) || (config == NULL) ||
      (pwm_channel != TIM_CHANNEL_1) ||
      (config->pid.output_min < 0.0f) ||
      (config->pid.output_max > 100.0f) ||
      (config->pid.output_min > config->pid.output_max) ||
      (config->pid.integral_min > config->pid.integral_max) ||
      (config->feedback_mode > TEMPERATURE_FEEDBACK_CHANNEL_4))
  {
    return HAL_ERROR;
  }

  memset(control, 0, sizeof(*control));
  control->pwm_timer = pwm_timer;
  control->pwm_channel = pwm_channel;
  control->temperature_app = temperature_app;
  control->config = *config;
  PID_Init(&control->pid, &config->pid);

  /* PD1/EN 固定为高有效：逻辑 0% 持续低，PWM 有效脉冲为高电平。 */
  CLEAR_BIT(pwm_timer->Instance->CCER, TIM_CCER_CC1P);

  TemperatureControl_SetDuty(control, 0.0f);
  status = HAL_TIM_PWM_Start(pwm_timer, pwm_channel);
  return status;
}

void TemperatureControl_Stop(TemperatureControl *control)
{
  if (control == NULL)
  {
    return;
  }
  TemperatureControl_SetDuty(control, 0.0f);
  PID_Reset(&control->pid);
}

void TemperatureControl_Task(TemperatureControl *control)
{
  PT1000_AppSnapshot snapshot;
  float feedback_c;
  float error;
  float dt_seconds;

  if ((control == NULL) || (control->temperature_app == NULL))
  {
    return;
  }

  PT1000_AppGetSnapshot(control->temperature_app, &snapshot);
  if ((snapshot.sequence == 0U) ||
      (snapshot.sequence == control->last_sequence))
  {
    return;
  }
  control->last_sequence = snapshot.sequence;

  if ((control->config.enabled == 0U) ||
      (TemperatureControl_GetFeedback(control, &snapshot, &feedback_c) == 0U))
  {
    TemperatureControl_Stop(control);
    control->last_update_tick = snapshot.tick_ms;
    return;
  }

  if (control->last_update_tick == 0U)
  {
    dt_seconds = 1.0f;
  }
  else
  {
    dt_seconds = (float)(snapshot.tick_ms - control->last_update_tick) / 1000.0f;
    if ((dt_seconds <= 0.0f) || (dt_seconds > 10.0f))
    {
      dt_seconds = 1.0f;
      PID_Reset(&control->pid);
    }
  }
  control->last_update_tick = snapshot.tick_ms;
  control->feedback_c = feedback_c;

  /* 仅加热极性：低于设定温度时误差为正，只允许输出 0～100%。 */
  error = control->config.setpoint_c - feedback_c;

  TemperatureControl_SetDuty(control,
      PID_Update(&control->pid, error, dt_seconds));
}
