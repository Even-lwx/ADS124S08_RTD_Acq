/**
 * @file pid.c
 * @brief 带积分限幅和输出限幅的离散位置式 PID 实现。
 */
#include "pid.h"

#include <stddef.h>

static float PID_Clamp(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

void PID_Init(PID_Controller *controller, const PID_Config *config)
{
  if ((controller == NULL) || (config == NULL))
  {
    return;
  }
  controller->config = *config;
  PID_Reset(controller);
}

void PID_Reset(PID_Controller *controller)
{
  if (controller == NULL)
  {
    return;
  }
  controller->integral = 0.0f;
  controller->previous_error = 0.0f;
  controller->initialized = 0U;
}

float PID_Update(PID_Controller *controller, float error, float dt_seconds)
{
  float derivative = 0.0f;
  float output;

  if ((controller == NULL) || (dt_seconds <= 0.0f))
  {
    return 0.0f;
  }

  /* 首次更新没有历史误差，微分项取 0，避免启动冲击。 */
  if (controller->initialized != 0U)
  {
    derivative = (error - controller->previous_error) / dt_seconds;
  }
  else
  {
    controller->initialized = 1U;
  }

  /* 积分项独立限幅，用于抑制长时间满功率时的积分饱和。 */
  controller->integral += controller->config.ki * error * dt_seconds;
  controller->integral = PID_Clamp(controller->integral,
                                   controller->config.integral_min,
                                   controller->config.integral_max);

  output = controller->config.kp * error + controller->integral +
           controller->config.kd * derivative;
  output = PID_Clamp(output, controller->config.output_min,
                     controller->config.output_max);
  controller->previous_error = error;
  return output;
}
