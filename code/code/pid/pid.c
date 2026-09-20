/**
 * @file pid.c
 * @brief 带积分限幅和输出限幅的离散位置式 PID 实现。
 */
#include "pid.h"

#include <stddef.h>

/** @brief 将数值约束在闭区间 [minimum, maximum] 内。 */
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
  /* 配置按值复制，调用方后续修改原 config 不会影响当前实例。 */
  controller->config = *config;
  PID_Reset(controller);
  /*
   * 预置值只在初始化时装载，用作当前热系统的维持功率先验值。
   * 后续安全停止仍通过 PID_Reset 清零积分，避免故障恢复前残留输出状态。
   */
  controller->integral = PID_Clamp(config->integral_initial,
                                   config->integral_min,
                                   config->integral_max);
}

void PID_Reset(PID_Controller *controller)
{
  if (controller == NULL)
  {
    return;
  }
  controller->proportional = 0.0f;
  controller->integral = 0.0f;
  controller->derivative = 0.0f;
  controller->output = 0.0f;
  controller->previous_error = 0.0f;
  controller->filtered_derivative = 0.0f;
  controller->initialized = 0U;
}

float PID_UpdateConditional(PID_Controller *controller,
                            float error,
                            float dt_seconds,
                            uint8_t integral_enabled)
{
  float derivative = 0.0f;
  float filter_alpha;
  float candidate_integral;
  float candidate_output;
  float output;

  if ((controller == NULL) || (dt_seconds <= 0.0f))
  {
    return 0.0f;
  }

  /* 首次更新没有历史误差，微分项取 0，避免启动冲击。 */
  if (controller->initialized != 0U)
  {
    derivative = (error - controller->previous_error) / dt_seconds;
    if (controller->config.derivative_filter_time_constant_s > 0.0f)
    {
      /* 一阶低通抑制 ADC 量化噪声被 D 项放大。 */
      filter_alpha = dt_seconds /
          (controller->config.derivative_filter_time_constant_s +
           dt_seconds);
      controller->filtered_derivative +=
          filter_alpha * (derivative - controller->filtered_derivative);
      derivative = controller->filtered_derivative;
    }
    else
    {
      controller->filtered_derivative = derivative;
    }
  }
  else
  {
    controller->initialized = 1U;
    controller->filtered_derivative = 0.0f;
  }

  /* 先计算 P、D，积分候选值在确认不会加重输出饱和后再写回。 */
  controller->proportional = controller->config.kp * error;
  controller->derivative = controller->config.kd * derivative;
  candidate_integral = PID_Clamp(
      controller->integral + controller->config.ki * error * dt_seconds,
      controller->config.integral_min,
      controller->config.integral_max);
  candidate_output = controller->proportional + candidate_integral +
                     controller->derivative;

  /*
   * 条件积分抗饱和：输出已超过上限且误差仍为正时，拒绝继续增加积分；
   * 输出已低于下限且误差仍为负时同理。反向误差仍允许积分回退，
   * 因而温度接近或越过设定点后可以及时降低持续加热功率。
   */
  if ((integral_enabled != 0U) &&
      !(((candidate_output > controller->config.output_max) &&
         (error > 0.0f)) ||
        ((candidate_output < controller->config.output_min) &&
         (error < 0.0f))))
  {
    controller->integral = candidate_integral;
  }

  /* 位置式 PID：u(k)=Kp*e(k)+积分项+Kd*(e(k)-e(k-1))/dt。 */
  output = controller->proportional + controller->integral +
           controller->derivative;
  output = PID_Clamp(output, controller->config.output_min,
                     controller->config.output_max);
  controller->output = output;
  controller->previous_error = error;
  return output;
}

float PID_Update(PID_Controller *controller, float error, float dt_seconds)
{
  /* 通用接口默认始终允许积分，保持原有调用行为不变。 */
  return PID_UpdateConditional(controller, error, dt_seconds, 1U);
}
