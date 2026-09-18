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
  controller->initialized = 0U;
}

float PID_Update(PID_Controller *controller, float error, float dt_seconds)
{
  float derivative = 0.0f;
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
  }
  else
  {
    controller->initialized = 1U;
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
  if (!(((candidate_output > controller->config.output_max) &&
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
