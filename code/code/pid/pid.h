/**
 * @file pid.h
 * @brief 通用离散 PID 控制器接口。
 */
#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  /** 比例、积分、微分系数，以及输出和积分项的独立限幅。 */
  float kp;
  float ki;
  float kd;
  float output_min;
  float output_max;
  float integral_min;
  float integral_max;
} PID_Config;

typedef struct
{
  /** 控制器运行状态；previous_error 用于计算误差微分。 */
  PID_Config config;
  float integral;
  float previous_error;
  uint8_t initialized;
} PID_Controller;

/** @brief 复制参数并清零控制器历史状态。 */
void PID_Init(PID_Controller *controller, const PID_Config *config);
/** @brief 清除积分和微分历史，不改变配置参数。 */
void PID_Reset(PID_Controller *controller);
/** @brief 按给定误差和真实时间间隔更新 PID，返回限幅后的输出。 */
float PID_Update(PID_Controller *controller, float error, float dt_seconds);

#ifdef __cplusplus
}
#endif

#endif
