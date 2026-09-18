/**
 * @file pid.h
 * @brief 带条件积分抗饱和的通用离散 PID 控制器接口。
 */
#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief PID 参数和限幅配置。 */
typedef struct
{
  float kp; /**< 比例系数，直接乘以当前误差。 */
  float ki; /**< 积分系数，按 ki*error*dt 累加。 */
  float kd; /**< 微分系数，乘以误差变化率。 */
  float output_min; /**< 控制器总输出下限；加热应用必须不小于 0。 */
  float output_max; /**< 控制器总输出上限；占空比应用必须不大于 100。 */
  float integral_min; /**< 积分项下限，用于抑制积分饱和。 */
  float integral_max; /**< 积分项上限，用于抑制积分饱和。 */
  float integral_initial; /**< 初始化时装载的积分预置值。 */
} PID_Config;

/** @brief PID 运行实例，包含固定配置和跨周期历史状态。 */
typedef struct
{
  PID_Config config; /**< 初始化时复制的 PID 配置。 */
  float proportional; /**< 最近一次计算的比例项 Kp*error。 */
  float integral; /**< 已包含 ki 的积分项，可直接加入控制器输出。 */
  float derivative; /**< 最近一次计算的微分项 Kd*d(error)/dt。 */
  float output; /**< 最近一次经过限幅的 PID 总输出。 */
  float previous_error; /**< 上一次误差，用于计算离散微分。 */
  uint8_t initialized; /**< 0=尚无历史误差，首次更新不计算微分。 */
} PID_Controller;

/**
 * @brief 复制 PID 参数、清零历史状态并装载积分预置值。
 * @param controller 待初始化的控制器实例。
 * @param config PID 系数和限幅；函数内部按值复制。
 */
void PID_Init(PID_Controller *controller, const PID_Config *config);
/**
 * @brief 清除 P/I/D 分量、输出、上次误差和初始化标志。
 * @param controller 控制器实例；空指针将被安全忽略。
 * @note 安全停止调用本函数时积分清零，不会重新装载积分预置值。
 */
void PID_Reset(PID_Controller *controller);
/**
 * @brief 按位置式离散 PID 公式计算一次新输出。
 * @param controller 已初始化的控制器实例。
 * @param error 当前误差；本项目传入“设定温度-反馈温度”。
 * @param dt_seconds 与上次更新之间的时间，单位 s，必须大于 0。
 * @note 输出饱和且当前误差会加重饱和时暂停积分，反向误差仍可释放积分。
 * @return 经输出限幅后的控制量；参数非法时返回 0。
 */
float PID_Update(PID_Controller *controller, float error, float dt_seconds);

#ifdef __cplusplus
}
#endif

#endif
