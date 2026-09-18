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

/** @brief 从四路温度中选择 PID 反馈量的策略。 */
typedef enum
{
  TEMPERATURE_FEEDBACK_AVERAGE = 0, /**< 四路算术平均值，默认策略。 */
  TEMPERATURE_FEEDBACK_MAXIMUM,     /**< 四路最大值，适合限制最热点。 */
  TEMPERATURE_FEEDBACK_MINIMUM,     /**< 四路最小值，适合优先加热最冷点。 */
  TEMPERATURE_FEEDBACK_CHANNEL_1,   /**< 仅使用第 1 路温度。 */
  TEMPERATURE_FEEDBACK_CHANNEL_2,   /**< 仅使用第 2 路温度。 */
  TEMPERATURE_FEEDBACK_CHANNEL_3,   /**< 仅使用第 3 路温度。 */
  TEMPERATURE_FEEDBACK_CHANNEL_4    /**< 仅使用第 4 路温度。 */
} TemperatureControl_FeedbackMode;

/** @brief 加热控制器的用户配置。 */
typedef struct
{
  float setpoint_c; /**< 目标温度，单位 ℃。 */
  PID_Config pid;   /**< PID 系数、输出限幅和积分限幅。 */
  TemperatureControl_FeedbackMode feedback_mode; /**< 四路反馈合成策略。 */
  uint8_t enabled;  /**< 非 0 允许加热，0 强制输出 0% 并清积分。 */
} TemperatureControl_Config;

/** @brief 温控实例，保存硬件绑定、PID 状态和最近一次控制结果。 */
typedef struct
{
  TIM_HandleTypeDef *pwm_timer; /**< 高有效 PWM 使用的 HAL 定时器句柄。 */
  uint32_t pwm_channel; /**< PWM 通道；当前硬件固定为 TIM_CHANNEL_1。 */
  PT1000_App *temperature_app; /**< 四路温度快照的数据源。 */
  TemperatureControl_Config config; /**< 初始化时复制的控制配置。 */
  PID_Controller pid; /**< PID 积分、上次误差等内部状态。 */
  uint32_t last_sequence; /**< 已处理的采样轮次，防止重复计算。 */
  uint32_t last_update_tick; /**< 上次控制更新时间戳，用于计算 dt。 */
  float feedback_c; /**< 最近一次有效的合成反馈温度，单位 ℃。 */
  float duty_percent; /**< 当前高电平有效占空比，范围 0～100%。 */
} TemperatureControl;

/**
 * @brief 载入安全的默认温控配置。
 * @param config 待填写的配置结构体。
 * @note 默认设定 25 ℃、四路平均反馈、使能控制；PID 系数为 0，需实机整定。
 */
void TemperatureControl_GetDefaultConfig(TemperatureControl_Config *config);
/**
 * @brief 初始化仅加热控制器并启动 0% PWM。
 * @param control 温控实例。
 * @param pwm_timer TIM17 HAL 句柄。
 * @param pwm_channel 必须为 TIM_CHANNEL_1，对应 PD1/TIM17_CH1。
 * @param temperature_app 已初始化的四路温度应用。
 * @param config 用户配置，函数内部按值复制。
 * @return HAL_OK 表示 PWM 已启动；参数非法或定时器启动失败返回 HAL_ERROR。
 */
HAL_StatusTypeDef TemperatureControl_Init(
    TemperatureControl *control,
    TIM_HandleTypeDef *pwm_timer,
    uint32_t pwm_channel,
    PT1000_App *temperature_app,
    const TemperatureControl_Config *config);
/**
 * @brief 在出现一轮新温度时更新反馈、PID 和 PWM 占空比。
 * @param control 已初始化的温控实例。
 * @note 无新快照时立即返回；任一路无效时执行安全停止。
 */
void TemperatureControl_Task(TemperatureControl *control);
/**
 * @brief 立即把加热 PWM 归零并清除积分与微分历史。
 * @param control 温控实例；传入空指针时不执行操作。
 */
void TemperatureControl_Stop(TemperatureControl *control);

#ifdef __cplusplus
}
#endif

#endif
