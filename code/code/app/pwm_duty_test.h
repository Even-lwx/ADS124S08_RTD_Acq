/**
 * @file pwm_duty_test.h
 * @brief PD1/EN 高有效 PWM 阶梯占空比测试接口。
 */
#ifndef PWM_DUTY_TEST_H
#define PWM_DUTY_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"

/** @brief PWM 阶梯测试运行状态。 */
typedef struct
{
  TIM_HandleTypeDef *timer; /**< 输出 PWM 的 HAL 定时器句柄。 */
  uint32_t channel;         /**< 输出 PWM 的定时器通道。 */
  uint32_t next_step_tick;  /**< 下一次切换占空比的绝对毫秒时刻。 */
  uint8_t step_index;       /**< 当前占空比在测试表中的索引。 */
  uint8_t duty_percent;     /**< 当前高电平占空比，单位百分比。 */
} PWM_DutyTest;

/**
 * @brief 启动 PWM 阶梯测试，并立即输出 20% 高电平占空比。
 * @param test 测试实例。
 * @param timer TIM17 HAL 句柄。
 * @param channel 必须为 TIM_CHANNEL_1，对应 PD1/TIM17_CH1。
 * @return PWM 启动成功返回 HAL_OK，参数错误或启动失败返回 HAL_ERROR。
 */
HAL_StatusTypeDef PWM_DutyTest_Init(PWM_DutyTest *test,
                                    TIM_HandleTypeDef *timer,
                                    uint32_t channel);

/**
 * @brief 按 10 秒间隔循环输出 20%、40%、60%、80% 和 90% 占空比。
 * @param test 已成功初始化的测试实例。
 * @note 本函数不阻塞，需在主循环中持续调用。
 */
void PWM_DutyTest_Task(PWM_DutyTest *test);

#ifdef __cplusplus
}
#endif

#endif
