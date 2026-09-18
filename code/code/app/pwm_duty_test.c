/**
 * @file pwm_duty_test.c
 * @brief PD1/EN 高有效 PWM 阶梯占空比测试实现。
 *
 * TIM17 的周期为 1000 个计数，PWM1 模式且输出极性为高，因此 CCR 从
 * 200 递增到 1000 时，对应 20% 到 100% 的高电平占空比。
 */
#include "pwm_duty_test.h"

#include <stddef.h>
#include <string.h>

/** 每个占空比保持 10 秒，给万用表留出稳定显示时间。 */
#define PWM_DUTY_TEST_STEP_PERIOD_MS 10000U

/** 测试顺序；到达 100% 后重新从 20% 开始循环。 */
static const uint8_t duty_steps[] = {20U, 40U, 60U, 80U, 100U};

/**
 * @brief 把百分比换算为 CCR 计数值并更新高有效 PWM。
 * @param test 测试实例。
 * @param duty_percent 目标占空比，范围为 0～100。
 */
static void PWM_DutyTest_SetDuty(PWM_DutyTest *test, uint8_t duty_percent)
{
  uint32_t period_counts;
  uint32_t compare;

  if (duty_percent > 100U)
  {
    duty_percent = 100U;
  }

  /* ARR=999 时一个周期有 1000 个计数；CCR=1000 可得到持续高电平。 */
  period_counts = __HAL_TIM_GET_AUTORELOAD(test->timer) + 1U;
  compare = ((uint32_t)duty_percent * period_counts + 50U) / 100U;
  __HAL_TIM_SET_COMPARE(test->timer, test->channel, compare);
  test->duty_percent = duty_percent;
}

HAL_StatusTypeDef PWM_DutyTest_Init(PWM_DutyTest *test,
                                    TIM_HandleTypeDef *timer,
                                    uint32_t channel)
{
  HAL_StatusTypeDef status;

  if ((test == NULL) || (timer == NULL) || (timer->Instance == NULL) ||
      (channel != TIM_CHANNEL_1))
  {
    return HAL_ERROR;
  }

  memset(test, 0, sizeof(*test));
  test->timer = timer;
  test->channel = channel;

  /* PD1/EN 为高电平有效，清除 CC1P 可确保 PWM 有效脉冲保持高电平。 */
  CLEAR_BIT(timer->Instance->CCER, TIM_CCER_CC1P);

  /* 在定时器开始计数前写入第一档，切换 PD1 到复用功能后直接输出 20%。 */
  PWM_DutyTest_SetDuty(test, duty_steps[0]);
  status = HAL_TIM_PWM_Start(timer, channel);
  if (status != HAL_OK)
  {
    PWM_DutyTest_SetDuty(test, 0U);
    return status;
  }

  test->next_step_tick = HAL_GetTick() + PWM_DUTY_TEST_STEP_PERIOD_MS;
  return HAL_OK;
}

void PWM_DutyTest_Task(PWM_DutyTest *test)
{
  uint32_t now;

  if ((test == NULL) || (test->timer == NULL))
  {
    return;
  }

  now = HAL_GetTick();
  /* 有符号差值比较可在 HAL 毫秒计数器回绕时继续正常工作。 */
  if ((int32_t)(now - test->next_step_tick) < 0)
  {
    return;
  }

  test->step_index++;
  if (test->step_index >= (uint8_t)(sizeof(duty_steps) / sizeof(duty_steps[0])))
  {
    test->step_index = 0U;
  }
  PWM_DutyTest_SetDuty(test, duty_steps[test->step_index]);

  /* 从原计划时刻继续累计，减少主循环阻塞造成的长期时间漂移。 */
  test->next_step_tick += PWM_DUTY_TEST_STEP_PERIOD_MS;
  if ((int32_t)(now - test->next_step_tick) >= 0)
  {
    test->next_step_tick = now + PWM_DUTY_TEST_STEP_PERIOD_MS;
  }
}
