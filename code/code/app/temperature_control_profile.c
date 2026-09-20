/**
 * @file temperature_control_profile.c
 * @brief 9~24 V 输入范围内的温控前馈和动态参数计算。
 *
 * 依据 2026-09-18 在 680 ohm 限流电阻、42 ℃目标下的实测日志：
 * 9 V、12 V、15 V、18 V 稳态尾段平均积分项分别约为 10.78%、8.36%、
 * 7.43%、7.27%。2026-09-20 的 18 V 冷启动曲线还显示：30% 输出使温度
 * 首次到达 42 ℃后继续上升至 45.59 ℃。电压表因此只保留可解释的执行器参数：
 * 稳态前馈初值和安全输出上限。闭环动态由统一的参考轨迹和速率反馈处理，
 * 不再按电压或绝对温度拟合多组 Kp/Kd。
 */
#include "temperature_control_profile.h"

#include <stddef.h>

/** 日志拟合的电压/维持占空比锚点。 */
typedef struct
{
  float voltage_v;             /**< 输入电压，单位 V。 */
  float integral_initial_pct;  /**< 对应的积分初值，单位 %。 */
  float output_max_pct;        /**< 对应的最大 PWM 占空比，单位 %。 */
} TemperatureVoltagePoint;

static const TemperatureVoltagePoint voltage_points[] =
{
  /* VIN, 积分前馈, 安全输出上限 */
  { 9.0f,  10.78f, 30.0f },
  { 12.0f,  8.36f, 24.0f },
  { 15.0f,  7.43f, 24.0f },
  { 18.0f,  7.27f, 24.0f },
  { 24.0f,  7.10f, 18.0f }
};

/**
 * @brief 在两个浮点锚点之间进行线性插值。
 * @param x 当前输入值。
 * @param x0 左侧输入锚点。
 * @param x1 右侧输入锚点。
 * @param y0 左侧输出锚点。
 * @param y1 右侧输出锚点。
 * @return 插值后的输出值。
 */
static float TemperatureControl_Interpolate(float x,
                                             float x0,
                                             float x1,
                                             float y0,
                                             float y1)
{
  return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

/**
 * @brief 在日志锚点之间线性插值整组电压参数。
 * @param voltage_v 输入电压，调用方已保证在 9~24 V 内。
 * @param lower 返回左侧锚点。
 * @param upper 返回右侧锚点；位于端点时与 lower 相同。
 */
static void TemperatureControl_FindInterval(
    float voltage_v,
    const TemperatureVoltagePoint **lower,
    const TemperatureVoltagePoint **upper)
{
  uint32_t index;
  const uint32_t point_count =
      (uint32_t)(sizeof(voltage_points) / sizeof(voltage_points[0]));

  if (voltage_v <= voltage_points[0].voltage_v)
  {
    *lower = &voltage_points[0];
    *upper = &voltage_points[0];
    return;
  }
  if (voltage_v >= voltage_points[point_count - 1U].voltage_v)
  {
    *lower = &voltage_points[point_count - 1U];
    *upper = &voltage_points[point_count - 1U];
    return;
  }

  for (index = 1U; index < point_count; index++)
  {
    if (voltage_v <= voltage_points[index].voltage_v)
    {
      *lower = &voltage_points[index - 1U];
      *upper = &voltage_points[index];
      return;
    }
  }

  *lower = &voltage_points[point_count - 1U];
  *upper = &voltage_points[point_count - 1U];
}

void TemperatureControl_ApplyVoltageProfile(
    TemperatureControl_Config *config,
    float input_voltage_v)
{
  float clamped_voltage_v;
  const TemperatureVoltagePoint *lower;
  const TemperatureVoltagePoint *upper;

  if (config == NULL)
  {
    return;
  }

  /* 日志只覆盖 9~18 V，24 V 是设计上限；边界外采用最近端点值。 */
  clamped_voltage_v = input_voltage_v;
  if (clamped_voltage_v < 9.0f)
  {
    clamped_voltage_v = 9.0f;
  }
  else if (clamped_voltage_v > 24.0f)
  {
    clamped_voltage_v = 24.0f;
  }

  TemperatureControl_FindInterval(clamped_voltage_v, &lower, &upper);
  if (lower == upper)
  {
    config->pid.integral_initial = lower->integral_initial_pct;
    config->pid.output_max = lower->output_max_pct;
    return;
  }

  config->pid.integral_initial = TemperatureControl_Interpolate(
      clamped_voltage_v, lower->voltage_v, upper->voltage_v,
      lower->integral_initial_pct, upper->integral_initial_pct);
  config->pid.output_max = TemperatureControl_Interpolate(
      clamped_voltage_v, lower->voltage_v, upper->voltage_v,
      lower->output_max_pct, upper->output_max_pct);
}
