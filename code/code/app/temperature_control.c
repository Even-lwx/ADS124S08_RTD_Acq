/**
 * @file temperature_control.c
 * @brief 四路反馈选择、仅加热 PID 与高有效 PWM 输出实现。
 *
 * 安全策略：有效温度少于两路、控制禁用或采样时间异常时，
 * PWM 归零并清积分。
 */
#include "temperature_control.h"

#include <stddef.h>
#include <string.h>

/** PID 整定帧使用阻塞发送时的最大等待时间。 */
#define TEMPERATURE_CONTROL_UART_TIMEOUT_MS 100U
/** 进入温控计算所需的最少有效传感器数量。 */
#define TEMPERATURE_CONTROL_MIN_VALID_SENSORS 2U
/** 首次尚无历史时间戳时采用的标称 5 Hz 控制周期。 */
#define TEMPERATURE_CONTROL_INITIAL_DT_SECONDS 0.2f

/**
 * @brief 把无符号整数以十进制追加到 FireWater 帧。
 * @note 手工格式化可避免引入 printf 浮点支持。
 */
static char *TemperatureControl_AppendUnsigned(char *destination,
                                               uint32_t value)
{
  char reverse[10];
  uint8_t count = 0U;

  do
  {
    reverse[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  } while (value != 0U);

  while (count > 0U)
  {
    *destination++ = reverse[--count];
  }
  return destination;
}

/**
 * @brief 把浮点数四舍五入为两位小数后追加到 FireWater 帧。
 * @note PID 参数、温度和占空比范围较小，转换后不会超出 int32_t。
 */
static char *TemperatureControl_AppendFloat2(char *destination, float value)
{
  int32_t scaled;
  uint32_t magnitude;

  scaled = (int32_t)(value * 100.0f + ((value >= 0.0f) ? 0.5f : -0.5f));
  if (scaled < 0)
  {
    *destination++ = '-';
    magnitude = (uint32_t)(-(scaled + 1)) + 1U;
  }
  else
  {
    magnitude = (uint32_t)scaled;
  }

  destination = TemperatureControl_AppendUnsigned(destination,
                                                   magnitude / 100U);
  *destination++ = '.';
  *destination++ = (char)('0' + ((magnitude / 10U) % 10U));
  *destination++ = (char)('0' + (magnitude % 10U));
  return destination;
}

/**
 * @brief 把 PID 系数四舍五入为三位小数后追加到 FireWater 帧。
 * @note 仅 KP/KI/KD 使用三位小数，确保 Ki=0.025 不会显示成 0.03。
 */
static char *TemperatureControl_AppendFloat3(char *destination, float value)
{
  int32_t scaled;
  uint32_t magnitude;

  scaled = (int32_t)(value * 1000.0f + ((value >= 0.0f) ? 0.5f : -0.5f));
  if (scaled < 0)
  {
    *destination++ = '-';
    magnitude = (uint32_t)(-(scaled + 1)) + 1U;
  }
  else
  {
    magnitude = (uint32_t)scaled;
  }

  destination = TemperatureControl_AppendUnsigned(destination,
                                                   magnitude / 1000U);
  *destination++ = '.';
  *destination++ = (char)('0' + ((magnitude / 100U) % 10U));
  *destination++ = (char)('0' + ((magnitude / 10U) % 10U));
  *destination++ = (char)('0' + (magnitude % 10U));
  return destination;
}

/**
 * @brief 从 USART2 发送一帧纯数字 FireWater 温控数据。
 * @param control 温控实例。
 * @param snapshot 与本次 PID 计算对应的四路温度快照。
 * @param feedback_valid 非 0 表示本轮反馈满足控制条件。
 * @param valid_count 本轮有效温度传感器数量。
 * @note 固定列顺序为 T1,T2,T3,T4,SP,FB,ERR,KP,KI,KD,P,I,D,OUT,
 *       LIM,DT,VALID,STATE，仅以 '\n' 结尾。STATE=2 表示过温锁定。
 */
static void TemperatureControl_SendTelemetry(
    TemperatureControl *control,
    const PT1000_AppSnapshot *snapshot,
    uint8_t feedback_valid,
    uint8_t valid_count)
{
  char frame[192];
  char *cursor = frame;
  UART_HandleTypeDef *uart;
  uint8_t channel;
  uint8_t state;

  if ((control->config.telemetry_enabled == 0U) ||
      (control->temperature_app == NULL) ||
      (control->temperature_app->output_uart == NULL))
  {
    return;
  }
  uart = control->temperature_app->output_uart;

  /* 失效通道用 0.00 占位，VALID 和 STATE 可用于区分真实 0 ℃。 */
  for (channel = 0U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
  {
    cursor = TemperatureControl_AppendFloat2(
        cursor,
        (snapshot->valid[channel] != 0U) ?
            snapshot->temperature_c[channel] : 0.0f);
    *cursor++ = ',';
  }

  cursor = TemperatureControl_AppendFloat2(cursor, control->config.setpoint_c);
  *cursor++ = ',';
  if (feedback_valid != 0U)
  {
    cursor = TemperatureControl_AppendFloat2(cursor, control->feedback_c);
  }
  else
  {
    cursor = TemperatureControl_AppendFloat2(cursor, 0.0f);
  }
  *cursor++ = ',';
  if (feedback_valid != 0U)
  {
    cursor = TemperatureControl_AppendFloat2(cursor, control->error_c);
  }
  else
  {
    cursor = TemperatureControl_AppendFloat2(cursor, 0.0f);
  }
  *cursor++ = ',';
  cursor = TemperatureControl_AppendFloat3(cursor, control->pid.config.kp);
  *cursor++ = ',';
  cursor = TemperatureControl_AppendFloat3(cursor, control->pid.config.ki);
  *cursor++ = ',';
  cursor = TemperatureControl_AppendFloat3(cursor, control->pid.config.kd);
  *cursor++ = ',';
  cursor = TemperatureControl_AppendFloat2(cursor, control->pid.proportional);
  *cursor++ = ',';
  cursor = TemperatureControl_AppendFloat2(cursor, control->pid.integral);
  *cursor++ = ',';
  cursor = TemperatureControl_AppendFloat2(cursor, control->pid.derivative);
  *cursor++ = ',';
  cursor = TemperatureControl_AppendFloat2(cursor, control->duty_percent);
  *cursor++ = ',';
  cursor = TemperatureControl_AppendFloat2(cursor,
                                           control->pid.config.output_max);
  *cursor++ = ',';
  cursor = TemperatureControl_AppendFloat2(cursor, control->dt_seconds);
  *cursor++ = ',';
  cursor = TemperatureControl_AppendUnsigned(cursor, valid_count);
  *cursor++ = ',';
  if (control->overtemperature_latched != 0U)
  {
    state = 2U;
  }
  else
  {
    state = ((control->config.enabled != 0U) &&
             (feedback_valid != 0U)) ? 1U : 0U;
  }
  cursor = TemperatureControl_AppendUnsigned(cursor, state);
  *cursor++ = '\n';

  (void)HAL_UART_Transmit(uart, (uint8_t *)frame,
                          (uint16_t)(cursor - frame),
                          TEMPERATURE_CONTROL_UART_TIMEOUT_MS);
}

/**
 * @brief 对占空比进行 0～配置上限限幅，并转换为定时器 CCR 计数值。
 * @param control 温控实例，必须已绑定 PWM 定时器。
 * @param duty_percent 期望的高电平占空比百分数。
 */
static void TemperatureControl_SetDuty(TemperatureControl *control,
                                       float duty_percent)
{
  uint32_t period_counts;
  uint32_t compare;

  if (duty_percent < 0.0f)
  {
    duty_percent = 0.0f;
  }
  else if (duty_percent > control->config.pid.output_max)
  {
    /* 执行器层再做限幅，避免异常调用绕过 PID 的输出限幅。 */
    duty_percent = control->config.pid.output_max;
  }

  /* CCR/周期计数换算为百分比；实际限幅值由上层 PID 配置决定。 */
  period_counts = __HAL_TIM_GET_AUTORELOAD(control->pwm_timer) + 1U;
  compare = (uint32_t)(duty_percent * (float)period_counts / 100.0f + 0.5f);
  __HAL_TIM_SET_COMPARE(control->pwm_timer, control->pwm_channel, compare);
  control->duty_percent = duty_percent;
}

/**
 * @brief 检查四路有效性，并按配置合成为一个 PID 反馈温度。
 * @param control 温控实例，用于读取反馈选择策略。
 * @param snapshot 最新四路温度快照。
 * @param feedback_c 返回合成后的温度，单位 ℃。
 * @param valid_count 返回本轮有效传感器数量。
 * @return 至少两路有效且反馈策略可用时返回 1，否则返回 0。
 */
static uint8_t TemperatureControl_GetFeedback(
    const TemperatureControl *control,
    const PT1000_AppSnapshot *snapshot,
    float *feedback_c,
    uint8_t *valid_count)
{
  uint8_t channel;
  uint8_t first_valid = PT1000_APP_CHANNEL_COUNT;

  *valid_count = 0U;
  for (channel = 0U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
  {
    if (snapshot->valid[channel] != 0U)
    {
      if (first_valid == PT1000_APP_CHANNEL_COUNT)
      {
        first_valid = channel;
      }
      (*valid_count)++;
    }
  }
  /* 容许损失两路传感器，但少于两路时不再允许加热。 */
  if (*valid_count < TEMPERATURE_CONTROL_MIN_VALID_SENSORS)
  {
    return 0U;
  }

  switch (control->config.feedback_mode)
  {
    case TEMPERATURE_FEEDBACK_MAXIMUM:
      *feedback_c = snapshot->temperature_c[first_valid];
      for (channel = 0U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
      {
        if ((snapshot->valid[channel] != 0U) &&
            (snapshot->temperature_c[channel] > *feedback_c))
        {
          *feedback_c = snapshot->temperature_c[channel];
        }
      }
      break;

    case TEMPERATURE_FEEDBACK_MINIMUM:
      *feedback_c = snapshot->temperature_c[first_valid];
      for (channel = 0U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
      {
        if ((snapshot->valid[channel] != 0U) &&
            (snapshot->temperature_c[channel] < *feedback_c))
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
      /* 固定通道失效时不用其他通道悄然替代，避免反馈对象改变。 */
      if (snapshot->valid[channel] == 0U)
      {
        return 0U;
      }
      *feedback_c = snapshot->temperature_c[channel];
      break;

    case TEMPERATURE_FEEDBACK_AVERAGE:
    default:
      *feedback_c = 0.0f;
      for (channel = 0U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
      {
        if (snapshot->valid[channel] != 0U)
        {
          *feedback_c += snapshot->temperature_c[channel];
        }
      }
      *feedback_c /= (float)(*valid_count);
      break;
  }
  return 1U;
}

/**
 * @brief 检查任一有效传感器是否已达到独立过温阈值。
 * @param snapshot 当前四路温度快照。
 * @param threshold_c 过温锁定阈值，单位 ℃。
 * @return 任一有效通道过温返回 1，否则返回 0。
 */
static uint8_t TemperatureControl_IsOvertemperature(
    const PT1000_AppSnapshot *snapshot,
    float threshold_c)
{
  uint8_t channel;

  for (channel = 0U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
  {
    if ((snapshot->valid[channel] != 0U) &&
        (snapshot->temperature_c[channel] >= threshold_c))
    {
      return 1U;
    }
  }
  return 0U;
}

void TemperatureControl_GetDefaultConfig(TemperatureControl_Config *config)
{
  if (config == NULL)
  {
    return;
  }
  /* 先清零，确保以后扩展字段时仍有确定的默认状态。 */
  memset(config, 0, sizeof(*config));
  config->setpoint_c = 25.0f;
  config->pid.kp = 0.0f;
  config->pid.ki = 0.0f;
  config->pid.kd = 0.0f;
  config->pid.output_min = 0.0f;
  config->pid.output_max = 100.0f;
  config->pid.integral_min = 0.0f;
  config->pid.integral_max = 100.0f;
  config->pid.integral_initial = 0.0f;
  config->feedback_mode = TEMPERATURE_FEEDBACK_AVERAGE;
  config->overtemperature_c = 48.0f;
  config->enabled = 1U;
  config->telemetry_enabled = 0U;
}

HAL_StatusTypeDef TemperatureControl_Init(
    TemperatureControl *control,
    TIM_HandleTypeDef *pwm_timer,
    uint32_t pwm_channel,
    PT1000_App *temperature_app,
    const TemperatureControl_Config *config)
{
  HAL_StatusTypeDef status;

  /* 输出和积分范围必须自洽，且仅允许本板连接的通道 1。 */
  if ((control == NULL) || (pwm_timer == NULL) ||
      (temperature_app == NULL) || (config == NULL) ||
      (pwm_channel != TIM_CHANNEL_1) ||
      (config->pid.output_min < 0.0f) ||
      (config->pid.output_max > 100.0f) ||
      (config->pid.output_min > config->pid.output_max) ||
      (config->pid.integral_min > config->pid.integral_max) ||
      (config->pid.integral_initial < config->pid.integral_min) ||
      (config->pid.integral_initial > config->pid.integral_max) ||
      (config->overtemperature_c <= config->setpoint_c) ||
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

  /* 在使能定时器输出之前先写 CCR=0，保证启动瞬间不加热。 */
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
  control->error_c = 0.0f;
  control->dt_seconds = 0.0f;
}

void TemperatureControl_Task(TemperatureControl *control)
{
  PT1000_AppSnapshot snapshot;
  float feedback_c;
  float dt_seconds;
  uint8_t valid_count = 0U;
  uint8_t feedback_valid;

  if ((control == NULL) || (control->temperature_app == NULL))
  {
    return;
  }

  /* sequence 由采集任务每轮递增，用它确保一轮数据只计算一次 PID。 */
  PT1000_AppGetSnapshot(control->temperature_app, &snapshot);
  if ((snapshot.sequence == 0U) ||
      (snapshot.sequence == control->last_sequence))
  {
    return;
  }
  control->last_sequence = snapshot.sequence;

  /* 禁用或传感器异常时立即归零输出，并清除可能积累的积分项。 */
  feedback_valid = TemperatureControl_GetFeedback(control, &snapshot,
                                                   &feedback_c,
                                                   &valid_count);
  if (feedback_valid != 0U)
  {
    /*
     * 先保存本轮反馈与误差，再判断保护状态。这样触发过温锁定的那一帧
     * 仍能显示导致停机的最高温度，而不是把诊断所需数据清成 0。
     */
    control->feedback_c = feedback_c;
    control->error_c = control->config.setpoint_c - feedback_c;
  }
  else
  {
    control->feedback_c = 0.0f;
    control->error_c = 0.0f;
  }
  /*
   * 过温保护不依赖 PID 反馈策略：即使未被选为反馈的通道
   * 超温，也立即锁定停机，避免平均值或单通道模式掩盖局部过热。
   */
  if (TemperatureControl_IsOvertemperature(
          &snapshot, control->config.overtemperature_c) != 0U)
  {
    control->overtemperature_latched = 1U;
  }
  if ((control->config.enabled == 0U) || (feedback_valid == 0U) ||
      (control->overtemperature_latched != 0U))
  {
    TemperatureControl_Stop(control);
    if (feedback_valid != 0U)
    {
      /* Stop 会清除控制状态；恢复只读诊断量供过温/禁用帧显示。 */
      control->feedback_c = feedback_c;
      control->error_c = control->config.setpoint_c - feedback_c;
    }
    control->last_update_tick = snapshot.tick_ms;
    TemperatureControl_SendTelemetry(control, &snapshot, feedback_valid,
                                     valid_count);
    return;
  }

  /* 首次更新按标称 0.2 秒计算；之后采用真实采样间隔以降低周期抖动影响。 */
  if (control->last_update_tick == 0U)
  {
    dt_seconds = TEMPERATURE_CONTROL_INITIAL_DT_SECONDS;
  }
  else
  {
    dt_seconds = (float)(snapshot.tick_ms - control->last_update_tick) / 1000.0f;
    /* 时间戳异常或停顿过久时重置 PID，避免积分和微分突然跳变。 */
    if ((dt_seconds <= 0.0f) || (dt_seconds > 10.0f))
    {
      dt_seconds = TEMPERATURE_CONTROL_INITIAL_DT_SECONDS;
      PID_Reset(&control->pid);
    }
  }
  control->last_update_tick = snapshot.tick_ms;
  control->dt_seconds = dt_seconds;

  /* 仅加热极性：低于设定温度时误差为正，负输出固定截止为 0%。 */
  TemperatureControl_SetDuty(control,
      PID_Update(&control->pid, control->error_c, dt_seconds));
  TemperatureControl_SendTelemetry(control, &snapshot, 1U, valid_count);
}
