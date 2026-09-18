/**
 * @file board_io.c
 * @brief 板载低有效按键和低有效 LED 的实现。
 */
#include "board_io.h"

#include "main.h"
#include "stm32g0xx_hal.h"

/** SW2 原始电平连续保持此时间后，才更新对外发布的稳定状态。 */
#define BOARD_BUTTON_DEBOUNCE_MS 20U

/** 加热时 LED1 的翻转间隔，完整亮灭周期为 200 ms。 */
#define BOARD_HEATING_LED_FAST_INTERVAL_MS 100U

/** 未加热时 LED1 的翻转间隔，完整亮灭周期为 1000 ms。 */
#define BOARD_HEATING_LED_SLOW_INTERVAL_MS 500U

/** 软件消抖状态；事件采用锁存方式，读取事件后才清零。 */
typedef struct
{
  uint8_t stable_pressed;    /**< 已通过消抖、对外可见的按键状态。 */
  uint8_t candidate_pressed; /**< 当前正在观察的候选原始状态。 */
  uint8_t pressed_event;     /**< 稳定状态切换到按下后锁存为 1。 */
  uint8_t released_event;    /**< 稳定状态切换到释放后锁存为 1。 */
  uint32_t candidate_tick;   /**< 候选状态最后一次变化的毫秒时间戳。 */
} BoardButtonState;

/** 单按键驱动的全局软件状态，仅由本模块访问。 */
static BoardButtonState button_state;

/** LED1 加热状态指示的非阻塞节拍状态。 */
typedef struct
{
  uint8_t heating;          /**< 上一次调用时的加热状态。 */
  uint8_t led_on;           /**< LED1 当前逻辑状态：1 亮，0 灭。 */
  uint32_t last_toggle_tick; /**< 上一次翻转 LED1 的毫秒时间戳。 */
} BoardHeatingIndicatorState;

/** LED1 加热状态指示实例，仅由本模块访问。 */
static BoardHeatingIndicatorState heating_indicator_state;

/**
 * @brief 读取 SW2 的瞬时物理电平并转换为逻辑状态。
 * @return PB3 为低时返回 1（按下），为高时返回 0（释放）。
 * @note 返回值尚未消抖，仅供 BoardButton_Update() 内部使用。
 */
static uint8_t BoardButton_ReadRaw(void)
{
  return (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET) ?
         1U : 0U;
}

/**
 * @brief 把逻辑 LED 编号映射为 STM32 GPIO 端口和引脚。
 * @param led LED 枚举。
 * @param port 返回 GPIO 端口。
 * @param pin 返回 GPIO 引脚掩码。
 * @return 映射成功返回 1，参数或 LED 编号非法返回 0。
 */
static uint8_t BoardLED_GetPin(BoardLED led, GPIO_TypeDef **port,
                               uint16_t *pin)
{
  if ((port == 0) || (pin == 0))
  {
    return 0U;
  }

  switch (led)
  {
    case BOARD_LED_1:
      *port = LED1_GPIO_Port;
      *pin = LED1_Pin;
      return 1U;

    case BOARD_LED_3:
      *port = LED3_GPIO_Port;
      *pin = LED3_Pin;
      return 1U;

    default:
      return 0U;
  }
}

void BoardIO_Init(void)
{
  uint8_t pressed;

  /* 初始化明确关闭两颗低有效 LED，避免继承复位前或调试器留下的状态。 */
  BoardLED_Set(BOARD_LED_1, 0U);
  BoardLED_Set(BOARD_LED_3, 0U);

  /* 用当前真实电平同时初始化稳定态和候选态，上电时不虚构按键事件。 */
  pressed = BoardButton_ReadRaw();
  button_state.stable_pressed = pressed;
  button_state.candidate_pressed = pressed;
  button_state.pressed_event = 0U;
  button_state.released_event = 0U;
  button_state.candidate_tick = HAL_GetTick();

  /* 上电从“未加热、LED 熄灭”开始，500 ms 后进入慢闪节拍。 */
  heating_indicator_state.heating = 0U;
  heating_indicator_state.led_on = 0U;
  heating_indicator_state.last_toggle_tick = HAL_GetTick();
}

void BoardLED_Set(BoardLED led, uint8_t on)
{
  GPIO_TypeDef *port;
  uint16_t pin;

  if (BoardLED_GetPin(led, &port, &pin) == 0U)
  {
    return;
  }

  /* LED 阴极由 MCU 驱动：输出低电平点亮，输出高电平熄灭。 */
  HAL_GPIO_WritePin(port, pin,
                    (on != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void BoardLED_Toggle(BoardLED led)
{
  GPIO_TypeDef *port;
  uint16_t pin;

  if (BoardLED_GetPin(led, &port, &pin) != 0U)
  {
    /* 低有效只影响“设置”语义，直接翻转物理电平仍等价于亮灭翻转。 */
    HAL_GPIO_TogglePin(port, pin);
  }
}

void BoardHeatingIndicator_Update(uint8_t heating)
{
  uint8_t current_heating = (heating != 0U) ? 1U : 0U;
  uint32_t interval_ms;
  uint32_t now = HAL_GetTick();

  if (current_heating != heating_indicator_state.heating)
  {
    /* 状态改变时立即点亮一次，使快闪/慢闪模式的切换能够直接看见。 */
    heating_indicator_state.heating = current_heating;
    heating_indicator_state.led_on = 1U;
    heating_indicator_state.last_toggle_tick = now;
    BoardLED_Set(BOARD_LED_1, 1U);
    return;
  }

  interval_ms = (current_heating != 0U) ?
                BOARD_HEATING_LED_FAST_INTERVAL_MS :
                BOARD_HEATING_LED_SLOW_INTERVAL_MS;

  /* 无符号时间差允许 HAL 毫秒计数器回绕，不需要阻塞延时。 */
  if ((now - heating_indicator_state.last_toggle_tick) >= interval_ms)
  {
    heating_indicator_state.last_toggle_tick = now;
    heating_indicator_state.led_on ^= 1U;
    BoardLED_Set(BOARD_LED_1, heating_indicator_state.led_on);
  }
}

void BoardButton_Update(void)
{
  uint8_t pressed = BoardButton_ReadRaw();
  uint32_t now = HAL_GetTick();

  /* 原始状态变化后重新计时，连续稳定 20 ms 才承认新的按键状态。 */
  if (pressed != button_state.candidate_pressed)
  {
    button_state.candidate_pressed = pressed;
    button_state.candidate_tick = now;
  }
  else if ((pressed != button_state.stable_pressed) &&
           ((now - button_state.candidate_tick) >= BOARD_BUTTON_DEBOUNCE_MS))
  {
    /* 候选电平稳定达到门限，发布新状态并锁存对应边沿事件。 */
    button_state.stable_pressed = pressed;
    if (pressed != 0U)
    {
      button_state.pressed_event = 1U;
    }
    else
    {
      button_state.released_event = 1U;
    }
  }
}

uint8_t BoardButton_IsPressed(void)
{
  return button_state.stable_pressed;
}

uint8_t BoardButton_GetPressedEvent(void)
{
  /* 先保存后清零，使每个锁存事件最多被应用层消费一次。 */
  uint8_t event = button_state.pressed_event;
  button_state.pressed_event = 0U;
  return event;
}

uint8_t BoardButton_GetReleasedEvent(void)
{
  /* 先保存后清零，使每个锁存事件最多被应用层消费一次。 */
  uint8_t event = button_state.released_event;
  button_state.released_event = 0U;
  return event;
}
