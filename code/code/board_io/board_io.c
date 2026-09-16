/**
 * @file board_io.c
 * @brief 板载低有效按键和低有效 LED 的实现。
 */
#include "board_io.h"

#include "main.h"
#include "stm32g0xx_hal.h"

#define BOARD_BUTTON_DEBOUNCE_MS 20U

/** 软件消抖状态；事件采用锁存方式，读取事件后才清零。 */
typedef struct
{
  uint8_t stable_pressed;
  uint8_t candidate_pressed;
  uint8_t pressed_event;
  uint8_t released_event;
  uint32_t candidate_tick;
} BoardButtonState;

static BoardButtonState button_state;

/** 按键硬件为低有效，将 GPIO 电平统一转换为逻辑按下状态。 */
static uint8_t BoardButton_ReadRaw(void)
{
  return (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET) ?
         1U : 0U;
}

/** 根据 LED 编号返回端口和引脚，返回 0 表示编号非法。 */
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

  BoardLED_Set(BOARD_LED_1, 0U);
  BoardLED_Set(BOARD_LED_3, 0U);

  pressed = BoardButton_ReadRaw();
  button_state.stable_pressed = pressed;
  button_state.candidate_pressed = pressed;
  button_state.pressed_event = 0U;
  button_state.released_event = 0U;
  button_state.candidate_tick = HAL_GetTick();
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
    HAL_GPIO_TogglePin(port, pin);
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
  uint8_t event = button_state.pressed_event;
  button_state.pressed_event = 0U;
  return event;
}

uint8_t BoardButton_GetReleasedEvent(void)
{
  uint8_t event = button_state.released_event;
  button_state.released_event = 0U;
  return event;
}
