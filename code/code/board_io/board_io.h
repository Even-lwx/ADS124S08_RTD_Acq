/**
 * @file board_io.h
 * @brief PCB 板载按键与指示灯驱动的公共接口。
 *
 * 原理图依据：PB3 通过 R20 接 SW2 节点，R19 将该节点上拉到 3V3，
 * 因此按键按下为低电平；PA12/PA15 接 LED 阴极，两个 LED 均为低电平点亮。
 */
#ifndef BOARD_IO_H
#define BOARD_IO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 板载 LED 编号，与原理图器件位号一一对应。 */
typedef enum
{
  BOARD_LED_1 = 0, /**< 原理图 LED1，对应 PA12。 */
  BOARD_LED_3,     /**< 原理图 LED3，对应 PA15。 */
  BOARD_LED_COUNT  /**< LED 数量及枚举边界，不作为有效 LED 传入接口。 */
} BoardLED;

/**
 * @brief 初始化板级 IO 软件状态，并将 LED1、LED3 关闭。
 * @note 必须在 MX_GPIO_Init() 之后调用。
 */
void BoardIO_Init(void);

/**
 * @brief 设置指定 LED 的逻辑状态。
 * @param led BOARD_LED_1 或 BOARD_LED_3。
 * @param on 非 0 表示点亮，0 表示熄灭；驱动内部负责低有效转换。
 */
void BoardLED_Set(BoardLED led, uint8_t on);

/**
 * @brief 翻转指定 LED 的当前物理输出状态。
 * @param led BOARD_LED_1 或 BOARD_LED_3；非法编号将被忽略。
 */
void BoardLED_Toggle(BoardLED led);

/**
 * @brief 根据加热输出状态更新 LED1（PA12）的非阻塞闪烁节拍。
 * @param heating 非 0 表示当前 PWM 正在加热，0 表示当前未加热。
 * @note 加热时 LED1 每 100 ms 翻转一次；未加热时每 500 ms 翻转一次。
 *       本函数内部使用 HAL_GetTick() 计时，应在主循环中持续调用。
 */
void BoardHeatingIndicator_Update(uint8_t heating);

/**
 * @brief 更新按键消抖状态。
 * @note 此函数不阻塞，应在主循环中持续调用；消抖时间固定为 20 ms。
 */
void BoardButton_Update(void);

/**
 * @brief 获取当前稳定的按键逻辑状态，不会清除任何事件。
 * @return 消抖后的按键状态：1 表示按下，0 表示释放。
 */
uint8_t BoardButton_IsPressed(void);

/**
 * @brief 读取并清除一次“按下”事件。
 * @return 自上次读取以来发生过稳定按下时返回 1，否则返回 0。
 * @note 若事件发生后一直未读取，标志保持为 1；不累计多次事件数量。
 */
uint8_t BoardButton_GetPressedEvent(void);

/**
 * @brief 读取并清除一次“释放”事件。
 * @return 自上次读取以来发生过稳定释放时返回 1，否则返回 0。
 * @note 若事件发生后一直未读取，标志保持为 1；不累计多次事件数量。
 */
uint8_t BoardButton_GetReleasedEvent(void);

#ifdef __cplusplus
}
#endif

#endif
