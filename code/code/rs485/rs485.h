/**
 * @file rs485.h
 * @brief 基于独立方向控制引脚的半双工 RS485 驱动接口。
 *
 * 本板使用 PC6 同时控制 SN65HVD75 的 DE 和 /RE：高电平发送，
 * 低电平接收。USART1 的字节接收由中断连续完成，主循环通过总线
 * 静默时间判断一帧是否结束。
 */
#ifndef RS485_H
#define RS485_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"

/** 接收缓冲区容量；足以容纳本项目支持的 Modbus RTU 请求。 */
#define RS485_RX_BUFFER_SIZE 64U

/** @brief RS485 驱动运行实例。 */
typedef struct
{
  UART_HandleTypeDef *uart; /**< 绑定的 UART，本工程固定为 USART1。 */
  GPIO_TypeDef *direction_port; /**< DE、/RE 共用方向引脚的 GPIO 端口。 */
  uint16_t direction_pin; /**< DE、/RE 共用方向引脚。 */
  uint32_t frame_gap_ms; /**< 判定一帧结束所需的最小静默时间。 */
  uint8_t rx_byte; /**< HAL 单字节中断接收的临时存储。 */
  volatile uint8_t rx_buffer[RS485_RX_BUFFER_SIZE]; /**< 中断写入的接收帧。 */
  volatile uint16_t rx_length; /**< 当前已接收的字节数。 */
  volatile uint32_t last_rx_tick; /**< 最近一个字节到达的毫秒时间。 */
  volatile uint8_t rx_overflow; /**< 非 0 表示当前帧超过缓冲区，应整帧丢弃。 */
  volatile uint8_t tx_busy; /**< 非 0 表示 UART 正在发送，方向保持为高。 */
} RS485_HandleTypeDef;

/**
 * @brief 初始化半双工驱动并启动单字节中断接收。
 * @param handle 驱动实例。
 * @param uart USART1 HAL 句柄。
 * @param direction_port DE、/RE 方向引脚端口。
 * @param direction_pin DE、/RE 方向引脚。
 * @param frame_gap_ms 帧间静默时间；9600 bit/s 下使用 4 ms。
 * @return HAL_OK 表示已进入接收状态，否则返回 HAL_ERROR。
 */
HAL_StatusTypeDef RS485_Init(RS485_HandleTypeDef *handle,
                             UART_HandleTypeDef *uart,
                             GPIO_TypeDef *direction_port,
                             uint16_t direction_pin,
                             uint32_t frame_gap_ms);

/**
 * @brief 尝试取出一帧已经结束的接收数据。
 * @param handle 驱动实例。
 * @param now_ms 当前 HAL 毫秒计数。
 * @param destination 帧数据目标缓冲区。
 * @param capacity 目标缓冲区容量。
 * @param length 返回实际帧长度。
 * @return 1 表示取得一帧；0 表示尚无完整帧或溢出帧已被丢弃。
 * @note 只有最近一个字节后静默达到 frame_gap_ms 才会交付帧。
 */
uint8_t RS485_TryReadFrame(RS485_HandleTypeDef *handle,
                           uint32_t now_ms,
                           uint8_t *destination,
                           uint16_t capacity,
                           uint16_t *length);

/**
 * @brief 使用中断方式发送一帧，并自动切换为发送方向。
 * @param handle 驱动实例。
 * @param data 发送缓冲区；发送完成前必须保持有效。
 * @param length 发送字节数。
 * @return HAL_UART_Transmit_IT() 的状态。
 */
HAL_StatusTypeDef RS485_Transmit_IT(RS485_HandleTypeDef *handle,
                                   uint8_t *data,
                                   uint16_t length);

/** @brief 从 HAL UART 接收完成回调中转入，保存一个字节并继续接收。 */
void RS485_OnUartRxCompleteISR(RS485_HandleTypeDef *handle,
                               UART_HandleTypeDef *uart);
/** @brief 从 HAL UART 发送完成回调中转入，恢复接收方向和中断接收。 */
void RS485_OnUartTxCompleteISR(RS485_HandleTypeDef *handle,
                               UART_HandleTypeDef *uart);
/** @brief 从 HAL UART 错误回调中转入，清理错误并恢复接收。 */
void RS485_OnUartErrorISR(RS485_HandleTypeDef *handle,
                          UART_HandleTypeDef *uart);

#ifdef __cplusplus
}
#endif

#endif
