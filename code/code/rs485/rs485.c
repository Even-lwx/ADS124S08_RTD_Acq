/**
 * @file rs485.c
 * @brief USART 中断收发与 SN65HVD75 方向切换实现。
 */
#include "rs485.h"

#include <stddef.h>
#include <string.h>

/** @brief 将收发器切换到接收状态，PC6=0。 */
static void RS485_SetReceiveMode(RS485_HandleTypeDef *handle)
{
  HAL_GPIO_WritePin(handle->direction_port, handle->direction_pin,
                    GPIO_PIN_RESET);
}

/** @brief 将收发器切换到发送状态，PC6=1。 */
static void RS485_SetTransmitMode(RS485_HandleTypeDef *handle)
{
  HAL_GPIO_WritePin(handle->direction_port, handle->direction_pin,
                    GPIO_PIN_SET);
}

/**
 * @brief 清除可能阻止后续接收的 UART 错误标志。
 * @note FE/NE/ORE 都可能由插拔、总线浮动或方向切换瞬间产生。
 */
static void RS485_ClearUartErrors(UART_HandleTypeDef *uart)
{
  __HAL_UART_CLEAR_FLAG(uart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                              UART_CLEAR_FEF | UART_CLEAR_PEF);
  __HAL_UART_SEND_REQ(uart, UART_RXDATA_FLUSH_REQUEST);
}

/** @brief 在接收方向下重新挂起一个字节的中断接收。 */
static HAL_StatusTypeDef RS485_StartReceive(RS485_HandleTypeDef *handle)
{
  RS485_SetReceiveMode(handle);
  return HAL_UART_Receive_IT(handle->uart, &handle->rx_byte, 1U);
}

HAL_StatusTypeDef RS485_Init(RS485_HandleTypeDef *handle,
                             UART_HandleTypeDef *uart,
                             GPIO_TypeDef *direction_port,
                             uint16_t direction_pin,
                             uint32_t frame_gap_ms)
{
  if ((handle == NULL) || (uart == NULL) || (direction_port == NULL) ||
      (direction_pin == 0U) || (frame_gap_ms == 0U))
  {
    return HAL_ERROR;
  }

  memset(handle, 0, sizeof(*handle));
  handle->uart = uart;
  handle->direction_port = direction_port;
  handle->direction_pin = direction_pin;
  handle->frame_gap_ms = frame_gap_ms;
  RS485_ClearUartErrors(uart);
  return RS485_StartReceive(handle);
}

uint8_t RS485_TryReadFrame(RS485_HandleTypeDef *handle,
                           uint32_t now_ms,
                           uint8_t *destination,
                           uint16_t capacity,
                           uint16_t *length)
{
  uint32_t interrupt_state;
  uint16_t frame_length;
  uint16_t index;

  if ((handle == NULL) || (destination == NULL) || (length == NULL) ||
      (capacity == 0U) || (handle->tx_busy != 0U) ||
      (handle->rx_length == 0U) ||
      ((uint32_t)(now_ms - handle->last_rx_tick) < handle->frame_gap_ms))
  {
    return 0U;
  }

  /*
   * 只在复制长度和清空共享缓冲区的短时间内屏蔽中断。即使新的起始字节
   * 恰好在此时到达，UART 硬件也会保留 RXNE，恢复中断后仍会被读取。
   */
  interrupt_state = __get_PRIMASK();
  __disable_irq();
  frame_length = handle->rx_length;
  if ((handle->rx_overflow != 0U) || (frame_length > capacity))
  {
    handle->rx_length = 0U;
    handle->rx_overflow = 0U;
    if (interrupt_state == 0U)
    {
      __enable_irq();
    }
    return 0U;
  }
  for (index = 0U; index < frame_length; index++)
  {
    destination[index] = handle->rx_buffer[index];
  }
  handle->rx_length = 0U;
  handle->rx_overflow = 0U;
  if (interrupt_state == 0U)
  {
    __enable_irq();
  }

  *length = frame_length;
  return 1U;
}

HAL_StatusTypeDef RS485_Transmit_IT(RS485_HandleTypeDef *handle,
                                   uint8_t *data,
                                   uint16_t length)
{
  HAL_StatusTypeDef status;

  if ((handle == NULL) || (handle->uart == NULL) || (data == NULL) ||
      (length == 0U) || (handle->tx_busy != 0U))
  {
    return HAL_ERROR;
  }

  /* 取消为下一字节挂起的接收，避免发送期间 HAL 仍认为 RX 正忙。 */
  (void)HAL_UART_AbortReceive(handle->uart);
  RS485_ClearUartErrors(handle->uart);
  handle->tx_busy = 1U;
  RS485_SetTransmitMode(handle);
  status = HAL_UART_Transmit_IT(handle->uart, data, length);
  if (status != HAL_OK)
  {
    handle->tx_busy = 0U;
    (void)RS485_StartReceive(handle);
  }
  return status;
}

void RS485_OnUartRxCompleteISR(RS485_HandleTypeDef *handle,
                               UART_HandleTypeDef *uart)
{
  if ((handle == NULL) || (uart != handle->uart) ||
      (handle->tx_busy != 0U))
  {
    return;
  }

  if (handle->rx_length < RS485_RX_BUFFER_SIZE)
  {
    handle->rx_buffer[handle->rx_length++] = handle->rx_byte;
  }
  else
  {
    /* 缓冲区溢出后继续吸收本帧，待静默期到来时统一丢弃。 */
    handle->rx_overflow = 1U;
  }
  handle->last_rx_tick = HAL_GetTick();

  if (HAL_UART_Receive_IT(handle->uart, &handle->rx_byte, 1U) != HAL_OK)
  {
    /* 下一次 UART 错误回调或主循环重启可恢复，方向始终保持接收。 */
    RS485_SetReceiveMode(handle);
  }
}

void RS485_OnUartTxCompleteISR(RS485_HandleTypeDef *handle,
                               UART_HandleTypeDef *uart)
{
  if ((handle == NULL) || (uart != handle->uart) ||
      (handle->tx_busy == 0U))
  {
    return;
  }

  /* HAL 的发送完成回调发生在 TC 置位后，最后一个停止位已经离开总线。 */
  handle->tx_busy = 0U;
  RS485_ClearUartErrors(handle->uart);
  (void)RS485_StartReceive(handle);
}

void RS485_OnUartErrorISR(RS485_HandleTypeDef *handle,
                          UART_HandleTypeDef *uart)
{
  if ((handle == NULL) || (uart != handle->uart))
  {
    return;
  }

  /* 错误所在帧不再可信，整帧丢弃并从接收方向重新同步。 */
  handle->tx_busy = 0U;
  handle->rx_length = 0U;
  handle->rx_overflow = 0U;
  RS485_SetReceiveMode(handle);
  RS485_ClearUartErrors(handle->uart);
  (void)HAL_UART_Receive_IT(handle->uart, &handle->rx_byte, 1U);
}
