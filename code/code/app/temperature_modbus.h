/**
 * @file temperature_modbus.h
 * @brief 四路温控系统的 Modbus RTU 寄存器映射接口。
 */
#ifndef TEMPERATURE_MODBUS_H
#define TEMPERATURE_MODBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "modbus_rtu_slave.h"
#include "temperature_control.h"

/** 9600 8N1 下大于3.5个字符时间的帧间静默时间。 */
#define TEMPERATURE_MODBUS_FRAME_GAP_MS 4U
/** 无效温度或反馈的固定寄存器编码。 */
#define TEMPERATURE_MODBUS_INVALID_VALUE 0x8000U

/** @brief 温控保持寄存器地址。 */
typedef enum
{
  TEMPERATURE_MODBUS_REG_STATUS = 0x0000U, /**< 系统状态位，只读。 */
  TEMPERATURE_MODBUS_REG_TEMPERATURE_1 = 0x0001U, /**< CH1温度，只读。 */
  TEMPERATURE_MODBUS_REG_TEMPERATURE_2 = 0x0002U, /**< CH2温度，只读。 */
  TEMPERATURE_MODBUS_REG_TEMPERATURE_3 = 0x0003U, /**< CH3温度，只读。 */
  TEMPERATURE_MODBUS_REG_TEMPERATURE_4 = 0x0004U, /**< CH4温度，只读。 */
  TEMPERATURE_MODBUS_REG_FEEDBACK = 0x0005U, /**< PID反馈温度，只读。 */
  TEMPERATURE_MODBUS_REG_PWM_DUTY = 0x0006U, /**< PWM占空比，只读。 */
  TEMPERATURE_MODBUS_REG_SETPOINT = 0x0007U, /**< 目标温度，读写。 */
  TEMPERATURE_MODBUS_REG_ENABLE = 0x0008U /**< 加热使能，读写。 */
} TemperatureModbus_Register;

/** @brief 0x0000状态寄存器的位定义。 */
typedef enum
{
  TEMPERATURE_MODBUS_STATUS_ENABLED = (1U << 0), /**< 软件允许加热。 */
  TEMPERATURE_MODBUS_STATUS_HEATING = (1U << 1), /**< PWM大于0%。 */
  TEMPERATURE_MODBUS_STATUS_FEEDBACK_VALID = (1U << 2), /**< 至少两路有效。 */
  TEMPERATURE_MODBUS_STATUS_OVERTEMPERATURE = (1U << 3), /**< 已过温锁定。 */
  TEMPERATURE_MODBUS_STATUS_ADC_READY = (1U << 4), /**< ADC已初始化。 */
  TEMPERATURE_MODBUS_STATUS_CHANNEL_1_VALID = (1U << 8), /**< CH1有效。 */
  TEMPERATURE_MODBUS_STATUS_CHANNEL_2_VALID = (1U << 9), /**< CH2有效。 */
  TEMPERATURE_MODBUS_STATUS_CHANNEL_3_VALID = (1U << 10), /**< CH3有效。 */
  TEMPERATURE_MODBUS_STATUS_CHANNEL_4_VALID = (1U << 11) /**< CH4有效。 */
} TemperatureModbus_StatusBit;

/** @brief 温控 Modbus RTU 从机实例。 */
typedef struct
{
  RS485_HandleTypeDef rs485; /**< USART1和PC6半双工驱动。 */
  ModbusRTUSlave_HandleTypeDef slave; /**< 通用RTU从机协议层。 */
  PT1000_App *temperature_app; /**< 四路温度快照来源。 */
  TemperatureControl *control; /**< PID、PWM和保护状态来源。 */
  uint8_t initialized; /**< 非0表示协议已经启动。 */
} TemperatureModbus_HandleTypeDef;

/** @brief 初始化温控 Modbus RTU 从机并启动USART1接收中断。 */
HAL_StatusTypeDef TemperatureModbus_Init(
    TemperatureModbus_HandleTypeDef *handle,
    UART_HandleTypeDef *uart,
    GPIO_TypeDef *direction_port,
    uint16_t direction_pin,
    uint8_t slave_address,
    PT1000_App *temperature_app,
    TemperatureControl *control);

/** @brief 在主循环中处理请求和响应。 */
void TemperatureModbus_Process(TemperatureModbus_HandleTypeDef *handle,
                               uint32_t now_ms);
/** @brief 转发USART1单字节接收完成回调。 */
void TemperatureModbus_OnUartRxCompleteISR(
    TemperatureModbus_HandleTypeDef *handle,
    UART_HandleTypeDef *uart);
/** @brief 转发USART1整帧发送完成回调。 */
void TemperatureModbus_OnUartTxCompleteISR(
    TemperatureModbus_HandleTypeDef *handle,
    UART_HandleTypeDef *uart);
/** @brief 转发USART1错误回调。 */
void TemperatureModbus_OnUartErrorISR(
    TemperatureModbus_HandleTypeDef *handle,
    UART_HandleTypeDef *uart);

#ifdef __cplusplus
}
#endif

#endif
