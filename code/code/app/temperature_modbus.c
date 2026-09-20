/**
 * @file temperature_modbus.c
 * @brief 温度、PWM、目标温度和启停状态的 Modbus 保持寄存器实现。
 */
#include "temperature_modbus.h"

#include <stddef.h>
#include <string.h>

/** 允许PID控制所需的最少有效测温通道数。 */
#define TEMPERATURE_MODBUS_MIN_VALID_CHANNELS 2U

/** @brief 将浮点量四舍五入为放大100倍的有符号寄存器值。 */
static uint16_t TemperatureModbus_EncodeHundredths(float value)
{
  float scaled = value * 100.0f;
  int32_t rounded;

  if (scaled > 32767.0f)
  {
    scaled = 32767.0f;
  }
  else if (scaled < -32767.0f)
  {
    /* -32768专门保留给无效值，不用于正常温度。 */
    scaled = -32767.0f;
  }
  rounded = (int32_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
  return (uint16_t)(int16_t)rounded;
}

/** @brief 统计快照中有效的温度通道。 */
static uint8_t TemperatureModbus_CountValid(
    const PT1000_AppSnapshot *snapshot)
{
  uint8_t channel;
  uint8_t count = 0U;

  for (channel = 0U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
  {
    if (snapshot->valid[channel] != 0U)
    {
      count++;
    }
  }
  return count;
}

/** @brief 根据当前快照生成状态寄存器。 */
static uint16_t TemperatureModbus_BuildStatus(
    const TemperatureModbus_HandleTypeDef *handle,
    const PT1000_AppSnapshot *snapshot)
{
  uint16_t status = 0U;
  uint8_t channel;

  if (handle->control->config.enabled != 0U)
  {
    status |= TEMPERATURE_MODBUS_STATUS_ENABLED;
  }
  if (handle->control->duty_percent > 0.0f)
  {
    status |= TEMPERATURE_MODBUS_STATUS_HEATING;
  }
  if (TemperatureModbus_CountValid(snapshot) >=
      TEMPERATURE_MODBUS_MIN_VALID_CHANNELS)
  {
    status |= TEMPERATURE_MODBUS_STATUS_FEEDBACK_VALID;
  }
  if (handle->control->overtemperature_latched != 0U)
  {
    status |= TEMPERATURE_MODBUS_STATUS_OVERTEMPERATURE;
  }
  if (handle->temperature_app->adc_ready != 0U)
  {
    status |= TEMPERATURE_MODBUS_STATUS_ADC_READY;
  }
  for (channel = 0U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
  {
    if (snapshot->valid[channel] != 0U)
    {
      status |= (uint16_t)(1U << (8U + channel));
    }
  }
  return status;
}

/** @brief 读取一个温控保持寄存器。 */
static ModbusRegister_Status TemperatureModbus_ReadRegister(
    void *context, uint16_t address, uint16_t *value)
{
  TemperatureModbus_HandleTypeDef *handle =
      (TemperatureModbus_HandleTypeDef *)context;
  PT1000_AppSnapshot snapshot;
  uint8_t channel;

  if ((handle == NULL) || (value == NULL))
  {
    return MODBUS_REGISTER_DEVICE_FAILURE;
  }
  PT1000_AppGetSnapshot(handle->temperature_app, &snapshot);

  if (address == TEMPERATURE_MODBUS_REG_STATUS)
  {
    *value = TemperatureModbus_BuildStatus(handle, &snapshot);
  }
  else if ((address >= TEMPERATURE_MODBUS_REG_TEMPERATURE_1) &&
           (address <= TEMPERATURE_MODBUS_REG_TEMPERATURE_4))
  {
    channel = (uint8_t)(address - TEMPERATURE_MODBUS_REG_TEMPERATURE_1);
    *value = (snapshot.valid[channel] != 0U) ?
        TemperatureModbus_EncodeHundredths(snapshot.temperature_c[channel]) :
        TEMPERATURE_MODBUS_INVALID_VALUE;
  }
  else if (address == TEMPERATURE_MODBUS_REG_FEEDBACK)
  {
    *value = (TemperatureModbus_CountValid(&snapshot) >=
              TEMPERATURE_MODBUS_MIN_VALID_CHANNELS) ?
        TemperatureModbus_EncodeHundredths(handle->control->feedback_c) :
        TEMPERATURE_MODBUS_INVALID_VALUE;
  }
  else if (address == TEMPERATURE_MODBUS_REG_PWM_DUTY)
  {
    *value = TemperatureModbus_EncodeHundredths(
        handle->control->duty_percent);
  }
  else if (address == TEMPERATURE_MODBUS_REG_SETPOINT)
  {
    *value = TemperatureModbus_EncodeHundredths(
        handle->control->config.setpoint_c);
  }
  else if (address == TEMPERATURE_MODBUS_REG_ENABLE)
  {
    *value = (handle->control->config.enabled != 0U) ? 1U : 0U;
  }
  else
  {
    return MODBUS_REGISTER_ILLEGAL_ADDRESS;
  }
  return MODBUS_REGISTER_OK;
}

/** @brief 写入目标温度或加热使能寄存器。 */
static ModbusRegister_Status TemperatureModbus_WriteRegister(
    void *context, uint16_t address, uint16_t value)
{
  TemperatureModbus_HandleTypeDef *handle =
      (TemperatureModbus_HandleTypeDef *)context;
  float setpoint_c;

  if (handle == NULL)
  {
    return MODBUS_REGISTER_DEVICE_FAILURE;
  }
  if (address == TEMPERATURE_MODBUS_REG_SETPOINT)
  {
    setpoint_c = (float)value / 100.0f;
    if (setpoint_c >= handle->control->config.overtemperature_c)
    {
      return MODBUS_REGISTER_ILLEGAL_VALUE;
    }
    return (TemperatureControl_SetSetpoint(handle->control, setpoint_c) ==
            HAL_OK) ? MODBUS_REGISTER_OK : MODBUS_REGISTER_DEVICE_FAILURE;
  }
  if (address == TEMPERATURE_MODBUS_REG_ENABLE)
  {
    if (value > 1U)
    {
      return MODBUS_REGISTER_ILLEGAL_VALUE;
    }
    return (TemperatureControl_SetEnabled(handle->control, (uint8_t)value) ==
            HAL_OK) ? MODBUS_REGISTER_OK : MODBUS_REGISTER_DEVICE_FAILURE;
  }
  return MODBUS_REGISTER_ILLEGAL_ADDRESS;
}

HAL_StatusTypeDef TemperatureModbus_Init(
    TemperatureModbus_HandleTypeDef *handle,
    UART_HandleTypeDef *uart,
    GPIO_TypeDef *direction_port,
    uint16_t direction_pin,
    uint8_t slave_address,
    PT1000_App *temperature_app,
    TemperatureControl *control)
{
  if ((handle == NULL) || (uart == NULL) || (temperature_app == NULL) ||
      (control == NULL))
  {
    return HAL_ERROR;
  }

  memset(handle, 0, sizeof(*handle));
  handle->temperature_app = temperature_app;
  handle->control = control;
  /* 启动接收前允许HAL回调进入底层驱动，避免首字节到达窗口丢失。 */
  handle->initialized = 1U;
  if (RS485_Init(&handle->rs485, uart, direction_port, direction_pin,
                 TEMPERATURE_MODBUS_FRAME_GAP_MS) != HAL_OK)
  {
    handle->initialized = 0U;
    return HAL_ERROR;
  }
  if (ModbusRTUSlave_Init(&handle->slave, &handle->rs485, slave_address,
                          TemperatureModbus_ReadRegister,
                          TemperatureModbus_WriteRegister, handle) != HAL_OK)
  {
    handle->initialized = 0U;
    return HAL_ERROR;
  }
  return HAL_OK;
}

void TemperatureModbus_Process(TemperatureModbus_HandleTypeDef *handle,
                               uint32_t now_ms)
{
  if ((handle != NULL) && (handle->initialized != 0U))
  {
    ModbusRTUSlave_Process(&handle->slave, now_ms);
  }
}

void TemperatureModbus_OnUartRxCompleteISR(
    TemperatureModbus_HandleTypeDef *handle,
    UART_HandleTypeDef *uart)
{
  if ((handle != NULL) && (handle->initialized != 0U))
  {
    RS485_OnUartRxCompleteISR(&handle->rs485, uart);
  }
}

void TemperatureModbus_OnUartTxCompleteISR(
    TemperatureModbus_HandleTypeDef *handle,
    UART_HandleTypeDef *uart)
{
  if ((handle != NULL) && (handle->initialized != 0U))
  {
    RS485_OnUartTxCompleteISR(&handle->rs485, uart);
  }
}

void TemperatureModbus_OnUartErrorISR(
    TemperatureModbus_HandleTypeDef *handle,
    UART_HandleTypeDef *uart)
{
  if ((handle != NULL) && (handle->initialized != 0U))
  {
    RS485_OnUartErrorISR(&handle->rs485, uart);
  }
}
