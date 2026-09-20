/**
 * @file modbus_rtu_slave.c
 * @brief Modbus RTU 从机CRC校验、功能码解析和异常响应实现。
 */
#include "modbus_rtu_slave.h"

#include <stddef.h>
#include <string.h>

/** 读取保持寄存器功能码。 */
#define MODBUS_FUNCTION_READ_HOLDING_REGISTERS 0x03U
/** 写单个保持寄存器功能码。 */
#define MODBUS_FUNCTION_WRITE_SINGLE_REGISTER  0x06U
/** 非法功能异常码。 */
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION       0x01U
/** 非法数据地址异常码。 */
#define MODBUS_EXCEPTION_ILLEGAL_ADDRESS        0x02U
/** 非法数据值异常码。 */
#define MODBUS_EXCEPTION_ILLEGAL_VALUE          0x03U
/** 从机设备故障异常码。 */
#define MODBUS_EXCEPTION_DEVICE_FAILURE         0x04U

/** @brief 从 Modbus 大端字段读取16位数值。 */
static uint16_t ModbusRTUSlave_ReadU16(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

/** @brief 把16位数值按 Modbus 大端顺序写入帧。 */
static void ModbusRTUSlave_WriteU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value >> 8);
  data[1] = (uint8_t)value;
}

/** @brief 追加低字节在前的CRC并异步发送完整响应。 */
static void ModbusRTUSlave_Send(ModbusRTUSlave_HandleTypeDef *handle,
                                uint16_t payload_length)
{
  uint16_t crc = ModbusRTUSlave_CRC16(handle->response, payload_length);

  handle->response[payload_length] = (uint8_t)crc;
  handle->response[payload_length + 1U] = (uint8_t)(crc >> 8);
  (void)RS485_Transmit_IT(handle->rs485, handle->response,
                         (uint16_t)(payload_length + 2U));
}

/** @brief 发送标准三字节异常响应正文。 */
static void ModbusRTUSlave_SendException(
    ModbusRTUSlave_HandleTypeDef *handle,
    uint8_t function,
    uint8_t exception_code)
{
  handle->response[0] = handle->slave_address;
  handle->response[1] = (uint8_t)(function | 0x80U);
  handle->response[2] = exception_code;
  ModbusRTUSlave_Send(handle, 3U);
}

/** @brief 把业务回调结果转换为Modbus异常码。 */
static uint8_t ModbusRTUSlave_StatusToException(ModbusRegister_Status status)
{
  switch (status)
  {
    case MODBUS_REGISTER_ILLEGAL_ADDRESS:
      return MODBUS_EXCEPTION_ILLEGAL_ADDRESS;
    case MODBUS_REGISTER_ILLEGAL_VALUE:
      return MODBUS_EXCEPTION_ILLEGAL_VALUE;
    case MODBUS_REGISTER_DEVICE_FAILURE:
    default:
      return MODBUS_EXCEPTION_DEVICE_FAILURE;
  }
}

/** @brief 处理0x03请求并读取连续保持寄存器。 */
static void ModbusRTUSlave_ProcessRead(ModbusRTUSlave_HandleTypeDef *handle)
{
  uint16_t start = ModbusRTUSlave_ReadU16(&handle->request[2]);
  uint16_t quantity = ModbusRTUSlave_ReadU16(&handle->request[4]);
  uint16_t value;
  uint16_t index;
  ModbusRegister_Status status;

  if ((quantity == 0U) || (quantity > 125U) ||
      ((5U + (2U * quantity)) > MODBUS_RTU_SLAVE_BUFFER_SIZE) ||
      (((uint32_t)start + quantity) > 0x10000UL))
  {
    ModbusRTUSlave_SendException(handle,
        MODBUS_FUNCTION_READ_HOLDING_REGISTERS,
        MODBUS_EXCEPTION_ILLEGAL_VALUE);
    return;
  }

  handle->response[0] = handle->slave_address;
  handle->response[1] = MODBUS_FUNCTION_READ_HOLDING_REGISTERS;
  handle->response[2] = (uint8_t)(quantity * 2U);
  for (index = 0U; index < quantity; index++)
  {
    status = handle->read_register(handle->register_context,
                                   (uint16_t)(start + index), &value);
    if (status != MODBUS_REGISTER_OK)
    {
      ModbusRTUSlave_SendException(handle,
          MODBUS_FUNCTION_READ_HOLDING_REGISTERS,
          ModbusRTUSlave_StatusToException(status));
      return;
    }
    ModbusRTUSlave_WriteU16(&handle->response[3U + (2U * index)], value);
  }
  ModbusRTUSlave_Send(handle, (uint16_t)(3U + (2U * quantity)));
}

/** @brief 处理0x06请求，成功时回显请求正文。 */
static void ModbusRTUSlave_ProcessWrite(ModbusRTUSlave_HandleTypeDef *handle)
{
  uint16_t address = ModbusRTUSlave_ReadU16(&handle->request[2]);
  uint16_t value = ModbusRTUSlave_ReadU16(&handle->request[4]);
  ModbusRegister_Status status = handle->write_register(
      handle->register_context, address, value);

  if (status != MODBUS_REGISTER_OK)
  {
    ModbusRTUSlave_SendException(handle,
        MODBUS_FUNCTION_WRITE_SINGLE_REGISTER,
        ModbusRTUSlave_StatusToException(status));
    return;
  }
  memcpy(handle->response, handle->request, 6U);
  ModbusRTUSlave_Send(handle, 6U);
}

uint16_t ModbusRTUSlave_CRC16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint16_t byte_index;
  uint8_t bit_index;

  if (data == NULL)
  {
    return 0U;
  }
  for (byte_index = 0U; byte_index < length; byte_index++)
  {
    crc ^= data[byte_index];
    for (bit_index = 0U; bit_index < 8U; bit_index++)
    {
      crc = ((crc & 1U) != 0U) ?
          (uint16_t)((crc >> 1) ^ 0xA001U) : (uint16_t)(crc >> 1);
    }
  }
  return crc;
}

HAL_StatusTypeDef ModbusRTUSlave_Init(
    ModbusRTUSlave_HandleTypeDef *handle,
    RS485_HandleTypeDef *rs485,
    uint8_t slave_address,
    ModbusRegister_ReadCallback read_register,
    ModbusRegister_WriteCallback write_register,
    void *register_context)
{
  if ((handle == NULL) || (rs485 == NULL) || (slave_address == 0U) ||
      (slave_address > 247U) || (read_register == NULL) ||
      (write_register == NULL))
  {
    return HAL_ERROR;
  }

  memset(handle, 0, sizeof(*handle));
  handle->rs485 = rs485;
  handle->slave_address = slave_address;
  handle->read_register = read_register;
  handle->write_register = write_register;
  handle->register_context = register_context;
  return HAL_OK;
}

void ModbusRTUSlave_Process(ModbusRTUSlave_HandleTypeDef *handle,
                            uint32_t now_ms)
{
  uint16_t length;
  uint16_t received_crc;
  uint16_t calculated_crc;
  uint8_t function;

  if ((handle == NULL) || (handle->rs485 == NULL) ||
      (handle->rs485->tx_busy != 0U) ||
      (RS485_TryReadFrame(handle->rs485, now_ms, handle->request,
                          sizeof(handle->request), &length) == 0U))
  {
    return;
  }

  /* 无法可靠识别的帧静默丢弃，避免在共享总线上制造额外冲突。 */
  if (length < 4U)
  {
    return;
  }
  received_crc = (uint16_t)handle->request[length - 2U] |
                 ((uint16_t)handle->request[length - 1U] << 8);
  calculated_crc = ModbusRTUSlave_CRC16(handle->request,
                                        (uint16_t)(length - 2U));
  if ((received_crc != calculated_crc) ||
      (handle->request[0] != handle->slave_address))
  {
    /* 本项目不执行地址0广播写入，防止广播命令意外启动加热。 */
    return;
  }

  function = handle->request[1];
  if (((function == MODBUS_FUNCTION_READ_HOLDING_REGISTERS) ||
       (function == MODBUS_FUNCTION_WRITE_SINGLE_REGISTER)) &&
      (length != 8U))
  {
    return;
  }
  if (function == MODBUS_FUNCTION_READ_HOLDING_REGISTERS)
  {
    ModbusRTUSlave_ProcessRead(handle);
  }
  else if (function == MODBUS_FUNCTION_WRITE_SINGLE_REGISTER)
  {
    ModbusRTUSlave_ProcessWrite(handle);
  }
  else
  {
    ModbusRTUSlave_SendException(handle, function,
                                 MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
  }
}
