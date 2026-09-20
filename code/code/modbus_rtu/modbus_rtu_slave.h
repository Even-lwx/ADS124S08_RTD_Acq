/**
 * @file modbus_rtu_slave.h
 * @brief 通用 Modbus RTU 从机协议接口。
 *
 * 当前实现支持 0x03 读保持寄存器和 0x06 写单个保持寄存器。
 * 具体寄存器含义由上层回调提供，协议层只处理帧、CRC和异常响应。
 */
#ifndef MODBUS_RTU_SLAVE_H
#define MODBUS_RTU_SLAVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rs485.h"

/** 请求和响应缓冲区容量。 */
#define MODBUS_RTU_SLAVE_BUFFER_SIZE 64U

/** @brief 业务寄存器访问结果，协议层会将其转换为异常码。 */
typedef enum
{
  MODBUS_REGISTER_OK = 0, /**< 寄存器访问成功。 */
  MODBUS_REGISTER_ILLEGAL_ADDRESS, /**< 寄存器不存在或不可执行该操作。 */
  MODBUS_REGISTER_ILLEGAL_VALUE, /**< 写入值超出允许范围。 */
  MODBUS_REGISTER_DEVICE_FAILURE /**< 业务层内部错误。 */
} ModbusRegister_Status;

/** @brief 读取一个保持寄存器的业务回调。 */
typedef ModbusRegister_Status (*ModbusRegister_ReadCallback)(
    void *context, uint16_t address, uint16_t *value);
/** @brief 写入一个保持寄存器的业务回调。 */
typedef ModbusRegister_Status (*ModbusRegister_WriteCallback)(
    void *context, uint16_t address, uint16_t value);

/** @brief Modbus RTU 从机实例。 */
typedef struct
{
  RS485_HandleTypeDef *rs485; /**< 半双工物理层实例。 */
  uint8_t slave_address; /**< 本机地址，合法范围为1～247。 */
  ModbusRegister_ReadCallback read_register; /**< 寄存器读取回调。 */
  ModbusRegister_WriteCallback write_register; /**< 寄存器写入回调。 */
  void *register_context; /**< 传递给业务回调的上下文。 */
  uint8_t request[MODBUS_RTU_SLAVE_BUFFER_SIZE]; /**< 请求解析缓冲区。 */
  uint8_t response[MODBUS_RTU_SLAVE_BUFFER_SIZE]; /**< 异步发送响应缓冲区。 */
} ModbusRTUSlave_HandleTypeDef;

/**
 * @brief 计算标准 Modbus CRC16。
 * @note 初值为0xFFFF，多项式为0xA001；发送时低字节在前。
 */
uint16_t ModbusRTUSlave_CRC16(const uint8_t *data, uint16_t length);

/** @brief 初始化协议实例并绑定业务寄存器回调。 */
HAL_StatusTypeDef ModbusRTUSlave_Init(
    ModbusRTUSlave_HandleTypeDef *handle,
    RS485_HandleTypeDef *rs485,
    uint8_t slave_address,
    ModbusRegister_ReadCallback read_register,
    ModbusRegister_WriteCallback write_register,
    void *register_context);

/** @brief 在主循环中检查完整请求并生成响应。 */
void ModbusRTUSlave_Process(ModbusRTUSlave_HandleTypeDef *handle,
                            uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
