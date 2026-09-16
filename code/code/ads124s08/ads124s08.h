/**
 * @file ads124s08.h
 * @brief ADS124S08 SPI 驱动公共接口。
 *
 * 命令与寄存器地址依据 TI ADS124S08 数据手册 SBAS660C 第 69～76 页。
 */
#ifndef ADS124S08_H
#define ADS124S08_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"
#include <stdint.h>

/** 驱动状态码，用于区分参数、通信、校验和转换异常。 */
typedef enum
{
  ADS124S08_OK = 0,
  ADS124S08_ERROR_PARAM,
  ADS124S08_ERROR_SPI,
  ADS124S08_ERROR_ID,
  ADS124S08_ERROR_VERIFY,
  ADS124S08_ERROR_TIMEOUT,
  ADS124S08_ERROR_SATURATED
} ADS124S08_Status;

typedef enum
{
  /* ADS124S08 SPI 命令字，数值直接对应数据手册命令表。 */
  ADS124S08_COMMAND_NOP       = 0x00,
  ADS124S08_COMMAND_WAKEUP    = 0x02,
  ADS124S08_COMMAND_POWERDOWN = 0x04,
  ADS124S08_COMMAND_RESET     = 0x06,
  ADS124S08_COMMAND_START     = 0x08,
  ADS124S08_COMMAND_STOP      = 0x0A,
  ADS124S08_COMMAND_SYOCAL    = 0x16,
  ADS124S08_COMMAND_SYGCAL    = 0x17,
  ADS124S08_COMMAND_SFOCAL    = 0x19,
  ADS124S08_COMMAND_RDATA     = 0x12
} ADS124S08_Command;

typedef enum
{
  /* ADS124S08 寄存器地址，寄存器映射见数据手册第 72 页。 */
  ADS124S08_REG_ID       = 0x00,
  ADS124S08_REG_STATUS   = 0x01,
  ADS124S08_REG_INPMUX   = 0x02,
  ADS124S08_REG_PGA      = 0x03,
  ADS124S08_REG_DATARATE = 0x04,
  ADS124S08_REG_REF      = 0x05,
  ADS124S08_REG_IDACMAG  = 0x06,
  ADS124S08_REG_IDACMUX  = 0x07,
  ADS124S08_REG_VBIAS    = 0x08,
  ADS124S08_REG_SYS      = 0x09,
  ADS124S08_REG_OFCAL0   = 0x0A,
  ADS124S08_REG_OFCAL1   = 0x0B,
  ADS124S08_REG_OFCAL2   = 0x0C,
  ADS124S08_REG_FSCAL0   = 0x0D,
  ADS124S08_REG_FSCAL1   = 0x0E,
  ADS124S08_REG_FSCAL2   = 0x0F,
  ADS124S08_REG_GPIODAT  = 0x10,
  ADS124S08_REG_GPIOCON  = 0x11
} ADS124S08_Register;

typedef struct
{
  /** SPI、软件片选以及复用的 DOUT/DRDY 引脚配置。 */
  SPI_HandleTypeDef *spi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  GPIO_TypeDef *dout_port;
  uint16_t dout_pin;
  uint32_t spi_timeout_ms;
} ADS124S08_HandleTypeDef;

/** @brief 软件复位、核对设备 ID，并写入 PT1000 采集所需公共配置。 */
ADS124S08_Status ADS124S08_Init(ADS124S08_HandleTypeDef *device);
/** @brief 在一次独立的 CS 时序内发送单字节系统命令。 */
ADS124S08_Status ADS124S08_SendCommand(ADS124S08_HandleTypeDef *device,
                                       uint8_t command);
/** @brief 从 start_address 开始连续读取 count 个寄存器。 */
ADS124S08_Status ADS124S08_ReadRegisters(ADS124S08_HandleTypeDef *device,
                                         uint8_t start_address,
                                         uint8_t *data,
                                         uint8_t count);
/** @brief 从 start_address 开始连续写入 count 个寄存器。 */
ADS124S08_Status ADS124S08_WriteRegisters(ADS124S08_HandleTypeDef *device,
                                          uint8_t start_address,
                                          const uint8_t *data,
                                          uint8_t count);
/** @brief 配置差分输入与 IDAC1 输出，IDAC2 始终保持断开。 */
ADS124S08_Status ADS124S08_ConfigureChannel(ADS124S08_HandleTypeDef *device,
                                            uint8_t positive_input,
                                            uint8_t negative_input,
                                            uint8_t idac1_output);
/** @brief 启动单次转换，轮询 DOUT/DRDY，再读取并符号扩展 24 位结果。 */
ADS124S08_Status ADS124S08_ReadSingle(ADS124S08_HandleTypeDef *device,
                                      int32_t *code,
                                      uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
