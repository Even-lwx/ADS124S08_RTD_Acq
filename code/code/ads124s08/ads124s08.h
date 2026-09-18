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

/**
 * @brief ADS124S08 驱动返回状态。
 *
 * 所有可能失败的公共接口均返回该枚举，调用者可据此判断是否需要重新初始化
 * ADC、丢弃本次采样或执行安全停机。
 */
typedef enum
{
  ADS124S08_OK = 0,             /**< 操作成功。 */
  ADS124S08_ERROR_PARAM,        /**< 指针、寄存器范围或通道编号非法。 */
  ADS124S08_ERROR_SPI,          /**< STM32 HAL SPI 收发失败或超时。 */
  ADS124S08_ERROR_ID,           /**< ID 寄存器不是 ADS124S08 的器件编码。 */
  ADS124S08_ERROR_VERIFY,       /**< 寄存器写入后的读回值不一致。 */
  ADS124S08_ERROR_TIMEOUT,      /**< START 后等待 DOUT/DRDY 变低超过指定时间。 */
  ADS124S08_ERROR_SATURATED     /**< 24 位转换结果达到正或负满量程。 */
} ADS124S08_Status;

/** @brief ADS124S08 单字节系统命令，编码来自 SBAS660C 命令表。 */
typedef enum
{
  ADS124S08_COMMAND_NOP       = 0x00, /**< 空操作。 */
  ADS124S08_COMMAND_WAKEUP    = 0x02, /**< 从掉电模式唤醒。 */
  ADS124S08_COMMAND_POWERDOWN = 0x04, /**< 进入掉电模式。 */
  ADS124S08_COMMAND_RESET     = 0x06, /**< 软件复位寄存器和数字逻辑。 */
  ADS124S08_COMMAND_START     = 0x08, /**< 启动一次或连续转换。 */
  ADS124S08_COMMAND_STOP      = 0x0A, /**< 停止连续转换。 */
  ADS124S08_COMMAND_SYOCAL    = 0x16, /**< 系统失调校准。 */
  ADS124S08_COMMAND_SYGCAL    = 0x17, /**< 系统增益校准。 */
  ADS124S08_COMMAND_SFOCAL    = 0x19, /**< 自失调校准。 */
  ADS124S08_COMMAND_RDATA     = 0x12  /**< 读取最近一次转换结果。 */
} ADS124S08_Command;

/** @brief ADS124S08 寄存器地址，映射来自 SBAS660C 第 72 页。 */
typedef enum
{
  ADS124S08_REG_ID       = 0x00, /**< 器件 ID。 */
  ADS124S08_REG_STATUS   = 0x01, /**< 状态标志。 */
  ADS124S08_REG_INPMUX   = 0x02, /**< ADC 正、负输入多路复用选择。 */
  ADS124S08_REG_PGA      = 0x03, /**< PGA 旁路、使能及增益。 */
  ADS124S08_REG_DATARATE = 0x04, /**< 转换模式、滤波器和数据率。 */
  ADS124S08_REG_REF      = 0x05, /**< ADC 参考源与内部参考模式。 */
  ADS124S08_REG_IDACMAG  = 0x06, /**< IDAC 电流幅值。 */
  ADS124S08_REG_IDACMUX  = 0x07, /**< IDAC1、IDAC2 输出引脚选择。 */
  ADS124S08_REG_VBIAS    = 0x08, /**< 传感器偏置电压选择。 */
  ADS124S08_REG_SYS      = 0x09, /**< 系统监测、CRC 与数据状态设置。 */
  ADS124S08_REG_OFCAL0   = 0x0A, /**< 失调校准系数低字节。 */
  ADS124S08_REG_OFCAL1   = 0x0B, /**< 失调校准系数中字节。 */
  ADS124S08_REG_OFCAL2   = 0x0C, /**< 失调校准系数高字节。 */
  ADS124S08_REG_FSCAL0   = 0x0D, /**< 满量程校准系数低字节。 */
  ADS124S08_REG_FSCAL1   = 0x0E, /**< 满量程校准系数中字节。 */
  ADS124S08_REG_FSCAL2   = 0x0F, /**< 满量程校准系数高字节。 */
  ADS124S08_REG_GPIODAT  = 0x10, /**< ADS124S08 GPIO 数据。 */
  ADS124S08_REG_GPIOCON  = 0x11  /**< ADS124S08 GPIO 方向与功能。 */
} ADS124S08_Register;

/** @brief ADS124S08 驱动实例所需的 STM32 硬件资源。 */
typedef struct
{
  SPI_HandleTypeDef *spi; /**< HAL SPI 句柄；工程中绑定 SPI1。 */
  GPIO_TypeDef *cs_port;  /**< 软件片选 CS# 所在 GPIO 端口。 */
  uint16_t cs_pin;        /**< 软件片选 CS# 的 GPIO 引脚掩码。 */
  GPIO_TypeDef *dout_port;/**< DOUT/DRDY 所在 GPIO 端口。 */
  uint16_t dout_pin;      /**< DOUT/DRDY 的 GPIO 引脚掩码。 */
  uint32_t spi_timeout_ms;/**< 每次 HAL SPI 阻塞传输的超时时间，单位 ms。 */
} ADS124S08_HandleTypeDef;

/**
 * @brief 初始化 ADS124S08 并校验关键配置。
 * @param device 已填写 SPI、CS#、DOUT/DRDY 和 SPI 超时的驱动句柄。
 * @return ADS124S08_OK 表示器件 ID 和全部关键寄存器校验通过，否则返回错误码。
 * @note 依次执行软件复位、复位等待、ID 检查和 PGA/DATARATE/REF/IDAC/SYS 配置。
 */
ADS124S08_Status ADS124S08_Init(ADS124S08_HandleTypeDef *device);
/**
 * @brief 在一次独立的 CS# 低电平窗口内发送单字节系统命令。
 * @param device 驱动句柄。
 * @param command ADS124S08_Command 中的命令字，或数据手册定义的其他合法命令。
 * @return 操作状态。
 */
ADS124S08_Status ADS124S08_SendCommand(ADS124S08_HandleTypeDef *device,
                                       uint8_t command);
/**
 * @brief 使用 RREG 命令连续读取寄存器。
 * @param device 驱动句柄。
 * @param start_address 首个寄存器地址，范围 0x00～0x11。
 * @param data 接收缓冲区，至少可容纳 count 字节。
 * @param count 读取字节数，必须大于 0 且不得越过 GPIOCON。
 * @return 操作状态。
 */
ADS124S08_Status ADS124S08_ReadRegisters(ADS124S08_HandleTypeDef *device,
                                         uint8_t start_address,
                                         uint8_t *data,
                                         uint8_t count);
/**
 * @brief 使用 WREG 命令连续写入寄存器。
 * @param device 驱动句柄。
 * @param start_address 首个寄存器地址，范围 0x00～0x11。
 * @param data 待写入的数据缓冲区。
 * @param count 写入字节数，必须大于 0 且不得越过 GPIOCON。
 * @return 操作状态；本函数不自动读回校验。
 */
ADS124S08_Status ADS124S08_WriteRegisters(ADS124S08_HandleTypeDef *device,
                                          uint8_t start_address,
                                          const uint8_t *data,
                                          uint8_t count);
/**
 * @brief 配置一个 PT1000 通道的差分输入和激励电流路径。
 * @param device 驱动句柄。
 * @param positive_input ADC 正输入 AIN 编号，范围 0～11。
 * @param negative_input ADC 负输入 AIN 编号，范围 0～11。
 * @param idac1_output IDAC1 输出 AIN 编号，范围 0～11。
 * @return 写入并读回 INPMUX～IDACMUX 后的校验状态。
 * @note IDAC2 固定写为断开，IDAC1 电流幅值使用初始化时配置的 250 uA。
 */
ADS124S08_Status ADS124S08_ConfigureChannel(ADS124S08_HandleTypeDef *device,
                                            uint8_t positive_input,
                                            uint8_t negative_input,
                                            uint8_t idac1_output);
/**
 * @brief 完成当前通道的一次阻塞式单次转换。
 * @param device 驱动句柄。
 * @param code 用于返回符号扩展后的 24 位有符号 ADC 原始码。
 * @param timeout_ms START 后等待 DOUT/DRDY 变低的最大时间，单位 ms。
 * @return 转换状态；满量程码返回 ADS124S08_ERROR_SATURATED。
 * @note START 前先读 INPMUX 恢复 DOUT/DRDY；START 后至少等待 35 ms，
 *       避免把上一次残留的低电平和旧数据误判为本次转换结果。
 */
ADS124S08_Status ADS124S08_ReadSingle(ADS124S08_HandleTypeDef *device,
                                      int32_t *code,
                                      uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
