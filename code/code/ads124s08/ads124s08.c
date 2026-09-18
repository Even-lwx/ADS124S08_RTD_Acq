/**
 * @file ads124s08.c
 * @brief ADS124S08 的 SPI 命令、寄存器配置及单次转换实现。
 *
 * SPI 使用 Mode 1（CPOL=0、CPHA=1），CS 由 PA5 软件控制。板上未连接
 * 独立 DRDY#，因此转换期间在 CS 拉低时读取 DOUT/DRDY 电平。
 */
#include "ads124s08.h"

/** RREG/WREG 命令高 3 位；低 5 位携带首寄存器地址。 */
#define ADS124S08_CMD_RREG        0x20U
#define ADS124S08_CMD_WREG        0x40U

/** ID 寄存器低 3 位为器件型号，ADS124S08 的编码为 000b。 */
#define ADS124S08_DEVICE_ID_MASK  0x07U
#define ADS124S08_DEVICE_ID_VALUE 0x00U

/*
 * 四路 PT1000 比例测量公共配置：PGA=1，单次转换，低延迟滤波 20 SPS，
 * 参考源为 REFP0/REFN0；内部参考常开供 IDAC 使用，IDAC 电流为 250 uA。
 * 各宏值按 SBAS660C 寄存器位定义组合得到。
 */
#define ADS124S08_PGA_VALUE       0x08U
#define ADS124S08_DATARATE_VALUE  0x34U
/* 外部 REFP0/REFN0 参考、内部参考常开，并开启 0.3 V 欠压监视。 */
#define ADS124S08_REF_VALUE       0x52U
#define ADS124S08_IDACMAG_VALUE   0x04U
#define ADS124S08_SYS_VALUE       0x10U
#define ADS124S08_IDAC_DISCONNECT 0x0FU

/**
 * 20 SPS、低延迟滤波、单次模式首个结果典型为 56.504 ms。
 * 取 70 ms 覆盖内部时钟偏差和软件调度误差，防止读取上一次的旧数据。
 */
#define ADS124S08_FIRST_CONVERSION_WAIT_MS 70U

/**
 * 内部 2.5 V 参考在 REFOUT 外接 1 uF 电容时，数据手册给出的 0.001% 建立
 * 时间典型值为 5.9 ms。取 10 ms 后再启用 IDAC，避免参考尚未稳定时启动转换。
 */
#define ADS124S08_INTERNAL_REFERENCE_WAIT_MS 10U

/** @brief 拉低软件 CS#，开始一次完整的 SPI 命令事务。 */
static void ADS124S08_Select(const ADS124S08_HandleTypeDef *device)
{
  HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
}

/** @brief 拉高软件 CS#，结束当前 SPI 命令事务。 */
static void ADS124S08_Deselect(const ADS124S08_HandleTypeDef *device)
{
  HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);
}

/**
 * @brief 将 HAL SPI 发送结果统一映射为本驱动状态码。
 * @param device 驱动句柄。
 * @param data 待发送缓冲区。
 * @param size 字节数。
 * @return ADS124S08_OK 或 ADS124S08_ERROR_SPI。
 */
static ADS124S08_Status ADS124S08_Transmit(ADS124S08_HandleTypeDef *device,
                                           uint8_t *data,
                                           uint16_t size)
{
  if (HAL_SPI_Transmit(device->spi, data, size, device->spi_timeout_ms) != HAL_OK)
  {
    return ADS124S08_ERROR_SPI;
  }
  return ADS124S08_OK;
}

ADS124S08_Status ADS124S08_SendCommand(ADS124S08_HandleTypeDef *device,
                                       uint8_t command)
{
  ADS124S08_Status status;

  if ((device == NULL) || (device->spi == NULL))
  {
    return ADS124S08_ERROR_PARAM;
  }

  /* 系统命令不带参数，一个 CS# 窗口只发送一个命令字。 */
  ADS124S08_Select(device);
  status = ADS124S08_Transmit(device, &command, 1U);
  ADS124S08_Deselect(device);
  return status;
}

ADS124S08_Status ADS124S08_ReadRegisters(ADS124S08_HandleTypeDef *device,
                                         uint8_t start_address,
                                         uint8_t *data,
                                         uint8_t count)
{
  uint8_t command[2];
  uint8_t dummy = 0U;
  uint8_t index;
  ADS124S08_Status status;

  if ((device == NULL) || (data == NULL) || (count == 0U) ||
      (start_address > ADS124S08_REG_GPIOCON) ||
      (((uint16_t)start_address + count) > ((uint16_t)ADS124S08_REG_GPIOCON + 1U)))
  {
    return ADS124S08_ERROR_PARAM;
  }

  /* 第二个命令字按协议填写“读取数量减 1”。 */
  command[0] = (uint8_t)(ADS124S08_CMD_RREG | (start_address & 0x1FU));
  command[1] = (uint8_t)(count - 1U);

  ADS124S08_Select(device);
  status = ADS124S08_Transmit(device, command, sizeof(command));
  if (status == ADS124S08_OK)
  {
    /* 主机发送 0x00 产生 SCLK，同时逐字节接收寄存器数据。 */
    for (index = 0U; index < count; index++)
    {
      if (HAL_SPI_TransmitReceive(device->spi, &dummy, &data[index], 1U,
                                  device->spi_timeout_ms) != HAL_OK)
      {
        status = ADS124S08_ERROR_SPI;
        break;
      }
    }
  }
  ADS124S08_Deselect(device);
  return status;
}

ADS124S08_Status ADS124S08_WriteRegisters(ADS124S08_HandleTypeDef *device,
                                          uint8_t start_address,
                                          const uint8_t *data,
                                          uint8_t count)
{
  uint8_t command[2];
  ADS124S08_Status status;

  if ((device == NULL) || (data == NULL) || (count == 0U) ||
      (start_address > ADS124S08_REG_GPIOCON) ||
      (((uint16_t)start_address + count) > ((uint16_t)ADS124S08_REG_GPIOCON + 1U)))
  {
    return ADS124S08_ERROR_PARAM;
  }

  /* 第二个命令字按协议填写“写入数量减 1”。 */
  command[0] = (uint8_t)(ADS124S08_CMD_WREG | (start_address & 0x1FU));
  command[1] = (uint8_t)(count - 1U);

  ADS124S08_Select(device);
  status = ADS124S08_Transmit(device, command, sizeof(command));
  if (status == ADS124S08_OK)
  {
    status = ADS124S08_Transmit(device, (uint8_t *)data, count);
  }
  ADS124S08_Deselect(device);
  return status;
}

/**
 * @brief 写入单个寄存器后立即读回比较。
 * @return 写入、读取或比较阶段产生的状态码。
 */
static ADS124S08_Status ADS124S08_WriteAndVerify(
    ADS124S08_HandleTypeDef *device, uint8_t address, uint8_t value)
{
  uint8_t readback = 0U;
  ADS124S08_Status status;

  status = ADS124S08_WriteRegisters(device, address, &value, 1U);
  if (status != ADS124S08_OK)
  {
    return status;
  }
  status = ADS124S08_ReadRegisters(device, address, &readback, 1U);
  if (status != ADS124S08_OK)
  {
    return status;
  }
  return (readback == value) ? ADS124S08_OK : ADS124S08_ERROR_VERIFY;
}

ADS124S08_Status ADS124S08_Init(ADS124S08_HandleTypeDef *device)
{
  uint8_t device_id = 0U;
  ADS124S08_Status status;

  if ((device == NULL) || (device->spi == NULL) ||
      (device->cs_port == NULL) || (device->dout_port == NULL))
  {
    return ADS124S08_ERROR_PARAM;
  }

  ADS124S08_Deselect(device);
  status = ADS124S08_SendCommand(device, ADS124S08_COMMAND_RESET);
  if (status != ADS124S08_OK)
  {
    return status;
  }

  /* 软件复位后至少等待 4096 个 4.096 MHz ADC 时钟；2 ms 留有裕量。 */
  HAL_Delay(2U);

  status = ADS124S08_ReadRegisters(device, ADS124S08_REG_ID, &device_id, 1U);
  if (status != ADS124S08_OK)
  {
    return status;
  }
  if ((device_id & ADS124S08_DEVICE_ID_MASK) != ADS124S08_DEVICE_ID_VALUE)
  {
    return ADS124S08_ERROR_ID;
  }

  /* 逐项写入并校验，任一配置失败立即退出，避免带错误配置继续采样。 */
  status = ADS124S08_WriteAndVerify(device, ADS124S08_REG_PGA,
                                    ADS124S08_PGA_VALUE);
  if (status != ADS124S08_OK)
  {
    return status;
  }
  status = ADS124S08_WriteAndVerify(device, ADS124S08_REG_DATARATE,
                                    ADS124S08_DATARATE_VALUE);
  if (status != ADS124S08_OK)
  {
    return status;
  }
  status = ADS124S08_WriteAndVerify(device, ADS124S08_REG_REF,
                                    ADS124S08_REF_VALUE);
  if (status != ADS124S08_OK)
  {
    return status;
  }

  /* IDAC 依赖内部参考；必须先让 REFCON=10b 对应的内部参考充分建立。 */
  HAL_Delay(ADS124S08_INTERNAL_REFERENCE_WAIT_MS);

  status = ADS124S08_WriteAndVerify(device, ADS124S08_REG_IDACMAG,
                                    ADS124S08_IDACMAG_VALUE);
  if (status != ADS124S08_OK)
  {
    return status;
  }
  status = ADS124S08_WriteAndVerify(device, ADS124S08_REG_SYS,
                                    ADS124S08_SYS_VALUE);
  return status;
}

ADS124S08_Status ADS124S08_ConfigureChannel(ADS124S08_HandleTypeDef *device,
                                            uint8_t positive_input,
                                            uint8_t negative_input,
                                            uint8_t idac1_output)
{
  uint8_t registers[6];
  uint8_t readback[6];
  ADS124S08_Status status;

  /* ADS124S08 的 INPMUX 正、负端均只允许选择 AIN0~AIN11、AINCOM；
   * REFP0/REFN0 只能作为参考输入，不能填写到 MUXN。 */
  if ((positive_input > 11U) || (negative_input > 11U) ||
      (idac1_output > 11U))
  {
    return ADS124S08_ERROR_PARAM;
  }

  /* 连续写 INPMUX～IDACMUX，减少通道切换期间的 SPI 事务数量。 */
  registers[0] = (uint8_t)((positive_input << 4) | negative_input);
  registers[1] = ADS124S08_PGA_VALUE;
  registers[2] = ADS124S08_DATARATE_VALUE;
  registers[3] = ADS124S08_REF_VALUE;
  registers[4] = ADS124S08_IDACMAG_VALUE;
  registers[5] = (uint8_t)((ADS124S08_IDAC_DISCONNECT << 4) | idac1_output);

  status = ADS124S08_WriteRegisters(device, ADS124S08_REG_INPMUX,
                                     registers, sizeof(registers));
  if (status != ADS124S08_OK)
  {
    return status;
  }
  status = ADS124S08_ReadRegisters(device, ADS124S08_REG_INPMUX,
                                    readback, sizeof(readback));
  if (status != ADS124S08_OK)
  {
    return status;
  }
  /* 六个连续寄存器必须全部一致，防止通道或 IDAC 路由配置错误。 */
  for (uint8_t index = 0U; index < sizeof(registers); index++)
  {
    if (readback[index] != registers[index])
    {
      return ADS124S08_ERROR_VERIFY;
    }
  }
  return ADS124S08_OK;
}

/**
 * @brief 轮询复用的 DOUT/DRDY 引脚，等待单次转换结束。
 * @param device 驱动句柄。
 * @param timeout_ms 最大等待时间，使用 HAL_GetTick() 计时。
 * @return 引脚变低返回 OK，超时返回 ERROR_TIMEOUT。
 * @note 每次读取前短暂拉低 CS#；循环间隔 1 ms，避免持续占用 CPU。
 */
static ADS124S08_Status ADS124S08_WaitDataReady(
    ADS124S08_HandleTypeDef *device, uint32_t timeout_ms)
{
  uint32_t start_tick = HAL_GetTick();
  GPIO_PinState state;

  /*
   * 数据手册 SBAS660C 第 71 页规定：CS 拉低后，DOUT/DRDY 为低表示
   * 新转换数据已经就绪，为高表示尚未就绪。调用本函数前已经通过读取
   * LSB=1 的 INPMUX 寄存器强制该线恢复为高，因此这里不再额外要求观察
   * 一次“高电平阶段”，只等待本次转换产生的下降状态。
   */
  do
  {
    ADS124S08_Select(device);
    state = HAL_GPIO_ReadPin(device->dout_port, device->dout_pin);
    ADS124S08_Deselect(device);
    if (state == GPIO_PIN_RESET)
    {
      return ADS124S08_OK;
    }
    HAL_Delay(1U);
  } while ((HAL_GetTick() - start_tick) < timeout_ms);

  return ADS124S08_ERROR_TIMEOUT;
}

ADS124S08_Status ADS124S08_ReadSingle(ADS124S08_HandleTypeDef *device,
                                      int32_t *code,
                                      uint32_t timeout_ms)
{
  uint8_t command = ADS124S08_COMMAND_RDATA;
  uint8_t dummy[3] = {0U, 0U, 0U};
  uint8_t data[3] = {0U, 0U, 0U};
  uint8_t inpmux;
  uint32_t raw;
  ADS124S08_Status status;

  if ((device == NULL) || (code == NULL))
  {
    return ADS124S08_ERROR_PARAM;
  }

  /*
   * DOUT/DRDY 与 SPI MISO 复用。数据手册第 71 页要求用该脚轮询时，
   * 必须在 CS 拉高前把 DOUT/DRDY 强制为高。四路 INPMUX 的负输入均为
   * 奇数 AIN，故读回 INPMUX 后最后移出的最低位恒为 1。先执行本次读取，
   * 可清除上次数据就绪留下的低电平，使后续检测到的低电平只属于新转换。
   */
  status = ADS124S08_ReadRegisters(device, ADS124S08_REG_INPMUX, &inpmux, 1U);
  if (status != ADS124S08_OK)
  {
    return status;
  }

  status = ADS124S08_SendCommand(device, ADS124S08_COMMAND_START);
  if (status != ADS124S08_OK)
  {
    return status;
  }

  /*
   * DOUT/DRDY 在 START 前可能仍保留旧数据就绪状态。数据手册表 13 给出
   * 当前 20 SPS 低延迟单次转换首个结果约需 56.504 ms，因此先等待
   * 70 ms，再用剩余超时时间确认新数据已经就绪。
   */
  if (timeout_ms <= ADS124S08_FIRST_CONVERSION_WAIT_MS)
  {
    HAL_Delay(timeout_ms);
    return ADS124S08_ERROR_TIMEOUT;
  }
  HAL_Delay(ADS124S08_FIRST_CONVERSION_WAIT_MS);
  status = ADS124S08_WaitDataReady(
      device, timeout_ms - ADS124S08_FIRST_CONVERSION_WAIT_MS);
  if (status != ADS124S08_OK)
  {
    return status;
  }

  ADS124S08_Select(device);
  status = ADS124S08_Transmit(device, &command, 1U);
  if (status == ADS124S08_OK)
  {
    if (HAL_SPI_TransmitReceive(device->spi, dummy, data, sizeof(data),
                                device->spi_timeout_ms) != HAL_OK)
    {
      status = ADS124S08_ERROR_SPI;
    }
  }
  ADS124S08_Deselect(device);
  if (status != ADS124S08_OK)
  {
    return status;
  }

  /* ADC 按高字节在前输出 24 位二进制补码；bit23 为符号位。 */
  raw = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
  if ((raw & 0x00800000UL) != 0UL)
  {
    raw |= 0xFF000000UL;
  }
  *code = (int32_t)raw;

  /*
   * 根据数据手册第 71 页建议，在复用 DOUT/DRDY 轮询时，释放 CS 前通过
   * RREG 产生 SCLK，使 DOUT/DRDY 恢复高电平。四路负输入均为奇数，
   * 因而读回 INPMUX 后最后移出的最低位为 1。
   */
  status = ADS124S08_ReadRegisters(device, ADS124S08_REG_INPMUX, &inpmux, 1U);
  if (status != ADS124S08_OK)
  {
    return status;
  }
  /* +FS=0x7FFFFF、-FS=0x800000 均视为饱和，不参与温度计算。 */
  if (((*code) == 0x007FFFFF) || ((*code) == (int32_t)0xFF800000UL))
  {
    return ADS124S08_ERROR_SATURATED;
  }
  return ADS124S08_OK;
}
