/**
 * @file ads124s08.c
 * @brief ADS124S08 的 SPI 命令、寄存器配置及单次转换实现。
 *
 * SPI 使用 Mode 1（CPOL=0、CPHA=1），CS 由 PA5 软件控制。板上未连接
 * 独立 DRDY#，因此转换期间在 CS 拉低时读取 DOUT/DRDY 电平。
 */
#include "ads124s08.h"

#define ADS124S08_CMD_RREG        0x20U
#define ADS124S08_CMD_WREG        0x40U

#define ADS124S08_DEVICE_ID_MASK  0x07U
#define ADS124S08_DEVICE_ID_VALUE 0x00U

/*
 * 四路 PT1000 比例测量公共配置：PGA=1，单次转换，低延迟滤波 20 SPS，
 * 参考源为 REFP0/REFN0；内部参考常开供 IDAC 使用，IDAC 电流为 250 uA。
 * 各宏值按 SBAS660C 寄存器位定义组合得到。
 */
#define ADS124S08_PGA_VALUE       0x08U
#define ADS124S08_DATARATE_VALUE  0x34U
#define ADS124S08_REF_VALUE       0x12U
#define ADS124S08_IDACMAG_VALUE   0x04U
#define ADS124S08_SYS_VALUE       0x10U
#define ADS124S08_IDAC_DISCONNECT 0x0FU

static void ADS124S08_Select(const ADS124S08_HandleTypeDef *device)
{
  HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
}

static void ADS124S08_Deselect(const ADS124S08_HandleTypeDef *device)
{
  HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);
}

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

  command[0] = (uint8_t)(ADS124S08_CMD_RREG | (start_address & 0x1FU));
  command[1] = (uint8_t)(count - 1U);

  ADS124S08_Select(device);
  status = ADS124S08_Transmit(device, command, sizeof(command));
  if (status == ADS124S08_OK)
  {
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
  for (uint8_t index = 0U; index < sizeof(registers); index++)
  {
    if (readback[index] != registers[index])
    {
      return ADS124S08_ERROR_VERIFY;
    }
  }
  return ADS124S08_OK;
}

static ADS124S08_Status ADS124S08_WaitDataReady(
    ADS124S08_HandleTypeDef *device, uint32_t timeout_ms)
{
  uint32_t start_tick = HAL_GetTick();

  do
  {
    GPIO_PinState state;
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

  status = ADS124S08_SendCommand(device, ADS124S08_COMMAND_START);
  if (status != ADS124S08_OK)
  {
    return status;
  }
  status = ADS124S08_WaitDataReady(device, timeout_ms);
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
  if (((*code) == 0x007FFFFF) || ((*code) == (int32_t)0xFF800000UL))
  {
    return ADS124S08_ERROR_SATURATED;
  }
  return ADS124S08_OK;
}
