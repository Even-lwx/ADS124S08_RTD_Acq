/**
 * @file pt1000_app.c
 * @brief 四路 PT1000 顺序采样、有效性检查和 USART2 数据输出实现。
 */
#include "pt1000_app.h"

#include <string.h>

/** 一轮采样的目标周期为 1 秒。 */
#define PT1000_APP_PERIOD_MS             1000U
/** 20 SPS 单次转换的理论时间约 50 ms，150 ms 为异常超时裕量。 */
#define PT1000_APP_CONVERSION_TIMEOUT_MS 150U
/** 一帧正式数据阻塞发送的最大等待时间。 */
#define PT1000_APP_UART_TIMEOUT_MS       100U
/** ADS124S08 当前寄存器配置使用 PGA=1。 */
#define PT1000_APP_PGA_GAIN              1.0f
/** 本应用允许发布给温控模块的实际工作温度范围。 */
#define PT1000_APP_MIN_TEMPERATURE_C    -50.0f
#define PT1000_APP_MAX_TEMPERATURE_C     200.0f

/** @brief 单个 PCB 测温通道的 ADC 差分输入和 IDAC1 路由。 */
typedef struct
{
  uint8_t positive_input; /**< ADC 正输入 AIN 编号。 */
  uint8_t negative_input; /**< ADC 负输入 AIN 编号。 */
  uint8_t idac_output;    /**< 250 uA IDAC1 输出 AIN 编号。 */
} PT1000_Channel;

/** 四路固定硬件映射，数组下标 0～3 对应输出帧中的通道 1～4。 */
static const PT1000_Channel channels[PT1000_APP_CHANNEL_COUNT] =
{
  /*
   * 原理图中，AIN11/10/9/8 的 IDAC 激励线与对应 ADC 采样线最终在
   * PT1000 焊盘处汇合；电压仍由 AIN0/1、AIN2/3、AIN4/5、AIN6/7
   * 四组差分输入测量。下表仅保存芯片引脚编号，不代表数组索引。
   */
  {0U, 1U, 11U},
  {2U, 3U, 10U},
  {4U, 5U,  9U},
  {6U, 7U,  8U}
};

/**
 * @brief 将无符号整数以十进制追加到字符缓冲区。
 * @return 指向最后一个已写字符之后的位置，便于连续组帧。
 * @note 调用者负责保证缓冲区空间足够，本函数不会写字符串结束符。
 */
static char *PT1000_AppendUnsigned(char *destination, uint32_t value)
{
  /* 避免引入 printf 浮点库，减小 Cortex-M0+ 固件体积。 */
  char reverse[10];
  uint8_t count = 0U;

  do
  {
    reverse[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  } while (value != 0U);

  while (count > 0U)
  {
    *destination++ = reverse[--count];
  }
  return destination;
}

/**
 * @brief 将有符号 ADC 码以十进制追加到字符缓冲区。
 * @note 使用 -(value+1)+1 的写法安全处理 INT32_MIN，避免有符号溢出。
 */
static char *PT1000_AppendSigned(char *destination, int32_t value)
{
  uint32_t magnitude;
  if (value < 0)
  {
    *destination++ = '-';
    magnitude = (uint32_t)(-(value + 1)) + 1U;
  }
  else
  {
    magnitude = (uint32_t)value;
  }
  return PT1000_AppendUnsigned(destination, magnitude);
}

/**
 * @brief 将温度格式化为两位小数，或在无效时写入固定文本 nan。
 * @return 新的缓冲区写指针；不写字符串结束符。
 */
static char *PT1000_AppendTemperature(char *destination,
                                      float temperature_c,
                                      uint8_t valid)
{
  int32_t scaled;
  uint32_t magnitude;

  if (valid == 0U)
  {
    memcpy(destination, "nan", 3U);
    return destination + 3;
  }

  /* 先四舍五入到 0.01 ℃整数，随后手工插入小数点。 */
  scaled = (int32_t)(temperature_c * 100.0f +
                     ((temperature_c >= 0.0f) ? 0.5f : -0.5f));
  if (scaled < 0)
  {
    *destination++ = '-';
    magnitude = (uint32_t)(-(scaled + 1)) + 1U;
  }
  else
  {
    magnitude = (uint32_t)scaled;
  }

  destination = PT1000_AppendUnsigned(destination, magnitude / 100U);
  *destination++ = '.';
  *destination++ = (char)('0' + ((magnitude / 10U) % 10U));
  *destination++ = (char)('0' + (magnitude % 10U));
  return destination;
}

void PT1000_AppGetDefaultConfig(PT1000_AppConfig *config)
{
  uint8_t channel;
  if (config == NULL)
  {
    return;
  }
  config->reference_resistance_ohm = 3000.0f;
  config->nominal_resistance_ohm = 1000.0f;
  for (channel = 0U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
  {
    config->channel_calibration[channel].resistance_gain = 1.0f;
    config->channel_calibration[channel].resistance_offset_ohm = 0.0f;
  }
}

void PT1000_AppInit(PT1000_App *app,
                    SPI_HandleTypeDef *spi,
                    UART_HandleTypeDef *output_uart,
                    GPIO_TypeDef *cs_port,
                    uint16_t cs_pin,
                    GPIO_TypeDef *dout_port,
                    uint16_t dout_pin,
                    const PT1000_AppConfig *config)
{
  if ((app == NULL) || (config == NULL))
  {
    return;
  }

  memset(app, 0, sizeof(*app));
  app->adc.spi = spi;
  app->adc.cs_port = cs_port;
  app->adc.cs_pin = cs_pin;
  app->adc.dout_port = dout_port;
  app->adc.dout_pin = dout_pin;
  /* 20 ms 仅约束单次 SPI 事务，不等同于 ADC 转换等待时间。 */
  app->adc.spi_timeout_ms = 20U;
  app->output_uart = output_uart;
  app->config = *config;
  app->adc_ready = (ADS124S08_Init(&app->adc) == ADS124S08_OK) ? 1U : 0U;
  app->next_sample_tick = HAL_GetTick();
}

void PT1000_AppTask(PT1000_App *app)
{
  int32_t raw[PT1000_APP_CHANNEL_COUNT] = {0, 0, 0, 0};
  float temperature[PT1000_APP_CHANNEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint8_t valid[PT1000_APP_CHANNEL_COUNT] = {0U, 0U, 0U, 0U};
  /* 正式帧包含 8 个字段和 CRLF，128 字节可覆盖全部 24 位码值和温度文本。 */
  /* 单线程应用使用静态缓冲区，给 HAL SPI/UART 调用链保留更多栈空间。 */
  static char frame[128];
  char *cursor = frame;
  uint8_t channel;
  uint32_t now;

  if ((app == NULL) || (app->output_uart == NULL))
  {
    return;
  }

  /* 使用有符号差值比较，可正确跨越 HAL_GetTick() 的 32 位回绕。 */
  now = HAL_GetTick();
  if ((int32_t)(now - app->next_sample_tick) < 0)
  {
    return;
  }
  /* 在原计划时间上递增，避免把约 200 ms 采样耗时累加进周期。 */
  app->next_sample_tick += PT1000_APP_PERIOD_MS;
  if ((int32_t)(now - app->next_sample_tick) >= 0)
  {
    app->next_sample_tick = now + PT1000_APP_PERIOD_MS;
  }

  if (app->adc_ready == 0U)
  {
    /* 通信故障后每轮尝试重新初始化，输出结构仍保持固定。 */
    app->adc_ready = (ADS124S08_Init(&app->adc) == ADS124S08_OK) ? 1U : 0U;
  }

  if (app->adc_ready != 0U)
  {
    /* 每路均先切换差分输入和 IDAC，再启动一次新的单次转换。 */
    for (channel = 0U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
    {
      ADS124S08_Status status = ADS124S08_ConfigureChannel(
          &app->adc, channels[channel].positive_input,
          channels[channel].negative_input, channels[channel].idac_output);
      if (status == ADS124S08_OK)
      {
        status = ADS124S08_ReadSingle(&app->adc, &raw[channel],
                                      PT1000_APP_CONVERSION_TIMEOUT_MS);
      }
      if (status == ADS124S08_OK)
      {
        float resistance = PT1000_CodeToResistance(
            raw[channel], app->config.reference_resistance_ohm,
            PT1000_APP_PGA_GAIN, &app->config.channel_calibration[channel]);
        valid[channel] = PT1000_ResistanceToTemperature(
            resistance, app->config.nominal_resistance_ohm,
            &temperature[channel]);
        if ((temperature[channel] < PT1000_APP_MIN_TEMPERATURE_C) ||
            (temperature[channel] > PT1000_APP_MAX_TEMPERATURE_C))
        {
          valid[channel] = 0U;
        }
      }
      /* SPI、ID 或寄存器校验错误说明器件状态不可继续信任，下轮重新初始化。 */
      else if ((status == ADS124S08_ERROR_SPI) ||
               (status == ADS124S08_ERROR_ID) ||
               (status == ADS124S08_ERROR_VERIFY))
      {
        app->adc_ready = 0U;
        break;
      }
    }
  }

  /* 固定正式帧：raw1,temp1,...,raw4,temp4\r\n；异常温度写为 nan。 */
  for (channel = 0U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
  {
    cursor = PT1000_AppendSigned(cursor, raw[channel]);
    *cursor++ = ',';
    cursor = PT1000_AppendTemperature(cursor, temperature[channel],
                                      valid[channel]);
    if (channel != (PT1000_APP_CHANNEL_COUNT - 1U))
    {
      *cursor++ = ',';
    }

    app->snapshot.raw[channel] = raw[channel];
    app->snapshot.temperature_c[channel] = temperature[channel];
    app->snapshot.valid[channel] = valid[channel];
  }
  *cursor++ = '\r';
  *cursor++ = '\n';

  /* 先完整更新快照内容，最后递增 sequence，供控制任务识别新一轮数据。 */
  app->snapshot.tick_ms = now;
  app->snapshot.sequence++;

  (void)HAL_UART_Transmit(app->output_uart, (uint8_t *)frame,
                          (uint16_t)(cursor - frame),
                          PT1000_APP_UART_TIMEOUT_MS);
}

void PT1000_AppGetSnapshot(const PT1000_App *app,
                           PT1000_AppSnapshot *snapshot)
{
  if ((app == NULL) || (snapshot == NULL))
  {
    return;
  }
  *snapshot = app->snapshot;
}
