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
/** 一帧调试串口阻塞发送的最大等待时间。 */
#define PT1000_APP_UART_TIMEOUT_MS       100U
/** ADS124S08 当前寄存器配置使用 PGA=1。 */
#define PT1000_APP_PGA_GAIN              1.0f
/** 本应用允许发布给温控模块的实际工作温度范围。 */
#define PT1000_APP_MIN_TEMPERATURE_C    -50.0f
#define PT1000_APP_MAX_TEMPERATURE_C     200.0f

/** 诊断帧发送超时；USART2 为 115200 8N1，帧内容允许阻塞发送。 */
#define PT1000_APP_DEBUG_UART_TIMEOUT_MS 200U

/*
 * 以下值仅用于上板诊断，不改变正式四路比例测量配置。
 * 0x3A：选择内部 2.5 V 参考，正负参考缓冲均旁路，内部参考保持常开。
 * 0x70：SYS_MON=011b 测量 AVDD/4，其余位保持正常配置的默认值。
 */
#define PT1000_APP_INTERNAL_REF_VALUE     0x3AU
#define PT1000_APP_ANALOG_MON_SYS_VALUE  0x70U
#define PT1000_APP_NORMAL_SYS_VALUE      0x10U
#define PT1000_APP_PGA_VALUE             0x08U
#define PT1000_APP_DATARATE_VALUE        0x34U
#define PT1000_APP_IDAC_250UA_VALUE      0x04U
#define PT1000_APP_IDAC_OFF_VALUE        0x00U
#define PT1000_APP_IDAC_DISCONNECTED     0xFFU
#define PT1000_APP_IDAC1_AIN11           0xFBU
#define PT1000_APP_INPMUX_AIN0_AIN1      0x01U
#define PT1000_APP_INTERNAL_REF_WAIT_MS  10U

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
  /* 原理图固定映射：差分输入 AIN0/1、2/3、4/5、6/7；IDAC1 依次到 11～8。 */
  /* 原理图采用四线制：AIN11/10/9/8 仅作为 IDAC1 激励输出脚，
   * ADC 电压采样脚仍为 AIN0/1、AIN2/3、AIN4/5、AIN6/7。 */
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

/** 将 8 位数值追加为两个十六进制字符。 */
static char *PT1000_AppendHex8(char *destination, uint8_t value)
{
  static const char hex[] = "0123456789ABCDEF";
  *destination++ = hex[(value >> 4) & 0x0FU];
  *destination++ = hex[value & 0x0FU];
  return destination;
}

/** 将驱动状态码追加为无符号十进制文本。 */
static char *PT1000_AppendStatus(char *destination, ADS124S08_Status status)
{
  return PT1000_AppendUnsigned(destination, (uint32_t)status);
}

/**
 * @brief 让 IDAC1 绕过 PT1000，直接经 AIN1 和 R17 建立比例参考电压。
 * @param app 应用实例。
 * @return 本次自检转换锁存的 ADS124S08 STATUS；读取失败返回 0xFF。
 *
 * 原理图中 AIN1 经 100 Ω 接到 U1 回流侧，该节点同时连接 REFP0，REFP0
 * 再经 R17=3 kΩ 接 AGND。把 IDAC1 临时路由到 AIN1 后，正常情况下无需
 * PT1000 或 AIN11 参与即可得到约 0.75 V 参考电压，FL_REF_L0 应清零。
 */
static uint8_t PT1000_AppTestReferencePath(PT1000_App *app)
{
  int32_t discarded_code = 0;
  uint8_t adc_status = 0xFFU;
  ADS124S08_Status status;

  status = ADS124S08_ConfigureChannel(&app->adc, 0U, 1U, 1U);
  if (status == ADS124S08_OK)
  {
    /* 只关心本次转换锁存的参考监测位，转换码和饱和状态均不参与判断。 */
    status = ADS124S08_ReadSingle(&app->adc, &discarded_code,
                                  PT1000_APP_CONVERSION_TIMEOUT_MS);
    if (((status != ADS124S08_OK) &&
         (status != ADS124S08_ERROR_SATURATED)) ||
        (ADS124S08_ReadRegisters(&app->adc, ADS124S08_REG_STATUS,
                                 &adc_status, 1U) != ADS124S08_OK))
    {
      adc_status = 0xFFU;
    }
  }
  return adc_status;
}

/**
 * @brief 写入一组临时诊断寄存器，并逐字节读回校验。
 * @param app 应用实例。
 * @param registers 从 INPMUX(02h) 到 SYS(09h) 的 8 字节配置。
 * @return SPI 写入、读取或读回比较结果。
 *
 * 诊断配置覆盖输入、参考、IDAC 和系统监测寄存器。调用者完成转换后必须重新
 * 调用 ADS124S08_ConfigureChannel()，恢复正式 PT1000 比例测量配置。
 */
static ADS124S08_Status PT1000_AppWriteDiagnosticRegisters(
    PT1000_App *app, const uint8_t registers[8])
{
  uint8_t readback[8];
  uint8_t index;
  ADS124S08_Status status;

  status = ADS124S08_WriteRegisters(&app->adc, ADS124S08_REG_INPMUX,
                                     registers, 8U);
  if (status != ADS124S08_OK)
  {
    return status;
  }

  status = ADS124S08_ReadRegisters(&app->adc, ADS124S08_REG_INPMUX,
                                    readback, 8U);
  if (status != ADS124S08_OK)
  {
    return status;
  }

  for (index = 0U; index < 8U; index++)
  {
    if (readback[index] != registers[index])
    {
      return ADS124S08_ERROR_VERIFY;
    }
  }
  return ADS124S08_OK;
}

/**
 * @brief 使用内部 2.5 V 参考测量 AVDD/4，检查 ADS124S08 模拟核心供电。
 * @param app 应用实例。
 * @param code 返回有符号 24 位原始码。
 * @return 配置或转换状态；3.3 V AVDD 正常时原始码应约为 2768241。
 *
 * SYS_MON=011b 会在芯片内部把 AVDD/4 接到 ADC，因此该测试不依赖外部
 * PT1000、R17 或 REFP0。若本测试也满量程或失败，应优先检查 AVDD、AVSS、
 * REFOUT/REFCOM 的 1 uF 电容及 ADS124S08 焊接。
 */
static ADS124S08_Status PT1000_AppTestAnalogSupply(PT1000_App *app,
                                                    int32_t *code)
{
  const uint8_t registers[8] =
  {
    PT1000_APP_INPMUX_AIN0_AIN1,
    PT1000_APP_PGA_VALUE,
    PT1000_APP_DATARATE_VALUE,
    PT1000_APP_INTERNAL_REF_VALUE,
    PT1000_APP_IDAC_OFF_VALUE,
    PT1000_APP_IDAC_DISCONNECTED,
    0x00U,
    PT1000_APP_ANALOG_MON_SYS_VALUE
  };
  ADS124S08_Status status;

  status = PT1000_AppWriteDiagnosticRegisters(app, registers);
  if (status != ADS124S08_OK)
  {
    return status;
  }

  /* 切换到内部参考后留出大于数据手册 5.9 ms 典型建立时间的裕量。 */
  HAL_Delay(PT1000_APP_INTERNAL_REF_WAIT_MS);
  return ADS124S08_ReadSingle(&app->adc, code,
                              PT1000_APP_CONVERSION_TIMEOUT_MS);
}

/**
 * @brief 使用内部参考测量 CH1 的 1 kΩ 测试电阻压降，单独检查 IDAC 回路。
 * @param app 应用实例。
 * @param code 返回 AIN0-AIN1 的有符号 24 位原始码。
 * @return 配置或转换状态；250 uA 流过 1 kΩ 时原始码应约为 838861。
 *
 * 该测试仍按原理图把 IDAC1 路由到 AIN11，但 ADC 改用内部 2.5 V 参考，
 * 因而不使用 R17 产生的外部参考电压。若 AV 测试正常且本测试接近 838861，
 * 说明 IDAC、AIN11、U1、AIN0/AIN1 以及经 R17 到地的电流回路均可工作；此时
 * 若 RT 仍为 S81，应重点检查 REFP0 芯片引脚焊接/走线和 REFN0 接地。
 */
static ADS124S08_Status PT1000_AppTestIdacPath(PT1000_App *app,
                                               int32_t *code)
{
  const uint8_t registers[8] =
  {
    PT1000_APP_INPMUX_AIN0_AIN1,
    PT1000_APP_PGA_VALUE,
    PT1000_APP_DATARATE_VALUE,
    PT1000_APP_INTERNAL_REF_VALUE,
    PT1000_APP_IDAC_250UA_VALUE,
    PT1000_APP_IDAC1_AIN11,
    0x00U,
    PT1000_APP_NORMAL_SYS_VALUE
  };
  ADS124S08_Status status;

  status = PT1000_AppWriteDiagnosticRegisters(app, registers);
  if (status != ADS124S08_OK)
  {
    return status;
  }

  HAL_Delay(PT1000_APP_INTERNAL_REF_WAIT_MS);
  return ADS124S08_ReadSingle(&app->adc, code,
                              PT1000_APP_CONVERSION_TIMEOUT_MS);
}

/**
 * @brief 读取并输出 ADS124S08 诊断信息到 USART2。
 * @note 诊断帧与正式温度帧均从 USART2/H1 输出；USART1/RS485 不使用。
 */
static void PT1000_AppSendDebug(PT1000_App *app)
{
  uint8_t id = 0U;
  uint8_t status_reg = 0U;
  uint8_t regs[6] = {0U, 0U, 0U, 0U, 0U, 0U};
  ADS124S08_Status read_status;
  /*
   * 调试帧缓冲区使用静态存储，避免与 PT1000_AppTask() 的正式帧缓冲区
   * 同时占用 Cortex-M0+ 主栈。应用为单线程轮询，不存在重入访问。
   */
  static char frame[256];
  char *cursor = frame;
  uint8_t channel;

  if ((app == NULL) || (app->output_uart == NULL))
  {
    return;
  }

  *cursor++ = 'D'; *cursor++ = 'B'; *cursor++ = 'G'; *cursor++ = ',';
  *cursor++ = 'I'; *cursor++ = 'N'; *cursor++ = 'I'; *cursor++ = 'T'; *cursor++ = '=';
  cursor = PT1000_AppendStatus(cursor, app->last_init_status);
  *cursor++ = ',';

  /* 即使初始化失败也尝试读 ID 和寄存器，便于区分 SPI 断线与配置校验失败。 */
  read_status = ADS124S08_ReadRegisters(&app->adc, ADS124S08_REG_ID, &id, 1U);
  *cursor++ = 'I'; *cursor++ = 'D'; *cursor++ = '=';
  if (read_status == ADS124S08_OK) cursor = PT1000_AppendHex8(cursor, id);
  else { *cursor++ = 'X'; *cursor++ = 'X'; }
  *cursor++ = ',';

  read_status = ADS124S08_ReadRegisters(&app->adc, ADS124S08_REG_STATUS,
                                         &status_reg, 1U);
  *cursor++ = 'S'; *cursor++ = 'T'; *cursor++ = '=';
  if (read_status == ADS124S08_OK) cursor = PT1000_AppendHex8(cursor, status_reg);
  else { *cursor++ = 'X'; *cursor++ = 'X'; }
  *cursor++ = ',';

  read_status = ADS124S08_ReadRegisters(&app->adc, ADS124S08_REG_INPMUX,
                                         regs, sizeof(regs));
  *cursor++ = 'R'; *cursor++ = 'E'; *cursor++ = 'G'; *cursor++ = '=';
  if (read_status == ADS124S08_OK)
  {
    for (channel = 0U; channel < sizeof(regs); channel++)
    {
      cursor = PT1000_AppendHex8(cursor, regs[channel]);
      if (channel != (sizeof(regs) - 1U)) *cursor++ = ':';
    }
  }
  else { *cursor++ = 'X'; *cursor++ = 'X'; }

  *cursor++ = ','; *cursor++ = 'R'; *cursor++ = 'T'; *cursor++ = '=';
  *cursor++ = 'S';
  cursor = PT1000_AppendHex8(cursor, app->reference_test_status);

  *cursor++ = ','; *cursor++ = 'A'; *cursor++ = 'V'; *cursor++ = '=';
  cursor = PT1000_AppendStatus(cursor, app->analog_supply_test_status);
  *cursor++ = ':';
  cursor = PT1000_AppendSigned(cursor, app->analog_supply_test_code);

  *cursor++ = ','; *cursor++ = 'I'; *cursor++ = 'T'; *cursor++ = '=';
  cursor = PT1000_AppendStatus(cursor, app->idac_test_status);
  *cursor++ = ':';
  cursor = PT1000_AppendSigned(cursor, app->idac_test_code);

  for (channel = 0U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
  {
    *cursor++ = ','; *cursor++ = 'C';
    *cursor++ = (char)('1' + channel); *cursor++ = '=';
    cursor = PT1000_AppendStatus(cursor, app->channel_status[channel]);
    *cursor++ = ':';
    cursor = PT1000_AppendSigned(cursor, app->snapshot.raw[channel]);
    *cursor++ = ':'; *cursor++ = 'S';
    cursor = PT1000_AppendHex8(cursor, app->channel_adc_status[channel]);
  }
  *cursor++ = ','; *cursor++ = 'B'; *cursor++ = '=';
  cursor = PT1000_AppendHex8(cursor, app->adc.last_data[0]);
  *cursor++ = ':';
  cursor = PT1000_AppendHex8(cursor, app->adc.last_data[1]);
  *cursor++ = ':';
  cursor = PT1000_AppendHex8(cursor, app->adc.last_data[2]);
  *cursor++ = '\r';
  *cursor++ = '\n';
  (void)HAL_UART_Transmit(app->output_uart, (uint8_t *)frame,
                          (uint16_t)(cursor - frame),
                          PT1000_APP_DEBUG_UART_TIMEOUT_MS);
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
  app->last_init_status = ADS124S08_Init(&app->adc);
  app->adc_ready = (app->last_init_status == ADS124S08_OK) ? 1U : 0U;
  app->next_sample_tick = HAL_GetTick();
}

void PT1000_AppTask(PT1000_App *app)
{
  int32_t raw[PT1000_APP_CHANNEL_COUNT] = {0, 0, 0, 0};
  float temperature[PT1000_APP_CHANNEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint8_t valid[PT1000_APP_CHANNEL_COUNT] = {0U, 0U, 0U, 0U};
  /* 正式帧包含 8 个字段和 CRLF；保留余量，避免后续诊断字段污染正式帧。 */
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
    app->last_init_status = ADS124S08_Init(&app->adc);
    app->adc_ready = (app->last_init_status == ADS124S08_OK) ? 1U : 0U;
  }

  if (app->adc_ready != 0U)
  {
    /* 先执行一次不经过 PT1000/AIN11 的 IDAC 与参考电阻回路自检。 */
    app->reference_test_status = PT1000_AppTestReferencePath(app);

    /*
     * 再用内部参考分别检查模拟电源与 CH1 IDAC 路径。两项测试完成后，下面
     * 的 ADS124S08_ConfigureChannel() 会恢复外部比例参考和正常系统配置。
     */
    app->analog_supply_test_code = 0;
    app->analog_supply_test_status = PT1000_AppTestAnalogSupply(
        app, &app->analog_supply_test_code);
    app->idac_test_code = 0;
    app->idac_test_status = PT1000_AppTestIdacPath(
        app, &app->idac_test_code);

    /* 每路均先切换差分输入和 IDAC，再启动一次新的单次转换。 */
    for (channel = 0U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
    {
      uint8_t adc_status = 0xFFU;
      ADS124S08_Status status = ADS124S08_ConfigureChannel(
          &app->adc, channels[channel].positive_input,
          channels[channel].negative_input, channels[channel].idac_output);
      if (status == ADS124S08_OK)
      {
        status = ADS124S08_ReadSingle(&app->adc, &raw[channel],
                                      PT1000_APP_CONVERSION_TIMEOUT_MS);
      }
      app->channel_status[channel] = status;

      /*
       * 顶层 DBG 中的 ST 是整轮结束后读取的，只能反映最后一个通道。
       * 因此每路转换后立即保存 STATUS，用于判断该路 IDAC/参考回路是否欠压。
       * STATUS 读取失败不覆盖原转换状态，仅以 0xFF 标记诊断字段无效。
       */
      if (ADS124S08_ReadRegisters(&app->adc, ADS124S08_REG_STATUS,
                                  &adc_status, 1U) == ADS124S08_OK)
      {
        app->channel_adc_status[channel] = adc_status;
      }
      else
      {
        app->channel_adc_status[channel] = 0xFFU;
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

  if (app->adc_ready == 0U)
  {
    app->reference_test_status = 0xFFU;
    app->analog_supply_test_status = app->last_init_status;
    app->analog_supply_test_code = 0;
    app->idac_test_status = app->last_init_status;
    app->idac_test_code = 0;
    for (channel = 0U; channel < PT1000_APP_CHANNEL_COUNT; channel++)
    {
      app->channel_status[channel] = app->last_init_status;
      app->channel_adc_status[channel] = 0xFFU;
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
  PT1000_AppSendDebug(app);
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
