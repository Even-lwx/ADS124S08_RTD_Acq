/**
 * @file pt1000_app.h
 * @brief 四路 PT1000 轮询采集、数据快照和串口组帧接口。
 */
#ifndef PT1000_APP_H
#define PT1000_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ads124s08.h"
#include "pt1000.h"

#define PT1000_APP_CHANNEL_COUNT 4U

typedef struct
{
  /** 板级参考电阻、PT1000 标称电阻和各通道线性校准参数。 */
  float reference_resistance_ohm;
  float nominal_resistance_ohm;
  PT1000_Calibration channel_calibration[PT1000_APP_CHANNEL_COUNT];
} PT1000_AppConfig;

typedef struct
{
  /** 最近一轮采样快照；valid 为 0 时对应温度不可用于控制。 */
  int32_t raw[PT1000_APP_CHANNEL_COUNT];
  float temperature_c[PT1000_APP_CHANNEL_COUNT];
  uint8_t valid[PT1000_APP_CHANNEL_COUNT];
  uint32_t sequence;
  uint32_t tick_ms;
} PT1000_AppSnapshot;

typedef struct
{
  /** 应用实例，包含 ADC 驱动、串口、配置和一秒调度状态。 */
  ADS124S08_HandleTypeDef adc;
  UART_HandleTypeDef *output_uart;
  PT1000_AppConfig config;
  PT1000_AppSnapshot snapshot;
  uint32_t next_sample_tick;
  uint8_t adc_ready;
} PT1000_App;

/** @brief 载入 RREF=3 kΩ、R0=1 kΩ、各通道增益 1/偏移 0。 */
void PT1000_AppGetDefaultConfig(PT1000_AppConfig *config);
/** @brief 绑定硬件资源并初始化 ADS124S08。 */
void PT1000_AppInit(PT1000_App *app,
                    SPI_HandleTypeDef *spi,
                    UART_HandleTypeDef *output_uart,
                    GPIO_TypeDef *cs_port,
                    uint16_t cs_pin,
                    GPIO_TypeDef *dout_port,
                    uint16_t dout_pin,
                    const PT1000_AppConfig *config);
/** @brief 非阻塞周期入口；到期后采四路并从 USART2 输出固定八字段帧。 */
void PT1000_AppTask(PT1000_App *app);
/** @brief 复制最近一轮采样快照，供温控任务使用。 */
void PT1000_AppGetSnapshot(const PT1000_App *app,
                           PT1000_AppSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif
