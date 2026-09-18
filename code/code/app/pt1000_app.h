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

/** PCB 上固定接入的 PT1000 通道数量。 */
#define PT1000_APP_CHANNEL_COUNT 4U

/** @brief 四路测温应用的可调参数。 */
typedef struct
{
  float reference_resistance_ohm; /**< 板上 R17 的实测阻值，单位 Ω。 */
  float nominal_resistance_ohm;   /**< PT1000 在 0 ℃时的标称阻值 R0，单位 Ω。 */
  /** 各通道独立的电阻增益和偏移校准参数。 */
  PT1000_Calibration channel_calibration[PT1000_APP_CHANNEL_COUNT];
} PT1000_AppConfig;

/** @brief 一轮四通道采样完成后对外发布的数据快照。 */
typedef struct
{
  int32_t raw[PT1000_APP_CHANNEL_COUNT]; /**< 各通道符号扩展后的 24 位原始码。 */
  float temperature_c[PT1000_APP_CHANNEL_COUNT]; /**< 各通道换算温度，单位 ℃。 */
  uint8_t valid[PT1000_APP_CHANNEL_COUNT]; /**< 1=温度有效，0=采样或范围检查失败。 */
  uint32_t sequence; /**< 完成轮次计数；每发布一帧递增，用于检测新数据。 */
  uint32_t tick_ms;  /**< 本轮开始时的 HAL 毫秒时间戳。 */
} PT1000_AppSnapshot;

/** @brief 四路采集应用实例，保存硬件绑定、配置和调度状态。 */
typedef struct
{
  ADS124S08_HandleTypeDef adc; /**< 底层 ADS124S08 驱动实例。 */
  UART_HandleTypeDef *output_uart; /**< 正式数据输出串口，本工程绑定 USART2。 */
  PT1000_AppConfig config; /**< 初始化时复制并长期使用的测温配置。 */
  PT1000_AppSnapshot snapshot; /**< 最近一次完成的四通道数据。 */
  uint32_t next_sample_tick; /**< 下一轮采样的绝对毫秒时刻。 */
  uint8_t adc_ready; /**< 1=ADC 已初始化，0=下轮需尝试重新初始化。 */
  ADS124S08_Status last_init_status; /**< 最近一次 ADS124S08_Init() 返回值。 */
  ADS124S08_Status channel_status[PT1000_APP_CHANNEL_COUNT]; /**< 本轮各通道最终状态。 */
  /** 各通道转换结束后立即读取的 ADS124S08 STATUS，0xFF 表示读取失败。 */
  uint8_t channel_adc_status[PT1000_APP_CHANNEL_COUNT];
  /** IDAC1 直通 AIN1/REFP0 回流路径自检后的 STATUS，0xFF 表示自检失败。 */
  uint8_t reference_test_status;
  /** 使用内部 2.5 V 参考测量 AVDD/4 的驱动状态。 */
  ADS124S08_Status analog_supply_test_status;
  /** AVDD/4 系统监测的原始码；3.3 V AVDD 时应约为 2768241。 */
  int32_t analog_supply_test_code;
  /** 使用内部参考测量 CH1 与 IDAC1 的驱动状态。 */
  ADS124S08_Status idac_test_status;
  /** CH1 内部参考自检原始码；跨接 1 kΩ 时应约为 838861。 */
  int32_t idac_test_code;
} PT1000_App;

/**
 * @brief 载入默认的电阻和校准参数。
 * @param config 待填写的配置结构体。
 * @note 默认 RREF=3 kΩ、R0=1 kΩ，各通道电阻增益为 1、偏移为 0。
 */
void PT1000_AppGetDefaultConfig(PT1000_AppConfig *config);
/**
 * @brief 初始化四路测温应用并绑定底层硬件。
 * @param app 应用实例。
 * @param spi ADS124S08 使用的 HAL SPI 句柄。
 * @param output_uart 固定数据帧输出使用的 HAL UART 句柄。
 * @param cs_port ADS124S08 CS# 端口。
 * @param cs_pin ADS124S08 CS# 引脚。
 * @param dout_port ADS124S08 DOUT/DRDY 端口。
 * @param dout_pin ADS124S08 DOUT/DRDY 引脚。
 * @param config 用户配置；函数内部按值复制，调用后原结构可继续修改。
 */
void PT1000_AppInit(PT1000_App *app,
                    SPI_HandleTypeDef *spi,
                    UART_HandleTypeDef *output_uart,
                    GPIO_TypeDef *cs_port,
                    uint16_t cs_pin,
                    GPIO_TypeDef *dout_port,
                    uint16_t dout_pin,
                    const PT1000_AppConfig *config);
/**
 * @brief 执行一秒周期调度、四路采样、换算、快照发布和串口发送。
 * @param app 已初始化的应用实例。
 * @note 周期未到时立即返回；到期后的四次 ADC 转换和 UART 发送为阻塞操作。
 */
void PT1000_AppTask(PT1000_App *app);
/**
 * @brief 复制最近一轮采样快照，供 PID 温控任务读取。
 * @param app 应用实例。
 * @param snapshot 接收完整快照的目标结构体。
 */
void PT1000_AppGetSnapshot(const PT1000_App *app,
                           PT1000_AppSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif
