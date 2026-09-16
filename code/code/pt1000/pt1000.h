/**
 * @file pt1000.h
 * @brief PT1000 电阻与温度换算、通道标定接口。
 */
#ifndef PT1000_H
#define PT1000_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  /** 电阻线性标定参数：Rcal = Rraw * gain + offset。 */
  float resistance_gain;
  float resistance_offset_ohm;
} PT1000_Calibration;

/**
 * @brief 将 ADS124S08 有符号码转换为比例测量电阻值并应用标定。
 * @return 电阻值（Ω）；参考电阻或 PGA 增益非法时返回 -1。
 */
float PT1000_CodeToResistance(int32_t adc_code,
                              float reference_resistance_ohm,
                              float pga_gain,
                              const PT1000_Calibration *calibration);
/**
 * @brief 按 IEC 60751 Callendar-Van Dusen 方程将电阻换算为温度。
 * @return 成功返回 1，参数非法或超出 -200～850 ℃ 时返回 0。
 */
uint8_t PT1000_ResistanceToTemperature(float resistance_ohm,
                                       float nominal_resistance_ohm,
                                       float *temperature_c);

#ifdef __cplusplus
}
#endif

#endif
