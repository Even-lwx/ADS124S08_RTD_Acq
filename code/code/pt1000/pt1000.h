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

/** @brief 单通道电阻线性标定参数。 */
typedef struct
{
  float resistance_gain; /**< 电阻乘法增益；默认 1.0。 */
  float resistance_offset_ohm; /**< 增益修正后叠加的偏移量，单位 Ω；默认 0。 */
} PT1000_Calibration;

/**
 * @brief 将 ADS124S08 有符号码转换为比例测量电阻值并应用标定。
 * @param adc_code ADS124S08 符号扩展后的 24 位转换码。
 * @param reference_resistance_ohm 比例参考电阻 R17 的实测值，单位 Ω。
 * @param pga_gain ADC PGA 增益，本工程为 1。
 * @param calibration 可选的通道线性标定；传入空指针表示不校准。
 * @return 电阻值（Ω）；参考电阻或 PGA 增益非法时返回 -1。
 */
float PT1000_CodeToResistance(int32_t adc_code,
                              float reference_resistance_ohm,
                              float pga_gain,
                              const PT1000_Calibration *calibration);
/**
 * @brief 按 IEC 60751 Callendar-Van Dusen 方程将电阻换算为温度。
 * @param resistance_ohm 已校准的 PT1000 电阻，单位 Ω。
 * @param nominal_resistance_ohm 0 ℃标称电阻 R0，本工程为 1000 Ω。
 * @param temperature_c 成功时写入换算温度，单位 ℃。
 * @return 成功返回 1，参数非法或超出 -200～850 ℃ 时返回 0。
 */
uint8_t PT1000_ResistanceToTemperature(float resistance_ohm,
                                       float nominal_resistance_ohm,
                                       float *temperature_c);

#ifdef __cplusplus
}
#endif

#endif
