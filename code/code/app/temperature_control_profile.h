/**
 * @file temperature_control_profile.h
 * @brief 根据输入电压选择温控器的前馈参数。
 */
#ifndef TEMPERATURE_CONTROL_PROFILE_H
#define TEMPERATURE_CONTROL_PROFILE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "temperature_control.h"

/**
 * @brief 按实际输入电压修正温控配置中的前馈初值和安全输出上限。
 * @param config 待修改的温控配置。
 * @param input_voltage_v 开关电源 VIN 电压，单位 V。
 * @note Kp/Ki/Kd 和参考轨迹不随电压查表，避免对单组实验环境过拟合。
 *       Ki 只在目标附近用于在线学习实际维持功率。
 */
void TemperatureControl_ApplyVoltageProfile(
    TemperatureControl_Config *config,
    float input_voltage_v);

#ifdef __cplusplus
}
#endif

#endif /* TEMPERATURE_CONTROL_PROFILE_H */
