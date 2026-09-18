/**
 * @file pt1000.c
 * @brief ADS124S08 比例码值、电阻与 IEC 60751 温度换算实现。
 */
#include "pt1000.h"

#include <math.h>

/** IEC 60751 标准 Callendar-Van Dusen 系数。 */
#define PT1000_CVD_A       3.9083e-3f
#define PT1000_CVD_B      -5.775e-7f
#define PT1000_CVD_C      -4.183e-12f
/** CVD 方程标准有效温度范围。 */
#define PT1000_MIN_TEMP_C -200.0f
#define PT1000_MAX_TEMP_C  850.0f
/** 24 位双极性 ADC 的正半量程 2^23。 */
#define PT1000_ADC_SCALE   8388608.0f

float PT1000_CodeToResistance(int32_t adc_code,
                              float reference_resistance_ohm,
                              float pga_gain,
                              const PT1000_Calibration *calibration)
{
  float resistance;

  if ((reference_resistance_ohm <= 0.0f) || (pga_gain <= 0.0f))
  {
    return -1.0f;
  }

  /* 比例测量中 IDAC 电流约去：RRTD = Code * RREF / (2^23 * PGA)。 */
  resistance = ((float)adc_code * reference_resistance_ohm) /
               (PT1000_ADC_SCALE * pga_gain);
  if (calibration != 0)
  {
    resistance = resistance * calibration->resistance_gain +
                 calibration->resistance_offset_ohm;
  }
  return resistance;
}

/**
 * @brief CVD 正向方程：根据温度计算理论电阻。
 * @param nominal_resistance_ohm 0 ℃标称电阻 R0。
 * @param temperature_c 温度，单位 ℃。
 * @return 理论电阻，单位 Ω。
 * @note 该内部函数同时用于范围检查和负温区牛顿迭代。
 */
static float PT1000_ResistanceAtTemperature(float nominal_resistance_ohm,
                                             float temperature_c)
{
  /* IEC 60751：0 ℃以上使用 A/B 项，0 ℃以下额外使用 C 项。 */
  if (temperature_c >= 0.0f)
  {
    return nominal_resistance_ohm *
           (1.0f + PT1000_CVD_A * temperature_c +
            PT1000_CVD_B * temperature_c * temperature_c);
  }
  return nominal_resistance_ohm *
         (1.0f + PT1000_CVD_A * temperature_c +
          PT1000_CVD_B * temperature_c * temperature_c +
          PT1000_CVD_C * (temperature_c - 100.0f) *
              temperature_c * temperature_c * temperature_c);
}

uint8_t PT1000_ResistanceToTemperature(float resistance_ohm,
                                       float nominal_resistance_ohm,
                                       float *temperature_c)
{
  float ratio;
  float temperature;

  if ((temperature_c == 0) || (resistance_ohm <= 0.0f) ||
      (nominal_resistance_ohm <= 0.0f))
  {
    return 0U;
  }

  /* 先把标准温度边界换算成电阻边界，拒绝无物理解的输入。 */
  if ((resistance_ohm < PT1000_ResistanceAtTemperature(
                            nominal_resistance_ohm, PT1000_MIN_TEMP_C)) ||
      (resistance_ohm > PT1000_ResistanceAtTemperature(
                            nominal_resistance_ohm, PT1000_MAX_TEMP_C)))
  {
    return 0U;
  }

  ratio = resistance_ohm / nominal_resistance_ohm;
  if (resistance_ohm >= nominal_resistance_ohm)
  {
    /* 正温区为二次方程，可直接取物理有效根。 */
    float discriminant = PT1000_CVD_A * PT1000_CVD_A -
                         4.0f * PT1000_CVD_B * (1.0f - ratio);
    if (discriminant < 0.0f)
    {
      return 0U;
    }
    temperature = (-PT1000_CVD_A + sqrtf(discriminant)) /
                  (2.0f * PT1000_CVD_B);
  }
  else
  {
    uint8_t iteration;
    /* 负温区含四次项，使用牛顿法迭代求解 CVD 反函数。 */
    temperature = (ratio - 1.0f) / PT1000_CVD_A;
    /* 固定 8 次迭代可避免不确定执行时间，并满足当前浮点精度需求。 */
    for (iteration = 0U; iteration < 8U; iteration++)
    {
      float t2 = temperature * temperature;
      float calculated = PT1000_ResistanceAtTemperature(
          nominal_resistance_ohm, temperature);
      float derivative = nominal_resistance_ohm *
          (PT1000_CVD_A + 2.0f * PT1000_CVD_B * temperature +
           PT1000_CVD_C * (4.0f * temperature * t2 - 300.0f * t2));
      if (fabsf(derivative) < 1.0e-6f)
      {
        return 0U;
      }
      /* Newton-Raphson：T(n+1)=T(n)-[R(T)-Rmeas]/R'(T)。 */
      temperature -= (calculated - resistance_ohm) / derivative;
    }
  }

  if ((temperature < PT1000_MIN_TEMP_C) ||
      (temperature > PT1000_MAX_TEMP_C))
  {
    return 0U;
  }
  *temperature_c = temperature;
  return 1U;
}
