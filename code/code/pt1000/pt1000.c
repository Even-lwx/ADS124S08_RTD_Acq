#include "pt1000.h"

#include <math.h>

#define PT1000_CVD_A       3.9083e-3f
#define PT1000_CVD_B      -5.775e-7f
#define PT1000_CVD_C      -4.183e-12f
#define PT1000_MIN_TEMP_C -200.0f
#define PT1000_MAX_TEMP_C  850.0f
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

  resistance = ((float)adc_code * reference_resistance_ohm) /
               (PT1000_ADC_SCALE * pga_gain);
  if (calibration != 0)
  {
    resistance = resistance * calibration->resistance_gain +
                 calibration->resistance_offset_ohm;
  }
  return resistance;
}

static float PT1000_ResistanceAtTemperature(float nominal_resistance_ohm,
                                             float temperature_c)
{
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
    temperature = (ratio - 1.0f) / PT1000_CVD_A;
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
