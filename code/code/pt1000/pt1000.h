#ifndef PT1000_H
#define PT1000_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  float resistance_gain;
  float resistance_offset_ohm;
} PT1000_Calibration;

float PT1000_CodeToResistance(int32_t adc_code,
                              float reference_resistance_ohm,
                              float pga_gain,
                              const PT1000_Calibration *calibration);
uint8_t PT1000_ResistanceToTemperature(float resistance_ohm,
                                       float nominal_resistance_ohm,
                                       float *temperature_c);

#ifdef __cplusplus
}
#endif

#endif
