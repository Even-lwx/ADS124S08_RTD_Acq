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
  float reference_resistance_ohm;
  float nominal_resistance_ohm;
  PT1000_Calibration channel_calibration[PT1000_APP_CHANNEL_COUNT];
} PT1000_AppConfig;

typedef struct
{
  ADS124S08_HandleTypeDef adc;
  UART_HandleTypeDef *output_uart;
  PT1000_AppConfig config;
  uint32_t next_sample_tick;
  uint8_t adc_ready;
} PT1000_App;

void PT1000_AppGetDefaultConfig(PT1000_AppConfig *config);
void PT1000_AppInit(PT1000_App *app,
                    SPI_HandleTypeDef *spi,
                    UART_HandleTypeDef *output_uart,
                    GPIO_TypeDef *cs_port,
                    uint16_t cs_pin,
                    GPIO_TypeDef *dout_port,
                    uint16_t dout_pin,
                    const PT1000_AppConfig *config);
void PT1000_AppTask(PT1000_App *app);

#ifdef __cplusplus
}
#endif

#endif
