#ifndef ADS124S08_H
#define ADS124S08_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"
#include <stdint.h>

/* ADS124S08 data sheet SBAS660C, register map on page 72. */
typedef enum
{
  ADS124S08_OK = 0,
  ADS124S08_ERROR_PARAM,
  ADS124S08_ERROR_SPI,
  ADS124S08_ERROR_ID,
  ADS124S08_ERROR_VERIFY,
  ADS124S08_ERROR_TIMEOUT,
  ADS124S08_ERROR_SATURATED
} ADS124S08_Status;

typedef enum
{
  ADS124S08_COMMAND_NOP       = 0x00,
  ADS124S08_COMMAND_WAKEUP    = 0x02,
  ADS124S08_COMMAND_POWERDOWN = 0x04,
  ADS124S08_COMMAND_RESET     = 0x06,
  ADS124S08_COMMAND_START     = 0x08,
  ADS124S08_COMMAND_STOP      = 0x0A,
  ADS124S08_COMMAND_SYOCAL    = 0x16,
  ADS124S08_COMMAND_SYGCAL    = 0x17,
  ADS124S08_COMMAND_SFOCAL    = 0x19,
  ADS124S08_COMMAND_RDATA     = 0x12
} ADS124S08_Command;

typedef enum
{
  ADS124S08_REG_ID       = 0x00,
  ADS124S08_REG_STATUS   = 0x01,
  ADS124S08_REG_INPMUX   = 0x02,
  ADS124S08_REG_PGA      = 0x03,
  ADS124S08_REG_DATARATE = 0x04,
  ADS124S08_REG_REF      = 0x05,
  ADS124S08_REG_IDACMAG  = 0x06,
  ADS124S08_REG_IDACMUX  = 0x07,
  ADS124S08_REG_VBIAS    = 0x08,
  ADS124S08_REG_SYS      = 0x09,
  ADS124S08_REG_OFCAL0   = 0x0A,
  ADS124S08_REG_OFCAL1   = 0x0B,
  ADS124S08_REG_OFCAL2   = 0x0C,
  ADS124S08_REG_FSCAL0   = 0x0D,
  ADS124S08_REG_FSCAL1   = 0x0E,
  ADS124S08_REG_FSCAL2   = 0x0F,
  ADS124S08_REG_GPIODAT  = 0x10,
  ADS124S08_REG_GPIOCON  = 0x11
} ADS124S08_Register;

typedef struct
{
  SPI_HandleTypeDef *spi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  GPIO_TypeDef *dout_port;
  uint16_t dout_pin;
  uint32_t spi_timeout_ms;
} ADS124S08_HandleTypeDef;

ADS124S08_Status ADS124S08_Init(ADS124S08_HandleTypeDef *device);
ADS124S08_Status ADS124S08_SendCommand(ADS124S08_HandleTypeDef *device,
                                       uint8_t command);
ADS124S08_Status ADS124S08_ReadRegisters(ADS124S08_HandleTypeDef *device,
                                         uint8_t start_address,
                                         uint8_t *data,
                                         uint8_t count);
ADS124S08_Status ADS124S08_WriteRegisters(ADS124S08_HandleTypeDef *device,
                                          uint8_t start_address,
                                          const uint8_t *data,
                                          uint8_t count);
ADS124S08_Status ADS124S08_ConfigureChannel(ADS124S08_HandleTypeDef *device,
                                            uint8_t positive_input,
                                            uint8_t negative_input,
                                            uint8_t idac1_output);
ADS124S08_Status ADS124S08_ReadSingle(ADS124S08_HandleTypeDef *device,
                                      int32_t *code,
                                      uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
