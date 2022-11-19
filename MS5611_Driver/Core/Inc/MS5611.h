/*
 * MS5611.h
 *
 *  Created on: 17 Nov. 2022
 *      Author: Angus McLennan, adapted from (c) 2014 Korneliusz Jarzebski www.jarzebski.pl
 */

#ifndef INC_MS5611_H_
#define INC_MS5611_H_

#endif /* INC_MS5611_H_ */

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define MS5611_ADDRESS                (0x77)
#define MS5611_CMD_ADC_READ           (0x00)
#define MS5611_CMD_RESET              (0x1E)
#define MS5611_CMD_CONV_D1            (0x40)
#define MS5611_CMD_CONV_D2            (0x50)
#define MS5611_CMD_READ_PROM          (0xA2)

typedef enum
{
    MS5611_ULTRA_HIGH_RES   = 0x08,
    MS5611_HIGH_RES         = 0x06,
    MS5611_STANDARD         = 0x04,
    MS5611_LOW_POWER        = 0x02,
    MS5611_ULTRA_LOW_POWER  = 0x00
} ms5611_osr_t;

typedef struct {
    SPI_HandleTypeDef* hspi;
    /* Barometer paramters */
    GPIO_TypeDef*      baro_CS_port;
	uint16_t		   baro_CS_pin;
    uint8_t            conversion_time;
    uint16_t           fc[6];
    uint8_t            uosr;
    int32_t            TEMP2;
    int64_t            OFF2;
    int64_t            SENS2;
}MS5611_Handle;

bool MS5611_init(MS5611_Handle* ms5611, ms5611_osr_t osr);
void MS5611_setOversampling(MS5611_Handle* ms5611, ms5611_osr_t osr);
ms5611_osr_t MS5611_getOversampling(MS5611_Handle* ms5611);
void MS5611_reset(MS5611_Handle* ms5611);
void MS5611_readPROM(MS5611_Handle* ms5611);
uint32_t MS5611_readRawTemperature(MS5611_Handle* ms5611);
uint32_t MS5611_readRawPressure(MS5611_Handle* ms5611);
int32_t MS5611_readPressure(MS5611_Handle* ms5611, bool compensation);
double MS5611_readTemperature(MS5611_Handle* ms5611, bool compensation);
double MS5611_getAltitude(double pressure, double seaLevelPressure);
double MS5611_getSeaLevel(double pressure, double altitude);
void MS5611_readRegister16(MS5611_Handle* ms5611, uint8_t register_addr, uint16_t* data);
void MS5611_readRegister16(MS5611_Handle* ms5611, uint8_t register_addr, uint16_t* data);
void MS5611_readRegister24(MS5611_Handle* ms5611, uint8_t register_addr, uint32_t* data);
void MS5611_readRegister(MS5611_Handle* ms5611, uint8_t register_addr, uint8_t* data);
void MS5611_writeRegister(MS5611_Handle* ms5611, uint8_t register_addr, uint8_t value);
void MS5611_sendCommand(MS5611_Handle* ms5611, uint8_t cmd);
