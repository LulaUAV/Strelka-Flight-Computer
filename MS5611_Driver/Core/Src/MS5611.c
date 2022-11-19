/*
 * MS5611.c
 *
 *  Created on: 17 Nov. 2022
 *      Author: Angus McLennan, adapted from (c) 2014 Korneliusz Jarzebski www.jarzebski.pl
 */

#include <math.h>
#include "MS5611.h"

bool MS5611_init(MS5611_Handle* ms5611, ms5611_osr_t osr) {
    MS5611_reset(ms5611);

    MS5611_setOversampling(ms5611, osr);

    osDelay(100);

    MS5611_readPROM(ms5611);

    return true;
}

// Set oversampling value
void MS5611_setOversampling(MS5611_Handle* ms5611, ms5611_osr_t osr) {
    switch (osr)
    {
	case MS5611_ULTRA_LOW_POWER:
		ms5611->conversion_time = 1;
	    break;
	case MS5611_LOW_POWER:
		ms5611->conversion_time = 2;
	    break;
	case MS5611_STANDARD:
		ms5611->conversion_time = 3;
	    break;
	case MS5611_HIGH_RES:
		ms5611->conversion_time = 5;
	    break;
	case MS5611_ULTRA_HIGH_RES:
		ms5611->conversion_time = 10;
	    break;
    }

    ms5611->uosr = osr;
}

// Get oversampling value
ms5611_osr_t MS5611_getOversampling(MS5611_Handle* ms5611) {
    return (ms5611_osr_t)ms5611->uosr;
}

void MS5611_reset(MS5611_Handle* ms5611) {   
    MS5611_sendCommand(ms5611, MS5611_CMD_RESET);
}

void MS5611_readPROM(MS5611_Handle* ms5611) {
    uint16_t data;
    for (uint8_t offset = 0; offset < 6; offset++)
    {
        MS5611_readRegister16(ms5611, MS5611_CMD_READ_PROM + (offset * 2), &data);
	    ms5611->fc[offset] = data;
    }
}

uint32_t MS5611_readRawTemperature(MS5611_Handle* ms5611) {

    MS5611_sendCommand(ms5611, MS5611_CMD_CONV_D2 + ms5611->uosr);
    osDelay(ms5611->conversion_time);

    uint32_t data;
    MS5611_readRegister24(ms5611, MS5611_CMD_ADC_READ, &data);
    return data; 
}

uint32_t MS5611_readRawPressure(MS5611_Handle* ms5611) {

    MS5611_sendCommand(ms5611, MS5611_CMD_CONV_D1 + ms5611->uosr);

    osDelay(ms5611->conversion_time);

    uint32_t data;
    MS5611_readRegister24(ms5611, MS5611_CMD_ADC_READ, &data);
    return data; 
}

int32_t MS5611_readPressure(MS5611_Handle* ms5611, bool compensation) {
    uint32_t D1 = readRawPressure();

    uint32_t D2 = readRawTemperature();
    int32_t dT = D2 - (uint32_t)ms5611->fc[4] * 256;

    int64_t OFF = (int64_t)ms5611->fc[1] * 65536 + (int64_t)ms5611->fc[3] * dT / 128;
    int64_t SENS = (int64_t)ms5611->fc[0] * 32768 + (int64_t)ms5611->fc[2] * dT / 256;

    if (compensation)
    {
	int32_t TEMP = 2000 + ((int64_t) dT * ms5611->fc[5]) / 8388608;

	ms5611->OFF2 = 0;
	ms5611->SENS2 = 0;

	if (TEMP < 2000)
	{
	    ms5611->OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
	    ms5611->SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
	}

	if (TEMP < -1500)
	{
	    ms5611->OFF2 = ms5611->OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
	    ms5611->SENS2 = ms5611->SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
	}

	OFF = OFF - ms5611->OFF2;
	SENS = SENS - ms5611->SENS2;
    }

    uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;

    return P;
}

double MS5611_readTemperature(MS5611_Handle* ms5611, bool compensation) {
    uint32_t D2 = readRawTemperature();
    int32_t dT = D2 - (uint32_t)ms5611->fc[4] * 256;

    int32_t TEMP = 2000 + ((int64_t) dT * ms5611->fc[5]) / 8388608;

    ms5611->TEMP2 = 0;

    if (compensation)
    {
	if (TEMP < 2000)
	{
	    ms5611->TEMP2 = (dT * dT) / (2 << 30);
	}
    }

    TEMP = TEMP - ms5611->TEMP2;

    return ((double)TEMP/100);
}

// Calculate altitude from Pressure & Sea level pressure
double MS5611_getAltitude(double pressure, double seaLevelPressure) {
    return (44330.0f * (1.0f - pow((double)pressure / (double)seaLevelPressure, 0.1902949f)));
}

// Calculate sea level from Pressure given on specific altitude
double MS5611_getSeaLevel(double pressure, double altitude) {
    return ((double)pressure / pow(1.0f - ((double)altitude / 44330.0f), 5.255f));
}

// Read 16-bit from register (oops MSB, LSB)
void MS5611_readRegister16(MS5611_Handle* ms5611, uint8_t register_addr, uint16_t* data) {
    uint16_t value;

    uint8_t read_data[2];

    HAL_GPIO_WritePin(ms5611->baro_CS_port, ms5611->baro_CS_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(ms5611->hspi, register_addr, &read_data, sizeof(read_data), 1000);
    HAL_GPIO_WritePin(ms5611->baro_CS_port, ms5611->baro_CS_pin, GPIO_PIN_SET);

    value = read_data[0] << 8 | read_data[1];
    data = &value;
}

// Read 24-bit from register (oops XSB, MSB, LSB)
void MS5611_readRegister24(MS5611_Handle* ms5611, uint8_t register_addr, uint32_t* data) {
    uint32_t value;
    uint8_t read_data[4];

    HAL_GPIO_WritePin(ms5611->baro_CS_port, ms5611->baro_CS_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(ms5611->hspi, register_addr, &read_data, sizeof(read_data), 1000);
    HAL_GPIO_WritePin(ms5611->baro_CS_port, ms5611->baro_CS_pin, GPIO_PIN_SET);

    // Check byte order is correct
    value = ((int32_t)read_data[0] << 16) | ((int32_t)read_data[1] << 8) | read_data[2];

    data = &value;
}

void MS5611_readRegister(MS5611_Handle* ms5611, uint8_t register_addr, uint8_t* data) {
    uint8_t read_data;

    HAL_GPIO_WritePin(ms5611->baro_CS_port, ms5611->baro_CS_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(ms5611->hspi, register_addr, &read_data, sizeof(read_data), 1000);
    HAL_GPIO_WritePin(ms5611->baro_CS_port, ms5611->baro_CS_pin, GPIO_PIN_SET);

    data = &read_data;
}

void MS5611_writeRegister(MS5611_Handle* ms5611, uint8_t register_addr, uint8_t value) {

    HAL_GPIO_WritePin(ms5611->baro_CS_port, ms5611->baro_CS_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = HAL_SPI_Transmit(ms5611->hspi, &register_addr, sizeof(register_addr), 1000);
    result = HAL_SPI_Transmit(ms5611->hspi, &value, sizeof(value), 1000);
    HAL_GPIO_WritePin(ms5611->baro_CS_port, ms5611->baro_CS_pin, GPIO_PIN_SET);
}

void MS5611_sendCommand(MS5611_Handle* ms5611, uint8_t cmd) {

    HAL_GPIO_WritePin(ms5611->baro_CS_port, ms5611->baro_CS_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = HAL_SPI_Transmit(ms5611->hspi, &cmd, sizeof(cmd), 1000);
    HAL_GPIO_WritePin(ms5611->baro_CS_port, ms5611->baro_CS_pin, GPIO_PIN_SET);
}
