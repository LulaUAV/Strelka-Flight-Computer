/*
 * data_logging.h
 *
 *  Created on: Nov 3, 2022
 *      Author: Angus McLennan
 */

#ifndef DATA_LOGGING_H_
#define DATA_LOGGING_H_

#endif /* DATA_LOGGING_H_ */

/* Includes */
/* Generic C libraries */
#include <stdint.h>
#include <string.h> // memcpy
#include <stdlib.h> //realloc

/* STM32 specific libraries */
#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
// #include "stm32f4xx_hal_sd.h"
#include "fatfs.h"
// #include "cmsis_os.h"
#include "ms5611.h"
#include "bmx055.h"
#include "max_m10.h"

typedef struct {
    SD_HandleTypeDef* hsd;
    uint8_t rtext[_MAX_SS];             /* File read buffer */
} data_logging_handle;

void data_logging_init();
void log_gps(char* data, size_t len);
void log_baro(char* data, size_t len);
void log_accel(char* data, size_t len);
void log_gyro(char* data, size_t len);
void log_mag(char* data, size_t len);
void log_to_file(data_logging_handle* data_handle, char* filename, char* data, size_t len);
void data_logging_init(data_logging_handle* data_handle);