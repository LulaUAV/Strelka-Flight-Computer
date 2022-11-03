/*
 * actuator_control.h
 *
 *  Created on: Nov 3, 2022
 *      Author: Angus McLennan
 */

#ifndef ACTUATOR_CONTROL_H_
#define ACTUATOR_CONTROL_H_



#endif /* ACTUATOR_CONTROL_H_ */

/* Includes */
/* Generic C libraries */
#include <stdint.h>
#include <string.h> // memcpy
#include <stdlib.h> //realloc

/* STM32 specific libraries */
#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_crc.h"

#define SET_SERVO_POS_ID 0x44

typedef enum
{
    SET_COMMAND_SUCCESS      = 0x00U,   /*!< Actuation was successfuly executed                 */  
    SPI_COMMS_FAIL           = 0x01U,   /*!< Communication failure                              */
    SET_COMMAND_FAILURE      = 0x02U,   /*!< Actuation failed to execute                        */
    SPI_NO_RESPONSE          = 0x03U    /*!< Communication did not response                     */
} actuator_Control_StateTypeDef;

typedef struct {
    SPI_HandleTypeDef* ext_spi;
    CRC_HandleTypeDef* hcrc;
}external_actuator_handle;

void actuator_control_init(external_actuator_handle* actuator_handle);
actuator_Control_StateTypeDef set_servo_positions(external_actuator_handle* actuator_handle, uint32_t servo_positions[4]);
uint8_t calculate_crc32(external_actuator_handle* actuator_handle, uint8_t* received_buffer, size_t len);
