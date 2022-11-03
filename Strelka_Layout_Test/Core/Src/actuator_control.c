/*
 * actuator_control.c
 *
 *  Created on: Nov 3, 2022
 *      Author: Angus McLennan
 */

#include "actuator_control.h"

/**
  * @brief  Initialises functions within this library.
  * @param external_actuator_handle* actuator_handle struct containting configuration
  * @retval None
  */
void actuator_control_init(external_actuator_handle* actuator_handle) {
//    actuator_handle->hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
//    actuator_handle->hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
//    actuator_handle->hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
//    actuator_handle->hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
//    actuator_handle->hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
}

/**
  * @brief  Sets the angular positions of four servos.
  * @param external_actuator_handle* actuator_handle struct containing configuration
  * @param uint32_t servo_positions[4] array of size four contatining angular positions of servos
  * ranging from 0 -> 2^32
  * @retval Actuator_Control_StateTypeDef Command execution response
  */
actuator_Control_StateTypeDef set_servo_positions(external_actuator_handle* actuator_handle, uint32_t servo_positions[4]) {
    uint8_t send_buffer[21];
    send_buffer[0] = SET_SERVO_POS_ID;
    memcpy(&send_buffer[1], servo_positions, sizeof(servo_positions));
    uint32_t send_crc = HAL_CRC_Calculate(actuator_handle->hcrc, send_buffer, sizeof(send_buffer)-1);
    // Add CRC
    memcpy(&send_buffer[17], ~send_crc, sizeof(send_crc));
    
    // Transmit data
    HAL_StatusTypeDef SPI_result = HAL_SPI_Transmit(actuator_handle->hcrc, send_buffer, sizeof(send_buffer), HAL_MAX_DELAY);
    // Receive response
    uint8_t receive_buffer[6];
    HAL_SPI_Receive(actuator_handle->hcrc, receive_buffer, sizeof(receive_buffer), 5);
    if (calculate_crc32(actuator_handle, receive_buffer, sizeof(receive_buffer)) && receive_buffer[0] == SET_SERVO_POS_ID) {
        return receive_buffer[1];
    }
    else return SPI_result;
}

/**
  * @brief  Calculates the 32-bit CRC in hardware
  * @param  uint8_t* received_buffer Buffer received over SPI
  * @param  size_t len Length of buffer recevied
  * @retval Boolean to indicate successful CRC comparison
  */
uint8_t calculate_crc32(external_actuator_handle* actuator_handle, uint8_t* received_buffer, size_t len) {
    uint32_t receive_crc;
    // Copy in received CRC
    memcpy(&receive_crc, &received_buffer[len-4], sizeof(uint32_t));
    uint32_t receive_calc_crc = HAL_CRC_Calculate(actuator_handle->hcrc, (uint32_t*)received_buffer, sizeof(received_buffer)-4);
    return receive_calc_crc == receive_crc;
}
 
