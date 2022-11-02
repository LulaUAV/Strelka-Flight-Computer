/*
 * Ematch_actuators.c
 *
 *  Created on: Nov 2, 2022
 *      Author: Angus McLennan
 */

#include "Ematch_actuators.h"





void fire_main_ematch(ematch_pins* ematch_pins) {
    // Toggle ematch to fire
    HAL_GPIO_WritePin(ematch_pins->main_h_port, ematch_pins->main_h_pin, 1);
    HAL_GPIO_WritePin(ematch_pins->main_l_port, ematch_pins->main_l_pin, 1);
    // Delay non-blocking for 2 seconds
    osDelay(2000);
    // Toggle ematch pins low
    HAL_GPIO_WritePin(ematch_pins->main_h_port, ematch_pins->main_h_pin, 0);
    HAL_GPIO_WritePin(ematch_pins->main_l_port, ematch_pins->main_l_pin, 0);
}