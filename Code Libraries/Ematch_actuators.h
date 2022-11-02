/*
 * Ematch_actuators.h
 *
 *  Created on: Nov 2, 2022
 *      Author: Angus McLennan
 */

#ifndef EMATCH_ACTUATORS_H_
#define EMATCH_ACTUATORS_H_



#endif /* EMATCH_ACTUATORS_H_ */

/* Includes */
/* Generic C libraries */
#include <stdint.h>

/* STM32 specific libraries */
#include "stm32f4xx_hal_gpio.h"
#include "cmsis_os2.h"

/* Struct definition */
typedef struct {
    /* Define hardware pins */ 
    // Main deploy
    GPIO_TypeDef * const main_h_port;
    uint8_t main_h_pin;
    GPIO_TypeDef * const main_l_port;
    uint8_t main_l_pin;
    GPIO_TypeDef * const main_cont_port;
    uint8_t main_cont_pin;

    // Drogue deploy
    GPIO_TypeDef * const drogue_h_port;
    uint8_t drogue_h_pin;
    GPIO_TypeDef * const drogue_l_port;
    uint8_t drogue_l_pin;
    GPIO_TypeDef * const drogue_cont_port;
    uint8_t drogue_cont_pin;

    // Continuity test enable
    GPIO_TypeDef * const cont_test_en_port;
    uint8_t cont_test_en_pin;
}ematch_pins;


/* Function prototypes */
void fire_main_ematch(ematch_pins* ematch_pins);
void fire_drogue_ematch(ematch_pins* ematch_pins);
double test_drogue_continuity(ematch_pins* ematch_pins);
double test_main_continuity(ematch_pins* ematch_pins);