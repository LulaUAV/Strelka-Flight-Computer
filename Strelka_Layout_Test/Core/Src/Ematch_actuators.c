/*
 * Ematch_actuators.c
 *
 *  Created on: Nov 2, 2022
 *      Author: Angus McLennan
 */

#include "Ematch_actuators.h"

/**
  * @brief  Initialises features of library.
  * @param  ematch_pins* ematch_pins is a struct containting pin configurations.
  * @retval None
  */
void init(ematch_pins* ematch_pins) {
  // Calibrate ADC on start up
  HAL_ADCEx_Calibration_Start(ematch_pins->main_continuity_ADC);
  HAL_ADCEx_Calibration_Start(ematch_pins->drogue_continuity_ADC);
}

/**
  * @brief  Fires main ematch.
  * @param  ematch_pins* ematch_pins is a struct containting pin configurations.
  * @retval None
  */
void fire_main_ematch(ematch_pins* ematch_pins) {
    // Toggle ematch to fire
    HAL_GPIO_WritePin(ematch_pins->main_h_port, ematch_pins->main_h_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ematch_pins->main_l_port, ematch_pins->main_l_pin, GPIO_PIN_SET);
    // Delay non-blocking for 2 seconds
    osDelay(2000);
    // Toggle ematch pins low
    HAL_GPIO_WritePin(ematch_pins->main_h_port, ematch_pins->main_h_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ematch_pins->main_l_port, ematch_pins->main_l_pin, GPIO_PIN_RESET);
}

/**
  * @brief  Fires drogue ematch.
  * @param  ematch_pins* ematch_pins is a struct containting pin configurations.
  * @retval None
  */
void fire_drogue_ematch(ematch_pins* ematch_pins) {
    // Toggle ematch to fire
    HAL_GPIO_WritePin(ematch_pins->drogue_h_port, ematch_pins->drogue_h_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ematch_pins->drogue_l_port, ematch_pins->drogue_l_pin, GPIO_PIN_SET);
    // Delay non-blocking for 2 seconds
    osDelay(2000);
    // Toggle ematch pins low
    HAL_GPIO_WritePin(ematch_pins->drogue_h_port, ematch_pins->drogue_h_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ematch_pins->drogue_l_port, ematch_pins->drogue_l_pin, GPIO_PIN_RESET);
}

/**
  * @brief  Tests drogue ematch continuity.
  * @param  ematch_pins* ematch_pins is a struct containting pin configurations.
  * @retval ADC 12-bit output from continuity sense pin
  */
uint16_t test_drogue_continuity(ematch_pins* ematch_pins) {
    // Toggle ematch fire L pin
    HAL_GPIO_WritePin(ematch_pins->drogue_l_port, ematch_pins->drogue_l_pin, GPIO_PIN_SET);
    // Toggle continuity test enable pin
    HAL_GPIO_WritePin(ematch_pins->cont_test_en_port, ematch_pins->cont_test_en_pin, GPIO_PIN_SET);

    // Read ADC
    HAL_ADC_Start(ematch_pins->drogue_continuity_ADC);
    // Poll ADC
    HAL_ADC_PollForConversion(ematch_pins->drogue_continuity_ADC, 1);
    uint16_t adc_value = HAL_ADC_GetValue(ematch_pins->drogue_continuity_ADC);

    // Toggle ematch fire L pin
    HAL_GPIO_WritePin(ematch_pins->drogue_l_port, ematch_pins->drogue_l_pin, GPIO_PIN_RESET);
    // Toggle continuity test enable pin
    HAL_GPIO_WritePin(ematch_pins->cont_test_en_port, ematch_pins->cont_test_en_pin, GPIO_PIN_RESET);
    
    return adc_value;
}

/**
  * @brief  Tests main ematch continuity.
  * @param  ematch_pins* ematch_pins is a struct containting pin configurations.
  * @retval ADC 12-bit output from continuity sense pin
  */
uint16_t test_main_continuity(ematch_pins* ematch_pins) {
    // Toggle ematch fire L pin
    HAL_GPIO_WritePin(ematch_pins->main_l_port, ematch_pins->main_l_pin, GPIO_PIN_SET);
    // Toggle continuity test enable pin
    HAL_GPIO_WritePin(ematch_pins->cont_test_en_port, ematch_pins->cont_test_en_pin, GPIO_PIN_SET);

    // Read ADC
    HAL_ADC_Start(ematch_pins->main_continuity_ADC);
    // Poll ADC
    HAL_ADC_PollForConversion(ematch_pins->main_continuity_ADC, 1);
    uint16_t adc_value = HAL_ADC_GetValue(ematch_pins->main_continuity_ADC);

    // Toggle ematch fire L pin
    HAL_GPIO_WritePin(ematch_pins->main_l_port, ematch_pins->main_l_pin, GPIO_PIN_RESET);
    // Toggle continuity test enable pin
    HAL_GPIO_WritePin(ematch_pins->cont_test_en_port, ematch_pins->cont_test_en_pin, GPIO_PIN_RESET);
    
    return adc_value;
}