/*
 * hall_sensor.h
 *
 *  Created on: Apr 24, 2026
 *      Author: skadi
 */

#ifndef INC_HALL_SENSOR_H_
#define INC_HALL_SENSOR_H_

#include "main.h"
#include <stdint.h>

/* Hall sensor reconstruction API.
 * main.c initializes it and forwards Hall EXTI edges with a TIM5 timestamp.
 * adc_sense.c calls Hall_ControlTick() at the 10 kHz control tick before FOC.
 */

typedef struct
{
    /* Raw state and validity flags used by FOC and CAN telemetry. */
    uint8_t hall_state;
    uint8_t previous_state;
    uint8_t state_valid;
    uint8_t speed_valid;
    uint8_t angle_valid;
    int8_t direction;

    uint32_t edge_count;
    uint32_t invalid_state_count;
    uint32_t invalid_transition_count;
    uint32_t duplicate_edge_count;
    uint32_t glitch_count;

    uint32_t last_edge_us;
    uint32_t dt_sum_us;

    /* Reconstructed electrical angle/speed. FOC uses theta_e_rad and
     * mechanical_rad_s after applying its own direction/offset calibration.
     */
    float theta_e_rad;
    float theta_e_deg;
    float theta_r_deg;
    float omega_e_rad_s;
    float omega_corr_rad_s;
    float mechanical_rad_s;
} hall_sensor_data_t;

typedef struct
{
    /* Debug/event record for Hall transitions. Kept for local diagnostics. */
    uint32_t sequence;
    uint32_t timestamp_us;
    uint8_t previous_state;
    uint8_t hall_state;
    int8_t direction;
    float open_loop_theta_e_rad;
} hall_transition_event_t;

/* Read the three GPIO pins and pack them as H1:H2:H3 bits. */
uint8_t Hall_ReadState(void);

/* Validate whether a raw 3-bit state belongs to the six legal Hall states. */
uint8_t Hall_IsValidState(uint8_t state);
const char* Hall_StateName(uint8_t state);

/* Print compact current Hall state over UART. */
void Hall_PrintState(void);

/* Called from main.c after TIM5 is initialized. */
void Hall_Init(void);

/* Called from main.c HAL_GPIO_EXTI_Callback() for every Hall edge. */
void Hall_EXTI_Callback(uint16_t GPIO_Pin, uint32_t timestamp_us);

/* Called from adc_sense.c at 10 kHz to update interpolated angle and speed. */
void Hall_ControlTick(void);

/* Foreground-safe copy, used by commands and CAN telemetry. */
hall_sensor_data_t Hall_GetData(void);

/* ISR/control-path copy, used immediately before foc_control_adc_tick(). */
hall_sensor_data_t Hall_GetDataFromISR(void);

/* Optional debug-event queue helpers. Current CAN telemetry no longer uses 0x108. */
uint8_t Hall_PopTransitionEvent(hall_transition_event_t *event);
uint32_t Hall_GetTransitionEventOverflowCount(void);

#endif /* INC_HALL_SENSOR_H_ */
