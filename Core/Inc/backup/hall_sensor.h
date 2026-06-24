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

typedef struct
{
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

    float theta_e_rad;
    float theta_e_deg;
    float theta_r_deg;
    float omega_e_rad_s;
    float omega_corr_rad_s;
    float mechanical_rpm;
} hall_sensor_data_t;

typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_us;
    uint8_t previous_state;
    uint8_t hall_state;
    int8_t direction;
    float open_loop_theta_e_rad;
} hall_transition_event_t;

uint8_t Hall_ReadState(void);
uint8_t Hall_IsValidState(uint8_t state);
const char* Hall_StateName(uint8_t state);
void Hall_PrintState(void);

void Hall_Init(void);
void Hall_EXTI_Callback(uint16_t GPIO_Pin, uint32_t timestamp_us);
void Hall_ControlTick(void);
hall_sensor_data_t Hall_GetData(void);
hall_sensor_data_t Hall_GetDataFromISR(void);
uint8_t Hall_PopTransitionEvent(hall_transition_event_t *event);
uint32_t Hall_GetTransitionEventOverflowCount(void);

#endif /* INC_HALL_SENSOR_H_ */
