/*
 * open_loop.h
 */

#ifndef INC_OPEN_LOOP_H_
#define INC_OPEN_LOOP_H_

#include "main.h"
#include <stdint.h>

void open_loop_init(void);
uint8_t open_loop_start(float modulation, float freq_e_hz);
void open_loop_set(float modulation, float freq_e_hz);
void open_loop_stop(void);
void open_loop_adc_tick(float dt_s);
void open_loop_poll(void);
void open_loop_print_status(void);
uint8_t open_loop_is_running(void);
float open_loop_get_modulation(void);
float open_loop_get_target_modulation(void);
float open_loop_get_freq_e_hz(void);
float open_loop_get_target_freq_e_hz(void);
float open_loop_get_theta_e_rad(void);

#endif /* INC_OPEN_LOOP_H_ */
