/*
 * phase2_test.h
 *
 *  Created on: Apr 28, 2026
 *      Author: skadi
 */

#ifndef INC_PHASE2_TEST_H_
#define INC_PHASE2_TEST_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef enum
{
    PHASE2_INV_DISABLED = 0,
    PHASE2_INV_ENABLED,
    PHASE2_INV_FAULT_LATCHED
} phase2_inv_state_t;

void phase2_test_init(UART_HandleTypeDef *huart);
void phase2_test_poll(void);
void phase2_test_uart_rx_callback(UART_HandleTypeDef *huart);

uint8_t phase2_fault_active(void);
uint8_t phase2_fault_latched(void);
phase2_inv_state_t phase2_get_inverter_state(void);
uint8_t phase2_pwm_is_enabled(void);
float phase2_get_test_duty(void);
void phase2_pwm_stop_all(void);
void phase2_inverter_disable(void);
void phase2_inverter_enable(void);
void phase2_print_status(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_PHASE2_TEST_H_ */
