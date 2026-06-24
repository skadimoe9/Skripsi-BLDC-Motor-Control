/*
 * vesc_uart.h
 *
 *  Created on: Apr 23, 2026
 *      Author: skadimoe9/fadhli
 */

#ifndef INC_VESC_UART_H_
#define INC_VESC_UART_H_

#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

void uart_print(const char *s);
void process_received_line(const char *line);
void telemetry_send(float vin, float current, uint8_t hall, uint8_t sector);

#endif /* INC_VESC_UART_H_ */
