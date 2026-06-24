/*
 * vesc_can.h
 *
 *  Created on: Apr 24, 2026
 *      Author: skadi
 */

#ifndef INC_VESC_CAN_H_
#define INC_VESC_CAN_H_

#include "main.h"
#include <stdint.h>

void can_test_init(void);
void can_test_send_dummy(void);
void can_test_poll_rx(void);

#endif /* INC_VESC_CAN_H_ */
