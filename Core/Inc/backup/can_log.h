/*
 * can_log.h
 *
 *  Created on: Jun 2, 2026
 *      Author: skadi
 *
 * CAN telemetry logging interface.
 *
 * Current purpose:
 * - start CAN peripheral
 * - send periodic telemetry frames
 * - send ADC, Hall, motor status, setpoint, and FOC/FW-related data
 *
 * Notes:
 * - CAN command receive will be handled later or separated if needed.
 * - FOC Id/Iq telemetry is populated when foc_control is running.
 * - Field Weakening status remains zero until that layer is implemented.
 */

#ifndef INC_CAN_LOG_H_
#define INC_CAN_LOG_H_

#include <stdint.h>
#include "hw_conf.h"

// =========================
// CAN initialization
// =========================
//
// Call CANLog_Start() once after MX_CANx_Init().
// This starts CAN peripheral and enables notification if needed.

void CANLog_Start(void);

// =========================
// Periodic CAN task
// =========================
//
// Call this function inside while(1).
// This function decides when to send telemetry frames
// based on CAN telemetry period configuration.

void CANLog_Task(void);

// =========================
// Manual telemetry transmit functions
// =========================
//
// These can be called manually for testing.
// CANLog_Task() may also call these internally.

void CANLog_SendHall(void);
void CANLog_SendCurrent(void);
void CANLog_SendVoltage(void);
void CANLog_SendDQ(void);
void CANLog_SendStatus(void);
void CANLog_SendSetpoint(void);
void CANLog_SendAngleDebug(void);
void CANLog_SendPhaseSmooth(void);

// =========================
// Passive D/Q diagnostic convention
// =========================
// These settings affect only passive D/Q telemetry in CAN frame 0x104.
// They do not change PWM output, ADC calibration, gate control, or future FOC
// control until explicitly wired into a controller.

#define CANLOG_DQ_MAP_I1_A_I3_C      0U
#define CANLOG_DQ_MAP_I3_A_I1_C      1U
#define CANLOG_DQ_MAP_I1_A_I3_B      2U
#define CANLOG_DQ_MAP_I3_A_I1_B      3U
#define CANLOG_DQ_MAP_I1_B_I3_C      4U
#define CANLOG_DQ_MAP_I3_B_I1_C      5U
#define CANLOG_DQ_MAP_COUNT          6U

uint8_t CANLog_SetDQMap(uint8_t map);
uint8_t CANLog_GetDQMap(void);
const char *CANLog_GetDQMapName(uint8_t map);

uint8_t CANLog_SetDQThetaDirection(int8_t direction);
int8_t CANLog_GetDQThetaDirection(void);

void CANLog_SetDQThetaOffsetDeg(float offset_deg);
float CANLog_GetDQThetaOffsetDeg(void);

uint8_t CANLog_SetDQCurrentSigns(int8_t i1_sign, int8_t i3_sign);
int8_t CANLog_GetDQI1Sign(void);
int8_t CANLog_GetDQI3Sign(void);

// =========================
// Utility / status
// =========================

uint8_t CANLog_IsStarted(void);
uint32_t CANLog_GetTxCounter(void);
uint32_t CANLog_GetTxErrorCounter(void);

#endif /* INC_CAN_LOG_H_ */
