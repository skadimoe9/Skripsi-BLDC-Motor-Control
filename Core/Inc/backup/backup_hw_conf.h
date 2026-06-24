/*
 * hw_conf.h
 *
 *  Created on: Apr 24, 2026
 *      Author: skadi
 */

#ifndef INC_HW_CONF_H_
#define INC_HW_CONF_H_

#include "main.h"

// =========================
// ADC base configuration
// =========================

#define ADC_VREF                    3.3f
#define ADC_MAX_12BIT               4095.0f

// =========================
// Voltage sensing
// =========================

#define VIN_R1                      39000.0f
#define VIN_R2                      2200.0f
#define VIN_DIV_RATIO               ((VIN_R1 + VIN_R2) / VIN_R2)

// Kalau phase voltage divider sama dengan VBUS, pakai ini.
// Kalau beda, nanti pisahkan nilainya.

#define VPHASE_R1                   39000.0f
#define VPHASE_R2                   2200.0f
#define VPHASE_DIV_RATIO            ((VPHASE_R1 + VPHASE_R2) / VPHASE_R2)

// =========================
// Current sensing
// =========================

#define CURRENT_AMP_GAIN            10.0f
#define CURRENT_SHUNT_RES           0.001f

// Untuk current sensor bidirectional, offset biasanya sekitar Vref/2.
// Nilai ini nanti lebih baik dikalibrasi dari ADC raw saat no-current.

#define CURRENT_OFFSET_V            (ADC_VREF * 0.5f) // Nanti kalibrasi
#define CURRENT_OFFSET_RAW          (ADC_MAX_12BIT * 0.5f) // Nanti kalibrasi lagi aja

// I = (Vadc - Voffset) / (gain * Rshunt)
// Vadc hasil bacaan

#define CURRENT_A_PER_V             (1.0f / (CURRENT_AMP_GAIN * CURRENT_SHUNT_RES))

// =========================
// Passive FOC observer current mapping
// =========================
// Hardware schematic assumption:
// logical ia is measured by i1, logical ic is measured by i3,
// and logical ib is reconstructed as -(ia + ic) because phase B has no shunt.

#define FOC_CURRENT_IA_SIGN         1.0f
#define FOC_CURRENT_IC_SIGN         1.0f
#define FOC_THETA_OFFSET_RAD        0.0f//-1.57079633f

// =========================
// ADC1 + ADC2 independent DMA buffers
// =========================

#define ADC1_NUM_CONV               3
#define ADC2_NUM_CONV               3

// Sesuaikan dengan rank di ADC.
// Rank 1 -> buffer index 0
// Rank 2 -> buffer index 1
// Rank 3 -> buffer index 2
// ADC1 rank 2 -> CURR2 / default I3
// ADC2 rank 2 -> default I1

#define ADC1_IDX_SENS3              0
#define ADC1_IDX_CURR2              1
#define ADC1_IDX_VREFINT            2

#define ADC2_IDX_SENS2              0
#define ADC2_IDX_CURR_PB1           1
#define ADC2_IDX_ADC_EXT2           2

// =========================
// ADC3 standalone DMA buffer
// =========================

#define ADC3_NUM_CONV               3

// Sesuaikan dengan rank ADC3 kamu.
#define ADC3_IDX_PHASE_A            0
#define ADC3_IDX_MOSFET_TEMP        1
#define ADC3_IDX_VBUS	            2

// =========================
// Hall sensor pins
// =========================

//#define HW_HALL_A_GPIO_Port            GPIOB
//#define HW_HALL_A_Pin                  GPIO_PIN_6

//#define HW_HALL_B_GPIO_Port            GPIOB
//#define HW_HALL_B_Pin                  GPIO_PIN_7

//#define HW_HALL_C_GPIO_Port            GPIOC
//#define HW_HALL_C_Pin                  GPIO_PIN_11

// =========================
// Motor configuration
// =========================

#define MOTOR_POLE_PAIRS            15.0f

// =========================
// PWM phase mapping
// =========================
// Hardware mapping from board phase labels to TIM1 channels.
// TIM1_CH1 = W, TIM1_CH2 = V, TIM1_CH3 = U.

#define HW_PWM_PHASE_U_CHANNEL      TIM_CHANNEL_3
#define HW_PWM_PHASE_V_CHANNEL      TIM_CHANNEL_2
#define HW_PWM_PHASE_W_CHANNEL      TIM_CHANNEL_1

// =========================
// Gate driver pins
// =========================
//
// DRV8302-related control/status pins.
// EN_GATE : enable/disable gate driver
// FAULT   : fault status from gate driver, usually active-low
// DC_CAL  : current shunt amplifier calibration control

#define HW_GATE_EN_GPIO_Port        GPIOC
#define HW_GATE_EN_Pin              GPIO_PIN_10

#define HW_GATE_FAULT_GPIO_Port     GPIOC
#define HW_GATE_FAULT_Pin           GPIO_PIN_12

#define HW_GATE_DC_CAL_GPIO_Port    GPIOB
#define HW_GATE_DC_CAL_Pin          GPIO_PIN_12

#define HW_GATE_FAULT_ACTIVE_STATE   GPIO_PIN_RESET
#define HW_GATE_FAULT_NORMAL_STATE   GPIO_PIN_SET

// =========================
// CAN telemetry IDs
// =========================
//
// CAN ID lebih kecil punya prioritas lebih tinggi di bus CAN.
// Urutan dibuat sesuai prioritas debug

// 0x101 HALL
// Byte 0-1 : calculated_rpm
// Byte 2-3 : theta_r_x100
// Byte 4-5 : theta_e_x100
// Byte 6   : raw hall_state bitfield, decimal 1..6
// Byte 7   : sequence low byte
#define CAN_ID_HALL    0x101

// 0x102 CURRENT
// Byte 0-1 : ia_x100
// Byte 2-3 : ib_x100
// Byte 4-5 : ic_x100
// Byte 6-7 : reserved
#define CAN_ID_CURRENT       0x102

// 0x103 VOLTAGE
// Byte 0-1 : vbus_x10
// Byte 2-3 : vphase_a_x10
// Byte 4-5 : vphase_b_x10
// Byte 6-7 : vphase_c_x10
#define CAN_ID_VOLTAGE   0x103

// 0x104 DQ
// Byte 0-1 : id_x100
// Byte 2-3 : iq_x100
// Byte 4-5 : calculated_duty_x10000
// Byte 6-7 : reserved
#define CAN_ID_DQ             0x104

// 0x105 STATUS
// Byte 0   : mode
// Byte 1   : fault_code
// Byte 2   : pwm_enabled
// Byte 3   : gate_enabled
// Byte 4   : fw_enabled
// Byte 5   : fw_active/status
// Byte 6-7 : sequence
#define CAN_ID_STATUS         0x105

// 0x106 SETPOINT
// Byte 0-1 : max_current_x100
// Byte 2-3 : fw_max_current_x100
// Byte 4-5 : target_electrical_rpm
// Byte 6-7 : target_duty_x10000
#define CAN_ID_SETPOINT       0x106

// 0x107 ANGLE DEBUG
// Byte 0-1 : open_loop_theta_e_x100, signed deg, wrapped -180..180
// Byte 2-3 : hall_theta_e_x100, signed deg, wrapped -180..180
// Byte 4-5 : theta_error_x100 = open_loop - hall, signed deg, wrapped -180..180
// Byte 6   : raw hall_state bitfield, decimal 1..6
// Byte 7   : hall direction code, 0=negative, 1=zero/unknown, 2=positive
#define CAN_ID_ANGLE_DEBUG    0x107

// 0x108 HALL TRANSITION EVENT
// Byte 0-3 : TIM5 timestamp_us, unsigned 32-bit
// Byte 4-5 : open_loop_theta_e_x100 at Hall edge, signed deg, wrapped -180..180
// Byte 6   : previous Hall state in high nibble, new Hall state in low nibble
// Byte 7   : bits 0-1 direction code, bits 2-7 transition sequence modulo 64
#define CAN_ID_HALL_EVENT      0x108

// 0x109 SMOOTH PHASE VOLTAGE
// Commanded/estimated sine phase voltages, not raw ADC samples.
// Byte 0-1 : vphase_a_smooth_x100, signed V, relative to DC midpoint
// Byte 2-3 : vphase_b_smooth_x100, signed V, relative to DC midpoint
// Byte 4-5 : vphase_c_smooth_x100, signed V, relative to DC midpoint
// Byte 6-7 : sequence
#define CAN_ID_PHASE_SMOOTH    0x109

/*
 * 10 ms  → 100 Hz
 * 20 ms  → 50 Hz
 * 50 ms  → 20 Hz
 * 100 ms → 10 Hz
 */

#define CAN_RATE_ANGLE_SPEED_MS     20
#define CAN_RATE_CURRENT_MS         20
#define CAN_RATE_ELECTRICAL_MS      20
#define CAN_RATE_DQ_MS              20
#define CAN_RATE_STATUS_MS          20
#define CAN_RATE_SETPOINT_MS        100

// =========================
// UART command
// =========================

#define UART_CMD_BUFFER_SIZE         64U

#endif /* INC_HW_CONF_H_ */
