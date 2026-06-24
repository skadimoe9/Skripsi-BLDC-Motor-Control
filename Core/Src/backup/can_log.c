/*
 * can_log.c
 *
 *  Created on: Jun 2, 2026
 *      Author: skadi
 *
 * CAN telemetry logging implementation.
 *
 * Design:
 * - Take synchronized telemetry snapshot every CAN_SNAPSHOT_PERIOD_MS.
 * - Send 0x101 to 0x107 from the same snapshot.
 * - Send 0x108 Hall transition events from the foreground task.
 * - Use pending scheduler so frames are not dropped when CAN TX mailbox is full.
 * - Send at most one pending frame per CANLog_Task() call.
 */

#include "can_log.h"
#include "main.h"
#include "adc_sense.h"
#include "foc_control.h"
#include "foc_math.h"
#include "hall_sensor.h"
#include "open_loop.h"
#include "phase2_test.h"

#include <math.h>

// =========================
// External CAN handle
// =========================

extern CAN_HandleTypeDef hcan1;

// =========================
// Snapshot period fallback
// =========================

#ifndef CAN_SNAPSHOT_PERIOD_MS
#define CAN_SNAPSHOT_PERIOD_MS      20U
#endif

#ifndef CANLOG_DQ_TELEMETRY_LPF_ALPHA
#define CANLOG_DQ_TELEMETRY_LPF_ALPHA  0.25f
#endif

#define CANLOG_DEG_TO_RAD           0.01745329251994329577f
#define CANLOG_RAD_TO_DEG           57.2957795130823208768f
#define CANLOG_2PI_OVER_3           2.09439510239319549231f

// =========================
// Internal CAN log state
// =========================

static uint8_t can_log_started = 0;

static uint32_t can_tx_counter = 0;
static uint32_t can_tx_error_counter = 0;

static uint32_t can_last_snapshot_tick = 0;

// =========================
// CAN pending scheduler
// =========================

#define CAN_PENDING_HALL            (1U << 0)
#define CAN_PENDING_CURRENT         (1U << 1)
#define CAN_PENDING_VOLTAGE         (1U << 2)
#define CAN_PENDING_DQ              (1U << 3)
#define CAN_PENDING_STATUS          (1U << 4)
#define CAN_PENDING_SETPOINT        (1U << 5)
#define CAN_PENDING_ANGLE_DEBUG     (1U << 6)
#define CAN_PENDING_PHASE_SMOOTH    (1U << 7)

#define CAN_PENDING_ALL             (CAN_PENDING_HALL    | \
                                     CAN_PENDING_CURRENT | \
                                     CAN_PENDING_VOLTAGE | \
                                     CAN_PENDING_DQ      | \
                                     CAN_PENDING_STATUS  | \
                                     CAN_PENDING_SETPOINT | \
                                     CAN_PENDING_ANGLE_DEBUG | \
                                     CAN_PENDING_PHASE_SMOOTH)

static uint8_t can_pending_mask = 0U;

// =========================
// Telemetry snapshot
// =========================

typedef struct {
    uint16_t seq;
    uint32_t tick_ms;

    int16_t rpm;
    int16_t theta_r_x100;
    int16_t theta_e_x100;
    uint8_t hall_state;
    int16_t open_loop_theta_e_x100;
    int16_t theta_error_x100;
    uint8_t hall_dir_code;

    int16_t ia_x100;
    int16_t ib_x100;
    int16_t ic_x100;

    uint16_t vbus_x10;
    uint16_t vphase_a_x10;
    uint16_t vphase_b_x10;
    uint16_t vphase_c_x10;

    int16_t vphase_a_smooth_x100;
    int16_t vphase_b_smooth_x100;
    int16_t vphase_c_smooth_x100;

    int16_t id_x100;
    int16_t iq_x100;
    uint16_t duty_x10000;

    uint8_t mode;
    uint8_t fault_code;
    uint8_t pwm_enabled;
    uint8_t gate_enabled;
    uint8_t fw_enabled;

    uint16_t max_current_x100;
    uint16_t fw_max_current_x100;
    int16_t target_erpm;
    uint16_t target_duty_x10000;

} can_telem_snapshot_t;

static can_telem_snapshot_t can_snapshot = {0};

static hall_transition_event_t can_pending_hall_event = {0};
static uint8_t can_hall_event_pending = 0U;

// =========================
// Live Expression debug data
// =========================

volatile uint8_t can_debug_started = 0;
volatile uint8_t can_debug_pending_mask = 0;
volatile uint8_t can_debug_tx_mailbox_free = 0;
volatile uint8_t can_debug_last_id_low = 0;

volatile uint32_t can_debug_tx_counter = 0;
volatile uint32_t can_debug_tx_error_counter = 0;
volatile uint32_t can_debug_mailbox_full_counter = 0;

volatile uint32_t can_debug_snapshot_counter = 0;
volatile uint32_t can_debug_snapshot_skip_counter = 0;

volatile uint32_t can_debug_sent_101 = 0;
volatile uint32_t can_debug_sent_102 = 0;
volatile uint32_t can_debug_sent_103 = 0;
volatile uint32_t can_debug_sent_104 = 0;
volatile uint32_t can_debug_sent_105 = 0;
volatile uint32_t can_debug_sent_106 = 0;
volatile uint32_t can_debug_sent_107 = 0;
volatile uint32_t can_debug_sent_108 = 0;
volatile uint32_t can_debug_sent_109 = 0;
volatile uint32_t can_debug_hall_event_overflow_count = 0;
volatile uint8_t can_debug_hall_event_pending = 0;

// =========================
// Passive D/Q convention debug state
// =========================
// Default keeps the previous behavior: CURR1/i1 -> A, CURR2/i3 -> C,
// positive Hall theta, and compile-time theta/current sign offsets.

static uint8_t can_dq_map = CANLOG_DQ_MAP_I1_A_I3_C;
static int8_t can_dq_theta_direction = 1;
static float can_dq_theta_offset_rad = FOC_THETA_OFFSET_RAD;
static int8_t can_dq_i1_sign = 0;
static int8_t can_dq_i3_sign = 0;

static uint8_t can_dq_telem_filter_ready = 0U;
static float can_id_telem_filter_a = 0.0f;
static float can_iq_telem_filter_a = 0.0f;

static void CANLog_FilterDQTelemetry(float id_in,
                                     float iq_in,
                                     float *id_out,
                                     float *iq_out);

// =========================
// Private helper: clamp
// =========================

static int16_t CANLog_ClampInt16(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }

    if (value < -32768) {
        return -32768;
    }

    return (int16_t)value;
}

static float CANLog_WrapDeg180(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }

    while (deg < -180.0f) {
        deg += 360.0f;
    }

    return deg;
}

static uint16_t CANLog_ClampUint16(int32_t value)
{
    if (value > 65535) {
        return 65535U;
    }

    if (value < 0) {
        return 0U;
    }

    return (uint16_t)value;
}

static int32_t CANLog_FloatToS32(float value, float scale)
{
    float scaled = value * scale;

    if (scaled >= 0.0f) {
        return (int32_t)(scaled + 0.5f);
    }

    return (int32_t)(scaled - 0.5f);
}

static void CANLog_FilterDQTelemetry(float id_in,
                                     float iq_in,
                                     float *id_out,
                                     float *iq_out)
{
    float alpha = CANLOG_DQ_TELEMETRY_LPF_ALPHA;

    if (alpha < 0.0f) {
        alpha = 0.0f;
    } else if (alpha > 1.0f) {
        alpha = 1.0f;
    }

    if (can_dq_telem_filter_ready == 0U) {
        can_id_telem_filter_a = id_in;
        can_iq_telem_filter_a = iq_in;
        can_dq_telem_filter_ready = 1U;
    } else {
        can_id_telem_filter_a += alpha * (id_in - can_id_telem_filter_a);
        can_iq_telem_filter_a += alpha * (iq_in - can_iq_telem_filter_a);
    }

    *id_out = can_id_telem_filter_a;
    *iq_out = can_iq_telem_filter_a;
}

static int8_t CANLog_SignFromFloat(float value)
{
    return (value < 0.0f) ? -1 : 1;
}

int8_t CANLog_GetDQI1Sign(void)
{
    if (can_dq_i1_sign == 0) {
        return CANLog_SignFromFloat(FOC_CURRENT_IA_SIGN);
    }

    return can_dq_i1_sign;
}

int8_t CANLog_GetDQI3Sign(void)
{
    if (can_dq_i3_sign == 0) {
        return CANLog_SignFromFloat(FOC_CURRENT_IC_SIGN);
    }

    return can_dq_i3_sign;
}

uint8_t CANLog_SetDQCurrentSigns(int8_t i1_sign, int8_t i3_sign)
{
    if ((i1_sign == 0) || (i3_sign == 0)) {
        return 0U;
    }

    can_dq_i1_sign = (i1_sign < 0) ? -1 : 1;
    can_dq_i3_sign = (i3_sign < 0) ? -1 : 1;

    return 1U;
}

uint8_t CANLog_SetDQMap(uint8_t map)
{
    if (map >= CANLOG_DQ_MAP_COUNT) {
        return 0U;
    }

    can_dq_map = map;
    return 1U;
}

uint8_t CANLog_GetDQMap(void)
{
    return can_dq_map;
}

const char *CANLog_GetDQMapName(uint8_t map)
{
    switch (map) {
        case CANLOG_DQ_MAP_I1_A_I3_C:
            return "i1=A,i3=C";

        case CANLOG_DQ_MAP_I3_A_I1_C:
            return "i3=A,i1=C";

        case CANLOG_DQ_MAP_I1_A_I3_B:
            return "i1=A,i3=B";

        case CANLOG_DQ_MAP_I3_A_I1_B:
            return "i3=A,i1=B";

        case CANLOG_DQ_MAP_I1_B_I3_C:
            return "i1=B,i3=C";

        case CANLOG_DQ_MAP_I3_B_I1_C:
            return "i3=B,i1=C";

        default:
            return "invalid";
    }
}

uint8_t CANLog_SetDQThetaDirection(int8_t direction)
{
    if (direction == 0) {
        return 0U;
    }

    can_dq_theta_direction = (direction < 0) ? -1 : 1;
    return 1U;
}

int8_t CANLog_GetDQThetaDirection(void)
{
    return can_dq_theta_direction;
}

void CANLog_SetDQThetaOffsetDeg(float offset_deg)
{
    can_dq_theta_offset_rad = offset_deg * CANLOG_DEG_TO_RAD;
}

float CANLog_GetDQThetaOffsetDeg(void)
{
    return can_dq_theta_offset_rad * CANLOG_RAD_TO_DEG;
}

static uint32_t CANLog_FloatToU32(float value, float scale)
{
    float scaled = value * scale;

    if (scaled <= 0.0f) {
        return 0U;
    }

    return (uint32_t)(scaled + 0.5f);
}

static void CANLog_GetTelemetryPhaseCurrents(const adc_sense_data_t *adc,
                                             float *ia,
                                             float *ib,
                                             float *ic)
{
    *ia = FOC_CURRENT_IA_SIGN * adc->i1_a;
    *ic = FOC_CURRENT_IC_SIGN * adc->i3_a;
    *ib = -(*ia + *ic);
}

static void CANLog_GetDQPhaseCurrents(const adc_sense_data_t *adc,
                                      float *ia,
                                      float *ib,
                                      float *ic)
{
    float i1 = (float)CANLog_GetDQI1Sign() * adc->i1_a;
    float i3 = (float)CANLog_GetDQI3Sign() * adc->i3_a;

    switch (can_dq_map) {
        case CANLOG_DQ_MAP_I3_A_I1_C:
            *ia = i3;
            *ic = i1;
            *ib = -(*ia + *ic);
            break;

        case CANLOG_DQ_MAP_I1_A_I3_B:
            *ia = i1;
            *ib = i3;
            *ic = -(*ia + *ib);
            break;

        case CANLOG_DQ_MAP_I3_A_I1_B:
            *ia = i3;
            *ib = i1;
            *ic = -(*ia + *ib);
            break;

        case CANLOG_DQ_MAP_I1_B_I3_C:
            *ib = i1;
            *ic = i3;
            *ia = -(*ib + *ic);
            break;

        case CANLOG_DQ_MAP_I3_B_I1_C:
            *ib = i3;
            *ic = i1;
            *ia = -(*ib + *ic);
            break;

        case CANLOG_DQ_MAP_I1_A_I3_C:
        default:
            *ia = i1;
            *ic = i3;
            *ib = -(*ia + *ic);
            break;
    }
}

static void CANLog_CalcPassiveDQ(const adc_sense_data_t *adc,
                                 const hall_sensor_data_t *hall,
                                 float ia,
                                 float ib,
                                 float ic,
                                 float *id,
                                 float *iq)
{
    *id = 0.0f;
    *iq = 0.0f;

    if ((adc->dma_started == 0U) ||
        (adc->current_calibrated == 0U) ||
        (hall->state_valid == 0U) ||
        (hall->angle_valid == 0U))
    {
        return;
    }

    float theta_e_rad = hall->theta_e_rad;
    if (can_dq_theta_direction < 0) {
        theta_e_rad = -theta_e_rad;
    }

    foc_alpha_beta_t i_ab = foc_clarke(ia, ib, ic);
    foc_dq_t i_dq = foc_park(i_ab, theta_e_rad + can_dq_theta_offset_rad);

    *id = i_dq.d;
    *iq = i_dq.q;
}

static uint8_t CANLog_ReadMode(void)
{
    if (phase2_fault_latched() || phase2_fault_active()) {
        return 4U; /* FAULT */
    }

    if (foc_control_is_running()) {
        return 2U; /* FOC */
    }

    if (open_loop_is_running()) {
        return 1U; /* OPEN */
    }

    if (phase2_get_inverter_state() == PHASE2_INV_ENABLED) {
        return 3U; /* STANDBY */
    }

    return 0U; /* IDLE */
}

// =========================
// Private helper: packing
// =========================

static void CANLog_PackU16(uint8_t *data, uint8_t index, uint16_t value)
{
    data[index]     = (uint8_t)(value & 0xFFU);
    data[index + 1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void CANLog_PackS16(uint8_t *data, uint8_t index, int16_t value)
{
    uint16_t raw = (uint16_t)value;

    data[index]     = (uint8_t)(raw & 0xFFU);
    data[index + 1] = (uint8_t)((raw >> 8) & 0xFFU);
}

static void CANLog_PackU32(uint8_t *data, uint8_t index, uint32_t value)
{
    data[index]     = (uint8_t)(value & 0xFFU);
    data[index + 1] = (uint8_t)((value >> 8) & 0xFFU);
    data[index + 2] = (uint8_t)((value >> 16) & 0xFFU);
    data[index + 3] = (uint8_t)((value >> 24) & 0xFFU);
}

// =========================
// Private helper: CAN send
// =========================
//
// Return:
// 1 = frame accepted into TX mailbox
// 0 = failed / mailbox full

static uint8_t CANLog_SendFrame(uint32_t std_id, const uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    uint32_t free_level = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    can_debug_tx_mailbox_free = (uint8_t)free_level;

    if (free_level == 0U) {
        can_tx_error_counter++;
        can_debug_tx_error_counter = can_tx_error_counter;
        can_debug_mailbox_full_counter++;
        return 0U;
    }

    tx_header.StdId = std_id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t *)data, &tx_mailbox) == HAL_OK) {
        can_tx_counter++;

        can_debug_tx_counter = can_tx_counter;
        can_debug_last_id_low = (uint8_t)(std_id & 0xFFU);

        return 1U;
    }

    can_tx_error_counter++;
    can_debug_tx_error_counter = can_tx_error_counter;

    return 0U;
}

// =========================
// Snapshot update
// =========================

static void CANLog_UpdateSnapshot(void)
{
    adc_sense_data_t adc = adc_sense_get_data();
    hall_sensor_data_t hall = Hall_GetData();
    uint8_t pwm_enabled = phase2_pwm_is_enabled();
    uint8_t gate_enabled = (HAL_GPIO_ReadPin(EN_GATE_GPIO_Port, EN_GATE_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    float actual_duty = open_loop_is_running() ? open_loop_get_modulation() : 0.0f;
    float target_duty = open_loop_get_target_modulation();
    foc_control_status_t foc_status = foc_control_get_status();
    float ia = 0.0f;
    float ib = 0.0f;
    float ic = 0.0f;
    float dq_ia = 0.0f;
    float dq_ib = 0.0f;
    float dq_ic = 0.0f;
    float passive_id = 0.0f;
    float passive_iq = 0.0f;
    float telem_id = 0.0f;
    float telem_iq = 0.0f;

    CANLog_GetTelemetryPhaseCurrents(&adc, &ia, &ib, &ic);
    CANLog_GetDQPhaseCurrents(&adc, &dq_ia, &dq_ib, &dq_ic);
    CANLog_CalcPassiveDQ(&adc, &hall, dq_ia, dq_ib, dq_ic, &passive_id, &passive_iq);

    if (foc_status.running != 0U) {
        passive_id = foc_status.id_a;
        passive_iq = foc_status.iq_a;
        actual_duty = foc_status.modulation;
        target_duty = 0.0f;
    }

    if (!open_loop_is_running() && pwm_enabled) {
        actual_duty = phase2_get_test_duty();
        target_duty = phase2_get_test_duty();
    }

    float smooth_va_v = 0.0f;
    float smooth_vb_v = 0.0f;
    float smooth_vc_v = 0.0f;
    if (foc_status.running != 0U) {
        smooth_va_v = foc_status.vphase_u_smooth_v;
        smooth_vb_v = foc_status.vphase_v_smooth_v;
        smooth_vc_v = foc_status.vphase_w_smooth_v;
    } else if (open_loop_is_running() && adc.dma_started) {
        float theta = open_loop_get_theta_e_rad();
        float amplitude_v = adc.vbus_v * actual_duty;
        smooth_va_v = amplitude_v * sinf(theta);
        smooth_vb_v = amplitude_v * sinf(theta - CANLOG_2PI_OVER_3);
        smooth_vc_v = amplitude_v * sinf(theta + CANLOG_2PI_OVER_3);
    }

    can_snapshot.seq++;
    can_snapshot.tick_ms = HAL_GetTick();

    can_snapshot.rpm =
        CANLog_ClampInt16(CANLog_FloatToS32(hall.mechanical_rpm, 1.0f));

    can_snapshot.theta_r_x100 =
        CANLog_ClampInt16(CANLog_FloatToS32(hall.theta_r_deg, 100.0f));

    can_snapshot.theta_e_x100 =
        CANLog_ClampInt16(CANLog_FloatToS32(CANLog_WrapDeg180(hall.theta_e_deg), 100.0f));

    can_snapshot.hall_state = hall.hall_state;

    float open_loop_theta_deg = CANLog_WrapDeg180(open_loop_get_theta_e_rad() * CANLOG_RAD_TO_DEG);
    float hall_theta_deg = CANLog_WrapDeg180(hall.theta_e_deg);
    float theta_error_deg = CANLog_WrapDeg180(open_loop_theta_deg - hall_theta_deg);

    can_snapshot.open_loop_theta_e_x100 =
        CANLog_ClampInt16(CANLog_FloatToS32(open_loop_theta_deg, 100.0f));

    can_snapshot.theta_error_x100 =
        CANLog_ClampInt16(CANLog_FloatToS32(theta_error_deg, 100.0f));

    if (hall.direction > 0) {
        can_snapshot.hall_dir_code = 2U;
    } else if (hall.direction < 0) {
        can_snapshot.hall_dir_code = 0U;
    } else {
        can_snapshot.hall_dir_code = 1U;
    }

    if (adc.dma_started) {
        can_snapshot.ia_x100 =
            CANLog_ClampInt16(CANLog_FloatToS32(ia, 100.0f));

        can_snapshot.ib_x100 =
            CANLog_ClampInt16(CANLog_FloatToS32(ib, 100.0f));

        can_snapshot.ic_x100 =
            CANLog_ClampInt16(CANLog_FloatToS32(ic, 100.0f));

        can_snapshot.vbus_x10 =
            CANLog_ClampUint16((int32_t)CANLog_FloatToU32(adc.vbus_v, 10.0f));

        can_snapshot.vphase_a_x10 =
            CANLog_ClampUint16((int32_t)CANLog_FloatToU32(adc.vphase_a_v, 10.0f));

        can_snapshot.vphase_b_x10 =
            CANLog_ClampUint16((int32_t)CANLog_FloatToU32(adc.vphase_b_v, 10.0f));

        can_snapshot.vphase_c_x10 =
            CANLog_ClampUint16((int32_t)CANLog_FloatToU32(adc.vphase_c_v, 10.0f));
    } else {
        can_snapshot.ia_x100 = 0;
        can_snapshot.ib_x100 = 0;
        can_snapshot.ic_x100 = 0;
        can_snapshot.vbus_x10 = 0U;
        can_snapshot.vphase_a_x10 = 0U;
        can_snapshot.vphase_b_x10 = 0U;
        can_snapshot.vphase_c_x10 = 0U;
    }

    can_snapshot.vphase_a_smooth_x100 =
        CANLog_ClampInt16(CANLog_FloatToS32(smooth_va_v, 100.0f));
    can_snapshot.vphase_b_smooth_x100 =
        CANLog_ClampInt16(CANLog_FloatToS32(smooth_vb_v, 100.0f));
    can_snapshot.vphase_c_smooth_x100 =
        CANLog_ClampInt16(CANLog_FloatToS32(smooth_vc_v, 100.0f));

    CANLog_FilterDQTelemetry(passive_id, passive_iq, &telem_id, &telem_iq);

    can_snapshot.id_x100 =
        CANLog_ClampInt16(CANLog_FloatToS32(telem_id, 100.0f));

    can_snapshot.iq_x100 =
        CANLog_ClampInt16(CANLog_FloatToS32(telem_iq, 100.0f));

    can_snapshot.duty_x10000 =
        CANLog_ClampUint16((int32_t)CANLog_FloatToU32(actual_duty, 10000.0f));

    can_snapshot.mode = CANLog_ReadMode();
    can_snapshot.fault_code = (can_snapshot.mode == 4U) ? 6U : 0U;
    can_snapshot.pwm_enabled = pwm_enabled;
    can_snapshot.gate_enabled = gate_enabled;
    can_snapshot.fw_enabled = 0U;             // TODO: add FW enabled getter later

    can_snapshot.max_current_x100 =
        CANLog_ClampUint16(0);                // TODO: add max current getter later

    can_snapshot.fw_max_current_x100 =
        CANLog_ClampUint16(0);                // TODO: add FW max current getter later

    can_snapshot.target_erpm =
        CANLog_ClampInt16(CANLog_FloatToS32(open_loop_get_target_freq_e_hz(), 60.0f));

    can_snapshot.target_duty_x10000 =
        CANLog_ClampUint16((int32_t)CANLog_FloatToU32(target_duty, 10000.0f));

    can_debug_snapshot_counter++;
}

// =========================
// CAN initialization
// =========================

void CANLog_Start(void)
{
    if (can_log_started) {
        return;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        can_tx_error_counter++;
        can_debug_tx_error_counter = can_tx_error_counter;
        return;
    }

    can_log_started = 1U;

    uint32_t now = HAL_GetTick();
    can_last_snapshot_tick = now;

    can_pending_mask = 0U;
    can_hall_event_pending = 0U;

    CANLog_UpdateSnapshot();

    can_debug_started = 1U;
    can_debug_pending_mask = can_pending_mask;
    can_debug_hall_event_pending = can_hall_event_pending;
    can_debug_hall_event_overflow_count = Hall_GetTransitionEventOverflowCount();
    can_debug_tx_mailbox_free =
        (uint8_t)HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
}

// =========================
// Private send from snapshot
// =========================

static uint8_t CANLog_SendHallFromSnapshot(void)
{
    uint8_t data[8] = {0};

    CANLog_PackS16(data, 0, can_snapshot.rpm);
    CANLog_PackS16(data, 2, can_snapshot.theta_r_x100);
    CANLog_PackS16(data, 4, can_snapshot.theta_e_x100);

    data[6] = can_snapshot.hall_state;
    data[7] = (uint8_t)(can_snapshot.seq & 0xFFU);

    return CANLog_SendFrame(CAN_ID_HALL, data);
}

static uint8_t CANLog_SendCurrentFromSnapshot(void)
{
    uint8_t data[8] = {0};

    CANLog_PackS16(data, 0, can_snapshot.ia_x100);
    CANLog_PackS16(data, 2, can_snapshot.ib_x100);
    CANLog_PackS16(data, 4, can_snapshot.ic_x100);

    CANLog_PackU16(data, 6, can_snapshot.seq);

    return CANLog_SendFrame(CAN_ID_CURRENT, data);
}

static uint8_t CANLog_SendVoltageFromSnapshot(void)
{
    uint8_t data[8] = {0};

    CANLog_PackU16(data, 0, can_snapshot.vbus_x10);
    CANLog_PackU16(data, 2, can_snapshot.vphase_a_x10);
    CANLog_PackU16(data, 4, can_snapshot.vphase_b_x10);
    CANLog_PackU16(data, 6, can_snapshot.vphase_c_x10);

    return CANLog_SendFrame(CAN_ID_VOLTAGE, data);
}

static uint8_t CANLog_SendPhaseSmoothFromSnapshot(void)
{
    uint8_t data[8] = {0};

    CANLog_PackS16(data, 0, can_snapshot.vphase_a_smooth_x100);
    CANLog_PackS16(data, 2, can_snapshot.vphase_b_smooth_x100);
    CANLog_PackS16(data, 4, can_snapshot.vphase_c_smooth_x100);
    CANLog_PackU16(data, 6, can_snapshot.seq);

    return CANLog_SendFrame(CAN_ID_PHASE_SMOOTH, data);
}

static uint8_t CANLog_SendDQFromSnapshot(void)
{
    uint8_t data[8] = {0};

    CANLog_PackS16(data, 0, can_snapshot.id_x100);
    CANLog_PackS16(data, 2, can_snapshot.iq_x100);
    CANLog_PackU16(data, 4, can_snapshot.duty_x10000);

    CANLog_PackU16(data, 6, can_snapshot.seq);

    return CANLog_SendFrame(CAN_ID_DQ, data);
}

static uint8_t CANLog_SendStatusFromSnapshot(void)
{
    uint8_t data[8] = {0};

    data[0] = can_snapshot.mode;
    data[1] = can_snapshot.fault_code;
    data[2] = can_snapshot.pwm_enabled;
    data[3] = can_snapshot.gate_enabled;
    data[4] = can_snapshot.fw_enabled;
    data[5] = 0U;                            // TODO: add FW active/status byte later if needed
    CANLog_PackU16(data, 6, can_snapshot.seq);

    return CANLog_SendFrame(CAN_ID_STATUS, data);
}

static uint8_t CANLog_SendSetpointFromSnapshot(void)
{
    uint8_t data[8] = {0};

    CANLog_PackU16(data, 0, can_snapshot.max_current_x100);
    CANLog_PackU16(data, 2, can_snapshot.fw_max_current_x100);
    CANLog_PackS16(data, 4, can_snapshot.target_erpm);
    CANLog_PackU16(data, 6, can_snapshot.target_duty_x10000);

    return CANLog_SendFrame(CAN_ID_SETPOINT, data);
}

static uint8_t CANLog_SendAngleDebugFromSnapshot(void)
{
    uint8_t data[8] = {0};

    CANLog_PackS16(data, 0, can_snapshot.open_loop_theta_e_x100);
    CANLog_PackS16(data, 2, can_snapshot.theta_e_x100);
    CANLog_PackS16(data, 4, can_snapshot.theta_error_x100);
    data[6] = can_snapshot.hall_state;
    data[7] = can_snapshot.hall_dir_code;

    return CANLog_SendFrame(CAN_ID_ANGLE_DEBUG, data);
}

static uint8_t CANLog_SendHallEventFrame(const hall_transition_event_t *event)
{
    uint8_t data[8] = {0};
    uint8_t direction_code;
    uint8_t sequence_mod64;
    int16_t open_loop_theta_x100;

    if (event->direction > 0) {
        direction_code = 2U;
    } else if (event->direction < 0) {
        direction_code = 0U;
    } else {
        direction_code = 1U;
    }

    open_loop_theta_x100 = CANLog_ClampInt16(
        CANLog_FloatToS32(CANLog_WrapDeg180(event->open_loop_theta_e_rad * CANLOG_RAD_TO_DEG), 100.0f));

    sequence_mod64 = (uint8_t)(event->sequence & 0x3FU);

    CANLog_PackU32(data, 0, event->timestamp_us);
    CANLog_PackS16(data, 4, open_loop_theta_x100);
    data[6] = (uint8_t)(((event->previous_state & 0x0FU) << 4) |
                        (event->hall_state & 0x0FU));
    data[7] = (uint8_t)((sequence_mod64 << 2) | (direction_code & 0x03U));

    return CANLog_SendFrame(CAN_ID_HALL_EVENT, data);
}

static uint8_t CANLog_ServiceHallEventOneFrame(void)
{
    if (can_hall_event_pending == 0U) {
        if (Hall_PopTransitionEvent(&can_pending_hall_event) == 0U) {
            return 0U;
        }

        can_hall_event_pending = 1U;
    }

    if (CANLog_SendHallEventFrame(&can_pending_hall_event)) {
        can_hall_event_pending = 0U;
        can_debug_sent_108++;
    }

    return 1U;
}

// =========================
// Pending frame scheduler
// =========================

static void CANLog_ServicePendingOneFrame(void)
{
    /*
     * Send at most one pending frame per CANLog_Task() call.
     *
     * Priority:
     * 1. STATUS      : mode/fault/status
     * 2. SETPOINT    : UART command verification
     * 3. DQ          : PWM duty / future FOC DQ
     * 4. HALL        : Hall/speed
     * 5. HALL/OL DBG : commanded open-loop angle vs Hall angle
     * 6. CURRENT     : phase current
     * 7. VOLTAGE     : raw sampled voltages
     * 8. PHASE SMOOTH: commanded sine phase voltage for plotting
     */

    if ((can_pending_mask & CAN_PENDING_STATUS) != 0U) {
        if (CANLog_SendStatusFromSnapshot()) {
            can_pending_mask &= (uint8_t)~CAN_PENDING_STATUS;
            can_debug_sent_105++;
        }
        return;
    }

    if ((can_pending_mask & CAN_PENDING_SETPOINT) != 0U) {
        if (CANLog_SendSetpointFromSnapshot()) {
            can_pending_mask &= (uint8_t)~CAN_PENDING_SETPOINT;
            can_debug_sent_106++;
        }
        return;
    }

    if ((can_pending_mask & CAN_PENDING_DQ) != 0U) {
        if (CANLog_SendDQFromSnapshot()) {
            can_pending_mask &= (uint8_t)~CAN_PENDING_DQ;
            can_debug_sent_104++;
        }
        return;
    }

    if ((can_pending_mask & CAN_PENDING_HALL) != 0U) {
        if (CANLog_SendHallFromSnapshot()) {
            can_pending_mask &= (uint8_t)~CAN_PENDING_HALL;
            can_debug_sent_101++;
        }
        return;
    }

    if ((can_pending_mask & CAN_PENDING_ANGLE_DEBUG) != 0U) {
        if (CANLog_SendAngleDebugFromSnapshot()) {
            can_pending_mask &= (uint8_t)~CAN_PENDING_ANGLE_DEBUG;
            can_debug_sent_107++;
        }
        return;
    }

    if ((can_pending_mask & CAN_PENDING_CURRENT) != 0U) {
        if (CANLog_SendCurrentFromSnapshot()) {
            can_pending_mask &= (uint8_t)~CAN_PENDING_CURRENT;
            can_debug_sent_102++;
        }
        return;
    }

    if ((can_pending_mask & CAN_PENDING_VOLTAGE) != 0U) {
        if (CANLog_SendVoltageFromSnapshot()) {
            can_pending_mask &= (uint8_t)~CAN_PENDING_VOLTAGE;
            can_debug_sent_103++;
        }
        return;
    }

    if ((can_pending_mask & CAN_PENDING_PHASE_SMOOTH) != 0U) {
        if (CANLog_SendPhaseSmoothFromSnapshot()) {
            can_pending_mask &= (uint8_t)~CAN_PENDING_PHASE_SMOOTH;
            can_debug_sent_109++;
        }
        return;
    }
}

// =========================
// Periodic CAN task
// =========================

void CANLog_Task(void)
{
    if (!can_log_started) {
        return;
    }

    uint32_t now = HAL_GetTick();

    /*
     * Only create a new snapshot when the previous packet has fully left
     * the pending scheduler.
     *
     * This keeps all telemetry frames in one packet coming from the same
     * synchronized snapshot.
     */

    if ((now - can_last_snapshot_tick) >= CAN_SNAPSHOT_PERIOD_MS) {
        can_last_snapshot_tick = now;

        if (can_pending_mask == 0U) {
            CANLog_UpdateSnapshot();
            can_pending_mask = CAN_PENDING_ALL;
        } else {
            /*
             * Previous snapshot is still pending.
             * Do not overwrite snapshot data, otherwise packet consistency is lost.
             */
            can_debug_snapshot_skip_counter++;
        }
    }

    if (CANLog_ServiceHallEventOneFrame() == 0U) {
        CANLog_ServicePendingOneFrame();
    }

    can_debug_pending_mask = can_pending_mask;
    can_debug_hall_event_pending = can_hall_event_pending;
    can_debug_hall_event_overflow_count = Hall_GetTransitionEventOverflowCount();
    can_debug_tx_mailbox_free =
        (uint8_t)HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
}

// =========================
// Manual telemetry transmit functions
// =========================
//
// These are kept as void to avoid changing can_log.h.
// They send the latest synchronized snapshot.

void CANLog_SendHall(void)
{
    (void)CANLog_SendHallFromSnapshot();
}

void CANLog_SendCurrent(void)
{
    (void)CANLog_SendCurrentFromSnapshot();
}

void CANLog_SendVoltage(void)
{
    (void)CANLog_SendVoltageFromSnapshot();
}

void CANLog_SendPhaseSmooth(void)
{
    (void)CANLog_SendPhaseSmoothFromSnapshot();
}

void CANLog_SendDQ(void)
{
    (void)CANLog_SendDQFromSnapshot();
}

void CANLog_SendStatus(void)
{
    (void)CANLog_SendStatusFromSnapshot();
}

void CANLog_SendSetpoint(void)
{
    (void)CANLog_SendSetpointFromSnapshot();
}

void CANLog_SendAngleDebug(void)
{
    (void)CANLog_SendAngleDebugFromSnapshot();
}

// =========================
// Utility / status
// =========================

uint8_t CANLog_IsStarted(void)
{
    return can_log_started;
}

uint32_t CANLog_GetTxCounter(void)
{
    return can_tx_counter;
}

uint32_t CANLog_GetTxErrorCounter(void)
{
    return can_tx_error_counter;
}
