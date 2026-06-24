/*
 * hall_sensor.c
 *
 * Hall EXTI capture and angle reconstruction based on:
 * "Accurate Angle Representation From Misplaced Hall-Effect Switch Sensors".
 */

#include "hall_sensor.h"
#include "hw_conf.h"
#include "main.h"
#include "open_loop.h"
#include "vesc_uart.h"

#include <stdio.h>
#include <string.h>

extern TIM_HandleTypeDef htim5;

#define HALL_PI                 3.14159265358979323846f
#define HALL_TWO_PI             6.28318530717958647692f
#define HALL_PI_OVER_3          1.04719755119659774615f
#define HALL_RAD_TO_DEG         57.2957795130823208768f
#define HALL_RAD_S_TO_RPM       9.549296585513720146f

#define HALL_INVALID_INDEX      0xFFU
#define HALL_MIN_EDGE_US        5U
#define HALL_TIMEOUT_MIN_US     50000U

#define HALL_SNAP_NONE          0U
#define HALL_SNAP_SECTOR_MID    1U
#define HALL_SNAP_EDGE          2U

#define HALL_TRANSITION_EVENT_QUEUE_LEN  16U

/* H1..H6 are defined in the user's verified positive Hall sequence order.
 * Raw state 4 is the 0-degree sector center and spans 330..30 degrees.
 */
#define HALL_H1_INDEX           0U
#define HALL_H2_INDEX           1U
#define HALL_H3_INDEX           2U
#define HALL_H4_INDEX           3U
#define HALL_H5_INDEX           4U
#define HALL_H6_INDEX           5U

/* The paper corrects at H5 for clockwise and H6 for anticlockwise. */
#define HALL_POSITIVE_DIRECTION_IS_ANTICLOCKWISE  1U

static const uint8_t s_hall_positive_sequence[6] = {
    0b100, 0b110, 0b010, 0b011, 0b001, 0b101
};

/* Calibrated from foc_hall_events_60.csv and foc_hall_events_-60.csv.
 * Event open-loop theta is converted to voltage-vector angle by subtracting
 * 90 electrical degrees, then positive/negative runs are averaged to cancel
 * load angle.
 */
static const float s_theta_hall_positive_rad[6] = {
    5.8317604294f, /* 334.135 deg */
    0.5606246994f, /*  32.121 deg */
    1.6083388578f, /*  92.151 deg */
    2.6838574775f, /* 153.774 deg */
    3.7025492185f, /* 212.140 deg */
    4.7477887199f  /* 272.028 deg */
};

static const float s_theta_hall_negative_rad[6] = {
    0.5606246994f, /*  32.121 deg */
    1.6083388578f, /*  92.151 deg */
    2.6838574775f, /* 153.774 deg */
    3.7025492185f, /* 212.140 deg */
    4.7477887199f, /* 272.028 deg */
    5.8317604294f  /* 334.135 deg */
};

static const float s_theta_hall_mid_rad[6] = {
    0.0545999108f, /*   3.128 deg */
    1.0844817786f, /*  62.136 deg */
    2.1460981676f, /* 122.962 deg */
    3.1932033480f, /* 182.957 deg */
    4.2251689692f, /* 242.084 deg */
    5.2897745747f  /* 303.082 deg */
};

static hall_sensor_data_t s_hall;

static uint8_t s_initialized;
static uint8_t s_hall_index;

static uint32_t s_dt_us[3];
static uint8_t s_dt_pos;
static uint8_t s_dt_count;
static uint32_t s_valid_transition_count;
static uint32_t s_last_angle_update_us;
static uint32_t s_speed_timeout_us;

static uint8_t s_pending_speed_update;
static uint8_t s_pending_correction;
static uint8_t s_pending_angle_snap;
static uint8_t s_pending_angle_index;
static uint8_t s_pending_correction_index;
static int8_t s_pending_angle_direction;
static int8_t s_pending_correction_direction;

static hall_transition_event_t s_event_queue[HALL_TRANSITION_EVENT_QUEUE_LEN];
static uint8_t s_event_head;
static uint8_t s_event_tail;
static uint32_t s_event_sequence;
static uint32_t s_event_overflow_count;

static uint8_t Hall_PinIsHall(uint16_t GPIO_Pin);
static uint8_t Hall_StateToIndex(uint8_t state);
static int8_t Hall_TransitionDirection(uint8_t old_index, uint8_t new_index);
static uint8_t Hall_IsCorrectionEdge(uint8_t new_index, int8_t direction);
static void Hall_ResetTimingLocked(void);
static void Hall_QueueAngleSnapLocked(uint8_t mode, uint8_t index, int8_t direction);
static void Hall_QueueTransitionEventLocked(uint8_t previous_state,
                                            uint8_t hall_state,
                                            int8_t direction,
                                            uint32_t timestamp_us);
static void Hall_ResyncLocked(uint8_t state, uint8_t index, uint32_t timestamp_us, uint8_t snap_mid);
static void Hall_AddDtLocked(uint32_t dt_us);
static float Hall_SectorMidAngleRad(uint8_t index);
static float Hall_EdgeAngleRad(uint8_t index, int8_t direction);
static float Hall_Wrap0To2Pi(float angle_rad);
static float Hall_WrapMinusPiToPi(float angle_rad);
static uint32_t Hall_TimeoutFromDtSum(uint32_t dt_sum_us);
static void Hall_UpdateDerived(void);

uint8_t Hall_ReadState(void)
{
    uint8_t h1 = HAL_GPIO_ReadPin(HALL1_GPIO_Port, HALL1_Pin) ? 1U : 0U;
    uint8_t h2 = HAL_GPIO_ReadPin(HALL2_GPIO_Port, HALL2_Pin) ? 1U : 0U;
    uint8_t h3 = HAL_GPIO_ReadPin(HALL3_GPIO_Port, HALL3_Pin) ? 1U : 0U;

    return (uint8_t)((h1 << 2) | (h2 << 1) | h3);
}

uint8_t Hall_IsValidState(uint8_t state)
{
    return (Hall_StateToIndex(state) != HALL_INVALID_INDEX) ? 1U : 0U;
}

const char* Hall_StateName(uint8_t state)
{
    return Hall_IsValidState(state) ? "VALID" : "INVALID";
}

void Hall_Init(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    memset(&s_hall, 0, sizeof(s_hall));
    memset(s_dt_us, 0, sizeof(s_dt_us));

    s_initialized = 1U;
    s_hall_index = HALL_INVALID_INDEX;
    s_dt_pos = 0U;
    s_dt_count = 0U;
    s_valid_transition_count = 0U;
    s_pending_speed_update = 0U;
    s_pending_correction = 0U;
    s_pending_angle_snap = HALL_SNAP_NONE;
    s_pending_angle_index = HALL_INVALID_INDEX;
    s_pending_correction_index = HALL_INVALID_INDEX;
    s_pending_angle_direction = 0;
    s_pending_correction_direction = 0;
    s_speed_timeout_us = HALL_TIMEOUT_MIN_US;
    s_event_head = 0U;
    s_event_tail = 0U;
    s_event_sequence = 0U;
    s_event_overflow_count = 0U;

    uint32_t now_us = __HAL_TIM_GET_COUNTER(&htim5);
    s_last_angle_update_us = now_us;

    uint8_t state = Hall_ReadState();
    uint8_t index = Hall_StateToIndex(state);

    s_hall.hall_state = state;
    s_hall.previous_state = state;
    s_hall.last_edge_us = now_us;

    if (index != HALL_INVALID_INDEX)
    {
        s_hall_index = index;
        s_hall.state_valid = 1U;
        s_hall.angle_valid = 1U;
        s_hall.theta_e_rad = Hall_SectorMidAngleRad(index);
        Hall_UpdateDerived();
    }

    if (primask == 0U)
    {
        __enable_irq();
    }
}

void Hall_EXTI_Callback(uint16_t GPIO_Pin, uint32_t timestamp_us)
{
    if (!Hall_PinIsHall(GPIO_Pin))
    {
        return;
    }

    if (s_initialized == 0U)
    {
        return;
    }

    uint8_t state = Hall_ReadState();
    uint8_t new_index = Hall_StateToIndex(state);

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (new_index == HALL_INVALID_INDEX)
    {
        s_hall.invalid_state_count++;
        s_hall.state_valid = 0U;
        s_hall.speed_valid = 0U;
        s_hall.omega_e_rad_s = 0.0f;
        s_hall.omega_corr_rad_s = 0.0f;
        s_hall.hall_state = state;

        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }

    if (s_hall.state_valid == 0U)
    {
        Hall_ResyncLocked(state, new_index, timestamp_us, 1U);

        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }

    if (state == s_hall.hall_state)
    {
        s_hall.duplicate_edge_count++;

        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }

    uint8_t old_index = s_hall_index;
    uint8_t previous_state = s_hall.hall_state;
    int8_t direction = Hall_TransitionDirection(old_index, new_index);

    if (direction == 0)
    {
        s_hall.invalid_transition_count++;
        Hall_ResyncLocked(state, new_index, timestamp_us, 1U);

        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }

    uint32_t dt_us = timestamp_us - s_hall.last_edge_us;
    if (dt_us < HALL_MIN_EDGE_US)
    {
        s_hall.glitch_count++;
        Hall_ResyncLocked(state, new_index, timestamp_us, 1U);

        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }

    if ((s_hall.direction != 0) && (s_hall.direction != direction))
    {
        s_hall.previous_state = previous_state;
        s_hall.hall_state = state;
        s_hall.state_valid = 1U;
        s_hall.speed_valid = 0U;
        s_hall.direction = direction;
        s_hall.last_edge_us = timestamp_us;
        s_hall_index = new_index;
        s_hall.edge_count++;
        s_hall.omega_e_rad_s = 0.0f;
        s_hall.omega_corr_rad_s = 0.0f;
        Hall_ResetTimingLocked();
        Hall_QueueAngleSnapLocked(HALL_SNAP_EDGE, new_index, direction);
        Hall_QueueTransitionEventLocked(previous_state, state, direction, timestamp_us);

        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }

    Hall_AddDtLocked(dt_us);

    s_hall.previous_state = previous_state;
    s_hall.hall_state = state;
    s_hall.state_valid = 1U;
    s_hall.direction = direction;
    s_hall.last_edge_us = timestamp_us;
    s_hall_index = new_index;
    s_hall.edge_count++;
    s_valid_transition_count++;

    if (s_dt_count >= 3U)
    {
        s_pending_speed_update = 1U;
    }

    if ((s_valid_transition_count <= 6U) || (s_hall.speed_valid == 0U))
    {
        Hall_QueueAngleSnapLocked(HALL_SNAP_EDGE, new_index, direction);
    }

    if ((s_valid_transition_count >= 6U) && Hall_IsCorrectionEdge(new_index, direction))
    {
        s_pending_correction = 1U;
        s_pending_correction_index = new_index;
        s_pending_correction_direction = direction;
    }

    Hall_QueueTransitionEventLocked(previous_state, state, direction, timestamp_us);

    if (primask == 0U)
    {
        __enable_irq();
    }
}

void Hall_ControlTick(void)
{
    if (s_initialized == 0U)
    {
        return;
    }

    uint32_t now_us = __HAL_TIM_GET_COUNTER(&htim5);

    if (s_last_angle_update_us == 0U)
    {
        s_last_angle_update_us = now_us;
    }

    if (s_hall.state_valid == 0U)
    {
        s_last_angle_update_us = now_us;
        s_hall.mechanical_rpm = 0.0f;
        return;
    }

    if (s_pending_angle_snap != HALL_SNAP_NONE)
    {
        if (s_pending_angle_snap == HALL_SNAP_EDGE)
        {
            s_hall.theta_e_rad = Hall_EdgeAngleRad(s_pending_angle_index, s_pending_angle_direction);
        }
        else
        {
            s_hall.theta_e_rad = Hall_SectorMidAngleRad(s_pending_angle_index);
        }

        s_hall.angle_valid = 1U;
        s_pending_angle_snap = HALL_SNAP_NONE;
        s_last_angle_update_us = now_us;
    }

    if (s_pending_speed_update != 0U)
    {
        if ((s_hall.dt_sum_us > 0U) && (s_hall.direction != 0))
        {
            float sum_dt_s = (float)s_hall.dt_sum_us * 0.000001f;
            s_hall.omega_e_rad_s = ((float)s_hall.direction * HALL_PI) / sum_dt_s;
            s_speed_timeout_us = Hall_TimeoutFromDtSum(s_hall.dt_sum_us);
            s_hall.speed_valid = 1U;

            if (s_valid_transition_count >= 6U)
            {
                s_hall.angle_valid = 1U;
            }
        }

        s_pending_speed_update = 0U;
    }

    if (s_pending_correction != 0U)
    {
        if ((s_hall.speed_valid != 0U) && (s_hall.angle_valid != 0U) && (s_hall.dt_sum_us > 0U))
        {
            float target = Hall_EdgeAngleRad(s_pending_correction_index, s_pending_correction_direction);
            float err = Hall_WrapMinusPiToPi(target - s_hall.theta_e_rad);
            float sum_dt_s = (float)s_hall.dt_sum_us * 0.000001f;
            s_hall.omega_corr_rad_s = (err / sum_dt_s) * 0.5f;
        }

        s_pending_correction = 0U;
    }

    uint32_t dt_us = now_us - s_last_angle_update_us;
    if (dt_us > 0U)
    {
        s_last_angle_update_us = now_us;

        if (s_hall.speed_valid != 0U)
        {
            uint32_t since_edge_us = now_us - s_hall.last_edge_us;
            if (since_edge_us > s_speed_timeout_us)
            {
                s_hall.speed_valid = 0U;
                s_hall.omega_e_rad_s = 0.0f;
                s_hall.omega_corr_rad_s = 0.0f;
                s_hall.mechanical_rpm = 0.0f;
            }
            else if (s_hall.angle_valid != 0U)
            {
                float dt_s = (float)dt_us * 0.000001f;
                s_hall.theta_e_rad += (s_hall.omega_e_rad_s + s_hall.omega_corr_rad_s) * dt_s;
                s_hall.theta_e_rad = Hall_Wrap0To2Pi(s_hall.theta_e_rad);
            }
        }
    }

    Hall_UpdateDerived();
}

hall_sensor_data_t Hall_GetData(void)
{
    hall_sensor_data_t data;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    data = s_hall;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return data;
}

hall_sensor_data_t Hall_GetDataFromISR(void)
{
    return s_hall;
}

uint8_t Hall_PopTransitionEvent(hall_transition_event_t *event)
{
    if (event == 0)
    {
        return 0U;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (s_event_tail == s_event_head)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }

        return 0U;
    }

    *event = s_event_queue[s_event_tail];

    s_event_tail++;
    if (s_event_tail >= HALL_TRANSITION_EVENT_QUEUE_LEN)
    {
        s_event_tail = 0U;
    }

    if (primask == 0U)
    {
        __enable_irq();
    }

    return 1U;
}

uint32_t Hall_GetTransitionEventOverflowCount(void)
{
    uint32_t count;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    count = s_event_overflow_count;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return count;
}

void Hall_PrintState(void)
{
    char msg[160];
    hall_sensor_data_t hall = Hall_GetData();

    snprintf(msg, sizeof(msg),
             "Hall: %u%u%u | state=%u | %s | dir=%d | rpm=%ld | theta_e_x100=%ld\r\n",
             (hall.hall_state >> 2) & 1U,
             (hall.hall_state >> 1) & 1U,
             hall.hall_state & 1U,
             hall.hall_state,
             Hall_StateName(hall.hall_state),
             (int)hall.direction,
             (long)(hall.mechanical_rpm >= 0.0f ? hall.mechanical_rpm + 0.5f : hall.mechanical_rpm - 0.5f),
             (long)(hall.theta_e_deg * 100.0f));

    uart_print(msg);
}

static uint8_t Hall_PinIsHall(uint16_t GPIO_Pin)
{
    return ((GPIO_Pin == HALL1_Pin) ||
            (GPIO_Pin == HALL2_Pin) ||
            (GPIO_Pin == HALL3_Pin)) ? 1U : 0U;
}

static uint8_t Hall_StateToIndex(uint8_t state)
{
    for (uint8_t i = 0U; i < 6U; i++)
    {
        if (s_hall_positive_sequence[i] == state)
        {
            return i;
        }
    }

    return HALL_INVALID_INDEX;
}

static int8_t Hall_TransitionDirection(uint8_t old_index, uint8_t new_index)
{
    if ((old_index >= 6U) || (new_index >= 6U))
    {
        return 0;
    }

    if (((old_index + 1U) % 6U) == new_index)
    {
        return 1;
    }

    if (((old_index + 5U) % 6U) == new_index)
    {
        return -1;
    }

    return 0;
}

static uint8_t Hall_IsCorrectionEdge(uint8_t new_index, int8_t direction)
{
    if (direction > 0)
    {
#if HALL_POSITIVE_DIRECTION_IS_ANTICLOCKWISE
        return (new_index == HALL_H6_INDEX) ? 1U : 0U;
#else
        return (new_index == HALL_H5_INDEX) ? 1U : 0U;
#endif
    }

    if (direction < 0)
    {
#if HALL_POSITIVE_DIRECTION_IS_ANTICLOCKWISE
        return (new_index == HALL_H5_INDEX) ? 1U : 0U;
#else
        return (new_index == HALL_H6_INDEX) ? 1U : 0U;
#endif
    }

    return 0U;
}

static void Hall_ResetTimingLocked(void)
{
    memset(s_dt_us, 0, sizeof(s_dt_us));
    s_dt_pos = 0U;
    s_dt_count = 0U;
    s_valid_transition_count = 0U;
    s_hall.dt_sum_us = 0U;
    s_pending_speed_update = 0U;
    s_pending_correction = 0U;
    s_speed_timeout_us = HALL_TIMEOUT_MIN_US;
}

static void Hall_QueueAngleSnapLocked(uint8_t mode, uint8_t index, int8_t direction)
{
    s_pending_angle_snap = mode;
    s_pending_angle_index = index;
    s_pending_angle_direction = direction;
}

static void Hall_QueueTransitionEventLocked(uint8_t previous_state,
                                            uint8_t hall_state,
                                            int8_t direction,
                                            uint32_t timestamp_us)
{
    uint8_t next_head;

    s_event_sequence++;

    next_head = s_event_head + 1U;
    if (next_head >= HALL_TRANSITION_EVENT_QUEUE_LEN)
    {
        next_head = 0U;
    }

    if (next_head == s_event_tail)
    {
        s_event_overflow_count++;
        return;
    }

    s_event_queue[s_event_head].sequence = s_event_sequence;
    s_event_queue[s_event_head].timestamp_us = timestamp_us;
    s_event_queue[s_event_head].previous_state = previous_state;
    s_event_queue[s_event_head].hall_state = hall_state;
    s_event_queue[s_event_head].direction = direction;
    s_event_queue[s_event_head].open_loop_theta_e_rad = open_loop_get_theta_e_rad();

    s_event_head = next_head;
}

static void Hall_ResyncLocked(uint8_t state, uint8_t index, uint32_t timestamp_us, uint8_t snap_mid)
{
    s_hall.previous_state = s_hall.hall_state;
    s_hall.hall_state = state;
    s_hall.state_valid = 1U;
    s_hall.speed_valid = 0U;
    s_hall.direction = 0;
    s_hall.last_edge_us = timestamp_us;
    s_hall.omega_e_rad_s = 0.0f;
    s_hall.omega_corr_rad_s = 0.0f;
    s_hall.mechanical_rpm = 0.0f;
    s_hall_index = index;

    Hall_ResetTimingLocked();

    if (snap_mid != 0U)
    {
        Hall_QueueAngleSnapLocked(HALL_SNAP_SECTOR_MID, index, 0);
    }
}

static void Hall_AddDtLocked(uint32_t dt_us)
{
    if (s_dt_count < 3U)
    {
        s_dt_us[s_dt_pos] = dt_us;
        s_hall.dt_sum_us += dt_us;
        s_dt_count++;
    }
    else
    {
        s_hall.dt_sum_us -= s_dt_us[s_dt_pos];
        s_dt_us[s_dt_pos] = dt_us;
        s_hall.dt_sum_us += dt_us;
    }

    s_dt_pos++;
    if (s_dt_pos >= 3U)
    {
        s_dt_pos = 0U;
    }
}

static float Hall_SectorMidAngleRad(uint8_t index)
{
    if (index >= 6U)
    {
        return 0.0f;
    }

    return s_theta_hall_mid_rad[index];
}

static float Hall_EdgeAngleRad(uint8_t index, int8_t direction)
{
    if (index >= 6U)
    {
        return 0.0f;
    }

    if (direction < 0)
    {
        return s_theta_hall_negative_rad[index];
    }

    return s_theta_hall_positive_rad[index];
}

static float Hall_Wrap0To2Pi(float angle_rad)
{
    while (angle_rad >= HALL_TWO_PI)
    {
        angle_rad -= HALL_TWO_PI;
    }

    while (angle_rad < 0.0f)
    {
        angle_rad += HALL_TWO_PI;
    }

    return angle_rad;
}

static float Hall_WrapMinusPiToPi(float angle_rad)
{
    while (angle_rad > HALL_PI)
    {
        angle_rad -= HALL_TWO_PI;
    }

    while (angle_rad < -HALL_PI)
    {
        angle_rad += HALL_TWO_PI;
    }

    return angle_rad;
}

static uint32_t Hall_TimeoutFromDtSum(uint32_t dt_sum_us)
{
    uint32_t timeout_us;

    if (dt_sum_us > (UINT32_MAX / 2U))
    {
        timeout_us = UINT32_MAX;
    }
    else
    {
        timeout_us = dt_sum_us * 2U;
    }

    if (timeout_us < HALL_TIMEOUT_MIN_US)
    {
        timeout_us = HALL_TIMEOUT_MIN_US;
    }

    return timeout_us;
}

static void Hall_UpdateDerived(void)
{
    float pole_pairs = MOTOR_POLE_PAIRS;
    if (pole_pairs <= 0.0f)
    {
        pole_pairs = 1.0f;
    }

    s_hall.theta_e_rad = Hall_Wrap0To2Pi(s_hall.theta_e_rad);
    s_hall.theta_e_deg = s_hall.theta_e_rad * HALL_RAD_TO_DEG;
    s_hall.theta_r_deg = s_hall.theta_e_deg / pole_pairs;

    if (s_hall.speed_valid != 0U)
    {
        s_hall.mechanical_rpm = (s_hall.omega_e_rad_s * HALL_RAD_S_TO_RPM) / pole_pairs;
    }
    else
    {
        s_hall.mechanical_rpm = 0.0f;
    }
}
