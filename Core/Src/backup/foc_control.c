#include "foc_control.h"

#include "adc_sense.h"
#include "foc_math.h"
#include "hall_sensor.h"
#include "hw_conf.h"
#include "vesc_uart.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

extern TIM_HandleTypeDef htim1;

#ifndef FOC_LOOP_TD
#define FOC_LOOP_TD 				  0.002f
#endif

#ifndef FOC_CURRENT_KP_D
#define FOC_CURRENT_KP_D              (FOC_MOTOR_LD_H/FOC_LOOP_TD)
#endif

#ifndef FOC_CURRENT_KI_D
#define FOC_CURRENT_KI_D              (FOC_MOTOR_RS_OHM/FOC_LOOP_TD)
#endif

#ifndef FOC_CURRENT_KP_Q
#define FOC_CURRENT_KP_Q              (FOC_MOTOR_LQ_H/FOC_LOOP_TD)
#endif

#ifndef FOC_CURRENT_KI_Q
#define FOC_CURRENT_KI_Q              (FOC_MOTOR_RS_OHM/FOC_LOOP_TD)
#endif

#ifndef FOC_SPEED_KP_A_PER_RPM
#define FOC_SPEED_KP_A_PER_RPM        0.01f
#endif

#ifndef FOC_SPEED_KI_A_PER_RPM_S
#define FOC_SPEED_KI_A_PER_RPM_S      0.05f
#endif

#ifndef FOC_MOTOR_RS_OHM
#define FOC_MOTOR_RS_OHM              0.2363f
#endif

#ifndef FOC_MOTOR_LD_H
#define FOC_MOTOR_LD_H                0.00044456f
#endif

#ifndef FOC_MOTOR_LQ_H
#define FOC_MOTOR_LQ_H                0.00044456f
#endif

#ifndef FOC_MOTOR_PSI_WB
#define FOC_MOTOR_PSI_WB              0.021403f
#endif

#ifndef FOC_MAX_CURRENT_REF_A
#define FOC_MAX_CURRENT_REF_A         3.0f
#endif

#ifndef FOC_DEFAULT_IQ_LIMIT_A
#define FOC_DEFAULT_IQ_LIMIT_A        1.5f
#endif

#ifndef FOC_PHASE_CURRENT_LIMIT_A
#define FOC_PHASE_CURRENT_LIMIT_A     15.0f
#endif

#ifndef FOC_VBUS_MIN_V
#define FOC_VBUS_MIN_V                8.0f
#endif

#ifndef FOC_VOLTAGE_LIMIT_FRAC
#define FOC_VOLTAGE_LIMIT_FRAC        0.95f // Base speed kita di sini, dengan Vbus 38.8 maka nanti sekitar 633 RPM
#endif

#ifndef FOC_VOLTAGE_LIMIT_MAX_V
#define FOC_VOLTAGE_LIMIT_MAX_V       45.0f
#endif

#ifndef FOC_DUTY_MAX
#define FOC_DUTY_MAX                  0.95f
#endif

#ifndef FOC_SPEED_RPM_MAX
#define FOC_SPEED_RPM_MAX             667.0f
#endif

#ifndef FOC_SPEED_LOOP_DIV
#define FOC_SPEED_LOOP_DIV            20U
#endif

#define FOC_PI                         3.14159265358979323846f
#define FOC_TWO_PI                     6.28318530717958647692f
#define FOC_INV_SQRT3                  0.5773502691896258f

static volatile uint8_t s_running;
static volatile foc_control_mode_t s_mode;
static volatile foc_control_fault_t s_fault;

static float s_id_ref_a;
static float s_iq_ref_a;
static float s_rpm_ref;
static float s_iq_limit_a;

static float s_id_int_v;
static float s_iq_int_v;
static float s_speed_int_a;
static uint32_t s_speed_tick_div;

static int8_t s_theta_direction = 1;
static float s_theta_offset_rad = FOC_THETA_OFFSET_RAD;

static foc_control_status_t s_status;

static float foc_clampf(float value, float min_v, float max_v);
static float foc_absf(float value);
static int32_t foc_float_to_i32_scaled(float value, float scale);
static float foc_wrap_0_2pi(float angle_rad);
static void foc_reset_control_state(void);
static void foc_start_pwm_all(void);
static void foc_stop_pwm_all(void);
static void foc_stop_pwm_all_from_isr(void);
static void foc_set_duty(uint32_t channel, float duty);
static void foc_apply_svpwm(float valpha_v, float vbeta_v, float vbus_v,
                            float *duty_u, float *duty_v, float *duty_w,
                            float *modulation);
static void foc_stop_with_fault_from_isr(foc_control_fault_t fault);
static const char *foc_mode_name(foc_control_mode_t mode);
static const char *foc_fault_name(foc_control_fault_t fault);

void foc_control_init(void)
{
    __disable_irq();
    s_running = 0U;
    s_mode = FOC_CONTROL_MODE_OFF;
    s_fault = FOC_CONTROL_FAULT_NONE;
    s_id_ref_a = 0.0f;
    s_iq_ref_a = 0.0f;
    s_rpm_ref = 0.0f;
    s_iq_limit_a = FOC_DEFAULT_IQ_LIMIT_A;
    s_id_int_v = 0.0f;
    s_iq_int_v = 0.0f;
    s_speed_int_a = 0.0f;
    s_speed_tick_div = 0U;
    memset(&s_status, 0, sizeof(s_status));
    s_status.mode = FOC_CONTROL_MODE_OFF;
    s_status.fault = FOC_CONTROL_FAULT_NONE;
    __enable_irq();

    foc_stop_pwm_all();
}

uint8_t foc_control_start_current(float id_ref_a, float iq_ref_a)
{
    adc_sense_data_t adc = adc_sense_get_data();
    hall_sensor_data_t hall = Hall_GetData();

    if ((adc.dma_started == 0U) || (adc.current_calibrated == 0U))
    {
        s_fault = FOC_CONTROL_FAULT_NOT_CALIBRATED;
        return 0U;
    }

    if ((hall.state_valid == 0U) || (hall.angle_valid == 0U))
    {
        s_fault = FOC_CONTROL_FAULT_HALL_INVALID;
        return 0U;
    }

    id_ref_a = foc_clampf(id_ref_a, -FOC_MAX_CURRENT_REF_A, FOC_MAX_CURRENT_REF_A);
    iq_ref_a = foc_clampf(iq_ref_a, -FOC_MAX_CURRENT_REF_A, FOC_MAX_CURRENT_REF_A);

    foc_stop_pwm_all();
    foc_reset_control_state();

    __disable_irq();
    s_id_ref_a = id_ref_a;
    s_iq_ref_a = iq_ref_a;
    s_rpm_ref = 0.0f;
    s_iq_limit_a = FOC_DEFAULT_IQ_LIMIT_A;
    s_mode = FOC_CONTROL_MODE_CURRENT;
    s_fault = FOC_CONTROL_FAULT_NONE;
    s_running = 1U;
    __enable_irq();

    foc_start_pwm_all();
    return 1U;
}

uint8_t foc_control_start_speed(float rpm_ref, float iq_limit_a)
{
    adc_sense_data_t adc = adc_sense_get_data();
    hall_sensor_data_t hall = Hall_GetData();

    if ((adc.dma_started == 0U) || (adc.current_calibrated == 0U))
    {
        s_fault = FOC_CONTROL_FAULT_NOT_CALIBRATED;
        return 0U;
    }

    if ((hall.state_valid == 0U) || (hall.angle_valid == 0U))
    {
        s_fault = FOC_CONTROL_FAULT_HALL_INVALID;
        return 0U;
    }

    rpm_ref = foc_clampf(rpm_ref, -FOC_SPEED_RPM_MAX, FOC_SPEED_RPM_MAX);
    iq_limit_a = foc_clampf(foc_absf(iq_limit_a), 0.1f, FOC_MAX_CURRENT_REF_A);

    foc_stop_pwm_all();
    foc_reset_control_state();

    __disable_irq();
    s_id_ref_a = 0.0f;
    s_iq_ref_a = 0.0f;
    s_rpm_ref = rpm_ref;
    s_iq_limit_a = iq_limit_a;
    s_mode = FOC_CONTROL_MODE_SPEED;
    s_fault = FOC_CONTROL_FAULT_NONE;
    s_running = 1U;
    __enable_irq();

    foc_start_pwm_all();
    return 1U;
}

void foc_control_set_current_refs(float id_ref_a, float iq_ref_a)
{
    id_ref_a = foc_clampf(id_ref_a, -FOC_MAX_CURRENT_REF_A, FOC_MAX_CURRENT_REF_A);
    iq_ref_a = foc_clampf(iq_ref_a, -FOC_MAX_CURRENT_REF_A, FOC_MAX_CURRENT_REF_A);

    __disable_irq();
    s_id_ref_a = id_ref_a;
    s_iq_ref_a = iq_ref_a;
    __enable_irq();
}

void foc_control_set_speed_ref(float rpm_ref, float iq_limit_a)
{
    rpm_ref = foc_clampf(rpm_ref, -FOC_SPEED_RPM_MAX, FOC_SPEED_RPM_MAX);
    iq_limit_a = foc_clampf(foc_absf(iq_limit_a), 0.1f, FOC_MAX_CURRENT_REF_A);

    __disable_irq();
    s_rpm_ref = rpm_ref;
    s_iq_limit_a = iq_limit_a;
    __enable_irq();
}

void foc_control_stop(void)
{
    __disable_irq();
    s_running = 0U;
    s_mode = FOC_CONTROL_MODE_OFF;
    s_id_ref_a = 0.0f;
    s_iq_ref_a = 0.0f;
    s_rpm_ref = 0.0f;
    s_status.running = 0U;
    s_status.mode = FOC_CONTROL_MODE_OFF;
    __enable_irq();

    foc_stop_pwm_all();
}

void foc_control_adc_tick(float dt_s, const adc_sense_data_t *adc, const hall_sensor_data_t *hall)
{
    if ((s_running == 0U) || (dt_s <= 0.0f) || (adc == NULL) || (hall == NULL))
    {
        return;
    }

    if ((adc->dma_started == 0U) || (adc->current_calibrated == 0U))
    {
        foc_stop_with_fault_from_isr(FOC_CONTROL_FAULT_NOT_CALIBRATED);
        return;
    }

    if ((hall->state_valid == 0U) || (hall->angle_valid == 0U))
    {
        foc_stop_with_fault_from_isr(FOC_CONTROL_FAULT_HALL_INVALID);
        return;
    }

    if (adc->vbus_v < FOC_VBUS_MIN_V)
    {
        foc_stop_with_fault_from_isr(FOC_CONTROL_FAULT_VBUS_INVALID);
        return;
    }

    float ia = FOC_CURRENT_IA_SIGN * adc->i1_a;
    float ic = FOC_CURRENT_IC_SIGN * adc->i3_a;
    float ib = -(ia + ic);
    float max_phase_current = foc_absf(ia);
    if (foc_absf(ib) > max_phase_current)
    {
        max_phase_current = foc_absf(ib);
    }
    if (foc_absf(ic) > max_phase_current)
    {
        max_phase_current = foc_absf(ic);
    }

    if (max_phase_current > FOC_PHASE_CURRENT_LIMIT_A)
    {
        foc_stop_with_fault_from_isr(FOC_CONTROL_FAULT_OVERCURRENT);
        return;
    }

    float theta_dir = (s_theta_direction < 0) ? -1.0f : 1.0f;
    float theta = foc_wrap_0_2pi((theta_dir * hall->theta_e_rad) + s_theta_offset_rad);
    float omega_e = theta_dir * (hall->omega_e_rad_s + hall->omega_corr_rad_s);
    float rpm_mech = theta_dir * hall->mechanical_rpm;

    foc_alpha_beta_t i_ab = foc_clarke(ia, ib, ic);
    foc_dq_t i_dq = foc_park(i_ab, theta);

    float id_ref = s_id_ref_a;
    float iq_ref = s_iq_ref_a;

    if (s_mode == FOC_CONTROL_MODE_SPEED)
    {
        s_speed_tick_div++;
        if (s_speed_tick_div >= FOC_SPEED_LOOP_DIV)
        {
            s_speed_tick_div = 0U;

            float speed_dt_s = dt_s * (float)FOC_SPEED_LOOP_DIV;
            float rpm_error = s_rpm_ref - rpm_mech;
            float old_speed_int = s_speed_int_a;

            s_speed_int_a += FOC_SPEED_KI_A_PER_RPM_S * rpm_error * speed_dt_s;
            float speed_iq = (FOC_SPEED_KP_A_PER_RPM * rpm_error) + s_speed_int_a;
            float iq_limit = foc_clampf(s_iq_limit_a, 0.1f, FOC_MAX_CURRENT_REF_A);

            if (speed_iq > iq_limit)
            {
                speed_iq = iq_limit;
                s_speed_int_a = old_speed_int;
            }
            else if (speed_iq < -iq_limit)
            {
                speed_iq = -iq_limit;
                s_speed_int_a = old_speed_int;
            }

            s_iq_ref_a = speed_iq;
        }

        id_ref = 0.0f;
        iq_ref = s_iq_ref_a;
    }

    id_ref = foc_clampf(id_ref, -FOC_MAX_CURRENT_REF_A, FOC_MAX_CURRENT_REF_A);
    iq_ref = foc_clampf(iq_ref, -FOC_MAX_CURRENT_REF_A, FOC_MAX_CURRENT_REF_A);

    float err_d = id_ref - i_dq.d;
    float err_q = iq_ref - i_dq.q;
    float old_id_int_v = s_id_int_v;
    float old_iq_int_v = s_iq_int_v;

    s_id_int_v += FOC_CURRENT_KI_D * err_d * dt_s;
    s_iq_int_v += FOC_CURRENT_KI_Q * err_q * dt_s;

    float pi_d = (FOC_CURRENT_KP_D * err_d) + s_id_int_v;
    float pi_q = (FOC_CURRENT_KP_Q * err_q) + s_iq_int_v;
    foc_dq_t v_dq;
    v_dq.d = pi_d - (omega_e * FOC_MOTOR_LQ_H * i_dq.q);
    v_dq.q = pi_q + (omega_e * FOC_MOTOR_LD_H * i_dq.d) + (omega_e * FOC_MOTOR_PSI_WB);

    float vbus_v = adc->vbus_v;
    float v_limit = vbus_v * FOC_INV_SQRT3 * FOC_VOLTAGE_LIMIT_FRAC;
    if (v_limit > FOC_VOLTAGE_LIMIT_MAX_V)
    {
        v_limit = FOC_VOLTAGE_LIMIT_MAX_V;
    }

    float vref = sqrtf((v_dq.d * v_dq.d) + (v_dq.q * v_dq.q));
    if ((vref > v_limit) && (vref > 0.001f))
    {
        float scale = v_limit / vref;
        v_dq.d *= scale;
        v_dq.q *= scale;
        vref = v_limit;
        s_id_int_v = old_id_int_v;
        s_iq_int_v = old_iq_int_v;
    }

    foc_alpha_beta_t v_ab = foc_inv_park(v_dq, theta);
    float duty_u = 0.5f;
    float duty_v = 0.5f;
    float duty_w = 0.5f;
    float modulation = 0.0f;
    foc_apply_svpwm(v_ab.alpha, v_ab.beta, vbus_v, &duty_u, &duty_v, &duty_w, &modulation);

    foc_set_duty(HW_PWM_PHASE_U_CHANNEL, duty_u);
    foc_set_duty(HW_PWM_PHASE_V_CHANNEL, duty_v);
    foc_set_duty(HW_PWM_PHASE_W_CHANNEL, duty_w);

    foc_abc_t v_phase = foc_inv_clarke(v_ab);

    s_status.running = 1U;
    s_status.mode = s_mode;
    s_status.fault = s_fault;
    s_status.id_ref_a = id_ref;
    s_status.iq_ref_a = iq_ref;
    s_status.rpm_ref = s_rpm_ref;
    s_status.iq_limit_a = s_iq_limit_a;
    s_status.id_a = i_dq.d;
    s_status.iq_a = i_dq.q;
    s_status.rpm_mech = rpm_mech;
    s_status.vd_v = v_dq.d;
    s_status.vq_v = v_dq.q;
    s_status.valpha_v = v_ab.alpha;
    s_status.vbeta_v = v_ab.beta;
    s_status.vref_v = vref;
    s_status.voltage_limit_v = v_limit;
    s_status.modulation = modulation;
    s_status.duty_u = duty_u;
    s_status.duty_v = duty_v;
    s_status.duty_w = duty_w;
    s_status.vphase_u_smooth_v = v_phase.a;
    s_status.vphase_v_smooth_v = v_phase.b;
    s_status.vphase_w_smooth_v = v_phase.c;
}

uint8_t foc_control_is_running(void)
{
    return s_running;
}

foc_control_mode_t foc_control_get_mode(void)
{
    return s_mode;
}

foc_control_status_t foc_control_get_status(void)
{
    foc_control_status_t status;

    __disable_irq();
    status = s_status;
    status.running = s_running;
    status.mode = s_mode;
    status.fault = s_fault;
    status.id_ref_a = s_id_ref_a;
    status.iq_ref_a = s_iq_ref_a;
    status.rpm_ref = s_rpm_ref;
    status.iq_limit_a = s_iq_limit_a;
    __enable_irq();

    return status;
}

void foc_control_set_theta_offset_deg(float offset_deg)
{
    if (offset_deg > 360.0f)
    {
        offset_deg = 360.0f;
    }
    else if (offset_deg < -360.0f)
    {
        offset_deg = -360.0f;
    }

    __disable_irq();
    s_theta_offset_rad = offset_deg * (FOC_PI / 180.0f);
    __enable_irq();
}

float foc_control_get_theta_offset_deg(void)
{
    return s_theta_offset_rad * (180.0f / FOC_PI);
}

uint8_t foc_control_set_theta_direction(int8_t direction)
{
    if (direction == 0)
    {
        return 0U;
    }

    __disable_irq();
    s_theta_direction = (direction < 0) ? -1 : 1;
    __enable_irq();
    return 1U;
}

int8_t foc_control_get_theta_direction(void)
{
    return s_theta_direction;
}

void foc_control_print_status(void)
{
    foc_control_status_t st = foc_control_get_status();
    char msg[384];

    snprintf(msg, sizeof(msg),
             "FOC,run=%u,mode=%s,fault=%s,id_ref_mA=%ld,iq_ref_mA=%ld,rpm_ref_x10=%ld,rpm_x10=%ld,id_mA=%ld,iq_mA=%ld,vd_mV=%ld,vq_mV=%ld,vref_mV=%ld,vlim_mV=%ld,mod_x10000=%ld,du_x10000=%ld,dv_x10000=%ld,dw_x10000=%ld,theta_dir=%d,theta_off_x100=%ld\r\n",
             st.running,
             foc_mode_name(st.mode),
             foc_fault_name(st.fault),
             (long)foc_float_to_i32_scaled(st.id_ref_a, 1000.0f),
             (long)foc_float_to_i32_scaled(st.iq_ref_a, 1000.0f),
             (long)foc_float_to_i32_scaled(st.rpm_ref, 10.0f),
             (long)foc_float_to_i32_scaled(st.rpm_mech, 10.0f),
             (long)foc_float_to_i32_scaled(st.id_a, 1000.0f),
             (long)foc_float_to_i32_scaled(st.iq_a, 1000.0f),
             (long)foc_float_to_i32_scaled(st.vd_v, 1000.0f),
             (long)foc_float_to_i32_scaled(st.vq_v, 1000.0f),
             (long)foc_float_to_i32_scaled(st.vref_v, 1000.0f),
             (long)foc_float_to_i32_scaled(st.voltage_limit_v, 1000.0f),
             (long)foc_float_to_i32_scaled(st.modulation, 10000.0f),
             (long)foc_float_to_i32_scaled(st.duty_u, 10000.0f),
             (long)foc_float_to_i32_scaled(st.duty_v, 10000.0f),
             (long)foc_float_to_i32_scaled(st.duty_w, 10000.0f),
             (int)s_theta_direction,
             (long)foc_float_to_i32_scaled(foc_control_get_theta_offset_deg(), 100.0f));

    uart_print(msg);
}

static float foc_clampf(float value, float min_v, float max_v)
{
    if (value < min_v)
    {
        return min_v;
    }

    if (value > max_v)
    {
        return max_v;
    }

    return value;
}

static float foc_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t foc_float_to_i32_scaled(float value, float scale)
{
    float scaled = value * scale;
    return (scaled >= 0.0f) ? (int32_t)(scaled + 0.5f) : (int32_t)(scaled - 0.5f);
}

static float foc_wrap_0_2pi(float angle_rad)
{
    while (angle_rad >= FOC_TWO_PI)
    {
        angle_rad -= FOC_TWO_PI;
    }

    while (angle_rad < 0.0f)
    {
        angle_rad += FOC_TWO_PI;
    }

    return angle_rad;
}

static void foc_reset_control_state(void)
{
    __disable_irq();
    s_id_int_v = 0.0f;
    s_iq_int_v = 0.0f;
    s_speed_int_a = 0.0f;
    s_speed_tick_div = 0U;
    memset(&s_status, 0, sizeof(s_status));
    s_status.mode = FOC_CONTROL_MODE_OFF;
    s_status.fault = FOC_CONTROL_FAULT_NONE;
    __enable_irq();
}

static void foc_start_pwm_all(void)
{
    foc_set_duty(TIM_CHANNEL_1, 0.5f);
    foc_set_duty(TIM_CHANNEL_2, 0.5f);
    foc_set_duty(TIM_CHANNEL_3, 0.5f);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

static void foc_stop_pwm_all(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);

    __HAL_TIM_ENABLE(&htim1);
}

static void foc_stop_pwm_all_from_isr(void)
{
    htim1.Instance->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                              TIM_CCER_CC2E | TIM_CCER_CC2NE |
                              TIM_CCER_CC3E | TIM_CCER_CC3NE);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
    __HAL_TIM_ENABLE(&htim1);
}

static void foc_set_duty(uint32_t channel, float duty)
{
    duty = foc_clampf(duty, 0.0f, FOC_DUTY_MAX);

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t pulse = (uint32_t)((float)(arr + 1U) * duty);
    if (pulse > arr)
    {
        pulse = arr;
    }

    __HAL_TIM_SET_COMPARE(&htim1, channel, pulse);
}

static void foc_apply_svpwm(float valpha_v, float vbeta_v, float vbus_v,
                            float *duty_u, float *duty_v, float *duty_w,
                            float *modulation)
{
    float vref = sqrtf((valpha_v * valpha_v) + (vbeta_v * vbeta_v));
    float vmax = vbus_v * FOC_INV_SQRT3;
    float theta = atan2f(vbeta_v, valpha_v);

    if (theta < 0.0f)
    {
        theta += FOC_TWO_PI;
    }

    if ((vmax <= 0.1f) || (vref <= 0.001f))
    {
        *duty_u = 0.5f;
        *duty_v = 0.5f;
        *duty_w = 0.5f;
        *modulation = 0.0f;
        return;
    }

    if (vref > vmax)
    {
        vref = vmax;
    }

    float mod = vref / vmax;
    int sector = (int)(theta / (FOC_PI / 3.0f));
    if (sector >= 6)
    {
        sector = 0;
    }

    float beta = theta - ((float)sector * (FOC_PI / 3.0f));
    float t1 = mod * sinf((FOC_PI / 3.0f) - beta);
    float t2 = mod * sinf(beta);
    float t0_2 = (1.0f - t1 - t2) * 0.5f;

    float t0_f = t0_2;
    float t1_f = t1 + t2 + t0_2;
    float t2_f = t2 + t0_2;
    float t3_f = t1 + t0_2;

    switch (sector)
    {
        case 0:
            *duty_u = t1_f;
            *duty_v = t2_f;
            *duty_w = t0_f;
            break;

        case 1:
            *duty_u = t3_f;
            *duty_v = t1_f;
            *duty_w = t0_f;
            break;

        case 2:
            *duty_u = t0_f;
            *duty_v = t1_f;
            *duty_w = t2_f;
            break;

        case 3:
            *duty_u = t0_f;
            *duty_v = t3_f;
            *duty_w = t1_f;
            break;

        case 4:
            *duty_u = t2_f;
            *duty_v = t0_f;
            *duty_w = t1_f;
            break;

        case 5:
        default:
            *duty_u = t1_f;
            *duty_v = t0_f;
            *duty_w = t3_f;
            break;
    }

    *duty_u = foc_clampf(*duty_u, 0.0f, FOC_DUTY_MAX);
    *duty_v = foc_clampf(*duty_v, 0.0f, FOC_DUTY_MAX);
    *duty_w = foc_clampf(*duty_w, 0.0f, FOC_DUTY_MAX);
    *modulation = mod;
}

static void foc_stop_with_fault_from_isr(foc_control_fault_t fault)
{
    s_running = 0U;
    s_mode = FOC_CONTROL_MODE_OFF;
    s_fault = fault;
    s_status.running = 0U;
    s_status.mode = FOC_CONTROL_MODE_OFF;
    s_status.fault = fault;
    foc_stop_pwm_all_from_isr();
}

static const char *foc_mode_name(foc_control_mode_t mode)
{
    switch (mode)
    {
        case FOC_CONTROL_MODE_CURRENT:
            return "CURRENT";

        case FOC_CONTROL_MODE_SPEED:
            return "SPEED";

        case FOC_CONTROL_MODE_OFF:
        default:
            return "OFF";
    }
}

static const char *foc_fault_name(foc_control_fault_t fault)
{
    switch (fault)
    {
        case FOC_CONTROL_FAULT_NOT_CALIBRATED:
            return "NOT_CALIBRATED";

        case FOC_CONTROL_FAULT_HALL_INVALID:
            return "HALL_INVALID";

        case FOC_CONTROL_FAULT_OVERCURRENT:
            return "OVERCURRENT";

        case FOC_CONTROL_FAULT_VBUS_INVALID:
            return "VBUS_INVALID";

        case FOC_CONTROL_FAULT_NONE:
        default:
            return "NONE";
    }
}
