/*
 * open_loop.c
 *
 * Simple 3-phase open-loop sine PWM.
 */

#include "open_loop.h"
#include "hw_conf.h"
#include "vesc_uart.h"

#include <math.h>
#include <stdio.h>

extern TIM_HandleTypeDef htim1;

#define OL_TWO_PI              6.28318530718f
#define OL_2PI_OVER_3          2.09439510239f

#define OL_MOD_MAX             0.70f
#define OL_FREQ_E_MAX_HZ       50.0f     // electrical Hz, about 200 mechanical RPM with 15 pole pairs
#define OL_MOD_RAMP_PER_SEC    0.05f
#define OL_FREQ_RAMP_PER_SEC   2.0f

static uint8_t s_running = 0U;

static float s_theta = 0.0f;

static float s_mod = 0.0f;
static float s_freq_e_hz = 0.0f;

static float s_target_mod = 0.0f;
static float s_target_freq_e_hz = 0.0f;

static float ol_clampf(float x, float min_v, float max_v);
static float ol_ramp_to(float now, float target, float step);
static void ol_set_duty(uint32_t channel, float duty);
static void ol_write_3phase(float theta, float modulation);
static void ol_start_pwm_all(void);
static void ol_stop_pwm_all(void);

void open_loop_init(void)
{
    s_running = 0U;

    s_theta = 0.0f;

    s_mod = 0.0f;
    s_freq_e_hz = 0.0f;

    s_target_mod = 0.0f;
    s_target_freq_e_hz = 0.0f;

    ol_stop_pwm_all();
}

uint8_t open_loop_start(float modulation, float freq_e_hz)
{
    modulation = ol_clampf(modulation, 0.0f, OL_MOD_MAX);
    freq_e_hz = ol_clampf(freq_e_hz, -OL_FREQ_E_MAX_HZ, OL_FREQ_E_MAX_HZ);

    s_theta = 0.0f;

    s_mod = 0.0f;
    s_freq_e_hz = 0.0f;

    s_target_mod = modulation;
    s_target_freq_e_hz = freq_e_hz;

    /*
     * Mulai dari duty 50% semua.
     * Line-to-line average = 0, jadi relatif aman saat start.
     */
    ol_write_3phase(0.0f, 0.0f);
    ol_start_pwm_all();

    s_running = 1U;

    return 1U;
}

void open_loop_set(float modulation, float freq_e_hz)
{
    s_target_mod = ol_clampf(modulation, 0.0f, OL_MOD_MAX);
    s_target_freq_e_hz = ol_clampf(freq_e_hz, -OL_FREQ_E_MAX_HZ, OL_FREQ_E_MAX_HZ);
}

void open_loop_stop(void)
{
    s_running = 0U;

    s_mod = 0.0f;
    s_freq_e_hz = 0.0f;
    s_target_mod = 0.0f;
    s_target_freq_e_hz = 0.0f;

    ol_stop_pwm_all();
}

void open_loop_adc_tick(float dt_s)
{
    if (!s_running)
    {
        return;
    }

    if (dt_s <= 0.0f)
    {
        return;
    }

    s_mod = ol_ramp_to(s_mod, s_target_mod, OL_MOD_RAMP_PER_SEC * dt_s);
    s_freq_e_hz = ol_ramp_to(s_freq_e_hz, s_target_freq_e_hz, OL_FREQ_RAMP_PER_SEC * dt_s);

    s_theta += OL_TWO_PI * s_freq_e_hz * dt_s;

    while (s_theta >= OL_TWO_PI)
    {
        s_theta -= OL_TWO_PI;
    }

    while (s_theta < 0.0f)
    {
        s_theta += OL_TWO_PI;
    }

    ol_write_3phase(s_theta, s_mod);
}

void open_loop_poll(void)
{
    /* Runtime updates are synchronized from the ADC callback now. */
}

void open_loop_print_status(void)
{
    char msg[160];

    int mod_x1000 = (int)(s_mod * 1000.0f);
    int target_mod_x1000 = (int)(s_target_mod * 1000.0f);

    int freq_x1000 = (int)(s_freq_e_hz * 1000.0f);
    int target_freq_x1000 = (int)(s_target_freq_e_hz * 1000.0f);

    snprintf(msg, sizeof(msg),
             "OL,running=%u,mod_x1000=%d,target_mod_x1000=%d,freq_mHz=%d,target_freq_mHz=%d\r\n",
             s_running,
             mod_x1000,
             target_mod_x1000,
             freq_x1000,
             target_freq_x1000);

    uart_print(msg);
}

uint8_t open_loop_is_running(void)
{
    return s_running;
}

float open_loop_get_modulation(void)
{
    return s_mod;
}

float open_loop_get_target_modulation(void)
{
    return s_target_mod;
}

float open_loop_get_freq_e_hz(void)
{
    return s_freq_e_hz;
}

float open_loop_get_target_freq_e_hz(void)
{
    return s_target_freq_e_hz;
}

float open_loop_get_theta_e_rad(void)
{
    return s_theta;
}

static void ol_write_3phase(float theta, float modulation)
{
    float du = 0.5f + modulation * sinf(theta);
    float dv = 0.5f + modulation * sinf(theta - OL_2PI_OVER_3);
    float dw = 0.5f + modulation * sinf(theta + OL_2PI_OVER_3);

    ol_set_duty(HW_PWM_PHASE_U_CHANNEL, du);
    ol_set_duty(HW_PWM_PHASE_V_CHANNEL, dv);
    ol_set_duty(HW_PWM_PHASE_W_CHANNEL, dw);
}

static void ol_set_duty(uint32_t channel, float duty)
{
    duty = ol_clampf(duty, 0.0f, 0.95f);

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t pulse = (uint32_t)((float)(arr + 1U) * duty);

    if (pulse > arr)
    {
        pulse = arr;
    }

    __HAL_TIM_SET_COMPARE(&htim1, channel, pulse);
}

static void ol_start_pwm_all(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

static void ol_stop_pwm_all(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    /* Keep TIM1 update TRGO alive for TIM2-synchronized ADC sampling. */
    __HAL_TIM_ENABLE(&htim1);
}

static float ol_clampf(float x, float min_v, float max_v)
{
    if (x < min_v)
    {
        return min_v;
    }

    if (x > max_v)
    {
        return max_v;
    }

    return x;
}

static float ol_ramp_to(float now, float target, float step)
{
    if (now < target)
    {
        now += step;

        if (now > target)
        {
            now = target;
        }
    }
    else if (now > target)
    {
        now -= step;

        if (now < target)
        {
            now = target;
        }
    }

    return now;
}
