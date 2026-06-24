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

/* Hall-based FOC controller.
 *
 * Runtime flow:
 * - phase2_test.c calls foc_control_init() during board/app init.
 * - UART commands in phase2_test.c call the start/set/stop functions.
 * - adc_sense.c calls foc_control_adc_tick() from the ADC processing path at
 *   ADC_SENSE_CONTROL_TICK_HZ = 10 kHz after Hall_ControlTick().
 * - Field weakening runs at 500 Hz before the speed IP loop. It averages the 10 kHz
 *   voltage/speed samples from the previous FW period, then follows the thesis
 *   voltage-limit method: compare |Vdq| to Vmax, PI-generate negative Id, then
 *   reduce available Iq so sqrt(Id^2 + Iq^2) stays inside the current limit.
 * - Speed mode runs an outer IP controller every FOC_SPEED_LOOP_DIV current
 *   ticks (10 kHz / 10 = 1 kHz) and outputs an Iq reference.
 * - Every current tick runs: current sampling -> Clarke/Park -> current PI ->
 *   motor decoupling/feed-forward -> voltage limiting -> inverse Park -> SVPWM.
 */

#ifndef FOC_LOOP_TD
/* Current PI tuning time constant, not the ADC interrupt period. */
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

#ifndef FOC_SPEED_KP_A_PER_RADS
#define FOC_SPEED_KP_A_PER_RADS        0.04f  /* IP tuning. Previous PI value: 0.1f */
#endif

#ifndef FOC_SPEED_KI_A_PER_RADS_S
#define FOC_SPEED_KI_A_PER_RADS_S      0.4f   /* IP tuning. Previous PI value: 0.5f */
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

#ifndef FOC_I_MAX_A
/* Radius of the allowed stator-current vector in the d-q plane, in amperes.
 * This is not DC bus current and not an independent Id/Iq axis limit.
 */
#define FOC_I_MAX_A                   5.0f
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
#define FOC_VOLTAGE_LIMIT_FRAC        0.95f // About 66 rad/s mechanical at 38.8 V before field weakening (ideally), realistically 60 rad/s
#endif

#ifndef FOC_VOLTAGE_LIMIT_MAX_V
#define FOC_VOLTAGE_LIMIT_MAX_V       45.0f
#endif

#ifndef FOC_DUTY_MAX
#define FOC_DUTY_MAX                  0.95f
#endif

#ifndef FOC_SPEED_RADS_MAX
#define FOC_SPEED_RADS_MAX             70.0f
#endif

#ifndef FOC_SPEED_LOOP_DIV
#define FOC_SPEED_LOOP_DIV            10U
#endif

#ifndef FOC_FIELD_WEAKENING_LOOP_DIV
#define FOC_FIELD_WEAKENING_LOOP_DIV  20U
#endif

#ifndef FOC_FIELD_WEAKENING_KP_A_PER_V
#define FOC_FIELD_WEAKENING_KP_A_PER_V 0.05f
#endif

#ifndef FOC_FIELD_WEAKENING_KI_A_PER_V_S
#define FOC_FIELD_WEAKENING_KI_A_PER_V_S 1.0f
#endif

#ifndef FOC_FIELD_WEAKENING_ID_RAMP_A_PER_S
#define FOC_FIELD_WEAKENING_ID_RAMP_A_PER_S 2.0f
#endif

#ifndef FOC_FIELD_WEAKENING_SPEED_ON_FRAC
#define FOC_FIELD_WEAKENING_SPEED_ON_FRAC 0.90f
#endif

#ifndef FOC_FIELD_WEAKENING_SPEED_OFF_FRAC
#define FOC_FIELD_WEAKENING_SPEED_OFF_FRAC 0.85f
#endif

#ifndef FOC_FIELD_WEAKENING_VOLTAGE_ON_FRAC
#define FOC_FIELD_WEAKENING_VOLTAGE_ON_FRAC 0.98f
#endif

#ifndef FOC_FIELD_WEAKENING_VOLTAGE_OFF_FRAC
#define FOC_FIELD_WEAKENING_VOLTAGE_OFF_FRAC 0.94f
#endif

#ifndef FOC_FIELD_WEAKENING_ID_INACTIVE_A
#define FOC_FIELD_WEAKENING_ID_INACTIVE_A 0.05f
#endif

#ifndef FOC_FIELD_WEAKENING_DEFAULT_MAX_A
#define FOC_FIELD_WEAKENING_DEFAULT_MAX_A 0.5f
#endif

#define FOC_PI                         3.14159265358979323846f
#define FOC_TWO_PI                     6.28318530717958647692f
#define FOC_INV_SQRT3                  0.5773502691896258f

static volatile uint8_t s_running;
static volatile foc_control_mode_t s_mode;
static volatile foc_control_fault_t s_fault;

static float s_id_ref_a;
static float s_iq_ref_a;
static float s_rads_ref;
static float s_iq_limit_a;

static float s_id_int_v;
static float s_iq_int_v;
static float s_speed_int_a;
static uint32_t s_speed_tick_div;

static uint8_t s_fw_enabled;
static uint8_t s_fw_loop_active;
static float s_fw_max_current_a;
static float s_fw_int_a;
static float s_fw_id_ref_a;
static uint32_t s_fw_tick_div;

/* FW is a slower 500 Hz loop, but the current loop runs at 10 kHz. These sums
 * collect the previous FW period's requested voltage and speed samples so the
 * FW PI acts on a representative 2 ms block average, not one noisy instant.
 */
static float s_fw_vref_sum_v;
static float s_fw_vlimit_sum_v;
static float s_fw_mech_rads_sum;
static uint32_t s_fw_sample_count;
static float s_fw_base_rads;
static float s_fw_voltage_util;

static int8_t s_theta_direction = 1;
static float s_theta_offset_rad = FOC_THETA_OFFSET_RAD;

static foc_control_status_t s_status;

/* Small local helpers and hardware actions used by the real-time loop. */
static float foc_clampf(float value, float min_v, float max_v);
static float foc_absf(float value);
static float foc_iq_limit_for_id(float id_ref_a);
static void foc_limit_dq_current_refs(float *id_ref_a, float *iq_ref_a);
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

/* Called once from phase2_test.c init. Resets software state and leaves PWM off. */
void foc_control_init(void)
{
    __disable_irq();
    s_running = 0U;
    s_mode = FOC_CONTROL_MODE_OFF;
    s_fault = FOC_CONTROL_FAULT_NONE;
    s_id_ref_a = 0.0f;
    s_iq_ref_a = 0.0f;
    s_rads_ref = 0.0f;
    s_iq_limit_a = foc_clampf(FOC_DEFAULT_IQ_LIMIT_A, 0.1f, FOC_I_MAX_A);
    s_id_int_v = 0.0f;
    s_iq_int_v = 0.0f;
    s_speed_int_a = 0.0f;
    s_speed_tick_div = 0U;
    s_fw_enabled = 0U;
    s_fw_loop_active = 0U;
    s_fw_max_current_a = foc_clampf(FOC_FIELD_WEAKENING_DEFAULT_MAX_A, 0.0f, FOC_I_MAX_A);
    s_fw_int_a = 0.0f;
    s_fw_id_ref_a = 0.0f;
    s_fw_tick_div = 0U;
    s_fw_vref_sum_v = 0.0f;
    s_fw_vlimit_sum_v = 0.0f;
    s_fw_mech_rads_sum = 0.0f;
    s_fw_sample_count = 0U;
    s_fw_base_rads = 0.0f;
    s_fw_voltage_util = 0.0f;
    memset(&s_status, 0, sizeof(s_status));
    s_status.mode = FOC_CONTROL_MODE_OFF;
    s_status.fault = FOC_CONTROL_FAULT_NONE;
    __enable_irq();

    foc_stop_pwm_all();
}

/* Start direct current-control mode from UART command `foc_start_current`.
 * Requires calibrated ADC current sensing and a valid Hall angle before PWM starts.
 */
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

    foc_limit_dq_current_refs(&id_ref_a, &iq_ref_a);

    foc_stop_pwm_all();
    foc_reset_control_state();

    __disable_irq();
    s_id_ref_a = id_ref_a;
    s_iq_ref_a = iq_ref_a;
    s_rads_ref = 0.0f;
    s_iq_limit_a = foc_clampf(FOC_DEFAULT_IQ_LIMIT_A, 0.1f, FOC_I_MAX_A);
    s_mode = FOC_CONTROL_MODE_CURRENT;
    s_fault = FOC_CONTROL_FAULT_NONE;
    s_running = 1U;
    __enable_irq();

    foc_start_pwm_all();
    return 1U;
}

/* Start speed-control mode from UART command `foc_start_rads`.
 * The outer speed IP loop generates Iq; voltage-limit FW may add negative Id.
 */
uint8_t foc_control_start_speed(float rads_ref, float iq_limit_a)
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

    rads_ref = foc_clampf(rads_ref, -FOC_SPEED_RADS_MAX, FOC_SPEED_RADS_MAX);
    iq_limit_a = foc_clampf(foc_absf(iq_limit_a), 0.1f, FOC_I_MAX_A);

    foc_stop_pwm_all();
    foc_reset_control_state();

    __disable_irq();
    s_id_ref_a = 0.0f;
    s_iq_ref_a = 0.0f;
    s_rads_ref = rads_ref;
    s_iq_limit_a = iq_limit_a;
    s_mode = FOC_CONTROL_MODE_SPEED;
    s_fault = FOC_CONTROL_FAULT_NONE;
    s_running = 1U;
    __enable_irq();

    foc_start_pwm_all();
    return 1U;
}

/* Update Id/Iq references while current mode is running. */
void foc_control_set_current_refs(float id_ref_a, float iq_ref_a)
{
    foc_limit_dq_current_refs(&id_ref_a, &iq_ref_a);

    __disable_irq();
    s_id_ref_a = id_ref_a;
    s_iq_ref_a = iq_ref_a;
    __enable_irq();
}

/* Update mechanical rad/s speed target and Iq limit while speed mode is running. */
void foc_control_set_speed_ref(float rads_ref, float iq_limit_a)
{
    rads_ref = foc_clampf(rads_ref, -FOC_SPEED_RADS_MAX, FOC_SPEED_RADS_MAX);
    iq_limit_a = foc_clampf(foc_absf(iq_limit_a), 0.1f, FOC_I_MAX_A);

    __disable_irq();
    s_rads_ref = rads_ref;
    s_iq_limit_a = iq_limit_a;
    __enable_irq();
}

/* Configure voltage-limit field weakening from UART command `foc_fw`.
 * max_current_a means max allowed negative d-axis current magnitude: |-Id_FW|.
 */
void foc_control_set_field_weakening(uint8_t enabled, float max_current_a)
{
    max_current_a = foc_clampf(foc_absf(max_current_a), 0.0f, FOC_I_MAX_A);

    __disable_irq();
    s_fw_enabled = ((enabled != 0U) && (max_current_a > 0.0f)) ? 1U : 0U;
    s_fw_max_current_a = max_current_a;
    if (s_fw_enabled == 0U)
    {
        s_fw_loop_active = 0U;
        s_fw_int_a = 0.0f;
    }
    __enable_irq();
}

/* Stop FOC from the foreground command path and disable PWM outputs safely. */
void foc_control_stop(void)
{
    __disable_irq();
    s_running = 0U;
    s_mode = FOC_CONTROL_MODE_OFF;
    s_id_ref_a = 0.0f;
    s_iq_ref_a = 0.0f;
    s_rads_ref = 0.0f;
    s_fw_loop_active = 0U;
    s_fw_int_a = 0.0f;
    s_fw_id_ref_a = 0.0f;
    s_fw_vref_sum_v = 0.0f;
    s_fw_vlimit_sum_v = 0.0f;
    s_fw_mech_rads_sum = 0.0f;
    s_fw_sample_count = 0U;
    s_status.running = 0U;
    s_status.mode = FOC_CONTROL_MODE_OFF;
    __enable_irq();

    foc_stop_pwm_all();
}

/* Real-time FOC tick called by adc_sense.c at 10 kHz from the ADC processing path.
 * Keep this function deterministic: no UART prints, blocking HAL calls, or malloc.
 * MAIN LOOP FOC-NYA
 */
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

    /* Reconstruct three phase currents. Only two shunts are measured here; the
     * third current is inferred from ia + ib + ic = 0.
     */
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

    /* Apply user-calibrated Hall direction and electrical angle offset before
     * transforming measured currents into the rotor d-q frame.
     */
    float theta_dir = (s_theta_direction < 0) ? -1.0f : 1.0f;
    float theta = foc_wrap_0_2pi((theta_dir * hall->theta_e_rad) + s_theta_offset_rad);
    float omega_e = theta_dir * (hall->omega_e_rad_s + hall->omega_corr_rad_s);
    float rads_mech = theta_dir * hall->mechanical_rad_s;

    /* Clarke/Park: phase currents -> stationary alpha-beta -> rotor d-q. */
    foc_alpha_beta_t i_ab = foc_clarke(ia, ib, ic);
    foc_dq_t i_dq = foc_park(i_ab, theta);

    float id_ref = s_id_ref_a;
    float iq_ref = s_iq_ref_a;

    float vbus_v = adc->vbus_v;
    float v_limit = vbus_v * FOC_INV_SQRT3 * FOC_VOLTAGE_LIMIT_FRAC;
    if (v_limit > FOC_VOLTAGE_LIMIT_MAX_V)
    {
        v_limit = FOC_VOLTAGE_LIMIT_MAX_V;
    }

    if (s_mode == FOC_CONTROL_MODE_SPEED)
    {
        s_fw_tick_div++;
        if (s_fw_tick_div >= FOC_FIELD_WEAKENING_LOOP_DIV)
        {
            s_fw_tick_div = 0U;

            float pole_pairs = MOTOR_POLE_PAIRS;
            if (pole_pairs <= 0.0f)
            {
                pole_pairs = 1.0f;
            }

            float fw_dt_s = dt_s * (float)FOC_FIELD_WEAKENING_LOOP_DIV;
            float fw_target_id_a = 0.0f;
            float fw_max_current = foc_clampf(s_fw_max_current_a, 0.0f, FOC_I_MAX_A);
            float fw_vref_v = 0.0f;
            float fw_v_limit = v_limit;
            float fw_mech_rads = rads_mech;

            /* Average the samples collected since the last FW update. At the
             * nominal 10 kHz/500 Hz rates this is 20 current-loop samples.
             */
            if (s_fw_sample_count > 0U)
            {
                float inv_count = 1.0f / (float)s_fw_sample_count;
                fw_vref_v = s_fw_vref_sum_v * inv_count;
                fw_v_limit = s_fw_vlimit_sum_v * inv_count;
                fw_mech_rads = s_fw_mech_rads_sum * inv_count;
            }

            /* Consume one FW-period block average and start collecting the next
             * 10 kHz sample window. The averaged data is used only by the 500 Hz
             * FW loop; the current PI below keeps using each raw current sample.
             */
            s_fw_vref_sum_v = 0.0f;
            s_fw_vlimit_sum_v = 0.0f;
            s_fw_mech_rads_sum = 0.0f;
            s_fw_sample_count = 0U;

            float fw_base_rads = FOC_SPEED_RADS_MAX;
            if (FOC_MOTOR_PSI_WB > 0.0f)
            {
                fw_base_rads = fw_v_limit / (FOC_MOTOR_PSI_WB * pole_pairs);
            }
            float fw_voltage_util = (fw_v_limit > 0.1f) ? (fw_vref_v / fw_v_limit) : 0.0f;
            float abs_target_rads = foc_absf(s_rads_ref);
            float abs_mech_rads = foc_absf(fw_mech_rads);
            uint8_t same_direction = (((s_rads_ref >= 0.0f) && (fw_mech_rads >= 0.0f)) ||
                                      ((s_rads_ref <= 0.0f) && (fw_mech_rads <= 0.0f))) ? 1U : 0U;

            /* Voltage-limit FW from the thesis draft:
             * ev = Vmax - |Vdq|, PI(ev) generates negative Id, and Id is ramped
             * so entering/leaving FW does not step the current reference.
             * The speed gate prevents startup voltage spikes from enabling FW.
             */
            if ((s_fw_enabled != 0U) && (fw_max_current > 0.0f) && (FOC_MOTOR_PSI_WB > 0.0f))
            {
                float speed_on_rads = fw_base_rads * FOC_FIELD_WEAKENING_SPEED_ON_FRAC;
                float speed_off_rads = fw_base_rads * FOC_FIELD_WEAKENING_SPEED_OFF_FRAC;

                if (s_fw_loop_active != 0U)
                {
                    if ((same_direction == 0U) ||
                        (abs_target_rads < speed_off_rads) ||
                        (abs_mech_rads < speed_off_rads) ||
                        ((fw_voltage_util < FOC_FIELD_WEAKENING_VOLTAGE_OFF_FRAC) &&
                         (s_fw_id_ref_a > -FOC_FIELD_WEAKENING_ID_INACTIVE_A)))
                    {
                        s_fw_loop_active = 0U;
                    }
                }
                else if ((same_direction != 0U) &&
                         (abs_target_rads >= speed_on_rads) &&
                         (abs_mech_rads >= speed_on_rads) &&
                         (fw_voltage_util >= FOC_FIELD_WEAKENING_VOLTAGE_ON_FRAC))
                {
                    s_fw_loop_active = 1U;
                }

                if (s_fw_loop_active != 0U)
                {
                    float voltage_error = fw_v_limit - fw_vref_v;
                    float id_min = -fw_max_current;

                    /* FW PI anti-windup: clamp the integrator to the range that
                     * still produces an Id command inside [id_min, 0]. Without
                     * the upper clamp, positive voltage margin can wind the
                     * integrator upward, release negative Id too aggressively,
                     * and delay FW from re-entering when voltage demand rises.
                     */
                    float id_pterm = FOC_FIELD_WEAKENING_KP_A_PER_V * voltage_error;
                    s_fw_int_a += FOC_FIELD_WEAKENING_KI_A_PER_V_S * voltage_error * fw_dt_s;
                    s_fw_int_a = foc_clampf(s_fw_int_a, id_min - id_pterm, -id_pterm);

                    float id_cmd = id_pterm + s_fw_int_a;
                    fw_target_id_a = foc_clampf(id_cmd, id_min, 0.0f);
                }
                else
                {
                    s_fw_int_a = 0.0f;
                }
            }
            else
            {
                s_fw_loop_active = 0U;
                s_fw_int_a = 0.0f;
            }

            float fw_step = FOC_FIELD_WEAKENING_ID_RAMP_A_PER_S * fw_dt_s;
            if (s_fw_id_ref_a < (fw_target_id_a - fw_step))
            {
                s_fw_id_ref_a += fw_step;
            }
            else if (s_fw_id_ref_a > (fw_target_id_a + fw_step))
            {
                s_fw_id_ref_a -= fw_step;
            }
            else
            {
                s_fw_id_ref_a = fw_target_id_a;
            }

            s_fw_id_ref_a = foc_clampf(s_fw_id_ref_a, -fw_max_current, 0.0f);
            s_fw_base_rads = fw_base_rads;
            s_fw_voltage_util = fw_voltage_util;
        }

        s_speed_tick_div++;
        if (s_speed_tick_div >= FOC_SPEED_LOOP_DIV)
        {
            s_speed_tick_div = 0U;

            /* Outer speed IP controller runs at 1 kHz. The integrator acts on
             * speed error, while proportional feedback acts on measured speed.
             * This avoids a proportional kick when the speed target changes.
             */
            float speed_dt_s = dt_s * (float)FOC_SPEED_LOOP_DIV;
            float rads_error = s_rads_ref - rads_mech;

            s_speed_int_a += FOC_SPEED_KI_A_PER_RADS_S * rads_error * speed_dt_s;
            float speed_pterm = -(FOC_SPEED_KP_A_PER_RADS * rads_mech);

            /* I_MAX is the d-q current-circle radius. Negative FW Id uses part
             * of that radius, so the speed IP loop can only request the remaining Iq.
             */
            float iq_limit = foc_clampf(s_iq_limit_a, 0.1f, FOC_I_MAX_A);
            float iq_available = foc_iq_limit_for_id(s_fw_id_ref_a);
            if (iq_limit > iq_available)
            {
                iq_limit = iq_available;
            }

            /* IP anti-windup: clamp the integrator, not the already-combined
             * output. Because Iq = I_term + P_term and P_term = -Kp*speed, the
             * valid integrator range shifts with measured speed. This lets the
             * integrator keep the internal offset needed at high speed/FW while
             * still guaranteeing -iq_limit <= Iq <= iq_limit.
             */
            float speed_int_min = -iq_limit - speed_pterm;
            float speed_int_max = iq_limit - speed_pterm;
            s_speed_int_a = foc_clampf(s_speed_int_a, speed_int_min, speed_int_max);

            float speed_iq = s_speed_int_a + speed_pterm;
            speed_iq = foc_clampf(speed_iq, -iq_limit, iq_limit);

            /* Previous rollback anti-windup used for the speed PI controller.
             * Keep this reference here because it can be restored if the speed
             * loop is changed back to PI form: Iq = Kp*(target-speed) + I_term.
             *
             * float old_speed_int = s_speed_int_a;
             * s_speed_int_a += FOC_SPEED_KI_A_PER_RADS_S * rads_error * speed_dt_s;
             * float speed_iq = (FOC_SPEED_KP_A_PER_RADS * rads_error) + s_speed_int_a;
             * if (speed_iq > iq_limit) {
             *     speed_iq = iq_limit;
             *     s_speed_int_a = old_speed_int;
             * } else if (speed_iq < -iq_limit) {
             *     speed_iq = -iq_limit;
             *     s_speed_int_a = old_speed_int;
             * }
             */

            s_iq_ref_a = speed_iq;
        }

        id_ref = s_fw_id_ref_a;
        iq_ref = s_iq_ref_a;
    }

    foc_limit_dq_current_refs(&id_ref, &iq_ref);

    /* Inner current PI runs every 10 kHz tick. Id controls flux; Iq controls
     * torque. Id is zero unless voltage-limit FW requests negative Id.
     */
    float err_d = id_ref - i_dq.d;
    float err_q = iq_ref - i_dq.q;
    float old_id_int_v = s_id_int_v;
    float old_iq_int_v = s_iq_int_v;

    s_id_int_v += FOC_CURRENT_KI_D * err_d * dt_s;
    s_iq_int_v += FOC_CURRENT_KI_Q * err_q * dt_s;

    float pi_d = (FOC_CURRENT_KP_D * err_d) + s_id_int_v;
    float pi_q = (FOC_CURRENT_KP_Q * err_q) + s_iq_int_v;

    /* D/Q voltage command with PMSM cross-coupling and back-EMF feed-forward:
     * vd subtracts omega*Lq*Iq, vq adds omega*Ld*Id + omega*psi.
     */
    foc_dq_t v_dq;
    v_dq.d = pi_d - (omega_e * FOC_MOTOR_LQ_H * i_dq.q);
    v_dq.q = pi_q + (omega_e * FOC_MOTOR_LD_H * i_dq.d) + (omega_e * FOC_MOTOR_PSI_WB);

    /* Use the requested voltage before limiting for FW averaging. That keeps
     * the FW voltage-error method sensitive to demand beyond available Vbus,
     * while the actual PWM command below is still safely clipped to v_limit.
     */
    float vref_raw = sqrtf((v_dq.d * v_dq.d) + (v_dq.q * v_dq.q));
    float vref = vref_raw;
    if (s_mode == FOC_CONTROL_MODE_SPEED)
    {
        /* 10 kHz producer side of the FW block average. The 500 Hz FW loop
         * consumes and clears these sums before starting the next block.
         */
        s_fw_vref_sum_v += vref_raw;
        s_fw_vlimit_sum_v += v_limit;
        s_fw_mech_rads_sum += rads_mech;
        s_fw_sample_count++;
    }
    if ((vref > v_limit) && (vref > 0.001f))
    {
        /* Keep the requested voltage vector inside available SVPWM voltage.
         * Roll back PI integrators to prevent windup while saturated.
         */
        float scale = v_limit / vref;
        v_dq.d *= scale;
        v_dq.q *= scale;
        vref = v_limit;
        s_id_int_v = old_id_int_v;
        s_iq_int_v = old_iq_int_v;
    }

    /* Inverse Park converts commanded d-q voltage back to alpha-beta. SVPWM
     * then converts that voltage vector into three timer duty cycles.
     */
    foc_alpha_beta_t v_ab = foc_inv_park(v_dq, theta);
    float duty_u = 0.5f;
    float duty_v = 0.5f;
    float duty_w = 0.5f;
    float modulation = 0.0f;
    foc_apply_svpwm(v_ab.alpha, v_ab.beta, vbus_v, &duty_u, &duty_v, &duty_w, &modulation);

    foc_set_duty(HW_PWM_PHASE_U_CHANNEL, duty_u);
    foc_set_duty(HW_PWM_PHASE_V_CHANNEL, duty_v);
    foc_set_duty(HW_PWM_PHASE_W_CHANNEL, duty_w);

    /* Inverse Clarke is for telemetry only: estimated commanded phase voltages. */
    foc_abc_t v_phase = foc_inv_clarke(v_ab);

    s_status.running = 1U;
    s_status.mode = s_mode;
    s_status.fault = s_fault;
    s_status.id_ref_a = id_ref;
    s_status.iq_ref_a = iq_ref;
    s_status.rads_ref = s_rads_ref;
    s_status.iq_limit_a = s_iq_limit_a;
    s_status.fw_enabled = s_fw_enabled;
    s_status.fw_active = ((s_fw_loop_active != 0U) || (s_fw_id_ref_a < -0.001f)) ? 1U : 0U;
    s_status.fw_id_ref_a = s_fw_id_ref_a;
    s_status.fw_max_current_a = s_fw_max_current_a;
    s_status.fw_base_rads = s_fw_base_rads;
    s_status.fw_voltage_util = s_fw_voltage_util;
    s_status.id_a = i_dq.d;
    s_status.iq_a = i_dq.q;
    s_status.rads_mech = rads_mech;
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

/* Used by phase2_test.c interlocks and can_log.c mode selection. */
uint8_t foc_control_is_running(void)
{
    return s_running;
}

/* Used by phase2_test.c to reject set commands for the wrong active mode. */
foc_control_mode_t foc_control_get_mode(void)
{
    return s_mode;
}

/* Snapshot is copied with IRQs masked because the ADC path updates it at 10 kHz. */
foc_control_status_t foc_control_get_status(void)
{
    foc_control_status_t status;

    __disable_irq();
    status = s_status;
    status.running = s_running;
    status.mode = s_mode;
    status.fault = s_fault;
    status.id_ref_a = (s_mode == FOC_CONTROL_MODE_SPEED) ? s_fw_id_ref_a : s_id_ref_a;
    status.iq_ref_a = s_iq_ref_a;
    status.rads_ref = s_rads_ref;
    status.iq_limit_a = s_iq_limit_a;
    status.fw_enabled = s_fw_enabled;
    status.fw_active = ((s_fw_loop_active != 0U) || (s_fw_id_ref_a < -0.001f)) ? 1U : 0U;
    status.fw_id_ref_a = s_fw_id_ref_a;
    status.fw_max_current_a = s_fw_max_current_a;
    status.fw_base_rads = s_fw_base_rads;
    status.fw_voltage_util = s_fw_voltage_util;
    __enable_irq();

    return status;
}

/* Electrical angle offset calibration, usually adjusted from UART command. */
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

/* Return the configured Hall electrical angle offset in degrees. */
float foc_control_get_theta_offset_deg(void)
{
    return s_theta_offset_rad * (180.0f / FOC_PI);
}

/* Select Hall angle polarity. Wrong sign causes positive Iq to fight rotation. */
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

/* Return current Hall angle polarity. */
int8_t foc_control_get_theta_direction(void)
{
    return s_theta_direction;
}

/* Human-readable UART snapshot for tuning and bench tests. */
void foc_control_print_status(void)
{
    foc_control_status_t st = foc_control_get_status();
    char msg[512];

    snprintf(msg, sizeof(msg),
             "FOC,run=%u,mode=%s,fault=%s,id_ref_mA=%ld,iq_ref_mA=%ld,rads_ref_x10=%ld,rads_mech_x10=%ld,id_mA=%ld,iq_mA=%ld,vd_mV=%ld,vq_mV=%ld,vref_mV=%ld,vlim_mV=%ld,mod_x10000=%ld,fw_en=%u,fw_act=%u,fw_id_mA=%ld,fw_max_mA=%ld,fw_base_x10=%ld,fw_vutil_x1000=%ld,du_x10000=%ld,dv_x10000=%ld,dw_x10000=%ld,theta_dir=%d,theta_off_x100=%ld\r\n",
             st.running,
             foc_mode_name(st.mode),
             foc_fault_name(st.fault),
             (long)foc_float_to_i32_scaled(st.id_ref_a, 1000.0f),
             (long)foc_float_to_i32_scaled(st.iq_ref_a, 1000.0f),
             (long)foc_float_to_i32_scaled(st.rads_ref, 10.0f),
             (long)foc_float_to_i32_scaled(st.rads_mech, 10.0f),
             (long)foc_float_to_i32_scaled(st.id_a, 1000.0f),
             (long)foc_float_to_i32_scaled(st.iq_a, 1000.0f),
             (long)foc_float_to_i32_scaled(st.vd_v, 1000.0f),
             (long)foc_float_to_i32_scaled(st.vq_v, 1000.0f),
             (long)foc_float_to_i32_scaled(st.vref_v, 1000.0f),
             (long)foc_float_to_i32_scaled(st.voltage_limit_v, 1000.0f),
             (long)foc_float_to_i32_scaled(st.modulation, 10000.0f),
             st.fw_enabled,
             st.fw_active,
             (long)foc_float_to_i32_scaled(st.fw_id_ref_a, 1000.0f),
             (long)foc_float_to_i32_scaled(st.fw_max_current_a, 1000.0f),
             (long)foc_float_to_i32_scaled(st.fw_base_rads, 10.0f),
             (long)foc_float_to_i32_scaled(st.fw_voltage_util, 1000.0f),
             (long)foc_float_to_i32_scaled(st.duty_u, 10000.0f),
             (long)foc_float_to_i32_scaled(st.duty_v, 10000.0f),
             (long)foc_float_to_i32_scaled(st.duty_w, 10000.0f),
             (int)s_theta_direction,
             (long)foc_float_to_i32_scaled(foc_control_get_theta_offset_deg(), 100.0f));

    uart_print(msg);
}

/* Clamp helper for currents, speed, voltage and duty limits. */
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

/* Local absolute value helper to avoid double promotion from fabs(). */
static float foc_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* FOC_I_MAX_A is the radius of the safe d-q stator-current circle. Once FW
 * consumes part of that radius with negative Id, only the remaining circle
 * projection is available for torque-producing Iq.
 */
static float foc_iq_limit_for_id(float id_ref_a)
{
    float id_abs = foc_absf(id_ref_a);

    if (id_abs >= FOC_I_MAX_A)
    {
        return 0.0f;
    }

    return sqrtf((FOC_I_MAX_A * FOC_I_MAX_A) - (id_ref_a * id_ref_a));
}

/* Final safety clamp before the current PI: sqrt(Id^2 + Iq^2) <= FOC_I_MAX_A.
 * This is separate from DC bus current, voltage limit, and modulation limit.
 */
static void foc_limit_dq_current_refs(float *id_ref_a, float *iq_ref_a)
{
    if ((id_ref_a == NULL) || (iq_ref_a == NULL))
    {
        return;
    }

    *id_ref_a = foc_clampf(*id_ref_a, -FOC_I_MAX_A, FOC_I_MAX_A);

    float iq_limit = foc_iq_limit_for_id(*id_ref_a);
    *iq_ref_a = foc_clampf(*iq_ref_a, -iq_limit, iq_limit);
}

/* Convert float telemetry values to scaled integers for UART printing. */
static int32_t foc_float_to_i32_scaled(float value, float scale)
{
    float scaled = value * scale;
    return (scaled >= 0.0f) ? (int32_t)(scaled + 0.5f) : (int32_t)(scaled - 0.5f);
}

/* Wrap electrical angle to the range [0, 2*pi). */
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

/* Clear PI integrators and telemetry snapshot before starting a new mode. */
static void foc_reset_control_state(void)
{
    __disable_irq();
    s_id_int_v = 0.0f;
    s_iq_int_v = 0.0f;
    s_speed_int_a = 0.0f;
    s_speed_tick_div = 0U;
    s_fw_loop_active = 0U;
    s_fw_int_a = 0.0f;
    s_fw_id_ref_a = 0.0f;
    s_fw_tick_div = 0U;
    s_fw_vref_sum_v = 0.0f;
    s_fw_vlimit_sum_v = 0.0f;
    s_fw_mech_rads_sum = 0.0f;
    s_fw_sample_count = 0U;
    s_fw_base_rads = 0.0f;
    s_fw_voltage_util = 0.0f;
    memset(&s_status, 0, sizeof(s_status));
    s_status.mode = FOC_CONTROL_MODE_OFF;
    s_status.fault = FOC_CONTROL_FAULT_NONE;
    __enable_irq();
}

/* Enable high- and low-side TIM1 PWM outputs at neutral duty. */
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

/* Foreground PWM stop path used by commands/start-mode transitions. */
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

/* ISR-safe stop path used by fault handling inside foc_control_adc_tick(). */
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

/* Write one TIM1 compare register after clamping duty into the safe range. */
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

/* Space Vector PWM.
 * Input: requested alpha-beta voltage vector.
 * Output: centered duty_u/v/w for TIM1 complementary PWM.
 */
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

    /* Active vector times inside the 60-degree sector plus split zero time. */
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

/* Latch fault state and disable PWM from inside the 10 kHz ADC path. */
static void foc_stop_with_fault_from_isr(foc_control_fault_t fault)
{
    s_running = 0U;
    s_mode = FOC_CONTROL_MODE_OFF;
    s_fault = fault;
    s_fw_loop_active = 0U;
    s_fw_int_a = 0.0f;
    s_fw_id_ref_a = 0.0f;
    s_fw_vref_sum_v = 0.0f;
    s_fw_vlimit_sum_v = 0.0f;
    s_fw_mech_rads_sum = 0.0f;
    s_fw_sample_count = 0U;
    s_status.running = 0U;
    s_status.mode = FOC_CONTROL_MODE_OFF;
    s_status.fault = fault;
    foc_stop_pwm_all_from_isr();
}

/* Text helper used only by foc_control_print_status(). */
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

/* Text helper used only by foc_control_print_status(). */
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
