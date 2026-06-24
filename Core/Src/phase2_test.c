/*
 * phase2_test.c
 *
 *  Created on: Apr 28, 2026
 *      Author: skadi
 */

#include "phase2_test.h"
#include "vesc_uart.h"
#include "foc_control.h"
#include "open_loop.h"
#include "can_log.h"
#include "hw_conf.h"

/*
 * Set 0 while ADC current sense is not yet moved to DMA.
 * This lets phase2_test.c compile without current_sense.c/.h.
 */
#ifndef PHASE2_USE_CURRENT_SENSE
#define PHASE2_USE_CURRENT_SENSE 1
#endif

#if PHASE2_USE_CURRENT_SENSE
#include "adc_sense.h"
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>

/*
 * Handle dari main.c / CubeMX.
 * htim1 dipakai untuk PWM 3-phase.
 */
extern TIM_HandleTypeDef htim1;

/*
 * Konfigurasi Fase 2.
 */
#define PHASE2_UART_BUF_SIZE        64U
#define PHASE2_DEFAULT_DUTY         0.05f
#define PHASE2_GATE_READY_DELAY_MS  10U
#define PHASE2_DC_CAL_SETTLE_MS     2U
#define PHASE2_OL_RPM_MAX           700
#define PHASE2_FOC_RADS_MAX         70
#define PHASE2_FOC_I_MAX_MA         ((int32_t)((FOC_I_MAX_A * 1000.0f) + 0.5f))

/*
 * Asumsi awal:
 * FAULT normal = HIGH
 * FAULT aktif = LOW
 *
 * Ini cocok dengan GPIO FAULT yang Anda set input pull-up.
 * Kalau nanti hasil stat menunjukkan logikanya kebalik,
 * ubah PHASE2_FAULT_ACTIVE_LOW jadi 0.
 */
#define PHASE2_FAULT_ACTIVE_LOW     1U

static uint8_t phase2_parse_two_i32(const char *s, int32_t *a, int32_t *b);
static float phase2_mech_rpm_to_e_hz(int32_t rpm);
static int32_t phase2_float_to_i32_round(float value);
static void phase2_start_high_side(uint32_t channel);

static UART_HandleTypeDef *s_huart = NULL;

static uint8_t s_rx_char;
static char s_rx_buf[PHASE2_UART_BUF_SIZE];
static uint8_t s_rx_idx = 0;

static volatile phase2_inv_state_t s_inv_state = PHASE2_INV_DISABLED;
static volatile uint8_t s_fault_latched = 0;
static volatile uint8_t s_cmd_ready = 0;
static char s_cmd_buf[PHASE2_UART_BUF_SIZE];

static uint8_t s_auto_status_enable = 0;
static uint32_t s_last_status_tick = 0;

static float s_test_duty = PHASE2_DEFAULT_DUTY;

static void phase2_uart_printf(const char *fmt, ...);
static void phase2_process_cmd(char *cmd);
static void phase2_print_dq_config(void);
static uint8_t phase2_foc_precheck(void);
#if PHASE2_USE_CURRENT_SENSE
static void phase2_calibrate_current_offset(void);
#endif

static void phase2_set_duty(uint32_t channel, float duty);
static void phase2_set_all_duty(float duty);

static void phase2_start_low_side(uint32_t channel);
static void phase2_start_complementary(uint32_t channel);

static const char *phase2_state_str(void);
static const char *phase2_channel_name(uint32_t channel);

void phase2_test_init(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    s_rx_idx = 0;
    memset(s_rx_buf, 0, sizeof(s_rx_buf));

    s_inv_state = PHASE2_INV_DISABLED;
    s_fault_latched = 0;
    s_test_duty = PHASE2_DEFAULT_DUTY;

    /*
     * Kondisi boot harus aman:
     * - semua PWM stop
     * - CCR 0
     * - EN_GATE low
     */
    phase2_inverter_disable();
    foc_control_init();
    open_loop_init();

    /*
     * Start UART RX interrupt 1 byte.
     */
    HAL_UART_Receive_IT(s_huart, &s_rx_char, 1);

    uart_print("\r\n");
    uart_print("================================\r\n");
    uart_print("PHASE 2 POWER STAGE TEST READY\r\n");
    uart_print("================================\r\n");
    uart_print("Commands:\r\n");
    uart_print("  help\r\n");
    uart_print("  stat\r\n");
    uart_print("  autostat_on\r\n");
    uart_print("  autostat_off\r\n");
    uart_print("  en\r\n");
    uart_print("  dis\r\n");
    uart_print("  stop\r\n");
    uart_print("  clear_fault\r\n");
    uart_print("  duty <0.0-0.95>\r\n");
    uart_print("  test_ul / test_vl / test_wl\r\n");
    uart_print("  test_uh / test_vh / test_wh\r\n");
    uart_print("  test_u  / test_v  / test_w  complementary\r\n");
#if PHASE2_USE_CURRENT_SENSE
    uart_print("  adc_buf\r\n");
    uart_print("  adc_stat\r\n");
    uart_print("  adc_restart\r\n");
    uart_print("  adc_trig\r\n");
    uart_print("  adc_samp [ccr2]\r\n");
    uart_print("  adc_stats_all\r\n");
    uart_print("  cur_raw\r\n");
    uart_print("  cur_calib\r\n");
    uart_print("  cur_offset\r\n");
    uart_print("  cur_mA\r\n");
    uart_print("  cur_stats\r\n");
    uart_print("  cur_all\r\n");
#else
    uart_print("  current sense commands disabled: ADC DMA not ready\r\n");
#endif
    uart_print("  ol_start <mod_x1000> <freq_mHz>\r\n");
    uart_print("  ol_set <mod_x1000> <freq_mHz>\r\n");
    uart_print("  ol_start_rpm <duty_pct> <rpm>\r\n");
    uart_print("  ol_set_rpm <duty_pct> <rpm>\r\n");
    uart_print("  ol_stop\r\n");
    uart_print("  ol_stat\r\n");
    uart_print("  dq_stat\r\n");
    uart_print("  dq_map <0-5>\r\n");
    uart_print("  dq_theta <1|-1>\r\n");
    uart_print("  dq_offset <deg>\r\n");
    uart_print("  dq_sign <i1_sign> <i3_sign>\r\n");
    uart_print("  foc_start_iq <id_mA> <iq_mA>\r\n");
    uart_print("  foc_set_iq <id_mA> <iq_mA>\r\n");
    uart_print("  foc_start_rads <rads> <iq_limit_mA>\r\n");
    uart_print("  foc_set_rads <rads> <iq_limit_mA>\r\n");
    uart_print("  foc_fw <0|1> <max_id_mA>\r\n");
    uart_print("  foc_stop / foc_stat\r\n");
    uart_print("  foc_theta <deg> / foc_dir <1|-1>\r\n");
    uart_print("\r\n");

    CANLog_Start();
}

void phase2_test_poll(void)
{
	 if (s_cmd_ready)
	    {
	        char cmd_local[PHASE2_UART_BUF_SIZE];

	        __disable_irq();
	        strncpy(cmd_local, s_cmd_buf, sizeof(cmd_local) - 1U);
	        cmd_local[sizeof(cmd_local) - 1U] = '\0';
	        s_cmd_ready = 0U;
	        __enable_irq();

	        phase2_process_cmd(cmd_local);
	    }
    /*
     * Fault dilatch hanya ketika inverter sedang enabled.
     * Ini penting karena beberapa gate driver bisa menunjukkan fault/UVLO
     * saat EN_GATE masih off atau supply belum siap.
     */
    if (s_inv_state == PHASE2_INV_ENABLED)
    {
        if (phase2_fault_active())
        {
            s_fault_latched = 1;
            s_inv_state = PHASE2_INV_FAULT_LATCHED;

            foc_control_stop();
            phase2_pwm_stop_all();
            HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);

            uart_print("ERR,FAULT_LATCHED_PWM_DISABLED\r\n");
        }
    }

    if (s_auto_status_enable)
    {
        if (HAL_GetTick() - s_last_status_tick >= 500U)
        {
            s_last_status_tick = HAL_GetTick();
            phase2_print_status();
        }
    }
    open_loop_poll();
    CANLog_Task();


}

void phase2_test_uart_rx_callback(UART_HandleTypeDef *huart)
{
    if (s_huart == NULL)
    {
        return;
    }

    if (huart->Instance != s_huart->Instance)
    {
        return;
    }

    if (s_rx_char == '\n' || s_rx_char == '\r')
    {
        if (s_rx_idx > 0U)
        {
        	s_rx_buf[s_rx_idx] = '\0';

        	strncpy(s_cmd_buf, s_rx_buf, sizeof(s_cmd_buf) - 1U);
        	s_cmd_buf[sizeof(s_cmd_buf) - 1U] = '\0';
        	s_cmd_ready = 1U;

        	s_rx_idx = 0;
        	memset(s_rx_buf, 0, sizeof(s_rx_buf));
        }
    }
    else
    {
        if (s_rx_idx < (PHASE2_UART_BUF_SIZE - 1U))
        {
            s_rx_buf[s_rx_idx++] = (char)s_rx_char;
        }
        else
        {
            s_rx_idx = 0;
            memset(s_rx_buf, 0, sizeof(s_rx_buf));
            uart_print("ERR,UART_BUFFER_OVERFLOW\r\n");
        }
    }

    HAL_UART_Receive_IT(s_huart, &s_rx_char, 1);
}

uint8_t phase2_fault_active(void)
{
#if PHASE2_FAULT_ACTIVE_LOW
    return (HAL_GPIO_ReadPin(FAULT_GPIO_Port, FAULT_Pin) == GPIO_PIN_RESET);
#else
    return (HAL_GPIO_ReadPin(FAULT_GPIO_Port, FAULT_Pin) == GPIO_PIN_SET);
#endif
}

uint8_t phase2_fault_latched(void)
{
    return s_fault_latched;
}

phase2_inv_state_t phase2_get_inverter_state(void)
{
    return s_inv_state;
}

uint8_t phase2_pwm_is_enabled(void)
{
    uint32_t ccer = htim1.Instance->CCER;

    return ((ccer & (TIM_CCER_CC1E  | TIM_CCER_CC1NE |
                     TIM_CCER_CC2E  | TIM_CCER_CC2NE |
                     TIM_CCER_CC3E  | TIM_CCER_CC3NE)) != 0U) ? 1U : 0U;
}

float phase2_get_test_duty(void)
{
    return s_test_duty;
}

void phase2_pwm_stop_all(void)
{
    /*
     * Stop high-side PWM.
     */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

    /*
     * Stop low-side complementary PWM.
     */
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    /*
     * Paksa compare register ke 0.
     */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    /* Keep TIM1 update TRGO alive for TIM2-synchronized ADC sampling. */
    __HAL_TIM_ENABLE(&htim1);
}

void phase2_inverter_disable(void)
{
    foc_control_stop();
    open_loop_stop();
    phase2_pwm_stop_all();

    HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_RESET);

    /*
     * Disable gate driver.
     */
    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);

    if (s_inv_state != PHASE2_INV_FAULT_LATCHED)
    {
        s_inv_state = PHASE2_INV_DISABLED;
    }
}

void phase2_inverter_enable(void)
{
    if (s_fault_latched)
    {
        uart_print("ERR,FAULT_LATCHED_CLEAR_FIRST\r\n");
        return;
    }

    if (phase2_fault_active())
    {
        s_fault_latched = 1;
        s_inv_state = PHASE2_INV_FAULT_LATCHED;
        phase2_inverter_disable();
        uart_print("ERR,FAULT_ACTIVE_BEFORE_ENABLE\r\n");
        return;
    }

    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_SET);
    HAL_Delay(PHASE2_GATE_READY_DELAY_MS);

    if (phase2_fault_active())
    {
        s_fault_latched = 1;
        s_inv_state = PHASE2_INV_FAULT_LATCHED;
        phase2_inverter_disable();
        uart_print("ERR,FAULT_AFTER_EN_GATE\r\n");
        return;
    }

    s_inv_state = PHASE2_INV_ENABLED;
    uart_print("OK,INV_ENABLED\r\n");
}

void phase2_print_status(void)
{
    phase2_uart_printf(
        "STAT,state=%s,en=%d,dc_cal=%d,fault_pin=%d,fault_active=%d,latched=%d,duty=%.3f,arr=%lu,ccr_u=%lu,ccr_v=%lu,ccr_w=%lu\r\n",
        phase2_state_str(),
        HAL_GPIO_ReadPin(EN_GATE_GPIO_Port, EN_GATE_Pin),
        HAL_GPIO_ReadPin(DC_CAL_GPIO_Port, DC_CAL_Pin),
        HAL_GPIO_ReadPin(FAULT_GPIO_Port, FAULT_Pin),
        phase2_fault_active(),
        s_fault_latched,
        s_test_duty,
        (uint32_t)__HAL_TIM_GET_AUTORELOAD(&htim1),
        (uint32_t)__HAL_TIM_GET_COMPARE(&htim1, HW_PWM_PHASE_U_CHANNEL),
        (uint32_t)__HAL_TIM_GET_COMPARE(&htim1, HW_PWM_PHASE_V_CHANNEL),
        (uint32_t)__HAL_TIM_GET_COMPARE(&htim1, HW_PWM_PHASE_W_CHANNEL)
    );
}

#if PHASE2_USE_CURRENT_SENSE
static void phase2_calibrate_current_offset(void)
{
    open_loop_stop();
    phase2_pwm_stop_all();

    HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_SET);
    HAL_Delay(PHASE2_GATE_READY_DELAY_MS);

    if (phase2_fault_active())
    {
        HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);
        s_fault_latched = 1U;
        s_inv_state = PHASE2_INV_FAULT_LATCHED;
        uart_print("ERR,FAULT_AFTER_EN_GATE_CURRENT_CALIB\r\n");
        return;
    }

    HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_SET);
    HAL_Delay(PHASE2_DC_CAL_SETTLE_MS);

    uart_print("OK,CUR_CALIB_DC_CAL_ACTIVE\r\n");
    uint8_t ok = adc_sense_calibrate_current_offset_sync(ADC_SENSE_OFFSET_DEFAULT_SAMPLES);
    if (ok)
    {
        adc_sense_print_all();
    }

    HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_RESET);
    HAL_Delay(PHASE2_DC_CAL_SETTLE_MS);

    if (ok)
    {
        uart_print("OK,CUR_CALIB_DONE_GATE_STILL_ON\r\n");
    }
    else
    {
        uart_print("ERR,CUR_CALIB_ADC_TIMEOUT\r\n");
    }
}
#endif

static void phase2_process_cmd(char *cmd)
{
    if (strcmp(cmd, "help") == 0)
    {
        uart_print("Commands:\r\n");
        uart_print("  stat\r\n");
        uart_print("  autostat_on / autostat_off\r\n");
        uart_print("  en / dis / stop / clear_fault\r\n");
        uart_print("  duty <0.0-0.95>\r\n");
        uart_print("  test_ul / test_vl / test_wl\r\n");
        uart_print("  test_u  / test_v  / test_w\r\n");
        uart_print("  ol_start <mod_x1000> <freq_mHz>\r\n");
        uart_print("  ol_set <mod_x1000> <freq_mHz>\r\n");
        uart_print("  ol_start_rpm <duty_pct> <rpm>\r\n");
        uart_print("  ol_set_rpm <duty_pct> <rpm>\r\n");
        uart_print("  ol_stop\r\n");
        uart_print("  ol_stat\r\n");
        uart_print("  dq_stat\r\n");
        uart_print("  dq_map <0-5>\r\n");
        uart_print("  dq_theta <1|-1>\r\n");
        uart_print("  dq_offset <deg>\r\n");
        uart_print("  dq_sign <i1_sign> <i3_sign>\r\n");
        uart_print("  foc_start_iq <id_mA> <iq_mA>\r\n");
        uart_print("  foc_set_iq <id_mA> <iq_mA>\r\n");
        uart_print("  foc_start_rads <rads> <iq_limit_mA>\r\n");
        uart_print("  foc_set_rads <rads> <iq_limit_mA>\r\n");
        uart_print("  foc_fw <0|1> <max_id_mA>\r\n");
        uart_print("  foc_stop / foc_stat\r\n");
        uart_print("  foc_theta <deg> / foc_dir <1|-1>\r\n");
#if PHASE2_USE_CURRENT_SENSE
        uart_print("  adc_buf / adc_raw / adc_all / adc_stat\r\n");
        uart_print("  adc_restart / adc_trig\r\n");
        uart_print("  cur_stats / adc_stats_all\r\n");
        uart_print("  adc_samp [ccr2]\r\n");
#endif
    }
    else if (strcmp(cmd, "stat") == 0)
    {
        phase2_print_status();
    }
    else if (strcmp(cmd, "autostat_on") == 0)
    {
        s_auto_status_enable = 1;
        uart_print("OK,AUTOSTAT_ON\r\n");
    }
    else if (strcmp(cmd, "autostat_off") == 0)
    {
        s_auto_status_enable = 0;
        uart_print("OK,AUTOSTAT_OFF\r\n");
    }
    else if (strcmp(cmd, "en") == 0)
    {
        phase2_inverter_enable();
    }
    else if (strcmp(cmd, "dis") == 0)
    {
        foc_control_stop();
        open_loop_stop();
        phase2_inverter_disable();
        uart_print("OK,INV_DISABLED\r\n");
    }
    else if (strcmp(cmd, "stop") == 0)
    {
        foc_control_stop();
        open_loop_stop();
        phase2_pwm_stop_all();
        uart_print("OK,PWM_STOPPED\r\n");
    }
    else if (strcmp(cmd, "clear_fault") == 0)
    {
        if (phase2_fault_active())
        {
            uart_print("ERR,FAULT_PIN_STILL_ACTIVE\r\n");
        }
        else
        {
            s_fault_latched = 0;
            s_inv_state = PHASE2_INV_DISABLED;
            foc_control_stop();
            open_loop_stop();
            phase2_pwm_stop_all();
            HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);
            uart_print("OK,FAULT_CLEARED\r\n");
        }
    }
    else if (strncmp(cmd, "duty ", 5) == 0)
    {
        float duty = (float)atof(&cmd[5]);

        if (duty < 0.0f || duty > 0.95f)
        {
            uart_print("ERR,DUTY_RANGE_0_TO_0.95\r\n");
            return;
        }

        s_test_duty = duty;
        if (!foc_control_is_running())
        {
            phase2_set_all_duty(s_test_duty);
        }

        phase2_uart_printf("OK,DUTY_SET,%.3f\r\n", s_test_duty);
    }
    else if (strcmp(cmd, "test_ul") == 0)
    {
        phase2_start_low_side(HW_PWM_PHASE_U_CHANNEL);
    }
    else if (strcmp(cmd, "test_vl") == 0)
    {
        phase2_start_low_side(HW_PWM_PHASE_V_CHANNEL);
    }
    else if (strcmp(cmd, "test_wl") == 0)
    {
        phase2_start_low_side(HW_PWM_PHASE_W_CHANNEL);
    }
    else if (strcmp(cmd, "test_uh") == 0)
    {
        phase2_start_high_side(HW_PWM_PHASE_U_CHANNEL);
    }
    else if (strcmp(cmd, "test_vh") == 0)
    {
        phase2_start_high_side(HW_PWM_PHASE_V_CHANNEL);
    }
    else if (strcmp(cmd, "test_wh") == 0)
    {
        phase2_start_high_side(HW_PWM_PHASE_W_CHANNEL);
    }
    else if (strcmp(cmd, "test_u") == 0)
    {
        phase2_start_complementary(HW_PWM_PHASE_U_CHANNEL);
    }
    else if (strcmp(cmd, "test_v") == 0)
    {
        phase2_start_complementary(HW_PWM_PHASE_V_CHANNEL);
    }
    else if (strcmp(cmd, "test_w") == 0)
    {
        phase2_start_complementary(HW_PWM_PHASE_W_CHANNEL);
    }
#if PHASE2_USE_CURRENT_SENSE
    else if (strcmp(cmd, "adc_buf") == 0)
    {
        adc_sense_print_dma_buffer();
    }
    else if (strcmp(cmd, "adc_stat") == 0)
    {
        adc_sense_print_status();
    }
    else if (strcmp(cmd, "adc_restart") == 0)
    {
        HAL_StatusTypeDef st = adc_sense_start();

        if (st == HAL_OK)
        {
            uart_print("OK,ADC_RESTARTED\r\n");
        }
        else
        {
            phase2_uart_printf("ERR,ADC_RESTART,%ld\r\n", (long)st);
        }

        adc_sense_print_status();
        adc_sense_print_dma_buffer();
    }
    else if (strcmp(cmd, "adc_trig") == 0)
    {
        adc_sense_force_trigger();
        HAL_Delay(1U);
        adc_sense_print_status();
        adc_sense_print_dma_buffer();
    }
    else if (strcmp(cmd, "adc_raw") == 0 || strcmp(cmd, "cur_raw") == 0)
    {
        adc_sense_print_raw();
    }
    else if (strcmp(cmd, "adc_calib") == 0 || strcmp(cmd, "cur_calib") == 0)
    {
        phase2_calibrate_current_offset();
    }
    else if (strcmp(cmd, "adc_offset") == 0 || strcmp(cmd, "cur_offset") == 0)
    {
        adc_sense_print_offset();
    }
    else if (strcmp(cmd, "cur_mA") == 0)
    {
        adc_sense_print_current();
    }
    else if (strcmp(cmd, "cur_stats") == 0)
    {
        adc_sense_print_current_stats(1000U);
    }
    else if (strcmp(cmd, "adc_stats_all") == 0)
    {
        adc_sense_print_all_rank_stats(1000U);
    }
    else if (strcmp(cmd, "adc_samp") == 0)
    {
        adc_sense_print_sample_timing();
    }
    else if (strncmp(cmd, "adc_samp ", 9) == 0)
    {
        char *endptr;
        long ccr2 = strtol(&cmd[9], &endptr, 10);

        if ((endptr == &cmd[9]) || (ccr2 <= 0L))
        {
            uart_print("ERR,FORMAT_adc_samp_ccr2\r\n");
            return;
        }

        if (adc_sense_set_sample_ccr2((uint32_t)ccr2))
        {
            uart_print("OK,ADC_SAMP_SET\r\n");
            adc_sense_print_sample_timing();
        }
        else
        {
            uart_print("ERR,ADC_SAMP_RANGE\r\n");
        }
    }
    else if (strcmp(cmd, "vin") == 0)
    {
        adc_sense_print_vbus();
    }
    else if (strcmp(cmd, "adc_all") == 0 || strcmp(cmd, "cur_all") == 0)
    {
        adc_sense_print_all();
    }
#else
    else if (strncmp(cmd, "cur_", 4) == 0)
    {
        uart_print("ERR,CURRENT_SENSE_DISABLED_ADC_DMA_NOT_READY\r\n");
    }
#endif
    else if (strncmp(cmd, "ol_start_rpm ", 13) == 0)
    {
        if (s_inv_state != PHASE2_INV_ENABLED)
        {
            uart_print("ERR,INV_NOT_ENABLED_SEND_en_FIRST\r\n");
            return;
        }

        if (s_fault_latched || phase2_fault_active())
        {
            s_fault_latched = 1;
            s_inv_state = PHASE2_INV_FAULT_LATCHED;
            phase2_inverter_disable();
            uart_print("ERR,FAULT_OPEN_LOOP_BLOCKED\r\n");
            return;
        }

        int32_t duty_pct = 0;
        int32_t rpm = 0;

        if (!phase2_parse_two_i32(&cmd[13], &duty_pct, &rpm))
        {
            uart_print("ERR,FORMAT_ol_start_rpm_dutyPct_rpm\r\n");
            return;
        }

        if ((duty_pct < 0) || (duty_pct > 70))
        {
            uart_print("ERR,DUTY_PCT_RANGE_0_TO_70\r\n");
            return;
        }

        if ((rpm < -PHASE2_OL_RPM_MAX) || (rpm > PHASE2_OL_RPM_MAX))
        {
            uart_print("ERR,RPM_RANGE_-200_TO_200\r\n");
            return;
        }

        float mod = (float)duty_pct / 100.0f;
        float freq = phase2_mech_rpm_to_e_hz(rpm);

        phase2_pwm_stop_all();

        foc_control_stop();
        open_loop_start(mod, freq);

        int32_t target_mod_x1000 = phase2_float_to_i32_round(open_loop_get_target_modulation() * 1000.0f);
        int32_t target_rpm = phase2_float_to_i32_round((open_loop_get_target_freq_e_hz() * 60.0f) / MOTOR_POLE_PAIRS);

        phase2_uart_printf("OK,OPEN_LOOP_RPM_STARTED,cmd_duty_pct=%ld,cmd_rpm=%ld,target_mod_x1000=%ld,target_rpm=%ld\r\n",
                           (long)duty_pct,
                           (long)rpm,
                           (long)target_mod_x1000,
                           (long)target_rpm);
    }
    else if (strncmp(cmd, "ol_set_rpm ", 11) == 0)
    {
        int32_t duty_pct = 0;
        int32_t rpm = 0;

        if (!phase2_parse_two_i32(&cmd[11], &duty_pct, &rpm))
        {
            uart_print("ERR,FORMAT_ol_set_rpm_dutyPct_rpm\r\n");
            return;
        }

        if ((duty_pct < 0) || (duty_pct > 70))
        {
            uart_print("ERR,DUTY_PCT_RANGE_0_TO_70\r\n");
            return;
        }

        if ((rpm < -PHASE2_OL_RPM_MAX) || (rpm > PHASE2_OL_RPM_MAX))
        {
            uart_print("ERR,RPM_RANGE_-200_TO_200\r\n");
            return;
        }

        float mod = (float)duty_pct / 100.0f;
        float freq = phase2_mech_rpm_to_e_hz(rpm);

        open_loop_set(mod, freq);

        int32_t target_mod_x1000 = phase2_float_to_i32_round(open_loop_get_target_modulation() * 1000.0f);
        int32_t target_rpm = phase2_float_to_i32_round((open_loop_get_target_freq_e_hz() * 60.0f) / MOTOR_POLE_PAIRS);

        phase2_uart_printf("OK,OPEN_LOOP_RPM_SET,cmd_duty_pct=%ld,cmd_rpm=%ld,target_mod_x1000=%ld,target_rpm=%ld\r\n",
                           (long)duty_pct,
                           (long)rpm,
                           (long)target_mod_x1000,
                           (long)target_rpm);
    }
    else if (strncmp(cmd, "ol_start ", 9) == 0)
    {
        if (s_inv_state != PHASE2_INV_ENABLED)
        {
            uart_print("ERR,INV_NOT_ENABLED_SEND_en_FIRST\r\n");
            return;
        }

        if (s_fault_latched || phase2_fault_active())
        {
            s_fault_latched = 1;
            s_inv_state = PHASE2_INV_FAULT_LATCHED;
            phase2_inverter_disable();
            uart_print("ERR,FAULT_OPEN_LOOP_BLOCKED\r\n");
            return;
        }

        int32_t mod_x1000 = 0;
        int32_t freq_mHz = 0;

        if (!phase2_parse_two_i32(&cmd[9], &mod_x1000, &freq_mHz))
        {
            uart_print("ERR,FORMAT_ol_start_modx1000_freqmHz\r\n");
            return;
        }

        float mod = (float)mod_x1000 / 1000.0f;
        float freq = (float)freq_mHz / 1000.0f;

        phase2_pwm_stop_all();

        foc_control_stop();
        open_loop_start(mod, freq);

        uart_print("OK,OPEN_LOOP_STARTED\r\n");
        open_loop_print_status();
    }
    else if (strncmp(cmd, "ol_set ", 7) == 0)
    {
        int32_t mod_x1000 = 0;
        int32_t freq_mHz = 0;

        if (!phase2_parse_two_i32(&cmd[7], &mod_x1000, &freq_mHz))
        {
            uart_print("ERR,FORMAT_ol_set_modx1000_freqmHz\r\n");
            return;
        }

        float mod = (float)mod_x1000 / 1000.0f;
        float freq = (float)freq_mHz / 1000.0f;

        open_loop_set(mod, freq);

        uart_print("OK,OPEN_LOOP_SET\r\n");
        open_loop_print_status();
    }
    else if (strcmp(cmd, "ol_stop") == 0)
    {
        open_loop_stop();
        uart_print("OK,OPEN_LOOP_STOPPED\r\n");
    }
    else if (strncmp(cmd, "foc_start_iq ", 13) == 0)
    {
        int32_t id_mA = 0;
        int32_t iq_mA = 0;

        if (!phase2_parse_two_i32(&cmd[13], &id_mA, &iq_mA))
        {
            uart_print("ERR,FORMAT_foc_start_iq_idmA_iqmA\r\n");
            return;
        }

        if ((id_mA < -3000) || (id_mA > 3000) || (iq_mA < -3000) || (iq_mA > 3000))
        {
            uart_print("ERR,FOC_CURRENT_RANGE_-3000_TO_3000_mA\r\n");
            return;
        }

        if (!phase2_foc_precheck())
        {
            return;
        }

        open_loop_stop();

        if (foc_control_start_current((float)id_mA * 0.001f, (float)iq_mA * 0.001f))
        {
            uart_print("OK,FOC_CURRENT_STARTED\r\n");
            foc_control_print_status();
        }
        else
        {
            uart_print("ERR,FOC_CURRENT_START\r\n");
            foc_control_print_status();
        }
    }
    else if (strncmp(cmd, "foc_set_iq ", 11) == 0)
    {
        int32_t id_mA = 0;
        int32_t iq_mA = 0;

        if (!phase2_parse_two_i32(&cmd[11], &id_mA, &iq_mA))
        {
            uart_print("ERR,FORMAT_foc_set_iq_idmA_iqmA\r\n");
            return;
        }

        if (foc_control_get_mode() != FOC_CONTROL_MODE_CURRENT)
        {
            uart_print("ERR,FOC_NOT_IN_CURRENT_MODE\r\n");
            return;
        }

        foc_control_set_current_refs((float)id_mA * 0.001f, (float)iq_mA * 0.001f);
        uart_print("OK,FOC_CURRENT_SET\r\n");
        foc_control_print_status();
    }
    else if (strncmp(cmd, "foc_start_rads ", 15) == 0)
    {
        int32_t rads = 0;
        int32_t iq_limit_mA = 0;

        if (!phase2_parse_two_i32(&cmd[15], &rads, &iq_limit_mA))
        {
            uart_print("ERR,FORMAT_foc_start_rads_rads_iqLimitmA\r\n");
            return;
        }

        if ((rads < -PHASE2_FOC_RADS_MAX) || (rads > PHASE2_FOC_RADS_MAX))
        {
            uart_print("ERR,RADS_RANGE_-70_TO_70\r\n");
            return;
        }

        if ((iq_limit_mA < 100) || (iq_limit_mA > PHASE2_FOC_I_MAX_MA))
        {
            uart_print("ERR,FOC_IQ_LIMIT_RANGE_100_TO_I_MAX_mA\r\n");
            return;
        }

        if (!phase2_foc_precheck())
        {
            return;
        }

        open_loop_stop();

        if (foc_control_start_speed((float)rads, (float)iq_limit_mA * 0.001f))
        {
            uart_print("OK,FOC_SPEED_STARTED\r\n");
            foc_control_print_status();
        }
        else
        {
            uart_print("ERR,FOC_SPEED_START\r\n");
            foc_control_print_status();
        }
    }
    else if (strncmp(cmd, "foc_set_rads ", 13) == 0)
    {
        int32_t rads = 0;
        int32_t iq_limit_mA = 0;

        if (!phase2_parse_two_i32(&cmd[13], &rads, &iq_limit_mA))
        {
            uart_print("ERR,FORMAT_foc_set_rads_rads_iqLimitmA\r\n");
            return;
        }

        if (foc_control_get_mode() != FOC_CONTROL_MODE_SPEED)
        {
            uart_print("ERR,FOC_NOT_IN_SPEED_MODE\r\n");
            return;
        }

        if ((rads < -PHASE2_FOC_RADS_MAX) || (rads > PHASE2_FOC_RADS_MAX))
        {
            uart_print("ERR,RADS_RANGE_-70_TO_70\r\n");
            return;
        }

        if ((iq_limit_mA < 100) || (iq_limit_mA > PHASE2_FOC_I_MAX_MA))
        {
            uart_print("ERR,FOC_IQ_LIMIT_RANGE_100_TO_I_MAX_mA\r\n");
            return;
        }

        foc_control_set_speed_ref((float)rads, (float)iq_limit_mA * 0.001f);
        uart_print("OK,FOC_SPEED_SET\r\n");
        foc_control_print_status();
    }
    else if (strncmp(cmd, "foc_fw ", 7) == 0)
    {
        int32_t enable = 0;
        int32_t max_id_mA = 0;

        if (!phase2_parse_two_i32(&cmd[7], &enable, &max_id_mA))
        {
            uart_print("ERR,FORMAT_foc_fw_enable_maxIdmA\r\n");
            return;
        }

        if ((enable != 0) && (enable != 1))
        {
            uart_print("ERR,FOC_FW_ENABLE_0_OR_1\r\n");
            return;
        }

        if ((max_id_mA < 0) || (max_id_mA > PHASE2_FOC_I_MAX_MA))
        {
            uart_print("ERR,FOC_FW_MAX_RANGE_0_TO_I_MAX_mA\r\n");
            return;
        }

        foc_control_set_field_weakening((uint8_t)enable, (float)max_id_mA * 0.001f);
        uart_print("OK,FOC_FW_SET\r\n");
        foc_control_print_status();
    }
    else if (strcmp(cmd, "foc_stop") == 0)
    {
        foc_control_stop();
        uart_print("OK,FOC_STOPPED\r\n");
    }
    else if (strcmp(cmd, "foc_stat") == 0)
    {
        foc_control_print_status();
    }
    else if (strncmp(cmd, "foc_theta ", 10) == 0)
    {
        float offset_deg = (float)atof(&cmd[10]);
        foc_control_set_theta_offset_deg(offset_deg);
        uart_print("OK,FOC_THETA_OFFSET_SET\r\n");
        foc_control_print_status();
    }
    else if (strncmp(cmd, "foc_dir ", 8) == 0)
    {
        char *endptr;
        long dir = strtol(&cmd[8], &endptr, 10);

        if ((endptr == &cmd[8]) || ((dir != 1L) && (dir != -1L)))
        {
            uart_print("ERR,FOC_DIR_USE_1_OR_-1\r\n");
            return;
        }

        if (!foc_control_set_theta_direction((int8_t)dir))
        {
            uart_print("ERR,FOC_DIR_SET\r\n");
            return;
        }

        uart_print("OK,FOC_DIR_SET\r\n");
        foc_control_print_status();
    }
    else if (strcmp(cmd, "ol_stat") == 0)
    {
        open_loop_print_status();
    }
    else if (strcmp(cmd, "dq_stat") == 0)
    {
        phase2_print_dq_config();
    }
    else if (strncmp(cmd, "dq_map ", 7) == 0)
    {
        char *endptr;
        long map = strtol(&cmd[7], &endptr, 10);

        if ((endptr == &cmd[7]) || (map < 0L) || (map >= (long)CANLOG_DQ_MAP_COUNT))
        {
            uart_print("ERR,DQ_MAP_RANGE_0_TO_5\r\n");
            return;
        }

        if (!CANLog_SetDQMap((uint8_t)map))
        {
            uart_print("ERR,DQ_MAP_SET\r\n");
            return;
        }

        uart_print("OK,DQ_MAP_SET\r\n");
        phase2_print_dq_config();
    }
    else if (strncmp(cmd, "dq_theta ", 9) == 0)
    {
        char *endptr;
        long dir = strtol(&cmd[9], &endptr, 10);

        if ((endptr == &cmd[9]) || ((dir != 1L) && (dir != -1L)))
        {
            uart_print("ERR,DQ_THETA_USE_1_OR_-1\r\n");
            return;
        }

        if (!CANLog_SetDQThetaDirection((int8_t)dir))
        {
            uart_print("ERR,DQ_THETA_SET\r\n");
            return;
        }

        uart_print("OK,DQ_THETA_SET\r\n");
        phase2_print_dq_config();
    }
    else if (strncmp(cmd, "dq_offset ", 10) == 0)
    {
        float offset_deg = (float)atof(&cmd[10]);

        if ((offset_deg < -360.0f) || (offset_deg > 360.0f))
        {
            uart_print("ERR,DQ_OFFSET_RANGE_-360_TO_360\r\n");
            return;
        }

        CANLog_SetDQThetaOffsetDeg(offset_deg);
        uart_print("OK,DQ_OFFSET_SET\r\n");
        phase2_print_dq_config();
    }
    else if (strncmp(cmd, "dq_sign ", 8) == 0)
    {
        int32_t i1_sign = 0;
        int32_t i3_sign = 0;

        if (!phase2_parse_two_i32(&cmd[8], &i1_sign, &i3_sign))
        {
            uart_print("ERR,FORMAT_dq_sign_i1_i3\r\n");
            return;
        }

        if (((i1_sign != 1) && (i1_sign != -1)) ||
            ((i3_sign != 1) && (i3_sign != -1)))
        {
            uart_print("ERR,DQ_SIGN_USE_1_OR_-1\r\n");
            return;
        }

        if (!CANLog_SetDQCurrentSigns((int8_t)i1_sign, (int8_t)i3_sign))
        {
            uart_print("ERR,DQ_SIGN_SET\r\n");
            return;
        }

        uart_print("OK,DQ_SIGN_SET\r\n");
        phase2_print_dq_config();
    }
    else
    {
        uart_print("ERR,UNKNOWN_CMD\r\n");
    }
}

static void phase2_print_dq_config(void)
{
    uint8_t map = CANLog_GetDQMap();

    phase2_uart_printf(
        "DQ_CFG,map=%u,%s,theta_dir=%d,theta_offset_deg=%.2f,i1_sign=%d,i3_sign=%d\r\n",
        map,
        CANLog_GetDQMapName(map),
        (int)CANLog_GetDQThetaDirection(),
        CANLog_GetDQThetaOffsetDeg(),
        (int)CANLog_GetDQI1Sign(),
        (int)CANLog_GetDQI3Sign());

    uart_print("DQ_MAPS,0:i1=A,i3=C 1:i3=A,i1=C 2:i1=A,i3=B 3:i3=A,i1=B 4:i1=B,i3=C 5:i3=B,i1=C\r\n");
}

static uint8_t phase2_foc_precheck(void)
{
    if (s_inv_state != PHASE2_INV_ENABLED)
    {
        uart_print("ERR,INV_NOT_ENABLED_SEND_en_FIRST\r\n");
        return 0U;
    }

    if (s_fault_latched || phase2_fault_active())
    {
        s_fault_latched = 1;
        s_inv_state = PHASE2_INV_FAULT_LATCHED;
        phase2_inverter_disable();
        uart_print("ERR,FAULT_FOC_BLOCKED\r\n");
        return 0U;
    }

#if PHASE2_USE_CURRENT_SENSE
    adc_sense_data_t adc = adc_sense_get_data();
    if ((adc.dma_started == 0U) || (adc.current_calibrated == 0U))
    {
        uart_print("ERR,FOC_NEEDS_cur_calib_FIRST\r\n");
        return 0U;
    }
#else
    uart_print("ERR,FOC_NEEDS_CURRENT_SENSE\r\n");
    return 0U;
#endif

    return 1U;
}

static void phase2_start_low_side(uint32_t channel)
{
    if (s_inv_state != PHASE2_INV_ENABLED)
    {
        uart_print("ERR,INV_NOT_ENABLED_SEND_en_FIRST\r\n");
        return;
    }

    if (s_fault_latched || phase2_fault_active())
    {
        s_fault_latched = 1;
        s_inv_state = PHASE2_INV_FAULT_LATCHED;
        phase2_inverter_disable();
        uart_print("ERR,FAULT_PWM_BLOCKED\r\n");
        return;
    }

    /*
     * Stop semua dulu supaya hanya satu low-side yang aktif.
     */
    foc_control_stop();
    phase2_pwm_stop_all();

    phase2_set_duty(channel, s_test_duty);

    /*
     * Start CHxN saja:
     * CH1N = UL
     * CH2N = VL
     * CH3N = WL
     */
    HAL_TIMEx_PWMN_Start(&htim1, channel);

    phase2_uart_printf("OK,LOW_SIDE_STARTED,%s,duty=%.3f\r\n",
                       phase2_channel_name(channel),
                       s_test_duty);
}

static void phase2_start_high_side(uint32_t channel)
{
    if (s_inv_state != PHASE2_INV_ENABLED)
    {
        uart_print("ERR,INV_NOT_ENABLED_SEND_en_FIRST\r\n");
        return;
    }

    if (s_fault_latched || phase2_fault_active())
    {
        s_fault_latched = 1;
        s_inv_state = PHASE2_INV_FAULT_LATCHED;
        phase2_inverter_disable();
        uart_print("ERR,FAULT_PWM_BLOCKED\r\n");
        return;
    }

    /*
     * Stop semua dulu supaya hanya satu high-side yang aktif.
     */
    foc_control_stop();
    phase2_pwm_stop_all();

    phase2_set_duty(channel, s_test_duty);

    /*
     * Start CHx saja:
     * CH1 = UH
     * CH2 = VH
     * CH3 = WH
     */
    HAL_TIM_PWM_Start(&htim1, channel);

    phase2_uart_printf("OK,HIGH_SIDE_STARTED,%s,duty=%.3f\r\n",
                       phase2_channel_name(channel),
                       s_test_duty);
}

static void phase2_start_complementary(uint32_t channel)
{
    if (s_inv_state != PHASE2_INV_ENABLED)
    {
        uart_print("ERR,INV_NOT_ENABLED_SEND_en_FIRST\r\n");
        return;
    }

    if (s_fault_latched || phase2_fault_active())
    {
        s_fault_latched = 1;
        s_inv_state = PHASE2_INV_FAULT_LATCHED;
        phase2_inverter_disable();
        uart_print("ERR,FAULT_PWM_BLOCKED\r\n");
        return;
    }

    /*
     * Stop semua dulu supaya hanya satu leg yang aktif.
     */
    foc_control_stop();
    phase2_pwm_stop_all();

    phase2_set_duty(channel, s_test_duty);

    /*
     * Start high-side + low-side complementary satu leg.
     * Deadtime otomatis mengikuti konfigurasi TIM1.
     */
    HAL_TIM_PWM_Start(&htim1, channel);     // CHx
    HAL_TIMEx_PWMN_Start(&htim1, channel);  // CHxN

    phase2_uart_printf("OK,COMPLEMENTARY_STARTED,%s,duty=%.3f\r\n",
                       phase2_channel_name(channel),
                       s_test_duty);
}

static void phase2_set_duty(uint32_t channel, float duty)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t pulse = (uint32_t)((float)(arr + 1U) * duty);

    if (pulse > arr)
    {
        pulse = arr;
    }

    __HAL_TIM_SET_COMPARE(&htim1, channel, pulse);
}

static void phase2_set_all_duty(float duty)
{
    phase2_set_duty(HW_PWM_PHASE_U_CHANNEL, duty);
    phase2_set_duty(HW_PWM_PHASE_V_CHANNEL, duty);
    phase2_set_duty(HW_PWM_PHASE_W_CHANNEL, duty);
}

static const char *phase2_state_str(void)
{
    switch (s_inv_state)
    {
        case PHASE2_INV_DISABLED:
            return "DISABLED";

        case PHASE2_INV_ENABLED:
            return "ENABLED";

        case PHASE2_INV_FAULT_LATCHED:
            return "FAULT";

        default:
            return "UNKNOWN";
    }
}

static const char *phase2_channel_name(uint32_t channel)
{
    switch (channel)
    {
        case HW_PWM_PHASE_U_CHANNEL:
            return "U";

        case HW_PWM_PHASE_V_CHANNEL:
            return "V";

        case HW_PWM_PHASE_W_CHANNEL:
            return "W";

        default:
            return "UNKNOWN";
    }
}

static float phase2_mech_rpm_to_e_hz(int32_t rpm)
{
    return ((float)rpm * MOTOR_POLE_PAIRS) / 60.0f;
}

static int32_t phase2_float_to_i32_round(float value)
{
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) : (int32_t)(value - 0.5f);
}

static uint8_t phase2_parse_two_i32(const char *s, int32_t *a, int32_t *b)
{
    char *endptr;

    long v1 = strtol(s, &endptr, 10);

    if (endptr == s)
    {
        return 0U;
    }

    while (*endptr == ' ')
    {
        endptr++;
    }

    char *p2 = endptr;
    long v2 = strtol(p2, &endptr, 10);

    if (endptr == p2)
    {
        return 0U;
    }

    *a = (int32_t)v1;
    *b = (int32_t)v2;

    return 1U;
}

static void phase2_uart_printf(const char *fmt, ...)
{
    char buf[192];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    uart_print(buf);
}
