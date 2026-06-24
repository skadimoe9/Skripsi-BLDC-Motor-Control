#include "adc_sense.h"
#include "hw_conf.h"
#include "foc_control.h"
#include "open_loop.h"
#include "hall_sensor.h"
#include "vesc_uart.h"

#include <stdio.h>
#include <string.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

#ifndef ADC_MAX_12BIT
#define ADC_MAX_12BIT       4095.0f
#endif

#ifndef ADC_VREF
#define ADC_VREF            3.3f
#endif

#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES   0.001f
#endif

#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN    10.0f
#endif

#ifndef VIN_DIV_RATIO
#define VIN_DIV_RATIO       11.0f
#endif

#ifndef VPHASE_DIV_RATIO
#define VPHASE_DIV_RATIO    VIN_DIV_RATIO
#endif

#ifndef ADC_SENSE_CURRENT_LPF_ALPHA
#define ADC_SENSE_CURRENT_LPF_ALPHA  0.35f
#endif

#ifndef ADC_SENSE_CONTROL_TICK_HZ
#define ADC_SENSE_CONTROL_TICK_HZ    10000.0f
#endif

#define ADC_SENSE_CONTROL_DT_S       (1.0f / ADC_SENSE_CONTROL_TICK_HZ)

/*
 * ADC trigger chain:
 *   TIM1 TRGO update -> TIM2 reset -> TIM2 CH2 compare at CCR2 -> ADC TIM2_CC2 trigger.
 * Keep this enabled so ADC samples remain locked to the PWM timebase.
 */
#ifndef ADC_SENSE_TIM2_SYNC_TO_TIM1
#define ADC_SENSE_TIM2_SYNC_TO_TIM1  1U
#endif

#if ((ADC_SENSE_I1_INDEX >= ADC_SENSE_ADC1_DMA_LEN) || (ADC_SENSE_I1_INDEX >= ADC_SENSE_ADC2_DMA_LEN))
#error "ADC_SENSE_I1_INDEX outside ADC DMA length"
#endif

#if ((ADC_SENSE_I3_INDEX >= ADC_SENSE_ADC1_DMA_LEN) || (ADC_SENSE_I3_INDEX >= ADC_SENSE_ADC2_DMA_LEN))
#error "ADC_SENSE_I3_INDEX outside ADC DMA length"
#endif

#if (ADC_SENSE_VBUS_INDEX >= ADC_SENSE_ADC3_DMA_LEN)
#error "ADC_SENSE_VBUS_INDEX outside ADC_SENSE_ADC3_DMA_LEN"
#endif

#if (ADC3_IDX_PHASE_A >= ADC_SENSE_ADC3_DMA_LEN)
#error "ADC3_IDX_PHASE_A outside ADC_SENSE_ADC3_DMA_LEN"
#endif

#if (ADC2_IDX_SENS2 >= ADC_SENSE_ADC2_DMA_LEN)
#error "ADC2_IDX_SENS2 outside ADC_SENSE_ADC2_DMA_LEN"
#endif

#if (ADC1_IDX_SENS3 >= ADC_SENSE_ADC1_DMA_LEN)
#error "ADC1_IDX_SENS3 outside ADC_SENSE_ADC1_DMA_LEN"
#endif

#if ((ADC_SENSE_I1_FROM_ADC != ADC_SENSE_FROM_ADC1) && (ADC_SENSE_I1_FROM_ADC != ADC_SENSE_FROM_ADC2))
#error "ADC_SENSE_I1_FROM_ADC must be ADC_SENSE_FROM_ADC1 or ADC_SENSE_FROM_ADC2"
#endif

#if ((ADC_SENSE_I3_FROM_ADC != ADC_SENSE_FROM_ADC1) && (ADC_SENSE_I3_FROM_ADC != ADC_SENSE_FROM_ADC2))
#error "ADC_SENSE_I3_FROM_ADC must be ADC_SENSE_FROM_ADC1 or ADC_SENSE_FROM_ADC2"
#endif

/* ADC1, ADC2, and ADC3 independent regular DMA buffers. */
static uint16_t s_adc1_dma_buf[ADC_SENSE_ADC1_DMA_LEN];
static uint16_t s_adc2_dma_buf[ADC_SENSE_ADC2_DMA_LEN];
static uint16_t s_adc3_dma_buf[ADC_SENSE_ADC3_DMA_LEN];

static adc_sense_data_t s_adc;

static volatile uint32_t s_adc1_dma_half_count;
static volatile uint32_t s_adc1_dma_full_count;
static volatile uint32_t s_adc2_dma_half_count;
static volatile uint32_t s_adc2_dma_full_count;
static volatile uint32_t s_adc3_dma_half_count;
static volatile uint32_t s_adc3_dma_full_count;
static volatile uint32_t s_adc1_error_count;
static volatile uint32_t s_adc2_error_count;
static volatile uint32_t s_adc3_error_count;
static volatile uint32_t s_adc_current_frame_count;
static volatile uint32_t s_adc_current_sync_miss_count;

static volatile uint8_t s_adc1_current_frame_ready;
static volatile uint8_t s_adc2_current_frame_ready;
static volatile uint8_t s_current_filter_ready;

static HAL_StatusTypeDef s_start_adc1_status = HAL_OK;
static HAL_StatusTypeDef s_start_adc2_status = HAL_OK;
static HAL_StatusTypeDef s_start_adc3_status = HAL_OK;
static HAL_StatusTypeDef s_start_tim1_status = HAL_OK;
static HAL_StatusTypeDef s_config_tim2_status = HAL_OK;
static HAL_StatusTypeDef s_start_tim2_status = HAL_OK;

static uint16_t adc_sense_selected_raw(uint32_t index, uint32_t adc_sel);
static uint8_t adc_sense_wait_new_frame(uint32_t timeout_ms, uint8_t include_adc3);
static uint8_t adc_sense_wait_new_current_frame(uint32_t timeout_ms);
static uint8_t adc_sense_wait_new_all_frame(uint32_t timeout_ms);
static uint8_t adc_sense_update_after_new_frame(uint32_t timeout_ms, uint8_t include_adc3);
static void adc_sense_update_coherent(uint8_t include_adc3);
static void adc_sense_copy_dma_snapshot(uint8_t include_adc3);
static void adc_sense_update_current_from_snapshot(void);
static void adc_sense_reset_current_filter(float i1_a, float i3_a);
static void adc_sense_try_process_current_frame_from_isr(void);
static void adc_sense_process_current_frame_from_isr(void);
static void adc_sense_print_rank_stats_line(const char *tag,
                                            const uint16_t *min_v,
                                            const uint16_t *max_v,
                                            const uint64_t *sum_v,
                                            uint32_t samples);
static float adc_sense_raw_to_pin_voltage(uint16_t raw);
static float adc_sense_raw_to_bus_voltage(uint16_t raw);
static float adc_sense_raw_to_phase_voltage(uint16_t raw);
static int32_t adc_sense_float_to_mA(float current_a);
static uint32_t adc_sense_float_to_mV(float voltage_v);
static uint32_t adc_sense_float_to_cV(float voltage_v);
static uint32_t adc_sense_scale_to_uA_per_count(float scale_a_per_count);
//static HAL_StatusTypeDef adc_sense_config_tim2_oc2ref_trigger(void);
//static void adc_sense_prepare_tim2_trigger(void);

void adc_sense_init(void)
{
    memset(s_adc1_dma_buf, 0, sizeof(s_adc1_dma_buf));
    memset(s_adc2_dma_buf, 0, sizeof(s_adc2_dma_buf));
    memset(s_adc3_dma_buf, 0, sizeof(s_adc3_dma_buf));
    memset(&s_adc, 0, sizeof(s_adc));

    /* Default midpoint sensor current 12-bit. Calibrate before using real current. */
    s_adc.offset_i1 = ADC_MAX_12BIT * 0.5f;
    s_adc.offset_i3 = ADC_MAX_12BIT * 0.5f;

    s_adc.scale_a_per_count =
        (ADC_VREF / ADC_MAX_12BIT) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);

    s_adc.current_calibrated = 0U;
    s_adc.dma_started = 0U;

    s_adc1_dma_half_count = 0U;
    s_adc1_dma_full_count = 0U;
    s_adc2_dma_half_count = 0U;
    s_adc2_dma_full_count = 0U;
    s_adc3_dma_half_count = 0U;
    s_adc3_dma_full_count = 0U;
    s_adc1_error_count = 0U;
    s_adc2_error_count = 0U;
    s_adc3_error_count = 0U;
    s_adc_current_frame_count = 0U;
    s_adc_current_sync_miss_count = 0U;
    s_adc1_current_frame_ready = 0U;
    s_adc2_current_frame_ready = 0U;
    s_current_filter_ready = 0U;

    s_start_adc1_status = HAL_OK;
    s_start_adc2_status = HAL_OK;
    s_start_adc3_status = HAL_OK;
    s_start_tim1_status = HAL_OK;
    s_config_tim2_status = HAL_OK;
    s_start_tim2_status = HAL_OK;
}

HAL_StatusTypeDef adc_sense_start(void)
{
    HAL_StatusTypeDef st;

    s_adc.dma_started = 0U;
    s_adc1_current_frame_ready = 0U;
    s_adc2_current_frame_ready = 0U;
    s_current_filter_ready = 0U;

    /* Make restart from UART deterministic. Ignore stop return values here. */
    HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_2);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc3);
    __HAL_TIM_DISABLE(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE | TIM_FLAG_CC2);

    //adc_sense_prepare_tim2_trigger();

    //st = adc_sense_config_tim2_oc2ref_trigger();
    st = HAL_OK;
    s_config_tim2_status = st;
    if (st != HAL_OK)
    {
        return st;
    }

    /* TIM1 must keep running even when PWM outputs are off; its update TRGO resets TIM2. */
    __HAL_TIM_ENABLE(&htim1);
    s_start_tim1_status = HAL_OK;

    /* Start all ADC DMA streams first. External TIM2_CC2 triggers fill them together. */
    st = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc1_dma_buf, ADC_SENSE_ADC1_DMA_LEN);
    s_start_adc1_status = st;
    if (st != HAL_OK)
    {
        return st;
    }

    st = HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_adc2_dma_buf, ADC_SENSE_ADC2_DMA_LEN);
    s_start_adc2_status = st;
    if (st != HAL_OK)
    {
        HAL_ADC_Stop_DMA(&hadc1);
        return st;
    }

    st = HAL_ADC_Start_DMA(&hadc3, (uint32_t *)s_adc3_dma_buf, ADC_SENSE_ADC3_DMA_LEN);
    s_start_adc3_status = st;
    if (st != HAL_OK)
    {
        HAL_ADC_Stop_DMA(&hadc2);
        HAL_ADC_Stop_DMA(&hadc1);
        return st;
    }

    /* ADCs are configured for TIM2_CC2 rising edge, generated at CCR2. */
    st = HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2);
    s_start_tim2_status = st;
    if (st != HAL_OK)
    {
        HAL_ADC_Stop_DMA(&hadc3);
        HAL_ADC_Stop_DMA(&hadc2);
        HAL_ADC_Stop_DMA(&hadc1);
        return st;
    }

    /* Move TIM2 just before CCR2 so adc_buf should change even before waiting long. */
    adc_sense_force_trigger();

    s_adc.dma_started = 1U;
    return HAL_OK;
}

void adc_sense_stop(void)
{
    HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_2);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc3);
    s_adc.dma_started = 0U;
    s_adc1_current_frame_ready = 0U;
    s_adc2_current_frame_ready = 0U;
}

void adc_sense_force_trigger(void)
{
    uint32_t ccr2 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);

    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC2);

    if (ccr2 > 1U)
    {
        __HAL_TIM_SET_COUNTER(&htim2, ccr2 - 1U);
    }
    else
    {
        __HAL_TIM_SET_COUNTER(&htim2, 0U);
    }
}

uint8_t adc_sense_set_sample_ccr2(uint32_t ccr2)
{
    if (htim2.Instance == NULL)
    {
        return 0U;
    }

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
    if ((ccr2 == 0U) || (ccr2 >= arr))
    {
        return 0U;
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr2);
    return 1U;
}

uint32_t adc_sense_get_sample_ccr2(void)
{
    if (htim2.Instance == NULL)
    {
        return 0U;
    }

    return __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
}

void adc_sense_update(void)
{
    adc_sense_copy_dma_snapshot(1U);
}

uint8_t adc_sense_calibrate_current_offset(uint32_t samples)
{
    return adc_sense_calibrate_current_offset_sync(samples);
}

uint8_t adc_sense_calibrate_current_offset_sync(uint32_t samples)
{
    if (samples == 0U)
    {
        samples = ADC_SENSE_OFFSET_DEFAULT_SAMPLES;
    }

    uint64_t sum_i1 = 0ULL;
    uint64_t sum_i3 = 0ULL;

    for (uint32_t i = 0U; i < 50U; i++)
    {
        if (!adc_sense_wait_new_current_frame(10U))
        {
            return 0U;
        }
    }

    for (uint32_t i = 0U; i < samples; i++)
    {
        if (!adc_sense_wait_new_current_frame(10U))
        {
            return 0U;
        }

        adc_sense_update_coherent(0U);
        sum_i1 += s_adc.raw_i1;
        sum_i3 += s_adc.raw_i3;
    }

    s_adc.offset_i1 = (float)sum_i1 / (float)samples;
    s_adc.offset_i3 = (float)sum_i3 / (float)samples;
    s_adc.current_calibrated = 1U;
    adc_sense_reset_current_filter(0.0f, 0.0f);

    adc_sense_update();
    return 1U;
}

adc_sense_data_t adc_sense_get_data(void)
{
    adc_sense_data_t data;

    __disable_irq();
    adc_sense_copy_dma_snapshot(1U);
    data = s_adc;
    __enable_irq();

    return data;
}

float adc_sense_get_i1_a(void)
{
    float current;

    __disable_irq();
    current = s_adc.i1_a;
    __enable_irq();

    return current;
}

float adc_sense_get_i2_a(void)
{
    float current;

    __disable_irq();
    current = s_adc.i2_a;
    __enable_irq();

    return current;
}

float adc_sense_get_i3_a(void)
{
    float current;

    __disable_irq();
    current = s_adc.i3_a;
    __enable_irq();

    return current;
}

float adc_sense_get_vbus_v(void)
{
    adc_sense_update();
    return s_adc.vbus_v;
}

void adc_sense_print_raw(void)
{
    if (!adc_sense_update_after_new_frame(10U, 1U))
    {
        uart_print("ERR,ADC_RAW_TIMEOUT\r\n");
        return;
    }

    char msg[192];
    snprintf(msg, sizeof(msg),
             "ADC_RAW,i1=%u,i3=%u,vbus=%u,dma=%u,cal=%u\r\n",
             s_adc.raw_i1,
             s_adc.raw_i3,
             s_adc.raw_vbus,
             s_adc.dma_started,
             s_adc.current_calibrated);

    uart_print(msg);
}

void adc_sense_print_dma_buffer(void)
{
    adc_sense_update();

    char msg[256];
    snprintf(msg, sizeof(msg),
             "ADC_BUF,a1r1=%u,a2r1=%u,a1r2=%u,a2r2=%u,a1r3=%u,a2r3=%u,a3r1=%u,a3r2=%u,a3r3=%u\r\n",
             s_adc.adc1_raw[0], s_adc.adc2_raw[0],
             s_adc.adc1_raw[1], s_adc.adc2_raw[1],
             s_adc.adc1_raw[2], s_adc.adc2_raw[2],
             s_adc.adc3_raw[0], s_adc.adc3_raw[1], s_adc.adc3_raw[2]);

    uart_print(msg);
}

void adc_sense_print_current(void)
{
    adc_sense_update();

    int32_t i1_mA = adc_sense_float_to_mA(s_adc.i1_a);
    int32_t i2_mA = adc_sense_float_to_mA(s_adc.i2_a);
    int32_t i3_mA = adc_sense_float_to_mA(s_adc.i3_a);

    char msg[160];
    snprintf(msg, sizeof(msg),
             "CUR_mA,i1=%ld,i2=%ld,i3=%ld,cal=%u\r\n",
             (long)i1_mA,
             (long)i2_mA,
             (long)i3_mA,
             s_adc.current_calibrated);

    uart_print(msg);
}

void adc_sense_print_current_stats(uint32_t samples)
{
    if (samples == 0U)
    {
        samples = 1000U;
    }

    uint16_t min_i1 = 0xFFFFU;
    uint16_t min_i3 = 0xFFFFU;
    uint16_t max_i1 = 0U;
    uint16_t max_i3 = 0U;
    float sum_delta_i1 = 0.0f;
    float sum_delta_i3 = 0.0f;
    float min_i1_a = 1.0e30f;
    float max_i1_a = -1.0e30f;
    float min_i3_a = 1.0e30f;
    float max_i3_a = -1.0e30f;

    for (uint32_t i = 0U; i < samples; i++)
    {
        if (!adc_sense_wait_new_current_frame(10U))
        {
            uart_print("ERR,CUR_STATS_ADC_TIMEOUT\r\n");
            return;
        }

        adc_sense_update_coherent(0U);

        if (s_adc.raw_i1 < min_i1)
        {
            min_i1 = s_adc.raw_i1;
        }
        if (s_adc.raw_i1 > max_i1)
        {
            max_i1 = s_adc.raw_i1;
        }
        if (s_adc.raw_i3 < min_i3)
        {
            min_i3 = s_adc.raw_i3;
        }
        if (s_adc.raw_i3 > max_i3)
        {
            max_i3 = s_adc.raw_i3;
        }

        float delta_i1 = (float)s_adc.raw_i1 - s_adc.offset_i1;
        float delta_i3 = (float)s_adc.raw_i3 - s_adc.offset_i3;
        float i1_a = ADC_SENSE_I1_SIGN * delta_i1 * s_adc.scale_a_per_count * 0.576292027f;
        float i3_a = ADC_SENSE_I3_SIGN * delta_i3 * s_adc.scale_a_per_count * 0.959745732f;

        sum_delta_i1 += delta_i1;
        sum_delta_i3 += delta_i3;

        if (i1_a < min_i1_a)
        {
            min_i1_a = i1_a;
        }
        if (i1_a > max_i1_a)
        {
            max_i1_a = i1_a;
        }
        if (i3_a < min_i3_a)
        {
            min_i3_a = i3_a;
        }
        if (i3_a > max_i3_a)
        {
            max_i3_a = i3_a;
        }
    }

    float avg_delta_i1_x1000_f = (sum_delta_i1 * 1000.0f) / (float)samples;
    float avg_delta_i3_x1000_f = (sum_delta_i3 * 1000.0f) / (float)samples;

    int32_t avg_delta_i1_x1000 = (avg_delta_i1_x1000_f >= 0.0f) ?
        (int32_t)(avg_delta_i1_x1000_f + 0.5f) :
        (int32_t)(avg_delta_i1_x1000_f - 0.5f);
    int32_t avg_delta_i3_x1000 = (avg_delta_i3_x1000_f >= 0.0f) ?
        (int32_t)(avg_delta_i3_x1000_f + 0.5f) :
        (int32_t)(avg_delta_i3_x1000_f - 0.5f);

    char msg[256];
    snprintf(msg, sizeof(msg),
             "CUR_STATS,n=%lu,i1_min=%u,i1_max=%u,i1_pp=%u,i1_avg_delta_x1000=%ld,i3_min=%u,i3_max=%u,i3_pp=%u,i3_avg_delta_x1000=%ld\r\n",
             (unsigned long)samples,
             (unsigned int)min_i1,
             (unsigned int)max_i1,
             (unsigned int)(max_i1 - min_i1),
             (long)avg_delta_i1_x1000,
             (unsigned int)min_i3,
             (unsigned int)max_i3,
             (unsigned int)(max_i3 - min_i3),
             (long)avg_delta_i3_x1000);

    uart_print(msg);

    int32_t min_i1_mA = adc_sense_float_to_mA(min_i1_a);
    int32_t max_i1_mA = adc_sense_float_to_mA(max_i1_a);
    int32_t pp_i1_mA = adc_sense_float_to_mA(max_i1_a - min_i1_a);
    int32_t min_i3_mA = adc_sense_float_to_mA(min_i3_a);
    int32_t max_i3_mA = adc_sense_float_to_mA(max_i3_a);
    int32_t pp_i3_mA = adc_sense_float_to_mA(max_i3_a - min_i3_a);
    uint32_t scale_uA_per_count = adc_sense_scale_to_uA_per_count(s_adc.scale_a_per_count);

    snprintf(msg, sizeof(msg),
             "CUR_STATS_A,n=%lu,i1_min_mA=%ld,i1_max_mA=%ld,i1_pp_mA=%ld,i3_min_mA=%ld,i3_max_mA=%ld,i3_pp_mA=%ld,scale_uA_cnt=%lu\r\n",
             (unsigned long)samples,
             (long)min_i1_mA,
             (long)max_i1_mA,
             (long)pp_i1_mA,
             (long)min_i3_mA,
             (long)max_i3_mA,
             (long)pp_i3_mA,
             (unsigned long)scale_uA_per_count);

    uart_print(msg);
}

void adc_sense_print_all_rank_stats(uint32_t samples)
{
    if (samples == 0U)
    {
        samples = 1000U;
    }

    uint16_t min_adc1[ADC_SENSE_ADC1_DMA_LEN];
    uint16_t min_adc2[ADC_SENSE_ADC2_DMA_LEN];
    uint16_t min_adc3[ADC_SENSE_ADC3_DMA_LEN];
    uint16_t max_adc1[ADC_SENSE_ADC1_DMA_LEN];
    uint16_t max_adc2[ADC_SENSE_ADC2_DMA_LEN];
    uint16_t max_adc3[ADC_SENSE_ADC3_DMA_LEN];
    uint64_t sum_adc1[ADC_SENSE_ADC1_DMA_LEN];
    uint64_t sum_adc2[ADC_SENSE_ADC2_DMA_LEN];
    uint64_t sum_adc3[ADC_SENSE_ADC3_DMA_LEN];

    for (uint32_t i = 0U; i < ADC_SENSE_ADC1_DMA_LEN; i++)
    {
        min_adc1[i] = 0xFFFFU;
        max_adc1[i] = 0U;
        sum_adc1[i] = 0ULL;
    }

    for (uint32_t i = 0U; i < ADC_SENSE_ADC2_DMA_LEN; i++)
    {
        min_adc2[i] = 0xFFFFU;
        max_adc2[i] = 0U;
        sum_adc2[i] = 0ULL;
    }

    for (uint32_t i = 0U; i < ADC_SENSE_ADC3_DMA_LEN; i++)
    {
        min_adc3[i] = 0xFFFFU;
        max_adc3[i] = 0U;
        sum_adc3[i] = 0ULL;
    }

    for (uint32_t sample = 0U; sample < samples; sample++)
    {
        if (!adc_sense_wait_new_all_frame(10U))
        {
            uart_print("ERR,ADC_STATS_ADC_TIMEOUT\r\n");
            return;
        }

        adc_sense_update_coherent(1U);

        for (uint32_t i = 0U; i < ADC_SENSE_ADC1_DMA_LEN; i++)
        {
            uint16_t raw = s_adc.adc1_raw[i];
            if (raw < min_adc1[i])
            {
                min_adc1[i] = raw;
            }
            if (raw > max_adc1[i])
            {
                max_adc1[i] = raw;
            }
            sum_adc1[i] += raw;
        }

        for (uint32_t i = 0U; i < ADC_SENSE_ADC2_DMA_LEN; i++)
        {
            uint16_t raw = s_adc.adc2_raw[i];
            if (raw < min_adc2[i])
            {
                min_adc2[i] = raw;
            }
            if (raw > max_adc2[i])
            {
                max_adc2[i] = raw;
            }
            sum_adc2[i] += raw;
        }

        for (uint32_t i = 0U; i < ADC_SENSE_ADC3_DMA_LEN; i++)
        {
            uint16_t raw = s_adc.adc3_raw[i];
            if (raw < min_adc3[i])
            {
                min_adc3[i] = raw;
            }
            if (raw > max_adc3[i])
            {
                max_adc3[i] = raw;
            }
            sum_adc3[i] += raw;
        }
    }

    adc_sense_print_rank_stats_line("ADC1_STATS", min_adc1, max_adc1, sum_adc1, samples);
    adc_sense_print_rank_stats_line("ADC2_STATS", min_adc2, max_adc2, sum_adc2, samples);
    adc_sense_print_rank_stats_line("ADC3_STATS", min_adc3, max_adc3, sum_adc3, samples);
}

void adc_sense_print_sample_timing(void)
{
    char msg[192];
    snprintf(msg, sizeof(msg),
             "ADC_SAMP,arr=%lu,ccr2=%lu,cnt=%lu,cr1=0x%08lX,smcr=0x%08lX\r\n",
             (unsigned long)__HAL_TIM_GET_AUTORELOAD(&htim2),
             (unsigned long)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2),
             (unsigned long)__HAL_TIM_GET_COUNTER(&htim2),
             (unsigned long)htim2.Instance->CR1,
             (unsigned long)htim2.Instance->SMCR);
    uart_print(msg);
}

void adc_sense_print_offset(void)
{
    char msg[160];

    uint32_t off_i1_x1000 = (uint32_t)(s_adc.offset_i1 * 1000.0f);
    uint32_t off_i3_x1000 = (uint32_t)(s_adc.offset_i3 * 1000.0f);

    snprintf(msg, sizeof(msg),
             "CUR_OFFSET,i1_x1000=%lu,i3_x1000=%lu,cal=%u\r\n",
             (unsigned long)off_i1_x1000,
             (unsigned long)off_i3_x1000,
             s_adc.current_calibrated);

    uart_print(msg);
}

void adc_sense_print_vbus(void)
{
    adc_sense_update();

    float vpin = adc_sense_raw_to_pin_voltage(s_adc.raw_vbus);
    uint32_t vpin_mV = adc_sense_float_to_mV(vpin);
    uint32_t vbus_cV = adc_sense_float_to_cV(s_adc.vbus_v);

    char msg[160];
    snprintf(msg, sizeof(msg),
             "VBUS,raw=%u,vpin=%lu.%03luV,vbus=%lu.%02luV\r\n",
             s_adc.raw_vbus,
             (unsigned long)(vpin_mV / 1000U),
             (unsigned long)(vpin_mV % 1000U),
             (unsigned long)(vbus_cV / 100U),
             (unsigned long)(vbus_cV % 100U));

    uart_print(msg);
}

void adc_sense_print_all(void)
{
    adc_sense_update();

    int32_t i1_mA = adc_sense_float_to_mA(s_adc.i1_a);
    int32_t i2_mA = adc_sense_float_to_mA(s_adc.i2_a);
    int32_t i3_mA = adc_sense_float_to_mA(s_adc.i3_a);

    uint32_t off_i1_x1000 = (uint32_t)(s_adc.offset_i1 * 1000.0f);
    uint32_t off_i3_x1000 = (uint32_t)(s_adc.offset_i3 * 1000.0f);
    uint32_t vbus_cV = adc_sense_float_to_cV(s_adc.vbus_v);
    uint32_t scale_uA_per_count = adc_sense_scale_to_uA_per_count(s_adc.scale_a_per_count);

    char msg[256];
    snprintf(msg, sizeof(msg),
             "ADC_ALL,raw_i1=%u,raw_i3=%u,raw_vbus=%u,off_i1_x1000=%lu,off_i3_x1000=%lu,i1_mA=%ld,i2_mA=%ld,i3_mA=%ld,vbus=%lu.%02luV,scale_uA_cnt=%lu,cal=%u,dma=%u\r\n",
             s_adc.raw_i1,
             s_adc.raw_i3,
             s_adc.raw_vbus,
             (unsigned long)off_i1_x1000,
             (unsigned long)off_i3_x1000,
             (long)i1_mA,
             (long)i2_mA,
             (long)i3_mA,
             (unsigned long)(vbus_cV / 100U),
             (unsigned long)(vbus_cV % 100U),
             (unsigned long)scale_uA_per_count,
             s_adc.current_calibrated,
             s_adc.dma_started);

    uart_print(msg);
}

void adc_sense_print_status(void)
{
    char msg[384];

    uint32_t ndtr1 = (hdma_adc1.Instance != NULL) ? hdma_adc1.Instance->NDTR : 0U;
    uint32_t ndtr2 = (hdma_adc2.Instance != NULL) ? hdma_adc2.Instance->NDTR : 0U;
    uint32_t ndtr3 = (hdma_adc3.Instance != NULL) ? hdma_adc3.Instance->NDTR : 0U;

    snprintf(msg, sizeof(msg),
             "ADC_STAT,dma=%u,st_adc1=%lu,st_adc2=%lu,st_adc3=%lu,st_tim1=%lu,cfg_tim2=%lu,st_tim2=%lu,h1=%lu,c1=%lu,h2=%lu,c2=%lu,h3=%lu,c3=%lu,e1=%lu,e2=%lu,e3=%lu\r\n",
             s_adc.dma_started,
             (unsigned long)s_start_adc1_status,
             (unsigned long)s_start_adc2_status,
             (unsigned long)s_start_adc3_status,
             (unsigned long)s_start_tim1_status,
             (unsigned long)s_config_tim2_status,
             (unsigned long)s_start_tim2_status,
             (unsigned long)s_adc1_dma_half_count,
             (unsigned long)s_adc1_dma_full_count,
             (unsigned long)s_adc2_dma_half_count,
             (unsigned long)s_adc2_dma_full_count,
             (unsigned long)s_adc3_dma_half_count,
             (unsigned long)s_adc3_dma_full_count,
             (unsigned long)s_adc1_error_count,
             (unsigned long)s_adc2_error_count,
             (unsigned long)s_adc3_error_count);
    uart_print(msg);

    snprintf(msg, sizeof(msg),
             "ADC_TIM,t1_cr1=0x%08lX,t1_cr2=0x%08lX,t2_cr1=0x%08lX,t2_cr2=0x%08lX,t2_smcr=0x%08lX,t2_ccmr1=0x%08lX,t2_sr=0x%08lX,t2_cnt=%lu,t2_ccr2=%lu,t2_ccer=0x%08lX,ndtr1=%lu,ndtr2=%lu,ndtr3=%lu\r\n",
             (unsigned long)htim1.Instance->CR1,
             (unsigned long)htim1.Instance->CR2,
             (unsigned long)htim2.Instance->CR1,
             (unsigned long)htim2.Instance->CR2,
             (unsigned long)htim2.Instance->SMCR,
             (unsigned long)htim2.Instance->CCMR1,
             (unsigned long)htim2.Instance->SR,
             (unsigned long)htim2.Instance->CNT,
             (unsigned long)htim2.Instance->CCR2,
             (unsigned long)htim2.Instance->CCER,
             (unsigned long)ndtr1,
             (unsigned long)ndtr2,
             (unsigned long)ndtr3);
    uart_print(msg);

    snprintf(msg, sizeof(msg),
             "ADC_REG,a1cr2=0x%08lX,a2cr2=0x%08lX,a3cr2=0x%08lX,h1st=0x%08lX,h2st=0x%08lX,h3st=0x%08lX,e1=0x%08lX,e2=0x%08lX,e3=0x%08lX\r\n",
             (unsigned long)hadc1.Instance->CR2,
             (unsigned long)hadc2.Instance->CR2,
             (unsigned long)hadc3.Instance->CR2,
             (unsigned long)hadc1.State,
             (unsigned long)hadc2.State,
             (unsigned long)hadc3.State,
             (unsigned long)hadc1.ErrorCode,
             (unsigned long)hadc2.ErrorCode,
             (unsigned long)hadc3.ErrorCode);
    uart_print(msg);
}

void adc_sense_adc_half_cplt_callback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        s_adc1_dma_half_count++;
    }
    else if (hadc->Instance == ADC2)
    {
        s_adc2_dma_half_count++;
    }
    else if (hadc->Instance == ADC3)
    {
        s_adc3_dma_half_count++;
    }
}

void adc_sense_adc_cplt_callback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        s_adc1_dma_full_count++;
        if (s_adc1_current_frame_ready != 0U)
        {
            s_adc_current_sync_miss_count++;
        }
        s_adc1_current_frame_ready = 1U;
        adc_sense_try_process_current_frame_from_isr();
    }
    else if (hadc->Instance == ADC2)
    {
        s_adc2_dma_full_count++;
        if (s_adc2_current_frame_ready != 0U)
        {
            s_adc_current_sync_miss_count++;
        }
        s_adc2_current_frame_ready = 1U;
        adc_sense_try_process_current_frame_from_isr();
    }
    else if (hadc->Instance == ADC3)
    {
        s_adc3_dma_full_count++;
    }
}

void adc_sense_adc_error_callback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        s_adc1_error_count++;
    }
    else if (hadc->Instance == ADC2)
    {
        s_adc2_error_count++;
    }
    else if (hadc->Instance == ADC3)
    {
        s_adc3_error_count++;
    }
}

static uint16_t adc_sense_selected_raw(uint32_t index, uint32_t adc_sel)
{
    if (adc_sel == ADC_SENSE_FROM_ADC2)
    {
        return (index < ADC_SENSE_ADC2_DMA_LEN) ? s_adc2_dma_buf[index] : 0U;
    }

    return (index < ADC_SENSE_ADC1_DMA_LEN) ? s_adc1_dma_buf[index] : 0U;
}

static void adc_sense_copy_dma_snapshot(uint8_t include_adc3)
{
    for (uint32_t i = 0U; i < ADC_SENSE_ADC1_DMA_LEN; i++)
    {
        s_adc.adc1_raw[i] = s_adc1_dma_buf[i];
    }

    for (uint32_t i = 0U; i < ADC_SENSE_ADC2_DMA_LEN; i++)
    {
        s_adc.adc2_raw[i] = s_adc2_dma_buf[i];
    }

    s_adc.raw_i1 = adc_sense_selected_raw(ADC_SENSE_I1_INDEX, ADC_SENSE_I1_FROM_ADC);
    s_adc.raw_i3 = adc_sense_selected_raw(ADC_SENSE_I3_INDEX, ADC_SENSE_I3_FROM_ADC);
    s_adc.raw_vphase_b = s_adc2_dma_buf[ADC2_IDX_SENS2];
    s_adc.raw_vphase_c = s_adc1_dma_buf[ADC1_IDX_SENS3];
    s_adc.vphase_b_v = adc_sense_raw_to_phase_voltage(s_adc.raw_vphase_b);
    s_adc.vphase_c_v = adc_sense_raw_to_phase_voltage(s_adc.raw_vphase_c);

    if (include_adc3 != 0U)
    {
        for (uint32_t i = 0U; i < ADC_SENSE_ADC3_DMA_LEN; i++)
        {
            s_adc.adc3_raw[i] = s_adc3_dma_buf[i];
        }

        s_adc.raw_vbus = s_adc3_dma_buf[ADC_SENSE_VBUS_INDEX];
        s_adc.raw_vphase_a = s_adc3_dma_buf[ADC3_IDX_PHASE_A];
        s_adc.vbus_v = adc_sense_raw_to_bus_voltage(s_adc.raw_vbus);
        s_adc.vphase_a_v = adc_sense_raw_to_phase_voltage(s_adc.raw_vphase_a);
    }
}

static void adc_sense_update_current_from_snapshot(void)
{
    float delta_i1 = (float)s_adc.raw_i1 - s_adc.offset_i1;
    float delta_i3 = (float)s_adc.raw_i3 - s_adc.offset_i3;

    float i1_sample = ADC_SENSE_I1_SIGN * delta_i1 * s_adc.scale_a_per_count;
    float i3_sample = ADC_SENSE_I3_SIGN * delta_i3 * s_adc.scale_a_per_count;

    //COMMENT BUAT NYALAIN FILTER
    s_adc.i1_a = i1_sample * 0.576292027f; // Hasil kalibrasi RMS clamp + RMS adc, gain correction = RMS_Clamp/RMS_ADC (SAAT VBUS 36V)
    s_adc.i3_a = i3_sample * 0.959745732f; // Hasil kalibrasi juga (SAAT VBUS 36V)
    s_adc.i2_a = -(i1_sample + i3_sample);

    //UNCOMMENT BUAT NYALAIN FILTER
    /*if (s_current_filter_ready == 0U)
    {
        adc_sense_reset_current_filter(i1_sample, i3_sample);
        return;
    }


    //s_adc.i1_a += ADC_SENSE_CURRENT_LPF_ALPHA * (i1_sample - s_adc.i1_a);
    //s_adc.i3_a += ADC_SENSE_CURRENT_LPF_ALPHA * (i3_sample - s_adc.i3_a);
    //s_adc.i2_a = -(s_adc.i1_a + s_adc.i3_a);*/
}

static void adc_sense_reset_current_filter(float i1_a, float i3_a)
{
    s_adc.i1_a = i1_a;
    s_adc.i3_a = i3_a;
    s_adc.i2_a = -(i1_a + i3_a);
    s_current_filter_ready = 1U;
}

static void adc_sense_try_process_current_frame_from_isr(void)
{
    if ((s_adc1_current_frame_ready != 0U) &&
        (s_adc2_current_frame_ready != 0U))
    {
        s_adc1_current_frame_ready = 0U;
        s_adc2_current_frame_ready = 0U;
        adc_sense_process_current_frame_from_isr();
    }
}

static void adc_sense_process_current_frame_from_isr(void)
{
    static uint32_t decimation_counter = 0;
    static float sum_i1_a = 0.0f;
    static float sum_i3_a = 0.0f;

    adc_sense_copy_dma_snapshot(1U);
    adc_sense_update_current_from_snapshot();

    // Accumulate the current samples for averaging
    sum_i1_a += s_adc.i1_a;
    sum_i3_a += s_adc.i3_a;

    decimation_counter++;
    if (decimation_counter >= 2)
    {
        decimation_counter = 0;
        
        // Take the average
        s_adc.i1_a = sum_i1_a * 0.5f;
        s_adc.i3_a = sum_i3_a * 0.5f;
        s_adc.i2_a = -(s_adc.i1_a + s_adc.i3_a);
        
        sum_i1_a = 0.0f;
        sum_i3_a = 0.0f;

        s_adc_current_frame_count++;

        Hall_ControlTick();
        hall_sensor_data_t hall = Hall_GetDataFromISR();
        open_loop_adc_tick(ADC_SENSE_CONTROL_DT_S);
        foc_control_adc_tick(ADC_SENSE_CONTROL_DT_S, &s_adc, &hall);
    }
}

static uint8_t adc_sense_wait_new_frame(uint32_t timeout_ms, uint8_t include_adc3)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t adc1_full = s_adc1_dma_full_count;
    uint32_t adc2_full = s_adc2_dma_full_count;
    uint32_t adc3_full = s_adc3_dma_full_count;

    while ((HAL_GetTick() - start_tick) <= timeout_ms)
    {
        if ((s_adc1_dma_full_count != adc1_full) &&
            (s_adc2_dma_full_count != adc2_full) &&
            (!include_adc3 || (s_adc3_dma_full_count != adc3_full)))
        {
            return 1U;
        }
    }

    return 0U;
}

static uint8_t adc_sense_wait_new_current_frame(uint32_t timeout_ms)
{
    return adc_sense_wait_new_frame(timeout_ms, 0U);
}

static uint8_t adc_sense_wait_new_all_frame(uint32_t timeout_ms)
{
    return adc_sense_wait_new_frame(timeout_ms, 1U);
}

static uint8_t adc_sense_update_after_new_frame(uint32_t timeout_ms, uint8_t include_adc3)
{
    if (!adc_sense_wait_new_frame(timeout_ms, include_adc3))
    {
        return 0U;
    }

    adc_sense_update_coherent(include_adc3);
    return 1U;
}

static void adc_sense_update_coherent(uint8_t include_adc3)
{
    for (uint32_t attempt = 0U; attempt < 3U; attempt++)
    {
        uint32_t adc1_full = s_adc1_dma_full_count;
        uint32_t adc2_full = s_adc2_dma_full_count;
        uint32_t adc3_full = s_adc3_dma_full_count;

        adc_sense_update();

        if ((s_adc1_dma_full_count == adc1_full) &&
            (s_adc2_dma_full_count == adc2_full) &&
            (!include_adc3 || (s_adc3_dma_full_count == adc3_full)))
        {
            return;
        }
    }
}

static void adc_sense_print_rank_stats_line(const char *tag,
                                            const uint16_t *min_v,
                                            const uint16_t *max_v,
                                            const uint64_t *sum_v,
                                            uint32_t samples)
{
    uint32_t avg0_x1000 = (uint32_t)(((sum_v[0] * 1000ULL) + ((uint64_t)samples / 2ULL)) / (uint64_t)samples);
    uint32_t avg1_x1000 = (uint32_t)(((sum_v[1] * 1000ULL) + ((uint64_t)samples / 2ULL)) / (uint64_t)samples);
    uint32_t avg2_x1000 = (uint32_t)(((sum_v[2] * 1000ULL) + ((uint64_t)samples / 2ULL)) / (uint64_t)samples);

    char msg[256];
    snprintf(msg, sizeof(msg),
             "%s,n=%lu,r1_min=%u,r1_max=%u,r1_pp=%u,r1_avg_x1000=%lu,r2_min=%u,r2_max=%u,r2_pp=%u,r2_avg_x1000=%lu,r3_min=%u,r3_max=%u,r3_pp=%u,r3_avg_x1000=%lu\r\n",
             tag,
             (unsigned long)samples,
             (unsigned int)min_v[0],
             (unsigned int)max_v[0],
             (unsigned int)(max_v[0] - min_v[0]),
             (unsigned long)avg0_x1000,
             (unsigned int)min_v[1],
             (unsigned int)max_v[1],
             (unsigned int)(max_v[1] - min_v[1]),
             (unsigned long)avg1_x1000,
             (unsigned int)min_v[2],
             (unsigned int)max_v[2],
             (unsigned int)(max_v[2] - min_v[2]),
             (unsigned long)avg2_x1000);
    uart_print(msg);
}

/*static HAL_StatusTypeDef adc_sense_config_tim2_oc2ref_trigger(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    //sConfigOC.OCMode = TIM_OCMODE_TIMING;
    //sConfigOC.Pulse = 1050U;
    //sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    //sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_StatusTypeDef st = HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
    if (st == HAL_OK)
    {
        __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_2);
    }

    return st;
}*/

/*static void adc_sense_prepare_tim2_trigger(void)
{
    __HAL_TIM_DISABLE(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE | TIM_FLAG_CC2);

#if ADC_SENSE_TIM2_SYNC_TO_TIM1
    MODIFY_REG(htim2.Instance->SMCR, TIM_SMCR_SMS | TIM_SMCR_TS,
               TIM_SLAVEMODE_RESET | TIM_TS_ITR0);
#else
    MODIFY_REG(htim2.Instance->SMCR, TIM_SMCR_SMS | TIM_SMCR_TS, 0U);
#endif
}*/

static float adc_sense_raw_to_pin_voltage(uint16_t raw)
{
    return ((float)raw / ADC_MAX_12BIT) * ADC_VREF;
}

static float adc_sense_raw_to_bus_voltage(uint16_t raw)
{
    return adc_sense_raw_to_pin_voltage(raw) * VIN_DIV_RATIO;
}

static float adc_sense_raw_to_phase_voltage(uint16_t raw)
{
    return adc_sense_raw_to_pin_voltage(raw) * VPHASE_DIV_RATIO;
}

static int32_t adc_sense_float_to_mA(float current_a)
{
    float value = current_a * 1000.0f;

    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }
    else
    {
        return (int32_t)(value - 0.5f);
    }
}

static uint32_t adc_sense_float_to_mV(float voltage_v)
{
    if (voltage_v < 0.0f)
    {
        voltage_v = 0.0f;
    }
    return (uint32_t)((voltage_v * 1000.0f) + 0.5f);
}

static uint32_t adc_sense_float_to_cV(float voltage_v)
{
    if (voltage_v < 0.0f)
    {
        voltage_v = 0.0f;
    }
    return (uint32_t)((voltage_v * 100.0f) + 0.5f);
}

static uint32_t adc_sense_scale_to_uA_per_count(float scale_a_per_count)
{
    float value = scale_a_per_count * 1000000.0f;

    if (value < 0.0f)
    {
        value = -value;
    }

    return (uint32_t)(value + 0.5f);
}
