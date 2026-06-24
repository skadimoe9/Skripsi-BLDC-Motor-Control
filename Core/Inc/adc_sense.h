#ifndef INC_ADC_SENSE_H_
#define INC_ADC_SENSE_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * adc_sense.h
 * Gabungan current sense + VIN/VBUS sense berbasis independent ADC DMA.
 *
 * Mapping default dibuat berdasarkan konfigurasi CubeMX Anda:
 *
 * ADC1 independent DMA, trigger TIM2_CC2, 3 rank:
 *   adc1[0] = ADC1 CH0
 *   adc1[1] = ADC1 CH8 / PB0 / CURR2   <-- default current sense I3
 *   adc1[2] = ADC1 VREFINT
 *
 * ADC2 independent DMA, trigger TIM2_CC2, 3 rank:
 *   adc2[0] = ADC2 CH1
 *   adc2[1] = ADC2 CH9 / PB1           <-- default current sense I1
 *   adc2[2] = ADC2 CH6
 *
 * ADC3 independent DMA, trigger TIM2_CC2, 3 rank:
 *   adc3[0] = ADC3 CH2
 *   adc3[1] = ADC3 CH3
 *   adc3[2] = ADC3 CH12 / PC2 / VIN_SENS <-- default VBUS
 */

#define ADC_SENSE_ADC1_DMA_LEN       3U
#define ADC_SENSE_ADC2_DMA_LEN       3U
#define ADC_SENSE_ADC3_DMA_LEN       3U

#define ADC_SENSE_FROM_ADC1          1U
#define ADC_SENSE_FROM_ADC2          2U

/* Current sense default mapping from rank 2 on ADC1 and ADC2. */
#ifndef ADC_SENSE_I1_INDEX
#define ADC_SENSE_I1_INDEX           1U     /* ADC2 rank 2: CH9 / PB1 */
#endif

#ifndef ADC_SENSE_I1_FROM_ADC
#define ADC_SENSE_I1_FROM_ADC        ADC_SENSE_FROM_ADC1
#endif

#ifndef ADC_SENSE_I3_INDEX
#define ADC_SENSE_I3_INDEX           1U     /* ADC1 rank 2: CH8 / PB0 / CURR2 */
#endif

#ifndef ADC_SENSE_I3_FROM_ADC
#define ADC_SENSE_I3_FROM_ADC        ADC_SENSE_FROM_ADC2
#endif

/* VBUS/VIN default mapping from ADC3 rank 3. */
#ifndef ADC_SENSE_VBUS_INDEX
#define ADC_SENSE_VBUS_INDEX         2U     /* ADC3 rank 3: CH12 / PC2 / VIN_SENS */
#endif

#ifndef ADC_SENSE_I1_SIGN
#define ADC_SENSE_I1_SIGN            1.0f
#endif

#ifndef ADC_SENSE_I3_SIGN
#define ADC_SENSE_I3_SIGN            1.0f
#endif

#ifndef ADC_SENSE_OFFSET_DEFAULT_SAMPLES
#define ADC_SENSE_OFFSET_DEFAULT_SAMPLES  1000U
#endif

typedef struct
{
    /* Full raw DMA snapshot for debugging rank mapping. */
    uint16_t adc1_raw[ADC_SENSE_ADC1_DMA_LEN];
    uint16_t adc2_raw[ADC_SENSE_ADC2_DMA_LEN];
    uint16_t adc3_raw[ADC_SENSE_ADC3_DMA_LEN];

    /* Selected channels used by control/debug. */
    uint16_t raw_i1;
    uint16_t raw_i3;
    uint16_t raw_vbus;
    uint16_t raw_vphase_a;
    uint16_t raw_vphase_b;
    uint16_t raw_vphase_c;

    float offset_i1;
    float offset_i3;

    float i1_a;
    float i2_a;
    float i3_a;
    float vbus_v;
    float vphase_a_v;
    float vphase_b_v;
    float vphase_c_v;

    float scale_a_per_count;

    uint8_t current_calibrated;
    uint8_t dma_started;
} adc_sense_data_t;

void adc_sense_init(void);
HAL_StatusTypeDef adc_sense_start(void);
void adc_sense_stop(void);
void adc_sense_force_trigger(void);
uint8_t adc_sense_set_sample_ccr2(uint32_t ccr2);
uint32_t adc_sense_get_sample_ccr2(void);
void adc_sense_update(void);
uint8_t adc_sense_calibrate_current_offset(uint32_t samples);
uint8_t adc_sense_calibrate_current_offset_sync(uint32_t samples);

adc_sense_data_t adc_sense_get_data(void);

float adc_sense_get_i1_a(void);
float adc_sense_get_i2_a(void);
float adc_sense_get_i3_a(void);
float adc_sense_get_vbus_v(void);

void adc_sense_print_raw(void);
void adc_sense_print_dma_buffer(void);
void adc_sense_print_current(void);
void adc_sense_print_current_stats(uint32_t samples);
void adc_sense_print_all_rank_stats(uint32_t samples);
void adc_sense_print_sample_timing(void);
void adc_sense_print_offset(void);
void adc_sense_print_vbus(void);
void adc_sense_print_all(void);
void adc_sense_print_status(void);

void adc_sense_adc_half_cplt_callback(ADC_HandleTypeDef *hadc);
void adc_sense_adc_cplt_callback(ADC_HandleTypeDef *hadc);
void adc_sense_adc_error_callback(ADC_HandleTypeDef *hadc);

#ifdef __cplusplus
}
#endif

#endif /* INC_ADC_SENSE_H_ */
