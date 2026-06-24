#ifndef INC_FOC_CONTROL_H_
#define INC_FOC_CONTROL_H_

#include "main.h"
#include "adc_sense.h"
#include "hall_sensor.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    FOC_CONTROL_MODE_OFF = 0,
    FOC_CONTROL_MODE_CURRENT,
    FOC_CONTROL_MODE_SPEED
} foc_control_mode_t;

typedef enum
{
    FOC_CONTROL_FAULT_NONE = 0,
    FOC_CONTROL_FAULT_NOT_CALIBRATED,
    FOC_CONTROL_FAULT_HALL_INVALID,
    FOC_CONTROL_FAULT_OVERCURRENT,
    FOC_CONTROL_FAULT_VBUS_INVALID
} foc_control_fault_t;

typedef struct
{
    uint8_t running;
    foc_control_mode_t mode;
    foc_control_fault_t fault;

    float id_ref_a;
    float iq_ref_a;
    float rpm_ref;
    float iq_limit_a;

    float id_a;
    float iq_a;
    float rpm_mech;

    float vd_v;
    float vq_v;
    float valpha_v;
    float vbeta_v;
    float vref_v;
    float voltage_limit_v;
    float modulation;

    float duty_u;
    float duty_v;
    float duty_w;

    float vphase_u_smooth_v;
    float vphase_v_smooth_v;
    float vphase_w_smooth_v;
} foc_control_status_t;

void foc_control_init(void);
void foc_control_adc_tick(float dt_s, const adc_sense_data_t *adc, const hall_sensor_data_t *hall);

uint8_t foc_control_start_current(float id_ref_a, float iq_ref_a);
uint8_t foc_control_start_speed(float rpm_ref, float iq_limit_a);
void foc_control_set_current_refs(float id_ref_a, float iq_ref_a);
void foc_control_set_speed_ref(float rpm_ref, float iq_limit_a);
void foc_control_stop(void);

uint8_t foc_control_is_running(void);
foc_control_mode_t foc_control_get_mode(void);
foc_control_status_t foc_control_get_status(void);

void foc_control_set_theta_offset_deg(float offset_deg);
float foc_control_get_theta_offset_deg(void);
uint8_t foc_control_set_theta_direction(int8_t direction);
int8_t foc_control_get_theta_direction(void);
void foc_control_print_status(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_FOC_CONTROL_H_ */
