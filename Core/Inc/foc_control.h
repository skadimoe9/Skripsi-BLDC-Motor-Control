#ifndef INC_FOC_CONTROL_H_
#define INC_FOC_CONTROL_H_

#include "main.h"
#include "adc_sense.h"
#include "hall_sensor.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* FOC controller public API.
 * Start/stop/set functions are called from phase2_test.c UART commands.
 * The real-time loop is foc_control_adc_tick(), called from adc_sense.c at
 * 10 kHz after ADC current averaging and Hall angle update.
 */

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
    /* Public snapshot copied by CAN telemetry and the UART status command. */
    uint8_t running;
    foc_control_mode_t mode;
    foc_control_fault_t fault;

    float id_ref_a;
    float iq_ref_a;
    float rads_ref;
    float iq_limit_a;

    uint8_t fw_enabled;
    uint8_t fw_active;
    float fw_id_ref_a;
    float fw_max_current_a;
    float fw_base_rads;
    float fw_voltage_util;

    float id_a;
    float iq_a;
    float rads_mech;

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

/* Reset controller state and force PWM off; called once during phase2 init. */
void foc_control_init(void);

/* Current-loop tick: adc_sense.c calls this from the ADC processing path.
 * Expected dt_s is 0.0001 s with the current 10 kHz control tick.
 */
void foc_control_adc_tick(float dt_s, const adc_sense_data_t *adc, const hall_sensor_data_t *hall);

/* Start closed-loop current mode with direct Id/Iq references. */
uint8_t foc_control_start_current(float id_ref_a, float iq_ref_a);

/* Start speed mode; the outer speed PI converts rad/s error to Iq reference. */
uint8_t foc_control_start_speed(float rads_ref, float iq_limit_a);

/* Update references while already running. */
void foc_control_set_current_refs(float id_ref_a, float iq_ref_a);
void foc_control_set_speed_ref(float rads_ref, float iq_limit_a);

/* Configure voltage-limit field weakening. max_current_a is max |-Id_FW|. */
void foc_control_set_field_weakening(uint8_t enabled, float max_current_a);

/* Stop FOC and disable complementary PWM outputs. */
void foc_control_stop(void);

uint8_t foc_control_is_running(void);
foc_control_mode_t foc_control_get_mode(void);

/* Safe snapshot used by CANLog_Task() and foc_control_print_status(). */
foc_control_status_t foc_control_get_status(void);

/* Hall calibration knobs used from CLI commands. */
void foc_control_set_theta_offset_deg(float offset_deg);
float foc_control_get_theta_offset_deg(void);
uint8_t foc_control_set_theta_direction(int8_t direction);
int8_t foc_control_get_theta_direction(void);

/* Print the current status snapshot over UART for manual tuning/debug. */
void foc_control_print_status(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_FOC_CONTROL_H_ */
