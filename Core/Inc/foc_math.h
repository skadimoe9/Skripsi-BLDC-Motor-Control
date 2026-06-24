#ifndef INC_FOC_MATH_H_
#define INC_FOC_MATH_H_

#include <math.h>

/* Small header-only FOC transform helpers.
 * Used by foc_control.c in the real-time current loop and by can_log.c for
 * passive D/Q telemetry. These functions do not touch hardware or global state.
 */

typedef struct
{
    float alpha;
    float beta;
} foc_alpha_beta_t;

typedef struct
{
    float d;
    float q;
} foc_dq_t;

typedef struct
{
    float a;
    float b;
    float c;
} foc_abc_t;

#define FOC_SQRT_2_OVER_3       0.8164965809277260f
#define FOC_SQRT_3_OVER_2       0.8660254037844386f

/* Clarke transform: phase currents/voltages a-b-c -> stationary alpha-beta.
 * foc_control_adc_tick() uses this before Park to obtain measured Id/Iq.
 */
static inline foc_alpha_beta_t foc_clarke(float ia, float ib, float ic)
{
    foc_alpha_beta_t out;
    out.alpha = FOC_SQRT_2_OVER_3 * (ia - (0.5f * ib) - (0.5f * ic));
    out.beta = FOC_SQRT_2_OVER_3 * FOC_SQRT_3_OVER_2 * (ib - ic);
    return out;
}

/* Park transform: stationary alpha-beta -> rotating d-q frame.
 * theta_e_rad is the rotor electrical angle from Hall reconstruction.
 */
static inline foc_dq_t foc_park(foc_alpha_beta_t i_ab, float theta_e_rad)
{
    float c = cosf(theta_e_rad);
    float s = sinf(theta_e_rad);

    foc_dq_t out;
    out.d = (i_ab.alpha * c) + (i_ab.beta * s);
    out.q = (-i_ab.alpha * s) + (i_ab.beta * c);
    return out;
}

/* Inverse Park: commanded d-q voltage -> stationary alpha-beta voltage.
 * foc_control_adc_tick() feeds the result into SVPWM.
 */
static inline foc_alpha_beta_t foc_inv_park(foc_dq_t v_dq, float theta_e_rad)
{
    float c = cosf(theta_e_rad);
    float s = sinf(theta_e_rad);

    foc_alpha_beta_t out;
    out.alpha = (v_dq.d * c) - (v_dq.q * s);
    out.beta = (v_dq.d * s) + (v_dq.q * c);
    return out;
}

/* Inverse Clarke: alpha-beta voltage -> three phase voltage estimate.
 * Used for smooth CAN telemetry; PWM duty is produced separately by SVPWM.
 */
static inline foc_abc_t foc_inv_clarke(foc_alpha_beta_t v_ab)
{
    foc_abc_t out;
    out.a = FOC_SQRT_2_OVER_3 * v_ab.alpha;
    out.b = FOC_SQRT_2_OVER_3 * ((-0.5f * v_ab.alpha) + (FOC_SQRT_3_OVER_2 * v_ab.beta));
    out.c = FOC_SQRT_2_OVER_3 * ((-0.5f * v_ab.alpha) - (FOC_SQRT_3_OVER_2 * v_ab.beta));
    return out;
}

#endif /* INC_FOC_MATH_H_ */
