#ifndef _ADV_SPEED_CTRL_PISF_
#define _ADV_SPEED_CTRL_PISF_

#include <math.h>
#include <math_f.h>

typedef struct adv_speed_ctrl_pisf_s {
	float   ts;
    float   omega_m_pu_hat;
    float   omega_l_pu_hat; 
    float   torque_theta_pu_hat;
    float   torque_output_ctrl_i_pu;
    float   torque_output_ctrl_pu;
    float   inertia_m_pu;
    float   inertia_l_pu; 
    float   young_module_rope_pu; 
    float   viscosity_m_pu;
    float   viscosity_l_pu; 
    float   viscosity_theta_pu;
    float   torque_m_pu_lim; 
    float   kp;
    float   ki;
    float   k_isf_1;
    float   k_isf_2;
    float   k_isf_3;
    float   l_sobs_1;
    float   l_sobs_2;
    float   l_sobs_3;
    float   trm_a11;
    float   trm_a12;
    float   trm_a13;
    float   trm_a21;
    float   trm_a22;
    float   trm_a23;
    float   trm_a31;
    float   trm_a32;
    float   trm_a33;
    float   inpm_b1;
    float   inpm_b2;
    float   inpm_b3;
} adv_speed_ctrl_pisf_t;
#define ADV_SPEED_CTRL_PISF adv_speed_ctrl_pisf_t

void adv_speed_ctrl_pisf_init(
    volatile ADV_SPEED_CTRL_PISF *f,
    volatile float     		ts,
    volatile float          omega_m_nom, 
    volatile float          torque_m_nom, 
    volatile float          inertia_m, 
    volatile float          inertia_l, 
    volatile float          young_module_rope, 
    volatile float          viscosity_m, 
    volatile float          viscosity_l, 
    volatile float          viscosity_theta,
    float                   torque_m_pu_lim, 
    volatile float          kp,
    volatile float          ki,
    volatile float          k_isf_1,
    volatile float          k_isf_2,
    volatile float          k_isf_3,
    volatile float          l_sobs_1,
    volatile float          l_sobs_2,
    volatile float          l_sobs_3
);

void adv_speed_ctrl_pisf_ts(volatile ADV_SPEED_CTRL_PISF *f, volatile float ts);

void adv_speed_ctrl_pisf_reset(volatile ADV_SPEED_CTRL_PISF *f);

void adv_speed_ctrl_pisf_update(
    volatile ADV_SPEED_CTRL_PISF        *f, 
    volatile float                      torque_m_pu
);

float adv_speed_ctrl_pisf_process(
    volatile ADV_SPEED_CTRL_PISF    *f, 
    volatile float                  omega_m_pu_ref, 
    volatile float                  omega_m_pu
);

#endif
