#ifndef _ADV_SPEED_CTRL_PISFLE_
#define _ADV_SPEED_CTRL_PISFLE_

#include <math.h>
#include <math_f.h>

typedef struct adv_speed_ctrl_pisfle_s {
	float   ts;
    float   omega_m_pu_hat;
    float   omega_l_pu_hat; 
    float   torque_theta_pu_hat;
    float   torque_l_pu_hat;
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
    float   k_isf_4;
    float   l_sobs_1;
    float   l_sobs_2;
    float   l_sobs_3;
    float   l_sobs_4;
    float   trm_a11;
    float   trm_a12;
    float   trm_a13;
    float   trm_a14;
    float   trm_a21;
    float   trm_a22;
    float   trm_a23;
    float   trm_a24;
    float   trm_a31;
    float   trm_a32;
    float   trm_a33;
    float   trm_a34;
    float   trm_a41;
    float   trm_a42;
    float   trm_a43;
    float   trm_a44;
    float   inpm_b1;
    float   inpm_b2;
    float   inpm_b3;
    float   inpm_b4;
} adv_speed_ctrl_pisfle_t;
#define ADV_SPEED_CTRL_PISFLE adv_speed_ctrl_pisfle_t

void adv_speed_ctrl_pisfle_init(
    volatile ADV_SPEED_CTRL_PISFLE *f,
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
    volatile float          k_isf_4,
    volatile float          l_sobs_1,
    volatile float          l_sobs_2,
    volatile float          l_sobs_3,
    volatile float          l_sobs_4
);

void adv_speed_ctrl_pisfle_ts(volatile ADV_SPEED_CTRL_PISFLE *f, volatile float ts);

void adv_speed_ctrl_pisfle_reset(volatile ADV_SPEED_CTRL_PISFLE *f);

void adv_speed_ctrl_pisfle_update(
    volatile ADV_SPEED_CTRL_PISFLE          *f, 
    volatile float                          torque_m_pu
);

float adv_speed_ctrl_pisfle_process(
    volatile ADV_SPEED_CTRL_PISFLE      *f, 
    volatile float                      omega_m_pu_ref, 
    volatile float                      omega_m_pu
);

#endif
