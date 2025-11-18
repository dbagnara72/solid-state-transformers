#ifndef _ADV_SPEED_CTRL_PISF_SIMULINK_
#define _ADV_SPEED_CTRL_PISF_SIMULINK_

typedef struct adv_speed_ctrl_pisf_output_s {
	float			torque_output_ref_pu;		/*  */
	float			omega_m_pu_hat;				/*  */
	float			omega_l_pu_hat;				/*  */
	float			torque_theta_pu_hat;		/*  */
} adv_speed_ctrl_pisf_output_t;
#define ADV_SPEED_CTRL_PISF_OUTPUT adv_speed_ctrl_pisf_output_t

extern ADV_SPEED_CTRL_PISF     adv_speed_ctrl_pisf;
extern unsigned int       adv_speed_ctrl_pisf_initialized;

ADV_SPEED_CTRL_PISF_OUTPUT adv_speed_ctrl_pisf_process_simulink(
    unsigned char   reset,
    const float     ts, 
    const float     omega_m_pu_ref, 
    const float     omega_m_pu, 
    const float     torque_m_pu, 
    const float     torque_nom, 
    const float     omega_nom, 
    const float     inertia_m, 
    const float     inertia_l, 
    const float     young_module_rope, 
    const float     viscosity_m, 
    const float     viscosity_l, 
    const float     viscosity_theta,
    const float     torque_m_pu_lim, 
    const float     kp,
    const float     ki,
    const float     k_isf_1,
    const float     k_isf_2,
    const float     k_isf_3,
    const float     l_sobs_1,
    const float     l_sobs_2,
    const float     l_sobs_3
);
#endif
