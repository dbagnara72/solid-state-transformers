#ifndef _BEMF_OBSV_LOAD_EST_SIMULINK_
#define _BEMF_OBSV_LOAD_EST_SIMULINK_

#define BEMF_OBSV_LOAD_EST_INSTANCES				16

typedef struct bemf_obsv_load_est_output_s {
	float					omega_pu_hat;				/* omega pu hat */
	float					omega_pu_hat_flt;			/* omega pu hat filtered */
	float					theta_hat;					/* theta hat */
	float					psi_r_alpha_hat;			/* psi rotor alpha hat */
	float					psi_r_beta_hat;				/* psi rotor beta hat */
} bemf_obsv_load_est_output_t;
#define BEMF_OBSV_LOAD_EST_OUTPUT bemf_obsv_load_est_output_t

extern BEMF_OBSV_LOAD_EST state_obsv_load_est;
extern unsigned int state_obsv_load_est_initialized;

BEMF_OBSV_LOAD_EST_OUTPUT bemf_obsv_load_est_process_simulink(unsigned char reset, const float u_alpha, const float u_beta, 
	const float i_alpha, const float i_beta, const float u_dc, const float motor_torque_pu, const float ts, 
	const float bemf_obsv_fb_p_1, const float bemf_obsv_p_1, const float bemf_obsv_fb_p_2, const float bemf_obsv_p_2, 
	const float motorc_omega_bez, const float motorc_m_scale, const float motorc_rs_norm, const float motorc_ls_norm, 
	const float motorc_psi_m_norm, const float motorc_load_inertia_norm, const float bemf_obsv_luenberger_1, 
	const float bemf_obsv_luenberger_2, const float bemf_obsv_luenberger_3, const float phase_compensation_omega, 
	const float phase_compensation_torque, const float omega_flt_fcut);
#endif
