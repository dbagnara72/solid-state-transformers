#ifndef _BEMF_OBSV_SIMULINK_
#define _BEMF_OBSV_SIMULINK_

#define BEMF_OBSV_INSTANCES				16

typedef struct bemf_obsv_output_s {
	float					omega_pu_hat;				/* omega pu hat */
	float					omega_pu_hat_flt;			/* omega pu hat filtered */
	float					theta_hat;					/* theta hat */
	float					psi_r_alpha_hat;			/* psi rotor alpha hat */
	float					psi_r_beta_hat;				/* psi rotor beta hat */
} bemf_obsv_output_t;
#define BEMF_OBSV_OUTPUT bemf_obsv_output_t

extern BEMF_OBSV state_obsv;
extern unsigned int state_obsv_initialized;

BEMF_OBSV_OUTPUT bemf_obsv_process_simulink(unsigned char reset, const float u_alpha, const float u_beta, 
		const float i_alpha, const float i_beta, const float u_dc, const float ts, 
		const float bemf_obsv_fb_p_1, const float bemf_obsv_p_1, const float bemf_obsv_fb_p_2, 
		const float bemf_obsv_p_2, const float motorc_omega_bez, const float motorc_m_scale, 
		const float motorc_rs_norm, const float motorc_ls_norm, const float motorc_psi_m_norm, 
		const float bemf_obsv_kalman_omega, 
		const float bemf_obsv_kalman_theta, const float phase_compensation_omega, 
		const float phase_compensation_torque, const float omega_flt_fcut);
#endif
