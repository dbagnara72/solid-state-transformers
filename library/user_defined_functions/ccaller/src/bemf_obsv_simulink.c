
#include <bemf_obsv.h>
#include <bemf_obsv_simulink.h>

BEMF_OBSV state_obsv = {0.0F};
unsigned int state_obsv_initialized = 0;

// ------------------------------------------------------------------------------
BEMF_OBSV_OUTPUT bemf_obsv_process_simulink(unsigned char reset, const float u_alpha, const float u_beta, 
		const float i_alpha, const float i_beta, const float u_dc, const float ts, 
		const float bemf_obsv_fb_p_1, const float bemf_obsv_p_1,const float bemf_obsv_fb_p_2, 
		const float bemf_obsv_p_2, const float motorc_omega_bez, const float motorc_m_scale, 
		const float motorc_rs_norm, const float motorc_ls_norm, const float motorc_phi_m_norm, 
		const float bemf_obsv_kalman_omega, const float bemf_obsv_kalman_theta, const float phase_compensation_omega, 
		const float phase_compensation_torque, const float omega_flt_fcut
)
{

	if (!state_obsv_initialized) {
	    bemf_obsv_init(&state_obsv, bemf_obsv_fb_p_1, bemf_obsv_p_1, bemf_obsv_fb_p_2, bemf_obsv_p_2, motorc_omega_bez, motorc_m_scale, motorc_rs_norm,
		motorc_ls_norm, motorc_phi_m_norm, bemf_obsv_kalman_omega, bemf_obsv_kalman_theta, phase_compensation_omega, phase_compensation_torque, omega_flt_fcut);
		state_obsv_initialized = 1;
	}

	bemf_obsv_ts(&state_obsv, ts);

	if (reset) {
		bemf_obsv_reset(&state_obsv);
	}
	
	obsv_omega_flt_init(&state_obsv);

	const float output_value = bemf_obsv_process(&state_obsv, u_alpha, u_beta, i_alpha, i_beta, u_dc);
	const float omega_pu_hat = output_value;

	const BEMF_OBSV_OUTPUT bemf_obsv_output = {
		.omega_pu_hat = omega_pu_hat,
		.omega_pu_hat_flt = state_obsv.omega_hat_flt,
		.theta_hat = state_obsv.theta_hat,
		.psi_r_alpha_hat = state_obsv.psi_r_alpha_hat,
		.psi_r_beta_hat = state_obsv.psi_r_beta_hat
	};
	return bemf_obsv_output;
}

