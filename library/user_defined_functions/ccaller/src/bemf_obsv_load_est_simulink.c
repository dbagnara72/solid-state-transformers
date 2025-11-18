
#include <bemf_obsv_load_est.h>
#include <bemf_obsv_load_est_simulink.h>

BEMF_OBSV_LOAD_EST state_obsv_load_est = {0.0F};
unsigned int state_obsv_load_est_initialized = 0;

// ------------------------------------------------------------------------------
BEMF_OBSV_LOAD_EST_OUTPUT bemf_obsv_load_est_process_simulink(unsigned char reset, const float u_alpha, const float u_beta, 
	const float i_alpha, const float i_beta, const float u_dc, const float motor_torque_pu, const float ts, 
	const float bemf_obsv_fb_p_1, const float bemf_obsv_p_1,const float bemf_obsv_fb_p_2, const float bemf_obsv_p_2, 
	const float motorc_omega_bez, const float motorc_m_scale, const float motorc_rs_norm, const float motorc_ls_norm, 
	const float motorc_phi_m_norm, const float motorc_load_inertia_norm, const float bemf_obsv_luenberger_1, 
	const float bemf_obsv_luenberger_2, const float bemf_obsv_luenberger_3, const float phase_compensation_omega, 
	const float phase_compensation_torque, const float omega_flt_fcut)
{

	if (!state_obsv_load_est_initialized) {
	    bemf_obsv_load_est_init(&state_obsv_load_est, bemf_obsv_fb_p_1, bemf_obsv_p_1, bemf_obsv_fb_p_2, bemf_obsv_p_2, motorc_omega_bez, motorc_m_scale, motorc_rs_norm,
		motorc_ls_norm, motorc_phi_m_norm, motorc_load_inertia_norm, bemf_obsv_luenberger_1, bemf_obsv_luenberger_2, bemf_obsv_luenberger_3, phase_compensation_omega, phase_compensation_torque, omega_flt_fcut);
		state_obsv_load_est_initialized = 1;
	}

	bemf_obsv_load_est_ts(&state_obsv_load_est, ts);

	if (reset) {
		bemf_obsv_load_est_reset(&state_obsv_load_est);
	}
	
	obsv_load_est_omega_flt_init(&state_obsv_load_est);

	const float output_value = bemf_obsv_load_est_process(&state_obsv_load_est, u_alpha, u_beta, i_alpha, i_beta, u_dc, motor_torque_pu);
	const float omega_pu_hat = output_value;

	const BEMF_OBSV_LOAD_EST_OUTPUT bemf_obsv_load_est_output = {
		.omega_pu_hat = omega_pu_hat,
		.omega_pu_hat_flt = state_obsv_load_est.omega_hat_flt,
		.theta_hat = state_obsv_load_est.theta_hat,
		.psi_r_alpha_hat = state_obsv_load_est.psi_r_alpha_hat,
		.psi_r_beta_hat = state_obsv_load_est.psi_r_beta_hat
	};
	return bemf_obsv_load_est_output;
}

