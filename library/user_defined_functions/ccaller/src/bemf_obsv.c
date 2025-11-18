#include <bemf_obsv.h>

void bemf_obsv_init(volatile BEMF_OBSV *f, volatile float bemf_obsv_fb_p_1, volatile float bemf_obsv_p_1, 
	volatile float bemf_obsv_fb_p_2, volatile float bemf_obsv_p_2, volatile float motorc_omega_bez, volatile float motorc_m_scale, 
	volatile float motorc_rs_norm, volatile float motorc_ls_norm, volatile float motorc_psi_m_norm, 
	volatile float bemf_obsv_kalman_omega, volatile float bemf_obsv_kalman_theta, volatile float phase_compensation_omega, 
	volatile float phase_compensation_torque, volatile float omega_flt_fcut)
{
	f->ts = 0.0;
	f->bemf_obsv_fb_p_1 = bemf_obsv_fb_p_1;
	f->bemf_obsv_p_1 = bemf_obsv_p_1;
	f->bemf_obsv_fb_p_2 = bemf_obsv_fb_p_2;
	f->bemf_obsv_p_2 = bemf_obsv_p_2;
	f->motorc_omega_bez = motorc_omega_bez;
	f->motorc_omega_bez_i = 1.0/motorc_omega_bez;
	f->motorc_m_scale = motorc_m_scale;
	f->motorc_rs_norm = motorc_rs_norm;
	f->motorc_ls_norm = motorc_ls_norm;
	f->motorc_psi_m_norm = motorc_psi_m_norm;
	f->bemf_obsv_kalman_omega = bemf_obsv_kalman_omega;
	f->bemf_obsv_kalman_theta = bemf_obsv_kalman_theta;
	f->phase_compensation_omega = phase_compensation_omega;
	f->phase_compensation_torque = phase_compensation_torque;
	f->omega_flt_fcut = omega_flt_fcut;
	bemf_obsv_reset(f);
}

void bemf_obsv_update(volatile BEMF_OBSV *f, volatile float bemf_obsv_fb_p_1, volatile float bemf_obsv_p_1, 
volatile float bemf_obsv_fb_p_2, volatile float bemf_obsv_p_2)
{
	f->bemf_obsv_fb_p_1 = bemf_obsv_fb_p_1;
	f->bemf_obsv_p_1 = bemf_obsv_p_1;
	f->bemf_obsv_fb_p_2 = bemf_obsv_fb_p_2;
	f->bemf_obsv_p_2 = bemf_obsv_p_2;
}

void bemf_obsv_ts(volatile BEMF_OBSV *f, volatile float ts)
{
	f->ts = ts;
}

void obsv_omega_flt_init(volatile BEMF_OBSV *f)
{
	f->omega_flt_g0 = f->omega_flt_fcut*f->ts*MATH_2PI;
	f->omega_flt_g1 = 1.0-f->omega_flt_g0;
}

void bemf_obsv_reset(volatile BEMF_OBSV *f)
{
	f->psi_s_alpha_hat = 0.0;
	f->psi_r_alpha_hat = 0.0;
	f->psi_r_alpha_hat_hat = 0.0;
	f->psi_ls_alpha_hat = 0.0;
	f->psi_s_beta_hat = 0.0;
	f->psi_r_beta_hat = 0.0;
	f->psi_r_beta_hat_hat = 0.0;
	f->psi_ls_beta_hat = 0.0;
	f->theta_hat = 0.0;
	f->omega_hat = 0.0;
	f->omega_hat_flt = 0.0;
}

float bemf_obsv_process(volatile BEMF_OBSV *f, volatile float u_alpha, volatile float u_beta, volatile float i_alpha, 
	volatile float i_beta, volatile float u_dc)
{
	float bemf_obsv_fb_p = 0.0;
	float bemf_obsv_p = 0.0;

	const float omega_lim_top = BEMF_OBSERVER_UPDATE_TH + BEMF_OBSERVER_UPDATE_HY;
	const float omega_lim_bottom = BEMF_OBSERVER_UPDATE_TH - BEMF_OBSERVER_UPDATE_HY;

	const float theta_hat = f->theta_hat;
	const float omega_hat = f->omega_hat;
	const float omega_hat_flt = f->omega_hat_flt;

	const float psi_m_alpha = f->motorc_psi_m_norm * cosf(theta_hat);
	const float psi_m_beta = f->motorc_psi_m_norm * sinf(theta_hat);

	bemf_obsv_fb_p = f->bemf_obsv_fb_p_1 * (omega_hat_flt - omega_lim_top)/(omega_lim_bottom - omega_lim_top) - 
									f->bemf_obsv_fb_p_2 * (omega_hat_flt - omega_lim_bottom)/(omega_lim_bottom - omega_lim_top);	
	bemf_obsv_p = f->bemf_obsv_p_1 * (omega_hat_flt - omega_lim_top)/(omega_lim_bottom - omega_lim_top) - 
									f->bemf_obsv_p_2 * (omega_hat_flt - omega_lim_bottom)/(omega_lim_bottom - omega_lim_top);

	if (bemf_obsv_fb_p > f->bemf_obsv_fb_p_2)
	{
		bemf_obsv_fb_p = f->bemf_obsv_fb_p_2;
	}
	
	if (bemf_obsv_fb_p < f->bemf_obsv_fb_p_1)
	{
		bemf_obsv_fb_p = f->bemf_obsv_fb_p_1;
	}
	
	if (bemf_obsv_p > f->bemf_obsv_p_2)
	{
		bemf_obsv_p = f->bemf_obsv_p_2;
	}
		
	if (bemf_obsv_p < f->bemf_obsv_p_1)
	{
		bemf_obsv_p = f->bemf_obsv_p_1;
	}

	const float psi_s_alpha_hat_hat = (bemf_obsv_fb_p * (psi_m_alpha - f->psi_r_alpha_hat) + f->motorc_m_scale * u_dc * u_alpha - 
	                    f->motorc_rs_norm * i_alpha - bemf_obsv_p * f->psi_s_alpha_hat) * f->motorc_omega_bez * f->ts + f->psi_s_alpha_hat;

	const float psi_s_beta_hat_hat = (bemf_obsv_fb_p * (psi_m_beta - f->psi_r_beta_hat) + f->motorc_m_scale * u_dc * u_beta - 
	                    f->motorc_rs_norm * i_beta - bemf_obsv_p * f->psi_s_beta_hat) * f->motorc_omega_bez * f->ts + f->psi_s_beta_hat;

	const float psi_ls_alpha_hat_hat = f->motorc_ls_norm * i_alpha;
	const float psi_ls_beta_hat_hat = f->motorc_ls_norm * i_beta;

	const float psi_r_alpha_hat_hat = f->psi_s_alpha_hat - f->psi_ls_alpha_hat;
	const float psi_r_beta_hat_hat = f->psi_s_beta_hat - f->psi_ls_beta_hat;

	const float psi_m_alpha_hat = f->psi_r_alpha_hat;
	const float psi_m_beta_hat = f->psi_r_beta_hat;

	const float theta_bemf_hat = atan2f(psi_m_beta_hat, psi_m_alpha_hat);
	const float theta_bemf_compensated_hat = theta_bemf_hat + f->phase_compensation_omega * omega_hat_flt;

	const float dtheta = theta_bemf_compensated_hat - theta_hat;
	const float dtheta_bounded = - MATH_PI + fmodf(dtheta + MATH_3PI, MATH_2PI);

    const float omega_hat_hat = dtheta_bounded * f->bemf_obsv_kalman_omega * f->motorc_omega_bez_i + omega_hat;
	const float theta_hat_hat_unbounded = dtheta_bounded * f->bemf_obsv_kalman_theta + theta_hat + omega_hat * f->motorc_omega_bez * f->ts;
	const float theta_hat_hat = - MATH_PI + fmodf(theta_hat_hat_unbounded + MATH_3PI, MATH_2PI);
	

	f->psi_s_alpha_hat = psi_s_alpha_hat_hat;
	f->psi_s_beta_hat = psi_s_beta_hat_hat;

	f->psi_ls_alpha_hat = psi_ls_alpha_hat_hat;
	f->psi_ls_beta_hat = psi_ls_beta_hat_hat;

	f->psi_r_alpha_hat = f->psi_r_alpha_hat_hat;
	f->psi_r_beta_hat = f->psi_r_beta_hat_hat;

	f->psi_r_alpha_hat_hat = psi_r_alpha_hat_hat;
	f->psi_r_beta_hat_hat = psi_r_beta_hat_hat;

	f->theta_hat = theta_hat_hat;
	f->omega_hat = omega_hat_hat;	
	f->omega_hat_flt = omega_hat_hat * f->omega_flt_g0 + omega_hat_flt * f->omega_flt_g1;

	return omega_hat;
}

