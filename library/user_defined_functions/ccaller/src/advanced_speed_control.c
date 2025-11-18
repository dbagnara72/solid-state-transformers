#include <advanced_speed_control.h>

void adv_speed_ctrl_init(
    volatile ADV_SPEED_CTRL     *f,
    volatile float     			ts,
	volatile float     			omega_nom,
	volatile float     			torque_nom,
    volatile float     			inertia_m,
    volatile float     			inertia_l,
    volatile float     			young_module_rope,
    volatile float     			viscosity_m,
    volatile float     			viscosity_l,
    volatile float     			viscosity_theta,
    volatile float     			k_isf_1,
    volatile float     			k_isf_2,
    volatile float     			k_isf_3,
    volatile float     			k_isf_4,
    volatile float     			l_sobs_1,
    volatile float     			l_sobs_2,
    volatile float     			l_sobs_3
)
{
	/* set task length - start */
	f->ts = ts;
	/* set task length - end */
	
	/* per unit model parameters calculation - start */
	f->inertia_m_pu = 0.5f * inertia_m * omega_nom / torque_nom;
	f->inertia_l_pu = 0.5f * inertia_l * omega_nom / torque_nom;
	f->young_module_rope_pu = young_module_rope / torque_nom;
	f->viscosity_m_pu = viscosity_m / torque_nom;
	f->viscosity_l_pu = viscosity_l / torque_nom;
	f->viscosity_theta_pu = viscosity_theta / torque_nom;
	/* per unit model parameters calculation - end */
	
	/* state feedback and state observer gains - start */
	f->k_isf_1 = k_isf_1;
	f->k_isf_2 = k_isf_2;
	f->k_isf_3 = k_isf_3;
	f->k_isf_4 = k_isf_4;
	f->l_sobs_1 = l_sobs_1;
	f->l_sobs_2 = l_sobs_2;
	f->l_sobs_3 = l_sobs_3;
	/* state feedback and state observer gains - end */

	/* state space model calculation - start */
	f->trm_a11 = 1.0f - ts * (f->viscosity_m_pu + f->viscosity_theta_pu) / f->inertia_m_pu;
	f->trm_a12 = ts * f->viscosity_theta_pu / f->inertia_m_pu;
	f->trm_a13 = -ts / f->inertia_m_pu;
	
	f->trm_a21 = ts * f->viscosity_theta_pu / f->inertia_l_pu;
	f->trm_a22 = 1.0f - ts * (f->viscosity_l_pu + f->viscosity_theta_pu) / f->inertia_l_pu;
	f->trm_a23 = ts / f->inertia_l_pu;
	
	f->trm_a31 = ts * f->young_module_rope_pu;
	f->trm_a32 = -ts * f->young_module_rope_pu;
	f->trm_a33 = 1.0f;
	
	f->inpm_b1 = ts / f->inertia_m_pu;
	f->inpm_b2 = 0;
	f->inpm_b3 = 0;
	/* state space model calculation - end */

	adv_speed_ctrl_reset(f);
}


void adv_speed_ctrl_update(
    volatile ADV_SPEED_CTRL     *f, 
    volatile float              torque_m_pu
)
{
	f->torque_output_ctrl_pu = torque_m_pu;
}

void adv_speed_ctrl_ts(volatile ADV_SPEED_CTRL *f, volatile float ts)
{
	f->ts = ts;
}

void adv_speed_ctrl_reset(volatile ADV_SPEED_CTRL *f)
{
	f->omega_m_pu_hat = 0.0;
	f->omega_l_pu_hat = 0.0;
	f->torque_theta_pu_hat = 0.0;
	f->torque_output_ctrl_pu = 0.0;
	f->torque_output_ctrl_i_pu = 0.0;
}

float adv_speed_ctrl_process(volatile ADV_SPEED_CTRL *f, volatile float omega_m_pu_ref, volatile float omega_m_pu)
{
	/* torque output variables for antiwind-up - start */
	float torque_output_ctrl = 0.0f;
	float torque_output_ctrl_i_sf = 0.0f;
	const float torque_output_ctrl_i = f->torque_output_ctrl_i_pu;
	/* torque output variables for antiwind-up - start */

	const float ts = f->ts;	

	/* state space vector fixing - start */
	const float omega_m_hat = f->omega_m_pu_hat;
	const float omega_l_hat = f->omega_l_pu_hat;
	const float torque_theta_hat = f->torque_theta_pu_hat;
	/* state space vector fixing - end */

	/* error elements for integral state feedback and state observer - start */
	const float omega_m_hat_tilde = omega_m_pu - omega_m_hat;
	const float omega_m_tilde = omega_m_pu_ref - omega_m_pu;
	/* error elements for integral state feedback and state observer - end */

	/* state space model calculation - start */
	const float trm_a11 = f->trm_a11;
	const float trm_a12 = f->trm_a12;
	const float trm_a13 = f->trm_a13;
	
	const float trm_a21 = f->trm_a21;
	const float trm_a22 = f->trm_a22;
	const float trm_a23 = f->trm_a23;
	
	const float trm_a31 = f->trm_a31;
	const float trm_a32 = f->trm_a32;
	const float trm_a33 = f->trm_a33;
	
	const float inpm_b1 = f->inpm_b1;
	const float inpm_b2 = f->inpm_b2;
	const float inpm_b3 = f->inpm_b3;
	/* state space model calculation - end */

	/* integral state feedback gains and state observer gains assignments - start */
	const float obsgm_l1 = f->l_sobs_1;
	const float obsgm_l2 = f->l_sobs_2;
	const float obsgm_l3 = f->l_sobs_3;
	
	const float isfb_k1 = f->k_isf_1;
	const float isfb_k2 = f->k_isf_2;
	const float isfb_k3 = f->k_isf_3;
	const float isfb_k4 = f->k_isf_4;
	/* integral state feedback gains and state observer gains assignments - end */

	/* integral state feedback with state observer - start */
	torque_output_ctrl = torque_output_ctrl_i - isfb_k1 * omega_m_hat - isfb_k2 * omega_l_hat -
							isfb_k3 * torque_theta_hat;

	if (torque_output_ctrl >= 1.4f) {
		torque_output_ctrl_i_sf = torque_output_ctrl_i;
		torque_output_ctrl = 1.4f;
	}
	else if (torque_output_ctrl < -1.4f) {
		torque_output_ctrl_i_sf = torque_output_ctrl_i;
		torque_output_ctrl = -1.4f;
	}
	else {
		torque_output_ctrl_i_sf = -isfb_k4 * omega_m_tilde + torque_output_ctrl_i;
	}
	
	const float omega_m_hat_hat = torque_output_ctrl * inpm_b1 + obsgm_l1 * omega_m_hat_tilde + 
							trm_a11 * omega_m_hat + trm_a12 * omega_l_hat + trm_a13 * torque_theta_hat;
	
	const float omega_l_hat_hat = torque_output_ctrl * inpm_b2 + obsgm_l2 * omega_m_hat_tilde + 
							trm_a21 * omega_m_hat + trm_a22 * omega_l_hat + trm_a23 * torque_theta_hat;
	
	const float torque_theta_hat_hat = torque_output_ctrl * inpm_b3 + obsgm_l3 * omega_m_hat_tilde + 
							trm_a31 * omega_m_hat + trm_a32 * omega_l_hat + trm_a33 * torque_theta_hat;
	/* integral state feedback with state observer - end */

	/* reload state vector - start */
	f->omega_m_pu_hat = omega_m_hat_hat;
	f->omega_l_pu_hat = omega_l_hat_hat;
	f->torque_theta_pu_hat = torque_theta_hat_hat;
	f->torque_output_ctrl_i_pu = torque_output_ctrl_i_sf;
	/* reload state vector - end */

	/* outputs update - start */
	f->torque_output_ctrl_pu = torque_output_ctrl;
	return torque_output_ctrl;
	/* outputs update - end */

}

