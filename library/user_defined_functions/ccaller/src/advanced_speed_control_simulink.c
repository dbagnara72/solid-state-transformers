
#include <advanced_speed_control.h>
#include <advanced_speed_control_simulink.h>

ADV_SPEED_CTRL    adv_speed_ctrl                  = {0.0F};
unsigned int      adv_speed_ctrl_initialized      = 0;

// ------------------------------------------------------------------------------
ADV_SPEED_CTRL_OUTPUT adv_speed_ctrl_process_simulink(
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
    const float     k_isf_1,
    const float     k_isf_2,
    const float     k_isf_3,
    const float     k_isf_4,
    const float     l_sobs_1,
    const float     l_sobs_2,
    const float     l_sobs_3
)
{

	if (!adv_speed_ctrl_initialized) {
		adv_speed_ctrl_init(&adv_speed_ctrl, ts, omega_nom, torque_nom, inertia_m, inertia_l, 
			young_module_rope, viscosity_m, viscosity_l, viscosity_theta, k_isf_1, k_isf_2, 
			k_isf_3, k_isf_4, l_sobs_1, l_sobs_2, l_sobs_3
		);
		adv_speed_ctrl_initialized = 1;
	}

	adv_speed_ctrl_ts(&adv_speed_ctrl, ts);

	if (reset) {
		adv_speed_ctrl_init(&adv_speed_ctrl, ts, omega_nom, torque_nom, inertia_m, inertia_l, 
			young_module_rope, viscosity_m, viscosity_l, viscosity_theta, k_isf_1, k_isf_2, 
			k_isf_3, k_isf_4, l_sobs_1, l_sobs_2, l_sobs_3
		);
		adv_speed_ctrl_ts(&adv_speed_ctrl, ts);
		adv_speed_ctrl_reset(&adv_speed_ctrl);
	}
	const float torque_output_ref_pu = adv_speed_ctrl_process(&adv_speed_ctrl, omega_m_pu_ref, omega_m_pu);
	
	const ADV_SPEED_CTRL_OUTPUT advanced_speed_ctrl_output = {
				.torque_output_ref_pu = adv_speed_ctrl.torque_output_ctrl_pu,
				.omega_m_pu_hat = adv_speed_ctrl.omega_m_pu_hat,
				.omega_l_pu_hat = adv_speed_ctrl.omega_l_pu_hat,
				.torque_theta_pu_hat = adv_speed_ctrl.torque_theta_pu_hat
	};

	return advanced_speed_ctrl_output;
}

