
#include <sv_pwm_simulink.h>
#include <sv_pwm.h>

// ------------------------------------------------------------------------------
SV_PWM_OUTPUT sv_pwm_process_simulink(const float ualpha, const float ubeta, const float ts,
					unsigned int enable)
{
	static SVPWM ctrl = {0};

	ctrl.ualpha = ualpha;
	ctrl.ubeta = ubeta;
	ctrl.ts = ts;
	ctrl.enable = enable;

    sv_pwm_process(&ctrl);

	const SV_PWM_OUTPUT ctrl_out = {
		.da = ctrl.da,
		.db = ctrl.db,
		.dc = ctrl.dc
	};
	return ctrl_out;
}
