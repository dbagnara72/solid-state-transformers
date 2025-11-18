#ifndef _SV_PWM_SIMULINK_
#define _SV_PWM_SIMULINK_

typedef struct sv_pwm_output_s {
	float	da;
	float	db;
	float	dc;
} sv_pwm_output_t;
#define SV_PWM_OUTPUT sv_pwm_output_t

SV_PWM_OUTPUT sv_pwm_process_simulink(const float ualpha, const float ubeta, const float ts, unsigned int enable);

#endif