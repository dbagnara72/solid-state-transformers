#ifndef _SV_PWM_
#define _SV_PWM_

#include <math.h>
#include <math_f.h>

typedef struct sv_pwm_s
{
	float	ts;
	float	ualpha; /* u_alpha */
	float 	ubeta;  /* u_beta */
	float	da;
	float	db;
	float	dc;
	unsigned int enable;
} sv_pwm_t;

#define SVPWM sv_pwm_t

void sv_pwm_process(SVPWM *c);

#endif