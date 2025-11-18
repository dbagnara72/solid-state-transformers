/* Twelve pulses rectifier */

#include "../include/twelve_pulse_rect.h"

// ------------------------------------------------------------------------------
float twvprProcess(TWVPRCTRL *twvpr, const float wt, const float alpha, const int block, TWVPR_PY *py, TWVPR_PD *pd)
{
	float phaseshift = 0;
	
	phaseshift = wt + MATH_PI_6 + MATH_PI_3 * 5;
	twvpr->ramp_d_1 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
    phaseshift = wt + MATH_PI_6 + MATH_PI_3 * 4;
	twvpr->ramp_d_2 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
    phaseshift = wt + MATH_PI_6 + MATH_PI_3 * 3;
	twvpr->ramp_d_3 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
    phaseshift = wt + MATH_PI_6 + MATH_PI_3 * 2;
	twvpr->ramp_d_4 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
    phaseshift = wt + MATH_PI_6 + MATH_PI_3;
	twvpr->ramp_d_5 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
    phaseshift = wt + MATH_PI_6;
	twvpr->ramp_d_6 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
	phaseshift = wt + MATH_PI_3 * 5;
	twvpr->ramp_y_1 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
    phaseshift = wt + MATH_PI_3 * 4;
	twvpr->ramp_y_2 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
    phaseshift = wt + MATH_PI_3 * 3;
	twvpr->ramp_y_3 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
    phaseshift = wt + MATH_PI_3 * 2;
	twvpr->ramp_y_4 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
    phaseshift = wt + MATH_PI_3;
	twvpr->ramp_y_5 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
    phaseshift = wt;
	twvpr->ramp_y_6 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
	/* +++++++++++ */
	
	if (twvpr->ramp_d_1 >= alpha)
		twvpr->synch_dA1 = 1;
	else
		twvpr->synch_dA1 = 0;
	
	if (twvpr->ramp_d_1 <= twvpr->synch_d1)
		twvpr->synch_dB1 = 1;
	else
		twvpr->synch_dB1 = 0;

	if (twvpr->synch_dA1 == 1)
		twvpr->synch_d1 = alpha + PWIDTH;
	else
		twvpr->synch_d1 = twvpr->synch_d1 + PWIDTH;
		
	if (!block)
		pd->pd_1 = twvpr->synch_dA1 * twvpr->synch_dB1;
	else
		pd->pd_1 = 0;
	
	/* +++++++++++ */	
	
	if (twvpr->ramp_d_2 >= alpha)
		twvpr->synch_dA2 = 1;
	else
		twvpr->synch_dA2 = 0;

	if (twvpr->ramp_d_2 <= twvpr->synch_d2)
		twvpr->synch_dB2 = 1;
	else
		twvpr->synch_dB2 = 0;

	if (twvpr->synch_dA2 == 1)
		twvpr->synch_d2 = alpha + PWIDTH;
	else
		twvpr->synch_d2 = twvpr->synch_d2 + PWIDTH;
		
	if (!block)
		pd->pd_2 = twvpr->synch_dA2 * twvpr->synch_dB2;
	else
		pd->pd_2 = 0;
	
	/* +++++++++++ */
	
	if (twvpr->ramp_d_3 >= alpha)
		twvpr->synch_dA3 = 1;
	else
		twvpr->synch_dA3 = 0;

	if (twvpr->ramp_d_3 <= twvpr->synch_d3)
		twvpr->synch_dB3 = 1;
	else
		twvpr->synch_dB3 = 0;

	if (twvpr->synch_dA3 == 1)
		twvpr->synch_d3 = alpha + PWIDTH;
	else
		twvpr->synch_d3 = twvpr->synch_d3 + PWIDTH;
		
	if (!block)
		pd->pd_3 = twvpr->synch_dA3 * twvpr->synch_dB3;
	else
		pd->pd_3 = 0;

	/* +++++++++++ */

	if (twvpr->ramp_d_4 >= alpha)
		twvpr->synch_dA4 = 1;
	else
		twvpr->synch_dA4 = 0;

	if (twvpr->ramp_d_4 <= twvpr->synch_d4)
		twvpr->synch_dB4 = 1;
	else
		twvpr->synch_dB4 = 0;

	if (twvpr->synch_dA4 == 1)
		twvpr->synch_d4 = alpha + PWIDTH;
	else
		twvpr->synch_d4 = twvpr->synch_d4 + PWIDTH;
		
	if (!block)
		pd->pd_4 = twvpr->synch_dA4 * twvpr->synch_dB4;
	else
		pd->pd_4 = 0;

	/* +++++++++++ */

	if (twvpr->ramp_d_5 >= alpha)
		twvpr->synch_dA5 = 1;
	else
		twvpr->synch_dA5 = 0;

	if (twvpr->ramp_d_5 <= twvpr->synch_d5)
		twvpr->synch_dB5 = 1;
	else
		twvpr->synch_dB5 = 0;

	if (twvpr->synch_dA5 == 1)
		twvpr->synch_d5 = alpha + PWIDTH;
	else
		twvpr->synch_d5 = twvpr->synch_d5 + PWIDTH;
		
	if (!block)
		pd->pd_5 = twvpr->synch_dA5 * twvpr->synch_dB5;
	else
		pd->pd_5 = 0;

	/* +++++++++++ */

	if (twvpr->ramp_d_6 >= alpha)
		twvpr->synch_dA6 = 1;
	else
		twvpr->synch_dA6 = 0;

	if (twvpr->ramp_d_6 <= twvpr->synch_d6)
		twvpr->synch_dB6 = 1;
	else
		twvpr->synch_dB6 = 0;

	if (twvpr->synch_dA6 == 1)
		twvpr->synch_d6 = alpha + PWIDTH;
	else
		twvpr->synch_d6 = twvpr->synch_d6 + PWIDTH;
		
	if (!block)
		pd->pd_6 = twvpr->synch_dA6 * twvpr->synch_dB6;
	else
		pd->pd_6 = 0;
	
	/* +++++++++++ */
	
	if (twvpr->ramp_y_1 >= alpha)
		twvpr->synch_yA1 = 1;
	else
		twvpr->synch_yA1 = 0;
	
	if (twvpr->ramp_y_1 <= twvpr->synch_y1)
		twvpr->synch_yB1 = 1;
	else
		twvpr->synch_yB1 = 0;

	if (twvpr->synch_yA1 == 1)
		twvpr->synch_y1 = alpha + PWIDTH;
	else
		twvpr->synch_y1 = twvpr->synch_y1 + PWIDTH;
		
	if (!block)
		py->py_1 = twvpr->synch_yA1 * twvpr->synch_yB1;
	else
		py->py_1 = 0;
	
	/* +++++++++++ */	
	
	if (twvpr->ramp_y_2 >= alpha)
		twvpr->synch_yA2 = 1;
	else
		twvpr->synch_yA2 = 0;

	if (twvpr->ramp_y_2 <= twvpr->synch_y2)
		twvpr->synch_yB2 = 1;
	else
		twvpr->synch_yB2 = 0;

	if (twvpr->synch_yA2 == 1)
		twvpr->synch_y2 = alpha + PWIDTH;
	else
		twvpr->synch_y2 = twvpr->synch_y2 + PWIDTH;
		
	if (!block)
		py->py_2 = twvpr->synch_yA2 * twvpr->synch_yB2;
	else
		py->py_2 = 0;
	
	/* +++++++++++ */
	
	if (twvpr->ramp_y_3 >= alpha)
		twvpr->synch_yA3 = 1;
	else
		twvpr->synch_yA3 = 0;

	if (twvpr->ramp_y_3 <= twvpr->synch_y3)
		twvpr->synch_yB3 = 1;
	else
		twvpr->synch_yB3 = 0;

	if (twvpr->synch_yA3 == 1)
		twvpr->synch_y3 = alpha + PWIDTH;
	else
		twvpr->synch_y3 = twvpr->synch_y3 + PWIDTH;
		
	if (!block)
		py->py_3 = twvpr->synch_yA3 * twvpr->synch_yB3;
	else
		py->py_3 = 0;

	/* +++++++++++ */

	if (twvpr->ramp_y_4 >= alpha)
		twvpr->synch_yA4 = 1;
	else
		twvpr->synch_yA4 = 0;

	if (twvpr->ramp_y_4 <= twvpr->synch_y4)
		twvpr->synch_yB4 = 1;
	else
		twvpr->synch_yB4 = 0;

	if (twvpr->synch_yA4 == 1)
		twvpr->synch_y4 = alpha + PWIDTH;
	else
		twvpr->synch_y4 = twvpr->synch_y4 + PWIDTH;
		
	if (!block)
		py->py_4 = twvpr->synch_yA4 * twvpr->synch_yB4;
	else
		py->py_4 = 0;

	/* +++++++++++ */

	if (twvpr->ramp_y_5 >= alpha)
		twvpr->synch_yA5 = 1;
	else
		twvpr->synch_yA5 = 0;

	if (twvpr->ramp_y_5 <= twvpr->synch_y5)
		twvpr->synch_yB5 = 1;
	else
		twvpr->synch_yB5 = 0;

	if (twvpr->synch_yA5 == 1)
		twvpr->synch_y5 = alpha + PWIDTH;
	else
		twvpr->synch_y5 = twvpr->synch_y5 + PWIDTH;
		
	if (!block)
		py->py_5 = twvpr->synch_yA5 * twvpr->synch_yB5;
	else
		py->py_5 = 0;

	/* +++++++++++ */

	if (twvpr->ramp_y_6 >= alpha)
		twvpr->synch_yA6 = 1;
	else
		twvpr->synch_yA6 = 0;

	if (twvpr->ramp_y_6 <= twvpr->synch_y6)
		twvpr->synch_yB6 = 1;
	else
		twvpr->synch_yB6 = 0;

	if (twvpr->synch_yA6 == 1)
		twvpr->synch_y6 = alpha + PWIDTH;
	else
		twvpr->synch_y6 = twvpr->synch_y6 + PWIDTH;
		
	if (!block)
		py->py_6 = twvpr->synch_yA6 * twvpr->synch_yB6;
	else
		py->py_6 = 0;
	
}

void twvprProcessSimulink(const float wt, const float alpha, const int block, TWVPR_PY *py, TWVPR_PD *pd)
{
	static TWVPRCTRL twvpr_inst; 
	static TWVPR_PY py_inst; 
	static TWVPR_PD pd_inst; 
	twvprProcess(&twvpr_inst, wt, alpha, block, &py_inst, &pd_inst);
	*py = py_inst;
	*pd = pd_inst;
}











