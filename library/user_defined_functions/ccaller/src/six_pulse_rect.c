/* six pulses rectifier */

#include "../include/six_pulse_rect.h"

// ------------------------------------------------------------------------------
float sprcProcess(sprc_t *spr, const float wt, const float alpha, const int block, spr_p_t *p)
{
	float phaseshift = 0;
	
	phaseshift = wt + MATH_PI_6 + MATH_5PI_3;
	spr->ramp_1 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
    phaseshift = wt + MATH_PI_6 + MATH_4PI_3;
	spr->ramp_2 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
    phaseshift = wt + MATH_PI_6 + MATH_PI;
	spr->ramp_3 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
    phaseshift = wt + MATH_PI_6 + MATH_2PI_3;
	spr->ramp_4 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
    phaseshift = wt + MATH_PI_6 + MATH_PI_3;
	spr->ramp_5 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
    phaseshift = wt + MATH_PI_6;
	spr->ramp_6 = (phaseshift - floorf(phaseshift / MATH_2PI) * MATH_2PI);
	
	/* +++++++++++ */
	
	if (spr->ramp_1 >= alpha)
		spr->synch_A1 = 1;
	else
		spr->synch_A1 = 0;
	
	if (spr->ramp_1 <= spr->synch_1)
		spr->synch_B1 = 1;
	else
		spr->synch_B1 = 0;

	if (spr->synch_A1 == 1)
		spr->synch_1 = alpha + PWIDTH;
	else
		spr->synch_1 = spr->synch_1 + PWIDTH;
		
	if (!block)
		p->p1 = spr->synch_A1 * spr->synch_B1;
	else
		p->p1 = 0;
	
	/* +++++++++++ */	
	
	if (spr->ramp_2 >= alpha)
		spr->synch_A2 = 1;
	else
		spr->synch_A2 = 0;

	if (spr->ramp_2 <= spr->synch_2)
		spr->synch_B2 = 1;
	else
		spr->synch_B2 = 0;

	if (spr->synch_A2 == 1)
		spr->synch_2 = alpha + PWIDTH;
	else
		spr->synch_2 = spr->synch_2 + PWIDTH;
		
	if (!block)
		p->p2 = spr->synch_A2 * spr->synch_B2;
	else
		p->p2 = 0;
	
	/* +++++++++++ */
	
	if (spr->ramp_3 >= alpha)
		spr->synch_A3 = 1;
	else
		spr->synch_A3 = 0;

	if (spr->ramp_3 <= spr->synch_3)
		spr->synch_B3 = 1;
	else
		spr->synch_B3 = 0;

	if (spr->synch_A3 == 1)
		spr->synch_3 = alpha + PWIDTH;
	else
		spr->synch_3 = spr->synch_3 + PWIDTH;
		
	if (!block)
		p->p3 = spr->synch_A3 * spr->synch_B3;
	else
		p->p3 = 0;

	/* +++++++++++ */

	if (spr->ramp_4 >= alpha)
		spr->synch_A4 = 1;
	else
		spr->synch_A4 = 0;

	if (spr->ramp_4 <= spr->synch_4)
		spr->synch_B4 = 1;
	else
		spr->synch_B4 = 0;

	if (spr->synch_A4 == 1)
		spr->synch_4 = alpha + PWIDTH;
	else
		spr->synch_4 = spr->synch_4 + PWIDTH;
		
	if (!block)
		p->p4 = spr->synch_A4 * spr->synch_B4;
	else
		p->p4 = 0;

	/* +++++++++++ */

	if (spr->ramp_5 >= alpha)
		spr->synch_A5 = 1;
	else
		spr->synch_A5 = 0;

	if (spr->ramp_5 <= spr->synch_5)
		spr->synch_B5 = 1;
	else
		spr->synch_B5 = 0;

	if (spr->synch_A5 == 1)
		spr->synch_5 = alpha + PWIDTH;
	else
		spr->synch_5 = spr->synch_5 + PWIDTH;
		
	if (!block)
		p->p5 = spr->synch_A5 * spr->synch_B5;
	else
		p->p5 = 0;

	/* +++++++++++ */

	if (spr->ramp_6 >= alpha)
		spr->synch_A6 = 1;
	else
		spr->synch_A6 = 0;

	if (spr->ramp_6 <= spr->synch_6)
		spr->synch_B6 = 1;
	else
		spr->synch_B6 = 0;

	if (spr->synch_A6 == 1)
		spr->synch_6 = alpha + PWIDTH;
	else
		spr->synch_6 = spr->synch_6 + PWIDTH;
		
	if (!block)
		p->p6 = spr->synch_A6 * spr->synch_B6;
	else
		p->p6 = 0;
	
}

void sprcProcessSimulink(const float wt, const float alpha, const int block, spr_p_t *p)
{
	static sprc_t spr_inst; 
	static spr_p_t p_inst; 
	sprcProcess(&spr_inst, wt, alpha, block, &p_inst);
	*p = p_inst;
}











