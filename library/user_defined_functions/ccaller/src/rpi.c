
/*
 * rpi.c
 *
 *  Created on: Feb 20, 2024
 *
 */	

#include <rpi.h>
#define UDC_MIN 0.25f

void rpi_init(volatile RPI *rpi_ctrl, volatile float ts, volatile float kp_rpi, volatile float ki_rpi, 
		volatile float omega, volatile float delta, volatile float u_out_lim)
{
	rpi_ctrl->ts = ts;
	rpi_ctrl->kp_rpi = kp_rpi;
	rpi_ctrl->ki_rpi = ki_rpi;
	rpi_ctrl->a11 = 1.0f;
	rpi_ctrl->a12 = ts;
	rpi_ctrl->a21 = -omega*omega*ts;
	rpi_ctrl->a22 = 1.0f - 2*delta*omega*ts;
	rpi_ctrl->u_out_lim = u_out_lim;
	rpi_reset(rpi_ctrl);	
}

void rpi_ts(volatile RPI *rpi_ctrl, volatile float ts)
{
	rpi_ctrl->ts = ts;
}

void rpi_reset(volatile RPI *rpi_ctrl)
{
	rpi_ctrl->x1 = 0.0;
	rpi_ctrl->x2 = 0.0;
	rpi_ctrl->y_out = 0.0;
	rpi_ctrl->clip_active = 0;
}

float rpi_process(volatile RPI *rpi_ctrl, volatile float i_ref, 
	volatile float i, volatile float u_dc) {

	const float a11 = rpi_ctrl->a11;
	const float a12 = rpi_ctrl->a12;
	const float a21 = rpi_ctrl->a21;
	const float a22 = rpi_ctrl->a22;
	const float kp_rpi = rpi_ctrl->kp_rpi;
	const float ki_rpi = rpi_ctrl->ki_rpi;
	
	const float i_tilde = i_ref - i;
	const float u_p_out = i_tilde * kp_rpi;

	const float x1_z = rpi_ctrl->x1;
	const float x2_z = rpi_ctrl->x2;

	if (rpi_ctrl->y_out > rpi_ctrl->u_out_lim) {
		rpi_ctrl->clip_active = 1;
	}
	else if (rpi_ctrl->y_out < -rpi_ctrl->u_out_lim) {
		rpi_ctrl->clip_active = 1;
	}
	else {
		rpi_ctrl->clip_active = 0;
	}

	if (!rpi_ctrl->clip_active) {
		rpi_ctrl->x1 = a11*x1_z + a12*x2_z;
		rpi_ctrl->x2 = i_tilde * ki_rpi * rpi_ctrl->ts + a21*x1_z + a22*x2_z;
	}
	else {
		rpi_ctrl->x1 = x1_z;
		rpi_ctrl->x2 = x2_z;
	}

	const float udc = u_dc > UDC_MIN ? u_dc : UDC_MIN;  // avoid division by zero
	rpi_ctrl->y_out = (x2_z + u_p_out)/udc;

	const float u_out = rpi_ctrl->y_out;

	return u_out;
}

