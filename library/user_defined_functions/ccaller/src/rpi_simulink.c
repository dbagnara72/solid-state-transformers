
/*
 * rpi_simulink.c
 *
 *  Created on: Feb 20, 2024
 *
 */

#include <rpi.h>
#include <rpi_simulink.h>

unsigned int rpi_ctrl_initialized = 0;
RPI rpi_ctrl;

// ------------------------------------------------------------------------------
rpi_output_t rpi_process_simulink(unsigned char reset, const float i_ref, const float i, 
		const float u_dc, const float ts, const float kp_rpi, const float ki_rpi, 
		const float omega, const float delta, const float u_lim) {

	if (rpi_ctrl_initialized == 0) {
	    rpi_init(&rpi_ctrl, ts, kp_rpi, ki_rpi, omega, delta, u_lim);
		rpi_ctrl_initialized = 1;
	}

	if (reset) {
		rpi_reset(&rpi_ctrl);
	}

	rpi_output_t rpi_output;
	rpi_output.u_out = rpi_process(&rpi_ctrl, i_ref, i, u_dc);
	
	return rpi_output;
}

