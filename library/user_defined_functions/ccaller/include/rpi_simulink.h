#ifndef _DQVECTOR_PI_SIMULINK_
#define _DQVECTOR_PI_SIMULINK_

typedef struct rpi_output_s {
	float		u_out;		/*  */	
} rpi_output_t;

extern RPI rpi_ctrl;
extern unsigned int rpi_ctrl_initialized;

extern rpi_output_t rpi_process_simulink(unsigned char reset, const float i_ref, const float i, 
		const float u_dc, const float ts, const float kp_rpi, const float ki_rpi, const float omega, const float delta, 
		const float u_lim);

#endif
