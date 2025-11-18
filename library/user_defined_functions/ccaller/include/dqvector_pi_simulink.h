#ifndef _DQVECTOR_PI_SIMULINK_
#define _DQVECTOR_PI_SIMULINK_

typedef struct dqvector_pi_output_s {
	float					ud;		/*  */
	float					uq;		/*  */
} dqvector_pi_output_t;
#define DQVECTOR_PI_OUTPUT dqvector_pi_output_t

extern DQVECTOR_PI dqpi_ctrl;
extern unsigned int dqpi_ctrl_initialized;

extern DQVECTOR_PI_OUTPUT dqvector_pi_process_simulink(unsigned char reset, const float id_ref, const float id, 
		const float iq_ref, const float iq, const float u_dc, const float ts, 
		const float kp_id, const float ki_id, const float kp_iq, const float ki_iq, 
		const float u_lim);

#endif
