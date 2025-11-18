#ifndef _DQPLL_THYR_SIMULINK_
#define _DQPLL_THYR_SIMULINK_

#define DQPLL_THYR_INSTANCES				1

typedef struct dqpll_thyr_output_s {
	float				omega_pu_hat;		/*  */
	float				gamma_hat;			/*  */
	float				u_thyr_xi;			/*  */
	float				u_thyr_eta;			/*  */
	int					dqpll_pll_state;	/*  */
} dqpll_thyr_output_t;
#define DQPLL_THYR_OUTPUT dqpll_thyr_output_t

extern DQPLL_THYR dqpll_thyr_ctrl;
extern unsigned int dqpll_thyr_ctrl_initialized;

DQPLL_THYR_OUTPUT dqpll_thyr_process_simulink(unsigned char reset, const float u_phase_r, const float u_phase_s, 
	const float u_phase_t, const float ts);
#endif
