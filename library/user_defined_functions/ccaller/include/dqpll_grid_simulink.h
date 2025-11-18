#ifndef _DQPLL_GRID_SIMULINK_
#define _DQPLL_GRID_SIMULINK_

#define DQPLL_GRID_INSTANCES				1

typedef struct dqpll_grid_output_s {
	float				omega_pu_hat;		/*  */
	float				gamma_hat;			/*  */
	float				u_grid_xi;			/*  */
	float				u_grid_eta;			/*  */
	int					dqpll_pll_state;	/*  */
} dqpll_grid_output_t;
#define DQPLL_GRID_OUTPUT dqpll_grid_output_t

extern DQPLL_GRID dqpll_grid_ctrl;
extern unsigned int dqpll_grid_ctrl_initialized;

DQPLL_GRID_OUTPUT dqpll_grid_process_simulink(unsigned char reset, const float u_phase_r, const float u_phase_s, 
	const float u_phase_t, const float ts);
#endif
