#include <dqpll_grid.h>
#include <dqpll_grid_simulink.h>

DQPLL_GRID dqpll_grid_ctrl = {0.0f};
unsigned int dqpll_grid_ctrl_initialized = 0;

// ------------------------------------------------------------------------------
DQPLL_GRID_OUTPUT dqpll_grid_process_simulink(unsigned char reset, const float u_phase_r, const float u_phase_s, 
		const float u_phase_t, const float ts ) {

	if (!dqpll_grid_ctrl_initialized) {
	    dqpll_grid_init(&dqpll_grid_ctrl);
		dqpll_grid_ctrl_initialized = 1;
	}

	dqpll_grid_ts(&dqpll_grid_ctrl, ts);

	if (reset) {
		dqpll_grid_reset(&dqpll_grid_ctrl);
	}
	
	const float output_value = dqpll_grid_process(&dqpll_grid_ctrl, u_phase_r, u_phase_s, u_phase_t);

	const DQPLL_GRID_OUTPUT dqpll_grid_ctrl_output = {
		.omega_pu_hat = dqpll_grid_ctrl.omega_hat,
		.gamma_hat = dqpll_grid_ctrl.gamma_hat,
		.u_grid_xi = dqpll_grid_ctrl.u_xi,
		.u_grid_eta = dqpll_grid_ctrl.u_eta,
		.dqpll_pll_state = dqpll_grid_ctrl.dqpll_state
	};
	return dqpll_grid_ctrl_output;
}

