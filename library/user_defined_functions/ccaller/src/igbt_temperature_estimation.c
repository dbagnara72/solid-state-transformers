#include "igbt_temperature_estimation.h"

/*******************************************************************/
void IgbtTemperatureEstimation_Init(igbt_temperature_estimation_t *const igbt_temperature_estimation, const igbt_temperature_estimation_in_t *const input, const igbt_temperature_estimation_config_t *const config)
{
	/* Connect input and config. */
	igbt_temperature_estimation->input  = *input;
	igbt_temperature_estimation->config = *config;
	
	/* Initialize submodules. */
	
	/* Reset output, state & submodules. */
	IgbtTemperatureEstimation_Reset(igbt_temperature_estimation);
}

/*******************************************************************/
void IgbtTemperatureEstimation_Reset(igbt_temperature_estimation_t *const igbt_temperature_estimation)
{
	/* Reset output, state & submodules. */
}

/*******************************************************************/
void IgbtTemperatureEstimation_Process(igbt_temperature_estimation_t *const igbt_temperature_estimation)
{
	
}
