
#include "igbt_temperature_estimation_simulink.h"

igbt_temperature_estimation_sim_t IgbtTemperatureEstimation_Simulate(, uint8_t reset_simulink, uint8_t enable_simulink)
{
    static igbt_temperature_estimation_t igbt_temperature_estimation = { 0 };
    static int initialized = 0;

    if (!initialized) {
        const igbt_temperature_estimation_in_t input = {
            
        };

        const igbt_temperature_estimation_config_t config = {
            
        };

        IgbtTemperatureEstimation_Init(&igbt_temperature_estimation, &input, &config);

        initialized = 1;
    }

    if (reset_simulink) {
        IgbtTemperatureEstimation_Reset(&igbt_temperature_estimation);
    } else if (enable_simulink){
        IgbtTemperatureEstimation_Process(&igbt_temperature_estimation);
    }
    
    const igbt_temperature_estimation_sim_t sim_output = {
        
    };

    return sim_output;
}
