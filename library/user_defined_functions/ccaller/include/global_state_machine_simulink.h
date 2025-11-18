#ifndef _GLOBAL_STATE_MACHINE_SIMULINK_
#define _GLOBAL_STATE_MACHINE_SIMULINK_

typedef struct global_state_machine_output_s {
    unsigned int global_state;
    unsigned int global_state_error;
    unsigned int global_cmd;
    unsigned int global_cmd_z;
    unsigned int inverter_ctrl_state;
    unsigned int inverter_state_error;
    unsigned int inverter_cmd;
    unsigned int inverter_cmd_z;

    unsigned int pwm_enable;
    unsigned int pwm_enable_z;
    unsigned int pwm_start;
    unsigned int pwm_start_z;
    unsigned int pwm_stop;
    unsigned int pwm_stop_z;
} global_state_machine_output_t;

#define GLOBAL_OUTPUTS global_state_machine_output_t

GLOBAL_OUTPUTS global_state_machine_process_simulink(unsigned int global_cmd, unsigned int reset);

#endif