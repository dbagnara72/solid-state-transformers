
#ifndef _GLOBAL_STATE_MACHINE_
#define _GLOBAL_STATE_MACHINE_

#define GLOBAL_CMD_STOP                 1
#define GLOBAL_CMD_STANDBY              2
#define GLOBAL_CMD_RUN                  3
#define GLOBAL_CMD_RESET                5

#define GLOBAL_STATE_STOP				1
#define GLOBAL_STATE_READY			    2
#define GLOBAL_STATE_CONNECTING         5
#define GLOBAL_STATE_RUN		        4
#define GLOBAL_STATE_DISCONNECTING      9
#define GLOBAL_STATE_ERROR              11

#define INVERTER_CMD_STOP               1
#define INVERTER_CMD_STANDBY            2
#define INVERTER_CMD_RUN                3
#define INVERTER_CMD_RESET              5

#define INVERTER_STATE_STOP             1
#define INVERTER_STATE_READY            2
#define INVERTER_STATE_CONNECTING       5
#define INVERTER_STATE_RUN              4
#define INVERTER_STATE_DISCONNECTING    9
#define INVERTER_STATE_ERROR            11

#define NONE_ERROR                      0

#define INVERTER_PWM_START              1
#define INVERTER_PWM_STOP               0

#define INVERTER_DCLINK_OVERVOLTAGE_ERROR       0x00000001
#define INVERTER_PHU_OVERCURRENT_ERROR          0x00000010
#define INVERTER_PHV_OVERCURRENT_ERROR          0x00000100
#define INVERTER_PHW_OVERCURRENT_ERROR          0x00001000
#define INVERTER_PHU_DRIVER_ERROR               0x00010000
#define INVERTER_PHV_DRIVER_ERROR               0x00100000
#define INVERTER_PHW_DRIVER_ERROR               0x01000000
#define INVERTER_IGBT_OVERTEMPERATURE_ERROR     0x10000000
#define INVERTER_NONE_ERROR                     0x00000000

typedef struct state_machine_s {
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
} state_machine_t;
#define STATE_MACHINE state_machine_t

void global_state_machine_process(STATE_MACHINE *f);

void inverter_state_machine_process(STATE_MACHINE *i);

void global_state_machine_init(STATE_MACHINE *f);

#endif