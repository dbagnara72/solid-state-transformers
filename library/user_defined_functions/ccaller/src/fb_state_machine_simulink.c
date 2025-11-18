#include <fb_state_machine_simulink.h>
#include <fb_state_machine.h>

GLOBAL_OUTPUTS global_state_machine_process_simulink(float udc, float duty, unsigned int reset)
{
    static STATE_MACHINE ctrl = {0};
    if (reset)
    {
        global_state_machine_init(&ctrl);
    }

    if (udc > UDC_MIN)
    {
        ctrl.global_cmd = GLOBAL_CMD_STANDBY;
        ctrl.pwm_enable = INVERTER_PWM_STOP;
        if (duty > DUTY_TH)
        {
            ctrl.global_cmd = GLOBAL_CMD_RUN;
        }
    }
    else
    {
        ctrl.global_cmd = GLOBAL_CMD_STOP;
        ctrl.pwm_enable = INVERTER_PWM_STOP;
    }

    
    global_state_machine_process(&ctrl);
    inverter_state_machine_process(&ctrl);

    const GLOBAL_OUTPUTS ctrl_out = {
        .global_cmd = ctrl.global_cmd,
        .global_cmd_z = ctrl.global_cmd_z,
        .global_state = ctrl.global_state,
        .global_state_error = ctrl.global_state_error,
        .inverter_cmd = ctrl.inverter_cmd,
        .inverter_cmd_z = ctrl.inverter_cmd_z,
        .inverter_ctrl_state = ctrl.inverter_ctrl_state,
        .inverter_state_error = ctrl.inverter_state_error,
        .pwm_enable = ctrl.pwm_enable,
        .pwm_enable_z = ctrl.pwm_enable_z,
        .pwm_start = ctrl.pwm_start,
        .pwm_start_z = ctrl.pwm_start_z
    };
    return ctrl_out;
}

