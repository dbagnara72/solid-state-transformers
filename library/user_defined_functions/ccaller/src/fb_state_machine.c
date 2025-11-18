
#include <fb_state_machine.h>

void global_state_machine_process(STATE_MACHINE *f)
{
    switch (f->global_state)
 	{
  		case GLOBAL_STATE_STOP:
            f->inverter_cmd = INVERTER_CMD_STOP;
            
            if ((f->global_state_error)||(f->inverter_state_error))
            {
                f->global_state = GLOBAL_STATE_ERROR;
                f->inverter_cmd = INVERTER_CMD_STOP;
            }	
  			
            if ((f->global_cmd == GLOBAL_CMD_STANDBY) || (f->global_cmd == GLOBAL_CMD_RUN))
  			{
                f->inverter_cmd = INVERTER_CMD_STANDBY;
			}
			else
            {
                f->inverter_cmd = INVERTER_CMD_STOP;
            }

			if ((f->global_cmd == GLOBAL_CMD_STANDBY) && (f->inverter_ctrl_state == INVERTER_STATE_READY))
  			{
                f->global_state = GLOBAL_STATE_READY;
                f->inverter_cmd = INVERTER_CMD_STANDBY;
			}
  		break;

  		case GLOBAL_STATE_ERROR:
            f->inverter_cmd = INVERTER_CMD_STOP;

            if (f->global_cmd == GLOBAL_CMD_RESET)
  			{
  				global_state_machine_init(f);
  			}
		break;

  		case GLOBAL_STATE_CONNECTING:

  		case GLOBAL_STATE_READY:
            f->inverter_cmd = INVERTER_CMD_STANDBY;
            
            if ((f->global_state_error)||(f->inverter_state_error))
            {
                f->global_state = GLOBAL_STATE_ERROR;
                f->inverter_cmd = INVERTER_CMD_STOP;
            }	
  			
            if (f->global_cmd == GLOBAL_CMD_RUN)
  			{
                f->inverter_cmd = INVERTER_CMD_RUN;
  			}
            else if ((f->global_cmd == GLOBAL_CMD_STANDBY))
  			{
                f->inverter_cmd = INVERTER_CMD_STANDBY;
			}
			else
            {
                f->inverter_cmd = INVERTER_CMD_STOP;
            }

			if ((f->global_cmd == GLOBAL_CMD_RUN) && (f->inverter_ctrl_state == INVERTER_STATE_RUN))
  			{
                f->global_state = GLOBAL_STATE_RUN;
                f->inverter_cmd = INVERTER_CMD_RUN;
			}

			if ((f->global_cmd == GLOBAL_CMD_STOP) && (f->inverter_ctrl_state == INVERTER_STATE_STOP))
  			{
                f->global_state = GLOBAL_STATE_STOP;
                f->inverter_cmd = INVERTER_CMD_STOP;
			}
		break;

  		case GLOBAL_STATE_RUN:
            f->inverter_cmd = INVERTER_CMD_RUN;
            
            if ((f->global_state_error)||(f->inverter_state_error))
            {
                f->global_state = GLOBAL_STATE_ERROR;
                f->inverter_cmd = INVERTER_CMD_STOP;
            }	
  			
            if (f->global_cmd == GLOBAL_CMD_RUN)
  			{
                f->inverter_cmd = INVERTER_CMD_RUN;
  			}
            else if ((f->global_cmd == GLOBAL_CMD_STANDBY)||(f->global_cmd == GLOBAL_CMD_STOP)||(f->global_cmd == GLOBAL_CMD_RESET))
  			{
                f->inverter_cmd = INVERTER_CMD_STANDBY;
                f->global_state = GLOBAL_STATE_READY;
			}
			else
            {
                f->inverter_cmd = INVERTER_CMD_STOP;
                f->global_state = GLOBAL_STATE_STOP;
            }

			if ((f->global_cmd == GLOBAL_CMD_RUN) && (f->inverter_ctrl_state == INVERTER_STATE_RUN))
  			{
                f->global_state = GLOBAL_STATE_RUN;
                f->inverter_cmd = INVERTER_CMD_RUN;
			}

			if ((f->global_cmd == GLOBAL_CMD_STOP) && (f->inverter_ctrl_state == INVERTER_STATE_STOP))
  			{
                f->global_state = GLOBAL_STATE_STOP;
                f->inverter_cmd = INVERTER_CMD_STOP;
			}
            
            if ((f->global_cmd == GLOBAL_CMD_STANDBY) && (f->inverter_ctrl_state == INVERTER_STATE_READY))
  			{
                f->global_state = GLOBAL_STATE_READY;
                f->inverter_cmd = INVERTER_CMD_STANDBY;
			}
  		break;

  		case GLOBAL_STATE_DISCONNECTING:

  		break;

        }

}

void global_state_machine_init(STATE_MACHINE *f)
{
    f->global_cmd = GLOBAL_CMD_RESET;
    f->global_cmd_z = GLOBAL_CMD_RESET;
    f->global_state = GLOBAL_STATE_STOP;
    f->global_state_error = NONE_ERROR;
    f->inverter_cmd = INVERTER_CMD_STOP;
    f->inverter_cmd_z = INVERTER_CMD_STOP;
    f->inverter_ctrl_state = INVERTER_STATE_STOP;
    f->inverter_state_error = NONE_ERROR;
    f->pwm_enable = INVERTER_PWM_STOP;
    f->pwm_enable_z = INVERTER_PWM_STOP;
    f->pwm_start = INVERTER_PWM_STOP;
    f->pwm_start_z = INVERTER_PWM_STOP;
    f->pwm_stop = INVERTER_PWM_STOP;
    f->pwm_stop_z = INVERTER_PWM_STOP;
}

void inverter_state_machine_process(STATE_MACHINE *i)
{
	switch (i->inverter_ctrl_state)
	{
		case INVERTER_STATE_STOP:
            i->pwm_enable = INVERTER_PWM_STOP;
            
            if (i->inverter_state_error)
            {
                i->inverter_ctrl_state = INVERTER_STATE_ERROR;
                i->inverter_cmd = INVERTER_CMD_STOP;
            }	
  			
            if (i->inverter_cmd == INVERTER_CMD_RUN)
  			{
                i->inverter_ctrl_state = INVERTER_STATE_READY;
  			}
            else if (i->inverter_cmd == INVERTER_CMD_STANDBY)
  			{
                i->inverter_ctrl_state = INVERTER_STATE_READY;
			}
			else
            {
                i->inverter_ctrl_state = INVERTER_STATE_STOP;
            }
		break;
		case INVERTER_STATE_READY:
            i->pwm_enable = INVERTER_PWM_STOP;
            
            if (i->inverter_state_error)
            {
                i->inverter_ctrl_state = INVERTER_STATE_ERROR;
                i->inverter_cmd = INVERTER_CMD_STOP;
            }	
  			
            if (i->inverter_cmd == INVERTER_CMD_RUN)
  			{
                i->inverter_ctrl_state = INVERTER_STATE_CONNECTING;
  			}
            else if (i->inverter_cmd == INVERTER_CMD_STANDBY)
  			{
                i->inverter_ctrl_state = INVERTER_STATE_READY;
			}
			else
            {
                i->inverter_ctrl_state = INVERTER_STATE_STOP;
            }
		break;

		case INVERTER_STATE_CONNECTING:
            i->pwm_enable = INVERTER_PWM_STOP;
            
            if (i->inverter_state_error)
            {
                i->inverter_ctrl_state = INVERTER_STATE_ERROR;
                i->inverter_cmd = INVERTER_CMD_STOP;
            }	
  			
            if (i->inverter_cmd == INVERTER_CMD_RUN)
  			{
                i->inverter_ctrl_state = INVERTER_STATE_RUN;
  			}
            else
  			{
                i->inverter_ctrl_state = INVERTER_STATE_CONNECTING;
			}
		break;

		case INVERTER_STATE_RUN:
            i->pwm_enable = INVERTER_PWM_START;
            
            if (i->inverter_state_error)
            {
                i->inverter_ctrl_state = INVERTER_STATE_ERROR;
                i->inverter_cmd = INVERTER_CMD_STOP;
            }	
  			
            if (i->inverter_cmd != INVERTER_CMD_RUN)
  			{
                i->inverter_ctrl_state = INVERTER_STATE_DISCONNECTING;
			}
			

		break;

        case INVERTER_STATE_DISCONNECTING:
            i->pwm_enable = INVERTER_PWM_STOP;

            if (i->inverter_state_error)
            {
                i->inverter_ctrl_state = INVERTER_STATE_ERROR;
                i->inverter_cmd = INVERTER_CMD_STOP;
            }	
  			else
            {
                i->inverter_ctrl_state = INVERTER_STATE_STOP;
            }
        break;

        case INVERTER_STATE_ERROR:
            i->pwm_enable = INVERTER_PWM_STOP;

            if (i->inverter_cmd == INVERTER_CMD_RESET)
            {
                i->inverter_state_error = NONE_ERROR;
                i->inverter_ctrl_state = INVERTER_STATE_STOP;
            }

        break;
	}
}