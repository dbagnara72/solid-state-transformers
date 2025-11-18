#ifndef _DSMAVGFLT_SIMULINK_
#define _DSMAVGFLT_SIMULINK_

#define NDSMAVGFLT_INSTANCES				31

typedef struct dsmavgflt_output_s {
	float 				dsmavg_output;    // moving average filter output
} dsmavgflt_output_t;

#define DSMAVGFLT_OUTPUT dsmavgflt_output_t

DSMAVGFLT_OUTPUT dsmavgflt_process_simulink(const float input, const float period, const float ts, unsigned char reset, unsigned char instance);

#endif
