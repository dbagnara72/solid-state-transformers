#ifndef _MAVGFLT_SIMULINK_
#define _MAVGFLT_SIMULINK_

#define NMAVGFLT_INSTANCES				31

typedef struct mavgflt_output_s {
	float								ts;		        // sampling time
	float								sshort;		    // sum short (N-1 samples)
	float 								slong;          // sum long (N samples)
	float								pinput;         // previous input
	unsigned int						cbpointer;      // current buffer pointer
	unsigned int						cbsample;      // current buffer sample
	float 								mavg_output;    // moving average filter output
} mavgflt_output_t;
#define MAVGFLT_OUTPUT mavgflt_output_t

MAVGFLT_OUTPUT mavgflt_process_simulink(const float input, const float period, const float ts, unsigned char reset, unsigned char instance);

#endif
