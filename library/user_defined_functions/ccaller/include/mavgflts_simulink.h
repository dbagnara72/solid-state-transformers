#ifndef _MAVGFLTS_SIMULINK_
#define _MAVGFLTS_SIMULINK_

#define NMAVGFLTS_INSTANCES				31

typedef struct mavgflts_output_s {
	float				ts;		        // sampling time
	float				sum_value;		// sum of values
	float				pinput;         // previous input
	unsigned int		cbpointer;      // current buffer pointer
	unsigned int		cbsample;      	// current buffer sample
	float 				mavgs_output;    // moving average filter output
} mavgflts_output_t;

#define MAVGFLTS_OUTPUT mavgflts_output_t

MAVGFLTS_OUTPUT mavgflts_process_simulink(const float input, const float period, const float ts, unsigned char reset, unsigned char instance);

#endif
